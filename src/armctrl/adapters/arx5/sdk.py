"""基于方舟 `arx5_interface` 的真实硬件适配器。

这一层只做两件事：
1. 复用 SDK 已有的控制器、状态对象和调试接口。
2. 把 SDK 风格的接口翻译成 armctrl 内部统一协议。

这里刻意不重复实现动力学、重力补偿、控制律等底层逻辑。
bringup 阶段已经证明，这类逻辑应该尽量直接复用 SDK，而不是在外层再造一个弱化版本。

这一轮和 URDF / 重力补偿有关的实机排障，最后确认了 3 个事实：
1. `robot_config.urdf_path` 必须在创建控制器前覆盖，否则 SDK 继续读取 wheel 内自带模型。
2. SDK 运行时会打印 `Using eef_link for kinematics and link6 for inverse dynamics`，
   也就是运动学末端是 `eef_link`，逆动力学最后一节却是 `link6`。
3. 因此如果把 D435i payload 只写在 fixed joint 后面的 `eef_link`，
   重力补偿完全读不到；必须把等效质量、质心、惯量并入 `link6 <inertial>`。
"""

from __future__ import annotations

import importlib
import time
from collections.abc import Callable, Iterable
from pathlib import Path

from armctrl.calibration.models import apply_gripper_calibration
from armctrl.calibration.store import GripperCalibrationStore
from armctrl.protocol.enums import ArmMode, CommandStatus, DebugProfileName, ErrorCode
from armctrl.protocol.errors import ArmctrlError
from armctrl.protocol.models import (
    CommandResponse,
    DebugProfileRequest,
    EEFStateModel,
    JointStateModel,
    MoveEEFRequest,
    RobotState,
)


class Arx5SDKAdapter:
    """ARX5 真实硬件适配器。

    这是标准的 Adapter / Facade 组合：
    - Adapter：把 SDK 的对象模型转成 armctrl 的请求/响应模型。
    - Facade：对上层隐藏 SDK 初始化、控制器配置和状态抓取细节。
    """

    name = "sdk"
    _PROJECT_ROOT = Path(__file__).resolve().parents[4]
    # X5 是当前项目的主 bringup 机型。
    # 这轮排障后默认改成项目侧 `X5_camera.urdf`，
    # 原因不是“想覆盖 SDK”，而是 SDK 默认 `X5.urdf` 没有带相机 payload。
    # 如果这里不切过去，实机虽然能连通，但重力补偿一直按空载模型算。
    _DEFAULT_X5_CAMERA_URDF = _PROJECT_ROOT / "configs" / "models" / "X5_camera.urdf"
    # 这组比例不是 SDK 官方值，而是 armctrl 为“手推调试”额外定义的工程 profile。
    # 目标不是精准控制，而是：
    # 1. 保留重力补偿；
    # 2. 比 set_to_damping() 更轻；
    # 3. 又不至于完全失去姿态记忆。
    _ZERO_GRAVITY_KP_SCALE = 0.0
    _ZERO_GRAVITY_KD_SCALE = 0.1
    _ZERO_GRAVITY_GRIPPER_KP_SCALE = 0.0
    _ZERO_GRAVITY_GRIPPER_KD_SCALE = 0.0

    def __init__(
        self,
        model: str = "X5",
        interface: str = "can0",
        gravity_compensation: bool | None = None,
        urdf_path: str | None = None,
        log_level: str = "WARNING",
        resume_gain_duration_s: float = 0.4,
        sleep_fn: Callable[[float], None] = time.sleep,
        calibration_store: GripperCalibrationStore | None = None,
    ) -> None:
        # 这些配置项全部保留为显式参数，原因是 bringup 时经常需要：
        # - 换模型；
        # - 换 CAN 接口；
        # - 切换是否启用重力补偿；
        # - 调整 SDK 日志级别排障。
        self.model = model
        self.interface = interface
        self.gravity_compensation = gravity_compensation
        self.urdf_path = urdf_path
        self.log_level = log_level
        self.resume_gain_duration_s = float(resume_gain_duration_s)
        self._sleep_fn = sleep_fn
        # 夹爪标定值不再从临时命令行参数进入。
        # 这里统一读取项目侧标定文件，让 health / teleop / ident 使用同一份配置。
        self._calibration_store = calibration_store or GripperCalibrationStore()
        # `_sdk` 保存动态导入的模块对象，`_controller` 保存真实控制器实例。
        self._sdk = None
        self._controller = None
        self._mode = ArmMode.DISCONNECTED
        self._last_error: ArmctrlError | None = None

    def connect(self) -> CommandResponse:
        try:
            # 用动态导入而不是静态 import，是为了让 fake / 单测环境无需安装 SDK 也能工作。
            self._sdk = importlib.import_module("arx5_interface")
            # 配置对象仍然全部交给 SDK 工厂创建，避免本项目复制配置常量。
            robot_config = self._sdk.RobotConfigFactory.get_instance().get_config(self.model)
            # URDF 仍然走 SDK 的 robot_config 通道，不在外层复制动力学逻辑。
            # 优先级：
            # 1. CLI / 调用方显式传入的 urdf_path；
            # 2. X5 模型默认使用项目侧带 D435i payload 的 X5_camera.urdf；
            # 3. 其他模型继续使用 SDK 自带配置。
            # 这里有一个这轮排障确认过的细节：
            # 只要 controller 还没构造，就可以安全替换 `robot_config.urdf_path`；
            # 一旦 `Arx5CartesianController(...)` 已经创建完成，求解器链路也已经定死，
            # 后面再改路径不会影响当前控制器里的动力学模型。
            resolved_urdf_path = self._resolve_urdf_path()
            if resolved_urdf_path is not None:
                robot_config.urdf_path = str(resolved_urdf_path)
            # 这里把项目侧标定文件应用到 SDK 配置。
            # 如果这台机器的夹爪方向和 SDK 默认值相反，
            # 就靠这一步在 controller 构造前完成 bootstrap。
            apply_gripper_calibration(robot_config, self._calibration_store.load(self.model))
            controller_config = self._sdk.ControllerConfigFactory.get_instance().get_config(
                "cartesian_controller",
                robot_config.joint_dof,
            )
            # 重力补偿必须在控制器创建前配置，因为很多 SDK 会在构造时固化控制器参数。
            if self.gravity_compensation is not None:
                controller_config.gravity_compensation = self.gravity_compensation
            self._controller = self._sdk.Arx5CartesianController(
                robot_config,
                controller_config,
                self.interface,
            )
            # 日志级别属于可选增强项，只有 SDK 和控制器都支持时才设置。
            if hasattr(self._sdk, "LogLevel") and hasattr(self._controller, "set_log_level"):
                level = getattr(self._sdk.LogLevel, self.log_level, None)
                if level is not None:
                    self._controller.set_log_level(level)
            self._mode = ArmMode.IDLE
            return CommandResponse(CommandStatus.COMPLETED, "sdk adapter connected", state=self.get_state())
        except ModuleNotFoundError:
            self._last_error = ArmctrlError(
                ErrorCode.SDK_UNAVAILABLE,
                "arx5_interface is not importable in this environment",
            )
            return CommandResponse(CommandStatus.FAULTED, "sdk unavailable", error=self._last_error)
        except Exception as exc:
            self._last_error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            self._mode = ArmMode.FAULTED
            return CommandResponse(CommandStatus.FAULTED, "sdk connect failed", error=self._last_error)

    def _resolve_urdf_path(self) -> Path | None:
        if self.urdf_path:
            explicit_path = Path(self.urdf_path).expanduser().resolve()
            if not explicit_path.is_file():
                raise FileNotFoundError(f"URDF file not found: {explicit_path}")
            return explicit_path
        # 这里保留“显式路径优先、X5 默认项目模型、其他模型保持 SDK 原样”。
        # 这样做是为了兼顾两件事：
        # 1. 实机默认能直接吃到带 D435i payload 的配置；
        # 2. bringup / A-B 对比时仍能随时切回 SDK 原始 URDF。
        if self.model == "X5" and self._DEFAULT_X5_CAMERA_URDF.is_file():
            return self._DEFAULT_X5_CAMERA_URDF
        return None

    def _require_controller(self):
        # 懒连接模式：首次真正使用控制器时才补做 connect。
        # 这样可以让上层统一走 executor，不需要手动区分“先连后用”。
        if self._controller is None:
            response = self.connect()
            if response.status is not CommandStatus.COMPLETED:
                raise self._last_error or ArmctrlError(ErrorCode.SDK_ERROR, "SDK controller unavailable")
        return self._controller

    def get_state(self) -> RobotState:
        if self._controller is None:
            # 未连接时仍返回结构完整的零状态。
            # 这样 GUI / CLI / 测试都能稳定读取字段，不需要到处判空。
            zero_joint = JointStateModel(
                pos=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                vel=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                torque=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            )
            zero_eef = EEFStateModel((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            return RobotState(self._mode, zero_joint, zero_eef, False, self.name, self._last_error)
        controller = self._controller
        joint_state = controller.get_joint_state()
        eef_state = controller.get_eef_state()
        # 这里做一次“SDK 对象 -> 纯 Python 数据模型”的边界收口。
        # 好处是：
        # 1. 上层不直接依赖 SDK 类；
        # 2. 响应对象可以直接 JSON 化；
        # 3. 单测可以完全绕过 SDK。
        joint = JointStateModel(
            pos=tuple(float(value) for value in joint_state.pos()),
            vel=tuple(float(value) for value in joint_state.vel()),
            torque=tuple(float(value) for value in joint_state.torque()),
            gripper_pos=float(joint_state.gripper_pos),
            gripper_vel=float(joint_state.gripper_vel),
            gripper_torque=float(joint_state.gripper_torque),
            timestamp=float(joint_state.timestamp),
        )
        eef = EEFStateModel(
            pose_6d=tuple(float(value) for value in eef_state.pose_6d()),
            gripper_pos=float(eef_state.gripper_pos),
            gripper_vel=float(eef_state.gripper_vel),
            gripper_torque=float(eef_state.gripper_torque),
            timestamp=float(eef_state.timestamp),
        )
        return RobotState(self._mode, joint, eef, True, self.name, self._last_error)

    def get_home_pose(self) -> tuple[float, ...]:
        controller = self._require_controller()
        return tuple(float(value) for value in controller.get_home_pose())

    def move_eef(self, request: MoveEEFRequest) -> CommandResponse:
        if request.plan_only:
            # plan-only 保持和 fake adapter 一致的语义：
            # 只做协议层通过，不触碰真实硬件。
            return CommandResponse(
                CommandStatus.COMPLETED,
                "sdk eef request validated as plan-only",
                command_id=request.command_id,
                state=self.get_state(),
            )
        try:
            controller = self._require_controller()
            # SDK 在 connect 和 set_to_damping 之后默认可能处在 “kp=0 的阻尼态”。
            # 这里不再瞬间恢复默认增益，而是模仿 reset_to_home 的做法按控制周期渐变，
            # 避免从零刚度 damping 切回高刚度 cartesian 控制时整机抖一下。
            # 这也是这轮手柄排障里确认过的问题：
            # 如果从 damping 态直接 `set_eef_cmd`，终端看起来命令发出去了，
            # 但真实效果常常只是内部目标刷新，末端没有明显受控运动。
            self._ensure_motion_gain(controller)
            # SDK 仍然要求使用它自己的 `EEFState` 命令对象。
            # 这里不自己拼别的格式，避免出现字段顺序、单位或时间戳语义偏差。
            eef_cmd = self._sdk.EEFState()
            eef_cmd.pose_6d()[:] = request.pose_6d
            eef_cmd.gripper_pos = request.gripper_pos
            # preview 时间等价于“希望命令在未来哪个控制时刻生效”。
            # 常见公式：
            #   t_cmd = t_now + preview_time_s
            eef_cmd.timestamp = controller.get_timestamp() + request.preview_time_s
            controller.set_eef_cmd(eef_cmd)
            self._mode = ArmMode.TELEOP
            return CommandResponse(
                CommandStatus.COMPLETED,
                "sdk eef command sent",
                command_id=request.command_id,
                state=self.get_state(),
            )
        except Exception as exc:
            self._last_error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            self._mode = ArmMode.FAULTED
            return CommandResponse(CommandStatus.FAULTED, "sdk eef command failed", error=self._last_error)

    def _ensure_motion_gain(self, controller) -> None:
        # 只在当前位置控制增益确实为零时恢复默认增益。
        # 这样不会覆盖 low_gain / compliance 这类非零增益 profile，
        # 但能把 connect 后初始阻尼态、deadman 释放后的 damping 态重新切回 cartesian 控制。
        if not hasattr(controller, "get_gain") or not hasattr(controller, "set_gain"):
            return
        current_gain = controller.get_gain()
        if not self._is_zero_gain(current_gain):
            return
        controller_config = controller.get_controller_config()
        default_gain = self._sdk.Gain(
            controller_config.default_kp,
            controller_config.default_kd,
            controller_config.default_gripper_kp,
            controller_config.default_gripper_kd,
        )
        self._ramp_gain(controller, current_gain, default_gain, controller_config)

    def _ramp_gain(self, controller, start_gain, target_gain, controller_config) -> None:
        # SDK 的 reset_to_home 在 controller_dt 节拍下线性插值 gain。
        # 这里采用同样思路：插值期间插值器仍固定在 damping 时的当前关节状态，
        # 恢复刚度后才继续发送新的 EEF 命令。
        controller_dt = float(getattr(controller_config, "controller_dt", 0.002))
        if controller_dt <= 0:
            controller_dt = 0.002
        steps = max(1, int(round(self.resume_gain_duration_s / controller_dt)))
        for step_index in range(1, steps + 1):
            alpha = step_index / steps
            controller.set_gain(start_gain * (1.0 - alpha) + target_gain * alpha)
            if step_index < steps:
                self._sleep_fn(controller_dt)

    def _sync_eef_target_to_current_state(self, controller) -> None:
        # 低增益 / 零重力拖动在进入前必须先把插值目标同步到当前实测状态。
        # 否则如果上一段 teleop 还留着一个旧目标，再把 kp 从 0 拉回非零，
        # SDK 会认为“当前命令位置”和“当前实测位置”差太远，导致跳动甚至直接报错。
        if self._sdk is None:
            return
        current_eef = controller.get_eef_state()
        eef_cmd = self._sdk.EEFState()
        eef_cmd.pose_6d()[:] = tuple(float(value) for value in current_eef.pose_6d())
        eef_cmd.gripper_pos = float(current_eef.gripper_pos)
        eef_cmd.gripper_vel = 0.0
        eef_cmd.gripper_torque = 0.0
        controller_dt = float(getattr(controller.get_controller_config(), "controller_dt", 0.002))
        eef_cmd.timestamp = controller.get_timestamp() + max(controller_dt, 0.002)
        controller.set_eef_cmd(eef_cmd)
        self._sleep_fn(max(controller_dt, 0.002))

    def _build_zero_gravity_drag_gain(self, controller_config):
        return self._sdk.Gain(
            [float(value) * self._ZERO_GRAVITY_KP_SCALE for value in controller_config.default_kp],
            [float(value) * self._ZERO_GRAVITY_KD_SCALE for value in controller_config.default_kd],
            float(controller_config.default_gripper_kp) * self._ZERO_GRAVITY_GRIPPER_KP_SCALE,
            float(controller_config.default_gripper_kd) * self._ZERO_GRAVITY_GRIPPER_KD_SCALE,
        )

    def zero_gravity_drag(self) -> CommandResponse:
        try:
            controller = self._require_controller()
            controller_config = controller.get_controller_config()
            self._sync_eef_target_to_current_state(controller)
            current_gain = controller.get_gain()
            target_gain = self._build_zero_gravity_drag_gain(controller_config)
            self._ramp_gain(controller, current_gain, target_gain, controller_config)
            self._mode = ArmMode.ZERO_GRAVITY_DRAG
            detail = {
                "gravity_compensation_enabled": bool(getattr(controller_config, "gravity_compensation", False)),
                "profile_kp_scale": self._ZERO_GRAVITY_KP_SCALE,
                "profile_kd_scale": self._ZERO_GRAVITY_KD_SCALE,
            }
            return CommandResponse(
                CommandStatus.COMPLETED,
                "sdk zero-gravity drag enabled",
                state=self.get_state(),
                detail=detail,
            )
        except Exception as exc:
            self._last_error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            self._mode = ArmMode.FAULTED
            return CommandResponse(CommandStatus.FAULTED, "sdk zero-gravity drag failed", error=self._last_error)

    def _is_zero_gain(self, gain) -> bool:
        # pybind 返回的 kp 可能是 numpy array，单测里也可能是 list。
        # 这里统一按可迭代数值处理，只要任一 kp 非零，就认为已经不是 damping 增益。
        kp_values = gain.kp() if hasattr(gain, "kp") else ()
        if not isinstance(kp_values, Iterable):
            return False
        return max((abs(float(value)) for value in kp_values), default=0.0) <= 1e-9

    def apply_debug_profile(self, request: DebugProfileRequest) -> CommandResponse:
        if request.plan_only:
            return CommandResponse(
                CommandStatus.COMPLETED,
                f"sdk debug profile {request.name.value} validated as plan-only",
                command_id=request.command_id,
                state=self.get_state(),
            )
        try:
            controller = self._require_controller()
            # 这里显式写成 if/elif，而不是字典分派。
            # 原因是不同 profile 的副作用很不一样，展开写更利于新手阅读和之后加保护逻辑。
            if request.name == DebugProfileName.ZERO_GRAVITY_DRAG:
                return self.zero_gravity_drag()
            if request.name == DebugProfileName.DAMPING:
                return self.damping()
            if request.name == DebugProfileName.RESET_HOME:
                controller.reset_to_home()
                self._mode = ArmMode.IDLE
            elif request.name == DebugProfileName.LOW_GAIN_PASSIVE:
                # 这里直接调用 SDK 的 gain 接口。
                # 目标不是发明新的“低增益模式”，而是复用 SDK 已验证的控制参数通道。
                gain = controller.get_gain()
                controller.set_gain(gain * 0.2)
                self._mode = ArmMode.MAINTENANCE
            elif request.name == DebugProfileName.COMPLIANCE_SLOW:
                gain = controller.get_gain()
                controller.set_gain(gain * 0.5)
                self._mode = ArmMode.MAINTENANCE
            elif request.name == DebugProfileName.GRAVITY_COMPENSATION_STARTUP:
                # 这个 profile 不是运行期热切换项。
                # 用户如果要切到重补，应当在控制器构造前通过参数决定。
                raise ArmctrlError(
                    ErrorCode.INVALID_REQUEST,
                    "gravity_compensation must be configured before controller creation",
                )
            return CommandResponse(
                CommandStatus.COMPLETED,
                f"sdk debug profile {request.name.value} applied",
                command_id=request.command_id,
                state=self.get_state(),
            )
        except ArmctrlError as exc:
            return CommandResponse(CommandStatus.REJECTED, exc.message, command_id=request.command_id, error=exc)
        except Exception as exc:
            self._last_error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            self._mode = ArmMode.FAULTED
            return CommandResponse(CommandStatus.FAULTED, "sdk debug profile failed", error=self._last_error)

    def damping(self) -> CommandResponse:
        try:
            controller = self._require_controller()
            # damping 是最关键的安全落态，单独暴露成清晰接口。
            # 它的工程语义更接近“阻尼 / 被动 / 放手”，不是“零重力保持”。
            # 因此末端加了相机后，joint4 在 damping 下缓慢下垂是可以出现的，
            # 这不等于 payload 没写进去，而是因为这里本来就不是主动重力补偿保持态。
            controller.set_to_damping()
            self._mode = ArmMode.DAMPING
            return CommandResponse(CommandStatus.COMPLETED, "sdk damping enabled", state=self.get_state())
        except Exception as exc:
            self._last_error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            self._mode = ArmMode.FAULTED
            return CommandResponse(CommandStatus.FAULTED, "sdk damping failed", error=self._last_error)

    def cancel(self) -> CommandResponse:
        # 当前系统里 cancel 的工程语义就是“回到 damping”。
        # 这样不依赖 SDK 是否提供真正的命令撤销队列。
        response = self.damping()
        if response.status is CommandStatus.COMPLETED:
            return CommandResponse(CommandStatus.CANCELLED, "sdk command cancelled via damping", state=self.get_state())
        return response
