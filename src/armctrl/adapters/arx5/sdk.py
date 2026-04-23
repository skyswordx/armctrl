"""基于方舟 `arx5_interface` 的真实硬件适配器。

这一层只做两件事：
1. 复用 SDK 已有的控制器、状态对象和调试接口。
2. 把 SDK 风格的接口翻译成 armctrl 内部统一协议。

这里刻意不重复实现动力学、重力补偿、控制律等底层逻辑。
bringup 阶段已经证明，这类逻辑应该尽量直接复用 SDK，而不是在外层再造一个弱化版本。
"""

from __future__ import annotations

import importlib
from collections.abc import Iterable

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

    def __init__(
        self,
        model: str = "X5",
        interface: str = "can0",
        gravity_compensation: bool | None = None,
        log_level: str = "WARNING",
    ) -> None:
        # 这些配置项全部保留为显式参数，原因是 bringup 时经常需要：
        # - 换模型；
        # - 换 CAN 接口；
        # - 切换是否启用重力补偿；
        # - 调整 SDK 日志级别排障。
        self.model = model
        self.interface = interface
        self.gravity_compensation = gravity_compensation
        self.log_level = log_level
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
            # 这时仅调用 set_eef_cmd 只会更新插值目标，不会把关节重新拉回位置控制。
            # 用户观察到“Y/reset_home 后能动、松 RB 进 damping 后再也动不了”，
            # 本质上就是 reset_to_home 恢复了默认增益，而普通 teleop 没有恢复。
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
        controller.set_gain(default_gain)

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
