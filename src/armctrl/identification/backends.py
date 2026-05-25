"""参数辨识硬件后端。

后端协议只暴露关节空间读写能力。
轨迹生成、数据记录和后处理不需要知道底层是 ARX5、Piper 还是 fake。
"""

from __future__ import annotations

import importlib
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Protocol

from armctrl.calibration.models import apply_gripper_calibration
from armctrl.calibration.store import GripperCalibrationStore
from armctrl.identification.models import JointSample, TrajectoryPoint
from armctrl.protocol.enums import CommandStatus, ErrorCode
from armctrl.protocol.errors import ArmctrlError
from armctrl.protocol.models import CommandResponse


class JointRobotIO(Protocol):
    """关节空间辨识后端协议。"""

    name: str
    model: str
    dof: int

    def connect(self) -> CommandResponse:
        ...

    def reset_home(self) -> CommandResponse:
        ...

    def send_joint_trajectory(self, points: Sequence[TrajectoryPoint]) -> CommandResponse:
        ...

    def begin_joint_trajectory(self, points: Sequence[TrajectoryPoint]) -> CommandResponse:
        ...

    def send_joint_command(self, point: TrajectoryPoint) -> CommandResponse:
        ...

    def read_sample(self, command: TrajectoryPoint, phase: str) -> JointSample:
        ...

    def damping(self) -> CommandResponse:
        ...

    def hold_joint_position_until_cancelled(
        self,
        point: TrajectoryPoint,
        sleep_fn: Callable[[float], None],
        update_hz: float,
    ) -> CommandResponse:
        ...


class FakeJointRobotIO:
    """无硬件辨识后端。

    fake 后端模拟“完美跟踪命令”的理想情况。
    它只用于验证软件数据链路，不用于评估真实动力学。
    """

    name = "fake_joint"

    def __init__(self, dof: int = 6, model: str = "fake") -> None:
        self.dof = int(dof)
        self.model = model
        self._last_command = TrajectoryPoint(0.0, tuple(0.0 for _ in range(dof)), tuple(0.0 for _ in range(dof)), tuple(0.0 for _ in range(dof)))
        self._connected = False

    def connect(self) -> CommandResponse:
        self._connected = True
        return CommandResponse(CommandStatus.COMPLETED, "fake joint backend connected")

    def reset_home(self) -> CommandResponse:
        zero = tuple(0.0 for _ in range(self.dof))
        self._last_command = TrajectoryPoint(0.0, zero, zero, zero, "fake_home")
        return CommandResponse(CommandStatus.COMPLETED, "fake joint backend reset home")

    def send_joint_trajectory(self, points: Sequence[TrajectoryPoint]) -> CommandResponse:
        if not points:
            return CommandResponse(
                CommandStatus.REJECTED,
                "trajectory is empty",
                error=ArmctrlError(ErrorCode.INVALID_REQUEST, "trajectory is empty"),
            )
        self._last_command = points[0]
        return CommandResponse(CommandStatus.COMPLETED, f"fake accepted {len(points)} joint waypoints")

    def read_sample(self, command: TrajectoryPoint, phase: str) -> JointSample:
        # 这里给一个很轻的“伪力矩”模型，方便后处理测试不全是零：
        #   tau = 0.10*q + 0.01*dq
        # 真实辨识不能使用 fake 数据。
        self._last_command = command
        tau = tuple(0.10 * command.q[index] + 0.01 * command.dq[index] for index in range(self.dof))
        return JointSample(
            t_s=command.t_s,
            monotonic_s=time.monotonic(),
            phase=phase,
            q=command.q,
            dq=command.dq,
            tau_meas=tau,
            q_cmd=command.q,
            dq_cmd=command.dq,
            ddq_cmd=command.ddq,
            tau_cmd=tuple(0.0 for _ in range(self.dof)),
            source_timestamp_s=command.t_s,
        )

    def damping(self) -> CommandResponse:
        return CommandResponse(CommandStatus.COMPLETED, "fake joint backend damping")

    def hold_joint_position_until_cancelled(
        self,
        point: TrajectoryPoint,
        sleep_fn: Callable[[float], None],
        update_hz: float,
    ) -> CommandResponse:
        self._last_command = point
        return CommandResponse(
            CommandStatus.COMPLETED,
            "fake joint backend hold completed",
            detail={"hold_cycles": 0, "update_hz": float(update_hz), "q": point.q},
        )


class Arx5JointRobotIO:
    """ARX5 SDK 关节空间辨识后端。

    这里直接复用 SDK 的 `Arx5JointController`、`JointState` 和 `set_joint_traj`。
    armctrl 不重写插值器、不重写 CAN 通信、不重写电机力矩换算。
    """

    name = "sdk_joint"
    _PROJECT_ROOT = Path(__file__).resolve().parents[3]
    # 关节空间辨识后端和 teleop 一样，都必须吃到同一份项目侧 URDF。
    # 否则前面实机调出来的 payload、质心和惯量，到了采集链路又退回 SDK 原始模型，
    # 后处理回归矩阵和真实控制器模型就会不一致。
    _DEFAULT_X5_CAMERA_URDF = _PROJECT_ROOT / "configs" / "models" / "X5_camera.urdf"

    def __init__(
        self,
        *,
        model: str = "X5",
        interface: str = "can0",
        urdf_path: str | None = None,
        log_level: str = "WARNING",
        start_delay_s: float = 0.20,
        resume_gain_duration_s: float = 0.4,
        sleep_fn: Callable[[float], None] = time.sleep,
        calibration_store: GripperCalibrationStore | None = None,
    ) -> None:
        self.model = model
        self.interface = interface
        self.urdf_path = urdf_path
        self.log_level = log_level
        self.start_delay_s = float(start_delay_s)
        self.resume_gain_duration_s = float(resume_gain_duration_s)
        self._sleep_fn = sleep_fn
        self._calibration_store = calibration_store or GripperCalibrationStore()
        self.dof = 6
        self._sdk = None
        self._controller = None
        self._last_stream_base_timestamp = 0.0
        self._last_sent_commands = 0

    def connect(self) -> CommandResponse:
        try:
            self._sdk = importlib.import_module("arx5_interface")
            robot_config = self._sdk.RobotConfigFactory.get_instance().get_config(self.model)
            resolved_urdf = self._resolve_urdf_path()
            if resolved_urdf is not None:
                robot_config.urdf_path = str(resolved_urdf)
            # 采集链路也必须读项目侧标定文件。
            # 否则 teleop 和 identification 看到的 gripper 宽度、方向会不一致。
            apply_gripper_calibration(robot_config, self._calibration_store.load(self.model))
            self.dof = int(robot_config.joint_dof)
            controller_config = self._sdk.ControllerConfigFactory.get_instance().get_config(
                "joint_controller",
                robot_config.joint_dof,
            )
            if hasattr(controller_config, "background_send_recv"):
                controller_config.background_send_recv = True
            self._controller = self._sdk.Arx5JointController(robot_config, controller_config, self.interface)
            if hasattr(self._sdk, "LogLevel") and hasattr(self._controller, "set_log_level"):
                level = getattr(self._sdk.LogLevel, self.log_level, None)
                if level is not None:
                    self._controller.set_log_level(level)
            return CommandResponse(CommandStatus.COMPLETED, "sdk joint backend connected")
        except ModuleNotFoundError:
            error = ArmctrlError(ErrorCode.SDK_UNAVAILABLE, "arx5_interface is not importable in this environment")
            return CommandResponse(CommandStatus.FAULTED, "sdk unavailable", error=error)
        except Exception as exc:
            error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            return CommandResponse(CommandStatus.FAULTED, "sdk joint backend connect failed", error=error)

    def _resolve_urdf_path(self) -> Path | None:
        if self.urdf_path:
            explicit = Path(self.urdf_path).expanduser().resolve()
            if not explicit.is_file():
                raise FileNotFoundError(f"URDF file not found: {explicit}")
            return explicit
        # 这里沿用 teleop 的默认规则：
        # X5 默认优先项目模型，保证 SDK joint controller、辨识采集和 cartesian teleop
        # 看到的是同一套 payload 参数。
        if self.model == "X5" and self._DEFAULT_X5_CAMERA_URDF.is_file():
            return self._DEFAULT_X5_CAMERA_URDF
        return None

    def _require_controller(self):
        if self._controller is None:
            response = self.connect()
            if response.status is not CommandStatus.COMPLETED:
                raise response.error or ArmctrlError(ErrorCode.SDK_ERROR, "SDK joint backend unavailable")
        return self._controller

    def reset_home(self) -> CommandResponse:
        try:
            self._require_controller().reset_to_home()
            return CommandResponse(CommandStatus.COMPLETED, "sdk joint backend reset home")
        except Exception as exc:
            error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            return CommandResponse(CommandStatus.FAULTED, "sdk joint reset home failed", error=error)

    def _build_motion_gain(self, controller_config):
        if self._sdk is None or not hasattr(self._sdk, "Gain"):
            return None
        return self._sdk.Gain(
            controller_config.default_kp,
            controller_config.default_kd,
            controller_config.default_gripper_kp,
            controller_config.default_gripper_kd,
        )

    def _gain_matches(self, left_gain, right_gain) -> bool:
        left_kp = tuple(float(value) for value in left_gain.kp()) if hasattr(left_gain, "kp") else ()
        right_kp = tuple(float(value) for value in right_gain.kp()) if hasattr(right_gain, "kp") else ()
        left_kd = tuple(float(value) for value in left_gain.kd()) if hasattr(left_gain, "kd") else ()
        right_kd = tuple(float(value) for value in right_gain.kd()) if hasattr(right_gain, "kd") else ()
        if left_kp != right_kp or left_kd != right_kd:
            return False
        return (
            abs(float(getattr(left_gain, "gripper_kp", 0.0)) - float(getattr(right_gain, "gripper_kp", 0.0))) <= 1e-9
            and abs(float(getattr(left_gain, "gripper_kd", 0.0)) - float(getattr(right_gain, "gripper_kd", 0.0)))
            <= 1e-9
        )

    def _ensure_motion_gain(self, controller) -> None:
        # SDK joint controller 在 connect / set_to_damping 后默认处在“kp=0 的阻尼态”。
        # 如果这里只 set_joint_traj，插值目标会更新，但电机不会进入位置跟踪，
        # 实机表现通常就是“CAN 使能、抖一下、基本不动”。
        if not hasattr(controller, "get_gain") or not hasattr(controller, "set_gain"):
            return
        controller_config = controller.get_controller_config()
        target_gain = self._build_motion_gain(controller_config)
        if target_gain is None:
            return
        current_gain = controller.get_gain()
        if self._gain_matches(current_gain, target_gain):
            return
        controller_dt = float(getattr(controller_config, "controller_dt", 0.002))
        if controller_dt <= 0:
            controller_dt = 0.002
        steps = max(1, int(round(self.resume_gain_duration_s / controller_dt)))
        for step_index in range(1, steps + 1):
            alpha = step_index / steps
            controller.set_gain(current_gain * (1.0 - alpha) + target_gain * alpha)
            if step_index < steps:
                self._sleep_fn(controller_dt)

    def _joint_state_from_vectors(
        self,
        *,
        q: Sequence[float],
        dq: Sequence[float] | None = None,
        timestamp: float,
    ):
        joint_state = self._sdk.JointState(self.dof)
        joint_state.pos()[:] = tuple(float(value) for value in q)
        joint_state.vel()[:] = tuple(float(value) for value in (dq or (0.0 for _ in range(self.dof))))
        joint_state.torque()[:] = tuple(0.0 for _ in range(self.dof))
        joint_state.timestamp = float(timestamp)
        joint_state.gripper_pos = 0.0
        return joint_state

    def _sync_joint_target_to_current_state(self, controller) -> None:
        current_state = controller.get_joint_state()
        controller_dt = float(getattr(controller.get_controller_config(), "controller_dt", 0.002))
        if controller_dt <= 0:
            controller_dt = 0.002
        sync_cmd = self._joint_state_from_vectors(
            q=tuple(float(value) for value in current_state.pos()),
            dq=tuple(0.0 for _ in range(self.dof)),
            timestamp=float(controller.get_timestamp()) + controller_dt,
        )
        controller.set_joint_cmd(sync_cmd)
        self._sleep_fn(controller_dt)

    def send_joint_trajectory(self, points: Sequence[TrajectoryPoint]) -> CommandResponse:
        begin_response = self.begin_joint_trajectory(points)
        if begin_response.status is not CommandStatus.COMPLETED:
            return begin_response
        for point in points:
            response = self.send_joint_command(point)
            if response.status is not CommandStatus.COMPLETED:
                return response
        return CommandResponse(CommandStatus.COMPLETED, f"sdk streamed {self._last_sent_commands} joint commands")

    def begin_joint_trajectory(self, points: Sequence[TrajectoryPoint]) -> CommandResponse:
        try:
            if not points:
                raise ArmctrlError(ErrorCode.INVALID_REQUEST, "trajectory is empty")
            controller = self._require_controller()
            self._ensure_motion_gain(controller)
            self._sync_joint_target_to_current_state(controller)
            self._last_stream_base_timestamp = float(controller.get_timestamp()) + self.start_delay_s
            self._last_sent_commands = 0
            return CommandResponse(CommandStatus.COMPLETED, "sdk joint command stream prepared")
        except ArmctrlError as exc:
            return CommandResponse(CommandStatus.REJECTED, exc.message, error=exc)
        except Exception as exc:
            error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            return CommandResponse(CommandStatus.FAULTED, "sdk prepare joint command stream failed", error=error)

    def send_joint_command(self, point: TrajectoryPoint) -> CommandResponse:
        try:
            controller = self._require_controller()
            joint_state = self._joint_state_from_vectors(
                q=point.q,
                dq=point.dq,
                timestamp=max(0.0, self._last_stream_base_timestamp + point.t_s),
            )
            controller.set_joint_cmd(joint_state)
            self._last_sent_commands += 1
            return CommandResponse(CommandStatus.COMPLETED, "sdk joint command sent")
        except Exception as exc:
            error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            return CommandResponse(CommandStatus.FAULTED, "sdk set_joint_cmd failed", error=error)

    def read_sample(self, command: TrajectoryPoint, phase: str) -> JointSample:
        controller = self._require_controller()
        joint_state = controller.get_joint_state()
        try:
            joint_cmd = controller.get_joint_cmd()
            tau_cmd = tuple(float(value) for value in joint_cmd.torque())
        except Exception:
            tau_cmd = tuple(0.0 for _ in range(self.dof))
        return JointSample(
            t_s=command.t_s,
            monotonic_s=time.monotonic(),
            phase=phase,
            q=tuple(float(value) for value in joint_state.pos()),
            dq=tuple(float(value) for value in joint_state.vel()),
            tau_meas=tuple(float(value) for value in joint_state.torque()),
            q_cmd=command.q,
            dq_cmd=command.dq,
            ddq_cmd=command.ddq,
            tau_cmd=tau_cmd,
            source_timestamp_s=float(getattr(joint_state, "timestamp", 0.0)),
        )

    def damping(self) -> CommandResponse:
        try:
            self._require_controller().set_to_damping()
            return CommandResponse(CommandStatus.COMPLETED, "sdk joint backend damping")
        except Exception as exc:
            error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            return CommandResponse(CommandStatus.FAULTED, "sdk joint damping failed", error=error)

    def hold_joint_position_until_cancelled(
        self,
        point: TrajectoryPoint,
        sleep_fn: Callable[[float], None],
        update_hz: float,
    ) -> CommandResponse:
        controller = self._require_controller()
        period_s = 1.0 / max(float(update_hz), 1.0)
        controller_dt = float(getattr(controller.get_controller_config(), "controller_dt", 0.002))
        lookahead_s = max(0.04, controller_dt * 5.0)
        hold_cycles = 0
        while True:
            timestamp = float(controller.get_timestamp()) + lookahead_s
            joint_state = self._joint_state_from_vectors(
                q=point.q,
                dq=tuple(0.0 for _ in range(self.dof)),
                timestamp=timestamp,
            )
            controller.set_joint_cmd(joint_state)
            hold_cycles += 1
            sleep_fn(period_s)


def build_joint_backend(
    *,
    adapter: str,
    model: str,
    interface: str,
    dof: int,
    urdf_path: str | None = None,
) -> JointRobotIO:
    """按 CLI 参数构建辨识后端。"""

    if adapter == "fake":
        return FakeJointRobotIO(dof=dof, model=model)
    if adapter == "sdk":
        return Arx5JointRobotIO(
            model=model,
            interface=interface,
            urdf_path=urdf_path,
        )
    raise ArmctrlError(ErrorCode.INVALID_REQUEST, f"unsupported identification adapter {adapter}")
