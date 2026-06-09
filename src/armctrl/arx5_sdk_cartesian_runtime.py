from __future__ import annotations

from dataclasses import dataclass
import importlib
from pathlib import Path
import time
from typing import Callable

from armctrl.motion_runtime import (
    JointStateSnapshot,
    MotionAuditSample,
    MotionExecutionResult,
    MotionMode,
)


@dataclass
class Arx5SdkCartesianRuntimeBackend:
    """ARX5 SDK Cartesian controller adapter for runtime-owned EEF commands.

    This adapter intentionally does not provide a joint fallback. It sends SDK
    EEFState commands through Arx5CartesianController after the ArmRuntime queue
    has granted ownership and performed live-readiness checks.
    """

    model: str = "X5"
    interface: str = "can0"
    arx5_module: object | None = None
    controller: object | None = None
    urdf_path: str | None = None
    gravity_compensation: bool = True
    controller_dt_s: float | None = None
    preview_time_s: float | None = None
    resume_gain_duration_s: float = 0.4
    monotonic: Callable[[], float] = time.monotonic
    sleep: Callable[[float], None] = time.sleep

    _DEFAULT_X5_CAMERA_URDF = (
        Path(__file__).resolve().parents[2] / "configs" / "models" / "X5_camera.urdf"
    )

    def enter_hold_or_damping(self) -> None:
        self._ensure_controller()

    def send_joint_command(
        self,
        q: tuple[float, ...],
        *,
        producer: str,
        mode: MotionMode,
        monotonic_s: float,
        trajectory_time_s: float | None = None,
        dq: tuple[float, ...] | None = None,
    ) -> None:
        raise RuntimeError(
            "arx5 sdk_cartesian backend cannot execute joint commands; "
            "use arx5_sdk joint backend for SysID/trajectory replay"
        )

    def hold(self) -> str:
        controller = self._ensure_controller()
        current = self._current_eef_state(controller)
        cmd = self._new_eef_state()
        cmd.pose_6d()[:] = current["pose_6d"]
        cmd.gripper_pos = current["gripper_pos"]
        cmd.gripper_vel = 0.0
        cmd.gripper_torque = 0.0
        cmd.timestamp = self._sdk_timestamp(controller) + self._effective_preview_s(controller)
        controller.set_eef_cmd(cmd)
        return MotionMode.HOLD.value

    def damping(self) -> None:
        if self.controller is None:
            return
        self.controller.set_to_damping()

    def read_joint_state(self) -> JointStateSnapshot:
        controller = self._ensure_controller()
        joint_state = controller.get_joint_state()
        pos = joint_state.pos()
        vel = joint_state.vel()
        torque = joint_state.torque()
        return JointStateSnapshot(
            q_meas=tuple(float(value) for value in pos),
            dq_meas=tuple(float(value) for value in vel),
            tau_meas=tuple(float(value) for value in torque),
            fault_flags=_fault_flags_from_sources(joint_state, controller),
        )

    def execute_eef_command(
        self,
        command: dict[str, object],
        *,
        owner: str,
        watchdog=None,
        on_sample=None,
    ) -> MotionExecutionResult:
        controller = self._ensure_controller()
        watchdog_event = watchdog() if watchdog is not None else None
        if watchdog_event is not None:
            self.damping()
            return _eef_result(
                status="faulted",
                owner=owner,
                sent_times=[],
                sample=None,
                controller_dt_s=self.controller_dt_s,
                landing_mode=MotionMode.DAMPING.value,
                error={"type": "Watchdog", "message": str(watchdog_event)},
            )
        before_state = self.read_joint_state()
        target_pose = self._target_pose_from_command(command, controller=controller)
        self._prepare_cartesian_takeover(controller)
        sent_s = float(self.monotonic())
        cmd = self._new_eef_state()
        cmd.pose_6d()[:] = target_pose
        cmd.gripper_pos = self._current_eef_state(controller)["gripper_pos"]
        cmd.gripper_vel = 0.0
        cmd.gripper_torque = 0.0
        cmd.timestamp = self._sdk_timestamp(controller) + self._effective_preview_s(controller)
        controller.set_eef_cmd(cmd)
        after_state = self.read_joint_state()
        sample = MotionAuditSample(
            sent_monotonic_s=sent_s,
            q_cmd=before_state.q_meas,
            dq_cmd=tuple(0.0 for _ in before_state.q_meas),
            q_meas=after_state.q_meas,
            dq_meas=after_state.dq_meas,
            tau_meas=after_state.tau_meas,
            fault_flags=after_state.fault_flags,
            producer=owner,
            mode=MotionMode.AGENT_SERVO.value,
        )
        if on_sample is not None:
            on_sample(sample)
        if after_state.fault_flags:
            self.damping()
            return _eef_result(
                status="faulted",
                owner=owner,
                sent_times=[sent_s],
                sample=sample,
                controller_dt_s=self.controller_dt_s,
                landing_mode=MotionMode.DAMPING.value,
            )
        return _eef_result(
            status="completed",
            owner=owner,
            sent_times=[sent_s],
            sample=sample,
            controller_dt_s=self.controller_dt_s,
            landing_mode=MotionMode.HOLD.value,
        )

    def _ensure_controller(self):
        if self.controller is not None:
            if self.controller_dt_s is None:
                self.controller_dt_s = _controller_dt_s(self.controller)
            return self.controller
        arx5 = self._load_arx5()
        robot_config = arx5.RobotConfigFactory.get_instance().get_config(self.model)
        resolved_urdf = self._resolve_urdf_path()
        if resolved_urdf is not None:
            robot_config.urdf_path = str(resolved_urdf)
        controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "cartesian_controller",
            robot_config.joint_dof,
        )
        controller_dt_s = self.controller_dt_s or _positive_float_or_none(
            _sdk_attr_value(controller_config, "controller_dt")
        )
        if controller_dt_s is not None:
            controller_config.controller_dt = float(controller_dt_s)
            self.controller_dt_s = float(controller_dt_s)
        controller_config.gravity_compensation = bool(self.gravity_compensation)
        if hasattr(controller_config, "background_send_recv"):
            controller_config.background_send_recv = True
        self.controller = arx5.Arx5CartesianController(
            robot_config,
            controller_config,
            self.interface,
        )
        if self.controller_dt_s is None:
            self.controller_dt_s = _controller_dt_s(self.controller)
        return self.controller

    def _load_arx5(self):
        if self.arx5_module is None:
            self.arx5_module = importlib.import_module("arx5_interface")
        return self.arx5_module

    def _resolve_urdf_path(self) -> Path | None:
        if self.urdf_path:
            path = Path(self.urdf_path).expanduser().resolve()
            if not path.is_file():
                raise FileNotFoundError(f"URDF file not found: {path}")
            return path
        if self.model == "X5" and self._DEFAULT_X5_CAMERA_URDF.is_file():
            return self._DEFAULT_X5_CAMERA_URDF
        return None

    def _new_eef_state(self):
        arx5 = self._load_arx5()
        return arx5.EEFState()

    def _current_eef_state(self, controller) -> dict[str, object]:
        state = controller.get_eef_state()
        return {
            "pose_6d": [float(value) for value in state.pose_6d()],
            "gripper_pos": float(getattr(state, "gripper_pos", 0.0)),
        }

    def _target_pose_from_command(
        self,
        command: dict[str, object],
        *,
        controller,
    ) -> list[float]:
        kind = command.get("kind")
        eef_command = command.get("eef_command")
        if not isinstance(eef_command, dict):
            raise ValueError("EEF command requires eef_command payload")
        current_pose = list(self._current_eef_state(controller)["pose_6d"])
        if len(current_pose) != 6:
            raise ValueError("SDK EEF pose_6d must contain 6 values")
        if kind == "eef_pose_delta":
            delta = _triple(eef_command.get("delta_position_m")) + _triple(
                eef_command.get("delta_rpy_rad")
            )
        elif kind == "eef_twist":
            control_period_s = float(eef_command.get("control_period_s"))
            if control_period_s <= 0.0:
                raise ValueError("control_period_s must be positive")
            delta = [
                value * control_period_s
                for value in (
                    _triple(eef_command.get("linear_mps"))
                    + _triple(eef_command.get("angular_rps"))
                )
            ]
        else:
            raise ValueError(f"unsupported ARX5 SDK Cartesian EEF command kind: {kind}")
        return [pose + step for pose, step in zip(current_pose, delta, strict=True)]

    def _prepare_cartesian_takeover(self, controller) -> None:
        self._sync_eef_target_to_current_state(controller)
        self._ensure_motion_gain(controller)

    def _sync_eef_target_to_current_state(self, controller) -> None:
        current = self._current_eef_state(controller)
        cmd = self._new_eef_state()
        cmd.pose_6d()[:] = current["pose_6d"]
        cmd.gripper_pos = current["gripper_pos"]
        cmd.gripper_vel = 0.0
        cmd.gripper_torque = 0.0
        controller_dt_s = self.controller_dt_s or _controller_dt_s(controller)
        cmd.timestamp = self._sdk_timestamp(controller) + max(controller_dt_s, 0.002)
        controller.set_eef_cmd(cmd)
        self.sleep(max(controller_dt_s, 0.002))

    def _ensure_motion_gain(self, controller) -> None:
        get_gain = getattr(controller, "get_gain", None)
        set_gain = getattr(controller, "set_gain", None)
        get_controller_config = getattr(controller, "get_controller_config", None)
        if not callable(get_gain) or not callable(set_gain) or not callable(get_controller_config):
            return
        target_gain = self._build_default_gain(get_controller_config())
        if target_gain is None:
            return
        current_gain = get_gain()
        if self._gain_matches(current_gain, target_gain):
            return
        dt_s = self.controller_dt_s or _controller_dt_s(controller)
        steps = max(1, int(round(float(self.resume_gain_duration_s) / dt_s)))
        for step_index in range(1, steps + 1):
            alpha = step_index / steps
            set_gain(current_gain * (1.0 - alpha) + target_gain * alpha)
            if step_index < steps:
                self.sleep(dt_s)

    def _build_default_gain(self, controller_config):
        arx5 = self._load_arx5()
        gain_cls = getattr(arx5, "Gain", None)
        if gain_cls is None:
            return None
        values = [
            _sdk_attr_value(controller_config, "default_kp"),
            _sdk_attr_value(controller_config, "default_kd"),
            _sdk_attr_value(controller_config, "default_gripper_kp"),
            _sdk_attr_value(controller_config, "default_gripper_kd"),
        ]
        if any(value is None for value in values):
            return None
        return gain_cls(*values)

    def _gain_matches(self, left_gain, right_gain) -> bool:
        return (
            _numeric_attrs_match(left_gain, right_gain, "kp")
            and _numeric_attrs_match(left_gain, right_gain, "kd")
            and _numeric_attrs_match(left_gain, right_gain, "gripper_kp")
            and _numeric_attrs_match(left_gain, right_gain, "gripper_kd")
        )

    def _sdk_timestamp(self, controller) -> float:
        get_timestamp = getattr(controller, "get_timestamp", None)
        return float(get_timestamp()) if callable(get_timestamp) else 0.0

    def _effective_preview_s(self, controller) -> float:
        if self.preview_time_s is not None:
            return max(0.0, float(self.preview_time_s))
        config = controller.get_controller_config()
        preview_s = _positive_float_or_none(
            _sdk_attr_value(config, "default_preview_time")
        )
        if preview_s is not None:
            return preview_s
        return max(0.04, (_controller_dt_s(controller) or 0.002) * 5.0)


def _eef_result(
    *,
    status: str,
    owner: str,
    sent_times: list[float],
    sample: MotionAuditSample | None,
    controller_dt_s: float | None,
    landing_mode: str,
    error: dict[str, str] | None = None,
) -> MotionExecutionResult:
    return MotionExecutionResult(
        status=status,
        producer=owner,
        mode=MotionMode.AGENT_SERVO.value,
        trajectory_sample_hz=None,
        actual_send_hz=None,
        send_jitter_ms_p95=None,
        send_jitter_ms_p99=None,
        controller_dt_s=controller_dt_s,
        samples=tuple([] if sample is None else [sample]),
        landing_mode=landing_mode,
        error=error,
    )


def _controller_dt_s(controller) -> float:
    get_controller_config = getattr(controller, "get_controller_config", None)
    config = get_controller_config() if callable(get_controller_config) else None
    value = _positive_float_or_none(_sdk_attr_value(config, "controller_dt"))
    return value if value is not None else 0.002


def _sdk_attr_value(source: object, name: str):
    if source is None:
        return None
    try:
        value = getattr(source, name)
    except AttributeError:
        return None
    if callable(value):
        try:
            value = value()
        except TypeError:
            return None
    return value


def _positive_float_or_none(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if parsed <= 0.0:
        return None
    return parsed


def _numeric_attrs_match(left: object, right: object, name: str) -> bool:
    left_values = _float_tuple_or_none(_sdk_attr_value(left, name))
    right_values = _float_tuple_or_none(_sdk_attr_value(right, name))
    if left_values is None or right_values is None:
        return False
    if len(left_values) != len(right_values):
        return False
    return all(
        abs(left_value - right_value) <= 1e-9
        for left_value, right_value in zip(left_values, right_values, strict=True)
    )


def _float_tuple_or_none(value: object) -> tuple[float, ...] | None:
    if isinstance(value, (str, bytes)):
        parsed = _positive_float_or_none(value)
        return (parsed,) if parsed is not None else None
    try:
        iterator = iter(value)  # type: ignore[arg-type]
    except TypeError:
        parsed = _positive_float_or_none(value)
        return (parsed,) if parsed is not None else None
    values: list[float] = []
    for item in iterator:
        try:
            values.append(float(item))
        except (TypeError, ValueError):
            return None
    return tuple(values)


def _fault_flags_from_sources(*sources: object) -> tuple[str, ...]:
    names = ("fault_flags", "faults", "error_flags", "errors", "fault", "error")
    flags: list[str] = []
    for source in sources:
        for name in names:
            try:
                value = getattr(source, name)
            except AttributeError:
                continue
            except Exception as error:
                flags.append(f"{name}_read_failed:{error.__class__.__name__}")
                continue
            if callable(value):
                try:
                    value = value()
                except TypeError:
                    continue
                except Exception as error:
                    flags.append(f"{name}_read_failed:{error.__class__.__name__}")
                    continue
            flags.extend(_fault_flag_tokens(value, source_name=name))
    return tuple(dict.fromkeys(flags))


def _fault_flag_tokens(value: object, *, source_name: str) -> list[str]:
    if value is None or value is False or value == 0 or value == "":
        return []
    if value is True:
        return [source_name]
    if isinstance(value, str):
        return [value]
    if isinstance(value, dict):
        tokens: list[str] = []
        for key, item in value.items():
            if item is None or item is False or item == 0 or item == "":
                continue
            tokens.append(str(key) if item is True else f"{key}:{item}")
        return tokens
    if isinstance(value, set):
        return [str(item) for item in sorted(value, key=str) if item]
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value if item]
    return [f"{source_name}:{value}"]


def _triple(values: object) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError("ARX5 SDK Cartesian EEF vector must contain exactly 3 values")
    return [float(value) for value in values]
