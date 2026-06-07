from __future__ import annotations

from dataclasses import dataclass
import importlib
import importlib.util
import math
import time

from armctrl.acceptance import build_real_motion_acceptance
from armctrl.motion_runtime import (
    FakeMotionBackend,
    JointTrajectoryPoint,
    MotionExecutionResult,
    MotionRuntime,
)
from armctrl.runtime_session import (
    runtime_status_prerequisite_status,
    runtime_status_summary,
)
from armctrl.sysid_run import Arx5InterfaceCollectionBackend

SDK_HOLD_DAMPING_CONFIRMATION = "I UNDERSTAND THIS WILL CHANGE THE ARM CONTROL MODE"
SDK_TINY_MOTION_CONFIRMATION = "I UNDERSTAND THIS WILL MOVE ONE JOINT A TINY AMOUNT"
SDK_TINY_MOTION_EXECUTE_CONFIRMATION = (
    "I UNDERSTAND THIS WILL EXECUTE THE TINY MOTION PLAN"
)
SDK_ARM_SESSION_CONFIRMATION = "I UNDERSTAND THIS WILL ARM THE SDK SESSION WITHOUT MOVING"
SDK_RECOVER_STARTUP_CONFIRMATION = (
    "I UNDERSTAND THIS WILL RECOVER THE REAL ARM TO STARTUP POSE"
)
SDK_JOG_REAL_CONFIRMATION = "I UNDERSTAND THIS WILL JOG THE REAL ARM"
DEFAULT_Q_CURRENT_MAX_ERROR_RAD = 0.02


class SdkMeasuredStateMismatchError(RuntimeError):
    def __init__(
        self,
        *,
        operator_q_current: tuple[float, ...],
        measured_q_current: tuple[float, ...],
        max_error_rad: float,
    ) -> None:
        error_rad = tuple(
            measured - operator
            for operator, measured in zip(operator_q_current, measured_q_current)
        )
        self.operator_q_current = operator_q_current
        self.measured_q_current = measured_q_current
        self.error_rad = error_rad
        self.max_error_rad = float(max_error_rad)
        self.max_abs_error_rad = max(abs(value) for value in error_rad)
        super().__init__(
            "q_current does not match measured SDK state: "
            f"max_abs_error_rad={self.max_abs_error_rad:.6f} "
            f"> max_error_rad={self.max_error_rad:.6f}"
        )

    def to_rejection_payload(self) -> dict[str, object]:
        return {
            "operator_q_current": list(self.operator_q_current),
            "measured_q_current": list(self.measured_q_current),
            "q_current_error_rad": list(self.error_rad),
            "q_current_error_max_abs_rad": self.max_abs_error_rad,
            "q_current_max_error_rad": self.max_error_rad,
        }


@dataclass(frozen=True)
class SdkPreflightResult:
    model: str
    interface: str
    sdk: dict[str, str]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": "armctrl.sysid_sdk_preflight.v1",
            "model": self.model,
            "interface": self.interface,
            "read_only": True,
            "movement_allowed": False,
            "sdk": self.sdk,
            "next_gate": "sdk_runner_confirm_then_hardware_validation",
            "notes": [
                "preflight only checks importability and requested session labels",
                "it does not open CAN, instantiate hardware objects, or send motion commands",
            ],
        }


class SdkPreflight:
    def run(self, *, model: str, interface: str) -> SdkPreflightResult:
        return SdkPreflightResult(
            model=model,
            interface=interface,
            sdk=_module_status("arx5_interface"),
        )


@dataclass(frozen=True)
class SdkDoctorResult:
    model: str
    interface: str
    sdk: dict[str, str]
    interface_status: dict[str, object]
    controller: dict[str, object]
    state_read: dict[str, object]
    timestamp_policy: dict[str, object]
    measurement_window: dict[str, object]
    motion_commands_sent: bool

    def to_json(self) -> dict[str, object]:
        return {
            "schema": "armctrl.sysid_sdk_doctor.v1",
            "model": self.model,
            "interface": self.interface,
            "read_only": True,
            "movement_allowed": False,
            "sdk": self.sdk,
            "interface_status": self.interface_status,
            "controller": self.controller,
            "state_read": self.state_read,
            "timestamp_policy": self.timestamp_policy,
            "measurement_window": self.measurement_window,
            "motion_commands_sent": self.motion_commands_sent,
            "doctor_gate": _sdk_doctor_gate(
                interface_status=self.interface_status,
                controller=self.controller,
                state_read=self.state_read,
                timestamp_policy=self.timestamp_policy,
                motion_commands_sent=self.motion_commands_sent,
            ),
            "next_gate": "hold_damping_validation_before_tiny_motion",
            "notes": [
                "doctor initializes the SDK controller only to read config and state",
                "doctor must not call reset, trajectory, joint command, or damping motion APIs",
            ],
        }


class SdkDoctor:
    def __init__(
        self,
        *,
        arx5_module: object | None = None,
        monotonic=time.monotonic,
        sleep=time.sleep,
    ) -> None:
        self._arx5 = arx5_module
        self._monotonic = monotonic
        self._sleep = sleep

    def run(
        self,
        *,
        model: str,
        interface: str,
        state_sample_count: int = 10,
        state_sample_period_s: float = 0.01,
    ) -> SdkDoctorResult:
        if state_sample_count < 1:
            raise ValueError("state_sample_count must be >= 1")
        if state_sample_period_s < 0.0:
            raise ValueError("state_sample_period_s must be >= 0")
        arx5 = self._load_arx5()
        if arx5 is None:
            return SdkDoctorResult(
                model=model,
                interface=interface,
                sdk=_module_status("arx5_interface"),
                interface_status={
                    "interface": interface,
                    "status": "not_checked",
                    "reason": "sdk_missing",
                },
                controller={"status": "sdk_missing", "controller_dt_s": None},
                state_read={"status": "not_run", "sample_count": 0, "state_read_hz": None},
                timestamp_policy={
                    "clock": "host_monotonic",
                    "monotonic": None,
                },
                measurement_window=_measurement_window(
                    read_times_s=[],
                    requested_sample_count=state_sample_count,
                    requested_sample_period_s=state_sample_period_s,
                ),
                motion_commands_sent=False,
            )
        robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
        controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "joint_controller",
            robot_config.joint_dof,
        )
        controller_dt_s = _optional_float(
            getattr(controller_config, "controller_dt", None)
        )
        try:
            controller = arx5.Arx5JointController(
                robot_config,
                controller_config,
                interface,
            )
        except Exception as exc:
            return SdkDoctorResult(
                model=model,
                interface=interface,
                sdk={"module": "arx5_interface", "status": "available"},
                interface_status={
                    "interface": interface,
                    "status": "open_failed",
                    "reason": str(exc),
                },
                controller={
                    "status": "init_failed",
                    "controller_dt_s": controller_dt_s,
                    "joint_dof": int(robot_config.joint_dof),
                },
                state_read={
                    "status": "not_run",
                    "sample_count": 0,
                    "state_read_hz": None,
                },
                timestamp_policy={
                    "clock": "host_monotonic",
                    "monotonic": None,
                },
                measurement_window=_measurement_window(
                    read_times_s=[],
                    requested_sample_count=state_sample_count,
                    requested_sample_period_s=state_sample_period_s,
                ),
                motion_commands_sent=False,
            )
        read_times: list[float] = []
        last_q_meas: tuple[float, ...] | None = None
        try:
            for sample_index in range(state_sample_count):
                read_s = float(self._monotonic())
                joint_state = controller.get_joint_state()
                last_q_meas = tuple(float(value) for value in joint_state.pos())
                read_times.append(read_s)
                if sample_index < state_sample_count - 1:
                    self._sleep(state_sample_period_s)
        except Exception as exc:
            return SdkDoctorResult(
                model=model,
                interface=interface,
                sdk={"module": "arx5_interface", "status": "available"},
                interface_status={
                    "interface": interface,
                    "status": "opened_read_only",
                },
                controller={
                    "status": "initialized_read_only",
                    "controller_dt_s": controller_dt_s,
                    "joint_dof": int(robot_config.joint_dof),
                },
                state_read={
                    "status": "failed",
                    "sample_count": len(read_times),
                    "state_read_hz": _sample_hz_from_times(read_times),
                    "q_meas_last": (
                        list(last_q_meas)
                        if last_q_meas is not None
                        else None
                    ),
                    "reason": str(exc),
                },
                timestamp_policy={
                    "clock": "host_monotonic",
                    "monotonic": _is_monotonic(read_times) if read_times else None,
                },
                measurement_window=_measurement_window(
                    read_times_s=read_times,
                    requested_sample_count=state_sample_count,
                    requested_sample_period_s=state_sample_period_s,
                ),
                motion_commands_sent=False,
            )
        return SdkDoctorResult(
            model=model,
            interface=interface,
            sdk={"module": "arx5_interface", "status": "available"},
            interface_status={
                "interface": interface,
                "status": "opened_read_only",
            },
            controller={
                "status": "initialized_read_only",
                "controller_dt_s": controller_dt_s,
                "joint_dof": int(robot_config.joint_dof),
            },
            state_read={
                "status": "ok",
                "sample_count": state_sample_count,
                "state_read_hz": _sample_hz_from_times(read_times),
                "q_meas_last": (
                    list(last_q_meas)
                    if last_q_meas is not None
                    else None
                ),
            },
            timestamp_policy={
                "clock": "host_monotonic",
                "monotonic": _is_monotonic(read_times),
            },
            measurement_window=_measurement_window(
                read_times_s=read_times,
                requested_sample_count=state_sample_count,
                requested_sample_period_s=state_sample_period_s,
            ),
            motion_commands_sent=False,
        )

    def _load_arx5(self):
        if self._arx5 is not None:
            return self._arx5
        try:
            self._arx5 = importlib.import_module("arx5_interface")
        except ImportError:
            return None
        return self._arx5


@dataclass(frozen=True)
class SdkHoldDampingCheckResult:
    model: str
    interface: str
    sdk: dict[str, str]
    hold: dict[str, object]
    damping: dict[str, object]
    joint_commands_sent: bool
    prerequisites: dict[str, str]

    def to_json(self) -> dict[str, object]:
        mode_call_sequence = _mode_call_sequence(hold=self.hold, damping=self.damping)
        landing_policy = _landing_policy(hold=self.hold, damping=self.damping)
        hold_damping_gate = _sdk_hold_damping_gate(
            hold=self.hold,
            damping=self.damping,
            mode_call_sequence=mode_call_sequence,
            joint_commands_sent=self.joint_commands_sent,
        )
        return {
            "schema": "armctrl.sysid_sdk_hold_damping_check.v1",
            "model": self.model,
            "interface": self.interface,
            "movement_allowed": False,
            "mode_change_allowed": hold_damping_gate["status"] == "pass",
            "requires_confirm": SDK_HOLD_DAMPING_CONFIRMATION,
            "sdk": self.sdk,
            "landing_policy": landing_policy,
            "hold": self.hold,
            "damping": self.damping,
            "mode_call_sequence": mode_call_sequence,
            "joint_commands_sent": self.joint_commands_sent,
            "prerequisites": self.prerequisites,
            "hold_damping_gate": hold_damping_gate,
            "fault_landing_mode": "damping",
            "next_gate": "tiny_motion_requires_separate_confirmation_and_limits",
            "notes": [
                "this gate may change controller mode but must not send joint commands",
                "run only after sdk-doctor has produced a valid controller/state artifact",
            ],
        }


class SdkHoldDampingCheck:
    def __init__(self, *, arx5_module: object | None = None) -> None:
        self._arx5 = arx5_module

    def run(
        self,
        *,
        model: str,
        interface: str,
        confirm: str,
        doctor_artifact: dict[str, object] | None = None,
    ) -> SdkHoldDampingCheckResult:
        if confirm != SDK_HOLD_DAMPING_CONFIRMATION:
            raise PermissionError(
                "sdk hold/damping check requires explicit operator confirmation"
            )
        prerequisites: dict[str, str] = {}
        if doctor_artifact is not None:
            prerequisites["doctor"] = _doctor_prerequisite_status(doctor_artifact)
            if prerequisites["doctor"] != "pass":
                raise RuntimeError("sdk doctor prerequisite failed")
        arx5 = self._load_arx5()
        if arx5 is None:
            return SdkHoldDampingCheckResult(
                model=model,
                interface=interface,
                sdk=_module_status("arx5_interface"),
                hold={"status": "not_run", "reason": "sdk_missing"},
                damping={"status": "not_run", "reason": "sdk_missing"},
                joint_commands_sent=False,
                prerequisites=prerequisites,
            )
        robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
        controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "joint_controller",
            robot_config.joint_dof,
        )
        controller = arx5.Arx5JointController(
            robot_config,
            controller_config,
            interface,
        )
        hold = _call_optional_controller_mode(controller, "set_to_hold")
        damping = _call_optional_controller_mode(controller, "set_to_damping")
        return SdkHoldDampingCheckResult(
            model=model,
            interface=interface,
            sdk={"module": "arx5_interface", "status": "available"},
            hold=hold,
            damping=damping,
            joint_commands_sent=False,
            prerequisites=prerequisites,
        )

    def _load_arx5(self):
        if self._arx5 is not None:
            return self._arx5
        try:
            self._arx5 = importlib.import_module("arx5_interface")
        except ImportError:
            return None
        return self._arx5


@dataclass(frozen=True)
class SdkArmSessionResult:
    model: str
    interface: str
    controller_dt_s: float | None
    q_meas: tuple[float, ...]
    landing_policy: str
    prerequisites: dict[str, str]
    joint_commands_sent: bool = False

    def to_json(self) -> dict[str, object]:
        return {
            "schema": "armctrl.sdk_arm_session.v1",
            "model": self.model,
            "interface": self.interface,
            "session_status": "armed",
            "movement_allowed": True,
            "joint_commands_sent": self.joint_commands_sent,
            "requires_confirm": SDK_ARM_SESSION_CONFIRMATION,
            "prerequisites": self.prerequisites,
            "controller": {
                "controller_dt_s": self.controller_dt_s,
            },
            "state": {
                "q_meas": list(self.q_meas),
            },
            "landing_policy": self.landing_policy,
            "fault_landing_mode": "damping",
            "next_gate": "sdk-jog-real-or-agent-sysid-runtime",
            "notes": [
                "session arming opens the SDK and reads measured state without joint commands",
                "downstream real motion must re-check measured state before sending commands",
            ],
        }


class SdkArmSession:
    def __init__(
        self,
        *,
        arx5_module: object | None = None,
        monotonic=time.monotonic,
        sleep=time.sleep,
    ) -> None:
        self._arx5 = arx5_module
        self._monotonic = monotonic
        self._sleep = sleep

    def run(
        self,
        *,
        model: str,
        interface: str,
        confirm: str,
        doctor_artifact: dict[str, object],
        hold_damping_artifact: dict[str, object],
    ) -> SdkArmSessionResult:
        if confirm != SDK_ARM_SESSION_CONFIRMATION:
            raise PermissionError("sdk arm session requires explicit operator confirmation")
        prerequisites = tiny_motion_execute_prerequisite_statuses(
            doctor_artifact=doctor_artifact,
            hold_damping_artifact=hold_damping_artifact,
        )
        failed = [name for name, status in prerequisites.items() if status != "pass"]
        if failed:
            raise RuntimeError("sdk arm session prerequisites failed: " + ", ".join(failed))
        backend = Arx5InterfaceCollectionBackend(
            model=model,
            interface=interface,
            arx5_module=self._arx5,
            controller_dt_s=_controller_dt_from_doctor_artifact(doctor_artifact),
            monotonic=self._monotonic,
            sleep=self._sleep,
        )
        backend.enter_hold_or_damping()
        try:
            state = backend.read_joint_state()
            landing_policy = _session_landing_policy_from_artifact(
                hold_damping_artifact
            )
            return SdkArmSessionResult(
                model=model,
                interface=interface,
                controller_dt_s=getattr(backend, "controller_dt_s", None),
                q_meas=state.q_meas,
                landing_policy=landing_policy,
                prerequisites=prerequisites,
            )
        finally:
            backend.damping()


@dataclass(frozen=True)
class SdkTinyMotionPlanResult:
    joint_index: int
    delta_rad: float
    max_delta_rad: float
    prerequisites: dict[str, str]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
            "movement_allowed": False,
            "tiny_motion_eligible": True,
            "requires_confirm": SDK_TINY_MOTION_CONFIRMATION,
            "command": {
                "joint_index": self.joint_index,
                "delta_rad": self.delta_rad,
            },
            "limits": {
                "max_delta_rad": self.max_delta_rad,
            },
            "prerequisites": self.prerequisites,
            "next_gate": "tiny_motion_execute_on_target_with_runtime_logs",
            "notes": [
                "this command only creates a plan; it must not move hardware",
                "execution requires a separate runtime log with q_cmd and q_meas",
            ],
        }


class SdkTinyMotionPlanner:
    def __init__(self, *, max_delta_rad: float = 0.005) -> None:
        if max_delta_rad <= 0.0:
            raise ValueError("max_delta_rad must be positive")
        self._max_delta_rad = float(max_delta_rad)

    def plan(
        self,
        *,
        doctor_artifact: dict[str, object],
        hold_damping_artifact: dict[str, object],
        joint_index: int,
        delta_rad: float,
        confirm: str,
    ) -> SdkTinyMotionPlanResult:
        if confirm != SDK_TINY_MOTION_CONFIRMATION:
            raise PermissionError(
                "tiny motion plan requires explicit operator confirmation"
            )
        if joint_index < 1:
            raise ValueError("joint_index must be >= 1")
        if abs(delta_rad) > self._max_delta_rad:
            raise ValueError(
                f"delta_rad must be within +/-{self._max_delta_rad:.6f} rad"
            )
        prerequisites = {
            "doctor": _doctor_prerequisite_status(doctor_artifact),
            "hold_damping": _hold_damping_prerequisite_status(hold_damping_artifact),
        }
        failed = [
            name
            for name, status in prerequisites.items()
            if status != "pass"
        ]
        if failed:
            raise RuntimeError(
                "tiny motion prerequisites failed: " + ", ".join(failed)
            )
        return SdkTinyMotionPlanResult(
            joint_index=joint_index,
            delta_rad=float(delta_rad),
            max_delta_rad=self._max_delta_rad,
            prerequisites=prerequisites,
        )


@dataclass(frozen=True)
class SdkTinyMotionExecuteResult:
    backend: str
    q_start: tuple[float, ...]
    q_target: tuple[float, ...]
    motion_result: MotionExecutionResult
    prerequisites: dict[str, str]

    def to_json(self) -> dict[str, object]:
        motion_runtime = _runtime_result_manifest(self.motion_result)
        tracking = _tiny_motion_tracking_summary(motion_runtime)
        motion_runtime["tracking"] = tracking
        movement_command_sent = bool(self.motion_result.samples)
        return {
            "schema": "armctrl.sysid_sdk_tiny_motion_execute.v1",
            "backend": self.backend,
            "run_status": self.motion_result.status,
            "hardware_motion": self.backend != "fake",
            "movement_command_sent": movement_command_sent,
            "prerequisites": self.prerequisites,
            "q_start": list(self.q_start),
            "q_target": list(self.q_target),
            "motion_runtime": motion_runtime,
            "tracking": tracking,
            "acceptance": build_real_motion_acceptance(
                stage="tiny_motion",
                motion_runtime=motion_runtime,
                hardware_motion=self.backend != "fake",
                movement_command_sent=movement_command_sent,
                readiness_passed=(
                    all(status == "pass" for status in self.prerequisites.values())
                    if self.prerequisites
                    else None
                ),
            ),
            "fault_landing_mode": "damping",
        }


class SdkTinyMotionExecutor:
    def __init__(
        self,
        *,
        arx5_module: object | None = None,
        max_q_current_error_rad: float = DEFAULT_Q_CURRENT_MAX_ERROR_RAD,
        monotonic=time.monotonic,
        sleep=time.sleep,
    ) -> None:
        if max_q_current_error_rad <= 0.0:
            raise ValueError("max_q_current_error_rad must be positive")
        self._arx5 = arx5_module
        self._max_q_current_error_rad = float(max_q_current_error_rad)
        self._monotonic = monotonic
        self._sleep = sleep
        self.fake_backend = FakeMotionBackend()

    def execute(
        self,
        *,
        plan_artifact: dict[str, object],
        backend_name: str,
        model: str | None = None,
        interface: str | None = None,
        dof: int,
        q_current: tuple[float, ...],
        confirm: str,
        send_hz: float = 50.0,
        doctor_artifact: dict[str, object] | None = None,
        hold_damping_artifact: dict[str, object] | None = None,
        max_q_current_error_rad: float | None = None,
    ) -> SdkTinyMotionExecuteResult:
        if confirm != SDK_TINY_MOTION_EXECUTE_CONFIRMATION:
            raise PermissionError(
                "tiny motion execute requires explicit operator confirmation"
            )
        if send_hz <= 0.0:
            raise ValueError("send_hz must be positive")
        prerequisites: dict[str, str] = {}
        if backend_name == "arx5_sdk":
            prerequisites = tiny_motion_execute_prerequisite_statuses(
                doctor_artifact=doctor_artifact,
                hold_damping_artifact=hold_damping_artifact,
            )
            failed = [
                name
                for name, status in prerequisites.items()
                if status != "pass"
            ]
            if failed:
                raise RuntimeError(
                    "tiny motion execution prerequisites failed: "
                    + ", ".join(failed)
                )
        q_start = tuple(float(value) for value in q_current)
        if len(q_start) != dof:
            raise ValueError("q_current length must match dof")
        backend = self._backend_for_name(
            backend_name,
            model=model,
            interface=interface,
            doctor_artifact=doctor_artifact,
        )
        if backend_name == "arx5_sdk":
            if max_q_current_error_rad is not None and max_q_current_error_rad <= 0.0:
                raise ValueError("max_q_current_error_rad must be positive")
            measured_q_start = self._measured_q_current_or_damping(
                backend,
                operator_q_current=q_start,
                dof=dof,
                max_error_rad=(
                    self._max_q_current_error_rad
                    if max_q_current_error_rad is None
                    else float(max_q_current_error_rad)
                ),
            )
            q_start = measured_q_start
        q_target = _tiny_motion_target_from_plan(
            plan_artifact,
            dof=dof,
            q_current=q_start,
        )
        runtime = MotionRuntime(
            backend=backend,
            monotonic=self._monotonic,
            sleep=self._sleep,
        )
        motion_result = runtime.execute_trajectory(
            [
                JointTrajectoryPoint(time_s=0.0, q=q_start),
                JointTrajectoryPoint(time_s=1.0 / send_hz, q=q_target),
            ],
            producer="tiny_motion",
            trajectory_sample_hz=send_hz,
            hold_after=True,
        )
        return SdkTinyMotionExecuteResult(
            backend=backend_name,
            q_start=q_start,
            q_target=q_target,
            motion_result=motion_result,
            prerequisites=prerequisites,
        )

    def _backend_for_name(
        self,
        backend_name: str,
        *,
        model: str | None,
        interface: str | None,
        doctor_artifact: dict[str, object] | None = None,
    ):
        if backend_name == "fake":
            return self.fake_backend
        if backend_name != "arx5_sdk":
            raise NotImplementedError(f"unsupported tiny motion backend: {backend_name}")
        if model is None or interface is None:
            raise ValueError("model and interface are required for arx5_sdk backend")
        backend = Arx5InterfaceCollectionBackend(
            model=model,
            interface=interface,
            arx5_module=self._arx5,
            controller_dt_s=_controller_dt_from_doctor_artifact(doctor_artifact),
            monotonic=self._monotonic,
            sleep=self._sleep,
        )
        backend.enter_hold_or_damping()
        return backend

    def _measured_q_current_or_damping(
        self,
        backend,
        *,
        operator_q_current: tuple[float, ...],
        dof: int,
        max_error_rad: float,
    ) -> tuple[float, ...]:
        measured_q_current = tuple(float(value) for value in backend.read_joint_state().q_meas)
        if len(measured_q_current) != dof:
            backend.damping()
            raise RuntimeError(
                "measured SDK q_current length does not match dof: "
                f"{len(measured_q_current)} != {dof}"
            )
        error_rad = [
            measured - operator
            for operator, measured in zip(operator_q_current, measured_q_current)
        ]
        max_abs_error_rad = max(abs(value) for value in error_rad)
        if max_abs_error_rad > max_error_rad:
            backend.damping()
            raise SdkMeasuredStateMismatchError(
                operator_q_current=operator_q_current,
                measured_q_current=measured_q_current,
                max_error_rad=max_error_rad,
            )
        return measured_q_current


@dataclass(frozen=True)
class SdkJogRealResult:
    model: str
    interface: str
    q_start: tuple[float, ...]
    q_target: tuple[float, ...]
    motion_result: MotionExecutionResult
    session: dict[str, object]

    def to_json(self) -> dict[str, object]:
        motion_runtime = _runtime_result_manifest(self.motion_result)
        movement_command_sent = bool(self.motion_result.samples)
        return {
            "schema": "armctrl.sdk_jog_real.v1",
            "run_status": self.motion_result.status,
            "model": self.model,
            "interface": self.interface,
            "hardware_motion": True,
            "movement_command_sent": movement_command_sent,
            "requires_confirm": SDK_JOG_REAL_CONFIRMATION,
            "q_start": list(self.q_start),
            "q_target": list(self.q_target),
            "session": self.session,
            "motion_runtime": motion_runtime,
            "fault_landing_mode": "damping",
            "next_gate": (
                "agent-or-sysid-runtime"
                if self.motion_result.status == "completed"
                else "inspect-jog-runtime-before-continuing"
            ),
        }


class SdkJogReal:
    def __init__(
        self,
        *,
        arx5_module: object | None = None,
        monotonic=time.monotonic,
        sleep=time.sleep,
    ) -> None:
        self._arx5 = arx5_module
        self._monotonic = monotonic
        self._sleep = sleep

    def run(
        self,
        *,
        session_artifact: dict[str, object],
        joint_index: int,
        delta_rad: float,
        max_delta_rad: float = 0.005,
        send_hz: float = 50.0,
        confirm: str,
        max_q_current_error_rad: float = DEFAULT_Q_CURRENT_MAX_ERROR_RAD,
    ) -> SdkJogRealResult:
        if confirm != SDK_JOG_REAL_CONFIRMATION:
            raise PermissionError("sdk jog real requires explicit operator confirmation")
        if send_hz <= 0.0:
            raise ValueError("send_hz must be positive")
        if max_delta_rad <= 0.0:
            raise ValueError("max_delta_rad must be positive")
        if abs(delta_rad) > max_delta_rad:
            raise ValueError(f"delta_rad must be within +/-{max_delta_rad:.6f} rad")
        model, interface, q_start, controller_dt_s = _session_motion_inputs(
            session_artifact
        )
        dof = len(q_start)
        if not 1 <= joint_index <= dof:
            raise ValueError("joint_index is out of range")
        backend = Arx5InterfaceCollectionBackend(
            model=model,
            interface=interface,
            arx5_module=self._arx5,
            controller_dt_s=controller_dt_s,
            monotonic=self._monotonic,
            sleep=self._sleep,
        )
        backend.enter_hold_or_damping()
        try:
            measured_q_start = SdkTinyMotionExecutor(
                max_q_current_error_rad=max_q_current_error_rad,
                monotonic=self._monotonic,
                sleep=self._sleep,
            )._measured_q_current_or_damping(
                backend,
                operator_q_current=q_start,
                dof=dof,
                max_error_rad=max_q_current_error_rad,
            )
            q_target = list(measured_q_start)
            q_target[joint_index - 1] += float(delta_rad)
            runtime = MotionRuntime(
                backend=backend,
                monotonic=self._monotonic,
                sleep=self._sleep,
            )
            motion_result = runtime.execute_trajectory(
                [
                    JointTrajectoryPoint(time_s=0.0, q=measured_q_start),
                    JointTrajectoryPoint(time_s=1.0 / send_hz, q=tuple(q_target)),
                ],
                producer="sdk_jog",
                trajectory_sample_hz=send_hz,
                hold_after=True,
            )
            return SdkJogRealResult(
                model=model,
                interface=interface,
                q_start=measured_q_start,
                q_target=tuple(q_target),
                motion_result=motion_result,
                session={
                    "schema": session_artifact.get("schema"),
                    "session_status": session_artifact.get("session_status")
                    or session_artifact.get("status"),
                    "landing_policy": session_artifact.get("landing_policy"),
                },
            )
        except Exception:
            backend.damping()
            raise
        except BaseException:
            backend.damping()
            raise


@dataclass(frozen=True)
class SdkStartupRecoveryResult:
    model: str
    interface: str
    q_start: tuple[float, ...]
    q_target: tuple[float, ...]
    max_joint_step_rad: float
    motion_result: MotionExecutionResult
    active_hold: dict[str, object]
    session: dict[str, object]

    def to_json(self) -> dict[str, object]:
        motion_runtime = _runtime_result_manifest(self.motion_result)
        movement_command_sent = bool(self.motion_result.samples)
        plan = _startup_recovery_plan_summary(
            q_start=self.q_start,
            q_target=self.q_target,
            max_joint_step_rad=self.max_joint_step_rad,
            trajectory=[
                tuple(sample.q_cmd)
                for sample in self.motion_result.samples
            ],
        )
        return {
            "schema": "armctrl.sdk_startup_recovery.v1",
            "run_status": self.motion_result.status,
            "model": self.model,
            "interface": self.interface,
            "hardware_motion": True,
            "movement_command_sent": movement_command_sent,
            "requires_confirm": SDK_RECOVER_STARTUP_CONFIRMATION,
            "q_start": list(self.q_start),
            "q_target": list(self.q_target),
            "recovery_plan": plan,
            "session": self.session,
            "motion_runtime": motion_runtime,
            "active_hold": self.active_hold,
            "fault_landing_mode": "damping",
            "next_gate": (
                "sdk-jog-real-or-agent-sysid-runtime"
                if self.motion_result.status == "completed"
                else "inspect-startup-recovery-runtime-before-continuing"
            ),
            "notes": [
                "startup recovery always starts from freshly measured SDK q_meas",
                "session artifact proves SDK/mode prerequisites, not the current pose",
            ],
        }


class SdkStartupRecovery:
    def __init__(
        self,
        *,
        arx5_module: object | None = None,
        monotonic=time.monotonic,
        sleep=time.sleep,
    ) -> None:
        self._arx5 = arx5_module
        self._monotonic = monotonic
        self._sleep = sleep

    def run(
        self,
        *,
        session_artifact: dict[str, object],
        q_target: tuple[float, ...],
        send_hz: float = 50.0,
        hold_seconds: float | None = None,
        hold_hz: float = 50.0,
        max_joint_step_rad: float = 0.01,
        confirm: str,
    ) -> SdkStartupRecoveryResult:
        if confirm != SDK_RECOVER_STARTUP_CONFIRMATION:
            raise PermissionError(
                "sdk startup recovery requires explicit operator confirmation"
            )
        if send_hz <= 0.0:
            raise ValueError("send_hz must be positive")
        if hold_seconds is not None and hold_seconds < 0.0:
            raise ValueError("hold_seconds must be non-negative")
        if hold_hz <= 0.0:
            raise ValueError("hold_hz must be positive")
        if max_joint_step_rad <= 0.0:
            raise ValueError("max_joint_step_rad must be positive")
        model, interface, session_q, controller_dt_s = _session_motion_inputs(
            session_artifact
        )
        q_target_tuple = tuple(float(value) for value in q_target)
        if len(q_target_tuple) != len(session_q):
            raise ValueError("q_target length must match session state")
        backend = Arx5InterfaceCollectionBackend(
            model=model,
            interface=interface,
            arx5_module=self._arx5,
            controller_dt_s=controller_dt_s,
            monotonic=self._monotonic,
            sleep=self._sleep,
            shutdown_to_passive=False,
        )
        backend.enter_hold_or_damping()
        try:
            measured_q_start = tuple(
                float(value)
                for value in backend.read_joint_state().q_meas
            )
            if len(measured_q_start) != len(q_target_tuple):
                raise RuntimeError(
                    "measured SDK q_current length does not match q_target: "
                    f"{len(measured_q_start)} != {len(q_target_tuple)}"
                )
            trajectory = _startup_recovery_trajectory(
                q_start=measured_q_start,
                q_target=q_target_tuple,
                max_joint_step_rad=float(max_joint_step_rad),
                send_hz=float(send_hz),
            )
            runtime = MotionRuntime(
                backend=backend,
                monotonic=self._monotonic,
                sleep=self._sleep,
            )
            motion_result = runtime.execute_trajectory(
                trajectory,
                producer="sdk_startup_recovery",
                trajectory_sample_hz=send_hz,
                hold_after=True,
            )
            active_hold = backend.hold_joint_position_for_duration(
                q_target_tuple,
                duration_s=None if hold_seconds is None else float(hold_seconds),
                hold_hz=float(hold_hz),
            )
            return SdkStartupRecoveryResult(
                model=model,
                interface=interface,
                q_start=measured_q_start,
                q_target=q_target_tuple,
                max_joint_step_rad=float(max_joint_step_rad),
                motion_result=motion_result,
                active_hold=active_hold,
                session={
                    "schema": session_artifact.get("schema"),
                    "session_status": session_artifact.get("session_status")
                    or session_artifact.get("status"),
                    "landing_policy": session_artifact.get("landing_policy"),
                },
            )
        except Exception:
            backend.damping()
            raise
        except BaseException:
            backend.damping()
            raise


@dataclass(frozen=True)
class SdkHandshakeStep:
    name: str
    purpose: str
    movement_allowed: bool

    def to_json(self) -> dict[str, object]:
        return {
            "name": self.name,
            "purpose": self.purpose,
            "movement_allowed": self.movement_allowed,
        }


@dataclass(frozen=True)
class SdkHandshakePlanResult:
    model: str
    interface: str
    steps: tuple[SdkHandshakeStep, ...]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": "armctrl.sysid_sdk_handshake_plan.v1",
            "model": self.model,
            "interface": self.interface,
            "read_only": True,
            "movement_allowed": False,
            "requires_confirm": "I UNDERSTAND THIS WILL MOVE THE ARM",
            "fault_landing_mode": "damping",
            "steps": [step.to_json() for step in self.steps],
            "next_gate": "sdk_runner_confirm_then_hardware_validation",
            "notes": [
                "handshake planning is read-only and does not import or instantiate the SDK",
                "real collection must enter a verified hold or damping state before recording",
                "faults and Ctrl-C must land in damping during hardware validation",
            ],
        }


class SdkHandshakePlanner:
    def plan(self, *, model: str, interface: str) -> SdkHandshakePlanResult:
        return SdkHandshakePlanResult(
            model=model,
            interface=interface,
            steps=(
                SdkHandshakeStep(
                    name="sdk_preflight",
                    purpose="check SDK availability and requested labels without opening CAN",
                    movement_allowed=False,
                ),
                SdkHandshakeStep(
                    name="operator_confirm",
                    purpose="require explicit human confirmation before any future motion command",
                    movement_allowed=False,
                ),
                SdkHandshakeStep(
                    name="enter_hold_or_damping",
                    purpose="initialize the SDK controller without reset-to-home before collection",
                    movement_allowed=False,
                ),
                SdkHandshakeStep(
                    name="start_recording_after_safe_state",
                    purpose="start logs only after the safe state is reached and verified",
                    movement_allowed=False,
                ),
                SdkHandshakeStep(
                    name="fault_or_ctrl_c_to_damping",
                    purpose="route interruption and controller faults to damping",
                    movement_allowed=False,
                ),
            ),
        )


@dataclass(frozen=True)
class SdkAgentSysIdSmokeReadinessResult:
    prerequisites: dict[str, str]
    motion_gate_summary: dict[str, object]
    motion_gate_name: str

    def to_json(self) -> dict[str, object]:
        smoke_allowed = all(
            status == "pass" for status in self.prerequisites.values()
        )
        motion_gate_key = self.motion_gate_name
        return {
            "schema": "armctrl.sysid_agent_smoke_readiness.v1",
            "read_only": True,
            "movement_allowed": False,
            "agent_sysid_smoke_allowed": smoke_allowed,
            "prerequisites": self.prerequisites,
            motion_gate_key: self.motion_gate_summary,
            "next_gate": (
                "agent_smoke_then_sysid_smoke_on_target"
                if smoke_allowed
                else "start live ArmRuntime hold session before Agent/SysID smoke"
            ),
            "notes": [
                "readiness check only reads artifacts and does not open the SDK",
                "runtime_status proves current live hold when provided; startup/tiny artifacts are legacy gates",
                "Agent/SysID smoke still requires explicit operator approval on the target robot",
            ],
        }


class SdkAgentSysIdSmokeReadinessChecker:
    def check(
        self,
        *,
        doctor_artifact: dict[str, object],
        hold_damping_artifact: dict[str, object],
        tiny_motion_artifact: dict[str, object] | None = None,
        startup_recovery_artifact: dict[str, object] | None = None,
        runtime_status_artifact: dict[str, object] | None = None,
    ) -> SdkAgentSysIdSmokeReadinessResult:
        if runtime_status_artifact is not None:
            motion_gate_name = "runtime_status"
            motion_gate_status = runtime_status_prerequisite_status(
                runtime_status_artifact
            )
            motion_gate_summary = runtime_status_summary(runtime_status_artifact)
        elif startup_recovery_artifact is not None:
            motion_gate_name = "startup_recovery"
            motion_gate_status = _startup_recovery_prerequisite_status(
                startup_recovery_artifact
            )
            motion_gate_summary = _startup_recovery_summary(
                startup_recovery_artifact
            )
        else:
            motion_gate_name = "tiny_motion"
            motion_gate_status = _tiny_motion_execute_prerequisite_status(
                tiny_motion_artifact or {}
            )
            motion_gate_summary = _tiny_motion_execute_summary(
                tiny_motion_artifact or {}
            )
        return SdkAgentSysIdSmokeReadinessResult(
            prerequisites={
                "doctor": _doctor_prerequisite_status(doctor_artifact),
                "hold_damping": _hold_damping_prerequisite_status(
                    hold_damping_artifact
                ),
                motion_gate_name: motion_gate_status,
            },
            motion_gate_summary=motion_gate_summary,
            motion_gate_name=motion_gate_name,
        )


def _module_status(module_name: str) -> dict[str, str]:
    spec = importlib.util.find_spec(module_name)
    return {
        "module": module_name,
        "status": "available" if spec is not None else "missing",
    }


def _sdk_doctor_gate(
    *,
    interface_status: dict[str, object],
    controller: dict[str, object],
    state_read: dict[str, object],
    timestamp_policy: dict[str, object],
    motion_commands_sent: bool,
) -> dict[str, object]:
    checks = {
        "interface_opened_read_only": interface_status.get("status")
        == "opened_read_only",
        "controller_initialized_read_only": controller.get("status")
        == "initialized_read_only",
        "controller_dt_measured": controller.get("controller_dt_s") is not None,
        "state_read_ok": state_read.get("status") == "ok",
        "state_read_hz_measured": state_read.get("state_read_hz") is not None,
        "timestamps_monotonic": timestamp_policy.get("monotonic") is True,
        "no_motion_commands_sent": motion_commands_sent is False,
    }
    failed = [name for name, passed in checks.items() if not passed]
    return {
        "status": "pass" if not failed else "fail",
        "checks": checks,
        "failed_checks": failed,
    }


def _sample_hz_from_times(times_s: list[float]) -> float | None:
    if len(times_s) < 2:
        return None
    elapsed_s = times_s[-1] - times_s[0]
    if elapsed_s <= 0.0:
        return None
    return (len(times_s) - 1) / elapsed_s


def _measurement_window(
    *,
    read_times_s: list[float],
    requested_sample_count: int,
    requested_sample_period_s: float,
) -> dict[str, object]:
    if not read_times_s:
        return {
            "requested_sample_count": requested_sample_count,
            "requested_sample_period_s": requested_sample_period_s,
            "observed_sample_count": 0,
            "first_read_monotonic_s": None,
            "last_read_monotonic_s": None,
            "duration_s": None,
        }
    return {
        "requested_sample_count": requested_sample_count,
        "requested_sample_period_s": requested_sample_period_s,
        "observed_sample_count": len(read_times_s),
        "first_read_monotonic_s": read_times_s[0],
        "last_read_monotonic_s": read_times_s[-1],
        "duration_s": read_times_s[-1] - read_times_s[0],
    }


def _is_monotonic(times_s: list[float]) -> bool:
    return all(
        current >= previous
        for previous, current in zip(times_s, times_s[1:])
    )


def _optional_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


def _controller_dt_from_doctor_artifact(
    doctor_artifact: dict[str, object] | None,
) -> float | None:
    if not isinstance(doctor_artifact, dict):
        return None
    controller = doctor_artifact.get("controller")
    if not isinstance(controller, dict):
        return None
    controller_dt_s = _optional_float(controller.get("controller_dt_s"))
    if controller_dt_s is None or controller_dt_s <= 0.0:
        return None
    return controller_dt_s


def _call_optional_controller_mode(controller: object, method_name: str) -> dict[str, object]:
    method = getattr(controller, method_name, None)
    if not callable(method):
        return {"status": "unsupported", "method": method_name}
    try:
        method()
    except Exception as exc:
        return {"status": "failed", "method": method_name, "reason": str(exc)}
    return {"status": "called", "method": method_name}


def _mode_call_sequence(
    *,
    hold: dict[str, object],
    damping: dict[str, object],
) -> list[dict[str, object]]:
    return [
        {
            "name": "hold",
            "method": hold.get("method"),
            "status": hold.get("status"),
        },
        {
            "name": "damping",
            "method": damping.get("method"),
            "status": damping.get("status"),
        },
    ]


def _mode_call_sequence_is_safe(value: object) -> bool:
    if not isinstance(value, list) or len(value) != 2:
        return False
    hold, damping = value
    if not isinstance(hold, dict) or not isinstance(damping, dict):
        return False
    hold_then_damping = (
        hold.get("name") == "hold"
        and hold.get("method") == "set_to_hold"
        and hold.get("status") == "called"
        and damping.get("name") == "damping"
        and damping.get("method") == "set_to_damping"
        and damping.get("status") == "called"
    )
    damping_only = (
        hold.get("name") == "hold"
        and hold.get("method") == "set_to_hold"
        and hold.get("status") == "unsupported"
        and damping.get("name") == "damping"
        and damping.get("method") == "set_to_damping"
        and damping.get("status") == "called"
    )
    return hold_then_damping or damping_only


def _landing_policy(
    *,
    hold: dict[str, object],
    damping: dict[str, object],
) -> str:
    if hold.get("status") == "called" and damping.get("status") == "called":
        return "hold_then_damping"
    if hold.get("status") == "unsupported" and damping.get("status") == "called":
        return "damping_only"
    return "unsupported"


def _sdk_hold_damping_gate(
    *,
    hold: dict[str, object],
    damping: dict[str, object],
    mode_call_sequence: list[dict[str, object]],
    joint_commands_sent: bool,
) -> dict[str, object]:
    landing_policy = _landing_policy(hold=hold, damping=damping)
    checks = {
        "damping_called": damping.get("status") == "called",
        "landing_policy_supported": landing_policy
        in {"hold_then_damping", "damping_only"},
        "mode_call_sequence_recorded": len(mode_call_sequence) == 2,
        "mode_call_sequence_safe_order": _mode_call_sequence_is_safe(
            mode_call_sequence
        ),
        "no_joint_commands_sent": joint_commands_sent is False,
    }
    failed = [name for name, passed in checks.items() if not passed]
    return {
        "status": "pass" if not failed else "fail",
        "checks": checks,
        "failed_checks": failed,
    }


def _doctor_prerequisite_status(artifact: dict[str, object]) -> str:
    interface_status = artifact.get("interface_status")
    controller = artifact.get("controller")
    state_read = artifact.get("state_read")
    timestamp_policy = artifact.get("timestamp_policy")
    doctor_gate = artifact.get("doctor_gate")
    if artifact.get("schema") != "armctrl.sysid_sdk_doctor.v1":
        return "fail"
    if isinstance(doctor_gate, dict) and doctor_gate.get("status") != "pass":
        return "fail"
    if (
        not isinstance(interface_status, dict)
        or interface_status.get("status") != "opened_read_only"
    ):
        return "fail"
    if not isinstance(controller, dict) or controller.get("status") != "initialized_read_only":
        return "fail"
    if controller.get("controller_dt_s") is None:
        return "fail"
    if not isinstance(state_read, dict) or state_read.get("status") != "ok":
        return "fail"
    if state_read.get("state_read_hz") is None:
        return "fail"
    if not isinstance(timestamp_policy, dict) or timestamp_policy.get("monotonic") is not True:
        return "fail"
    if artifact.get("motion_commands_sent") is not False:
        return "fail"
    return "pass"


def _hold_damping_prerequisite_status(artifact: dict[str, object]) -> str:
    hold = artifact.get("hold")
    damping = artifact.get("damping")
    mode_call_sequence = artifact.get("mode_call_sequence")
    hold_damping_gate = artifact.get("hold_damping_gate")
    landing_policy = artifact.get("landing_policy")
    if landing_policy is None and isinstance(hold, dict) and isinstance(damping, dict):
        landing_policy = _landing_policy(hold=hold, damping=damping)
    if artifact.get("schema") != "armctrl.sysid_sdk_hold_damping_check.v1":
        return "fail"
    if (
        isinstance(hold_damping_gate, dict)
        and hold_damping_gate.get("status") != "pass"
    ):
        return "fail"
    if not isinstance(damping, dict) or damping.get("status") != "called":
        return "fail"
    if landing_policy not in {"hold_then_damping", "damping_only"}:
        return "fail"
    if not isinstance(hold, dict):
        return "fail"
    if landing_policy == "hold_then_damping" and hold.get("status") != "called":
        return "fail"
    if landing_policy == "damping_only" and hold.get("status") != "unsupported":
        return "fail"
    if not _mode_call_sequence_is_safe(mode_call_sequence):
        return "fail"
    if artifact.get("joint_commands_sent") is not False:
        return "fail"
    return "pass"


def _tiny_motion_execute_prerequisite_status(artifact: dict[str, object]) -> str:
    motion_runtime = artifact.get("motion_runtime")
    acceptance = artifact.get("acceptance")
    if artifact.get("schema") != "armctrl.sysid_sdk_tiny_motion_execute.v1":
        return "fail"
    if isinstance(acceptance, dict) and acceptance.get("status") != "pass":
        return "fail"
    if artifact.get("backend") != "arx5_sdk":
        return "fail"
    if artifact.get("hardware_motion") is not True:
        return "fail"
    if artifact.get("movement_command_sent") is not True:
        return "fail"
    if artifact.get("fault_landing_mode") != "damping":
        return "fail"
    if not isinstance(motion_runtime, dict):
        return "fail"
    if motion_runtime.get("status") != "completed":
        return "fail"
    if motion_runtime.get("landing_mode") not in {"hold", "damping"}:
        return "fail"
    if motion_runtime.get("sample_count", 0) < 2:
        return "fail"
    if motion_runtime.get("actual_send_hz") is None:
        return "fail"
    if motion_runtime.get("controller_dt_s") is None:
        return "fail"
    if motion_runtime.get("fault_flags") not in ([], ()):
        return "fail"
    tracking = _tiny_motion_tracking_summary(motion_runtime)
    if tracking["q_cmd_delta_max_abs_rad"] is None:
        return "fail"
    if tracking["q_meas_delta_max_abs_rad"] is None:
        return "fail"
    if tracking["q_cmd_delta_max_abs_rad"] <= 0.0:
        return "fail"
    if tracking["q_meas_delta_max_abs_rad"] <= 0.0:
        return "fail"
    return "pass"


def _tiny_motion_execute_summary(artifact: dict[str, object]) -> dict[str, object]:
    motion_runtime = artifact.get("motion_runtime")
    if not isinstance(motion_runtime, dict):
        motion_runtime = {}
    tracking = _tiny_motion_tracking_summary(motion_runtime)
    return {
        "backend": artifact.get("backend"),
        "hardware_motion": artifact.get("hardware_motion"),
        "movement_command_sent": artifact.get("movement_command_sent"),
        "acceptance": artifact.get("acceptance"),
        "fault_landing_mode": artifact.get("fault_landing_mode"),
        "trajectory_sample_hz": motion_runtime.get("trajectory_sample_hz"),
        "actual_send_hz": motion_runtime.get("actual_send_hz"),
        "controller_dt_s": motion_runtime.get("controller_dt_s"),
        "sample_count": motion_runtime.get("sample_count"),
        "fault_flags": motion_runtime.get("fault_flags"),
        "landing_mode": motion_runtime.get("landing_mode"),
        "tracking": tracking,
    }


def _startup_recovery_prerequisite_status(artifact: dict[str, object]) -> str:
    motion_runtime = artifact.get("motion_runtime")
    if artifact.get("schema") != "armctrl.sdk_startup_recovery.v1":
        return "fail"
    if artifact.get("hardware_motion") is not True:
        return "fail"
    if artifact.get("movement_command_sent") is not True:
        return "fail"
    if artifact.get("run_status") != "completed":
        return "fail"
    if not isinstance(motion_runtime, dict):
        return "fail"
    if motion_runtime.get("status") != "completed":
        return "fail"
    if motion_runtime.get("fault_flags") not in ([], ()):
        return "fail"
    if motion_runtime.get("actual_send_hz") is None:
        return "fail"
    if motion_runtime.get("controller_dt_s") is None:
        return "fail"
    return "pass"


def _startup_recovery_summary(artifact: dict[str, object]) -> dict[str, object]:
    motion_runtime = artifact.get("motion_runtime")
    if not isinstance(motion_runtime, dict):
        motion_runtime = {}
    return {
        "schema": artifact.get("schema"),
        "run_status": artifact.get("run_status"),
        "hardware_motion": artifact.get("hardware_motion"),
        "movement_command_sent": artifact.get("movement_command_sent"),
        "q_start": artifact.get("q_start"),
        "q_target": artifact.get("q_target"),
        "recovery_plan": artifact.get("recovery_plan"),
        "fault_landing_mode": artifact.get("fault_landing_mode"),
        "trajectory_sample_hz": motion_runtime.get("trajectory_sample_hz"),
        "actual_send_hz": motion_runtime.get("actual_send_hz"),
        "controller_dt_s": motion_runtime.get("controller_dt_s"),
        "sample_count": motion_runtime.get("sample_count"),
        "fault_flags": motion_runtime.get("fault_flags"),
        "landing_mode": motion_runtime.get("landing_mode"),
    }


def _tiny_motion_tracking_summary(
    motion_runtime: dict[str, object],
) -> dict[str, object]:
    samples = motion_runtime.get("samples")
    if not isinstance(samples, list) or len(samples) < 2:
        return {
            "q_cmd_delta_rad": None,
            "q_meas_delta_rad": None,
            "q_cmd_delta_max_abs_rad": None,
            "q_meas_delta_max_abs_rad": None,
            "final_tracking_error_rad": None,
            "final_tracking_error_max_abs_rad": None,
        }
    first = samples[0]
    last = samples[-1]
    if not isinstance(first, dict) or not isinstance(last, dict):
        return {
            "q_cmd_delta_rad": None,
            "q_meas_delta_rad": None,
            "q_cmd_delta_max_abs_rad": None,
            "q_meas_delta_max_abs_rad": None,
            "final_tracking_error_rad": None,
            "final_tracking_error_max_abs_rad": None,
        }
    q_cmd_delta = _vector_delta(first.get("q_cmd"), last.get("q_cmd"))
    q_meas_delta = _vector_delta(first.get("q_meas"), last.get("q_meas"))
    final_tracking_error = _vector_delta(last.get("q_cmd"), last.get("q_meas"))
    return {
        "q_cmd_delta_rad": q_cmd_delta,
        "q_meas_delta_rad": q_meas_delta,
        "q_cmd_delta_max_abs_rad": _max_abs_or_none(q_cmd_delta),
        "q_meas_delta_max_abs_rad": _max_abs_or_none(q_meas_delta),
        "final_tracking_error_rad": final_tracking_error,
        "final_tracking_error_max_abs_rad": _max_abs_or_none(final_tracking_error),
    }


def _vector_delta(
    start: object,
    end: object,
) -> list[float] | None:
    if not isinstance(start, list | tuple) or not isinstance(end, list | tuple):
        return None
    if len(start) == 0 or len(start) != len(end):
        return None
    return [float(end_value) - float(start_value) for start_value, end_value in zip(start, end)]


def _max_abs_or_none(values: list[float] | None) -> float | None:
    if values is None:
        return None
    return max(abs(value) for value in values)


def tiny_motion_execute_prerequisite_statuses(
    *,
    doctor_artifact: dict[str, object] | None,
    hold_damping_artifact: dict[str, object] | None,
) -> dict[str, str]:
    return {
        "doctor": _doctor_prerequisite_status(doctor_artifact or {}),
        "hold_damping": _hold_damping_prerequisite_status(
            hold_damping_artifact or {}
        ),
    }


def _session_landing_policy_from_artifact(artifact: dict[str, object]) -> str:
    landing_policy = artifact.get("landing_policy")
    if isinstance(landing_policy, str):
        return landing_policy
    hold = artifact.get("hold")
    damping = artifact.get("damping")
    if isinstance(hold, dict) and isinstance(damping, dict):
        return _landing_policy(hold=hold, damping=damping)
    return "unknown"


def _session_motion_inputs(
    session_artifact: dict[str, object],
) -> tuple[str, str, tuple[float, ...], float | None]:
    if session_artifact.get("schema") != "armctrl.sdk_arm_session.v1":
        raise ValueError("session artifact must use schema armctrl.sdk_arm_session.v1")
    session_status = session_artifact.get("session_status") or session_artifact.get("status")
    if session_status != "armed":
        raise ValueError("session artifact is not armed")
    model = session_artifact.get("model")
    interface = session_artifact.get("interface")
    if not isinstance(model, str) or not model:
        raise ValueError("session artifact is missing model")
    if not isinstance(interface, str) or not interface:
        raise ValueError("session artifact is missing interface")
    state = session_artifact.get("state")
    if not isinstance(state, dict):
        raise ValueError("session artifact is missing state")
    q_meas = state.get("q_meas")
    if not isinstance(q_meas, list | tuple) or not q_meas:
        raise ValueError("session artifact is missing state.q_meas")
    controller = session_artifact.get("controller")
    controller_dt_s = None
    if isinstance(controller, dict):
        controller_dt_s = _optional_float(controller.get("controller_dt_s"))
    return (
        model,
        interface,
        tuple(float(value) for value in q_meas),
        controller_dt_s,
    )


def _startup_recovery_trajectory(
    *,
    q_start: tuple[float, ...],
    q_target: tuple[float, ...],
    max_joint_step_rad: float,
    send_hz: float,
) -> list[JointTrajectoryPoint]:
    if len(q_start) != len(q_target):
        raise ValueError("q_start and q_target lengths must match")
    max_delta = max(
        (abs(target - start) for start, target in zip(q_start, q_target)),
        default=0.0,
    )
    step_count = max(1, int(math.ceil(max_delta / max_joint_step_rad)))
    points: list[JointTrajectoryPoint] = []
    for step_index in range(step_count + 1):
        ratio = step_index / step_count
        q_cmd = tuple(
            start + (target - start) * ratio
            for start, target in zip(q_start, q_target)
        )
        points.append(
            JointTrajectoryPoint(
                time_s=step_index / float(send_hz),
                q=q_cmd,
            )
        )
    return points


def _startup_recovery_plan_summary(
    *,
    q_start: tuple[float, ...],
    q_target: tuple[float, ...],
    max_joint_step_rad: float,
    trajectory: list[tuple[float, ...]],
) -> dict[str, object]:
    max_planned_step = 0.0
    for previous, current in zip(trajectory, trajectory[1:]):
        max_planned_step = max(
            max_planned_step,
            max(abs(end - start) for start, end in zip(previous, current)),
        )
    return {
        "q_start_source": "sdk_measured_state",
        "q_target_source": "operator_startup_pose",
        "max_joint_step_rad": float(max_joint_step_rad),
        "max_delta_rad": max(
            (abs(target - start) for start, target in zip(q_start, q_target)),
            default=0.0,
        ),
        "step_count": max(0, len(trajectory) - 1),
        "sample_count": len(trajectory),
        "max_planned_step_rad": max_planned_step,
    }


def _tiny_motion_target_from_plan(
    plan_artifact: dict[str, object],
    *,
    dof: int,
    q_current: tuple[float, ...],
) -> tuple[float, ...]:
    if plan_artifact.get("schema") != "armctrl.sysid_sdk_tiny_motion_plan.v1":
        raise ValueError("invalid tiny motion plan schema")
    if plan_artifact.get("tiny_motion_eligible") is not True:
        raise ValueError("tiny motion plan is not eligible")
    command = plan_artifact.get("command")
    if not isinstance(command, dict):
        raise ValueError("tiny motion plan is missing command")
    joint_index = int(command["joint_index"])
    delta_rad = float(command["delta_rad"])
    if not 1 <= joint_index <= dof:
        raise ValueError("joint_index is out of range")
    q_target = list(q_current)
    q_target[joint_index - 1] += delta_rad
    return tuple(q_target)


def _runtime_result_manifest(result: MotionExecutionResult) -> dict[str, object]:
    manifest = {
        "schema": "armctrl.motion_runtime_result.v1",
        "status": result.status,
        "producer": result.producer,
        "mode": result.mode,
        "trajectory_sample_hz": result.trajectory_sample_hz,
        "actual_send_hz": result.actual_send_hz,
        "send_jitter_ms_p95": result.send_jitter_ms_p95,
        "send_jitter_ms_p99": result.send_jitter_ms_p99,
        "controller_dt_s": result.controller_dt_s,
        "sample_count": len(result.samples),
        "fault_flags": sorted(
            {
                fault_flag
                for sample in result.samples
                for fault_flag in sample.fault_flags
            }
        ),
        "samples": [
            {
                "sent_monotonic_s": sample.sent_monotonic_s,
                "q_cmd": list(sample.q_cmd),
                "q_meas": list(sample.q_meas),
                "dq_meas": list(sample.dq_meas),
                "tau_meas": list(sample.tau_meas),
                "fault_flags": list(sample.fault_flags),
                "producer": sample.producer,
                "mode": sample.mode,
            }
            for sample in result.samples
        ],
        "landing_mode": result.landing_mode,
    }
    if result.error is not None:
        manifest["error"] = result.error
    return manifest
