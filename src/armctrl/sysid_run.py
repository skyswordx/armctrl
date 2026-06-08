from __future__ import annotations

from dataclasses import dataclass
import csv
import importlib
import json
import math
import time
from typing import Protocol

from armctrl.acceptance import build_real_motion_acceptance
from armctrl.motion_runtime import (
    JointStateSnapshot,
    JointTrajectoryPoint,
    MotionExecutionResult,
    MotionMode,
    MotionRuntime,
)
from armctrl.runtime_session import (
    acquire_owner_from_artifact,
    release_owner_from_artifact,
)
from armctrl.sysid import SysIdPlanRequest, SysIdPlanner, trajectory_rows

SDK_CONFIRMATION = "I UNDERSTAND THIS WILL MOVE THE ARM"


@dataclass(frozen=True)
class SysIdRunResult:
    schema: str
    adapter: str
    sample_count: int
    artifacts: dict[str, str]
    run_status: str = "completed"

    def to_json(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "adapter": self.adapter,
            "run_status": self.run_status,
            "sample_count": self.sample_count,
            "artifacts": self.artifacts,
        }


class FakeSysIdRunner:
    def run(self, request: SysIdPlanRequest) -> SysIdRunResult:
        request.output_dir.mkdir(parents=True, exist_ok=True)
        raw_samples_path = request.output_dir / "raw_samples.csv"
        manifest_path = request.output_dir / "manifest.json"
        plan = SysIdPlanner.default().write_plan(request)
        raw_rows = _raw_sample_rows(request)
        runtime_owner_lease = None
        runtime_session_after_release = None
        if request.runtime_session_artifact_path is not None:
            runtime_owner_lease = acquire_owner_from_artifact(
                session_artifact_path=request.runtime_session_artifact_path,
                owner="sysid",
                mode="trajectory_replay",
                expected_q_start=request.q_center,
                max_start_error_rad=0.02,
                heartbeat_timeout_s=max(1.0, 2.0 / float(request.sample_hz)),
                max_heartbeat_age_s=5.0,
            )
            request.runtime_session_artifact_path.write_text(
                json.dumps(runtime_owner_lease, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )

        with raw_samples_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(raw_rows[0]))
            writer.writeheader()
            writer.writerows(raw_rows)

        if request.runtime_session_artifact_path is not None:
            runtime_session_after_release = release_owner_from_artifact(
                session_artifact_path=request.runtime_session_artifact_path,
                owner="sysid",
                max_heartbeat_age_s=5.0,
            )
            request.runtime_session_artifact_path.write_text(
                json.dumps(
                    runtime_session_after_release,
                    ensure_ascii=False,
                    indent=2,
                ),
                encoding="utf-8",
            )

        manifest = {
            "schema": "armctrl.sysid_run_manifest.v1",
            "adapter": "fake",
            "run_status": "completed",
            "profile": plan.profile.to_json(),
            "request": {
                "dof": request.dof,
                "sample_hz": request.sample_hz,
                "duration_s": request.duration_s,
                "amplitude_rad": request.amplitude_rad,
                "q_center": list(request.q_center),
                "urdf_path": request.urdf_path,
                "safe_config_path": request.safe_config_path,
            },
            "sample_count": len(raw_rows),
            "handoff": plan.handoff,
            "artifacts": {
                "planned_trajectory": plan.artifacts["planned_trajectory"],
                "raw_samples": str(raw_samples_path),
                "manifest": str(manifest_path),
            },
            "plan_safety": plan.artifact_safety,
        }
        if runtime_owner_lease is not None:
            manifest["runtime"] = _runtime_owner_manifest(
                runtime_owner_lease=runtime_owner_lease,
                runtime_session_after_release=runtime_session_after_release,
            )
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return SysIdRunResult(
            schema="armctrl.sysid_run.v1",
            adapter="fake",
            sample_count=len(raw_rows),
            artifacts={
                "planned_trajectory": plan.artifacts["planned_trajectory"],
                "raw_samples": str(raw_samples_path),
                "manifest": str(manifest_path),
            },
        )


class SdkCollectionBackend(Protocol):
    def enter_hold_or_damping(self) -> None:
        ...

    def read_samples(self, request: SysIdPlanRequest) -> list[dict[str, str]]:
        ...

    def enter_damping(self) -> None:
        ...


class SdkSysIdRunner:
    def __init__(self, *, backend: SdkCollectionBackend) -> None:
        self._backend = backend

    def run(self, request: SysIdPlanRequest, *, confirm: str) -> SysIdRunResult:
        if confirm != SDK_CONFIRMATION:
            raise PermissionError("sdk sysid runner requires explicit operator confirmation")
        request.output_dir.mkdir(parents=True, exist_ok=True)
        raw_samples_path = request.output_dir / "raw_samples.csv"
        manifest_path = request.output_dir / "manifest.json"
        plan = SysIdPlanner.default().write_plan(request)
        if not (plan.artifact_safety or {}).get("allowed", False):
            raise RuntimeError("planned trajectory did not pass safety checks")

        self._backend.enter_hold_or_damping()
        try:
            raw_rows = self._backend.read_samples(request)
        finally:
            self._backend.enter_damping()

        with raw_samples_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(raw_rows[0]))
            writer.writeheader()
            writer.writerows(raw_rows)

        manifest = {
            "schema": "armctrl.sysid_run_manifest.v1",
            "adapter": "sdk",
            "run_status": _sdk_run_status_from_backend(self._backend),
            "profile": plan.profile.to_json(),
            "request": {
                "dof": request.dof,
                "sample_hz": request.sample_hz,
                "duration_s": request.duration_s,
                "amplitude_rad": request.amplitude_rad,
                "q_center": list(request.q_center),
                "urdf_path": request.urdf_path,
                "safe_config_path": request.safe_config_path,
            },
            "sample_count": len(raw_rows),
            "handoff": plan.handoff,
            "artifacts": {
                "planned_trajectory": plan.artifacts["planned_trajectory"],
                "raw_samples": str(raw_samples_path),
                "manifest": str(manifest_path),
            },
            "plan_safety": plan.artifact_safety,
            "safety": {
                "requires_confirm": SDK_CONFIRMATION,
                "movement_allowed": True,
                "recording_starts_after_safe_state": True,
                "fault_landing_mode": "damping",
            },
        }
        motion_result = getattr(self._backend, "last_motion_result", None)
        if motion_result is not None:
            manifest["motion_runtime"] = _motion_runtime_manifest(motion_result)
            manifest["acceptance"] = build_real_motion_acceptance(
                stage="sysid_smoke",
                motion_runtime=manifest["motion_runtime"],
                hardware_motion=True,
                movement_command_sent=True,
            )
        ramp_result = getattr(self._backend, "last_ramp_result", None)
        if ramp_result is not None:
            manifest["ramp_runtime"] = _motion_runtime_manifest(ramp_result)
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return SysIdRunResult(
            schema="armctrl.sysid_run.v1",
            adapter="sdk",
            sample_count=len(raw_rows),
            artifacts={
                "planned_trajectory": plan.artifacts["planned_trajectory"],
                "raw_samples": str(raw_samples_path),
                "manifest": str(manifest_path),
            },
            run_status=str(manifest["run_status"]),
        )


class Arx5InterfaceCollectionBackend:
    def __init__(
        self,
        *,
        model: str,
        interface: str,
        arx5_module: object | None = None,
        max_joint_step_rad: float = 0.01,
        controller_dt_s: float | None = None,
        monotonic=time.monotonic,
        sleep=time.sleep,
        start_delay_s: float = 0.20,
        resume_gain_duration_s: float = 0.4,
        shutdown_to_passive: bool | None = None,
    ) -> None:
        self._model = model
        self._interface = interface
        self._arx5 = arx5_module
        self._max_joint_step_rad = max_joint_step_rad
        if controller_dt_s is not None and (
            not math.isfinite(float(controller_dt_s)) or float(controller_dt_s) <= 0.0
        ):
            raise ValueError("controller_dt_s must be positive when provided")
        self._requested_controller_dt_s = (
            float(controller_dt_s) if controller_dt_s is not None else None
        )
        self._monotonic = monotonic
        self._sleep = sleep
        self._start_delay_s = float(start_delay_s)
        self._resume_gain_duration_s = float(resume_gain_duration_s)
        self._shutdown_to_passive = shutdown_to_passive
        self._controller = None
        self._controller_dt_s: float | None = None
        self._stream_base_timestamp_s = 0.0
        self._stream_started = False
        self.last_ramp_result: MotionExecutionResult | None = None
        self.last_motion_result: MotionExecutionResult | None = None

    def enter_hold_or_damping(self) -> None:
        arx5 = self._load_arx5()
        robot_config = arx5.RobotConfigFactory.get_instance().get_config(self._model)
        controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "joint_controller",
            robot_config.joint_dof,
        )
        controller_dt_s = self._requested_controller_dt_s
        if controller_dt_s is None:
            controller_dt_s = _positive_float_or_none(
                getattr(controller_config, "controller_dt", None)
            )
        if controller_dt_s is None:
            controller_dt_s = 0.01
        controller_config.controller_dt = controller_dt_s
        controller_config.gravity_compensation = True
        controller_config.background_send_recv = True
        if self._shutdown_to_passive is not None and hasattr(
            controller_config,
            "shutdown_to_passive",
        ):
            controller_config.shutdown_to_passive = bool(self._shutdown_to_passive)
        self._controller_dt_s = float(controller_dt_s)
        self._controller = arx5.Arx5JointController(
            robot_config,
            controller_config,
            self._interface,
        )

    def read_samples(self, request: SysIdPlanRequest) -> list[dict[str, str]]:
        if self._controller is None:
            raise RuntimeError("sdk controller is not initialized")
        rows: list[dict[str, str]] = []
        planned_rows = trajectory_rows(request)
        self.last_ramp_result = None
        if planned_rows:
            self._ramp_to_first_target(request, planned_rows[0])
        runtime = MotionRuntime(
            backend=self,
            monotonic=self._monotonic,
            sleep=self._sleep,
        )
        self.last_motion_result = runtime.execute_trajectory(
            [
                JointTrajectoryPoint(
                    time_s=float(planned["time_s"]),
                    q=_q_cmd_from_plan(request, planned),
                )
                for planned in planned_rows
            ],
            producer="sysid",
            trajectory_sample_hz=float(request.sample_hz),
        )
        rows.extend(
            self._rows_from_motion_result(
                request,
                planned_rows=planned_rows,
                result=self.last_motion_result,
            )
        )
        return rows

    def enter_damping(self) -> None:
        if self._controller is not None:
            self._controller.set_to_damping()

    @property
    def controller_dt_s(self) -> float | None:
        return self._controller_dt_s

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
        if self._controller is None:
            raise RuntimeError("sdk controller is not initialized")
        if not self._stream_started:
            self.begin_joint_trajectory([JointTrajectoryPoint(time_s=0.0, q=q)])
        point_time_s = float(trajectory_time_s) if trajectory_time_s is not None else 0.0
        cmd = self._joint_state_from_positions_tuple(
            q,
            timestamp_s=self._stream_base_timestamp_s + max(0.0, point_time_s),
            dq_cmd=dq,
        )
        self._controller.set_joint_cmd(cmd)

    def begin_joint_trajectory(self, points: list[JointTrajectoryPoint]) -> None:
        if self._controller is None:
            raise RuntimeError("sdk controller is not initialized")
        if not points:
            return
        self._ensure_motion_gain()
        self._sync_joint_target_to_current_state()
        get_timestamp = getattr(self._controller, "get_timestamp", None)
        sdk_now_s = float(get_timestamp()) if callable(get_timestamp) else 0.0
        self._stream_base_timestamp_s = sdk_now_s + self._start_delay_s
        self._stream_started = True

    def hold(self) -> str:
        if self._controller is None:
            return MotionMode.DAMPING.value
        set_to_hold = getattr(self._controller, "set_to_hold", None)
        if callable(set_to_hold):
            set_to_hold()
            return MotionMode.HOLD.value
        if self._stream_started:
            return MotionMode.HOLD.value
        self.enter_damping()
        return MotionMode.DAMPING.value

    def hold_joint_position_for_duration(
        self,
        q: tuple[float, ...],
        *,
        duration_s: float | None,
        hold_hz: float,
    ) -> dict[str, object]:
        if self._controller is None:
            raise RuntimeError("sdk controller is not initialized")
        if duration_s is not None and duration_s <= 0.0:
            return {
                "requested": False,
                "duration_s": float(duration_s),
                "hold_hz": float(hold_hz),
                "command_count": 0,
                "landing_mode": self.hold(),
            }
        if hold_hz <= 0.0:
            raise ValueError("hold_hz must be positive")
        period_s = 1.0 / float(hold_hz)
        controller_dt_s = self._controller_dt_s or 0.002
        lookahead_s = max(0.04, controller_dt_s * 5.0)
        command_limit = (
            max(1, int(math.ceil(float(duration_s) * float(hold_hz))))
            if duration_s is not None
            else None
        )
        get_timestamp = getattr(self._controller, "get_timestamp", None)
        command_count = 0
        while command_limit is None or command_count < command_limit:
            sdk_now_s = float(get_timestamp()) if callable(get_timestamp) else 0.0
            cmd = self._joint_state_from_positions_tuple(
                tuple(float(value) for value in q),
                timestamp_s=sdk_now_s + lookahead_s,
            )
            self._controller.set_joint_cmd(cmd)
            command_count += 1
            if command_limit is None or command_count < command_limit:
                self._sleep(period_s)
        return {
            "requested": True,
            "duration_s": None if duration_s is None else float(duration_s),
            "hold_hz": float(hold_hz),
            "command_count": command_count,
            "until_interrupt": duration_s is None,
            "landing_mode": MotionMode.HOLD.value,
        }

    def damping(self) -> None:
        self.enter_damping()

    def read_joint_state(self) -> JointStateSnapshot:
        if self._controller is None:
            raise RuntimeError("sdk controller is not initialized")
        joint_state = self._controller.get_joint_state()
        pos = joint_state.pos()
        vel = joint_state.vel()
        torque = joint_state.torque()
        return JointStateSnapshot(
            q_meas=tuple(float(value) for value in pos),
            dq_meas=tuple(float(value) for value in vel),
            tau_meas=tuple(float(value) for value in torque),
            fault_flags=_fault_flags_from_sources(joint_state, self._controller),
        )

    def _ramp_to_first_target(
        self,
        request: SysIdPlanRequest,
        first_planned: dict[str, str],
    ) -> None:
        if self._controller is None:
            raise RuntimeError("sdk controller is not initialized")
        current_state = self._controller.get_joint_state()
        current = [
            float(current_state.pos()[joint_index])
            for joint_index in range(request.dof)
        ]
        target = [
            float(first_planned[f"q_cmd_{joint_index + 1}"])
            for joint_index in range(request.dof)
        ]
        max_delta = max(abs(end - start) for start, end in zip(current, target))
        if max_delta == 0.0:
            return
        step_limit = max(self._max_joint_step_rad, 1e-6)
        step_count = int(max_delta / step_limit)
        if max_delta % step_limit:
            step_count += 1
        ramp_points: list[JointTrajectoryPoint] = []
        for step_index in range(1, step_count + 1):
            ratio = step_index / step_count
            q_cmd = tuple(
                start + (end - start) * ratio
                for start, end in zip(current, target)
            )
            ramp_points.append(
                JointTrajectoryPoint(
                    time_s=(step_index - 1) / float(request.sample_hz),
                    q=q_cmd,
                )
            )
        runtime = MotionRuntime(
            backend=self,
            monotonic=self._monotonic,
            sleep=self._sleep,
        )
        self.last_ramp_result = runtime.execute_trajectory(
            ramp_points,
            producer="sysid_ramp",
            trajectory_sample_hz=float(request.sample_hz),
        )

    def _load_arx5(self):
        if self._arx5 is None:
            self._arx5 = importlib.import_module("arx5_interface")
        return self._arx5

    def _joint_state_from_positions_tuple(
        self,
        q_cmd: tuple[float, ...],
        *,
        timestamp_s: float = 0.0,
        dq_cmd: tuple[float, ...] | None = None,
    ):
        arx5 = self._load_arx5()
        cmd = arx5.JointState(len(q_cmd))
        for joint_index, joint_position in enumerate(q_cmd):
            cmd.pos()[joint_index] = joint_position
        if hasattr(cmd, "vel"):
            cmd.vel()[:] = (
                tuple(float(value) for value in dq_cmd)
                if dq_cmd is not None
                else tuple(0.0 for _ in q_cmd)
            )
        if hasattr(cmd, "torque"):
            cmd.torque()[:] = tuple(0.0 for _ in q_cmd)
        cmd.timestamp = float(timestamp_s)
        cmd.gripper_pos = 0.0
        return cmd

    def _ensure_motion_gain(self) -> None:
        if self._controller is None:
            raise RuntimeError("sdk controller is not initialized")
        get_gain = getattr(self._controller, "get_gain", None)
        set_gain = getattr(self._controller, "set_gain", None)
        get_controller_config = getattr(self._controller, "get_controller_config", None)
        if not callable(get_gain) or not callable(set_gain) or not callable(get_controller_config):
            return
        target_gain = self._build_motion_gain(get_controller_config())
        if target_gain is None:
            return
        current_gain = get_gain()
        if self._gain_matches(current_gain, target_gain):
            return
        controller_dt_s = self._controller_dt_s or 0.002
        steps = max(1, int(round(self._resume_gain_duration_s / controller_dt_s)))
        for step_index in range(1, steps + 1):
            alpha = step_index / steps
            set_gain(current_gain * (1.0 - alpha) + target_gain * alpha)
            if step_index < steps:
                self._sleep(controller_dt_s)

    def _build_motion_gain(self, controller_config):
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

    def _sync_joint_target_to_current_state(self) -> None:
        if self._controller is None:
            raise RuntimeError("sdk controller is not initialized")
        get_timestamp = getattr(self._controller, "get_timestamp", None)
        controller_dt_s = self._controller_dt_s or 0.002
        sdk_now_s = float(get_timestamp()) if callable(get_timestamp) else 0.0
        current_state = self._controller.get_joint_state()
        q_current = tuple(float(value) for value in current_state.pos())
        cmd = self._joint_state_from_positions_tuple(
            q_current,
            timestamp_s=sdk_now_s + controller_dt_s,
        )
        self._controller.set_joint_cmd(cmd)
        self._sleep(controller_dt_s)

    def _rows_from_motion_result(
        self,
        request: SysIdPlanRequest,
        *,
        planned_rows: list[dict[str, str]],
        result: MotionExecutionResult,
    ) -> list[dict[str, str]]:
        rows: list[dict[str, str]] = []
        for planned, sample in zip(planned_rows, result.samples, strict=True):
            row = dict(planned)
            for joint_index in range(request.dof):
                row[f"q_cmd_{joint_index + 1}"] = f"{sample.q_cmd[joint_index]:.6f}"
                row[f"q_{joint_index + 1}"] = f"{sample.q_meas[joint_index]:.6f}"
                dq = (
                    sample.dq_meas[joint_index]
                    if joint_index < len(sample.dq_meas)
                    else 0.0
                )
                tau = (
                    sample.tau_meas[joint_index]
                    if joint_index < len(sample.tau_meas)
                    else 0.0
                )
                row[f"dq_{joint_index + 1}"] = f"{dq:.6f}"
                row[f"tau_meas_{joint_index + 1}"] = f"{tau:.6f}"
            rows.append(row)
        return rows


def _raw_sample_rows(request: SysIdPlanRequest) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for planned in trajectory_rows(request):
        row = dict(planned)
        for joint_index in range(request.dof):
            q_cmd = float(planned[f"q_cmd_{joint_index + 1}"])
            row[f"q_{joint_index + 1}"] = f"{q_cmd:.6f}"
            row[f"dq_{joint_index + 1}"] = "0.000000"
            row[f"tau_meas_{joint_index + 1}"] = "0.000000"
        rows.append(row)
    return rows


def _positive_float_or_none(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed) or parsed <= 0.0:
        return None
    return parsed


def _sdk_attr_value(source: object, name: str):
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


def _numeric_sdk_attr(source: object, name: str) -> float | None:
    return _positive_float_or_none(_sdk_attr_value(source, name))


def _numeric_attrs_match(left: object, right: object, name: str) -> bool:
    left_values = _float_tuple_or_none(_sdk_attr_value(left, name))
    right_values = _float_tuple_or_none(_sdk_attr_value(right, name))
    if left_values is None or right_values is None:
        return False
    if len(left_values) != len(right_values):
        return False
    return all(abs(left_value - right_value) <= 1e-9 for left_value, right_value in zip(left_values, right_values))


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


def _sdk_run_status_from_backend(backend: SdkCollectionBackend) -> str:
    motion_result = getattr(backend, "last_motion_result", None)
    if isinstance(motion_result, MotionExecutionResult):
        return motion_result.status
    return "completed"


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


def _q_cmd_from_plan(
    request: SysIdPlanRequest,
    planned: dict[str, str],
) -> tuple[float, ...]:
    return tuple(
        float(planned[f"q_cmd_{joint_index + 1}"])
        for joint_index in range(request.dof)
    )


def _motion_runtime_manifest(result: MotionExecutionResult) -> dict[str, object]:
    fault_flags = sorted(
        {
            fault_flag
            for sample in result.samples
            for fault_flag in sample.fault_flags
        }
    )
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
        "fault_flags": fault_flags,
        "tracking": _motion_tracking_summary(result.samples),
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


def _runtime_owner_manifest(
    *,
    runtime_owner_lease: dict[str, object],
    runtime_session_after_release: dict[str, object] | None,
) -> dict[str, object]:
    return {
        "backend": runtime_owner_lease.get("backend"),
        "owner": runtime_owner_lease.get("owner"),
        "mode": runtime_owner_lease.get("mode"),
        "single_owner_runtime_session": True,
        "runtime_session_id": runtime_owner_lease.get("runtime_session_id"),
        "owner_lease": runtime_owner_lease.get("owner_lease"),
        "release": (
            {
                "mode": runtime_session_after_release.get("mode"),
                "owner": runtime_session_after_release.get("owner"),
                "readiness": runtime_session_after_release.get("readiness"),
            }
            if runtime_session_after_release is not None
            else None
        ),
    }


def _motion_tracking_summary(samples) -> dict[str, object]:
    if not samples:
        return {
            "q_cmd_delta_rad": None,
            "q_meas_delta_rad": None,
            "q_cmd_delta_max_abs_rad": None,
            "q_meas_delta_max_abs_rad": None,
            "max_abs_sample_tracking_error_rad": None,
            "final_tracking_error_rad": None,
            "final_tracking_error_max_abs_rad": None,
        }
    q_cmd_delta = None
    q_meas_delta = None
    if len(samples) >= 2:
        q_cmd_delta = _vector_delta(samples[0].q_cmd, samples[-1].q_cmd)
        q_meas_delta = _vector_delta(samples[0].q_meas, samples[-1].q_meas)
    final_tracking_error = _vector_delta(samples[-1].q_cmd, samples[-1].q_meas)
    sample_errors = [
        error
        for sample in samples
        for error in (_vector_delta(sample.q_cmd, sample.q_meas) or [])
    ]
    return {
        "q_cmd_delta_rad": q_cmd_delta,
        "q_meas_delta_rad": q_meas_delta,
        "q_cmd_delta_max_abs_rad": _max_abs_or_none(q_cmd_delta),
        "q_meas_delta_max_abs_rad": _max_abs_or_none(q_meas_delta),
        "max_abs_sample_tracking_error_rad": _max_abs_or_none(sample_errors),
        "final_tracking_error_rad": final_tracking_error,
        "final_tracking_error_max_abs_rad": _max_abs_or_none(final_tracking_error),
    }


def _vector_delta(start: object, end: object) -> list[float] | None:
    if not isinstance(start, list | tuple) or not isinstance(end, list | tuple):
        return None
    if len(start) == 0 or len(start) != len(end):
        return None
    return [
        float(end_value) - float(start_value)
        for start_value, end_value in zip(start, end)
    ]


def _max_abs_or_none(values: list[float] | None) -> float | None:
    if values is None:
        return None
    if not values:
        return None
    return max(abs(value) for value in values)


class SdkSysIdRunnerGate:
    def evaluate(self, *, adapter: str, confirm: str | None) -> dict[str, object]:
        if confirm != SDK_CONFIRMATION:
            return self.reject_without_confirmation(adapter=adapter)
        return self.reject_without_backend(adapter=adapter)

    def reject_without_confirmation(self, *, adapter: str) -> dict[str, object]:
        return {
            "status": "rejected",
            "schema": "armctrl.sysid_run.v1",
            "adapter": adapter,
            "reason": "sdk sysid runner requires explicit operator confirmation",
            "requires_confirm": SDK_CONFIRMATION,
            "movement_allowed": False,
            "fault_landing_mode": "damping",
            "recording_starts_after_safe_state": True,
            "next_gate": "run sysid sdk-handshake-plan before enabling sdk runner",
        }

    def reject_without_backend(self, *, adapter: str) -> dict[str, object]:
        return {
            "status": "rejected",
            "schema": "armctrl.sysid_run.v1",
            "adapter": adapter,
            "reason": "real sdk sysid runner is not implemented in this clean rebuild",
            "requires_confirm": SDK_CONFIRMATION,
            "confirm_received": True,
            "movement_allowed": False,
            "fault_landing_mode": "damping",
            "recording_starts_after_safe_state": True,
            "next_gate": "implement and verify an arx5_interface SdkCollectionBackend before moving hardware",
        }

    def reject_without_readiness_artifact(self, *, adapter: str) -> dict[str, object]:
        return {
            "status": "rejected",
            "schema": "armctrl.sysid_run.v1",
            "adapter": adapter,
            "reason": "sdk sysid runner requires sdk-agent-sysid-smoke-readiness artifact",
            "requires_confirm": SDK_CONFIRMATION,
            "confirm_received": True,
            "movement_allowed": False,
            "readiness_artifact_path": None,
            "fault_landing_mode": "damping",
            "recording_starts_after_safe_state": True,
            "next_gate": "run sysid sdk-agent-sysid-smoke-readiness after real tiny motion before sdk sysid run",
        }

    def reject_failed_readiness(
        self,
        *,
        adapter: str,
        readiness_artifact_path: str,
        readiness_artifact: dict[str, object],
    ) -> dict[str, object]:
        return {
            "status": "rejected",
            "schema": "armctrl.sysid_run.v1",
            "adapter": adapter,
            "reason": "sdk-agent-sysid-smoke-readiness is not passed",
            "requires_confirm": SDK_CONFIRMATION,
            "confirm_received": True,
            "movement_allowed": False,
            "readiness_artifact_path": readiness_artifact_path,
            "readiness": {
                "agent_sysid_smoke_allowed": readiness_artifact.get(
                    "agent_sysid_smoke_allowed"
                ),
                "prerequisites": readiness_artifact.get("prerequisites"),
            },
            "fault_landing_mode": "damping",
            "recording_starts_after_safe_state": True,
            "next_gate": "complete real tiny motion before sdk sysid run",
        }

    def reject_sdk_unavailable(self, *, adapter: str) -> dict[str, object]:
        return {
            "status": "rejected",
            "schema": "armctrl.sysid_run.v1",
            "adapter": adapter,
            "reason": "arx5_interface is not importable in this environment",
            "requires_confirm": SDK_CONFIRMATION,
            "confirm_received": True,
            "movement_allowed": False,
            "fault_landing_mode": "damping",
            "recording_starts_after_safe_state": True,
            "next_gate": "install arx5_interface on the target Linux host before running sdk collection",
        }

    def reject_unsafe_plan(self, *, adapter: str) -> dict[str, object]:
        return {
            "status": "rejected",
            "schema": "armctrl.sysid_run.v1",
            "adapter": adapter,
            "reason": "planned trajectory did not pass safety checks",
            "requires_confirm": SDK_CONFIRMATION,
            "confirm_received": True,
            "movement_allowed": False,
            "fault_landing_mode": "damping",
            "recording_starts_after_safe_state": True,
            "next_gate": "run sysid plan with smaller amplitude or safer q-center and inspect manifest safety checks",
        }
