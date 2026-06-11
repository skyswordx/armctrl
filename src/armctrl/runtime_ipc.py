"""File-backed command queue for a long-lived arm runtime process."""

from __future__ import annotations

from pathlib import Path
import json
import time
from uuid import uuid4
from typing import Mapping, Sequence

from armctrl.motion_runtime import (
    ArmRuntime,
    ArmRuntimeError,
    JointIntentFrame,
    JointTrajectoryPoint,
    MotionBackend,
    MotionAuditSample,
    MotionExecutionResult,
    MotionMode,
    MotionRuntime,
)
from armctrl.runtime_session import (
    RuntimeSessionError,
    heartbeat_runtime_session_payload,
    runtime_readiness,
)

RUNTIME_COMMAND_SCHEMA = "armctrl.arm_runtime_command.v1"
RUNTIME_COMMAND_RESULT_SCHEMA = "armctrl.arm_runtime_command_result.v1"
START_POSE_POLICIES = {"live_hold", "safe_center", "explicit_q", "current_measured_pose"}
EEF_COMMAND_KINDS = {"eef_pose_delta", "eef_twist", "eef_pose"}
EEF_BACKENDS = {"moveit_servo", "sdk_cartesian"}
DEFAULT_EEF_MAX_LINEAR_STEP_M = 0.005
DEFAULT_EEF_MAX_ANGULAR_STEP_RAD = 0.05
DEFAULT_AGENT_MAX_JOINT_VELOCITY_RAD_S = 0.25
DEFAULT_TRACKING_ERROR_GRACE_SAMPLES = 3
DEFAULT_TRACKING_ERROR_CONSECUTIVE_SAMPLES = 3


def submit_trajectory_command(
    *,
    session_artifact_path: Path,
    owner: str,
    expected_q_start: Sequence[float],
    q_points: Sequence[Sequence[float]],
    dq_points: Sequence[Sequence[float]] | None = None,
    ddq_points: Sequence[Sequence[float]] | None = None,
    artifact_policy: dict[str, object] | None = None,
    send_hz: float,
    trajectory_sample_hz: float | None = None,
    max_start_error_rad: float,
    heartbeat_timeout_s: float,
    max_heartbeat_age_s: float,
    output_path: Path | None = None,
    start_pose_policy: str = "live_hold",
    max_tracking_error_rad: float | None = None,
    tracking_error_grace_samples: int | None = DEFAULT_TRACKING_ERROR_GRACE_SAMPLES,
    tracking_error_consecutive_samples: int | None = (
        DEFAULT_TRACKING_ERROR_CONSECUTIVE_SAMPLES
    ),
    max_tau_abs: float | None = None,
    max_joint_segment_delta_rad: float | None = None,
    max_joint_velocity_rad_s: float | None = None,
    max_joint_acceleration_rad_s2: float | None = None,
) -> dict[str, object]:
    if not q_points:
        raise ValueError("at least one --q-point is required")
    q_start = _float_list(expected_q_start, name="expected_q_start")
    points = [_float_list(point, name="q_point") for point in q_points]
    for point in points:
        if len(point) != len(q_start):
            raise ValueError("all q points must match expected_q_start length")
    velocities = (
        None
        if dq_points is None
        else [_float_list(point, name="dq_point") for point in dq_points]
    )
    if velocities is not None:
        if len(velocities) != len(points):
            raise ValueError("dq_points length must match q_points length")
        for point in velocities:
            if len(point) != len(q_start):
                raise ValueError("all dq points must match expected_q_start length")
    accelerations = (
        None
        if ddq_points is None
        else [_float_list(point, name="ddq_point") for point in ddq_points]
    )
    if accelerations is not None:
        if len(accelerations) != len(points):
            raise ValueError("ddq_points length must match q_points length")
        for point in accelerations:
            if len(point) != len(q_start):
                raise ValueError("all ddq points must match expected_q_start length")
    if send_hz <= 0.0:
        raise ValueError("send_hz must be positive")
    if trajectory_sample_hz is None:
        trajectory_sample_hz = float(send_hz)
    if trajectory_sample_hz <= 0.0:
        raise ValueError("trajectory_sample_hz must be positive")
    joint_trajectory_safety = _joint_trajectory_safety_guard(
        owner=owner,
        q_points=points,
        trajectory_sample_hz=float(trajectory_sample_hz),
        max_joint_segment_delta_rad=max_joint_segment_delta_rad,
        max_joint_velocity_rad_s=max_joint_velocity_rad_s,
        max_joint_acceleration_rad_s2=max_joint_acceleration_rad_s2,
    )
    if joint_trajectory_safety["status"] != "pass":
        raise ValueError(
            "joint trajectory safety failed: "
            + ", ".join(
                str(item) for item in joint_trajectory_safety["failed_checks"]
            )
        )

    session = _read_json_object(session_artifact_path)
    start_pose_guard = _ensure_can_queue_command(
        session,
        owner=owner,
        expected_q_start=q_start,
        start_pose_policy=start_pose_policy,
        max_start_error_rad=float(max_start_error_rad),
        heartbeat_timeout_s=float(heartbeat_timeout_s),
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    command_id = str(uuid4())
    queue_dir = runtime_command_queue_dir(session_artifact_path)
    pending_path = queue_dir / "pending" / f"{command_id}.json"
    result_path = queue_dir / "results" / f"{command_id}.json"
    command = {
        "schema": RUNTIME_COMMAND_SCHEMA,
        "command_id": command_id,
        "kind": "joint_trajectory",
        "source": str(owner),
        "owner": str(owner),
        "mode": MotionMode.TRAJECTORY_REPLAY.value,
        "start_pose_policy": str(start_pose_policy),
        "expected_q_start": q_start,
        "start_pose_guard": start_pose_guard,
        "max_start_error_rad": float(max_start_error_rad),
        "heartbeat_timeout_s": float(heartbeat_timeout_s),
        "max_heartbeat_age_s": float(max_heartbeat_age_s),
        "send_hz": float(send_hz),
        "trajectory_sample_hz": float(trajectory_sample_hz),
        "q_points": points,
        "joint_trajectory_safety": joint_trajectory_safety,
        "max_joint_segment_delta_rad": (
            None
            if max_joint_segment_delta_rad is None
            else float(max_joint_segment_delta_rad)
        ),
        "max_joint_velocity_rad_s": (
            None
            if max_joint_velocity_rad_s is None
            else float(max_joint_velocity_rad_s)
        ),
        "max_joint_acceleration_rad_s2": (
            None
            if max_joint_acceleration_rad_s2 is None
            else float(max_joint_acceleration_rad_s2)
        ),
        "submitted_wall_time_s": time.time(),
        "session_artifact": str(session_artifact_path),
        "result_artifact": str(result_path),
    }
    if velocities is not None:
        command["dq_points"] = velocities
    if accelerations is not None:
        command["ddq_points"] = accelerations
    if artifact_policy is not None:
        command["artifact_policy"] = dict(artifact_policy)
    if max_tracking_error_rad is not None:
        command["max_tracking_error_rad"] = float(max_tracking_error_rad)
        command["tracking_error_grace_samples"] = int(
            DEFAULT_TRACKING_ERROR_GRACE_SAMPLES
            if tracking_error_grace_samples is None
            else tracking_error_grace_samples
        )
        command["tracking_error_consecutive_samples"] = int(
            DEFAULT_TRACKING_ERROR_CONSECUTIVE_SAMPLES
            if tracking_error_consecutive_samples is None
            else tracking_error_consecutive_samples
        )
    if max_tau_abs is not None:
        command["max_tau_abs"] = float(max_tau_abs)
    pending_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    _write_json_atomic(pending_path, command)
    payload = {
        "status": "queued",
        "schema": "armctrl.arm_runtime_submit.v1",
        "command_id": command_id,
        "owner": str(owner),
        "mode": MotionMode.TRAJECTORY_REPLAY.value,
        "runtime_session_id": session.get("runtime_session_id"),
        "movement_command_sent": False,
        "start_pose_policy": str(start_pose_policy),
        "start_pose_guard": start_pose_guard,
        "joint_trajectory_safety": joint_trajectory_safety,
        "runtime": {
            "single_motion_owner": True,
            "queue": str(queue_dir),
            "session_artifact": str(session_artifact_path),
        },
        "artifacts": {
            "command": str(pending_path),
            "result": str(result_path),
        },
    }
    if artifact_policy is not None:
        payload["artifact_policy"] = dict(artifact_policy)
    return _write_optional_output(payload, output_path)


def submit_intent_command(
    *,
    session_artifact_path: Path,
    owner: str,
    expected_q_start: Sequence[float],
    q_target: Sequence[float],
    control_period_s: float,
    send_hz: float,
    max_joint_delta_rad: float | None,
    max_start_error_rad: float,
    heartbeat_timeout_s: float,
    max_heartbeat_age_s: float,
    output_path: Path | None = None,
    start_pose_policy: str = "live_hold",
    max_tracking_error_rad: float | None = None,
    tracking_error_grace_samples: int | None = DEFAULT_TRACKING_ERROR_GRACE_SAMPLES,
    tracking_error_consecutive_samples: int | None = (
        DEFAULT_TRACKING_ERROR_CONSECUTIVE_SAMPLES
    ),
    max_tau_abs: float | None = None,
    max_joint_velocity_rad_s: float | None = DEFAULT_AGENT_MAX_JOINT_VELOCITY_RAD_S,
) -> dict[str, object]:
    q_start = _float_list(expected_q_start, name="expected_q_start")
    target = _float_list(q_target, name="q_target")
    if len(target) != len(q_start):
        raise ValueError("q_target must match expected_q_start length")
    if control_period_s <= 0.0:
        raise ValueError("control_period_s must be positive")
    if send_hz <= 0.0:
        raise ValueError("send_hz must be positive")
    joint_intent_safety = _joint_intent_safety_guard(
        q_start=q_start,
        q_target=target,
        control_period_s=float(control_period_s),
        max_joint_delta_rad=max_joint_delta_rad,
        max_joint_velocity_rad_s=max_joint_velocity_rad_s,
    )
    if joint_intent_safety["status"] != "pass":
        raise ValueError(
            "joint intent safety failed: "
            + ", ".join(str(item) for item in joint_intent_safety["failed_checks"])
        )
    expected_runtime_sample_count = (
        max(1, int(round(float(control_period_s) * float(send_hz)))) + 1
    )
    intent_trajectory_contract = _joint_intent_trajectory_contract(
        q_start=q_start,
        q_target=target,
        control_period_s=float(control_period_s),
        send_hz=float(send_hz),
        max_joint_delta_rad=max_joint_delta_rad,
        max_joint_velocity_rad_s=max_joint_velocity_rad_s,
    )
    session = _read_json_object(session_artifact_path)
    start_pose_guard = _ensure_can_queue_command(
        session,
        owner=owner,
        expected_q_start=q_start,
        start_pose_policy=start_pose_policy,
        max_start_error_rad=float(max_start_error_rad),
        heartbeat_timeout_s=float(heartbeat_timeout_s),
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    command_id = str(uuid4())
    queue_dir = runtime_command_queue_dir(session_artifact_path)
    pending_path = queue_dir / "pending" / f"{command_id}.json"
    result_path = queue_dir / "results" / f"{command_id}.json"
    command = {
        "schema": RUNTIME_COMMAND_SCHEMA,
        "command_id": command_id,
        "kind": "joint_intent",
        "source": str(owner),
        "owner": str(owner),
        "mode": MotionMode.AGENT_SERVO.value,
        "start_pose_policy": str(start_pose_policy),
        "expected_q_start": q_start,
        "start_pose_guard": start_pose_guard,
        "q_target": target,
        "control_period_s": float(control_period_s),
        "max_joint_delta_rad": (
            None if max_joint_delta_rad is None else float(max_joint_delta_rad)
        ),
        "max_joint_velocity_rad_s": (
            None
            if max_joint_velocity_rad_s is None
            else float(max_joint_velocity_rad_s)
        ),
        "joint_intent_safety": joint_intent_safety,
        "intent_trajectory_contract": intent_trajectory_contract,
        "resampling_policy": "intent_frame_to_runtime_send_hz",
        "interpolation_policy": "smoothstep_intent_frame",
        "expected_runtime_sample_count": expected_runtime_sample_count,
        "max_start_error_rad": float(max_start_error_rad),
        "heartbeat_timeout_s": float(heartbeat_timeout_s),
        "max_heartbeat_age_s": float(max_heartbeat_age_s),
        "send_hz": float(send_hz),
        "submitted_wall_time_s": time.time(),
        "session_artifact": str(session_artifact_path),
        "result_artifact": str(result_path),
    }
    if max_tracking_error_rad is not None:
        command["max_tracking_error_rad"] = float(max_tracking_error_rad)
        command["tracking_error_grace_samples"] = int(
            DEFAULT_TRACKING_ERROR_GRACE_SAMPLES
            if tracking_error_grace_samples is None
            else tracking_error_grace_samples
        )
        command["tracking_error_consecutive_samples"] = int(
            DEFAULT_TRACKING_ERROR_CONSECUTIVE_SAMPLES
            if tracking_error_consecutive_samples is None
            else tracking_error_consecutive_samples
        )
    if max_tau_abs is not None:
        command["max_tau_abs"] = float(max_tau_abs)
    pending_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    _write_json_atomic(pending_path, command)
    payload = {
        "status": "queued",
        "schema": "armctrl.arm_runtime_submit.v1",
        "command_id": command_id,
        "owner": str(owner),
        "mode": MotionMode.AGENT_SERVO.value,
        "runtime_session_id": session.get("runtime_session_id"),
        "movement_command_sent": False,
        "start_pose_policy": str(start_pose_policy),
        "start_pose_guard": start_pose_guard,
        "joint_intent_safety": joint_intent_safety,
        "intent_trajectory_contract": intent_trajectory_contract,
        "resampling_policy": "intent_frame_to_runtime_send_hz",
        "interpolation_policy": "smoothstep_intent_frame",
        "expected_runtime_sample_count": expected_runtime_sample_count,
        "runtime": {
            "single_motion_owner": True,
            "queue": str(queue_dir),
            "session_artifact": str(session_artifact_path),
        },
        "artifacts": {
            "command": str(pending_path),
            "result": str(result_path),
        },
    }
    return _write_optional_output(payload, output_path)


def submit_eef_command(
    *,
    session_artifact_path: Path,
    owner: str,
    backend: str,
    kind: str,
    frame: str,
    expected_q_start: Sequence[float],
    control_period_s: float,
    send_hz: float,
    max_start_error_rad: float,
    heartbeat_timeout_s: float,
    max_heartbeat_age_s: float,
    delta_position_m: Sequence[float] | None = None,
    delta_rpy_rad: Sequence[float] | None = None,
    linear_mps: Sequence[float] | None = None,
    angular_rps: Sequence[float] | None = None,
    position_m: Sequence[float] | None = None,
    rpy_rad: Sequence[float] | None = None,
    max_linear_step_m: float = DEFAULT_EEF_MAX_LINEAR_STEP_M,
    max_angular_step_rad: float = DEFAULT_EEF_MAX_ANGULAR_STEP_RAD,
    output_path: Path | None = None,
    start_pose_policy: str = "live_hold",
) -> dict[str, object]:
    if kind not in EEF_COMMAND_KINDS:
        raise ValueError(f"unsupported EEF runtime command kind: {kind}")
    if backend not in EEF_BACKENDS:
        raise ValueError(
            "EEF runtime supports backend=moveit_servo or backend=sdk_cartesian"
        )
    q_start = _float_list(expected_q_start, name="expected_q_start")
    if control_period_s <= 0.0:
        raise ValueError("control_period_s must be positive")
    if send_hz <= 0.0:
        raise ValueError("send_hz must be positive")
    if not frame:
        raise ValueError("frame must not be empty")
    if kind == "eef_pose_delta":
        if delta_position_m is None or delta_rpy_rad is None:
            raise ValueError("eef_pose_delta requires delta_position_m and delta_rpy_rad")
        eef_command = {
            "frame": str(frame),
            "delta_position_m": _fixed_float_list(
                delta_position_m,
                name="delta_position_m",
                length=3,
            ),
            "delta_rpy_rad": _fixed_float_list(
                delta_rpy_rad,
                name="delta_rpy_rad",
                length=3,
            ),
            "control_period_s": float(control_period_s),
        }
    elif kind == "eef_twist":
        if linear_mps is None or angular_rps is None:
            raise ValueError("eef_twist requires linear_mps and angular_rps")
        eef_command = {
            "frame": str(frame),
            "linear_mps": _fixed_float_list(linear_mps, name="linear_mps", length=3),
            "angular_rps": _fixed_float_list(angular_rps, name="angular_rps", length=3),
            "control_period_s": float(control_period_s),
        }
    else:
        if position_m is None or rpy_rad is None:
            raise ValueError("eef_pose requires position_m and rpy_rad")
        eef_command = {
            "frame": str(frame),
            "position_m": _fixed_float_list(position_m, name="position_m", length=3),
            "rpy_rad": _fixed_float_list(rpy_rad, name="rpy_rad", length=3),
            "control_period_s": float(control_period_s),
            "pose_reference_limiter": "adapter_live_reference_limit",
        }
    eef_reference_limit = _eef_reference_limit_guard(
        kind=str(kind),
        eef_command=eef_command,
        max_linear_step_m=max_linear_step_m,
        max_angular_step_rad=max_angular_step_rad,
    )
    if eef_reference_limit["status"] != "pass":
        raise ValueError(
            "EEF reference limit failed: "
            + ", ".join(str(item) for item in eef_reference_limit["failed_checks"])
        )

    session = _read_json_object(session_artifact_path)
    start_pose_guard = _ensure_can_queue_command(
        session,
        owner=owner,
        expected_q_start=q_start,
        start_pose_policy=start_pose_policy,
        max_start_error_rad=float(max_start_error_rad),
        heartbeat_timeout_s=float(heartbeat_timeout_s),
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    adapter_gate = _eef_submit_adapter_gate(
        session=session,
        requested_backend=str(backend),
    )
    if adapter_gate["status"] != "pass":
        payload = {
            "status": "blocked",
            "schema": "armctrl.arm_runtime_submit.v1",
            "owner": str(owner),
            "mode": MotionMode.AGENT_SERVO.value,
            "command_space": "eef",
            "backend": str(backend),
            "runtime_session_id": session.get("runtime_session_id"),
            "movement_command_sent": False,
            "hardware_executable_now": False,
            "start_pose_policy": str(start_pose_policy),
            "start_pose_guard": start_pose_guard,
            "eef_reference_limit": eef_reference_limit,
            "eef_adapter_manager": adapter_gate["eef_adapter_manager"],
            "runtime_controller_manager": session.get("runtime_controller_manager"),
            "reason": adapter_gate["reason"],
            "runtime": {
                "single_motion_owner": True,
                "session_artifact": str(session_artifact_path),
            },
            "mature_backend_policy": _eef_mature_backend_policy(str(backend)),
            "next_gate": (
                "restart or configure the long-lived runtime with a mature EEF "
                "adapter registry for this backend; do not use disconnected "
                "sdk_cartesian takeover or heuristic joint fallback"
            ),
        }
        return _write_optional_output(payload, output_path)
    command_id = str(uuid4())
    queue_dir = runtime_command_queue_dir(session_artifact_path)
    pending_path = queue_dir / "pending" / f"{command_id}.json"
    result_path = queue_dir / "results" / f"{command_id}.json"
    command = {
        "schema": RUNTIME_COMMAND_SCHEMA,
        "command_id": command_id,
        "kind": str(kind),
        "command_space": "eef",
        "source": str(owner),
        "owner": str(owner),
        "mode": MotionMode.AGENT_SERVO.value,
        "backend": str(backend),
        "start_pose_policy": str(start_pose_policy),
        "expected_q_start": q_start,
        "start_pose_guard": start_pose_guard,
        "eef_command": eef_command,
        "eef_reference_limit": eef_reference_limit,
        "max_start_error_rad": float(max_start_error_rad),
        "heartbeat_timeout_s": float(heartbeat_timeout_s),
        "max_heartbeat_age_s": float(max_heartbeat_age_s),
        "send_hz": float(send_hz),
        "submitted_wall_time_s": time.time(),
        "session_artifact": str(session_artifact_path),
        "result_artifact": str(result_path),
        "hardware_execution_requires_backend_adapter": True,
        "mature_backend_policy": _eef_mature_backend_policy(str(backend)),
    }
    pending_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    _write_json_atomic(pending_path, command)
    payload = {
        "status": "queued",
        "schema": "armctrl.arm_runtime_submit.v1",
        "command_id": command_id,
        "owner": str(owner),
        "mode": MotionMode.AGENT_SERVO.value,
        "command_space": "eef",
        "backend": str(backend),
        "runtime_session_id": session.get("runtime_session_id"),
        "movement_command_sent": False,
        "hardware_executable_now": True,
        "start_pose_policy": str(start_pose_policy),
        "start_pose_guard": start_pose_guard,
        "eef_reference_limit": eef_reference_limit,
        "eef_adapter_manager": adapter_gate["eef_adapter_manager"],
        "runtime_controller_manager": session.get("runtime_controller_manager"),
        "runtime": {
            "single_motion_owner": True,
            "queue": str(queue_dir),
            "session_artifact": str(session_artifact_path),
        },
        "artifacts": {
            "command": str(pending_path),
            "result": str(result_path),
        },
        "mature_backend_policy": command["mature_backend_policy"],
        "next_gate": (
            "serve with a configured mature EEF backend adapter inside the same "
            "long-lived runtime; disconnected sdk_cartesian takeover is diagnostic-only"
        ),
    }
    return _write_optional_output(payload, output_path)


def runtime_command_queue_dir(session_artifact_path: Path) -> Path:
    return session_artifact_path.parent / f"{session_artifact_path.stem}_commands"


def _eef_mature_backend_policy(backend: str) -> dict[str, object]:
    return {
        "selected": str(backend),
        "no_heuristic_joint_fallback": True,
        "execution_model": "continuous_owner_eef_servo",
        "disconnected_takeover_allowed": False,
        "bumpless_switch_required": True,
        "warmup_policy": (
            "seed the EEF target from current FK, run zero-command warmup, "
            "and only then accept live EEF owner commands"
        ),
        "reference_limit_policy": (
            "reject EEF commands whose per-tick linear/angular reference "
            "step exceeds configured limits; do not silently clamp"
        ),
        "fallback_policy": "stay_in_hold_or_damping; never release SDK/CAN between modes",
        "planned_path_alternative": (
            "reviewed EEF pose goals may compile to joint_trajectory, but "
            "that is not the realtime EEF servo path"
        ),
    }


def _eef_submit_adapter_gate(
    *,
    session: dict[str, object],
    requested_backend: str,
) -> dict[str, object]:
    manager = session.get("eef_adapter_manager")
    if not isinstance(manager, dict):
        manager = {
            "schema": "armctrl.eef_adapter_manager.v1",
            "status": "unconfigured",
            "configured_adapters": [],
            "eef_command_executable": False,
            "adapter_registry_required": True,
            "primary_backend_fallback_allowed": False,
            "disconnected_takeover_allowed": False,
        }
    configured = [
        str(adapter)
        for adapter in manager.get("configured_adapters", [])
        if isinstance(adapter, str)
    ]
    executable = bool(manager.get("eef_command_executable"))
    if executable and str(requested_backend) in configured:
        return {
            "status": "pass",
            "eef_adapter_manager": manager,
            "reason": "requested EEF backend is configured in the live runtime adapter registry",
        }
    return {
        "status": "blocked",
        "eef_adapter_manager": manager,
        "reason": (
            "EEF command requires a configured mature EEF backend adapter inside "
            "the live runtime before it can be queued; requested_backend="
            f"{requested_backend!s}, configured_adapters={configured!r}"
        ),
    }


def execute_pending_runtime_commands(
    *,
    session_artifact_path: Path,
    backend: MotionBackend,
    runtime: ArmRuntime | None = None,
    eef_backends: Mapping[str, MotionBackend] | None = None,
    max_heartbeat_age_s: float,
) -> dict[str, object] | None:
    queue_dir = runtime_command_queue_dir(session_artifact_path)
    pending_dir = queue_dir / "pending"
    if not pending_dir.exists():
        return None
    command_paths = sorted(pending_dir.glob("*.json"))
    if not command_paths:
        return None
    command_path = _next_pending_command_path(command_paths)
    running_path = queue_dir / "running" / command_path.name
    running_path.parent.mkdir(parents=True, exist_ok=True)
    command_path.replace(running_path)
    command = _read_json_object(running_path)
    command["dequeued_wall_time_s"] = time.time()
    _write_json_atomic(running_path, command)
    result = execute_runtime_command(
        command=command,
        session_artifact_path=session_artifact_path,
        backend=backend,
        runtime=runtime,
        eef_backends=eef_backends,
        max_heartbeat_age_s=max_heartbeat_age_s,
    )
    result_path = Path(str(command.get("result_artifact") or ""))
    if not result_path:
        result_path = queue_dir / "results" / running_path.name
    result_path.parent.mkdir(parents=True, exist_ok=True)
    _write_json_atomic(result_path, result)
    archive_path = queue_dir / "completed" / running_path.name
    archive_path.parent.mkdir(parents=True, exist_ok=True)
    running_path.replace(archive_path)
    return result


def execute_runtime_command(
    *,
    command: dict[str, object],
    session_artifact_path: Path,
    backend: MotionBackend,
    runtime: ArmRuntime | None = None,
    eef_backends: Mapping[str, MotionBackend] | None = None,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    if command.get("schema") != RUNTIME_COMMAND_SCHEMA:
        raise ValueError("invalid runtime command schema")
    session = _read_json_object(session_artifact_path)
    owner = str(command.get("owner"))
    mode = str(command.get("mode"))
    owner_acquire_started_wall_time_s = time.time()
    try:
        _ensure_can_queue_command(
            session,
            owner=owner,
            expected_q_start=_object_float_list(command.get("expected_q_start")),
            start_pose_policy=str(command.get("start_pose_policy", "live_hold")),
            max_start_error_rad=float(command.get("max_start_error_rad", 0.02)),
            heartbeat_timeout_s=float(command.get("heartbeat_timeout_s", 0.5)),
            max_heartbeat_age_s=float(max_heartbeat_age_s),
        )
        command_backend = _resolve_command_execution_backend(
            command,
            session=session,
            primary_backend=backend,
            eef_backends=eef_backends,
        )
        eef_controller_manager = _eef_controller_manager_contract(
            command=command,
            session=session,
            phase="resolved_backend",
        )
        _ensure_command_has_executable_backend(command, backend=command_backend)
        live_runtime = runtime
        if live_runtime is None:
            live_runtime = ArmRuntime(
                backend=backend,
                safe_center=tuple(_object_float_list(session.get("safe_center"))),
                runtime_session_id=str(session.get("runtime_session_id")),
            )
            live_runtime.mark_hold_safe(
                q_hold=tuple(_object_float_list(session.get("q_hold")))
            )
        expected_q_start = _owner_expected_q_start(command, backend=backend)
        lease = live_runtime.acquire_owner(
            owner=owner,
            mode=MotionMode(mode),
            expected_q_start=expected_q_start,
            max_start_error_rad=float(command.get("max_start_error_rad", 0.02)),
            heartbeat_timeout_s=float(command.get("heartbeat_timeout_s", 0.5)),
        )
        owner_acquired_wall_time_s = time.time()
        session = _session_from_runtime_status(
            session,
            status=live_runtime.status(),
            max_heartbeat_age_s=max_heartbeat_age_s,
        )
        owner_wall_time_s = time.time()
        session["owner_lease"] = {
            "schema": "armctrl.arm_runtime_owner_lease.v1",
            "runtime_session_id": lease.runtime_session_id,
            "owner": lease.owner,
            "mode": lease.mode,
            "heartbeat_timeout_s": float(command.get("heartbeat_timeout_s", 0.5)),
            "heartbeat_wall_time_s": owner_wall_time_s,
            "acquired_wall_time_s": owner_wall_time_s,
            "landing_policy": "watchdog_to_damping_release_to_hold_safe",
        }
        if command.get("kind") == "joint_intent":
            session["owner_lease"].update(_intent_owner_watchdog_contract(command))
        session["owner_deadman"] = {
            "owner": lease.owner,
            "mode": lease.mode,
            "heartbeat_wall_time_s": owner_wall_time_s,
            "heartbeat_age_s": 0.0,
            "heartbeat_timeout_s": float(command.get("heartbeat_timeout_s", 0.5)),
            "fresh": True,
        }
        session["readiness"] = runtime_readiness(session)
        _write_json_atomic(session_artifact_path, session)
        progress = _active_owner_status_progress(
            session_artifact_path=session_artifact_path,
            runtime=live_runtime,
            owner=owner,
            mode=mode,
            heartbeat_timeout_s=float(command.get("heartbeat_timeout_s", 0.5)),
            max_heartbeat_age_s=max_heartbeat_age_s,
            min_period_s=0.05,
        )
        status_update = progress.stats
        first_send_wall_time_s: float | None = None

        def on_motion_sample(sample: MotionAuditSample) -> None:
            nonlocal first_send_wall_time_s
            if first_send_wall_time_s is None:
                first_send_wall_time_s = time.time()
            progress(sample)

        eef_switch: dict[str, object] | None = None
        if command.get("kind") in EEF_COMMAND_KINDS:
            eef_switch = _prepare_eef_servo_switch(
                command=command,
                backend=command_backend,
            )
            eef_controller_manager = _eef_controller_manager_contract(
                command=command,
                session=session,
                phase="warmup_passed",
                eef_switch=eef_switch,
            )
        motion = _execute_motion_command(
            command,
            runtime=live_runtime,
            backend=command_backend,
            watchdog=lambda: _owner_timeout_watchdog(
                session_artifact_path=session_artifact_path,
                owner=owner,
                executor_progress_wall_time_s=progress.executor_progress_wall_time_s(),
            ),
            on_sample=on_motion_sample,
        )
        watchdog = _owner_timeout_watchdog(
            session_artifact_path=session_artifact_path,
            owner=owner,
            executor_progress_wall_time_s=progress.executor_progress_wall_time_s(),
        )
        if watchdog is not None:
            backend.damping()
            session = _fault_session_for_owner_timeout(
                session=_read_json_object(session_artifact_path),
                watchdog=watchdog,
                max_heartbeat_age_s=max_heartbeat_age_s,
            )
            _write_json_atomic(session_artifact_path, session)
            return _command_result_payload(
                command=command,
                runtime_session_id=session.get("runtime_session_id"),
                session_artifact_path=session_artifact_path,
                status="faulted",
                landing_mode=MotionMode.DAMPING.value,
                motion=motion,
                reason="owner_heartbeat_timeout",
                watchdog=watchdog,
                owner_acquire_started_wall_time_s=owner_acquire_started_wall_time_s,
                owner_acquired_wall_time_s=owner_acquired_wall_time_s,
                first_send_wall_time_s=first_send_wall_time_s,
                status_update=status_update,
                eef_switch=eef_switch,
                eef_controller_manager=eef_controller_manager,
            )
        if motion.status == "completed":
            progress.publish()
            status = live_runtime.status()
            session = _session_from_runtime_status(
                session,
                status=status,
                max_heartbeat_age_s=max_heartbeat_age_s,
            )
            if session.get("mode") == "hold_safe":
                session["hold_fresh"] = True
                session["last_hold_wall_time_s"] = time.time()
                session["hold_age_s"] = 0.0
                session["readiness"] = runtime_readiness(session)
                session["status"] = (
                    "ok"
                    if session["readiness"]["agent_sysid_smoke_allowed"] is True
                    else "blocked"
                )
        else:
            progress.publish()
            if motion.landing_mode == MotionMode.HOLD.value:
                session = _session_from_runtime_status(
                    session,
                    status=live_runtime.status(),
                    max_heartbeat_age_s=max_heartbeat_age_s,
                )
                if session.get("mode") == "hold_safe":
                    session["hold_fresh"] = True
                    session["last_hold_wall_time_s"] = time.time()
                    session["hold_age_s"] = 0.0
                    session["readiness"] = runtime_readiness(session)
                    session["status"] = (
                        "ok"
                        if session["readiness"]["agent_sysid_smoke_allowed"] is True
                        else "blocked"
                    )
            else:
                session["status"] = "faulted"
                session["mode"] = motion.landing_mode
                session["owner"] = None
                session["owner_lease"] = None
                session["readiness"] = runtime_readiness(session)
        _write_json_atomic(session_artifact_path, session)
        return _command_result_payload(
            command=command,
            runtime_session_id=session.get("runtime_session_id"),
            session_artifact_path=session_artifact_path,
            status=motion.status,
            landing_mode=motion.landing_mode,
            motion=motion,
            reason=None,
            owner_acquire_started_wall_time_s=owner_acquire_started_wall_time_s,
            owner_acquired_wall_time_s=owner_acquired_wall_time_s,
            first_send_wall_time_s=first_send_wall_time_s,
            status_update=status_update,
            eef_switch=eef_switch,
            eef_controller_manager=eef_controller_manager,
        )
    except (ArmRuntimeError, RuntimeSessionError, ValueError) as error:
        rejected_wall_time_s = time.time()
        return {
            "status": "rejected",
            "schema": RUNTIME_COMMAND_RESULT_SCHEMA,
            "command_id": command.get("command_id"),
            "owner": owner,
            "mode": mode,
            "command_space": (
                "eef" if command.get("kind") in EEF_COMMAND_KINDS else None
            ),
            "kind": command.get("kind"),
            "start_pose_policy": str(command.get("start_pose_policy", "live_hold")),
            "movement_command_sent": False,
            "reason": str(error),
            "start_pose_guard": _error_start_pose_guard(error),
            "landing_mode": None,
            "eef_command": (
                _eef_motion_contract(command)
                if command.get("kind") in EEF_COMMAND_KINDS
                else None
            ),
            "eef_switch": (
                _eef_switch_rejected_contract(command)
                if command.get("kind") in EEF_COMMAND_KINDS
                else None
            ),
            "eef_controller_manager": (
                _eef_controller_manager_contract(
                    command=command,
                    session=session,
                    phase="rejected",
                    rejection_reason=str(error),
                )
                if command.get("kind") in EEF_COMMAND_KINDS
                else None
            ),
            "timing": {
                "submitted_wall_time_s": _optional_float(
                    command.get("submitted_wall_time_s")
                ),
                "dequeued_wall_time_s": _optional_float(
                    command.get("dequeued_wall_time_s")
                ),
                "owner_acquire_started_wall_time_s": owner_acquire_started_wall_time_s,
                "owner_acquired_wall_time_s": None,
                "first_send_wall_time_s": None,
                "first_send_monotonic_s": None,
                "last_send_monotonic_s": None,
                "completed_wall_time_s": rejected_wall_time_s,
                "queue_latency_s": _duration_s(
                    _optional_float(command.get("dequeued_wall_time_s")),
                    _optional_float(command.get("submitted_wall_time_s")),
                ),
                "acquire_latency_s": None,
                "first_send_latency_s": None,
                "execution_elapsed_s": None,
                "total_wall_latency_s": _duration_s(
                    rejected_wall_time_s,
                    _optional_float(command.get("submitted_wall_time_s")),
                ),
            },
        }


def _next_pending_command_path(command_paths: Sequence[Path]) -> Path:
    return min(command_paths, key=_pending_command_sort_key)


def _pending_command_sort_key(command_path: Path) -> tuple[float, str]:
    try:
        command = _read_json_object(command_path)
        submitted_wall_time_s = float(command.get("submitted_wall_time_s"))
    except (OSError, ValueError, TypeError):
        submitted_wall_time_s = float("inf")
    return submitted_wall_time_s, command_path.name


def _execute_motion_command(
    command: dict[str, object],
    *,
    runtime: ArmRuntime,
    backend: MotionBackend,
    watchdog=None,
    on_sample=None,
) -> MotionExecutionResult:
    kind = command.get("kind")
    send_hz = float(command.get("send_hz", 50.0))
    if kind == "joint_trajectory":
        q_points = command.get("q_points")
        if not isinstance(q_points, list):
            raise ValueError("trajectory command requires q_points")
        trajectory_sample_hz = float(command.get("trajectory_sample_hz", send_hz))
        raw_points = [tuple(_object_float_list(q_point)) for q_point in q_points]
        _ensure_joint_trajectory_safety(command, q_points=raw_points)
        dq_points = command.get("dq_points")
        raw_velocities = (
            None
            if not isinstance(dq_points, list)
            else [tuple(_object_float_list(dq_point)) for dq_point in dq_points]
        )
        points = _runtime_trajectory_points(
            raw_points,
            dq_points=raw_velocities,
            trajectory_sample_hz=trajectory_sample_hz,
            runtime_send_hz=send_hz,
        )
        return runtime.execute_owner_trajectory(
            points,
            owner=str(command.get("owner")),
            trajectory_sample_hz=send_hz,
            watchdog=watchdog,
            on_sample=on_sample,
            max_tracking_error_rad=_optional_float(
                command.get("max_tracking_error_rad")
            ),
            tracking_error_grace_samples=_optional_int(
                command.get("tracking_error_grace_samples")
            )
            or 0,
            tracking_error_consecutive_samples=_optional_int(
                command.get("tracking_error_consecutive_samples")
            )
            or 1,
            max_tau_abs=_optional_float(command.get("max_tau_abs")),
        )
    if kind == "joint_intent":
        _ensure_joint_intent_safety(command)
        return runtime.execute_owner_intent_frame(
            JointIntentFrame(
                q_start=tuple(_object_float_list(command.get("expected_q_start"))),
                q_target=tuple(_object_float_list(command.get("q_target"))),
                control_period_s=float(command.get("control_period_s")),
                max_joint_delta_rad=(
                    None
                    if command.get("max_joint_delta_rad") is None
                    else float(command.get("max_joint_delta_rad"))
                ),
                max_joint_velocity_rad_s=(
                    None
                    if command.get("max_joint_velocity_rad_s") is None
                    else float(command.get("max_joint_velocity_rad_s"))
                ),
            ),
            owner=str(command.get("owner")),
            send_hz=send_hz,
            watchdog=watchdog,
            on_sample=on_sample,
            max_tracking_error_rad=_optional_float(
                command.get("max_tracking_error_rad")
            ),
            tracking_error_grace_samples=_optional_int(
                command.get("tracking_error_grace_samples")
            )
            or 0,
            tracking_error_consecutive_samples=_optional_int(
                command.get("tracking_error_consecutive_samples")
            )
            or 1,
            max_tau_abs=_optional_float(command.get("max_tau_abs")),
        )
    if kind in EEF_COMMAND_KINDS:
        _ensure_eef_reference_limit(command)
        runtime._raise_if_not_owned(owner=str(command.get("owner")), mode=MotionMode.AGENT_SERVO)
        executor = getattr(backend, "execute_eef_command", None)
        if not callable(executor):
            raise ValueError(
                "EEF runtime command requires a configured mature backend adapter "
                "(moveit_servo or sdk_cartesian); heuristic joint fallback is forbidden"
            )
        result = executor(
            command,
            owner=str(command.get("owner")),
            watchdog=watchdog,
            on_sample=on_sample,
        )
        return runtime._finish_owner_motion(owner=str(command.get("owner")), result=result)
    raise ValueError(f"unsupported runtime command kind: {kind}")


def _resolve_command_execution_backend(
    command: dict[str, object],
    *,
    session: dict[str, object],
    primary_backend: MotionBackend,
    eef_backends: Mapping[str, MotionBackend] | None,
) -> MotionBackend:
    command["_resolved_runtime_backend"] = str(session.get("backend") or "unknown")
    if command.get("kind") not in EEF_COMMAND_KINDS:
        return primary_backend
    adapter_name = str(command.get("backend") or "")
    if eef_backends is not None and adapter_name in eef_backends:
        command["_resolved_eef_adapter"] = adapter_name
        command["_eef_adapter_resolution"] = "configured_adapter_registry"
        return eef_backends[adapter_name]
    command["_resolved_eef_adapter"] = None
    command["_eef_adapter_resolution"] = "missing_adapter_registry"
    return primary_backend


def _prepare_eef_servo_switch(
    *,
    command: dict[str, object],
    backend: MotionBackend,
) -> dict[str, object]:
    """Prepare an EEF backend without releasing the live runtime owner."""

    q_before = tuple(float(value) for value in backend.read_joint_state().q_meas)
    prepare = getattr(backend, "prepare_eef_servo_switch", None)
    if callable(prepare):
        payload = prepare(command)
        if not isinstance(payload, dict):
            raise ValueError("prepare_eef_servo_switch must return a JSON object")
        payload = dict(payload)
    else:
        payload = {
            "schema": "armctrl.eef_servo_switch.v1",
            "status": "pass",
            "policy": "default_live_state_zero_command_warmup",
            "backend": command.get("backend"),
            "target_seed": "current_live_state",
            "zero_command_warmup_ticks": 1,
            "checks": {
                "sdk_owner_released": False,
                "target_seeded_from_current_state": True,
                "zero_command_warmup_completed": True,
            },
        }
    checks = payload.get("checks")
    if not isinstance(checks, dict):
        checks = {}
    checks = dict(checks)
    checks.setdefault("sdk_owner_released", False)
    checks.setdefault("target_seeded_from_current_state", True)
    checks.setdefault("zero_command_warmup_completed", True)
    checks["live_q_read_before_switch"] = bool(q_before)
    q_after = tuple(float(value) for value in backend.read_joint_state().q_meas)
    checks["live_q_read_after_switch"] = bool(q_after)
    payload["checks"] = checks
    payload.setdefault("schema", "armctrl.eef_servo_switch.v1")
    payload.setdefault("status", "pass")
    payload.setdefault("backend", command.get("backend"))
    payload.setdefault("policy", "continuous_owner_bumpless_switch")
    payload["disconnected_takeover_allowed"] = False
    payload["sdk_owner_released"] = bool(checks.get("sdk_owner_released"))
    payload["q_before"] = list(q_before)
    payload["q_after"] = list(q_after)
    failed_checks = [
        name
        for name, passed in checks.items()
        if name != "sdk_owner_released" and isinstance(passed, bool) and not passed
    ]
    if checks.get("sdk_owner_released") is True:
        failed_checks.append("sdk_owner_released")
    payload["failed_checks"] = failed_checks
    if failed_checks or payload["sdk_owner_released"]:
        payload["status"] = "fail"
        raise ValueError(
            "EEF servo switch warmup failed: " + ", ".join(failed_checks or ["sdk_owner_released"])
        )
    return payload


def _owner_expected_q_start(
    command: dict[str, object],
    *,
    backend: MotionBackend,
) -> tuple[float, ...]:
    if (
        command.get("kind") in EEF_COMMAND_KINDS
        and str(command.get("start_pose_policy", "")) == "current_measured_pose"
    ):
        return tuple(float(value) for value in backend.read_joint_state().q_meas)
    return tuple(_object_float_list(command.get("expected_q_start")))


def _ensure_command_has_executable_backend(
    command: dict[str, object],
    *,
    backend: MotionBackend,
) -> None:
    if command.get("kind") not in EEF_COMMAND_KINDS:
        return
    if command.get("_eef_adapter_resolution") != "configured_adapter_registry":
        raise ValueError(
            "EEF runtime command requires a configured mature backend adapter "
            "inside the same long-lived runtime; primary backend fallback and "
            "heuristic joint fallback are forbidden"
        )
    executor = getattr(backend, "execute_eef_command", None)
    if callable(executor):
        return
    raise ValueError(
        "EEF runtime command requires a configured mature backend adapter "
        "(moveit_servo or sdk_cartesian); heuristic joint fallback is forbidden"
    )


def _session_from_runtime_status(
    session: dict[str, object],
    *,
    status: dict[str, object],
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    updated = dict(session)
    updated["status"] = "ok"
    updated["mode"] = status.get("mode")
    updated["owner"] = status.get("owner")
    updated["owner_lease"] = None
    updated["owner_deadman"] = None
    updated["q_meas"] = list(status.get("q_meas") or [])
    updated["q_hold"] = list(status.get("q_hold") or [])
    updated["fault_flags"] = list(status.get("fault_flags") or [])
    updated["hold_fresh"] = False
    updated["hold_age_s"] = None
    updated["last_hold_wall_time_s"] = None
    updated = heartbeat_runtime_session_payload(
        updated,
        max_heartbeat_age_s=max_heartbeat_age_s,
    )
    return updated


def _active_owner_status_progress(
    *,
    session_artifact_path: Path,
    runtime: ArmRuntime,
    owner: str,
    mode: str,
    heartbeat_timeout_s: float,
    max_heartbeat_age_s: float,
    min_period_s: float,
):
    last_due_monotonic_s: float | None = None
    executor_progress_wall_time_s: float | None = None
    pending_publish = False
    stats: dict[str, object] = {
        "policy": "deferred_throttled_active_owner_status",
        "send_loop_write_policy": "deferred_publish_outside_send_loop",
        "sample_count": 0,
        "session_write_count": 0,
        "min_period_s": float(min_period_s),
        "target_status_hz": 1.0 / float(min_period_s),
        "deferred_publish_count": 0,
    }

    def on_sample(sample: MotionAuditSample) -> None:
        nonlocal last_due_monotonic_s, pending_publish, executor_progress_wall_time_s
        stats["sample_count"] = int(stats["sample_count"]) + 1
        executor_progress_wall_time_s = time.time()
        if (
            last_due_monotonic_s is not None
            and sample.sent_monotonic_s - last_due_monotonic_s < min_period_s
        ):
            return
        last_due_monotonic_s = sample.sent_monotonic_s
        pending_publish = True
        stats["deferred_publish_count"] = int(stats["deferred_publish_count"]) + 1

    def publish() -> None:
        nonlocal pending_publish
        if not pending_publish:
            return
        pending_publish = False
        _refresh_active_owner_session(
            session_artifact_path=session_artifact_path,
            runtime=runtime,
            owner=owner,
            mode=mode,
            heartbeat_timeout_s=heartbeat_timeout_s,
            max_heartbeat_age_s=max_heartbeat_age_s,
            executor_progress_wall_time_s=executor_progress_wall_time_s,
        )
        stats["session_write_count"] = int(stats["session_write_count"]) + 1

    def executor_progress_wall_time() -> float | None:
        return executor_progress_wall_time_s

    on_sample.stats = stats
    on_sample.publish = publish
    on_sample.executor_progress_wall_time_s = executor_progress_wall_time
    return on_sample


def _refresh_active_owner_session(
    *,
    session_artifact_path: Path,
    runtime: ArmRuntime,
    owner: str,
    mode: str,
    heartbeat_timeout_s: float,
    max_heartbeat_age_s: float,
    executor_progress_wall_time_s: float | None = None,
) -> None:
    session = _read_json_object(session_artifact_path)
    if session.get("owner") != owner:
        return
    owner_lease = session.get("owner_lease")
    acquired_wall_time_s = (
        owner_lease.get("acquired_wall_time_s")
        if isinstance(owner_lease, dict)
        else None
    )
    owner_heartbeat_wall_time_s = (
        owner_lease.get("heartbeat_wall_time_s")
        if isinstance(owner_lease, dict)
        else None
    )
    status = runtime.status()
    updated = _session_from_runtime_status(
        session,
        status=status,
        max_heartbeat_age_s=max_heartbeat_age_s,
    )
    updated["owner"] = owner
    updated["mode"] = mode
    updated["owner_lease"] = {
        "schema": "armctrl.arm_runtime_owner_lease.v1",
        "runtime_session_id": status.get("runtime_session_id"),
        "owner": owner,
        "mode": mode,
        "heartbeat_timeout_s": float(heartbeat_timeout_s),
        "heartbeat_wall_time_s": owner_heartbeat_wall_time_s,
        "executor_progress_wall_time_s": executor_progress_wall_time_s or time.time(),
        "acquired_wall_time_s": acquired_wall_time_s,
        "landing_policy": "watchdog_to_damping_release_to_hold_safe",
    }
    if mode == MotionMode.AGENT_SERVO.value and isinstance(owner_lease, dict):
        for key in (
            "last_intent_wall_time_s",
            "missed_intent_timeout_s",
            "fault_timeout_s",
            "missed_intent_held",
            "landing_policy",
        ):
            if key in owner_lease:
                updated["owner_lease"][key] = owner_lease[key]
    owner_heartbeat_age_s = _wall_age_s(owner_heartbeat_wall_time_s)
    updated["owner_deadman"] = {
        "owner": owner,
        "mode": mode,
        "heartbeat_wall_time_s": owner_heartbeat_wall_time_s,
        "heartbeat_age_s": owner_heartbeat_age_s,
        "heartbeat_timeout_s": float(heartbeat_timeout_s),
        "fresh": (
            owner_heartbeat_age_s is not None
            and owner_heartbeat_age_s < float(heartbeat_timeout_s)
        ),
    }
    updated["status_publish"] = {
        "policy": "throttled_observation_not_owner_heartbeat",
        "wall_time_s": time.time(),
    }
    updated["readiness"] = runtime_readiness(updated)
    _write_json_atomic(session_artifact_path, updated)


def _command_result_payload(
    *,
    command: dict[str, object],
    runtime_session_id: object,
    session_artifact_path: Path,
    status: str,
    landing_mode: str,
    motion: MotionExecutionResult,
    reason: str | None,
    owner_acquire_started_wall_time_s: float,
    owner_acquired_wall_time_s: float,
    first_send_wall_time_s: float | None,
    watchdog: dict[str, object] | None = None,
    status_update: dict[str, object] | None = None,
    eef_switch: dict[str, object] | None = None,
    eef_controller_manager: dict[str, object] | None = None,
) -> dict[str, object]:
    completed_wall_time_s = time.time()
    timing = _command_timing_summary(
        command=command,
        motion=motion,
        owner_acquire_started_wall_time_s=owner_acquire_started_wall_time_s,
        owner_acquired_wall_time_s=owner_acquired_wall_time_s,
        first_send_wall_time_s=first_send_wall_time_s,
        completed_wall_time_s=completed_wall_time_s,
    )
    send_period = _send_period_summary(motion.samples)
    runtime_send_hz = _command_runtime_send_hz(command, motion)
    expected_period_s = 1.0 / runtime_send_hz if runtime_send_hz else None
    trajectory_sample_hz = _command_trajectory_sample_hz(command, motion)
    resampling_policy = _trajectory_resampling_policy(
        trajectory_sample_hz=trajectory_sample_hz,
        runtime_send_hz=runtime_send_hz,
        kind=str(command.get("kind")),
    )
    interpolation_policy = _trajectory_interpolation_policy(
        resampling_policy=resampling_policy,
        kind=str(command.get("kind")),
    )
    motion_payload = {
        "status": motion.status,
        "producer": motion.producer,
        "mode": motion.mode,
        "trajectory_sample_hz": trajectory_sample_hz,
        "runtime_send_hz": runtime_send_hz,
        "resampling_policy": resampling_policy,
        "interpolation_policy": interpolation_policy,
        "actual_send_hz": motion.actual_send_hz,
        "send_jitter_ms_p95": motion.send_jitter_ms_p95,
        "send_jitter_ms_p99": motion.send_jitter_ms_p99,
        "send_jitter_ms_summary": _send_jitter_summary(
            motion.samples,
            expected_period_s=expected_period_s,
        ),
        "dt_error_ms_summary": _dt_error_summary(
            motion.samples,
            expected_period_s=expected_period_s,
        ),
        "send_period_s": send_period,
        "dt_min_s": send_period["min"],
        "dt_max_s": send_period["max"],
        "dt_avg_s": send_period["avg"],
        "tracking_error": _tracking_error_summary(motion.samples),
        "max_tracking_error_rad": _optional_float(
            command.get("max_tracking_error_rad")
        ),
        "tracking_error_policy": _tracking_error_policy(command),
        "tau_meas": _tau_summary(motion.samples),
        "max_tau_abs": _optional_float(command.get("max_tau_abs")),
        "controller_dt_s": motion.controller_dt_s,
        "sample_count": len(motion.samples),
        "samples": [_motion_sample_manifest(sample) for sample in motion.samples],
        "landing_mode": motion.landing_mode,
        "error": motion.error,
    }
    status_update_payload = dict(status_update) if status_update is not None else None
    payload = {
        "status": status,
        "schema": RUNTIME_COMMAND_RESULT_SCHEMA,
        "command_id": command.get("command_id"),
        "owner": command.get("owner"),
        "mode": command.get("mode"),
        "kind": command.get("kind"),
        "command_space": (
            "eef" if command.get("kind") in EEF_COMMAND_KINDS else "joint"
        ),
        "runtime_session_id": runtime_session_id,
        "movement_command_sent": bool(motion.samples),
        "start_pose_policy": str(command.get("start_pose_policy", "live_hold")),
        "start_pose_guard": command.get("start_pose_guard"),
        "landing_mode": landing_mode,
        "reason": reason,
        "timing": timing,
        "artifacts": {
            "session": str(session_artifact_path),
            "result": str(command.get("result_artifact") or ""),
        },
        "motion": motion_payload,
        "acceptance": _command_acceptance_summary(
            status=status,
            command=command,
            timing=timing,
            motion=motion_payload,
            status_update=status_update_payload,
        ),
    }
    if status_update_payload is not None:
        payload["status_update"] = status_update_payload
    if command.get("kind") == "joint_trajectory":
        payload["motion"].update(_joint_trajectory_motion_contract(command))
    if command.get("kind") == "joint_intent":
        payload["motion"].update(_intent_motion_contract(command))
    if command.get("kind") in EEF_COMMAND_KINDS:
        payload["motion"]["eef_command"] = _eef_motion_contract(command)
        payload["eef_switch"] = eef_switch or _eef_switch_rejected_contract(command)
        payload["eef_controller_manager"] = (
            eef_controller_manager
            or _eef_controller_manager_contract(
                command=command,
                session={},
                phase="unknown",
            )
        )
    if watchdog is not None:
        payload["watchdog"] = watchdog
    return payload


def _eef_motion_contract(command: dict[str, object]) -> dict[str, object]:
    return {
        "kind": command.get("kind"),
        "backend": command.get("backend"),
        "runtime_backend": command.get("_resolved_runtime_backend"),
        "eef_adapter": command.get("_resolved_eef_adapter"),
        "adapter_resolution": command.get("_eef_adapter_resolution"),
        "command_space": "eef",
        "eef_command": command.get("eef_command"),
        "eef_reference_limit": command.get("eef_reference_limit"),
        "mature_backend_policy": command.get("mature_backend_policy"),
    }


def _tracking_error_policy(command: dict[str, object]) -> dict[str, object]:
    max_error = _optional_float(command.get("max_tracking_error_rad"))
    grace_samples = _optional_int(command.get("tracking_error_grace_samples"))
    consecutive_samples = _optional_int(
        command.get("tracking_error_consecutive_samples")
    )
    return {
        "schema": "armctrl.tracking_error_policy.v1",
        "enabled": max_error is not None and max_error > 0.0,
        "max_tracking_error_rad": max_error,
        "grace_samples": 0 if grace_samples is None else grace_samples,
        "consecutive_samples": (
            1 if consecutive_samples is None else consecutive_samples
        ),
        "landing_mode_on_violation": MotionMode.HOLD.value,
        "damping_on_tracking_error": False,
        "policy": "controlled_hold_after_debounced_tracking_error",
    }


def _joint_trajectory_motion_contract(command: dict[str, object]) -> dict[str, object]:
    artifact_policy = command.get("artifact_policy")
    return {
        "command_space": "joint",
        "joint_trajectory_safety": command.get("joint_trajectory_safety"),
        "max_joint_segment_delta_rad": _optional_float(
            command.get("max_joint_segment_delta_rad")
        ),
        "max_joint_velocity_rad_s": _optional_float(
            command.get("max_joint_velocity_rad_s")
        ),
        "artifact_policy": artifact_policy if isinstance(artifact_policy, dict) else None,
        "q_policy": (
            artifact_policy.get("q_cmd") if isinstance(artifact_policy, dict) else None
        ),
        "dq_policy": (
            artifact_policy.get("dq_cmd") if isinstance(artifact_policy, dict) else None
        ),
        "ddq_policy": (
            artifact_policy.get("ddq_cmd") if isinstance(artifact_policy, dict) else None
        ),
        "runtime_interpolator": "cubic_hermite_joint_position_velocity",
        "runtime_velocity_source": (
            "preserved_or_compiled_dq_points"
            if isinstance(command.get("dq_points"), list)
            else "runtime_finite_difference_dq_points"
        ),
    }


def _eef_switch_rejected_contract(command: dict[str, object]) -> dict[str, object]:
    return {
        "schema": "armctrl.eef_servo_switch.v1",
        "status": "not_run",
        "backend": command.get("backend"),
        "policy": "continuous_owner_bumpless_switch",
        "disconnected_takeover_allowed": False,
        "sdk_owner_released": False,
        "reason": "command rejected before EEF servo switch warmup",
    }


def _eef_controller_manager_contract(
    *,
    command: dict[str, object],
    session: dict[str, object],
    phase: str,
    eef_switch: dict[str, object] | None = None,
    rejection_reason: str | None = None,
) -> dict[str, object]:
    adapter_resolution = command.get("_eef_adapter_resolution")
    warmup_status = (
        eef_switch.get("status")
        if isinstance(eef_switch, dict)
        else ("not_run" if phase == "rejected" else None)
    )
    ready = (
        command.get("kind") in EEF_COMMAND_KINDS
        and adapter_resolution == "configured_adapter_registry"
        and warmup_status == "pass"
    )
    start_pose_policy = str(command.get("start_pose_policy", "live_hold"))
    return {
        "schema": "armctrl.eef_controller_manager.v1",
        "status": "ready" if ready else ("rejected" if phase == "rejected" else "pending"),
        "phase": str(phase),
        "runtime_session_id": session.get("runtime_session_id"),
        "owner": command.get("owner"),
        "command_kind": command.get("kind"),
        "command_space": "eef",
        "runtime_backend": command.get("_resolved_runtime_backend"),
        "eef_adapter": command.get("_resolved_eef_adapter"),
        "adapter_resolution": adapter_resolution,
        "start_pose_policy": start_pose_policy,
        "pose_start_contract": (
            "live runtime hold pose"
            if start_pose_policy == "live_hold"
            else start_pose_policy
        ),
        "passive_safe_policy": (
            "passive/droop pose is not an Agent EEF controlled start pose; "
            "EEF owner may start only after live runtime hold/readiness"
        ),
        "safe_position_policy": (
            "SAFE_CENTER is a joint runtime hold/recovery pose; EEF servo "
            "initializes target from current live FK after owner acquisition"
        ),
        "mode_switch_policy": "runtime_internal_controller_manager",
        "controller_sequence": [
            "joint_hold_active",
            "resolve_explicit_eef_adapter",
            "acquire_single_runtime_owner",
            "seed_eef_target_from_current_state",
            "zero_command_warmup",
            "execute_eef_servo_command",
            "release_to_hold_or_fault_to_damping",
        ],
        "bumpless_switch_required": True,
        "adapter_registry_required": True,
        "primary_backend_fallback_allowed": False,
        "disconnected_takeover_allowed": False,
        "no_heuristic_joint_fallback": True,
        "warmup_gate": eef_switch or _eef_switch_rejected_contract(command),
        "rejection_reason": rejection_reason,
    }


def _command_timing_summary(
    *,
    command: dict[str, object],
    motion: MotionExecutionResult,
    owner_acquire_started_wall_time_s: float,
    owner_acquired_wall_time_s: float,
    first_send_wall_time_s: float | None,
    completed_wall_time_s: float,
) -> dict[str, object]:
    submitted_wall_time_s = _optional_float(command.get("submitted_wall_time_s"))
    dequeued_wall_time_s = _optional_float(command.get("dequeued_wall_time_s"))
    first_send_monotonic_s = (
        motion.samples[0].sent_monotonic_s if motion.samples else None
    )
    last_send_monotonic_s = (
        motion.samples[-1].sent_monotonic_s if motion.samples else None
    )
    execution_elapsed_s = (
        last_send_monotonic_s - first_send_monotonic_s
        if first_send_monotonic_s is not None and last_send_monotonic_s is not None
        else None
    )
    return {
        "submitted_wall_time_s": submitted_wall_time_s,
        "dequeued_wall_time_s": dequeued_wall_time_s,
        "owner_acquire_started_wall_time_s": owner_acquire_started_wall_time_s,
        "owner_acquired_wall_time_s": owner_acquired_wall_time_s,
        "first_send_wall_time_s": first_send_wall_time_s,
        "first_send_monotonic_s": first_send_monotonic_s,
        "last_send_monotonic_s": last_send_monotonic_s,
        "completed_wall_time_s": completed_wall_time_s,
        "queue_latency_s": _duration_s(dequeued_wall_time_s, submitted_wall_time_s),
        "acquire_latency_s": _duration_s(
            owner_acquired_wall_time_s,
            dequeued_wall_time_s,
        ),
        "first_send_latency_s": _duration_s(
            first_send_wall_time_s,
            owner_acquired_wall_time_s,
        ),
        "execution_elapsed_s": execution_elapsed_s,
        "total_wall_latency_s": _duration_s(completed_wall_time_s, submitted_wall_time_s),
    }


def _command_acceptance_summary(
    *,
    status: str,
    command: dict[str, object],
    timing: dict[str, object],
    motion: dict[str, object],
    status_update: dict[str, object] | None,
) -> dict[str, object]:
    timing_gate = _timing_gate_summary(
        command=command,
        timing=timing,
        motion=motion,
    )
    status_publish_gate = _status_publish_gate_summary(status_update)
    checks = {
        "command_completed": status == "completed",
        "motion_completed": motion.get("status") == "completed",
        "timing_gate": timing_gate["status"] == "pass",
        "status_publish_gate": status_publish_gate["status"] == "pass",
    }
    return {
        "status": "pass" if all(checks.values()) else "fail",
        "checks": checks,
        "timing_gate": timing_gate,
        "status_publish_gate": status_publish_gate,
    }


def _status_publish_gate_summary(
    status_update: dict[str, object] | None,
) -> dict[str, object]:
    sample_count = _optional_int(
        status_update.get("sample_count") if status_update is not None else None
    )
    session_write_count = _optional_int(
        status_update.get("session_write_count") if status_update is not None else None
    )
    target_status_hz = _optional_float(
        status_update.get("target_status_hz") if status_update is not None else None
    )
    min_period_s = _optional_float(
        status_update.get("min_period_s") if status_update is not None else None
    )
    send_loop_write_policy = (
        status_update.get("send_loop_write_policy")
        if status_update is not None
        else None
    )
    session_write_ratio = (
        None
        if sample_count is None or sample_count <= 0 or session_write_count is None
        else session_write_count / sample_count
    )
    enough_samples_for_ratio_gate = sample_count is not None and sample_count >= 10
    checks = {
        "status_update_present": status_update is not None,
        "session_writes_below_sample_count": (
            sample_count is not None
            and session_write_count is not None
            and (
                session_write_count < sample_count
                or not enough_samples_for_ratio_gate
            )
        ),
        "session_writes_below_10_percent_samples": (
            (session_write_ratio is not None and session_write_ratio < 0.1)
            or not enough_samples_for_ratio_gate
        ),
        "send_loop_write_policy_deferred": (
            send_loop_write_policy == "deferred_publish_outside_send_loop"
        ),
        "target_status_hz_within_limit": (
            target_status_hz is not None and target_status_hz <= 20.0
        ),
    }
    return {
        "status": "pass" if all(checks.values()) else "fail",
        "checks": checks,
        "limits": {
            "session_write_ratio_max": 0.1,
            "target_status_hz_max": 20.0,
        },
        "metrics": {
            "sample_count": sample_count,
            "session_write_count": session_write_count,
            "session_write_ratio": session_write_ratio,
            "ratio_gate_min_sample_count": 10,
            "target_status_hz": target_status_hz,
            "min_period_s": min_period_s,
            "send_loop_write_policy": send_loop_write_policy,
        },
    }


def _timing_gate_summary(
    *,
    command: dict[str, object],
    timing: dict[str, object],
    motion: dict[str, object],
) -> dict[str, object]:
    runtime_send_hz = _optional_float(motion.get("runtime_send_hz"))
    actual_send_hz = _optional_float(motion.get("actual_send_hz"))
    dt_min_s = _optional_float(motion.get("dt_min_s"))
    dt_max_s = _optional_float(motion.get("dt_max_s"))
    sample_count = int(motion.get("sample_count") or 0)
    expected_sample_count = _expected_runtime_sample_count(command)
    queue_latency_s = _optional_float(timing.get("queue_latency_s"))
    expected_period_s = 1.0 / runtime_send_hz if runtime_send_hz else None
    send_hz_error_ratio = (
        None
        if runtime_send_hz is None or actual_send_hz is None or runtime_send_hz <= 0.0
        else abs(actual_send_hz - runtime_send_hz) / runtime_send_hz
    )
    dt_range_error_ms = (
        None
        if expected_period_s is None or dt_min_s is None or dt_max_s is None
        else max(abs(dt_min_s - expected_period_s), abs(dt_max_s - expected_period_s))
        * 1000.0
    )
    p99_jitter_ms = _optional_float(motion.get("send_jitter_ms_p99"))
    if p99_jitter_ms is None:
        jitter_summary = motion.get("send_jitter_ms_summary")
        if isinstance(jitter_summary, dict):
            p99_jitter_ms = _optional_float(jitter_summary.get("max"))
    checks = {
        "actual_send_hz_present": actual_send_hz is not None,
        "actual_send_hz_close_to_runtime_send_hz": (
            send_hz_error_ratio is not None and send_hz_error_ratio <= 0.05
        ),
        "send_jitter_p99_within_limit": (
            p99_jitter_ms is not None and p99_jitter_ms <= 5.0
        ),
        "dt_range_within_limit": (
            dt_range_error_ms is not None and dt_range_error_ms <= 5.0
        ),
        "queue_latency_present": queue_latency_s is not None,
        "sample_count_matches_expected": (
            expected_sample_count is not None and sample_count == expected_sample_count
        ),
    }
    return {
        "status": "pass" if all(checks.values()) else "fail",
        "checks": checks,
        "limits": {
            "actual_send_hz_error_ratio_max": 0.05,
            "send_jitter_p99_ms_max": 5.0,
            "dt_range_error_ms_max": 5.0,
        },
        "metrics": {
            "runtime_send_hz": runtime_send_hz,
            "actual_send_hz": actual_send_hz,
            "actual_send_hz_error_ratio": send_hz_error_ratio,
            "send_jitter_ms_p99": p99_jitter_ms,
            "dt_min_s": dt_min_s,
            "dt_max_s": dt_max_s,
            "dt_range_error_ms": dt_range_error_ms,
            "queue_latency_s": queue_latency_s,
            "sample_count": sample_count,
            "expected_sample_count": expected_sample_count,
        },
    }


def _expected_runtime_sample_count(command: dict[str, object]) -> int | None:
    kind = command.get("kind")
    if kind == "joint_trajectory":
        q_points = command.get("q_points")
        if not isinstance(q_points, list) or not q_points:
            return None
        if len(q_points) == 1:
            return 1
        trajectory_sample_hz = _optional_float(command.get("trajectory_sample_hz"))
        runtime_send_hz = _optional_float(command.get("send_hz"))
        if (
            trajectory_sample_hz is None
            or runtime_send_hz is None
            or trajectory_sample_hz <= 0.0
            or runtime_send_hz <= 0.0
        ):
            return len(q_points)
        duration_s = (len(q_points) - 1) / trajectory_sample_hz
        return max(2, int(round(duration_s * runtime_send_hz)) + 1)
    if kind == "joint_intent":
        control_period_s = _optional_float(command.get("control_period_s"))
        send_hz = _optional_float(command.get("send_hz"))
        if (
            control_period_s is None
            or send_hz is None
            or control_period_s <= 0.0
            or send_hz <= 0.0
        ):
            return None
        return max(1, int(round(control_period_s * send_hz))) + 1
    return None


def _send_period_summary(samples: Sequence[MotionAuditSample]) -> dict[str, object]:
    if len(samples) < 2:
        return {"min": None, "max": None, "avg": None, "count": 0}
    periods = [
        right.sent_monotonic_s - left.sent_monotonic_s
        for left, right in zip(samples, samples[1:])
    ]
    return {
        "min": min(periods),
        "max": max(periods),
        "avg": sum(periods) / len(periods),
        "count": len(periods),
    }


def _send_jitter_summary(
    samples: Sequence[MotionAuditSample],
    *,
    expected_period_s: float | None,
) -> dict[str, object]:
    if expected_period_s is None or expected_period_s <= 0.0 or len(samples) < 2:
        return {"min": None, "max": None, "avg": None, "count": 0}
    jitters_ms = [
        abs((right.sent_monotonic_s - left.sent_monotonic_s) - expected_period_s)
        * 1000.0
        for left, right in zip(samples, samples[1:])
    ]
    return {
        "min": min(jitters_ms),
        "max": max(jitters_ms),
        "avg": sum(jitters_ms) / len(jitters_ms),
        "count": len(jitters_ms),
        "buckets": _jitter_buckets(jitters_ms),
    }


def _dt_error_summary(
    samples: Sequence[MotionAuditSample],
    *,
    expected_period_s: float | None,
) -> dict[str, object]:
    if expected_period_s is None or expected_period_s <= 0.0 or len(samples) < 2:
        return {"min": None, "max": None, "avg": None, "count": 0}
    errors_ms = [
        ((right.sent_monotonic_s - left.sent_monotonic_s) - expected_period_s)
        * 1000.0
        for left, right in zip(samples, samples[1:])
    ]
    return {
        "min": min(errors_ms),
        "max": max(errors_ms),
        "avg": sum(errors_ms) / len(errors_ms),
        "count": len(errors_ms),
        "buckets": _dt_error_buckets(errors_ms),
    }


def _dt_error_buckets(errors_ms: Sequence[float]) -> dict[str, int]:
    buckets = {
        "early_gt_5": 0,
        "early_le_5": 0,
        "early_le_2": 0,
        "early_le_1": 0,
        "on_time_le_0_5": 0,
        "late_le_1": 0,
        "late_le_2": 0,
        "late_le_5": 0,
        "late_gt_5": 0,
    }
    for error_ms in errors_ms:
        if error_ms < -5.0:
            buckets["early_gt_5"] += 1
        elif error_ms < -2.0:
            buckets["early_le_5"] += 1
        elif error_ms < -1.0:
            buckets["early_le_2"] += 1
        elif error_ms < -0.5:
            buckets["early_le_1"] += 1
        elif error_ms <= 0.5:
            buckets["on_time_le_0_5"] += 1
        elif error_ms <= 1.0:
            buckets["late_le_1"] += 1
        elif error_ms <= 2.0:
            buckets["late_le_2"] += 1
        elif error_ms <= 5.0:
            buckets["late_le_5"] += 1
        else:
            buckets["late_gt_5"] += 1
    return buckets


def _jitter_buckets(jitters_ms: Sequence[float]) -> dict[str, int]:
    buckets = {"le_0_5": 0, "le_1": 0, "le_2": 0, "le_5": 0, "gt_5": 0}
    for jitter_ms in jitters_ms:
        if jitter_ms <= 0.5:
            buckets["le_0_5"] += 1
        elif jitter_ms <= 1.0:
            buckets["le_1"] += 1
        elif jitter_ms <= 2.0:
            buckets["le_2"] += 1
        elif jitter_ms <= 5.0:
            buckets["le_5"] += 1
        else:
            buckets["gt_5"] += 1
    return buckets


def _tracking_error_summary(samples: Sequence[MotionAuditSample]) -> dict[str, object]:
    if not samples:
        return {
            "sample_count": 0,
            "max_abs_rad": None,
            "final_abs_rad": None,
            "per_joint_max_abs_rad": [],
            "final_error_rad": [],
        }
    max_dof = max(
        len(sample.q_cmd)
        for sample in samples
    )
    per_joint_max_abs = [0.0] * max_dof
    max_abs = 0.0
    final_error: list[float] = []
    for sample in samples:
        errors = [
            float(measured) - float(commanded)
            for commanded, measured in zip(sample.q_cmd, sample.q_meas, strict=False)
        ]
        if sample is samples[-1]:
            final_error = errors
        for index, error in enumerate(errors):
            abs_error = abs(error)
            per_joint_max_abs[index] = max(per_joint_max_abs[index], abs_error)
            max_abs = max(max_abs, abs_error)
    final_abs = max((abs(error) for error in final_error), default=None)
    return {
        "sample_count": len(samples),
        "max_abs_rad": max_abs,
        "final_abs_rad": final_abs,
        "per_joint_max_abs_rad": per_joint_max_abs,
        "final_error_rad": final_error,
    }


def _tau_summary(samples: Sequence[MotionAuditSample]) -> dict[str, object]:
    if not samples:
        return {
            "sample_count": 0,
            "max_abs": None,
            "final_abs": None,
            "per_joint_max_abs": [],
            "final_tau": [],
        }
    max_dof = max((len(sample.tau_meas) for sample in samples), default=0)
    per_joint_max_abs = [0.0] * max_dof
    max_abs = 0.0
    final_tau = [float(value) for value in samples[-1].tau_meas]
    for sample in samples:
        for index, tau in enumerate(sample.tau_meas):
            abs_tau = abs(float(tau))
            per_joint_max_abs[index] = max(per_joint_max_abs[index], abs_tau)
            max_abs = max(max_abs, abs_tau)
    final_abs = max((abs(value) for value in final_tau), default=None)
    return {
        "sample_count": len(samples),
        "max_abs": max_abs,
        "final_abs": final_abs,
        "per_joint_max_abs": per_joint_max_abs,
        "final_tau": final_tau,
    }


def _runtime_trajectory_points(
    q_points: Sequence[tuple[float, ...]],
    *,
    dq_points: Sequence[tuple[float, ...]] | None = None,
    trajectory_sample_hz: float,
    runtime_send_hz: float,
) -> list[JointTrajectoryPoint]:
    if not q_points:
        raise ValueError("trajectory command requires q_points")
    if trajectory_sample_hz <= 0.0:
        raise ValueError("trajectory_sample_hz must be positive")
    if runtime_send_hz <= 0.0:
        raise ValueError("send_hz must be positive")
    if dq_points is not None:
        if len(dq_points) != len(q_points):
            raise ValueError("dq_points length must match q_points length")
        for q_point, dq_point in zip(q_points, dq_points, strict=True):
            if len(dq_point) != len(q_point):
                raise ValueError("dq_points must match q point length")
    effective_dq_points = (
        list(dq_points)
        if dq_points is not None
        else _finite_difference_velocities(
            q_points,
            trajectory_sample_hz=trajectory_sample_hz,
        )
    )
    if len(q_points) == 1:
        return [
            JointTrajectoryPoint(
                time_s=0.0,
                q=tuple(q_points[0]),
                dq=tuple(effective_dq_points[0]),
            )
        ]
    duration_s = (len(q_points) - 1) / float(trajectory_sample_hz)
    sample_count = int(round(duration_s * float(runtime_send_hz))) + 1
    sample_count = max(2, sample_count)
    return [
        JointTrajectoryPoint(
            time_s=index / float(runtime_send_hz),
            q=_sample_cubic_hermite_joint_trajectory(
                q_points,
                effective_dq_points,
                trajectory_sample_hz=trajectory_sample_hz,
                sample_time_s=min(duration_s, index / float(runtime_send_hz)),
                derivative=False,
            ),
            dq=_sample_cubic_hermite_joint_trajectory(
                q_points,
                effective_dq_points,
                trajectory_sample_hz=trajectory_sample_hz,
                sample_time_s=min(duration_s, index / float(runtime_send_hz)),
                derivative=True,
            ),
        )
        for index in range(sample_count)
    ]


def _finite_difference_velocities(
    q_points: Sequence[tuple[float, ...]],
    *,
    trajectory_sample_hz: float,
) -> list[tuple[float, ...]]:
    if not q_points:
        return []
    if len(q_points) == 1:
        return [tuple(0.0 for _ in q_points[0])]
    dt_s = 1.0 / float(trajectory_sample_hz)
    velocities: list[tuple[float, ...]] = []
    for index, point in enumerate(q_points):
        if index == 0:
            left = point
            right = q_points[index + 1]
            divisor = dt_s
        elif index == len(q_points) - 1:
            left = q_points[index - 1]
            right = point
            divisor = dt_s
        else:
            left = q_points[index - 1]
            right = q_points[index + 1]
            divisor = 2.0 * dt_s
        velocities.append(
            tuple(
                (right_value - left_value) / divisor
                for left_value, right_value in zip(left, right, strict=True)
            )
        )
    return velocities


def _sample_linear_joint_trajectory(
    q_points: Sequence[tuple[float, ...]],
    *,
    trajectory_sample_hz: float,
    sample_time_s: float,
) -> tuple[float, ...]:
    if sample_time_s <= 0.0:
        return tuple(q_points[0])
    raw_index = sample_time_s * float(trajectory_sample_hz)
    left_index = min(int(raw_index), len(q_points) - 1)
    right_index = min(left_index + 1, len(q_points) - 1)
    if left_index == right_index:
        return tuple(q_points[left_index])
    ratio = raw_index - left_index
    return tuple(
        left + (right - left) * ratio
        for left, right in zip(q_points[left_index], q_points[right_index], strict=True)
    )


def _sample_cubic_hermite_joint_trajectory(
    q_points: Sequence[tuple[float, ...]],
    dq_points: Sequence[tuple[float, ...]],
    *,
    trajectory_sample_hz: float,
    sample_time_s: float,
    derivative: bool,
) -> tuple[float, ...]:
    if sample_time_s <= 0.0 or len(q_points) == 1:
        return tuple(dq_points[0] if derivative else q_points[0])
    dt_s = 1.0 / float(trajectory_sample_hz)
    raw_index = sample_time_s * float(trajectory_sample_hz)
    left_index = min(int(raw_index), len(q_points) - 1)
    right_index = min(left_index + 1, len(q_points) - 1)
    if left_index == right_index:
        return tuple(dq_points[left_index] if derivative else q_points[left_index])
    s = raw_index - left_index
    q0 = q_points[left_index]
    q1 = q_points[right_index]
    v0 = dq_points[left_index]
    v1 = dq_points[right_index]
    if derivative:
        return tuple(
            ((6.0 * s * s - 6.0 * s) / dt_s) * left
            + (3.0 * s * s - 4.0 * s + 1.0) * left_vel
            + ((-6.0 * s * s + 6.0 * s) / dt_s) * right
            + (3.0 * s * s - 2.0 * s) * right_vel
            for left, right, left_vel, right_vel in zip(q0, q1, v0, v1, strict=True)
        )
    h00 = 2.0 * s * s * s - 3.0 * s * s + 1.0
    h10 = s * s * s - 2.0 * s * s + s
    h01 = -2.0 * s * s * s + 3.0 * s * s
    h11 = s * s * s - s * s
    values: list[float] = []
    for left, right, left_vel, right_vel in zip(q0, q1, v0, v1, strict=True):
        value = (
            h00 * left
            + h10 * dt_s * left_vel
            + h01 * right
            + h11 * dt_s * right_vel
        )
        lower = min(left, right)
        upper = max(left, right)
        values.append(min(max(value, lower), upper))
    return tuple(values)


def _command_runtime_send_hz(
    command: dict[str, object],
    motion: MotionExecutionResult,
) -> float | None:
    return _optional_float(command.get("send_hz")) or motion.trajectory_sample_hz


def _command_trajectory_sample_hz(
    command: dict[str, object],
    motion: MotionExecutionResult,
) -> float | None:
    if command.get("kind") != "joint_trajectory":
        return motion.trajectory_sample_hz
    return _optional_float(command.get("trajectory_sample_hz")) or motion.trajectory_sample_hz


def _trajectory_resampling_policy(
    *,
    trajectory_sample_hz: float | None,
    runtime_send_hz: float | None,
    kind: str,
) -> str:
    if kind != "joint_trajectory":
        return "intent_frame_to_runtime_send_hz"
    if (
        trajectory_sample_hz is not None
        and runtime_send_hz is not None
        and abs(trajectory_sample_hz - runtime_send_hz) <= 1e-9
    ):
        return "none_sample_hz_matches_send_hz"
    return "cubic_hermite_time_resample"


def _trajectory_interpolation_policy(*, resampling_policy: str, kind: str) -> str:
    if kind != "joint_trajectory":
        return "smoothstep_intent_frame"
    if resampling_policy == "cubic_hermite_time_resample":
        return "bounded_cubic_hermite_joint_position_velocity"
    return "pre_sampled_joint_positions"


def _intent_motion_contract(command: dict[str, object]) -> dict[str, object]:
    control_period_s = _optional_float(command.get("control_period_s"))
    return {
        "agent_intent_hz": (
            None if control_period_s is None else 1.0 / control_period_s
        ),
        "interpolation_policy": "smoothstep_intent_frame",
        "intent_trajectory_contract": command.get("intent_trajectory_contract"),
        "max_joint_delta_rad": _optional_float(command.get("max_joint_delta_rad")),
        "max_joint_velocity_rad_s": _optional_float(
            command.get("max_joint_velocity_rad_s")
        ),
        "joint_intent_safety": command.get("joint_intent_safety"),
        "missed_intent_policy": "hold_then_damping",
        "missed_intent_timeout_s": 0.3,
        "fault_timeout_s": _optional_float(command.get("heartbeat_timeout_s")),
    }


def _joint_intent_trajectory_contract(
    *,
    q_start: Sequence[float],
    q_target: Sequence[float],
    control_period_s: float,
    send_hz: float,
    max_joint_delta_rad: float | None,
    max_joint_velocity_rad_s: float | None,
) -> dict[str, object]:
    if control_period_s <= 0.0:
        raise ValueError("control_period_s must be positive")
    if send_hz <= 0.0:
        raise ValueError("send_hz must be positive")
    deltas = [
        float(target) - float(start)
        for start, target in zip(q_start, q_target, strict=True)
    ]
    max_abs_delta = max((abs(value) for value in deltas), default=0.0)
    max_abs_velocity = max_abs_delta / float(control_period_s)
    expected_runtime_sample_count = (
        max(1, int(round(float(control_period_s) * float(send_hz)))) + 1
    )
    return {
        "schema": "armctrl.joint_intent_trajectory_contract.v1",
        "command_space": "joint",
        "source_mode": "joint_intent",
        "q_policy": "live_hold_to_target_smoothstep",
        "dq_policy": "derived_smoothstep_analytic",
        "ddq_policy": "not_commanded_runtime_smoothstep",
        "interpolation_policy": "smoothstep_intent_frame",
        "resampling_policy": "intent_frame_to_runtime_send_hz",
        "runtime_send_hz": float(send_hz),
        "control_period_s": float(control_period_s),
        "expected_runtime_sample_count": expected_runtime_sample_count,
        "max_joint_delta_rad": (
            None if max_joint_delta_rad is None else float(max_joint_delta_rad)
        ),
        "max_joint_velocity_rad_s": (
            None
            if max_joint_velocity_rad_s is None
            else float(max_joint_velocity_rad_s)
        ),
        "per_joint_delta_rad": deltas,
        "max_abs_delta_rad": max_abs_delta,
        "max_abs_velocity_rad_s": max_abs_velocity,
        "runtime_controller": "MotionRuntime.execute_owner_intent_frame",
        "artifact_role": (
            "short-horizon smooth trajectory contract for low-frequency Agent "
            "joint targets"
        ),
    }


def _intent_owner_watchdog_contract(command: dict[str, object]) -> dict[str, object]:
    now_s = time.time()
    return {
        "landing_policy": "missed_intent_hold_then_damping",
        "last_intent_wall_time_s": now_s,
        "missed_intent_timeout_s": 0.3,
        "fault_timeout_s": _optional_float(command.get("heartbeat_timeout_s")),
        "missed_intent_held": False,
    }


def _ensure_eef_reference_limit(command: dict[str, object]) -> None:
    eef_command = command.get("eef_command")
    if not isinstance(eef_command, dict):
        raise ValueError("EEF command requires eef_command payload")
    existing = command.get("eef_reference_limit")
    existing_max_linear = (
        _optional_float(existing.get("max_linear_step_m"))
        if isinstance(existing, dict)
        else None
    )
    existing_max_angular = (
        _optional_float(existing.get("max_angular_step_rad"))
        if isinstance(existing, dict)
        else None
    )
    guard = _eef_reference_limit_guard(
        kind=str(command.get("kind")),
        eef_command=eef_command,
        max_linear_step_m=(
            DEFAULT_EEF_MAX_LINEAR_STEP_M
            if existing_max_linear is None
            else existing_max_linear
        ),
        max_angular_step_rad=(
            DEFAULT_EEF_MAX_ANGULAR_STEP_RAD
            if existing_max_angular is None
            else existing_max_angular
        ),
    )
    command["eef_reference_limit"] = guard
    if guard["status"] != "pass":
        raise ValueError(
            "EEF reference limit failed: "
            + ", ".join(str(item) for item in guard["failed_checks"])
        )


def _ensure_joint_trajectory_safety(
    command: dict[str, object],
    *,
    q_points: Sequence[Sequence[float]],
) -> None:
    existing = command.get("joint_trajectory_safety")
    existing_max_segment_delta = (
        _optional_float(existing.get("max_joint_segment_delta_rad"))
        if isinstance(existing, dict)
        else None
    )
    existing_max_velocity = (
        _optional_float(existing.get("max_joint_velocity_rad_s"))
        if isinstance(existing, dict)
        else None
    )
    existing_max_acceleration = (
        _optional_float(existing.get("max_joint_acceleration_rad_s2"))
        if isinstance(existing, dict)
        else None
    )
    owner = str(command.get("owner"))
    max_velocity = existing_max_velocity
    if max_velocity is None and owner == "agent":
        max_velocity = DEFAULT_AGENT_MAX_JOINT_VELOCITY_RAD_S
    guard = _joint_trajectory_safety_guard(
        owner=owner,
        q_points=q_points,
        trajectory_sample_hz=float(
            command.get("trajectory_sample_hz", command.get("send_hz", 50.0))
        ),
        max_joint_segment_delta_rad=existing_max_segment_delta,
        max_joint_velocity_rad_s=max_velocity,
        max_joint_acceleration_rad_s2=existing_max_acceleration,
    )
    command["joint_trajectory_safety"] = guard
    if guard["status"] != "pass":
        raise ValueError(
            "joint trajectory safety failed: "
            + ", ".join(str(item) for item in guard["failed_checks"])
        )


def _ensure_joint_intent_safety(command: dict[str, object]) -> None:
    existing = command.get("joint_intent_safety")
    existing_max_delta = (
        _optional_float(existing.get("max_joint_delta_rad"))
        if isinstance(existing, dict)
        else None
    )
    existing_max_velocity = (
        _optional_float(existing.get("max_joint_velocity_rad_s"))
        if isinstance(existing, dict)
        else None
    )
    command_max_delta = _optional_float(command.get("max_joint_delta_rad"))
    command_max_velocity = _optional_float(command.get("max_joint_velocity_rad_s"))
    max_delta = existing_max_delta if existing_max_delta is not None else command_max_delta
    max_velocity = (
        existing_max_velocity
        if existing_max_velocity is not None
        else command_max_velocity
    )
    if max_velocity is None and str(command.get("owner")) == "agent":
        max_velocity = DEFAULT_AGENT_MAX_JOINT_VELOCITY_RAD_S
    guard = _joint_intent_safety_guard(
        q_start=_object_float_list(command.get("expected_q_start")),
        q_target=_object_float_list(command.get("q_target")),
        control_period_s=float(command.get("control_period_s")),
        max_joint_delta_rad=max_delta,
        max_joint_velocity_rad_s=max_velocity,
    )
    command["joint_intent_safety"] = guard
    if guard["status"] != "pass":
        raise ValueError(
            "joint intent safety failed: "
            + ", ".join(str(item) for item in guard["failed_checks"])
        )


def _eef_reference_limit_guard(
    *,
    kind: str,
    eef_command: dict[str, object],
    max_linear_step_m: float,
    max_angular_step_rad: float,
) -> dict[str, object]:
    max_linear = float(max_linear_step_m)
    max_angular = float(max_angular_step_rad)
    if max_linear <= 0.0:
        raise ValueError("max_linear_step_m must be positive")
    if max_angular <= 0.0:
        raise ValueError("max_angular_step_rad must be positive")
    if kind == "eef_pose_delta":
        linear_step = _fixed_float_list(
            eef_command.get("delta_position_m"), name="delta_position_m", length=3
        )
        angular_step = _fixed_float_list(
            eef_command.get("delta_rpy_rad"), name="delta_rpy_rad", length=3
        )
    elif kind == "eef_twist":
        control_period_s = float(eef_command.get("control_period_s"))
        if control_period_s <= 0.0:
            raise ValueError("control_period_s must be positive")
        linear_step = [
            value * control_period_s
            for value in _fixed_float_list(
                eef_command.get("linear_mps"), name="linear_mps", length=3
            )
        ]
        angular_step = [
            value * control_period_s
            for value in _fixed_float_list(
                eef_command.get("angular_rps"), name="angular_rps", length=3
            )
        ]
    elif kind == "eef_pose":
        target_position = _fixed_float_list(
            eef_command.get("position_m"), name="position_m", length=3
        )
        target_rpy = _fixed_float_list(
            eef_command.get("rpy_rad"), name="rpy_rad", length=3
        )
        pose_limiter = eef_command.get("pose_reference_limiter")
        if pose_limiter != "adapter_live_reference_limit":
            return {
                "schema": "armctrl.eef_reference_limit.v1",
                "status": "fail",
                "policy": "require_adapter_live_reference_limit_for_absolute_pose",
                "max_linear_step_m": max_linear,
                "max_angular_step_rad": max_angular,
                "target_position_m": target_position,
                "target_rpy_rad": target_rpy,
                "linear_step_max_abs_m": max(
                    (abs(value) for value in target_position),
                    default=0.0,
                ),
                "angular_step_max_abs_rad": max(
                    (abs(value) for value in target_rpy),
                    default=0.0,
                ),
                "failed_checks": ["pose_reference_limiter_configured"],
            }
        return {
            "schema": "armctrl.eef_reference_limit.v1",
            "status": "pass",
            "policy": "adapter_live_reference_limit_for_absolute_pose",
            "max_linear_step_m": max_linear,
            "max_angular_step_rad": max_angular,
            "target_position_m": target_position,
            "target_rpy_rad": target_rpy,
            "pose_reference_limiter": pose_limiter,
            "failed_checks": [],
        }
    else:
        raise ValueError(f"unsupported EEF runtime command kind: {kind}")
    linear_max_abs = max((abs(value) for value in linear_step), default=0.0)
    angular_max_abs = max((abs(value) for value in angular_step), default=0.0)
    failed_checks: list[str] = []
    if linear_max_abs > max_linear:
        failed_checks.append("linear_step_within_limit")
    if angular_max_abs > max_angular:
        failed_checks.append("angular_step_within_limit")
    return {
        "schema": "armctrl.eef_reference_limit.v1",
        "status": "pass" if not failed_checks else "fail",
        "policy": "reject_oversized_eef_reference_step",
        "max_linear_step_m": max_linear,
        "max_angular_step_rad": max_angular,
        "linear_step_m": linear_step,
        "angular_step_rad": angular_step,
        "linear_step_max_abs_m": linear_max_abs,
        "angular_step_max_abs_rad": angular_max_abs,
        "failed_checks": failed_checks,
    }


def _joint_intent_safety_guard(
    *,
    q_start: Sequence[float],
    q_target: Sequence[float],
    control_period_s: float,
    max_joint_delta_rad: float | None,
    max_joint_velocity_rad_s: float | None,
) -> dict[str, object]:
    if control_period_s <= 0.0:
        raise ValueError("control_period_s must be positive")
    if len(q_start) != len(q_target):
        raise ValueError("q_start and q_target must have the same length")
    deltas = [float(target) - float(start) for start, target in zip(q_start, q_target)]
    per_joint_abs_delta = [abs(value) for value in deltas]
    per_joint_abs_velocity = [
        abs(value) / float(control_period_s) for value in deltas
    ]
    max_abs_delta = max(per_joint_abs_delta, default=0.0)
    max_abs_velocity = max(per_joint_abs_velocity, default=0.0)
    failed_checks: list[str] = []
    max_delta = None if max_joint_delta_rad is None else float(max_joint_delta_rad)
    if max_delta is not None:
        if max_delta <= 0.0:
            raise ValueError("max_joint_delta_rad must be positive")
        if max_abs_delta > max_delta:
            failed_checks.append("joint_delta_within_limit")
    max_velocity = (
        None
        if max_joint_velocity_rad_s is None
        else float(max_joint_velocity_rad_s)
    )
    if max_velocity is not None:
        if max_velocity <= 0.0:
            raise ValueError("max_joint_velocity_rad_s must be positive")
        if max_abs_velocity > max_velocity:
            failed_checks.append("joint_velocity_within_limit")
    return {
        "schema": "armctrl.joint_intent_safety.v1",
        "status": "pass" if not failed_checks else "fail",
        "policy": "reject_oversized_or_too_fast_joint_intent",
        "control_period_s": float(control_period_s),
        "max_joint_delta_rad": max_delta,
        "max_joint_velocity_rad_s": max_velocity,
        "per_joint_delta_rad": deltas,
        "per_joint_abs_delta_rad": per_joint_abs_delta,
        "per_joint_abs_velocity_rad_s": per_joint_abs_velocity,
        "max_abs_delta_rad": max_abs_delta,
        "max_abs_velocity_rad_s": max_abs_velocity,
        "failed_checks": failed_checks,
    }


def _joint_trajectory_safety_guard(
    *,
    owner: str,
    q_points: Sequence[Sequence[float]],
    trajectory_sample_hz: float,
    max_joint_segment_delta_rad: float | None,
    max_joint_velocity_rad_s: float | None,
    max_joint_acceleration_rad_s2: float | None,
) -> dict[str, object]:
    if trajectory_sample_hz <= 0.0:
        raise ValueError("trajectory_sample_hz must be positive")
    max_segment_delta = (
        None
        if max_joint_segment_delta_rad is None
        else float(max_joint_segment_delta_rad)
    )
    if max_segment_delta is not None and max_segment_delta <= 0.0:
        raise ValueError("max_joint_segment_delta_rad must be positive")
    max_velocity = (
        None
        if max_joint_velocity_rad_s is None
        else float(max_joint_velocity_rad_s)
    )
    if max_velocity is not None and max_velocity <= 0.0:
        raise ValueError("max_joint_velocity_rad_s must be positive")
    max_acceleration = (
        None
        if max_joint_acceleration_rad_s2 is None
        else float(max_joint_acceleration_rad_s2)
    )
    if max_acceleration is not None and max_acceleration <= 0.0:
        raise ValueError("max_joint_acceleration_rad_s2 must be positive")

    segment_deltas: list[list[float]] = []
    segment_abs_deltas: list[list[float]] = []
    segment_abs_velocities: list[list[float]] = []
    for previous, current in zip(q_points, q_points[1:]):
        deltas = [
            float(current_value) - float(previous_value)
            for previous_value, current_value in zip(previous, current)
        ]
        abs_deltas = [abs(value) for value in deltas]
        segment_deltas.append(deltas)
        segment_abs_deltas.append(abs_deltas)
        segment_abs_velocities.append(
            [value * float(trajectory_sample_hz) for value in abs_deltas]
        )
    segment_abs_accelerations: list[list[float]] = []
    for previous_velocity, current_velocity in zip(
        segment_abs_velocities,
        segment_abs_velocities[1:],
    ):
        segment_abs_accelerations.append(
            [
                abs(float(current_value) - float(previous_value))
                * float(trajectory_sample_hz)
                for previous_value, current_value in zip(
                    previous_velocity,
                    current_velocity,
                )
            ]
        )
    max_abs_delta = max(
        (value for segment in segment_abs_deltas for value in segment),
        default=0.0,
    )
    max_abs_velocity = max(
        (value for segment in segment_abs_velocities for value in segment),
        default=0.0,
    )
    max_abs_acceleration = max(
        (value for segment in segment_abs_accelerations for value in segment),
        default=0.0,
    )
    failed_checks: list[str] = []
    if max_segment_delta is not None and max_abs_delta > max_segment_delta:
        failed_checks.append("joint_segment_delta_within_limit")
    if max_velocity is not None and max_abs_velocity > max_velocity:
        failed_checks.append("joint_velocity_within_limit")
    if max_acceleration is not None and max_abs_acceleration > max_acceleration:
        failed_checks.append("joint_acceleration_within_limit")
    policy = (
        "reject_oversized_or_too_fast_agent_joint_trajectory"
        if str(owner) == "agent"
        else "record_only_for_non_agent_joint_trajectory"
    )
    return {
        "schema": "armctrl.joint_trajectory_safety.v1",
        "status": "pass" if not failed_checks else "fail",
        "policy": policy,
        "owner": str(owner),
        "trajectory_sample_hz": float(trajectory_sample_hz),
        "max_joint_segment_delta_rad": max_segment_delta,
        "max_joint_velocity_rad_s": max_velocity,
        "max_joint_acceleration_rad_s2": max_acceleration,
        "segment_delta_rad": segment_deltas,
        "segment_abs_delta_rad": segment_abs_deltas,
        "segment_abs_velocity_rad_s": segment_abs_velocities,
        "segment_abs_acceleration_rad_s2": segment_abs_accelerations,
        "max_segment_abs_delta_rad": max_abs_delta,
        "max_segment_abs_velocity_rad_s": max_abs_velocity,
        "max_segment_abs_acceleration_rad_s2": max_abs_acceleration,
        "failed_checks": failed_checks,
    }


def _optional_float(value: object) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _optional_int(value: object) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _duration_s(later: float | None, earlier: float | None) -> float | None:
    if later is None or earlier is None:
        return None
    return max(0.0, later - earlier)


def _motion_sample_manifest(sample: MotionAuditSample) -> dict[str, object]:
    return {
        "sent_monotonic_s": sample.sent_monotonic_s,
        "q_cmd": list(sample.q_cmd),
        "dq_cmd": list(sample.dq_cmd),
        "q_meas": list(sample.q_meas),
        "dq_meas": list(sample.dq_meas),
        "tau_meas": list(sample.tau_meas),
        "fault_flags": list(sample.fault_flags),
        "producer": sample.producer,
        "mode": sample.mode,
    }


def _owner_timeout_watchdog(
    *,
    session_artifact_path: Path,
    owner: str,
    executor_progress_wall_time_s: float | None = None,
) -> dict[str, object] | None:
    session = _read_json_object(session_artifact_path)
    owner_lease = session.get("owner_lease")
    if session.get("owner") != owner or not isinstance(owner_lease, dict):
        return None
    try:
        heartbeat_wall_time_s = float(
            executor_progress_wall_time_s
            or owner_lease.get("executor_progress_wall_time_s")
            or owner_lease.get("heartbeat_wall_time_s")
        )
        heartbeat_timeout_s = float(owner_lease.get("heartbeat_timeout_s"))
    except (TypeError, ValueError):
        return None
    now_s = time.time()
    heartbeat_age_s = max(0.0, now_s - heartbeat_wall_time_s)
    if heartbeat_age_s < heartbeat_timeout_s:
        return None
    return {
        "owner": owner,
        "landing_mode": MotionMode.DAMPING.value,
        "reason": "owner_heartbeat_timeout",
        "heartbeat_age_s": heartbeat_age_s,
        "heartbeat_timeout_s": heartbeat_timeout_s,
        "heartbeat_source": (
            "executor_progress"
            if (
                executor_progress_wall_time_s is not None
                or owner_lease.get("executor_progress_wall_time_s") is not None
            )
            else "owner_deadman"
        ),
    }


def _fault_session_for_owner_timeout(
    *,
    session: dict[str, object],
    watchdog: dict[str, object],
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    updated = dict(session)
    updated["status"] = "faulted"
    updated["mode"] = MotionMode.DAMPING.value
    updated["owner"] = None
    updated["owner_lease"] = None
    updated["watchdog"] = watchdog
    updated["heartbeat"] = {
        "wall_time_s": time.time(),
        "age_s": 0.0,
        "max_age_s": float(max_heartbeat_age_s),
        "fresh": True,
    }
    updated["readiness"] = runtime_readiness(updated)
    return updated


def _ensure_can_queue_command(
    session: dict[str, object],
    *,
    owner: str,
    expected_q_start: Sequence[float],
    start_pose_policy: str,
    max_start_error_rad: float,
    heartbeat_timeout_s: float,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    if session.get("owner") is not None:
        raise RuntimeSessionError(
            f"runtime is owned by {session.get('owner')}",
            session,
        )
    if session.get("mode") != "hold_safe":
        raise RuntimeSessionError(
            f"runtime must be hold_safe before command, got {session.get('mode')}",
            session,
        )
    if heartbeat_timeout_s <= 0.0:
        raise ValueError("heartbeat_timeout_s must be positive")
    if not owner:
        raise ValueError("owner must not be empty")
    freshness_guard = _live_hold_freshness_guard(
        session,
        max_age_s=float(max_heartbeat_age_s),
    )
    if freshness_guard["status"] != "pass":
        error_payload = dict(session)
        error_payload["freshness_guard"] = freshness_guard
        raise RuntimeSessionError("runtime live hold evidence is stale", error_payload)
    start_pose_guard = _start_pose_guard(
        session,
        expected_q_start=expected_q_start,
        start_pose_policy=start_pose_policy,
        max_start_error_rad=max_start_error_rad,
    )
    if start_pose_guard["status"] != "pass":
        error_payload = dict(session)
        error_payload["expected_q_start"] = list(expected_q_start)
        error_payload["max_start_error_rad"] = float(max_start_error_rad)
        error_payload["start_pose_policy"] = start_pose_guard["policy"]
        error_payload["start_pose_guard"] = start_pose_guard
        raise RuntimeSessionError(
            _start_pose_guard_error_message(start_pose_guard),
            error_payload,
        )
    return start_pose_guard


def _live_hold_freshness_guard(
    session: dict[str, object],
    *,
    max_age_s: float,
) -> dict[str, object]:
    heartbeat = session.get("heartbeat")
    heartbeat_wall_time_s = (
        heartbeat.get("wall_time_s") if isinstance(heartbeat, dict) else None
    )
    heartbeat_age_s = _wall_age_s(heartbeat_wall_time_s)
    hold_age_s = _wall_age_s(session.get("last_hold_wall_time_s"))
    failed_checks: list[str] = []
    if heartbeat_age_s is None or heartbeat_age_s > float(max_age_s):
        failed_checks.append("heartbeat_fresh")
    if session.get("hold_fresh") is not True:
        failed_checks.append("hold_fresh")
    if hold_age_s is None or hold_age_s > float(max_age_s):
        failed_checks.append("hold_tick_fresh")
    return {
        "status": "pass" if not failed_checks else "fail",
        "max_age_s": float(max_age_s),
        "heartbeat_age_s": heartbeat_age_s,
        "hold_age_s": hold_age_s,
        "failed_checks": failed_checks,
    }


def _wall_age_s(wall_time_s: object) -> float | None:
    try:
        timestamp = float(wall_time_s)
    except (TypeError, ValueError):
        return None
    return max(0.0, time.time() - timestamp)


def _start_pose_guard(
    session: dict[str, object],
    *,
    expected_q_start: Sequence[float],
    start_pose_policy: str,
    max_start_error_rad: float,
) -> dict[str, object]:
    policy = str(start_pose_policy)
    if policy not in START_POSE_POLICIES:
        raise ValueError(
            "start_pose_policy must be one of "
            + ", ".join(sorted(START_POSE_POLICIES))
        )
    q_hold = _optional_float_sequence(session.get("q_hold"))
    q_meas = _optional_float_sequence(session.get("q_meas"))
    safe_center = _optional_float_sequence(session.get("safe_center"))
    expected = [float(value) for value in expected_q_start]
    q_meas_to_q_hold = _max_abs_error(q_meas, q_hold)
    q_hold_to_expected = _max_abs_error(q_hold, expected)
    q_hold_to_safe_center = _max_abs_error(q_hold, safe_center)
    failed_checks: list[str] = []
    if (
        policy != "current_measured_pose"
        and (q_meas_to_q_hold is None or q_meas_to_q_hold > float(max_start_error_rad))
    ):
        failed_checks.append("q_meas_close_to_q_hold")
    if policy == "live_hold":
        if q_hold_to_expected is None or q_hold_to_expected > float(max_start_error_rad):
            failed_checks.append("q_hold_close_to_expected_start")
    elif policy == "safe_center":
        if (
            q_hold_to_safe_center is None
            or q_hold_to_safe_center > float(max_start_error_rad)
        ):
            failed_checks.append("q_hold_close_to_safe_center")
        if q_hold_to_expected is None or q_hold_to_expected > float(max_start_error_rad):
            failed_checks.append("expected_start_matches_current_q_hold")
    elif policy == "current_measured_pose":
        pass
    elif q_hold_to_expected is None or q_hold_to_expected > float(max_start_error_rad):
        failed_checks.append("q_hold_close_to_explicit_q")
    return {
        "status": "pass" if not failed_checks else "fail",
        "policy": policy,
        "expected_q_start": expected,
        "q_hold": q_hold,
        "q_meas": q_meas,
        "safe_center": safe_center,
        "max_start_error_rad": float(max_start_error_rad),
        "q_meas_to_q_hold_max_abs_rad": q_meas_to_q_hold,
        "q_hold_to_expected_start_max_abs_rad": q_hold_to_expected,
        "q_hold_to_safe_center_max_abs_rad": q_hold_to_safe_center,
        "failed_checks": failed_checks,
    }


def _start_pose_guard_error_message(start_pose_guard: dict[str, object]) -> str:
    policy = start_pose_guard.get("policy")
    failed_checks = start_pose_guard.get("failed_checks")
    if policy == "explicit_q" and isinstance(failed_checks, list):
        if "q_hold_close_to_explicit_q" in failed_checks:
            return "explicit_q start pose requires runtime q_hold to be pre-positioned"
    if policy == "safe_center" and isinstance(failed_checks, list):
        if "q_hold_close_to_safe_center" in failed_checks:
            return "safe_center start pose requires runtime q_hold at SAFE_CENTER"
    if isinstance(failed_checks, list) and "q_meas_close_to_q_hold" in failed_checks:
        return "current q_meas is not close to runtime q_hold"
    return "runtime start pose guard failed"


def _optional_float_sequence(values: object) -> list[float] | None:
    if not isinstance(values, (list, tuple)):
        return None
    return [float(value) for value in values]


def _max_abs_error(left: Sequence[float] | None, right: Sequence[float] | None) -> float | None:
    if left is None or right is None or len(left) != len(right):
        return None
    return max(
        (
            abs(float(left_value) - float(right_value))
            for left_value, right_value in zip(left, right, strict=True)
        ),
        default=0.0,
    )


def _error_start_pose_guard(error: Exception) -> object:
    if isinstance(error, RuntimeSessionError):
        return error.payload.get("start_pose_guard")
    return None


def _q_close(left: object, right: Sequence[float], max_error_rad: float) -> bool:
    if not isinstance(left, (list, tuple)):
        return False
    if len(left) != len(right):
        return False
    return all(
        abs(float(left_value) - float(right_value)) <= float(max_error_rad)
        for left_value, right_value in zip(left, right, strict=True)
    )


def _float_list(values: Sequence[float], *, name: str) -> list[float]:
    result = [float(value) for value in values]
    if not result:
        raise ValueError(f"{name} must not be empty")
    return result


def _fixed_float_list(
    values: Sequence[float],
    *,
    name: str,
    length: int,
) -> list[float]:
    result = _float_list(values, name=name)
    if len(result) != length:
        raise ValueError(f"{name} must contain exactly {length} values")
    return result


def _object_float_list(values: object) -> list[float]:
    if not isinstance(values, (list, tuple)):
        raise ValueError("expected a numeric list")
    return [float(value) for value in values]


def _write_optional_output(
    payload: dict[str, object],
    output_path: Path | None,
) -> dict[str, object]:
    if output_path is None:
        return payload
    output_path.parent.mkdir(parents=True, exist_ok=True)
    _write_json_atomic(output_path, payload)
    return payload


def _write_json_atomic(path: Path, payload: dict[str, object]) -> None:
    tmp_path = path.with_name(f"{path.name}.{uuid4().hex}.tmp")
    tmp_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    tmp_path.replace(path)


def _read_json_object(path: Path) -> dict[str, object]:
    last_error: Exception | None = None
    for _ in range(50):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError(f"{path} does not contain a JSON object")
            return payload
        except (FileNotFoundError, json.JSONDecodeError) as error:
            last_error = error
            time.sleep(0.01)
    if last_error is not None:
        raise last_error
    raise FileNotFoundError(path)
