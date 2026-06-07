"""File-backed command queue for a long-lived arm runtime process."""

from __future__ import annotations

from pathlib import Path
import json
import time
from uuid import uuid4
from typing import Sequence

from armctrl.motion_runtime import (
    ArmRuntime,
    ArmRuntimeError,
    JointIntentFrame,
    JointTrajectoryPoint,
    MotionBackend,
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


def submit_trajectory_command(
    *,
    session_artifact_path: Path,
    owner: str,
    expected_q_start: Sequence[float],
    q_points: Sequence[Sequence[float]],
    send_hz: float,
    max_start_error_rad: float,
    heartbeat_timeout_s: float,
    max_heartbeat_age_s: float,
    output_path: Path | None = None,
) -> dict[str, object]:
    if not q_points:
        raise ValueError("at least one --q-point is required")
    q_start = _float_list(expected_q_start, name="expected_q_start")
    points = [_float_list(point, name="q_point") for point in q_points]
    for point in points:
        if len(point) != len(q_start):
            raise ValueError("all q points must match expected_q_start length")
    if send_hz <= 0.0:
        raise ValueError("send_hz must be positive")

    session = _read_json_object(session_artifact_path)
    _ensure_can_queue_command(
        session,
        owner=owner,
        expected_q_start=q_start,
        max_start_error_rad=float(max_start_error_rad),
        heartbeat_timeout_s=float(heartbeat_timeout_s),
    )
    command_id = str(uuid4())
    queue_dir = runtime_command_queue_dir(session_artifact_path)
    pending_path = queue_dir / "pending" / f"{command_id}.json"
    result_path = queue_dir / "results" / f"{command_id}.json"
    command = {
        "schema": RUNTIME_COMMAND_SCHEMA,
        "command_id": command_id,
        "kind": "trajectory",
        "owner": str(owner),
        "mode": MotionMode.TRAJECTORY_REPLAY.value,
        "expected_q_start": q_start,
        "max_start_error_rad": float(max_start_error_rad),
        "heartbeat_timeout_s": float(heartbeat_timeout_s),
        "send_hz": float(send_hz),
        "q_points": points,
        "submitted_wall_time_s": time.time(),
        "session_artifact": str(session_artifact_path),
        "result_artifact": str(result_path),
    }
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
) -> dict[str, object]:
    q_start = _float_list(expected_q_start, name="expected_q_start")
    target = _float_list(q_target, name="q_target")
    if len(target) != len(q_start):
        raise ValueError("q_target must match expected_q_start length")
    if control_period_s <= 0.0:
        raise ValueError("control_period_s must be positive")
    if send_hz <= 0.0:
        raise ValueError("send_hz must be positive")
    session = _read_json_object(session_artifact_path)
    _ensure_can_queue_command(
        session,
        owner=owner,
        expected_q_start=q_start,
        max_start_error_rad=float(max_start_error_rad),
        heartbeat_timeout_s=float(heartbeat_timeout_s),
    )
    command_id = str(uuid4())
    queue_dir = runtime_command_queue_dir(session_artifact_path)
    pending_path = queue_dir / "pending" / f"{command_id}.json"
    result_path = queue_dir / "results" / f"{command_id}.json"
    command = {
        "schema": RUNTIME_COMMAND_SCHEMA,
        "command_id": command_id,
        "kind": "intent",
        "owner": str(owner),
        "mode": MotionMode.AGENT_SERVO.value,
        "expected_q_start": q_start,
        "q_target": target,
        "control_period_s": float(control_period_s),
        "max_joint_delta_rad": (
            None if max_joint_delta_rad is None else float(max_joint_delta_rad)
        ),
        "max_start_error_rad": float(max_start_error_rad),
        "heartbeat_timeout_s": float(heartbeat_timeout_s),
        "send_hz": float(send_hz),
        "submitted_wall_time_s": time.time(),
        "session_artifact": str(session_artifact_path),
        "result_artifact": str(result_path),
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
        "runtime_session_id": session.get("runtime_session_id"),
        "movement_command_sent": False,
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


def runtime_command_queue_dir(session_artifact_path: Path) -> Path:
    return session_artifact_path.parent / f"{session_artifact_path.stem}_commands"


def execute_pending_runtime_commands(
    *,
    session_artifact_path: Path,
    backend: MotionBackend,
    max_heartbeat_age_s: float,
) -> dict[str, object] | None:
    queue_dir = runtime_command_queue_dir(session_artifact_path)
    pending_dir = queue_dir / "pending"
    if not pending_dir.exists():
        return None
    command_paths = sorted(pending_dir.glob("*.json"))
    if not command_paths:
        return None
    command_path = command_paths[0]
    running_path = queue_dir / "running" / command_path.name
    running_path.parent.mkdir(parents=True, exist_ok=True)
    command_path.replace(running_path)
    command = _read_json_object(running_path)
    result = execute_runtime_command(
        command=command,
        session_artifact_path=session_artifact_path,
        backend=backend,
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
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    if command.get("schema") != RUNTIME_COMMAND_SCHEMA:
        raise ValueError("invalid runtime command schema")
    session = _read_json_object(session_artifact_path)
    owner = str(command.get("owner"))
    mode = str(command.get("mode"))
    try:
        _ensure_can_queue_command(
            session,
            owner=owner,
            expected_q_start=_object_float_list(command.get("expected_q_start")),
            max_start_error_rad=float(command.get("max_start_error_rad", 0.02)),
            heartbeat_timeout_s=float(command.get("heartbeat_timeout_s", 0.5)),
        )
        runtime = ArmRuntime(
            backend=backend,
            safe_center=tuple(_object_float_list(session.get("safe_center"))),
            runtime_session_id=str(session.get("runtime_session_id")),
        )
        runtime.mark_hold_safe(q_hold=tuple(_object_float_list(session.get("q_hold"))))
        lease = runtime.acquire_owner(
            owner=owner,
            mode=MotionMode(mode),
            expected_q_start=tuple(_object_float_list(command.get("expected_q_start"))),
            max_start_error_rad=float(command.get("max_start_error_rad", 0.02)),
            heartbeat_timeout_s=float(command.get("heartbeat_timeout_s", 0.5)),
        )
        motion = _execute_motion_command(command, backend=backend)
        if motion.status == "completed":
            lease.release()
            status = runtime.status()
            session = _session_from_runtime_status(
                session,
                status=status,
                max_heartbeat_age_s=max_heartbeat_age_s,
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
            status=motion.status,
            landing_mode=motion.landing_mode,
            motion=motion,
            reason=None,
        )
    except (ArmRuntimeError, RuntimeSessionError, ValueError) as error:
        return {
            "status": "rejected",
            "schema": RUNTIME_COMMAND_RESULT_SCHEMA,
            "command_id": command.get("command_id"),
            "owner": owner,
            "mode": mode,
            "movement_command_sent": False,
            "reason": str(error),
            "landing_mode": None,
        }


def _execute_motion_command(
    command: dict[str, object],
    *,
    backend: MotionBackend,
) -> MotionExecutionResult:
    motion = MotionRuntime(backend=backend)
    kind = command.get("kind")
    send_hz = float(command.get("send_hz", 50.0))
    if kind == "trajectory":
        q_points = command.get("q_points")
        if not isinstance(q_points, list):
            raise ValueError("trajectory command requires q_points")
        points = [
            JointTrajectoryPoint(
                time_s=index / send_hz,
                q=tuple(_object_float_list(q_point)),
            )
            for index, q_point in enumerate(q_points)
        ]
        return motion.execute_trajectory(
            points,
            producer=str(command.get("owner")),
            trajectory_sample_hz=send_hz,
            hold_after=True,
        )
    if kind == "intent":
        return motion.execute_intent_frame(
            JointIntentFrame(
                q_start=tuple(_object_float_list(command.get("expected_q_start"))),
                q_target=tuple(_object_float_list(command.get("q_target"))),
                control_period_s=float(command.get("control_period_s")),
                max_joint_delta_rad=(
                    None
                    if command.get("max_joint_delta_rad") is None
                    else float(command.get("max_joint_delta_rad"))
                ),
            ),
            producer=str(command.get("owner")),
            send_hz=send_hz,
            hold_after=True,
        )
    raise ValueError(f"unsupported runtime command kind: {kind}")


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
    updated["q_meas"] = list(status.get("q_meas") or [])
    updated["q_hold"] = list(status.get("q_hold") or [])
    updated["fault_flags"] = list(status.get("fault_flags") or [])
    updated = heartbeat_runtime_session_payload(
        updated,
        max_heartbeat_age_s=max_heartbeat_age_s,
    )
    return updated


def _command_result_payload(
    *,
    command: dict[str, object],
    status: str,
    landing_mode: str,
    motion: MotionExecutionResult,
    reason: str | None,
) -> dict[str, object]:
    return {
        "status": status,
        "schema": RUNTIME_COMMAND_RESULT_SCHEMA,
        "command_id": command.get("command_id"),
        "owner": command.get("owner"),
        "mode": command.get("mode"),
        "movement_command_sent": bool(motion.samples),
        "landing_mode": landing_mode,
        "reason": reason,
        "motion": {
            "status": motion.status,
            "producer": motion.producer,
            "mode": motion.mode,
            "trajectory_sample_hz": motion.trajectory_sample_hz,
            "actual_send_hz": motion.actual_send_hz,
            "send_jitter_ms_p95": motion.send_jitter_ms_p95,
            "send_jitter_ms_p99": motion.send_jitter_ms_p99,
            "controller_dt_s": motion.controller_dt_s,
            "sample_count": len(motion.samples),
            "landing_mode": motion.landing_mode,
            "error": motion.error,
        },
    }


def _ensure_can_queue_command(
    session: dict[str, object],
    *,
    owner: str,
    expected_q_start: Sequence[float],
    max_start_error_rad: float,
    heartbeat_timeout_s: float,
) -> None:
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
    if not _q_close(session.get("q_meas"), expected_q_start, max_start_error_rad):
        raise RuntimeSessionError(
            "current q_meas is not close to expected start pose",
            session,
        )
    if not owner:
        raise ValueError("owner must not be empty")


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
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} does not contain a JSON object")
    return payload
