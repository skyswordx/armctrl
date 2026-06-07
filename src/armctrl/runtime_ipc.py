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


def submit_trajectory_command(
    *,
    session_artifact_path: Path,
    owner: str,
    expected_q_start: Sequence[float],
    q_points: Sequence[Sequence[float]],
    send_hz: float,
    trajectory_sample_hz: float | None = None,
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
    if trajectory_sample_hz is None:
        trajectory_sample_hz = float(send_hz)
    if trajectory_sample_hz <= 0.0:
        raise ValueError("trajectory_sample_hz must be positive")

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
        "trajectory_sample_hz": float(trajectory_sample_hz),
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
    runtime: ArmRuntime | None = None,
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
    command["dequeued_wall_time_s"] = time.time()
    _write_json_atomic(running_path, command)
    result = execute_runtime_command(
        command=command,
        session_artifact_path=session_artifact_path,
        backend=backend,
        runtime=runtime,
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
            max_start_error_rad=float(command.get("max_start_error_rad", 0.02)),
            heartbeat_timeout_s=float(command.get("heartbeat_timeout_s", 0.5)),
        )
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
        lease = live_runtime.acquire_owner(
            owner=owner,
            mode=MotionMode(mode),
            expected_q_start=tuple(_object_float_list(command.get("expected_q_start"))),
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
        progress = _active_owner_session_progress_throttle(
            session_artifact_path=session_artifact_path,
            runtime=live_runtime,
            owner=owner,
            mode=mode,
            heartbeat_timeout_s=float(command.get("heartbeat_timeout_s", 0.5)),
            max_heartbeat_age_s=max_heartbeat_age_s,
            min_period_s=min(
                0.2,
                max(0.001, float(command.get("heartbeat_timeout_s", 0.5)) / 2.0),
            ),
        )
        first_send_wall_time_s: float | None = None

        def on_motion_sample(sample: MotionAuditSample) -> None:
            nonlocal first_send_wall_time_s
            if first_send_wall_time_s is None:
                first_send_wall_time_s = time.time()
            progress(sample)

        motion = _execute_motion_command(
            command,
            runtime=live_runtime,
            watchdog=lambda: _owner_timeout_watchdog(
                session_artifact_path=session_artifact_path,
                owner=owner,
            ),
            on_sample=on_motion_sample,
        )
        watchdog = _owner_timeout_watchdog(
            session_artifact_path=session_artifact_path,
            owner=owner,
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
            )
        if motion.status == "completed":
            status = live_runtime.status()
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
            runtime_session_id=session.get("runtime_session_id"),
            session_artifact_path=session_artifact_path,
            status=motion.status,
            landing_mode=motion.landing_mode,
            motion=motion,
            reason=None,
            owner_acquire_started_wall_time_s=owner_acquire_started_wall_time_s,
            owner_acquired_wall_time_s=owner_acquired_wall_time_s,
            first_send_wall_time_s=first_send_wall_time_s,
        )
    except (ArmRuntimeError, RuntimeSessionError, ValueError) as error:
        rejected_wall_time_s = time.time()
        return {
            "status": "rejected",
            "schema": RUNTIME_COMMAND_RESULT_SCHEMA,
            "command_id": command.get("command_id"),
            "owner": owner,
            "mode": mode,
            "movement_command_sent": False,
            "reason": str(error),
            "landing_mode": None,
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


def _execute_motion_command(
    command: dict[str, object],
    *,
    runtime: ArmRuntime,
    watchdog=None,
    on_sample=None,
) -> MotionExecutionResult:
    kind = command.get("kind")
    send_hz = float(command.get("send_hz", 50.0))
    if kind == "trajectory":
        q_points = command.get("q_points")
        if not isinstance(q_points, list):
            raise ValueError("trajectory command requires q_points")
        trajectory_sample_hz = float(command.get("trajectory_sample_hz", send_hz))
        raw_points = [tuple(_object_float_list(q_point)) for q_point in q_points]
        points = _runtime_trajectory_points(
            raw_points,
            trajectory_sample_hz=trajectory_sample_hz,
            runtime_send_hz=send_hz,
        )
        return runtime.execute_owner_trajectory(
            points,
            owner=str(command.get("owner")),
            trajectory_sample_hz=send_hz,
            watchdog=watchdog,
            on_sample=on_sample,
        )
    if kind == "intent":
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
            ),
            owner=str(command.get("owner")),
            send_hz=send_hz,
            watchdog=watchdog,
            on_sample=on_sample,
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


def _active_owner_session_progress_throttle(
    *,
    session_artifact_path: Path,
    runtime: ArmRuntime,
    owner: str,
    mode: str,
    heartbeat_timeout_s: float,
    max_heartbeat_age_s: float,
    min_period_s: float,
):
    last_write_monotonic_s: float | None = None

    def on_sample(sample: MotionAuditSample) -> None:
        nonlocal last_write_monotonic_s
        if (
            last_write_monotonic_s is not None
            and sample.sent_monotonic_s - last_write_monotonic_s < min_period_s
        ):
            return
        last_write_monotonic_s = sample.sent_monotonic_s
        _refresh_active_owner_session(
            session_artifact_path=session_artifact_path,
            runtime=runtime,
            owner=owner,
            mode=mode,
            heartbeat_timeout_s=heartbeat_timeout_s,
            max_heartbeat_age_s=max_heartbeat_age_s,
        )

    return on_sample


def _refresh_active_owner_session(
    *,
    session_artifact_path: Path,
    runtime: ArmRuntime,
    owner: str,
    mode: str,
    heartbeat_timeout_s: float,
    max_heartbeat_age_s: float,
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
        "heartbeat_wall_time_s": time.time(),
        "acquired_wall_time_s": acquired_wall_time_s,
        "landing_policy": "watchdog_to_damping_release_to_hold_safe",
    }
    updated["owner_deadman"] = {
        "owner": owner,
        "mode": mode,
        "heartbeat_wall_time_s": updated["owner_lease"]["heartbeat_wall_time_s"],
        "heartbeat_age_s": 0.0,
        "heartbeat_timeout_s": float(heartbeat_timeout_s),
        "fresh": True,
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
    payload = {
        "status": status,
        "schema": RUNTIME_COMMAND_RESULT_SCHEMA,
        "command_id": command.get("command_id"),
        "owner": command.get("owner"),
        "mode": command.get("mode"),
        "runtime_session_id": runtime_session_id,
        "movement_command_sent": bool(motion.samples),
        "landing_mode": landing_mode,
        "reason": reason,
        "timing": timing,
        "artifacts": {
            "session": str(session_artifact_path),
            "result": str(command.get("result_artifact") or ""),
        },
        "motion": {
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
            "send_period_s": send_period,
            "dt_min_s": send_period["min"],
            "dt_max_s": send_period["max"],
            "dt_avg_s": send_period["avg"],
            "controller_dt_s": motion.controller_dt_s,
            "sample_count": len(motion.samples),
            "samples": [_motion_sample_manifest(sample) for sample in motion.samples],
            "landing_mode": motion.landing_mode,
            "error": motion.error,
        },
    }
    if watchdog is not None:
        payload["watchdog"] = watchdog
    return payload


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


def _runtime_trajectory_points(
    q_points: Sequence[tuple[float, ...]],
    *,
    trajectory_sample_hz: float,
    runtime_send_hz: float,
) -> list[JointTrajectoryPoint]:
    if not q_points:
        raise ValueError("trajectory command requires q_points")
    if trajectory_sample_hz <= 0.0:
        raise ValueError("trajectory_sample_hz must be positive")
    if runtime_send_hz <= 0.0:
        raise ValueError("send_hz must be positive")
    if len(q_points) == 1:
        return [JointTrajectoryPoint(time_s=0.0, q=tuple(q_points[0]))]
    duration_s = (len(q_points) - 1) / float(trajectory_sample_hz)
    sample_count = int(round(duration_s * float(runtime_send_hz))) + 1
    sample_count = max(2, sample_count)
    return [
        JointTrajectoryPoint(
            time_s=index / float(runtime_send_hz),
            q=_sample_linear_joint_trajectory(
                q_points,
                trajectory_sample_hz=trajectory_sample_hz,
                sample_time_s=min(duration_s, index / float(runtime_send_hz)),
            ),
        )
        for index in range(sample_count)
    ]


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


def _command_runtime_send_hz(
    command: dict[str, object],
    motion: MotionExecutionResult,
) -> float | None:
    return _optional_float(command.get("send_hz")) or motion.trajectory_sample_hz


def _command_trajectory_sample_hz(
    command: dict[str, object],
    motion: MotionExecutionResult,
) -> float | None:
    if command.get("kind") != "trajectory":
        return motion.trajectory_sample_hz
    return _optional_float(command.get("trajectory_sample_hz")) or motion.trajectory_sample_hz


def _trajectory_resampling_policy(
    *,
    trajectory_sample_hz: float | None,
    runtime_send_hz: float | None,
    kind: str,
) -> str:
    if kind != "trajectory":
        return "intent_frame_to_runtime_send_hz"
    if (
        trajectory_sample_hz is not None
        and runtime_send_hz is not None
        and abs(trajectory_sample_hz - runtime_send_hz) <= 1e-9
    ):
        return "none_sample_hz_matches_send_hz"
    return "linear_time_resample"


def _trajectory_interpolation_policy(*, resampling_policy: str, kind: str) -> str:
    if kind != "trajectory":
        return "linear_intent_frame"
    if resampling_policy == "linear_time_resample":
        return "linear_joint_position"
    return "pre_sampled_joint_positions"


def _optional_float(value: object) -> float | None:
    try:
        return float(value)
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
) -> dict[str, object] | None:
    session = _read_json_object(session_artifact_path)
    owner_lease = session.get("owner_lease")
    if session.get("owner") != owner or not isinstance(owner_lease, dict):
        return None
    try:
        heartbeat_wall_time_s = float(owner_lease.get("heartbeat_wall_time_s"))
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
        error_payload = dict(session)
        error_payload["expected_q_start"] = list(expected_q_start)
        error_payload["max_start_error_rad"] = float(max_start_error_rad)
        if isinstance(session.get("q_meas"), (list, tuple)):
            q_meas = [float(value) for value in session["q_meas"]]
            error_payload["q_start_error_rad"] = [
                measured - expected
                for measured, expected in zip(q_meas, expected_q_start, strict=True)
            ]
            error_payload["q_start_error_max_abs_rad"] = max(
                (abs(value) for value in error_payload["q_start_error_rad"]),
                default=0.0,
            )
        raise RuntimeSessionError(
            "current q_meas is not close to expected start pose",
            error_payload,
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
