"""Live ArmRuntime session artifact helpers.

This module is intentionally small: it defines the CLI/session contract that
Agent, SysID, Recipe, and future teleop owners attach to. The fake backend path
is a contract test surface; real ARX5 daemon ownership can reuse the same status
shape without changing callers.
"""

from __future__ import annotations

from pathlib import Path
import json
import time
from typing import Sequence

from armctrl.motion_runtime import (
    ArmRuntime,
    ArmRuntimeMode,
    FakeMotionBackend,
    MotionMode,
)

RUNTIME_SESSION_SCHEMA = "armctrl.arm_runtime_session.v1"
RUNTIME_STATUS_SCHEMA = "armctrl.arm_runtime_status.v1"


def start_fake_runtime_session(
    *,
    q_current: Sequence[float],
    safe_center: Sequence[float],
    send_hz: float,
    hold_hz: float,
    max_joint_step_rad: float,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    q_current_tuple = _float_tuple(q_current, name="q_current")
    safe_center_tuple = _float_tuple(safe_center, name="safe_center")
    if len(q_current_tuple) != len(safe_center_tuple):
        raise ValueError("q_current and safe_center lengths must match")
    if send_hz <= 0.0:
        raise ValueError("send_hz must be positive")
    if hold_hz <= 0.0:
        raise ValueError("hold_hz must be positive")
    if max_joint_step_rad <= 0.0:
        raise ValueError("max_joint_step_rad must be positive")

    backend = FakeMotionBackend()
    backend.send_joint_command(
        q_current_tuple,
        producer="runtime_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=safe_center_tuple,
    )
    recovery = runtime.recover_to_safe(
        send_hz=float(send_hz),
        max_joint_step_rad=float(max_joint_step_rad),
    )
    status = runtime.status()
    return runtime_status_payload(
        status=status,
        schema=RUNTIME_SESSION_SCHEMA,
        backend="fake",
        send_hz=float(send_hz),
        hold_hz=float(hold_hz),
        max_heartbeat_age_s=float(max_heartbeat_age_s),
        recovery={
            "status": recovery.status,
            "producer": recovery.producer,
            "mode": recovery.mode,
            "trajectory_sample_hz": recovery.trajectory_sample_hz,
            "actual_send_hz": recovery.actual_send_hz,
            "send_jitter_ms_p95": recovery.send_jitter_ms_p95,
            "send_jitter_ms_p99": recovery.send_jitter_ms_p99,
            "controller_dt_s": recovery.controller_dt_s,
            "sample_count": len(recovery.samples),
            "landing_mode": recovery.landing_mode,
        },
    )


def runtime_status_from_artifact(
    *,
    session_artifact_path: Path,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    payload = _read_json_object(session_artifact_path)
    if payload.get("schema") not in {RUNTIME_SESSION_SCHEMA, RUNTIME_STATUS_SCHEMA}:
        raise ValueError("invalid runtime session/status schema")
    return refresh_runtime_status_payload(
        payload,
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )


def refresh_runtime_status_payload(
    payload: dict[str, object],
    *,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    refreshed = dict(payload)
    refreshed["schema"] = RUNTIME_STATUS_SCHEMA
    refreshed["status"] = "ok"
    heartbeat = _heartbeat_status(
        refreshed.get("heartbeat"),
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    refreshed["heartbeat"] = heartbeat
    readiness = runtime_readiness(refreshed)
    refreshed["readiness"] = readiness
    if readiness["agent_sysid_smoke_allowed"] is not True:
        refreshed["status"] = "blocked"
    return refreshed


def runtime_readiness(payload: dict[str, object]) -> dict[str, object]:
    heartbeat = payload.get("heartbeat")
    failed_checks: list[str] = []
    if payload.get("schema") not in {RUNTIME_SESSION_SCHEMA, RUNTIME_STATUS_SCHEMA}:
        failed_checks.append("schema")
    if payload.get("mode") != ArmRuntimeMode.HOLD_SAFE.value:
        failed_checks.append("mode_hold_safe")
    if payload.get("owner") is not None:
        failed_checks.append("no_owner")
    if not _q_close(payload.get("q_meas"), payload.get("q_hold"), max_error_rad=0.02):
        failed_checks.append("q_meas_close_to_hold")
    if not _q_close(payload.get("q_hold"), payload.get("safe_center"), max_error_rad=0.02):
        failed_checks.append("q_hold_close_to_safe_center")
    if not isinstance(heartbeat, dict) or heartbeat.get("fresh") is not True:
        failed_checks.append("heartbeat_fresh")
    if payload.get("fault_flags") not in ([], ()):
        failed_checks.append("no_fault_flags")
    return {
        "agent_sysid_smoke_allowed": not failed_checks,
        "failed_checks": failed_checks,
    }


def runtime_status_prerequisite_status(payload: dict[str, object]) -> str:
    return "pass" if runtime_readiness(payload)["agent_sysid_smoke_allowed"] is True else "fail"


def runtime_status_summary(payload: dict[str, object]) -> dict[str, object]:
    heartbeat = payload.get("heartbeat")
    readiness = payload.get("readiness")
    return {
        "schema": payload.get("schema"),
        "runtime_session_id": payload.get("runtime_session_id"),
        "mode": payload.get("mode"),
        "owner": payload.get("owner"),
        "q_meas": payload.get("q_meas"),
        "q_hold": payload.get("q_hold"),
        "safe_center": payload.get("safe_center"),
        "controller_dt_s": payload.get("controller_dt_s"),
        "send_hz": payload.get("send_hz"),
        "hold_hz": payload.get("hold_hz"),
        "fault_flags": payload.get("fault_flags"),
        "heartbeat": heartbeat if isinstance(heartbeat, dict) else None,
        "readiness": readiness if isinstance(readiness, dict) else runtime_readiness(payload),
    }


def runtime_status_payload(
    *,
    status: dict[str, object],
    schema: str,
    backend: str,
    send_hz: float,
    hold_hz: float,
    max_heartbeat_age_s: float,
    recovery: dict[str, object] | None = None,
) -> dict[str, object]:
    heartbeat = _heartbeat_status(
        {
            "wall_time_s": time.time(),
            "max_age_s": float(max_heartbeat_age_s),
        },
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    payload = {
        "status": "ok",
        "schema": schema,
        "backend": backend,
        "runtime_session_id": status.get("runtime_session_id"),
        "mode": status.get("mode"),
        "owner": status.get("owner"),
        "q_meas": list(status.get("q_meas") or []),
        "q_hold": list(status.get("q_hold") or []),
        "safe_center": list(status.get("safe_center") or []),
        "controller_dt_s": status.get("controller_dt_s"),
        "send_hz": float(send_hz),
        "hold_hz": float(hold_hz),
        "fault_flags": list(status.get("fault_flags") or []),
        "heartbeat": heartbeat,
        "runtime_contract": {
            "state_machine": "passive_safe->recover_to_safe->hold_safe->owner_lease",
            "single_motion_owner": True,
            "readiness_requires_live_hold": True,
        },
    }
    if recovery is not None:
        payload["recovery"] = recovery
    payload["readiness"] = runtime_readiness(payload)
    if payload["readiness"]["agent_sysid_smoke_allowed"] is not True:
        payload["status"] = "blocked"
    return payload


def _heartbeat_status(
    heartbeat: object,
    *,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    wall_time_s = None
    if isinstance(heartbeat, dict):
        try:
            wall_time_s = float(heartbeat.get("wall_time_s"))
        except (TypeError, ValueError):
            wall_time_s = None
    now_s = time.time()
    age_s = None if wall_time_s is None else max(0.0, now_s - wall_time_s)
    return {
        "wall_time_s": wall_time_s,
        "age_s": age_s,
        "max_age_s": float(max_heartbeat_age_s),
        "fresh": age_s is not None and age_s <= float(max_heartbeat_age_s),
    }


def _q_close(left: object, right: object, *, max_error_rad: float) -> bool:
    if not isinstance(left, (list, tuple)) or not isinstance(right, (list, tuple)):
        return False
    if len(left) != len(right):
        return False
    try:
        return all(
            abs(float(left_value) - float(right_value)) <= max_error_rad
            for left_value, right_value in zip(left, right, strict=True)
        )
    except (TypeError, ValueError):
        return False


def _float_tuple(values: Sequence[float], *, name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not result:
        raise ValueError(f"{name} must not be empty")
    return result


def _read_json_object(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} does not contain a JSON object")
    return payload
