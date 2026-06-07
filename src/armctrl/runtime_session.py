"""Live ArmRuntime session artifact helpers.

This module is intentionally small: it defines the CLI/session contract that
Agent, SysID, Recipe, and future teleop owners attach to. The fake backend path
is a contract test surface; real ARX5 daemon ownership can reuse the same status
shape without changing callers.
"""

from __future__ import annotations

from pathlib import Path
import importlib.util
import json
import time
from typing import Sequence

from armctrl.motion_runtime import (
    ArmRuntime,
    ArmRuntimeMode,
    FakeMotionBackend,
    MotionBackend,
    MotionMode,
)

RUNTIME_SESSION_SCHEMA = "armctrl.arm_runtime_session.v1"
RUNTIME_STATUS_SCHEMA = "armctrl.arm_runtime_status.v1"
ARX5_RUNTIME_START_CONFIRMATION = (
    "I UNDERSTAND THIS WILL START THE REAL ARM RUNTIME"
)


class RuntimeSessionError(RuntimeError):
    def __init__(self, message: str, payload: dict[str, object]) -> None:
        self.payload = payload
        super().__init__(message)


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


def start_arx5_runtime_session(
    *,
    backend: MotionBackend,
    model: str,
    interface: str,
    safe_center: Sequence[float],
    send_hz: float,
    hold_hz: float,
    max_joint_step_rad: float,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    enter = getattr(backend, "enter_hold_or_damping", None)
    if callable(enter):
        enter()
    runtime = ArmRuntime(
        backend=backend,
        safe_center=_float_tuple(safe_center, name="safe_center"),
    )
    recovery = runtime.recover_to_safe(
        send_hz=float(send_hz),
        max_joint_step_rad=float(max_joint_step_rad),
    )
    status = runtime.status()
    payload = runtime_status_payload(
        status=status,
        schema=RUNTIME_SESSION_SCHEMA,
        backend="arx5_sdk",
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
    payload.update(
        {
            "model": model,
            "interface": interface,
            "hardware_motion": True,
            "sdk_opened": True,
            "movement_command_sent": bool(recovery.samples),
            "requires_confirm": ARX5_RUNTIME_START_CONFIRMATION,
            "fault_landing_mode": MotionMode.DAMPING.value,
        }
    )
    return payload


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


def recover_runtime_session_from_artifact(
    *,
    session_artifact_path: Path,
    safe_center: Sequence[float],
    send_hz: float,
    hold_hz: float,
    max_joint_step_rad: float,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    payload = _read_json_object(session_artifact_path)
    if payload.get("schema") not in {RUNTIME_SESSION_SCHEMA, RUNTIME_STATUS_SCHEMA}:
        raise ValueError("invalid runtime session/status schema")
    if payload.get("owner") is not None:
        raise RuntimeSessionError(
            f"runtime is owned by {payload.get('owner')}",
            payload,
        )
    q_current = payload.get("q_meas")
    if not isinstance(q_current, (list, tuple)):
        raise RuntimeSessionError("runtime session has no measured q_meas", payload)
    recovered = start_fake_runtime_session(
        q_current=q_current,
        safe_center=safe_center,
        send_hz=send_hz,
        hold_hz=hold_hz,
        max_joint_step_rad=max_joint_step_rad,
        max_heartbeat_age_s=max_heartbeat_age_s,
    )
    recovered["runtime_session_id"] = payload.get(
        "runtime_session_id",
        recovered.get("runtime_session_id"),
    )
    recovered["lifecycle"] = {
        "action": "recover",
        "from_mode": payload.get("mode"),
        "to_mode": ArmRuntimeMode.HOLD_SAFE.value,
    }
    recovered["readiness"] = runtime_readiness(recovered)
    if recovered["readiness"]["agent_sysid_smoke_allowed"] is not True:
        recovered["status"] = "blocked"
    return recovered


def stop_runtime_session_from_artifact(
    *,
    session_artifact_path: Path,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    payload = _read_json_object(session_artifact_path)
    if payload.get("schema") not in {RUNTIME_SESSION_SCHEMA, RUNTIME_STATUS_SCHEMA}:
        raise ValueError("invalid runtime session/status schema")
    now_s = time.time()
    stopped = dict(payload)
    stopped["status"] = "stopped"
    stopped["schema"] = RUNTIME_SESSION_SCHEMA
    stopped["mode"] = MotionMode.DAMPING.value
    stopped["owner"] = None
    stopped["owner_lease"] = None
    stopped["owner_deadman"] = None
    stopped["landing_mode"] = MotionMode.DAMPING.value
    stopped["heartbeat"] = _heartbeat_status(
        {"wall_time_s": now_s},
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    stopped["lifecycle"] = {
        "action": "stop",
        "from_mode": payload.get("mode"),
        "to_mode": MotionMode.DAMPING.value,
    }
    stopped["readiness"] = runtime_readiness(stopped)
    return stopped


def heartbeat_runtime_session_payload(
    payload: dict[str, object],
    *,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    refreshed = dict(payload)
    refreshed["heartbeat"] = _heartbeat_status(
        {"wall_time_s": time.time()},
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    refreshed["readiness"] = runtime_readiness(refreshed)
    if refreshed.get("status") not in {"stopped", "faulted"}:
        refreshed["status"] = (
            "ok"
            if refreshed["readiness"]["agent_sysid_smoke_allowed"] is True
            else "blocked"
        )
    return refreshed


def record_runtime_hold_tick(
    payload: dict[str, object],
    *,
    q_meas: Sequence[float] | None,
    fault_flags: Sequence[str] | None,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    """Refresh live hold evidence written by the serving runtime process."""
    updated = dict(payload)
    now_s = time.time()
    if q_meas is not None:
        updated["q_meas"] = list(_float_tuple(q_meas, name="q_meas"))
    if fault_flags is not None:
        updated["fault_flags"] = [str(flag) for flag in fault_flags]
    updated["hold_tick_count"] = int(updated.get("hold_tick_count") or 0) + 1
    updated["last_hold_wall_time_s"] = now_s
    updated["hold_fresh"] = True
    updated["heartbeat"] = _heartbeat_status(
        {"wall_time_s": now_s},
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    updated["readiness"] = runtime_readiness(updated)
    if updated.get("status") not in {"stopped", "faulted"}:
        updated["status"] = (
            "ok"
            if updated["readiness"]["agent_sysid_smoke_allowed"] is True
            else "blocked"
        )
    return updated


def reject_arx5_runtime_start_without_confirmation(
    *,
    model: str,
    interface: str,
) -> dict[str, object]:
    return _arx5_runtime_start_rejection(
        model=model,
        interface=interface,
        reason="arx5 runtime start requires explicit operator confirmation",
        next_gate="confirm arx5 runtime start on the robot host",
    )


def arx5_runtime_start_preflight(
    *,
    model: str,
    interface: str,
    confirm: str | None,
) -> dict[str, object] | None:
    if confirm != ARX5_RUNTIME_START_CONFIRMATION:
        return reject_arx5_runtime_start_without_confirmation(
            model=model,
            interface=interface,
        )
    if importlib.util.find_spec("arx5_interface") is None:
        return _arx5_runtime_start_rejection(
            model=model,
            interface=interface,
            reason="arx5_interface is not importable in this environment",
            next_gate="run on the robot host with arx5_interface installed",
        )
    return None


def _arx5_runtime_start_rejection(
    *,
    model: str,
    interface: str,
    reason: str,
    next_gate: str,
) -> dict[str, object]:
    return {
        "status": "rejected",
        "schema": RUNTIME_SESSION_SCHEMA,
        "backend": "arx5_sdk",
        "model": model,
        "interface": interface,
        "reason": reason,
        "requires_confirm": ARX5_RUNTIME_START_CONFIRMATION,
        "hardware_motion": True,
        "movement_command_sent": False,
        "sdk_opened": False,
        "fault_landing_mode": MotionMode.DAMPING.value,
        "next_gate": next_gate,
    }


def acquire_owner_from_artifact(
    *,
    session_artifact_path: Path,
    owner: str,
    mode: str,
    expected_q_start: Sequence[float],
    max_start_error_rad: float,
    heartbeat_timeout_s: float,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    payload = refresh_runtime_status_payload(
        _read_json_object(session_artifact_path),
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    if payload.get("owner") is not None:
        raise RuntimeSessionError(f"runtime is owned by {payload.get('owner')}", payload)
    if payload.get("mode") != ArmRuntimeMode.HOLD_SAFE.value:
        raise RuntimeSessionError(
            f"runtime must be hold_safe before acquire, got {payload.get('mode')}",
            payload,
        )
    if payload.get("readiness", {}).get("agent_sysid_smoke_allowed") is not True:
        raise RuntimeSessionError("runtime readiness is not pass", payload)
    if heartbeat_timeout_s <= 0.0:
        raise ValueError("heartbeat_timeout_s must be positive")
    expected_q_start_tuple = _float_tuple(
        expected_q_start,
        name="expected_q_start",
    )
    if not _q_close(
        payload.get("q_meas"),
        list(expected_q_start_tuple),
        max_error_rad=float(max_start_error_rad),
    ):
        raise RuntimeSessionError(
            "current q_meas is not close to expected start pose",
            payload,
        )

    now_s = time.time()
    updated = dict(payload)
    updated["status"] = "ok"
    updated["mode"] = str(mode)
    updated["owner"] = str(owner)
    updated["owner_lease"] = {
        "schema": "armctrl.arm_runtime_owner_lease.v1",
        "runtime_session_id": updated.get("runtime_session_id"),
        "owner": str(owner),
        "mode": str(mode),
        "heartbeat_timeout_s": float(heartbeat_timeout_s),
        "heartbeat_wall_time_s": now_s,
        "acquired_wall_time_s": now_s,
        "landing_policy": "watchdog_to_damping_release_to_hold_safe",
    }
    updated["owner_deadman"] = _owner_deadman_status(updated)
    updated["heartbeat"] = _heartbeat_status(
        {"wall_time_s": now_s},
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    updated["readiness"] = runtime_readiness(updated)
    return updated


def release_owner_from_artifact(
    *,
    session_artifact_path: Path,
    owner: str,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    payload = refresh_runtime_status_payload(
        _read_json_object(session_artifact_path),
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    if payload.get("owner") != owner:
        raise RuntimeSessionError(f"runtime is not owned by {owner}", payload)
    updated = dict(payload)
    q_meas = list(updated.get("q_meas") or [])
    now_s = time.time()
    updated["status"] = "ok"
    updated["mode"] = ArmRuntimeMode.HOLD_SAFE.value
    updated["owner"] = None
    updated["owner_lease"] = None
    updated["owner_deadman"] = None
    updated["q_hold"] = q_meas
    updated["heartbeat"] = _heartbeat_status(
        {"wall_time_s": now_s},
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    updated["readiness"] = runtime_readiness(updated)
    return updated


def owner_heartbeat_from_artifact(
    *,
    session_artifact_path: Path,
    owner: str,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    payload = refresh_runtime_status_payload(
        _read_json_object(session_artifact_path),
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    owner_lease = payload.get("owner_lease")
    if payload.get("owner") != owner or not isinstance(owner_lease, dict):
        raise RuntimeSessionError(f"runtime is not owned by {owner}", payload)
    now_s = time.time()
    updated = dict(payload)
    updated["status"] = "ok"
    updated_lease = dict(owner_lease)
    updated_lease["heartbeat_wall_time_s"] = now_s
    updated["owner_lease"] = updated_lease
    updated["owner_deadman"] = _owner_deadman_status(updated)
    updated["heartbeat"] = _heartbeat_status(
        {"wall_time_s": now_s},
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    updated["readiness"] = runtime_readiness(updated)
    if updated["readiness"]["agent_sysid_smoke_allowed"] is not True:
        updated["status"] = "blocked"
    return updated


def watchdog_tick_from_artifact(
    *,
    session_artifact_path: Path,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    payload = refresh_runtime_status_payload(
        _read_json_object(session_artifact_path),
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    owner_lease = payload.get("owner_lease")
    if payload.get("owner") is None or not isinstance(owner_lease, dict):
        return payload
    heartbeat_wall_time_s = _float_or_none(owner_lease.get("heartbeat_wall_time_s"))
    heartbeat_timeout_s = _float_or_none(owner_lease.get("heartbeat_timeout_s"))
    now_s = time.time()
    if (
        heartbeat_wall_time_s is None
        or heartbeat_timeout_s is None
        or now_s - heartbeat_wall_time_s < heartbeat_timeout_s
    ):
        return payload

    updated = dict(payload)
    owner = updated.get("owner")
    updated["status"] = "faulted"
    updated["mode"] = MotionMode.DAMPING.value
    updated["owner"] = None
    updated["owner_lease"] = None
    updated["owner_deadman"] = None
    updated["watchdog"] = {
        "owner": owner,
        "landing_mode": MotionMode.DAMPING.value,
        "reason": "owner_heartbeat_timeout",
        "heartbeat_age_s": max(0.0, now_s - heartbeat_wall_time_s),
        "heartbeat_timeout_s": heartbeat_timeout_s,
    }
    updated["heartbeat"] = _heartbeat_status(
        {"wall_time_s": now_s},
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    updated["readiness"] = runtime_readiness(updated)
    return updated


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
    refreshed["owner_deadman"] = _owner_deadman_status(refreshed)
    hold = _hold_fresh_status(
        refreshed.get("last_hold_wall_time_s"),
        max_heartbeat_age_s=float(max_heartbeat_age_s),
    )
    refreshed["hold_fresh"] = hold["fresh"]
    refreshed["hold_age_s"] = hold["age_s"]
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
    if payload.get("hold_fresh") is not True:
        failed_checks.append("hold_fresh")
    if payload.get("fault_flags") not in ([], ()):
        failed_checks.append("no_fault_flags")
    return {
        "agent_sysid_smoke_allowed": not failed_checks,
        "failed_checks": failed_checks,
    }


def runtime_status_prerequisite_status(payload: dict[str, object]) -> str:
    readiness = runtime_readiness(payload)
    return "pass" if readiness["agent_sysid_smoke_allowed"] is True else "fail"


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
        "hold_tick_count": payload.get("hold_tick_count"),
        "last_hold_wall_time_s": payload.get("last_hold_wall_time_s"),
        "hold_fresh": payload.get("hold_fresh"),
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
        "hold_tick_count": 0,
        "last_hold_wall_time_s": None,
        "hold_fresh": False,
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


def _owner_deadman_status(payload: dict[str, object]) -> dict[str, object] | None:
    owner_lease = payload.get("owner_lease")
    owner = payload.get("owner")
    if owner is None or not isinstance(owner_lease, dict):
        return None
    heartbeat_wall_time_s = _float_or_none(owner_lease.get("heartbeat_wall_time_s"))
    heartbeat_timeout_s = _float_or_none(owner_lease.get("heartbeat_timeout_s"))
    now_s = time.time()
    heartbeat_age_s = (
        None
        if heartbeat_wall_time_s is None
        else max(0.0, now_s - heartbeat_wall_time_s)
    )
    return {
        "owner": owner,
        "mode": owner_lease.get("mode"),
        "heartbeat_wall_time_s": heartbeat_wall_time_s,
        "heartbeat_age_s": heartbeat_age_s,
        "heartbeat_timeout_s": heartbeat_timeout_s,
        "fresh": (
            heartbeat_age_s is not None
            and heartbeat_timeout_s is not None
            and heartbeat_age_s < heartbeat_timeout_s
        ),
    }


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


def _hold_fresh_status(
    last_hold_wall_time_s: object,
    *,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    wall_time_s = _float_or_none(last_hold_wall_time_s)
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


def _float_or_none(value: object) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _read_json_object(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} does not contain a JSON object")
    return payload
