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
from uuid import uuid4
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
SUPPORTED_EEF_RUNTIME_ADAPTERS = ("moveit_servo", "sdk_cartesian")


class RuntimeSessionError(RuntimeError):
    def __init__(self, message: str, payload: dict[str, object]) -> None:
        self.payload = payload
        super().__init__(message)


def eef_adapter_manager_payload(
    *,
    primary_backend: str,
    configured_adapters: Sequence[str] | None = None,
    live_reference_adapters: Sequence[str] | None = None,
) -> dict[str, object]:
    """Describe the in-runtime EEF adapter registry for status/artifact audit."""

    configured = []
    for adapter in configured_adapters or ():
        adapter_name = str(adapter)
        if adapter_name not in configured:
            configured.append(adapter_name)
    live_reference = []
    for adapter in live_reference_adapters or ():
        adapter_name = str(adapter)
        if adapter_name in configured and adapter_name not in live_reference:
            live_reference.append(adapter_name)
    if "sdk_cartesian" in configured and "sdk_cartesian" not in live_reference:
        live_reference.append("sdk_cartesian")
    missing = [
        adapter
        for adapter in SUPPORTED_EEF_RUNTIME_ADAPTERS
        if adapter not in configured
    ]
    command_capabilities = _eef_command_capabilities(
        configured_adapters=configured,
        live_reference_adapters=live_reference,
    )
    adapter_status = {
        adapter: {
            "configured": adapter in configured,
            "executable": _adapter_has_any_executable_eef_capability(
                adapter=adapter,
                command_capabilities=command_capabilities,
            ),
            "command_capabilities": {
                command_kind: dict(
                    capability.get("adapters", {}).get(adapter, {})
                    if isinstance(capability.get("adapters"), dict)
                    else {}
                )
                for command_kind, capability in command_capabilities.items()
            },
            "hardware_scope": (
                "fake_rehearsal" if str(primary_backend) == "fake" else "runtime_adapter"
            ),
            "requires_bumpless_switch": True,
            "requires_prepare_hook": True,
        }
        for adapter in SUPPORTED_EEF_RUNTIME_ADAPTERS
    }
    return {
        "schema": "armctrl.eef_adapter_manager.v1",
        "status": "ready" if configured else "unconfigured",
        "primary_runtime_backend": str(primary_backend),
        "supported_adapters": list(SUPPORTED_EEF_RUNTIME_ADAPTERS),
        "configured_adapters": configured,
        "live_reference_adapters": live_reference,
        "missing_adapters": missing,
        "adapter_status": adapter_status,
        "adapter_registry_required": True,
        "primary_backend_fallback_allowed": False,
        "disconnected_takeover_allowed": False,
        "no_heuristic_joint_fallback": True,
        "bumpless_switch_required": True,
        "eef_command_capabilities": command_capabilities,
        "eef_command_executable": any(
            bool(capability.get("executable"))
            for capability in command_capabilities.values()
        ),
        "policy": (
            "EEF commands must resolve through an explicit in-runtime mature "
            "adapter registry; the primary runtime backend may not be used as "
            "a silent fallback."
        ),
    }


def _eef_command_capabilities(
    *,
    configured_adapters: Sequence[str],
    live_reference_adapters: Sequence[str],
) -> dict[str, dict[str, object]]:
    configured = set(str(adapter) for adapter in configured_adapters)
    live_reference = set(str(adapter) for adapter in live_reference_adapters)

    def adapter_capability(
        adapter: str,
        *,
        requires_live_reference: bool = False,
    ) -> dict[str, object]:
        configured_now = adapter in configured
        live_reference_ready = adapter in live_reference
        executable = configured_now and (
            live_reference_ready if requires_live_reference else True
        )
        reason = "ready"
        if not configured_now:
            reason = "adapter_not_configured"
        elif requires_live_reference and not live_reference_ready:
            reason = "requires_live_reference"
        return {
            "configured": configured_now,
            "live_reference": live_reference_ready,
            "executable": executable,
            "reason": reason,
        }

    capabilities = {
        "eef_pose_delta": {
            "requires_live_reference": False,
            "adapters": {
                "moveit_servo": adapter_capability("moveit_servo"),
                "sdk_cartesian": adapter_capability("sdk_cartesian"),
            },
        },
        "eef_twist": {
            "requires_live_reference": False,
            "adapters": {
                "moveit_servo": adapter_capability("moveit_servo"),
                "sdk_cartesian": adapter_capability("sdk_cartesian"),
            },
        },
        "eef_pose": {
            "requires_live_reference": True,
            "adapters": {
                "moveit_servo": adapter_capability(
                    "moveit_servo",
                    requires_live_reference=True,
                ),
                "sdk_cartesian": adapter_capability(
                    "sdk_cartesian",
                    requires_live_reference=True,
                ),
            },
        },
    }
    for command_kind, capability in capabilities.items():
        adapters = capability["adapters"]
        executable_adapters = [
            adapter
            for adapter, adapter_capability_payload in adapters.items()
            if adapter_capability_payload["executable"] is True
        ]
        reason = "ready" if executable_adapters else _first_adapter_block_reason(adapters)
        capability["executable"] = bool(executable_adapters)
        capability["executable_adapters"] = executable_adapters
        capability["reason"] = reason
    return capabilities


def _first_adapter_block_reason(adapters: dict[str, dict[str, object]]) -> str:
    reasons = [
        str(payload.get("reason"))
        for payload in adapters.values()
        if payload.get("configured") is True
    ]
    if reasons:
        return reasons[0]
    return "adapter_not_configured"


def _adapter_has_any_executable_eef_capability(
    *,
    adapter: str,
    command_capabilities: dict[str, dict[str, object]],
) -> bool:
    for capability in command_capabilities.values():
        adapters = capability.get("adapters")
        if not isinstance(adapters, dict):
            continue
        adapter_payload = adapters.get(adapter)
        if isinstance(adapter_payload, dict) and adapter_payload.get("executable") is True:
            return True
    return False


def runtime_controller_manager_payload(
    *,
    backend: str,
    eef_adapter_manager: dict[str, object] | None = None,
) -> dict[str, object]:
    """Describe runtime-owned controllers available to Agent/SysID/Recipe/UI."""

    eef_manager = (
        eef_adapter_manager
        if isinstance(eef_adapter_manager, dict)
        else eef_adapter_manager_payload(primary_backend=str(backend))
    )
    eef_ready = bool(eef_manager.get("eef_command_executable"))
    eef_status = "available" if eef_ready else "needs_mature_adapter"
    eef_capabilities = (
        eef_manager.get("eef_command_capabilities")
        if isinstance(eef_manager.get("eef_command_capabilities"), dict)
        else {}
    )
    return {
        "schema": "armctrl.runtime_controller_manager.v1",
        "status": "ok",
        "primary_runtime_backend": str(backend),
        "sdk_can_singleton": True,
        "mode_switch_owner": "arm_runtime",
        "controller_switch_policy": "runtime_internal_no_cli_reopen",
        "controllers": {
            "joint_hold": {
                "status": "available",
                "command_space": "joint",
                "role": "safe live hold and release target",
                "requires_live_hold": True,
            },
            "joint_trajectory": {
                "status": "available",
                "command_space": "joint",
                "role": "SysID/Recipe/preposition reviewed trajectory replay",
                "contract": "q_points/dq_points/ddq_points/sample_hz/send_hz",
                "safety": "start_pose_guard + segment/velocity/tracking evidence",
            },
            "joint_intent": {
                "status": "available",
                "command_space": "joint",
                "role": "Agent low-frequency target intent shaped into short horizon trajectory",
                "safety": "max_joint_delta + max_joint_velocity + smoothstep artifact",
            },
            "eef_servo": {
                "status": eef_status,
                "command_space": "eef",
                "role": "Agent/Teleop EEF delta/twist/pose through mature servo adapter",
                "adapter_registry_required": True,
                "configured_adapters": list(eef_manager.get("configured_adapters") or []),
                "missing_adapters": list(eef_manager.get("missing_adapters") or []),
                "command_capabilities": eef_capabilities,
                "bumpless_switch_required": True,
                "start_pose_policy": "live runtime hold; passive/droop is not a controlled Agent EEF start",
                "safe_position_policy": "SAFE_CENTER is maintained by joint_hold before EEF target is seeded from current FK",
            },
        },
        "fallback_policy": {
            "primary_backend_fallback_allowed": False,
            "disconnected_takeover_allowed": False,
            "no_heuristic_joint_fallback": True,
            "fault_landing_mode": MotionMode.DAMPING.value,
        },
    }


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


def start_arx5_cartesian_runtime_session(
    *,
    backend: MotionBackend,
    model: str,
    interface: str,
    safe_center: Sequence[float],
    send_hz: float,
    hold_hz: float,
    max_start_error_rad: float,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    enter = getattr(backend, "enter_hold_or_damping", None)
    if callable(enter):
        enter()
    safe_center_tuple = _float_tuple(safe_center, name="safe_center")
    state = backend.read_joint_state()
    q_meas = tuple(float(value) for value in state.q_meas)
    if len(q_meas) != len(safe_center_tuple):
        try:
            backend.damping()
        finally:
            raise RuntimeError(
                "sdk_cartesian runtime requires current q_meas length to match SAFE_CENTER"
            )
    startup_guard = {
        "status": "pass",
        "policy": "current_measured_cartesian_takeover",
        "q_meas": list(q_meas),
        "safe_center": list(safe_center_tuple),
        "max_start_error_rad": float(max_start_error_rad),
        "q_meas_to_safe_center_max_abs_rad": max(
            abs(measured - expected)
            for measured, expected in zip(q_meas, safe_center_tuple, strict=True)
        ),
        "failed_checks": [],
    }
    runtime = ArmRuntime(
        backend=backend,
        safe_center=safe_center_tuple,
    )
    runtime.mark_hold_safe(q_hold=q_meas)
    hold_mode = backend.hold()
    status = runtime.status()
    payload = runtime_status_payload(
        status=status,
        schema=RUNTIME_SESSION_SCHEMA,
        backend="sdk_cartesian",
        send_hz=float(send_hz),
        hold_hz=float(hold_hz),
        max_heartbeat_age_s=float(max_heartbeat_age_s),
        recovery={
            "status": "skipped",
            "reason": "sdk_cartesian runtime takes over from current measured EEF pose; use arx5_sdk joint runtime for SysID SAFE_CENTER recovery",
            "landing_mode": hold_mode or MotionMode.HOLD.value,
        },
    )
    payload["last_hold_wall_time_s"] = time.time()
    payload["hold_fresh"] = True
    payload["hold_age_s"] = 0.0
    payload["startup_guard"] = startup_guard
    payload.update(
        {
            "model": model,
            "interface": interface,
            "hardware_motion": True,
            "sdk_opened": True,
            "movement_command_sent": True,
            "requires_confirm": ARX5_RUNTIME_START_CONFIRMATION,
            "fault_landing_mode": MotionMode.DAMPING.value,
            "readiness_pose_policy": "current_measured_pose",
            "runtime_contract": {
                "state_machine": "current_measured_cartesian_takeover->hold_safe->owner_lease",
                "single_motion_owner": True,
                "readiness_requires_live_hold": True,
                "backend_role": "agent_eef_sdk_cartesian",
                "droop_recovery_supported": False,
                "safe_center_required": False,
            },
        }
    )
    payload["readiness"] = runtime_readiness(payload)
    payload["status"] = (
        "ok"
        if payload["readiness"]["agent_sysid_smoke_allowed"] is True
        else "blocked"
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
    agent_watchdog = _agent_intent_watchdog_event(payload, owner_lease)
    if agent_watchdog is not None:
        _write_json_atomic(session_artifact_path, agent_watchdog)
        return agent_watchdog
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
    _write_json_atomic(session_artifact_path, updated)
    return updated


def _agent_intent_watchdog_event(
    payload: dict[str, object],
    owner_lease: dict[str, object],
) -> dict[str, object] | None:
    if (
        payload.get("owner") != "agent"
        or owner_lease.get("mode") != MotionMode.AGENT_SERVO.value
    ):
        return None
    last_intent_wall_time_s = _float_or_none(owner_lease.get("last_intent_wall_time_s"))
    missed_intent_timeout_s = _float_or_none(
        owner_lease.get("missed_intent_timeout_s")
    )
    fault_timeout_s = _float_or_none(owner_lease.get("fault_timeout_s"))
    if (
        last_intent_wall_time_s is None
        or missed_intent_timeout_s is None
        or fault_timeout_s is None
    ):
        return None
    now_s = time.time()
    intent_age_s = max(0.0, now_s - last_intent_wall_time_s)
    owner = payload.get("owner")
    if intent_age_s >= fault_timeout_s:
        updated = dict(payload)
        updated["status"] = "faulted"
        updated["mode"] = MotionMode.DAMPING.value
        updated["owner"] = None
        updated["owner_lease"] = None
        updated["owner_deadman"] = None
        updated["watchdog"] = {
            "owner": owner,
            "landing_mode": MotionMode.DAMPING.value,
            "reason": "agent_intent_fault_timeout",
            "intent_age_s": intent_age_s,
            "fault_timeout_s": fault_timeout_s,
        }
        updated["heartbeat"] = _heartbeat_status(
            {"wall_time_s": now_s},
            max_heartbeat_age_s=max(1.0, fault_timeout_s),
        )
        updated["readiness"] = runtime_readiness(updated)
        return updated
    if (
        owner_lease.get("missed_intent_held") is not True
        and intent_age_s >= missed_intent_timeout_s
    ):
        updated = dict(payload)
        updated["status"] = "ok"
        updated["mode"] = ArmRuntimeMode.HOLD_SAFE.value
        updated["owner"] = None
        updated["owner_lease"] = None
        updated["owner_deadman"] = None
        updated["last_hold_wall_time_s"] = now_s
        updated["hold_fresh"] = True
        updated["hold_age_s"] = 0.0
        updated["watchdog"] = {
            "owner": owner,
            "landing_mode": MotionMode.HOLD.value,
            "reason": "missed_agent_intent_timeout",
            "intent_age_s": intent_age_s,
            "missed_intent_timeout_s": missed_intent_timeout_s,
        }
        updated["heartbeat"] = _heartbeat_status(
            {"wall_time_s": now_s},
            max_heartbeat_age_s=max(1.0, missed_intent_timeout_s),
        )
        updated["readiness"] = runtime_readiness(updated)
        return updated
    return None


def refresh_runtime_status_payload(
    payload: dict[str, object],
    *,
    max_heartbeat_age_s: float,
) -> dict[str, object]:
    refreshed = dict(payload)
    refreshed["schema"] = RUNTIME_STATUS_SCHEMA
    refreshed["status"] = "ok"
    if not isinstance(refreshed.get("eef_adapter_manager"), dict):
        refreshed["eef_adapter_manager"] = eef_adapter_manager_payload(
            primary_backend=str(refreshed.get("backend") or "unknown"),
            configured_adapters=(),
        )
    refreshed["runtime_controller_manager"] = runtime_controller_manager_payload(
        backend=str(refreshed.get("backend") or "unknown"),
        eef_adapter_manager=refreshed["eef_adapter_manager"],
    )
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
    safe_center_failed_checks: list[str] = []
    pose_policy = str(payload.get("readiness_pose_policy") or "safe_center")
    if payload.get("schema") not in {RUNTIME_SESSION_SCHEMA, RUNTIME_STATUS_SCHEMA}:
        failed_checks.append("schema")
        safe_center_failed_checks.append("schema")
    if payload.get("mode") != ArmRuntimeMode.HOLD_SAFE.value:
        failed_checks.append("mode_hold_safe")
        safe_center_failed_checks.append("mode_hold_safe")
    if payload.get("owner") is not None:
        failed_checks.append("no_owner")
        safe_center_failed_checks.append("no_owner")
    if not _q_close(payload.get("q_meas"), payload.get("q_hold"), max_error_rad=0.02):
        safe_center_failed_checks.append("q_meas_close_to_hold")
        if pose_policy != "current_measured_pose":
            failed_checks.append("q_meas_close_to_hold")
    if not _q_close(payload.get("q_hold"), payload.get("safe_center"), max_error_rad=0.02):
        safe_center_failed_checks.append("q_hold_close_to_safe_center")
    if not isinstance(heartbeat, dict) or heartbeat.get("fresh") is not True:
        failed_checks.append("heartbeat_fresh")
        safe_center_failed_checks.append("heartbeat_fresh")
    if payload.get("hold_fresh") is not True:
        failed_checks.append("hold_fresh")
        safe_center_failed_checks.append("hold_fresh")
    if payload.get("fault_flags") not in ([], ()):
        failed_checks.append("no_fault_flags")
        safe_center_failed_checks.append("no_fault_flags")
    return {
        "agent_sysid_smoke_allowed": not failed_checks,
        "live_hold_allowed": not failed_checks,
        "safe_center_allowed": not safe_center_failed_checks,
        "pose_policy": pose_policy,
        "failed_checks": failed_checks,
        "safe_center_failed_checks": safe_center_failed_checks,
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
        "eef_adapter_manager": payload.get("eef_adapter_manager"),
        "runtime_controller_manager": payload.get("runtime_controller_manager"),
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
        "eef_adapter_manager": eef_adapter_manager_payload(
            primary_backend=str(backend),
            configured_adapters=(),
        ),
    }
    payload["runtime_controller_manager"] = runtime_controller_manager_payload(
        backend=str(backend),
        eef_adapter_manager=payload["eef_adapter_manager"],
    )
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


def _write_json_atomic(path: Path, payload: dict[str, object]) -> None:
    tmp_path = path.with_name(f"{path.name}.{uuid4().hex}.tmp")
    tmp_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    tmp_path.replace(path)
