from __future__ import annotations

from typing import Literal

AcceptanceStage = Literal["tiny_motion", "agent_smoke", "sysid_smoke"]


def build_real_motion_acceptance(
    *,
    stage: AcceptanceStage,
    motion_runtime: dict[str, object] | None,
    hardware_motion: bool,
    movement_command_sent: bool,
    readiness_passed: bool | None = None,
) -> dict[str, object]:
    """Summarize whether one real-motion artifact is good enough for the next gate."""

    runtime = motion_runtime or {}
    tracking = runtime.get("tracking")
    if not isinstance(tracking, dict):
        tracking = {}
    checks = {
        "hardware_motion_reported": _boolean_check(hardware_motion),
        "movement_command_sent": _boolean_check(movement_command_sent),
        "runtime_completed": _boolean_check(runtime.get("status") == "completed"),
        "controller_dt_measured": _present_check(runtime.get("controller_dt_s")),
        "actual_send_hz_measured": _present_check(runtime.get("actual_send_hz")),
        "send_jitter_measured": _boolean_check(
            runtime.get("send_jitter_ms_p95") is not None
            and runtime.get("send_jitter_ms_p99") is not None
        ),
        "sample_count_positive": _boolean_check(_sample_count(runtime) > 0),
        "no_fault_flags": _fault_check(runtime.get("fault_flags")),
        "final_tracking_error_measured": _present_check(
            tracking.get("final_tracking_error_max_abs_rad")
        ),
        "q_meas_responded_to_commanded_motion": _q_meas_response_check(tracking),
        "readiness_gate": _readiness_check(readiness_passed),
    }
    status = _acceptance_status(checks)
    return {
        "schema": "armctrl.real_motion_acceptance.v1",
        "stage": stage,
        "status": status,
        "hardware_motion": bool(hardware_motion),
        "movement_command_sent": bool(movement_command_sent),
        "sample_count": _sample_count(runtime),
        "checks": checks,
        "next_gate": _next_gate(stage=stage, status=status),
        "evidence_limit": (
            "artifact summary only; keep raw runtime logs and operator notes before "
            "claiming hardware acceptance"
        ),
    }


def _boolean_check(value: bool) -> dict[str, object]:
    return {"status": "pass" if value else "fail"}


def _present_check(value: object) -> dict[str, object]:
    return {"status": "pass" if value is not None else "not_available"}


def _fault_check(fault_flags: object) -> dict[str, object]:
    flags = sorted(str(flag) for flag in fault_flags) if isinstance(fault_flags, list) else []
    if flags:
        return {"status": "fail", "fault_flags": flags}
    return {"status": "pass", "fault_flags": []}


def _readiness_check(readiness_passed: bool | None) -> dict[str, object]:
    if readiness_passed is None:
        return {"status": "not_available"}
    return _boolean_check(readiness_passed)


def _q_meas_response_check(tracking: dict[str, object]) -> dict[str, object]:
    q_cmd_delta = _optional_float(tracking.get("q_cmd_delta_max_abs_rad"))
    q_meas_delta = _optional_float(tracking.get("q_meas_delta_max_abs_rad"))
    if q_cmd_delta is None or q_cmd_delta <= 0.0:
        return {
            "status": "not_applicable",
            "reason": "q_cmd_delta_missing_or_zero",
        }
    if q_meas_delta is None:
        return {"status": "fail", "reason": "q_meas_delta_missing"}
    return {
        "status": "pass" if q_meas_delta > 0.0 else "fail",
        "q_cmd_delta_max_abs_rad": q_cmd_delta,
        "q_meas_delta_max_abs_rad": q_meas_delta,
    }


def _acceptance_status(checks: dict[str, dict[str, object]]) -> str:
    statuses = [str(check.get("status")) for check in checks.values()]
    if "fail" in statuses:
        return "review_required"
    incomplete_statuses = [
        status
        for check in checks.values()
        for status in [str(check.get("status"))]
        if status == "not_available"
    ]
    if incomplete_statuses:
        return "incomplete"
    return "pass"


def _next_gate(*, stage: AcceptanceStage, status: str) -> str:
    if status != "pass":
        return "inspect_acceptance_checks_before_next_hardware_gate"
    if stage == "tiny_motion":
        return "agent_sysid_smoke_readiness"
    if stage == "agent_smoke":
        return "sysid_smoke"
    return "operator_review_before_larger_sysid_or_parameter_solve"


def _sample_count(runtime: dict[str, object]) -> int:
    value = runtime.get("sample_count")
    if isinstance(value, int):
        return value
    return 0


def _optional_float(value: object) -> float | None:
    if isinstance(value, int | float):
        return float(value)
    return None
