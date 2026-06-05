from __future__ import annotations

from dataclasses import dataclass
from itertools import product
import json
from pathlib import Path
from typing import Any, Protocol

import yaml

from armctrl.sysid import SysIdPlanRequest, SysIdPlanner
from armctrl.sysid_trajectory_backend import TrajectoryCommandError


class SysIdPlanWriter(Protocol):
    def write_plan(self, request: SysIdPlanRequest):
        ...


@dataclass(frozen=True)
class OedScanRequest:
    profile_name: str
    dof: int
    sample_hz: float
    q_center: tuple[float, ...]
    urdf_path: str
    safe_config_path: str
    output_dir: Path
    durations_s: tuple[float, ...]
    amplitudes_rad: tuple[float, ...]
    n_wps_values: tuple[int, ...]
    stack_reps_values: tuple[int, ...]
    ipopt_max_iterations: int
    condition_number_threshold: float
    random_seed_values: tuple[int, ...] = (1,)
    trajectory_command_argv: tuple[str, ...] | None = None


class OedScanRunner:
    def __init__(self, planner: SysIdPlanWriter | None = None) -> None:
        self._planner = planner or SysIdPlanner.default()

    def run(self, request: OedScanRequest) -> dict[str, Any]:
        request.output_dir.mkdir(parents=True, exist_ok=True)
        attempts: list[dict[str, Any]] = []
        for attempt_index, values in enumerate(
            product(
                request.durations_s,
                request.amplitudes_rad,
                request.n_wps_values,
                request.stack_reps_values,
                request.random_seed_values,
            ),
            start=1,
        ):
            duration_s, amplitude_rad, n_wps, stack_reps, random_seed = values
            attempts.append(
                self._run_attempt(
                    request,
                    attempt_id=f"attempt-{attempt_index:03d}",
                    duration_s=float(duration_s),
                    amplitude_rad=float(amplitude_rad),
                    n_wps=int(n_wps),
                    stack_reps=int(stack_reps),
                    random_seed=int(random_seed),
                )
            )
        result = {
            "schema": "armctrl.sysid_oed_scan.v1",
            "profile": request.profile_name,
            "sample_hz": request.sample_hz,
            "urdf_path": request.urdf_path,
            "base_safe_config": request.safe_config_path,
            "attempt_count": len(attempts),
            "attempts": attempts,
            "best_attempt": _best_attempt(attempts),
            "best_diagnostic_attempt": _best_diagnostic_attempt(attempts),
            "artifacts": {
                "summary": str(request.output_dir / "oed_scan_summary.json"),
            },
            "next_gate": (
                "rerun scan with trajectory_command on WSL if best_attempt is plan-only; "
                "use only attempts whose safety gate and OED quality gate both pass"
            ),
        }
        (request.output_dir / "oed_scan_summary.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return result

    def _run_attempt(
        self,
        request: OedScanRequest,
        *,
        attempt_id: str,
        duration_s: float,
        amplitude_rad: float,
        n_wps: int,
        stack_reps: int,
        random_seed: int,
    ) -> dict[str, Any]:
        attempt_dir = request.output_dir / attempt_id
        attempt_dir.mkdir(parents=True, exist_ok=True)
        safe_config_path = attempt_dir / "x5.safe.yaml"
        _write_attempt_safe_config(
            Path(request.safe_config_path),
            safe_config_path,
            n_wps=n_wps,
            stack_reps=stack_reps,
            random_seed=random_seed,
            ipopt_max_iterations=request.ipopt_max_iterations,
            condition_number_threshold=request.condition_number_threshold,
        )
        parameters = {
            "duration_s": duration_s,
            "amplitude_rad": amplitude_rad,
            "n_wps": n_wps,
            "stack_reps": stack_reps,
            "random_seed": random_seed,
            "ipopt_max_iterations": request.ipopt_max_iterations,
            "condition_number_threshold": request.condition_number_threshold,
        }
        plan_request = SysIdPlanRequest(
            profile_name=request.profile_name,
            dof=request.dof,
            sample_hz=request.sample_hz,
            duration_s=duration_s,
            amplitude_rad=amplitude_rad,
            q_center=request.q_center,
            urdf_path=request.urdf_path,
            safe_config_path=str(safe_config_path),
            output_dir=attempt_dir,
            trajectory_command_argv=request.trajectory_command_argv,
        )
        try:
            plan = self._planner.write_plan(plan_request)
        except TrajectoryCommandError as exc:
            classification = _classify_optimizer_failure(exc.detail)
            return {
                "attempt_id": attempt_id,
                "status": "faulted",
                "parameters": parameters,
                "safe_config": str(safe_config_path),
                "output_dir": str(attempt_dir),
                "error": {
                    "code": "trajectory_command_failed",
                    "message": str(exc),
                    "detail": exc.detail,
                },
                "failure_classification": classification,
            }
        payload = plan.to_json()
        backend = payload.get("trajectory_backend", {}) or {}
        safety = payload.get("safety", {}) or {}
        attempt = {
            "attempt_id": attempt_id,
            "status": "ok",
            "parameters": parameters,
            "safe_config": str(safe_config_path),
            "output_dir": str(attempt_dir),
            "safety_allowed": bool(safety.get("allowed", False)),
            "safety_reason": str(safety.get("reason", "")),
            "oed_valid": bool(backend.get("oed_valid", False)),
            "hardware_execution_eligible": bool(
                backend.get("hardware_execution_eligible", False)
            ),
            "oed_quality_gate": backend.get("oed_quality_gate"),
            "condition_number": backend.get("condition_number"),
            "rank": backend.get("rank"),
            "timing_contract": backend.get("timing_contract"),
            "sampling_contract": backend.get("sampling_contract"),
            "artifacts": payload.get("artifacts", {}),
        }
        candidate_source = backend.get("candidate_source")
        if isinstance(candidate_source, dict) and isinstance(
            candidate_source.get("generated_by"),
            dict,
        ):
            attempt["trajectory_command"] = candidate_source["generated_by"]
            attempt["failure_classification"] = _classify_optimizer_failure(
                candidate_source["generated_by"]
            )
        return attempt


def _write_attempt_safe_config(
    source: Path,
    destination: Path,
    *,
    n_wps: int,
    stack_reps: int,
    random_seed: int,
    ipopt_max_iterations: int,
    condition_number_threshold: float,
) -> None:
    raw = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    safety = raw.setdefault("safety", {})
    if not isinstance(safety, dict):
        raise ValueError("base safe config safety must be a mapping")
    sysid = safety.setdefault("sysid", {})
    if not isinstance(sysid, dict):
        raise ValueError("base safe config safety.sysid must be a mapping")
    sysid["oed"] = {
        "n_wps": n_wps,
        "stack_reps": stack_reps,
        "random_seed": random_seed,
        "ipopt_max_iterations": ipopt_max_iterations,
        "condition_number_threshold": condition_number_threshold,
    }
    destination.write_text(
        yaml.safe_dump(raw, sort_keys=False),
        encoding="utf-8",
    )


def _best_attempt(attempts: list[dict[str, Any]]) -> dict[str, Any] | None:
    ok_attempts = [attempt for attempt in attempts if attempt.get("status") == "ok"]
    if not ok_attempts:
        return None
    safe_attempts = [
        attempt for attempt in ok_attempts if attempt.get("safety_allowed") is True
    ]
    candidates = safe_attempts or ok_attempts

    def key(attempt: dict[str, Any]) -> tuple[int, float]:
        gate = attempt.get("oed_quality_gate")
        gate_pass = isinstance(gate, dict) and gate.get("status") == "pass"
        condition = attempt.get("condition_number")
        numeric_condition = (
            float(condition)
            if isinstance(condition, int | float) and condition > 0
            else float("inf")
        )
        return (0 if gate_pass else 1, numeric_condition)

    best = min(candidates, key=key)
    return {
        "attempt_id": best["attempt_id"],
        "status": best["status"],
        "safety_allowed": best.get("safety_allowed"),
        "oed_quality_gate": best.get("oed_quality_gate"),
        "condition_number": best.get("condition_number"),
        "rank": best.get("rank"),
        "parameters": best.get("parameters"),
        "output_dir": best.get("output_dir"),
    }


def _best_diagnostic_attempt(attempts: list[dict[str, Any]]) -> dict[str, Any] | None:
    candidates = [
        attempt
        for attempt in attempts
        if _attempt_optimizer_diagnostics(attempt) is not None
        or isinstance(attempt.get("failure_classification"), dict)
    ]
    if not candidates:
        return None
    best = min(candidates, key=_diagnostic_attempt_key)
    return {
        "attempt_id": best["attempt_id"],
        "status": best["status"],
        "failure_classification": best.get("failure_classification"),
        "optimizer_diagnostics": _attempt_optimizer_diagnostics(best),
        "parameters": best.get("parameters"),
        "output_dir": best.get("output_dir"),
    }


def _diagnostic_attempt_key(attempt: dict[str, Any]) -> tuple[int, float, float, float]:
    classification = attempt.get("failure_classification")
    kind = (
        str(classification.get("kind"))
        if isinstance(classification, dict)
        else "unknown"
    )
    kind_rank = {
        "optimizer_converged": 0,
        "optimizer_dual_infeasible": 1,
        "optimizer_not_converged": 2,
        "optimizer_constraint_infeasible": 3,
        "optimizer_not_reported": 4,
    }.get(kind, 5)
    diagnostics = _attempt_optimizer_diagnostics(attempt) or {}
    constraint_violation = _diagnostic_float(
        diagnostics,
        "constraint_violation_unscaled",
    )
    dual_infeasibility = _diagnostic_float(
        diagnostics,
        "dual_infeasibility_unscaled",
    )
    objective = _diagnostic_float(diagnostics, "objective_unscaled")
    return (
        kind_rank,
        constraint_violation if constraint_violation is not None else float("inf"),
        dual_infeasibility if dual_infeasibility is not None else float("inf"),
        objective if objective is not None else float("inf"),
    )


def _attempt_optimizer_diagnostics(attempt: dict[str, Any]) -> dict[str, Any] | None:
    error = attempt.get("error")
    if isinstance(error, dict):
        detail = error.get("detail")
        if isinstance(detail, dict) and isinstance(
            detail.get("optimizer_diagnostics"),
            dict,
        ):
            return detail["optimizer_diagnostics"]
    trajectory_command = attempt.get("trajectory_command")
    if isinstance(trajectory_command, dict) and isinstance(
        trajectory_command.get("optimizer_diagnostics"),
        dict,
    ):
        return trajectory_command["optimizer_diagnostics"]
    return None


def _classify_optimizer_failure(command_result: dict[str, Any]) -> dict[str, str]:
    convergence = command_result.get("optimizer_convergence")
    diagnostics = command_result.get("optimizer_diagnostics")
    if not isinstance(convergence, dict):
        return {
            "kind": "optimizer_not_reported",
            "next_action": "capture FIGAROH/IPOPT stdout before changing trajectory parameters",
        }
    if convergence.get("status") == "pass":
        return {
            "kind": "optimizer_converged",
            "next_action": "check regressor condition and simulation gates",
        }
    if not isinstance(diagnostics, dict):
        return {
            "kind": "optimizer_not_converged",
            "next_action": "inspect FIGAROH/IPOPT diagnostics before changing hardware safety limits",
        }
    constraint_violation = _diagnostic_float(
        diagnostics,
        "constraint_violation_unscaled",
    )
    dual_infeasibility = _diagnostic_float(
        diagnostics,
        "dual_infeasibility_unscaled",
    )
    if constraint_violation is not None and constraint_violation > 1e-6:
        return {
            "kind": "optimizer_constraint_infeasible",
            "next_action": "relax or repair trajectory constraints before increasing motion amplitude",
        }
    if dual_infeasibility is not None and dual_infeasibility > 1e-3:
        return {
            "kind": "optimizer_dual_infeasible",
            "next_action": "tune IPOPT scaling/initialization or reduce objective ill-conditioning before changing hardware safety limits",
        }
    return {
        "kind": "optimizer_not_converged",
        "next_action": "scan seeds and timing knobs, then compare optimizer diagnostics and regressor condition",
    }


def _diagnostic_float(diagnostics: dict[str, Any], key: str) -> float | None:
    value = diagnostics.get(key)
    if isinstance(value, int | float):
        return float(value)
    return None
