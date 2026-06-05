from __future__ import annotations

import csv
from dataclasses import dataclass
import importlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from armctrl.workspace import WorkspaceSafetyConfig


@dataclass(frozen=True)
class SysIdTrajectoryPlan:
    rows: list[dict[str, str]]
    backend: dict[str, Any]


@dataclass(frozen=True)
class CandidateTrajectory:
    rows: list[dict[str, str]]
    raw_sample_count: int


class TrajectoryCommandError(ValueError):
    def __init__(self, detail: dict[str, Any]) -> None:
        self.detail = detail
        super().__init__(
            "trajectory command failed with exit code "
            f"{detail.get('exit_code')}"
        )


def plan_sysid_trajectory(
    request: object,
    *,
    write_artifacts: bool = True,
) -> SysIdTrajectoryPlan:
    """Plan a SysID trajectory through mature-tool boundaries.

    armctrl only prepares the FIGAROH/OED request and emits a conservative
    smoke fallback when the real robot-specific optimizer is unavailable.
    """
    candidate_path = getattr(request, "candidate_trajectory_path", None)
    trajectory_command_argv = getattr(request, "trajectory_command_argv", None)
    if candidate_path is not None and trajectory_command_argv is not None:
        raise ValueError(
            "--candidate-trajectory and --trajectory-command are mutually exclusive"
        )
    figaroh_config_path = Path(request.output_dir) / "figaroh_trajectory_request.json"
    figaroh_config = _figaroh_request_config(request)
    if write_artifacts:
        figaroh_config_path.parent.mkdir(parents=True, exist_ok=True)
        figaroh_config_path.write_text(
            json.dumps(figaroh_config, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    command_result = None
    if trajectory_command_argv is not None:
        if not write_artifacts:
            raise ValueError("trajectory command requires artifact output")
        candidate_path = Path(request.output_dir) / "external_candidate_trajectory.csv"
        command_result = _run_trajectory_command(
            tuple(str(arg) for arg in trajectory_command_argv),
            figaroh_config_path=figaroh_config_path,
            candidate_path=candidate_path,
        )
    candidate = None
    if candidate_path is not None:
        candidate = _candidate_rows(
            Path(candidate_path),
            dof=int(request.dof),
            sample_hz=float(request.sample_hz),
        )
        rows = candidate.rows
    else:
        rows = _fallback_rows(
            request,
            timing=figaroh_config["figaroh"]["timing"],
        )
    backend = _backend_metadata(
        request,
        rows=rows,
        candidate_path=Path(candidate_path) if candidate_path is not None else None,
        raw_candidate_sample_count=(
            candidate.raw_sample_count if candidate is not None else None
        ),
        command_result=command_result,
        figaroh_config_path=figaroh_config_path,
        figaroh_config=figaroh_config,
    )
    return SysIdTrajectoryPlan(rows=rows, backend=backend)


def _backend_metadata(
    request: object,
    *,
    rows: list[dict[str, str]],
    candidate_path: Path | None,
    raw_candidate_sample_count: int | None,
    command_result: dict[str, Any] | None,
    figaroh_config_path: Path,
    figaroh_config: dict[str, Any],
) -> dict[str, Any]:
    dependency_status = {
        "figaroh": _figaroh_status(),
        "pinocchio": _module_status("pinocchio"),
        "cyipopt": _module_status("cyipopt"),
    }
    regressor_score = _pinocchio_regressor_score(
        request,
        rows=rows,
        backend_status=dependency_status["pinocchio"],
    )
    missing = [
        name
        for name, status in dependency_status.items()
        if status["status"] != "available"
    ]
    optimizer_ready = not missing and Path(str(request.urdf_path)).exists()
    status = "available_not_configured" if optimizer_ready else "fallback"
    fallback = _fallback_metadata(request, rows=rows, used=candidate_path is None)
    selected = _selected_backend(candidate_path, command_result, fallback["name"])
    oed_quality_gate = _oed_quality_gate(
        selected=selected,
        command_result=command_result,
        regressor_score=regressor_score,
        condition_number_threshold=float(
            figaroh_config["figaroh"]["quality_gate"][
                "condition_number_threshold"
            ]
        ),
    )
    metadata: dict[str, Any] = {
        "requested": "figaroh_optimal_trajectory",
        "selected": selected,
        "status": "external_candidate" if candidate_path is not None else status,
        "profile": str(request.profile_name),
        "profile_role": _requested_profile_role(str(request.profile_name)),
        "oed_valid": oed_quality_gate["status"] == "pass",
        "hardware_execution_eligible": oed_quality_gate["status"] == "pass",
        "next_gate": _next_oed_gate(oed_quality_gate),
        "implementation_boundary": (
            "armctrl orchestrates; FIGAROH owns optimal excitation math"
        ),
        "figaroh_vendor_reference": {
            "class": "figaroh.optimal.BaseOptimalTrajectory",
            "source_path": str(_figaroh_base_optimal_source()),
            "trajectory_parameterization": "cubic spline",
            "objective": "base regressor condition number",
            "constraints": [
                "joint_position",
                "joint_velocity",
                "joint_torque",
                "collision",
            ],
            "optimizer": "IPOPT/cyipopt",
        },
        "dependency_status": dependency_status,
        "regressor_score": regressor_score,
        "condition_number": regressor_score.get("effective_condition_number"),
        "rank": regressor_score.get("rank"),
        "objective": regressor_score.get("objective"),
        "oed_quality_gate": oed_quality_gate,
        "timing_contract": figaroh_config["figaroh"]["timing"],
        "sampling_contract": figaroh_config["sampling"],
        "artifacts": {
            "figaroh_config": str(figaroh_config_path),
        },
        "fallback": fallback,
    }
    if candidate_path is not None:
        metadata["candidate_source"] = {
            "path": str(candidate_path),
            "format": "csv",
            "required_columns": _required_candidate_columns(int(request.dof)),
            "sample_count": len(rows),
            "raw_sample_count": raw_candidate_sample_count,
            "resampled_to_sample_hz": float(request.sample_hz),
        }
        if command_result is not None:
            metadata["candidate_source"]["generated_by"] = command_result
    return metadata


def _oed_quality_gate(
    *,
    selected: str,
    command_result: dict[str, Any] | None,
    regressor_score: dict[str, Any],
    condition_number_threshold: float,
) -> dict[str, Any]:
    reasons: list[str] = []
    if selected != "external_oed_command":
        reasons.append("external_figaroh_oed_not_run")
    convergence = (
        command_result.get("optimizer_convergence")
        if command_result is not None
        else None
    )
    if isinstance(convergence, dict) and convergence.get("status") == "fail":
        reasons.append("optimizer_not_converged")
    elif command_result is not None and not isinstance(convergence, dict):
        reasons.append("optimizer_convergence_not_reported")
    if regressor_score.get("status") != "computed":
        reasons.append("pinocchio_regressor_not_computed")
    else:
        condition = float(regressor_score.get("effective_condition_number", float("inf")))
        rank = int(regressor_score.get("rank", 0))
        if rank <= 0:
            reasons.append("regressor_rank_zero")
        if condition > condition_number_threshold:
            reasons.append("regressor_condition_too_high")
    return {
        "status": "fail" if reasons else "pass",
        "reasons": reasons,
        "condition_number_threshold": condition_number_threshold,
        "note": (
            "This gate is for offline OED quality only. Passing simulation safety "
            "does not imply identified parameters are valid."
        ),
    }


def _next_oed_gate(oed_quality_gate: dict[str, Any]) -> str:
    if oed_quality_gate["status"] == "pass":
        return "run_sysid_safety_preview_before_sdk_execution"
    return "improve_figaroh_oed_until_regressor_quality_gate_passes"


def _selected_backend(
    candidate_path: Path | None,
    command_result: dict[str, Any] | None,
    fallback_name: str,
) -> str:
    if command_result is not None:
        return "external_oed_command"
    if candidate_path is not None:
        return "external_candidate_trajectory"
    return fallback_name


def _run_trajectory_command(
    command_argv: tuple[str, ...],
    *,
    figaroh_config_path: Path,
    candidate_path: Path,
) -> dict[str, Any]:
    if not command_argv:
        raise ValueError("trajectory command cannot be empty")
    candidate_path.parent.mkdir(parents=True, exist_ok=True)
    env = {
        **os.environ,
        "ARMCTRL_FIGAROH_REQUEST": str(figaroh_config_path),
        "ARMCTRL_CANDIDATE_TRAJECTORY": str(candidate_path),
    }
    completed = subprocess.run(
        list(command_argv),
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )
    if completed.returncode != 0:
        raise TrajectoryCommandError(
            _trajectory_command_result(command_argv, completed)
        )
    return _trajectory_command_result(command_argv, completed)


def _trajectory_command_result(
    command_argv: tuple[str, ...],
    completed: subprocess.CompletedProcess[str],
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "command_argv": list(command_argv),
        "exit_code": completed.returncode,
    }
    stdout = completed.stdout.strip()
    stderr = completed.stderr.strip()
    if stdout:
        result["stdout"] = stdout
        stdout_json = _extract_last_json_object(stdout)
        if stdout_json is not None:
            result["stdout_json"] = stdout_json
        result["optimizer_convergence"] = _optimizer_convergence_from_stdout(stdout)
    if stderr:
        result["stderr"] = stderr
    return result


def _extract_last_json_object(text: str) -> dict[str, Any] | None:
    for line in reversed(text.splitlines()):
        stripped = line.strip()
        if not stripped.startswith("{") or not stripped.endswith("}"):
            continue
        try:
            decoded = json.loads(stripped)
        except json.JSONDecodeError:
            continue
        if isinstance(decoded, dict):
            return decoded
    return None


def _optimizer_convergence_from_stdout(stdout: str) -> dict[str, str]:
    if "EXIT: Optimal Solution Found." in stdout:
        return {"status": "pass", "reason": "optimal_solution_found"}
    if "EXIT: Maximum Number of Iterations Exceeded." in stdout:
        return {"status": "fail", "reason": "max_iterations_exceeded"}
    if "EXIT:" in stdout:
        exit_line = next(
            (
                line.strip()
                for line in reversed(stdout.splitlines())
                if line.strip().startswith("EXIT:")
            ),
            "EXIT: unknown",
        )
        return {"status": "fail", "reason": _slugify_optimizer_exit(exit_line)}
    return {"status": "not_evaluated", "reason": "optimizer_exit_not_reported"}


def _slugify_optimizer_exit(exit_line: str) -> str:
    text = exit_line.replace("EXIT:", "").strip().lower()
    slug = "".join(char if char.isalnum() else "_" for char in text)
    return "_".join(part for part in slug.split("_") if part) or "unknown_exit"


def _fallback_metadata(
    request: object,
    *,
    rows: list[dict[str, str]],
    used: bool,
) -> dict[str, Any]:
    if not used:
        return {
            "name": "deterministic_smoke_probe",
            "status": "not_used",
            "reason": "external candidate trajectory was provided",
        }
    return {
        "name": "deterministic_smoke_probe",
        "status": "used",
        "profile_role": _fallback_profile_role(str(request.profile_name)),
        "oed_valid": False,
        "reason": (
            "robot-specific FIGAROH BaseOptimalTrajectory subclass/config is not "
            "wired into armctrl yet"
        ),
        "sample_count": len(rows),
    }


def _figaroh_request_config(request: object) -> dict[str, Any]:
    constraints = _figaroh_numeric_constraints(request)
    timing = _figaroh_timing(request, constraints=constraints)
    oed_config = read_sysid_oed_config(Path(str(request.safe_config_path)))
    requested_sample_count = _sample_count_for_duration(
        float(request.duration_s),
        sample_hz=float(request.sample_hz),
    )
    effective_sample_count = _sample_count_for_duration(
        float(timing["effective_duration_s"]),
        sample_hz=float(request.sample_hz),
    )
    return {
        "schema": "armctrl.figaroh_optimal_trajectory_request.v1",
        "profile": str(request.profile_name),
        "model": {
            "urdf_path": str(request.urdf_path),
            "dof": int(request.dof),
            "active_joints": [f"joint{index + 1}" for index in range(int(request.dof))],
        },
        "sampling": {
            "sample_hz": float(request.sample_hz),
            "duration_s": float(request.duration_s),
            "requested_sample_count": requested_sample_count,
            "effective_duration_s": float(timing["effective_duration_s"]),
            "effective_sample_count": effective_sample_count,
            "sample_count": effective_sample_count,
        },
        "seed": {
            "q_center": list(request.q_center),
            "amplitude_rad": float(request.amplitude_rad),
        },
        "constraints": constraints,
        "figaroh": {
            "class": "figaroh.optimal.BaseOptimalTrajectory",
            "objective": "condition_number(base_regressor)",
            "parameterization": "cubic_spline",
            "constraint_manager": "figaroh.optimal.contraints.TrajectoryConstraintManager",
            "timing": timing,
            "optimizer": {
                "ipopt_max_iterations": oed_config["ipopt_max_iterations"],
            },
            "quality_gate": {
                "condition_number_threshold": oed_config[
                    "condition_number_threshold"
                ],
            },
            "expected_outputs": [
                "optimized_waypoints",
                "planned_trajectory",
                "base_regressor_condition_number",
                "rank",
            ],
        },
        "armctrl_boundary": (
            "armctrl writes this request, runs safety gates on returned trajectory, "
            "and records evidence; FIGAROH/Pinocchio own OED/regressor math"
        ),
    }


def _figaroh_timing(
    request: object,
    *,
    constraints: dict[str, Any],
) -> dict[str, Any]:
    oed_config = read_sysid_oed_config(Path(str(request.safe_config_path)))
    n_wps = int(oed_config["n_wps"])
    stack_reps = int(oed_config["stack_reps"])
    sample_hz = float(request.sample_hz)
    requested_duration_s = float(request.duration_s)
    requested_segment_duration_s = requested_duration_s / stack_reps
    requested_t_s = requested_segment_duration_s / max(1, n_wps - 1)
    min_t_s = _minimum_waypoint_duration_from_limits(constraints)
    t_s = max(requested_t_s, min_t_s, 1.0 / sample_hz)
    segment_duration_s = t_s * max(1, n_wps - 1)
    effective_duration_s = segment_duration_s * stack_reps
    adjusted = effective_duration_s > requested_duration_s + 1e-9
    return {
        "execution_sample_hz": round(sample_hz, 12),
        "execution_sample_period_s": round(1.0 / sample_hz, 12),
        "n_wps": n_wps,
        "stack_reps": stack_reps,
        "requested_duration_s": round(requested_duration_s, 12),
        "requested_segment_duration_s": round(requested_segment_duration_s, 12),
        "minimum_waypoint_duration_s": round(min_t_s, 12),
        "waypoint_duration_s": round(t_s, 12),
        "segment_duration_s": round(segment_duration_s, 12),
        "effective_duration_s": round(effective_duration_s, 12),
        "duration_adjusted_for_safety": adjusted,
        "duration_adjustment_reason": (
            "max_joint_step_velocity_limit"
            if adjusted
            else "requested_duration_satisfied"
        ),
    }


def _minimum_waypoint_duration_from_limits(constraints: dict[str, Any]) -> float:
    minimum = 0.0
    for q_pair, dq_pair in zip(
        constraints["joint_limits_rad"],
        constraints["velocity_limits_rad_s"],
    ):
        span = abs(float(q_pair[1]) - float(q_pair[0]))
        velocity = max(abs(float(dq_pair[0])), abs(float(dq_pair[1])))
        if span <= 0.0 or velocity <= 0.0:
            continue
        # Cubic rest-to-rest profiles need margin above average velocity;
        # keeping this conservative prevents FIGAROH from optimizing inside an
        # infeasible safety box.
        minimum = max(minimum, 2.0 * span / velocity)
    return minimum


def _figaroh_numeric_constraints(request: object) -> dict[str, Any]:
    safe_config = WorkspaceSafetyConfig.from_yaml(Path(str(request.safe_config_path)))
    urdf_limits = _read_urdf_motion_limits(
        Path(str(request.urdf_path)),
        active_joints=[f"joint{index + 1}" for index in range(int(request.dof))],
    )
    q_center = [float(value) for value in request.q_center]
    amplitude = min(abs(float(request.amplitude_rad)), safe_config.max_sysid_amplitude_rad)
    sample_hz = float(request.sample_hz)
    max_joint_step_rad = float(safe_config.max_joint_step_rad)
    derived_velocity_limit = max_joint_step_rad * sample_hz
    joint_limits = []
    velocity_limits = []
    effort_limits = []
    for index, joint in enumerate(urdf_limits):
        center = q_center[index]
        lower = max(float(joint["lower"]), center - amplitude)
        upper = min(float(joint["upper"]), center + amplitude)
        if lower > upper:
            clamped = min(max(center, float(joint["lower"])), float(joint["upper"]))
            lower = clamped
            upper = clamped
        joint_limits.append([round(lower, 12), round(upper, 12)])
        velocity = min(abs(float(joint["velocity"])), derived_velocity_limit)
        velocity_limits.append([-round(velocity, 12), round(velocity, 12)])
        effort = abs(float(joint["effort"]))
        effort_limits.append([-round(effort, 12), round(effort, 12)])
    return {
        "safe_config_path": str(request.safe_config_path),
        "urdf_limits": True,
        "workspace_gate": True,
        "collision_gate": True,
        "torque_gate": True,
        "max_joint_step_rad": max_joint_step_rad,
        "derived_velocity_limit_rad_s": round(derived_velocity_limit, 12),
        "joint_limits_rad": joint_limits,
        "velocity_limits_rad_s": velocity_limits,
        "effort_limits_nm": effort_limits,
        "joint_relation_constraints": read_sysid_joint_relation_constraints(
            Path(str(request.safe_config_path))
        ),
    }


def read_sysid_joint_relation_constraints(path: Path) -> list[dict[str, Any]]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    safety = raw.get("safety", {}) or {}
    sysid = safety.get("sysid", {}) or {}
    constraints = sysid.get("joint_relation_constraints", []) or []
    if not isinstance(constraints, list):
        raise ValueError("safety.sysid.joint_relation_constraints must be a list")
    parsed: list[dict[str, Any]] = []
    for index, item in enumerate(constraints):
        if not isinstance(item, dict):
            raise ValueError(
                f"safety.sysid.joint_relation_constraints[{index}] must be a mapping"
            )
        name = item.get("name", f"joint_relation_{index + 1}")
        left_joint = int(item["left_joint"])
        right_joint = int(item["right_joint"])
        min_delta = float(item["min_delta_rad"])
        max_delta = float(item["max_delta_rad"])
        if left_joint <= 0 or right_joint <= 0:
            raise ValueError("joint relation indices are 1-based and must be positive")
        if min_delta > max_delta:
            raise ValueError(
                f"safety.sysid.joint_relation_constraints[{index}] has min > max"
            )
        parsed.append(
            {
                "name": str(name),
                "left_joint": left_joint,
                "right_joint": right_joint,
                "min_delta_rad": min_delta,
                "max_delta_rad": max_delta,
            }
        )
    return parsed


def read_sysid_oed_config(path: Path) -> dict[str, Any]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    safety = raw.get("safety", {}) or {}
    sysid = safety.get("sysid", {}) or {}
    oed = sysid.get("oed", {}) or {}
    if not isinstance(oed, dict):
        raise ValueError("safety.sysid.oed must be a mapping")
    n_wps = int(oed.get("n_wps", 5))
    stack_reps = int(oed.get("stack_reps", 1))
    ipopt_max_iterations = int(oed.get("ipopt_max_iterations", 200))
    condition_number_threshold = float(
        oed.get("condition_number_threshold", 1000.0)
    )
    if n_wps < 2:
        raise ValueError("safety.sysid.oed.n_wps must be >= 2")
    if stack_reps < 1:
        raise ValueError("safety.sysid.oed.stack_reps must be >= 1")
    if ipopt_max_iterations < 1:
        raise ValueError("safety.sysid.oed.ipopt_max_iterations must be >= 1")
    if condition_number_threshold <= 0.0:
        raise ValueError(
            "safety.sysid.oed.condition_number_threshold must be positive"
        )
    return {
        "n_wps": n_wps,
        "stack_reps": stack_reps,
        "ipopt_max_iterations": ipopt_max_iterations,
        "condition_number_threshold": condition_number_threshold,
    }


def evaluate_sysid_joint_relation_samples(
    safe_config_path: Path,
    *,
    samples: list[tuple[float, ...]],
) -> dict[str, Any]:
    constraints = read_sysid_joint_relation_constraints(safe_config_path)
    violations: list[dict[str, Any]] = []
    for sample_index, sample in enumerate(samples):
        for constraint in constraints:
            left_index = int(constraint["left_joint"]) - 1
            right_index = int(constraint["right_joint"]) - 1
            if left_index >= len(sample) or right_index >= len(sample):
                violations.append(
                    {
                        "sample_index": sample_index,
                        "constraint": constraint["name"],
                        "reason": "joint_index_out_of_range",
                    }
                )
                continue
            delta = float(sample[left_index]) - float(sample[right_index])
            min_delta = float(constraint["min_delta_rad"])
            max_delta = float(constraint["max_delta_rad"])
            if min_delta <= delta <= max_delta:
                continue
            violations.append(
                {
                    "sample_index": sample_index,
                    "constraint": constraint["name"],
                    "left_joint": constraint["left_joint"],
                    "right_joint": constraint["right_joint"],
                    "delta_rad": delta,
                    "min_delta_rad": min_delta,
                    "max_delta_rad": max_delta,
                }
            )
    return {
        "status": "fail" if violations else "pass",
        "constraints": constraints,
        "violations": violations,
    }


def _read_urdf_motion_limits(
    urdf_path: Path,
    *,
    active_joints: list[str],
) -> list[dict[str, float]]:
    root = ET.parse(urdf_path).getroot()
    limits_by_name: dict[str, dict[str, float]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        if name is None:
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        limits_by_name[name] = {
            "lower": float(limit.attrib.get("lower", "-3.141592653589793")),
            "upper": float(limit.attrib.get("upper", "3.141592653589793")),
            "velocity": float(limit.attrib.get("velocity", "1.0")),
            "effort": float(limit.attrib.get("effort", "100.0")),
        }
    missing = [name for name in active_joints if name not in limits_by_name]
    if missing:
        raise ValueError("URDF missing active joint limits: " + ", ".join(missing))
    return [limits_by_name[name] for name in active_joints]


def _fallback_rows(
    request: object,
    *,
    timing: dict[str, Any] | None = None,
) -> list[dict[str, str]]:
    duration_s = (
        float(timing["effective_duration_s"])
        if timing is not None and "effective_duration_s" in timing
        else float(request.duration_s)
    )
    sample_count = _sample_count_for_duration(
        duration_s,
        sample_hz=float(request.sample_hz),
    )
    rows: list[dict[str, str]] = []
    for sample_index in range(sample_count):
        t = sample_index / float(request.sample_hz)
        u = sample_index / max(1, sample_count - 1)
        coefficients = _fallback_coefficients(str(request.profile_name), u, int(request.dof))
        row = {"time_s": f"{t:.6f}"}
        for joint_index in range(int(request.dof)):
            q = float(request.q_center[joint_index])
            q += float(request.amplitude_rad) * coefficients[joint_index]
            row[f"q_cmd_{joint_index + 1}"] = f"{q:.6f}"
        rows.append(row)
    return rows


def _candidate_rows(path: Path, *, dof: int, sample_hz: float) -> CandidateTrajectory:
    if not path.exists():
        raise ValueError(f"candidate trajectory not found: {path}")
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        missing_columns = [
            column
            for column in _required_candidate_columns(dof)
            if column not in (reader.fieldnames or [])
        ]
        if missing_columns:
            raise ValueError(
                "candidate trajectory missing columns: "
                + ", ".join(missing_columns)
            )
        rows = [
            {
                column: _format_candidate_value(row[column], column=column)
                for column in _required_candidate_columns(dof)
            }
            for row in reader
        ]
    if not rows:
        raise ValueError(f"candidate trajectory has no rows: {path}")
    return CandidateTrajectory(
        rows=_resample_candidate_rows(rows, dof=dof, sample_hz=sample_hz),
        raw_sample_count=len(rows),
    )


def _resample_candidate_rows(
    rows: list[dict[str, str]],
    *,
    dof: int,
    sample_hz: float,
) -> list[dict[str, str]]:
    if len(rows) <= 1:
        return [_row_with_normalized_time(rows[0], dof=dof, time_s=0.0)]
    if sample_hz <= 0.0:
        raise ValueError("sample_hz must be positive")
    raw_times = np.asarray([float(row["time_s"]) for row in rows], dtype=float)
    if np.any(np.diff(raw_times) <= 0.0):
        raise ValueError("candidate trajectory time_s must be strictly increasing")
    normalized_times = raw_times - raw_times[0]
    duration_s = float(normalized_times[-1])
    _validate_duration_on_execution_grid(duration_s, sample_hz=sample_hz)
    sample_count = _sample_count_for_duration(duration_s, sample_hz=sample_hz)
    target_times = np.asarray(
        [index / sample_hz for index in range(sample_count)],
        dtype=float,
    )
    q_matrix = np.asarray(
        [
            [float(row[f"q_cmd_{joint_index + 1}"]) for joint_index in range(dof)]
            for row in rows
        ],
        dtype=float,
    )
    resampled: list[dict[str, str]] = []
    for time_s in target_times:
        row = {"time_s": f"{float(time_s):.6f}"}
        for joint_index in range(dof):
            value = np.interp(
                float(time_s),
                normalized_times,
                q_matrix[:, joint_index],
            )
            row[f"q_cmd_{joint_index + 1}"] = f"{float(value):.6f}"
        resampled.append(row)
    return resampled


def _validate_duration_on_execution_grid(duration_s: float, *, sample_hz: float) -> None:
    cycles = duration_s * sample_hz
    nearest_cycle = round(cycles)
    if abs(cycles - nearest_cycle) <= 1e-6:
        return
    sample_period_s = 1.0 / sample_hz
    raise ValueError(
        "candidate trajectory duration must align with execution sample period "
        f"{sample_period_s:.6f}s; got duration {duration_s:.6f}s"
    )


def _row_with_normalized_time(
    row: dict[str, str],
    *,
    dof: int,
    time_s: float,
) -> dict[str, str]:
    normalized = {"time_s": f"{time_s:.6f}"}
    for joint_index in range(dof):
        normalized[f"q_cmd_{joint_index + 1}"] = _format_candidate_value(
            row[f"q_cmd_{joint_index + 1}"],
            column=f"q_cmd_{joint_index + 1}",
        )
    return normalized


def _required_candidate_columns(dof: int) -> list[str]:
    return ["time_s"] + [f"q_cmd_{index + 1}" for index in range(dof)]


def _format_candidate_value(value: str, *, column: str) -> str:
    numeric = float(value)
    if column == "time_s":
        return f"{numeric:.6f}"
    return f"{numeric:.6f}"


def _fallback_coefficients(profile_name: str, u: float, dof: int) -> tuple[float, ...]:
    phase = 2.0 * math.pi * u
    if profile_name == "friction_sweep":
        active_joint = min(dof - 1, int(u * dof))
        local = (u * dof) - active_joint
        values = [0.0] * dof
        values[active_joint] = math.sin(2.0 * math.pi * local)
        return tuple(values)
    if profile_name == "gravity_sweep":
        envelope = math.sin(math.pi * u) ** 2
        values = [
            1.60 * math.sin(phase),
            1.20 * math.sin(phase + 0.4),
            1.20 * math.sin(phase + 0.4),
            0.70 * math.sin(phase + 1.2),
            0.50 * math.sin(phase + 1.8),
            0.40 * math.sin(phase + 2.4),
        ]
        return tuple(envelope * value for value in _fit_dof(values, dof))
    envelope = math.sin(math.pi * u) ** 2
    values = [
        0.80 * math.sin(phase),
        0.55 * math.sin(phase + 0.45),
        0.55 * math.sin(phase + 0.45),
        0.45 * math.sin(2.0 * phase + 0.9),
        0.35 * math.sin(3.0 * phase + 1.4),
        0.30 * math.sin(2.0 * phase + 2.1),
    ]
    return tuple(envelope * value for value in _fit_dof(values, dof))


def _fit_dof(values: list[float], dof: int) -> tuple[float, ...]:
    if dof <= len(values):
        return tuple(values[:dof])
    return tuple(values + [0.0] * (dof - len(values)))


def _sample_count(request: object) -> int:
    return _sample_count_for_duration(
        float(request.duration_s),
        sample_hz=float(request.sample_hz),
    )


def _sample_count_for_duration(duration_s: float, *, sample_hz: float) -> int:
    return int(round(float(duration_s) * float(sample_hz))) + 1


def _module_status(module_name: str) -> dict[str, str]:
    if module_name in sys.modules:
        return {"status": "available", "module": module_name}
    try:
        spec = importlib.util.find_spec(module_name)
    except ValueError:
        spec = None
    return {
        "status": "available" if spec else "missing",
        "module": module_name,
    }


def _pinocchio_regressor_score(
    request: object,
    *,
    rows: list[dict[str, str]],
    backend_status: dict[str, str],
) -> dict[str, Any]:
    if backend_status["status"] != "available":
        return {
            "backend": "pinocchio",
            "status": "not_evaluated",
            "reason": "pinocchio module is not importable in this environment",
        }
    try:
        pinocchio = importlib.import_module("pinocchio")
        urdf_path = _resolve_path(str(request.urdf_path))
        model = pinocchio.buildModelFromUrdf(str(urdf_path))
        data = model.createData()
        q_matrix = _q_matrix_from_rows(rows, dof=int(request.dof))
        dt = 1.0 / float(request.sample_hz)
        dq_matrix = _finite_difference(q_matrix, dt=dt)
        ddq_matrix = _finite_difference(dq_matrix, dt=dt)
        regressors = []
        for q, dq, ddq in zip(q_matrix, dq_matrix, ddq_matrix):
            q_pin = _fit_vector(q, int(model.nq))
            dq_pin = _fit_vector(dq, int(model.nv))
            ddq_pin = _fit_vector(ddq, int(model.nv))
            regressor = pinocchio.computeJointTorqueRegressor(
                model,
                data,
                q_pin,
                dq_pin,
                ddq_pin,
            )
            if regressor is None:
                regressor = data.jointTorqueRegressor
            regressors.append(np.asarray(regressor, dtype=float))
        y_matrix = np.vstack(regressors)
        condition = _matrix_condition(y_matrix)
        return {
            "backend": "pinocchio",
            "status": "computed",
            "objective": "effective_condition_number",
            **condition,
        }
    except Exception as exc:  # pragma: no cover - user-facing diagnostic.
        return {
            "backend": "pinocchio",
            "status": "failed",
            "reason": str(exc),
        }


def _resolve_path(path_text: str) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    return Path.cwd() / path


def _q_matrix_from_rows(rows: list[dict[str, str]], *, dof: int) -> np.ndarray:
    return np.asarray(
        [
            [float(row[f"q_cmd_{joint_index + 1}"]) for joint_index in range(dof)]
            for row in rows
        ],
        dtype=float,
    )


def _finite_difference(values: np.ndarray, *, dt: float) -> np.ndarray:
    if values.shape[0] <= 1:
        return np.zeros_like(values)
    edge_order = 2 if values.shape[0] > 2 else 1
    return np.gradient(values, dt, axis=0, edge_order=edge_order)


def _fit_vector(values: np.ndarray, size: int) -> np.ndarray:
    if values.size == size:
        return np.asarray(values, dtype=float)
    fitted = np.zeros(size, dtype=float)
    fitted[: min(size, values.size)] = values[: min(size, values.size)]
    return fitted


def _matrix_condition(y_matrix: np.ndarray) -> dict[str, Any]:
    singular_values = np.linalg.svd(y_matrix, compute_uv=False)
    if singular_values.size == 0:
        return {
            "row_count": int(y_matrix.shape[0]),
            "column_count": int(y_matrix.shape[1]),
            "rank": 0,
            "effective_condition_number": float("inf"),
        }
    tolerance = np.finfo(float).eps * max(y_matrix.shape) * singular_values[0]
    nonzero_singular_values = singular_values[singular_values > tolerance]
    rank = int(nonzero_singular_values.size)
    condition_number = (
        float(nonzero_singular_values[0] / nonzero_singular_values[-1])
        if rank
        else float("inf")
    )
    return {
        "row_count": int(y_matrix.shape[0]),
        "column_count": int(y_matrix.shape[1]),
        "rank": rank,
        "effective_condition_number": condition_number,
    }


def _figaroh_status() -> dict[str, str]:
    status = _module_status("figaroh")
    if status["status"] == "available":
        return status
    source_path = _figaroh_base_optimal_source()
    if source_path.exists():
        return {
            "status": "vendored_source",
            "module": "figaroh",
            "source_path": str(source_path),
        }
    return status


def _figaroh_base_optimal_source() -> Path:
    return (
        Path(__file__).resolve().parents[2]
        / "vendor"
        / "figaroh"
        / "src"
        / "figaroh"
        / "optimal"
        / "base_optimal_trajectory.py"
    )


def _requested_profile_role(profile_name: str) -> str:
    if profile_name == "fourier_multisine":
        return "oed"
    if profile_name == "friction_sweep":
        return "friction_probe"
    return "gravity_probe"


def _fallback_profile_role(profile_name: str) -> str:
    if profile_name == "friction_sweep":
        return "friction_smoke_test"
    if profile_name == "gravity_sweep":
        return "gravity_smoke_test"
    return "smoke_test"
