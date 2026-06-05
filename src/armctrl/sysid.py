from __future__ import annotations

from dataclasses import dataclass
import csv
import json
from pathlib import Path
from typing import Any, Iterable

from armctrl.limits import (
    LimitDecision,
    UrdfJointLimits,
    evaluate_joint_limit_samples,
)
from armctrl.simulation import TrajectoryPreviewer
from armctrl.sysid_trajectory_backend import (
    evaluate_sysid_joint_relation_samples,
    plan_sysid_trajectory,
    read_sysid_profile_safety,
)
from armctrl.workspace import WorkspaceSafetyConfig, evaluate_workspace_fk_clearance


@dataclass(frozen=True)
class SysIdSafety:
    allowed: bool
    reason: str

    def to_json(self) -> dict[str, object]:
        return {"allowed": self.allowed, "reason": self.reason}


@dataclass(frozen=True)
class SysIdProfile:
    name: str
    purpose: str
    tool_boundary: dict[str, str]

    def to_json(self) -> dict[str, object]:
        return {
            "name": self.name,
            "purpose": self.purpose,
            "tool_boundary": self.tool_boundary,
        }


@dataclass(frozen=True)
class SysIdPlan:
    schema: str
    profile: SysIdProfile
    safety: SysIdSafety
    handoff: dict[str, object]
    artifacts: dict[str, str] | None = None
    artifact_safety: dict[str, object] | None = None
    trajectory_backend: dict[str, Any] | None = None

    def to_json(self) -> dict[str, object]:
        payload: dict[str, object] = {
            "schema": self.schema,
            "profile": self.profile.to_json(),
            "safety": self.safety.to_json(),
            "handoff": self.handoff,
        }
        if self.artifacts is not None:
            payload["artifacts"] = self.artifacts
        if self.artifact_safety is not None:
            payload["artifact_safety"] = self.artifact_safety
        if self.trajectory_backend is not None:
            payload["trajectory_backend"] = self.trajectory_backend
        return payload


@dataclass(frozen=True)
class SysIdPlanRequest:
    profile_name: str
    dof: int
    sample_hz: float
    duration_s: float
    amplitude_rad: float
    q_center: tuple[float, ...]
    urdf_path: str
    safe_config_path: str
    output_dir: Path
    render_path: Path | None = None
    candidate_trajectory_path: Path | None = None
    trajectory_command_argv: tuple[str, ...] | None = None
    trajectory_command_timeout_s: float | None = None


class SysIdPlanner:
    def __init__(self, profiles: Iterable[SysIdProfile]) -> None:
        self._profiles = {profile.name: profile for profile in profiles}

    @classmethod
    def default(cls) -> "SysIdPlanner":
        common_boundary = {
            "trajectory_optimization": "FIGAROH",
            "regressor_solver": "Pinocchio",
            "base_parameter_extraction": "FIGAROH",
            "dataset_interface": "LeRobot-compatible metadata",
        }
        return cls(
            [
                SysIdProfile(
                    name="fourier_multisine",
                    purpose="dynamic coupled excitation for offline rigid-body identification",
                    tool_boundary=common_boundary,
                ),
                SysIdProfile(
                    name="friction_sweep",
                    purpose="joint friction and torque bias estimation",
                    tool_boundary=common_boundary,
                ),
                SysIdProfile(
                    name="gravity_sweep",
                    purpose="quasi-static gravity and payload residual identification",
                    tool_boundary=common_boundary,
                ),
            ]
        )

    def list_profiles(self) -> list[SysIdProfile]:
        return [self._profiles[name] for name in sorted(self._profiles)]

    def plan(self, profile_name: str, *, execute: bool) -> SysIdPlan:
        profile = self._profiles[profile_name]
        safety = (
            SysIdSafety(
                allowed=False,
                reason="sysid execution runner is not implemented in clean rebuild",
            )
            if execute
            else SysIdSafety(allowed=True, reason="plan-only sysid preview")
        )
        return SysIdPlan(
            schema="armctrl.sysid_plan.v1",
            profile=profile,
            safety=safety,
            handoff={
                "dataset_contract": "lerobot-compatible",
                "solver_backends": ["pinocchio", "figaroh"],
                "trajectory_optimizer": "figaroh",
                "hardware_required_for_execute": True,
            },
        )

    def write_plan(self, request: SysIdPlanRequest) -> SysIdPlan:
        plan = self.plan(request.profile_name, execute=False)
        request.output_dir.mkdir(parents=True, exist_ok=True)
        trajectory_path = request.output_dir / "planned_trajectory.csv"
        execution_trajectory_path = request.output_dir / "execution_trajectory.csv"
        manifest_path = request.output_dir / "manifest.json"
        preview_path = request.output_dir / "trajectory_preview.json"
        preview_html_path = request.output_dir / "preview.html"
        render_path = request.render_path
        safe_config = WorkspaceSafetyConfig.from_yaml(Path(request.safe_config_path))
        parameter_decision = _evaluate_sysid_parameters(request, safe_config)
        trajectory_plan = plan_sysid_trajectory(request)
        rows = trajectory_plan.rows
        execution_rows = trajectory_plan.execution_rows or rows
        q_samples = _q_samples_from_rows(execution_rows, dof=request.dof)
        limit_decision = evaluate_joint_limit_samples(
            UrdfJointLimits.from_urdf(Path(request.urdf_path)),
            samples=q_samples,
        )
        step_decision = _evaluate_trajectory_steps(
            execution_rows,
            dof=request.dof,
            max_joint_step_rad=safe_config.max_joint_step_rad,
        )
        relation_decision = evaluate_sysid_joint_relation_samples(
            Path(request.safe_config_path),
            samples=q_samples,
            profile_name=request.profile_name,
        )
        workspace_decision = evaluate_workspace_fk_clearance(
            Path(request.urdf_path),
            safe_config,
            samples=q_samples,
        )
        safety_allowed = (
            plan.safety.allowed
            and parameter_decision.status == "pass"
            and step_decision.status == "pass"
            and relation_decision["status"] == "pass"
            and limit_decision.status == "pass"
            and workspace_decision.status == "pass"
        )

        with trajectory_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        with execution_trajectory_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(execution_rows[0]))
            writer.writeheader()
            writer.writerows(execution_rows)

        previewer = TrajectoryPreviewer()
        preview = previewer.preview(
            trajectory_path=execution_trajectory_path,
            urdf_path=Path(request.urdf_path),
            safe_config_path=Path(request.safe_config_path),
            render_path=preview_html_path,
        )
        extra_render: dict[str, object] | None = None
        if render_path is not None and render_path != preview_html_path:
            extra_preview = previewer.preview(
                trajectory_path=execution_trajectory_path,
                urdf_path=Path(request.urdf_path),
                safe_config_path=Path(request.safe_config_path),
                render_path=render_path,
            )
            raw_render = extra_preview.get("render")
            if isinstance(raw_render, dict):
                extra_render = raw_render
        preview_path.write_text(
            json.dumps(preview, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        simulation_status = (
            "pass" if preview["safety"]["allowed"] is True else "fail"
        )
        safety_allowed = safety_allowed and simulation_status == "pass"

        safety_reason = _safety_reason(
            plan_reason=plan.safety.reason,
            parameter_status=parameter_decision.status,
            step_status=step_decision.status,
            relation_status=str(relation_decision["status"]),
            limit_status=limit_decision.status,
            workspace_status=workspace_decision.status,
            simulation_status=simulation_status,
        )
        artifacts = {
            "planned_trajectory": str(trajectory_path),
            "execution_trajectory": str(execution_trajectory_path),
            "trajectory_preview": str(preview_path),
            "preview_html": str(preview_html_path),
            "manifest": str(manifest_path),
        }
        if extra_render is not None and render_path is not None:
            artifacts["trajectory_render"] = str(render_path)
        manifest = {
            "schema": "armctrl.ident_plan_manifest.v1",
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
            "safety": {
                "allowed": safety_allowed,
                "reason": safety_reason,
                "checks": {
                    "urdf_limit_check": {
                        "status": limit_decision.status,
                        "violations": limit_decision.violations,
                    },
                    "sysid_parameter_check": {
                        "status": parameter_decision.status,
                        "violations": parameter_decision.violations,
                    },
                    "trajectory_step_check": {
                        "status": step_decision.status,
                        "trajectory": "execution_trajectory",
                        "violations": step_decision.violations,
                    },
                    "joint_relation_check": relation_decision,
                    "workspace_clearance_check": {
                        "status": workspace_decision.status,
                        "method": workspace_decision.method,
                        "violations": workspace_decision.violations,
                    },
                    "simulation_check": {
                        "status": simulation_status,
                        "backend": preview["backend"],
                        "zone_check": preview["safety"]["zone_check"],
                        "clearance_check": preview["safety"]["clearance_check"],
                    },
                    "hardware_execution": "not_requested",
                },
            },
            "trajectory_backend": trajectory_plan.backend,
            "handoff": plan.handoff,
            "artifacts": artifacts,
        }
        if extra_render is not None:
            manifest["render"] = extra_render
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return SysIdPlan(
            schema=plan.schema,
            profile=plan.profile,
            safety=SysIdSafety(allowed=safety_allowed, reason=safety_reason),
            handoff=plan.handoff,
            artifacts=artifacts,
            artifact_safety={
                "allowed": safety_allowed,
                "urdf_limit_check": {
                    "status": limit_decision.status,
                    "violation_count": len(limit_decision.violations),
                },
                "sysid_parameter_check": {
                    "status": parameter_decision.status,
                    "violation_count": len(parameter_decision.violations),
                },
                "trajectory_step_check": {
                    "status": step_decision.status,
                    "trajectory": "execution_trajectory",
                    "violation_count": len(step_decision.violations),
                },
                "joint_relation_check": {
                    "status": relation_decision["status"],
                    "violation_count": len(relation_decision["violations"]),
                },
                "workspace_clearance_check": {
                    "status": workspace_decision.status,
                    "method": workspace_decision.method,
                    "violation_count": len(workspace_decision.violations),
                },
                "simulation_check": {
                    "status": simulation_status,
                    "backend": preview["backend"]["selected"],
                    "violation_count": len(
                        preview["safety"]["zone_check"]["violations"]
                    )
                    + len(preview["safety"]["clearance_check"]["violations"]),
                },
            },
            trajectory_backend=trajectory_plan.backend,
        )


def trajectory_rows(request: SysIdPlanRequest) -> list[dict[str, str]]:
    return plan_sysid_trajectory(request, write_artifacts=False).rows


def _q_samples_from_rows(
    rows: list[dict[str, str]],
    *,
    dof: int,
) -> list[tuple[float, ...]]:
    return [
        tuple(float(row[f"q_cmd_{joint_index + 1}"]) for joint_index in range(dof))
        for row in rows
    ]


def _safety_reason(
    *,
    plan_reason: str,
    parameter_status: str,
    step_status: str,
    relation_status: str,
    limit_status: str,
    workspace_status: str,
    simulation_status: str,
) -> str:
    # Detailed violations are recorded in the manifest; this message stays compact
    # for CLI consumers that only need the first gate reason.
    if parameter_status == "fail":
        return "planned sysid parameters exceed safety config"
    if step_status == "fail":
        return "planned trajectory exceeds max joint step"
    if relation_status == "fail":
        return "planned trajectory violates joint relation constraints"
    if limit_status == "fail":
        return "planned trajectory violates URDF joint limits"
    if workspace_status == "fail":
        return "planned trajectory violates workspace clearance proxy"
    if simulation_status == "fail":
        return "planned trajectory failed simulation safety preview"
    return plan_reason


def _evaluate_sysid_parameters(
    request: SysIdPlanRequest,
    config: WorkspaceSafetyConfig,
) -> LimitDecision:
    violations: list[dict[str, object]] = []
    profile_safety = read_sysid_profile_safety(
        Path(request.safe_config_path),
        profile_name=request.profile_name,
    )
    max_sysid_amplitude_rad = float(profile_safety["max_sysid_amplitude_rad"])
    if request.duration_s > config.max_sysid_duration_s:
        violations.append(
            {
                "check": "max_sysid_duration_s",
                "value": request.duration_s,
                "maximum": config.max_sysid_duration_s,
            }
        )
    if request.sample_hz > config.max_sysid_sample_hz:
        violations.append(
            {
                "check": "max_sysid_sample_hz",
                "value": request.sample_hz,
                "maximum": config.max_sysid_sample_hz,
            }
        )
    if request.amplitude_rad > max_sysid_amplitude_rad:
        violations.append(
            {
                "check": "max_sysid_amplitude_rad",
                "value": request.amplitude_rad,
                "maximum": max_sysid_amplitude_rad,
                "profile": request.profile_name,
            }
        )
    return LimitDecision(
        status="fail" if violations else "pass",
        violations=violations,
    )


def _evaluate_trajectory_steps(
    rows: list[dict[str, str]],
    *,
    dof: int,
    max_joint_step_rad: float,
) -> LimitDecision:
    violations: list[dict[str, object]] = []
    for row_index, (previous, current) in enumerate(zip(rows, rows[1:]), start=1):
        for joint_index in range(dof):
            previous_q = float(previous[f"q_cmd_{joint_index + 1}"])
            current_q = float(current[f"q_cmd_{joint_index + 1}"])
            step_rad = abs(current_q - previous_q)
            if step_rad > max_joint_step_rad:
                violations.append(
                    {
                        "check": "max_joint_step_rad",
                        "sample_index": row_index,
                        "joint": joint_index + 1,
                        "value": step_rad,
                        "maximum": max_joint_step_rad,
                    }
                )
    return LimitDecision(
        status="fail" if violations else "pass",
        violations=violations,
    )
