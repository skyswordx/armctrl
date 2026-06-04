from __future__ import annotations

from dataclasses import dataclass
import csv
import json
import math
from pathlib import Path
from typing import Iterable

from armctrl.limits import LimitDecision, UrdfJointLimits, evaluate_joint_limits
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
                "hardware_required_for_execute": True,
            },
        )

    def write_plan(self, request: SysIdPlanRequest) -> SysIdPlan:
        plan = self.plan(request.profile_name, execute=False)
        request.output_dir.mkdir(parents=True, exist_ok=True)
        trajectory_path = request.output_dir / "planned_trajectory.csv"
        manifest_path = request.output_dir / "manifest.json"
        limit_decision = evaluate_joint_limits(
            UrdfJointLimits.from_urdf(Path(request.urdf_path)),
            q_center=request.q_center,
            amplitude_rad=request.amplitude_rad,
        )
        safe_config = WorkspaceSafetyConfig.from_yaml(Path(request.safe_config_path))
        parameter_decision = _evaluate_sysid_parameters(request, safe_config)
        rows = trajectory_rows(request)
        workspace_decision = evaluate_workspace_fk_clearance(
            Path(request.urdf_path),
            safe_config,
            samples=_q_samples_from_rows(rows, dof=request.dof),
        )
        safety_allowed = (
            plan.safety.allowed
            and parameter_decision.status == "pass"
            and limit_decision.status == "pass"
            and workspace_decision.status == "pass"
        )

        with trajectory_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

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
                "reason": _safety_reason(
                    plan_reason=plan.safety.reason,
                    limit_status=limit_decision.status,
                    workspace_status=workspace_decision.status,
                ),
                "checks": {
                    "urdf_limit_check": {
                        "status": limit_decision.status,
                        "violations": limit_decision.violations,
                    },
                    "sysid_parameter_check": {
                        "status": parameter_decision.status,
                        "violations": parameter_decision.violations,
                    },
                    "workspace_clearance_check": {
                        "status": workspace_decision.status,
                        "method": workspace_decision.method,
                        "violations": workspace_decision.violations,
                    },
                    "hardware_execution": "not_requested",
                },
            },
            "handoff": plan.handoff,
            "artifacts": {
                "planned_trajectory": str(trajectory_path),
                "manifest": str(manifest_path),
            },
        }
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return SysIdPlan(
            schema=plan.schema,
            profile=plan.profile,
            safety=plan.safety,
            handoff=plan.handoff,
            artifacts={
                "planned_trajectory": str(trajectory_path),
                "manifest": str(manifest_path),
            },
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
                "workspace_clearance_check": {
                    "status": workspace_decision.status,
                    "method": workspace_decision.method,
                    "violation_count": len(workspace_decision.violations),
                },
            },
        )


def trajectory_rows(request: SysIdPlanRequest) -> list[dict[str, str]]:
    sample_count = int(round(request.duration_s * request.sample_hz)) + 1
    rows: list[dict[str, str]] = []
    for sample_index in range(sample_count):
        t = sample_index / request.sample_hz
        phase = 2.0 * math.pi * (t / request.duration_s if request.duration_s else 0.0)
        row = {"time_s": f"{t:.6f}"}
        for joint_index in range(request.dof):
            center = request.q_center[joint_index]
            offset = request.amplitude_rad * math.sin(phase) if joint_index == 0 else 0.0
            row[f"q_cmd_{joint_index + 1}"] = f"{center + offset:.6f}"
        rows.append(row)
    return rows


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
    limit_status: str,
    workspace_status: str,
) -> str:
    # Detailed violations are recorded in the manifest; this message stays compact
    # for CLI consumers that only need the first gate reason.
    if limit_status == "fail":
        return "planned trajectory violates URDF joint limits"
    if workspace_status == "fail":
        return "planned trajectory violates workspace clearance proxy"
    return plan_reason


def _evaluate_sysid_parameters(
    request: SysIdPlanRequest,
    config: WorkspaceSafetyConfig,
) -> LimitDecision:
    violations: list[dict[str, object]] = []
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
    if request.amplitude_rad > config.max_sysid_amplitude_rad:
        violations.append(
            {
                "check": "max_sysid_amplitude_rad",
                "value": request.amplitude_rad,
                "maximum": config.max_sysid_amplitude_rad,
            }
        )
    return LimitDecision(
        status="fail" if violations else "pass",
        violations=violations,
    )
