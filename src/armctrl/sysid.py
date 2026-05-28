from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


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

    def to_json(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "profile": self.profile.to_json(),
            "safety": self.safety.to_json(),
            "handoff": self.handoff,
        }


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
