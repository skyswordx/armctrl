from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ParameterUpdate:
    name: str
    value: float
    source: str

    @property
    def group(self) -> str:
        return self.name.rsplit(".", maxsplit=1)[-1]


@dataclass(frozen=True)
class OnlineDecision:
    allowed: bool
    mode: str
    reason: str


class OnlineIdentificationPolicy:
    def __init__(self, allowed_parameter_groups: tuple[str, ...]) -> None:
        self.allowed_parameter_groups = allowed_parameter_groups
        self.mode = "shadow"

    @classmethod
    def default(cls) -> "OnlineIdentificationPolicy":
        return cls(
            (
                "torque_bias",
                "viscous_friction",
                "coulomb_friction",
                "gravity_residual",
            )
        )

    def evaluate(self, update: ParameterUpdate) -> OnlineDecision:
        if update.group not in self.allowed_parameter_groups:
            return OnlineDecision(
                allowed=False,
                mode=self.mode,
                reason="full rigid-body parameters require offline SysID",
            )
        return OnlineDecision(
            allowed=True,
            mode=self.mode,
            reason="conservative online update accepted for shadow evaluation",
        )

    def to_json(self) -> dict[str, object]:
        return {
            "schema": "armctrl.online_identification_policy.v1",
            "mode": self.mode,
            "allowed_parameter_groups": list(self.allowed_parameter_groups),
            "requires_manual_promotion": True,
            "forbidden_online_groups": ["mass", "center_of_mass", "inertia"],
        }
