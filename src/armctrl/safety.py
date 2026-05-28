from __future__ import annotations

from dataclasses import dataclass

from armctrl.recipes import Recipe


@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    reason: str
    required_backend: str

    def to_json(self) -> dict[str, object]:
        return {
            "allowed": self.allowed,
            "reason": self.reason,
            "required_backend": self.required_backend,
        }


class SafetyGate:
    def evaluate(self, recipe: Recipe, *, plan_only: bool) -> SafetyDecision:
        if recipe.moves_hardware and not plan_only:
            return SafetyDecision(
                allowed=False,
                reason="hardware recipe requires an execution backend",
                required_backend=recipe.required_backend,
            )
        return SafetyDecision(
            allowed=True,
            reason="plan-only recipe preview",
            required_backend=recipe.required_backend,
        )
