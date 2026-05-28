from __future__ import annotations

from dataclasses import dataclass

from armctrl.recipes import Recipe


@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    reason: str
    required_backend: str
    dry_run: bool
    movement_allowed: bool
    risk_explanation: tuple[str, ...]

    def to_json(self) -> dict[str, object]:
        return {
            "allowed": self.allowed,
            "reason": self.reason,
            "required_backend": self.required_backend,
            "dry_run": self.dry_run,
            "movement_allowed": self.movement_allowed,
            "risk_explanation": list(self.risk_explanation),
        }


class SafetyGate:
    def evaluate(self, recipe: Recipe, *, plan_only: bool) -> SafetyDecision:
        if recipe.moves_hardware and not plan_only:
            return SafetyDecision(
                allowed=False,
                reason="hardware recipe requires an execution backend",
                required_backend=recipe.required_backend,
                dry_run=False,
                movement_allowed=False,
                risk_explanation=(
                    "recipe would move hardware",
                    "hardware backend is not configured in clean rebuild",
                    "execution remains blocked until safety and backend gates pass",
                ),
            )
        return SafetyDecision(
            allowed=True,
            reason="plan-only recipe preview",
            required_backend=recipe.required_backend,
            dry_run=plan_only,
            movement_allowed=False if recipe.moves_hardware else plan_only,
            risk_explanation=(
                "recipe would move hardware when execution is enabled",
                "current response is a dry-run preview only",
                "planned joint targets must pass backend, limit, and workspace checks before execution",
            )
            if recipe.moves_hardware
            else ("offline recipe",),
        )
