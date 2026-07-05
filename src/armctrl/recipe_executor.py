from __future__ import annotations

from dataclasses import dataclass

from armctrl.safety import SafetyDecision


@dataclass(frozen=True)
class RecipeExecutorGate:
    status: str
    state: str
    hardware_backend: str
    safety_gate_required: bool
    reason: str
    action: str | None = None

    def to_json(self) -> dict[str, object]:
        payload: dict[str, object] = {
            "status": self.status,
            "state": self.state,
            "hardware_backend": self.hardware_backend,
            "safety_gate_required": self.safety_gate_required,
            "reason": self.reason,
        }
        if self.action is not None:
            payload["action"] = self.action
        return payload


class RecipeExecutor:
    def __init__(self, *, hardware_backend: str = "not_configured") -> None:
        self._hardware_backend = hardware_backend

    def evaluate(self, safety: SafetyDecision) -> RecipeExecutorGate:
        if not safety.allowed:
            return RecipeExecutorGate(
                status="blocked",
                state="idle",
                hardware_backend=self._hardware_backend,
                safety_gate_required=True,
                reason=safety.reason,
            )
        if self._hardware_backend == "not_configured":
            return RecipeExecutorGate(
                status="blocked",
                state="idle",
                hardware_backend=self._hardware_backend,
                safety_gate_required=True,
                reason="hardware backend is not configured",
            )
        if self._hardware_backend != "sim":
            return RecipeExecutorGate(
                status="blocked",
                state="idle",
                hardware_backend=self._hardware_backend,
                safety_gate_required=True,
                reason="execution backend contract exists but only sim is enabled in clean rebuild",
            )
        return RecipeExecutorGate(
            status="ready",
            state="idle",
            hardware_backend=self._hardware_backend,
            safety_gate_required=True,
            reason="safety gate passed and backend configured",
        )

    def status(self) -> RecipeExecutorGate:
        return RecipeExecutorGate(
            status="ok",
            state="idle",
            hardware_backend=self._hardware_backend,
            safety_gate_required=True,
            reason="no active recipe execution session",
        )

    def cancel(self) -> RecipeExecutorGate:
        return RecipeExecutorGate(
            status="ok",
            state="idle",
            hardware_backend=self._hardware_backend,
            safety_gate_required=True,
            reason="no active recipe execution session",
            action="no_active_session",
        )
