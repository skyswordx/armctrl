from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path


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


@dataclass(frozen=True)
class OnlineAuditRequest:
    update: ParameterUpdate
    window_start_s: float
    window_end_s: float
    residual_before: float
    residual_after: float
    saturation_status: str
    rollback_target: str
    output_path: Path


@dataclass(frozen=True)
class OnlineAuditResult:
    schema: str
    decision: OnlineDecision
    artifacts: dict[str, str]
    policy: dict[str, object]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "decision": {
                "allowed": self.decision.allowed,
                "mode": self.decision.mode,
                "reason": self.decision.reason,
            },
            "allowed_parameter_groups": self.policy["allowed_parameter_groups"],
            "requires_manual_promotion": self.policy["requires_manual_promotion"],
            "artifacts": self.artifacts,
        }


class OnlineIdentificationAuditor:
    def __init__(self, policy: OnlineIdentificationPolicy) -> None:
        self._policy = policy

    @classmethod
    def default(cls) -> "OnlineIdentificationAuditor":
        return cls(OnlineIdentificationPolicy.default())

    def record(self, request: OnlineAuditRequest) -> OnlineAuditResult:
        decision = self._policy.evaluate(request.update)
        record = {
            "schema": "armctrl.online_identification_audit_record.v1",
            "parameter": request.update.name,
            "value": request.update.value,
            "source": request.update.source,
            "mode": decision.mode,
            "decision": {
                "allowed": decision.allowed,
                "reason": decision.reason,
            },
            "window": {
                "start_s": request.window_start_s,
                "end_s": request.window_end_s,
            },
            "residual_before": request.residual_before,
            "residual_after": request.residual_after,
            "residual_delta": round(
                request.residual_after - request.residual_before,
                12,
            ),
            "saturation_status": request.saturation_status,
            "rollback_target": request.rollback_target,
            "manual_confirmation_required": True,
        }
        request.output_path.parent.mkdir(parents=True, exist_ok=True)
        with request.output_path.open("a", encoding="utf-8") as file:
            file.write(json.dumps(record, ensure_ascii=False) + "\n")
        return OnlineAuditResult(
            schema="armctrl.online_identification_audit.v1",
            decision=decision,
            artifacts={"audit_log": str(request.output_path)},
            policy=self._policy.to_json(),
        )
