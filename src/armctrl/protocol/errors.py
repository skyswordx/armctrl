from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from armctrl.protocol.enums import ErrorCode


@dataclass(frozen=True)
class ArmctrlError(Exception):
    code: ErrorCode
    message: str
    detail: dict[str, Any] | None = None

    def __str__(self) -> str:
        return f"{self.code.value}: {self.message}"

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {"code": self.code.value, "message": self.message}
        if self.detail:
            payload["detail"] = self.detail
        return payload

    @classmethod
    def from_dict(cls, payload: dict[str, Any] | None) -> ArmctrlError | None:
        if not payload:
            return None
        return cls(
            code=ErrorCode(payload["code"]),
            message=str(payload["message"]),
            detail=payload.get("detail"),
        )


@dataclass(frozen=True)
class ValidationResult:
    allowed: bool
    error: ArmctrlError | None = None

    @classmethod
    def ok(cls) -> ValidationResult:
        return cls(True, None)

    @classmethod
    def reject(
        cls,
        code: ErrorCode,
        message: str,
        detail: dict[str, Any] | None = None,
    ) -> ValidationResult:
        return cls(False, ArmctrlError(code, message, detail))
