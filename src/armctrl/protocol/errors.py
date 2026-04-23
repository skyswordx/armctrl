from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from armctrl.protocol.enums import ErrorCode


@dataclass(frozen=True)
class ArmctrlError(Exception):
    """项目内统一错误对象。

    这里继承 Exception，是为了既能抛异常，也能序列化成响应体。
    """

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
        # 这个反序列化入口主要服务于 JSON 响应恢复和测试。
        if not payload:
            return None
        return cls(
            code=ErrorCode(payload["code"]),
            message=str(payload["message"]),
            detail=payload.get("detail"),
        )


@dataclass(frozen=True)
class ValidationResult:
    """安全检查或权限检查的结果对象。"""

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
        # 这里把创建错误对象的样板代码集中掉，调用方更干净。
        return cls(False, ArmctrlError(code, message, detail))
