from __future__ import annotations

from enum import StrEnum


class AdapterKind(StrEnum):
    # 用 StrEnum 的原因是：
    # 1. CLI 直接使用字符串更自然；
    # 2. JSON 序列化不需要额外转换表。
    FAKE = "fake"
    SDK = "sdk"


class ArmMode(StrEnum):
    # 这些模式不是 UI 标签，而是系统状态机的稳定枚举值。
    DISCONNECTED = "disconnected"
    IDLE = "idle"
    TELEOP = "teleop"
    DAMPING = "damping"
    FAULTED = "faulted"
    MAINTENANCE = "maintenance"


class CommandStatus(StrEnum):
    # 响应状态覆盖了“收到、执行、拒绝、取消、故障”这几类语义。
    RECEIVED = "received"
    ACCEPTED = "accepted"
    RUNNING = "running"
    COMPLETED = "completed"
    REJECTED = "rejected"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"
    FAULTED = "faulted"


class ErrorCode(StrEnum):
    # 错误码和错误消息分离，方便脚本稳定匹配。
    SAFETY_REJECTED = "safety_rejected"
    SDK_UNAVAILABLE = "sdk_unavailable"
    SDK_ERROR = "sdk_error"
    INVALID_REQUEST = "invalid_request"
    HARDWARE_REQUIRED = "hardware_required"
    MAINTENANCE_REQUIRED = "maintenance_required"
    CONFIRMATION_REQUIRED = "confirmation_required"


class DebugProfileName(StrEnum):
    # profile 名称既用于 CLI 参数，也用于手柄快捷键映射。
    DAMPING = "damping"
    RESET_HOME = "reset_home"
    LOW_GAIN_PASSIVE = "low_gain_passive"
    COMPLIANCE_SLOW = "compliance_slow"
    GRAVITY_COMPENSATION_STARTUP = "gravity_compensation_startup"
