from __future__ import annotations

from enum import StrEnum


class AdapterKind(StrEnum):
    FAKE = "fake"
    SDK = "sdk"


class ArmMode(StrEnum):
    DISCONNECTED = "disconnected"
    IDLE = "idle"
    TELEOP = "teleop"
    DAMPING = "damping"
    FAULTED = "faulted"
    MAINTENANCE = "maintenance"


class CommandStatus(StrEnum):
    RECEIVED = "received"
    ACCEPTED = "accepted"
    RUNNING = "running"
    COMPLETED = "completed"
    REJECTED = "rejected"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"
    FAULTED = "faulted"


class ErrorCode(StrEnum):
    SAFETY_REJECTED = "safety_rejected"
    SDK_UNAVAILABLE = "sdk_unavailable"
    SDK_ERROR = "sdk_error"
    INVALID_REQUEST = "invalid_request"
    HARDWARE_REQUIRED = "hardware_required"
    MAINTENANCE_REQUIRED = "maintenance_required"
    CONFIRMATION_REQUIRED = "confirmation_required"


class DebugProfileName(StrEnum):
    DAMPING = "damping"
    RESET_HOME = "reset_home"
    LOW_GAIN_PASSIVE = "low_gain_passive"
    COMPLIANCE_SLOW = "compliance_slow"
    GRAVITY_COMPENSATION_STARTUP = "gravity_compensation_startup"
