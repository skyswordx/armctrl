from __future__ import annotations

import json
import time
import uuid
from dataclasses import asdict, dataclass, field, is_dataclass
from enum import Enum
from typing import Any

from armctrl.protocol.enums import ArmMode, CommandStatus, DebugProfileName
from armctrl.protocol.errors import ArmctrlError


def _tuple_of_floats(name: str, values: tuple[float, ...], expected: int) -> tuple[float, ...]:
    if len(values) != expected:
        raise ValueError(f"{name} must contain {expected} values")
    return tuple(float(value) for value in values)


def _json_default(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, ArmctrlError):
        return value.to_dict()
    if is_dataclass(value):
        return asdict(value)
    raise TypeError(f"{type(value).__name__} is not JSON serializable")


@dataclass(frozen=True)
class EEFStateModel:
    pose_6d: tuple[float, ...]
    gripper_pos: float = 0.0
    gripper_vel: float = 0.0
    gripper_torque: float = 0.0
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "pose_6d", _tuple_of_floats("pose_6d", self.pose_6d, 6))
        object.__setattr__(self, "gripper_pos", float(self.gripper_pos))
        object.__setattr__(self, "gripper_vel", float(self.gripper_vel))
        object.__setattr__(self, "gripper_torque", float(self.gripper_torque))
        object.__setattr__(self, "timestamp", float(self.timestamp))


@dataclass(frozen=True)
class JointStateModel:
    pos: tuple[float, ...]
    vel: tuple[float, ...]
    torque: tuple[float, ...]
    gripper_pos: float = 0.0
    gripper_vel: float = 0.0
    gripper_torque: float = 0.0
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        dof = len(self.pos)
        if dof == 0:
            raise ValueError("pos must not be empty")
        object.__setattr__(self, "pos", tuple(float(value) for value in self.pos))
        object.__setattr__(self, "vel", _tuple_of_floats("vel", self.vel, dof))
        object.__setattr__(self, "torque", _tuple_of_floats("torque", self.torque, dof))
        object.__setattr__(self, "gripper_pos", float(self.gripper_pos))
        object.__setattr__(self, "gripper_vel", float(self.gripper_vel))
        object.__setattr__(self, "gripper_torque", float(self.gripper_torque))
        object.__setattr__(self, "timestamp", float(self.timestamp))


@dataclass(frozen=True)
class RobotState:
    mode: ArmMode
    joint: JointStateModel
    eef: EEFStateModel
    connected: bool
    adapter: str
    last_error: ArmctrlError | None = None


@dataclass(frozen=True)
class MoveEEFRequest:
    pose_6d: tuple[float, ...]
    gripper_pos: float = 0.0
    preview_time_s: float = 0.1
    profile: str = "teleop_slow"
    plan_only: bool = False
    command_id: str = field(default_factory=lambda: uuid.uuid4().hex)

    def __post_init__(self) -> None:
        object.__setattr__(self, "pose_6d", _tuple_of_floats("pose_6d", self.pose_6d, 6))
        object.__setattr__(self, "gripper_pos", float(self.gripper_pos))
        object.__setattr__(self, "preview_time_s", float(self.preview_time_s))


@dataclass(frozen=True)
class TeleopCommand:
    translation_m: tuple[float, ...] = (0.0, 0.0, 0.0)
    rotation_rad: tuple[float, ...] = (0.0, 0.0, 0.0)
    gripper_delta: float = 0.0
    deadman: bool = False
    source: str = "xbox"
    timestamp: float = field(default_factory=time.monotonic)
    debug_profile: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "translation_m",
            _tuple_of_floats("translation_m", self.translation_m, 3),
        )
        object.__setattr__(
            self,
            "rotation_rad",
            _tuple_of_floats("rotation_rad", self.rotation_rad, 3),
        )
        object.__setattr__(self, "gripper_delta", float(self.gripper_delta))
        object.__setattr__(self, "timestamp", float(self.timestamp))


@dataclass(frozen=True)
class DebugProfileRequest:
    name: DebugProfileName
    maintenance: bool = False
    confirm: bool = False
    plan_only: bool = False
    command_id: str = field(default_factory=lambda: uuid.uuid4().hex)

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", DebugProfileName(self.name))


@dataclass(frozen=True)
class CommandResponse:
    status: CommandStatus
    message: str
    command_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    error: ArmctrlError | None = None
    state: RobotState | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "status", CommandStatus(self.status))

    def to_dict(self) -> dict[str, Any]:
        payload = json.loads(json.dumps(asdict(self), default=_json_default))
        if self.error:
            payload["error"] = self.error.to_dict()
        return payload

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, sort_keys=True)

    @classmethod
    def from_json(cls, payload: str) -> CommandResponse:
        data = json.loads(payload)
        return cls(
            status=CommandStatus(data["status"]),
            message=data["message"],
            command_id=data["command_id"],
            error=ArmctrlError.from_dict(data.get("error")),
        )
