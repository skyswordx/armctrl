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
    # 这个辅助函数解决两个问题：
    # 1. 把外部传入的数值统一转成 float；
    # 2. 在 dataclass 构造时立刻校验维度，尽早失败。
    if len(values) != expected:
        raise ValueError(f"{name} must contain {expected} values")
    return tuple(float(value) for value in values)


def _json_default(value: Any) -> Any:
    # 统一处理项目里最常见的三类“非原生 JSON 对象”：
    # 枚举、领域错误、dataclass。
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, ArmctrlError):
        return value.to_dict()
    if is_dataclass(value):
        return asdict(value)
    raise TypeError(f"{type(value).__name__} is not JSON serializable")


@dataclass(frozen=True)
class EEFStateModel:
    """末端执行器状态。

    pose_6d 的含义固定为：
    `[x, y, z, roll, pitch, yaw]`
    单位分别是米和弧度。
    """

    pose_6d: tuple[float, ...]
    gripper_pos: float = 0.0
    gripper_vel: float = 0.0
    gripper_torque: float = 0.0
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        # dataclass 设为 frozen 后，必须用 object.__setattr__ 做规范化写入。
        object.__setattr__(self, "pose_6d", _tuple_of_floats("pose_6d", self.pose_6d, 6))
        object.__setattr__(self, "gripper_pos", float(self.gripper_pos))
        object.__setattr__(self, "gripper_vel", float(self.gripper_vel))
        object.__setattr__(self, "gripper_torque", float(self.gripper_torque))
        object.__setattr__(self, "timestamp", float(self.timestamp))


@dataclass(frozen=True)
class JointStateModel:
    """关节空间状态。

    这里允许不同自由度长度，但要求 pos / vel / torque 长度一致。
    """

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
        # 先确定自由度，再把其他向量校验到相同长度。
        object.__setattr__(self, "pos", tuple(float(value) for value in self.pos))
        object.__setattr__(self, "vel", _tuple_of_floats("vel", self.vel, dof))
        object.__setattr__(self, "torque", _tuple_of_floats("torque", self.torque, dof))
        object.__setattr__(self, "gripper_pos", float(self.gripper_pos))
        object.__setattr__(self, "gripper_vel", float(self.gripper_vel))
        object.__setattr__(self, "gripper_torque", float(self.gripper_torque))
        object.__setattr__(self, "timestamp", float(self.timestamp))


@dataclass(frozen=True)
class RobotState:
    """系统级机器人状态快照。

    这是 UI、CLI、测试和日志共同使用的统一状态模型。
    """

    mode: ArmMode
    joint: JointStateModel
    eef: EEFStateModel
    connected: bool
    adapter: str
    last_error: ArmctrlError | None = None


@dataclass(frozen=True)
class MoveEEFRequest:
    """末端位姿控制请求。

    这里的 request 不是 SDK 原生命令，而是业务层的标准输入。
    """

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
    """手柄或其他实时输入设备产生的增量命令。

    这里同时保存两层量：
    1. 速度量：摇杆归一化值映射到 m/s、rad/s。
    2. 本拍增量：速度量乘以 dt 后得到的 m、rad。
    """

    translation_m: tuple[float, ...] = (0.0, 0.0, 0.0)
    rotation_rad: tuple[float, ...] = (0.0, 0.0, 0.0)
    gripper_delta: float = 0.0
    translation_velocity_mps: tuple[float, ...] = (0.0, 0.0, 0.0)
    rotation_velocity_radps: tuple[float, ...] = (0.0, 0.0, 0.0)
    gripper_velocity_mps: float = 0.0
    dt_s: float = 0.01
    deadman: bool = False
    source: str = "xbox"
    timestamp: float = field(default_factory=time.monotonic)
    debug_profile: str | None = None

    def __post_init__(self) -> None:
        # 为了让下游公式更简单，这里在构造期就保证位移、速度的维度都合法。
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
        object.__setattr__(
            self,
            "translation_velocity_mps",
            _tuple_of_floats("translation_velocity_mps", self.translation_velocity_mps, 3),
        )
        object.__setattr__(
            self,
            "rotation_velocity_radps",
            _tuple_of_floats("rotation_velocity_radps", self.rotation_velocity_radps, 3),
        )
        object.__setattr__(self, "gripper_delta", float(self.gripper_delta))
        object.__setattr__(self, "gripper_velocity_mps", float(self.gripper_velocity_mps))
        object.__setattr__(self, "dt_s", float(self.dt_s))
        object.__setattr__(self, "timestamp", float(self.timestamp))


@dataclass(frozen=True)
class DebugProfileRequest:
    """调试 profile 请求。"""

    name: DebugProfileName
    maintenance: bool = False
    confirm: bool = False
    plan_only: bool = False
    command_id: str = field(default_factory=lambda: uuid.uuid4().hex)

    def __post_init__(self) -> None:
        # 这里允许外部传字符串，再统一收敛成枚举。
        object.__setattr__(self, "name", DebugProfileName(self.name))


@dataclass(frozen=True)
class CommandResponse:
    """统一命令响应。

    这是系统里最核心的返回对象：
    - CLI 文本输出基于它；
    - JSON 输出基于它；
    - GUI 面板快照也基于它。
    """

    status: CommandStatus
    message: str
    command_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    error: ArmctrlError | None = None
    state: RobotState | None = None
    detail: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "status", CommandStatus(self.status))

    def to_dict(self) -> dict[str, Any]:
        # 这里先借助 dataclass -> dict，再通过 json 做一轮“可序列化化简”。
        payload = json.loads(json.dumps(asdict(self), default=_json_default))
        if self.error:
            payload["error"] = self.error.to_dict()
        return payload

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, sort_keys=True)

    @classmethod
    def from_json(cls, payload: str) -> CommandResponse:
        # 当前 from_json 只恢复最常用的顶层字段。
        # 如果未来需要完整恢复 state，可在这里继续扩展。
        data = json.loads(payload)
        return cls(
            status=CommandStatus(data["status"]),
            message=data["message"],
            command_id=data["command_id"],
            error=ArmctrlError.from_dict(data.get("error")),
            detail=data.get("detail", {}),
        )
