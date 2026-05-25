"""参数辨识模块的数据模型。

这里的模型刻意保持纯 Python dataclass，不绑定 ARX5 SDK。
后续接入松灵 Piper 或其他机械臂时，只要提供相同的关节 I/O 后端，
轨迹生成、数据记录和后处理都可以复用。
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field
from typing import Any


def tuple_of_floats(name: str, values: tuple[float, ...], expected: int | None = None) -> tuple[float, ...]:
    # 所有轨迹和采样向量都先在边界处转成不可变 tuple。
    # 这样后续写 CSV、写 manifest、做安全检查时不会被外部列表原地改掉。
    converted = tuple(float(value) for value in values)
    if expected is not None and len(converted) != expected:
        raise ValueError(f"{name} must contain {expected} values")
    if expected is None and len(converted) == 0:
        raise ValueError(f"{name} must not be empty")
    return converted


@dataclass(frozen=True)
class TrajectoryPoint:
    """单个关节轨迹点。

    `q / dq / ddq` 单位分别是 rad、rad/s、rad/s²。
    `t_s` 是相对轨迹起点的时间，不是系统绝对时间。
    """

    t_s: float
    q: tuple[float, ...]
    dq: tuple[float, ...]
    ddq: tuple[float, ...]
    phase: str = "trajectory"

    def __post_init__(self) -> None:
        dof = len(self.q)
        object.__setattr__(self, "t_s", float(self.t_s))
        object.__setattr__(self, "q", tuple_of_floats("q", self.q))
        object.__setattr__(self, "dq", tuple_of_floats("dq", self.dq, dof))
        object.__setattr__(self, "ddq", tuple_of_floats("ddq", self.ddq, dof))

    @property
    def dof(self) -> int:
        return len(self.q)


@dataclass(frozen=True)
class ExcitationProfile:
    """一段可执行或可预览的激励轨迹。"""

    name: str
    description: str
    sample_hz: float
    points: tuple[TrajectoryPoint, ...]
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.points:
            raise ValueError("profile points must not be empty")
        dof = self.points[0].dof
        for point in self.points:
            if point.dof != dof:
                raise ValueError("all trajectory points must use the same dof")
        object.__setattr__(self, "sample_hz", float(self.sample_hz))
        object.__setattr__(self, "points", tuple(self.points))

    @property
    def dof(self) -> int:
        return self.points[0].dof

    @property
    def duration_s(self) -> float:
        return self.points[-1].t_s

    def summary(self) -> dict[str, Any]:
        # summary 是 CLI、manifest 和测试共同使用的轻量摘要。
        # 不放完整点列，避免 JSON 响应过大。
        planned_joint_ranges_rad = []
        planned_joint_ranges_deg = []
        for joint_index in range(self.dof):
            values = [point.q[joint_index] for point in self.points]
            range_rad = max(values) - min(values)
            planned_joint_ranges_rad.append(range_rad)
            planned_joint_ranges_deg.append(math.degrees(range_rad))
        return {
            "profile_name": self.name,
            "description": self.description,
            "dof": self.dof,
            "sample_hz": self.sample_hz,
            "duration_s": self.duration_s,
            "point_count": len(self.points),
            "planned_joint_ranges_rad": planned_joint_ranges_rad,
            "planned_joint_ranges_deg": planned_joint_ranges_deg,
            "metadata": self.metadata,
        }


@dataclass(frozen=True)
class JointSample:
    """真实采集到的一行辨识数据。

    `tau_meas` 使用当前后端能提供的力矩反馈。
    对 ARX5 SDK 来说，这个值来自 SDK 的电流换算力矩反馈，
    不能直接等同于外置六维力传感器的真实关节力矩。
    """

    t_s: float
    monotonic_s: float
    phase: str
    q: tuple[float, ...]
    dq: tuple[float, ...]
    tau_meas: tuple[float, ...]
    q_cmd: tuple[float, ...]
    dq_cmd: tuple[float, ...]
    ddq_cmd: tuple[float, ...]
    tau_cmd: tuple[float, ...]
    source_timestamp_s: float = 0.0

    def __post_init__(self) -> None:
        dof = len(self.q)
        object.__setattr__(self, "t_s", float(self.t_s))
        object.__setattr__(self, "monotonic_s", float(self.monotonic_s))
        object.__setattr__(self, "q", tuple_of_floats("q", self.q))
        object.__setattr__(self, "dq", tuple_of_floats("dq", self.dq, dof))
        object.__setattr__(self, "tau_meas", tuple_of_floats("tau_meas", self.tau_meas, dof))
        object.__setattr__(self, "q_cmd", tuple_of_floats("q_cmd", self.q_cmd, dof))
        object.__setattr__(self, "dq_cmd", tuple_of_floats("dq_cmd", self.dq_cmd, dof))
        object.__setattr__(self, "ddq_cmd", tuple_of_floats("ddq_cmd", self.ddq_cmd, dof))
        object.__setattr__(self, "tau_cmd", tuple_of_floats("tau_cmd", self.tau_cmd, dof))
        object.__setattr__(self, "source_timestamp_s", float(self.source_timestamp_s))


@dataclass(frozen=True)
class DatasetManifest:
    """辨识数据集 manifest。

    manifest 保存“数据怎么来的”，CSV 保存“每一行测到了什么”。
    后处理工具优先读取 manifest 来确认模型、后端、采样频率和列名。
    """

    profile_name: str
    backend_name: str
    model: str
    dof: int
    sample_hz: float
    duration_s: float
    raw_samples: str
    planned_trajectory: str | None = None
    created_at_s: float = field(default_factory=time.time)
    urdf_path: str | None = None
    execute: bool = False
    columns: dict[str, list[str]] = field(default_factory=dict)
    lerobot_contract: dict[str, Any] = field(default_factory=dict)
    profile_metadata: dict[str, Any] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    tool_hints: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def vector_column_names(prefix: str, dof: int) -> list[str]:
    # 列名从 1 开始，和机械臂关节编号的习惯一致。
    return [f"{prefix}_{index}" for index in range(1, dof + 1)]
