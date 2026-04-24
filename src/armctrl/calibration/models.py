"""夹爪标定数据模型。

这里故意只保留和项目运行直接相关的最小字段：
- `gripper_open_readout`：SDK 用来把电机角度换算成夹爪开口宽度；
- `gripper_width`：夹爪完全张开时的物理宽度；
- `closed_motor_readout`：当前流程里通常会被校到 0，用来把标定过程讲清楚。

项目层只负责保存和应用这些值，
不会在外层重新实现一套夹爪运动学。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any


def _utc_now_iso() -> str:
    # 标定文件需要稳定、可追踪的时间戳。
    # 这里统一保存成 UTC ISO8601，避免不同主机本地时区把调试记录搅乱。
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


@dataclass(frozen=True)
class GripperCalibration:
    """项目侧夹爪标定记录。"""

    model: str
    gripper_open_readout: float
    gripper_width: float
    closed_motor_readout: float = 0.0
    source: str = "manual_set"
    interface: str | None = None
    updated_at: str = field(default_factory=_utc_now_iso)
    notes: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "model", str(self.model))
        object.__setattr__(self, "gripper_open_readout", float(self.gripper_open_readout))
        object.__setattr__(self, "gripper_width", float(self.gripper_width))
        object.__setattr__(self, "closed_motor_readout", float(self.closed_motor_readout))
        object.__setattr__(self, "source", str(self.source))
        object.__setattr__(self, "interface", None if self.interface is None else str(self.interface))
        object.__setattr__(self, "updated_at", str(self.updated_at))
        object.__setattr__(self, "notes", str(self.notes))

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "GripperCalibration":
        # 这里按显式字段恢复，避免无关字段悄悄混入 dataclass。
        return cls(
            model=payload["model"],
            gripper_open_readout=payload["gripper_open_readout"],
            gripper_width=payload["gripper_width"],
            closed_motor_readout=payload.get("closed_motor_readout", 0.0),
            source=payload.get("source", "manual_set"),
            interface=payload.get("interface"),
            updated_at=payload.get("updated_at", _utc_now_iso()),
            notes=payload.get("notes", ""),
        )


def apply_gripper_calibration(robot_config, calibration: GripperCalibration | None) -> None:
    """把项目侧标定值写入 SDK `RobotConfig`。

    这一层只改 SDK 本来就公开的两个配置字段。
    它不是临时命令行覆盖，而是项目配置到 SDK 配置对象的单向同步。
    """

    if calibration is None:
        return
    robot_config.gripper_open_readout = calibration.gripper_open_readout
    robot_config.gripper_width = calibration.gripper_width
