from __future__ import annotations

from dataclasses import dataclass

from armctrl.protocol.enums import DebugProfileName


@dataclass(frozen=True)
class MotionLimits:
    """运动限幅配置。

    这些值不是动力学最优参数，而是软件层的保守边界。
    """

    workspace_min: tuple[float, float, float] = (0.05, -0.45, 0.02)
    workspace_max: tuple[float, float, float] = (0.75, 0.45, 0.65)
    max_translation_step_m: float = 0.005
    max_rotation_step_rad: float = 0.05
    min_gripper_pos: float = 0.0
    max_gripper_pos: float = 0.12
    max_gripper_step: float = 0.003

    def contains_position(self, xyz: tuple[float, float, float]) -> bool:
        # 使用 all(...) 可以把三个坐标轴的边界判断写成紧凑的逐轴检查。
        return all(
            self.workspace_min[index] <= xyz[index] <= self.workspace_max[index]
            for index in range(3)
        )

    def clamp_gripper_pos(self, position: float) -> float:
        # clamp 公式：
        #   clamp(x) = max(min_value, min(max_value, x))
        return max(self.min_gripper_pos, min(self.max_gripper_pos, position))


@dataclass(frozen=True)
class DebugProfile:
    """调试 profile 的静态描述对象。"""

    name: DebugProfileName
    label: str
    requires_maintenance: bool
    requires_restart: bool = False


class DebugProfileRegistry:
    """调试 profile 注册表。

    这里放的是“权限与运行语义元信息”，
    不是 ARX5 适配器的具体 gain 参数。
    """

    def __init__(self, profiles: list[DebugProfile]) -> None:
        self._profiles = {profile.name: profile for profile in profiles}

    @classmethod
    def default(cls) -> "DebugProfileRegistry":
        return cls(
            [
                DebugProfile(DebugProfileName.ZERO_GRAVITY_DRAG, "Gravity compensation + low-gain hand guiding", False),
                DebugProfile(DebugProfileName.DAMPING, "SDK damping mode", False),
                DebugProfile(DebugProfileName.RESET_HOME, "SDK reset to home", True),
                DebugProfile(DebugProfileName.LOW_GAIN_PASSIVE, "Low-gain passive check", True),
                DebugProfile(DebugProfileName.COMPLIANCE_SLOW, "Slow compliance profile", True),
                DebugProfile(
                    DebugProfileName.GRAVITY_COMPENSATION_STARTUP,
                    "Startup gravity compensation",
                    True,
                    requires_restart=True,
                ),
            ]
        )

    def get(self, name: DebugProfileName | str) -> DebugProfile:
        return self._profiles[DebugProfileName(name)]

    def list(self) -> list[DebugProfile]:
        return list(self._profiles.values())
