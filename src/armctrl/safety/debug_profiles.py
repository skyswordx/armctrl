from __future__ import annotations

from dataclasses import dataclass

from armctrl.protocol.enums import DebugProfileName


@dataclass(frozen=True)
class DebugProfile:
    """调试 profile 的静态描述对象。"""

    name: DebugProfileName
    label: str
    requires_maintenance: bool
    requires_restart: bool = False


class DebugProfileRegistry:
    """profile 注册表。

    这体现的是 registry pattern：
    - 所有 profile 元数据集中维护；
    - CLI、手柄映射、安全检查共享同一事实来源。
    """

    def __init__(self, profiles: list[DebugProfile]) -> None:
        self._profiles = {profile.name: profile for profile in profiles}

    @classmethod
    def default(cls) -> DebugProfileRegistry:
        # 默认表只保留当前项目明确支持的 profile。
        return cls(
            [
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
