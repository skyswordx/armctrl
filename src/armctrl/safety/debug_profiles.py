from __future__ import annotations

from dataclasses import dataclass

from armctrl.protocol.enums import DebugProfileName


@dataclass(frozen=True)
class DebugProfile:
    name: DebugProfileName
    label: str
    requires_maintenance: bool
    requires_restart: bool = False


class DebugProfileRegistry:
    def __init__(self, profiles: list[DebugProfile]) -> None:
        self._profiles = {profile.name: profile for profile in profiles}

    @classmethod
    def default(cls) -> DebugProfileRegistry:
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
