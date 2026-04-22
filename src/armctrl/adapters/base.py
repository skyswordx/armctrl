from __future__ import annotations

from typing import Protocol

from armctrl.protocol.models import (
    CommandResponse,
    DebugProfileRequest,
    MoveEEFRequest,
    RobotState,
)


class ArmAdapter(Protocol):
    name: str

    def connect(self) -> CommandResponse:
        ...

    def get_state(self) -> RobotState:
        ...

    def get_home_pose(self) -> tuple[float, ...]:
        ...

    def move_eef(self, request: MoveEEFRequest) -> CommandResponse:
        ...

    def apply_debug_profile(self, request: DebugProfileRequest) -> CommandResponse:
        ...

    def damping(self) -> CommandResponse:
        ...

    def cancel(self) -> CommandResponse:
        ...
