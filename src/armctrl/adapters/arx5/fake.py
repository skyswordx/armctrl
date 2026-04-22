from __future__ import annotations

import time

from armctrl.protocol.enums import ArmMode, CommandStatus, DebugProfileName
from armctrl.protocol.models import (
    CommandResponse,
    DebugProfileRequest,
    EEFStateModel,
    JointStateModel,
    MoveEEFRequest,
    RobotState,
)


class FakeArx5Adapter:
    name = "fake"

    def __init__(self) -> None:
        now = time.monotonic()
        self._home_pose = (0.3, 0.0, 0.2, 0.0, 0.0, 0.0)
        self._eef = EEFStateModel(self._home_pose, timestamp=now)
        self._joint = JointStateModel(
            pos=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            vel=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            torque=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            timestamp=now,
        )
        self._mode = ArmMode.IDLE
        self.connected = True

    def connect(self) -> CommandResponse:
        self.connected = True
        self._mode = ArmMode.IDLE
        return CommandResponse(CommandStatus.COMPLETED, "fake adapter connected", state=self.get_state())

    def get_state(self) -> RobotState:
        return RobotState(
            mode=self._mode,
            joint=self._joint,
            eef=self._eef,
            connected=self.connected,
            adapter=self.name,
        )

    def get_home_pose(self) -> tuple[float, ...]:
        return self._home_pose

    def move_eef(self, request: MoveEEFRequest) -> CommandResponse:
        if not request.plan_only:
            self._eef = EEFStateModel(
                pose_6d=request.pose_6d,
                gripper_pos=request.gripper_pos,
                timestamp=time.monotonic() + request.preview_time_s,
            )
            self._mode = ArmMode.TELEOP
        return CommandResponse(
            CommandStatus.COMPLETED,
            "fake eef request accepted",
            command_id=request.command_id,
            state=self.get_state(),
        )

    def apply_debug_profile(self, request: DebugProfileRequest) -> CommandResponse:
        if request.name == DebugProfileName.DAMPING:
            return self.damping()
        if request.name == DebugProfileName.RESET_HOME and not request.plan_only:
            self._eef = EEFStateModel(self._home_pose, timestamp=time.monotonic())
            self._mode = ArmMode.IDLE
        return CommandResponse(
            CommandStatus.COMPLETED,
            f"fake debug profile {request.name.value} accepted",
            command_id=request.command_id,
            state=self.get_state(),
        )

    def damping(self) -> CommandResponse:
        self._mode = ArmMode.DAMPING
        return CommandResponse(CommandStatus.COMPLETED, "fake damping enabled", state=self.get_state())

    def cancel(self) -> CommandResponse:
        self._mode = ArmMode.DAMPING
        return CommandResponse(CommandStatus.CANCELLED, "fake command cancelled", state=self.get_state())
