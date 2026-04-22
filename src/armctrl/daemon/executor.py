from __future__ import annotations

from armctrl.adapters.base import ArmAdapter
from armctrl.protocol.enums import CommandStatus, DebugProfileName
from armctrl.protocol.models import (
    CommandResponse,
    DebugProfileRequest,
    MoveEEFRequest,
    TeleopCommand,
)
from armctrl.safety.guard import SafetyGuard


class ArmCommandExecutor:
    def __init__(
        self,
        adapter: ArmAdapter,
        guard: SafetyGuard | None = None,
        maintenance: bool = False,
        confirm_debug_profiles: bool = False,
        plan_only_debug_profiles: bool = False,
    ) -> None:
        self.adapter = adapter
        self.guard = guard or SafetyGuard()
        self.maintenance = maintenance
        self.confirm_debug_profiles = confirm_debug_profiles
        self.plan_only_debug_profiles = plan_only_debug_profiles
        self._teleop_target_pose: tuple[float, ...] | None = None
        self._teleop_target_gripper: float | None = None

    def health(self) -> CommandResponse:
        state = self.adapter.get_state()
        return CommandResponse(CommandStatus.COMPLETED, "armctrl executor healthy", state=state)

    def state(self) -> CommandResponse:
        return CommandResponse(CommandStatus.COMPLETED, "state read", state=self.adapter.get_state())

    def move_eef(self, request: MoveEEFRequest) -> CommandResponse:
        validation = self.guard.validate_move_eef(request)
        if not validation.allowed:
            return CommandResponse(
                CommandStatus.REJECTED,
                validation.error.message if validation.error else "move rejected",
                command_id=request.command_id,
                error=validation.error,
                state=self.adapter.get_state(),
            )
        return self.adapter.move_eef(request)

    def handle_teleop(self, command: TeleopCommand) -> CommandResponse:
        if command.debug_profile:
            self._reset_teleop_target()
            try:
                profile_request = DebugProfileRequest(
                    name=DebugProfileName(command.debug_profile),
                    maintenance=self.maintenance,
                    confirm=self.confirm_debug_profiles,
                    plan_only=self.plan_only_debug_profiles,
                )
            except ValueError:
                return CommandResponse(CommandStatus.REJECTED, f"unknown debug profile {command.debug_profile}")
            return self.apply_debug_profile(profile_request)
        if not command.deadman:
            self._reset_teleop_target()
            return self.damping()
        validation = self.guard.validate_teleop(command)
        if not validation.allowed:
            return CommandResponse(
                CommandStatus.REJECTED,
                validation.error.message if validation.error else "teleop rejected",
                error=validation.error,
                state=self.adapter.get_state(),
            )
        state = self.adapter.get_state()
        if self._teleop_target_pose is None:
            self._teleop_target_pose = state.eef.pose_6d
        if self._teleop_target_gripper is None:
            self._teleop_target_gripper = self.guard.limits.clamp_gripper_pos(state.eef.gripper_pos)
        target_pose = tuple(
            self._teleop_target_pose[index]
            + (command.translation_m[index] if index < 3 else command.rotation_rad[index - 3])
            for index in range(6)
        )
        target_gripper = self.guard.limits.clamp_gripper_pos(
            self._teleop_target_gripper + command.gripper_delta
        )
        request = MoveEEFRequest(
            pose_6d=target_pose,
            gripper_pos=target_gripper,
            preview_time_s=0.1,
            profile="xbox_teleop_slow",
        )
        response = self.move_eef(request)
        if response.status is CommandStatus.COMPLETED:
            self._teleop_target_pose = target_pose
            self._teleop_target_gripper = target_gripper
        return response

    def apply_debug_profile(self, request: DebugProfileRequest) -> CommandResponse:
        validation = self.guard.validate_debug_profile(request)
        if not validation.allowed:
            return CommandResponse(
                CommandStatus.REJECTED,
                validation.error.message if validation.error else "debug profile rejected",
                command_id=request.command_id,
                error=validation.error,
                state=self.adapter.get_state(),
            )
        return self.adapter.apply_debug_profile(request)

    def damping(self) -> CommandResponse:
        return self.adapter.damping()

    def cancel(self) -> CommandResponse:
        self._reset_teleop_target()
        return self.adapter.cancel()

    def _reset_teleop_target(self) -> None:
        self._teleop_target_pose = None
        self._teleop_target_gripper = None
