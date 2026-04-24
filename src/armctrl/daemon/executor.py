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
    """命令执行器。

    这个类是典型的 application service：
    - 输入是业务命令对象；
    - 中间先走安全检查；
    - 最后委托给 adapter 执行。
    """

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
        # 下面两个字段用于 teleop 的“目标积分”。
        # 公式可写成：
        #   target[k+1] = target[k] + delta[k]
        # 而不是每一拍都从测量值重新出发。
        self._teleop_target_pose: tuple[float, ...] | None = None
        self._teleop_target_gripper: float | None = None
        self._deadman_was_active = False

    def health(self) -> CommandResponse:
        # health 只是读取当前状态并包装成统一响应。
        state = self.adapter.get_state()
        return CommandResponse(CommandStatus.COMPLETED, "armctrl executor healthy", state=state)

    def state(self) -> CommandResponse:
        return CommandResponse(CommandStatus.COMPLETED, "state read", state=self.adapter.get_state())

    def move_eef(self, request: MoveEEFRequest) -> CommandResponse:
        # 所有末端运动请求都必须先过 safety guard。
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
        # 第一类命令：手柄按键触发的 profile 切换。
        if command.debug_profile:
            self._reset_teleop_target()
            self._deadman_was_active = False
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
        # 第二类命令：deadman 松开，直接进入 damping。
        if not command.deadman:
            if self._deadman_was_active:
                self._reset_teleop_target()
                self._deadman_was_active = False
                # 默认放手后不再直接掉到纯 damping。
                # 新默认是“零重力拖动”：
                # - 保留 SDK 启动时已有的 gravity compensation；
                # - 把增益降到很低，便于继续手推；
                # - 但 X 键和 teleop 退出时仍保留真正的 damping 作为硬安全落态。
                return self.apply_debug_profile(
                    DebugProfileRequest(
                        name=DebugProfileName.ZERO_GRAVITY_DRAG,
                    )
                )
            return self.state()
        # 第三类命令：正常 teleop 增量控制。
        validation = self.guard.validate_teleop(command)
        if not validation.allowed:
            return CommandResponse(
                CommandStatus.REJECTED,
                validation.error.message if validation.error else "teleop rejected",
                error=validation.error,
                state=self.adapter.get_state(),
            )
        state = self.adapter.get_state()
        # 首次进入 teleop 时，用当前测量状态初始化目标。
        if self._teleop_target_pose is None:
            self._teleop_target_pose = state.eef.pose_6d
        if self._teleop_target_gripper is None:
            self._teleop_target_gripper = self.guard.limits.clamp_gripper_pos(state.eef.gripper_pos)
        self._deadman_was_active = True
        # 这里用积分目标，而不是“当前测量值 + 本拍增量”。
        # 这么做的好处是减少反馈噪声直接进入目标命令，手感更连续。
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
        # 只有发送成功才更新内部积分目标，避免错误命令污染下一拍基线。
        if response.status is CommandStatus.COMPLETED:
            self._teleop_target_pose = target_pose
            self._teleop_target_gripper = target_gripper
        return self._with_teleop_detail(response, target_pose, target_gripper, command)

    def apply_debug_profile(self, request: DebugProfileRequest) -> CommandResponse:
        # profile 本身也要先过权限与维护状态检查。
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
        # cancel 时清空 teleop 内部目标，避免下一次恢复时沿用旧积分状态。
        self._reset_teleop_target()
        return self.adapter.cancel()

    def _reset_teleop_target(self) -> None:
        # 这个辅助函数很小，但语义非常关键：
        # 它代表“teleop 会话结束，内部目标状态作废”。
        self._teleop_target_pose = None
        self._teleop_target_gripper = None
        self._deadman_was_active = False

    def _with_teleop_detail(
        self,
        response: CommandResponse,
        target_pose: tuple[float, ...],
        target_gripper: float,
        command: TeleopCommand,
    ) -> CommandResponse:
        # adapter 返回的是统一响应；这里重新包装一层，把“累计目标”和“速度命令”
        # 放进 detail，供 GUI 区分“本拍增量”“累计目标”“真实反馈”。
        detail = dict(response.detail)
        detail.update(
            {
                "target_pose_6d": list(target_pose),
                "target_gripper_pos": target_gripper,
                "command_dt_s": command.dt_s,
                "translation_velocity_mps": list(command.translation_velocity_mps),
                "rotation_velocity_radps": list(command.rotation_velocity_radps),
                "gripper_velocity_mps": command.gripper_velocity_mps,
            }
        )
        return CommandResponse(
            response.status,
            response.message,
            command_id=response.command_id,
            error=response.error,
            state=response.state,
            detail=detail,
        )
