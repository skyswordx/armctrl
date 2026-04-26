from __future__ import annotations

from math import fabs

from armctrl.protocol.enums import ErrorCode
from armctrl.protocol.errors import ValidationResult
from armctrl.protocol.models import DebugProfileRequest, MoveEEFRequest, TeleopCommand
from armctrl.safety.profiles import DebugProfileRegistry, MotionLimits


class SafetyGuard:
    """业务层安全守卫。

    这里做的是“发送命令之前的轻量保护”：
    - 工作空间边界；
    - 单步增量边界；
    - 维护权限和确认权限。
    """

    def __init__(
        self,
        limits: MotionLimits | None = None,
        debug_profiles: DebugProfileRegistry | None = None,
    ) -> None:
        self.limits = limits or MotionLimits()
        self.debug_profiles = debug_profiles or DebugProfileRegistry.default()

    def validate_move_eef(self, request: MoveEEFRequest) -> ValidationResult:
        # 当前版本只对 xyz 工作空间做边界检查。
        # orientation 仍交给上层策略和底层控制器配合处理。
        xyz = request.pose_6d[:3]
        if not self.limits.contains_position((xyz[0], xyz[1], xyz[2])):
            return ValidationResult.reject(
                ErrorCode.SAFETY_REJECTED,
                "EEF target outside configured workspace",
                {"pose_6d": list(request.pose_6d)},
            )
        if not self.limits.min_gripper_pos <= request.gripper_pos <= self.limits.max_gripper_pos:
            return ValidationResult.reject(
                ErrorCode.SAFETY_REJECTED,
                "gripper target outside configured range",
                {"gripper_pos": request.gripper_pos},
            )
        return ValidationResult.ok()

    def validate_teleop(self, command: TeleopCommand) -> ValidationResult:
        # deadman 是实时遥操作里最基础的人工确认机制。
        if not command.deadman:
            return ValidationResult.reject(ErrorCode.SAFETY_REJECTED, "deadman is not active")
        # 下面三段检查都属于“单拍增量限幅”，而不是全局轨迹约束。
        max_translation = max(fabs(value) for value in command.translation_m)
        if max_translation > self.limits.max_translation_step_m:
            return ValidationResult.reject(
                ErrorCode.SAFETY_REJECTED,
                "teleop translation step is too large",
                {"max_translation": max_translation},
            )
        max_rotation = max(fabs(value) for value in command.rotation_rad)
        if max_rotation > self.limits.max_rotation_step_rad:
            return ValidationResult.reject(
                ErrorCode.SAFETY_REJECTED,
                "teleop rotation step is too large",
                {"max_rotation": max_rotation},
            )
        if fabs(command.gripper_delta) > self.limits.max_gripper_step:
            return ValidationResult.reject(
                ErrorCode.SAFETY_REJECTED,
                "teleop gripper step is too large",
                {"gripper_delta": command.gripper_delta},
            )
        return ValidationResult.ok()

    def validate_debug_profile(self, request: DebugProfileRequest) -> ValidationResult:
        # profile 权限校验有明确顺序：
        # 1. 先检查是否必须启动前配置；
        # 2. 再检查是否处于 maintenance；
        # 3. 最后检查是否给了显式确认。
        profile = self.debug_profiles.get(request.name)
        if profile.requires_restart and not request.plan_only:
            return ValidationResult.reject(
                ErrorCode.INVALID_REQUEST,
                f"{profile.name.value} must be configured before controller creation",
            )
        if profile.requires_maintenance and not request.maintenance:
            return ValidationResult.reject(
                ErrorCode.MAINTENANCE_REQUIRED,
                f"{profile.name.value} requires maintenance mode",
            )
        if profile.requires_maintenance and not request.confirm:
            return ValidationResult.reject(
                ErrorCode.CONFIRMATION_REQUIRED,
                f"{profile.name.value} requires explicit confirmation",
            )
        return ValidationResult.ok()
