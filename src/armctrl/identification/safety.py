"""辨识轨迹安全检查。

这里做的是离线轨迹预检查：
- 关节位置不越界；
- 关节速度不越界；
- 关节加速度不越界；
- 相邻采样点时间单调递增。

真实硬件执行时仍然需要 SDK、驱动器和急停机制共同保护。
"""

from __future__ import annotations

from dataclasses import dataclass

from armctrl.identification.models import ExcitationProfile
from armctrl.protocol.enums import ErrorCode
from armctrl.protocol.errors import ValidationResult


@dataclass(frozen=True)
class TrajectorySafetyLimits:
    """关节空间激励轨迹限幅。"""

    joint_min: tuple[float, ...]
    joint_max: tuple[float, ...]
    velocity_max: tuple[float, ...]
    acceleration_max: tuple[float, ...]

    @classmethod
    def conservative(cls, dof: int) -> TrajectorySafetyLimits:
        # 这里故意选相对保守的默认值。
        # 真实项目应在机械臂型号配置中覆盖这些值，而不是长期依赖默认值。
        return cls(
            joint_min=tuple(-1.5 for _ in range(dof)),
            joint_max=tuple(1.5 for _ in range(dof)),
            velocity_max=tuple(0.8 for _ in range(dof)),
            acceleration_max=tuple(5.0 for _ in range(dof)),
        )

    def __post_init__(self) -> None:
        dof = len(self.joint_min)
        if not (len(self.joint_max) == len(self.velocity_max) == len(self.acceleration_max) == dof):
            raise ValueError("all safety limit vectors must use the same dof")


X5_URDF_SAFETY_LIMITS = TrajectorySafetyLimits(
    joint_min=(-1.5, 0.10, 0.10, -1.20, -1.20, -1.20),
    joint_max=(1.5, 2.75, 2.75, 1.20, 1.20, 1.20),
    velocity_max=(0.8, 0.55, 0.55, 0.8, 0.8, 0.8),
    acceleration_max=(5.0, 3.0, 3.0, 5.0, 5.0, 5.0),
)


X5_COORDINATE_CONTRACT = {
    "source": "official_x5_urdf_and_sdk_joint_space",
    "sdk_to_urdf_joint_order": "identity",
    "sdk_to_urdf_sign": [1, 1, 1, 1, 1, 1],
    "sdk_to_urdf_offset_rad": [0, 0, 0, 0, 0, 0],
    "note": (
        "The X5 SDK joint position convention is treated as identical to the official X5 URDF joint order, "
        "sign, and zero offset. Keep this contract explicit so calibration checks can update it if hardware "
        "evidence ever contradicts the assumption."
    ),
}


def model_safety_limits(model: str | None, dof: int) -> TrajectorySafetyLimits:
    if model == "X5" and dof == 6:
        return TrajectorySafetyLimits(
            joint_min=X5_URDF_SAFETY_LIMITS.joint_min[:dof],
            joint_max=X5_URDF_SAFETY_LIMITS.joint_max[:dof],
            velocity_max=X5_URDF_SAFETY_LIMITS.velocity_max[:dof],
            acceleration_max=X5_URDF_SAFETY_LIMITS.acceleration_max[:dof],
        )
    return TrajectorySafetyLimits.conservative(dof)


def model_coordinate_contract(model: str | None, dof: int) -> dict:
    if model == "X5" and dof == 6:
        contract = dict(X5_COORDINATE_CONTRACT)
        contract["sdk_to_urdf_sign"] = contract["sdk_to_urdf_sign"][:dof]
        contract["sdk_to_urdf_offset_rad"] = contract["sdk_to_urdf_offset_rad"][:dof]
        return contract
    return {
        "source": "generic_joint_space",
        "sdk_to_urdf_joint_order": "unknown",
        "sdk_to_urdf_sign": [1 for _ in range(dof)],
        "sdk_to_urdf_offset_rad": [0 for _ in range(dof)],
        "note": "No model-specific SDK/URDF coordinate contract is configured.",
    }


def validate_trajectory(profile: ExcitationProfile, limits: TrajectorySafetyLimits | None = None) -> ValidationResult:
    """验证轨迹是否满足软件层安全限幅。"""

    active_limits = limits or TrajectorySafetyLimits.conservative(profile.dof)
    if len(active_limits.joint_min) != profile.dof:
        return ValidationResult.reject(
            ErrorCode.INVALID_REQUEST,
            "trajectory dof does not match safety limit dof",
            {"trajectory_dof": profile.dof, "limit_dof": len(active_limits.joint_min)},
        )
    previous_t = -1.0
    for point_index, point in enumerate(profile.points):
        if point.t_s < previous_t:
            return ValidationResult.reject(
                ErrorCode.SAFETY_REJECTED,
                "trajectory timestamps must be monotonic",
                {"point_index": point_index, "t_s": point.t_s, "previous_t_s": previous_t},
            )
        previous_t = point.t_s
        for joint_index, position in enumerate(point.q):
            if position < active_limits.joint_min[joint_index] or position > active_limits.joint_max[joint_index]:
                return ValidationResult.reject(
                    ErrorCode.SAFETY_REJECTED,
                    "trajectory joint position outside configured range",
                    {"point_index": point_index, "joint_index": joint_index, "position": position},
                )
            velocity = abs(point.dq[joint_index])
            if velocity > active_limits.velocity_max[joint_index]:
                return ValidationResult.reject(
                    ErrorCode.SAFETY_REJECTED,
                    "trajectory joint velocity outside configured range",
                    {"point_index": point_index, "joint_index": joint_index, "velocity": velocity},
                )
            acceleration = abs(point.ddq[joint_index])
            if acceleration > active_limits.acceleration_max[joint_index]:
                return ValidationResult.reject(
                    ErrorCode.SAFETY_REJECTED,
                    "trajectory joint acceleration outside configured range",
                    {"point_index": point_index, "joint_index": joint_index, "acceleration": acceleration},
                )
    return ValidationResult.ok()

