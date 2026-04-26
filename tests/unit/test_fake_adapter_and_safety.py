"""fake adapter 与安全层测试。

这组测试覆盖两件核心事情：
1. fake 适配器是否忠实模拟协议语义；
2. safety guard 是否把明显危险或非法的请求挡在执行前。
"""

from armctrl.adapters.arx5.fake import FakeArx5Adapter
from armctrl.protocol.enums import DebugProfileName
from armctrl.protocol.models import DebugProfileRequest, MoveEEFRequest, TeleopCommand
from armctrl.safety.guard import SafetyGuard
from armctrl.safety.profiles import DebugProfileRegistry, MotionLimits


def test_fake_adapter_plan_only_does_not_move():
    # plan-only 语义是“验证通过，但不触发真实位姿变化”。
    adapter = FakeArx5Adapter()
    before = adapter.get_state().eef.pose_6d
    response = adapter.move_eef(
        MoveEEFRequest(
            pose_6d=(0.31, 0.0, 0.2, 0.0, 0.0, 0.0),
            plan_only=True,
        )
    )
    assert response.status.value == "completed"
    assert adapter.get_state().eef.pose_6d == before


def test_safety_rejects_large_teleop_step():
    # 单步增量过大时，应在 teleop 校验阶段直接拒绝。
    guard = SafetyGuard(MotionLimits(max_translation_step_m=0.01))
    command = TeleopCommand(
        translation_m=(0.2, 0.0, 0.0),
        rotation_rad=(0.0, 0.0, 0.0),
        deadman=True,
    )
    result = guard.validate_teleop(command)
    assert not result.allowed


def test_debug_profile_requires_maintenance():
    # 维护态 profile 不能在普通运行态下直接切换。
    registry = DebugProfileRegistry.default()
    guard = SafetyGuard(MotionLimits(), registry)
    request = DebugProfileRequest(
        name=DebugProfileName.LOW_GAIN_PASSIVE,
        maintenance=False,
        confirm=False,
    )
    result = guard.validate_debug_profile(request)
    assert not result.allowed


def test_gravity_compensation_startup_rejected_at_runtime():
    # 重补启动是构造期配置，不是运行中热切换 profile。
    guard = SafetyGuard()
    request = DebugProfileRequest(
        name=DebugProfileName.GRAVITY_COMPENSATION_STARTUP,
        maintenance=True,
        confirm=True,
        plan_only=False,
    )
    result = guard.validate_debug_profile(request)
    assert not result.allowed
    assert result.error is not None
    assert result.error.code.value == "invalid_request"
