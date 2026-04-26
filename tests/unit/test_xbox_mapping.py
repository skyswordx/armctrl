"""Xbox 映射、调试快照和运行循环测试。

这组测试重点不是“手柄能不能连上”，
而是输入语义、控制语义和 UI 快照语义在代码层是否稳定。
"""

import time

import pytest
from armctrl.adapters.arx5.fake import FakeArx5Adapter
from armctrl.daemon.executor import ArmCommandExecutor
from armctrl.teleop.mapping import XboxInputEvent, XboxMapper, XboxState
from armctrl.teleop.xbox import (
    CONTROL_HINTS,
    CONTROLLER_FIELDS,
    XboxDebugRunner,
    build_control_snapshot,
    build_controller_snapshot,
    build_response_snapshot,
    hz_to_period_ms,
)


class RecordingDashboard:
    def __init__(self) -> None:
        # 用最小的假面板记录 runner 推送过来的快照，方便断言 UI 数据内容。
        self.frames: list[tuple[dict, dict]] = []
        self.closed = False

    def push(self, controller_snapshot: dict, control_snapshot: dict) -> None:
        self.frames.append((controller_snapshot, control_snapshot))

    def close(self) -> None:
        self.closed = True


def test_deadman_required_for_motion():
    # RB 没按住时，摇杆输入不应转成任何位移命令。
    mapper = XboxMapper()
    state = XboxState(axes={"ABS_Y": -1.0}, buttons={"BTN_TR": False})
    command = mapper.to_command(state, now=1.0)
    assert not command.deadman
    assert command.translation_m == (0.0, 0.0, 0.0)


def test_right_bumper_enables_slow_x_jog():
    # 左摇杆前推 + RB 按住，应当产生 x 正方向的小步 jog。
    mapper = XboxMapper(max_translation_step_m=0.002)
    state = XboxState(axes={"ABS_Y": -1.0}, buttons={"BTN_TR": True})
    command = mapper.to_command(state, now=1.0)
    assert command.deadman
    assert command.translation_m[0] > 0.0
    assert command.translation_velocity_mps[0] == pytest.approx(0.2)
    assert command.translation_m[0] == pytest.approx(0.002)


def test_mapper_uses_dt_to_compute_per_tick_delta():
    # 速度和 dt 分离后，50Hz 控制周期下每拍位移应为 0.2m/s * 0.02s。
    mapper = XboxMapper(max_translation_speed_mps=0.2)
    state = XboxState(axes={"ABS_Y": -1.0}, buttons={"BTN_TR": True})
    command = mapper.to_command(state, now=1.0, dt_s=0.02)
    assert command.translation_velocity_mps[0] == pytest.approx(0.2)
    assert command.translation_m[0] == pytest.approx(0.004)
    assert command.dt_s == pytest.approx(0.02)


def test_zikway_layout_uses_abs_z_rz_for_right_stick_and_gas_brake_for_triggers():
    # 当前设备布局下，右摇杆和扳机字段与传统 Xbox 兼容布局不同。
    # 这个测试把项目当前认定的“候选主字段”锁定下来。
    mapper = XboxMapper()
    state = XboxState(
        axes={
            "ABS_X": 0.0,
            "ABS_Y": 0.0,
            "ABS_Z": 0.6,
            "ABS_RZ": -0.4,
            "ABS_GAS": 0.0,
            "ABS_BRAKE": 1.0,
        },
        buttons={"BTN_TR": True},
    )
    command = mapper.to_command(state, now=1.0)
    assert command.deadman
    assert command.translation_m[2] > 0.0
    assert command.rotation_rad[2] > 0.0
    assert command.gripper_delta < 0.0


def test_hat_axes_drive_roll_and_pitch_for_full_6d_pose():
    # 方向键补足 roll / pitch，形成完整 6D 增量控制。
    mapper = XboxMapper()
    state = XboxState(
        axes={
            "ABS_HAT0X": 1.0,
            "ABS_HAT0Y": -1.0,
        },
        buttons={"BTN_TR": True},
    )
    command = mapper.to_command(state, now=1.0)
    assert command.deadman
    assert command.rotation_rad[0] > 0.0
    assert command.rotation_rad[1] > 0.0


def test_x_button_requests_damping_profile():
    # X 键是快速回到 damping 的直接入口。
    mapper = XboxMapper()
    state = XboxState(buttons={"BTN_X": True})
    command = mapper.to_command(state, now=1.0)
    assert command.debug_profile == "damping"


def test_a_button_requests_low_gain_profile():
    # A 键触发低增益被动模式请求。
    mapper = XboxMapper()
    state = XboxState(buttons={"BTN_A": True})
    command = mapper.to_command(state, now=1.0)
    assert command.debug_profile == "low_gain_passive"


def test_event_runner_moves_fake_adapter_and_damps_at_exit():
    # runner 正常跑完后，无论中间经历过什么 profile，退出都应回到真正的 damping。
    adapter = FakeArx5Adapter()
    executor = ArmCommandExecutor(adapter)
    events = [
        XboxInputEvent(event_type=1, code=311, value=1, timestamp=1.0),
        XboxInputEvent(event_type=3, code=1, value=-32767, timestamp=1.1),
    ]
    runner = XboxDebugRunner(executor, XboxMapper(), events, max_events=2)
    response = runner.run()
    assert response.status.value == "completed"
    assert adapter.get_state().mode.value == "damping"


def test_deadman_release_enters_damping():
    # runner 收到 deadman 松开后，期间会先切到 zero_gravity_drag，
    # 但 run() finally 仍会在退出前打回真正的 damping。
    adapter = FakeArx5Adapter()
    executor = ArmCommandExecutor(adapter)
    events = [
        XboxInputEvent(event_type=1, code=311, value=1, timestamp=1.0),
        XboxInputEvent(event_type=3, code=1, value=-32767, timestamp=1.1),
        XboxInputEvent(event_type=1, code=311, value=0, timestamp=1.2),
    ]
    runner = XboxDebugRunner(executor, XboxMapper(), events, max_events=3)
    response = runner.run()
    assert response.status.value == "completed"
    assert adapter.get_state().mode.value == "damping"


def test_xbox_maintenance_profile_can_apply_with_confirmation():
    # 维护态 + 明确确认后，按钮触发的 debug profile 才允许真正执行。
    adapter = FakeArx5Adapter()
    executor = ArmCommandExecutor(
        adapter,
        maintenance=True,
        confirm_debug_profiles=True,
    )
    events = [XboxInputEvent(event_type=1, code=304, value=1, timestamp=1.0)]
    runner = XboxDebugRunner(executor, XboxMapper(), events, max_events=1)
    response = runner.run()
    assert response.status.value == "completed"


def test_snapshot_extracts_controller_and_control_fields():
    # 这里验证 GUI/调试面板看到的字段是否已经从业务对象中正确展开。
    state = XboxState(
        axes={"ABS_Y": -1.0, "ABS_X": 0.25},
        buttons={"BTN_TR": True, "BTN_X": False},
    )
    event = XboxInputEvent(event_type=3, code=1, value=-32767, timestamp=1.1)
    command = XboxMapper().to_command(state, now=1.1)
    adapter = FakeArx5Adapter()
    executor = ArmCommandExecutor(adapter)
    response = executor.handle_teleop(command)

    controller = build_controller_snapshot("/dev/input/event10", event, state)
    control = build_control_snapshot(command, response, "event")

    assert controller["source"] == "/dev/input/event10"
    assert controller["last_event_name"] == "ABS_Y"
    assert controller["last_event_normalized"] == -1.0
    assert controller["axis_ABS_Y"] == -1.0
    assert "axis_ABS_RX" not in controller
    assert "axis_ABS_RY" not in controller
    assert controller["button_BTN_TR"] is True
    assert control["phase"] == "event"
    assert control["command_deadman"] is True
    assert control["command_dt_s"] == 0.01
    assert control["command_velocity_x"] > 0.0
    assert control["command_translation_x"] > 0.0
    # 第一拍现在先做 teleop 接管同步，因此目标应先落在当前实测值。
    assert control["target_eef_x"] == pytest.approx(0.3)
    assert control["response_status"] == "completed"
    assert control["state_mode"] == "teleop"


def test_response_snapshot_extracts_eef_joint_and_error_fields():
    # 响应快照需要把 EEF、joint 和错误信息拆成稳定字段。
    adapter = FakeArx5Adapter()
    executor = ArmCommandExecutor(adapter)
    response = executor.health()

    snapshot = build_response_snapshot(response, "health")

    assert snapshot["phase"] == "health"
    assert snapshot["response_status"] == "completed"
    assert snapshot["state_adapter"] == "fake"
    assert snapshot["state_eef_x"] == 0.3
    assert snapshot["joint_pos_1"] == 0.0
    assert snapshot["joint_torque_6"] == 0.0


def test_control_hints_cover_buttons_and_motion_mapping():
    # UI 里的帮助文案本身也是交互契约的一部分。
    joined = "\n".join(CONTROL_HINTS)
    assert "RB / BTN_TR" in joined
    assert "zero_gravity_drag" in joined
    assert "左摇杆上下" in joined
    assert "右摇杆左右" in joined
    assert "方向键" in joined
    assert "A / BTN_A" in joined
    assert "SELECT / BTN_SELECT" in joined


def test_controller_fields_use_candidate_right_stick_only():
    # 当前 UI 只展示确认过的右摇杆候选字段，不再展示历史遗留字段。
    keys = {key for key, _ in CONTROLLER_FIELDS}
    assert "axis_ABS_Z" in keys
    assert "axis_ABS_RZ" in keys
    assert "axis_ABS_RX" not in keys
    assert "axis_ABS_RY" not in keys


def test_runner_coalesces_burst_events_into_single_control_tick():
    class CountingExecutor:
        def __init__(self) -> None:
            self.adapter = FakeArx5Adapter()
            self.command_count = 0

        def state(self):
            # 这个轻量 executor 只保留 runner 需要的最小接口。
            from armctrl.protocol.enums import CommandStatus
            from armctrl.protocol.models import CommandResponse

            return CommandResponse(CommandStatus.COMPLETED, "state", state=self.adapter.get_state())

        def handle_teleop(self, command):
            # 这里不关心命令内容，只验证一次控制拍只发一次命令。
            from armctrl.protocol.enums import CommandStatus
            from armctrl.protocol.models import CommandResponse

            self.command_count += 1
            return CommandResponse(CommandStatus.COMPLETED, "ok", state=self.adapter.get_state())

        def damping(self):
            from armctrl.protocol.enums import CommandStatus
            from armctrl.protocol.models import CommandResponse

            return CommandResponse(CommandStatus.COMPLETED, "damping", state=self.adapter.get_state())

    executor = CountingExecutor()
    events = [
        XboxInputEvent(event_type=1, code=311, value=1, timestamp=1.0),
        XboxInputEvent(event_type=3, code=1, value=-32767, timestamp=1.0),
    ]
    runner = XboxDebugRunner(executor, XboxMapper(), events, rate_hz=100.0, max_events=2)
    runner.run()
    assert executor.command_count == 1


def test_ui_refresh_50hz_maps_to_20ms():
    # Tk `after()` 使用毫秒，因此 50Hz 应映射到 20ms。
    assert hz_to_period_ms(50.0) == 20


def test_event_runner_pushes_extracted_snapshots_to_dashboard():
    # runner 应当把提取后的快照推给 UI，而不是把原始对象泄漏给面板层。
    adapter = FakeArx5Adapter()
    executor = ArmCommandExecutor(adapter)
    dashboard = RecordingDashboard()
    events = [
        XboxInputEvent(event_type=1, code=311, value=1, timestamp=1.0),
        XboxInputEvent(event_type=3, code=1, value=-32767, timestamp=1.1),
    ]
    runner = XboxDebugRunner(
        executor,
        XboxMapper(),
        events,
        max_events=2,
        source="/dev/input/event10",
        dashboard=dashboard,
    )
    response = runner.run()
    assert response.status.value == "completed"
    assert dashboard.closed
    assert len(dashboard.frames) >= 2
    controller_snapshot, control_snapshot = dashboard.frames[-1]
    assert controller_snapshot["source"] == "/dev/input/event10"
    assert "axis_ABS_Y" in controller_snapshot
    assert "response_status" in control_snapshot


def test_event_runner_keeps_jogging_while_deadman_is_held():
    # 这个测试验证“无新事件时也继续按固定频率发送控制”。
    # 只要 RB 仍按住，runner 就应持续输出 jog。
    adapter = FakeArx5Adapter()
    executor = ArmCommandExecutor(adapter)

    def delayed_events():
        yield XboxInputEvent(event_type=1, code=311, value=1, timestamp=time.monotonic())
        yield XboxInputEvent(event_type=3, code=1, value=-32767, timestamp=time.monotonic())
        time.sleep(0.12)
        yield XboxInputEvent(event_type=1, code=311, value=0, timestamp=time.monotonic())

    runner = XboxDebugRunner(
        executor,
        XboxMapper(max_translation_step_m=0.002),
        delayed_events(),
        rate_hz=50.0,
        max_events=3,
    )
    response = runner.run()
    assert response.status.value == "completed"
    assert adapter.get_state().eef.pose_6d[0] > 0.304
