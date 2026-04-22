import time

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
        self.frames: list[tuple[dict, dict]] = []
        self.closed = False

    def push(self, controller_snapshot: dict, control_snapshot: dict) -> None:
        self.frames.append((controller_snapshot, control_snapshot))

    def close(self) -> None:
        self.closed = True


def test_deadman_required_for_motion():
    mapper = XboxMapper()
    state = XboxState(axes={"ABS_Y": -1.0}, buttons={"BTN_TR": False})
    command = mapper.to_command(state, now=1.0)
    assert not command.deadman
    assert command.translation_m == (0.0, 0.0, 0.0)


def test_right_bumper_enables_slow_x_jog():
    mapper = XboxMapper(max_translation_step_m=0.002)
    state = XboxState(axes={"ABS_Y": -1.0}, buttons={"BTN_TR": True})
    command = mapper.to_command(state, now=1.0)
    assert command.deadman
    assert command.translation_m[0] > 0.0


def test_zikway_layout_uses_abs_z_rz_for_right_stick_and_gas_brake_for_triggers():
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
    mapper = XboxMapper()
    state = XboxState(buttons={"BTN_X": True})
    command = mapper.to_command(state, now=1.0)
    assert command.debug_profile == "damping"


def test_a_button_requests_low_gain_profile():
    mapper = XboxMapper()
    state = XboxState(buttons={"BTN_A": True})
    command = mapper.to_command(state, now=1.0)
    assert command.debug_profile == "low_gain_passive"


def test_event_runner_moves_fake_adapter_and_damps_at_exit():
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
    assert control["command_translation_x"] > 0.0
    assert control["response_status"] == "completed"
    assert control["state_mode"] == "teleop"


def test_response_snapshot_extracts_eef_joint_and_error_fields():
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
    joined = "\n".join(CONTROL_HINTS)
    assert "RB / BTN_TR" in joined
    assert "左摇杆上下" in joined
    assert "右摇杆左右" in joined
    assert "方向键" in joined
    assert "A / BTN_A" in joined
    assert "SELECT / BTN_SELECT" in joined


def test_controller_fields_use_candidate_right_stick_only():
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
            from armctrl.protocol.enums import CommandStatus
            from armctrl.protocol.models import CommandResponse

            return CommandResponse(CommandStatus.COMPLETED, "state", state=self.adapter.get_state())

        def handle_teleop(self, command):
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
    assert hz_to_period_ms(50.0) == 20


def test_event_runner_pushes_extracted_snapshots_to_dashboard():
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
