import json

import pytest
from armctrl.adapters.arx5.fake import FakeArx5Adapter
from armctrl.cli import arx5ctl
from armctrl.cli.arx5ctl import build_parser, main
from armctrl.daemon.executor import ArmCommandExecutor
from armctrl.protocol.models import EEFStateModel, MoveEEFRequest, TeleopCommand


def test_executor_teleop_moves_fake_adapter_when_deadman_active():
    adapter = FakeArx5Adapter()
    executor = ArmCommandExecutor(adapter)
    response = executor.handle_teleop(
        TeleopCommand(
            translation_m=(0.001, 0.0, 0.0),
            rotation_rad=(0.0, 0.0, 0.0),
            deadman=True,
        )
    )
    assert response.status.value == "completed"
    assert adapter.get_state().eef.pose_6d[0] > 0.3


def test_cli_health_json(capsys):
    code = main(["health", "--adapter", "fake", "--json"])
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "completed"


def test_cli_health_pretty_json(capsys):
    code = main(["health", "--adapter", "fake", "--json", "--pretty"])
    assert code == 0
    output = capsys.readouterr().out
    assert "{\n" in output
    payload = json.loads(output)
    assert payload["status"] == "completed"


def test_cli_health_gui_shows_response_without_stdout(monkeypatch, capsys):
    shown = []

    def fake_show_response_dashboard(response, source: str) -> None:
        shown.append((response, source))

    monkeypatch.setattr(arx5ctl, "show_response_dashboard", fake_show_response_dashboard)

    code = main(["health", "--adapter", "fake", "--gui"])

    assert code == 0
    assert capsys.readouterr().out == ""
    assert shown
    response, source = shown[0]
    assert response.status.value == "completed"
    assert source == "health:fake"


def test_cli_sdk_motion_requires_confirmation(capsys):
    code = main(["move-eef", "--adapter", "sdk", "--pose", "0.3", "0", "0.2", "0", "0", "0", "--execute", "--json"])
    assert code == 2
    payload = json.loads(capsys.readouterr().out)
    assert payload["error"]["code"] == "confirmation_required"


def test_cli_teleop_has_no_debug_events_option():
    parser = build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(["teleop-xbox", "--debug-events"])


def test_cli_teleop_defaults_to_100hz_control_and_50hz_ui():
    parser = build_parser()
    args = parser.parse_args(["teleop-xbox"])
    assert args.rate_hz == 100.0
    assert args.ui_hz == 50.0


def test_cli_teleop_json_stays_machine_readable(capsys):
    code = main(
        [
            "teleop-xbox",
            "--adapter",
            "fake",
            "--event-jsonl",
            "tests/fixtures/xbox_sample.jsonl",
            "--max-events",
            "2",
            "--json",
        ]
    )
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "completed"
    assert payload["state"]["mode"] == "damping"


def test_executor_teleop_clamps_negative_gripper_feedback():
    adapter = FakeArx5Adapter()
    adapter._eef = EEFStateModel(
        pose_6d=(0.3, 0.0, 0.2, 0.0, 0.0, 0.0),
        gripper_pos=-0.002,
        timestamp=1.0,
    )
    executor = ArmCommandExecutor(adapter)
    response = executor.handle_teleop(
        TeleopCommand(
            translation_m=(0.001, 0.0, 0.0),
            rotation_rad=(0.0, 0.0, 0.0),
            deadman=True,
        )
    )
    assert response.status.value == "completed"
    assert response.state is not None
    assert response.state.eef.pose_6d[0] > 0.3
    assert response.state.eef.gripper_pos == 0.0


def test_executor_teleop_integrates_target_independent_of_feedback():
    class StaticFeedbackAdapter(FakeArx5Adapter):
        def __init__(self) -> None:
            super().__init__()
            self.requests: list[MoveEEFRequest] = []
            self._feedback = super().get_state()

        def get_state(self):
            return self._feedback

        def move_eef(self, request: MoveEEFRequest):
            self.requests.append(request)
            return super().move_eef(request)

    adapter = StaticFeedbackAdapter()
    executor = ArmCommandExecutor(adapter)
    command = TeleopCommand(
        translation_m=(0.001, 0.0, 0.0),
        rotation_rad=(0.0, 0.0, 0.0),
        deadman=True,
    )

    executor.handle_teleop(command)
    executor.handle_teleop(command)

    assert len(adapter.requests) == 2
    assert adapter.requests[0].pose_6d[0] == pytest.approx(0.301)
    assert adapter.requests[1].pose_6d[0] == pytest.approx(0.302)
