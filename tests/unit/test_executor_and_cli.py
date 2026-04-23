"""执行器与 CLI 的单元测试。

这些测试除了防回归，还有一个教学目的：
它们明确写出了系统的对外承诺，例如确认机制、GUI/JSON 互斥和 teleop 积分目标语义。
"""

import json

import pytest
from armctrl.adapters.arx5.fake import FakeArx5Adapter
from armctrl.cli import arx5ctl
from armctrl.cli.arx5ctl import build_parser, main
from armctrl.daemon.executor import ArmCommandExecutor
from armctrl.protocol.models import EEFStateModel, MoveEEFRequest, TeleopCommand


def test_executor_teleop_moves_fake_adapter_when_deadman_active():
    # deadman 按住后，teleop 命令应当真正进入运动路径，而不是被 executor 直接挡掉。
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
    # `--json` 是脚本接口，输出必须可直接反序列化。
    code = main(["health", "--adapter", "fake", "--json"])
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "completed"


def test_cli_health_pretty_json(capsys):
    # `--pretty` 只影响格式，不改变 JSON 语义。
    code = main(["health", "--adapter", "fake", "--json", "--pretty"])
    assert code == 0
    output = capsys.readouterr().out
    assert "{\n" in output
    payload = json.loads(output)
    assert payload["status"] == "completed"


def test_cli_health_gui_shows_response_without_stdout(monkeypatch, capsys):
    # GUI 模式下，响应应当进入面板而不是继续污染 stdout。
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
    # 真实 SDK 运动命令必须显式确认，防止误操作。
    code = main(["move-eef", "--adapter", "sdk", "--pose", "0.3", "0", "0.2", "0", "0", "0", "--execute", "--json"])
    assert code == 2
    payload = json.loads(capsys.readouterr().out)
    assert payload["error"]["code"] == "confirmation_required"


def test_cli_teleop_has_no_debug_events_option():
    # 历史上的临时调试参数已经删除，不应再被解析器接受。
    parser = build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(["teleop-xbox", "--debug-events"])


def test_cli_teleop_defaults_to_100hz_control_and_50hz_ui():
    # 当前默认策略是控制 100Hz、UI 50Hz，两者故意不绑死在一起。
    parser = build_parser()
    args = parser.parse_args(["teleop-xbox"])
    assert args.rate_hz == 100.0
    assert args.ui_hz == 50.0


def test_cli_accepts_urdf_path_override():
    # `--urdf-path` 是真实 SDK 的模型覆盖入口。
    # fake adapter 也能解析这个参数，但不会使用它。
    parser = build_parser()
    args = parser.parse_args(["health", "--adapter", "sdk", "--urdf-path", "configs/models/X5_camera.urdf"])
    assert args.urdf_path == "configs/models/X5_camera.urdf"


def test_cli_teleop_json_stays_machine_readable(capsys):
    # teleop 即便经过完整事件回放，`--json` 仍然只输出最终统一响应。
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
    # 反馈侧如果出现负夹爪位置，executor 应当先夹紧到安全范围再继续积分。
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
    # 这里专门验证“内部目标积分”语义。
    # 也就是第二拍目标应当基于第一拍目标继续累加，
    # 而不是每次都从滞后的反馈值重新出发。
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


def test_executor_teleop_centered_stick_holds_accumulated_target():
    # 摇杆回中后，本拍增量为 0，但累计目标应停在上一拍位置，而不是回到原点。
    adapter = FakeArx5Adapter()
    executor = ArmCommandExecutor(adapter)

    move_response = executor.handle_teleop(
        TeleopCommand(
            translation_m=(0.002, 0.0, 0.0),
            translation_velocity_mps=(0.2, 0.0, 0.0),
            dt_s=0.01,
            deadman=True,
        )
    )
    hold_response = executor.handle_teleop(
        TeleopCommand(
            translation_m=(0.0, 0.0, 0.0),
            translation_velocity_mps=(0.0, 0.0, 0.0),
            dt_s=0.01,
            deadman=True,
        )
    )

    assert move_response.status.value == "completed"
    assert hold_response.status.value == "completed"
    assert hold_response.state is not None
    assert hold_response.state.eef.pose_6d[0] == pytest.approx(0.302)
    assert hold_response.detail["target_pose_6d"][0] == pytest.approx(0.302)


def test_executor_deadman_release_enters_damping_only_once():
    # deadman 松开时需要切一次 damping，
    # 但空闲控制拍不应反复重进 damping，避免真实 SDK 被重复打回阻尼态。
    class CountingAdapter(FakeArx5Adapter):
        def __init__(self) -> None:
            super().__init__()
            self.damping_calls = 0

        def damping(self):
            self.damping_calls += 1
            return super().damping()

    adapter = CountingAdapter()
    executor = ArmCommandExecutor(adapter)

    active_response = executor.handle_teleop(
        TeleopCommand(
            translation_m=(0.001, 0.0, 0.0),
            deadman=True,
        )
    )
    release_response = executor.handle_teleop(TeleopCommand(deadman=False))
    idle_response = executor.handle_teleop(TeleopCommand(deadman=False))

    assert active_response.status.value == "completed"
    assert release_response.status.value == "completed"
    assert idle_response.status.value == "completed"
    assert adapter.damping_calls == 1
