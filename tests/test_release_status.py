import json
import subprocess
import sys


def test_cli_release_status_reports_sdk_smoke_rc_and_hardware_pending() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "release", "status", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    milestone_ids = [milestone["id"] for milestone in payload["milestones"]]

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.release_status.v1"
    assert payload["version"] == "0.6.0-rc.2"
    assert payload["branch"] == "codex/armctrl-clean-rebuild"
    assert (
        payload["release_readiness"]
        == "sdk_smoke_runner_nonhardware_verified"
    )
    assert milestone_ids == [
        "v0.2.0",
        "v0.3.0",
        "v0.4.0",
        "v0.5.0",
        "v0.6.0-rc.2",
    ]
    assert payload["milestones"][-1]["status"] == "rc_nonhardware_verified_hardware_pending"
    assert "real_sdk_runner_hardware_validation" in payload["hardware_pending"]
    assert "sdk_gravity_smoke_on_target_linux" in payload["hardware_pending"]
    assert "native_lerobot_record_on_target_linux" in payload["hardware_pending"]
    assert "native_lerobot_rollout_on_target_linux" in payload["hardware_pending"]
    assert "n100d_pinocchio_figaroh_validation" in payload["hardware_pending"]
    assert payload["verification"]["local_commands"] == [
        "uv run pytest -q",
        "uv run python -m compileall src tests",
        "uv run armctrl release status --json",
        "uv sync --extra dev --extra lerobot",
        "uv run armctrl lerobot doctor --model X5 --robot-interface can0 --teleop-interface can1 --json",
        "uv run armctrl sysid run gravity_sweep --adapter sdk --duration 8 --amplitude 0.5 --q-center 0 0.30 0.30 0 0 0 --output runs/tmp-confirm-check --confirm 'I UNDERSTAND THIS WILL MOVE THE ARM' --json",
    ]
    assert payload["verification"]["test_count"] >= 59
    assert payload["deferred_validation"]["requires_hardware"] == [
        "real_sdk_runner_hardware_validation",
        "native_lerobot_record_on_target_linux",
        "native_lerobot_rollout_on_target_linux",
        "sdk_preflight_on_target_linux",
        "sdk_handshake_plan_on_target_linux",
        "sdk_gravity_smoke_on_target_linux",
        "hold_damping_ctrl_c_hardware_landing",
        "hardware_ab_control_benefit_test",
    ]
    assert payload["deferred_validation"]["requires_external_tool"] == [
        "n100d_pinocchio_figaroh_validation",
    ]


def test_cli_release_notes_reports_v060_rc_summary() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "release", "notes", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.release_notes.v1"
    assert payload["version"] == "0.6.0-rc.2"
    assert payload["title"] == "armctrl 0.6.0-rc.2 SDK smoke runner"
    assert "sdk_smoke_runner_nonhardware_verified" in payload["summary"]
    assert payload["sections"]["included"] == [
        "governance and safety boundary",
        "offline SysID loop contracts",
        "conservative online identification policy",
        "Agent recipe CLI skill",
        "LeRobot doctor, native command plans, and dataset metadata bridge",
        "SDK gravity smoke runner with confirmation, safety gates, and damping landing path",
    ]
    assert "real_sdk_runner_hardware_validation" in payload["sections"]["deferred"]
    assert payload["markdown"].startswith(
        "# armctrl 0.6.0-rc.2 SDK smoke runner"
    )
