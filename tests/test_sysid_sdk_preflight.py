import json
import subprocess
import sys


def test_cli_sysid_sdk_preflight_is_read_only_and_reports_sdk_import_status() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "sdk-preflight",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_sdk_preflight.v1"
    assert payload["model"] == "X5"
    assert payload["interface"] == "can0"
    assert payload["read_only"] is True
    assert payload["movement_allowed"] is False
    assert payload["sdk"]["module"] == "arx5_interface"
    assert payload["sdk"]["status"] in {"available", "missing"}
    assert payload["next_gate"] == "sdk_runner_confirm_then_hardware_validation"


def test_cli_sysid_sdk_handshake_plan_is_read_only_and_requires_confirmation() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "sdk-handshake-plan",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    step_names = [step["name"] for step in payload["steps"]]

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_sdk_handshake_plan.v1"
    assert payload["read_only"] is True
    assert payload["movement_allowed"] is False
    assert payload["requires_confirm"] == "I UNDERSTAND THIS WILL MOVE THE ARM"
    assert payload["fault_landing_mode"] == "damping"
    assert payload["next_gate"] == "sdk_runner_confirm_then_hardware_validation"
    assert step_names == [
        "sdk_preflight",
        "operator_confirm",
        "enter_hold_or_damping",
        "start_recording_after_safe_state",
        "fault_or_ctrl_c_to_damping",
    ]
