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
    assert payload["next_gate"] == "real_sdk_runner_pending"
