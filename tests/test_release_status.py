import json
import subprocess
import sys


def test_cli_release_status_reports_contract_complete_and_hardware_pending() -> None:
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
    assert payload["version"] == "0.5.0"
    assert payload["branch"] == "codex/armctrl-clean-rebuild"
    assert payload["release_readiness"] == "contracts_complete_hardware_pending"
    assert milestone_ids[:4] == ["v0.2.0", "v0.3.0", "v0.4.0", "v0.5.0"]
    assert payload["milestones"][-1]["status"] == "contract_complete"
    assert "real_sdk_runner" in payload["hardware_pending"]
    assert "n100d_pinocchio_figaroh_validation" in payload["hardware_pending"]
