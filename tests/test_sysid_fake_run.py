import csv
import json
import subprocess
import sys
from pathlib import Path

from armctrl.sysid import SysIdPlanRequest
from armctrl.sysid_run import SdkSysIdRunner, SDK_CONFIRMATION


class RecordingBackend:
    def __init__(self) -> None:
        self.events: list[str] = []

    def enter_hold_or_damping(self) -> None:
        self.events.append("enter_hold_or_damping")

    def read_samples(self, request: SysIdPlanRequest) -> list[dict[str, str]]:
        self.events.append("read_samples")
        return [
            {
                "time_s": "0.000000",
                **{f"q_cmd_{index + 1}": "0.000000" for index in range(request.dof)},
                **{f"q_{index + 1}": "0.000000" for index in range(request.dof)},
                **{f"dq_{index + 1}": "0.000000" for index in range(request.dof)},
                **{f"tau_meas_{index + 1}": "0.000000" for index in range(request.dof)},
            }
        ]

    def enter_damping(self) -> None:
        self.events.append("enter_damping")


def test_cli_sysid_run_fake_writes_raw_samples_and_manifest(tmp_path: Path) -> None:
    output_dir = tmp_path / "ident-run"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "fake",
            "--dof",
            "6",
            "--sample-hz",
            "20",
            "--duration",
            "2",
            "--amplitude",
            "0.1",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    raw_samples_path = output_dir / "raw_samples.csv"
    manifest_path = output_dir / "manifest.json"

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_run.v1"
    assert payload["adapter"] == "fake"
    assert payload["artifacts"]["raw_samples"] == str(raw_samples_path)
    assert raw_samples_path.exists()
    assert manifest_path.exists()

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema"] == "armctrl.sysid_run_manifest.v1"
    assert manifest["adapter"] == "fake"
    assert manifest["sample_count"] == 41
    assert manifest["request"]["urdf_path"] == "configs/models/X5_camera.urdf"
    assert manifest["request"]["dof"] == 6
    assert manifest["handoff"]["dataset_contract"] == "lerobot-compatible"

    with raw_samples_path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    assert rows[0]["time_s"] == "0.000000"
    assert "tau_meas_6" in rows[0]
    assert len(rows) == 41


def test_cli_sysid_run_sdk_is_rejected_until_runner_exists(tmp_path: Path) -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "sdk",
            "--output",
            str(tmp_path / "ident-run"),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["reason"] == "sdk sysid runner requires explicit operator confirmation"
    assert payload["requires_confirm"] == "I UNDERSTAND THIS WILL MOVE THE ARM"
    assert payload["movement_allowed"] is False
    assert payload["fault_landing_mode"] == "damping"
    assert payload["recording_starts_after_safe_state"] is True
    assert payload["next_gate"] == "run sysid sdk-handshake-plan before enabling sdk runner"


def test_sdk_sysid_runner_starts_recording_after_safe_state_and_lands_damping(
    tmp_path: Path,
) -> None:
    backend = RecordingBackend()
    request = SysIdPlanRequest(
        profile_name="gravity_sweep",
        dof=6,
        sample_hz=20,
        duration_s=1,
        amplitude_rad=0.1,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path / "ident-sdk",
    )

    result = SdkSysIdRunner(backend=backend).run(
        request,
        confirm=SDK_CONFIRMATION,
    )

    manifest = json.loads((request.output_dir / "manifest.json").read_text(encoding="utf-8"))

    assert result.adapter == "sdk"
    assert backend.events == [
        "enter_hold_or_damping",
        "read_samples",
        "enter_damping",
    ]
    assert manifest["adapter"] == "sdk"
    assert manifest["safety"]["recording_starts_after_safe_state"] is True
    assert manifest["safety"]["fault_landing_mode"] == "damping"
    assert manifest["safety"]["movement_allowed"] is True
