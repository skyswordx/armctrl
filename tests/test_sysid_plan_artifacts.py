import csv
import json
import subprocess
import sys
from pathlib import Path


def test_cli_sysid_plan_writes_manifest_and_trajectory(tmp_path: Path) -> None:
    output_dir = tmp_path / "ident-plan"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "gravity_sweep",
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
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    manifest_path = output_dir / "manifest.json"
    trajectory_path = output_dir / "planned_trajectory.csv"

    assert payload["artifacts"]["manifest"] == str(manifest_path)
    assert payload["artifacts"]["planned_trajectory"] == str(trajectory_path)
    assert payload["artifact_safety"]["allowed"] is True
    assert manifest_path.exists()
    assert trajectory_path.exists()

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema"] == "armctrl.ident_plan_manifest.v1"
    assert manifest["profile"]["name"] == "gravity_sweep"
    assert manifest["safety"]["allowed"] is True
    assert manifest["safety"]["checks"]["urdf_limit_check"]["status"] == "pass"
    assert manifest["handoff"]["solver_backends"] == ["pinocchio", "figaroh"]

    with trajectory_path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    assert rows[0]["time_s"] == "0.000000"
    assert "q_cmd_6" in rows[0]
    assert len(rows) == 41


def test_cli_sysid_execute_with_output_is_rejected(tmp_path: Path) -> None:
    output_dir = tmp_path / "ident-plan"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "gravity_sweep",
            "--execute",
            "--output",
            str(output_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.sysid_plan.v1"
    assert payload["safety"]["allowed"] is False
    assert not output_dir.exists()


def test_cli_sysid_plan_manifest_records_urdf_limit_failure(tmp_path: Path) -> None:
    output_dir = tmp_path / "ident-plan"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "gravity_sweep",
            "--q-center",
            "0",
            "99",
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
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))

    assert payload["artifact_safety"]["allowed"] is False
    assert manifest["safety"]["allowed"] is False
    assert manifest["safety"]["checks"]["urdf_limit_check"]["status"] == "fail"
    assert manifest["safety"]["checks"]["urdf_limit_check"]["violations"][0]["joint_index"] == 2
