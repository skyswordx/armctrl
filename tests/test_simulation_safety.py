import json
import importlib.util
import subprocess
import sys
from pathlib import Path

import pytest

import armctrl.simulation as simulation
from armctrl.simulation import SimulationDoctor, TrajectoryPreviewer
from armctrl.workspace import WorkspaceSafetyConfig


def test_safe_config_loads_named_allowed_and_forbidden_spaces() -> None:
    config = WorkspaceSafetyConfig.from_yaml(Path("configs/x5.safe.yaml"))

    assert config.allowed_workspace_boxes[0].name == "main_body_sweep_volume"
    assert config.allowed_workspace_boxes[0].min_m == (-0.35, -0.45, 0.02)
    assert config.allowed_workspace_boxes[0].max_m == (0.75, 0.45, 0.65)
    assert config.forbidden_workspace_boxes[0].name == "table_surface"
    assert config.forbidden_workspace_boxes[0].max_m[2] == 0.02
    assert config.simulation_backend_preference == (
        "pinocchio_coal",
        "mujoco",
        "moveit",
        "urdf_fk_fallback",
    )


def test_simulation_doctor_reports_mature_backend_importability() -> None:
    result = SimulationDoctor().run()

    assert result["schema"] == "armctrl.simulation_doctor.v1"
    assert result["movement_allowed"] is False
    names = [backend["name"] for backend in result["backends"]]
    assert names == ["pinocchio_coal", "mujoco", "moveit", "figaroh"]
    assert result["backends"][0]["role"] == "lightweight URDF geometry collision checks"
    assert result["backends"][1]["role"] == "contact and dynamics simulation preview"


def test_cli_sim_doctor_is_read_only_json() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "sim", "doctor", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.simulation_doctor.v1"
    assert payload["movement_allowed"] is False
    assert payload["backends"][2]["name"] == "moveit"


def test_sysid_plan_writes_trajectory_preview_and_includes_simulation_gate(
    tmp_path: Path,
) -> None:
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
            "100",
            "--duration",
            "2",
            "--amplitude",
            "0.05",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
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
    preview = json.loads(
        (output_dir / "trajectory_preview.json").read_text(encoding="utf-8")
    )

    assert payload["artifacts"]["trajectory_preview"] == str(
        output_dir / "trajectory_preview.json"
    )
    assert payload["artifact_safety"]["simulation_check"]["status"] == "pass"
    assert manifest["safety"]["checks"]["simulation_check"]["status"] == "pass"
    assert preview["schema"] == "armctrl.trajectory_preview.v1"
    assert preview["backend"]["selected"] in {
        "pinocchio_coal",
        "mujoco",
        "moveit",
        "urdf_fk_fallback",
    }
    assert preview["safety"]["zone_check"]["status"] == "pass"
    assert preview["trajectory_metrics"]["joint_ranges_rad"]["joint_1"] > 0.09


def test_trajectory_preview_rejects_forbidden_workspace_entry(tmp_path: Path) -> None:
    trajectory_path = tmp_path / "planned_trajectory.csv"
    trajectory_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0,3.0,1.0,0,0,0\n",
        encoding="utf-8",
    )

    result = TrajectoryPreviewer().preview(
        trajectory_path=trajectory_path,
        urdf_path=Path("configs/models/X5_camera.urdf"),
        safe_config_path=Path("configs/x5.safe.yaml"),
    )

    assert result["safety"]["allowed"] is False
    assert result["safety"]["zone_check"]["status"] == "fail"
    assert result["safety"]["zone_check"]["violations"][0]["check"] in {
        "outside_allowed_workspace",
        "inside_forbidden_workspace",
    }


def test_auto_preview_records_failed_mature_backend_attempt_and_falls_back(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    trajectory_path = tmp_path / "planned_trajectory.csv"
    trajectory_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0,0.3,0.3,0,0,0\n"
        "0.010000,0.001,0.3,0.3,0,0,0\n",
        encoding="utf-8",
    )

    original_find_spec = importlib.util.find_spec

    def fake_find_spec(name: str) -> object | None:
        if name in {"pinocchio", "coal"}:
            return object()
        return original_find_spec(name)

    monkeypatch.setattr(simulation.importlib.util, "find_spec", fake_find_spec)
    monkeypatch.setattr(
        simulation,
        "_pinocchio_coal_check",
        lambda *, urdf_path, q_samples: {
            "status": "not_evaluated",
            "method": "pinocchio_coal",
            "reason": "Mesh ./meshes/base_link.STL could not be found.",
        },
    )

    result = TrajectoryPreviewer().preview(
        trajectory_path=trajectory_path,
        urdf_path=Path("configs/models/X5_camera.urdf"),
        safe_config_path=Path("configs/x5.safe.yaml"),
    )

    assert result["backend"]["selected"] == "urdf_fk_fallback"
    assert result["backend"]["attempts"][0]["backend"] == "pinocchio_coal"
    assert result["backend"]["attempts"][0]["result"]["status"] == "not_evaluated"
    assert result["backend"]["result"]["status"] == "pass"
    assert result["safety"]["allowed"] is True
