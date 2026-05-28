import json
import subprocess
import sys
from pathlib import Path


def _create_solved_fake_dataset(output_dir: Path) -> None:
    subprocess.run(
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
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "postprocess",
            "--dataset",
            str(output_dir),
            "--solve",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def test_cli_sysid_package_rejects_when_solver_quality_is_not_complete(
    tmp_path: Path,
) -> None:
    dataset_dir = tmp_path / "ident-run"
    _create_solved_fake_dataset(dataset_dir)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "package",
            "--dataset",
            str(dataset_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    package_manifest = dataset_dir / "processed" / "parameter_package.json"

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.sysid_parameter_package.v1"
    assert payload["quality_gate"]["allowed"] is False
    assert "physical_consistency" in payload["quality_gate"]["missing"]
    assert "figaroh_base_parameters" in payload["quality_gate"]["missing"]
    assert payload["artifacts"]["solver_metrics"].endswith("solver_metrics.json")
    assert not package_manifest.exists()


def test_cli_sysid_package_writes_candidate_when_all_quality_gates_pass(
    tmp_path: Path,
) -> None:
    dataset_dir = tmp_path / "ident-run"
    _create_solved_fake_dataset(dataset_dir)
    solver_metrics_path = dataset_dir / "processed" / "solver_metrics.json"
    metrics = json.loads(solver_metrics_path.read_text(encoding="utf-8"))
    metrics["regressor_condition"]["pinocchio"] = {
        "status": "computed",
        "rank": 36,
        "effective_condition_number": 42.0,
        "row_count": 246,
        "column_count": 60,
    }
    metrics["prediction_error"]["pinocchio"] = {
        "status": "computed",
        "rmse_nm": 0.01,
        "sample_count": 41,
        "observation_count": 246,
        "parameter_count": 60,
    }
    metrics["physical_consistency"] = {"status": "pass"}
    metrics["figaroh_base_parameters"] = {"status": "available"}
    solver_metrics_path.write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "package",
            "--dataset",
            str(dataset_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    package_manifest = dataset_dir / "processed" / "parameter_package.json"

    assert payload["status"] == "ok"
    assert payload["quality_gate"]["allowed"] is True
    assert payload["quality_gate"]["missing"] == []
    assert payload["artifacts"]["parameter_package"] == str(package_manifest)
    assert package_manifest.exists()

    package = json.loads(package_manifest.read_text(encoding="utf-8"))
    assert package["schema"] == "armctrl.parameter_package.v1"
    assert package["package_version"] == "0.5.0"
    assert package["status"] == "candidate"
    assert package["signature"]["algorithm"] == "sha256"
    assert len(package["signature"]["value"]) == 64
    assert package["rollback"]["target"] == "previous_active_parameter_package"
    assert package["rollout"]["ab_validation"]["status"] == "required_before_activation"
    assert package["rollout"]["ab_validation"]["metrics"] == [
        "hold_pose_error",
        "joint_current_rms",
        "torque_prediction_residual",
        "operator_abort_count",
    ]


def test_cli_sysid_package_accepts_external_metrics_with_utf8_bom(
    tmp_path: Path,
) -> None:
    dataset_dir = tmp_path / "ident-run"
    _create_solved_fake_dataset(dataset_dir)
    solver_metrics_path = dataset_dir / "processed" / "solver_metrics.json"
    metrics = json.loads(solver_metrics_path.read_text(encoding="utf-8"))
    metrics["regressor_condition"]["pinocchio"] = {
        "status": "computed",
        "rank": 36,
        "effective_condition_number": 42.0,
        "row_count": 246,
        "column_count": 60,
    }
    metrics["prediction_error"]["pinocchio"] = {
        "status": "computed",
        "rmse_nm": 0.01,
        "sample_count": 41,
        "observation_count": 246,
        "parameter_count": 60,
    }
    metrics["physical_consistency"] = {"status": "pass"}
    metrics["figaroh_base_parameters"] = {"status": "available"}
    solver_metrics_path.write_text(
        "\ufeff" + json.dumps(metrics, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "package",
            "--dataset",
            str(dataset_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["quality_gate"]["allowed"] is True
