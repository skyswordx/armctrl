import json
import subprocess
import sys
import types
from pathlib import Path

import numpy as np

from armctrl.sysid_solve import SysIdSolver


def _create_processed_dataset(output_dir: Path) -> None:
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
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def test_cli_sysid_solve_writes_solver_artifacts_from_processed_dataset(
    tmp_path: Path,
) -> None:
    dataset_dir = tmp_path / "ident-run"
    _create_processed_dataset(dataset_dir)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "solve",
            "--dataset",
            str(dataset_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    solver_metrics = dataset_dir / "processed" / "solver_metrics.json"
    solver_report = dataset_dir / "processed" / "solver_report_zh.md"

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_solve.v1"
    assert payload["artifacts"]["solver_metrics"] == str(solver_metrics)
    assert payload["artifacts"]["solver_report"] == str(solver_report)
    assert solver_metrics.exists()
    assert solver_report.exists()

    metrics = json.loads(solver_metrics.read_text(encoding="utf-8"))
    assert metrics["schema"] == "armctrl.sysid_solver_quality.v1"
    assert metrics["sample_count"] == 41
    assert metrics["input_quality_status"] == "pass"
    assert metrics["overall_verdict"] in {"solver_ready", "solver_handoff_only"}
    assert metrics["backend_status"]["pinocchio"]["status"] in {"available", "missing"}
    assert metrics["backend_status"]["figaroh"]["status"] in {"available", "missing"}
    assert metrics["residual_summary"]["fake_zero_tau_rmse_nm"] == 0.0

    report = solver_report.read_text(encoding="utf-8")
    assert "SysID \u6c42\u89e3\u62a5\u544a" in report
    assert "Pinocchio" in report
    assert "FIGAROH" in report


def test_sysid_solve_computes_pinocchio_regressor_metrics_when_backend_exists(
    tmp_path: Path,
    monkeypatch,
) -> None:
    dataset_dir = tmp_path / "ident-run"
    _create_processed_dataset(dataset_dir)

    class FakeModel:
        nq = 6
        nv = 6

        def createData(self):
            return types.SimpleNamespace()

    def compute_joint_torque_regressor(model, data, q, v, a):
        base = np.zeros((model.nv, 60))
        base[:, : model.nv] = np.eye(model.nv)
        base[:, model.nv : 2 * model.nv] = np.diag(q)
        data.jointTorqueRegressor = base
        return base

    fake_pinocchio = types.SimpleNamespace(
        buildModelFromUrdf=lambda path: FakeModel(),
        computeJointTorqueRegressor=compute_joint_torque_regressor,
    )
    monkeypatch.setitem(sys.modules, "pinocchio", fake_pinocchio)

    SysIdSolver().run(dataset_dir)

    metrics = json.loads(
        (dataset_dir / "processed" / "solver_metrics.json").read_text(
            encoding="utf-8"
        )
    )
    condition = metrics["regressor_condition"]["pinocchio"]

    assert metrics["backend_status"]["pinocchio"]["status"] == "available"
    assert condition["status"] == "computed"
    assert condition["row_count"] == 246
    assert condition["column_count"] == 60
    assert condition["rank"] >= 6
    assert condition["effective_condition_number"] >= 1.0
