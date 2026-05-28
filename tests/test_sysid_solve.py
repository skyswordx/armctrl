import json
import subprocess
import sys
from pathlib import Path


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
