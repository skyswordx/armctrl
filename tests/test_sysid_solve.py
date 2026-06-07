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
    assert metrics["physical_consistency"] == {
        "status": "not_evaluated",
        "source": "figaroh_or_manual_review_required",
    }
    assert metrics["figaroh_base_parameters"] == {
        "status": "not_evaluated",
        "source": "figaroh_required",
    }
    assert metrics["residual_summary"]["fake_zero_tau_rmse_nm"] == 0.0

    report = solver_report.read_text(encoding="utf-8")
    assert "SysID \u6c42\u89e3\u62a5\u544a" in report
    assert "Pinocchio" in report
    assert "FIGAROH" in report
    assert "物理一致性" in report
    assert "基础参数" in report


def test_cli_sysid_solve_preserves_passed_sdk_source_run_evidence(
    tmp_path: Path,
) -> None:
    dataset_dir = tmp_path / "ident-sdk-run"
    _create_processed_dataset(dataset_dir)
    manifest_path = dataset_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest.update(
        {
            "adapter": "sdk",
            "run_status": "completed",
            "readiness": {
                "artifact_path": str(tmp_path / "readiness.json"),
                "agent_sysid_smoke_allowed": True,
            },
            "motion_runtime": {
                "schema": "armctrl.motion_runtime_result.v1",
                "status": "completed",
                "producer": "sysid",
                "mode": "trajectory_replay",
                "trajectory_sample_hz": 100.0,
                "actual_send_hz": 99.4,
                "send_jitter_ms_p95": 1.1,
                "send_jitter_ms_p99": 2.4,
                "controller_dt_s": 0.002,
                "sample_count": 11,
                "fault_flags": [],
                "tracking": {
                    "q_cmd_delta_max_abs_rad": 0.01,
                    "q_meas_delta_max_abs_rad": 0.009,
                    "max_abs_sample_tracking_error_rad": 0.0015,
                    "final_tracking_error_max_abs_rad": 0.0007,
                },
                "samples": [
                    {
                        "q_cmd": [0.0, 0.31],
                        "q_meas": [0.0, 0.3093],
                    }
                ],
                "landing_mode": "hold",
            },
            "acceptance": {
                "schema": "armctrl.real_motion_acceptance.v1",
                "stage": "sysid_smoke",
                "status": "pass",
                "next_gate": "operator_review_before_larger_sysid_or_parameter_solve",
            },
        }
    )
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

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
    metrics = json.loads(
        (dataset_dir / "processed" / "solver_metrics.json").read_text(
            encoding="utf-8"
        )
    )
    source_run = metrics["source_run"]

    assert payload["status"] == "ok"
    assert source_run == {
        "schema": "armctrl.sysid_solver_source_run.v1",
        "manifest_schema": "armctrl.sysid_run_manifest.v1",
        "adapter": "sdk",
        "run_status": "completed",
        "acceptance": {
            "stage": "sysid_smoke",
            "status": "pass",
            "next_gate": "operator_review_before_larger_sysid_or_parameter_solve",
        },
        "readiness": {
            "artifact_path": str(tmp_path / "readiness.json"),
            "agent_sysid_smoke_allowed": True,
        },
        "motion_runtime": {
            "status": "completed",
            "producer": "sysid",
            "mode": "trajectory_replay",
            "trajectory_sample_hz": 100.0,
            "actual_send_hz": 99.4,
            "send_jitter_ms_p95": 1.1,
            "send_jitter_ms_p99": 2.4,
            "controller_dt_s": 0.002,
            "sample_count": 11,
            "fault_flags": [],
            "tracking": {
                "q_cmd_delta_max_abs_rad": 0.01,
                "q_meas_delta_max_abs_rad": 0.009,
                "max_abs_sample_tracking_error_rad": 0.0015,
                "final_tracking_error_max_abs_rad": 0.0007,
            },
            "landing_mode": "hold",
        },
    }
    assert "samples" not in source_run["motion_runtime"]

    report = (dataset_dir / "processed" / "solver_report_zh.md").read_text(
        encoding="utf-8"
    )
    assert "- source_run.adapter: `sdk`" in report
    assert "- source_run.run_status: `completed`" in report
    assert "- source_run.acceptance: `sysid_smoke/pass`" in report
    assert "- source_run.readiness.agent_sysid_smoke_allowed: `True`" in report
    assert "- source_run.motion_runtime.status: `completed`" in report
    assert "- source_run.motion_runtime.actual_send_hz: `99.4`" in report
    assert "- source_run.motion_runtime.controller_dt_s: `0.002`" in report
    assert (
        "- source_run.tracking.final_tracking_error_max_abs_rad: `0.0007`" in report
    )
    assert "- source_run.motion_runtime.fault_flags: `[]`" in report
    assert "- source_run.motion_runtime.landing_mode: `hold`" in report


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


def test_sysid_solve_reports_pinocchio_prediction_error_for_solved_parameters(
    tmp_path: Path,
    monkeypatch,
) -> None:
    dataset_dir = tmp_path / "ident-run"
    _create_processed_dataset(dataset_dir)
    processed_csv = dataset_dir / "processed" / "processed_samples.csv"

    class FakeModel:
        nq = 6
        nv = 6

        def createData(self):
            return types.SimpleNamespace()

    true_parameters = np.array([1.0, -2.0, 3.0, -4.0, 5.0, -6.0])

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

    rows = processed_csv.read_text(encoding="utf-8").splitlines()
    header = rows[0].split(",")
    rewritten_rows = [rows[0]]
    for line in rows[1:]:
        values = line.split(",")
        row = dict(zip(header, values, strict=True))
        q = np.array([float(row[f"q_proc_{index + 1}"]) for index in range(6)])
        regressor = compute_joint_torque_regressor(
            FakeModel(),
            types.SimpleNamespace(),
            q,
            q * 0,
            q * 0,
        )
        tau = regressor @ np.r_[true_parameters, np.zeros(54)]
        for index, value in enumerate(tau, start=1):
            row[f"tau_proc_{index}"] = f"{value:.9f}"
        rewritten_rows.append(",".join(row[column] for column in header))
    processed_csv.write_text("\n".join(rewritten_rows) + "\n", encoding="utf-8")

    SysIdSolver().run(dataset_dir)

    metrics = json.loads(
        (dataset_dir / "processed" / "solver_metrics.json").read_text(
            encoding="utf-8"
        )
    )
    prediction = metrics["prediction_error"]["pinocchio"]

    assert prediction["status"] == "computed"
    assert prediction["rmse_nm"] < 1e-8
    assert prediction["sample_count"] == 41
    assert prediction["parameter_count"] == 60
