import csv
import json
import math
import os
import subprocess
import sys
import types
from pathlib import Path

from armctrl.sysid_measured import MeasuredSysIdAnalyzeRequest, MeasuredSysIdAnalyzer


def _write_samples(path: Path, *, scale: float, duration_s: float = 0.4) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = (
        ["sample_index", "sent_monotonic_s"]
        + [f"q_cmd_{index}" for index in range(1, 7)]
        + [f"dq_cmd_{index}" for index in range(1, 7)]
        + [f"q_meas_{index}" for index in range(1, 7)]
        + [f"dq_meas_{index}" for index in range(1, 7)]
        + [f"tau_meas_{index}" for index in range(1, 7)]
        + [f"q_error_{index}" for index in range(1, 7)]
        + ["fault_flags"]
    )
    sample_hz = 100.0
    rows = []
    for sample_index in range(int(duration_s * sample_hz) + 1):
        t = sample_index / sample_hz
        row: dict[str, object] = {
            "sample_index": sample_index,
            "sent_monotonic_s": 1000.0 + t,
            "fault_flags": "",
        }
        for joint in range(1, 7):
            phase = 0.3 * joint
            q_cmd = scale * math.sin(2.0 * math.pi * t + phase)
            tracking_error = 0.004 * joint * math.sin(4.0 * math.pi * t + phase)
            q_meas = q_cmd + tracking_error
            dq_cmd = scale * 2.0 * math.pi * math.cos(2.0 * math.pi * t + phase)
            dq_meas = dq_cmd + 0.02 * joint * math.cos(4.0 * math.pi * t + phase)
            tau_meas = 0.3 * joint + 0.5 * q_meas + 0.04 * dq_meas
            row[f"q_cmd_{joint}"] = q_cmd
            row[f"dq_cmd_{joint}"] = dq_cmd
            row[f"q_meas_{joint}"] = q_meas
            row[f"dq_meas_{joint}"] = dq_meas
            row[f"tau_meas_{joint}"] = tau_meas
            row[f"q_error_{joint}"] = q_meas - q_cmd
        rows.append(row)

    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _write_lab_dataset(root: Path) -> None:
    csv_dir = root / "derived_sysid_csv"
    _write_samples(csv_dir / "ident-fourier-full_385f291a_samples.csv", scale=0.45)
    _write_samples(
        csv_dir / "4e6e1eaa-02ba-4dd9-a5d3-1c1221501372_4e6e1eaa_samples.csv",
        scale=0.44,
    )
    _write_samples(
        csv_dir / "ident-fourier-reduced-0p25-normal_72c1056b_samples.csv",
        scale=0.12,
    )
    _write_samples(
        csv_dir / "ident-fourier-reduced-0p25-slow4x_b9973ba1_samples.csv",
        scale=0.12,
        duration_s=0.8,
    )


def test_cli_sysid_analyze_measured_writes_offline_artifacts_without_sdk_import(
    tmp_path: Path,
) -> None:
    dataset_dir = tmp_path / "lab-fourier"
    output_dir = tmp_path / "analysis"
    _write_lab_dataset(dataset_dir)
    poison_dir = tmp_path / "poison"
    poison_dir.mkdir()
    (poison_dir / "arx5_interface.py").write_text(
        "raise RuntimeError('arx5_interface must not be imported')\n",
        encoding="utf-8",
    )
    env = os.environ.copy()
    env["PYTHONPATH"] = str(poison_dir) + os.pathsep + env.get("PYTHONPATH", "")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "analyze-measured",
            "--dataset",
            str(dataset_dir),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--primary",
            "ident-fourier-full_385f291a",
            "--validation",
            "4e6e1eaa-02ba-4dd9-a5d3-1c1221501372_4e6e1eaa",
            "--validation",
            "ident-fourier-reduced-0p25-normal_72c1056b",
            "--validation",
            "ident-fourier-reduced-0p25-slow4x_b9973ba1",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_analyze_measured.v1"
    assert payload["movement_allowed"] is False
    assert payload["hardware_execution_eligible"] is False
    assert payload["parameter_bundle_status"] in {
        "preliminary_measured_fit",
        "offline_validation_ready",
        "physical_consistency_review_ready",
    }
    assert payload["tau_meas_semantics"]["status"] in {
        "unconfirmed",
        "effort_proxy",
        "current_proxy",
        "confirmed_nm",
    }

    artifacts = payload["artifacts"]
    expected_artifacts = {
        "tau_meas_trace_report",
        "tau_meas_trace",
        "processed_measured_samples",
        "preprocessing_summary",
        "preprocessing_report",
        "measured_quality_summary",
        "measured_quality_report",
        "measured_regressor_summary",
        "measured_regressor_report",
        "base_parameter_summary",
        "base_parameter_report",
        "solver_summary",
        "solver_report",
        "validation_summary",
        "validation_report",
        "estimator_comparison_summary",
        "estimator_comparison_report",
        "residual_diagnostics",
        "residual_diagnostics_summary",
        "residual_diagnostics_report",
        "physical_consistency_summary",
        "physical_consistency_report",
        "gravity_comp_offline_sanity",
        "gravity_comp_offline_sanity_report",
        "tau_pred",
        "tau_residual",
        "parameter_bundle_manifest",
        "parameter_bundle_parameters",
    }
    assert expected_artifacts.issubset(artifacts)
    for key in expected_artifacts:
        assert Path(artifacts[key]).exists(), key

    with Path(artifacts["processed_measured_samples"]).open(
        newline="",
        encoding="utf-8",
    ) as file:
        rows = list(csv.DictReader(file))
    assert rows
    assert rows[0]["dataset_label"] == "ident-fourier-full_385f291a"
    assert "dq_meas_filtered_6" in rows[0]
    assert "ddq_meas_filtered_6" in rows[0]
    assert "tau_meas_filtered_6" in rows[0]

    quality = json.loads(Path(artifacts["measured_quality_summary"]).read_text())
    assert quality["primary_label"] == "ident-fourier-full_385f291a"
    primary_quality = quality["datasets"]["ident-fourier-full_385f291a"]
    assert primary_quality["role"] == "primary"
    assert "tracking_warning" in primary_quality
    assert primary_quality["dt_jitter_rms_s"] >= 0.0
    assert primary_quality["dt_jitter_max_abs_s"] >= 0.0
    assert primary_quality["tau_saturation_check"]["status"] in {"pass", "warning"}
    assert len(primary_quality["tau_saturation_check"]["per_joint_near_max_fraction"]) == 6
    assert (
        quality["datasets"]["ident-fourier-reduced-0p25-slow4x_b9973ba1"]["role"]
        == "validation"
    )

    regressor = json.loads(Path(artifacts["measured_regressor_summary"]).read_text())
    assert regressor["backend"] == "pinocchio"
    assert regressor["datasets"]["ident-fourier-full_385f291a"]["status"] in {
        "computed",
        "not_evaluated",
    }
    if regressor["datasets"]["ident-fourier-full_385f291a"]["status"] == "not_evaluated":
        assert regressor["datasets"]["ident-fourier-full_385f291a"]["reason"]

    bundle_manifest = json.loads(
        Path(artifacts["parameter_bundle_manifest"]).read_text(encoding="utf-8")
    )
    assert bundle_manifest["status"] in {
        "preliminary_measured_fit",
        "offline_validation_ready",
        "physical_consistency_review_ready",
    }
    assert bundle_manifest["source_datasets"]["primary"] == "ident-fourier-full_385f291a"
    assert bundle_manifest["urdf_path"] == "configs/models/X5_camera.urdf"
    assert bundle_manifest["payload_model"]["d435_mass_included_in_link6"] is True
    assert "source_bundle" in bundle_manifest
    assert bundle_manifest["estimator_comparison_status"] in {
        "computed",
        "not_evaluated",
    }
    assert bundle_manifest["gravity_sanity_status"] in {"computed", "not_evaluated"}
    assert bundle_manifest["rollback_note"]

    bundle_parameters = json.loads(
        Path(artifacts["parameter_bundle_parameters"]).read_text(encoding="utf-8")
    )
    assert bundle_parameters["schema"] == "armctrl.x5_body_dynamics_parameters.v1"
    assert bundle_parameters["status"] in {
        "preliminary_measured_fit",
        "offline_validation_ready",
        "physical_consistency_review_ready",
    }
    assert "solver_summary" not in bundle_parameters


def test_measured_analyzer_computes_regressor_and_validation_when_pinocchio_exists(
    tmp_path: Path,
    monkeypatch,
) -> None:
    dataset_dir = tmp_path / "lab-fourier"
    output_dir = tmp_path / "analysis"
    _write_lab_dataset(dataset_dir)

    class FakeModel:
        nq = 6
        nv = 6

        def createData(self):
            return types.SimpleNamespace(jointTorqueRegressor=None)

    fake_pinocchio = types.SimpleNamespace()
    fake_pinocchio.buildModelFromUrdf = lambda _path: FakeModel()

    def fake_regressor(_model, _data, q, v, a):
        features = [1.0, *q, *v, *a]
        matrix = []
        for joint in range(6):
            row = [0.0 for _ in range(60)]
            for feature_index, feature in enumerate(features):
                row[(joint * 10 + feature_index) % 60] = float(feature)
            matrix.append(row)
        return matrix

    fake_pinocchio.computeJointTorqueRegressor = fake_regressor
    fake_pinocchio.computeGeneralizedGravity = (
        lambda _model, _data, q: [0.1 * (index + 1) + float(q[index]) for index in range(6)]
    )
    fake_pinocchio.__version__ = "fake"

    real_find_spec = __import__("importlib.util").util.find_spec

    def fake_find_spec(name):
        if name == "pinocchio":
            return object()
        return real_find_spec(name)

    monkeypatch.setitem(sys.modules, "pinocchio", fake_pinocchio)
    monkeypatch.setattr("importlib.util.find_spec", fake_find_spec)

    result = MeasuredSysIdAnalyzer().run(
        MeasuredSysIdAnalyzeRequest(
            dataset_dir=dataset_dir,
            urdf_path="configs/models/X5_camera.urdf",
            primary_label="ident-fourier-full_385f291a",
            validation_labels=(
                "4e6e1eaa-02ba-4dd9-a5d3-1c1221501372_4e6e1eaa",
                "ident-fourier-reduced-0p25-normal_72c1056b",
            ),
            output_dir=output_dir,
        )
    )

    payload = result.to_json()
    regressor = json.loads(
        Path(payload["artifacts"]["measured_regressor_summary"]).read_text(
            encoding="utf-8"
        )
    )
    validation = json.loads(
        Path(payload["artifacts"]["validation_summary"]).read_text(encoding="utf-8")
    )
    estimator_comparison = json.loads(
        Path(payload["artifacts"]["estimator_comparison_summary"]).read_text(
            encoding="utf-8"
        )
    )
    residual_diagnostics = json.loads(
        Path(payload["artifacts"]["residual_diagnostics_summary"]).read_text(
            encoding="utf-8"
        )
    )
    physical = json.loads(
        Path(payload["artifacts"]["physical_consistency_summary"]).read_text(
            encoding="utf-8"
        )
    )
    gravity = json.loads(
        Path(payload["artifacts"]["gravity_comp_offline_sanity"]).read_text(
            encoding="utf-8"
        )
    )
    solver = json.loads(
        Path(payload["artifacts"]["solver_summary"]).read_text(encoding="utf-8")
    )
    base = json.loads(
        Path(payload["artifacts"]["base_parameter_summary"]).read_text(encoding="utf-8")
    )
    tau_pred_rows = list(
        csv.DictReader(
            Path(payload["artifacts"]["tau_pred"]).open(newline="", encoding="utf-8")
        )
    )
    tau_residual_rows = list(
        csv.DictReader(
            Path(payload["artifacts"]["tau_residual"]).open(newline="", encoding="utf-8")
        )
    )

    assert regressor["datasets"]["ident-fourier-full_385f291a"]["status"] == "computed"
    assert regressor["datasets"]["ident-fourier-full_385f291a"]["rank"] > 0
    assert (
        regressor["datasets"]["ident-fourier-full_385f291a"][
            "effective_condition_number"
        ]
        > 0
    )
    assert (
        base["datasets"]["ident-fourier-full_385f291a"]["method"]
        == "numeric_qr_proxy"
    )
    primary_base = base["datasets"]["ident-fourier-full_385f291a"]
    primary_solver = solver["datasets"]["ident-fourier-full_385f291a"]
    assert primary_solver["parameter_space"] == "numeric_qr_proxy_base"
    assert primary_solver["selected_columns"] == primary_base["selected_columns"]
    assert primary_solver["parameter_count"] == len(primary_base["selected_columns"])
    assert len(primary_solver["per_joint_rmse"]) == 6
    assert len(primary_solver["per_joint_max_abs"]) == 6
    assert len(primary_solver["per_joint_nrmse"]) == 6
    assert validation["comparisons"][0]["status"] == "computed"
    assert len(validation["comparisons"][0]["per_joint_rmse"]) == 6
    assert estimator_comparison["status"] == "computed"
    assert set(estimator_comparison["estimators"]) >= {
        "OLS",
        "WLS_per_joint_residual_variance",
        "robust_soft_l1",
    }
    assert estimator_comparison["chosen_estimator"] in estimator_comparison["estimators"]
    assert "scipy.optimize.least_squares" in estimator_comparison["external_references"][0]["name"]
    assert residual_diagnostics["status"] == "computed"
    assert residual_diagnostics["datasets"][0]["top_outliers"]
    assert len(residual_diagnostics["datasets"][0]["per_joint_histogram_stats"]) == 6
    assert "correlation_with_q_dq_ddq" in residual_diagnostics["datasets"][0]
    assert physical["status"] == "review_ready"
    assert physical["base_parameter_limitation"]
    assert physical["full_inertial_projection_next_step"]
    assert gravity["status"] == "computed"
    assert gravity["identified_effort_model"]["status"] == "not_injected"
    assert gravity["can_use_for_real_nm_gravity_compensation"] is False
    assert tau_pred_rows
    assert tau_pred_rows[0]["dataset_label"] == "ident-fourier-full_385f291a"
    assert "tau_pred_6" in tau_pred_rows[0]
    assert tau_residual_rows
    assert tau_residual_rows[0]["dataset_label"] == "ident-fourier-full_385f291a"
    assert "tau_residual_6" in tau_residual_rows[0]

    bundle_parameters = json.loads(
        Path(payload["artifacts"]["parameter_bundle_parameters"]).read_text(
            encoding="utf-8"
        )
    )
    assert bundle_parameters["primary_label"] == "ident-fourier-full_385f291a"
    assert (
        bundle_parameters["active_parameter_set"]["parameter_space"]
        == "numeric_qr_proxy_base"
    )
    assert bundle_parameters["active_parameter_set"]["selected_columns"] == primary_base[
        "selected_columns"
    ]
    assert bundle_parameters["active_parameter_set"]["parameter_count"] == len(
        primary_base["selected_columns"]
    )
    assert len(bundle_parameters["active_parameter_set"]["parameter_vector"]) == len(
        primary_base["selected_columns"]
    )
    assert bundle_parameters["tau_signal_name"] == "tau_meas / effort signal"
    assert bundle_parameters["chosen_offline_estimator"] == estimator_comparison[
        "chosen_estimator"
    ]
    assert payload["parameter_bundle_status"] == "physical_consistency_review_ready"
