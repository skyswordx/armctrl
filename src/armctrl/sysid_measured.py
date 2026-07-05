from __future__ import annotations

from dataclasses import dataclass
import csv
import importlib.util
import json
import math
from pathlib import Path
from time import strftime
from typing import Any, Sequence

import numpy as np
from scipy.linalg import qr
from scipy.optimize import least_squares
from scipy.signal import savgol_filter


@dataclass(frozen=True)
class MeasuredSysIdAnalyzeRequest:
    dataset_dir: Path
    urdf_path: str
    primary_label: str
    validation_labels: tuple[str, ...]
    output_dir: Path
    filter_method: str = "savgol"
    savgol_window_samples: int = 21
    savgol_polyorder: int = 3


@dataclass(frozen=True)
class MeasuredSysIdAnalyzeResult:
    schema: str
    status: str
    parameter_bundle_status: str
    artifacts: dict[str, str]
    tau_meas_semantics: dict[str, object]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "status": self.status,
            "movement_allowed": False,
            "hardware_execution_eligible": False,
            "parameter_bundle_status": self.parameter_bundle_status,
            "tau_meas_semantics": self.tau_meas_semantics,
            "artifacts": self.artifacts,
        }


class MeasuredSysIdAnalyzer:
    def run(self, request: MeasuredSysIdAnalyzeRequest) -> MeasuredSysIdAnalyzeResult:
        request.output_dir.mkdir(parents=True, exist_ok=True)
        datasets = _load_requested_datasets(request)
        tau_trace = _tau_meas_trace()
        preprocessed = _preprocess_datasets(datasets, request)
        processed_path = request.output_dir / "processed_measured_samples.csv"
        _write_csv(processed_path, preprocessed["rows"])

        preprocessing_summary_path = request.output_dir / "preprocessing_summary.json"
        preprocessing_summary_path.write_text(
            json.dumps(preprocessed["summary"], ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        preprocessing_report_path = request.output_dir / "preprocessing_report.md"
        preprocessing_report_path.write_text(
            _preprocessing_report(preprocessed["summary"]),
            encoding="utf-8",
        )

        tau_trace_path = request.output_dir / "tau_meas_trace.json"
        tau_trace_path.write_text(
            json.dumps(tau_trace, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        tau_trace_report_path = request.output_dir / "tau_meas_trace_report.md"
        tau_trace_report_path.write_text(
            _tau_meas_trace_report(tau_trace),
            encoding="utf-8",
        )

        quality = _quality_summary(
            datasets=datasets,
            preprocessed=preprocessed,
            primary_label=request.primary_label,
            validation_labels=request.validation_labels,
        )
        quality_summary_path = request.output_dir / "measured_quality_summary.json"
        quality_summary_path.write_text(
            json.dumps(quality, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        quality_report_path = request.output_dir / "measured_quality_report.md"
        quality_report_path.write_text(_quality_report(quality), encoding="utf-8")

        regressor = _regressor_summary(
            preprocessed=preprocessed,
            urdf_path=request.urdf_path,
        )
        regressor_summary_path = request.output_dir / "measured_regressor_summary.json"
        regressor_summary_path.write_text(
            json.dumps(_json_sanitized(regressor), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        regressor_report_path = request.output_dir / "measured_regressor_report.md"
        regressor_report_path.write_text(_regressor_report(regressor), encoding="utf-8")

        base = _base_parameter_summary(regressor)
        base_summary_path = request.output_dir / "base_parameter_summary.json"
        base_summary_path.write_text(
            json.dumps(base, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        base_report_path = request.output_dir / "base_parameter_report.md"
        base_report_path.write_text(_base_parameter_report(base), encoding="utf-8")

        solver = _solver_summary(
            regressor=regressor,
            base=base,
            primary_label=request.primary_label,
        )
        solver_summary_path = request.output_dir / "solver_summary.json"
        solver_summary_path.write_text(
            json.dumps(solver, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        solver_report_path = request.output_dir / "solver_report.md"
        solver_report_path.write_text(_solver_report(solver), encoding="utf-8")

        validation = _validation_summary(
            regressor=regressor,
            base=base,
            primary_label=request.primary_label,
            validation_labels=request.validation_labels,
        )
        validation_summary_path = request.output_dir / "validation_summary.json"
        validation_summary_path.write_text(
            json.dumps(validation, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        validation_report_path = request.output_dir / "validation_report.md"
        validation_report_path.write_text(
            _validation_report(validation),
            encoding="utf-8",
        )

        estimator_comparison = _estimator_comparison_summary(
            regressor=regressor,
            base=base,
            primary_label=request.primary_label,
            validation_labels=request.validation_labels,
        )
        estimator_comparison_summary_path = request.output_dir / "estimator_comparison_summary.json"
        estimator_comparison_summary_path.write_text(
            json.dumps(_json_sanitized(estimator_comparison), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        estimator_comparison_report_path = request.output_dir / "estimator_comparison_report.md"
        estimator_comparison_report_path.write_text(
            _estimator_comparison_report(estimator_comparison),
            encoding="utf-8",
        )

        prediction = _prediction_artifacts(
            regressor=regressor,
            base=base,
            estimator_comparison=estimator_comparison,
            primary_label=request.primary_label,
            labels=(request.primary_label, *request.validation_labels),
        )
        tau_pred_path = request.output_dir / "tau_pred.csv"
        _write_csv(tau_pred_path, prediction["tau_pred_rows"])
        tau_residual_path = request.output_dir / "tau_residual.csv"
        _write_csv(tau_residual_path, prediction["tau_residual_rows"])

        residual_diagnostics = _residual_diagnostics(
            prediction=prediction,
            preprocessed=preprocessed,
            labels=(request.primary_label, *request.validation_labels),
        )
        residual_diagnostics_path = request.output_dir / "residual_diagnostics.csv"
        _write_csv(residual_diagnostics_path, residual_diagnostics["rows"])
        residual_diagnostics_summary_path = request.output_dir / "residual_diagnostics_summary.json"
        residual_diagnostics_summary_path.write_text(
            json.dumps(_json_sanitized(residual_diagnostics["summary"]), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        residual_diagnostics_report_path = request.output_dir / "residual_diagnostics_report.md"
        residual_diagnostics_report_path.write_text(
            _residual_diagnostics_report(residual_diagnostics["summary"]),
            encoding="utf-8",
        )

        physical = _physical_consistency_summary(
            base=base,
            estimator_comparison=estimator_comparison,
            primary_label=request.primary_label,
        )
        physical_summary_path = request.output_dir / "physical_consistency_summary.json"
        physical_summary_path.write_text(
            json.dumps(_json_sanitized(physical), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        physical_report_path = request.output_dir / "physical_consistency_report.md"
        physical_report_path.write_text(_physical_consistency_report(physical), encoding="utf-8")

        gravity = _gravity_comp_offline_sanity(
            preprocessed=preprocessed,
            prediction=prediction,
            urdf_path=request.urdf_path,
            primary_label=request.primary_label,
        )
        gravity_summary_path = request.output_dir / "gravity_comp_offline_sanity.json"
        gravity_summary_path.write_text(
            json.dumps(_json_sanitized(gravity), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        gravity_report_path = request.output_dir / "gravity_comp_offline_sanity_report.md"
        gravity_report_path.write_text(_gravity_comp_offline_sanity_report(gravity), encoding="utf-8")

        bundle_status = _bundle_status(
            regressor=regressor,
            validation=validation,
            estimator_comparison=estimator_comparison,
            physical=physical,
        )
        bundle_dir = request.output_dir / "parameter_bundles" / (
            "x5_body_dynamics_" + strftime("%Y%m%d-%H%M%S")
        )
        bundle_dir.mkdir(parents=True, exist_ok=True)
        bundle_manifest = _bundle_manifest(
            request=request,
            tau_trace=tau_trace,
            preprocessed=preprocessed,
            regressor=regressor,
            base=base,
            solver=solver,
            validation=validation,
            estimator_comparison=estimator_comparison,
            physical=physical,
            gravity=gravity,
            status=bundle_status,
        )
        parameter_bundle_manifest_path = bundle_dir / "manifest.json"
        parameter_bundle_manifest_path.write_text(
            json.dumps(bundle_manifest, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        parameter_bundle_parameters_path = bundle_dir / "parameters.json"
        parameter_bundle_payload = _parameter_bundle_payload(
            request=request,
            status=bundle_status,
            tau_trace=tau_trace,
            base=base,
            solver=solver,
            prediction=prediction,
            estimator_comparison=estimator_comparison,
        )
        _write_bundle_json(parameter_bundle_parameters_path, parameter_bundle_payload)
        _write_bundle_json(bundle_dir / "preprocessing_summary.json", preprocessed["summary"])
        _write_bundle_json(bundle_dir / "regressor_summary.json", regressor)
        _write_bundle_json(bundle_dir / "solver_summary.json", solver)
        _write_bundle_json(bundle_dir / "validation_summary.json", validation)
        _write_bundle_json(bundle_dir / "estimator_comparison_summary.json", estimator_comparison)
        _write_bundle_json(bundle_dir / "physical_consistency_summary.json", physical)
        _write_bundle_json(bundle_dir / "gravity_comp_offline_sanity.json", gravity)
        (bundle_dir / "limitations.md").write_text(
            _limitations_report(bundle_manifest),
            encoding="utf-8",
        )

        artifacts = {
            "tau_meas_trace_report": str(tau_trace_report_path),
            "tau_meas_trace": str(tau_trace_path),
            "processed_measured_samples": str(processed_path),
            "preprocessing_summary": str(preprocessing_summary_path),
            "preprocessing_report": str(preprocessing_report_path),
            "measured_quality_summary": str(quality_summary_path),
            "measured_quality_report": str(quality_report_path),
            "measured_regressor_summary": str(regressor_summary_path),
            "measured_regressor_report": str(regressor_report_path),
            "base_parameter_summary": str(base_summary_path),
            "base_parameter_report": str(base_report_path),
            "solver_summary": str(solver_summary_path),
            "solver_report": str(solver_report_path),
            "validation_summary": str(validation_summary_path),
            "validation_report": str(validation_report_path),
            "estimator_comparison_summary": str(estimator_comparison_summary_path),
            "estimator_comparison_report": str(estimator_comparison_report_path),
            "residual_diagnostics": str(residual_diagnostics_path),
            "residual_diagnostics_summary": str(residual_diagnostics_summary_path),
            "residual_diagnostics_report": str(residual_diagnostics_report_path),
            "physical_consistency_summary": str(physical_summary_path),
            "physical_consistency_report": str(physical_report_path),
            "gravity_comp_offline_sanity": str(gravity_summary_path),
            "gravity_comp_offline_sanity_report": str(gravity_report_path),
            "tau_pred": str(tau_pred_path),
            "tau_residual": str(tau_residual_path),
            "parameter_bundle_manifest": str(parameter_bundle_manifest_path),
            "parameter_bundle_parameters": str(parameter_bundle_parameters_path),
        }
        return MeasuredSysIdAnalyzeResult(
            schema="armctrl.sysid_analyze_measured.v1",
            status="ok",
            parameter_bundle_status=bundle_status,
            artifacts=artifacts,
            tau_meas_semantics=tau_trace["conclusion"],
        )


def _load_requested_datasets(
    request: MeasuredSysIdAnalyzeRequest,
) -> dict[str, dict[str, Any]]:
    csv_dir = request.dataset_dir / "derived_sysid_csv"
    labels = (request.primary_label, *request.validation_labels)
    datasets: dict[str, dict[str, Any]] = {}
    for label in labels:
        path = csv_dir / f"{label}_samples.csv"
        if not path.exists():
            matches = sorted(csv_dir.glob(f"{label}*_samples.csv"))
            if matches:
                path = matches[0]
        if not path.exists():
            raise FileNotFoundError(f"measured samples CSV not found for label {label}")
        rows = _read_csv(path)
        datasets[label] = {
            "label": label,
            "path": path,
            "rows": rows,
            "role": "primary" if label == request.primary_label else "validation",
        }
    return datasets


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _tau_meas_trace() -> dict[str, object]:
    return {
        "schema": "armctrl.tau_meas_trace.v1",
        "code_path": [
            "armctrl.motion_runtime.JointStateSnapshot.tau_meas",
            "armctrl.motion_runtime.MotionAuditSample.tau_meas",
            "armctrl.arx5_sdk_joint_runtime.Arx5SdkJointRuntimeBackend.read_joint_state",
            "self._controller.get_joint_state().torque()",
            "armctrl.arx5_sdk_joint_runtime.Arx5SdkJointRuntimeBackend._rows_from_motion_result",
            "derived_sysid_csv/tau_meas_1..6",
        ],
        "sdk_source_status": "python_binding_field_confirmed_unit_unconfirmed",
        "sdk_evidence": [
            "arx5_interface/python/arx5_interface.pyi defines class JointState with torque: NDArray and def torque() -> NDArray[np.float64]",
            "armctrl sysid runtime copies joint_state.torque() into JointStateSnapshot.tau_meas without scale conversion",
            "real_stanford_arx5_sdk/include/app/common.h defines JointState::torque as a VecDoF field",
            "real_stanford_arx5_sdk/src/app/controller_base.cpp fills joint_state_.torque from motor current_actual_float multiplied by motor-type torque constants",
            "the same SDK source comments 'Torque: matching the values (there must be something wrong)' and questions one EC_A4310 torque-constant expression, so offline analysis must not treat the scale as verified Nm",
        ],
        "external_practice_notes": [
            "Robot dynamics identification should use measured state and measured effort/torque, not command-only trajectories.",
            "Torque/current scaling must be validated before claiming physical inertial parameters.",
        ],
        "external_references": [
            {
                "name": "Pinocchio computeJointTorqueRegressor",
                "url": "https://docs.ros.org/en/rolling/p/pinocchio/generated/function_namespacepinocchio_1af97c3d2d695ef4636bf010c2ff6031e8.html",
            },
            {
                "name": "FIGAROH PyPI dynamic identification and filtering/pre-processing",
                "url": "https://pypi.org/project/figaroh/",
            },
        ],
        "conclusion": {
            "status": "effort_proxy",
            "signal_name": "tau_meas / effort signal",
            "can_claim_nm": False,
            "reason": "SDK binding exposes JointState.torque() and vendor C++ fills it from measured motor current times torque constants, but the inspected source contains torque-scaling uncertainty comments; the offline pass treats it as an effort proxy until Nm scale is externally validated.",
        },
    }


def _preprocess_datasets(
    datasets: dict[str, dict[str, Any]],
    request: MeasuredSysIdAnalyzeRequest,
) -> dict[str, Any]:
    output_rows: list[dict[str, object]] = []
    summaries: dict[str, object] = {}
    by_label: dict[str, dict[str, np.ndarray]] = {}
    for label, dataset in datasets.items():
        rows = dataset["rows"]
        time_s = _time_vector(rows)
        q_cmd = _matrix(rows, "q_cmd")
        q_meas = _matrix(rows, "q_meas")
        dq_raw = _matrix(rows, "dq_meas")
        tau_raw = _matrix(rows, "tau_meas")
        q_error = _matrix(rows, "q_error")
        q_filtered, dq_filtered, ddq_filtered, edge_drop = _filter_q(
            time_s,
            q_meas,
            window_samples=request.savgol_window_samples,
            polyorder=request.savgol_polyorder,
        )
        tau_filtered = _filter_signal(
            tau_raw,
            window_samples=min(request.savgol_window_samples, len(rows)),
            polyorder=min(request.savgol_polyorder, 3),
        )
        by_label[label] = {
            "time_s": time_s,
            "q_cmd": q_cmd,
            "q_meas": q_meas,
            "q_filtered": q_filtered,
            "dq_raw": dq_raw,
            "dq_filtered": dq_filtered,
            "ddq_filtered": ddq_filtered,
            "tau_raw": tau_raw,
            "tau_filtered": tau_filtered,
            "q_error": q_error,
            "edge_drop_mask": _edge_mask(len(rows), edge_drop),
        }
        summaries[label] = {
            "role": dataset["role"],
            "sample_count": len(rows),
            "filter_method": "savgol_derivative",
            "savgol_window_samples": int(_effective_window(len(rows), request.savgol_window_samples)),
            "savgol_polyorder": int(request.savgol_polyorder),
            "edge_drop_samples": int(edge_drop),
            "external_practice_note": (
                "Savitzky-Golay is used here for local polynomial smoothing and "
                "derivative estimation; zero-phase Butterworth/filtfilt remains "
                "a configured future option for longer datasets."
            ),
            "external_references": [
                {
                    "name": "SciPy savgol_filter",
                    "url": "https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.savgol_filter.html",
                },
                {
                    "name": "SciPy filtfilt zero-phase filtering",
                    "url": "https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.filtfilt.html",
                },
            ],
        }
        for row_index, source in enumerate(rows):
            row: dict[str, object] = {
                "dataset_label": label,
                "role": dataset["role"],
                "sample_index": source.get("sample_index", row_index),
                "time_s": float(time_s[row_index]),
                "fault_flags": source.get("fault_flags", ""),
            }
            for joint in range(6):
                suffix = str(joint + 1)
                row[f"q_cmd_{suffix}"] = float(q_cmd[row_index, joint])
                row[f"q_meas_{suffix}"] = float(q_meas[row_index, joint])
                row[f"q_meas_filtered_{suffix}"] = float(q_filtered[row_index, joint])
                row[f"dq_meas_raw_{suffix}"] = float(dq_raw[row_index, joint])
                row[f"dq_meas_filtered_{suffix}"] = float(dq_filtered[row_index, joint])
                row[f"ddq_meas_filtered_{suffix}"] = float(ddq_filtered[row_index, joint])
                row[f"tau_meas_raw_{suffix}"] = float(tau_raw[row_index, joint])
                row[f"tau_meas_filtered_{suffix}"] = float(tau_filtered[row_index, joint])
                row[f"q_error_{suffix}"] = float(q_error[row_index, joint])
            output_rows.append(row)
    return {
        "schema": "armctrl.measured_preprocessing.v1",
        "rows": output_rows,
        "by_label": by_label,
        "summary": {
            "schema": "armctrl.measured_preprocessing_summary.v1",
            "dataset_count": len(datasets),
            "filter_method": request.filter_method,
            "datasets": summaries,
        },
    }


def _time_vector(rows: list[dict[str, str]]) -> np.ndarray:
    raw = np.array([float(row["sent_monotonic_s"]) for row in rows], dtype=float)
    if raw.size:
        raw = raw - raw[0]
    return raw


def _matrix(rows: list[dict[str, str]], prefix: str) -> np.ndarray:
    return np.array(
        [[float(row[f"{prefix}_{joint}"]) for joint in range(1, 7)] for row in rows],
        dtype=float,
    )


def _effective_window(sample_count: int, requested: int) -> int:
    window = min(int(requested), sample_count if sample_count % 2 else sample_count - 1)
    window = max(5, window)
    if window >= sample_count:
        window = sample_count if sample_count % 2 else sample_count - 1
    if window % 2 == 0:
        window -= 1
    return max(3, window)


def _filter_q(
    time_s: np.ndarray,
    q: np.ndarray,
    *,
    window_samples: int,
    polyorder: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    window = _effective_window(len(time_s), window_samples)
    order = min(int(polyorder), window - 2)
    dt = float(np.mean(np.diff(time_s))) if len(time_s) > 1 else 0.01
    q_filtered = savgol_filter(q, window, order, axis=0, mode="interp")
    dq_filtered = savgol_filter(
        q,
        window,
        order,
        deriv=1,
        delta=dt,
        axis=0,
        mode="interp",
    )
    ddq_filtered = savgol_filter(
        q,
        window,
        order,
        deriv=2,
        delta=dt,
        axis=0,
        mode="interp",
    )
    return q_filtered, dq_filtered, ddq_filtered, window // 2


def _filter_signal(
    values: np.ndarray,
    *,
    window_samples: int,
    polyorder: int,
) -> np.ndarray:
    window = _effective_window(len(values), window_samples)
    order = min(int(polyorder), window - 2)
    return savgol_filter(values, window, order, axis=0, mode="interp")


def _edge_mask(sample_count: int, edge_drop: int) -> np.ndarray:
    mask = np.ones(sample_count, dtype=bool)
    if edge_drop > 0 and sample_count > 2 * edge_drop:
        mask[:edge_drop] = False
        mask[-edge_drop:] = False
    return mask


def _quality_summary(
    *,
    datasets: dict[str, dict[str, Any]],
    preprocessed: dict[str, Any],
    primary_label: str,
    validation_labels: Sequence[str],
) -> dict[str, object]:
    summary: dict[str, object] = {
        "schema": "armctrl.measured_quality_summary.v1",
        "primary_label": primary_label,
        "validation_labels": list(validation_labels),
        "datasets": {},
    }
    by_label = preprocessed["by_label"]
    for label, dataset in datasets.items():
        matrices = by_label[label]
        time_s = matrices["time_s"]
        q_error = matrices["q_error"]
        tau = matrices["tau_raw"]
        dt = np.diff(time_s)
        dt_mean = float(np.mean(dt)) if dt.size else None
        dt_jitter = dt - dt_mean if dt.size and dt_mean is not None else np.array([])
        max_tracking = float(np.max(np.abs(q_error))) if q_error.size else 0.0
        entry = {
            "role": dataset["role"],
            "sample_count": len(dataset["rows"]),
            "duration_s": float(time_s[-1]) if len(time_s) else 0.0,
            "actual_send_hz": float(1.0 / np.mean(dt)) if dt.size else None,
            "dt_mean_s": dt_mean,
            "dt_p99_s": float(np.percentile(dt, 99)) if dt.size else None,
            "dt_max_s": float(np.max(dt)) if dt.size else None,
            "dt_jitter_rms_s": float(np.sqrt(np.mean(dt_jitter * dt_jitter)))
            if dt_jitter.size
            else None,
            "dt_jitter_max_abs_s": float(np.max(np.abs(dt_jitter)))
            if dt_jitter.size
            else None,
            "fault_flags": sorted(
                {
                    row.get("fault_flags", "")
                    for row in dataset["rows"]
                    if row.get("fault_flags", "")
                }
            ),
            "q_range_rad": _float_list(np.ptp(matrices["q_meas"], axis=0)),
            "tracking_rms_rad": float(np.sqrt(np.mean(q_error * q_error))),
            "tracking_max_abs_rad": max_tracking,
            "per_joint_tracking_max_abs_rad": _float_list(np.max(np.abs(q_error), axis=0)),
            "tau_range": _float_list(np.ptp(tau, axis=0)),
            "tau_rms": _float_list(np.sqrt(np.mean(tau * tau, axis=0))),
            "tau_max_abs": _float_list(np.max(np.abs(tau), axis=0)),
            "tau_offset_estimate": _float_list(np.mean(tau, axis=0)),
            "tau_saturation_check": _tau_saturation_check(tau),
        }
        if label == primary_label and max_tracking >= 0.05:
            entry["tracking_warning"] = "tracking_max_abs_rad_above_0p05"
        elif label == primary_label:
            entry["tracking_warning"] = "none"
        if dataset["role"] == "validation":
            entry["usage_note"] = "validation_or_smoke_not_primary_identification"
        summary["datasets"][label] = entry
    return summary


def _tau_saturation_check(tau: np.ndarray) -> dict[str, object]:
    if tau.size == 0:
        return {
            "status": "warning",
            "basis": "observed_tau_signal_only_no_hardware_limit",
            "threshold_fraction_of_observed_max_abs": 0.98,
            "per_joint_near_max_fraction": [],
            "reason": "no tau samples",
        }
    max_abs = np.max(np.abs(tau), axis=0)
    thresholds = 0.98 * max_abs
    near_max: list[float] = []
    for joint in range(tau.shape[1]):
        threshold = thresholds[joint]
        if threshold <= 1e-12:
            near_max.append(0.0)
            continue
        near_max.append(float(np.mean(np.abs(tau[:, joint]) >= threshold)))
    status = "warning" if any(value > 0.1 for value in near_max) else "pass"
    return {
        "status": status,
        "basis": "observed_tau_signal_only_no_hardware_limit",
        "threshold_fraction_of_observed_max_abs": 0.98,
        "per_joint_near_max_fraction": near_max,
        "reason": "hardware current/torque saturation limits are not known in this offline pass",
    }


def _regressor_summary(
    *,
    preprocessed: dict[str, Any],
    urdf_path: str,
) -> dict[str, object]:
    summary: dict[str, object] = {
        "schema": "armctrl.measured_regressor_summary.v1",
        "backend": "pinocchio",
        "urdf_path": urdf_path,
        "datasets": {},
        "notes": [
            "computeJointTorqueRegressor is used when Pinocchio is importable.",
            "full-regressor effective condition is not FIGAROH symbolic base condition.",
        ],
        "external_references": [
            {
                "name": "Pinocchio computeJointTorqueRegressor",
                "url": "https://docs.ros.org/en/rolling/p/pinocchio/generated/function_namespacepinocchio_1af97c3d2d695ef4636bf010c2ff6031e8.html",
            }
        ],
    }
    if importlib.util.find_spec("pinocchio") is None:
        for label in preprocessed["by_label"]:
            summary["datasets"][label] = {
                "status": "not_evaluated",
                "reason": "pinocchio module is not importable in this environment",
            }
        return summary
    try:
        import pinocchio  # type: ignore[import-not-found]
    except Exception as exc:  # pragma: no cover - backend/environment dependent.
        for label in preprocessed["by_label"]:
            summary["datasets"][label] = {
                "status": "not_evaluated",
                "reason": f"pinocchio import failed: {exc}",
            }
        return summary
    try:
        model = pinocchio.buildModelFromUrdf(str(urdf_path))
        data = model.createData()
    except Exception as exc:  # pragma: no cover - backend/environment dependent.
        for label in preprocessed["by_label"]:
            summary["datasets"][label] = {
                "status": "not_evaluated",
                "reason": f"pinocchio model load failed: {exc}",
            }
        return summary
    for label, matrices in preprocessed["by_label"].items():
        try:
            y_matrix, tau_vector = _pinocchio_matrices(pinocchio, model, data, matrices)
            condition = _condition_stats(y_matrix)
            fit = _fit_result(y_matrix, tau_vector)
            summary["datasets"][label] = {
                "status": "computed",
                **condition,
                "tau_vector_count": int(tau_vector.size),
                "own_fit_prediction": fit,
                "_y_matrix": y_matrix,
                "_tau_vector": tau_vector,
            }
        except Exception as exc:  # pragma: no cover - user-facing evidence.
            summary["datasets"][label] = {
                "status": "failed",
                "reason": str(exc),
            }
    return summary


def _pinocchio_matrices(
    pinocchio: Any,
    model: Any,
    data: Any,
    matrices: dict[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    regressors: list[np.ndarray] = []
    tau_blocks: list[np.ndarray] = []
    mask = matrices["edge_drop_mask"]
    for index in range(len(matrices["time_s"])):
        if not bool(mask[index]):
            continue
        q = np.asarray(matrices["q_filtered"][index], dtype=float)
        v = np.asarray(matrices["dq_filtered"][index], dtype=float)
        a = np.asarray(matrices["ddq_filtered"][index], dtype=float)
        regressor = pinocchio.computeJointTorqueRegressor(model, data, q, v, a)
        if regressor is None:
            regressor = data.jointTorqueRegressor
        regressors.append(np.asarray(regressor, dtype=float))
        tau_blocks.append(np.asarray(matrices["tau_filtered"][index], dtype=float))
    if not regressors:
        raise ValueError("no samples left after edge-drop filtering")
    return np.vstack(regressors), np.concatenate(tau_blocks)


def _condition_stats(y_matrix: np.ndarray) -> dict[str, object]:
    singular_values = np.linalg.svd(y_matrix, compute_uv=False)
    if singular_values.size == 0:
        return {
            "row_count": int(y_matrix.shape[0]),
            "column_count": int(y_matrix.shape[1]),
            "rank": 0,
            "effective_condition_number": float("inf"),
            "singular_values": [],
        }
    tolerance = np.finfo(float).eps * max(y_matrix.shape) * singular_values[0]
    nonzero = singular_values[singular_values > tolerance]
    rank = int(nonzero.size)
    condition = float(nonzero[0] / nonzero[-1]) if rank else float("inf")
    return {
        "row_count": int(y_matrix.shape[0]),
        "column_count": int(y_matrix.shape[1]),
        "rank": rank,
        "effective_condition_number": condition,
        "singular_values": _float_list(singular_values),
    }


def _fit_result(y_matrix: np.ndarray, tau_vector: np.ndarray) -> dict[str, object]:
    parameter_vector, *_rest = np.linalg.lstsq(y_matrix, tau_vector, rcond=None)
    tau_pred = y_matrix @ parameter_vector
    metrics = _residual_metrics(tau_vector, tau_pred)
    return {
        "parameter_count": int(parameter_vector.size),
        **metrics,
        "parameter_vector": _float_list(parameter_vector),
        "_tau_pred": tau_pred,
        "_tau_residual": tau_vector - tau_pred,
    }


def _residual_metrics(tau_vector: np.ndarray, tau_pred: np.ndarray) -> dict[str, object]:
    residual = tau_vector - tau_pred
    joint_count = 6
    residual_by_joint = residual.reshape((-1, joint_count))
    tau_by_joint = tau_vector.reshape((-1, joint_count))
    rmse_by_joint = np.sqrt(np.mean(residual_by_joint * residual_by_joint, axis=0))
    max_by_joint = np.max(np.abs(residual_by_joint), axis=0)
    tau_range = np.ptp(tau_by_joint, axis=0)
    tau_rms = np.sqrt(np.mean(tau_by_joint * tau_by_joint, axis=0))
    denominators = np.where(tau_range > 1e-12, tau_range, tau_rms)
    denominators = np.where(denominators > 1e-12, denominators, 1.0)
    nrmse_by_joint = rmse_by_joint / denominators
    all_denominator = float(np.ptp(tau_vector))
    if all_denominator <= 1e-12:
        all_denominator = float(np.sqrt(np.mean(tau_vector * tau_vector)))
    if all_denominator <= 1e-12:
        all_denominator = 1.0
    rmse_all = float(np.sqrt(np.mean(residual * residual)))
    return {
        "rmse_all": rmse_all,
        "max_abs_all": float(np.max(np.abs(residual))),
        "nrmse_all": rmse_all / all_denominator,
        "nrmse_denominator": "peak_to_peak_tau_signal",
        "per_joint_rmse": _float_list(rmse_by_joint),
        "per_joint_max_abs": _float_list(max_by_joint),
        "per_joint_nrmse": _float_list(nrmse_by_joint),
    }


def _base_parameter_summary(regressor: dict[str, object]) -> dict[str, object]:
    datasets: dict[str, object] = {}
    for label, entry_obj in regressor["datasets"].items():
        entry = dict(entry_obj)
        y_matrix = entry.pop("_y_matrix", None)
        entry.pop("_tau_vector", None)
        if entry.get("status") != "computed" or y_matrix is None:
            datasets[label] = {
                "status": "not_evaluated",
                "method": "numeric_qr_proxy",
                "reason": entry.get("reason", "pinocchio regressor not computed"),
                "limitation": "FIGAROH symbolic base parameter extraction not run",
            }
            continue
        rank = int(entry["rank"])
        _q, _r, pivots = qr(y_matrix, mode="economic", pivoting=True)
        selected = [int(value) for value in sorted(pivots[:rank])]
        y_base = y_matrix[:, selected]
        condition = _condition_stats(y_base)
        datasets[label] = {
            "status": "computed",
            "method": "numeric_qr_proxy",
            "rank": rank,
            "condition_number": condition["effective_condition_number"],
            "selected_columns": selected,
            "limitation": "numeric QR proxy only; not FIGAROH symbolic base parameters",
        }
    return {
        "schema": "armctrl.base_parameter_summary.v1",
        "preferred_method": "figaroh_base_parameters",
        "active_method": "numeric_qr_proxy",
        "external_references": [
            {
                "name": "FIGAROH dynamic identification toolkit",
                "url": "https://pypi.org/project/figaroh/",
            }
        ],
        "datasets": datasets,
    }


def _base_selected_columns(base: dict[str, object], label: str) -> list[int] | None:
    datasets = base.get("datasets", {})
    entry = datasets.get(label, {}) if isinstance(datasets, dict) else {}
    if not isinstance(entry, dict) or entry.get("status") != "computed":
        return None
    selected = entry.get("selected_columns")
    if not isinstance(selected, list) or not selected:
        return None
    return [int(value) for value in selected]


def _solver_summary(
    *,
    regressor: dict[str, object],
    base: dict[str, object],
    primary_label: str,
) -> dict[str, object]:
    datasets: dict[str, object] = {}
    for label, entry_obj in regressor["datasets"].items():
        entry = dict(entry_obj)
        y_matrix = entry.pop("_y_matrix", None)
        tau_vector = entry.pop("_tau_vector", None)
        if entry.get("status") != "computed" or y_matrix is None or tau_vector is None:
            datasets[label] = {
                "status": "not_evaluated",
                "reason": entry.get("reason", "regressor not computed"),
            }
            continue
        selected_columns = _base_selected_columns(base, label)
        if selected_columns:
            fit_matrix = y_matrix[:, selected_columns]
            parameter_space = "numeric_qr_proxy_base"
        else:
            fit_matrix = y_matrix
            parameter_space = "full_regressor_fallback"
        fit = _fit_result(fit_matrix, tau_vector)
        datasets[label] = {
            "status": "computed",
            "estimator": "OLS",
            "tau_signal_name": "tau_meas / effort signal",
            "torque_scale_status": "unconfirmed",
            "parameter_space": parameter_space,
            "selected_columns": selected_columns or [],
            **{
                key: value
                for key, value in fit.items()
                if key != "parameter_vector" and not str(key).startswith("_")
            },
        }
    return {
        "schema": "armctrl.measured_solver_summary.v1",
        "primary_label": primary_label,
        "estimator": "OLS",
        "base_parameter_method": base["active_method"],
        "torque_scale_status": "unconfirmed",
        "datasets": datasets,
    }


def _validation_summary(
    *,
    regressor: dict[str, object],
    base: dict[str, object],
    primary_label: str,
    validation_labels: Sequence[str],
) -> dict[str, object]:
    dataset_entries = regressor["datasets"]
    primary = dataset_entries.get(primary_label, {})
    y_train = primary.get("_y_matrix") if isinstance(primary, dict) else None
    tau_train = primary.get("_tau_vector") if isinstance(primary, dict) else None
    selected_columns = _base_selected_columns(base, primary_label)
    comparisons: list[dict[str, object]] = []
    for label in validation_labels:
        target = dataset_entries.get(label, {})
        y_eval = target.get("_y_matrix") if isinstance(target, dict) else None
        tau_eval = target.get("_tau_vector") if isinstance(target, dict) else None
        if (
            y_train is None
            or tau_train is None
            or y_eval is None
            or tau_eval is None
            or not selected_columns
        ):
            comparisons.append(
                {
                    "train": primary_label,
                    "eval": label,
                    "status": "not_evaluated",
                    "reason": "regressor or primary base selected columns not computed",
                }
            )
            continue
        theta, *_ = np.linalg.lstsq(y_train[:, selected_columns], tau_train, rcond=None)
        tau_pred = y_eval[:, selected_columns] @ theta
        metrics = _residual_metrics(tau_eval, tau_pred)
        comparisons.append(
            {
                "train": primary_label,
                "eval": label,
                "status": "computed",
                "parameter_space": "numeric_qr_proxy_base",
                "selected_columns": selected_columns,
                **metrics,
            }
        )
    return {
        "schema": "armctrl.measured_validation_summary.v1",
        "primary_label": primary_label,
        "comparisons": comparisons,
    }


def _estimator_comparison_summary(
    *,
    regressor: dict[str, object],
    base: dict[str, object],
    primary_label: str,
    validation_labels: Sequence[str],
) -> dict[str, object]:
    dataset_entries = regressor["datasets"]
    primary = dataset_entries.get(primary_label, {})
    y_train_full = primary.get("_y_matrix") if isinstance(primary, dict) else None
    tau_train = primary.get("_tau_vector") if isinstance(primary, dict) else None
    selected_columns = _base_selected_columns(base, primary_label)
    common = {
        "schema": "armctrl.measured_estimator_comparison.v1",
        "primary_label": primary_label,
        "parameter_space": "numeric_qr_proxy_base",
        "weight_sources": [
            "per-joint residual variance from OLS baseline",
            "tau signal scale for normalization",
            "tracking-error warning retained in measured quality report",
        ],
        "external_references": [
            {
                "name": "scipy.optimize.least_squares robust losses",
                "url": "https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html",
            },
            {
                "name": "scipy.linalg.lstsq least-squares solver",
                "url": "https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.lstsq.html",
            },
        ],
    }
    if y_train_full is None or tau_train is None or not selected_columns:
        return {
            **common,
            "status": "not_evaluated",
            "reason": "primary regressor or base selected columns not computed",
            "estimators": [],
            "chosen_estimator": None,
            "results": {},
        }

    y_train = y_train_full[:, selected_columns]
    ols_theta = _ols_theta(y_train, tau_train)
    ols_pred = y_train @ ols_theta
    ols_residual = tau_train - ols_pred
    weights = _wls_weights_from_residual(ols_residual)
    estimator_thetas = {
        "OLS": ols_theta,
        "WLS_per_joint_residual_variance": _wls_theta(y_train, tau_train, weights),
        "robust_soft_l1": _robust_theta(y_train, tau_train, ols_theta),
    }

    results: dict[str, object] = {}
    for name, theta in estimator_thetas.items():
        train_metrics = _residual_metrics(tau_train, y_train @ theta)
        validations = []
        for label in validation_labels:
            target = dataset_entries.get(label, {})
            y_eval_full = target.get("_y_matrix") if isinstance(target, dict) else None
            tau_eval = target.get("_tau_vector") if isinstance(target, dict) else None
            if y_eval_full is None or tau_eval is None:
                validations.append(
                    {
                        "eval": label,
                        "status": "not_evaluated",
                        "reason": "validation regressor not computed",
                    }
                )
                continue
            metrics = _residual_metrics(tau_eval, y_eval_full[:, selected_columns] @ theta)
            validations.append({"eval": label, "status": "computed", **metrics})
        computed_validations = [
            item for item in validations if item.get("status") == "computed"
        ]
        mean_validation_nrmse = (
            float(np.mean([float(item["nrmse_all"]) for item in computed_validations]))
            if computed_validations
            else None
        )
        results[name] = {
            "status": "computed",
            "parameter_count": int(theta.size),
            "selected_columns": selected_columns,
            "train": train_metrics,
            "validation": validations,
            "mean_validation_nrmse": mean_validation_nrmse,
            "parameter_vector": _float_list(theta),
        }
    chosen = _choose_estimator(results)
    return {
        **common,
        "status": "computed",
        "estimators": list(estimator_thetas),
        "chosen_estimator": chosen,
        "results": results,
    }


def _ols_theta(y_matrix: np.ndarray, tau_vector: np.ndarray) -> np.ndarray:
    theta, *_ = np.linalg.lstsq(y_matrix, tau_vector, rcond=None)
    return np.asarray(theta, dtype=float)


def _wls_weights_from_residual(residual: np.ndarray) -> np.ndarray:
    residual_by_joint = residual.reshape((-1, 6))
    variance = np.var(residual_by_joint, axis=0)
    variance = np.where(variance > 1e-12, variance, np.max(variance) if variance.size else 1.0)
    variance = np.where(variance > 1e-12, variance, 1.0)
    joint_weights = 1.0 / variance
    joint_weights = joint_weights / np.mean(joint_weights)
    return np.tile(joint_weights, residual_by_joint.shape[0])


def _wls_theta(
    y_matrix: np.ndarray,
    tau_vector: np.ndarray,
    weights: np.ndarray,
) -> np.ndarray:
    sqrt_weights = np.sqrt(np.asarray(weights, dtype=float))
    weighted_y = y_matrix * sqrt_weights[:, None]
    weighted_tau = tau_vector * sqrt_weights
    theta, *_ = np.linalg.lstsq(weighted_y, weighted_tau, rcond=None)
    return np.asarray(theta, dtype=float)


def _robust_theta(
    y_matrix: np.ndarray,
    tau_vector: np.ndarray,
    initial_theta: np.ndarray,
) -> np.ndarray:
    initial_residual = tau_vector - y_matrix @ initial_theta
    f_scale = float(np.median(np.abs(initial_residual)))
    if f_scale <= 1e-9:
        f_scale = float(np.sqrt(np.mean(initial_residual * initial_residual)))
    if f_scale <= 1e-9:
        f_scale = 1.0
    result = least_squares(
        lambda theta: y_matrix @ theta - tau_vector,
        initial_theta,
        loss="soft_l1",
        f_scale=f_scale,
        max_nfev=200,
    )
    return np.asarray(result.x, dtype=float)


def _choose_estimator(results: dict[str, object]) -> str | None:
    computed = []
    for name, entry_obj in results.items():
        entry = entry_obj if isinstance(entry_obj, dict) else {}
        score = entry.get("mean_validation_nrmse")
        if isinstance(score, (int, float)) and math.isfinite(float(score)):
            computed.append((float(score), str(name)))
    if not computed:
        return None
    computed.sort(key=lambda item: (item[0], item[1] != "OLS"))
    return computed[0][1]


def _prediction_artifacts(
    *,
    regressor: dict[str, object],
    base: dict[str, object],
    estimator_comparison: dict[str, object],
    primary_label: str,
    labels: Sequence[str],
) -> dict[str, object]:
    dataset_entries = regressor["datasets"]
    primary = dataset_entries.get(primary_label, {})
    y_train = primary.get("_y_matrix") if isinstance(primary, dict) else None
    tau_train = primary.get("_tau_vector") if isinstance(primary, dict) else None
    selected_columns = _base_selected_columns(base, primary_label)
    chosen = estimator_comparison.get("chosen_estimator")
    results = estimator_comparison.get("results", {})
    chosen_result = results.get(chosen, {}) if isinstance(results, dict) else {}
    parameter_vector = (
        chosen_result.get("parameter_vector") if isinstance(chosen_result, dict) else None
    )
    if (
        y_train is None
        or tau_train is None
        or not selected_columns
        or not isinstance(parameter_vector, list)
    ):
        return {
            "status": "not_evaluated",
            "reason": "primary regressor, base selected columns, or estimator comparison not computed",
            "tau_pred_rows": [],
            "tau_residual_rows": [],
            "active_parameter_set": None,
            "_residual_by_label": {},
            "_tau_pred_by_label": {},
        }

    theta = np.asarray(parameter_vector, dtype=float)
    tau_pred_rows: list[dict[str, object]] = []
    tau_residual_rows: list[dict[str, object]] = []
    residual_by_label: dict[str, np.ndarray] = {}
    tau_pred_by_label: dict[str, np.ndarray] = {}
    for label in labels:
        entry = dataset_entries.get(label, {})
        y_eval = entry.get("_y_matrix") if isinstance(entry, dict) else None
        tau_eval = entry.get("_tau_vector") if isinstance(entry, dict) else None
        if y_eval is None or tau_eval is None:
            continue
        tau_pred = y_eval[:, selected_columns] @ theta
        residual = tau_eval - tau_pred
        residual_by_label[label] = residual
        tau_pred_by_label[label] = tau_pred
        tau_pred_by_sample = tau_pred.reshape((-1, 6))
        residual_by_sample = residual.reshape((-1, 6))
        for sample_index, values in enumerate(tau_pred_by_sample):
            row: dict[str, object] = {
                "dataset_label": label,
                "fit_source": primary_label,
                "sample_index_after_edge_drop": sample_index,
            }
            for joint in range(6):
                row[f"tau_pred_{joint + 1}"] = float(values[joint])
            tau_pred_rows.append(row)
        for sample_index, values in enumerate(residual_by_sample):
            row = {
                "dataset_label": label,
                "fit_source": primary_label,
                "sample_index_after_edge_drop": sample_index,
            }
            for joint in range(6):
                row[f"tau_residual_{joint + 1}"] = float(values[joint])
            tau_residual_rows.append(row)
    return {
        "status": "computed",
        "tau_pred_rows": tau_pred_rows,
        "tau_residual_rows": tau_residual_rows,
        "active_parameter_set": {
            "source_dataset": primary_label,
            "estimator": chosen,
            "parameter_space": "numeric_qr_proxy_base",
            "selected_columns": selected_columns,
            "parameter_count": int(theta.size),
            "parameter_vector": _float_list(theta),
        },
        "_residual_by_label": residual_by_label,
        "_tau_pred_by_label": tau_pred_by_label,
    }


def _residual_diagnostics(
    *,
    prediction: dict[str, object],
    preprocessed: dict[str, Any],
    labels: Sequence[str],
) -> dict[str, object]:
    residual_by_label = prediction.get("_residual_by_label", {})
    if not isinstance(residual_by_label, dict) or not residual_by_label:
        return {
            "rows": [],
            "summary": {
                "schema": "armctrl.measured_residual_diagnostics.v1",
                "status": "not_evaluated",
                "reason": "prediction residuals not computed",
                "datasets": [],
            },
        }
    rows: list[dict[str, object]] = []
    datasets = []
    for label in labels:
        residual = residual_by_label.get(label)
        matrices = preprocessed["by_label"].get(label)
        if residual is None or matrices is None:
            continue
        residual_matrix = np.asarray(residual, dtype=float).reshape((-1, 6))
        mask = np.asarray(matrices["edge_drop_mask"], dtype=bool)
        q = np.asarray(matrices["q_filtered"], dtype=float)[mask]
        dq = np.asarray(matrices["dq_filtered"], dtype=float)[mask]
        ddq = np.asarray(matrices["ddq_filtered"], dtype=float)[mask]
        histogram_stats = _per_joint_histogram_stats(residual_matrix)
        correlation = {
            "q": _residual_correlations(residual_matrix, q),
            "dq": _residual_correlations(residual_matrix, dq),
            "ddq": _residual_correlations(residual_matrix, ddq),
        }
        outliers = _top_outliers(label, residual_matrix, limit=10)
        for sample_index, values in enumerate(residual_matrix):
            row = {
                "dataset_label": label,
                "sample_index_after_edge_drop": sample_index,
                "residual_norm": float(np.linalg.norm(values)),
            }
            for joint in range(6):
                row[f"tau_residual_{joint + 1}"] = float(values[joint])
            rows.append(row)
        datasets.append(
            {
                "label": label,
                "status": "computed",
                "sample_count_after_edge_drop": int(residual_matrix.shape[0]),
                "per_joint_histogram_stats": histogram_stats,
                "correlation_with_q_dq_ddq": correlation,
                "top_outliers": outliers,
                "interpretation_notes": [
                    "large residuals may reflect unconfirmed tau/current scaling",
                    "tracking error can leak into q/dq/ddq-derived regressors",
                    "Savitzky-Golay derivative noise and missing model terms remain possible",
                ],
            }
        )
    return {
        "rows": rows,
        "summary": {
            "schema": "armctrl.measured_residual_diagnostics.v1",
            "status": "computed" if datasets else "not_evaluated",
            "datasets": datasets,
        },
    }


def _per_joint_histogram_stats(residual_matrix: np.ndarray) -> list[dict[str, object]]:
    stats = []
    for joint in range(6):
        values = residual_matrix[:, joint]
        stats.append(
            {
                "joint": joint + 1,
                "mean": float(np.mean(values)),
                "std": float(np.std(values)),
                "p50": float(np.percentile(values, 50)),
                "p95_abs": float(np.percentile(np.abs(values), 95)),
                "p99_abs": float(np.percentile(np.abs(values), 99)),
                "max_abs": float(np.max(np.abs(values))),
            }
        )
    return stats


def _residual_correlations(
    residual_matrix: np.ndarray,
    signal_matrix: np.ndarray,
) -> list[float]:
    values = []
    for joint in range(6):
        residual = residual_matrix[:, joint]
        signal = signal_matrix[:, joint]
        if np.std(residual) <= 1e-12 or np.std(signal) <= 1e-12:
            values.append(0.0)
            continue
        values.append(float(np.corrcoef(residual, signal)[0, 1]))
    return values


def _top_outliers(
    label: str,
    residual_matrix: np.ndarray,
    *,
    limit: int,
) -> list[dict[str, object]]:
    norms = np.linalg.norm(residual_matrix, axis=1)
    order = np.argsort(norms)[::-1][:limit]
    outliers = []
    for index in order:
        outliers.append(
            {
                "dataset_label": label,
                "sample_index_after_edge_drop": int(index),
                "residual_norm": float(norms[index]),
                "per_joint_residual": _float_list(residual_matrix[index]),
            }
        )
    return outliers


def _physical_consistency_summary(
    *,
    base: dict[str, object],
    estimator_comparison: dict[str, object],
    primary_label: str,
) -> dict[str, object]:
    chosen = estimator_comparison.get("chosen_estimator")
    results = estimator_comparison.get("results", {})
    chosen_result = results.get(chosen, {}) if isinstance(results, dict) else {}
    vector = chosen_result.get("parameter_vector") if isinstance(chosen_result, dict) else []
    values = np.asarray(vector, dtype=float) if isinstance(vector, list) else np.array([])
    primary_base = (
        base.get("datasets", {}).get(primary_label, {})
        if isinstance(base.get("datasets"), dict)
        else {}
    )
    sign_counts = {
        "positive": int(np.sum(values > 0.0)),
        "negative": int(np.sum(values < 0.0)),
        "near_zero": int(np.sum(np.abs(values) <= 1e-12)),
    }
    return {
        "schema": "armctrl.physical_consistency_screen.v1",
        "status": "review_ready" if values.size else "not_evaluated",
        "chosen_estimator": chosen,
        "parameter_space": "numeric_qr_proxy_base",
        "parameter_count": int(values.size),
        "sign_counts": sign_counts,
        "max_abs_parameter": float(np.max(np.abs(values))) if values.size else None,
        "base_rank": primary_base.get("rank") if isinstance(primary_base, dict) else None,
        "base_parameter_limitation": (
            "numeric QR base-proxy parameters are identifiable linear combinations, "
            "not directly full inertial link masses/COM/inertia tensors"
        ),
        "physical_consistency_status": "not_claimed",
        "full_inertial_projection_next_step": (
            "Use FIGAROH symbolic/base-parameter tooling or a physically consistent "
            "identification formulation before mapping to full inertial parameters."
        ),
        "external_references": [
            {
                "name": "FIGAROH dynamic identification toolkit",
                "url": "https://pypi.org/project/figaroh/",
            },
            {
                "name": "physically consistent robot dynamics identification literature",
                "url": "https://doi.org/10.1109/LRA.2019.2928787",
            },
        ],
    }


def _gravity_comp_offline_sanity(
    *,
    preprocessed: dict[str, Any],
    prediction: dict[str, object],
    urdf_path: str,
    primary_label: str,
) -> dict[str, object]:
    common = {
        "schema": "armctrl.gravity_comp_offline_sanity.v1",
        "urdf_path": urdf_path,
        "tau_signal_name": "tau_meas / effort signal",
        "can_use_for_real_nm_gravity_compensation": False,
        "identified_effort_model": {
            "status": "not_injected",
            "reason": (
                "current parameter vector is numeric QR base-proxy in effort-signal "
                "scale, not a full inertial model with confirmed Nm units"
            ),
        },
        "external_references": [
            {
                "name": "Pinocchio computeGeneralizedGravity",
                "url": "https://docs.ros.org/en/rolling/p/pinocchio/generated/function_namespacepinocchio_1a6cd0f621e83bfa0c74f89aa63682c866.html",
            }
        ],
    }
    if importlib.util.find_spec("pinocchio") is None:
        return {
            **common,
            "status": "not_evaluated",
            "reason": "pinocchio module is not importable in this environment",
            "default_model_gravity_samples": [],
        }
    try:
        import pinocchio  # type: ignore[import-not-found]

        model = pinocchio.buildModelFromUrdf(str(urdf_path))
        data = model.createData()
        q_matrix = np.asarray(
            preprocessed["by_label"][primary_label]["q_filtered"],
            dtype=float,
        )
        if not hasattr(pinocchio, "computeGeneralizedGravity"):
            raise AttributeError("pinocchio.computeGeneralizedGravity missing")
        indices = sorted({0, len(q_matrix) // 2, len(q_matrix) - 1})
        samples = []
        for index in indices:
            q = q_matrix[index]
            gravity = pinocchio.computeGeneralizedGravity(model, data, q)
            samples.append(
                {
                    "sample_index": int(index),
                    "q": _float_list(q),
                    "default_model_gravity": _float_list(gravity),
                }
            )
        return {
            **common,
            "status": "computed",
            "default_model_gravity_samples": samples,
            "effort_scale_prediction_available": prediction.get("status") == "computed",
            "conclusion": (
                "default URDF gravity torque can be computed offline, but identified "
                "base-proxy effort parameters are not injected as real Nm gravity compensation"
            ),
        }
    except Exception as exc:  # pragma: no cover - backend/environment dependent.
        return {
            **common,
            "status": "not_evaluated",
            "reason": str(exc),
            "default_model_gravity_samples": [],
        }


def _bundle_status(
    *,
    regressor: dict[str, object],
    validation: dict[str, object],
    estimator_comparison: dict[str, object],
    physical: dict[str, object],
) -> str:
    computed = [
        entry
        for entry in regressor["datasets"].values()
        if isinstance(entry, dict) and entry.get("status") == "computed"
    ]
    validation_computed = any(
        comparison.get("status") == "computed"
        for comparison in validation["comparisons"]
        if isinstance(comparison, dict)
    )
    if (
        computed
        and validation_computed
        and estimator_comparison.get("status") == "computed"
        and physical.get("status") == "review_ready"
    ):
        return "physical_consistency_review_ready"
    if computed and validation_computed and estimator_comparison.get("status") == "computed":
        return "offline_validation_ready"
    return "preliminary_measured_fit"


def _bundle_manifest(
    *,
    request: MeasuredSysIdAnalyzeRequest,
    tau_trace: dict[str, object],
    preprocessed: dict[str, object],
    regressor: dict[str, object],
    base: dict[str, object],
    solver: dict[str, object],
    validation: dict[str, object],
    estimator_comparison: dict[str, object],
    physical: dict[str, object],
    gravity: dict[str, object],
    status: str,
) -> dict[str, object]:
    return {
        "schema": "armctrl.x5_body_dynamics_bundle_manifest.v1",
        "status": status,
        "source_bundle": "runs/sysid-measured-analysis-goal-pinocchio-6",
        "source_datasets": {
            "root": str(request.dataset_dir),
            "primary": request.primary_label,
            "validation": list(request.validation_labels),
        },
        "urdf_path": request.urdf_path,
        "payload_model": {"d435_mass_included_in_link6": True},
        "tau_meas_semantics": tau_trace["conclusion"],
        "preprocessing": preprocessed["summary"],
        "regressor_backend": {
            "name": regressor["backend"],
            "status_by_dataset": {
                label: entry.get("status")
                for label, entry in regressor["datasets"].items()
                if isinstance(entry, dict)
            },
        },
        "base_parameter_method": base["active_method"],
        "estimator_comparison_status": estimator_comparison.get("status"),
        "chosen_offline_estimator": estimator_comparison.get("chosen_estimator"),
        "physical_consistency_status": physical.get("status"),
        "gravity_sanity_status": gravity.get("status"),
        "validation_status": _validation_status(validation),
        "limitations": [
            "tau_meas units/source are not confirmed as Nm in this offline pass",
            "numeric QR base proxy is not FIGAROH symbolic base parameter extraction",
            "parameter bundle is not production ready",
        ],
        "rollback_note": "Do not deploy automatically; compare against existing parameters with an A/B rollout and keep previous parameters available.",
    }


def _validation_status(validation: dict[str, object]) -> str:
    if any(
        comparison.get("status") == "computed"
        for comparison in validation["comparisons"]
        if isinstance(comparison, dict)
    ):
        return "computed"
    return "not_evaluated"


def _parameter_bundle_payload(
    *,
    request: MeasuredSysIdAnalyzeRequest,
    status: str,
    tau_trace: dict[str, object],
    base: dict[str, object],
    solver: dict[str, object],
    prediction: dict[str, object],
    estimator_comparison: dict[str, object],
) -> dict[str, object]:
    primary_solver = solver["datasets"].get(request.primary_label, {})
    active_parameter_set = prediction.get("active_parameter_set")
    return {
        "schema": "armctrl.x5_body_dynamics_parameters.v1",
        "status": status,
        "primary_label": request.primary_label,
        "validation_labels": list(request.validation_labels),
        "tau_signal_name": "tau_meas / effort signal",
        "tau_meas_semantics": tau_trace["conclusion"],
        "torque_scale_status": solver["torque_scale_status"],
        "estimator": solver["estimator"],
        "chosen_offline_estimator": estimator_comparison.get("chosen_estimator"),
        "base_parameter_method": base["active_method"],
        "active_parameter_set": active_parameter_set,
        "primary_fit_metrics": primary_solver
        if isinstance(primary_solver, dict)
        else {"status": "not_evaluated"},
        "limitations": [
            "The parameter vector is fit against tau_meas / effort signal, whose Nm scale is still unconfirmed.",
            "The active base route is numeric QR proxy unless a future FIGAROH symbolic base pass replaces it.",
            "This payload is preliminary and must be validated before control deployment.",
        ],
    }


def _write_bundle_json(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(_json_sanitized(payload), ensure_ascii=False, indent=2), encoding="utf-8")


def _json_sanitized(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {
            key: _json_sanitized(item)
            for key, item in value.items()
            if not str(key).startswith("_")
        }
    if isinstance(value, list):
        return [_json_sanitized(item) for item in value]
    return value


def _float_list(values: Any) -> list[float]:
    return [float(value) for value in np.asarray(values).reshape(-1)]


def _preprocessing_report(summary: dict[str, object]) -> str:
    lines = ["# Measured SysID Preprocessing", ""]
    for label, entry in summary["datasets"].items():
        lines.append(f"- {label}: `{entry['filter_method']}`, samples `{entry['sample_count']}`")
    return "\n".join(lines) + "\n"


def _tau_meas_trace_report(trace: dict[str, object]) -> str:
    conclusion = trace["conclusion"]
    return (
        "# tau_meas Trace\n\n"
        f"- status: `{conclusion['status']}`\n"
        f"- signal: `{conclusion['signal_name']}`\n"
        f"- can_claim_nm: `{conclusion['can_claim_nm']}`\n"
        f"- reason: {conclusion['reason']}\n"
    )


def _quality_report(summary: dict[str, object]) -> str:
    lines = [
        "# Measured SysID Quality",
        "",
        "| dataset | role | samples | max tracking rad | faults |",
        "| --- | --- | ---: | ---: | --- |",
    ]
    for label, entry in summary["datasets"].items():
        lines.append(
            f"| {label} | {entry['role']} | {entry['sample_count']} | "
            f"{entry['tracking_max_abs_rad']:.6f} | {entry['fault_flags']} |"
        )
    return "\n".join(lines) + "\n"


def _regressor_report(summary: dict[str, object]) -> str:
    lines = [
        "# Measured Pinocchio Regressor",
        "",
        "| dataset | status | rank | condition |",
        "| --- | --- | ---: | ---: |",
    ]
    for label, entry in summary["datasets"].items():
        rank = entry.get("rank", "")
        condition = entry.get("effective_condition_number", "")
        if isinstance(condition, float):
            condition = f"{condition:.6f}"
        lines.append(f"| {label} | {entry['status']} | {rank} | {condition} |")
    return "\n".join(lines) + "\n"


def _base_parameter_report(summary: dict[str, object]) -> str:
    lines = ["# Base Parameter Route", "", f"- active_method: `{summary['active_method']}`"]
    for label, entry in summary["datasets"].items():
        lines.append(f"- {label}: `{entry['status']}` / `{entry['method']}`")
    return "\n".join(lines) + "\n"


def _solver_report(summary: dict[str, object]) -> str:
    lines = ["# Measured SysID Solver", "", f"- estimator: `{summary['estimator']}`"]
    for label, entry in summary["datasets"].items():
        lines.append(f"- {label}: `{entry['status']}`")
    return "\n".join(lines) + "\n"


def _validation_report(summary: dict[str, object]) -> str:
    lines = [
        "# Measured SysID Validation",
        "",
        "| train | eval | status | rmse |",
        "| --- | --- | --- | ---: |",
    ]
    for comparison in summary["comparisons"]:
        rmse = comparison.get("rmse_all", "")
        if isinstance(rmse, float):
            rmse = f"{rmse:.6f}"
        lines.append(
            f"| {comparison['train']} | {comparison['eval']} | "
            f"{comparison['status']} | {rmse} |"
        )
    return "\n".join(lines) + "\n"


def _estimator_comparison_report(summary: dict[str, object]) -> str:
    lines = [
        "# Measured SysID Estimator Comparison",
        "",
        f"- status: `{summary.get('status')}`",
        f"- chosen_estimator: `{summary.get('chosen_estimator')}`",
        "",
        "| estimator | train nrmse | mean validation nrmse |",
        "| --- | ---: | ---: |",
    ]
    results = summary.get("results", {})
    if isinstance(results, dict):
        for name, entry in results.items():
            if not isinstance(entry, dict):
                continue
            train = entry.get("train", {})
            train_nrmse = train.get("nrmse_all") if isinstance(train, dict) else ""
            validation_nrmse = entry.get("mean_validation_nrmse", "")
            lines.append(
                f"| {name} | {_fmt_float(train_nrmse)} | {_fmt_float(validation_nrmse)} |"
            )
    return "\n".join(lines) + "\n"


def _residual_diagnostics_report(summary: dict[str, object]) -> str:
    lines = [
        "# Residual Diagnostics",
        "",
        f"- status: `{summary.get('status')}`",
        "",
        "| dataset | samples | top residual norm |",
        "| --- | ---: | ---: |",
    ]
    for entry in summary.get("datasets", []):
        if not isinstance(entry, dict):
            continue
        outliers = entry.get("top_outliers", [])
        top = outliers[0].get("residual_norm") if outliers else ""
        lines.append(
            f"| {entry.get('label')} | {entry.get('sample_count_after_edge_drop')} | "
            f"{_fmt_float(top)} |"
        )
    return "\n".join(lines) + "\n"


def _physical_consistency_report(summary: dict[str, object]) -> str:
    return (
        "# Physical Consistency Screen\n\n"
        f"- status: `{summary.get('status')}`\n"
        f"- physical_consistency_status: `{summary.get('physical_consistency_status')}`\n"
        f"- parameter_space: `{summary.get('parameter_space')}`\n"
        f"- limitation: {summary.get('base_parameter_limitation')}\n"
        f"- next_step: {summary.get('full_inertial_projection_next_step')}\n"
    )


def _gravity_comp_offline_sanity_report(summary: dict[str, object]) -> str:
    identified = summary.get("identified_effort_model", {})
    identified_status = identified.get("status") if isinstance(identified, dict) else ""
    return (
        "# Offline Gravity Compensation Sanity\n\n"
        f"- status: `{summary.get('status')}`\n"
        f"- identified_effort_model: `{identified_status}`\n"
        f"- can_use_for_real_nm_gravity_compensation: "
        f"`{summary.get('can_use_for_real_nm_gravity_compensation')}`\n"
        f"- sample_count: `{len(summary.get('default_model_gravity_samples', []))}`\n"
    )


def _fmt_float(value: object) -> str:
    if isinstance(value, (int, float)):
        return f"{float(value):.6f}"
    return str(value)


def _limitations_report(manifest: dict[str, object]) -> str:
    lines = ["# Parameter Bundle Limitations", ""]
    for limitation in manifest["limitations"]:
        lines.append(f"- {limitation}")
    return "\n".join(lines) + "\n"
