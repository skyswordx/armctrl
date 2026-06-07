from __future__ import annotations

from dataclasses import dataclass
import csv
import importlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np


@dataclass(frozen=True)
class SysIdSolveResult:
    schema: str
    sample_count: int
    artifacts: dict[str, str]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "sample_count": self.sample_count,
            "artifacts": self.artifacts,
        }


class SysIdSolver:
    def run(self, dataset_dir: Path) -> SysIdSolveResult:
        processed_dir = dataset_dir / "processed"
        processed_samples_path = processed_dir / "processed_samples.csv"
        quality_metrics_path = processed_dir / "quality_metrics.json"

        processed_rows = _read_csv(processed_samples_path)
        quality_metrics = json.loads(quality_metrics_path.read_text(encoding="utf-8"))
        manifest = json.loads((dataset_dir / "manifest.json").read_text(encoding="utf-8"))
        input_quality_status = str(quality_metrics["data_health"]["status"])
        backend_status = {
            "pinocchio": _module_status("pinocchio"),
            "figaroh": _module_status("figaroh"),
        }
        residual_summary = _fake_zero_tau_residual_summary(processed_rows)
        pinocchio_result = _pinocchio_solver_result(
            backend_status=backend_status["pinocchio"],
            manifest=manifest,
            rows=processed_rows,
            dataset_dir=dataset_dir,
        )
        regressor_condition = {"pinocchio": pinocchio_result["condition"]}
        prediction_error = {"pinocchio": pinocchio_result["prediction_error"]}
        overall_verdict = (
            "solver_ready"
            if input_quality_status == "pass"
            and regressor_condition["pinocchio"]["status"] == "computed"
            else "solver_handoff_only"
        )

        metrics = {
            "schema": "armctrl.sysid_solver_quality.v1",
            "sample_count": len(processed_rows),
            "source_run": _source_run_summary(manifest),
            "input_quality_status": input_quality_status,
            "overall_verdict": overall_verdict,
            "backend_status": backend_status,
            "regressor_condition": regressor_condition,
            "prediction_error": prediction_error,
            "physical_consistency": {
                "status": "not_evaluated",
                "source": "figaroh_or_manual_review_required",
            },
            "figaroh_base_parameters": {
                "status": "not_evaluated",
                "source": "figaroh_required",
            },
            "residual_summary": residual_summary,
            "solver_boundary": {
                "pinocchio": "deterministic regressor and least-squares backend",
                "figaroh": "external mature identification toolkit handoff",
                "note": "clean rebuild does not reimplement FIGAROH internals",
            },
        }

        solver_metrics_path = processed_dir / "solver_metrics.json"
        solver_metrics_path.write_text(
            json.dumps(metrics, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        solver_report_path = processed_dir / "solver_report_zh.md"
        solver_report_path.write_text(_solver_report(metrics), encoding="utf-8")

        return SysIdSolveResult(
            schema="armctrl.sysid_solve.v1",
            sample_count=len(processed_rows),
            artifacts={
                "solver_metrics": str(solver_metrics_path),
                "solver_report": str(solver_report_path),
            },
        )


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _module_status(module_name: str) -> dict[str, str]:
    if module_name in sys.modules:
        return {"status": "available", "module": module_name}
    spec = importlib.util.find_spec(module_name)
    if spec is None:
        return {"status": "missing", "module": module_name}
    return {"status": "available", "module": module_name}


def _source_run_summary(manifest: dict[str, object]) -> dict[str, object]:
    acceptance = _dict_value(manifest.get("acceptance"))
    readiness = _dict_value(manifest.get("readiness"))
    motion_runtime = _dict_value(manifest.get("motion_runtime"))
    tracking = _dict_value(motion_runtime.get("tracking"))
    return {
        "schema": "armctrl.sysid_solver_source_run.v1",
        "manifest_schema": manifest.get("schema"),
        "adapter": manifest.get("adapter"),
        "run_status": manifest.get("run_status"),
        "acceptance": _pick(
            acceptance,
            ("stage", "status", "next_gate"),
        ),
        "readiness": _pick(
            readiness,
            ("artifact_path", "agent_sysid_smoke_allowed"),
        ),
        "motion_runtime": {
            **_pick(
                motion_runtime,
                (
                    "status",
                    "producer",
                    "mode",
                    "trajectory_sample_hz",
                    "actual_send_hz",
                    "send_jitter_ms_p95",
                    "send_jitter_ms_p99",
                    "controller_dt_s",
                    "sample_count",
                ),
            ),
            "fault_flags": _fault_flags(motion_runtime.get("fault_flags")),
            "tracking": _pick(
                tracking,
                (
                    "q_cmd_delta_max_abs_rad",
                    "q_meas_delta_max_abs_rad",
                    "max_abs_sample_tracking_error_rad",
                    "final_tracking_error_max_abs_rad",
                ),
            ),
            **_pick(motion_runtime, ("landing_mode",)),
        },
    }


def _dict_value(value: object) -> dict[str, object]:
    if isinstance(value, dict):
        return value
    return {}


def _pick(
    source: dict[str, object],
    keys: tuple[str, ...],
) -> dict[str, object]:
    return {key: source[key] for key in keys if key in source}


def _fault_flags(value: object) -> list[str]:
    if isinstance(value, list | tuple):
        return [str(flag) for flag in value]
    return []


def _pinocchio_solver_result(
    *,
    backend_status: dict[str, str],
    manifest: dict[str, object],
    rows: list[dict[str, str]],
    dataset_dir: Path,
) -> dict[str, object]:
    if backend_status["status"] != "available":
        reason = "pinocchio module is not importable"
        return {
            "condition": {
                "status": "not_evaluated",
                "reason": reason,
            },
            "prediction_error": {
                "status": "not_evaluated",
                "reason": reason,
            },
        }
    try:
        y_matrix, tau_vector, sample_count = _pinocchio_matrices(
            manifest=manifest,
            rows=rows,
            dataset_dir=dataset_dir,
        )
        condition = _regressor_condition(y_matrix)
        parameter_vector, *_ = np.linalg.lstsq(y_matrix, tau_vector, rcond=None)
        tau_pred = y_matrix @ parameter_vector
        residual = tau_vector - tau_pred
        rmse = float(np.sqrt(np.mean(residual * residual))) if residual.size else 0.0
        return {
            "condition": condition,
            "prediction_error": {
                "status": "computed",
                "sample_count": sample_count,
                "observation_count": int(tau_vector.size),
                "parameter_count": int(parameter_vector.size),
                "rmse_nm": rmse,
            },
        }
    except Exception as exc:  # pragma: no cover - message is user-facing evidence.
        reason = str(exc)
        return {
            "condition": {
                "status": "failed",
                "reason": reason,
            },
            "prediction_error": {
                "status": "failed",
                "reason": reason,
            },
        }


def _pinocchio_matrices(
    *,
    manifest: dict[str, object],
    rows: list[dict[str, str]],
    dataset_dir: Path,
) -> tuple[np.ndarray, np.ndarray, int]:
    pinocchio = importlib.import_module("pinocchio")
    request = manifest["profile"] and manifest.get("request", {})
    urdf_path = _resolve_dataset_path(dataset_dir, str(request["urdf_path"]))
    model = pinocchio.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    regressors = []
    tau_blocks = []
    for row in rows:
        q = _row_vector(row, prefix="q_proc_", dof=model.nq)
        v = _row_vector(row, prefix="dq_proc_", dof=model.nv)
        a = np.zeros(model.nv)
        regressor = pinocchio.computeJointTorqueRegressor(model, data, q, v, a)
        if regressor is None:
            regressor = data.jointTorqueRegressor
        regressors.append(np.asarray(regressor, dtype=float))
        tau_blocks.append(_row_vector(row, prefix="tau_proc_", dof=model.nv))
    return np.vstack(regressors), np.concatenate(tau_blocks), len(rows)


def _regressor_condition(y_matrix: np.ndarray) -> dict[str, object]:
    singular_values = np.linalg.svd(y_matrix, compute_uv=False)
    if singular_values.size == 0:
        return {
            "status": "computed",
            "row_count": int(y_matrix.shape[0]),
            "column_count": int(y_matrix.shape[1]),
            "rank": 0,
            "effective_condition_number": float("inf"),
        }
    tolerance = np.finfo(float).eps * max(y_matrix.shape) * singular_values[0]
    nonzero_singular_values = singular_values[singular_values > tolerance]
    rank = int(nonzero_singular_values.size)
    condition_number = (
        float(nonzero_singular_values[0] / nonzero_singular_values[-1])
        if rank
        else float("inf")
    )
    return {
        "status": "computed",
        "row_count": int(y_matrix.shape[0]),
        "column_count": int(y_matrix.shape[1]),
        "rank": rank,
        "effective_condition_number": condition_number,
    }


def _resolve_dataset_path(dataset_dir: Path, path_text: str) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    candidate = dataset_dir / path
    if candidate.exists():
        return candidate
    return Path.cwd() / path


def _row_vector(row: dict[str, str], *, prefix: str, dof: int) -> np.ndarray:
    return np.array([float(row[f"{prefix}{index + 1}"]) for index in range(dof)])


def _fake_zero_tau_residual_summary(rows: list[dict[str, str]]) -> dict[str, float]:
    tau_values: list[float] = []
    for row in rows:
        for key, value in row.items():
            if key.startswith("tau_proc_"):
                tau_values.append(float(value))
    if not tau_values:
        return {"fake_zero_tau_rmse_nm": 0.0}
    mean_square = sum(value * value for value in tau_values) / len(tau_values)
    return {"fake_zero_tau_rmse_nm": mean_square**0.5}


def _solver_report(metrics: dict[str, object]) -> str:
    backend_status = metrics["backend_status"]
    pinocchio_condition = metrics["regressor_condition"]["pinocchio"]
    pinocchio_prediction = metrics["prediction_error"]["pinocchio"]
    physical_consistency = metrics["physical_consistency"]
    figaroh_base_parameters = metrics["figaroh_base_parameters"]
    residual_summary = metrics["residual_summary"]
    source_run = _dict_value(metrics.get("source_run"))
    title = "SysID \u6c42\u89e3\u62a5\u544a"
    note = (
        "\u5f53\u524d clean rebuild \u53ea\u56fa\u5b9a\u6c42\u89e3\u9636\u6bb5"
        "\u7684\u6587\u4ef6\u5951\u7ea6\u3001\u540e\u7aef\u53ef\u7528\u6027"
        "\u68c0\u67e5\u548c\u5047\u6570\u636e\u6b8b\u5dee\u5192\u70df\u68c0\u67e5\u3002"
        "\u771f\u5b9e Pinocchio regressor\u3001rank\u3001condition number\u3001"
        "prediction error \u548c physical consistency \u5c06\u5728\u540e\u7eed"
        "\u5c0f\u6b65\u63a5\u5165\u3002"
    )
    return (
        f"# {title}\n\n"
        f"- schema: `{metrics['schema']}`\n"
        f"- sample_count: `{metrics['sample_count']}`\n"
        f"- input_quality_status: `{metrics['input_quality_status']}`\n"
        f"- overall_verdict: `{metrics['overall_verdict']}`\n"
        f"- Pinocchio: `{backend_status['pinocchio']['status']}`\n"
        f"- FIGAROH: `{backend_status['figaroh']['status']}`\n"
        f"- Pinocchio regressor: `{pinocchio_condition['status']}`\n"
        f"- Pinocchio prediction_error: `{pinocchio_prediction['status']}`\n"
        f"- 物理一致性: `{physical_consistency['status']}`\n"
        f"- 基础参数: `{figaroh_base_parameters['status']}`\n"
        f"{_source_run_report(source_run)}"
        "- residual_summary: "
        f"`fake_zero_tau_rmse_nm={residual_summary['fake_zero_tau_rmse_nm']}`\n\n"
        f"{note}\n"
    )


def _source_run_report(source_run: dict[str, object]) -> str:
    acceptance = _dict_value(source_run.get("acceptance"))
    readiness = _dict_value(source_run.get("readiness"))
    motion_runtime = _dict_value(source_run.get("motion_runtime"))
    tracking = _dict_value(motion_runtime.get("tracking"))
    fault_flags = motion_runtime.get("fault_flags")
    if not isinstance(fault_flags, list):
        fault_flags = []
    return (
        f"- source_run.adapter: `{source_run.get('adapter')}`\n"
        f"- source_run.run_status: `{source_run.get('run_status')}`\n"
        "- source_run.acceptance: "
        f"`{acceptance.get('stage')}/{acceptance.get('status')}`\n"
        "- source_run.readiness.agent_sysid_smoke_allowed: "
        f"`{readiness.get('agent_sysid_smoke_allowed')}`\n"
        "- source_run.motion_runtime.status: "
        f"`{motion_runtime.get('status')}`\n"
        "- source_run.motion_runtime.actual_send_hz: "
        f"`{motion_runtime.get('actual_send_hz')}`\n"
        "- source_run.motion_runtime.controller_dt_s: "
        f"`{motion_runtime.get('controller_dt_s')}`\n"
        "- source_run.tracking.final_tracking_error_max_abs_rad: "
        f"`{tracking.get('final_tracking_error_max_abs_rad')}`\n"
        "- source_run.motion_runtime.fault_flags: "
        f"`{json.dumps(fault_flags, ensure_ascii=False)}`\n"
        "- source_run.motion_runtime.landing_mode: "
        f"`{motion_runtime.get('landing_mode')}`\n"
    )
