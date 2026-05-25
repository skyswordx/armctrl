"""Offline solver stage for processed identification datasets."""

from __future__ import annotations

import csv
import importlib.util
import json
import math
from pathlib import Path
from typing import Any


DOCUMENT_GATE_MAPPING = {
    "data_health": {
        "zh": "数据健康",
        "meaning": "采样时序、列完整性、数值合法性和力矩反馈是否可信。",
    },
    "excitation": {
        "zh": "激励充分性",
        "meaning": "关节姿态覆盖、跟踪情况和回归矩阵是否足以支撑辨识。",
    },
    "regressor_condition": {
        "zh": "回归矩阵条件数",
        "meaning": "Pinocchio/FIGAROH 构造的 Y 矩阵是否满秩、是否病态。",
    },
    "physical_consistency": {
        "zh": "参数物理一致性",
        "meaning": "最小二乘解是否满足正质量、正惯量等基本物理约束。",
    },
    "prediction_error": {
        "zh": "预测误差",
        "meaning": "用已辨识参数回代计算 tau_pred 后，残差是否足够小。",
    },
    "control_benefit": {
        "zh": "控制收益",
        "meaning": "上机 A/B 测试后，保持力矩、电流、拖动手感和姿态误差是否改善。",
    },
}


def solve_processed_dataset(
    *,
    output_dir: str | Path,
    processed_csv: str | Path,
    urdf_path: str | None,
    dof: int,
    tools: tuple[str, ...],
    quality_metrics: dict[str, Any],
) -> dict[str, Any]:
    """Run the fixed offline solver checks after postprocess."""

    output = Path(output_dir).expanduser().resolve()
    processed = Path(processed_csv).expanduser().resolve()
    metrics = {
        "schema": "armctrl-ident-solver-quality-v1",
        "processed_csv": str(processed),
        "urdf_path": urdf_path,
        "dof": int(dof),
        "document_gate_mapping": DOCUMENT_GATE_MAPPING,
        "input_quality": {
            "data_readiness_status": quality_metrics.get("data_readiness_status"),
            "data_health_status": quality_metrics.get("document_sections", {})
            .get("data_health", {})
            .get("status"),
            "excitation_status": quality_metrics.get("document_sections", {})
            .get("excitation", {})
            .get("status"),
        },
        "solvers": {},
        "overall_verdict": "not_evaluated",
    }

    selected = _selected_tool_names(tools)
    if "pinocchio" in selected:
        metrics["solvers"]["pinocchio"] = _run_pinocchio_solver(
            processed_csv=processed,
            urdf_path=urdf_path,
            dof=dof,
        )
    if "figaroh" in selected:
        metrics["solvers"]["figaroh"] = _figaroh_status()

    metrics["overall_verdict"] = _overall_verdict(metrics)
    metrics_path = output / "solver_metrics.json"
    report_path = output / "solver_report_zh.md"
    script_path = output / "run_solver_stage.py"
    metrics_path.write_text(json.dumps(metrics, ensure_ascii=False, sort_keys=True, indent=2), encoding="utf-8")
    report_path.write_text(_render_chinese_solver_report(metrics), encoding="utf-8")
    script_path.write_text(
        _render_solver_script(
            processed_csv=processed,
            urdf_path=urdf_path,
            dof=dof,
            tools=tools,
        ),
        encoding="utf-8",
    )
    return {
        "solver_metrics_json": str(metrics_path),
        "solver_report_zh": str(report_path),
        "solver_script": str(script_path),
        "solver_metrics": metrics,
    }


def _selected_tool_names(tools: tuple[str, ...]) -> set[str]:
    if not tools or tools == ("all",):
        return {"pinocchio", "figaroh", "flobaroid", "urdfly"}
    return set(tools)


def _run_pinocchio_solver(*, processed_csv: Path, urdf_path: str | None, dof: int) -> dict[str, Any]:
    if importlib.util.find_spec("pinocchio") is None:
        return {
            "status": "skipped",
            "reason": "Python module 'pinocchio' is not installed in this environment.",
            "stage": "import",
        }
    if not urdf_path:
        return {
            "status": "skipped",
            "reason": "URDF path was not provided; cannot build Pinocchio model.",
            "stage": "model_load",
        }
    resolved_urdf = Path(urdf_path).expanduser()
    if not resolved_urdf.is_file():
        return {
            "status": "skipped",
            "reason": f"URDF file not found: {resolved_urdf}",
            "stage": "model_load",
        }

    try:
        import numpy as np
        import pinocchio as pin

        model = pin.buildModelFromUrdf(str(resolved_urdf))
        data = model.createData()
        rows = _load_rows(processed_csv)
        if not rows:
            return {"status": "failed", "reason": "processed CSV has no rows", "stage": "load_data"}
        active_dof = min(int(dof), int(model.nv))
        train_mask = np.array([index % 5 != 0 for index in range(len(rows))], dtype=bool)
        runs = {}
        for mode in ("gravity_only", "full"):
            y_matrix, tau_matrix = _build_pinocchio_regression(
                pin=pin,
                model=model,
                data=data,
                rows=rows,
                dof=active_dof,
                mode=mode,
            )
            parameter_subset = _parameter_subset_for_mode(mode, int(y_matrix.shape[1]))
            y_solve = y_matrix[:, parameter_subset["selected_columns"]]
            equation_train_mask = np.repeat(train_mask, active_dof)
            tau_vector = tau_matrix.reshape(-1)
            pi_hat, _, lstsq_rank, _ = np.linalg.lstsq(
                y_solve[equation_train_mask],
                tau_vector[equation_train_mask],
                rcond=None,
            )
            tau_pred = (y_solve @ pi_hat).reshape(len(rows), active_dof)
            regressor = _regressor_metrics(y_solve)
            validation = _prediction_metrics(tau_matrix, tau_pred, train_mask == 0)
            train = _prediction_metrics(tau_matrix, tau_pred, train_mask)
            physical = _physical_consistency_for_mode(mode, model, pi_hat)
            runs[mode] = {
                "status": "completed",
                "interpretation": _mode_interpretation(mode),
                "parameter_subset": parameter_subset,
                "regressor": regressor,
                "lstsq_rank_train": int(lstsq_rank),
                "train_metrics": train,
                "validation_metrics": validation,
                "physical_consistency_min_norm_solution": physical,
                "verdict": _pinocchio_run_verdict(regressor, validation, physical),
            }
        return {
            "status": "completed",
            "model": {
                "name": model.name,
                "nq": int(model.nq),
                "nv": int(model.nv),
                "joint_names": [str(name) for name in model.names],
            },
            "split": {
                "method": "sample_index_mod_5_validation",
                "train_samples": int(train_mask.sum()),
                "validation_samples": int((~train_mask).sum()),
            },
            "runs": runs,
        }
    except Exception as exc:
        return {"status": "failed", "reason": str(exc), "stage": "solve"}


def _parameter_subset_for_mode(mode: str, parameter_count: int) -> dict[str, Any]:
    if mode != "gravity_only":
        return {
            "mode": "full_dynamic_parameters",
            "original_parameter_count": parameter_count,
            "selected_parameter_count": parameter_count,
            "selected_columns": list(range(parameter_count)),
            "meaning": "all Pinocchio dynamic parameters are used",
        }
    selected_columns = _gravity_base_parameter_columns(parameter_count)
    return {
        "mode": "gravity_base_columns",
        "original_parameter_count": parameter_count,
        "selected_parameter_count": len(selected_columns),
        "selected_columns": selected_columns,
        "meaning": (
            "mass and first-moment columns only; inertia tensor columns are omitted because "
            "quasi-static gravity sweeps do not excite them"
        ),
    }


def _gravity_base_parameter_columns(parameter_count: int) -> list[int]:
    selected: list[int] = []
    for block_start in range(0, parameter_count, 10):
        block_end = min(block_start + 10, parameter_count)
        selected.extend(range(block_start, min(block_start + 4, block_end)))
    return selected


def _figaroh_status() -> dict[str, Any]:
    if importlib.util.find_spec("figaroh") is None:
        return {
            "status": "skipped",
            "reason": "Python module 'figaroh' is not installed in this environment.",
        }
    return {
        "status": "available_not_invoked",
        "reason": (
            "FIGAROH is installed, but this fixed postprocess stage currently uses Pinocchio "
            "as the deterministic regressor backend. FIGAROH integration needs a robot-specific "
            "configuration mapping before it can safely replace the Pinocchio LS path."
        ),
    }


def _load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _build_pinocchio_regression(*, pin, model, data, rows: list[dict[str, str]], dof: int, mode: str):
    import numpy as np

    regressors = []
    torques = []
    for row in rows:
        q = np.array([float(row[f"q_proc_{index}"]) for index in range(1, model.nq + 1)], dtype=float)
        if mode == "gravity_only":
            dq = np.zeros(model.nv)
            ddq = np.zeros(model.nv)
        else:
            dq = np.array([float(row[f"dq_proc_{index}"]) for index in range(1, model.nv + 1)], dtype=float)
            ddq = np.array([float(row[f"ddq_proc_{index}"]) for index in range(1, model.nv + 1)], dtype=float)
        tau = np.array([float(row[f"tau_proc_{index}"]) for index in range(1, dof + 1)], dtype=float)
        regressors.append(pin.computeJointTorqueRegressor(model, data, q, dq, ddq)[:dof, :])
        torques.append(tau)
    return np.vstack(regressors), np.vstack(torques)


def _regressor_metrics(matrix) -> dict[str, Any]:
    import numpy as np

    singular_values = np.linalg.svd(matrix, compute_uv=False)
    tolerance = max(matrix.shape) * np.finfo(float).eps * singular_values[0]
    rank = int((singular_values > tolerance).sum())
    condition = math.inf if rank == 0 else float(singular_values[0] / singular_values[rank - 1])
    return {
        "row_count": int(matrix.shape[0]),
        "parameter_count": int(matrix.shape[1]),
        "rank": rank,
        "full_rank": rank == int(matrix.shape[1]),
        "condition_number_effective": condition,
        "singular_values_head": [float(value) for value in singular_values[:8]],
        "singular_values_tail_effective": [float(value) for value in singular_values[max(0, rank - 8) : rank]],
    }


def _prediction_metrics(tau, tau_pred, sample_mask) -> dict[str, Any]:
    import numpy as np

    selected_tau = tau[sample_mask]
    selected_pred = tau_pred[sample_mask]
    per_joint = []
    for joint in range(selected_tau.shape[1]):
        measured = selected_tau[:, joint]
        predicted = selected_pred[:, joint]
        residual = measured - predicted
        rmse = float(np.sqrt(np.mean(residual * residual)))
        tau_range = float(np.max(measured) - np.min(measured))
        tau_std = float(np.std(measured))
        ss_res = float(np.sum(residual * residual))
        ss_tot = float(np.sum((measured - np.mean(measured)) ** 2))
        per_joint.append(
            {
                "joint": joint + 1,
                "rmse_nm": rmse,
                "mae_nm": float(np.mean(np.abs(residual))),
                "p95_abs_nm": float(np.percentile(np.abs(residual), 95)),
                "tau_range_nm": tau_range,
                "tau_std_nm": tau_std,
                "nrmse_by_range": float(rmse / tau_range) if tau_range > 1e-12 else None,
                "nrmse_by_std": float(rmse / tau_std) if tau_std > 1e-12 else None,
                "r2": float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else None,
            }
        )
    flat_residual = (selected_tau - selected_pred).reshape(-1)
    return {
        "global_rmse_nm": float(np.sqrt(float(np.mean(flat_residual * flat_residual)))),
        "per_joint": per_joint,
    }


def _physical_consistency_from_min_norm(model, pi_hat) -> dict[str, Any]:
    links = []
    for index, name in enumerate(model.names):
        if index == 0:
            continue
        base = (index - 1) * 10
        params = pi_hat[base : base + 10]
        if len(params) < 10:
            continue
        mass = float(params[0])
        inertia_diag = [float(params[4]), float(params[6]), float(params[9])]
        links.append(
            {
                "joint_name": str(name),
                "mass": mass,
                "mass_positive": mass > 0.0,
                "inertia_diag_positive": all(value > 0.0 for value in inertia_diag),
                "inertia_diag": inertia_diag,
            }
        )
    return {
        "status": "pass"
        if links and all(link["mass_positive"] and link["inertia_diag_positive"] for link in links)
        else "fail",
        "links": links,
    }


def _physical_consistency_for_mode(mode: str, model, pi_hat) -> dict[str, Any]:
    if mode == "gravity_only":
        return {
            "status": "not_applicable",
            "reason": (
                "gravity_only solves a reduced mass/first-moment parameter vector, so it cannot be "
                "mapped back to full per-link inertia tensors for the standard physical consistency gate"
            ),
        }
    return _physical_consistency_from_min_norm(model, pi_hat)


def _mode_interpretation(mode: str) -> str:
    if mode == "gravity_only":
        return "将 dq/ddq 置零，只检查准静态重力项能否解释力矩趋势。"
    return "使用 q_proc/dq_proc/ddq_proc，检查完整动力学回归；会受数值微分噪声影响。"


def _pinocchio_run_verdict(regressor: dict[str, Any], validation: dict[str, Any], physical: dict[str, Any]) -> dict[str, Any]:
    bad_nrmse = [
        joint["joint"]
        for joint in validation["per_joint"]
        if joint["nrmse_by_range"] is None or joint["nrmse_by_range"] > 0.20
    ]
    bad_r2 = [joint["joint"] for joint in validation["per_joint"] if joint["r2"] is None or joint["r2"] < 0.80]
    return {
        "prediction_error_pass_by_20pct_nrmse_range": not bad_nrmse,
        "prediction_error_pass_by_r2_0p8": not bad_r2,
        "bad_nrmse_joints": bad_nrmse,
        "bad_r2_joints": bad_r2,
        "full_parameter_rank_identifiable": bool(regressor["full_rank"]),
        "physical_consistency_pass": physical["status"] == "pass" and bool(regressor["full_rank"]),
    }


def _overall_verdict(metrics: dict[str, Any]) -> str:
    pinocchio = metrics["solvers"].get("pinocchio")
    if not pinocchio or pinocchio.get("status") != "completed":
        return "solver_not_completed"
    gravity = pinocchio["runs"].get("gravity_only", {})
    full = pinocchio["runs"].get("full", {})
    gravity_prediction = gravity.get("verdict", {}).get("prediction_error_pass_by_20pct_nrmse_range", False)
    full_rank = full.get("verdict", {}).get("full_parameter_rank_identifiable", False)
    physical = full.get("verdict", {}).get("physical_consistency_pass", False)
    if gravity_prediction and full_rank and physical:
        return "pass"
    if gravity_prediction:
        return "gravity_trend_only"
    return "fail"


def _render_chinese_solver_report(metrics: dict[str, Any]) -> str:
    lines = [
        "# 参数辨识结果质量评估指标 - 自动求解报告",
        "",
        f"- 总体结论: `{metrics['overall_verdict']}`",
        f"- processed CSV: `{metrics['processed_csv']}`",
        f"- URDF: `{metrics.get('urdf_path') or '未提供'}`",
        "",
        "## 指标对照表",
        "",
        "| 文档指标 | 当前状态 | 怎么看 |",
        "| --- | --- | --- |",
    ]
    input_quality = metrics["input_quality"]
    lines.append(f"| 数据健康 | `{input_quality.get('data_health_status')}` | 先看 CSV、时间戳、数值和采样是否可靠。 |")
    lines.append(f"| 激励充分性 | `{input_quality.get('excitation_status')}` | 先看关节覆盖和实际跟踪，再看真实回归矩阵 rank。 |")
    pinocchio = metrics["solvers"].get("pinocchio", {})
    if pinocchio.get("status") == "completed":
        full = pinocchio["runs"]["full"]
        gravity = pinocchio["runs"]["gravity_only"]
        lines.append(
            "| 回归矩阵条件数 | `rank {rank}/{count}, cond {cond}` | rank 不满说明完整参数不可全部辨识；条件数越大越病态。 |".format(
                rank=full["regressor"]["rank"],
                count=full["regressor"]["parameter_count"],
                cond=_format_number(full["regressor"]["condition_number_effective"]),
            )
        )
        lines.append(
            "| 参数物理一致性 | `{status}` | 最小二乘参数需要正质量、正惯量；失败时不能直接上机。 |".format(
                status=full["physical_consistency_min_norm_solution"]["status"]
            )
        )
        lines.append(
            "| 预测误差 | `{status}` | 重点看验证集每关节 NRMSE 和 R2；NRMSE < 20% 只是起点，R2 低说明解释力弱。 |".format(
                status=gravity["verdict"]["prediction_error_pass_by_20pct_nrmse_range"]
            )
        )
    else:
        lines.append(f"| 回归矩阵条件数 | `not_evaluated` | Pinocchio 未完成: {pinocchio.get('reason', '未知原因')} |")
        lines.append("| 参数物理一致性 | `not_evaluated` | solver 未完成。 |")
        lines.append("| 预测误差 | `not_evaluated` | solver 未完成。 |")
    lines.append("| 控制收益 | `not_evaluated` | 需要上机 A/B 测试，离线 solver 不能替代。 |")

    lines.extend(["", "## Pinocchio 求解结果", ""])
    if pinocchio.get("status") != "completed":
        lines.append(f"- 状态: `{pinocchio.get('status')}`")
        lines.append(f"- 原因: {pinocchio.get('reason', '未知')}")
    else:
        for mode_name in ("gravity_only", "full"):
            run = pinocchio["runs"][mode_name]
            lines.extend(
                [
                    f"### {mode_name}",
                    "",
                    f"- 含义: {run['interpretation']}",
                    "- 回归矩阵: `{rows} x {cols}`, rank `{rank}/{cols}`, 有效条件数 `{cond}`".format(
                        rows=run["regressor"]["row_count"],
                        cols=run["regressor"]["parameter_count"],
                        rank=run["regressor"]["rank"],
                        cond=_format_number(run["regressor"]["condition_number_effective"]),
                    ),
                    f"- 预测误差门: `{run['verdict']['prediction_error_pass_by_20pct_nrmse_range']}`",
                    f"- R2 门: `{run['verdict']['prediction_error_pass_by_r2_0p8']}`",
                    f"- 物理一致性门: `{run['verdict']['physical_consistency_pass']}`",
                    "",
                    "| 关节 | 验证 RMSE(Nm) | NRMSE(range) | R2 | tau range(Nm) |",
                    "| --- | ---: | ---: | ---: | ---: |",
                ]
            )
            for joint in run["validation_metrics"]["per_joint"]:
                lines.append(
                    "| {joint} | {rmse:.6f} | {nrmse} | {r2} | {tau:.6f} |".format(
                        joint=joint["joint"],
                        rmse=joint["rmse_nm"],
                        nrmse=_format_optional(joint["nrmse_by_range"]),
                        r2=_format_optional(joint["r2"]),
                        tau=joint["tau_range_nm"],
                    )
                )
            lines.append("")

    figaroh = metrics["solvers"].get("figaroh")
    if figaroh is not None:
        lines.extend(
            [
                "## FIGAROH 状态",
                "",
                f"- 状态: `{figaroh.get('status')}`",
                f"- 说明: {figaroh.get('reason', '')}",
                "",
            ]
        )
    lines.extend(
        [
            "## 中文术语速查",
            "",
            "- RMSE: 均方根误差，单位 Nm，越小越好。",
            "- NRMSE(range): RMSE 除以该关节力矩变化范围，低于 10% 到 20% 通常才算较好的起点。",
            "- R2: 力矩变化解释率，接近 1 好；低于 0.8 说明模型解释力不足；负数通常很差。",
            "- rank: 回归矩阵有效秩。rank 小于参数数时，说明有些参数无法由这批数据区分。",
            "- condition number: 条件数。越大越病态，参数越容易被噪声放大。",
            "- physical consistency: 物理一致性，至少要求质量为正、惯量为正；不过关不能直接上机。",
            "",
        ]
    )
    return "\n".join(lines)


def _render_solver_script(*, processed_csv: Path, urdf_path: str | None, dof: int, tools: tuple[str, ...]) -> str:
    processed_literal = str(processed_csv).replace("\\", "\\\\")
    urdf_literal = None if urdf_path is None else str(urdf_path).replace("\\", "\\\\")
    tools_literal = tuple(tools)
    return f'''"""Re-run the fixed solver stage for this processed dataset."""

from __future__ import annotations

import json
from pathlib import Path

from armctrl.identification.solver import solve_processed_dataset


def main() -> None:
    output_dir = Path(__file__).resolve().parent
    result = solve_processed_dataset(
        output_dir=output_dir,
        processed_csv=Path(r"{processed_literal}"),
        urdf_path={urdf_literal!r},
        dof={int(dof)},
        tools={tools_literal!r},
        quality_metrics={{}},
    )
    print(json.dumps({{
        "solver_metrics_json": result["solver_metrics_json"],
        "solver_report_zh": result["solver_report_zh"],
    }}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
'''


def _format_number(value: float) -> str:
    if not math.isfinite(value):
        return "inf"
    return f"{value:.6g}"


def _format_optional(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:.6f}"
