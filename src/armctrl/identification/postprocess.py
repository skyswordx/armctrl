"""辨识数据后处理。

当前实现只做保守的通用预处理：
- 可选滑动平均；
- 用中心差分从平滑后位置计算速度和加速度；
- 同时输出同一条处理链上的 `q_proc / dq_proc / ddq_proc / tau_proc`；
- 写出统一 `processed_samples.csv`；
- 生成外部工具交接说明。

真正的滤波器阶数、截止频率、基参数提取和最小二乘求解应交给离线辨识工具配置。
"""

from __future__ import annotations

import csv
import json
import math
import statistics
from pathlib import Path

from armctrl.identification.safety import model_coordinate_contract, model_safety_limits
from armctrl.identification.solver import solve_processed_dataset
from armctrl.identification.tools import selected_tools, write_tool_handoff
from armctrl.protocol.enums import CommandStatus, ErrorCode
from armctrl.protocol.errors import ArmctrlError
from armctrl.protocol.models import CommandResponse


def postprocess_dataset(
    *,
    dataset_dir: str | Path,
    output_dir: str | Path,
    tools: tuple[str, ...] = ("all",),
    urdf_path: str | None = None,
    smoothing_window: int = 5,
    filter_mode: str = "moving_average",
) -> CommandResponse:
    """把 raw_samples.csv 转成 processed_samples.csv 并生成工具交接文件。"""

    dataset = Path(dataset_dir).expanduser().resolve()
    output = Path(output_dir).expanduser().resolve()
    try:
        manifest = _load_manifest(dataset)
        dof = int(manifest["dof"])
        rows = _load_rows(dataset / manifest["raw_samples"])
        processed_path = _write_processed(
            rows,
            output,
            dof=dof,
            smoothing_window=smoothing_window,
            filter_mode=filter_mode,
        )
        lerobot_contract_path = _write_lerobot_contract(output, manifest.get("lerobot_contract", {}))
        handoff_path = write_tool_handoff(
            output_dir=output,
            dataset_dir=dataset,
            processed_csv=processed_path,
            urdf_path=urdf_path or manifest.get("urdf_path"),
            tools=tools,
        )
        quality_metrics = _build_quality_metrics(
            manifest,
            rows,
            dataset_dir=dataset,
            tools=tools,
            preprocessing=_preprocessing_metadata(filter_mode=filter_mode, smoothing_window=smoothing_window),
        )
        quality_metrics_path = _write_quality_metrics(output, quality_metrics)
        quality_report_path = _write_quality_report(output, quality_metrics)
        solver_detail = solve_processed_dataset(
            output_dir=output,
            processed_csv=processed_path,
            urdf_path=urdf_path or manifest.get("urdf_path"),
            dof=dof,
            tools=tools,
            quality_metrics=quality_metrics,
        )
        return CommandResponse(
            CommandStatus.COMPLETED,
            "identification dataset postprocessed",
            detail={
                "dataset_dir": str(dataset),
                "output_dir": str(output),
                "processed_csv": str(processed_path),
                "tool_handoff": str(handoff_path),
                "lerobot_contract_json": str(lerobot_contract_path),
                "quality_metrics_json": str(quality_metrics_path),
                "quality_report_md": str(quality_report_path),
                "solver_metrics_json": solver_detail["solver_metrics_json"],
                "solver_report_zh": solver_detail["solver_report_zh"],
                "solver_script": solver_detail["solver_script"],
                "solver_metrics": solver_detail["solver_metrics"],
                "quality_metrics": quality_metrics,
                "dof": dof,
                "sample_count": len(rows),
                "lerobot_contract": manifest.get("lerobot_contract", {}),
            },
        )
    except Exception as exc:
        error = exc if isinstance(exc, ArmctrlError) else ArmctrlError(ErrorCode.INVALID_REQUEST, str(exc))
        return CommandResponse(CommandStatus.REJECTED, error.message, error=error)


def _load_manifest(dataset_dir: Path) -> dict:
    manifest_path = dataset_dir / "manifest.json"
    if not manifest_path.is_file():
        raise ArmctrlError(ErrorCode.INVALID_REQUEST, f"manifest not found: {manifest_path}")
    with manifest_path.open(encoding="utf-8") as file:
        return json.load(file)


def _load_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise ArmctrlError(ErrorCode.INVALID_REQUEST, f"raw samples not found: {path}")
    with path.open(encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _write_processed(
    rows: list[dict[str, str]],
    output_dir: Path,
    *,
    dof: int,
    smoothing_window: int,
    filter_mode: str,
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    times = [float(row["t_s"]) for row in rows]
    q_series = [[float(row[f"q_{joint}"]) for row in rows] for joint in range(1, dof + 1)]
    tau_series = [[float(row[f"tau_meas_{joint}"]) for row in rows] for joint in range(1, dof + 1)]
    # 这里故意让 q 和 tau 共享同一个简单平滑窗口。
    # 目标不是在 armctrl 内部做最终版辨识滤波器，而是先给离线工具一个
    # 自洽的 processed 四元组，避免后续脚本把 raw q/dq 与 proc ddq/tau 混合使用。
    filter_fn = _filter_function(filter_mode)
    q_smooth = [filter_fn(values, smoothing_window) for values in q_series]
    tau_smooth = [filter_fn(values, smoothing_window) for values in tau_series]
    dq_proc = [_central_difference(times, values) for values in q_smooth]
    ddq_proc = [_central_difference(times, values) for values in dq_proc]
    fieldnames = list(rows[0].keys()) + [f"q_proc_{index}" for index in range(1, dof + 1)] + [f"dq_proc_{index}" for index in range(1, dof + 1)] + [f"ddq_proc_{index}" for index in range(1, dof + 1)] + [f"tau_proc_{index}" for index in range(1, dof + 1)]
    output_path = output_dir / "processed_samples.csv"
    with output_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for row_index, row in enumerate(rows):
            output_row = dict(row)
            for joint in range(1, dof + 1):
                output_row[f"q_proc_{joint}"] = q_smooth[joint - 1][row_index]
                output_row[f"dq_proc_{joint}"] = dq_proc[joint - 1][row_index]
                output_row[f"ddq_proc_{joint}"] = ddq_proc[joint - 1][row_index]
                output_row[f"tau_proc_{joint}"] = tau_smooth[joint - 1][row_index]
            writer.writerow(output_row)
    return output_path


def _filter_function(filter_mode: str):
    if filter_mode == "moving_average":
        return _moving_average
    if filter_mode == "zero_phase_moving_average":
        return _zero_phase_moving_average
    raise ValueError(f"unsupported filter_mode: {filter_mode}")


def _preprocessing_metadata(*, filter_mode: str, smoothing_window: int) -> dict:
    return {
        "filter_mode": filter_mode,
        "smoothing_window": smoothing_window,
        "zero_phase": filter_mode.startswith("zero_phase"),
        "note": (
            "q_proc and tau_proc are filtered with the same offline chain before central-difference dq/ddq. "
            "zero_phase_moving_average applies the symmetric FIR forward and backward to avoid time lag."
        ),
    }


def _write_lerobot_contract(output_dir: Path, contract: dict) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "lerobot_contract.json"
    with output_path.open("w", encoding="utf-8") as file:
        json.dump(contract, file, ensure_ascii=False, sort_keys=True, indent=2)
    return output_path


def _moving_average(values: list[float], window: int) -> list[float]:
    if window <= 1:
        return list(values)
    radius = max(0, window // 2)
    result = []
    for index in range(len(values)):
        start = max(0, index - radius)
        end = min(len(values), index + radius + 1)
        result.append(sum(values[start:end]) / (end - start))
    return result


def _zero_phase_moving_average(values: list[float], window: int) -> list[float]:
    forward = _moving_average(values, window)
    backward = _moving_average(list(reversed(forward)), window)
    return list(reversed(backward))


def _central_difference(times: list[float], values: list[float]) -> list[float]:
    if len(values) <= 1:
        return [0.0 for _ in values]
    result: list[float] = []
    for index in range(len(values)):
        if index == 0:
            dt = max(times[1] - times[0], 1e-9)
            result.append((values[1] - values[0]) / dt)
        elif index == len(values) - 1:
            dt = max(times[-1] - times[-2], 1e-9)
            result.append((values[-1] - values[-2]) / dt)
        else:
            dt = max(times[index + 1] - times[index - 1], 1e-9)
            result.append((values[index + 1] - values[index - 1]) / dt)
    return result


def _build_quality_metrics(
    manifest: dict,
    rows: list[dict[str, str]],
    *,
    dataset_dir: Path | None = None,
    tools: tuple[str, ...],
    preprocessing: dict | None = None,
) -> dict:
    dof = int(manifest["dof"])
    times = _float_column(rows, "t_s")
    profile_name = manifest.get("profile_name")
    profile_metadata = dict(manifest.get("profile_metadata", {}))
    min_actual_range_deg = _profile_min_actual_range_deg(profile_name)
    if profile_name == "gravity_sweep":
        profile_metadata["gravity_min_actual_range_deg"] = min_actual_range_deg
    if profile_name == "fourier_multisine":
        profile_metadata["dynamic_min_actual_range_deg"] = min_actual_range_deg
    expected_sample_count = _expected_sample_count(manifest)
    data_health = _data_health(rows, dof=dof, times=times, expected_sample_count=expected_sample_count)
    planned_limits = _planned_trajectory_limits(manifest, dataset_dir=dataset_dir, dof=dof)
    joint_metrics = [
        _joint_quality(
            rows,
            joint=joint,
            min_actual_range_deg=min_actual_range_deg,
        )
        for joint in range(1, dof + 1)
    ]
    excitation_status = _worst_status([joint["excitation_status"] for joint in joint_metrics])
    tool_execution = _tool_execution_status(tools)
    document_sections = {
        "data_health": {
            "status": data_health["status"],
            "summary": "CSV columns, numeric values, sample count, and time base checks.",
        },
        "excitation": {
            "status": excitation_status,
            "summary": (
                "Joint tracking plus absolute angle range checks. For gravity_sweep/fourier_multisine, high command "
                "coverage alone can be misleading when the absolute angle range is too small for sin/cos excitation."
            ),
        },
        "regressor_condition": {
            "status": "not_evaluated",
            "summary": "Requires Pinocchio, FIGAROH, or URDFly to build the true dynamics regressor.",
        },
        "physical_consistency": {
            "status": "not_evaluated",
            "summary": "Requires external identification output with masses, inertias, centers of mass, and base parameters.",
        },
        "prediction_error": {
            "status": "not_evaluated",
            "summary": "Requires solved parameters to compute tau_pred, residuals, RMSE, NRMSE, and R2.",
        },
        "control_benefit": {
            "status": "not_evaluated",
            "summary": "Requires an on-robot A/B test against default or previous gravity compensation parameters.",
        },
    }
    planned_readiness = "fail" if planned_limits["status"] == "fail" else "pass"
    readiness_inputs = [data_health["status"], excitation_status, planned_readiness]
    readiness = "pass" if all(status == "pass" for status in readiness_inputs) else "fail"
    return {
        "schema": "armctrl-ident-quality-v1",
        "profile_name": profile_name,
        "profile_metadata": profile_metadata,
        "preprocessing": preprocessing or _preprocessing_metadata(filter_mode="moving_average", smoothing_window=5),
        "planned_trajectory_limits": planned_limits,
        "sample_count": len(rows),
        "expected_sample_count": expected_sample_count,
        "dof": dof,
        "thresholds": {
            "min_coverage_ratio": 0.8,
            "max_platform_mean_error_rad": 0.02,
            "min_actual_range_deg": min_actual_range_deg,
            "max_sample_count_error_ratio": 0.05,
        },
        "data_health": data_health,
        "joint_metrics": joint_metrics,
        "tool_execution": tool_execution,
        "document_sections": document_sections,
        "data_readiness_status": readiness,
        "handoff_note": (
            "ident-postprocess creates cleaned CSVs, LeRobot-compatible metadata, quality gates, "
            "external-tool handoff artifacts, and a fixed solver stage. Pinocchio runs immediately "
            "when importable and a URDF is provided; FIGAROH helpers are used where safe."
        ),
    }


def _profile_min_actual_range_deg(profile_name: str | None) -> float | None:
    if profile_name == "gravity_sweep":
        return 30.0
    if profile_name == "fourier_multisine":
        return 60.0
    return None


def _planned_trajectory_limits(manifest: dict, *, dataset_dir: Path | None, dof: int) -> dict:
    model = manifest.get("model")
    contract = model_coordinate_contract(model, dof)
    limits = model_safety_limits(model, dof)
    planned_name = manifest.get("planned_trajectory")
    if dataset_dir is None or not planned_name:
        return {
            "status": "not_evaluated",
            "reason": "manifest does not reference planned_trajectory",
            "model": model,
            "coordinate_contract": contract,
        }
    planned_path = dataset_dir / planned_name
    if not planned_path.is_file():
        return {
            "status": "not_evaluated",
            "reason": f"planned trajectory not found: {planned_path}",
            "model": model,
            "coordinate_contract": contract,
        }
    violations: list[dict] = []
    ranges = [
        {"joint": joint, "min_rad": math.inf, "max_rad": -math.inf}
        for joint in range(1, dof + 1)
    ]
    limit_specs = (
        ("position", "q_cmd", limits.joint_min, limits.joint_max, "rad"),
        (
            "velocity",
            "dq_cmd",
            tuple(-value for value in limits.velocity_max),
            limits.velocity_max,
            "rad/s",
        ),
        (
            "acceleration",
            "ddq_cmd",
            tuple(-value for value in limits.acceleration_max),
            limits.acceleration_max,
            "rad/s^2",
        ),
    )
    try:
        planned_rows = _load_rows(planned_path)
        for row_index, row in enumerate(planned_rows):
            for joint in range(1, dof + 1):
                position = float(row[f"q_cmd_{joint}"])
                ranges[joint - 1]["min_rad"] = min(ranges[joint - 1]["min_rad"], position)
                ranges[joint - 1]["max_rad"] = max(ranges[joint - 1]["max_rad"], position)
                for kind, prefix, lower_values, upper_values, unit in limit_specs:
                    value = float(row[f"{prefix}_{joint}"])
                    lower = lower_values[joint - 1]
                    upper = upper_values[joint - 1]
                    if value < lower or value > upper:
                        violations.append(
                            {
                                "row_index": row_index,
                                "joint": joint,
                                "kind": kind,
                                "column": f"{prefix}_{joint}",
                                "value": value,
                                "lower": lower,
                                "upper": upper,
                                "unit": unit,
                            }
                        )
        for item in ranges:
            if math.isinf(item["min_rad"]):
                item["min_rad"] = None
                item["max_rad"] = None
            else:
                item["range_deg"] = (item["max_rad"] - item["min_rad"]) * 180.0 / math.pi
        return {
            "status": "fail" if violations else "pass",
            "model": model,
            "coordinate_contract": contract,
            "joint_min": list(limits.joint_min),
            "joint_max": list(limits.joint_max),
            "velocity_max": list(limits.velocity_max),
            "acceleration_max": list(limits.acceleration_max),
            "ranges": ranges,
            "violation_count": len(violations),
            "violations": violations[:20],
        }
    except Exception as exc:
        return {
            "status": "not_evaluated",
            "reason": str(exc),
            "model": model,
            "coordinate_contract": contract,
        }


def _data_health(rows: list[dict[str, str]], *, dof: int, times: list[float], expected_sample_count: int | None) -> dict:
    required_columns = ["t_s", "phase"]
    for joint in range(1, dof + 1):
        required_columns.extend(
            [
                f"q_{joint}",
                f"dq_{joint}",
                f"tau_meas_{joint}",
                f"q_cmd_{joint}",
                f"dq_cmd_{joint}",
                f"ddq_cmd_{joint}",
            ]
        )
    missing_columns = [column for column in required_columns if rows and column not in rows[0]]
    nonfinite_values = _count_nonfinite(rows, [column for column in required_columns if column != "phase"])
    dt = [times[index] - times[index - 1] for index in range(1, len(times))]
    monotonic = all(value > 0 for value in dt)
    sample_count_error_ratio = None
    if expected_sample_count:
        sample_count_error_ratio = abs(len(rows) - expected_sample_count) / expected_sample_count
    dt_stats = {
        "mean_s": _mean(dt),
        "std_s": _pstdev(dt),
        "min_s": min(dt) if dt else None,
        "max_s": max(dt) if dt else None,
        "p95_s": _percentile(dt, 0.95),
    }
    status = "pass"
    if missing_columns or nonfinite_values or not rows or not monotonic:
        status = "fail"
    elif sample_count_error_ratio is not None and sample_count_error_ratio > 0.05:
        status = "warn"
    return {
        "status": status,
        "missing_columns": missing_columns,
        "nonfinite_values": nonfinite_values,
        "sample_count": len(rows),
        "expected_sample_count": expected_sample_count,
        "sample_count_error_ratio": sample_count_error_ratio,
        "time_monotonic": monotonic,
        "dt": dt_stats,
    }


def _joint_quality(rows: list[dict[str, str]], *, joint: int, min_actual_range_deg: float | None = None) -> dict:
    q = _float_column(rows, f"q_{joint}")
    q_cmd = _float_column(rows, f"q_cmd_{joint}")
    dq = _float_column(rows, f"dq_{joint}")
    tau = _float_column(rows, f"tau_meas_{joint}")
    q_range = _range(q)
    q_cmd_range = _range(q_cmd)
    q_actual_range_deg = math.degrees(q_range)
    absolute_range_status = (
        "not_applicable"
        if min_actual_range_deg is None
        else ("pass" if q_actual_range_deg >= min_actual_range_deg else "fail")
    )
    coverage_ratio = q_range / q_cmd_range if q_cmd_range > 1e-12 else None
    errors = [actual - command for actual, command in zip(q, q_cmd, strict=False)]
    abs_errors = [abs(value) for value in errors]
    direction_reach = {
        "positive": _direction_reach(rows, joint=joint, sign=1),
        "negative": _direction_reach(rows, joint=joint, sign=-1),
    }
    direction_statuses = [value["status"] for value in direction_reach.values() if value["status"] != "not_applicable"]
    excitation_status = "pass"
    if q_cmd_range > 1e-12 and (coverage_ratio is None or coverage_ratio < 0.8):
        excitation_status = "fail"
    if absolute_range_status != "not_applicable":
        excitation_status = _worst_status([excitation_status, absolute_range_status])
    if direction_statuses:
        excitation_status = _worst_status([excitation_status, *direction_statuses])
    return {
        "joint": joint,
        "q_cmd_range_rad": q_cmd_range,
        "q_cmd_range_deg": math.degrees(q_cmd_range),
        "q_actual_range_rad": q_range,
        "q_actual_range_deg": q_actual_range_deg,
        "absolute_range_status": absolute_range_status,
        "min_actual_range_deg": min_actual_range_deg,
        "coverage_ratio": coverage_ratio,
        "qerr_rms_rad": _rms(errors),
        "qerr_p95_abs_rad": _percentile(abs_errors, 0.95),
        "qerr_max_abs_rad": max(abs_errors) if abs_errors else None,
        "max_abs_dq_radps": max((abs(value) for value in dq), default=None),
        "tau_range_nm": _range(tau),
        "tau_std_nm": _pstdev(tau),
        "direction_reach": direction_reach,
        "excitation_status": excitation_status,
    }


def _direction_reach(rows: list[dict[str, str]], *, joint: int, sign: int) -> dict:
    q_cmd_values = _float_column(rows, f"q_cmd_{joint}")
    max_abs_cmd = max((abs(value) for value in q_cmd_values), default=0.0)
    if max_abs_cmd <= 1e-12:
        return {"status": "not_applicable", "sample_count": 0}
    threshold = 0.5 * max_abs_cmd
    selected = [row for row in rows if sign * _float_value(row.get(f"q_cmd_{joint}", "")) >= threshold]
    if not selected:
        return {"status": "not_applicable", "sample_count": 0}
    q = [_float_value(row[f"q_{joint}"]) for row in selected]
    q_cmd = [_float_value(row[f"q_cmd_{joint}"]) for row in selected]
    errors = [actual - command for actual, command in zip(q, q_cmd, strict=False)]
    mean_error = _mean(errors)
    status = "pass" if mean_error is not None and abs(mean_error) <= 0.02 else "fail"
    return {
        "status": status,
        "sample_count": len(selected),
        "cmd_mean_rad": _mean(q_cmd),
        "actual_mean_rad": _mean(q),
        "mean_error_rad": mean_error,
        "actual_std_rad": _pstdev(q),
    }


def _tool_execution_status(tools: tuple[str, ...]) -> dict:
    statuses = {}
    for tool in selected_tools(tools):
        statuses[tool.name] = {
            "status": "handoff_only",
            "installed_python_module": tool.installed,
            "label": tool.label,
            "note": "ident-postprocess runs the fixed Pinocchio solver stage when available and still writes handoff files for richer offline tooling.",
        }
    return statuses


def _write_quality_metrics(output_dir: Path, metrics: dict) -> Path:
    output_path = output_dir / "quality_metrics.json"
    with output_path.open("w", encoding="utf-8") as file:
        json.dump(metrics, file, ensure_ascii=False, sort_keys=True, indent=2)
    return output_path


def _write_quality_report(output_dir: Path, metrics: dict) -> Path:
    output_path = output_dir / "quality_report.md"
    lines = [
        "# 参数辨识数据质量报告",
        "",
        f"- 轨迹类型: `{metrics.get('profile_name')}`",
        f"- 数据进入求解器前置状态: `{metrics['data_readiness_status']}`",
        f"- 实际样本数: `{metrics['sample_count']}`",
        f"- 期望样本数: `{metrics['expected_sample_count']}`",
        "",
        "## 指标对照表",
        "",
        "| 文档指标 | 当前状态 | 说明 |",
        "| --- | --- | --- |",
    ]
    zh_sections = {
        "data_health": "数据健康",
        "excitation": "激励充分性",
        "regressor_condition": "回归矩阵条件数",
        "physical_consistency": "参数物理一致性",
        "prediction_error": "预测误差",
        "control_benefit": "控制收益",
    }
    for name, section in metrics["document_sections"].items():
        lines.append(f"| {zh_sections.get(name, name)} | `{section['status']}` | {section['summary']} |")
    planned_limits = metrics.get("planned_trajectory_limits", {})
    lines.append(
        "| Planned URDF/模型限位 | `{status}` | planned_trajectory.csv 是否越过模型安全范围；SDK/URDF 坐标契约: {contract} |".format(
            status=planned_limits.get("status", "not_evaluated"),
            contract=planned_limits.get("coordinate_contract", {}).get("sdk_to_urdf_joint_order", "unknown"),
        )
    )
    lines.extend(
        [
            "",
            "## 工具状态",
            "",
            "本报告只判断数据清洗和进入求解器之前的质量门。后续真实求解结果见 `solver_report_zh.md`。",
            "",
            "| 工具 | 状态 | Python 模块是否可导入 |",
            "| --- | --- | --- |",
        ]
    )
    for name, status in metrics["tool_execution"].items():
        lines.append(f"| {name} | {status['status']} | {status['installed_python_module']} |")
    lines.extend(
        [
            "",
            "## 关节覆盖与跟踪",
            "",
            "| 关节 | 激励状态 | 实际角度范围(deg) | 命令角度范围(deg) | 覆盖率 | 跟踪 RMS(rad) | 力矩范围(Nm) |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for joint in metrics["joint_metrics"]:
        lines.append(
            "| {joint} | {status} | {actual:.3f} | {cmd:.3f} | {coverage} | {qerr:.5f} | {tau:.5f} |".format(
                joint=joint["joint"],
                status=joint["excitation_status"],
                actual=joint["q_actual_range_deg"],
                cmd=joint["q_cmd_range_deg"],
                coverage=_format_optional(joint["coverage_ratio"]),
                qerr=joint["qerr_rms_rad"],
                tau=joint["tau_range_nm"],
            )
        )
    if planned_limits:
        lines.extend(
            [
                "",
                "## Planned 轨迹模型限位检查",
                "",
                f"- 状态: `{planned_limits.get('status')}`",
                f"- 模型: `{planned_limits.get('model')}`",
                f"- SDK/URDF 坐标契约: `{planned_limits.get('coordinate_contract', {}).get('sdk_to_urdf_joint_order', 'unknown')}`",
            ]
        )
        if planned_limits.get("violations"):
            lines.extend(
                [
                    "",
                    "| 行号 | 关节 | 类型 | 列 | 数值 | 下限 | 上限 | 单位 |",
                    "| ---: | ---: | --- | --- | ---: | ---: | ---: | --- |",
                ]
            )
            for item in planned_limits["violations"][:10]:
                lines.append(
                    "| {row} | {joint} | {kind} | `{column}` | {value:.6f} | {lower:.6f} | {upper:.6f} | {unit} |".format(
                        row=item["row_index"],
                        joint=item["joint"],
                        kind=item["kind"],
                        column=item["column"],
                        value=item["value"],
                        lower=item["lower"],
                        upper=item["upper"],
                        unit=item["unit"],
                    )
                )
    lines.extend(
        [
            "",
            "## 正负方向到达情况",
            "",
            "| 关节 | 方向 | 状态 | 命令均值(rad) | 实际均值(rad) | 平均误差(rad) |",
            "| --- | --- | --- | ---: | ---: | ---: |",
        ]
    )
    direction_zh = {"positive": "正向", "negative": "负向"}
    for joint in metrics["joint_metrics"]:
        for direction, reach in joint["direction_reach"].items():
            lines.append(
                "| {joint} | {direction} | {status} | {cmd} | {actual} | {error} |".format(
                    joint=joint["joint"],
                    direction=direction_zh.get(direction, direction),
                    status=reach["status"],
                    cmd=_format_optional(reach.get("cmd_mean_rad")),
                    actual=_format_optional(reach.get("actual_mean_rad")),
                    error=_format_optional(reach.get("mean_error_rad")),
                )
            )
    lines.extend(
        [
            "",
            "## 怎么解读",
            "",
            "- `data_readiness_status=pass` 只表示数据文件、时间戳、覆盖和跟踪足以进入下一阶段。",
            "- `回归矩阵条件数 / 参数物理一致性 / 预测误差` 必须看 `solver_report_zh.md`，因为这些项需要真实 Pinocchio/FIGAROH 求解。",
            "- `控制收益` 只能通过上机 A/B 测试判断，离线后处理无法替代。",
            "",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output_path


def _expected_sample_count(manifest: dict) -> int | None:
    duration = manifest.get("duration_s")
    sample_hz = manifest.get("sample_hz")
    if duration is None or sample_hz is None:
        return None
    return int(round(float(duration) * float(sample_hz))) + 1


def _float_column(rows: list[dict[str, str]], column: str) -> list[float]:
    values = []
    for row in rows:
        value = row.get(column)
        if value is None or value == "":
            continue
        values.append(_float_value(value))
    return values


def _float_value(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _count_nonfinite(rows: list[dict[str, str]], columns: list[str]) -> int:
    count = 0
    for row in rows:
        for column in columns:
            if column not in row:
                continue
            value = _float_value(row[column])
            if not math.isfinite(value):
                count += 1
    return count


def _range(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return 0.0
    return max(finite) - min(finite)


def _mean(values: list[float]) -> float | None:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return None
    return sum(finite) / len(finite)


def _pstdev(values: list[float]) -> float | None:
    finite = [value for value in values if math.isfinite(value)]
    if len(finite) < 2:
        return 0.0 if finite else None
    return statistics.pstdev(finite)


def _rms(values: list[float]) -> float | None:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return None
    return math.sqrt(sum(value * value for value in finite) / len(finite))


def _percentile(values: list[float], percentile: float) -> float | None:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return None
    index = min(len(finite) - 1, max(0, int(math.ceil(percentile * len(finite))) - 1))
    return finite[index]


def _worst_status(statuses: list[str]) -> str:
    order = {"not_evaluated": 0, "not_applicable": 0, "pass": 1, "warn": 2, "fail": 3}
    if not statuses:
        return "not_evaluated"
    return max(statuses, key=lambda status: order.get(status, 0))


def _format_optional(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:.5f}"
