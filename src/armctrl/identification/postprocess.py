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
from pathlib import Path

from armctrl.identification.tools import write_tool_handoff
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
) -> CommandResponse:
    """把 raw_samples.csv 转成 processed_samples.csv 并生成工具交接文件。"""

    dataset = Path(dataset_dir).expanduser().resolve()
    output = Path(output_dir).expanduser().resolve()
    try:
        manifest = _load_manifest(dataset)
        dof = int(manifest["dof"])
        rows = _load_rows(dataset / manifest["raw_samples"])
        processed_path = _write_processed(rows, output, dof=dof, smoothing_window=smoothing_window)
        lerobot_contract_path = _write_lerobot_contract(output, manifest.get("lerobot_contract", {}))
        handoff_path = write_tool_handoff(
            output_dir=output,
            dataset_dir=dataset,
            processed_csv=processed_path,
            urdf_path=urdf_path or manifest.get("urdf_path"),
            tools=tools,
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


def _write_processed(rows: list[dict[str, str]], output_dir: Path, *, dof: int, smoothing_window: int) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    times = [float(row["t_s"]) for row in rows]
    q_series = [[float(row[f"q_{joint}"]) for row in rows] for joint in range(1, dof + 1)]
    tau_series = [[float(row[f"tau_meas_{joint}"]) for row in rows] for joint in range(1, dof + 1)]
    # 这里故意让 q 和 tau 共享同一个简单平滑窗口。
    # 目标不是在 armctrl 内部做最终版辨识滤波器，而是先给离线工具一个
    # 自洽的 processed 四元组，避免后续脚本把 raw q/dq 与 proc ddq/tau 混合使用。
    q_smooth = [_moving_average(values, smoothing_window) for values in q_series]
    tau_smooth = [_moving_average(values, smoothing_window) for values in tau_series]
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
