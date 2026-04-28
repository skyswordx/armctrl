"""外部辨识工具交接文件。

armctrl 只准备数据和脚手架，不在这里重复实现外部工具已经成熟的回归矩阵与优化流程。
"""

from __future__ import annotations

import importlib.util
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ExternalToolProfile:
    name: str
    label: str
    python_module: str | None
    purpose: str
    handoff: str

    @property
    def installed(self) -> bool:
        if self.python_module is None:
            return False
        return importlib.util.find_spec(self.python_module) is not None


TOOL_PROFILES: dict[str, ExternalToolProfile] = {
    "pinocchio": ExternalToolProfile(
        name="pinocchio",
        label="Pinocchio",
        python_module="pinocchio",
        purpose="用 URDF 构建刚体动力学模型，并调用 computeJointTorqueRegressor 生成观测矩阵。",
        handoff="默认优先读取 processed_samples.csv 的 q_proc/dq_proc/ddq_proc/tau_proc；如需对比电流换算原始力矩噪声，可再单独切回 tau_meas。",
    ),
    "figaroh": ExternalToolProfile(
        name="figaroh",
        label="FIGAROH",
        python_module="figaroh",
        purpose="做标定与动力学辨识流程编排。",
        handoff="把 URDF、manifest.json 和 processed_samples.csv 映射成 FIGAROH identification 配置。",
    ),
    "flobaroid": ExternalToolProfile(
        name="flobaroid",
        label="FloBaRoID",
        python_module=None,
        purpose="URDF 驱动的参数辨识、滤波、OLS/WLS 和 URDF 参数输出流程。",
        handoff="把 processed_samples.csv 转成 FloBaRoID 期望的数据表，再使用其优化与参数输出流程。",
    ),
    "urdfly": ExternalToolProfile(
        name="urdfly",
        label="URDFly",
        python_module="urdfly",
        purpose="从 URDF 生成符号动力学回归矩阵代码。",
        handoff="用 URDFly 生成 regressor 代码，然后优先读取 processed_samples.csv 的 q_proc/dq_proc/ddq_proc/tau_proc 组装 Yπ=τ。",
    ),
}


def selected_tools(names: tuple[str, ...]) -> list[ExternalToolProfile]:
    if names == ("all",):
        return list(TOOL_PROFILES.values())
    tools = []
    for name in names:
        if name not in TOOL_PROFILES:
            raise ValueError(f"unsupported tool {name}")
        tools.append(TOOL_PROFILES[name])
    return tools


def write_tool_handoff(
    *,
    output_dir: Path,
    dataset_dir: Path,
    processed_csv: Path,
    urdf_path: str | None,
    tools: tuple[str, ...],
) -> Path:
    """写入离线工具交接说明和 Pinocchio 脚手架。"""

    output_dir.mkdir(parents=True, exist_ok=True)
    active_tools = selected_tools(tools)
    handoff_path = output_dir / "tool_handoff.md"
    lines = [
        "# Identification Tool Handoff",
        "",
        f"- dataset_dir: `{dataset_dir}`",
        f"- processed_csv: `{processed_csv}`",
        f"- urdf_path: `{urdf_path or 'not provided'}`",
        "",
        "## Data Contract",
        "",
        "- `q_* / dq_* / tau_meas_*`: 原始实测关节位置、速度和后端力矩反馈。",
        "- `q_proc_* / dq_proc_* / ddq_proc_* / tau_proc_*`: 同一条后处理链生成的默认辨识输入。",
        "- `ddq_proc_*`: 由平滑后的 `q_proc_*` 再经中心差分得到，不再和原始 `q_* / dq_*` 混用。",
        "- `tau_proc_*`: 对 `tau_meas_*` 做同窗口平滑后的结果，用于和 `q_proc_* / dq_proc_* / ddq_proc_*` 保持时间对齐策略一致。",
        "- `q_cmd_* / dq_cmd_* / ddq_cmd_*`: commanded excitation trajectory",
        "",
        "## Recommended Offline Tuple",
        "",
        "- 默认使用 `q_proc_* / dq_proc_* / ddq_proc_* / tau_proc_*` 进入回归矩阵与最小二乘流程。",
        "- 如果要评估电流换算力矩未经平滑时的影响，可在离线脚本里额外对比 `tau_meas_*`。",
        "",
        "## Tools",
        "",
    ]
    for tool in active_tools:
        installed = "yes" if tool.installed else "no"
        lines.extend(
            [
                f"### {tool.label}",
                "",
                f"- installed_python_module: `{installed}`",
                f"- purpose: {tool.purpose}",
                f"- handoff: {tool.handoff}",
                "",
            ]
        )
    lines.extend(
        [
        "## Identification Equation",
        "",
        "The offline target is `tau = Y(q, dq, ddq) * pi`.",
        "Use external tooling for regressor generation, base-parameter extraction, filtering strategy, and OLS/WLS solving.",
        "If the trajectory was created with `--optimize`, its current score is based on a surrogate feature matrix.",
        "Replace the surrogate score with a true regressor condition number once Pinocchio or URDFly regressor generation is wired in.",
        "",
    ]
    )
    handoff_path.write_text("\n".join(lines), encoding="utf-8")
    _write_pinocchio_skeleton(output_dir)
    return handoff_path


def _write_pinocchio_skeleton(output_dir: Path) -> None:
    # 这个脚本是交接骨架，不在单测中执行。
    # 默认使用统一 processed 四元组，避免把 raw q/dq 和 proc ddq/tau 混在一起。
    # 如果后续要比较 raw tau 与 filtered tau 的辨识残差，可只替换 tau 读取列。
    skeleton = '''"""Pinocchio regressor handoff skeleton.

Run after installing Pinocchio in a dedicated offline environment.
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
import pinocchio as pin


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as file:
        return list(csv.DictReader(file))


def main() -> None:
    dataset_dir = Path(__file__).resolve().parent
    processed_csv = dataset_dir / "processed_samples.csv"
    urdf_path = Path("configs/models/X5_camera.urdf")
    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    rows = load_rows(processed_csv)
    regressors = []
    torques = []
    for row in rows:
        # 统一使用同一条后处理链生成的 q/dq/ddq/tau。
        # 这样更容易保证求导、平滑和力矩序列在时间上采用一致策略。
        q = np.array([float(row[f"q_proc_{index}"]) for index in range(1, model.nv + 1)])
        dq = np.array([float(row[f"dq_proc_{index}"]) for index in range(1, model.nv + 1)])
        ddq = np.array([float(row[f"ddq_proc_{index}"]) for index in range(1, model.nv + 1)])
        tau = np.array([float(row[f"tau_proc_{index}"]) for index in range(1, model.nv + 1)])
        regressors.append(pin.computeJointTorqueRegressor(model, data, q, dq, ddq))
        torques.append(tau)
    y_matrix = np.vstack(regressors)
    tau_vector = np.concatenate(torques)
    print("Y:", y_matrix.shape, "tau:", tau_vector.shape)


if __name__ == "__main__":
    main()
'''
    (output_dir / "pinocchio_regressor_skeleton.py").write_text(skeleton, encoding="utf-8")
