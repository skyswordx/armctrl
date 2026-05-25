"""External tool handoff helpers."""

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
        purpose="Build rigid-body dynamics models and compute torque regressors.",
        handoff="Prefer processed_samples.csv with q_proc/dq_proc/ddq_proc/tau_proc; compare tau_meas only when needed.",
    ),
    "figaroh": ExternalToolProfile(
        name="figaroh",
        label="FIGAROH",
        python_module="figaroh",
        purpose="Do dynamics identification, trajectory optimization, and physical-consistency projection.",
        handoff="Map LeRobot-shaped manifest, URDF, and processed_samples.csv into FIGAROH identification config, then let the external tool compute base parameters, regressors, and consistency projection.",
    ),
    "flobaroid": ExternalToolProfile(
        name="flobaroid",
        label="FloBaRoID",
        python_module=None,
        purpose="URDF-driven parameter identification and OLS/WLS solver flow.",
        handoff="Convert processed_samples.csv into the table shape FloBaRoID expects and run its optimization/export pipeline.",
    ),
    "urdfly": ExternalToolProfile(
        name="urdfly",
        label="URDFly",
        python_module="urdfly",
        purpose="Generate symbolic dynamics regressor code from URDF.",
        handoff="Generate regressor code with URDFly, then assemble Y(pi)=tau from processed_samples.csv columns.",
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
        "- `q_* / dq_* / tau_meas_*`: raw measured joint state and torque feedback.",
        "- `q_proc_* / dq_proc_* / ddq_proc_* / tau_proc_*`: default offline identification input from one postprocess chain.",
        "- `lerobot_contract`: LeRobot-style observation/action feature contract and column map.",
        "- `ddq_proc_*`: centered differences of smoothed `q_proc_*`, not mixed with raw `q_* / dq_*`.",
        "- `tau_proc_*`: smoothed `tau_meas_*` aligned to the same time base as the processed kinematics.",
        "- `q_cmd_* / dq_cmd_* / ddq_cmd_*`: commanded excitation trajectory.",
        "",
        "## Recommended Offline Tuple",
        "",
        "- Use `q_proc_* / dq_proc_* / ddq_proc_* / tau_proc_*` as the default least-squares input.",
        "- `lerobot_contract` only unifies the interface semantics; it does not replace FIGAROH's offline math core.",
        "- If you want to study unsmoothed current-based torque estimates, compare against `tau_meas_*` in a separate offline script.",
        "- The fixed postprocess solver stage writes `solver_metrics.json`, `solver_report_zh.md`, and `run_solver_stage.py` after these handoff inputs are ready.",
        "- Pinocchio is used as the deterministic built-in regressor path when it is importable and a URDF is provided.",
        "- FIGAROH remains the preferred mature identification stack, but it needs a robot-specific configuration mapping before the automatic postprocess stage can safely invoke it end to end.",
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
            "The postprocess solver report now evaluates this equation through the fixed Pinocchio path when available.",
            "Use FIGAROH or other external tooling for base-parameter extraction, physical-consistency projection, filtering strategy, and OLS/WLS variants once their robot-specific configuration is ready.",
            "If the trajectory was created with `--optimize`, its current score is based on a surrogate feature matrix.",
            "Replace the surrogate score with a true regressor condition number from Pinocchio/FIGAROH before trusting full dynamic identification.",
            "",
        ]
    )
    handoff_path.write_text("\n".join(lines), encoding="utf-8")
    _write_pinocchio_skeleton(output_dir)
    return handoff_path


def _write_pinocchio_skeleton(output_dir: Path) -> None:
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
