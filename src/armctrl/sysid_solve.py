from __future__ import annotations

from dataclasses import dataclass
import csv
import importlib.util
import json
from pathlib import Path


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
        input_quality_status = str(quality_metrics["data_health"]["status"])
        backend_status = {
            "pinocchio": _module_status("pinocchio"),
            "figaroh": _module_status("figaroh"),
        }
        residual_summary = _fake_zero_tau_residual_summary(processed_rows)
        overall_verdict = (
            "solver_ready"
            if input_quality_status == "pass"
            and backend_status["pinocchio"]["status"] == "available"
            else "solver_handoff_only"
        )

        metrics = {
            "schema": "armctrl.sysid_solver_quality.v1",
            "sample_count": len(processed_rows),
            "input_quality_status": input_quality_status,
            "overall_verdict": overall_verdict,
            "backend_status": backend_status,
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
    spec = importlib.util.find_spec(module_name)
    if spec is None:
        return {"status": "missing", "module": module_name}
    return {"status": "available", "module": module_name}


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
    residual_summary = metrics["residual_summary"]
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
        "- residual_summary: "
        f"`fake_zero_tau_rmse_nm={residual_summary['fake_zero_tau_rmse_nm']}`\n\n"
        f"{note}\n"
    )
