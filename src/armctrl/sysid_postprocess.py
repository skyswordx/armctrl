from __future__ import annotations

from dataclasses import dataclass
import csv
import json
from pathlib import Path


@dataclass(frozen=True)
class SysIdPostprocessResult:
    schema: str
    sample_count: int
    artifacts: dict[str, str]
    solver: dict[str, object] | None = None

    def to_json(self) -> dict[str, object]:
        payload: dict[str, object] = {
            "schema": self.schema,
            "sample_count": self.sample_count,
            "artifacts": self.artifacts,
        }
        if self.solver is not None:
            payload["solver"] = self.solver
        return payload


class SysIdPostprocessor:
    def run(self, dataset_dir: Path) -> SysIdPostprocessResult:
        raw_samples_path = dataset_dir / "raw_samples.csv"
        manifest_path = dataset_dir / "manifest.json"
        processed_dir = dataset_dir / "processed"
        processed_dir.mkdir(parents=True, exist_ok=True)

        raw_rows = _read_csv(raw_samples_path)
        processed_rows = [_processed_row(row) for row in raw_rows]

        processed_samples_path = processed_dir / "processed_samples.csv"
        _write_csv(processed_samples_path, processed_rows)

        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        metrics = {
            "schema": "armctrl.sysid_quality.v1",
            "sample_count": len(processed_rows),
            "data_health": {
                "status": "pass" if processed_rows else "fail",
                "raw_columns": list(raw_rows[0]) if raw_rows else [],
            },
            "handoff": manifest["handoff"],
        }
        quality_metrics_path = processed_dir / "quality_metrics.json"
        quality_metrics_path.write_text(
            json.dumps(metrics, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        quality_report_path = processed_dir / "quality_report.md"
        quality_report_path.write_text(
            _quality_report(metrics),
            encoding="utf-8",
        )

        return SysIdPostprocessResult(
            schema="armctrl.sysid_postprocess.v1",
            sample_count=len(processed_rows),
            artifacts={
                "processed_samples": str(processed_samples_path),
                "quality_metrics": str(quality_metrics_path),
                "quality_report": str(quality_report_path),
            },
        )


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _processed_row(row: dict[str, str]) -> dict[str, str]:
    processed = {"time_s": row["time_s"]}
    for key, value in row.items():
        if key.startswith("q_") and key[2:].isdigit():
            processed[f"q_proc_{key[2:]}"] = value
        elif key.startswith("dq_") and key[3:].isdigit():
            processed[f"dq_proc_{key[3:]}"] = value
        elif key.startswith("tau_meas_"):
            processed[f"tau_proc_{key.removeprefix('tau_meas_')}"] = value
        elif key.startswith("q_cmd_"):
            processed[key] = value
    return processed


def _quality_report(metrics: dict[str, object]) -> str:
    return (
        "# SysID Quality Report\n\n"
        f"- schema: `{metrics['schema']}`\n"
        f"- sample_count: `{metrics['sample_count']}`\n"
        f"- data_health: `{metrics['data_health']['status']}`\n"
        "- note: fake runner data validates the offline file contract only.\n"
    )
