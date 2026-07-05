from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path


@dataclass(frozen=True)
class FigarohEvidenceAdapterResult:
    schema: str
    artifacts: dict[str, str]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "artifacts": self.artifacts,
        }


@dataclass(frozen=True)
class FigarohHandoffResult:
    schema: str
    artifacts: dict[str, str]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "artifacts": self.artifacts,
        }


class FigarohEvidenceAdapter:
    def run(self, input_path: Path, output_path: Path) -> FigarohEvidenceAdapterResult:
        figaroh_report = json.loads(input_path.read_text(encoding="utf-8-sig"))
        evidence = _adapt_figaroh_report(figaroh_report)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(evidence, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return FigarohEvidenceAdapterResult(
            schema="armctrl.figarohevidence_adapter.v1",
            artifacts={
                "figaroh_report": str(input_path),
                "external_evidence": str(output_path),
            },
        )


class FigarohHandoffWriter:
    def run(self, dataset_dir: Path, output_dir: Path) -> FigarohHandoffResult:
        processed_dir = dataset_dir / "processed"
        run_manifest = json.loads((dataset_dir / "manifest.json").read_text(encoding="utf-8"))
        request = run_manifest.get("request", {})
        output_dir.mkdir(parents=True, exist_ok=True)
        handoff_path = output_dir / "figaroh_handoff.json"
        figaroh_report_path = output_dir / "figaroh-report.json"
        armctrl_evidence_path = output_dir / "armctrl-evidence.json"
        handoff = {
            "schema": "armctrl.figaroh_handoff.v1",
            "dataset": {
                "root": str(dataset_dir),
                "processed_samples": str(processed_dir / "processed_samples.csv"),
                "quality_metrics": str(processed_dir / "quality_metrics.json"),
                "solver_metrics": str(processed_dir / "solver_metrics.json"),
            },
            "model": {
                "urdf_path": request.get("urdf_path"),
                "dof": request.get("dof"),
            },
            "task": {
                "source_tool": "figaroh",
                "required_outputs": ["base_parameters", "physical_consistency"],
                "armctrl_boundary": "armctrl prepares data and imports evidence; FIGAROH owns identification math",
            },
            "expected_outputs": {
                "figaroh_report": str(figaroh_report_path),
                "report_schema": "figaroh.identification.report.v1",
                "armctrl_evidence": str(armctrl_evidence_path),
            },
            "armctrl_import_command": [
                "armctrl",
                "sysid",
                "adapt-figaroh-evidence",
                "--input",
                str(figaroh_report_path),
                "--output",
                str(armctrl_evidence_path),
                "--json",
            ],
        }
        handoff_path.write_text(
            json.dumps(handoff, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return FigarohHandoffResult(
            schema="armctrl.figaroh_handoff.v1",
            artifacts={"figaroh_handoff": str(handoff_path)},
        )


def _adapt_figaroh_report(report: dict[str, object]) -> dict[str, object]:
    base_parameters = report.get("base_parameters", {})
    physical_consistency = report.get("physical_consistency", {})
    if not isinstance(base_parameters, dict):
        raise ValueError("figaroh report base_parameters must be an object")
    if not isinstance(physical_consistency, dict):
        raise ValueError("figaroh report physical_consistency must be an object")
    return {
        "schema": "armctrl.external_solver_evidence.v1",
        "source": "figaroh",
        "physical_consistency": physical_consistency,
        "figaroh_base_parameters": {
            "status": base_parameters.get("status", "missing"),
            "parameter_count": base_parameters.get("parameter_count"),
            "names": base_parameters.get("names", []),
            "basis": base_parameters.get("basis", "figaroh"),
        },
        "source_schema": report.get("schema", "unknown"),
    }
