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
