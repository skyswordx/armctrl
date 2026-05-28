from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path


@dataclass(frozen=True)
class SysIdEvidenceImportResult:
    schema: str
    evidence: dict[str, object]
    artifacts: dict[str, str]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "evidence": self.evidence,
            "artifacts": self.artifacts,
        }


class SysIdEvidenceImporter:
    def run(self, dataset_dir: Path, evidence_path: Path) -> SysIdEvidenceImportResult:
        processed_dir = dataset_dir / "processed"
        solver_metrics_path = processed_dir / "solver_metrics.json"
        metrics = json.loads(solver_metrics_path.read_text(encoding="utf-8"))
        evidence = json.loads(evidence_path.read_text(encoding="utf-8-sig"))
        _validate_evidence(evidence)

        metrics["external_evidence"] = {
            "schema": evidence["schema"],
            "source": evidence["source"],
            "path": str(evidence_path),
        }
        metrics["physical_consistency"] = evidence["physical_consistency"]
        metrics["figaroh_base_parameters"] = evidence["figaroh_base_parameters"]
        solver_metrics_path.write_text(
            json.dumps(metrics, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        return SysIdEvidenceImportResult(
            schema="armctrl.sysid_external_evidence.v1",
            evidence={
                "source": evidence["source"],
                "physical_consistency": evidence["physical_consistency"],
                "figaroh_base_parameters": evidence["figaroh_base_parameters"],
            },
            artifacts={
                "solver_metrics": str(solver_metrics_path),
                "external_evidence": str(evidence_path),
            },
        )


def _validate_evidence(evidence: dict[str, object]) -> None:
    if evidence.get("schema") != "armctrl.external_solver_evidence.v1":
        raise ValueError("unsupported external evidence schema")
    if evidence.get("source") not in {"figaroh", "pinocchio", "manual-review"}:
        raise ValueError("unsupported external evidence source")
    for key in ("physical_consistency", "figaroh_base_parameters"):
        if key not in evidence:
            raise ValueError(f"external evidence missing {key}")
