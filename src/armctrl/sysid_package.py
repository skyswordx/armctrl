from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

from armctrl import __version__


@dataclass(frozen=True)
class SysIdPackageResult:
    schema: str
    status: str
    quality_gate: dict[str, object]
    artifacts: dict[str, str]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "quality_gate": self.quality_gate,
            "artifacts": self.artifacts,
        }


class SysIdPackager:
    def run(self, dataset_dir: Path) -> SysIdPackageResult:
        processed_dir = dataset_dir / "processed"
        solver_metrics_path = processed_dir / "solver_metrics.json"
        package_path = processed_dir / "parameter_package.json"
        solver_metrics = json.loads(solver_metrics_path.read_text(encoding="utf-8-sig"))
        quality_gate = _quality_gate(solver_metrics)
        artifacts = {
            "solver_metrics": str(solver_metrics_path),
            "parameter_package": str(package_path),
        }

        if not quality_gate["allowed"]:
            return SysIdPackageResult(
                schema="armctrl.sysid_parameter_package.v1",
                status="rejected",
                quality_gate=quality_gate,
                artifacts=artifacts,
            )

        package = _candidate_package(
            dataset_dir=dataset_dir,
            solver_metrics_path=solver_metrics_path,
        )
        package_path.write_text(
            json.dumps(package, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return SysIdPackageResult(
            schema="armctrl.sysid_parameter_package.v1",
            status="ok",
            quality_gate=quality_gate,
            artifacts=artifacts,
        )


def _candidate_package(
    *,
    dataset_dir: Path,
    solver_metrics_path: Path,
) -> dict[str, object]:
    signature = hashlib.sha256(solver_metrics_path.read_bytes()).hexdigest()
    return {
            "schema": "armctrl.parameter_package.v1",
            "package_version": __version__,
            "source_dataset": str(dataset_dir),
            "solver_metrics": str(solver_metrics_path),
            "status": "candidate",
            "signature": {
                "algorithm": "sha256",
                "value": signature,
                "covers": ["solver_metrics"],
            },
            "rollback": {
                "target": "previous_active_parameter_package",
                "required_before_activation": True,
            },
            "rollout": {
                "activation_status": "blocked_until_ab_validation",
                "ab_validation": {
                    "status": "required_before_activation",
                    "metrics": [
                        "hold_pose_error",
                        "joint_current_rms",
                        "torque_prediction_residual",
                        "operator_abort_count",
                    ],
                },
            },
        }


def _quality_gate(metrics: dict[str, object]) -> dict[str, object]:
    missing: list[str] = []
    condition = metrics["regressor_condition"]["pinocchio"]
    prediction = metrics["prediction_error"]["pinocchio"]
    physical_consistency = metrics.get("physical_consistency", {})
    figaroh_base_parameters = metrics.get("figaroh_base_parameters", {})

    if condition["status"] != "computed":
        missing.append("pinocchio_regressor_condition")
    if prediction["status"] != "computed":
        missing.append("pinocchio_prediction_error")
    if physical_consistency.get("status") != "pass":
        missing.append("physical_consistency")
    if figaroh_base_parameters.get("status") != "available":
        missing.append("figaroh_base_parameters")

    return {
        "allowed": not missing,
        "missing": missing,
        "required": [
            "pinocchio_regressor_condition",
            "pinocchio_prediction_error",
            "physical_consistency",
            "figaroh_base_parameters",
        ],
        "handoff": {
            "figaroh": "base-parameter extraction and physical consistency remain external-tool responsibilities",
            "pinocchio": "regressor and prediction-error evidence",
        },
    }
