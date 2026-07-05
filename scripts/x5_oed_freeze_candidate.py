from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any, Sequence


def freeze_candidate(*, attempt_dir: Path, output_dir: Path) -> dict[str, Any]:
    attempt_dir = attempt_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    planned_path = attempt_dir / "planned_trajectory.csv"
    execution_path = attempt_dir / "execution_trajectory.csv"
    manifest_path = attempt_dir / "manifest.json"
    if not planned_path.exists():
        raise FileNotFoundError(f"missing planned trajectory: {planned_path}")
    if not manifest_path.exists():
        raise FileNotFoundError(f"missing attempt manifest: {manifest_path}")

    source_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    recommended_path = output_dir / "recommended_candidate.csv"
    shutil.copyfile(planned_path, recommended_path)

    source_artifacts = source_manifest.get("artifacts", {})
    backend = source_manifest.get("trajectory_backend", {})
    base_score = backend.get("base_regressor_score") or {}
    regressor_score = backend.get("regressor_score") or {}
    base_condition_number = (
        base_score.get("condition_number")
        or base_score.get("effective_condition_number")
    )
    pinocchio_effective_condition_number = regressor_score.get(
        "effective_condition_number"
    )
    target_condition_metric = (
        "figaroh_base_regressor"
        if base_condition_number is not None
        else "pinocchio_effective_regressor"
    )
    condition_number = (
        base_condition_number
        if base_condition_number is not None
        else pinocchio_effective_condition_number or backend.get("condition_number")
    )
    target_condition_number = 100.0
    target_condition_status, target_condition_margin = _target_condition_assessment(
        condition_number,
        target_condition_number=target_condition_number,
    )
    rank = backend.get("rank") or regressor_score.get("rank") or base_score.get("rank")
    oed_quality_status = (backend.get("oed_quality_gate") or {}).get("status")
    safety_allowed = (source_manifest.get("safety") or {}).get("allowed")
    source_execution = _resolve_artifact_path(
        source_artifacts.get("execution_trajectory"),
        attempt_dir=attempt_dir,
        default_path=execution_path,
    )

    frozen_manifest = {
        "schema": "armctrl.x5_oed_frozen_candidate.v1",
        "source_attempt": str(attempt_dir),
        "source_manifest": str(manifest_path),
        "source_planned_trajectory": str(planned_path),
        "source_execution_trajectory": str(source_execution),
        "recommended_candidate": str(recommended_path),
        "condition_number": condition_number,
        "base_regressor_condition_number": base_condition_number,
        "pinocchio_effective_condition_number": pinocchio_effective_condition_number,
        "rank": rank,
        "oed_quality_status": oed_quality_status,
        "safety_allowed": safety_allowed,
        "target_condition_number": target_condition_number,
        "target_condition_metric": target_condition_metric,
        "target_condition_status": target_condition_status,
        "target_condition_margin": target_condition_margin,
        "next_gate": _next_gate(
            target_condition_status=target_condition_status,
            oed_quality_status=oed_quality_status,
            safety_allowed=safety_allowed,
        ),
        "replay_hint": {
            "command": "armctrl sysid plan fourier_multisine",
            "candidate_trajectory": str(recommended_path),
        },
    }
    manifest_output_path = output_dir / "best_candidate_manifest.json"
    manifest_output_path.write_text(
        json.dumps(frozen_manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return {
        "schema": "armctrl.x5_oed_frozen_candidate.v1",
        "status": "ok",
        "artifacts": {
            "recommended_candidate": str(recommended_path),
            "manifest": str(manifest_output_path),
        },
        **frozen_manifest,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="x5_oed_freeze_candidate")
    parser.add_argument("--attempt", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    result = freeze_candidate(
        attempt_dir=Path(args.attempt),
        output_dir=Path(args.output),
    )
    if args.json:
        print(json.dumps(result, ensure_ascii=False))
    else:
        print(f"frozen candidate: {result['artifacts']['recommended_candidate']}")
        print(f"manifest: {result['artifacts']['manifest']}")
    return 0


def _resolve_artifact_path(
    raw_path: object,
    *,
    attempt_dir: Path,
    default_path: Path,
) -> Path:
    if raw_path is None:
        return default_path
    path = Path(str(raw_path))
    if path.is_absolute():
        return path
    candidates = [
        Path.cwd() / path,
        attempt_dir / path,
        attempt_dir.parent / path,
        attempt_dir.parent.parent / path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    return (Path.cwd() / path).resolve()


def _target_condition_assessment(
    condition_number: object,
    *,
    target_condition_number: float,
) -> tuple[str, float | None]:
    if not isinstance(condition_number, int | float):
        return "not_evaluated", None
    margin = float(condition_number) - float(target_condition_number)
    return ("pass" if margin <= 0.0 else "fail"), margin


def _next_gate(
    *,
    target_condition_status: str,
    oed_quality_status: object,
    safety_allowed: object,
) -> str:
    if target_condition_status != "pass":
        return "continue_structural_oed_search"
    if oed_quality_status != "pass":
        return "review_optimizer_convergence_offline"
    if safety_allowed is not True:
        return "repair_safety_gate_before_hardware_smoke"
    return "ready_for_hardware_smoke"


if __name__ == "__main__":
    raise SystemExit(main())
