from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


def build_followup_plan(
    *,
    manifest_path: Path,
    output_dir: Path,
    seeds: Sequence[int] = (2, 3, 4, 5),
    amplitudes: Sequence[float] = (0.45, 0.50, 0.55),
) -> dict[str, Any]:
    manifest_path = manifest_path.resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    output_dir.mkdir(parents=True, exist_ok=True)
    candidate_path = _resolve_manifest_path(
        manifest.get("recommended_candidate"),
        manifest_path=manifest_path,
    )
    replay_output = output_dir / "replay-plan"
    focused_scan_output = output_dir / "focused-scan"
    seed_args = " ".join(str(seed) for seed in seeds)
    amplitude_args = " ".join(_format_float(value) for value in amplitudes)
    replay_plan = (
        "uv run armctrl sysid plan fourier_multisine "
        f"--candidate-trajectory {candidate_path} "
        "--dof 6 --sample-hz 20 --duration 1 --amplitude 0.5 "
        "--q-center 0 0.3 0.3 0 0 0 "
        "--urdf-path configs/models/X5_camera.urdf "
        "--safe-config configs/x5.safe.yaml "
        f"--output {replay_output} --json"
    )
    focused_scan = (
        "uv run python scripts/x5_oed_scan.py "
        f"--output {focused_scan_output} "
        "--duration 1 "
        f"--amplitude {amplitude_args} "
        "--n-wps 5 "
        "--stack-reps 1 "
        f"--seed {seed_args} "
        "--sample-hz 20 "
        "--condition-number-threshold 500 "
        "--ipopt-max-iterations 300 "
        "--ipopt-print-level 5 "
        "--attempt-timeout-s 300 "
        "--trajectory-command uv run python scripts/x5_figaroh_oed.py"
    )
    result = {
        "schema": "armctrl.x5_oed_followup_plan.v1",
        "baseline": {
            "manifest": str(manifest_path),
            "candidate": str(candidate_path),
            "condition_number": manifest.get("condition_number"),
            "base_regressor_condition_number": manifest.get(
                "base_regressor_condition_number"
            ),
            "pinocchio_effective_condition_number": manifest.get(
                "pinocchio_effective_condition_number"
            ),
            "rank": manifest.get("rank"),
            "safety_allowed": manifest.get("safety_allowed"),
            "target_condition_number": manifest.get(
                "target_condition_number",
                100.0,
            ),
            "target_condition_status": manifest.get(
                "target_condition_status",
                _target_condition_status(manifest.get("condition_number")),
            ),
            "target_condition_margin": manifest.get(
                "target_condition_margin",
                _target_condition_margin(manifest.get("condition_number")),
            ),
            "next_gate": manifest.get(
                "next_gate",
                _next_gate_from_condition(manifest.get("condition_number")),
            ),
        },
        "warm_start_status": "not_supported_by_current_figaroh_wrapper",
        "warm_start_note": (
            "The current X5 FIGAROH wrapper does not expose initial waypoints "
            "from a frozen CSV; use replay plus focused multi-seed scans until "
            "that mature-backend hook is added."
        ),
        "host_contract": {
            "heavy_oed_scan": {
                "allowed_hosts": ["local_wsl", "workstation"],
                "disallowed_hosts": [
                    {
                        "host": "n100d",
                        "reason": "memory_constrained_for_figaroh_ipopt_oed_scan",
                    }
                ],
                "reason": (
                    "FIGAROH/IPOPT focused scans are memory-heavy and should run "
                    "on the local WSL/workstation environment where previous OED "
                    "searches were computed."
                ),
            },
            "n100d_role": "lightweight_replay_hardware_collection_postprocess_solver",
        },
        "commands": {
            "replay_plan": replay_plan,
            "focused_scan": focused_scan,
        },
        "acceptance": {
            "target_condition_number": 100.0,
            "required_rank": 36,
            "required_safety_allowed": True,
            "freeze_next_best_with": (
                "uv run python scripts/x5_oed_freeze_candidate.py "
                "--attempt <focused-scan/attempt-NNN> "
                "--output runs/x5-fourier-best-candidate-<date> --json"
            ),
        },
    }
    (output_dir / "followup_plan.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="x5_oed_followup_plan")
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--seed", nargs="+", type=int, default=[2, 3, 4, 5])
    parser.add_argument("--amplitude", nargs="+", type=float, default=[0.45, 0.50, 0.55])
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    result = build_followup_plan(
        manifest_path=Path(args.manifest),
        output_dir=Path(args.output),
        seeds=tuple(args.seed),
        amplitudes=tuple(args.amplitude),
    )
    if args.json:
        print(json.dumps(result, ensure_ascii=False))
    else:
        print(result["commands"]["replay_plan"])
        print(result["commands"]["focused_scan"])
    return 0


def _resolve_manifest_path(raw_path: object, *, manifest_path: Path) -> Path:
    if raw_path is None:
        raise ValueError("frozen manifest missing recommended_candidate")
    path = Path(str(raw_path))
    if path.is_absolute():
        return path
    candidates = [
        Path.cwd() / path,
        manifest_path.parent / path,
        manifest_path.parent.parent / path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    return (Path.cwd() / path).resolve()


def _format_float(value: float) -> str:
    return f"{float(value):g}"


def _target_condition_status(
    condition_number: object,
    *,
    target_condition_number: float = 100.0,
) -> str:
    if not isinstance(condition_number, int | float):
        return "not_evaluated"
    return "pass" if float(condition_number) <= target_condition_number else "fail"


def _target_condition_margin(
    condition_number: object,
    *,
    target_condition_number: float = 100.0,
) -> float | None:
    if not isinstance(condition_number, int | float):
        return None
    return float(condition_number) - target_condition_number


def _next_gate_from_condition(condition_number: object) -> str:
    return (
        "ready_for_hardware_smoke"
        if _target_condition_status(condition_number) == "pass"
        else "continue_focused_oed_search"
    )


if __name__ == "__main__":
    raise SystemExit(main())
