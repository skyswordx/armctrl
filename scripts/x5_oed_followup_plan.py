from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


def build_followup_plan(
    *,
    manifest_path: Path,
    output_dir: Path,
    seeds: Sequence[int] = (2, 3, 4),
    amplitudes: Sequence[float] = (0.45, 0.50, 0.55),
) -> dict[str, Any]:
    manifest_path = manifest_path.resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    output_dir.mkdir(parents=True, exist_ok=True)
    target_condition_metric = manifest.get(
        "target_condition_metric",
        "figaroh_base_regressor",
    )
    baseline_condition_number = (
        manifest.get("base_regressor_condition_number")
        if target_condition_metric == "figaroh_base_regressor"
        and manifest.get("base_regressor_condition_number") is not None
        else manifest.get("condition_number")
    )
    candidate_path = _resolve_manifest_path(
        manifest.get("recommended_candidate"),
        manifest_path=manifest_path,
    )
    replay_output = output_dir / "replay-plan"
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
    structural_combos = [
        ("A_duration2_nwps7_stack1", "A", 2.0, 7, 1),
        ("B_duration2_nwps9_stack1", "B", 2.0, 9, 1),
        ("C_duration3_nwps9_stack1", "C", 3.0, 9, 1),
        ("D_duration2_nwps7_stack2", "D", 2.0, 7, 2),
    ]
    structural_scans = {
        key: _structural_scan_command(
            output_dir=output_dir / "structural-scans" / key,
            duration_s=duration_s,
            n_wps=n_wps,
            stack_reps=stack_reps,
            amplitude_args=amplitude_args,
            seed_args=seed_args,
        )
        for key, _name, duration_s, n_wps, stack_reps in structural_combos
    }
    result = {
        "schema": "armctrl.x5_oed_followup_plan.v1",
        "baseline": {
            "manifest": str(manifest_path),
            "candidate": str(candidate_path),
            "condition_number": baseline_condition_number,
            "target_condition_metric": target_condition_metric,
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
                _target_condition_status(baseline_condition_number),
            ),
            "target_condition_margin": manifest.get(
                "target_condition_margin",
                _target_condition_margin(baseline_condition_number),
            ),
            "next_gate": manifest.get(
                "next_gate",
                _next_gate_from_condition(baseline_condition_number),
            ),
        },
        "scan_strategy": {
            "mode": "structural_parameterization_search",
            "rejected_mode": "seed_or_iteration_only_lottery",
            "combos": [
                {
                    "name": name,
                    "duration_s": duration_s,
                    "n_wps": n_wps,
                    "stack_reps": stack_reps,
                }
                for _key, name, duration_s, n_wps, stack_reps in structural_combos
            ],
            "amplitude_rad": [float(value) for value in amplitudes],
            "seed": [int(value) for value in seeds],
            "target_acceleration_rad_s2": 20.0,
        },
        "warm_start_status": "not_supported_by_current_figaroh_wrapper",
        "warm_start_note": (
            "The current X5 FIGAROH wrapper does not expose initial waypoints "
            "from a frozen CSV; use replay plus structural parameterization "
            "scans until that mature-backend hook is added."
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
                    "FIGAROH/IPOPT structural scans are memory-heavy and should "
                    "run on the local WSL/workstation environment where previous "
                    "OED searches were computed."
                ),
            },
            "n100d_role": "lightweight_replay_hardware_collection_postprocess_solver",
        },
        "commands": {
            "replay_plan": replay_plan,
            "structural_scans": structural_scans,
        },
        "acceptance": {
            "target_condition_number": 100.0,
            "target_condition_metric": "figaroh_base_regressor",
            "target_acceleration_rad_s2": 20.0,
            "required_rank": 36,
            "required_safety_allowed": True,
            "freeze_next_best_with": (
                "uv run python scripts/x5_oed_freeze_candidate.py "
                "--attempt <structural-scans/<combo>/attempt-NNN> "
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
    parser.add_argument("--seed", nargs="+", type=int, default=[2, 3, 4])
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
        for command in result["commands"]["structural_scans"].values():
            print(command)
    return 0


def _structural_scan_command(
    *,
    output_dir: Path,
    duration_s: float,
    n_wps: int,
    stack_reps: int,
    amplitude_args: str,
    seed_args: str,
) -> str:
    return (
        "uv run python scripts/x5_oed_scan.py "
        f"--output {output_dir} "
        f"--duration {_format_float(duration_s)} "
        f"--amplitude {amplitude_args} "
        f"--n-wps {n_wps} "
        f"--stack-reps {stack_reps} "
        f"--seed {seed_args} "
        "--sample-hz 20 "
        "--condition-number-threshold 500 "
        "--ipopt-max-iterations 300 "
        "--ipopt-print-level 5 "
        "--attempt-timeout-s 300 "
        "--trajectory-command uv run python scripts/x5_figaroh_oed.py"
    )


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
        else "continue_structural_oed_search"
    )


if __name__ == "__main__":
    raise SystemExit(main())
