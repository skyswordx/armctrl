from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from armctrl.sysid_oed_scan import OedScanRequest, OedScanRunner


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="x5_oed_scan")
    parser.add_argument("--profile", default="fourier_multisine")
    parser.add_argument("--dof", type=int, default=6)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--q-center", nargs="+", type=float, default=[0, 0.3, 0.3, 0, 0, 0])
    parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", nargs="+", type=float, required=True)
    parser.add_argument("--amplitude", nargs="+", type=float, required=True)
    parser.add_argument("--n-wps", nargs="+", type=int, default=[5])
    parser.add_argument("--stack-reps", nargs="+", type=int, default=[1])
    parser.add_argument("--ipopt-max-iterations", type=int, default=200)
    parser.add_argument("--condition-number-threshold", type=float, default=1000.0)
    parser.add_argument("--trajectory-command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    q_center = tuple(float(value) for value in args.q_center)
    if len(q_center) != args.dof:
        parser.error("--q-center length must match --dof")
    result = OedScanRunner().run(
        OedScanRequest(
            profile_name=args.profile,
            dof=args.dof,
            sample_hz=args.sample_hz,
            q_center=q_center,
            urdf_path=args.urdf_path,
            safe_config_path=args.safe_config,
            output_dir=Path(args.output),
            durations_s=tuple(args.duration),
            amplitudes_rad=tuple(args.amplitude),
            n_wps_values=tuple(args.n_wps),
            stack_reps_values=tuple(args.stack_reps),
            ipopt_max_iterations=args.ipopt_max_iterations,
            condition_number_threshold=args.condition_number_threshold,
            trajectory_command_argv=(
                tuple(args.trajectory_command)
                if args.trajectory_command is not None
                else None
            ),
        )
    )
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
