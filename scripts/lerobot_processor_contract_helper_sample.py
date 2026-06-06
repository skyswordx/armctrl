from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from armctrl.lerobot_bridge import (
    LeRobotRolloutPreviewRequest,
    LeRobotRolloutPreviewer,
)


SCHEMA = "armctrl.lerobot_processor_helper_preview.v1"
CONTRACT_SCHEMA = "armctrl.lerobot_eef_processor_contract.v1"


def _emit(payload: dict[str, object], *, as_json: bool) -> int:
    if as_json:
        sys.stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
    else:
        sys.stdout.write(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
    return 0


def _build_preview(
    processor_contract: dict[str, object],
    processor_contract_path: Path,
    *,
    urdf_path: Path,
    safe_config_path: Path,
    render_path: Path | None,
) -> dict[str, object]:
    if processor_contract.get("schema") != CONTRACT_SCHEMA:
        raise ValueError(
            f"processor contract file must use schema {CONTRACT_SCHEMA}"
        )
    runner_contract = dict(processor_contract["runner_contract"])
    resolved_backend = runner_contract.get("resolved_backend")
    if resolved_backend != "lerobot_rollout":
        raise RuntimeError(
            "unsupported processor runner backend for lerobot helper sample: "
            f"{resolved_backend}"
        )
    eef_plan_dir = Path(str(processor_contract["eef_plan_dir"]))
    preview = LeRobotRolloutPreviewer().preview(
        LeRobotRolloutPreviewRequest(
            eef_plan_dir=eef_plan_dir,
            urdf_path=urdf_path,
            safe_config_path=safe_config_path,
            render_path=render_path,
        )
    )
    return {
        "schema": SCHEMA,
        "movement_allowed": False,
        "processor_contract_path": str(processor_contract_path),
        "consumed_processor_contract": processor_contract,
        "eef_plan_dir": str(eef_plan_dir),
        "resolved_backend": resolved_backend,
        "review_status": preview["review_status"],
        "sim_preview": preview.get("sim_preview"),
        "synthesized_trajectory": preview["synthesized_trajectory"],
        "delegated_preview": preview,
        "ordered_steps": [
            {
                "id": "preview_rollout",
                "command": (
                    "uv run armctrl lerobot preview-rollout "
                    f"--eef-plan-dir {eef_plan_dir} --urdf-path {urdf_path} "
                    f"--safe-config {safe_config_path} --json"
                ),
            },
            {
                "id": "review_rollout",
                "depends_on": ["preview_rollout"],
                "command": (
                    "uv run armctrl lerobot review-rollout "
                    f"--eef-plan-dir {eef_plan_dir} --json"
                ),
            },
        ],
        "next_steps": preview["next_steps"],
        "notes": [
            "This script is a non-hardware sample of how an external LeRobot helper can consume armctrl.lerobot_eef_processor_contract.v1.",
            "It reuses the shared rollout preview/review chain instead of starting LeRobot execution.",
            "Use ordered_steps as the machine-readable sequence boundary so preview stays ahead of review.",
            "Use this as a boundary sample before wiring a mature rollout-side helper that emits reviewed joint trajectories from real policy/runtime output.",
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--processor-contract", required=True)
    parser.add_argument("--urdf-path", required=True)
    parser.add_argument("--safe-config", required=True)
    parser.add_argument("--render")
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args(argv)

    processor_contract_path = Path(args.processor_contract)
    try:
        processor_contract = json.loads(
            processor_contract_path.read_text(encoding="utf-8")
        )
    except FileNotFoundError as error:
        payload = {
            "status": "rejected",
            "schema": SCHEMA,
            "error": {"code": "missing_processor_contract", "message": str(error)},
            "movement_allowed": False,
            "processor_contract_path": str(processor_contract_path),
            "next_gate": (
                "export a valid armctrl.lerobot_eef_processor_contract.v1 artifact "
                "before using this helper sample"
            ),
        }
        return _emit(payload, as_json=args.as_json)
    except json.JSONDecodeError as error:
        payload = {
            "status": "rejected",
            "schema": SCHEMA,
            "error": {
                "code": "invalid_processor_contract_json",
                "message": str(error),
            },
            "movement_allowed": False,
            "processor_contract_path": str(processor_contract_path),
            "next_gate": "export a valid JSON processor contract before using this helper sample",
        }
        return _emit(payload, as_json=args.as_json)

    try:
        payload = _build_preview(
            processor_contract,
            processor_contract_path,
            urdf_path=Path(args.urdf_path),
            safe_config_path=Path(args.safe_config),
            render_path=Path(args.render) if args.render else None,
        )
    except ValueError as error:
        rejected = {
            "status": "rejected",
            "schema": SCHEMA,
            "error": {"code": "invalid_processor_contract", "message": str(error)},
            "movement_allowed": False,
            "processor_contract_path": str(processor_contract_path),
        }
        _emit(rejected, as_json=args.as_json)
        return 3
    except RuntimeError as error:
        rejected = {
            "status": "rejected",
            "schema": SCHEMA,
            "error": {
                "code": "unsupported_processor_backend",
                "message": str(error),
            },
            "movement_allowed": False,
            "processor_contract_path": str(processor_contract_path),
            "resolved_backend": processor_contract.get("runner_contract", {}).get(
                "resolved_backend"
            ),
            "next_gate": "use this helper sample only with lerobot_rollout processor contracts",
        }
        _emit(rejected, as_json=args.as_json)
        return 3

    payload = {"status": "ok", **payload}
    return _emit(payload, as_json=args.as_json)


if __name__ == "__main__":
    raise SystemExit(main())
