from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


SCHEMA = "armctrl.sdk_cartesian_helper_plan.v1"
RUNNER_SCHEMA = "armctrl.eef_runner_contract.v1"


def _emit(payload: dict[str, object], *, as_json: bool) -> int:
    if as_json:
        sys.stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
    else:
        sys.stdout.write(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
    return 0


def _build_payload(runner_contract: dict[str, object], runner_contract_path: Path) -> dict[str, object]:
    if runner_contract.get("schema") != RUNNER_SCHEMA:
        raise ValueError(f"runner contract file must use schema {RUNNER_SCHEMA}")
    resolved_backend = runner_contract.get("resolved_backend")
    if resolved_backend != "sdk_cartesian":
        raise RuntimeError(f"unsupported runner backend for sdk helper sample: {resolved_backend}")
    bridge_export = dict(runner_contract["bridge_export"])
    sdk_controller = dict(bridge_export["sdk_controller"])
    sdk_request = dict(bridge_export["sdk_request"])
    return {
        "schema": SCHEMA,
        "movement_allowed": False,
        "runner_contract_path": str(runner_contract_path),
        "plan_dir": str(runner_contract["plan_dir"]),
        "resolved_backend": resolved_backend,
        "runtime_owner": runner_contract["runner_api"]["owner"],
        "sdk_controller": sdk_controller,
        "sdk_session_plan": {
            "controller_class": sdk_controller["controller_class"],
            "command_object": sdk_controller["command_object"],
            "command_mode": sdk_controller["command_mode"],
            "submit_call": sdk_controller["submit_call"],
            "robot": sdk_request["robot"],
            "control_frame": sdk_request["control_frame"],
            "waypoint_count": sdk_request["waypoint_count"],
            "eef_waypoints": sdk_request["eef_waypoints"],
        },
        "review_output_contract": runner_contract["review_output_contract"],
        "reference_examples": runner_contract["runner_api"]["reference_examples"],
        "next_steps": [
            f"uv run armctrl eef sample-runner --runner-contract {runner_contract_path} --json",
            runner_contract["review_output_contract"]["review_command"],
        ],
        "notes": [
            "This script is a non-hardware sample of how an external sdk_cartesian helper can consume armctrl.eef_runner_contract.v1.",
            "It does not import or instantiate arx5_interface, and it does not move hardware.",
            "Use the generated session plan as the boundary artifact for a future mature SDK helper implementation.",
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner-contract", required=True)
    parser.add_argument("--output")
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args(argv)

    runner_contract_path = Path(args.runner_contract)
    try:
        runner_contract = json.loads(runner_contract_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        payload = {
            "status": "rejected",
            "schema": SCHEMA,
            "error": {"code": "missing_runner_contract", "message": str(error)},
            "movement_allowed": False,
            "runner_contract_path": str(runner_contract_path),
            "next_gate": "export a valid armctrl.eef_runner_contract.v1 artifact before using this helper sample",
        }
        return _emit(payload, as_json=args.as_json)
    except json.JSONDecodeError as error:
        payload = {
            "status": "rejected",
            "schema": SCHEMA,
            "error": {"code": "invalid_runner_contract_json", "message": str(error)},
            "movement_allowed": False,
            "runner_contract_path": str(runner_contract_path),
            "next_gate": "export a valid JSON runner contract artifact before using this helper sample",
        }
        return _emit(payload, as_json=args.as_json)

    try:
        payload = _build_payload(runner_contract, runner_contract_path)
    except ValueError as error:
        rejected = {
            "status": "rejected",
            "schema": SCHEMA,
            "error": {"code": "invalid_runner_contract", "message": str(error)},
            "movement_allowed": False,
            "runner_contract_path": str(runner_contract_path),
        }
        _emit(rejected, as_json=args.as_json)
        return 3
    except RuntimeError as error:
        rejected = {
            "status": "rejected",
            "schema": SCHEMA,
            "error": {"code": "unsupported_runner_backend", "message": str(error)},
            "movement_allowed": False,
            "runner_contract_path": str(runner_contract_path),
            "resolved_backend": runner_contract.get("resolved_backend"),
            "next_gate": "use this helper sample only with sdk_cartesian runner contracts",
        }
        _emit(rejected, as_json=args.as_json)
        return 3

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        payload["output"] = str(output_path)
    payload = {"status": "ok", **payload}
    return _emit(payload, as_json=args.as_json)


if __name__ == "__main__":
    raise SystemExit(main())
