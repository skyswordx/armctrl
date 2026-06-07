from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


SCHEMA = "armctrl.lerobot_agent_runtime_helper_plan.v1"
CONTRACT_SCHEMA = "armctrl.eef_agent_runtime_contract.v1"
NATIVE_LEROBOT_RUNTIME_BOUNDARY = {
    "runtime_owner": "native_lerobot_rollout",
    "armctrl_role": "processor_contract_audit_only",
    "motion_runtime_owner": False,
    "hardware_execution": "outside_armctrl",
}


def _emit(payload: dict[str, object], *, as_json: bool) -> int:
    if as_json:
        sys.stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
    else:
        sys.stdout.write(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
    return 0


def _build_payload(
    agent_runtime_contract: dict[str, object], agent_runtime_contract_path: Path
) -> dict[str, object]:
    if agent_runtime_contract.get("schema") != CONTRACT_SCHEMA:
        raise ValueError(
            f"agent runtime contract file must use schema {CONTRACT_SCHEMA}"
        )
    resolved_backend = agent_runtime_contract.get("resolved_backend")
    if resolved_backend != "lerobot_rollout":
        raise RuntimeError(
            "unsupported agent runtime backend for lerobot helper sample: "
            f"{resolved_backend}"
        )
    backend_session_contract = dict(agent_runtime_contract["backend_session_contract"])
    bridge_export = dict(agent_runtime_contract["bridge_export"])
    lerobot_action = dict(bridge_export["lerobot_action"])
    agent_action_schema = dict(agent_runtime_contract["agent_action_schema"])
    return {
        "schema": SCHEMA,
        "movement_allowed": False,
        "agent_runtime_contract_path": str(agent_runtime_contract_path),
        "plan_dir": str(agent_runtime_contract["plan_dir"]),
        "resolved_backend": resolved_backend,
        "runtime_owner": agent_runtime_contract["runtime_owner"],
        "runtime_boundary": NATIVE_LEROBOT_RUNTIME_BOUNDARY,
        "agent_action_schema": agent_action_schema,
        "processor_session_plan": {
            "session_kind": backend_session_contract["session_kind"],
            "control_mode": lerobot_action["control_mode"],
            "preferred_native_surface": lerobot_action["preferred_native_surface"],
            "feature_order": lerobot_action["feature_order"],
            "action_features": lerobot_action["action_features"],
            "action_hook": backend_session_contract["action_hook"],
            "observation_hook": backend_session_contract["observation_hook"],
            "action_stream_schema": {
                "stream_mode": agent_action_schema["stream_mode"],
                "frame": agent_action_schema["frame"],
                "action_id": agent_action_schema["action_id"],
            },
            "reference_docs": [
                "https://huggingface.co/docs/lerobot/main/inference",
                "https://huggingface.co/docs/lerobot/main/il_robots",
                "https://huggingface.co/docs/lerobot/introduction_processors",
            ],
        },
        "lerobot_action": lerobot_action,
        "review_output_contract": agent_runtime_contract["review_output_contract"],
        "ordered_steps": [
            {
                "id": "export_processor_contract",
                "command": (
                    "uv run armctrl lerobot export-processor-contract "
                    f"--eef-plan-dir {agent_runtime_contract['plan_dir']} --json"
                ),
            },
            {
                "id": "preview_rollout",
                "depends_on": ["export_processor_contract"],
                "command": (
                    "uv run armctrl lerobot preview-rollout "
                    f"--eef-plan-dir {agent_runtime_contract['plan_dir']} --json"
                ),
            },
            {
                "id": "review_rollout",
                "depends_on": ["preview_rollout"],
                "command": (
                    "uv run armctrl lerobot review-rollout "
                    f"--eef-plan-dir {agent_runtime_contract['plan_dir']} --json"
                ),
            },
        ],
        "next_steps": [
            (
                "uv run armctrl lerobot preview-rollout "
                f"--eef-plan-dir {agent_runtime_contract['plan_dir']} --json"
            ),
            (
                "uv run armctrl lerobot export-processor-contract "
                f"--eef-plan-dir {agent_runtime_contract['plan_dir']} --json"
            ),
            (
                "uv run armctrl lerobot review-rollout "
                f"--eef-plan-dir {agent_runtime_contract['plan_dir']} --json"
            ),
        ],
        "notes": [
            "This script is a non-hardware sample of how an external LeRobot helper can consume armctrl.eef_agent_runtime_contract.v1.",
            "It does not import lerobot, start rollout execution, or move hardware.",
            "Use ordered_steps as the machine-readable sequence boundary so export, preview, and review are not parallelized by accident.",
            "Use the generated processor session plan as the boundary artifact for a future mature LeRobot runtime helper.",
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--agent-runtime-contract", required=True)
    parser.add_argument("--output")
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args(argv)

    agent_runtime_contract_path = Path(args.agent_runtime_contract)
    try:
        agent_runtime_contract = json.loads(
            agent_runtime_contract_path.read_text(encoding="utf-8")
        )
    except FileNotFoundError as error:
        payload = {
            "status": "rejected",
            "schema": SCHEMA,
            "error": {"code": "missing_agent_runtime_contract", "message": str(error)},
            "movement_allowed": False,
            "agent_runtime_contract_path": str(agent_runtime_contract_path),
            "next_gate": (
                "export a valid armctrl.eef_agent_runtime_contract.v1 artifact "
                "before using this helper sample"
            ),
        }
        return _emit(payload, as_json=args.as_json)
    except json.JSONDecodeError as error:
        payload = {
            "status": "rejected",
            "schema": SCHEMA,
            "error": {
                "code": "invalid_agent_runtime_contract_json",
                "message": str(error),
            },
            "movement_allowed": False,
            "agent_runtime_contract_path": str(agent_runtime_contract_path),
            "next_gate": "export a valid JSON agent runtime contract before using this helper sample",
        }
        return _emit(payload, as_json=args.as_json)

    try:
        payload = _build_payload(agent_runtime_contract, agent_runtime_contract_path)
    except ValueError as error:
        rejected = {
            "status": "rejected",
            "schema": SCHEMA,
            "error": {"code": "invalid_agent_runtime_contract", "message": str(error)},
            "movement_allowed": False,
            "agent_runtime_contract_path": str(agent_runtime_contract_path),
        }
        _emit(rejected, as_json=args.as_json)
        return 3
    except RuntimeError as error:
        rejected = {
            "status": "rejected",
            "schema": SCHEMA,
            "error": {
                "code": "unsupported_runtime_backend",
                "message": str(error),
            },
            "movement_allowed": False,
            "agent_runtime_contract_path": str(agent_runtime_contract_path),
            "resolved_backend": agent_runtime_contract.get("resolved_backend"),
            "next_gate": "use this helper sample only with lerobot_rollout agent runtime contracts",
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
