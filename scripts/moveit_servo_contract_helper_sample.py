from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


SCHEMA = "armctrl.moveit_servo_helper_plan.v1"
RUNNER_SCHEMA = "armctrl.eef_runner_contract.v1"


def _emit(payload: dict[str, object], *, as_json: bool) -> int:
    if as_json:
        sys.stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
    else:
        sys.stdout.write(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
    return 0


def _build_payload(
    runner_contract: dict[str, object], runner_contract_path: Path
) -> dict[str, object]:
    if runner_contract.get("schema") != RUNNER_SCHEMA:
        raise ValueError(f"runner contract file must use schema {RUNNER_SCHEMA}")
    resolved_backend = runner_contract.get("resolved_backend")
    if resolved_backend != "moveit_servo":
        raise RuntimeError(
            f"unsupported runner backend for moveit servo helper sample: {resolved_backend}"
        )
    bridge_export = dict(runner_contract["bridge_export"])
    ros_contract = dict(bridge_export["ros_contract"])
    environment = dict(bridge_export["environment"])
    message_type = str(ros_contract["message_type"])
    if message_type == "geometry_msgs/msg/TwistStamped":
        command_topic = "~/delta_twist_cmds"
        command_payload = {
            "frame_id": ros_contract["frame_id"],
            "twist": ros_contract["twist"],
            "control_period_s": ros_contract["control_period_s"],
        }
        command_mode = "twist"
    elif message_type == "geometry_msgs/msg/PoseStamped":
        command_topic = "~/pose_command_in_topic"
        command_payload = {
            "frame_id": ros_contract["frame_id"],
            "pose_6d": ros_contract["pose_6d"],
        }
        command_mode = "pose"
    else:
        raise ValueError(
            "moveit servo helper sample only supports TwistStamped or PoseStamped contracts"
        )
    return {
        "schema": SCHEMA,
        "movement_allowed": False,
        "runner_contract_path": str(runner_contract_path),
        "plan_dir": str(runner_contract["plan_dir"]),
        "resolved_backend": resolved_backend,
        "runtime_owner": runner_contract["runner_api"]["owner"],
        "runtime_boundary": _runtime_boundary(str(runner_contract["runner_api"]["owner"])),
        "frequency_contract": _frequency_contract(ros_contract),
        "servo_session_plan": {
            "session_kind": "ros2_servo_node",
            "command_mode": command_mode,
            "message_type": message_type,
            "command_topic": command_topic,
            "command_payload": command_payload,
            "status_topic": "~/status",
            "pause_service": "~/pause_servo",
            "switch_command_type_service": "~/switch_command_type",
            "command_out_topic_param": "command_out_topic",
            "command_out_type_param": "command_out_type",
            "supported_output_types": [
                "trajectory_msgs/JointTrajectory",
                "std_msgs/Float64MultiArray",
            ],
            "source_hint": environment["source_hint"],
        },
        "review_output_contract": runner_contract["review_output_contract"],
        "reference_docs": runner_contract["runner_api"]["reference_docs"],
        "next_steps": [
            f"uv run armctrl eef sample-runner --runner-contract {runner_contract_path} --json",
            runner_contract["review_output_contract"]["review_command"],
        ],
        "notes": [
            "This script is a non-hardware sample of how an external MoveIt Servo helper can consume armctrl.eef_runner_contract.v1.",
            "It does not start ROS 2, publish commands, or move hardware.",
            "Use the generated session plan as the boundary artifact for a future mature MoveIt Servo helper implementation.",
        ],
    }


def _runtime_boundary(runtime_owner: str) -> dict[str, object]:
    return {
        "runtime_owner": runtime_owner,
        "armctrl_role": "contract_preview_audit_only",
        "motion_runtime_owner": False,
        "hardware_execution": "outside_armctrl",
    }


def _frequency_contract(ros_contract: dict[str, object]) -> dict[str, object]:
    control_period_s = ros_contract.get("control_period_s")
    agent_intent_hz = None
    if isinstance(control_period_s, int | float) and float(control_period_s) > 0.0:
        agent_intent_hz = 1.0 / float(control_period_s)
    return {
        "agent_intent_hz": agent_intent_hz,
        "command_publish_hz": "runtime_configured",
        "servo_loop_hz": "moveit_servo_runtime_configured",
        "actual_send_hz": "measure_in_runtime_artifact",
        "controller_dt_s": "not_owned_by_armctrl",
        "timestamp_policy": "ros_clock_or_servo_runtime_policy",
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
            "next_gate": "use this helper sample only with moveit_servo runner contracts",
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
