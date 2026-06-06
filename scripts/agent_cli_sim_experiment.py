from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any


SCHEMA = "armctrl.agent_cli_sim_experiment.v1"


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _json_path(output_dir: Path, index: int, name: str) -> Path:
    return output_dir / f"{index:02d}-{name}.json"


def _run_cli(
    *,
    output_dir: Path,
    index: int,
    name: str,
    args: list[str],
) -> tuple[dict[str, Any], dict[str, Any]]:
    command = [sys.executable, "-m", "armctrl.cli", *args, "--json"]
    env = dict(os.environ)
    env.setdefault("PYTHONUTF8", "1")
    completed = subprocess.run(
        command,
        cwd=_repo_root(),
        env=env,
        capture_output=True,
        text=True,
    )

    step_record: dict[str, Any] = {
        "index": index,
        "name": name,
        "command": " ".join(command),
        "returncode": completed.returncode,
    }
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError:
        payload = {
            "status": "failed",
            "schema": "armctrl.cli_raw_output.v1",
            "stdout": completed.stdout,
            "stderr": completed.stderr,
        }

    step_record["status"] = payload.get("status", "unknown")
    step_record["schema"] = payload.get("schema")
    step_record["artifact"] = str(_json_path(output_dir, index, name))
    if completed.returncode != 0:
        step_record["stderr"] = completed.stderr

    _json_path(output_dir, index, name).write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return step_record, payload


def _nested(payload: dict[str, Any], path: list[str], default: Any = None) -> Any:
    value: Any = payload
    for key in path:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def run_experiment(output_dir: Path) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    offcenter_dir = output_dir / "eef-offcenter"
    recipe_dir = output_dir / "recipe-home"
    eef_dir = output_dir / "eef-large"
    agent_flow_dir = output_dir / "agent-flow"
    processor_contract = eef_dir / "lerobot_processor_contract.json"

    steps: list[dict[str, Any]] = []
    payloads: dict[str, dict[str, Any]] = {}

    scripted_steps = [
        (
            "agent_flow_doctor",
            ["agent-flow", "doctor"],
        ),
        (
            "eef_offcenter_plan",
            [
                "eef",
                "plan-pose",
                "--frame",
                "eef_link",
                "--position",
                "0.55",
                "0.25",
                "0.25",
                "--rpy",
                "0.0",
                "0.0",
                "0.60",
                "--backend",
                "lerobot_rollout",
                "--output",
                str(offcenter_dir),
            ],
        ),
        (
            "eef_offcenter_preview",
            [
                "eef",
                "synthesize-preview",
                "--plan-dir",
                str(offcenter_dir),
                "--render",
                str(offcenter_dir / "offcenter_preview.html"),
            ],
        ),
        (
            "eef_offcenter_review",
            [
                "eef",
                "review",
                "--plan-dir",
                str(offcenter_dir),
                "--render",
                str(offcenter_dir / "offcenter_review.html"),
            ],
        ),
        (
            "recipe_home_plan",
            [
                "recipe",
                "plan",
                "home",
                "--output",
                str(recipe_dir),
                "--render",
                "trajectory_preview.html",
            ],
        ),
        (
            "recipe_agent_preset_contract",
            [
                "recipe",
                "export-agent-preset-contract",
                "--plan-dir",
                str(recipe_dir),
            ],
        ),
        (
            "eef_large_plan",
            [
                "eef",
                "plan-pose",
                "--frame",
                "eef_link",
                "--position",
                "0.55",
                "-0.25",
                "0.26",
                "--rpy",
                "0.0",
                "0.0",
                "-0.60",
                "--backend",
                "lerobot_rollout",
                "--output",
                str(eef_dir),
            ],
        ),
        (
            "eef_agent_session_plan",
            [
                "eef",
                "export-agent-session-plan",
                "--plan-dir",
                str(eef_dir),
            ],
        ),
        (
            "eef_synthesize_preview",
            [
                "eef",
                "synthesize-preview",
                "--plan-dir",
                str(eef_dir),
                "--recipe-plan-dir",
                str(recipe_dir),
                "--render",
                str(eef_dir / "large_preview.html"),
            ],
        ),
        (
            "eef_review",
            [
                "eef",
                "review",
                "--plan-dir",
                str(eef_dir),
                "--render",
                str(eef_dir / "review_preview.html"),
            ],
        ),
        (
            "lerobot_processor_contract",
            [
                "lerobot",
                "export-processor-contract",
                "--eef-plan-dir",
                str(eef_dir),
                "--output",
                str(processor_contract),
            ],
        ),
        (
            "lerobot_processor_helper_preview",
            [
                "lerobot",
                "processor-helper-preview",
                "--processor-contract",
                str(processor_contract),
                "--urdf-path",
                "configs/models/X5_camera.urdf",
                "--safe-config",
                "configs/x5.safe.yaml",
            ],
        ),
        (
            "agent_flow_plan",
            [
                "agent-flow",
                "plan",
                "--preset",
                "home",
                "--eef-mode",
                "pose_absolute",
                "--backend",
                "lerobot_rollout",
                "--position",
                "0.55",
                "-0.25",
                "0.26",
                "--rpy",
                "0.0",
                "0.0",
                "-0.60",
                "--output",
                str(agent_flow_dir),
            ],
        ),
        (
            "agent_flow_review",
            [
                "agent-flow",
                "review",
                "--contract",
                str(agent_flow_dir / "agent_flow_plan.json"),
            ],
        ),
    ]

    for index, (name, args) in enumerate(scripted_steps, start=1):
        step, payload = _run_cli(
            output_dir=output_dir,
            index=index,
            name=name,
            args=args,
        )
        steps.append(step)
        payloads[name] = payload
        if step["returncode"] != 0 or step["status"] not in {"ok", "completed"}:
            break

    recenter_allowed = bool(
        _nested(payloads.get("recipe_home_plan", {}), ["simulation_gate", "allowed"])
        or _nested(payloads.get("recipe_home_plan", {}), ["artifact_safety", "allowed"])
    )
    eef_action_id = _nested(
        payloads.get("eef_large_plan", {}),
        ["agent_action", "action_id"],
        _nested(payloads.get("eef_large_plan", {}), ["plan", "agent_action", "action_id"]),
    )
    offcenter_action_id = _nested(
        payloads.get("eef_offcenter_plan", {}),
        ["agent_action", "action_id"],
        _nested(
            payloads.get("eef_offcenter_plan", {}),
            ["plan", "agent_action", "action_id"],
        ),
    )
    eef_review_allowed = bool(
        _nested(payloads.get("eef_review", {}), ["sim_preview", "safety", "allowed"])
    )
    offcenter_review_allowed = bool(
        _nested(
            payloads.get("eef_offcenter_review", {}),
            ["sim_preview", "safety", "allowed"],
        )
    )
    synth_review_allowed = bool(
        _nested(
            payloads.get("eef_synthesize_preview", {}),
            ["review", "sim_preview", "safety", "allowed"],
        )
    )
    agent_flow_review_allowed = bool(
        _nested(
            payloads.get("agent_flow_review", {}),
            ["review", "sim_preview", "safety", "allowed"],
        )
    )
    processor_owner = _nested(
        payloads.get("eef_agent_session_plan", {}),
        ["agent_runtime_contract", "bridge_export", "lerobot_action", "agent_eef_compatibility", "processor_owner"],
        {},
    )
    if not processor_owner:
        processor_owner = _nested(
            payloads.get("lerobot_processor_contract", {}),
            ["lerobot_action", "agent_eef_compatibility", "processor_owner"],
            {},
        )
    eef_joint_ranges = _nested(
        payloads.get("eef_review", {}),
        ["sim_preview", "trajectory_metrics", "joint_ranges_rad"],
        {},
    )
    offcenter_joint_ranges = _nested(
        payloads.get("eef_offcenter_review", {}),
        ["sim_preview", "trajectory_metrics", "joint_ranges_rad"],
        {},
    )
    max_eef_joint_range = max(
        [float(value) for value in eef_joint_ranges.values()] or [0.0]
    )
    max_offcenter_joint_range = max(
        [float(value) for value in offcenter_joint_ranges.values()] or [0.0]
    )

    checks = {
        "all_steps_ok": all(step["status"] == "ok" for step in steps)
        and len(steps) == len(scripted_steps),
        "offcenter_eef_action_id": offcenter_action_id,
        "offcenter_eef_review_allowed": offcenter_review_allowed,
        "offcenter_joint_range_max_rad": max_offcenter_joint_range,
        "recenter_recipe_simulated": recenter_allowed,
        "eef_action_id": eef_action_id,
        "eef_joint_range_max_rad": max_eef_joint_range,
        "eef_synthesized_review_allowed": synth_review_allowed,
        "eef_review_allowed": eef_review_allowed,
        "agent_flow_review_allowed": agent_flow_review_allowed,
        "lerobot_processor_owner": processor_owner,
        "artifacts_exist": {
            "offcenter_plan": (offcenter_dir / "eef_plan.json").exists(),
            "offcenter_trajectory": (offcenter_dir / "backend_joint_trajectory.csv").exists(),
            "offcenter_render": (offcenter_dir / "offcenter_review.html").exists(),
            "recipe_trajectory": (recipe_dir / "planned_trajectory.csv").exists(),
            "recipe_render": (recipe_dir / "trajectory_preview.html").exists(),
            "eef_plan": (eef_dir / "eef_plan.json").exists(),
            "eef_trajectory": (eef_dir / "backend_joint_trajectory.csv").exists(),
            "agent_flow_contract": (agent_flow_dir / "agent_flow_plan.json").exists(),
        },
    }

    acceptance_status = (
        "pass"
        if checks["all_steps_ok"]
        and checks["offcenter_eef_action_id"] == "eef.pose_absolute"
        and checks["offcenter_eef_review_allowed"]
        and checks["offcenter_joint_range_max_rad"] >= 0.20
        and checks["recenter_recipe_simulated"]
        and checks["eef_action_id"] == "eef.pose_absolute"
        and checks["eef_joint_range_max_rad"] >= 0.20
        and checks["eef_synthesized_review_allowed"]
        and checks["eef_review_allowed"]
        and checks["agent_flow_review_allowed"]
        and processor_owner.get("action") == "robot_action_processor"
        and processor_owner.get("observation") == "robot_observation_processor"
        and all(checks["artifacts_exist"].values())
        else "fail"
    )

    summary = {
        "schema": SCHEMA,
        "acceptance_status": acceptance_status,
        "movement_allowed": False,
        "output_dir": str(output_dir),
        "intent_sequence": [
            "eef.pose_absolute.offcenter_preview",
            "preset.apply",
            "eef.pose_absolute.large_preview",
            "rollout.prepare",
        ],
        "checks": checks,
        "steps": steps,
        "artifacts": {
            "summary": str(output_dir / "agent_cli_sim_experiment.json"),
            "offcenter_plan_dir": str(offcenter_dir),
            "recipe_plan_dir": str(recipe_dir),
            "eef_plan_dir": str(eef_dir),
            "agent_flow_dir": str(agent_flow_dir),
            "offcenter_render": str(offcenter_dir / "offcenter_review.html"),
            "recipe_render": str(recipe_dir / "trajectory_preview.html"),
            "eef_render": str(eef_dir / "review_preview.html"),
        },
    }
    (output_dir / "agent_cli_sim_experiment.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a fixed non-hardware Agent CLI simulation experiment."
    )
    parser.add_argument("--output", default="runs/agent-cli-sim-experiment")
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args(argv)

    summary = run_experiment(Path(args.output))
    if args.as_json:
        print(json.dumps(summary, ensure_ascii=False))
    else:
        print(f"acceptance_status: {summary['acceptance_status']}")
        print(f"output_dir: {summary['output_dir']}")
        print(f"summary: {summary['artifacts']['summary']}")
    return 0 if summary["acceptance_status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
