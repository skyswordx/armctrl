from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from armctrl.runtime_session import record_runtime_hold_tick, runtime_readiness


SCHEMA = "armctrl.agent_cli_sim_experiment.v1"
FAKE_RUNTIME_MAX_HEARTBEAT_AGE_S = 30.0


@dataclass(frozen=True)
class AgentCliSimulationExperimentRequest:
    output_dir: Path


class AgentCliSimulationExperiment:
    def run(self, request: AgentCliSimulationExperimentRequest) -> dict[str, Any]:
        return _run_experiment(request.output_dir)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


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


def _run_experiment(output_dir: Path) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    offcenter_dir = output_dir / "eef-offcenter"
    recipe_dir = output_dir / "recipe-home"
    eef_dir = output_dir / "eef-large"
    agent_flow_dir = output_dir / "agent-flow"
    runtime_gateway_dir = output_dir / "runtime-gateway"
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

    runtime_gateway = _run_runtime_gateway_smoke(runtime_gateway_dir)

    return _summarize_experiment(
        output_dir=output_dir,
        offcenter_dir=offcenter_dir,
        recipe_dir=recipe_dir,
        eef_dir=eef_dir,
        agent_flow_dir=agent_flow_dir,
        runtime_gateway_dir=runtime_gateway_dir,
        steps=steps,
        scripted_step_count=len(scripted_steps),
        payloads=payloads,
        runtime_gateway=runtime_gateway,
    )


def _run_runtime_gateway_smoke(output_dir: Path) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    session_artifact = output_dir / "runtime_session.json"
    status_artifact = output_dir / "runtime_status.json"
    intent_submit_artifact = output_dir / "agent_joint_intent_submit.json"
    trajectory_submit_artifact = output_dir / "agent_joint_trajectory_submit.json"
    stop_artifact = output_dir / "runtime_stop.json"

    steps: list[dict[str, Any]] = []
    payloads: dict[str, dict[str, Any]] = {}
    commands = [
        (
            "runtime_start_fake",
            [
                "runtime",
                "start",
                "--backend",
                "fake",
                "--q-current",
                "0.0",
                "0.3",
                "0.3",
                "--safe-center",
                "0.0",
                "0.3",
                "0.3",
                "--output",
                str(session_artifact),
            ],
        ),
        (
            "runtime_status_fake",
            [
                "runtime",
                "status",
                "--session-artifact",
                str(session_artifact),
                "--max-heartbeat-age-s",
                str(FAKE_RUNTIME_MAX_HEARTBEAT_AGE_S),
                "--output",
                str(status_artifact),
            ],
        ),
        (
            "agent_joint_intent_submit",
            [
                "motion",
                "submit",
                "joint-intent",
                "--session-artifact",
                str(session_artifact),
                "--owner",
                "agent",
                "--expected-q-start",
                "0.0",
                "0.3",
                "0.3",
                "--q-target",
                "0.005",
                "0.3",
                "0.3",
                "--control-period-s",
                "0.1",
                "--send-hz",
                "50",
                "--max-joint-delta-rad",
                "0.01",
                "--max-tracking-error-rad",
                "0.03",
                "--max-heartbeat-age-s",
                str(FAKE_RUNTIME_MAX_HEARTBEAT_AGE_S),
                "--output",
                str(intent_submit_artifact),
            ],
        ),
        (
            "agent_joint_trajectory_submit",
            [
                "motion",
                "submit",
                "joint-trajectory",
                "--session-artifact",
                str(session_artifact),
                "--owner",
                "agent",
                "--expected-q-start",
                "0.0",
                "0.3",
                "0.3",
                "--q-point",
                "0.0",
                "0.3",
                "0.3",
                "--q-point",
                "0.002",
                "0.3",
                "0.3",
                "--send-hz",
                "50",
                "--trajectory-sample-hz",
                "50",
                "--max-heartbeat-age-s",
                str(FAKE_RUNTIME_MAX_HEARTBEAT_AGE_S),
                "--output",
                str(trajectory_submit_artifact),
            ],
        ),
        (
            "runtime_stop_fake",
            [
                "runtime",
                "stop",
                "--session-artifact",
                str(session_artifact),
                "--max-heartbeat-age-s",
                str(FAKE_RUNTIME_MAX_HEARTBEAT_AGE_S),
                "--output",
                str(stop_artifact),
            ],
        ),
    ]

    for index, (name, args) in enumerate(commands, start=1):
        if (
            name == "runtime_status_fake" or name.startswith("agent_joint_")
        ) and session_artifact.exists():
            _record_fake_hold_tick(session_artifact)
        step, payload = _run_cli(
            output_dir=output_dir,
            index=index,
            name=name,
            args=args,
        )
        steps.append(step)
        payloads[name] = payload
        if name == "runtime_start_fake" and session_artifact.exists():
            _record_fake_hold_tick(session_artifact)
        if step["returncode"] != 0 or step["status"] not in {
            "ok",
            "blocked",
            "queued",
            "stopped",
        }:
            break

    return {
        "schema": "armctrl.agent_runtime_gateway_sim.v1",
        "movement_allowed": False,
        "steps": steps,
        "payloads": payloads,
        "artifacts": {
            "session": str(session_artifact),
            "status": str(status_artifact),
            "joint_intent_submit": str(intent_submit_artifact),
            "joint_trajectory_submit": str(trajectory_submit_artifact),
            "stop": str(stop_artifact),
        },
    }


def _record_fake_hold_tick(session_artifact: Path) -> None:
    payload = json.loads(session_artifact.read_text(encoding="utf-8"))
    q_hold = tuple(float(value) for value in payload.get("q_hold", ()))
    updated = record_runtime_hold_tick(
        payload,
        q_meas=q_hold,
        fault_flags=(),
        max_heartbeat_age_s=FAKE_RUNTIME_MAX_HEARTBEAT_AGE_S,
    )
    now_s = time.time()
    updated["heartbeat"] = {
        "wall_time_s": now_s,
        "age_s": 0.0,
        "max_age_s": FAKE_RUNTIME_MAX_HEARTBEAT_AGE_S,
        "fresh": True,
    }
    updated["last_hold_wall_time_s"] = now_s
    updated["hold_fresh"] = True
    updated["hold_age_s"] = 0.0
    updated["readiness"] = runtime_readiness(updated)
    session_artifact.write_text(
        json.dumps(updated, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _summarize_experiment(
    *,
    output_dir: Path,
    offcenter_dir: Path,
    recipe_dir: Path,
    eef_dir: Path,
    agent_flow_dir: Path,
    runtime_gateway_dir: Path,
    steps: list[dict[str, Any]],
    scripted_step_count: int,
    payloads: dict[str, dict[str, Any]],
    runtime_gateway: dict[str, Any],
) -> dict[str, Any]:
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
        [
            "agent_runtime_contract",
            "bridge_export",
            "lerobot_action",
            "agent_eef_compatibility",
            "processor_owner",
        ],
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
    runtime_payloads = runtime_gateway.get("payloads", {})
    intent_submit = runtime_payloads.get("agent_joint_intent_submit", {})
    trajectory_submit = runtime_payloads.get("agent_joint_trajectory_submit", {})
    runtime_steps = list(runtime_gateway.get("steps", []))
    runtime_gateway_steps_ok = len(runtime_steps) == 5 and [
        step["status"] for step in runtime_steps
    ] == ["blocked", "ok", "queued", "queued", "stopped"]

    checks = {
        "all_steps_ok": all(step["status"] == "ok" for step in steps)
        and len(steps) == scripted_step_count,
        "offcenter_eef_action_id": offcenter_action_id,
        "offcenter_eef_review_allowed": offcenter_review_allowed,
        "offcenter_joint_range_max_rad": max_offcenter_joint_range,
        "recenter_recipe_simulated": recenter_allowed,
        "eef_action_id": eef_action_id,
        "eef_joint_range_max_rad": max_eef_joint_range,
        "eef_synthesized_review_allowed": synth_review_allowed,
        "eef_review_allowed": eef_review_allowed,
        "agent_flow_review_allowed": agent_flow_review_allowed,
        "runtime_gateway_steps_ok": runtime_gateway_steps_ok,
        "joint_intent_runtime_submit": {
            "status": intent_submit.get("status"),
            "command_surface": intent_submit.get("command_surface"),
            "motion_kind": intent_submit.get("motion_kind"),
            "owner": intent_submit.get("owner"),
            "mode": intent_submit.get("mode"),
            "movement_command_sent": intent_submit.get("movement_command_sent"),
        },
        "joint_trajectory_runtime_submit": {
            "status": trajectory_submit.get("status"),
            "command_surface": trajectory_submit.get("command_surface"),
            "motion_kind": trajectory_submit.get("motion_kind"),
            "owner": trajectory_submit.get("owner"),
            "mode": trajectory_submit.get("mode"),
            "movement_command_sent": trajectory_submit.get("movement_command_sent"),
        },
        "lerobot_processor_owner": processor_owner,
        "artifacts_exist": {
            "offcenter_plan": (offcenter_dir / "eef_plan.json").exists(),
            "offcenter_trajectory": (
                offcenter_dir / "backend_joint_trajectory.csv"
            ).exists(),
            "offcenter_render": (offcenter_dir / "offcenter_review.html").exists(),
            "recipe_trajectory": (recipe_dir / "planned_trajectory.csv").exists(),
            "recipe_render": (recipe_dir / "trajectory_preview.html").exists(),
            "eef_plan": (eef_dir / "eef_plan.json").exists(),
            "eef_trajectory": (eef_dir / "backend_joint_trajectory.csv").exists(),
            "agent_flow_contract": (agent_flow_dir / "agent_flow_plan.json").exists(),
            "runtime_gateway_session": (
                runtime_gateway_dir / "runtime_session.json"
            ).exists(),
            "runtime_gateway_intent_submit": (
                runtime_gateway_dir / "agent_joint_intent_submit.json"
            ).exists(),
            "runtime_gateway_trajectory_submit": (
                runtime_gateway_dir / "agent_joint_trajectory_submit.json"
            ).exists(),
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
        and checks["runtime_gateway_steps_ok"]
        and checks["joint_intent_runtime_submit"]["status"] == "queued"
        and checks["joint_intent_runtime_submit"]["owner"] == "agent"
        and checks["joint_intent_runtime_submit"]["mode"] == "agent_servo"
        and checks["joint_intent_runtime_submit"]["movement_command_sent"] is False
        and checks["joint_trajectory_runtime_submit"]["status"] == "queued"
        and checks["joint_trajectory_runtime_submit"]["owner"] == "agent"
        and checks["joint_trajectory_runtime_submit"]["mode"] == "trajectory_replay"
        and checks["joint_trajectory_runtime_submit"]["movement_command_sent"] is False
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
            "runtime_gateway_dir": str(runtime_gateway_dir),
            "offcenter_render": str(offcenter_dir / "offcenter_review.html"),
            "recipe_render": str(recipe_dir / "trajectory_preview.html"),
            "eef_render": str(eef_dir / "review_preview.html"),
        },
        "runtime_gateway": runtime_gateway,
    }
    (output_dir / "agent_cli_sim_experiment.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return summary
