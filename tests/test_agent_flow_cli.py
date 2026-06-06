import json
import subprocess
import sys
from pathlib import Path


def test_cli_agent_flow_doctor_reports_available_nonhardware_paths() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "doctor",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.agent_flow_doctor.v1"
    assert payload["movement_allowed"] is False
    assert payload["read_only"] is True
    assert payload["surfaces"]["recipe"]["status"] == "available"
    assert payload["surfaces"]["eef"]["status"] == "available"
    assert payload["surfaces"]["lerobot"]["status"] == "available"
    assert payload["surfaces"]["runtime_backends"]["sdk_cartesian"] in {"available", "missing"}
    assert payload["surfaces"]["runtime_backends"]["moveit_servo"] in {
        "available",
        "missing",
        "installed_not_sourced",
    }
    assert payload["surfaces"]["runtime_backends"]["lerobot_rollout"] == "contract_available"


def test_cli_agent_flow_plan_for_lerobot_pose_delta_closes_shared_review(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "agent-flow-lerobot"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_delta",
            "--backend",
            "lerobot_rollout",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.agent_flow_plan.v1"
    assert payload["movement_allowed"] is False
    assert payload["agent_motion_contract"]["schema"] == "armctrl.agent_motion_contract.v1"
    assert payload["agent_motion_contract"]["selected_action_id"] == "eef.pose_delta"
    assert payload["recommended_path"]["schema"] == "armctrl.recommended_path.v1"
    assert payload["recommended_path"]["profile"] == "preset_then_bounded_eef_then_review"
    assert payload["recommended_path"]["primary_entrypoint"] == "armctrl agent-flow plan"
    assert payload["recommended_path"]["steps"][2]["id"] == "export_agent_session_plan"
    assert "export-agent-session-plan" in payload["recommended_path"]["steps"][2]["command"]
    assert payload["agent_motion_contract"]["lerobot_compatibility"]["status"] == "compatible"
    assert (
        payload["agent_motion_contract"]["lerobot_compatibility"]["preferred_training_action_id"]
        == "eef.pose_delta"
    )
    assert (
        payload["agent_motion_contract"]["lerobot_compatibility"]["processor_owner"]["action"]
        == "robot_action_processor"
    )
    assert payload["preset"]["recipe"]["name"] == "home"
    assert payload["eef"]["plan"]["agent_action"]["action_id"] == "eef.pose_delta"
    assert payload["runtime"]["backend"] == "lerobot_rollout"
    assert payload["runtime"]["helper_plan"]["schema"] == "armctrl.lerobot_agent_runtime_helper_plan.v1"
    assert payload["runtime"]["processor_contract"]["schema"] == "armctrl.lerobot_eef_processor_contract.v1"
    assert payload["review"]["review_status"] == "completed"
    assert payload["review"]["sim_preview"]["safety"]["allowed"] is True
    assert payload["artifacts"]["recipe_plan_dir"] == str(output_dir / "recipe-plan")
    assert payload["artifacts"]["eef_plan_dir"] == str(output_dir / "eef-plan")
    assert payload["artifacts"]["agent_flow_contract"] == str(output_dir / "agent_flow_plan.json")
    assert (output_dir / "agent_flow_plan.json").exists()
    assert payload["ordered_steps"][0]["id"] == "plan_recipe_preset"
    assert any(step["id"] == "review_runtime_handoff" for step in payload["ordered_steps"])

    saved = json.loads((output_dir / "agent_flow_plan.json").read_text(encoding="utf-8"))
    assert saved["schema"] == "armctrl.agent_flow_plan.v1"
    assert saved["runtime"]["backend"] == "lerobot_rollout"


def test_cli_agent_flow_plan_for_sdk_pose_absolute_exports_helper_and_preview(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "agent-flow-sdk"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_absolute",
            "--backend",
            "sdk_cartesian",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["eef"]["plan"]["agent_action"]["action_id"] == "eef.pose_absolute"
    assert payload["agent_motion_contract"]["selected_action_id"] == "eef.pose_absolute"
    assert payload["recommended_path"]["steps"][2]["id"] == "export_agent_session_plan"
    assert "--backend sdk_cartesian" in payload["recommended_path"]["steps"][2]["command"]
    assert payload["runtime"]["backend"] == "sdk_cartesian"
    assert payload["runtime"]["runner_contract"]["schema"] == "armctrl.eef_runner_contract.v1"
    assert payload["runtime"]["helper_plan"]["schema"] == "armctrl.sdk_cartesian_helper_plan.v1"
    assert payload["review"]["schema"] == "armctrl.eef_runner_preview.v1"
    assert payload["review"]["review_status"] == "completed"
    assert payload["review"]["sim_preview"]["safety"]["allowed"] is True
    assert payload["artifacts"]["agent_flow_contract"] == str(output_dir / "agent_flow_plan.json")


def test_cli_agent_flow_plan_requires_mode_specific_arguments() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_delta",
            "--backend",
            "sdk_cartesian",
            "--output",
            "runs/agent-flow-invalid",
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert "--delta-position and --delta-rpy are required for --eef-mode pose_delta" in completed.stderr


def test_cli_agent_flow_review_replays_saved_contract(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "agent-flow-review"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_delta",
            "--backend",
            "lerobot_rollout",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "review",
            "--contract",
            str(output_dir / "agent_flow_plan.json"),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.agent_flow_review.v1"
    assert payload["movement_allowed"] is False
    assert payload["contract_path"] == str(output_dir / "agent_flow_plan.json")
    assert payload["backend"] == "lerobot_rollout"
    assert payload["recommended_path"]["profile"] == "preset_then_bounded_eef_then_review"
    assert payload["review"]["review_status"] == "completed"
    assert payload["review"]["sim_preview"]["safety"]["allowed"] is True
