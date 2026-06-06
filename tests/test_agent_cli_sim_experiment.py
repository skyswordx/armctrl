import json
import subprocess
import sys
from pathlib import Path


def test_agent_cli_sim_experiment_runs_recenter_and_eef_preview(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "agent-cli-sim"

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/agent_cli_sim_experiment.py",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["schema"] == "armctrl.agent_cli_sim_experiment.v1"
    assert payload["acceptance_status"] == "pass"
    assert payload["movement_allowed"] is False
    assert payload["checks"]["offcenter_eef_action_id"] == "eef.pose_absolute"
    assert payload["checks"]["offcenter_eef_review_allowed"] is True
    assert payload["checks"]["offcenter_joint_range_max_rad"] >= 0.20
    assert payload["checks"]["recenter_recipe_simulated"] is True
    assert payload["checks"]["eef_action_id"] == "eef.pose_absolute"
    assert payload["checks"]["eef_joint_range_max_rad"] >= 0.20
    assert payload["checks"]["eef_review_allowed"] is True
    assert payload["checks"]["agent_flow_review_allowed"] is True
    assert payload["checks"]["lerobot_processor_owner"]["action"] == "robot_action_processor"
    assert payload["checks"]["lerobot_processor_owner"]["observation"] == "robot_observation_processor"
    assert len(payload["steps"]) >= 7
    assert all(step["status"] == "ok" for step in payload["steps"])

    summary_path = output_dir / "agent_cli_sim_experiment.json"
    assert summary_path.exists()
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert summary["acceptance_status"] == "pass"

    assert (output_dir / "recipe-home" / "planned_trajectory.csv").exists()
    assert (output_dir / "recipe-home" / "trajectory_preview.html").exists()
    assert (output_dir / "eef-offcenter" / "eef_plan.json").exists()
    assert (output_dir / "eef-offcenter" / "backend_joint_trajectory.csv").exists()
    assert (output_dir / "eef-offcenter" / "offcenter_review.html").exists()
    assert (output_dir / "eef-large" / "eef_plan.json").exists()
    assert (output_dir / "eef-large" / "backend_joint_trajectory.csv").exists()
    assert (output_dir / "eef-large" / "review_preview.html").exists()
    assert (output_dir / "agent-flow" / "agent_flow_plan.json").exists()
