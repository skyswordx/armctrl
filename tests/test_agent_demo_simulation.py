import json
import subprocess
import sys
from pathlib import Path

from armctrl.agent_demo_simulation import (
    AgentDemoSimulation,
    AgentDemoSimulationRequest,
)


def _request(
    output_dir: Path, *, instruction: str, object_id: str
) -> AgentDemoSimulationRequest:
    return AgentDemoSimulationRequest(
        instruction=instruction,
        object_id=object_id,
        output_dir=output_dir,
        urdf_path=Path("configs/models/X5_camera.urdf"),
        safe_config_path=Path("configs/x5.safe.yaml"),
        backend="auto",
    )


def test_agent_demo_safe_object_writes_motion_preview(tmp_path: Path) -> None:
    output_dir = tmp_path / "red-cup-demo"

    payload = AgentDemoSimulation().run(
        _request(output_dir, instruction="pick the red cup", object_id="red_cup")
    )

    assert payload["schema"] == "armctrl.agent_demo_simulation.v1"
    assert payload["movement_allowed"] is False
    assert payload["hardware_motion_allowed"] is False
    assert payload["simulated_motion_allowed"] is True
    assert payload["perception"]["selected_object"]["id"] == "red_cup"
    assert payload["agent_command"]["parsed_intent"] == "pick_object"
    assert payload["safety_policy"]["simulated_action_allowed"] is True
    assert payload["safety_policy"]["hardware_action_allowed"] is False
    assert payload["danger_handling"]["action"] == "preview_and_log"
    assert all(value == "pass" for value in payload["acceptance_items"].values())

    trajectory_path = output_dir / "agent_demo_trajectory.csv"
    preview_path = output_dir / "agent_demo.html"
    summary_path = output_dir / "agent_demo.json"
    assert trajectory_path.exists()
    assert summary_path.exists()
    assert "URDF kinematic animation" in preview_path.read_text(encoding="utf-8")
    assert (
        json.loads(summary_path.read_text(encoding="utf-8"))["schema"]
        == payload["schema"]
    )


def test_agent_demo_dangerous_object_blocks_action_but_writes_preview(tmp_path: Path) -> None:
    output_dir = tmp_path / "knife-demo"

    payload = AgentDemoSimulation().run(
        _request(output_dir, instruction="pick the knife", object_id="knife")
    )

    assert payload["schema"] == "armctrl.agent_demo_simulation.v1"
    assert payload["movement_allowed"] is False
    assert payload["hardware_motion_allowed"] is False
    assert payload["simulated_motion_allowed"] is True
    assert payload["safety_policy"]["simulated_action_allowed"] is False
    assert payload["safety_policy"]["handling_mode"] == "blocked_preview_with_operator_notice"
    assert payload["safety_policy"]["failed_checks"] == ["dangerous_object"]
    assert payload["danger_handling"]["action"] == "block_hardware_and_notify"
    assert "危险" in payload["safety_policy"]["operator_notice"]

    preview_path = output_dir / "agent_demo.html"
    assert "URDF kinematic animation" in preview_path.read_text(encoding="utf-8")


def test_cli_sim_agent_demo_outputs_json_and_artifacts(tmp_path: Path) -> None:
    output_dir = tmp_path / "agent-demo-cli"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sim",
            "agent-demo",
            "--instruction",
            "pick the red cup",
            "--object",
            "red_cup",
            "--output",
            str(output_dir),
            "--backend",
            "auto",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.agent_demo_simulation.v1"
    assert payload["hardware_motion_allowed"] is False
    assert payload["safety_policy"]["simulated_action_allowed"] is True
    assert (output_dir / "agent_demo.json").exists()
    assert (output_dir / "agent_demo_trajectory.csv").exists()
    assert (output_dir / "agent_demo.html").exists()
