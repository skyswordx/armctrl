import json
import subprocess
import sys


def test_cli_release_status_reports_simulation_safety_rc_and_hardware_pending() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "release", "status", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    milestone_ids = [milestone["id"] for milestone in payload["milestones"]]

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.release_status.v1"
    assert payload["version"] == "0.6.0-rc.25"
    assert payload["branch"] == "codex/armctrl-clean-rebuild"
    assert (
        payload["release_readiness"]
        == "simulation_safety_preview_nonhardware_verified"
    )
    assert milestone_ids == [
        "v0.2.0",
        "v0.3.0",
        "v0.4.0",
        "v0.5.0",
        "v0.6.0-rc.25",
    ]
    assert payload["milestones"][-1]["status"] == "rc_nonhardware_verified_remote_backend_pending"
    assert "real_sdk_runner_hardware_validation" in payload["hardware_pending"]
    assert "sdk_gravity_smoke_on_target_linux" in payload["hardware_pending"]
    assert "native_lerobot_record_on_target_linux" in payload["hardware_pending"]
    assert "native_lerobot_rollout_on_target_linux" in payload["hardware_pending"]
    assert "n100d_pinocchio_figaroh_validation" in payload["hardware_pending"]
    assert "n100d_mujoco_moveit_pinocchio_coal_doctor" in payload["hardware_pending"]
    assert payload["verification"]["local_commands"] == [
        "uv sync --extra dev --extra sim",
        "uv run pytest -q",
        "uv run python -m compileall src tests",
        "uv run armctrl release status --json",
        "uv run armctrl sim doctor --json",
        "uv run armctrl agent-flow doctor --json",
        "uv sync --extra dev --extra eef",
        "uv run armctrl recipe plan home --output runs/recipe-preview-smoke --json",
        "uv run armctrl recipe export-eef-seed --plan-dir runs/recipe-preview-smoke --json",
        "uv run armctrl recipe export-agent-preset-contract --plan-dir runs/recipe-preview-smoke --json",
        "uv run armctrl agent-flow plan --preset home --eef-mode pose_delta --backend lerobot_rollout --delta-position 0.002 0.000 -0.003 --delta-rpy 0 0 0.02 --output runs/agent-flow-preview-smoke --json",
        "uv run armctrl agent-flow review --contract runs/agent-flow-preview-smoke/agent_flow_plan.json --json",
        "uv run python scripts/agent_cli_sim_experiment.py --output runs/agent-cli-sim-acceptance --json",
        "uv run armctrl eef plan-pose --frame eef_link --position 0.40 0.00 0.20 --rpy 0 0 0 --backend sdk_cartesian --output runs/eef-preview-smoke --json",
        "uv run armctrl eef plan-delta-pose --frame eef_link --delta-position 0.002 0.000 -0.003 --delta-rpy 0 0 0.02 --backend sdk_cartesian --output runs/eef-delta-preview-smoke --json",
        "uv run armctrl eef runtime-plan --plan-dir runs/eef-preview-smoke --model X5 --interface can0 --json",
        "uv run armctrl eef export-lerobot-action --plan-dir runs/eef-preview-smoke --json",
        "uv run armctrl eef export-sdk-cartesian --plan-dir runs/eef-preview-smoke --model X5 --interface can0 --json",
        "uv run armctrl eef export-moveit-servo --plan-dir runs/eef-preview-smoke --json",
        "uv run armctrl eef export-runtime-bridge --plan-dir runs/eef-preview-smoke --json",
        "uv run armctrl eef export-runner-contract --plan-dir runs/eef-preview-smoke --json",
        "uv run armctrl eef export-agent-runtime-contract --plan-dir runs/eef-preview-smoke --json",
        "uv run armctrl eef export-agent-session-plan --plan-dir runs/eef-preview-smoke --json",
        "uv run armctrl eef preview-runner --plan-dir runs/eef-preview-smoke --json",
        "uv run armctrl eef export-runner-contract --plan-dir runs/eef-preview-smoke --output runs/eef-preview-smoke/eef_runner_contract.json --json",
        "uv run armctrl eef sample-runner --runner-contract runs/eef-preview-smoke/eef_runner_contract.json --json",
        "uv run armctrl eef export-sdk-helper-plan --runner-contract runs/eef-preview-smoke/eef_runner_contract.json --json",
        "uv run python scripts/sdk_cartesian_contract_helper_sample.py --runner-contract runs/eef-preview-smoke/eef_runner_contract.json --json",
        "uv run armctrl eef export-agent-runtime-contract --plan-dir runs/eef-preview-smoke --backend lerobot_rollout --output runs/eef-preview-smoke/eef_agent_runtime_contract.json --json",
        "uv run armctrl lerobot agent-runtime-helper-plan --agent-runtime-contract runs/eef-preview-smoke/eef_agent_runtime_contract.json --json",
        "uv run python scripts/lerobot_agent_runtime_helper_sample.py --agent-runtime-contract runs/eef-preview-smoke/eef_agent_runtime_contract.json --json",
        "uv run armctrl lerobot export-processor-contract --eef-plan-dir runs/eef-preview-smoke --output runs/eef-preview-smoke/lerobot_processor_contract.json --json",
        "uv run armctrl lerobot processor-helper-preview --processor-contract runs/eef-preview-smoke/lerobot_processor_contract.json --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --json",
        "uv run python scripts/lerobot_processor_contract_helper_sample.py --processor-contract runs/eef-preview-smoke/lerobot_processor_contract.json --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --json",
        "uv run armctrl eef plan-pose --frame eef_link --position 0.40 0.00 0.20 --rpy 0 0 0 --backend moveit_servo --output runs/eef-moveit-helper-smoke --json",
        "uv run armctrl eef export-runner-contract --plan-dir runs/eef-moveit-helper-smoke --output runs/eef-moveit-helper-smoke/eef_runner_contract.json --json",
        "uv run armctrl eef export-moveit-helper-plan --runner-contract runs/eef-moveit-helper-smoke/eef_runner_contract.json --json",
        "uv run python scripts/moveit_servo_contract_helper_sample.py --runner-contract runs/eef-moveit-helper-smoke/eef_runner_contract.json --json",
        "uv run armctrl eef synthesize-preview --plan-dir runs/eef-preview-smoke --json",
        "uv run armctrl eef synthesize-preview --plan-dir runs/eef-delta-preview-smoke --json",
        "uv run armctrl eef stage-trajectory --plan-dir runs/eef-preview-smoke --trajectory runs/eef-preview-smoke/backend_joint_trajectory.csv --json",
        "uv run armctrl lerobot config-plan rollout --model X5 --robot-interface can0 --policy-path outputs/train/act_arx5/checkpoints/last/pretrained_model --eef-plan-dir runs/eef-preview-smoke --json",
        "uv run armctrl lerobot export-processor-contract --eef-plan-dir runs/eef-preview-smoke --json",
        "uv run armctrl lerobot preview-rollout --eef-plan-dir runs/eef-preview-smoke --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --json",
        "uv run armctrl lerobot stage-rollout-trajectory --eef-plan-dir runs/eef-preview-smoke --trajectory runs/eef-preview-smoke/backend_joint_trajectory.csv --json",
        "uv run armctrl lerobot review-rollout --eef-plan-dir runs/eef-preview-smoke --trajectory runs/eef-preview-smoke/backend_joint_trajectory.csv --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --json",
        "uv run armctrl eef review --plan-dir runs/eef-preview-smoke --json",
        "uv run armctrl sysid plan gravity_sweep --dof 6 --sample-hz 100 --duration 2 --amplitude 0.05 --q-center 0 0.30 0.30 0 0 0 --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --output runs/plan-preview-smoke --json",
        "uv sync --extra dev --extra lerobot",
        "uv run armctrl lerobot doctor --model X5 --robot-interface can0 --teleop-interface can1 --json",
        "uv run armctrl sysid compile-runtime --execution-trajectory runs/plan-preview-smoke/execution_trajectory.csv --output runs/sysid-runtime-compile-smoke --json",
        "uv run armctrl motion submit joint-trajectory --compiled-command runs/sysid-runtime-compile-smoke/compiled_motion_command.json --session-artifact runs/lab/runtime_session.json --owner sysid --json",
    ]
    all_local_commands = "\n".join(payload["verification"]["local_commands"])
    assert "sysid run gravity_sweep --adapter sdk" not in all_local_commands
    assert payload["verification"]["test_count"] == 190
    assert payload["deferred_validation"]["requires_hardware"] == [
        "real_sdk_runner_hardware_validation",
        "native_lerobot_record_on_target_linux",
        "native_lerobot_rollout_on_target_linux",
        "sdk_preflight_on_target_linux",
        "sdk_handshake_plan_on_target_linux",
        "sdk_gravity_smoke_on_target_linux",
        "hold_damping_ctrl_c_hardware_landing",
        "hardware_ab_control_benefit_test",
    ]
    assert payload["deferred_validation"]["requires_external_tool"] == [
        "n100d_pinocchio_figaroh_validation",
        "n100d_mujoco_moveit_pinocchio_coal_doctor",
    ]


def test_cli_release_notes_reports_v060_rc_summary() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "release", "notes", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.release_notes.v1"
    assert payload["version"] == "0.6.0-rc.25"
    assert payload["title"] == "armctrl 0.6.0-rc.25 simulation safety preview"
    assert "simulation_safety_preview_nonhardware_verified" in payload["summary"]
    assert payload["sections"]["included"] == [
        "governance and safety boundary",
        "offline SysID loop contracts",
        "conservative online identification policy",
        "Agent recipe CLI skill",
        "Top-level agent-flow doctor and durable review replay surfaces",
        "Agent Action Protocol and CLI skill usage manual for preset, EEF, and LeRobot rollout-preparation intents",
        "Fixed Agent CLI simulation experiment for off-center EEF preview, recenter, large EEF control, and LeRobot processor preview",
        "Recipe-to-EEF seed handoff for reviewed preset postures",
        "Recipe preset contract export for Agent-facing handoff",
        "EEF plan, runtime-plan, and shared simulation review contracts",
        "Agent-facing EEF action vocabulary for pose, pose_delta, and twist plans",
        "Top-level Agent session-plan export for realtime EEF loops",
        "Machine-readable Agent-to-LeRobot EEF compatibility contract with pose_delta as the preferred training action",
        "LeRobot-friendly EEF action export for Agent/runtime bridge",
        "ARX5 SDK cartesian bridge export for EEF plans",
        "MoveIt Servo bridge export for twist-style and pose-style EEF plans",
        "Unified runtime bridge export for EEF plans",
        "Runner contract export for EEF backend helpers",
        "Agent runtime contract export for streaming EEF action frames into mature backends",
        "LeRobot agent runtime helper sample for EEF action streams",
        "LeRobot processor contract helper sample for rollout preview closure",
        "CLI-native LeRobot helper-plan and helper-preview commands for Agent callers",
        "Runner preview helper for non-hardware mature-backend closure",
        "Sample runner helper that consumes serialized runner-contract artifacts",
        "CLI-native SDK and MoveIt helper-plan commands for Agent callers",
        "SDK cartesian helper sample script for external runtime integration",
        "MoveIt Servo helper sample script for external runtime integration",
        "EEF synthesize-preview helper for non-hardware joint preview closure",
        "Optional Pink dependency path for Linux EEF preview synthesis",
        "EEF backend trajectory staging into the shared review path",
        "LeRobot processor contract export for EEF runtime helpers",
        "Ordered-step sequencing metadata for LeRobot processor and helper contracts",
        "LeRobot rollout trajectory staging on the shared review path",
        "LeRobot rollout review on the shared simulation gate",
        "LeRobot rollout preview helper for non-hardware EEF-to-rollout closure",
        "LeRobot doctor, native command plans, and dataset metadata bridge",
        "SysID runtime compiler and runtime-owned joint-trajectory submit path",
        "Safety-space config plus simulation doctor and SysID trajectory preview gate",
    ]
    assert "real_sdk_runner_hardware_validation" in payload["sections"]["deferred"]
    assert payload["markdown"].startswith(
        "# armctrl 0.6.0-rc.25 simulation safety preview"
    )
