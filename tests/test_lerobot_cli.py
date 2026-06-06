import json
import subprocess
import sys
from pathlib import Path

import tomllib


def test_pyproject_exposes_lerobot_optional_integration_extra() -> None:
    pyproject = tomllib.loads(Path("pyproject.toml").read_text(encoding="utf-8"))

    extra = pyproject["project"]["optional-dependencies"]["lerobot"]

    assert "lerobot>=0.4.0" in extra
    assert "lerobot-robot-arx5==0.1.2; sys_platform == 'linux'" in extra
    assert "lerobot-teleoperator-arx5==0.1.1; sys_platform == 'linux'" in extra


def test_pyproject_exposes_eef_optional_integration_extra() -> None:
    pyproject = tomllib.loads(Path("pyproject.toml").read_text(encoding="utf-8"))

    extra = pyproject["project"]["optional-dependencies"]["eef"]

    assert "pin-pink>=4.2.0; sys_platform == 'linux'" in extra


def test_cli_lerobot_doctor_is_read_only_and_reports_plugin_status() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "lerobot",
            "doctor",
            "--model",
            "X5",
            "--robot-interface",
            "can0",
            "--teleop-interface",
            "can1",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.lerobot_doctor.v1"
    assert payload["read_only"] is True
    assert payload["movement_allowed"] is False
    assert payload["model"] == "X5"
    assert payload["interfaces"] == {"robot": "can0", "teleop": "can1"}
    assert payload["modules"]["lerobot"]["status"] in {"available", "missing"}
    assert payload["modules"]["lerobot_robot_arx5"]["status"] in {
        "available",
        "missing",
    }
    assert payload["modules"]["lerobot_teleoperator_arx5"]["status"] in {
        "available",
        "missing",
    }
    assert payload["next_gate"] == "run_native_lerobot_cli_outside_armctrl"


def test_cli_lerobot_record_plan_outputs_native_command_without_execution() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "lerobot",
            "config-plan",
            "record",
            "--model",
            "X5",
            "--robot-interface",
            "can0",
            "--teleop-interface",
            "can1",
            "--dataset-repo-id",
            "circlemoon/arx5-test",
            "--task",
            "pick cube",
            "--episodes",
            "3",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.lerobot_config_plan.v1"
    assert payload["plan_only"] is True
    assert payload["movement_allowed"] is False
    assert payload["mode"] == "record"
    assert payload["boundary"] == "native_lerobot_cli"
    assert payload["command"][0] == "lerobot-record"
    assert "--robot.type=arx5_follower" in payload["command"]
    assert "--teleop.type=arx5_leader" in payload["command"]
    assert "--robot.arm_model=X5" in payload["command"]
    assert "--robot.interface_name=can0" in payload["command"]
    assert "--teleop.interface_name=can1" in payload["command"]
    assert "--dataset.repo_id=circlemoon/arx5-test" in payload["command"]
    assert "--dataset.num_episodes=3" in payload["command"]
    assert "--dataset.single_task=pick cube" in payload["command"]


def test_cli_lerobot_train_and_rollout_plans_keep_training_outside_armctrl() -> None:
    train = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "lerobot",
            "config-plan",
            "train",
            "--dataset-repo-id",
            "circlemoon/arx5-test",
            "--policy",
            "act",
            "--output-dir",
            "outputs/train/act_arx5_test",
            "--job-name",
            "act_arx5_test",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    rollout = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "lerobot",
            "config-plan",
            "rollout",
            "--model",
            "X5",
            "--robot-interface",
            "can0",
            "--policy-path",
            "outputs/train/act_arx5_test/checkpoints/last/pretrained_model",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    train_payload = json.loads(train.stdout)
    rollout_payload = json.loads(rollout.stdout)

    assert train_payload["mode"] == "train"
    assert train_payload["command"][0] == "lerobot-train"
    assert "--dataset.repo_id=circlemoon/arx5-test" in train_payload["command"]
    assert "--policy.type=act" in train_payload["command"]
    assert train_payload["movement_allowed"] is False

    assert rollout_payload["mode"] == "rollout"
    assert rollout_payload["command"][0] == "lerobot-rollout"
    assert "--robot.type=arx5_follower" in rollout_payload["command"]
    assert (
        "--policy.path=outputs/train/act_arx5_test/checkpoints/last/pretrained_model"
        in rollout_payload["command"]
    )
    assert rollout_payload["movement_allowed"] is False
    assert rollout_payload["notes"][0].startswith("rollout can move hardware")


def test_cli_lerobot_rollout_plan_can_attach_eef_action_bridge(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    rollout = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "lerobot",
            "config-plan",
            "rollout",
            "--model",
            "X5",
            "--robot-interface",
            "can0",
            "--policy-path",
            "outputs/train/act_arx5_test/checkpoints/last/pretrained_model",
            "--eef-plan-dir",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(rollout.stdout)

    assert payload["status"] == "ok"
    assert payload["mode"] == "rollout"
    assert payload["eef_plan_dir"] == str(plan_dir)
    assert payload["eef_action_bridge"]["control_mode"] == "cartesian_pose_absolute"
    assert payload["eef_action_bridge"]["action_dict"]["eef.z"] == 0.2
    assert payload["runtime"]["owner"] == "native_lerobot_rollout"
    assert payload["processor_bridge"]["schema"] == "armctrl.lerobot_rollout_processor_bridge.v1"
    assert payload["processor_bridge"]["programmatic_api"]["action_hook"] == "robot_action_processor"
    assert (
        payload["processor_bridge"]["programmatic_api"]["observation_hook"]
        == "robot_observation_processor"
    )
    assert payload["processor_bridge"]["lerobot_action"]["control_mode"] == "cartesian_pose_absolute"
    assert any("huggingface.co/docs/lerobot" in ref for ref in payload["processor_bridge"]["reference_docs"])
    assert payload["artifact_contract"]["review_artifact"] == "backend_joint_trajectory.csv"
    assert payload["backend_review_contract"]["schema"] == "armctrl.eef_backend_review_contract.v1"
    assert any("stage-rollout-trajectory" in step for step in payload["next_steps"])
    assert any("review-rollout" in step for step in payload["next_steps"])
    assert any("aligned to the exported EEF LeRobot action contract" in note for note in payload["notes"])
    assert any("review-rollout" in note for note in payload["notes"])


def test_cli_lerobot_export_processor_contract_from_eef_plan(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
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
            "lerobot",
            "export-processor-contract",
            "--eef-plan-dir",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.lerobot_eef_processor_contract.v1"
    assert payload["lerobot_action"]["control_mode"] == "cartesian_pose_absolute"
    assert payload["lerobot_action"]["agent_eef_compatibility"]["status"] == "compatible"
    assert (
        payload["lerobot_action"]["agent_eef_compatibility"]["agent_action_id"]
        == "eef.pose_absolute"
    )
    assert (
        payload["lerobot_action"]["agent_eef_compatibility"]["preferred_training_action_id"]
        == "eef.pose_delta"
    )
    assert (
        payload["lerobot_action"]["agent_eef_compatibility"]["processor_owner"]["action"]
        == "robot_action_processor"
    )
    assert payload["processor_bridge"]["programmatic_api"]["action_hook"] == "robot_action_processor"
    assert (
        payload["processor_bridge"]["programmatic_api"]["observation_hook"]
        == "robot_observation_processor"
    )
    assert payload["runner_contract"]["schema"] == "armctrl.eef_runner_contract.v1"
    assert payload["runner_contract"]["resolved_backend"] == "lerobot_rollout"
    assert payload["ordered_steps"][0]["id"] == "export_runner_contract"
    assert payload["ordered_steps"][1]["id"] == "stage_rollout_trajectory"
    assert payload["ordered_steps"][1]["depends_on"] == ["export_runner_contract"]
    assert payload["ordered_steps"][2]["id"] == "review_rollout"
    assert payload["ordered_steps"][2]["depends_on"] == ["stage_rollout_trajectory"]
    assert any("stage-rollout-trajectory" in step for step in payload["next_steps"])
    assert any("review-rollout" in step for step in payload["next_steps"])


def test_cli_eef_export_agent_runtime_contract_for_lerobot_backend(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-lerobot-agent-runtime-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
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
            "eef",
            "export-agent-runtime-contract",
            "--plan-dir",
            str(plan_dir),
            "--backend",
            "lerobot_rollout",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_agent_runtime_contract.v1"
    assert payload["resolved_backend"] == "lerobot_rollout"
    assert payload["runtime_owner"] == "native_lerobot_rollout"
    assert payload["agent_action_schema"]["action_id"] == "eef.pose_absolute"
    assert payload["backend_session_contract"]["session_kind"] == "native_lerobot_rollout"
    assert payload["backend_session_contract"]["action_hook"] == "robot_action_processor"


def test_lerobot_agent_runtime_helper_sample_consumes_agent_runtime_contract(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-lerobot-agent-runtime-helper-plan"
    contract_path = tmp_path / "eef_agent_runtime_contract.json"
    output_path = tmp_path / "lerobot_agent_runtime_helper_plan.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-delta-pose",
            "--frame",
            "eef_link",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-agent-runtime-contract",
            "--plan-dir",
            str(plan_dir),
            "--backend",
            "lerobot_rollout",
            "--output",
            str(contract_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/lerobot_agent_runtime_helper_sample.py",
            "--agent-runtime-contract",
            str(contract_path),
            "--output",
            str(output_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(output_path.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.lerobot_agent_runtime_helper_plan.v1"
    assert payload["movement_allowed"] is False
    assert payload["resolved_backend"] == "lerobot_rollout"
    assert payload["runtime_owner"] == "native_lerobot_rollout"
    assert payload["agent_action_schema"]["action_id"] == "eef.pose_delta"
    assert payload["processor_session_plan"]["action_hook"] == "robot_action_processor"
    assert payload["processor_session_plan"]["observation_hook"] == "robot_observation_processor"
    assert payload["processor_session_plan"]["control_mode"] == "cartesian_delta"
    assert payload["review_output_contract"]["expected_trajectory_path"].endswith(
        "backend_joint_trajectory.csv"
    )
    assert payload["ordered_steps"][0]["id"] == "export_processor_contract"
    assert payload["ordered_steps"][1]["id"] == "preview_rollout"
    assert payload["ordered_steps"][1]["depends_on"] == ["export_processor_contract"]
    assert payload["ordered_steps"][2]["id"] == "review_rollout"
    assert payload["ordered_steps"][2]["depends_on"] == ["preview_rollout"]
    assert any("preview-rollout" in step for step in payload["next_steps"])
    assert written["schema"] == "armctrl.lerobot_agent_runtime_helper_plan.v1"


def test_cli_lerobot_agent_runtime_helper_plan_consumes_agent_runtime_contract(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-lerobot-agent-runtime-cli-plan"
    contract_path = tmp_path / "eef_agent_runtime_contract.json"
    output_path = tmp_path / "lerobot_agent_runtime_helper_plan.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-delta-pose",
            "--frame",
            "eef_link",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-agent-runtime-contract",
            "--plan-dir",
            str(plan_dir),
            "--backend",
            "lerobot_rollout",
            "--output",
            str(contract_path),
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
            "lerobot",
            "agent-runtime-helper-plan",
            "--agent-runtime-contract",
            str(contract_path),
            "--output",
            str(output_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(output_path.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.lerobot_agent_runtime_helper_plan.v1"
    assert payload["resolved_backend"] == "lerobot_rollout"
    assert payload["agent_action_schema"]["action_id"] == "eef.pose_delta"
    assert payload["ordered_steps"][0]["id"] == "export_processor_contract"
    assert payload["ordered_steps"][1]["depends_on"] == ["export_processor_contract"]
    assert payload["ordered_steps"][2]["depends_on"] == ["preview_rollout"]
    assert written["schema"] == "armctrl.lerobot_agent_runtime_helper_plan.v1"


def test_lerobot_agent_runtime_helper_sample_rejects_non_lerobot_backend(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-sdk-agent-runtime-reject-plan"
    contract_path = tmp_path / "eef_agent_runtime_contract.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-agent-runtime-contract",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/lerobot_agent_runtime_helper_sample.py",
            "--agent-runtime-contract",
            str(contract_path),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["error"]["code"] == "unsupported_runtime_backend"
    assert payload["resolved_backend"] == "sdk_cartesian"


def test_cli_lerobot_agent_runtime_helper_plan_rejects_non_lerobot_backend(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-sdk-agent-runtime-cli-reject-plan"
    contract_path = tmp_path / "eef_agent_runtime_contract.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-agent-runtime-contract",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
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
            "lerobot",
            "agent-runtime-helper-plan",
            "--agent-runtime-contract",
            str(contract_path),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["error"]["code"] == "unsupported_runtime_backend"


def test_lerobot_processor_contract_helper_sample_closes_preview_chain(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-lerobot-processor-helper-plan"
    contract_path = tmp_path / "lerobot_processor_contract.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "lerobot",
            "export-processor-contract",
            "--eef-plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/lerobot_processor_contract_helper_sample.py",
            "--processor-contract",
            str(contract_path),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.lerobot_processor_helper_preview.v1"
    assert payload["movement_allowed"] is False
    assert payload["processor_contract_path"] == str(contract_path)
    assert payload["consumed_processor_contract"]["schema"] == "armctrl.lerobot_eef_processor_contract.v1"
    assert payload["resolved_backend"] == "lerobot_rollout"
    assert payload["review_status"] == "completed"
    assert payload["sim_preview"]["safety"]["allowed"] is True
    assert payload["synthesized_trajectory"].endswith("backend_joint_trajectory.csv")
    assert payload["ordered_steps"][0]["id"] == "preview_rollout"
    assert payload["ordered_steps"][1]["id"] == "review_rollout"
    assert payload["ordered_steps"][1]["depends_on"] == ["preview_rollout"]


def test_cli_lerobot_processor_helper_preview_closes_preview_chain(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-lerobot-processor-cli-plan"
    contract_path = tmp_path / "lerobot_processor_contract.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "lerobot",
            "export-processor-contract",
            "--eef-plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
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
            "lerobot",
            "processor-helper-preview",
            "--processor-contract",
            str(contract_path),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.lerobot_processor_helper_preview.v1"
    assert payload["review_status"] == "completed"
    assert payload["ordered_steps"][0]["id"] == "preview_rollout"
    assert payload["ordered_steps"][1]["depends_on"] == ["preview_rollout"]


def test_lerobot_processor_contract_helper_sample_rejects_invalid_contract(
    tmp_path: Path,
) -> None:
    invalid_contract = tmp_path / "invalid_processor_contract.json"
    invalid_contract.write_text(
        json.dumps({"schema": "wrong.schema.v1"}, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/lerobot_processor_contract_helper_sample.py",
            "--processor-contract",
            str(invalid_contract),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["error"]["code"] == "invalid_processor_contract"


def test_cli_lerobot_processor_helper_preview_rejects_invalid_contract(
    tmp_path: Path,
) -> None:
    invalid_contract = tmp_path / "invalid_processor_contract.json"
    invalid_contract.write_text(
        json.dumps({"schema": "wrong.schema.v1"}, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "lerobot",
            "processor-helper-preview",
            "--processor-contract",
            str(invalid_contract),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["error"]["code"] == "invalid_processor_contract"


def test_cli_lerobot_export_processor_contract_rejects_missing_plan_artifacts(
    tmp_path: Path,
) -> None:
    missing_dir = tmp_path / "missing-plan"
    missing_dir.mkdir()

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "lerobot",
            "export-processor-contract",
            "--eef-plan-dir",
            str(missing_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.lerobot_eef_processor_contract.v1"
    assert payload["error"]["code"] == "missing_eef_plan_artifacts"
    assert "backend_request.json" in payload["error"]["message"]


def test_cli_lerobot_review_rollout_reuses_eef_simulation_chain(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-plan"
    trajectory_path = tmp_path / "backend_joint_trajectory.csv"
    trajectory_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.100000,0.020000,0.320000,0.320000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
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
            "lerobot",
            "review-rollout",
            "--eef-plan-dir",
            str(plan_dir),
            "--trajectory",
            str(trajectory_path),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.lerobot_rollout_review.v1"
    assert payload["review_status"] == "completed"
    assert payload["delegated_review"]["schema"] == "armctrl.eef_review.v1"
    assert payload["sim_preview"]["safety"]["allowed"] is True


def test_cli_lerobot_review_rollout_reports_deferred_when_backend_trajectory_missing(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
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
            "lerobot",
            "review-rollout",
            "--eef-plan-dir",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["review_status"] == "deferred"
    assert payload["delegated_review"]["review_status"] == "deferred"


def test_cli_lerobot_review_rollout_auto_discovers_default_backend_trajectory(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    trajectory_path = plan_dir / "backend_joint_trajectory.csv"
    trajectory_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.100000,0.020000,0.320000,0.320000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "lerobot",
            "review-rollout",
            "--eef-plan-dir",
            str(plan_dir),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["review_status"] == "completed"
    assert payload["delegated_review"]["reviewed_trajectory"] == str(trajectory_path)


def test_cli_lerobot_stage_rollout_trajectory_reuses_eef_staging_contract(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-plan-stage"
    source_trajectory = tmp_path / "rollout_joint_trajectory.csv"
    source_trajectory.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.100000,0.010000,0.310000,0.310000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    staged = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "lerobot",
            "stage-rollout-trajectory",
            "--eef-plan-dir",
            str(plan_dir),
            "--trajectory",
            str(source_trajectory),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(staged.stdout)
    staged_path = plan_dir / "backend_joint_trajectory.csv"

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.lerobot_rollout_stage_trajectory.v1"
    assert payload["eef_plan_dir"] == str(plan_dir)
    assert payload["delegated_stage"]["schema"] == "armctrl.eef_stage_trajectory.v1"
    assert payload["delegated_stage"]["staged_trajectory"] == str(staged_path)
    assert staged_path.read_text(encoding="utf-8") == source_trajectory.read_text(
        encoding="utf-8"
    )


def test_cli_lerobot_preview_rollout_closes_nonhardware_eef_loop(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-plan-preview"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
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
            "lerobot",
            "preview-rollout",
            "--eef-plan-dir",
            str(plan_dir),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.lerobot_rollout_preview.v1"
    assert payload["movement_allowed"] is False
    assert payload["processor_contract"]["schema"] == "armctrl.lerobot_eef_processor_contract.v1"
    assert payload["delegated_preview"]["schema"] == "armctrl.eef_preview_synthesis.v1"
    assert payload["review_status"] == "completed"
    assert payload["delegated_preview"]["review"]["review_status"] == "completed"
    assert payload["sim_preview"]["safety"]["allowed"] is True
    assert payload["synthesized_trajectory"].endswith("backend_joint_trajectory.csv")
    assert any("review-rollout" in step for step in payload["next_steps"])


def test_cli_lerobot_export_metadata_writes_bridge_file(tmp_path: Path) -> None:
    output = tmp_path / "lerobot_metadata.json"
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "lerobot",
            "export-metadata",
            "--dataset-repo-id",
            "circlemoon/arx5-test",
            "--parameter-bundle",
            "runs/sysid/processed/parameter_package.json",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--output",
            str(output),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(output.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.lerobot_metadata_export.v1"
    assert payload["output"] == str(output)
    assert written["schema"] == "armctrl.lerobot_metadata.v1"
    assert written["dataset_repo_id"] == "circlemoon/arx5-test"
    assert written["parameter_bundle"] == "runs/sysid/processed/parameter_package.json"
    assert written["safe_config"] == "configs/x5.safe.yaml"
    assert written["ownership"]["record_train_rollout"] == "LeRobot"
    assert written["ownership"]["safety_and_sysid"] == "armctrl"
