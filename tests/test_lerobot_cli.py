import json
import subprocess
import sys
from pathlib import Path


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
