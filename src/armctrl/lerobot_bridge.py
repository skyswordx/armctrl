from __future__ import annotations

import importlib.util
import json
from dataclasses import dataclass
from pathlib import Path


def _module_status(module: str) -> dict[str, str]:
    return {
        "module": module,
        "status": "available" if importlib.util.find_spec(module) else "missing",
    }


@dataclass(frozen=True)
class LeRobotDoctor:
    model: str
    robot_interface: str
    teleop_interface: str

    def to_json(self) -> dict[str, object]:
        return {
            "schema": "armctrl.lerobot_doctor.v1",
            "read_only": True,
            "movement_allowed": False,
            "model": self.model,
            "interfaces": {
                "robot": self.robot_interface,
                "teleop": self.teleop_interface,
            },
            "modules": {
                "lerobot": _module_status("lerobot"),
                "lerobot_robot_arx5": _module_status("lerobot_robot_arx5"),
                "lerobot_teleoperator_arx5": _module_status(
                    "lerobot_teleoperator_arx5"
                ),
                "arx5_interface": _module_status("arx5_interface"),
            },
            "next_gate": "run_native_lerobot_cli_outside_armctrl",
            "notes": [
                "doctor checks Python importability only",
                "it does not connect to CAN, cameras, Hugging Face, or robot hardware",
            ],
        }


@dataclass(frozen=True)
class LeRobotConfigPlanRequest:
    mode: str
    model: str = "X5"
    robot_interface: str = "can0"
    teleop_interface: str = "can1"
    dataset_repo_id: str | None = None
    task: str | None = None
    episodes: int = 10
    policy: str = "act"
    output_dir: str = "outputs/train/act_arx5"
    job_name: str = "act_arx5"
    policy_path: str | None = None


class LeRobotConfigPlanner:
    def plan(self, request: LeRobotConfigPlanRequest) -> dict[str, object]:
        if request.mode == "record":
            command = [
                "lerobot-record",
                "--robot.type=arx5_follower",
                f"--robot.arm_model={request.model}",
                f"--robot.interface_name={request.robot_interface}",
                "--teleop.type=arx5_leader",
                f"--teleop.arm_model={request.model}",
                f"--teleop.interface_name={request.teleop_interface}",
                f"--dataset.repo_id={request.dataset_repo_id}",
                f"--dataset.num_episodes={request.episodes}",
                f"--dataset.single_task={request.task}",
            ]
            notes = [
                "record can move hardware when the native LeRobot command is run",
                "armctrl only emits the command plan and does not execute it",
            ]
        elif request.mode == "train":
            command = [
                "lerobot-train",
                f"--dataset.repo_id={request.dataset_repo_id}",
                f"--policy.type={request.policy}",
                f"--output_dir={request.output_dir}",
                f"--job_name={request.job_name}",
            ]
            notes = [
                "training is owned by LeRobot and does not move hardware",
                "armctrl keeps only dataset and parameter-bundle metadata bridges",
            ]
        elif request.mode == "rollout":
            command = [
                "lerobot-rollout",
                "--robot.type=arx5_follower",
                f"--robot.arm_model={request.model}",
                f"--robot.interface_name={request.robot_interface}",
                f"--policy.path={request.policy_path}",
            ]
            notes = [
                "rollout can move hardware when the native LeRobot command is run",
                "add an armctrl safety bridge later only if policy actions must be bounded before hardware",
            ]
        else:
            raise ValueError(f"unsupported LeRobot config-plan mode: {request.mode}")

        return {
            "schema": "armctrl.lerobot_config_plan.v1",
            "mode": request.mode,
            "plan_only": True,
            "movement_allowed": False,
            "boundary": "native_lerobot_cli",
            "command": command,
            "notes": notes,
        }


@dataclass(frozen=True)
class LeRobotMetadataExport:
    dataset_repo_id: str
    parameter_bundle: str
    safe_config: str
    output: Path


class LeRobotMetadataExporter:
    def run(self, request: LeRobotMetadataExport) -> dict[str, object]:
        payload = {
            "schema": "armctrl.lerobot_metadata.v1",
            "dataset_repo_id": request.dataset_repo_id,
            "parameter_bundle": request.parameter_bundle,
            "safe_config": request.safe_config,
            "ownership": {
                "record_train_rollout": "LeRobot",
                "safety_and_sysid": "armctrl",
            },
            "notes": [
                "metadata is a bridge between LeRobot datasets and armctrl SysID packages",
                "this file is not a LeRobot dataset replacement",
            ],
        }
        request.output.parent.mkdir(parents=True, exist_ok=True)
        request.output.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return {
            "schema": "armctrl.lerobot_metadata_export.v1",
            "output": str(request.output),
            "metadata": payload,
        }
