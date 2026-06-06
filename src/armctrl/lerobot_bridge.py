from __future__ import annotations

import importlib.util
import json
from dataclasses import dataclass
from pathlib import Path

from armctrl.eef import (
    EefLeRobotExportRequest,
    EefLeRobotExporter,
    EefPreviewSynthesisRequest,
    EefPreviewSynthesizer,
    EefReviewRequest,
    EefReviewer,
    EefRunnerContractExporter,
    EefRunnerContractRequest,
    EefStageTrajectoryRequest,
    EefTrajectoryStager,
)


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
    eef_plan_dir: Path | None = None


@dataclass(frozen=True)
class LeRobotRolloutReviewRequest:
    eef_plan_dir: Path
    urdf_path: Path
    safe_config_path: Path
    trajectory_path: Path | None = None
    render_path: Path | None = None


@dataclass(frozen=True)
class LeRobotRolloutStageRequest:
    eef_plan_dir: Path
    trajectory_path: Path


@dataclass(frozen=True)
class LeRobotRolloutPreviewRequest:
    eef_plan_dir: Path
    urdf_path: Path
    safe_config_path: Path
    model: str = "X5"
    robot_interface: str = "can0"
    render_path: Path | None = None
    recipe_plan_dir: Path | None = None
    start_joints: tuple[float, ...] | None = None
    sample_hz: float = 50.0
    duration_s: float = 2.0


@dataclass(frozen=True)
class LeRobotProcessorContractRequest:
    eef_plan_dir: Path
    model: str = "X5"
    robot_interface: str = "can0"
    output_path: Path | None = None


@dataclass(frozen=True)
class LeRobotAgentRuntimeHelperPlanRequest:
    agent_runtime_contract_path: Path
    output_path: Path | None = None


@dataclass(frozen=True)
class LeRobotProcessorHelperPreviewRequest:
    processor_contract_path: Path
    urdf_path: Path
    safe_config_path: Path
    render_path: Path | None = None


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
                "armctrl should keep owning safety-gated EEF planning and reviewed joint-trajectory previews outside the native LeRobot runtime",
            ]
            eef_action_bridge = None
            if request.eef_plan_dir is not None:
                export_payload = EefLeRobotExporter().export(
                    EefLeRobotExportRequest(plan_dir=request.eef_plan_dir)
                )
                eef_action_bridge = export_payload["lerobot_action"]
                notes.append(
                    "This rollout plan is aligned to the exported EEF LeRobot action contract from armctrl eef plan artifacts."
                )
                notes.append(
                    "Use armctrl lerobot review-rollout with the same --eef-plan-dir to reuse the shared simulation preview gate before any execution discussion."
                )
        else:
            raise ValueError(f"unsupported LeRobot config-plan mode: {request.mode}")
        payload = {
            "schema": "armctrl.lerobot_config_plan.v1",
            "mode": request.mode,
            "plan_only": True,
            "movement_allowed": False,
            "boundary": "native_lerobot_cli",
            "command": command,
            "notes": notes,
        }
        if request.mode == "rollout" and request.eef_plan_dir is not None:
            backend_review_contract = json.loads(
                (request.eef_plan_dir / "backend_review_contract.json").read_text(
                    encoding="utf-8"
                )
            )
            payload["eef_plan_dir"] = str(request.eef_plan_dir)
            payload["eef_action_bridge"] = eef_action_bridge
            payload["runtime"] = {
                "owner": "native_lerobot_rollout",
                "session_kind": "native_lerobot_cli",
                "api_contract": {
                    "command_surface": "lerobot-rollout",
                    "action_alignment": "cartesian action mapping should follow exported EEF LeRobot action contract",
                },
            }
            payload["processor_bridge"] = {
                "schema": "armctrl.lerobot_rollout_processor_bridge.v1",
                "lerobot_action": eef_action_bridge,
                "programmatic_api": {
                    "entrypoint_family": "lerobot.rollout",
                    "action_hook": "robot_action_processor",
                    "observation_hook": "robot_observation_processor",
                    "purpose": "adapt EEF-oriented LeRobot action samples into robot-native rollout actions without changing armctrl plan artifacts",
                },
                "reference_docs": [
                    "https://huggingface.co/docs/lerobot/main/inference",
                    "https://huggingface.co/docs/lerobot/main/il_robots",
                ],
                "notes": [
                    "Use the exported EEF LeRobot action contract as the upstream action vocabulary for a custom robot_action_processor.",
                    "The processor should translate cartesian pose or delta actions into the rollout runtime's robot-native command surface.",
                ],
            }
            payload["artifact_contract"] = {
                "review_artifact": "backend_joint_trajectory.csv",
                "staging_command": (
                    "armctrl lerobot stage-rollout-trajectory --eef-plan-dir <dir> "
                    "--trajectory <joint_csv> --json"
                ),
                "review_command": (
                    "armctrl lerobot review-rollout --eef-plan-dir <dir> "
                    "[--trajectory <path>] --json"
                ),
            }
            payload["backend_review_contract"] = backend_review_contract
            payload["ordered_steps"] = [
                {
                    "id": "stage_rollout_trajectory",
                    "command": (
                        "uv run armctrl lerobot stage-rollout-trajectory "
                        f"--eef-plan-dir {request.eef_plan_dir} --trajectory <joint_csv> --json"
                    ),
                },
                {
                    "id": "review_rollout",
                    "depends_on": ["stage_rollout_trajectory"],
                    "command": (
                        "uv run armctrl lerobot review-rollout "
                        f"--eef-plan-dir {request.eef_plan_dir} --json"
                    ),
                },
            ]
            payload["next_steps"] = [
                f"uv run armctrl lerobot stage-rollout-trajectory --eef-plan-dir {request.eef_plan_dir} --trajectory <joint_csv> --json",
                f"uv run armctrl lerobot review-rollout --eef-plan-dir {request.eef_plan_dir} --json",
            ]
        return payload


class LeRobotProcessorContractExporter:
    def export(self, request: LeRobotProcessorContractRequest) -> dict[str, object]:
        required_artifacts = [
            request.eef_plan_dir / "backend_request.json",
            request.eef_plan_dir / "eef_plan.json",
            request.eef_plan_dir / "manifest.json",
            request.eef_plan_dir / "backend_review_contract.json",
        ]
        missing = [str(path) for path in required_artifacts if not path.exists()]
        if missing:
            raise FileNotFoundError(
                "missing eef plan artifacts required for lerobot processor contract: "
                + ", ".join(missing)
            )
        eef_export = EefLeRobotExporter().export(
            EefLeRobotExportRequest(plan_dir=request.eef_plan_dir)
        )
        runner_contract = EefRunnerContractExporter().export(
            EefRunnerContractRequest(
                plan_dir=request.eef_plan_dir,
                backend="lerobot_rollout",
                model=request.model,
                interface=request.robot_interface,
            )
        )
        payload = {
            "schema": "armctrl.lerobot_eef_processor_contract.v1",
            "movement_allowed": False,
            "eef_plan_dir": str(request.eef_plan_dir),
            "lerobot_action": eef_export["lerobot_action"],
            "processor_bridge": {
                "schema": "armctrl.lerobot_rollout_processor_bridge.v1",
                "lerobot_action": eef_export["lerobot_action"],
                "programmatic_api": {
                    "entrypoint_family": "lerobot.rollout",
                    "action_hook": "robot_action_processor",
                    "observation_hook": "robot_observation_processor",
                    "purpose": "adapt EEF-oriented LeRobot action samples into robot-native rollout actions while keeping armctrl as the safety-gated orchestration layer",
                },
                "reference_docs": [
                    "https://huggingface.co/docs/lerobot/main/inference",
                    "https://huggingface.co/docs/lerobot/main/il_robots",
                ],
                "notes": [
                    "Use the exported EEF LeRobot action contract as the upstream action vocabulary for a custom robot_action_processor.",
                    "Use robot_observation_processor to project rollout observations back into the same EEF-oriented semantics when the policy loop needs symmetric preprocessing.",
                ],
            },
            "runner_contract": runner_contract,
            "ordered_steps": [
                {
                    "id": "export_runner_contract",
                    "command": (
                        "uv run armctrl eef export-runner-contract "
                        f"--plan-dir {request.eef_plan_dir} --backend lerobot_rollout --json"
                    ),
                },
                {
                    "id": "stage_rollout_trajectory",
                    "depends_on": ["export_runner_contract"],
                    "command": (
                        "uv run armctrl lerobot stage-rollout-trajectory "
                        f"--eef-plan-dir {request.eef_plan_dir} --trajectory <joint_csv> --json"
                    ),
                },
                {
                    "id": "review_rollout",
                    "depends_on": ["stage_rollout_trajectory"],
                    "command": (
                        "uv run armctrl lerobot review-rollout "
                        f"--eef-plan-dir {request.eef_plan_dir} --json"
                    ),
                },
            ],
            "next_steps": [
                f"uv run armctrl eef export-runner-contract --plan-dir {request.eef_plan_dir} --backend lerobot_rollout --json",
                f"uv run armctrl lerobot stage-rollout-trajectory --eef-plan-dir {request.eef_plan_dir} --trajectory <joint_csv> --json",
                f"uv run armctrl lerobot review-rollout --eef-plan-dir {request.eef_plan_dir} --json",
            ],
            "notes": [
                "This is a non-executing handoff contract for programmatic LeRobot rollout helpers that consume EEF-shaped actions and return a reviewed joint trajectory to armctrl.",
                "armctrl still does not own LeRobot runtime execution, IK, or robot-native joint synthesis.",
            ],
        }
        if request.output_path is not None:
            request.output_path.parent.mkdir(parents=True, exist_ok=True)
            request.output_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            payload["output"] = str(request.output_path)
        return payload


class LeRobotAgentRuntimeHelperPlanner:
    def plan(self, request: LeRobotAgentRuntimeHelperPlanRequest) -> dict[str, object]:
        agent_runtime_contract = json.loads(
            request.agent_runtime_contract_path.read_text(encoding="utf-8")
        )
        if (
            agent_runtime_contract.get("schema")
            != "armctrl.eef_agent_runtime_contract.v1"
        ):
            raise ValueError(
                "agent runtime contract file must use schema "
                "armctrl.eef_agent_runtime_contract.v1"
            )
        resolved_backend = agent_runtime_contract.get("resolved_backend")
        if resolved_backend != "lerobot_rollout":
            raise RuntimeError(
                "unsupported agent runtime backend for lerobot helper sample: "
                f"{resolved_backend}"
            )
        backend_session_contract = dict(agent_runtime_contract["backend_session_contract"])
        bridge_export = dict(agent_runtime_contract["bridge_export"])
        lerobot_action = dict(bridge_export["lerobot_action"])
        agent_action_schema = dict(agent_runtime_contract["agent_action_schema"])
        payload = {
            "schema": "armctrl.lerobot_agent_runtime_helper_plan.v1",
            "movement_allowed": False,
            "agent_runtime_contract_path": str(request.agent_runtime_contract_path),
            "plan_dir": str(agent_runtime_contract["plan_dir"]),
            "resolved_backend": resolved_backend,
            "runtime_owner": agent_runtime_contract["runtime_owner"],
            "agent_action_schema": agent_action_schema,
            "processor_session_plan": {
                "session_kind": backend_session_contract["session_kind"],
                "control_mode": lerobot_action["control_mode"],
                "preferred_native_surface": lerobot_action["preferred_native_surface"],
                "feature_order": lerobot_action["feature_order"],
                "action_features": lerobot_action["action_features"],
                "action_hook": backend_session_contract["action_hook"],
                "observation_hook": backend_session_contract["observation_hook"],
                "action_stream_schema": {
                    "stream_mode": agent_action_schema["stream_mode"],
                    "frame": agent_action_schema["frame"],
                    "action_id": agent_action_schema["action_id"],
                },
                "reference_docs": [
                    "https://huggingface.co/docs/lerobot/main/inference",
                    "https://huggingface.co/docs/lerobot/main/il_robots",
                    "https://huggingface.co/docs/lerobot/introduction_processors",
                ],
            },
            "lerobot_action": lerobot_action,
            "review_output_contract": agent_runtime_contract["review_output_contract"],
            "ordered_steps": [
                {
                    "id": "export_processor_contract",
                    "command": (
                        "uv run armctrl lerobot export-processor-contract "
                        f"--eef-plan-dir {agent_runtime_contract['plan_dir']} --json"
                    ),
                },
                {
                    "id": "preview_rollout",
                    "depends_on": ["export_processor_contract"],
                    "command": (
                        "uv run armctrl lerobot preview-rollout "
                        f"--eef-plan-dir {agent_runtime_contract['plan_dir']} --json"
                    ),
                },
                {
                    "id": "review_rollout",
                    "depends_on": ["preview_rollout"],
                    "command": (
                        "uv run armctrl lerobot review-rollout "
                        f"--eef-plan-dir {agent_runtime_contract['plan_dir']} --json"
                    ),
                },
            ],
            "next_steps": [
                (
                    "uv run armctrl lerobot preview-rollout "
                    f"--eef-plan-dir {agent_runtime_contract['plan_dir']} --json"
                ),
                (
                    "uv run armctrl lerobot export-processor-contract "
                    f"--eef-plan-dir {agent_runtime_contract['plan_dir']} --json"
                ),
                (
                    "uv run armctrl lerobot review-rollout "
                    f"--eef-plan-dir {agent_runtime_contract['plan_dir']} --json"
                ),
            ],
            "notes": [
                "This is a non-hardware CLI helper plan for consuming armctrl.eef_agent_runtime_contract.v1.",
                "It does not import lerobot, start rollout execution, or move hardware.",
                "Use ordered_steps as the machine-readable sequence boundary so export, preview, and review are not parallelized by accident.",
                "Use the generated processor session plan as the boundary artifact for a future mature LeRobot runtime helper.",
            ],
        }
        if request.output_path is not None:
            request.output_path.parent.mkdir(parents=True, exist_ok=True)
            request.output_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            payload["output"] = str(request.output_path)
        return payload


class LeRobotProcessorHelperPreviewer:
    def preview(self, request: LeRobotProcessorHelperPreviewRequest) -> dict[str, object]:
        processor_contract = json.loads(
            request.processor_contract_path.read_text(encoding="utf-8")
        )
        if (
            processor_contract.get("schema")
            != "armctrl.lerobot_eef_processor_contract.v1"
        ):
            raise ValueError(
                "processor contract file must use schema "
                "armctrl.lerobot_eef_processor_contract.v1"
            )
        runner_contract = dict(processor_contract["runner_contract"])
        resolved_backend = runner_contract.get("resolved_backend")
        if resolved_backend != "lerobot_rollout":
            raise RuntimeError(
                "unsupported processor runner backend for lerobot helper sample: "
                f"{resolved_backend}"
            )
        eef_plan_dir = Path(str(processor_contract["eef_plan_dir"]))
        preview = LeRobotRolloutPreviewer().preview(
            LeRobotRolloutPreviewRequest(
                eef_plan_dir=eef_plan_dir,
                urdf_path=request.urdf_path,
                safe_config_path=request.safe_config_path,
                render_path=request.render_path,
            )
        )
        return {
            "schema": "armctrl.lerobot_processor_helper_preview.v1",
            "movement_allowed": False,
            "processor_contract_path": str(request.processor_contract_path),
            "consumed_processor_contract": processor_contract,
            "eef_plan_dir": str(eef_plan_dir),
            "resolved_backend": resolved_backend,
            "review_status": preview["review_status"],
            "sim_preview": preview.get("sim_preview"),
            "synthesized_trajectory": preview["synthesized_trajectory"],
            "delegated_preview": preview,
            "ordered_steps": [
                {
                    "id": "preview_rollout",
                    "command": (
                        "uv run armctrl lerobot preview-rollout "
                        f"--eef-plan-dir {eef_plan_dir} --urdf-path {request.urdf_path} "
                        f"--safe-config {request.safe_config_path} --json"
                    ),
                },
                {
                    "id": "review_rollout",
                    "depends_on": ["preview_rollout"],
                    "command": (
                        "uv run armctrl lerobot review-rollout "
                        f"--eef-plan-dir {eef_plan_dir} --json"
                    ),
                },
            ],
            "next_steps": preview["next_steps"],
            "notes": [
                "This is a non-hardware CLI helper preview for consuming armctrl.lerobot_eef_processor_contract.v1.",
                "It reuses the shared rollout preview/review chain instead of starting LeRobot execution.",
                "Use ordered_steps as the machine-readable sequence boundary so preview stays ahead of review.",
                "Use this as a boundary sample before wiring a mature rollout-side helper that emits reviewed joint trajectories from real policy/runtime output.",
            ],
        }


class LeRobotRolloutReviewer:
    def review(self, request: LeRobotRolloutReviewRequest) -> dict[str, object]:
        trajectory_path = request.trajectory_path
        if trajectory_path is None:
            candidate = request.eef_plan_dir / "backend_review_contract.json"
            if candidate.exists():
                review_contract = json.loads(candidate.read_text(encoding="utf-8"))
                candidate_trajectory = Path(
                    review_contract["expected_backend_trajectory"]
                )
                if candidate_trajectory.exists():
                    trajectory_path = candidate_trajectory
        delegated = EefReviewer().review(
            EefReviewRequest(
                plan_dir=request.eef_plan_dir,
                urdf_path=request.urdf_path,
                safe_config_path=request.safe_config_path,
                trajectory_path=trajectory_path,
                render_path=request.render_path,
            )
        )
        return {
            "schema": "armctrl.lerobot_rollout_review.v1",
            "movement_allowed": False,
            "eef_plan_dir": str(request.eef_plan_dir),
            "delegated_review": delegated,
            "review_status": delegated["review_status"],
            "sim_preview": delegated.get("sim_preview"),
            "next_gate": delegated["next_gate"],
            "notes": [
                "LeRobot rollout review reuses the shared EEF simulation preview chain instead of introducing a separate safety oracle.",
                "The native rollout runtime still remains outside armctrl; this command only evaluates the reviewed joint trajectory gate.",
            ],
        }


class LeRobotRolloutStager:
    def stage(self, request: LeRobotRolloutStageRequest) -> dict[str, object]:
        delegated = EefTrajectoryStager().stage(
            EefStageTrajectoryRequest(
                plan_dir=request.eef_plan_dir,
                trajectory_path=request.trajectory_path,
            )
        )
        return {
            "schema": "armctrl.lerobot_rollout_stage_trajectory.v1",
            "movement_allowed": False,
            "eef_plan_dir": str(request.eef_plan_dir),
            "delegated_stage": delegated,
            "next_gate": "run armctrl lerobot review-rollout --eef-plan-dir <dir> --json",
            "notes": [
                "LeRobot rollout staging reuses the shared EEF backend trajectory contract instead of introducing a second artifact layout.",
                "This command only stages mature-backend rollout output for later simulation review.",
            ],
        }


class LeRobotRolloutPreviewer:
    def preview(self, request: LeRobotRolloutPreviewRequest) -> dict[str, object]:
        processor_contract = LeRobotProcessorContractExporter().export(
            LeRobotProcessorContractRequest(
                eef_plan_dir=request.eef_plan_dir,
                model=request.model,
                robot_interface=request.robot_interface,
            )
        )
        delegated_preview = EefPreviewSynthesizer().synthesize(
            EefPreviewSynthesisRequest(
                plan_dir=request.eef_plan_dir,
                urdf_path=request.urdf_path,
                safe_config_path=request.safe_config_path,
                backend="pink",
                recipe_plan_dir=request.recipe_plan_dir,
                start_joints=request.start_joints,
                sample_hz=request.sample_hz,
                duration_s=request.duration_s,
                render_path=request.render_path,
            )
        )
        review = delegated_preview["review"]
        return {
            "schema": "armctrl.lerobot_rollout_preview.v1",
            "movement_allowed": False,
            "eef_plan_dir": str(request.eef_plan_dir),
            "processor_contract": processor_contract,
            "delegated_preview": delegated_preview,
            "synthesized_trajectory": delegated_preview["synthesized_trajectory"],
            "review_status": review["review_status"],
            "sim_preview": review.get("sim_preview"),
            "next_gate": review["next_gate"],
            "ordered_steps": [
                {
                    "id": "preview_rollout",
                    "command": (
                        "uv run armctrl lerobot preview-rollout "
                        f"--eef-plan-dir {request.eef_plan_dir} --urdf-path {request.urdf_path} "
                        f"--safe-config {request.safe_config_path} --json"
                    ),
                },
                {
                    "id": "review_rollout",
                    "depends_on": ["preview_rollout"],
                    "command": (
                        "uv run armctrl lerobot review-rollout "
                        f"--eef-plan-dir {request.eef_plan_dir} --json"
                    ),
                },
            ],
            "next_steps": [
                f"uv run armctrl lerobot export-processor-contract --eef-plan-dir {request.eef_plan_dir} --json",
                f"uv run armctrl lerobot review-rollout --eef-plan-dir {request.eef_plan_dir} --json",
            ],
            "notes": [
                "This is a non-hardware closure helper for Agent and rollout-side integration tests.",
                "It reuses the existing EEF preview synthesizer and shared simulation review chain instead of owning rollout execution.",
                "The LeRobot processor contract remains the authoritative bridge for robot_action_processor and robot_observation_processor integrations.",
            ],
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
