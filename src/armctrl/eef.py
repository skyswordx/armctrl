from __future__ import annotations

import csv
import importlib.util
import json
import math
import shutil
from dataclasses import dataclass
from pathlib import Path

from armctrl.simulation import TrajectoryPreviewer, _installed_ros_distribution
from armctrl.workspace import WorkspaceSafetyConfig


def _module_available(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


class EefDoctor:
    def run(self) -> dict[str, object]:
        moveit_modules = ("rclpy", "moveit_msgs", "moveit_configs_utils")
        moveit_importable = all(_module_available(name) for name in moveit_modules)
        ros_distro = _installed_ros_distribution()
        if moveit_importable:
            moveit_status = "available"
        elif ros_distro is not None:
            moveit_status = "installed_not_sourced"
        else:
            moveit_status = "missing"
        backends: dict[str, object] = {
            "moveit_servo": {
                "status": moveit_status,
                "modules": [
                    {
                        "module": name,
                        "importable": _module_available(name),
                    }
                    for name in moveit_modules
                ],
                "role": "ROS 2 MoveIt Servo Cartesian or joint-space realtime servo backend",
            },
            "pink": {
                "status": "available" if _module_available("pink") else "missing",
                "modules": [
                    {
                        "module": "pink",
                        "importable": _module_available("pink"),
                    },
                    {
                        "module": "pinocchio",
                        "importable": _module_available("pinocchio"),
                    },
                ],
                "role": "Pinocchio-based differential IK backend for bounded local EEF planning",
                "install_hint": "uv sync --extra dev --extra eef",
            },
            "sdk_cartesian": {
                "status": "available" if _module_available("arx5_interface") else "missing",
                "modules": [
                    {
                        "module": "arx5_interface",
                        "importable": _module_available("arx5_interface"),
                    }
                ],
                "role": "ARX5 SDK Cartesian controller backend using Arx5CartesianController and EEFState commands",
            },
        }
        if ros_distro is not None:
            backends["moveit_servo"]["ros_distro"] = ros_distro
            backends["moveit_servo"]["source_hint"] = (
                f"source /opt/ros/{ros_distro}/setup.bash"
            )
        return {
            "schema": "armctrl.eef_doctor.v1",
            "read_only": True,
            "movement_allowed": False,
            "backends": backends,
            "next_gate": "run_eef_plan_before_any_future_execution",
            "notes": [
                "armctrl does not implement its own servo, IK, or motion planner",
                "MoveIt Servo is the preferred realtime Cartesian backend on ROS 2 hosts",
                "Pink is a lightweight differential IK option for local bounded planning",
                "ARX5 SDK Cartesian control remains a mature execution owner when the robot workstation already uses arx5_interface directly",
                "Install the optional Pink preview helper on Linux with uv sync --extra dev --extra eef",
            ],
        }


@dataclass(frozen=True)
class EefRuntimePlanRequest:
    model: str = "X5"
    interface: str = "can0"
    backend: str | None = None
    policy_path: str | None = None
    plan_dir: Path | None = None


@dataclass(frozen=True)
class EefTwistRequest:
    frame: str
    linear_mps: tuple[float, float, float]
    angular_rps: tuple[float, float, float]
    backend: str
    safe_config_path: str
    control_period_s: float = 0.1
    output_dir: Path | None = None


@dataclass(frozen=True)
class EefPoseRequest:
    frame: str
    position_m: tuple[float, float, float]
    rpy_rad: tuple[float, float, float]
    backend: str
    safe_config_path: str
    output_dir: Path | None = None


@dataclass(frozen=True)
class EefDeltaPoseRequest:
    frame: str
    delta_position_m: tuple[float, float, float]
    delta_rpy_rad: tuple[float, float, float]
    backend: str
    safe_config_path: str
    control_period_s: float = 0.1
    output_dir: Path | None = None


@dataclass(frozen=True)
class EefReviewRequest:
    plan_dir: Path
    urdf_path: Path
    safe_config_path: Path
    trajectory_path: Path | None = None
    render_path: Path | None = None


@dataclass(frozen=True)
class EefLeRobotExportRequest:
    plan_dir: Path
    output_path: Path | None = None


@dataclass(frozen=True)
class EefSdkCartesianExportRequest:
    plan_dir: Path
    model: str = "X5"
    interface: str = "can0"
    output_path: Path | None = None


@dataclass(frozen=True)
class EefMoveItServoExportRequest:
    plan_dir: Path
    output_path: Path | None = None


@dataclass(frozen=True)
class EefRuntimeBridgeExportRequest:
    plan_dir: Path
    backend: str | None = None
    model: str = "X5"
    interface: str = "can0"
    output_path: Path | None = None


@dataclass(frozen=True)
class EefRunnerContractRequest:
    plan_dir: Path
    backend: str | None = None
    model: str = "X5"
    interface: str = "can0"
    output_path: Path | None = None


@dataclass(frozen=True)
class EefAgentRuntimeContractRequest:
    plan_dir: Path
    backend: str | None = None
    model: str = "X5"
    interface: str = "can0"
    output_path: Path | None = None


@dataclass(frozen=True)
class EefRunnerPreviewRequest:
    plan_dir: Path
    urdf_path: Path
    safe_config_path: Path
    backend: str | None = None
    model: str = "X5"
    interface: str = "can0"
    recipe_plan_dir: Path | None = None
    start_joints: tuple[float, ...] | None = None
    sample_hz: float = 50.0
    duration_s: float = 2.0
    render_path: Path | None = None


@dataclass(frozen=True)
class EefSampleRunnerRequest:
    runner_contract_path: Path
    urdf_path: Path
    safe_config_path: Path
    recipe_plan_dir: Path | None = None
    start_joints: tuple[float, ...] | None = None
    sample_hz: float = 50.0
    duration_s: float = 2.0
    render_path: Path | None = None


@dataclass(frozen=True)
class EefSdkHelperPlanRequest:
    runner_contract_path: Path
    output_path: Path | None = None


@dataclass(frozen=True)
class EefMoveItHelperPlanRequest:
    runner_contract_path: Path
    output_path: Path | None = None


@dataclass(frozen=True)
class EefAgentSessionPlanRequest:
    plan_dir: Path
    backend: str | None = None
    model: str = "X5"
    interface: str = "can0"
    output_path: Path | None = None


@dataclass(frozen=True)
class EefStageTrajectoryRequest:
    plan_dir: Path
    trajectory_path: Path


@dataclass(frozen=True)
class EefPreviewSynthesisRequest:
    plan_dir: Path
    urdf_path: Path
    safe_config_path: Path
    backend: str = "pink"
    start_joints: tuple[float, ...] | None = None
    recipe_plan_dir: Path | None = None
    sample_hz: float = 50.0
    duration_s: float = 2.0
    render_path: Path | None = None


class EefPlanner:
    def plan_twist(self, request: EefTwistRequest) -> dict[str, object]:
        config = WorkspaceSafetyConfig.from_yaml(Path(request.safe_config_path))
        gate = _evaluate_twist_gate(config, request)
        backend_handoff = _backend_handoff_for_twist(request)
        payload = {
            "schema": "armctrl.eef_plan.v1",
            "plan_only": True,
            "movement_allowed": False,
            "command_type": "twist",
            "boundary": "mature_eef_backend_adapter",
            "backend": {
                "requested": request.backend,
                "execution": "deferred",
            },
            "command": {
                "frame": request.frame,
                "linear_mps": list(request.linear_mps),
                "angular_rps": list(request.angular_rps),
                "control_period_s": request.control_period_s,
            },
            "gate": gate,
            "backend_handoff": backend_handoff,
            "agent_action": _agent_action_for_twist(
                request, backend_handoff=backend_handoff
            ),
            "lerobot_bridge": _lerobot_bridge_for_twist(request),
            "safety_contract": _safety_contract(config),
            "notes": _eef_notes(),
        }
        return _write_plan_artifacts(payload, request.output_dir, request.safe_config_path)

    def plan_pose(self, request: EefPoseRequest) -> dict[str, object]:
        config = WorkspaceSafetyConfig.from_yaml(Path(request.safe_config_path))
        gate = _evaluate_pose_gate(config, request)
        backend_handoff = _backend_handoff_for_pose(request)
        payload = {
            "schema": "armctrl.eef_plan.v1",
            "plan_only": True,
            "movement_allowed": False,
            "command_type": "pose",
            "boundary": "mature_eef_backend_adapter",
            "backend": {
                "requested": request.backend,
                "execution": "deferred",
            },
            "command": {
                "frame": request.frame,
                "position_m": list(request.position_m),
                "rpy_rad": list(request.rpy_rad),
            },
            "gate": gate,
            "backend_handoff": backend_handoff,
            "agent_action": _agent_action_for_pose(
                request, backend_handoff=backend_handoff
            ),
            "lerobot_bridge": _lerobot_bridge_for_pose(request),
            "safety_contract": _safety_contract(config),
            "notes": _eef_notes(),
        }
        return _write_plan_artifacts(payload, request.output_dir, request.safe_config_path)

    def plan_delta_pose(self, request: EefDeltaPoseRequest) -> dict[str, object]:
        config = WorkspaceSafetyConfig.from_yaml(Path(request.safe_config_path))
        gate = _evaluate_delta_pose_gate(config, request)
        backend_handoff = _backend_handoff_for_delta_pose(request)
        payload = {
            "schema": "armctrl.eef_plan.v1",
            "plan_only": True,
            "movement_allowed": False,
            "command_type": "pose_delta",
            "boundary": "mature_eef_backend_adapter",
            "backend": {
                "requested": request.backend,
                "execution": "deferred",
            },
            "command": {
                "frame": request.frame,
                "delta_position_m": list(request.delta_position_m),
                "delta_rpy_rad": list(request.delta_rpy_rad),
                "control_period_s": request.control_period_s,
            },
            "gate": gate,
            "backend_handoff": backend_handoff,
            "agent_action": _agent_action_for_delta_pose(
                request, backend_handoff=backend_handoff
            ),
            "lerobot_bridge": _lerobot_bridge_for_delta_pose(request),
            "safety_contract": _safety_contract(config),
            "notes": _eef_notes(),
        }
        return _write_plan_artifacts(payload, request.output_dir, request.safe_config_path)


class EefReviewer:
    def review(self, request: EefReviewRequest) -> dict[str, object]:
        backend_request_path = request.plan_dir / "backend_request.json"
        backend_review_contract_path = request.plan_dir / "backend_review_contract.json"
        eef_plan_path = request.plan_dir / "eef_plan.json"
        manifest_path = request.plan_dir / "manifest.json"
        backend_request = json.loads(backend_request_path.read_text(encoding="utf-8"))
        backend_review_contract = json.loads(
            backend_review_contract_path.read_text(encoding="utf-8")
        )
        eef_plan = json.loads(eef_plan_path.read_text(encoding="utf-8"))
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        trajectory_path = request.trajectory_path
        if trajectory_path is None:
            candidate = Path(backend_review_contract["expected_backend_trajectory"])
            if candidate.exists():
                trajectory_path = candidate
        if trajectory_path is None:
            return {
                "schema": "armctrl.eef_review.v1",
                "movement_allowed": False,
                "review_status": "deferred",
                "plan_dir": str(request.plan_dir),
                "backend_request": backend_request,
                "backend_review_contract": backend_review_contract,
                "eef_plan": eef_plan,
                "manifest": manifest,
                "reviewed_trajectory": None,
                "expected_backend_trajectory": backend_review_contract[
                    "expected_backend_trajectory"
                ],
                "sim_preview": None,
                "next_gate": (
                    "Provide a backend joint trajectory at the expected review path or pass --trajectory "
                    "to run the shared simulation preview chain."
                ),
            }
        sim_preview = TrajectoryPreviewer().preview(
            trajectory_path=trajectory_path,
            urdf_path=request.urdf_path,
            safe_config_path=request.safe_config_path,
            render_path=request.render_path,
        )
        return {
            "schema": "armctrl.eef_review.v1",
            "movement_allowed": False,
            "review_status": "completed",
            "plan_dir": str(request.plan_dir),
            "backend_request": backend_request,
            "backend_review_contract": backend_review_contract,
            "eef_plan": eef_plan,
            "manifest": manifest,
            "reviewed_trajectory": str(trajectory_path),
            "sim_preview": sim_preview,
            "next_gate": (
                "If sim_preview.safety.allowed is true, the mature backend trajectory is ready for later execution-gate work."
            ),
        }


class EefRuntimePlanner:
    def plan(self, request: EefRuntimePlanRequest) -> dict[str, object]:
        backend_request: dict[str, object] | None = None
        backend_review_contract: dict[str, object] | None = None
        backend = request.backend
        requested_backend_override = request.backend
        if request.plan_dir is not None:
            backend_request = json.loads(
                (request.plan_dir / "backend_request.json").read_text(encoding="utf-8")
            )
            backend_review_contract = json.loads(
                (request.plan_dir / "backend_review_contract.json").read_text(
                    encoding="utf-8"
                )
            )
            backend = (
                requested_backend_override
                if requested_backend_override is not None
                else str(backend_request["backend"]["requested"])
            )
        if backend is None:
            raise ValueError("EEF runtime backend is required when --plan-dir is not provided")
        if backend == "sdk_cartesian":
            payload = {
                "schema": "armctrl.eef_runtime_plan.v1",
                "movement_allowed": False,
                "plan_only": True,
                "backend": backend,
                "runtime": {
                    "owner": "arx5_interface_cartesian_controller",
                    "session_kind": "native_python_sdk",
                    "api_contract": {
                        "controller_class": "Arx5CartesianController",
                        "command_object": "EEFState",
                        "pose_surface": "EEFState.pose_6d",
                        "delta_surface": "EEFState.pose_6d_delta",
                        "submit_call": "controller.set_eef_cmd(eef_cmd)",
                        "reference_examples": [
                            "vendor/real_stanford_arx5_sdk/python/examples/keyboard_teleop.py",
                            "vendor/real_stanford_arx5_sdk/python/examples/cartesian_waypoint_scheduling.py",
                        ],
                    },
                },
                "reference_commands": [
                    [
                        "python",
                        "vendor/real_stanford_arx5_sdk/python/examples/keyboard_teleop.py",
                        request.model,
                        request.interface,
                    ]
                ],
                "artifact_contract": _artifact_contract(),
                "notes": [
                    "Use backend_request.json as the armctrl-side intent contract and synthesize a reviewed joint trajectory before any hardware execution.",
                    "The vendor keyboard teleop is a reference runtime surface only; Agent integrations should call the same mature SDK APIs programmatically instead of scraping terminal input.",
                ],
            }
            if request.plan_dir is not None:
                payload["bridge_artifact_preview"] = EefSdkCartesianExporter().export(
                    EefSdkCartesianExportRequest(
                        plan_dir=request.plan_dir,
                        model=request.model,
                        interface=request.interface,
                    )
                )
                payload["agent_runtime_profile"] = _eef_agent_runtime_profile(
                    plan_dir=request.plan_dir,
                    runtime_owner="arx5_interface_cartesian_controller",
                    preferred_next_surface="armctrl.eef.stage_trajectory",
                )
                payload["next_steps"] = _runtime_next_steps(
                    backend=backend,
                    plan_dir=request.plan_dir,
                )
            return _attach_runtime_plan_artifacts(
                payload,
                backend_request=backend_request,
                backend_review_contract=backend_review_contract,
                plan_dir=request.plan_dir,
            )
        if backend == "lerobot_rollout":
            command = [
                "lerobot-rollout",
                "--robot.type=arx5_follower",
                f"--robot.arm_model={request.model}",
                f"--robot.interface_name={request.interface}",
            ]
            if request.policy_path is not None:
                command.append(f"--policy.path={request.policy_path}")
            payload = {
                "schema": "armctrl.eef_runtime_plan.v1",
                "movement_allowed": False,
                "plan_only": True,
                "backend": backend,
                "runtime": {
                    "owner": "native_lerobot_rollout",
                    "session_kind": "native_lerobot_cli",
                    "api_contract": {
                        "command_surface": "lerobot-rollout",
                        "strategy_surface": "--strategy.type=<base|sentry|highlight|dagger>",
                        "default_strategy": "base",
                        "inference_surface": "--inference.type=<sync|rtc>",
                        "processor_hooks": [
                            "robot_action_processor",
                            "robot_observation_processor",
                        ],
                        "action_alignment": "cartesian action mapping should follow lerobot_bridge from eef plan artifacts",
                        "reference_docs": [
                            "https://huggingface.co/docs/lerobot/main/inference",
                            "https://huggingface.co/docs/lerobot/introduction_processors",
                        ],
                    },
                },
                "reference_commands": [command],
                "artifact_contract": _artifact_contract(),
                "notes": [
                    "LeRobot remains the rollout owner; armctrl only contributes safety-gated planning and review artifacts.",
                    "Policies that emit cartesian end-effector actions should align with the lerobot_bridge mapping in eef plan artifacts.",
                ],
            }
            if request.plan_dir is not None:
                payload["bridge_artifact_preview"] = EefLeRobotExporter().export(
                    EefLeRobotExportRequest(plan_dir=request.plan_dir)
                )
                payload["agent_runtime_profile"] = _eef_agent_runtime_profile(
                    plan_dir=request.plan_dir,
                    runtime_owner="native_lerobot_rollout",
                    preferred_next_surface="armctrl.eef.stage_trajectory",
                )
                payload["next_steps"] = _runtime_next_steps(
                    backend=backend,
                    plan_dir=request.plan_dir,
                )
            return _attach_runtime_plan_artifacts(
                payload,
                backend_request=backend_request,
                backend_review_contract=backend_review_contract,
                plan_dir=request.plan_dir,
            )
        if backend == "moveit_servo":
            command_surface = "TwistStamped"
            if backend_request is not None and str(backend_request["command_type"]) == "pose":
                command_surface = "PoseStamped"
            payload = {
                "schema": "armctrl.eef_runtime_plan.v1",
                "movement_allowed": False,
                "plan_only": True,
                "backend": backend,
                "runtime": {
                    "owner": "ros2_moveit_servo",
                    "session_kind": "ros2_servo_node",
                    "api_contract": {
                        "command_surface": command_surface,
                        "environment": "source /opt/ros/<distro>/setup.bash",
                        "reference_docs": [
                            "https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html",
                            "https://moveit.picknik.ai/main/doc/concepts/moveit_servo/moveit_servo.html",
                        ],
                        "input_topics": {
                            "twist": "~/delta_twist_cmds",
                            "pose": "~/pose_command_in_topic",
                        },
                        "output_topic_param": "command_out_topic",
                        "command_out_type_param": "command_out_type",
                        "supported_output_types": [
                            "trajectory_msgs/JointTrajectory",
                            "std_msgs/Float64MultiArray",
                        ],
                        "switch_command_type_service": "~/switch_command_type",
                        "pause_service": "~/pause_servo",
                        "status_topic": "~/status",
                        "cxx_interface": {
                            "command_types": [
                                "JointJogCommand",
                                "TwistCommand",
                                "PoseCommand",
                            ],
                            "output_type": "KinematicState",
                            "status_method": "servo.getStatus()",
                        },
                    },
                },
                "reference_commands": [],
                "artifact_contract": _artifact_contract(),
                "notes": [
                    "armctrl does not ship a MoveIt launch file in this clean rebuild branch.",
                    "Use the existing robot workstation MoveIt Servo deployment, then feed the generated joint trajectory back into armctrl eef review.",
                ],
            }
            if request.plan_dir is not None:
                payload["bridge_artifact_preview"] = EefMoveItServoExporter().export(
                    EefMoveItServoExportRequest(plan_dir=request.plan_dir)
                )
                payload["agent_runtime_profile"] = _eef_agent_runtime_profile(
                    plan_dir=request.plan_dir,
                    runtime_owner="ros2_moveit_servo",
                    preferred_next_surface="armctrl.eef.stage_trajectory",
                )
                payload["next_steps"] = _runtime_next_steps(
                    backend=backend,
                    plan_dir=request.plan_dir,
                )
            return _attach_runtime_plan_artifacts(
                payload,
                backend_request=backend_request,
                backend_review_contract=backend_review_contract,
                plan_dir=request.plan_dir,
            )
        raise ValueError(f"unsupported EEF runtime backend: {backend}")


class EefTrajectoryStager:
    def stage(self, request: EefStageTrajectoryRequest) -> dict[str, object]:
        backend_request = json.loads(
            (request.plan_dir / "backend_request.json").read_text(encoding="utf-8")
        )
        backend_review_contract = json.loads(
            (request.plan_dir / "backend_review_contract.json").read_text(
                encoding="utf-8"
            )
        )
        trajectory_validation = _validate_backend_trajectory(
            request.trajectory_path,
            required_columns=backend_review_contract["trajectory_format"][
                "required_columns"
            ],
        )
        staged_trajectory = Path(backend_review_contract["expected_backend_trajectory"])
        staged_trajectory.parent.mkdir(parents=True, exist_ok=True)
        if request.trajectory_path.resolve() != staged_trajectory.resolve():
            shutil.copyfile(request.trajectory_path, staged_trajectory)
        return {
            "schema": "armctrl.eef_stage_trajectory.v1",
            "movement_allowed": False,
            "plan_dir": str(request.plan_dir),
            "source_trajectory": str(request.trajectory_path),
            "staged_trajectory": str(staged_trajectory),
            "trajectory_validation": trajectory_validation,
            "backend_request": backend_request,
            "backend_review_contract": backend_review_contract,
            "next_gate": "run armctrl eef review --plan-dir <dir> --json",
            "notes": [
                "armctrl only stages a mature-backend joint trajectory into the shared review location.",
                "This command does not execute hardware, IK, servo, or policy runtime logic.",
            ],
        }


class EefPreviewSynthesizer:
    def synthesize(self, request: EefPreviewSynthesisRequest) -> dict[str, object]:
        backend_request_path = request.plan_dir / "backend_request.json"
        backend_review_contract_path = request.plan_dir / "backend_review_contract.json"
        missing = [
            str(path)
            for path in (backend_request_path, backend_review_contract_path)
            if not path.exists()
        ]
        if missing:
            raise FileNotFoundError(
                "missing eef plan artifacts required for preview synthesis: "
                + ", ".join(missing)
            )
        backend_request = json.loads(backend_request_path.read_text(encoding="utf-8"))
        backend_review_contract = json.loads(
            backend_review_contract_path.read_text(encoding="utf-8")
        )
        if backend_request["command_type"] not in {"pose", "pose_delta"}:
            raise ValueError(
                "preview synthesis currently supports only pose or pose_delta plans"
            )
        if request.backend != "pink":
            raise ValueError(f"unsupported preview synthesis backend: {request.backend}")
        start_joints, start_joint_source = _resolve_preview_start_joints(request)

        target_joints, synthesis = _synthesize_preview_target(
            backend_request=backend_request,
            start_joints=start_joints,
            urdf_path=request.urdf_path,
        )
        synthesized_trajectory = Path(
            backend_review_contract["expected_backend_trajectory"]
        )
        _write_preview_trajectory(
            output_path=synthesized_trajectory,
            start_joints=start_joints,
            target_joints=target_joints,
            sample_hz=request.sample_hz,
            duration_s=request.duration_s,
        )
        review = EefReviewer().review(
            EefReviewRequest(
                plan_dir=request.plan_dir,
                urdf_path=request.urdf_path,
                safe_config_path=request.safe_config_path,
                trajectory_path=synthesized_trajectory,
                render_path=request.render_path,
            )
        )
        return {
            "schema": "armctrl.eef_preview_synthesis.v1",
            "movement_allowed": False,
            "plan_dir": str(request.plan_dir),
            "backend": request.backend,
            "start_joints": list(start_joints),
            "start_joints_source": start_joint_source,
            "solved_joint_target": list(target_joints),
            "synthesis": synthesis,
            "synthesized_trajectory": str(synthesized_trajectory),
            "review": review,
            "notes": [
                "This helper only synthesizes a non-hardware joint preview and immediately reuses the shared review/simulation chain.",
                "Realtime execution still belongs to mature backends such as MoveIt Servo, LeRobot rollout, or the ARX5 SDK cartesian controller.",
                "Recipe plan EEF seeds can be used as the preview start state so preset actions and bounded EEF intents share one handoff surface.",
            ],
        }


class EefRuntimeBridgeExporter:
    def export(self, request: EefRuntimeBridgeExportRequest) -> dict[str, object]:
        backend_request = json.loads(
            (request.plan_dir / "backend_request.json").read_text(encoding="utf-8")
        )
        resolved_backend = (
            request.backend
            if request.backend is not None
            else str(backend_request["backend"]["requested"])
        )
        if resolved_backend == "sdk_cartesian":
            delegated_export = EefSdkCartesianExporter().export(
                EefSdkCartesianExportRequest(
                    plan_dir=request.plan_dir,
                    model=request.model,
                    interface=request.interface,
                    output_path=request.output_path,
                )
            )
        elif resolved_backend == "moveit_servo":
            delegated_export = EefMoveItServoExporter().export(
                EefMoveItServoExportRequest(
                    plan_dir=request.plan_dir,
                    output_path=request.output_path,
                )
            )
        elif resolved_backend == "lerobot_rollout":
            delegated_export = EefLeRobotExporter().export(
                EefLeRobotExportRequest(
                    plan_dir=request.plan_dir,
                    output_path=request.output_path,
                )
            )
        else:
            raise ValueError(f"unsupported runtime bridge backend: {resolved_backend}")
        return {
            "schema": "armctrl.eef_runtime_bridge_export.v1",
            "movement_allowed": False,
            "plan_dir": str(request.plan_dir),
            "requested_backend": request.backend,
            "resolved_backend": resolved_backend,
            "delegated_export": delegated_export,
            "notes": [
                "This command is a thin dispatcher over the backend-specific EEF bridge exporters.",
                "armctrl still delegates IK, servo, rollout, and hardware execution to mature backend owners.",
            ],
        }


class EefRunnerContractExporter:
    def export(self, request: EefRunnerContractRequest) -> dict[str, object]:
        bridge_export = EefRuntimeBridgeExporter().export(
            EefRuntimeBridgeExportRequest(
                plan_dir=request.plan_dir,
                backend=request.backend,
                model=request.model,
                interface=request.interface,
                output_path=request.output_path,
            )
        )
        backend_review_contract = json.loads(
            (request.plan_dir / "backend_review_contract.json").read_text(
                encoding="utf-8"
            )
        )
        resolved_backend = str(bridge_export["resolved_backend"])
        if resolved_backend == "sdk_cartesian":
            runner_api = {
                "owner": "arx5_interface_cartesian_controller",
                "entrypoint_family": "native_python_sdk",
                "command_object": "EEFState",
                "submit_call": "controller.set_eef_cmd(eef_cmd)",
                "reference_examples": bridge_export["delegated_export"][
                    "reference_examples"
                ],
            }
        elif resolved_backend == "moveit_servo":
            runner_api = {
                "owner": "ros2_moveit_servo",
                "entrypoint_family": "ros2_servo_node",
                "message_type": bridge_export["delegated_export"]["ros_contract"][
                    "message_type"
                ],
                "reference_docs": bridge_export["delegated_export"]["reference_docs"],
            }
        elif resolved_backend == "lerobot_rollout":
            runner_api = {
                "owner": "native_lerobot_rollout",
                "entrypoint_family": "lerobot.rollout",
                "action_hook": "robot_action_processor",
                "observation_hook": "robot_observation_processor",
                "reference_docs": [
                    "https://huggingface.co/docs/lerobot/main/inference",
                    "https://huggingface.co/docs/lerobot/main/il_robots",
                ],
            }
        else:
            raise ValueError(f"unsupported runner contract backend: {resolved_backend}")
        next_steps = [
            "uv run armctrl eef doctor --json",
            "uv run armctrl sim doctor --json",
            f"uv run armctrl eef synthesize-preview --plan-dir {request.plan_dir} --json",
        ]
        next_steps.extend(
            [
                (
                    f"uv run armctrl eef stage-trajectory --plan-dir {request.plan_dir} "
                    f"--trajectory <joint_csv> --json"
                ),
                f"uv run armctrl eef review --plan-dir {request.plan_dir} --json",
            ]
        )
        payload = {
            "schema": "armctrl.eef_runner_contract.v1",
            "movement_allowed": False,
            "plan_dir": str(request.plan_dir),
            "requested_backend": request.backend,
            "resolved_backend": resolved_backend,
            "bridge_export": bridge_export["delegated_export"],
            "agent_runtime_profile": _eef_agent_runtime_profile(
                plan_dir=request.plan_dir,
                runtime_owner=runner_api["owner"],
                preferred_next_surface="armctrl.eef.stage_trajectory",
            ),
            "runner_api": runner_api,
            "review_output_contract": {
                "expected_trajectory_path": backend_review_contract[
                    "expected_backend_trajectory"
                ],
                "required_columns": backend_review_contract["trajectory_format"][
                    "required_columns"
                ],
                "stage_command": (
                    f"uv run armctrl eef stage-trajectory --plan-dir {request.plan_dir} "
                    f"--trajectory <joint_csv> --json"
                ),
                "review_command": (
                    f"uv run armctrl eef review --plan-dir {request.plan_dir} --json"
                ),
            },
            "notes": [
                "This is a non-executing handoff contract for backend helpers that consume an EEF bridge artifact and emit a reviewed joint trajectory CSV.",
                "armctrl still does not run the mature backend session itself.",
            ],
            "next_steps": next_steps,
        }
        if request.output_path is not None:
            request.output_path.parent.mkdir(parents=True, exist_ok=True)
            request.output_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            payload["output"] = str(request.output_path)
        return payload


class EefAgentRuntimeContractExporter:
    def export(self, request: EefAgentRuntimeContractRequest) -> dict[str, object]:
        eef_plan = json.loads(
            (request.plan_dir / "eef_plan.json").read_text(encoding="utf-8")
        )
        backend_review_contract = json.loads(
            (request.plan_dir / "backend_review_contract.json").read_text(
                encoding="utf-8"
            )
        )
        bridge_export = EefRuntimeBridgeExporter().export(
            EefRuntimeBridgeExportRequest(
                plan_dir=request.plan_dir,
                backend=request.backend,
                model=request.model,
                interface=request.interface,
            )
        )
        resolved_backend = str(bridge_export["resolved_backend"])
        delegated_export = bridge_export["delegated_export"]
        agent_action = dict(eef_plan["agent_action"])
        runtime_owner: str
        backend_session_contract: dict[str, object]
        if resolved_backend == "sdk_cartesian":
            runtime_owner = "arx5_interface_cartesian_controller"
            backend_session_contract = {
                "session_kind": "native_python_sdk",
                "controller_class": delegated_export["sdk_controller"]["controller_class"],
                "command_object": delegated_export["sdk_controller"]["command_object"],
                "command_mode": delegated_export["sdk_controller"]["command_mode"],
                "submit_call": delegated_export["sdk_controller"]["submit_call"],
                "control_frame": delegated_export["sdk_request"]["control_frame"],
            }
        elif resolved_backend == "moveit_servo":
            runtime_owner = "ros2_moveit_servo"
            command_topic = (
                "~/pose_command_in_topic"
                if delegated_export["ros_contract"]["message_type"]
                == "geometry_msgs/msg/PoseStamped"
                else "~/delta_twist_cmds"
            )
            backend_session_contract = {
                "session_kind": "ros2_servo_node",
                "message_type": delegated_export["ros_contract"]["message_type"],
                "command_topic": command_topic,
                "status_topic": "~/status",
                "pause_service": "~/pause_servo",
                "switch_command_type_service": "~/switch_command_type",
            }
        elif resolved_backend == "lerobot_rollout":
            runtime_owner = "native_lerobot_rollout"
            backend_session_contract = {
                "session_kind": "native_lerobot_rollout",
                "action_hook": "robot_action_processor",
                "observation_hook": "robot_observation_processor",
            }
        else:
            raise ValueError(
                f"unsupported agent runtime contract backend: {resolved_backend}"
            )
        payload = {
            "schema": "armctrl.eef_agent_runtime_contract.v1",
            "movement_allowed": False,
            "plan_dir": str(request.plan_dir),
            "requested_backend": request.backend,
            "resolved_backend": resolved_backend,
            "runtime_owner": runtime_owner,
            "agent_runtime_profile": _eef_agent_runtime_profile(
                plan_dir=request.plan_dir,
                runtime_owner=runtime_owner,
                preferred_next_surface="armctrl.eef.stage_trajectory",
            ),
            "agent_action_schema": {
                "schema": agent_action["schema"],
                "action_id": agent_action["action_id"],
                "frame": agent_action["frame"],
                "stream_mode": "realtime_frame_sequence",
                "value_schema": agent_action["value"],
            },
            "backend_session_contract": backend_session_contract,
            "bridge_export": delegated_export,
            "review_output_contract": {
                "expected_trajectory_path": backend_review_contract[
                    "expected_backend_trajectory"
                ],
                "required_columns": backend_review_contract["trajectory_format"][
                    "required_columns"
                ],
            },
            "ordered_steps": [
                {
                    "id": "stage_trajectory",
                    "command": (
                        f"uv run armctrl eef stage-trajectory --plan-dir {request.plan_dir} "
                        "--trajectory <joint_csv> --json"
                    ),
                    "depends_on": [],
                    "parallel_safe_with": [],
                },
                {
                    "id": "review_trajectory",
                    "command": (
                        f"uv run armctrl eef review --plan-dir {request.plan_dir} --json"
                    ),
                    "depends_on": ["stage_trajectory"],
                    "parallel_safe_with": [],
                },
            ],
            "next_steps": [
                f"uv run armctrl eef stage-trajectory --plan-dir {request.plan_dir} --trajectory <joint_csv> --json",
                f"uv run armctrl eef review --plan-dir {request.plan_dir} --json",
            ],
            "notes": [
                "This contract is for Agent or helper loops that produce a sequence of EEF action frames against one stable action vocabulary.",
                "armctrl still does not execute the mature backend session itself; it defines the approved handoff and return path into shared review.",
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


class EefRunnerPreviewer:
    def preview(self, request: EefRunnerPreviewRequest) -> dict[str, object]:
        runner_contract = EefRunnerContractExporter().export(
            EefRunnerContractRequest(
                plan_dir=request.plan_dir,
                backend=request.backend,
                model=request.model,
                interface=request.interface,
            )
        )
        delegated_preview = EefPreviewSynthesizer().synthesize(
            EefPreviewSynthesisRequest(
                plan_dir=request.plan_dir,
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
            "schema": "armctrl.eef_runner_preview.v1",
            "movement_allowed": False,
            "plan_dir": str(request.plan_dir),
            "runner_contract": runner_contract,
            "delegated_preview": delegated_preview,
            "synthesized_trajectory": delegated_preview["synthesized_trajectory"],
            "review_status": review["review_status"],
            "sim_preview": review.get("sim_preview"),
            "next_gate": review["next_gate"],
            "next_steps": [
                f"uv run armctrl eef export-runner-contract --plan-dir {request.plan_dir} --json",
                f"uv run armctrl eef review --plan-dir {request.plan_dir} --json",
            ],
            "notes": [
                "This is a non-hardware closure helper for mature-backend runtime handoffs.",
                "It reuses the runner contract plus the existing preview synthesizer and shared simulation review chain.",
                "armctrl still does not execute the mature backend session itself.",
            ],
        }


class EefSdkHelperPlanExporter:
    def export(self, request: EefSdkHelperPlanRequest) -> dict[str, object]:
        runner_contract = json.loads(
            request.runner_contract_path.read_text(encoding="utf-8")
        )
        if runner_contract.get("schema") != "armctrl.eef_runner_contract.v1":
            raise ValueError(
                "runner contract file must use schema armctrl.eef_runner_contract.v1"
            )
        resolved_backend = runner_contract.get("resolved_backend")
        if resolved_backend != "sdk_cartesian":
            raise RuntimeError(
                "unsupported runner backend for sdk helper plan: "
                f"{resolved_backend}"
            )
        bridge_export = dict(runner_contract["bridge_export"])
        sdk_controller = dict(bridge_export["sdk_controller"])
        sdk_request = dict(bridge_export["sdk_request"])
        payload = {
            "schema": "armctrl.sdk_cartesian_helper_plan.v1",
            "movement_allowed": False,
            "runner_contract_path": str(request.runner_contract_path),
            "plan_dir": str(runner_contract["plan_dir"]),
            "resolved_backend": resolved_backend,
            "runtime_owner": runner_contract["runner_api"]["owner"],
            "sdk_controller": sdk_controller,
            "sdk_session_plan": {
                "controller_class": sdk_controller["controller_class"],
                "command_object": sdk_controller["command_object"],
                "command_mode": sdk_controller["command_mode"],
                "submit_call": sdk_controller["submit_call"],
                "robot": sdk_request["robot"],
                "control_frame": sdk_request["control_frame"],
                "waypoint_count": sdk_request["waypoint_count"],
                "eef_waypoints": sdk_request["eef_waypoints"],
            },
            "review_output_contract": runner_contract["review_output_contract"],
            "reference_examples": runner_contract["runner_api"]["reference_examples"],
            "ordered_steps": [
                {
                    "id": "sample_runner",
                    "command": (
                        "uv run armctrl eef sample-runner "
                        f"--runner-contract {request.runner_contract_path} --json"
                    ),
                },
                {
                    "id": "review_trajectory",
                    "depends_on": ["sample_runner"],
                    "command": runner_contract["review_output_contract"][
                        "review_command"
                    ],
                },
            ],
            "next_steps": [
                (
                    "uv run armctrl eef sample-runner "
                    f"--runner-contract {request.runner_contract_path} --json"
                ),
                runner_contract["review_output_contract"]["review_command"],
            ],
            "notes": [
                "This is a non-hardware CLI helper plan for sdk_cartesian runner contracts.",
                "It does not import or instantiate arx5_interface, and it does not move hardware.",
                "Use the generated session plan as the boundary artifact for a future mature SDK helper implementation.",
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


class EefMoveItHelperPlanExporter:
    def export(self, request: EefMoveItHelperPlanRequest) -> dict[str, object]:
        runner_contract = json.loads(
            request.runner_contract_path.read_text(encoding="utf-8")
        )
        if runner_contract.get("schema") != "armctrl.eef_runner_contract.v1":
            raise ValueError(
                "runner contract file must use schema armctrl.eef_runner_contract.v1"
            )
        resolved_backend = runner_contract.get("resolved_backend")
        if resolved_backend != "moveit_servo":
            raise RuntimeError(
                "unsupported runner backend for moveit helper plan: "
                f"{resolved_backend}"
            )
        bridge_export = dict(runner_contract["bridge_export"])
        ros_contract = dict(bridge_export["ros_contract"])
        environment = dict(bridge_export["environment"])
        message_type = str(ros_contract["message_type"])
        if message_type == "geometry_msgs/msg/TwistStamped":
            command_topic = "~/delta_twist_cmds"
            command_payload = {
                "frame_id": ros_contract["frame_id"],
                "twist": ros_contract["twist"],
                "control_period_s": ros_contract["control_period_s"],
            }
            command_mode = "twist"
        elif message_type == "geometry_msgs/msg/PoseStamped":
            command_topic = "~/pose_command_in_topic"
            command_payload = {
                "frame_id": ros_contract["frame_id"],
                "pose_6d": ros_contract["pose_6d"],
            }
            command_mode = "pose"
        else:
            raise ValueError(
                "moveit helper plan only supports TwistStamped or PoseStamped contracts"
            )
        payload = {
            "schema": "armctrl.moveit_servo_helper_plan.v1",
            "movement_allowed": False,
            "runner_contract_path": str(request.runner_contract_path),
            "plan_dir": str(runner_contract["plan_dir"]),
            "resolved_backend": resolved_backend,
            "runtime_owner": runner_contract["runner_api"]["owner"],
            "runtime_boundary": _moveit_servo_runtime_boundary(
                str(runner_contract["runner_api"]["owner"])
            ),
            "frequency_contract": _moveit_servo_frequency_contract(ros_contract),
            "servo_session_plan": {
                "session_kind": "ros2_servo_node",
                "command_mode": command_mode,
                "message_type": message_type,
                "command_topic": command_topic,
                "command_payload": command_payload,
                "status_topic": "~/status",
                "pause_service": "~/pause_servo",
                "switch_command_type_service": "~/switch_command_type",
                "command_out_topic_param": "command_out_topic",
                "command_out_type_param": "command_out_type",
                "supported_output_types": [
                    "trajectory_msgs/JointTrajectory",
                    "std_msgs/Float64MultiArray",
                ],
                "source_hint": environment["source_hint"],
            },
            "review_output_contract": runner_contract["review_output_contract"],
            "reference_docs": runner_contract["runner_api"]["reference_docs"],
            "ordered_steps": [
                {
                    "id": "sample_runner",
                    "command": (
                        "uv run armctrl eef sample-runner "
                        f"--runner-contract {request.runner_contract_path} --json"
                    ),
                },
                {
                    "id": "review_trajectory",
                    "depends_on": ["sample_runner"],
                    "command": runner_contract["review_output_contract"][
                        "review_command"
                    ],
                },
            ],
            "next_steps": [
                (
                    "uv run armctrl eef sample-runner "
                    f"--runner-contract {request.runner_contract_path} --json"
                ),
                runner_contract["review_output_contract"]["review_command"],
            ],
            "notes": [
                "This is a non-hardware CLI helper plan for MoveIt Servo runner contracts.",
                "It does not start ROS 2, publish commands, or move hardware.",
                "Use the generated session plan as the boundary artifact for a future mature MoveIt Servo helper implementation.",
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


class EefAgentSessionPlanExporter:
    def export(self, request: EefAgentSessionPlanRequest) -> dict[str, object]:
        agent_runtime_contract = EefAgentRuntimeContractExporter().export(
            EefAgentRuntimeContractRequest(
                plan_dir=request.plan_dir,
                backend=request.backend,
                model=request.model,
                interface=request.interface,
            )
        )
        resolved_backend = str(agent_runtime_contract["resolved_backend"])
        helper_plan: dict[str, object] | None
        runner_contract_path = request.plan_dir / "eef_runner_contract.json"
        if resolved_backend == "sdk_cartesian":
            EefRunnerContractExporter().export(
                EefRunnerContractRequest(
                    plan_dir=request.plan_dir,
                    backend=resolved_backend,
                    model=request.model,
                    interface=request.interface,
                    output_path=runner_contract_path,
                )
            )
            helper_plan = EefSdkHelperPlanExporter().export(
                EefSdkHelperPlanRequest(
                    runner_contract_path=runner_contract_path
                )
            )
        elif resolved_backend == "moveit_servo":
            EefRunnerContractExporter().export(
                EefRunnerContractRequest(
                    plan_dir=request.plan_dir,
                    backend=resolved_backend,
                    model=request.model,
                    interface=request.interface,
                    output_path=runner_contract_path,
                )
            )
            helper_plan = EefMoveItHelperPlanExporter().export(
                EefMoveItHelperPlanRequest(
                    runner_contract_path=runner_contract_path
                )
            )
        elif resolved_backend == "lerobot_rollout":
            helper_plan = None
        else:
            raise ValueError(
                f"unsupported agent session plan backend: {resolved_backend}"
            )
        payload = {
            "schema": "armctrl.eef_agent_session_plan.v1",
            "movement_allowed": False,
            "plan_dir": str(request.plan_dir),
            "requested_backend": request.backend,
            "resolved_backend": resolved_backend,
            "runtime_owner": agent_runtime_contract["runtime_owner"],
            "agent_runtime_contract": agent_runtime_contract,
            "helper_plan": helper_plan,
            "review_output_contract": dict(
                agent_runtime_contract["review_output_contract"]
            ),
            "recommended_path": _eef_agent_session_recommended_path(
                plan_dir=request.plan_dir,
                resolved_backend=resolved_backend,
            ),
            "ordered_steps": _agent_session_ordered_steps(
                plan_dir=request.plan_dir,
                resolved_backend=resolved_backend,
            ),
            "next_steps": _agent_session_next_steps(
                plan_dir=request.plan_dir,
                resolved_backend=resolved_backend,
            ),
            "notes": [
                "This is the top-level non-hardware session plan for Agent-driven realtime EEF control.",
                "Keep the Agent loop in the exported EEF action vocabulary and hand backend-native details to the helper plan or processor layer.",
                "armctrl still does not execute the mature backend session itself; it defines the approved session contract and the shared review return path.",
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


class EefSampleRunner:
    def run(self, request: EefSampleRunnerRequest) -> dict[str, object]:
        runner_contract = json.loads(
            request.runner_contract_path.read_text(encoding="utf-8")
        )
        if runner_contract.get("schema") != "armctrl.eef_runner_contract.v1":
            raise ValueError("runner contract file must use schema armctrl.eef_runner_contract.v1")
        plan_dir = Path(runner_contract["plan_dir"])
        delegated_preview = EefPreviewSynthesizer().synthesize(
            EefPreviewSynthesisRequest(
                plan_dir=plan_dir,
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
            "schema": "armctrl.eef_sample_runner.v1",
            "movement_allowed": False,
            "runner_contract_path": str(request.runner_contract_path),
            "consumed_runner_contract": runner_contract,
            "plan_dir": str(plan_dir),
            "resolved_backend": runner_contract["resolved_backend"],
            "delegated_preview": delegated_preview,
            "synthesized_trajectory": delegated_preview["synthesized_trajectory"],
            "review_status": review["review_status"],
            "sim_preview": review.get("sim_preview"),
            "next_gate": review["next_gate"],
            "next_steps": [
                runner_contract["review_output_contract"]["stage_command"],
                runner_contract["review_output_contract"]["review_command"],
            ],
            "notes": [
                "This is a non-hardware sample helper that consumes a serialized EEF runner contract file.",
                "It demonstrates how an external runtime helper can read the contract, emit the conventional reviewed trajectory artifact, and close the loop back into armctrl.",
                "armctrl still does not execute the mature backend session itself.",
            ],
        }


class EefLeRobotExporter:
    def export(self, request: EefLeRobotExportRequest) -> dict[str, object]:
        backend_request = json.loads(
            (request.plan_dir / "backend_request.json").read_text(encoding="utf-8")
        )
        eef_plan = json.loads(
            (request.plan_dir / "eef_plan.json").read_text(encoding="utf-8")
        )
        manifest = json.loads(
            (request.plan_dir / "manifest.json").read_text(encoding="utf-8")
        )
        lerobot_bridge = dict(backend_request["lerobot_bridge"])
        feature_order = list(lerobot_bridge["action_features"].keys())
        action_vector = [
            lerobot_bridge["value_mapping"][feature_name]
            for feature_name in feature_order
        ]
        lerobot_action = {
            "schema": "armctrl.eef_lerobot_action.v1",
            "control_mode": lerobot_bridge["control_mode"],
            "feature_order": feature_order,
            "action_vector": action_vector,
            "action_dict": dict(lerobot_bridge["value_mapping"]),
            "action_features": lerobot_bridge["action_features"],
            "preferred_native_surface": lerobot_bridge["preferred_native_surface"],
            "agent_eef_compatibility": {
                "status": "compatible",
                "agent_action_id": eef_plan["agent_action"]["action_id"],
                "agent_frame": eef_plan["agent_action"]["frame"],
                "agent_value_schema": eef_plan["agent_action"]["value"],
                "preferred_training_action_id": "eef.pose_delta",
                "processor_owner": {
                    "action": "robot_action_processor",
                    "observation": "robot_observation_processor",
                },
                "adaptation_rule": (
                    "Keep the upstream action in EEF space and let the LeRobot processor layer translate it into rollout-native commands."
                ),
            },
            "safety_contract": {
                **dict(lerobot_bridge["safety_contract"]),
                "movement_allowed": False,
                "review_status": "plan_only",
            },
        }
        payload = {
            "schema": "armctrl.eef_lerobot_action_export.v1",
            "movement_allowed": False,
            "plan_dir": str(request.plan_dir),
            "command_type": backend_request["command_type"],
            "backend": backend_request["backend"]["requested"],
            "lerobot_action": lerobot_action,
            "source_artifacts": {
                "backend_request": str(request.plan_dir / "backend_request.json"),
                "eef_plan": str(request.plan_dir / "eef_plan.json"),
                "manifest": str(request.plan_dir / "manifest.json"),
            },
            "notes": [
                "This export makes the EEF intent consumable as a LeRobot-friendly cartesian action sample.",
                "armctrl still requires shared review of a backend-produced joint trajectory before any future execution path.",
            ],
            "eef_plan": eef_plan,
            "manifest": manifest,
        }
        if request.output_path is not None:
            request.output_path.parent.mkdir(parents=True, exist_ok=True)
            request.output_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            payload["output"] = str(request.output_path)
        return payload


class EefSdkCartesianExporter:
    def export(self, request: EefSdkCartesianExportRequest) -> dict[str, object]:
        backend_request = json.loads(
            (request.plan_dir / "backend_request.json").read_text(encoding="utf-8")
        )
        eef_plan = json.loads(
            (request.plan_dir / "eef_plan.json").read_text(encoding="utf-8")
        )
        manifest = json.loads(
            (request.plan_dir / "manifest.json").read_text(encoding="utf-8")
        )
        backend_requested = backend_request["backend"]["requested"]
        if backend_requested != "sdk_cartesian":
            raise ValueError(
                "sdk cartesian export requires an eef plan whose requested backend is sdk_cartesian"
            )
        command_type = str(backend_request["command_type"])
        command = dict(backend_request["command"])
        if command_type == "pose":
            command_mode = "pose_6d_absolute"
            waypoint = {
                "timestamp_offset_s": 0.0,
                "pose_6d": [
                    *list(command["position_m"]),
                    *list(command["rpy_rad"]),
                ],
            }
            submit_call = "controller.set_eef_cmd(eef_cmd)"
        else:
            command_mode = "pose_6d_delta"
            deltas = backend_request["lerobot_bridge"]["value_mapping"]
            waypoint = {
                "timestamp_offset_s": 0.0,
                "pose_6d_delta": [
                    deltas["eef.dx"],
                    deltas["eef.dy"],
                    deltas["eef.dz"],
                    deltas["eef.droll"],
                    deltas["eef.dpitch"],
                    deltas["eef.dyaw"],
                ],
            }
            submit_call = "controller.set_eef_cmd(eef_cmd)"
        payload = {
            "schema": "armctrl.eef_sdk_cartesian_export.v1",
            "movement_allowed": False,
            "plan_dir": str(request.plan_dir),
            "backend": "sdk_cartesian",
            "model": request.model,
            "interface": request.interface,
            "runtime_owner": "arx5_interface_cartesian_controller",
            "sdk_controller": {
                "controller_kind": "cartesian_controller",
                "controller_class": "Arx5CartesianController",
                "command_object": "EEFState",
                "command_mode": command_mode,
                "submit_call": submit_call,
            },
            "sdk_request": {
                "robot": {
                    "model": request.model,
                    "interface": request.interface,
                },
                "control_frame": command["frame"],
                "waypoint_count": 1,
                "eef_waypoints": [waypoint],
            },
            "source_artifacts": {
                "backend_request": str(request.plan_dir / "backend_request.json"),
                "eef_plan": str(request.plan_dir / "eef_plan.json"),
                "manifest": str(request.plan_dir / "manifest.json"),
            },
            "reference_examples": [
                "vendor/real_stanford_arx5_sdk/python/examples/keyboard_teleop.py",
                "vendor/real_stanford_arx5_sdk/python/examples/cartesian_waypoint_scheduling.py",
            ],
            "notes": [
                "This export preserves armctrl as a non-hardware orchestration layer and emits a programmatic ARX5 SDK cartesian bridge artifact only.",
                "A mature backend still owns turning this request into real controller calls or a reviewed joint trajectory.",
            ],
            "eef_plan": eef_plan,
            "manifest": manifest,
        }
        if request.output_path is not None:
            request.output_path.parent.mkdir(parents=True, exist_ok=True)
            request.output_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            payload["output"] = str(request.output_path)
        return payload


class EefMoveItServoExporter:
    def export(self, request: EefMoveItServoExportRequest) -> dict[str, object]:
        backend_request = json.loads(
            (request.plan_dir / "backend_request.json").read_text(encoding="utf-8")
        )
        eef_plan = json.loads(
            (request.plan_dir / "eef_plan.json").read_text(encoding="utf-8")
        )
        manifest = json.loads(
            (request.plan_dir / "manifest.json").read_text(encoding="utf-8")
        )
        backend_requested = backend_request["backend"]["requested"]
        command_type = str(backend_request["command_type"])
        if backend_requested != "moveit_servo":
            raise ValueError(
                "moveit servo export requires an eef plan whose requested backend is moveit_servo"
            )
        command = dict(backend_request["command"])
        ros_distro = _installed_ros_distribution()
        if command_type == "twist":
            ros_contract = {
                "message_type": "geometry_msgs/msg/TwistStamped",
                "command_topic_kind": "cartesian_servo_twist",
                "frame_id": command["frame"],
                "twist": {
                    "linear": list(command["linear_mps"]),
                    "angular": list(command["angular_rps"]),
                },
                "control_period_s": command["control_period_s"],
            }
        elif command_type == "pose":
            ros_contract = {
                "message_type": "geometry_msgs/msg/PoseStamped",
                "command_topic_kind": "cartesian_servo_pose",
                "frame_id": command["frame"],
                "pose_6d": [
                    *list(command["position_m"]),
                    *list(command["rpy_rad"]),
                ],
            }
        elif command_type == "pose_delta":
            control_period_s = float(command["control_period_s"])
            ros_contract = {
                "message_type": "geometry_msgs/msg/TwistStamped",
                "command_topic_kind": "cartesian_servo_twist_from_pose_delta",
                "frame_id": command["frame"],
                "twist": {
                    "linear": [
                        float(value) / control_period_s
                        for value in command["delta_position_m"]
                    ],
                    "angular": [
                        float(value) / control_period_s
                        for value in command["delta_rpy_rad"]
                    ],
                },
                "control_period_s": control_period_s,
                "source_action_id": "eef.pose_delta",
                "delta_pose": {
                    "delta_position_m": list(command["delta_position_m"]),
                    "delta_rpy_rad": list(command["delta_rpy_rad"]),
                },
            }
        else:
            raise ValueError(
                f"moveit servo export does not support command_type={command_type}"
            )
        payload = {
            "schema": "armctrl.eef_moveit_servo_export.v1",
            "movement_allowed": False,
            "plan_dir": str(request.plan_dir),
            "backend": "moveit_servo",
            "runtime_owner": "ros2_moveit_servo",
            "ros_contract": ros_contract,
            "environment": {
                "ros_distro": ros_distro,
                "source_hint": (
                    f"source /opt/ros/{ros_distro}/setup.bash"
                    if ros_distro is not None
                    else "source /opt/ros/<distro>/setup.bash"
                ),
            },
            "source_artifacts": {
                "backend_request": str(request.plan_dir / "backend_request.json"),
                "eef_plan": str(request.plan_dir / "eef_plan.json"),
                "manifest": str(request.plan_dir / "manifest.json"),
            },
            "reference_docs": [
                "https://docs.ros.org/en/jazzy/p/moveit_servo/",
            ],
            "notes": [
                "This export preserves armctrl as a non-hardware orchestration layer and emits a ROS 2 MoveIt Servo bridge artifact only.",
                "A mature backend still owns turning this EEF request into a realtime servo session or a reviewed joint trajectory.",
            ],
            "eef_plan": eef_plan,
            "manifest": manifest,
        }
        if request.output_path is not None:
            request.output_path.parent.mkdir(parents=True, exist_ok=True)
            request.output_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            payload["output"] = str(request.output_path)
        return payload


def _safety_contract(config: WorkspaceSafetyConfig) -> dict[str, object]:
    return {
        "workspace_gate_required": True,
        "backend_execution_deferred": True,
        "max_translation_step_m": config.max_translation_step_m,
        "max_rotation_step_rad": config.max_rotation_step_rad,
        "allowed_workspace_boxes": [
            box.name for box in config.allowed_workspace_boxes
        ],
        "forbidden_workspace_boxes": [
            box.name for box in config.forbidden_workspace_boxes
        ],
    }


def _evaluate_pose_gate(
    config: WorkspaceSafetyConfig,
    request: EefPoseRequest,
) -> dict[str, object]:
    violations: list[dict[str, object]] = []
    position = request.position_m
    if not any(box.contains(position) for box in config.allowed_workspace_boxes):
        violations.append(
            {
                "check": "outside_allowed_workspace",
                "frame": request.frame,
                "position_m": list(position),
                "allowed_boxes": [box.name for box in config.allowed_workspace_boxes],
            }
        )
    forbidden = next(
        (box for box in config.forbidden_workspace_boxes if box.contains(position)),
        None,
    )
    if forbidden is not None:
        violations.append(
            {
                "check": "inside_forbidden_workspace",
                "frame": request.frame,
                "position_m": list(position),
                "forbidden_box": forbidden.name,
            }
        )
    return {
        "allowed": not violations,
        "position_box_check": {
            "status": "fail" if violations else "pass",
            "method": "workspace_box_zones",
            "violations": violations,
        },
        "simulation_chain": {
            "status": "precheck_only",
            "reason": "A mature backend must still synthesize and preview the joint-space motion before hardware execution.",
        },
    }


def _evaluate_twist_gate(
    config: WorkspaceSafetyConfig,
    request: EefTwistRequest,
) -> dict[str, object]:
    translation_step = max(abs(value) for value in request.linear_mps) * request.control_period_s
    rotation_step = max(abs(value) for value in request.angular_rps) * request.control_period_s
    violations: list[dict[str, object]] = []
    if translation_step > config.max_translation_step_m:
        violations.append(
            {
                "check": "translation_step_limit",
                "value": translation_step,
                "maximum": config.max_translation_step_m,
            }
        )
    if rotation_step > config.max_rotation_step_rad:
        violations.append(
            {
                "check": "rotation_step_limit",
                "value": rotation_step,
                "maximum": config.max_rotation_step_rad,
            }
        )
    return {
        "allowed": not violations,
        "twist_step_check": {
            "status": "fail" if violations else "pass",
            "method": "command_step_limit",
            "violations": violations,
        },
        "simulation_chain": {
            "status": "precheck_only",
            "reason": "A mature backend must still synthesize and preview the joint-space servo motion before hardware execution.",
        },
    }


def _evaluate_delta_pose_gate(
    config: WorkspaceSafetyConfig,
    request: EefDeltaPoseRequest,
) -> dict[str, object]:
    translation_step = max(abs(value) for value in request.delta_position_m)
    rotation_step = max(abs(value) for value in request.delta_rpy_rad)
    violations: list[dict[str, object]] = []
    if translation_step > config.max_translation_step_m:
        violations.append(
            {
                "check": "translation_step_limit",
                "value": translation_step,
                "maximum": config.max_translation_step_m,
            }
        )
    if rotation_step > config.max_rotation_step_rad:
        violations.append(
            {
                "check": "rotation_step_limit",
                "value": rotation_step,
                "maximum": config.max_rotation_step_rad,
            }
        )
    return {
        "allowed": not violations,
        "delta_step_check": {
            "status": "fail" if violations else "pass",
            "method": "command_step_limit",
            "violations": violations,
        },
        "simulation_chain": {
            "status": "precheck_only",
            "reason": "A mature backend must still synthesize and preview the joint-space servo motion before hardware execution.",
        },
    }


def _write_plan_artifacts(
    payload: dict[str, object],
    output_dir: Path | None,
    safe_config_path: str,
) -> dict[str, object]:
    if output_dir is None:
        return payload
    output_dir.mkdir(parents=True, exist_ok=True)
    plan_path = output_dir / "eef_plan.json"
    manifest_path = output_dir / "manifest.json"
    backend_request_path = output_dir / "backend_request.json"
    backend_review_contract_path = output_dir / "backend_review_contract.json"
    plan_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    backend_request_path.write_text(
        json.dumps(
            _backend_request_payload(payload, safe_config_path),
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    backend_review_contract_path.write_text(
        json.dumps(
            _backend_review_contract_payload(output_dir=output_dir, payload=payload),
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    manifest = {
        "schema": "armctrl.eef_plan_manifest.v1",
        "command_type": payload["command_type"],
        "boundary": payload["boundary"],
        "gate": payload["gate"],
        "artifacts": {
            "eef_plan": str(plan_path),
            "manifest": str(manifest_path),
            "backend_request": str(backend_request_path),
            "backend_review_contract": str(backend_review_contract_path),
        },
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    payload = dict(payload)
    payload["artifacts"] = {
        "eef_plan": str(plan_path),
        "manifest": str(manifest_path),
        "backend_request": str(backend_request_path),
        "backend_review_contract": str(backend_review_contract_path),
    }
    return payload


def _backend_handoff_for_twist(request: EefTwistRequest) -> dict[str, object]:
    if request.backend == "moveit_servo":
        return {
            "requested": "moveit_servo",
            "selected": "moveit_servo_twist_command",
            "oob_execution": False,
            "implementation_boundary": "armctrl orchestrates; mature EEF backends own IK/servo math",
            "command_surface": "TwistStamped",
            "reference_runtime": "ros2_moveit_servo",
        }
    if request.backend == "sdk_cartesian":
        return {
            "requested": "sdk_cartesian",
            "selected": "sdk_cartesian_twist_delta",
            "oob_execution": False,
            "implementation_boundary": "armctrl orchestrates; mature EEF backends own IK/servo math",
            "command_surface": "EEFState.pose_6d_delta",
            "reference_runtime": "arx5_interface_cartesian_controller",
        }
    return {
        "requested": request.backend,
        "selected": "backend_specific_twist_command",
        "oob_execution": False,
        "implementation_boundary": "armctrl orchestrates; mature EEF backends own IK/servo math",
        "command_surface": "backend_specific",
        "reference_runtime": "external_backend",
    }


def _backend_handoff_for_delta_pose(request: EefDeltaPoseRequest) -> dict[str, object]:
    if request.backend == "sdk_cartesian":
        return {
            "requested": "sdk_cartesian",
            "selected": "sdk_cartesian_pose_delta_command",
            "oob_execution": False,
            "implementation_boundary": "armctrl orchestrates; mature EEF backends own IK/servo math",
            "command_surface": "EEFState.pose_6d_delta",
            "reference_runtime": "arx5_interface_cartesian_controller",
        }
    if request.backend == "moveit_servo":
        return {
            "requested": "moveit_servo",
            "selected": "moveit_servo_pose_delta_via_twist_command",
            "oob_execution": False,
            "implementation_boundary": "armctrl orchestrates; mature EEF backends own IK/servo math",
            "command_surface": "TwistStamped",
            "reference_runtime": "ros2_moveit_servo",
        }
    return {
        "requested": request.backend,
        "selected": "backend_specific_pose_delta_command",
        "oob_execution": False,
        "implementation_boundary": "armctrl orchestrates; mature EEF backends own IK/servo math",
        "command_surface": "backend_specific",
        "reference_runtime": "external_backend",
    }


def _backend_handoff_for_pose(request: EefPoseRequest) -> dict[str, object]:
    if request.backend == "pink":
        selected = "pink_pose_task_request"
        command_surface = "SE3 pose task"
        runtime = "python_pinocchio_pink"
    elif request.backend == "moveit_servo":
        selected = "moveit_servo_pose_command"
        command_surface = "geometry_msgs/msg/PoseStamped"
        runtime = "ros2_moveit_servo"
    elif request.backend == "sdk_cartesian":
        selected = "sdk_cartesian_pose_command"
        command_surface = "EEFState.pose_6d"
        runtime = "arx5_interface_cartesian_controller"
    else:
        selected = "sdk_cartesian_eef_request"
        command_surface = "EEFState pose_6d"
        runtime = "arx5_cartesian_controller"
    return {
        "requested": request.backend,
        "selected": selected,
        "oob_execution": False,
        "implementation_boundary": "armctrl orchestrates; mature EEF backends own IK/servo math",
        "command_surface": command_surface,
        "reference_runtime": runtime,
    }


def _backend_request_payload(
    payload: dict[str, object],
    safe_config_path: str,
) -> dict[str, object]:
    return {
        "schema": "armctrl.eef_backend_request.v1",
        "command_type": payload["command_type"],
        "backend": {
            "requested": payload["backend_handoff"]["requested"],
            "selected": payload["backend_handoff"]["selected"],
            "command_surface": payload["backend_handoff"]["command_surface"],
            "reference_runtime": payload["backend_handoff"]["reference_runtime"],
        },
        "command": payload["command"],
        "gate": payload["gate"],
        "lerobot_bridge": payload["lerobot_bridge"],
        "safe_config_path": safe_config_path,
        "implementation_boundary": payload["backend_handoff"]["implementation_boundary"],
    }


def _artifact_contract() -> dict[str, object]:
    return {
        "plan_artifacts": [
            "eef_plan.json",
            "backend_request.json",
            "backend_review_contract.json",
        ],
        "review_artifact": "backend_joint_trajectory.csv",
        "staging_command": (
            "armctrl eef stage-trajectory --plan-dir <dir> --trajectory <path>"
        ),
        "review_command": "armctrl eef review --plan-dir <dir> [--trajectory <path>]",
    }


def _eef_agent_runtime_profile(
    *,
    plan_dir: Path,
    runtime_owner: str,
    preferred_next_surface: str,
) -> dict[str, object]:
    return {
        "schema": "armctrl.agent_runtime_profile.v1",
        "profile": "eef_runtime_handoff",
        "movement_allowed": False,
        "plan_dir": str(plan_dir),
        "runtime_owner": runtime_owner,
        "preferred_next_surface": preferred_next_surface,
        "review_chain": "shared_simulation_preview",
        "required_artifacts": {
            "backend_request": str(plan_dir / "backend_request.json"),
            "backend_review_contract": str(plan_dir / "backend_review_contract.json"),
            "eef_plan": str(plan_dir / "eef_plan.json"),
        },
    }


def _moveit_servo_runtime_boundary(runtime_owner: str) -> dict[str, object]:
    return {
        "runtime_owner": runtime_owner,
        "armctrl_role": "contract_preview_audit_only",
        "motion_runtime_owner": False,
        "hardware_execution": "outside_armctrl",
    }


def _moveit_servo_frequency_contract(
    ros_contract: dict[str, object],
) -> dict[str, object]:
    control_period_s = ros_contract.get("control_period_s")
    agent_intent_hz = None
    if isinstance(control_period_s, int | float) and float(control_period_s) > 0.0:
        agent_intent_hz = 1.0 / float(control_period_s)
    return {
        "agent_intent_hz": agent_intent_hz,
        "command_publish_hz": "runtime_configured",
        "servo_loop_hz": "moveit_servo_runtime_configured",
        "actual_send_hz": "measure_in_runtime_artifact",
        "controller_dt_s": "not_owned_by_armctrl",
        "timestamp_policy": "ros_clock_or_servo_runtime_policy",
    }


def _agent_action_for_pose(
    request: EefPoseRequest, *, backend_handoff: dict[str, object]
) -> dict[str, object]:
    return {
        "schema": "armctrl.agent_eef_action.v1",
        "action_id": "eef.pose_absolute",
        "frame": request.frame,
        "value": {
            "position_m": list(request.position_m),
            "rpy_rad": list(request.rpy_rad),
        },
        "backend_mapping": {
            "requested_backend": request.backend,
            "command_surface": backend_handoff["command_surface"],
            "reference_runtime": backend_handoff["reference_runtime"],
        },
    }


def _agent_action_for_twist(
    request: EefTwistRequest, *, backend_handoff: dict[str, object]
) -> dict[str, object]:
    return {
        "schema": "armctrl.agent_eef_action.v1",
        "action_id": "eef.twist",
        "frame": request.frame,
        "value": {
            "linear_mps": list(request.linear_mps),
            "angular_rps": list(request.angular_rps),
            "control_period_s": request.control_period_s,
        },
        "backend_mapping": {
            "requested_backend": request.backend,
            "command_surface": backend_handoff["command_surface"],
            "reference_runtime": backend_handoff["reference_runtime"],
        },
    }


def _agent_action_for_delta_pose(
    request: EefDeltaPoseRequest, *, backend_handoff: dict[str, object]
) -> dict[str, object]:
    return {
        "schema": "armctrl.agent_eef_action.v1",
        "action_id": "eef.pose_delta",
        "frame": request.frame,
        "value": {
            "delta_position_m": list(request.delta_position_m),
            "delta_rpy_rad": list(request.delta_rpy_rad),
            "control_period_s": request.control_period_s,
        },
        "backend_mapping": {
            "requested_backend": request.backend,
            "command_surface": backend_handoff["command_surface"],
            "reference_runtime": backend_handoff["reference_runtime"],
        },
    }


def _runtime_next_steps(*, backend: str, plan_dir: Path) -> list[str]:
    plan_dir_str = str(plan_dir)
    stage_step = (
        f"uv run armctrl eef stage-trajectory --plan-dir {plan_dir_str} "
        f"--trajectory <joint_csv> --json"
    )
    review_step = f"uv run armctrl eef review --plan-dir {plan_dir_str} --json"
    if backend == "sdk_cartesian":
        return [
            f"uv run armctrl eef export-sdk-cartesian --plan-dir {plan_dir_str} --json",
            stage_step,
            review_step,
        ]
    if backend == "moveit_servo":
        return [
            f"uv run armctrl eef export-moveit-servo --plan-dir {plan_dir_str} --json",
            stage_step,
            review_step,
        ]
    if backend == "lerobot_rollout":
        return [
            f"uv run armctrl eef export-lerobot-action --plan-dir {plan_dir_str} --json",
            f"uv run armctrl lerobot export-processor-contract --eef-plan-dir {plan_dir_str} --json",
            stage_step,
            review_step,
        ]
    return [stage_step, review_step]


def _agent_session_ordered_steps(
    *, plan_dir: Path, resolved_backend: str
) -> list[dict[str, object]]:
    plan_dir_str = str(plan_dir)
    steps: list[dict[str, object]] = [
        {
            "id": "export_agent_runtime_contract",
            "command": (
                "uv run armctrl eef export-agent-runtime-contract "
                f"--plan-dir {plan_dir_str} --backend {resolved_backend} --json"
            ),
            "depends_on": [],
            "parallel_safe_with": [],
        }
    ]
    if resolved_backend == "sdk_cartesian":
        steps.append(
            {
                "id": "export_backend_helper_plan",
                "command": (
                    "uv run armctrl eef export-sdk-helper-plan "
                    f"--runner-contract {plan_dir / 'eef_runner_contract.json'} --json"
                ),
                "depends_on": ["export_agent_runtime_contract"],
                "parallel_safe_with": [],
            }
        )
    elif resolved_backend == "moveit_servo":
        steps.append(
            {
                "id": "export_backend_helper_plan",
                "command": (
                    "uv run armctrl eef export-moveit-helper-plan "
                    f"--runner-contract {plan_dir / 'eef_runner_contract.json'} --json"
                ),
                "depends_on": ["export_agent_runtime_contract"],
                "parallel_safe_with": [],
            }
        )
    elif resolved_backend == "lerobot_rollout":
        steps.append(
            {
                "id": "export_processor_contract",
                "command": (
                    "uv run armctrl lerobot export-processor-contract "
                    f"--eef-plan-dir {plan_dir_str} --json"
                ),
                "depends_on": ["export_agent_runtime_contract"],
                "parallel_safe_with": [],
            }
        )
    steps.append(
        {
            "id": "review_trajectory",
            "command": f"uv run armctrl eef review --plan-dir {plan_dir_str} --json",
            "depends_on": [steps[-1]["id"]],
            "parallel_safe_with": [],
        }
    )
    return steps


def _agent_session_next_steps(*, plan_dir: Path, resolved_backend: str) -> list[str]:
    plan_dir_str = str(plan_dir)
    steps = [
        "uv run armctrl eef export-agent-runtime-contract "
        f"--plan-dir {plan_dir_str} --backend {resolved_backend} --json"
    ]
    if resolved_backend == "sdk_cartesian":
        steps.append(
            "uv run armctrl eef export-sdk-helper-plan "
            f"--runner-contract {plan_dir / 'eef_runner_contract.json'} --json"
        )
    elif resolved_backend == "moveit_servo":
        steps.append(
            "uv run armctrl eef export-moveit-helper-plan "
            f"--runner-contract {plan_dir / 'eef_runner_contract.json'} --json"
        )
    elif resolved_backend == "lerobot_rollout":
        steps.append(
            "uv run armctrl lerobot export-processor-contract "
            f"--eef-plan-dir {plan_dir_str} --json"
        )
    steps.append(f"uv run armctrl eef review --plan-dir {plan_dir_str} --json")
    return steps


def _eef_agent_session_recommended_path(
    *, plan_dir: Path, resolved_backend: str
) -> dict[str, object]:
    plan_dir_str = str(plan_dir)
    payload = {
        "schema": "armctrl.recommended_path.v1",
        "profile": "realtime_eef_loop_with_shared_review",
        "primary_entrypoint": "armctrl eef export-agent-session-plan",
        "why": (
            "Use one top-level EEF session contract for Agent loops instead of manually combining runtime contracts, helper plans, and review commands."
        ),
        "steps": [
            {
                "id": "export_agent_session_plan",
                "command": (
                    "uv run armctrl eef export-agent-session-plan "
                    f"--plan-dir {plan_dir_str} --backend {resolved_backend} --json"
                ),
            }
        ],
    }
    if resolved_backend == "sdk_cartesian":
        payload["steps"].append(
            {
                "id": "export_backend_helper_plan",
                "command": (
                    "uv run armctrl eef export-sdk-helper-plan "
                    f"--runner-contract {plan_dir / 'eef_runner_contract.json'} --json"
                ),
            }
        )
    elif resolved_backend == "moveit_servo":
        payload["steps"].append(
            {
                "id": "export_backend_helper_plan",
                "command": (
                    "uv run armctrl eef export-moveit-helper-plan "
                    f"--runner-contract {plan_dir / 'eef_runner_contract.json'} --json"
                ),
            }
        )
    elif resolved_backend == "lerobot_rollout":
        payload["steps"].append(
            {
                "id": "export_processor_contract",
                "command": (
                    "uv run armctrl lerobot export-processor-contract "
                    f"--eef-plan-dir {plan_dir_str} --json"
                ),
            }
        )
    payload["steps"].append(
        {
            "id": "review_trajectory",
            "command": f"uv run armctrl eef review --plan-dir {plan_dir_str} --json",
        }
    )
    return payload


def _attach_runtime_plan_artifacts(
    payload: dict[str, object],
    *,
    backend_request: dict[str, object] | None,
    backend_review_contract: dict[str, object] | None,
    plan_dir: Path | None,
) -> dict[str, object]:
    if backend_request is not None:
        payload["backend_request"] = backend_request
    if backend_review_contract is not None:
        payload["backend_review_contract"] = backend_review_contract
    if plan_dir is not None:
        payload["plan_dir"] = str(plan_dir)
    return payload


def _backend_review_contract_payload(
    *,
    output_dir: Path,
    payload: dict[str, object],
) -> dict[str, object]:
    expected_trajectory_path = output_dir / "backend_joint_trajectory.csv"
    return {
        "schema": "armctrl.eef_backend_review_contract.v1",
        "command_type": payload["command_type"],
        "expected_backend_trajectory": str(expected_trajectory_path),
        "trajectory_format": {
            "type": "joint_space_csv",
            "required_columns": [
                "time_s",
                "q_cmd_1",
                "q_cmd_2",
                "q_cmd_3",
                "q_cmd_4",
                "q_cmd_5",
                "q_cmd_6",
            ],
        },
        "review_cli": [
            "armctrl",
            "eef",
            "review",
            "--plan-dir",
            str(output_dir),
        ],
        "notes": [
            "Mature backends should synthesize a joint trajectory from backend_request.json and write it to the expected path.",
            "armctrl eef review will auto-discover the default backend trajectory if it is present.",
        ],
    }


def _lerobot_bridge_for_pose(request: EefPoseRequest) -> dict[str, object]:
    return {
        "schema": "armctrl.eef_lerobot_bridge.v1",
        "alignment": "lerobot_friendly_cartesian_contract",
        "control_mode": "cartesian_pose_absolute",
        "preferred_native_surface": "lerobot-rollout",
        "action_features": {
            "eef.x": {"type": "scalar", "unit": "m"},
            "eef.y": {"type": "scalar", "unit": "m"},
            "eef.z": {"type": "scalar", "unit": "m"},
            "eef.roll": {"type": "scalar", "unit": "rad"},
            "eef.pitch": {"type": "scalar", "unit": "rad"},
            "eef.yaw": {"type": "scalar", "unit": "rad"},
        },
        "value_mapping": {
            "eef.x": request.position_m[0],
            "eef.y": request.position_m[1],
            "eef.z": request.position_m[2],
            "eef.roll": request.rpy_rad[0],
            "eef.pitch": request.rpy_rad[1],
            "eef.yaw": request.rpy_rad[2],
        },
        "safety_contract": {
            "armctrl_review_required": True,
            "joint_trajectory_preview_required": True,
        },
    }


def _lerobot_bridge_for_twist(request: EefTwistRequest) -> dict[str, object]:
    return {
        "schema": "armctrl.eef_lerobot_bridge.v1",
        "alignment": "lerobot_friendly_cartesian_contract",
        "control_mode": "cartesian_delta",
        "preferred_native_surface": "lerobot-rollout",
        "action_features": {
            "eef.dx": {"type": "scalar", "unit": "m"},
            "eef.dy": {"type": "scalar", "unit": "m"},
            "eef.dz": {"type": "scalar", "unit": "m"},
            "eef.droll": {"type": "scalar", "unit": "rad"},
            "eef.dpitch": {"type": "scalar", "unit": "rad"},
            "eef.dyaw": {"type": "scalar", "unit": "rad"},
        },
        "value_mapping": {
            "eef.dx": request.linear_mps[0] * request.control_period_s,
            "eef.dy": request.linear_mps[1] * request.control_period_s,
            "eef.dz": request.linear_mps[2] * request.control_period_s,
            "eef.droll": request.angular_rps[0] * request.control_period_s,
            "eef.dpitch": request.angular_rps[1] * request.control_period_s,
            "eef.dyaw": request.angular_rps[2] * request.control_period_s,
        },
        "safety_contract": {
            "armctrl_review_required": True,
            "joint_trajectory_preview_required": True,
        },
    }


def _lerobot_bridge_for_delta_pose(request: EefDeltaPoseRequest) -> dict[str, object]:
    return {
        "schema": "armctrl.eef_lerobot_bridge.v1",
        "alignment": "lerobot_friendly_cartesian_contract",
        "control_mode": "cartesian_delta",
        "preferred_native_surface": "lerobot-rollout",
        "action_features": {
            "eef.dx": {"type": "scalar", "unit": "m"},
            "eef.dy": {"type": "scalar", "unit": "m"},
            "eef.dz": {"type": "scalar", "unit": "m"},
            "eef.droll": {"type": "scalar", "unit": "rad"},
            "eef.dpitch": {"type": "scalar", "unit": "rad"},
            "eef.dyaw": {"type": "scalar", "unit": "rad"},
        },
        "value_mapping": {
            "eef.dx": request.delta_position_m[0],
            "eef.dy": request.delta_position_m[1],
            "eef.dz": request.delta_position_m[2],
            "eef.droll": request.delta_rpy_rad[0],
            "eef.dpitch": request.delta_rpy_rad[1],
            "eef.dyaw": request.delta_rpy_rad[2],
        },
        "safety_contract": {
            "armctrl_review_required": True,
            "joint_trajectory_preview_required": True,
        },
    }


def _eef_notes() -> list[str]:
    return [
        "MoveIt Servo should own realtime Cartesian servo execution on ROS 2 hosts.",
        "Pink can provide bounded differential IK planning when a lightweight local backend is enough.",
        "ARX5 SDK Cartesian control can remain the execution owner on robot workstations that already use arx5_interface directly.",
        "EEF plan payloads expose a LeRobot-aligned cartesian action mapping so Agent and rollout surfaces can converge on one motion vocabulary.",
        "armctrl should first run safety-space and simulation gates before any future hardware execution path is enabled.",
    ]


def _synthesize_preview_target(
    *,
    backend_request: dict[str, object],
    start_joints: tuple[float, ...],
    urdf_path: Path,
) -> tuple[tuple[float, ...], dict[str, object]]:
    command_type = str(backend_request["command_type"])
    command = dict(backend_request["command"])
    frame = str(command["frame"])
    if command_type == "pose":
        command_position = tuple(float(value) for value in command["position_m"])
        command_rpy = tuple(float(value) for value in command["rpy_rad"])
    elif command_type == "pose_delta":
        command_position, command_rpy = _compose_delta_pose_preview_target(
            start_joints=start_joints,
            frame=frame,
            delta_position_m=tuple(float(value) for value in command["delta_position_m"]),
            delta_rpy_rad=tuple(float(value) for value in command["delta_rpy_rad"]),
            urdf_path=urdf_path,
        )
    else:
        raise ValueError(
            f"preview synthesis does not support command_type={command_type}"
        )
    if _module_available("pink") and _module_available("pinocchio"):
        try:
            target = _solve_pose_with_pink(
                urdf_path=urdf_path,
                frame=frame,
                position_m=command_position,
                rpy_rad=command_rpy,
                start_joints=start_joints,
            )
            return target, {
                "status": "ok",
                "method": "pink_offline_preview_solver",
                "degraded": False,
            }
        except Exception as error:
            fallback = _heuristic_preview_target(
                position_m=command_position,
                rpy_rad=command_rpy,
                start_joints=start_joints,
            )
            return fallback, {
                "status": "degraded",
                "method": "heuristic_joint_projection_after_pink_failure",
                "degraded": True,
                "reason": str(error),
            }
    fallback = _heuristic_preview_target(
        position_m=command_position,
        rpy_rad=command_rpy,
        start_joints=start_joints,
    )
    return fallback, {
        "status": "degraded",
        "method": "heuristic_joint_projection_without_pink",
        "degraded": True,
        "reason": "pink/pinocchio not importable in this environment",
    }


def _compose_delta_pose_preview_target(
    *,
    start_joints: tuple[float, ...],
    frame: str,
    delta_position_m: tuple[float, float, float],
    delta_rpy_rad: tuple[float, float, float],
    urdf_path: Path,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if _module_available("pinocchio"):
        try:
            current_position, current_rpy = _forward_pose_with_pinocchio(
                urdf_path=urdf_path,
                frame=frame,
                start_joints=start_joints,
            )
            return (
                tuple(
                    current + delta
                    for current, delta in zip(current_position, delta_position_m)
                ),
                tuple(
                    current + delta
                    for current, delta in zip(current_rpy, delta_rpy_rad)
                ),
            )
        except Exception:
            pass
    return _heuristic_delta_pose_target(
        start_joints=start_joints,
        delta_position_m=delta_position_m,
        delta_rpy_rad=delta_rpy_rad,
    )


def _heuristic_delta_pose_target(
    *,
    start_joints: tuple[float, ...],
    delta_position_m: tuple[float, float, float],
    delta_rpy_rad: tuple[float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    q1, q2, q3, q4, q5, q6 = start_joints
    dx, dy, dz = delta_position_m
    droll, dpitch, dyaw = delta_rpy_rad
    heuristic_position = (
        0.25 + (q3 - 0.30) * 0.12 + dx,
        (q1 * 0.25) + dy,
        0.35 - (q2 - 0.30) * 0.10 + dz,
    )
    heuristic_rpy = (
        q4 * 0.5 + droll,
        q5 * 0.5 + dpitch,
        q6 * 0.5 + dyaw,
    )
    return heuristic_position, heuristic_rpy


def _heuristic_preview_target(
    *,
    position_m: tuple[float, float, float],
    rpy_rad: tuple[float, float, float],
    start_joints: tuple[float, ...],
) -> tuple[float, ...]:
    x, y, z = position_m
    roll, pitch, yaw = rpy_rad
    q1 = max(-0.5, min(0.5, y * 1.2))
    q2 = max(0.2, min(0.9, 0.30 + (0.35 - z) * 1.2))
    q3 = max(0.2, min(0.9, 0.30 + (x - 0.25) * 1.4))
    q4 = max(-0.4, min(0.4, roll * 0.5))
    q5 = max(-0.4, min(0.4, pitch * 0.5))
    q6 = max(-0.5, min(0.5, yaw * 0.5))
    target = (q1, q2, q3, q4, q5, q6)
    blended: list[float] = []
    for start_q, target_q in zip(start_joints, target):
        delta = target_q - start_q
        limited_delta = max(-0.35, min(0.35, delta))
        blended.append(start_q + limited_delta)
    return tuple(blended)


def _resolve_preview_start_joints(
    request: EefPreviewSynthesisRequest,
) -> tuple[tuple[float, ...], str]:
    if request.start_joints is not None:
        return request.start_joints, "explicit_cli"
    if request.recipe_plan_dir is not None:
        seed_path = request.recipe_plan_dir / "eef_seed.json"
        if not seed_path.exists():
            raise FileNotFoundError(
                f"missing recipe EEF seed artifact required for preview synthesis: {seed_path}"
            )
        seed_payload = json.loads(seed_path.read_text(encoding="utf-8"))
        final_joints = seed_payload.get("final_joints")
        if not isinstance(final_joints, list) or len(final_joints) != 6:
            raise ValueError(
                "recipe EEF seed final_joints must be a 6-element list for preview synthesis"
            )
        return tuple(float(value) for value in final_joints), "recipe_eef_seed"
    return (0.0, 0.30, 0.30, 0.0, 0.0, 0.0), "default_safe_center"


def _solve_pose_with_pink(
    *,
    urdf_path: Path,
    frame: str,
    position_m: tuple[float, float, float],
    rpy_rad: tuple[float, float, float],
    start_joints: tuple[float, ...],
) -> tuple[float, ...]:
    import numpy as np
    import pinocchio as pin
    import pink
    from pink.solve_ik import solve_ik
    from pink.tasks import FrameTask, PostureTask

    model = pin.buildModelFromUrdf(str(urdf_path))
    if model.nq < len(start_joints):
        raise RuntimeError(
            f"pink model nq={model.nq} is smaller than requested joints={len(start_joints)}"
        )
    data = model.createData()
    q0 = np.array(start_joints, dtype=float)
    configuration = pink.Configuration(model, data, q0)
    frame_task = FrameTask(frame, position_cost=1.0, orientation_cost=1.0)
    posture_task = PostureTask(cost=1e-3)
    posture_task.set_target(q0.copy())
    target = pin.SE3(pin.rpy.rpyToMatrix(*rpy_rad), np.array(position_m, dtype=float))
    frame_task.set_target(target)

    dt = 0.02
    for _ in range(200):
        velocity = solve_ik(
            configuration,
            [frame_task, posture_task],
            dt,
            solver="quadprog",
        )
        configuration.integrate_inplace(velocity, dt)

    q_result = tuple(float(value) for value in configuration.q[: len(start_joints)])
    if any(not math.isfinite(value) for value in q_result):
        raise RuntimeError("pink preview synthesis produced non-finite joint targets")
    return q_result


def _forward_pose_with_pinocchio(
    *,
    urdf_path: Path,
    frame: str,
    start_joints: tuple[float, ...],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    import numpy as np
    import pinocchio as pin

    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    q = np.zeros(model.nq)
    q[: len(start_joints)] = np.array(start_joints, dtype=float)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    frame_id = model.getFrameId(frame)
    pose = data.oMf[frame_id]
    position = tuple(float(value) for value in pose.translation)
    rpy = tuple(float(value) for value in pin.rpy.matrixToRpy(pose.rotation))
    return position, rpy


def _write_preview_trajectory(
    *,
    output_path: Path,
    start_joints: tuple[float, ...],
    target_joints: tuple[float, ...],
    sample_hz: float,
    duration_s: float,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    sample_count = max(int(round(duration_s * sample_hz)) + 1, 2)
    rows: list[dict[str, str]] = []
    for sample_index in range(sample_count):
        alpha = sample_index / (sample_count - 1)
        smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        row = {"time_s": f"{sample_index / sample_hz:.6f}"}
        for joint_index, (start_q, target_q) in enumerate(
            zip(start_joints, target_joints),
            start=1,
        ):
            value = start_q + (target_q - start_q) * smooth_alpha
            row[f"q_cmd_{joint_index}"] = f"{value:.6f}"
        rows.append(row)
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _validate_backend_trajectory(
    trajectory_path: Path,
    *,
    required_columns: list[str],
) -> dict[str, object]:
    with trajectory_path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        missing_columns = [
            column for column in required_columns if column not in fieldnames
        ]
        if missing_columns:
            raise ValueError(
                "backend trajectory is missing required columns: "
                + ", ".join(missing_columns)
            )
        sample_count = sum(1 for _ in reader)
    if sample_count <= 0:
        raise ValueError("backend trajectory must contain at least one sample row")
    return {
        "status": "pass",
        "required_columns": required_columns,
        "fieldnames": fieldnames,
        "sample_count": sample_count,
    }
