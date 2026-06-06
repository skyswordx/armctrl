from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json

from armctrl.eef import (
    EefDoctor,
    EefAgentRuntimeContractExporter,
    EefAgentRuntimeContractRequest,
    EefDeltaPoseRequest,
    EefMoveItHelperPlanExporter,
    EefMoveItHelperPlanRequest,
    EefPlanner,
    EefPoseRequest,
    EefRunnerContractExporter,
    EefRunnerContractRequest,
    EefRunnerPreviewRequest,
    EefRunnerPreviewer,
    EefSdkHelperPlanExporter,
    EefSdkHelperPlanRequest,
    EefTwistRequest,
)
from armctrl.lerobot_bridge import (
    LeRobotAgentRuntimeHelperPlanRequest,
    LeRobotAgentRuntimeHelperPlanner,
    LeRobotProcessorContractExporter,
    LeRobotProcessorContractRequest,
    LeRobotProcessorHelperPreviewRequest,
    LeRobotProcessorHelperPreviewer,
)
from armctrl.recipe_runtime import (
    DEFAULT_RECIPE_START,
    RecipeAgentPresetContractExporter,
    RecipeAgentPresetContractRequest,
    RecipePlanRequest,
    RecipePlanner,
)


@dataclass(frozen=True)
class AgentFlowPlanRequest:
    preset: str
    eef_mode: str
    backend: str
    output_dir: Path
    safe_config_path: str = "configs/x5.safe.yaml"
    urdf_path: str = "configs/models/X5_camera.urdf"
    sample_hz: float = 50.0
    duration_s: float = 2.0
    frame: str = "eef_link"
    recipe_duration_s: float = 2.0
    recipe_sample_hz: float = 50.0
    position_m: tuple[float, float, float] | None = None
    rpy_rad: tuple[float, float, float] | None = None
    delta_position_m: tuple[float, float, float] | None = None
    delta_rpy_rad: tuple[float, float, float] | None = None
    linear_mps: tuple[float, float, float] | None = None
    angular_rps: tuple[float, float, float] | None = None
    control_period_s: float = 0.1
    model: str = "X5"
    interface: str = "can0"
    policy_path: str = "outputs/train/act_arx5/checkpoints/last/pretrained_model"


class AgentFlowPlanner:
    def plan(self, request: AgentFlowPlanRequest) -> dict[str, object]:
        recipe_plan_dir = request.output_dir / "recipe-plan"
        eef_plan_dir = request.output_dir / "eef-plan"
        contract_path = request.output_dir / "agent_flow_plan.json"
        request.output_dir.mkdir(parents=True, exist_ok=True)

        recipe_plan = RecipePlanner().write_plan(
            RecipePlanRequest(
                recipe_name=request.preset,
                start_joints=DEFAULT_RECIPE_START,
                sample_hz=request.recipe_sample_hz,
                duration_s=request.recipe_duration_s,
                urdf_path=request.urdf_path,
                safe_config_path=request.safe_config_path,
                output_dir=recipe_plan_dir,
            )
        )
        preset_contract = RecipeAgentPresetContractExporter().export(
            RecipeAgentPresetContractRequest(plan_dir=recipe_plan_dir)
        )

        planner = EefPlanner()
        if request.eef_mode == "pose_absolute":
            if request.position_m is None or request.rpy_rad is None:
                raise ValueError(
                    "--position and --rpy are required for --eef-mode pose_absolute"
                )
            eef_plan = planner.plan_pose(
                EefPoseRequest(
                    frame=request.frame,
                    position_m=request.position_m,
                    rpy_rad=request.rpy_rad,
                    backend=request.backend,
                    safe_config_path=request.safe_config_path,
                    output_dir=eef_plan_dir,
                )
            )
        elif request.eef_mode == "pose_delta":
            if request.delta_position_m is None or request.delta_rpy_rad is None:
                raise ValueError(
                    "--delta-position and --delta-rpy are required for --eef-mode pose_delta"
                )
            eef_plan = planner.plan_delta_pose(
                EefDeltaPoseRequest(
                    frame=request.frame,
                    delta_position_m=request.delta_position_m,
                    delta_rpy_rad=request.delta_rpy_rad,
                    backend=request.backend,
                    safe_config_path=request.safe_config_path,
                    control_period_s=request.control_period_s,
                    output_dir=eef_plan_dir,
                )
            )
        elif request.eef_mode == "twist":
            if request.linear_mps is None or request.angular_rps is None:
                raise ValueError(
                    "--linear and --angular are required for --eef-mode twist"
                )
            eef_plan = planner.plan_twist(
                EefTwistRequest(
                    frame=request.frame,
                    linear_mps=request.linear_mps,
                    angular_rps=request.angular_rps,
                    backend=request.backend,
                    safe_config_path=request.safe_config_path,
                    control_period_s=request.control_period_s,
                    output_dir=eef_plan_dir,
                )
            )
        else:
            raise ValueError(f"unsupported eef mode: {request.eef_mode}")

        runtime = self._runtime_branch(request=request, eef_plan_dir=eef_plan_dir)
        review = runtime["review"]

        payload = {
            "schema": "armctrl.agent_flow_plan.v1",
            "movement_allowed": False,
            "agent_motion_contract": _agent_motion_contract(
                backend=request.backend,
                eef_mode=request.eef_mode,
                frame=request.frame,
            ),
            "recommended_path": _agent_flow_recommended_path(
                request=request,
                recipe_plan_dir=recipe_plan_dir,
                eef_plan_dir=eef_plan_dir,
            ),
            "preset": {
                "recipe": recipe_plan["recipe"],
                "plan": recipe_plan,
                "contract": preset_contract,
            },
            "eef": {
                "mode": request.eef_mode,
                "plan": eef_plan,
            },
            "runtime": runtime["runtime"],
            "review": review,
            "artifacts": {
                "recipe_plan_dir": str(recipe_plan_dir),
                "eef_plan_dir": str(eef_plan_dir),
                "agent_flow_contract": str(contract_path),
            },
            "ordered_steps": runtime["ordered_steps"],
            "next_steps": runtime["next_steps"],
            "notes": [
                "This is a thin Agent-facing orchestration contract over existing recipe, EEF, and shared review surfaces.",
                "armctrl still does not execute hardware, IK, servo, or native rollout runtime ownership here.",
                "Use this flow when the Agent needs one stable entry point instead of manually stitching preset, EEF, runtime helper, and review commands together.",
            ],
        }
        contract_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return payload

    def _runtime_branch(
        self,
        *,
        request: AgentFlowPlanRequest,
        eef_plan_dir: Path,
    ) -> dict[str, object]:
        if request.backend == "lerobot_rollout":
            agent_runtime_contract = EefAgentRuntimeContractExporter().export(
                EefAgentRuntimeContractRequest(
                    plan_dir=eef_plan_dir,
                    backend="lerobot_rollout",
                    model=request.model,
                    interface=request.interface,
                    output_path=eef_plan_dir / "eef_agent_runtime_contract.json",
                )
            )
            helper_plan = LeRobotAgentRuntimeHelperPlanner().plan(
                LeRobotAgentRuntimeHelperPlanRequest(
                    agent_runtime_contract_path=eef_plan_dir
                    / "eef_agent_runtime_contract.json"
                )
            )
            processor_contract = LeRobotProcessorContractExporter().export(
                LeRobotProcessorContractRequest(
                    eef_plan_dir=eef_plan_dir,
                    model=request.model,
                    robot_interface=request.interface,
                    output_path=eef_plan_dir / "lerobot_processor_contract.json",
                )
            )
            review = LeRobotProcessorHelperPreviewer().preview(
                LeRobotProcessorHelperPreviewRequest(
                    processor_contract_path=eef_plan_dir
                    / "lerobot_processor_contract.json",
                    urdf_path=Path(request.urdf_path),
                    safe_config_path=Path(request.safe_config_path),
                )
            )
            return {
                "runtime": {
                    "backend": request.backend,
                    "agent_runtime_contract": agent_runtime_contract,
                    "helper_plan": helper_plan,
                    "processor_contract": processor_contract,
                },
                "review": review,
                "ordered_steps": [
                    {
                        "id": "plan_recipe_preset",
                        "command": _agent_flow_recipe_plan_command(
                            preset=request.preset,
                            recipe_plan_dir=eef_plan_dir.parent / "recipe-plan",
                        ),
                    },
                    {
                        "id": "plan_eef_intent",
                        "depends_on": ["plan_recipe_preset"],
                        "command": _agent_flow_eef_plan_command(
                            request=request,
                            eef_plan_dir=eef_plan_dir,
                        ),
                    },
                    {
                        "id": "export_processor_contract",
                        "depends_on": ["plan_eef_intent"],
                        "command": (
                            "uv run armctrl lerobot export-processor-contract "
                            f"--eef-plan-dir {eef_plan_dir} --json"
                        ),
                    },
                    {
                        "id": "review_runtime_handoff",
                        "depends_on": ["export_processor_contract"],
                        "command": (
                            "uv run armctrl lerobot processor-helper-preview "
                            f"--processor-contract {eef_plan_dir / 'lerobot_processor_contract.json'} "
                            f"--urdf-path {request.urdf_path} --safe-config {request.safe_config_path} --json"
                        ),
                    },
                ],
                "next_steps": review["next_steps"],
            }

        runner_contract = EefRunnerContractExporter().export(
            EefRunnerContractRequest(
                plan_dir=eef_plan_dir,
                backend=request.backend,
                model=request.model,
                interface=request.interface,
                output_path=eef_plan_dir / "eef_runner_contract.json",
            )
        )
        if request.backend == "sdk_cartesian":
            helper_plan = EefSdkHelperPlanExporter().export(
                EefSdkHelperPlanRequest(
                    runner_contract_path=eef_plan_dir / "eef_runner_contract.json"
                )
            )
        elif request.backend == "moveit_servo":
            helper_plan = EefMoveItHelperPlanExporter().export(
                EefMoveItHelperPlanRequest(
                    runner_contract_path=eef_plan_dir / "eef_runner_contract.json"
                )
            )
        else:
            raise ValueError(f"unsupported runtime backend: {request.backend}")
        review = EefRunnerPreviewer().preview(
            EefRunnerPreviewRequest(
                plan_dir=eef_plan_dir,
                urdf_path=Path(request.urdf_path),
                safe_config_path=Path(request.safe_config_path),
                backend=request.backend,
                model=request.model,
                interface=request.interface,
                recipe_plan_dir=eef_plan_dir.parent / "recipe-plan",
            )
        )
        return {
            "runtime": {
                "backend": request.backend,
                "runner_contract": runner_contract,
                "helper_plan": helper_plan,
            },
            "review": review,
            "ordered_steps": [
                {
                    "id": "plan_recipe_preset",
                    "command": _agent_flow_recipe_plan_command(
                        preset=request.preset,
                        recipe_plan_dir=eef_plan_dir.parent / "recipe-plan",
                    ),
                },
                {
                    "id": "plan_eef_intent",
                    "depends_on": ["plan_recipe_preset"],
                    "command": _agent_flow_eef_plan_command(
                        request=request,
                        eef_plan_dir=eef_plan_dir,
                    ),
                },
                {
                    "id": "export_runner_contract",
                    "depends_on": ["plan_eef_intent"],
                    "command": (
                        "uv run armctrl eef export-runner-contract "
                        f"--plan-dir {eef_plan_dir} --backend {request.backend} --json"
                    ),
                },
                {
                    "id": "review_runtime_handoff",
                    "depends_on": ["export_runner_contract"],
                    "command": (
                        "uv run armctrl eef preview-runner "
                        f"--plan-dir {eef_plan_dir} --json"
                    ),
                },
            ],
            "next_steps": review["next_steps"],
        }


class AgentFlowDoctor:
    def run(self) -> dict[str, object]:
        eef = EefDoctor().run()
        moveit_status = str(eef["backends"]["moveit_servo"]["status"])
        sdk_status = str(eef["backends"]["sdk_cartesian"]["status"])
        return {
            "schema": "armctrl.agent_flow_doctor.v1",
            "read_only": True,
            "movement_allowed": False,
            "surfaces": {
                "recipe": {
                    "status": "available",
                    "entrypoint": "armctrl recipe",
                },
                "eef": {
                    "status": "available",
                    "entrypoint": "armctrl eef",
                },
                "lerobot": {
                    "status": "available",
                    "entrypoint": "armctrl lerobot",
                },
                "runtime_backends": {
                    "sdk_cartesian": sdk_status,
                    "moveit_servo": moveit_status,
                    "lerobot_rollout": "contract_available",
                },
            },
            "next_gate": "run_agent_flow_plan_before_any_future_execution_discussion",
            "notes": [
                "agent-flow is a thin orchestration surface over recipe, eef, lerobot, and the shared simulation review chain.",
                "It does not own hardware execution, IK, servo math, or native rollout runtime execution.",
            ],
        }


class AgentFlowReviewer:
    def review(self, contract_path: Path) -> dict[str, object]:
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
        if contract.get("schema") != "armctrl.agent_flow_plan.v1":
            raise ValueError(
                "agent flow contract file must use schema armctrl.agent_flow_plan.v1"
            )
        review = contract["review"]
        return {
            "schema": "armctrl.agent_flow_review.v1",
            "movement_allowed": False,
            "contract_path": str(contract_path),
            "backend": contract["runtime"]["backend"],
            "recommended_path": contract.get("recommended_path"),
            "review": review,
            "artifacts": contract["artifacts"],
            "ordered_steps": contract["ordered_steps"],
            "next_steps": contract["next_steps"],
            "notes": [
                "This review surface replays the saved top-level agent-flow contract instead of creating a second review path.",
                "Use the saved contract as the durable handoff artifact between Agent planning and later execution discussions.",
            ],
        }


def _agent_flow_recipe_plan_command(*, preset: str, recipe_plan_dir: Path) -> str:
    return f"uv run armctrl recipe plan {preset} --output {recipe_plan_dir} --json"


def _agent_motion_contract(
    *, backend: str, eef_mode: str, frame: str
) -> dict[str, object]:
    recommended_action_id = {
        "pose_absolute": "eef.pose_absolute",
        "pose_delta": "eef.pose_delta",
        "twist": "eef.twist",
    }[eef_mode]
    payload = {
        "schema": "armctrl.agent_motion_contract.v1",
        "frame": frame,
        "supported_action_ids": [
            "eef.pose_absolute",
            "eef.pose_delta",
            "eef.twist",
        ],
        "selected_action_id": recommended_action_id,
        "backend": backend,
        "notes": [
            "Agent-facing motion should stay in the EEF vocabulary and not switch to backend-native command objects.",
            "armctrl remains a thin orchestration layer; mature runtime owners still consume backend-specific bridge artifacts.",
        ],
    }
    if backend == "lerobot_rollout":
        payload["lerobot_compatibility"] = {
            "status": "compatible",
            "preferred_training_action_id": "eef.pose_delta",
            "supported_action_ids": [
                "eef.pose_absolute",
                "eef.pose_delta",
                "eef.twist",
            ],
            "processor_owner": {
                "action": "robot_action_processor",
                "observation": "robot_observation_processor",
            },
            "adaptation_rule": (
                "Keep Agent commands in EEF space, then let LeRobot processors adapt them into rollout-native actions."
            ),
        }
    return payload


def _agent_flow_recommended_path(
    *,
    request: AgentFlowPlanRequest,
    recipe_plan_dir: Path,
    eef_plan_dir: Path,
) -> dict[str, object]:
    path = {
        "schema": "armctrl.recommended_path.v1",
        "profile": "preset_then_bounded_eef_then_review",
        "primary_entrypoint": "armctrl agent-flow plan",
        "why": (
            "Use one top-level non-hardware contract instead of stitching recipe, EEF, backend helper, and review surfaces manually."
        ),
        "steps": [
            {
                "id": "plan_recipe_preset",
                "command": _agent_flow_recipe_plan_command(
                    preset=request.preset,
                    recipe_plan_dir=recipe_plan_dir,
                ),
            },
            {
                "id": "plan_eef_intent",
                "command": _agent_flow_eef_plan_command(
                    request=request,
                    eef_plan_dir=eef_plan_dir,
                ),
            },
        ],
    }
    if request.backend == "lerobot_rollout":
        path["steps"].extend(
            [
                {
                    "id": "export_agent_session_plan",
                    "command": (
                        "uv run armctrl eef export-agent-session-plan "
                        f"--plan-dir {eef_plan_dir} --json"
                    ),
                },
                {
                    "id": "review_runtime_handoff",
                    "command": (
                        "uv run armctrl lerobot processor-helper-preview "
                        f"--processor-contract {eef_plan_dir / 'lerobot_processor_contract.json'} "
                        f"--urdf-path {request.urdf_path} --safe-config {request.safe_config_path} --json"
                    ),
                },
            ]
        )
    else:
        path["steps"].extend(
            [
                {
                    "id": "export_agent_session_plan",
                    "command": (
                        "uv run armctrl eef export-agent-session-plan "
                        f"--plan-dir {eef_plan_dir} --backend {request.backend} --json"
                    ),
                },
                {
                    "id": "review_runtime_handoff",
                    "command": (
                        "uv run armctrl eef preview-runner "
                        f"--plan-dir {eef_plan_dir} --json"
                    ),
                },
            ]
        )
    return path


def _agent_flow_eef_plan_command(
    *,
    request: AgentFlowPlanRequest,
    eef_plan_dir: Path,
) -> str:
    base = ["uv run armctrl eef"]
    if request.eef_mode == "pose_absolute":
        assert request.position_m is not None
        assert request.rpy_rad is not None
        return (
            f"{base[0]} plan-pose --frame {request.frame} "
            f"--position {request.position_m[0]} {request.position_m[1]} {request.position_m[2]} "
            f"--rpy {request.rpy_rad[0]} {request.rpy_rad[1]} {request.rpy_rad[2]} "
            f"--backend {request.backend} --output {eef_plan_dir} --json"
        )
    if request.eef_mode == "pose_delta":
        assert request.delta_position_m is not None
        assert request.delta_rpy_rad is not None
        return (
            f"{base[0]} plan-delta-pose --frame {request.frame} "
            f"--delta-position {request.delta_position_m[0]} {request.delta_position_m[1]} {request.delta_position_m[2]} "
            f"--delta-rpy {request.delta_rpy_rad[0]} {request.delta_rpy_rad[1]} {request.delta_rpy_rad[2]} "
            f"--backend {request.backend} --output {eef_plan_dir} --json"
        )
    assert request.linear_mps is not None
    assert request.angular_rps is not None
    return (
        f"{base[0]} plan-twist --frame {request.frame} "
        f"--linear {request.linear_mps[0]} {request.linear_mps[1]} {request.linear_mps[2]} "
        f"--angular {request.angular_rps[0]} {request.angular_rps[1]} {request.angular_rps[2]} "
        f"--backend {request.backend} --output {eef_plan_dir} --json"
    )
