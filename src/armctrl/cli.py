from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
from typing import Callable, Mapping, Sequence
from uuid import uuid4

from armctrl.acceptance import build_real_motion_acceptance
from armctrl.agent_flow import (
    AGENT_FLOW_REAL_RUNTIME_CONFIRMATION,
    AgentFlowDoctor,
    AgentFlowPlanRequest,
    AgentFlowPlanner,
    AgentFlowRealRuntimeSmokeRequest,
    AgentFlowRealRuntimeSmoker,
    AgentFlowReviewer,
    AgentFlowRuntimeSmokeRequest,
    AgentFlowRuntimeSmoker,
)
from armctrl.arx5_sdk_cartesian_runtime import Arx5SdkCartesianRuntimeBackend
from armctrl.recipes import RecipeCatalog
from armctrl.recipe_executor import RecipeExecutor
from armctrl.recipe_runtime import (
    DEFAULT_RECIPE_START,
    RecipeAgentPresetContractExporter,
    RecipeAgentPresetContractRequest,
    RecipeEefSeedExporter,
    RecipeEefSeedRequest,
    RecipePlanRequest,
    RecipePlanner,
    RecipeRuntimeSmokeRequest,
    RecipeRuntimeSmoker,
)
from armctrl.runtime_session import (
    ARX5_RUNTIME_START_CONFIRMATION,
    RuntimeSessionError,
    acquire_owner_from_artifact,
    arx5_runtime_start_preflight,
    eef_adapter_manager_payload,
    heartbeat_runtime_session_payload,
    owner_heartbeat_from_artifact,
    recover_runtime_session_from_artifact,
    record_runtime_hold_tick,
    refresh_runtime_status_payload,
    release_owner_from_artifact,
    runtime_readiness,
    runtime_controller_manager_payload,
    runtime_status_from_artifact,
    start_arx5_cartesian_runtime_session,
    start_arx5_runtime_session,
    stop_runtime_session_from_artifact,
    start_fake_runtime_session,
    watchdog_tick_from_artifact,
)
from armctrl.runtime_ipc import (
    DEFAULT_AGENT_MAX_JOINT_VELOCITY_RAD_S,
    DEFAULT_EEF_MAX_ANGULAR_STEP_RAD,
    DEFAULT_EEF_MAX_LINEAR_STEP_M,
    execute_pending_runtime_commands,
    submit_eef_command,
    submit_intent_command,
    submit_trajectory_command,
)
from armctrl.release_status import release_notes, release_status
from armctrl.lerobot_bridge import (
    LeRobotAgentRuntimeHelperPlanRequest,
    LeRobotAgentRuntimeHelperPlanner,
    LeRobotConfigPlanner,
    LeRobotConfigPlanRequest,
    LeRobotDoctor,
    LeRobotMetadataExport,
    LeRobotMetadataExporter,
    LeRobotProcessorContractExporter,
    LeRobotProcessorContractRequest,
    LeRobotProcessorHelperPreviewRequest,
    LeRobotProcessorHelperPreviewer,
    LeRobotRolloutPreviewRequest,
    LeRobotRolloutPreviewer,
    LeRobotRolloutReviewRequest,
    LeRobotRolloutReviewer,
    LeRobotRolloutStageRequest,
    LeRobotRolloutStager,
)
from armctrl.motion_runtime import ArmRuntime, FakeMotionBackend, MotionBackend, MotionMode
from armctrl.moveit_servo_runtime import MoveItServoRuntimeBackend
from armctrl.online_id import (
    OnlineAuditRequest,
    OnlineIdentificationAuditor,
    OnlineIdentificationPolicy,
    ParameterUpdate,
)
from armctrl.eef import (
    EefAgentSessionPlanExporter,
    EefAgentSessionPlanRequest,
    EefAgentRuntimeContractExporter,
    EefAgentRuntimeContractRequest,
    EefDeltaPoseRequest,
    EefDoctor,
    EefSdkHelperPlanExporter,
    EefSdkHelperPlanRequest,
    EefLeRobotExportRequest,
    EefLeRobotExporter,
    EefMoveItHelperPlanExporter,
    EefMoveItHelperPlanRequest,
    EefMoveItServoExportRequest,
    EefMoveItServoExporter,
    EefPlanner,
    EefPoseRequest,
    EefPreviewSynthesisRequest,
    EefPreviewSynthesizer,
    EefReviewRequest,
    EefReviewer,
    EefRunnerPreviewRequest,
    EefRunnerPreviewer,
    EefSampleRunnerRequest,
    EefSampleRunner,
    EefRuntimePlanRequest,
    EefRuntimeBridgeExportRequest,
    EefRuntimeBridgeExporter,
    EefRunnerContractRequest,
    EefRunnerContractExporter,
    EefRuntimePlanner,
    EefSdkCartesianExportRequest,
    EefSdkCartesianExporter,
    EefStageTrajectoryRequest,
    EefTrajectoryStager,
    EefTwistRequest,
)
from armctrl.safety import SafetyGate
from armctrl.simulation import SimulationDoctor, TrajectoryPreviewer
from armctrl.sysid import SysIdPlanner, SysIdPlanRequest
from armctrl.sysid_trajectory_backend import TrajectoryCommandError
from armctrl.sysid_evidence import SysIdEvidenceImporter
from armctrl.sysid_figaroh_adapter import FigarohEvidenceAdapter, FigarohHandoffWriter
from armctrl.sysid_measured import MeasuredSysIdAnalyzeRequest, MeasuredSysIdAnalyzer
from armctrl.sysid_package import SysIdPackager
from armctrl.sysid_postprocess import SysIdPostprocessor, SysIdPostprocessResult
from armctrl.arx5_sdk_joint_runtime import (
    Arx5SdkJointRuntimeBackend,
    ARX5_JOINT_RUNTIME_CONFIRMATION,
)
from armctrl.sysid_run import (
    FakeSysIdRunner,
)
from armctrl.sysid_review import SysIdOfflineReviewRequest, SysIdOfflineReviewer
from armctrl.sysid_sdk import (
    DEFAULT_Q_CURRENT_MAX_ERROR_RAD,
    SdkMeasuredStateMismatchError,
    SdkAgentSysIdSmokeReadinessChecker,
    SdkArmSession,
    SdkDoctor,
    SdkHandshakePlanner,
    SdkHoldDampingCheck,
    SdkJogReal,
    SdkPreflight,
    SdkStartupRecovery,
    SdkTinyMotionExecutor,
    SdkTinyMotionPlanner,
    tiny_motion_execute_prerequisite_statuses,
)
from armctrl.sysid_solve import SysIdSolver
from armctrl.workspace import WorkspaceSafetyConfig


MOTION_COMMAND_SURFACE = "armctrl.motion.submit.v1"
REPO_ROOT = Path(__file__).resolve().parents[2]

MOTION_PROFILES: dict[str, dict[str, object]] = {
    "lab-sysid": {
        "profile": "lab-sysid",
        "schema": "armctrl.motion_profile.v1",
        "description": "Lab SysID golden path: reviewed joint trajectory through the live runtime.",
        "source": "sysid",
        "owner": "sysid",
        "runtime_backend": "arx5_sdk",
        "start_pose_policy": "safe_center",
        "supported_motion_kinds": ["joint-trajectory"],
        "readiness": "live runtime status with safe_center hold",
        "removed_entrypoints": ["armctrl sysid run ... --adapter sdk"],
        "preferred_entrypoints": [
            "armctrl sysid compile-runtime",
            "armctrl motion submit joint-trajectory --compiled-command",
        ],
    },
    "lab-agent-eef": {
        "profile": "lab-agent-eef",
        "schema": "armctrl.motion_profile.v1",
        "description": (
            "Lab Agent EEF contract: EEF delta/twist/pose commands must run through "
            "the same long-lived runtime owner; disconnected sdk_cartesian takeover "
            "is diagnostic-only."
        ),
        "source": "agent",
        "owner": "agent",
        "runtime_backend": "arx5_sdk",
        "mature_servo_backends": ["moveit_servo", "sdk_cartesian"],
        "start_pose_policy": "controlled_eef_ready_hold",
        "supported_motion_kinds": ["eef-delta", "eef-twist", "eef-pose"],
        "readiness": (
            "live runtime status with controlled hold at SAFE_CENTER or "
            "AGENT_EEF_HOME; backend switch must be bumpless inside the runtime"
        ),
        "legacy_entrypoints": ["armctrl runtime submit-eef", "scripts/lab_agent_runtime_smoke.sh start-eef"],
        "diagnostic_only_entrypoints": ["scripts/lab_agent_runtime_smoke.sh start-eef"],
    },
    "lab-agent-joint": {
        "profile": "lab-agent-joint",
        "schema": "armctrl.motion_profile.v1",
        "description": "Lab Agent joint-space MVP: bounded low-frequency joint intent or checked waypoints.",
        "source": "agent",
        "owner": "agent",
        "runtime_backend": "arx5_sdk",
        "start_pose_policy": "live_hold",
        "supported_motion_kinds": ["joint-intent", "joint-trajectory"],
        "readiness": "live runtime status with hold_safe",
        "legacy_entrypoints": ["armctrl runtime submit-intent", "armctrl runtime submit-trajectory"],
    },
    "recipe": {
        "profile": "recipe",
        "schema": "armctrl.motion_profile.v1",
        "description": "Pre-reviewed recipe motions compiled to runtime trajectory or intent commands.",
        "source": "recipe",
        "owner": "recipe",
        "runtime_backend": "arx5_sdk",
        "start_pose_policy": "live_hold",
        "supported_motion_kinds": ["joint-trajectory", "joint-intent"],
        "readiness": "live runtime status with hold_safe",
        "legacy_entrypoints": ["armctrl recipe runtime-submit"],
    },
    "teleop": {
        "profile": "teleop",
        "schema": "armctrl.motion_profile.v1",
        "description": "Reserved manual/teleop profile; formalized now, hardware backend later.",
        "source": "teleop",
        "owner": "teleop",
        "runtime_backend": "not_configured",
        "start_pose_policy": "live_hold",
        "supported_motion_kinds": ["joint-jog", "eef-twist"],
        "readiness": "requires live runtime, deadman, and mature teleop adapter",
        "status": "reserved",
    },
}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="armctrl")
    subparsers = parser.add_subparsers(dest="command", required=True)

    runtime_parser = subparsers.add_parser("runtime")
    runtime_subparsers = runtime_parser.add_subparsers(
        dest="runtime_command",
        required=True,
    )

    runtime_start_parser = runtime_subparsers.add_parser("start")
    runtime_start_parser.add_argument(
        "--backend",
        choices=["fake", "arx5_sdk", "sdk_cartesian"],
        default="fake",
    )
    runtime_start_parser.add_argument("--model", default="X5")
    runtime_start_parser.add_argument("--interface", default="can0")
    runtime_start_parser.add_argument(
        "--q-current",
        nargs="+",
        type=float,
    )
    runtime_start_parser.add_argument(
        "--safe-center",
        nargs="+",
        type=float,
        required=True,
    )
    runtime_start_parser.add_argument("--send-hz", type=float, default=50.0)
    runtime_start_parser.add_argument("--hold-hz", type=float, default=50.0)
    runtime_start_parser.add_argument(
        "--max-joint-step-rad",
        type=float,
        default=0.01,
    )
    runtime_start_parser.add_argument(
        "--max-start-error-rad",
        type=float,
        default=0.02,
    )
    runtime_start_parser.add_argument(
        "--recover-to-safe-first",
        action="store_true",
        help=(
            "For sdk_cartesian only: first use the arx5_sdk joint controller to "
            "recover to SAFE_CENTER before opening the Cartesian runtime."
        ),
    )
    runtime_start_parser.add_argument(
        "--allow-diagnostic-sdk-cartesian-takeover",
        action="store_true",
        help=(
            "Allow the old disconnected sdk_cartesian takeover path for lab "
            "diagnostics. Formal Agent EEF must stay inside a long-lived runtime."
        ),
    )
    runtime_start_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    runtime_start_parser.add_argument("--serve", action="store_true")
    runtime_start_parser.add_argument(
        "--eef-adapter",
        action="append",
        choices=["moveit_servo"],
        default=[],
        help=(
            "Register an in-runtime EEF adapter for runtime-owned EEF commands. "
            "Currently implemented for fake/offline serve validation only; real "
            "hardware EEF requires a mature configured servo backend."
        ),
    )
    runtime_start_parser.add_argument(
        "--heartbeat-period-s",
        type=float,
        default=0.05,
    )
    runtime_start_parser.add_argument("--confirm")
    runtime_start_parser.add_argument("--output", required=True)
    runtime_start_parser.add_argument("--json", action="store_true", dest="as_json")

    runtime_status_parser = runtime_subparsers.add_parser("status")
    runtime_status_parser.add_argument("--session-artifact", required=True)
    runtime_status_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    runtime_status_parser.add_argument("--output")
    runtime_status_parser.add_argument("--json", action="store_true", dest="as_json")

    runtime_recover_parser = runtime_subparsers.add_parser("recover")
    runtime_recover_parser.add_argument("--session-artifact", required=True)
    runtime_recover_parser.add_argument(
        "--safe-center",
        nargs="+",
        type=float,
        required=True,
    )
    runtime_recover_parser.add_argument("--send-hz", type=float, default=50.0)
    runtime_recover_parser.add_argument("--hold-hz", type=float, default=50.0)
    runtime_recover_parser.add_argument(
        "--max-joint-step-rad",
        type=float,
        default=0.01,
    )
    runtime_recover_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    runtime_recover_parser.add_argument("--output")
    runtime_recover_parser.add_argument("--json", action="store_true", dest="as_json")

    runtime_stop_parser = runtime_subparsers.add_parser("stop")
    runtime_stop_parser.add_argument("--session-artifact", required=True)
    runtime_stop_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    runtime_stop_parser.add_argument("--output")
    runtime_stop_parser.add_argument("--json", action="store_true", dest="as_json")

    runtime_acquire_parser = runtime_subparsers.add_parser("acquire-owner")
    runtime_acquire_parser.add_argument("--session-artifact", required=True)
    runtime_acquire_parser.add_argument("--owner", required=True)
    runtime_acquire_parser.add_argument(
        "--mode",
        choices=["agent_servo", "trajectory_replay"],
        required=True,
    )
    runtime_acquire_parser.add_argument(
        "--expected-q-start",
        nargs="+",
        type=float,
        required=True,
    )
    runtime_acquire_parser.add_argument(
        "--max-start-error-rad",
        type=float,
        default=0.02,
    )
    runtime_acquire_parser.add_argument(
        "--heartbeat-timeout-s",
        type=float,
        default=0.5,
    )
    runtime_acquire_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    runtime_acquire_parser.add_argument("--output")
    runtime_acquire_parser.add_argument("--json", action="store_true", dest="as_json")

    runtime_release_parser = runtime_subparsers.add_parser("release-owner")
    runtime_release_parser.add_argument("--session-artifact", required=True)
    runtime_release_parser.add_argument("--owner", required=True)
    runtime_release_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    runtime_release_parser.add_argument("--output")
    runtime_release_parser.add_argument("--json", action="store_true", dest="as_json")

    runtime_owner_heartbeat_parser = runtime_subparsers.add_parser("owner-heartbeat")
    runtime_owner_heartbeat_parser.add_argument("--session-artifact", required=True)
    runtime_owner_heartbeat_parser.add_argument("--owner", required=True)
    runtime_owner_heartbeat_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    runtime_owner_heartbeat_parser.add_argument("--output")
    runtime_owner_heartbeat_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    runtime_watchdog_parser = runtime_subparsers.add_parser("watchdog-tick")
    runtime_watchdog_parser.add_argument("--session-artifact", required=True)
    runtime_watchdog_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    runtime_watchdog_parser.add_argument("--output")
    runtime_watchdog_parser.add_argument("--json", action="store_true", dest="as_json")

    runtime_result_check_parser = runtime_subparsers.add_parser("result-check")
    runtime_result_source = runtime_result_check_parser.add_mutually_exclusive_group(
        required=True
    )
    runtime_result_source.add_argument("--result-artifact")
    runtime_result_source.add_argument("--run-dir")
    runtime_result_check_parser.add_argument("--all", action="store_true")
    runtime_result_check_parser.add_argument(
        "--require-owner",
        action="append",
        default=[],
    )
    runtime_result_check_parser.add_argument("--expect-owner")
    runtime_result_check_parser.add_argument("--expect-mode")
    runtime_result_check_parser.add_argument("--expect-sample-count", type=int)
    runtime_result_check_parser.add_argument("--max-jitter-p99-ms", type=float)
    runtime_result_check_parser.add_argument("--max-tracking-error-rad", type=float)
    runtime_result_check_parser.add_argument("--output")
    runtime_result_check_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    runtime_preposition_parser = runtime_subparsers.add_parser("preposition")
    runtime_preposition_parser.add_argument("--session-artifact", required=True)
    runtime_preposition_parser.add_argument(
        "--q-target",
        nargs="+",
        type=float,
        required=True,
    )
    runtime_preposition_parser.add_argument("--send-hz", type=float, default=50.0)
    runtime_preposition_parser.add_argument(
        "--max-joint-step-rad",
        type=float,
        default=0.01,
    )
    runtime_preposition_parser.add_argument(
        "--max-start-error-rad",
        type=float,
        default=0.02,
    )
    runtime_preposition_parser.add_argument(
        "--heartbeat-timeout-s",
        type=float,
    )
    runtime_preposition_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    runtime_preposition_parser.add_argument("--output")
    runtime_preposition_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    runtime_submit_trajectory_parser = runtime_subparsers.add_parser(
        "submit-trajectory"
    )
    runtime_submit_trajectory_parser.add_argument("--session-artifact", required=True)
    runtime_submit_trajectory_parser.add_argument("--owner", required=True)
    runtime_submit_trajectory_parser.add_argument(
        "--expected-q-start",
        nargs="+",
        type=float,
        required=True,
    )
    runtime_submit_trajectory_parser.add_argument(
        "--q-point",
        nargs="+",
        type=float,
        action="append",
        required=True,
    )
    runtime_submit_trajectory_parser.add_argument("--send-hz", type=float, default=50.0)
    runtime_submit_trajectory_parser.add_argument(
        "--max-joint-segment-delta-rad",
        type=float,
    )
    runtime_submit_trajectory_parser.add_argument(
        "--max-joint-velocity-rad-s",
        type=float,
    )
    runtime_submit_trajectory_parser.add_argument(
        "--max-joint-acceleration-rad-s2",
        type=float,
    )
    runtime_submit_trajectory_parser.add_argument(
        "--max-tracking-error-rad",
        type=float,
    )
    runtime_submit_trajectory_parser.add_argument(
        "--max-tau-abs",
        type=float,
    )
    runtime_submit_trajectory_parser.add_argument(
        "--max-start-error-rad",
        type=float,
        default=0.02,
    )
    runtime_submit_trajectory_parser.add_argument(
        "--heartbeat-timeout-s",
        type=float,
        default=0.5,
    )
    runtime_submit_trajectory_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    runtime_submit_trajectory_parser.add_argument("--output")
    runtime_submit_trajectory_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    runtime_submit_intent_parser = runtime_subparsers.add_parser("submit-intent")
    runtime_submit_intent_parser.add_argument("--session-artifact", required=True)
    runtime_submit_intent_parser.add_argument("--owner", default="agent")
    runtime_submit_intent_parser.add_argument(
        "--expected-q-start",
        nargs="+",
        type=float,
        required=True,
    )
    runtime_submit_intent_parser.add_argument(
        "--q-target",
        nargs="+",
        type=float,
        required=True,
    )
    runtime_submit_intent_parser.add_argument(
        "--control-period-s",
        type=float,
        default=0.1,
    )
    runtime_submit_intent_parser.add_argument("--send-hz", type=float, default=50.0)
    runtime_submit_intent_parser.add_argument(
        "--max-tracking-error-rad",
        type=float,
    )
    runtime_submit_intent_parser.add_argument(
        "--max-tau-abs",
        type=float,
    )
    runtime_submit_intent_parser.add_argument(
        "--max-joint-delta-rad",
        type=float,
        default=0.005,
    )
    runtime_submit_intent_parser.add_argument(
        "--max-joint-velocity-rad-s",
        type=float,
        default=0.25,
    )
    runtime_submit_intent_parser.add_argument(
        "--max-joint-acceleration-rad-s2",
        type=float,
    )
    runtime_submit_intent_parser.add_argument(
        "--max-start-error-rad",
        type=float,
        default=0.02,
    )
    runtime_submit_intent_parser.add_argument(
        "--heartbeat-timeout-s",
        type=float,
        default=0.5,
    )
    runtime_submit_intent_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    runtime_submit_intent_parser.add_argument("--output")
    runtime_submit_intent_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    runtime_submit_eef_parser = runtime_subparsers.add_parser("submit-eef")
    runtime_submit_eef_parser.add_argument("--session-artifact", required=True)
    runtime_submit_eef_parser.add_argument("--owner", default="agent")
    runtime_submit_eef_parser.add_argument(
        "--backend",
        required=True,
        choices=["moveit_servo", "sdk_cartesian"],
    )
    runtime_submit_eef_parser.add_argument(
        "--kind",
        required=True,
        choices=["eef_pose_delta", "eef_twist"],
    )
    runtime_submit_eef_parser.add_argument("--frame", default="eef_link")
    runtime_submit_eef_parser.add_argument(
        "--expected-q-start",
        nargs="+",
        type=float,
        required=True,
    )
    runtime_submit_eef_parser.add_argument("--delta-position", nargs=3, type=float)
    runtime_submit_eef_parser.add_argument("--delta-rpy", nargs=3, type=float)
    runtime_submit_eef_parser.add_argument("--linear", nargs=3, type=float)
    runtime_submit_eef_parser.add_argument("--angular", nargs=3, type=float)
    runtime_submit_eef_parser.add_argument(
        "--control-period-s",
        type=float,
        default=0.1,
    )
    runtime_submit_eef_parser.add_argument("--send-hz", type=float, default=50.0)
    runtime_submit_eef_parser.add_argument(
        "--max-linear-step-m", type=float, default=DEFAULT_EEF_MAX_LINEAR_STEP_M
    )
    runtime_submit_eef_parser.add_argument(
        "--max-angular-step-rad", type=float, default=DEFAULT_EEF_MAX_ANGULAR_STEP_RAD
    )
    runtime_submit_eef_parser.add_argument(
        "--max-start-error-rad",
        type=float,
        default=0.02,
    )
    runtime_submit_eef_parser.add_argument(
        "--start-pose-policy",
        choices=["live_hold", "safe_center", "explicit_q", "current_measured_pose"],
        default="live_hold",
    )
    runtime_submit_eef_parser.add_argument(
        "--heartbeat-timeout-s",
        type=float,
        default=0.5,
    )
    runtime_submit_eef_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    runtime_submit_eef_parser.add_argument("--output")
    runtime_submit_eef_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    motion_parser = subparsers.add_parser("motion")
    motion_subparsers = motion_parser.add_subparsers(
        dest="motion_command",
        required=True,
    )
    motion_compile_parser = motion_subparsers.add_parser("compile")
    motion_compile_subparsers = motion_compile_parser.add_subparsers(
        dest="motion_kind",
        required=True,
    )
    motion_compile_joint_trajectory_parser = motion_compile_subparsers.add_parser(
        "joint-trajectory"
    )
    motion_compile_joint_trajectory_parser.add_argument("--source", default="agent")
    motion_compile_joint_trajectory_parser.add_argument("--owner", default="agent")
    motion_compile_joint_trajectory_parser.add_argument(
        "--expected-q-start",
        nargs="+",
        type=float,
        required=True,
    )
    motion_compile_joint_trajectory_parser.add_argument(
        "--q-target",
        nargs="+",
        type=float,
    )
    motion_compile_joint_trajectory_parser.add_argument(
        "--q-point",
        nargs="+",
        type=float,
        action="append",
    )
    motion_compile_joint_trajectory_parser.add_argument("--duration-s", type=float)
    motion_compile_joint_trajectory_parser.add_argument(
        "--sample-hz",
        type=float,
        required=True,
    )
    motion_compile_joint_trajectory_parser.add_argument("--send-hz", type=float)
    motion_compile_joint_trajectory_parser.add_argument(
        "--max-joint-segment-delta-rad",
        type=float,
    )
    motion_compile_joint_trajectory_parser.add_argument(
        "--max-joint-velocity-rad-s",
        type=float,
    )
    motion_compile_joint_trajectory_parser.add_argument(
        "--max-joint-acceleration-rad-s2",
        type=float,
    )
    motion_compile_joint_trajectory_parser.add_argument(
        "--start-pose-policy",
        choices=["live_hold", "safe_center", "explicit_q", "current_measured_pose"],
        default="live_hold",
    )
    motion_compile_joint_trajectory_parser.add_argument(
        "--max-tracking-error-rad",
        type=float,
    )
    motion_compile_joint_trajectory_parser.add_argument(
        "--tracking-error-grace-samples",
        type=int,
        default=3,
    )
    motion_compile_joint_trajectory_parser.add_argument(
        "--tracking-error-consecutive-samples",
        type=int,
        default=3,
    )
    motion_compile_joint_trajectory_parser.add_argument("--max-tau-abs", type=float)
    motion_compile_joint_trajectory_parser.add_argument("--output", required=True)
    motion_compile_joint_trajectory_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    motion_submit_parser = motion_subparsers.add_parser("submit")
    motion_submit_subparsers = motion_submit_parser.add_subparsers(
        dest="motion_kind",
        required=True,
    )

    motion_joint_trajectory_parser = motion_submit_subparsers.add_parser(
        "joint-trajectory"
    )
    motion_joint_trajectory_parser.add_argument("--session-artifact", required=True)
    motion_joint_trajectory_parser.add_argument("--owner", default="agent")
    motion_joint_trajectory_parser.add_argument(
        "--expected-q-start",
        nargs="+",
        type=float,
    )
    motion_joint_trajectory_parser.add_argument(
        "--q-point",
        nargs="+",
        type=float,
        action="append",
    )
    motion_joint_trajectory_parser.add_argument("--compiled-command")
    motion_joint_trajectory_parser.add_argument("--send-hz", type=float, default=50.0)
    motion_joint_trajectory_parser.add_argument(
        "--trajectory-sample-hz",
        type=float,
    )
    motion_joint_trajectory_parser.add_argument(
        "--max-joint-segment-delta-rad",
        type=float,
    )
    motion_joint_trajectory_parser.add_argument(
        "--max-joint-velocity-rad-s",
        type=float,
    )
    motion_joint_trajectory_parser.add_argument(
        "--max-joint-acceleration-rad-s2",
        type=float,
    )
    motion_joint_trajectory_parser.add_argument(
        "--start-pose-policy",
        choices=["live_hold", "safe_center", "explicit_q", "current_measured_pose"],
        default="live_hold",
    )
    motion_joint_trajectory_parser.add_argument(
        "--max-tracking-error-rad",
        type=float,
    )
    motion_joint_trajectory_parser.add_argument(
        "--tracking-error-grace-samples",
        type=int,
        default=3,
    )
    motion_joint_trajectory_parser.add_argument(
        "--tracking-error-consecutive-samples",
        type=int,
        default=3,
    )
    motion_joint_trajectory_parser.add_argument("--max-tau-abs", type=float)
    motion_joint_trajectory_parser.add_argument(
        "--max-start-error-rad",
        type=float,
        default=0.02,
    )
    motion_joint_trajectory_parser.add_argument(
        "--heartbeat-timeout-s",
        type=float,
        default=0.5,
    )
    motion_joint_trajectory_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    motion_joint_trajectory_parser.add_argument("--output")
    motion_joint_trajectory_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    motion_joint_intent_parser = motion_submit_subparsers.add_parser("joint-intent")
    motion_joint_intent_parser.add_argument("--session-artifact", required=True)
    motion_joint_intent_parser.add_argument("--owner", default="agent")
    motion_joint_intent_parser.add_argument(
        "--expected-q-start",
        nargs="+",
        type=float,
        required=True,
    )
    motion_joint_intent_parser.add_argument(
        "--q-target",
        nargs="+",
        type=float,
        required=True,
    )
    motion_joint_intent_parser.add_argument(
        "--control-period-s",
        type=float,
        default=0.1,
    )
    motion_joint_intent_parser.add_argument("--send-hz", type=float, default=50.0)
    motion_joint_intent_parser.add_argument(
        "--start-pose-policy",
        choices=["live_hold", "safe_center", "explicit_q", "current_measured_pose"],
        default="live_hold",
    )
    motion_joint_intent_parser.add_argument(
        "--max-joint-delta-rad",
        type=float,
        default=0.005,
    )
    motion_joint_intent_parser.add_argument(
        "--max-joint-velocity-rad-s",
        type=float,
        default=0.25,
    )
    motion_joint_intent_parser.add_argument(
        "--max-joint-acceleration-rad-s2",
        type=float,
    )
    motion_joint_intent_parser.add_argument(
        "--max-tracking-error-rad",
        type=float,
    )
    motion_joint_intent_parser.add_argument(
        "--tracking-error-grace-samples",
        type=int,
        default=3,
    )
    motion_joint_intent_parser.add_argument(
        "--tracking-error-consecutive-samples",
        type=int,
        default=3,
    )
    motion_joint_intent_parser.add_argument("--max-tau-abs", type=float)
    motion_joint_intent_parser.add_argument(
        "--max-start-error-rad",
        type=float,
        default=0.02,
    )
    motion_joint_intent_parser.add_argument(
        "--heartbeat-timeout-s",
        type=float,
        default=0.5,
    )
    motion_joint_intent_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    motion_joint_intent_parser.add_argument("--output")
    motion_joint_intent_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    motion_eef_delta_parser = motion_submit_subparsers.add_parser("eef-delta")
    motion_eef_delta_parser.add_argument("--session-artifact", required=True)
    motion_eef_delta_parser.add_argument("--owner", default="agent")
    motion_eef_delta_parser.add_argument(
        "--backend",
        required=True,
        choices=["moveit_servo", "sdk_cartesian"],
    )
    motion_eef_delta_parser.add_argument("--frame", default="eef_link")
    motion_eef_delta_parser.add_argument(
        "--expected-q-start",
        nargs="+",
        type=float,
        required=True,
    )
    motion_eef_delta_parser.add_argument(
        "--delta-position",
        nargs=3,
        type=float,
        required=True,
    )
    motion_eef_delta_parser.add_argument(
        "--delta-rpy",
        nargs=3,
        type=float,
        required=True,
    )
    motion_eef_delta_parser.add_argument("--control-period-s", type=float, default=0.1)
    motion_eef_delta_parser.add_argument("--send-hz", type=float, default=50.0)
    motion_eef_delta_parser.add_argument(
        "--max-linear-step-m", type=float, default=DEFAULT_EEF_MAX_LINEAR_STEP_M
    )
    motion_eef_delta_parser.add_argument(
        "--max-angular-step-rad", type=float, default=DEFAULT_EEF_MAX_ANGULAR_STEP_RAD
    )
    motion_eef_delta_parser.add_argument(
        "--start-pose-policy",
        choices=["live_hold", "safe_center", "explicit_q", "current_measured_pose"],
        default="live_hold",
    )
    motion_eef_delta_parser.add_argument(
        "--max-start-error-rad",
        type=float,
        default=0.02,
    )
    motion_eef_delta_parser.add_argument(
        "--heartbeat-timeout-s",
        type=float,
        default=0.5,
    )
    motion_eef_delta_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    motion_eef_delta_parser.add_argument("--output")
    motion_eef_delta_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    motion_eef_twist_parser = motion_submit_subparsers.add_parser("eef-twist")
    motion_eef_twist_parser.add_argument("--session-artifact", required=True)
    motion_eef_twist_parser.add_argument("--owner", default="agent")
    motion_eef_twist_parser.add_argument(
        "--backend",
        required=True,
        choices=["moveit_servo", "sdk_cartesian"],
    )
    motion_eef_twist_parser.add_argument("--frame", default="eef_link")
    motion_eef_twist_parser.add_argument(
        "--expected-q-start",
        nargs="+",
        type=float,
        required=True,
    )
    motion_eef_twist_parser.add_argument(
        "--linear",
        nargs=3,
        type=float,
        required=True,
    )
    motion_eef_twist_parser.add_argument(
        "--angular",
        nargs=3,
        type=float,
        required=True,
    )
    motion_eef_twist_parser.add_argument("--control-period-s", type=float, default=0.1)
    motion_eef_twist_parser.add_argument("--send-hz", type=float, default=50.0)
    motion_eef_twist_parser.add_argument(
        "--max-linear-step-m", type=float, default=DEFAULT_EEF_MAX_LINEAR_STEP_M
    )
    motion_eef_twist_parser.add_argument(
        "--max-angular-step-rad", type=float, default=DEFAULT_EEF_MAX_ANGULAR_STEP_RAD
    )
    motion_eef_twist_parser.add_argument(
        "--start-pose-policy",
        choices=["live_hold", "safe_center", "explicit_q", "current_measured_pose"],
        default="live_hold",
    )
    motion_eef_twist_parser.add_argument(
        "--max-start-error-rad",
        type=float,
        default=0.02,
    )
    motion_eef_twist_parser.add_argument(
        "--heartbeat-timeout-s",
        type=float,
        default=0.5,
    )
    motion_eef_twist_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    motion_eef_twist_parser.add_argument("--output")
    motion_eef_twist_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    motion_eef_pose_parser = motion_submit_subparsers.add_parser("eef-pose")
    motion_eef_pose_parser.add_argument("--session-artifact", required=True)
    motion_eef_pose_parser.add_argument("--owner", default="agent")
    motion_eef_pose_parser.add_argument(
        "--backend",
        choices=["moveit_servo", "sdk_cartesian"],
        default="moveit_servo",
    )
    motion_eef_pose_parser.add_argument("--frame", default="base_link")
    motion_eef_pose_parser.add_argument("--expected-q-start", nargs="+", type=float, required=True)
    motion_eef_pose_parser.add_argument("--position", nargs=3, type=float, required=True)
    motion_eef_pose_parser.add_argument("--rpy", nargs=3, type=float, required=True)
    motion_eef_pose_parser.add_argument("--control-period-s", type=float, default=0.1)
    motion_eef_pose_parser.add_argument("--send-hz", type=float, default=50.0)
    motion_eef_pose_parser.add_argument(
        "--start-pose-policy",
        choices=["live_hold", "safe_center", "explicit_q", "current_measured_pose"],
        default="live_hold",
    )
    motion_eef_pose_parser.add_argument(
        "--max-start-error-rad",
        type=float,
        default=0.02,
    )
    motion_eef_pose_parser.add_argument(
        "--heartbeat-timeout-s",
        type=float,
        default=0.5,
    )
    motion_eef_pose_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    motion_eef_pose_parser.add_argument("--output")
    motion_eef_pose_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    motion_joint_jog_parser = motion_submit_subparsers.add_parser("joint-jog")
    motion_joint_jog_parser.add_argument("--session-artifact", required=True)
    motion_joint_jog_parser.add_argument("--owner", default="teleop")
    motion_joint_jog_parser.add_argument("--joint-delta", nargs="+", type=float)
    motion_joint_jog_parser.add_argument("--velocity", nargs="+", type=float)
    motion_joint_jog_parser.add_argument("--deadman", action="store_true")
    motion_joint_jog_parser.add_argument("--output")
    motion_joint_jog_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    motion_result_parser = motion_subparsers.add_parser("result")
    motion_result_source = motion_result_parser.add_mutually_exclusive_group(
        required=True
    )
    motion_result_source.add_argument("--result-artifact")
    motion_result_source.add_argument("--run-dir")
    motion_result_parser.add_argument("--all", action="store_true")
    motion_result_parser.add_argument("--require-owner", action="append", default=[])
    motion_result_parser.add_argument("--expect-owner")
    motion_result_parser.add_argument("--expect-mode")
    motion_result_parser.add_argument("--expect-sample-count", type=int)
    motion_result_parser.add_argument("--max-jitter-p99-ms", type=float)
    motion_result_parser.add_argument("--max-tracking-error-rad", type=float)
    motion_result_parser.add_argument("--output")
    motion_result_parser.add_argument("--json", action="store_true", dest="as_json")

    profile_parser = subparsers.add_parser("profile")
    profile_subparsers = profile_parser.add_subparsers(
        dest="profile_command",
        required=True,
    )
    profile_list_parser = profile_subparsers.add_parser("list")
    profile_list_parser.add_argument("--json", action="store_true", dest="as_json")
    profile_show_parser = profile_subparsers.add_parser("show")
    profile_show_parser.add_argument("name")
    profile_show_parser.add_argument("--json", action="store_true", dest="as_json")

    console_parser = subparsers.add_parser("console")
    console_subparsers = console_parser.add_subparsers(
        dest="console_command",
        required=True,
    )
    console_status_parser = console_subparsers.add_parser("status")
    console_status_parser.add_argument("--session-artifact", required=True)
    console_status_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    console_status_parser.add_argument("--output")
    console_status_parser.add_argument("--json", action="store_true", dest="as_json")
    console_catalog_parser = console_subparsers.add_parser("catalog")
    console_catalog_parser.add_argument("--output")
    console_catalog_parser.add_argument("--json", action="store_true", dest="as_json")

    recipe_parser = subparsers.add_parser("recipe")
    recipe_subparsers = recipe_parser.add_subparsers(dest="recipe_command", required=True)

    list_parser = recipe_subparsers.add_parser("list")
    list_parser.add_argument("--json", action="store_true", dest="as_json")

    plan_parser = recipe_subparsers.add_parser("plan")
    plan_parser.add_argument("name")
    plan_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    plan_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    plan_parser.add_argument("--output")
    plan_parser.add_argument("--render", nargs="?", const="trajectory_preview.svg")
    plan_parser.add_argument("--sample-hz", type=float, default=50.0)
    plan_parser.add_argument("--duration", type=float, default=2.0)
    plan_parser.add_argument("--start-joints", nargs="+", type=float)
    plan_parser.add_argument("--json", action="store_true", dest="as_json")

    recipe_seed_parser = recipe_subparsers.add_parser("export-eef-seed")
    recipe_seed_parser.add_argument("--plan-dir", required=True)
    recipe_seed_parser.add_argument("--json", action="store_true", dest="as_json")

    recipe_agent_contract_parser = recipe_subparsers.add_parser(
        "export-agent-preset-contract"
    )
    recipe_agent_contract_parser.add_argument("--plan-dir", required=True)
    recipe_agent_contract_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    recipe_runtime_smoke_parser = recipe_subparsers.add_parser("runtime-smoke-fake")
    recipe_runtime_smoke_parser.add_argument("--plan-dir", required=True)
    recipe_runtime_smoke_parser.add_argument("--runtime-session-artifact")
    recipe_runtime_smoke_parser.add_argument("--output")
    recipe_runtime_smoke_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    recipe_runtime_submit_parser = recipe_subparsers.add_parser("runtime-submit")
    recipe_runtime_submit_parser.add_argument("--plan-dir", required=True)
    recipe_runtime_submit_parser.add_argument("--runtime-session-artifact", required=True)
    recipe_runtime_submit_parser.add_argument(
        "--max-start-error-rad",
        type=float,
        default=0.02,
    )
    recipe_runtime_submit_parser.add_argument(
        "--heartbeat-timeout-s",
        type=float,
    )
    recipe_runtime_submit_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    recipe_runtime_submit_parser.add_argument(
        "--max-tracking-error-rad",
        type=float,
    )
    recipe_runtime_submit_parser.add_argument(
        "--max-tau-abs",
        type=float,
    )
    recipe_runtime_submit_parser.add_argument(
        "--start-pose-policy",
        choices=["live_hold", "safe_center", "explicit_q"],
        default="live_hold",
    )
    recipe_runtime_submit_parser.add_argument("--output")
    recipe_runtime_submit_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    execute_parser = recipe_subparsers.add_parser("execute")
    execute_parser.add_argument("name")
    execute_parser.add_argument("--backend", default="not_configured")
    execute_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    execute_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    execute_parser.add_argument("--output")
    execute_parser.add_argument("--render", nargs="?", const="trajectory_preview.svg")
    execute_parser.add_argument("--sample-hz", type=float, default=50.0)
    execute_parser.add_argument("--duration", type=float, default=2.0)
    execute_parser.add_argument("--start-joints", nargs="+", type=float)
    execute_parser.add_argument("--json", action="store_true", dest="as_json")

    status_parser = recipe_subparsers.add_parser("status")
    status_parser.add_argument("--json", action="store_true", dest="as_json")

    cancel_parser = recipe_subparsers.add_parser("cancel")
    cancel_parser.add_argument("--json", action="store_true", dest="as_json")

    lerobot_parser = subparsers.add_parser("lerobot")
    lerobot_subparsers = lerobot_parser.add_subparsers(
        dest="lerobot_command",
        required=True,
    )

    lerobot_doctor_parser = lerobot_subparsers.add_parser("doctor")
    lerobot_doctor_parser.add_argument("--model", default="X5")
    lerobot_doctor_parser.add_argument("--robot-interface", default="can0")
    lerobot_doctor_parser.add_argument("--teleop-interface", default="can1")
    lerobot_doctor_parser.add_argument("--json", action="store_true", dest="as_json")

    lerobot_config_parser = lerobot_subparsers.add_parser("config-plan")
    lerobot_config_parser.add_argument("mode", choices=["record", "train", "rollout"])
    lerobot_config_parser.add_argument("--model", default="X5")
    lerobot_config_parser.add_argument("--robot-interface", default="can0")
    lerobot_config_parser.add_argument("--teleop-interface", default="can1")
    lerobot_config_parser.add_argument("--dataset-repo-id")
    lerobot_config_parser.add_argument("--task")
    lerobot_config_parser.add_argument("--episodes", type=int, default=10)
    lerobot_config_parser.add_argument("--policy", default="act")
    lerobot_config_parser.add_argument("--output-dir", default="outputs/train/act_arx5")
    lerobot_config_parser.add_argument("--job-name", default="act_arx5")
    lerobot_config_parser.add_argument("--policy-path")
    lerobot_config_parser.add_argument("--eef-plan-dir")
    lerobot_config_parser.add_argument("--json", action="store_true", dest="as_json")

    lerobot_review_parser = lerobot_subparsers.add_parser("review-rollout")
    lerobot_review_parser.add_argument("--eef-plan-dir", required=True)
    lerobot_review_parser.add_argument("--trajectory")
    lerobot_review_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    lerobot_review_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    lerobot_review_parser.add_argument("--render")
    lerobot_review_parser.add_argument("--json", action="store_true", dest="as_json")

    lerobot_stage_parser = lerobot_subparsers.add_parser("stage-rollout-trajectory")
    lerobot_stage_parser.add_argument("--eef-plan-dir", required=True)
    lerobot_stage_parser.add_argument("--trajectory", required=True)
    lerobot_stage_parser.add_argument("--json", action="store_true", dest="as_json")

    lerobot_preview_parser = lerobot_subparsers.add_parser("preview-rollout")
    lerobot_preview_parser.add_argument("--eef-plan-dir", required=True)
    lerobot_preview_parser.add_argument("--model", default="X5")
    lerobot_preview_parser.add_argument("--robot-interface", default="can0")
    lerobot_preview_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    lerobot_preview_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    lerobot_preview_parser.add_argument("--recipe-plan-dir")
    lerobot_preview_parser.add_argument("--start-joints", nargs="+", type=float)
    lerobot_preview_parser.add_argument("--sample-hz", type=float, default=50.0)
    lerobot_preview_parser.add_argument("--duration", type=float, default=2.0)
    lerobot_preview_parser.add_argument("--render")
    lerobot_preview_parser.add_argument("--json", action="store_true", dest="as_json")

    lerobot_processor_parser = lerobot_subparsers.add_parser("export-processor-contract")
    lerobot_processor_parser.add_argument("--eef-plan-dir", required=True)
    lerobot_processor_parser.add_argument("--model", default="X5")
    lerobot_processor_parser.add_argument("--robot-interface", default="can0")
    lerobot_processor_parser.add_argument("--output")
    lerobot_processor_parser.add_argument("--json", action="store_true", dest="as_json")

    lerobot_agent_runtime_helper_parser = lerobot_subparsers.add_parser(
        "agent-runtime-helper-plan"
    )
    lerobot_agent_runtime_helper_parser.add_argument(
        "--agent-runtime-contract", required=True
    )
    lerobot_agent_runtime_helper_parser.add_argument("--output")
    lerobot_agent_runtime_helper_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    lerobot_processor_helper_parser = lerobot_subparsers.add_parser(
        "processor-helper-preview"
    )
    lerobot_processor_helper_parser.add_argument("--processor-contract", required=True)
    lerobot_processor_helper_parser.add_argument(
        "--urdf-path", default="configs/models/X5_camera.urdf"
    )
    lerobot_processor_helper_parser.add_argument(
        "--safe-config", default="configs/x5.safe.yaml"
    )
    lerobot_processor_helper_parser.add_argument("--render")
    lerobot_processor_helper_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    lerobot_metadata_parser = lerobot_subparsers.add_parser("export-metadata")
    lerobot_metadata_parser.add_argument("--dataset-repo-id", required=True)
    lerobot_metadata_parser.add_argument("--parameter-bundle", required=True)
    lerobot_metadata_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    lerobot_metadata_parser.add_argument("--output", required=True)
    lerobot_metadata_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    agent_flow_parser = subparsers.add_parser("agent-flow")
    agent_flow_subparsers = agent_flow_parser.add_subparsers(
        dest="agent_flow_command",
        required=True,
    )

    agent_flow_plan_parser = agent_flow_subparsers.add_parser("plan")
    agent_flow_plan_parser.add_argument("--preset", required=True)
    agent_flow_plan_parser.add_argument(
        "--eef-mode",
        required=True,
        choices=["pose_absolute", "pose_delta", "twist"],
    )
    agent_flow_plan_parser.add_argument(
        "--backend",
        required=True,
        choices=["sdk_cartesian", "moveit_servo", "lerobot_rollout"],
    )
    agent_flow_plan_parser.add_argument("--frame", default="eef_link")
    agent_flow_plan_parser.add_argument("--position", nargs=3, type=float)
    agent_flow_plan_parser.add_argument("--rpy", nargs=3, type=float)
    agent_flow_plan_parser.add_argument("--delta-position", nargs=3, type=float)
    agent_flow_plan_parser.add_argument("--delta-rpy", nargs=3, type=float)
    agent_flow_plan_parser.add_argument("--linear", nargs=3, type=float)
    agent_flow_plan_parser.add_argument("--angular", nargs=3, type=float)
    agent_flow_plan_parser.add_argument("--control-period-s", type=float, default=0.1)
    agent_flow_plan_parser.add_argument("--model", default="X5")
    agent_flow_plan_parser.add_argument("--interface", default="can0")
    agent_flow_plan_parser.add_argument(
        "--policy-path",
        default="outputs/train/act_arx5/checkpoints/last/pretrained_model",
    )
    agent_flow_plan_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    agent_flow_plan_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    agent_flow_plan_parser.add_argument("--output", required=True)
    agent_flow_plan_parser.add_argument("--json", action="store_true", dest="as_json")

    agent_flow_doctor_parser = agent_flow_subparsers.add_parser("doctor")
    agent_flow_doctor_parser.add_argument("--json", action="store_true", dest="as_json")

    agent_flow_review_parser = agent_flow_subparsers.add_parser("review")
    agent_flow_review_parser.add_argument("--contract", required=True)
    agent_flow_review_parser.add_argument("--json", action="store_true", dest="as_json")

    agent_flow_runtime_smoke_parser = agent_flow_subparsers.add_parser(
        "runtime-smoke-fake"
    )
    agent_flow_runtime_smoke_parser.add_argument("--contract", required=True)
    agent_flow_runtime_smoke_parser.add_argument(
        "--q-start",
        nargs="+",
        type=float,
        required=True,
    )
    agent_flow_runtime_smoke_parser.add_argument(
        "--q-target",
        nargs="+",
        type=float,
        required=True,
    )
    agent_flow_runtime_smoke_parser.add_argument("--send-hz", type=float, default=50.0)
    agent_flow_runtime_smoke_parser.add_argument(
        "--max-joint-delta-rad",
        type=float,
        default=0.005,
    )
    agent_flow_runtime_smoke_parser.add_argument("--runtime-session-artifact")
    agent_flow_runtime_smoke_parser.add_argument("--output")
    agent_flow_runtime_smoke_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    agent_flow_real_runtime_smoke_parser = agent_flow_subparsers.add_parser(
        "runtime-smoke-real"
    )
    agent_flow_real_runtime_smoke_parser.add_argument("--contract", required=True)
    agent_flow_real_runtime_smoke_parser.add_argument(
        "--readiness-artifact",
        required=True,
    )
    agent_flow_real_runtime_smoke_parser.add_argument("--model", default="X5")
    agent_flow_real_runtime_smoke_parser.add_argument("--interface", required=True)
    agent_flow_real_runtime_smoke_parser.add_argument(
        "--q-start",
        nargs="+",
        type=float,
        required=True,
    )
    agent_flow_real_runtime_smoke_parser.add_argument(
        "--q-target",
        nargs="+",
        type=float,
        required=True,
    )
    agent_flow_real_runtime_smoke_parser.add_argument(
        "--send-hz",
        type=float,
        default=50.0,
    )
    agent_flow_real_runtime_smoke_parser.add_argument(
        "--max-joint-delta-rad",
        type=float,
        default=0.005,
    )
    agent_flow_real_runtime_smoke_parser.add_argument(
        "--max-joint-velocity-rad-s",
        type=float,
    )
    agent_flow_real_runtime_smoke_parser.add_argument(
        "--max-joint-acceleration-rad-s2",
        type=float,
    )
    agent_flow_real_runtime_smoke_parser.add_argument(
        "--max-tracking-error-rad",
        type=float,
    )
    agent_flow_real_runtime_smoke_parser.add_argument(
        "--max-tau-abs",
        type=float,
    )
    agent_flow_real_runtime_smoke_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    agent_flow_real_runtime_smoke_parser.add_argument("--runtime-session-artifact")
    agent_flow_real_runtime_smoke_parser.add_argument("--confirm", required=True)
    agent_flow_real_runtime_smoke_parser.add_argument("--output")
    agent_flow_real_runtime_smoke_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_parser = subparsers.add_parser("sysid")
    sysid_subparsers = sysid_parser.add_subparsers(dest="sysid_command", required=True)

    eef_parser = subparsers.add_parser("eef")
    eef_subparsers = eef_parser.add_subparsers(dest="eef_command", required=True)

    eef_doctor_parser = eef_subparsers.add_parser("doctor")
    eef_doctor_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_twist_parser = eef_subparsers.add_parser("plan-twist")
    eef_twist_parser.add_argument("--frame", default="eef_link")
    eef_twist_parser.add_argument("--linear", nargs=3, type=float, required=True)
    eef_twist_parser.add_argument("--angular", nargs=3, type=float, required=True)
    eef_twist_parser.add_argument("--backend", default="moveit_servo")
    eef_twist_parser.add_argument("--control-period-s", type=float, default=0.1)
    eef_twist_parser.add_argument("--output")
    eef_twist_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    eef_twist_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_pose_parser = eef_subparsers.add_parser("plan-pose")
    eef_pose_parser.add_argument("--frame", default="eef_link")
    eef_pose_parser.add_argument("--position", nargs=3, type=float, required=True)
    eef_pose_parser.add_argument("--rpy", nargs=3, type=float, required=True)
    eef_pose_parser.add_argument("--backend", default="pink")
    eef_pose_parser.add_argument("--output")
    eef_pose_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    eef_pose_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_delta_pose_parser = eef_subparsers.add_parser("plan-delta-pose")
    eef_delta_pose_parser.add_argument("--frame", default="eef_link")
    eef_delta_pose_parser.add_argument("--delta-position", nargs=3, type=float, required=True)
    eef_delta_pose_parser.add_argument("--delta-rpy", nargs=3, type=float, required=True)
    eef_delta_pose_parser.add_argument("--backend", default="sdk_cartesian")
    eef_delta_pose_parser.add_argument("--control-period-s", type=float, default=0.1)
    eef_delta_pose_parser.add_argument("--output")
    eef_delta_pose_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    eef_delta_pose_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_review_parser = eef_subparsers.add_parser("review")
    eef_review_parser.add_argument("--plan-dir", required=True)
    eef_review_parser.add_argument("--trajectory")
    eef_review_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    eef_review_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    eef_review_parser.add_argument("--render")
    eef_review_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_stage_parser = eef_subparsers.add_parser("stage-trajectory")
    eef_stage_parser.add_argument("--plan-dir", required=True)
    eef_stage_parser.add_argument("--trajectory", required=True)
    eef_stage_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_synthesize_parser = eef_subparsers.add_parser("synthesize-preview")
    eef_synthesize_parser.add_argument("--plan-dir", required=True)
    eef_synthesize_parser.add_argument("--backend", default="pink")
    eef_synthesize_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    eef_synthesize_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    eef_synthesize_parser.add_argument("--recipe-plan-dir")
    eef_synthesize_parser.add_argument("--start-joints", nargs="+", type=float)
    eef_synthesize_parser.add_argument("--sample-hz", type=float, default=50.0)
    eef_synthesize_parser.add_argument("--duration", type=float, default=2.0)
    eef_synthesize_parser.add_argument("--render")
    eef_synthesize_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_preview_runner_parser = eef_subparsers.add_parser("preview-runner")
    eef_preview_runner_parser.add_argument("--plan-dir", required=True)
    eef_preview_runner_parser.add_argument(
        "--backend",
        choices=["sdk_cartesian", "moveit_servo", "lerobot_rollout"],
    )
    eef_preview_runner_parser.add_argument("--model", default="X5")
    eef_preview_runner_parser.add_argument("--interface", default="can0")
    eef_preview_runner_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    eef_preview_runner_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    eef_preview_runner_parser.add_argument("--recipe-plan-dir")
    eef_preview_runner_parser.add_argument("--start-joints", nargs="+", type=float)
    eef_preview_runner_parser.add_argument("--sample-hz", type=float, default=50.0)
    eef_preview_runner_parser.add_argument("--duration", type=float, default=2.0)
    eef_preview_runner_parser.add_argument("--render")
    eef_preview_runner_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_sample_runner_parser = eef_subparsers.add_parser("sample-runner")
    eef_sample_runner_parser.add_argument("--runner-contract", required=True)
    eef_sample_runner_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    eef_sample_runner_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    eef_sample_runner_parser.add_argument("--recipe-plan-dir")
    eef_sample_runner_parser.add_argument("--start-joints", nargs="+", type=float)
    eef_sample_runner_parser.add_argument("--sample-hz", type=float, default=50.0)
    eef_sample_runner_parser.add_argument("--duration", type=float, default=2.0)
    eef_sample_runner_parser.add_argument("--render")
    eef_sample_runner_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_sdk_helper_plan_parser = eef_subparsers.add_parser("export-sdk-helper-plan")
    eef_sdk_helper_plan_parser.add_argument("--runner-contract", required=True)
    eef_sdk_helper_plan_parser.add_argument("--output")
    eef_sdk_helper_plan_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_moveit_helper_plan_parser = eef_subparsers.add_parser(
        "export-moveit-helper-plan"
    )
    eef_moveit_helper_plan_parser.add_argument("--runner-contract", required=True)
    eef_moveit_helper_plan_parser.add_argument("--output")
    eef_moveit_helper_plan_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_runtime_parser = eef_subparsers.add_parser("runtime-plan")
    eef_runtime_parser.add_argument(
        "--backend",
        choices=["sdk_cartesian", "moveit_servo", "lerobot_rollout"],
    )
    eef_runtime_parser.add_argument("--plan-dir")
    eef_runtime_parser.add_argument("--model", default="X5")
    eef_runtime_parser.add_argument("--interface", default="can0")
    eef_runtime_parser.add_argument("--policy-path")
    eef_runtime_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_lerobot_export_parser = eef_subparsers.add_parser("export-lerobot-action")
    eef_lerobot_export_parser.add_argument("--plan-dir", required=True)
    eef_lerobot_export_parser.add_argument("--output")
    eef_lerobot_export_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_sdk_export_parser = eef_subparsers.add_parser("export-sdk-cartesian")
    eef_sdk_export_parser.add_argument("--plan-dir", required=True)
    eef_sdk_export_parser.add_argument("--model", default="X5")
    eef_sdk_export_parser.add_argument("--interface", default="can0")
    eef_sdk_export_parser.add_argument("--output")
    eef_sdk_export_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_moveit_export_parser = eef_subparsers.add_parser("export-moveit-servo")
    eef_moveit_export_parser.add_argument("--plan-dir", required=True)
    eef_moveit_export_parser.add_argument("--output")
    eef_moveit_export_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_runtime_bridge_export_parser = eef_subparsers.add_parser("export-runtime-bridge")
    eef_runtime_bridge_export_parser.add_argument("--plan-dir", required=True)
    eef_runtime_bridge_export_parser.add_argument(
        "--backend",
        choices=["sdk_cartesian", "moveit_servo", "lerobot_rollout"],
    )
    eef_runtime_bridge_export_parser.add_argument("--model", default="X5")
    eef_runtime_bridge_export_parser.add_argument("--interface", default="can0")
    eef_runtime_bridge_export_parser.add_argument("--output")
    eef_runtime_bridge_export_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_runner_contract_parser = eef_subparsers.add_parser("export-runner-contract")
    eef_runner_contract_parser.add_argument("--plan-dir", required=True)
    eef_runner_contract_parser.add_argument(
        "--backend",
        choices=["sdk_cartesian", "moveit_servo", "lerobot_rollout"],
    )
    eef_runner_contract_parser.add_argument("--model", default="X5")
    eef_runner_contract_parser.add_argument("--interface", default="can0")
    eef_runner_contract_parser.add_argument("--output")
    eef_runner_contract_parser.add_argument("--json", action="store_true", dest="as_json")

    eef_agent_runtime_contract_parser = eef_subparsers.add_parser(
        "export-agent-runtime-contract"
    )
    eef_agent_runtime_contract_parser.add_argument("--plan-dir", required=True)
    eef_agent_runtime_contract_parser.add_argument(
        "--backend",
        choices=["sdk_cartesian", "moveit_servo", "lerobot_rollout"],
    )
    eef_agent_runtime_contract_parser.add_argument("--model", default="X5")
    eef_agent_runtime_contract_parser.add_argument("--interface", default="can0")
    eef_agent_runtime_contract_parser.add_argument("--output")
    eef_agent_runtime_contract_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    eef_agent_session_plan_parser = eef_subparsers.add_parser(
        "export-agent-session-plan"
    )
    eef_agent_session_plan_parser.add_argument("--plan-dir", required=True)
    eef_agent_session_plan_parser.add_argument(
        "--backend",
        choices=["sdk_cartesian", "moveit_servo", "lerobot_rollout"],
    )
    eef_agent_session_plan_parser.add_argument("--model", default="X5")
    eef_agent_session_plan_parser.add_argument("--interface", default="can0")
    eef_agent_session_plan_parser.add_argument("--output")
    eef_agent_session_plan_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    sim_parser = subparsers.add_parser("sim")
    sim_subparsers = sim_parser.add_subparsers(dest="sim_command", required=True)

    sim_doctor_parser = sim_subparsers.add_parser("doctor")
    sim_doctor_parser.add_argument("--json", action="store_true", dest="as_json")

    sim_preview_parser = sim_subparsers.add_parser("preview")
    sim_preview_parser.add_argument("--trajectory", required=True)
    sim_preview_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    sim_preview_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    sim_preview_parser.add_argument("--backend", default="auto")
    sim_preview_parser.add_argument("--render")
    sim_preview_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_plan_parser = sysid_subparsers.add_parser("plan")
    sysid_plan_parser.add_argument("profile")
    sysid_plan_parser.add_argument("--execute", action="store_true")
    sysid_plan_parser.add_argument("--dof", type=int, default=6)
    sysid_plan_parser.add_argument("--sample-hz", type=float, default=100.0)
    sysid_plan_parser.add_argument("--duration", type=float, default=10.0)
    sysid_plan_parser.add_argument("--amplitude", type=float, default=0.1)
    sysid_plan_parser.add_argument("--q-center", nargs="+", type=float)
    sysid_plan_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    sysid_plan_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    sysid_plan_parser.add_argument("--candidate-trajectory")
    sysid_plan_parser.add_argument("--output")
    sysid_plan_parser.add_argument("--render", nargs="?", const="trajectory_preview.svg")
    sysid_plan_parser.add_argument("--json", action="store_true", dest="as_json")
    sysid_plan_parser.add_argument("--trajectory-command", nargs=argparse.REMAINDER)

    sysid_run_parser = sysid_subparsers.add_parser("run")
    sysid_run_parser.add_argument("profile")
    sysid_run_parser.add_argument("--adapter", choices=["fake"], default="fake")
    sysid_run_parser.add_argument("--model", default="X5")
    sysid_run_parser.add_argument("--interface", default="can0")
    sysid_run_parser.add_argument("--dof", type=int, default=6)
    sysid_run_parser.add_argument("--sample-hz", type=float, default=100.0)
    sysid_run_parser.add_argument("--duration", type=float, default=10.0)
    sysid_run_parser.add_argument("--amplitude", type=float, default=0.1)
    sysid_run_parser.add_argument("--q-center", nargs="+", type=float)
    sysid_run_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    sysid_run_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    sysid_run_parser.add_argument("--candidate-trajectory")
    sysid_run_parser.add_argument("--output", required=True)
    sysid_run_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_compile_runtime_parser = sysid_subparsers.add_parser("compile-runtime")
    sysid_compile_runtime_parser.add_argument("--execution-trajectory", required=True)
    sysid_compile_runtime_parser.add_argument("--dof", type=int, default=6)
    sysid_compile_runtime_parser.add_argument("--sample-hz", type=float, default=100.0)
    sysid_compile_runtime_parser.add_argument(
        "--expected-q-start",
        nargs="+",
        type=float,
        required=True,
    )
    sysid_compile_runtime_parser.add_argument("--owner", default="sysid")
    sysid_compile_runtime_parser.add_argument(
        "--start-pose-policy",
        choices=["live_hold", "safe_center", "explicit_q", "current_measured_pose"],
        default="live_hold",
    )
    sysid_compile_runtime_parser.add_argument("--send-hz", type=float)
    sysid_compile_runtime_parser.add_argument("--max-tracking-error-rad", type=float)
    sysid_compile_runtime_parser.add_argument(
        "--tracking-error-grace-samples",
        type=int,
        default=3,
    )
    sysid_compile_runtime_parser.add_argument(
        "--tracking-error-consecutive-samples",
        type=int,
        default=3,
    )
    sysid_compile_runtime_parser.add_argument("--max-tau-abs", type=float)
    sysid_compile_runtime_parser.add_argument("--output", required=True)
    sysid_compile_runtime_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    sysid_postprocess_parser = sysid_subparsers.add_parser("postprocess")
    sysid_postprocess_parser.add_argument("--dataset", required=True)
    sysid_postprocess_parser.add_argument("--solve", action="store_true")
    sysid_postprocess_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_solve_parser = sysid_subparsers.add_parser("solve")
    sysid_solve_parser.add_argument("--dataset", required=True)
    sysid_solve_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_analyze_measured_parser = sysid_subparsers.add_parser("analyze-measured")
    sysid_analyze_measured_parser.add_argument("--dataset", required=True)
    sysid_analyze_measured_parser.add_argument("--urdf-path", required=True)
    sysid_analyze_measured_parser.add_argument("--primary", required=True)
    sysid_analyze_measured_parser.add_argument(
        "--validation",
        action="append",
        default=[],
    )
    sysid_analyze_measured_parser.add_argument("--output", required=True)
    sysid_analyze_measured_parser.add_argument(
        "--filter-method",
        choices=["savgol"],
        default="savgol",
    )
    sysid_analyze_measured_parser.add_argument(
        "--savgol-window-samples",
        type=int,
        default=21,
    )
    sysid_analyze_measured_parser.add_argument(
        "--savgol-polyorder",
        type=int,
        default=3,
    )
    sysid_analyze_measured_parser.add_argument(
        "--json", action="store_true", dest="as_json"
    )

    sysid_review_parser = sysid_subparsers.add_parser("review-candidate")
    sysid_review_parser.add_argument("--plan-dir", required=True)
    sysid_review_parser.add_argument("--trajectory")
    sysid_review_parser.add_argument("--output", required=True)
    sysid_review_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    sysid_review_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    sysid_review_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_package_parser = sysid_subparsers.add_parser("package")
    sysid_package_parser.add_argument("--dataset", required=True)
    sysid_package_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_evidence_parser = sysid_subparsers.add_parser("import-evidence")
    sysid_evidence_parser.add_argument("--dataset", required=True)
    sysid_evidence_parser.add_argument("--evidence", required=True)
    sysid_evidence_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_figaroh_adapter_parser = sysid_subparsers.add_parser(
        "adapt-figaroh-evidence"
    )
    sysid_figaroh_adapter_parser.add_argument("--input", required=True)
    sysid_figaroh_adapter_parser.add_argument("--output", required=True)
    sysid_figaroh_adapter_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_figaroh_handoff_parser = sysid_subparsers.add_parser("figaroh-handoff")
    sysid_figaroh_handoff_parser.add_argument("--dataset", required=True)
    sysid_figaroh_handoff_parser.add_argument("--output", required=True)
    sysid_figaroh_handoff_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_sdk_preflight_parser = sysid_subparsers.add_parser("sdk-preflight")
    sysid_sdk_preflight_parser.add_argument("--model", default="X5")
    sysid_sdk_preflight_parser.add_argument("--interface", required=True)
    sysid_sdk_preflight_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_sdk_doctor_parser = sysid_subparsers.add_parser("sdk-doctor")
    sysid_sdk_doctor_parser.add_argument("--model", default="X5")
    sysid_sdk_doctor_parser.add_argument("--interface", required=True)
    sysid_sdk_doctor_parser.add_argument("--state-sample-count", type=int, default=10)
    sysid_sdk_doctor_parser.add_argument(
        "--state-sample-period",
        type=float,
        default=0.01,
    )
    sysid_sdk_doctor_parser.add_argument("--output")
    sysid_sdk_doctor_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_sdk_hold_damping_parser = sysid_subparsers.add_parser(
        "sdk-hold-damping-check"
    )
    sysid_sdk_hold_damping_parser.add_argument("--model", default="X5")
    sysid_sdk_hold_damping_parser.add_argument("--interface", required=True)
    sysid_sdk_hold_damping_parser.add_argument("--confirm", required=True)
    sysid_sdk_hold_damping_parser.add_argument("--doctor-artifact", required=True)
    sysid_sdk_hold_damping_parser.add_argument("--output")
    sysid_sdk_hold_damping_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_sdk_arm_session_parser = sysid_subparsers.add_parser("sdk-arm-session")
    sysid_sdk_arm_session_parser.add_argument("--model", default="X5")
    sysid_sdk_arm_session_parser.add_argument("--interface", required=True)
    sysid_sdk_arm_session_parser.add_argument("--doctor-artifact", required=True)
    sysid_sdk_arm_session_parser.add_argument("--hold-damping-artifact", required=True)
    sysid_sdk_arm_session_parser.add_argument("--confirm", required=True)
    sysid_sdk_arm_session_parser.add_argument("--output")
    sysid_sdk_arm_session_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_sdk_jog_real_parser = sysid_subparsers.add_parser("sdk-jog-real")
    sysid_sdk_jog_real_parser.add_argument("--session-artifact", required=True)
    sysid_sdk_jog_real_parser.add_argument("--joint-index", type=int, required=True)
    sysid_sdk_jog_real_parser.add_argument("--delta-rad", type=float, required=True)
    sysid_sdk_jog_real_parser.add_argument(
        "--max-delta-rad",
        type=float,
        default=0.005,
    )
    sysid_sdk_jog_real_parser.add_argument("--send-hz", type=float, default=50.0)
    sysid_sdk_jog_real_parser.add_argument(
        "--max-q-current-error-rad",
        type=float,
        default=DEFAULT_Q_CURRENT_MAX_ERROR_RAD,
    )
    sysid_sdk_jog_real_parser.add_argument("--confirm", required=True)
    sysid_sdk_jog_real_parser.add_argument("--output")
    sysid_sdk_jog_real_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_sdk_recover_startup_parser = sysid_subparsers.add_parser(
        "sdk-recover-startup-real"
    )
    sysid_sdk_recover_startup_parser.add_argument("--session-artifact", required=True)
    sysid_sdk_recover_startup_parser.add_argument(
        "--q-target",
        type=float,
        nargs="+",
        default=[0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
    )
    sysid_sdk_recover_startup_parser.add_argument(
        "--safe-config",
        default="configs/x5.safe.yaml",
    )
    sysid_sdk_recover_startup_parser.add_argument(
        "--max-joint-step-rad",
        type=float,
    )
    sysid_sdk_recover_startup_parser.add_argument("--send-hz", type=float, default=50.0)
    sysid_sdk_recover_startup_parser.add_argument(
        "--hold-seconds",
        type=float,
        default=None,
        help=(
            "Keep streaming the recovered startup pose for this many seconds. "
            "Default holds until Ctrl-C; pass 0 to disable active hold."
        ),
    )
    sysid_sdk_recover_startup_parser.add_argument(
        "--hold-hz",
        type=float,
        default=50.0,
        help="Command frequency for the post-recovery active hold stream.",
    )
    sysid_sdk_recover_startup_parser.add_argument("--confirm", required=True)
    sysid_sdk_recover_startup_parser.add_argument("--output")
    sysid_sdk_recover_startup_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_sdk_tiny_motion_parser = sysid_subparsers.add_parser(
        "sdk-tiny-motion-plan"
    )
    sysid_sdk_tiny_motion_parser.add_argument("--doctor-artifact", required=True)
    sysid_sdk_tiny_motion_parser.add_argument(
        "--hold-damping-artifact",
        required=True,
    )
    sysid_sdk_tiny_motion_parser.add_argument("--joint-index", type=int, required=True)
    sysid_sdk_tiny_motion_parser.add_argument("--delta-rad", type=float, required=True)
    sysid_sdk_tiny_motion_parser.add_argument(
        "--max-delta-rad",
        type=float,
        default=0.005,
    )
    sysid_sdk_tiny_motion_parser.add_argument("--confirm", required=True)
    sysid_sdk_tiny_motion_parser.add_argument("--output")
    sysid_sdk_tiny_motion_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_sdk_tiny_motion_execute_fake_parser = sysid_subparsers.add_parser(
        "sdk-tiny-motion-execute-fake"
    )
    sysid_sdk_tiny_motion_execute_fake_parser.add_argument(
        "--plan-artifact",
        required=True,
    )
    sysid_sdk_tiny_motion_execute_fake_parser.add_argument(
        "--dof",
        type=int,
        required=True,
    )
    sysid_sdk_tiny_motion_execute_fake_parser.add_argument(
        "--q-current",
        type=float,
        nargs="+",
        required=True,
    )
    sysid_sdk_tiny_motion_execute_fake_parser.add_argument(
        "--send-hz",
        type=float,
        default=50.0,
    )
    sysid_sdk_tiny_motion_execute_fake_parser.add_argument("--confirm", required=True)
    sysid_sdk_tiny_motion_execute_fake_parser.add_argument("--output")
    sysid_sdk_tiny_motion_execute_fake_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_sdk_tiny_motion_execute_real_parser = sysid_subparsers.add_parser(
        "sdk-tiny-motion-execute-real"
    )
    sysid_sdk_tiny_motion_execute_real_parser.add_argument(
        "--plan-artifact",
        required=True,
    )
    sysid_sdk_tiny_motion_execute_real_parser.add_argument(
        "--doctor-artifact",
        required=True,
    )
    sysid_sdk_tiny_motion_execute_real_parser.add_argument(
        "--hold-damping-artifact",
        required=True,
    )
    sysid_sdk_tiny_motion_execute_real_parser.add_argument("--model", default="X5")
    sysid_sdk_tiny_motion_execute_real_parser.add_argument(
        "--interface",
        required=True,
    )
    sysid_sdk_tiny_motion_execute_real_parser.add_argument(
        "--dof",
        type=int,
        required=True,
    )
    sysid_sdk_tiny_motion_execute_real_parser.add_argument(
        "--q-current",
        type=float,
        nargs="+",
        required=True,
    )
    sysid_sdk_tiny_motion_execute_real_parser.add_argument(
        "--send-hz",
        type=float,
        default=50.0,
    )
    sysid_sdk_tiny_motion_execute_real_parser.add_argument(
        "--max-q-current-error-rad",
        type=float,
        default=DEFAULT_Q_CURRENT_MAX_ERROR_RAD,
        help=(
            "Reject real tiny motion if operator --q-current differs from measured "
            "SDK joint state by more than this per-joint max absolute error."
        ),
    )
    sysid_sdk_tiny_motion_execute_real_parser.add_argument("--confirm", required=True)
    sysid_sdk_tiny_motion_execute_real_parser.add_argument("--output")
    sysid_sdk_tiny_motion_execute_real_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_agent_sysid_smoke_readiness_parser = sysid_subparsers.add_parser(
        "sdk-agent-sysid-smoke-readiness"
    )
    sysid_agent_sysid_smoke_readiness_parser.add_argument(
        "--doctor-artifact",
        required=True,
    )
    sysid_agent_sysid_smoke_readiness_parser.add_argument(
        "--hold-damping-artifact",
        required=True,
    )
    sysid_agent_sysid_smoke_readiness_parser.add_argument(
        "--runtime-status-artifact",
        required=True,
    )
    sysid_agent_sysid_smoke_readiness_parser.add_argument(
        "--max-heartbeat-age-s",
        type=float,
        default=1.0,
    )
    sysid_agent_sysid_smoke_readiness_parser.add_argument("--output")
    sysid_agent_sysid_smoke_readiness_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_sdk_handshake_parser = sysid_subparsers.add_parser("sdk-handshake-plan")
    sysid_sdk_handshake_parser.add_argument("--model", default="X5")
    sysid_sdk_handshake_parser.add_argument("--interface", required=True)
    sysid_sdk_handshake_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    online_parser = subparsers.add_parser("online-id")
    online_subparsers = online_parser.add_subparsers(
        dest="online_command",
        required=True,
    )
    online_policy_parser = online_subparsers.add_parser("policy")
    online_policy_parser.add_argument("--json", action="store_true", dest="as_json")

    online_audit_parser = online_subparsers.add_parser("audit")
    online_audit_parser.add_argument("--parameter", required=True)
    online_audit_parser.add_argument("--value", type=float, required=True)
    online_audit_parser.add_argument("--source", required=True)
    online_audit_parser.add_argument("--window-start", type=float, required=True)
    online_audit_parser.add_argument("--window-end", type=float, required=True)
    online_audit_parser.add_argument("--residual-before", type=float, required=True)
    online_audit_parser.add_argument("--residual-after", type=float, required=True)
    online_audit_parser.add_argument("--saturation-status", required=True)
    online_audit_parser.add_argument("--rollback-target", required=True)
    online_audit_parser.add_argument("--output", required=True)
    online_audit_parser.add_argument("--json", action="store_true", dest="as_json")

    release_parser = subparsers.add_parser("release")
    release_subparsers = release_parser.add_subparsers(
        dest="release_command",
        required=True,
    )
    release_status_parser = release_subparsers.add_parser("status")
    release_status_parser.add_argument("--json", action="store_true", dest="as_json")

    release_notes_parser = release_subparsers.add_parser("notes")
    release_notes_parser.add_argument("--json", action="store_true", dest="as_json")

    args = parser.parse_args(argv)
    if not hasattr(args, "candidate_trajectory"):
        args.candidate_trajectory = None
    catalog = RecipeCatalog.default()

    if args.command == "runtime" and args.runtime_command == "start":
        if args.serve and args.output is None:
            parser.error("runtime start --serve requires --output")
        if args.backend == "arx5_sdk":
            rejected = arx5_runtime_start_preflight(
                model=args.model,
                interface=args.interface,
                confirm=args.confirm,
            )
            if rejected is not None:
                payload = _attach_output_artifact(
                    rejected,
                    args.output,
                    artifact_key="runtime_session",
                )
                _emit(payload, as_json=args.as_json)
                return 3
            backend = Arx5SdkJointRuntimeBackend(
                model=args.model,
                interface=args.interface,
                max_joint_step_rad=args.max_joint_step_rad,
                shutdown_to_passive=False,
            )
            try:
                payload = start_arx5_runtime_session(
                    backend=backend,
                    model=args.model,
                    interface=args.interface,
                    safe_center=tuple(args.safe_center),
                    send_hz=args.send_hz,
                    hold_hz=args.hold_hz,
                    max_joint_step_rad=args.max_joint_step_rad,
                    max_heartbeat_age_s=args.max_heartbeat_age_s,
                )
            except Exception as error:
                try:
                    backend.damping()
                except Exception:
                    pass
                payload = {
                    "status": "rejected",
                    "schema": "armctrl.arm_runtime_session.v1",
                    "backend": "arx5_sdk",
                    "model": args.model,
                    "interface": args.interface,
                    "requires_confirm": ARX5_RUNTIME_START_CONFIRMATION,
                    "hardware_motion": True,
                    "movement_command_sent": "unknown",
                    "sdk_opened": "unknown",
                    "reason": f"arx5 runtime start failed: {error}",
                    "fault_landing_mode": "damping",
                    "next_gate": "inspect robot state and SDK logs before retrying runtime start",
                }
                payload = _attach_output_artifact(
                    payload,
                    args.output,
                    artifact_key="runtime_session",
                )
                _emit(payload, as_json=args.as_json)
                return 3
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="runtime_session",
            )
            if args.serve:
                payload = _attach_eef_adapter_manager_from_args(
                    payload,
                    args,
                    allow_configured_adapters=False,
                )
                _write_json_atomic(Path(args.output), payload)
                _clear_runtime_stop_request(Path(args.output))
                runtime = _live_runtime_from_session_payload(
                    backend=backend,
                    payload=payload,
                )
                payload = _serve_runtime_session_until_stopped(
                    session_artifact_path=Path(args.output),
                    heartbeat_period_s=args.heartbeat_period_s,
                    max_heartbeat_age_s=args.max_heartbeat_age_s,
                    backend=backend,
                    runtime=runtime,
                    hold_tick=_arx5_active_hold_tick(
                        backend=backend,
                        hold_hz=args.hold_hz,
                    ),
                )
            _emit(payload, as_json=args.as_json)
            return 3
        if args.backend == "sdk_cartesian":
            if not args.allow_diagnostic_sdk_cartesian_takeover:
                payload = {
                    "status": "rejected",
                    "schema": "armctrl.arm_runtime_session.v1",
                    "backend": "sdk_cartesian",
                    "model": args.model,
                    "interface": args.interface,
                    "requires_confirm": ARX5_RUNTIME_START_CONFIRMATION,
                    "hardware_motion": True,
                    "movement_command_sent": False,
                    "sdk_opened": False,
                    "diagnostic_only": True,
                    "formal_runtime_gateway": False,
                    "reason": (
                        "runtime start --backend sdk_cartesian is a disconnected "
                        "diagnostic takeover, not the formal Agent EEF runtime path"
                    ),
                    "formal_agent_eef_path": (
                        "start arx5_sdk runtime once, keep SDK/CAN owner alive, "
                        "then submit eef-delta/eef-twist/eef-pose commands through "
                        "the runtime-owned motion surface"
                    ),
                    "requires_opt_in_flag": (
                        "--allow-diagnostic-sdk-cartesian-takeover"
                    ),
                    "fault_landing_mode": "damping",
                    "next_gate": (
                        "use armctrl runtime start --backend arx5_sdk --serve for "
                        "formal Agent EEF, or explicitly opt into diagnostic takeover"
                    ),
                }
                payload = _attach_output_artifact(
                    payload,
                    args.output,
                    artifact_key="runtime_session",
                )
                _emit(payload, as_json=args.as_json)
                return 3
            rejected = arx5_runtime_start_preflight(
                model=args.model,
                interface=args.interface,
                confirm=args.confirm,
            )
            if rejected is not None:
                rejected["backend"] = "sdk_cartesian"
                rejected["next_gate"] = "confirm sdk_cartesian runtime start on the robot host after arx5_sdk recovery to SAFE_CENTER"
                payload = _attach_output_artifact(
                    rejected,
                    args.output,
                    artifact_key="runtime_session",
                )
                _emit(payload, as_json=args.as_json)
                return 3
            recovery_payload: dict[str, object] | None = None
            if args.recover_to_safe_first:
                recovery_backend = Arx5SdkJointRuntimeBackend(
                    model=args.model,
                    interface=args.interface,
                    max_joint_step_rad=args.max_joint_step_rad,
                    shutdown_to_passive=False,
                )
                try:
                    recovery_payload = start_arx5_runtime_session(
                        backend=recovery_backend,
                        model=args.model,
                        interface=args.interface,
                        safe_center=tuple(args.safe_center),
                        send_hz=args.send_hz,
                        hold_hz=args.hold_hz,
                        max_joint_step_rad=args.max_joint_step_rad,
                        max_heartbeat_age_s=args.max_heartbeat_age_s,
                    )
                except Exception as error:
                    try:
                        recovery_backend.damping()
                    except Exception:
                        pass
                    payload = {
                        "status": "rejected",
                        "schema": "armctrl.arm_runtime_session.v1",
                        "backend": "sdk_cartesian",
                        "model": args.model,
                        "interface": args.interface,
                        "requires_confirm": ARX5_RUNTIME_START_CONFIRMATION,
                        "hardware_motion": True,
                        "movement_command_sent": "unknown",
                        "sdk_opened": "unknown",
                        "reason": f"sdk_cartesian pre-recovery failed: {error}",
                        "fault_landing_mode": "damping",
                        "next_gate": "inspect robot state and SDK logs before retrying Agent EEF runtime start",
                    }
                    payload = _attach_output_artifact(
                        payload,
                        args.output,
                        artifact_key="runtime_session",
                    )
                    _emit(payload, as_json=args.as_json)
                    return 3
                del recovery_backend
                time.sleep(0.05)
            backend = Arx5SdkCartesianRuntimeBackend(
                model=args.model,
                interface=args.interface,
            )
            try:
                payload = start_arx5_cartesian_runtime_session(
                    backend=backend,
                    model=args.model,
                    interface=args.interface,
                    safe_center=tuple(args.safe_center),
                    send_hz=args.send_hz,
                    hold_hz=args.hold_hz,
                    max_start_error_rad=args.max_start_error_rad,
                    max_heartbeat_age_s=args.max_heartbeat_age_s,
                )
            except Exception as error:
                try:
                    backend.damping()
                except Exception:
                    pass
                payload = {
                    "status": "rejected",
                    "schema": "armctrl.arm_runtime_session.v1",
                    "backend": "sdk_cartesian",
                    "model": args.model,
                    "interface": args.interface,
                    "requires_confirm": ARX5_RUNTIME_START_CONFIRMATION,
                    "hardware_motion": True,
                    "movement_command_sent": "unknown",
                    "sdk_opened": "unknown",
                    "reason": f"sdk_cartesian runtime start failed: {error}",
                    "fault_landing_mode": "damping",
                    "next_gate": "inspect sdk_cartesian current-pose takeover and measured q before retrying Agent EEF runtime",
                }
                if recovery_payload is not None:
                    payload["pre_recovery"] = {
                        "backend": "arx5_sdk",
                        "status": recovery_payload.get("status"),
                        "mode": recovery_payload.get("mode"),
                        "q_meas_after_recovery": recovery_payload.get("q_meas"),
                        "q_hold_after_recovery": recovery_payload.get("q_hold"),
                        "recovery": recovery_payload.get("recovery"),
                        "handoff_policy": "optional_joint_recover_then_current_pose_cartesian_takeover",
                    }
                payload = _attach_output_artifact(
                    payload,
                    args.output,
                    artifact_key="runtime_session",
                )
                _emit(payload, as_json=args.as_json)
                return 3
            if recovery_payload is not None:
                payload["pre_recovery"] = {
                    "backend": "arx5_sdk",
                    "status": recovery_payload.get("status"),
                    "mode": recovery_payload.get("mode"),
                    "q_start": recovery_payload.get("recovery", {}).get("q_start")
                    if isinstance(recovery_payload.get("recovery"), dict)
                    else None,
                    "q_meas_after_recovery": recovery_payload.get("q_meas"),
                    "q_hold_after_recovery": recovery_payload.get("q_hold"),
                    "recovery": recovery_payload.get("recovery"),
                    "handoff_policy": "optional_joint_recover_then_current_pose_cartesian_takeover",
                }
                contract = dict(payload.get("runtime_contract", {}))
                contract["droop_recovery_supported"] = True
                contract["recovery_backend"] = "arx5_sdk"
                contract["handoff_guard"] = "sdk_cartesian_current_pose_takeover_after_recovery"
                payload["runtime_contract"] = contract
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="runtime_session",
            )
            if args.serve:
                payload = _attach_eef_adapter_manager_from_args(
                    payload,
                    args,
                    allow_configured_adapters=False,
                )
                _write_json_atomic(Path(args.output), payload)
                _clear_runtime_stop_request(Path(args.output))
                runtime = _live_runtime_from_session_payload(
                    backend=backend,
                    payload=payload,
                )
                payload = _serve_runtime_session_until_stopped(
                    session_artifact_path=Path(args.output),
                    heartbeat_period_s=args.heartbeat_period_s,
                    max_heartbeat_age_s=args.max_heartbeat_age_s,
                    backend=backend,
                    runtime=runtime,
                    hold_tick=_sdk_cartesian_active_hold_tick(
                        backend=backend,
                        hold_hz=args.hold_hz,
                    ),
                )
            _emit(payload, as_json=args.as_json)
            return 3
        if args.q_current is None:
            parser.error("runtime start --backend fake requires --q-current")
        payload = start_fake_runtime_session(
            q_current=tuple(args.q_current),
            safe_center=tuple(args.safe_center),
            send_hz=args.send_hz,
            hold_hz=args.hold_hz,
            max_joint_step_rad=args.max_joint_step_rad,
            max_heartbeat_age_s=args.max_heartbeat_age_s,
        )
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="runtime_session",
        )
        if args.serve:
            payload = _attach_eef_adapter_manager_from_args(payload, args)
            _write_json_atomic(Path(args.output), payload)
            _clear_runtime_stop_request(Path(args.output))
            backend = FakeMotionBackend()
            backend.send_joint_command(
                tuple(float(value) for value in payload.get("q_meas") or []),
                producer="runtime_serve_bootstrap",
                mode=MotionMode.HOLD,
                monotonic_s=0.0,
            )
            runtime = _live_runtime_from_session_payload(
                backend=backend,
                payload=payload,
            )
            payload = _serve_runtime_session_until_stopped(
                session_artifact_path=Path(args.output),
                heartbeat_period_s=args.heartbeat_period_s,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
                backend=backend,
                runtime=runtime,
                eef_backends=_runtime_eef_backends_from_args(
                    args,
                    q_state=tuple(float(value) for value in payload.get("q_meas") or ()),
                ),
            )
        return _emit(payload, as_json=args.as_json)

    if args.command == "runtime" and args.runtime_command == "status":
        payload = runtime_status_from_artifact(
            session_artifact_path=Path(args.session_artifact),
            max_heartbeat_age_s=args.max_heartbeat_age_s,
        )
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="runtime_status",
        )
        _emit(payload, as_json=args.as_json)
        return 0 if payload.get("status") == "ok" else 3

    if args.command == "runtime" and args.runtime_command == "recover":
        try:
            payload = recover_runtime_session_from_artifact(
                session_artifact_path=Path(args.session_artifact),
                safe_center=tuple(args.safe_center),
                send_hz=args.send_hz,
                hold_hz=args.hold_hz,
                max_joint_step_rad=args.max_joint_step_rad,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
            )
        except RuntimeSessionError as error:
            payload = {
                **error.payload,
                "status": "rejected",
                "schema": "armctrl.arm_runtime_status.v1",
                "reason": str(error),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="runtime_session",
        )
        _emit(payload, as_json=args.as_json)
        return 0 if payload.get("status") == "ok" else 3

    if args.command == "runtime" and args.runtime_command == "stop":
        _write_runtime_stop_request(Path(args.session_artifact))
        payload = stop_runtime_session_from_artifact(
            session_artifact_path=Path(args.session_artifact),
            max_heartbeat_age_s=args.max_heartbeat_age_s,
        )
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="runtime_session",
        )
        _emit(payload, as_json=args.as_json)
        return 0 if payload.get("status") == "ok" else 3

    if args.command == "runtime" and args.runtime_command == "acquire-owner":
        try:
            payload = acquire_owner_from_artifact(
                session_artifact_path=Path(args.session_artifact),
                owner=args.owner,
                mode=args.mode,
                expected_q_start=tuple(args.expected_q_start),
                max_start_error_rad=args.max_start_error_rad,
                heartbeat_timeout_s=args.heartbeat_timeout_s,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
            )
        except RuntimeSessionError as error:
            payload = {
                **error.payload,
                "status": "rejected",
                "schema": "armctrl.arm_runtime_status.v1",
                "reason": str(error),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="runtime_session",
        )
        return _emit(payload, as_json=args.as_json)

    if args.command == "runtime" and args.runtime_command == "release-owner":
        try:
            payload = release_owner_from_artifact(
                session_artifact_path=Path(args.session_artifact),
                owner=args.owner,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
            )
        except RuntimeSessionError as error:
            payload = {
                **error.payload,
                "status": "rejected",
                "schema": "armctrl.arm_runtime_status.v1",
                "reason": str(error),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="runtime_session",
        )
        return _emit(payload, as_json=args.as_json)

    if args.command == "runtime" and args.runtime_command == "owner-heartbeat":
        try:
            payload = owner_heartbeat_from_artifact(
                session_artifact_path=Path(args.session_artifact),
                owner=args.owner,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
            )
        except RuntimeSessionError as error:
            payload = {
                **error.payload,
                "status": "rejected",
                "schema": "armctrl.arm_runtime_status.v1",
                "reason": str(error),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="runtime_session",
        )
        _emit(payload, as_json=args.as_json)
        return 0 if payload.get("status") in {"ok", "blocked"} else 3

    if args.command == "runtime" and args.runtime_command == "watchdog-tick":
        payload = watchdog_tick_from_artifact(
            session_artifact_path=Path(args.session_artifact),
            max_heartbeat_age_s=args.max_heartbeat_age_s,
        )
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="runtime_session",
        )
        _emit(payload, as_json=args.as_json)
        return 0 if payload.get("status") == "ok" else 3

    if args.command == "runtime" and args.runtime_command == "result-check":
        try:
            if args.all:
                payload = _runtime_result_check_all_payload(
                    run_dir=Path(args.run_dir) if args.run_dir is not None else None,
                    max_jitter_p99_ms=args.max_jitter_p99_ms,
                    max_tracking_error_rad=args.max_tracking_error_rad,
                    required_owners=args.require_owner,
                )
            else:
                payload = _runtime_result_check_payload(
                    result_artifact_path=_runtime_result_artifact_path_from_args(
                        result_artifact=args.result_artifact,
                        run_dir=args.run_dir,
                    ),
                    expect_owner=args.expect_owner,
                    expect_mode=args.expect_mode,
                    expect_sample_count=args.expect_sample_count,
                    max_jitter_p99_ms=args.max_jitter_p99_ms,
                    max_tracking_error_rad=args.max_tracking_error_rad,
                )
        except (OSError, ValueError) as error:
            payload = {
                "status": "fail",
                "schema": "armctrl.runtime_result_check.v1",
                "reason": str(error),
                "result_artifact": args.result_artifact,
                "run_dir": args.run_dir,
                "next_gate": "wait for live runtime command result artifact",
            }
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="runtime_result_check",
        )
        _emit(payload, as_json=args.as_json)
        return 0 if payload.get("status") == "pass" else 3

    if args.command == "motion" and args.motion_command == "result":
        try:
            if args.all:
                payload = _runtime_result_check_all_payload(
                    run_dir=Path(args.run_dir) if args.run_dir is not None else None,
                    max_jitter_p99_ms=args.max_jitter_p99_ms,
                    max_tracking_error_rad=args.max_tracking_error_rad,
                    required_owners=args.require_owner,
                )
            else:
                payload = _runtime_result_check_payload(
                    result_artifact_path=_runtime_result_artifact_path_from_args(
                        result_artifact=args.result_artifact,
                        run_dir=args.run_dir,
                    ),
                    expect_owner=args.expect_owner,
                    expect_mode=args.expect_mode,
                    expect_sample_count=args.expect_sample_count,
                    max_jitter_p99_ms=args.max_jitter_p99_ms,
                    max_tracking_error_rad=args.max_tracking_error_rad,
                )
        except (OSError, ValueError) as error:
            payload = {
                "status": "fail",
                "schema": "armctrl.motion_result_check.v1",
                "reason": str(error),
                "result_artifact": args.result_artifact,
                "run_dir": args.run_dir,
                "next_gate": "wait for live runtime command result artifact",
            }
        payload = dict(payload)
        payload["command_surface"] = "armctrl.motion.result.v1"
        payload["legacy_equivalent"] = "armctrl runtime result-check"
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="motion_result_check",
        )
        _emit(payload, as_json=args.as_json)
        return 0 if payload.get("status") == "pass" else 3

    if args.command == "runtime" and args.runtime_command == "preposition":
        try:
            runtime_status = runtime_status_from_artifact(
                session_artifact_path=Path(args.session_artifact),
                max_heartbeat_age_s=args.max_heartbeat_age_s,
            )
            q_start = _runtime_float_list(runtime_status.get("q_hold"), name="q_hold")
            q_target = [float(value) for value in args.q_target]
            q_points = _linear_q_points(
                q_start=q_start,
                q_target=q_target,
                max_joint_step_rad=args.max_joint_step_rad,
            )
            heartbeat_timeout_s = (
                float(args.heartbeat_timeout_s)
                if args.heartbeat_timeout_s is not None
                else max(0.5, (len(q_points) - 1) / float(args.send_hz) + 1.0)
            )
            queued = submit_trajectory_command(
                session_artifact_path=Path(args.session_artifact),
                owner="runtime_preposition",
                expected_q_start=q_start,
                q_points=q_points,
                send_hz=args.send_hz,
                max_start_error_rad=args.max_start_error_rad,
                heartbeat_timeout_s=heartbeat_timeout_s,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
                output_path=None,
                start_pose_policy="live_hold",
            )
        except (RuntimeSessionError, RuntimeError, ValueError) as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.runtime_preposition.v1",
                "reason": str(error),
                "movement_command_sent": False,
                "start_pose_policy": "live_hold",
                "fault_landing_mode": "damping",
                "next_gate": "restore live runtime hold_safe before preposition",
            }
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="runtime_preposition",
            )
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {
            "status": "queued",
            "schema": "armctrl.runtime_preposition.v1",
            "movement_command_sent": False,
            "owner": "runtime_preposition",
            "mode": "trajectory_replay",
            "q_start": q_start,
            "q_target": q_target,
            "sample_count": len(q_points),
            "send_hz": float(args.send_hz),
            "start_pose_policy": "live_hold",
            "start_pose_guard": queued.get("start_pose_guard"),
            "runtime": queued.get("runtime"),
            "runtime_command": {
                "command_id": queued["command_id"],
                "status": "queued",
                "artifacts": queued["artifacts"],
            },
            "artifacts": queued["artifacts"],
            "fault_landing_mode": "damping",
            "next_gate": "wait for live runtime preposition result artifact",
        }
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="runtime_preposition",
        )
        _emit(payload, as_json=args.as_json)
        return 0

    if args.command == "runtime" and args.runtime_command == "submit-trajectory":
        try:
            payload = submit_trajectory_command(
                session_artifact_path=Path(args.session_artifact),
                owner=args.owner,
                expected_q_start=tuple(args.expected_q_start),
                q_points=tuple(tuple(point) for point in args.q_point),
                send_hz=args.send_hz,
                max_start_error_rad=args.max_start_error_rad,
                heartbeat_timeout_s=args.heartbeat_timeout_s,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
                output_path=Path(args.output) if args.output else None,
                max_tracking_error_rad=args.max_tracking_error_rad,
                max_tau_abs=args.max_tau_abs,
                max_joint_segment_delta_rad=args.max_joint_segment_delta_rad,
                max_joint_velocity_rad_s=_agent_default_joint_velocity_limit(
                    owner=args.owner,
                    requested_limit=args.max_joint_velocity_rad_s,
                ),
                max_joint_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
            )
        except (RuntimeSessionError, ValueError) as error:
            error_payload = error.payload if isinstance(error, RuntimeSessionError) else {}
            payload = {
                **error_payload,
                "status": "rejected",
                "schema": "armctrl.arm_runtime_submit.v1",
                "reason": str(error),
                "movement_command_sent": False,
            }
            _emit(payload, as_json=args.as_json)
            return 3
        _emit(payload, as_json=args.as_json)
        return 0

    if args.command == "runtime" and args.runtime_command == "submit-intent":
        try:
            payload = submit_intent_command(
                session_artifact_path=Path(args.session_artifact),
                owner=args.owner,
                expected_q_start=tuple(args.expected_q_start),
                q_target=tuple(args.q_target),
                control_period_s=args.control_period_s,
                send_hz=args.send_hz,
                max_joint_delta_rad=args.max_joint_delta_rad,
                max_joint_velocity_rad_s=args.max_joint_velocity_rad_s,
                max_joint_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
                max_start_error_rad=args.max_start_error_rad,
                heartbeat_timeout_s=args.heartbeat_timeout_s,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
                output_path=Path(args.output) if args.output else None,
                max_tracking_error_rad=args.max_tracking_error_rad,
                max_tau_abs=args.max_tau_abs,
            )
        except (RuntimeSessionError, ValueError) as error:
            error_payload = error.payload if isinstance(error, RuntimeSessionError) else {}
            payload = {
                **error_payload,
                "status": "rejected",
                "schema": "armctrl.arm_runtime_submit.v1",
                "reason": str(error),
                "movement_command_sent": False,
            }
            _emit(payload, as_json=args.as_json)
            return 3
        _emit(payload, as_json=args.as_json)
        return 0

    if args.command == "runtime" and args.runtime_command == "submit-eef":
        try:
            payload = submit_eef_command(
                session_artifact_path=Path(args.session_artifact),
                owner=args.owner,
                backend=args.backend,
                kind=args.kind,
                frame=args.frame,
                expected_q_start=tuple(args.expected_q_start),
                control_period_s=args.control_period_s,
                send_hz=args.send_hz,
                start_pose_policy=args.start_pose_policy,
                max_start_error_rad=args.max_start_error_rad,
                heartbeat_timeout_s=args.heartbeat_timeout_s,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
                delta_position_m=(
                    tuple(args.delta_position)
                    if args.delta_position is not None
                    else None
                ),
                delta_rpy_rad=(
                    tuple(args.delta_rpy) if args.delta_rpy is not None else None
                ),
                linear_mps=tuple(args.linear) if args.linear is not None else None,
                angular_rps=(
                    tuple(args.angular) if args.angular is not None else None
                ),
                max_linear_step_m=args.max_linear_step_m,
                max_angular_step_rad=args.max_angular_step_rad,
                output_path=Path(args.output) if args.output else None,
            )
        except RuntimeSessionError as error:
            payload = {
                **error.payload,
                "status": "rejected",
                "schema": "armctrl.arm_runtime_submit.v1",
                "command_space": "eef",
                "reason": str(error),
                "movement_command_sent": False,
            }
            _emit(payload, as_json=args.as_json)
            return 3
        except ValueError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.arm_runtime_submit.v1",
                "command_space": "eef",
                "owner": args.owner,
                "mode": MotionMode.AGENT_SERVO.value,
                "backend": args.backend,
                "reason": str(error),
                "movement_command_sent": False,
            }
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        _emit(payload, as_json=args.as_json)
        return _motion_submit_exit_code(payload)

    if args.command == "motion" and args.motion_command == "submit":
        return _handle_motion_submit(args)

    if args.command == "motion" and args.motion_command == "compile":
        return _handle_motion_compile(args)

    if args.command == "profile" and args.profile_command == "list":
        payload = {
            "status": "ok",
            "schema": "armctrl.motion_profile_catalog.v1",
            "profiles": _motion_profile_catalog(include_description=True),
        }
        return _emit(payload, as_json=args.as_json)

    if args.command == "profile" and args.profile_command == "show":
        profile = MOTION_PROFILES.get(args.name)
        if profile is None:
            payload = {
                "status": "rejected",
                "schema": "armctrl.motion_profile.v1",
                "reason": f"unknown profile: {args.name}",
                "available_profiles": sorted(MOTION_PROFILES),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {"status": "ok", **profile}
        return _emit(payload, as_json=args.as_json)

    if args.command == "console" and args.console_command == "status":
        try:
            runtime_status = runtime_status_from_artifact(
                session_artifact_path=Path(args.session_artifact),
                max_heartbeat_age_s=args.max_heartbeat_age_s,
            )
            payload = {
                "status": "ok",
                "schema": "armctrl.runtime_console_status.v1",
                "runtime": runtime_status,
                "profiles": _motion_profile_catalog(include_description=False),
                "motion_surface": _motion_surface_catalog(),
            }
        except RuntimeSessionError as error:
            payload = {
                **error.payload,
                "status": "blocked",
                "schema": "armctrl.runtime_console_status.v1",
                "reason": str(error),
                "next_gate": "start or recover live runtime before using console",
            }
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="runtime_console_status",
        )
        _emit(payload, as_json=args.as_json)
        return 0 if payload.get("status") == "ok" else 3

    if args.command == "console" and args.console_command == "catalog":
        payload = {
            "status": "ok",
            "schema": "armctrl.runtime_console_catalog.v1",
            "profiles": _motion_profile_catalog(include_description=True),
            "motion_surface": _motion_surface_catalog(),
            "operator_surfaces": {
                "runtime_status": "armctrl console status --session-artifact <runtime_session.json>",
                "submit_motion": "armctrl motion submit <kind> ...",
                "check_result": "armctrl motion result --run-dir <run_dir>",
                "show_profile": "armctrl profile show <name>",
            },
            "legacy_policy": {
                "formal_control_surface": "motion/profile/console",
                "sysid_run_sdk": (
                    "parser-level removed; sysid run only accepts --adapter {fake}; "
                    "use sysid compile-runtime plus motion submit joint-trajectory"
                ),
                "runtime_submit_legacy": "compatibility/diagnostic surface; prefer armctrl motion submit",
            },
            "command_classes": _command_surface_classes(),
        }
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="runtime_console_catalog",
        )
        return _emit(payload, as_json=args.as_json)

    if args.command == "recipe" and args.recipe_command == "list":
        payload = {
            "status": "ok",
            "schema": "armctrl.recipe_catalog.v1",
            "recipes": [recipe.to_json() for recipe in catalog.list_recipes()],
        }
        return _emit(payload, as_json=args.as_json)

    if args.command == "recipe" and args.recipe_command == "plan":
        recipe = catalog.get(args.name)
        safety = SafetyGate().evaluate(recipe, plan_only=True)
        payload: dict[str, object] = {
            "status": "ok",
            "schema": "armctrl.recipe_plan.v1",
            "plan_only": True,
            "recipe": recipe.to_json(),
            "safety": safety.to_json(),
            "steps": [step.to_json() for step in recipe.steps],
        }
        if args.output:
            start_joints = tuple(args.start_joints or DEFAULT_RECIPE_START)
            if len(start_joints) != 6:
                parser.error("--start-joints must provide 6 values")
            render_path = None
            if args.render is not None:
                render_candidate = Path(args.render)
                render_path = (
                    Path(args.output) / render_candidate
                    if not render_candidate.is_absolute()
                    else render_candidate
                )
            runtime = RecipePlanner(catalog).write_plan(
                RecipePlanRequest(
                    recipe_name=args.name,
                    start_joints=start_joints,
                    sample_hz=args.sample_hz,
                    duration_s=args.duration,
                    urdf_path=args.urdf_path,
                    safe_config_path=args.safe_config,
                    output_dir=Path(args.output),
                    render_path=render_path,
                )
            )
            payload["artifacts"] = runtime["artifacts"]
            payload["agent_runtime_profile"] = runtime["agent_runtime_profile"]
            payload["artifact_safety"] = runtime["artifact_safety"]
            payload["next_steps"] = [
                f"uv run armctrl recipe export-eef-seed --plan-dir {args.output} --json",
                f"uv run armctrl recipe execute {args.name} --backend sim --output {args.output} --json",
            ]
        return _emit(payload, as_json=args.as_json)

    if args.command == "recipe" and args.recipe_command == "export-eef-seed":
        try:
            result = RecipeEefSeedExporter().export(
                RecipeEefSeedRequest(plan_dir=Path(args.plan_dir))
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.recipe_eef_seed.v1",
                "error": {
                    "code": "missing_recipe_plan_artifacts",
                    "message": str(error),
                },
                "movement_allowed": False,
                "plan_dir": str(Path(args.plan_dir)),
                "next_gate": "run armctrl recipe plan <name> --output <dir> --json before exporting an EEF seed",
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if (
        args.command == "recipe"
        and args.recipe_command == "export-agent-preset-contract"
    ):
        try:
            result = RecipeAgentPresetContractExporter().export(
                RecipeAgentPresetContractRequest(plan_dir=Path(args.plan_dir))
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.recipe_agent_preset_contract.v1",
                "error": {
                    "code": "missing_recipe_plan_artifacts",
                    "message": str(error),
                },
                "movement_allowed": False,
                "plan_dir": str(Path(args.plan_dir)),
                "next_gate": "run armctrl recipe plan <name> --output <dir> --json before exporting an Agent preset contract",
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "recipe" and args.recipe_command == "runtime-smoke-fake":
        if args.runtime_session_artifact:
            payload = {
                "status": "rejected",
                "schema": "armctrl.recipe_runtime_smoke.v1",
                "movement_allowed": False,
                "hardware_motion": False,
                "movement_command_sent": False,
                "reason": (
                    "recipe runtime-smoke-fake is pure fake; use recipe runtime-submit "
                    "to enqueue a live runtime owner command"
                ),
                "plan_dir": str(Path(args.plan_dir)),
                "runtime": {
                    "single_owner_runtime_session": False,
                    "session_artifact": str(Path(args.runtime_session_artifact)),
                },
                "next_gate": "run armctrl recipe runtime-submit --runtime-session-artifact",
            }
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        try:
            result = RecipeRuntimeSmoker().run(
                RecipeRuntimeSmokeRequest(
                    plan_dir=Path(args.plan_dir),
                    runtime_session_artifact_path=(
                        Path(args.runtime_session_artifact)
                        if args.runtime_session_artifact
                        else None
                    ),
                )
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.recipe_runtime_smoke.v1",
                "movement_allowed": False,
                "hardware_motion": False,
                "reason": str(error),
                "plan_dir": str(Path(args.plan_dir)),
                "next_gate": "run armctrl recipe plan <name> --output <dir> --json before runtime smoke",
            }
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        except RuntimeSessionError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.recipe_runtime_smoke.v1",
                "movement_allowed": False,
                "hardware_motion": False,
                "reason": str(error),
                "plan_dir": str(Path(args.plan_dir)),
                "runtime": {
                    "single_owner_runtime_session": True,
                    "runtime_session_id": error.payload.get("runtime_session_id"),
                    "mode": error.payload.get("mode"),
                    "owner": error.payload.get("owner"),
                    "readiness": error.payload.get("readiness"),
                },
                "next_gate": "release active runtime owner or recover runtime to hold_safe before recipe smoke",
            }
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        except RuntimeError as error:
            if str(error) == (
                "recipe runtime-smoke-fake is pure fake; use recipe runtime-submit "
                "to enqueue a live runtime owner command"
            ):
                payload = {
                    "status": "rejected",
                    "schema": "armctrl.recipe_runtime_smoke.v1",
                    "movement_allowed": False,
                    "hardware_motion": False,
                    "movement_command_sent": False,
                    "reason": str(error),
                    "plan_dir": str(Path(args.plan_dir)),
                    "next_gate": "run armctrl recipe runtime-submit --runtime-session-artifact",
                }
                payload = _attach_output_artifact(payload, args.output)
                _emit(payload, as_json=args.as_json)
                return 3
            if str(error) != "recipe plan safety gate is not passed":
                raise
            payload = {
                "status": "rejected",
                "schema": "armctrl.recipe_runtime_smoke.v1",
                "movement_allowed": False,
                "hardware_motion": False,
                "reason": str(error),
                "plan_dir": str(Path(args.plan_dir)),
                "next_gate": "repair recipe plan safety checks before runtime smoke",
            }
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {"status": "ok", **result}
        payload = _attach_output_artifact(payload, args.output)
        return _emit(payload, as_json=args.as_json)

    if args.command == "recipe" and args.recipe_command == "runtime-submit":
        plan_dir = Path(args.plan_dir)
        manifest_path = plan_dir / "manifest.json"
        trajectory_path = plan_dir / "planned_trajectory.csv"
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            safety = manifest.get("safety")
            if not isinstance(safety, dict) or safety.get("allowed") is not True:
                raise RuntimeError("recipe plan safety gate is not passed")
            request = manifest.get("request")
            if not isinstance(request, dict):
                raise ValueError("recipe manifest is missing request")
            start_joints = request.get("start_joints")
            if not isinstance(start_joints, list):
                raise ValueError("recipe manifest is missing request.start_joints")
            sample_hz = float(request.get("sample_hz"))
            q_points = _read_sysid_execution_q_points(
                trajectory_path,
                dof=len(start_joints),
            )
            queued = submit_trajectory_command(
                session_artifact_path=Path(args.runtime_session_artifact),
                owner="recipe",
                expected_q_start=q_points[0],
                q_points=q_points,
                send_hz=sample_hz,
                max_start_error_rad=args.max_start_error_rad,
                heartbeat_timeout_s=(
                    float(args.heartbeat_timeout_s)
                    if args.heartbeat_timeout_s is not None
                    else max(1.0, len(q_points) / sample_hz + 1.0)
                ),
                max_heartbeat_age_s=args.max_heartbeat_age_s,
                start_pose_policy=args.start_pose_policy,
                max_tracking_error_rad=args.max_tracking_error_rad,
                max_tau_abs=args.max_tau_abs,
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.recipe_runtime_submit.v1",
                "movement_allowed": False,
                "hardware_motion": False,
                "movement_command_sent": False,
                "reason": str(error),
                "plan_dir": str(plan_dir),
                "next_gate": "run armctrl recipe plan <name> --output <dir> --json before runtime submit",
            }
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        except (RuntimeSessionError, RuntimeError, ValueError) as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.recipe_runtime_submit.v1",
                "movement_allowed": False,
                "hardware_motion": False,
                "movement_command_sent": False,
                "reason": str(error),
                "plan_dir": str(plan_dir),
                "start_pose_policy": args.start_pose_policy,
                "start_pose_guard": _runtime_session_error_payload(error).get(
                    "start_pose_guard"
                ),
                "runtime": {
                    "single_owner_runtime_session": True,
                    "runtime_session_artifact": str(args.runtime_session_artifact),
                    "owner": "recipe",
                    "mode": "trajectory_replay",
                },
                "next_gate": "complete live runtime hold_safe readiness before Recipe runtime submit",
            }
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {
            "status": "queued",
            "schema": "armctrl.recipe_runtime_submit.v1",
            "movement_allowed": True,
            "hardware_motion": True,
            "movement_command_sent": False,
            "recipe_plan_dir": str(plan_dir),
            "recipe": manifest["recipe"],
            "safety": safety,
            "start_pose_policy": args.start_pose_policy,
            "start_pose_guard": queued.get("start_pose_guard"),
            "runtime": {
                "single_owner_runtime_session": True,
                "runtime_session_artifact": str(args.runtime_session_artifact),
                "owner": "recipe",
                "mode": "trajectory_replay",
                "queue": queued["runtime"]["queue"],
            },
            "runtime_command": {
                "command_id": queued["command_id"],
                "status": "queued",
                "artifacts": queued["artifacts"],
                "sample_count": len(q_points),
                "send_hz": sample_hz,
            },
            "fault_landing_mode": "damping",
            "next_gate": "wait for live runtime command result artifact",
        }
        payload = _attach_output_artifact(payload, args.output)
        _emit(payload, as_json=args.as_json)
        return 0

    if args.command == "recipe" and args.recipe_command == "execute":
        recipe = catalog.get(args.name)
        plan_only = args.backend == "sim"
        safety = SafetyGate().evaluate(recipe, plan_only=plan_only)
        executor = RecipeExecutor(hardware_backend=args.backend).evaluate(safety)
        payload = {
            "status": "rejected",
            "schema": "armctrl.recipe_execution.v1",
            "recipe": recipe.to_json(),
            "safety": safety.to_json(),
            "executor": executor.to_json(),
            "steps": [step.to_json() for step in recipe.steps],
        }
        if args.backend == "sim":
            start_joints = tuple(args.start_joints or DEFAULT_RECIPE_START)
            if len(start_joints) != 6:
                parser.error("--start-joints must provide 6 values")
            output_dir = Path(args.output) if args.output else Path("runs/recipe-sim-preview")
            render_path = None
            if args.render is not None:
                render_candidate = Path(args.render)
                render_path = (
                    output_dir / render_candidate
                    if not render_candidate.is_absolute()
                    else render_candidate
                )
            runtime = RecipePlanner(catalog).write_plan(
                RecipePlanRequest(
                    recipe_name=args.name,
                    start_joints=start_joints,
                    sample_hz=args.sample_hz,
                    duration_s=args.duration,
                    urdf_path=args.urdf_path,
                    safe_config_path=args.safe_config,
                    output_dir=output_dir,
                    render_path=render_path,
                )
            )
            payload["simulation_gate"] = runtime["artifact_safety"]
            payload["artifacts"] = runtime["artifacts"]
            payload["handoff"] = runtime["handoff"]
            payload["agent_runtime_profile"] = runtime["agent_runtime_profile"]
            payload["next_steps"] = [
                runtime["handoff"]["suggested_cli"]["export_eef_seed"],
                "uv run armctrl recipe status --json",
                runtime["handoff"]["suggested_cli"][
                    "eef_synthesize_preview_from_recipe"
                ],
            ]
            if executor.status == "ready" and runtime["artifact_safety"]["allowed"] is True:
                payload["status"] = "ok"
                return _emit(payload, as_json=args.as_json)
        _emit(payload, as_json=args.as_json)
        return 3

    if args.command == "recipe" and args.recipe_command == "status":
        executor = RecipeExecutor().status()
        payload = {
            "status": "ok",
            "schema": "armctrl.recipe_executor_status.v1",
            "executor": executor.to_json(),
        }
        return _emit(payload, as_json=args.as_json)

    if args.command == "recipe" and args.recipe_command == "cancel":
        executor = RecipeExecutor().cancel()
        payload = {
            "status": "ok",
            "schema": "armctrl.recipe_executor_cancel.v1",
            "executor": executor.to_json(),
        }
        return _emit(payload, as_json=args.as_json)

    if args.command == "lerobot" and args.lerobot_command == "doctor":
        result = LeRobotDoctor(
            model=args.model,
            robot_interface=args.robot_interface,
            teleop_interface=args.teleop_interface,
        )
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "lerobot" and args.lerobot_command == "config-plan":
        if args.mode in {"record", "train"} and not args.dataset_repo_id:
            parser.error("--dataset-repo-id is required for record/train")
        if args.mode == "record" and not args.task:
            parser.error("--task is required for record")
        if args.mode == "rollout" and not args.policy_path:
            parser.error("--policy-path is required for rollout")
        result = LeRobotConfigPlanner().plan(
            LeRobotConfigPlanRequest(
                mode=args.mode,
                model=args.model,
                robot_interface=args.robot_interface,
                teleop_interface=args.teleop_interface,
                dataset_repo_id=args.dataset_repo_id,
                task=args.task,
                episodes=args.episodes,
                policy=args.policy,
                output_dir=args.output_dir,
                job_name=args.job_name,
                policy_path=args.policy_path,
                eef_plan_dir=Path(args.eef_plan_dir) if args.eef_plan_dir else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "lerobot" and args.lerobot_command == "export-metadata":
        result = LeRobotMetadataExporter().run(
            LeRobotMetadataExport(
                dataset_repo_id=args.dataset_repo_id,
                parameter_bundle=args.parameter_bundle,
                safe_config=args.safe_config,
                output=Path(args.output),
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "agent-flow" and args.agent_flow_command == "plan":
        if args.eef_mode == "pose_absolute" and (
            args.position is None or args.rpy is None
        ):
            parser.error(
                "--position and --rpy are required for --eef-mode pose_absolute"
            )
        if args.eef_mode == "pose_delta" and (
            args.delta_position is None or args.delta_rpy is None
        ):
            parser.error(
                "--delta-position and --delta-rpy are required for --eef-mode pose_delta"
            )
        if args.eef_mode == "twist" and (
            args.linear is None or args.angular is None
        ):
            parser.error("--linear and --angular are required for --eef-mode twist")
        result = AgentFlowPlanner().plan(
            AgentFlowPlanRequest(
                preset=args.preset,
                eef_mode=args.eef_mode,
                backend=args.backend,
                output_dir=Path(args.output),
                safe_config_path=args.safe_config,
                urdf_path=args.urdf_path,
                frame=args.frame,
                position_m=tuple(args.position) if args.position else None,
                rpy_rad=tuple(args.rpy) if args.rpy else None,
                delta_position_m=(
                    tuple(args.delta_position) if args.delta_position else None
                ),
                delta_rpy_rad=tuple(args.delta_rpy) if args.delta_rpy else None,
                linear_mps=tuple(args.linear) if args.linear else None,
                angular_rps=tuple(args.angular) if args.angular else None,
                control_period_s=args.control_period_s,
                model=args.model,
                interface=args.interface,
                policy_path=args.policy_path,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "agent-flow" and args.agent_flow_command == "doctor":
        payload = {"status": "ok", **AgentFlowDoctor().run()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "agent-flow" and args.agent_flow_command == "review":
        payload = {
            "status": "ok",
            **AgentFlowReviewer().review(Path(args.contract)),
        }
        return _emit(payload, as_json=args.as_json)

    if args.command == "agent-flow" and args.agent_flow_command == "runtime-smoke-fake":
        if len(args.q_start) != len(args.q_target):
            parser.error("--q-start and --q-target must have the same length")
        request = AgentFlowRuntimeSmokeRequest(
            contract_path=Path(args.contract),
            q_start=tuple(args.q_start),
            q_target=tuple(args.q_target),
            send_hz=args.send_hz,
            max_joint_delta_rad=args.max_joint_delta_rad,
            runtime_session_artifact_path=(
                Path(args.runtime_session_artifact)
                if args.runtime_session_artifact is not None
                else None
            ),
        )
        try:
            payload = {"status": "ok", **AgentFlowRuntimeSmoker().run(request)}
        except ValueError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.agent_flow_runtime_smoke.v1",
                "movement_allowed": False,
                "hardware_motion": False,
                "producer": "agent",
                "reason": str(error),
                "contract_path": str(args.contract),
                "intent": {
                    "q_start": list(args.q_start),
                    "q_target": list(args.q_target),
                    "backend_send_hz": args.send_hz,
                    "max_joint_delta_rad": args.max_joint_delta_rad,
                },
                "next_gate": "reduce the checked Agent joint intent delta before runtime smoke",
            }
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        run_status = payload.get("run_status")
        if run_status is not None and run_status != "completed":
            payload["status"] = str(run_status)
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        payload = _attach_output_artifact(payload, args.output)
        return _emit(payload, as_json=args.as_json)

    if args.command == "agent-flow" and args.agent_flow_command == "runtime-smoke-real":
        if len(args.q_start) != len(args.q_target):
            parser.error("--q-start and --q-target must have the same length")
        if args.confirm != AGENT_FLOW_REAL_RUNTIME_CONFIRMATION:
            payload = {
                "status": "rejected",
                "schema": "armctrl.agent_flow_runtime_smoke_real.v1",
                "movement_allowed": False,
                "hardware_motion": False,
                "movement_command_sent": False,
                "producer": "agent",
                "backend": "arx5_sdk",
                "reason": "agent real runtime smoke requires explicit operator confirmation",
                "requires_confirm": args.confirm,
                "contract_path": str(args.contract),
                "readiness_artifact_path": str(args.readiness_artifact),
                "fault_landing_mode": "damping",
                "next_gate": (
                    "provide the exact Agent real runtime confirmation string after "
                    "all prerequisite gates pass"
                ),
            }
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        if args.runtime_session_artifact is None:
            payload = {
                "status": "rejected",
                "schema": "armctrl.agent_flow_runtime_smoke_real.v1",
                "movement_allowed": False,
                "hardware_motion": False,
                "movement_command_sent": False,
                "producer": "agent",
                "backend": "arx5_sdk",
                "reason": (
                    "real Agent runtime smoke requires --runtime-session-artifact "
                    "from armctrl runtime start --serve"
                ),
                "contract_path": str(args.contract),
                "readiness_artifact_path": str(args.readiness_artifact),
                "runtime": {
                    "single_owner_runtime_session": True,
                    "runtime_session_artifact": None,
                    "owner": None,
                },
                "fault_landing_mode": "damping",
                "next_gate": (
                    "start live arm runtime, recover/hold SAFE_CENTER, then attach "
                    "Agent owner lease"
                ),
            }
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        try:
            contract = _read_agent_flow_contract_for_runtime(Path(args.contract))
            readiness_artifact = json.loads(
                Path(args.readiness_artifact).read_text(encoding="utf-8")
            )
            if readiness_artifact.get("schema") != "armctrl.arm_runtime_status.v1":
                raise RuntimeError(
                    "real Agent runtime smoke requires live arm runtime status readiness"
                )
            runtime_status = _runtime_first_sysid_readiness_artifact(
                readiness_artifact,
                runtime_session_artifact_path=Path(args.runtime_session_artifact),
                max_heartbeat_age_s=args.max_heartbeat_age_s,
            )
            readiness = _read_agent_sysid_readiness_from_payload(runtime_status)
            start_pose = _runtime_live_hold_start_pose(
                runtime_status,
                dof=len(args.q_start),
                max_start_error_rad=0.02,
            )
            control_period_s = _agent_flow_contract_control_period_s(contract)
            q_start = tuple(start_pose["q_start"])
            requested_q_start = tuple(args.q_start)
            requested_q_target = tuple(args.q_target)
            q_delta = tuple(
                target - start
                for target, start in zip(
                    requested_q_target,
                    requested_q_start,
                    strict=True,
                )
            )
            q_target = tuple(
                start + delta for start, delta in zip(q_start, q_delta, strict=True)
            )
            queued = submit_intent_command(
                session_artifact_path=Path(args.runtime_session_artifact),
                owner="agent",
                expected_q_start=q_start,
                q_target=q_target,
                control_period_s=control_period_s,
                send_hz=args.send_hz,
                max_joint_delta_rad=args.max_joint_delta_rad,
                max_joint_velocity_rad_s=_agent_default_joint_velocity_limit(
                    owner="agent",
                    requested_limit=args.max_joint_velocity_rad_s,
                ),
                max_joint_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
                max_start_error_rad=0.02,
                heartbeat_timeout_s=0.5,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
                max_tracking_error_rad=args.max_tracking_error_rad,
                max_tau_abs=args.max_tau_abs,
            )
        except (RuntimeSessionError, RuntimeError, ValueError) as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.agent_flow_runtime_smoke_real.v1",
                "movement_allowed": False,
                "hardware_motion": False,
                "movement_command_sent": False,
                "producer": "agent",
                "backend": "arx5_sdk",
                "reason": str(error),
                "contract_path": str(args.contract),
                "readiness_artifact_path": str(args.readiness_artifact),
                "runtime": {
                    "single_owner_runtime_session": True,
                    "runtime_session_artifact": str(args.runtime_session_artifact),
                    "owner": "agent",
                    "mode": "agent_servo",
                },
                "fault_landing_mode": "damping",
                "next_gate": (
                    "refresh readiness with armctrl runtime status before Agent "
                    "runtime queue submit"
                ),
            }
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {
            "status": "queued",
            "schema": "armctrl.agent_flow_runtime_smoke_real.v1",
            "movement_allowed": True,
            "hardware_motion": True,
            "movement_command_sent": False,
            "producer": "agent",
            "backend": "arx5_sdk",
            "contract_path": str(args.contract),
            "readiness_artifact_path": str(args.readiness_artifact),
            "requires_confirm": AGENT_FLOW_REAL_RUNTIME_CONFIRMATION,
            "runtime": {
                "single_owner_runtime_session": True,
                "runtime_session_artifact": str(args.runtime_session_artifact),
                "owner": "agent",
                "mode": "agent_servo",
                "queue": queued["runtime"]["queue"],
            },
            "readiness": {
                "agent_sysid_smoke_allowed": (
                    runtime_status.get("readiness", {}).get(
                        "agent_sysid_smoke_allowed"
                    )
                    if isinstance(runtime_status.get("readiness"), dict)
                    else readiness.get("agent_sysid_smoke_allowed")
                ),
                "prerequisites": readiness.get("prerequisites"),
                "tiny_motion": readiness.get("tiny_motion"),
            },
            "runtime_start_pose": start_pose,
            "intent": {
                "q_start": list(q_start),
                "q_target": list(q_target),
                "requested_q_start": list(args.q_start),
                "requested_q_target": list(args.q_target),
                "requested_delta_rad": list(q_delta),
                "control_period_s": control_period_s,
                "agent_intent_hz": 1.0 / control_period_s,
                "backend_send_hz": args.send_hz,
                "max_joint_delta_rad": args.max_joint_delta_rad,
            },
            "runtime_command": {
                "command_id": queued["command_id"],
                "status": "queued",
                "artifacts": queued["artifacts"],
            },
            "fault_landing_mode": "damping",
            "next_gate": "wait for live runtime command result artifact",
        }
        payload = _attach_output_artifact(payload, args.output)
        _emit(payload, as_json=args.as_json)
        return 0
    if args.command == "lerobot" and args.lerobot_command == "review-rollout":
        result = LeRobotRolloutReviewer().review(
            LeRobotRolloutReviewRequest(
                eef_plan_dir=Path(args.eef_plan_dir),
                trajectory_path=Path(args.trajectory) if args.trajectory else None,
                urdf_path=Path(args.urdf_path),
                safe_config_path=Path(args.safe_config),
                render_path=Path(args.render) if args.render else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "lerobot" and args.lerobot_command == "stage-rollout-trajectory":
        result = LeRobotRolloutStager().stage(
            LeRobotRolloutStageRequest(
                eef_plan_dir=Path(args.eef_plan_dir),
                trajectory_path=Path(args.trajectory),
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "lerobot" and args.lerobot_command == "preview-rollout":
        start_joints = tuple(args.start_joints) if args.start_joints else None
        if start_joints is not None and len(start_joints) != 6:
            parser.error("--start-joints must provide 6 values")
        try:
            result = LeRobotRolloutPreviewer().preview(
                LeRobotRolloutPreviewRequest(
                    eef_plan_dir=Path(args.eef_plan_dir),
                    model=args.model,
                    robot_interface=args.robot_interface,
                    urdf_path=Path(args.urdf_path),
                    safe_config_path=Path(args.safe_config),
                    recipe_plan_dir=Path(args.recipe_plan_dir) if args.recipe_plan_dir else None,
                    start_joints=start_joints,
                    sample_hz=args.sample_hz,
                    duration_s=args.duration,
                    render_path=Path(args.render) if args.render else None,
                )
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.lerobot_rollout_preview.v1",
                "error": {
                    "code": (
                        "missing_recipe_plan_artifacts"
                        if args.recipe_plan_dir
                        else "missing_eef_plan_artifacts"
                    ),
                    "message": str(error),
                },
                "movement_allowed": False,
                "eef_plan_dir": str(Path(args.eef_plan_dir)),
                "recipe_plan_dir": str(Path(args.recipe_plan_dir)) if args.recipe_plan_dir else None,
                "next_gate": (
                    "run armctrl recipe plan <name> --output <dir> --json before using --recipe-plan-dir for lerobot preview-rollout"
                    if args.recipe_plan_dir
                    else "run armctrl eef plan-pose or plan-twist with --output <dir> before lerobot preview-rollout"
                ),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "lerobot" and args.lerobot_command == "export-processor-contract":
        try:
            result = LeRobotProcessorContractExporter().export(
                LeRobotProcessorContractRequest(
                    eef_plan_dir=Path(args.eef_plan_dir),
                    model=args.model,
                    robot_interface=args.robot_interface,
                    output_path=Path(args.output) if args.output else None,
                )
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.lerobot_eef_processor_contract.v1",
                "error": {
                    "code": "missing_eef_plan_artifacts",
                    "message": str(error),
                },
                "movement_allowed": False,
                "eef_plan_dir": str(Path(args.eef_plan_dir)),
                "next_gate": "run armctrl eef plan-pose or plan-twist with --output <dir> before exporting the LeRobot processor contract",
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "lerobot" and args.lerobot_command == "agent-runtime-helper-plan":
        contract_path = Path(args.agent_runtime_contract)
        try:
            result = LeRobotAgentRuntimeHelperPlanner().plan(
                LeRobotAgentRuntimeHelperPlanRequest(
                    agent_runtime_contract_path=contract_path,
                    output_path=Path(args.output) if args.output else None,
                )
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.lerobot_agent_runtime_helper_plan.v1",
                "error": {"code": "missing_agent_runtime_contract", "message": str(error)},
                "movement_allowed": False,
                "agent_runtime_contract_path": str(contract_path),
                "next_gate": (
                    "export a valid armctrl.eef_agent_runtime_contract.v1 artifact "
                    "before using this helper plan"
                ),
            }
            return _emit(payload, as_json=args.as_json)
        except json.JSONDecodeError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.lerobot_agent_runtime_helper_plan.v1",
                "error": {
                    "code": "invalid_agent_runtime_contract_json",
                    "message": str(error),
                },
                "movement_allowed": False,
                "agent_runtime_contract_path": str(contract_path),
                "next_gate": "export a valid JSON agent runtime contract before using this helper plan",
            }
            return _emit(payload, as_json=args.as_json)
        except ValueError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.lerobot_agent_runtime_helper_plan.v1",
                "error": {"code": "invalid_agent_runtime_contract", "message": str(error)},
                "movement_allowed": False,
                "agent_runtime_contract_path": str(contract_path),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        except RuntimeError as error:
            resolved_backend = None
            if contract_path.exists():
                try:
                    resolved_backend = json.loads(
                        contract_path.read_text(encoding="utf-8")
                    ).get("resolved_backend")
                except json.JSONDecodeError:
                    resolved_backend = None
            payload = {
                "status": "rejected",
                "schema": "armctrl.lerobot_agent_runtime_helper_plan.v1",
                "error": {"code": "unsupported_runtime_backend", "message": str(error)},
                "movement_allowed": False,
                "agent_runtime_contract_path": str(contract_path),
                "resolved_backend": resolved_backend,
                "next_gate": "use this helper plan only with lerobot_rollout agent runtime contracts",
            }
            _emit(payload, as_json=args.as_json)
            return 3
        return _emit({"status": "ok", **result}, as_json=args.as_json)

    if args.command == "lerobot" and args.lerobot_command == "processor-helper-preview":
        contract_path = Path(args.processor_contract)
        try:
            result = LeRobotProcessorHelperPreviewer().preview(
                LeRobotProcessorHelperPreviewRequest(
                    processor_contract_path=contract_path,
                    urdf_path=Path(args.urdf_path),
                    safe_config_path=Path(args.safe_config),
                    render_path=Path(args.render) if args.render else None,
                )
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.lerobot_processor_helper_preview.v1",
                "error": {"code": "missing_processor_contract", "message": str(error)},
                "movement_allowed": False,
                "processor_contract_path": str(contract_path),
                "next_gate": (
                    "export a valid armctrl.lerobot_eef_processor_contract.v1 artifact "
                    "before using this helper preview"
                ),
            }
            return _emit(payload, as_json=args.as_json)
        except json.JSONDecodeError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.lerobot_processor_helper_preview.v1",
                "error": {
                    "code": "invalid_processor_contract_json",
                    "message": str(error),
                },
                "movement_allowed": False,
                "processor_contract_path": str(contract_path),
                "next_gate": "export a valid JSON processor contract before using this helper preview",
            }
            return _emit(payload, as_json=args.as_json)
        except ValueError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.lerobot_processor_helper_preview.v1",
                "error": {"code": "invalid_processor_contract", "message": str(error)},
                "movement_allowed": False,
                "processor_contract_path": str(contract_path),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        except RuntimeError as error:
            resolved_backend = None
            if contract_path.exists():
                try:
                    resolved_backend = json.loads(
                        contract_path.read_text(encoding="utf-8")
                    ).get("runner_contract", {}).get("resolved_backend")
                except json.JSONDecodeError:
                    resolved_backend = None
            payload = {
                "status": "rejected",
                "schema": "armctrl.lerobot_processor_helper_preview.v1",
                "error": {"code": "unsupported_processor_backend", "message": str(error)},
                "movement_allowed": False,
                "processor_contract_path": str(contract_path),
                "resolved_backend": resolved_backend,
                "next_gate": "use this helper preview only with lerobot_rollout processor contracts",
            }
            _emit(payload, as_json=args.as_json)
            return 3
        return _emit({"status": "ok", **result}, as_json=args.as_json)

    if args.command == "sim" and args.sim_command == "doctor":
        payload = {"status": "ok", **SimulationDoctor().run()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sim" and args.sim_command == "preview":
        result = TrajectoryPreviewer().preview(
            trajectory_path=Path(args.trajectory),
            urdf_path=Path(args.urdf_path),
            safe_config_path=Path(args.safe_config),
            backend=args.backend,
            render_path=Path(args.render) if args.render else None,
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "doctor":
        payload = {"status": "ok", **EefDoctor().run()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "plan-twist":
        result = EefPlanner().plan_twist(
            EefTwistRequest(
                frame=args.frame,
                linear_mps=tuple(args.linear),
                angular_rps=tuple(args.angular),
                backend=args.backend,
                safe_config_path=args.safe_config,
                control_period_s=args.control_period_s,
                output_dir=Path(args.output) if args.output else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "plan-pose":
        result = EefPlanner().plan_pose(
            EefPoseRequest(
                frame=args.frame,
                position_m=tuple(args.position),
                rpy_rad=tuple(args.rpy),
                backend=args.backend,
                safe_config_path=args.safe_config,
                output_dir=Path(args.output) if args.output else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "plan-delta-pose":
        result = EefPlanner().plan_delta_pose(
            EefDeltaPoseRequest(
                frame=args.frame,
                delta_position_m=tuple(args.delta_position),
                delta_rpy_rad=tuple(args.delta_rpy),
                backend=args.backend,
                safe_config_path=args.safe_config,
                control_period_s=args.control_period_s,
                output_dir=Path(args.output) if args.output else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "review":
        result = EefReviewer().review(
            EefReviewRequest(
                plan_dir=Path(args.plan_dir),
                trajectory_path=Path(args.trajectory) if args.trajectory else None,
                urdf_path=Path(args.urdf_path),
                safe_config_path=Path(args.safe_config),
                render_path=Path(args.render) if args.render else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "stage-trajectory":
        result = EefTrajectoryStager().stage(
            EefStageTrajectoryRequest(
                plan_dir=Path(args.plan_dir),
                trajectory_path=Path(args.trajectory),
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "synthesize-preview":
        start_joints = tuple(args.start_joints) if args.start_joints else None
        if start_joints is not None and len(start_joints) != 6:
            parser.error("--start-joints must provide 6 values")
        try:
            result = EefPreviewSynthesizer().synthesize(
                EefPreviewSynthesisRequest(
                    plan_dir=Path(args.plan_dir),
                    urdf_path=Path(args.urdf_path),
                    safe_config_path=Path(args.safe_config),
                    backend=args.backend,
                    start_joints=start_joints,
                    recipe_plan_dir=Path(args.recipe_plan_dir) if args.recipe_plan_dir else None,
                    sample_hz=args.sample_hz,
                    duration_s=args.duration,
                    render_path=Path(args.render) if args.render else None,
                )
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.eef_preview_synthesis.v1",
                "error": {
                    "code": (
                        "missing_recipe_plan_artifacts"
                        if args.recipe_plan_dir
                        else "missing_eef_plan_artifacts"
                    ),
                    "message": str(error),
                },
                "movement_allowed": False,
                "plan_dir": str(Path(args.plan_dir)),
                "recipe_plan_dir": str(Path(args.recipe_plan_dir)) if args.recipe_plan_dir else None,
                "next_gate": (
                    "run armctrl recipe plan <name> --output <dir> --json before using --recipe-plan-dir for synthesize-preview"
                    if args.recipe_plan_dir
                    else "run armctrl eef plan-pose or plan-twist with --output <dir> before synthesize-preview"
                ),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "preview-runner":
        start_joints = tuple(args.start_joints) if args.start_joints else None
        if start_joints is not None and len(start_joints) != 6:
            parser.error("--start-joints must provide 6 values")
        try:
            result = EefRunnerPreviewer().preview(
                EefRunnerPreviewRequest(
                    plan_dir=Path(args.plan_dir),
                    backend=args.backend,
                    model=args.model,
                    interface=args.interface,
                    urdf_path=Path(args.urdf_path),
                    safe_config_path=Path(args.safe_config),
                    recipe_plan_dir=Path(args.recipe_plan_dir) if args.recipe_plan_dir else None,
                    start_joints=start_joints,
                    sample_hz=args.sample_hz,
                    duration_s=args.duration,
                    render_path=Path(args.render) if args.render else None,
                )
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.eef_runner_preview.v1",
                "error": {
                    "code": (
                        "missing_recipe_plan_artifacts"
                        if args.recipe_plan_dir
                        else "missing_eef_plan_artifacts"
                    ),
                    "message": str(error),
                },
                "movement_allowed": False,
                "plan_dir": str(Path(args.plan_dir)),
                "recipe_plan_dir": str(Path(args.recipe_plan_dir)) if args.recipe_plan_dir else None,
                "next_gate": (
                    "run armctrl recipe plan <name> --output <dir> --json before using --recipe-plan-dir for eef preview-runner"
                    if args.recipe_plan_dir
                    else "run armctrl eef plan-pose or plan-twist with --output <dir> before eef preview-runner"
                ),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "sample-runner":
        start_joints = tuple(args.start_joints) if args.start_joints else None
        if start_joints is not None and len(start_joints) != 6:
            parser.error("--start-joints must provide 6 values")
        try:
            result = EefSampleRunner().run(
                EefSampleRunnerRequest(
                    runner_contract_path=Path(args.runner_contract),
                    urdf_path=Path(args.urdf_path),
                    safe_config_path=Path(args.safe_config),
                    recipe_plan_dir=Path(args.recipe_plan_dir) if args.recipe_plan_dir else None,
                    start_joints=start_joints,
                    sample_hz=args.sample_hz,
                    duration_s=args.duration,
                    render_path=Path(args.render) if args.render else None,
                )
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.eef_sample_runner.v1",
                "error": {
                    "code": (
                        "missing_recipe_plan_artifacts"
                        if args.recipe_plan_dir
                        else "missing_eef_plan_artifacts"
                    ),
                    "message": str(error),
                },
                "movement_allowed": False,
                "runner_contract_path": str(Path(args.runner_contract)),
                "recipe_plan_dir": str(Path(args.recipe_plan_dir)) if args.recipe_plan_dir else None,
                "next_gate": (
                    "run armctrl recipe plan <name> --output <dir> --json before using --recipe-plan-dir for eef sample-runner"
                    if args.recipe_plan_dir
                    else "run armctrl eef export-runner-contract --plan-dir <dir> --output <path> --json before eef sample-runner"
                ),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        except ValueError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.eef_sample_runner.v1",
                "error": {
                    "code": "invalid_runner_contract",
                    "message": str(error),
                },
                "movement_allowed": False,
                "runner_contract_path": str(Path(args.runner_contract)),
                "next_gate": "export a valid armctrl.eef_runner_contract.v1 artifact before eef sample-runner",
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "export-sdk-helper-plan":
        contract_path = Path(args.runner_contract)
        try:
            result = EefSdkHelperPlanExporter().export(
                EefSdkHelperPlanRequest(
                    runner_contract_path=contract_path,
                    output_path=Path(args.output) if args.output else None,
                )
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.sdk_cartesian_helper_plan.v1",
                "error": {"code": "missing_runner_contract", "message": str(error)},
                "movement_allowed": False,
                "runner_contract_path": str(contract_path),
                "next_gate": "export a valid armctrl.eef_runner_contract.v1 artifact before using this helper plan",
            }
            return _emit(payload, as_json=args.as_json)
        except json.JSONDecodeError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.sdk_cartesian_helper_plan.v1",
                "error": {"code": "invalid_runner_contract_json", "message": str(error)},
                "movement_allowed": False,
                "runner_contract_path": str(contract_path),
                "next_gate": "export a valid JSON runner contract artifact before using this helper plan",
            }
            return _emit(payload, as_json=args.as_json)
        except ValueError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.sdk_cartesian_helper_plan.v1",
                "error": {"code": "invalid_runner_contract", "message": str(error)},
                "movement_allowed": False,
                "runner_contract_path": str(contract_path),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        except RuntimeError as error:
            resolved_backend = None
            if contract_path.exists():
                try:
                    resolved_backend = json.loads(
                        contract_path.read_text(encoding="utf-8")
                    ).get("resolved_backend")
                except json.JSONDecodeError:
                    resolved_backend = None
            payload = {
                "status": "rejected",
                "schema": "armctrl.sdk_cartesian_helper_plan.v1",
                "error": {"code": "unsupported_runner_backend", "message": str(error)},
                "movement_allowed": False,
                "runner_contract_path": str(contract_path),
                "resolved_backend": resolved_backend,
                "next_gate": "use this helper plan only with sdk_cartesian runner contracts",
            }
            _emit(payload, as_json=args.as_json)
            return 3
        return _emit({"status": "ok", **result}, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "export-moveit-helper-plan":
        contract_path = Path(args.runner_contract)
        try:
            result = EefMoveItHelperPlanExporter().export(
                EefMoveItHelperPlanRequest(
                    runner_contract_path=contract_path,
                    output_path=Path(args.output) if args.output else None,
                )
            )
        except FileNotFoundError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.moveit_servo_helper_plan.v1",
                "error": {"code": "missing_runner_contract", "message": str(error)},
                "movement_allowed": False,
                "runner_contract_path": str(contract_path),
                "next_gate": "export a valid armctrl.eef_runner_contract.v1 artifact before using this helper plan",
            }
            return _emit(payload, as_json=args.as_json)
        except json.JSONDecodeError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.moveit_servo_helper_plan.v1",
                "error": {"code": "invalid_runner_contract_json", "message": str(error)},
                "movement_allowed": False,
                "runner_contract_path": str(contract_path),
                "next_gate": "export a valid JSON runner contract artifact before using this helper plan",
            }
            return _emit(payload, as_json=args.as_json)
        except ValueError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.moveit_servo_helper_plan.v1",
                "error": {"code": "invalid_runner_contract", "message": str(error)},
                "movement_allowed": False,
                "runner_contract_path": str(contract_path),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        except RuntimeError as error:
            resolved_backend = None
            if contract_path.exists():
                try:
                    resolved_backend = json.loads(
                        contract_path.read_text(encoding="utf-8")
                    ).get("resolved_backend")
                except json.JSONDecodeError:
                    resolved_backend = None
            payload = {
                "status": "rejected",
                "schema": "armctrl.moveit_servo_helper_plan.v1",
                "error": {"code": "unsupported_runner_backend", "message": str(error)},
                "movement_allowed": False,
                "runner_contract_path": str(contract_path),
                "resolved_backend": resolved_backend,
                "next_gate": "use this helper plan only with moveit_servo runner contracts",
            }
            _emit(payload, as_json=args.as_json)
            return 3
        return _emit({"status": "ok", **result}, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "runtime-plan":
        if not args.backend and not args.plan_dir:
            parser.error("either --backend or --plan-dir is required for eef runtime-plan")
        if args.backend == "lerobot_rollout" and not args.policy_path:
            parser.error("--policy-path is required for lerobot_rollout")
        result = EefRuntimePlanner().plan(
            EefRuntimePlanRequest(
                backend=args.backend,
                model=args.model,
                interface=args.interface,
                policy_path=args.policy_path,
                plan_dir=Path(args.plan_dir) if args.plan_dir else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "export-lerobot-action":
        result = EefLeRobotExporter().export(
            EefLeRobotExportRequest(
                plan_dir=Path(args.plan_dir),
                output_path=Path(args.output) if args.output else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "export-sdk-cartesian":
        result = EefSdkCartesianExporter().export(
            EefSdkCartesianExportRequest(
                plan_dir=Path(args.plan_dir),
                model=args.model,
                interface=args.interface,
                output_path=Path(args.output) if args.output else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "export-moveit-servo":
        result = EefMoveItServoExporter().export(
            EefMoveItServoExportRequest(
                plan_dir=Path(args.plan_dir),
                output_path=Path(args.output) if args.output else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "export-runtime-bridge":
        result = EefRuntimeBridgeExporter().export(
            EefRuntimeBridgeExportRequest(
                plan_dir=Path(args.plan_dir),
                backend=args.backend,
                model=args.model,
                interface=args.interface,
                output_path=Path(args.output) if args.output else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "export-runner-contract":
        result = EefRunnerContractExporter().export(
            EefRunnerContractRequest(
                plan_dir=Path(args.plan_dir),
                backend=args.backend,
                model=args.model,
                interface=args.interface,
                output_path=Path(args.output) if args.output else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "export-agent-runtime-contract":
        result = EefAgentRuntimeContractExporter().export(
            EefAgentRuntimeContractRequest(
                plan_dir=Path(args.plan_dir),
                backend=args.backend,
                model=args.model,
                interface=args.interface,
                output_path=Path(args.output) if args.output else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "eef" and args.eef_command == "export-agent-session-plan":
        result = EefAgentSessionPlanExporter().export(
            EefAgentSessionPlanRequest(
                plan_dir=Path(args.plan_dir),
                backend=args.backend,
                model=args.model,
                interface=args.interface,
                output_path=Path(args.output) if args.output else None,
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "plan":
        planner = SysIdPlanner.default()
        if args.execute:
            plan = planner.plan(args.profile, execute=True)
            payload = {"status": "rejected", **plan.to_json()}
            _emit(payload, as_json=args.as_json)
            return 3
        if args.output:
            q_center = tuple(args.q_center or [0.0] * args.dof)
            if len(q_center) != args.dof:
                parser.error("--q-center length must match --dof")
            render_path = None
            if args.render is not None:
                render_candidate = Path(args.render)
                render_path = (
                    Path(args.output) / render_candidate
                    if not render_candidate.is_absolute()
                    else render_candidate
                )
            try:
                plan = planner.write_plan(
                    SysIdPlanRequest(
                        profile_name=args.profile,
                        dof=args.dof,
                        sample_hz=args.sample_hz,
                        duration_s=args.duration,
                        amplitude_rad=args.amplitude,
                        q_center=q_center,
                        urdf_path=_repo_default_path(args.urdf_path),
                        safe_config_path=_repo_default_path(args.safe_config),
                        output_dir=Path(args.output),
                        render_path=render_path,
                        candidate_trajectory_path=(
                            Path(args.candidate_trajectory)
                            if args.candidate_trajectory is not None
                            else None
                        ),
                    )
                )
            except TrajectoryCommandError as exc:
                payload = {
                    "status": "faulted",
                    "schema": "armctrl.sysid_plan.v1",
                    "profile": args.profile,
                    "error": {
                        "code": "trajectory_command_failed",
                        "message": str(exc),
                        "detail": exc.detail,
                    },
                }
                return _emit(payload, as_json=args.as_json) or 1
        else:
            plan = planner.plan(args.profile, execute=False)
        payload = {"status": "ok", **plan.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "compile-runtime":
        try:
            payload = _compile_sysid_runtime_command(
                execution_trajectory_path=Path(args.execution_trajectory),
                dof=args.dof,
                sample_hz=args.sample_hz,
                expected_q_start=args.expected_q_start,
                owner=args.owner,
                start_pose_policy=args.start_pose_policy,
                send_hz=args.send_hz,
                max_tracking_error_rad=args.max_tracking_error_rad,
                tracking_error_grace_samples=args.tracking_error_grace_samples,
                tracking_error_consecutive_samples=(
                    args.tracking_error_consecutive_samples
                ),
                max_tau_abs=args.max_tau_abs,
                output_dir=Path(args.output),
            )
        except (OSError, ValueError) as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.sysid_runtime_compile.v1",
                "command_surface": MOTION_COMMAND_SURFACE,
                "motion_kind": "joint-trajectory",
                "reason": str(error),
                "movement_command_sent": False,
                "hardware_motion": False,
                "next_gate": "inspect SysID execution trajectory before runtime compile",
            }
            _emit(payload, as_json=args.as_json)
            return 3
        _emit(payload, as_json=args.as_json)
        return 0

    if args.command == "sysid" and args.sysid_command == "run":
        q_center = tuple(args.q_center or [0.0] * args.dof)
        if len(q_center) != args.dof:
            parser.error("--q-center length must match --dof")
        try:
            result = FakeSysIdRunner().run(
                SysIdPlanRequest(
                    profile_name=args.profile,
                    dof=args.dof,
                    sample_hz=args.sample_hz,
                    duration_s=args.duration,
                    amplitude_rad=args.amplitude,
                    q_center=q_center,
                    urdf_path=_repo_default_path(args.urdf_path),
                    safe_config_path=_repo_default_path(args.safe_config),
                    output_dir=Path(args.output),
                )
            )
        except RuntimeSessionError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.sysid_run.v1",
                "adapter": "fake",
                "reason": str(error),
                "movement_allowed": False,
                "runtime": {
                    "single_owner_runtime_session": True,
                    "runtime_session_id": error.payload.get("runtime_session_id"),
                    "mode": error.payload.get("mode"),
                    "owner": error.payload.get("owner"),
                    "readiness": error.payload.get("readiness"),
                },
                "next_gate": (
                    "release active runtime owner or recover runtime to hold_safe "
                    "before compiling/submitting SysID runtime motion"
                ),
            }
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {"status": "ok", **result.to_json()}
        manifest_path = Path(result.artifacts["manifest"])
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if "runtime" in manifest:
            payload["runtime"] = manifest["runtime"]
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "postprocess":
        dataset_path = Path(args.dataset)
        result = SysIdPostprocessor().run(dataset_path)
        if args.solve:
            solver_gate = _sysid_solver_gate(dataset_path)
            if solver_gate["status"] != "pass":
                payload = {
                    "status": "blocked",
                    **result.to_json(),
                    "solver_gate": solver_gate,
                }
                _emit(payload, as_json=args.as_json)
                return 3
            solver_result = SysIdSolver().run(dataset_path)
            result = SysIdPostprocessResult(
                schema=result.schema,
                sample_count=result.sample_count,
                artifacts=result.artifacts,
                solver=solver_result.to_json(),
            )
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "solve":
        dataset_path = Path(args.dataset)
        solver_gate = _sysid_solver_gate(dataset_path)
        if solver_gate["status"] != "pass":
            payload = {
                "status": "blocked",
                "schema": "armctrl.sysid_solve.v1",
                "dataset": str(dataset_path),
                "solver_gate": solver_gate,
            }
            _emit(payload, as_json=args.as_json)
            return 3
        result = SysIdSolver().run(dataset_path)
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "analyze-measured":
        result = MeasuredSysIdAnalyzer().run(
            MeasuredSysIdAnalyzeRequest(
                dataset_dir=Path(args.dataset),
                urdf_path=args.urdf_path,
                primary_label=args.primary,
                validation_labels=tuple(args.validation),
                output_dir=Path(args.output),
                filter_method=args.filter_method,
                savgol_window_samples=args.savgol_window_samples,
                savgol_polyorder=args.savgol_polyorder,
            )
        )
        return _emit(result.to_json(), as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "review-candidate":
        result = SysIdOfflineReviewer().review(
            SysIdOfflineReviewRequest(
                plan_dir=Path(args.plan_dir),
                trajectory_path=Path(args.trajectory) if args.trajectory else None,
                output_dir=Path(args.output),
                urdf_path=Path(args.urdf_path),
                safe_config_path=Path(args.safe_config),
            )
        )
        payload = {"status": "ok", **result}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "package":
        result = SysIdPackager().run(Path(args.dataset))
        payload = {"status": result.status, **result.to_json()}
        _emit(payload, as_json=args.as_json)
        return 0 if result.status == "ok" else 3

    if args.command == "sysid" and args.sysid_command == "import-evidence":
        result = SysIdEvidenceImporter().run(Path(args.dataset), Path(args.evidence))
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "adapt-figaroh-evidence":
        result = FigarohEvidenceAdapter().run(Path(args.input), Path(args.output))
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "figaroh-handoff":
        result = FigarohHandoffWriter().run(Path(args.dataset), Path(args.output))
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "sdk-preflight":
        result = SdkPreflight().run(model=args.model, interface=args.interface)
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "sdk-doctor":
        result = SdkDoctor().run(
            model=args.model,
            interface=args.interface,
            state_sample_count=args.state_sample_count,
            state_sample_period_s=args.state_sample_period,
        )
        payload = {"status": "ok", **result.to_json()}
        doctor_gate = payload.get("doctor_gate")
        if isinstance(doctor_gate, dict) and doctor_gate.get("status") != "pass":
            payload["status"] = "blocked"
            payload["next_gate"] = "fix_sdk_doctor_gate_before_hold_damping"
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="doctor",
        )
        _emit(payload, as_json=args.as_json)
        return 0 if payload["status"] == "ok" else 3

    if args.command == "sysid" and args.sysid_command == "sdk-hold-damping-check":
        doctor_artifact = json.loads(
            Path(args.doctor_artifact).read_text(encoding="utf-8")
        )
        try:
            result = SdkHoldDampingCheck().run(
                model=args.model,
                interface=args.interface,
                confirm=args.confirm,
                doctor_artifact=doctor_artifact,
            )
        except RuntimeError as error:
            if str(error) != "sdk doctor prerequisite failed":
                raise
            payload = {
                "status": "rejected",
                "schema": "armctrl.sysid_sdk_hold_damping_check.v1",
                "model": args.model,
                "interface": args.interface,
                "movement_allowed": False,
                "mode_change_allowed": False,
                "requires_confirm": args.confirm,
                "joint_commands_sent": False,
                "prerequisites": {"doctor": "fail"},
                "reason": "sdk doctor prerequisite failed",
                "fault_landing_mode": "damping",
                "next_gate": "run sdk-doctor successfully before hold/damping",
            }
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="hold_damping",
            )
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {"status": "ok", **result.to_json()}
        hold_damping_gate = payload.get("hold_damping_gate")
        if (
            isinstance(hold_damping_gate, dict)
            and hold_damping_gate.get("status") != "pass"
        ):
            payload["status"] = "blocked"
            payload["next_gate"] = "fix_hold_damping_gate_before_tiny_motion"
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="hold_damping",
        )
        _emit(payload, as_json=args.as_json)
        return 0 if payload["status"] == "ok" else 3

    if args.command == "sysid" and args.sysid_command == "sdk-arm-session":
        doctor_artifact = json.loads(
            Path(args.doctor_artifact).read_text(encoding="utf-8")
        )
        hold_damping_artifact = json.loads(
            Path(args.hold_damping_artifact).read_text(encoding="utf-8")
        )
        try:
            result = SdkArmSession().run(
                model=args.model,
                interface=args.interface,
                confirm=args.confirm,
                doctor_artifact=doctor_artifact,
                hold_damping_artifact=hold_damping_artifact,
            )
        except RuntimeError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.sdk_arm_session.v1",
                "model": args.model,
                "interface": args.interface,
                "movement_allowed": False,
                "joint_commands_sent": False,
                "reason": str(error),
                "fault_landing_mode": "damping",
                "next_gate": "rerun sdk-doctor and sdk-hold-damping-check before arming session",
            }
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="session",
            )
            _emit(payload, as_json=args.as_json)
            return 3
        payload = {"status": "ok", **result.to_json()}
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="session",
        )
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "sdk-jog-real":
        session_artifact = json.loads(
            Path(args.session_artifact).read_text(encoding="utf-8")
        )
        try:
            result = SdkJogReal().run(
                session_artifact=session_artifact,
                joint_index=args.joint_index,
                delta_rad=args.delta_rad,
                max_delta_rad=args.max_delta_rad,
                send_hz=args.send_hz,
                confirm=args.confirm,
                max_q_current_error_rad=args.max_q_current_error_rad,
            )
        except (RuntimeError, ValueError) as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.sdk_jog_real.v1",
                "hardware_motion": "unknown",
                "movement_command_sent": "unknown",
                "reason": str(error),
                "fault_landing_mode": "damping",
                "next_gate": "inspect session and measured robot state before retrying jog",
            }
            payload = _diagnostic_only_sdk_payload(payload)
            payload = _attach_output_artifact(payload, args.output, artifact_key="jog")
            _emit(payload, as_json=args.as_json)
            return 3
        payload = _diagnostic_only_sdk_payload({"status": "ok", **result.to_json()})
        run_status = payload.get("run_status")
        if run_status is not None and run_status != "completed":
            payload["status"] = str(run_status)
            payload = _attach_output_artifact(payload, args.output, artifact_key="jog")
            _emit(payload, as_json=args.as_json)
            return 3
        payload = _attach_output_artifact(payload, args.output, artifact_key="jog")
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "sdk-recover-startup-real":
        session_artifact = json.loads(
            Path(args.session_artifact).read_text(encoding="utf-8")
        )
        max_joint_step_rad = args.max_joint_step_rad
        if max_joint_step_rad is None:
            max_joint_step_rad = WorkspaceSafetyConfig.from_yaml(
                Path(args.safe_config)
            ).max_joint_step_rad
        try:
            result = SdkStartupRecovery().run(
                session_artifact=session_artifact,
                q_target=tuple(args.q_target),
                send_hz=args.send_hz,
                hold_seconds=args.hold_seconds,
                hold_hz=args.hold_hz,
                max_joint_step_rad=max_joint_step_rad,
                confirm=args.confirm,
            )
        except (RuntimeError, ValueError) as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.sdk_startup_recovery.v1",
                "hardware_motion": "unknown",
                "movement_command_sent": "unknown",
                "reason": str(error),
                "fault_landing_mode": "damping",
                "next_gate": "inspect measured startup recovery inputs before retrying",
            }
            payload = _diagnostic_only_sdk_payload(payload)
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="startup_recovery",
            )
            _emit(payload, as_json=args.as_json)
            return 3
        payload = _diagnostic_only_sdk_payload({"status": "ok", **result.to_json()})
        run_status = payload.get("run_status")
        if run_status is not None and run_status != "completed":
            payload["status"] = str(run_status)
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="startup_recovery",
            )
            _emit(payload, as_json=args.as_json)
            return 3
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="startup_recovery",
        )
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "sdk-tiny-motion-plan":
        doctor_artifact = json.loads(
            Path(args.doctor_artifact).read_text(encoding="utf-8")
        )
        hold_damping_artifact = json.loads(
            Path(args.hold_damping_artifact).read_text(encoding="utf-8")
        )
        result = SdkTinyMotionPlanner(max_delta_rad=args.max_delta_rad).plan(
            doctor_artifact=doctor_artifact,
            hold_damping_artifact=hold_damping_artifact,
            joint_index=args.joint_index,
            delta_rad=args.delta_rad,
            confirm=args.confirm,
        )
        payload = {"status": "ok", **result.to_json()}
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="tiny_motion_plan",
        )
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "sdk-tiny-motion-execute-fake":
        plan_artifact = json.loads(
            Path(args.plan_artifact).read_text(encoding="utf-8")
        )
        result = SdkTinyMotionExecutor().execute(
            plan_artifact=plan_artifact,
            backend_name="fake",
            dof=args.dof,
            q_current=tuple(args.q_current),
            confirm=args.confirm,
            send_hz=args.send_hz,
        )
        payload = {"status": "ok", **result.to_json()}
        run_status = payload.get("run_status")
        if run_status is not None and run_status != "completed":
            payload["status"] = str(run_status)
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        payload = _attach_output_artifact(payload, args.output)
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "sdk-tiny-motion-execute-real":
        plan_artifact = json.loads(
            Path(args.plan_artifact).read_text(encoding="utf-8")
        )
        doctor_artifact = json.loads(
            Path(args.doctor_artifact).read_text(encoding="utf-8")
        )
        hold_damping_artifact = json.loads(
            Path(args.hold_damping_artifact).read_text(encoding="utf-8")
        )
        prerequisites = tiny_motion_execute_prerequisite_statuses(
            doctor_artifact=doctor_artifact,
            hold_damping_artifact=hold_damping_artifact,
        )
        try:
            result = SdkTinyMotionExecutor().execute(
                plan_artifact=plan_artifact,
                doctor_artifact=doctor_artifact,
                hold_damping_artifact=hold_damping_artifact,
                backend_name="arx5_sdk",
                model=args.model,
                interface=args.interface,
                dof=args.dof,
                q_current=tuple(args.q_current),
                confirm=args.confirm,
                send_hz=args.send_hz,
                max_q_current_error_rad=args.max_q_current_error_rad,
            )
        except ModuleNotFoundError as error:
            if error.name not in {None, "arx5_interface"}:
                raise
            payload = {
                "status": "rejected",
                "schema": "armctrl.sysid_sdk_tiny_motion_execute.v1",
                "backend": "arx5_sdk",
                "hardware_motion": False,
                "movement_command_sent": False,
                "prerequisites": prerequisites,
                "reason": "arx5_interface is not importable in this environment",
                "requires_confirm": args.confirm,
                "fault_landing_mode": "damping",
                "next_gate": "install arx5_interface on the target Linux host and rerun sdk-doctor before tiny motion",
            }
            payload = _diagnostic_only_sdk_payload(payload)
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        except SdkMeasuredStateMismatchError as error:
            payload = {
                "status": "rejected",
                "schema": "armctrl.sysid_sdk_tiny_motion_execute.v1",
                "backend": "arx5_sdk",
                "hardware_motion": True,
                "movement_command_sent": False,
                "prerequisites": prerequisites,
                "reason": str(error),
                "requires_confirm": args.confirm,
                "fault_landing_mode": "damping",
                "next_gate": "plan an explicit measured-state recovery before retrying tiny motion",
                **error.to_rejection_payload(),
            }
            payload = _diagnostic_only_sdk_payload(payload)
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        except RuntimeError as error:
            if not str(error).startswith(
                "tiny motion execution prerequisites failed:"
            ):
                payload = {
                    "status": "rejected",
                    "schema": "armctrl.sysid_sdk_tiny_motion_execute.v1",
                    "backend": "arx5_sdk",
                    "hardware_motion": "unknown",
                    "movement_command_sent": "unknown",
                    "prerequisites": prerequisites,
                    "reason": f"tiny motion runtime failed: {error}",
                    "requires_confirm": args.confirm,
                    "fault_landing_mode": "damping",
                    "next_gate": "inspect robot state and runtime logs before retrying tiny motion",
                }
            else:
                payload = {
                    "status": "rejected",
                    "schema": "armctrl.sysid_sdk_tiny_motion_execute.v1",
                    "backend": "arx5_sdk",
                    "hardware_motion": False,
                    "movement_command_sent": False,
                    "prerequisites": prerequisites,
                    "reason": str(error),
                    "requires_confirm": args.confirm,
                    "fault_landing_mode": "damping",
                    "next_gate": "rerun sdk-doctor and sdk-hold-damping-check before real tiny motion",
                }
            payload = _diagnostic_only_sdk_payload(payload)
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        payload = _diagnostic_only_sdk_payload({"status": "ok", **result.to_json()})
        run_status = payload.get("run_status")
        if run_status is not None and run_status != "completed":
            payload["status"] = str(run_status)
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        acceptance_status = _nonpassing_acceptance_status(payload)
        if acceptance_status is not None:
            payload["status"] = str(acceptance_status)
            payload = _attach_output_artifact(payload, args.output)
            _emit(payload, as_json=args.as_json)
            return 3
        payload = _attach_output_artifact(payload, args.output)
        return _emit(payload, as_json=args.as_json)

    if (
        args.command == "sysid"
        and args.sysid_command == "sdk-agent-sysid-smoke-readiness"
    ):
        doctor_artifact = json.loads(
            Path(args.doctor_artifact).read_text(encoding="utf-8")
        )
        hold_damping_artifact = json.loads(
            Path(args.hold_damping_artifact).read_text(encoding="utf-8")
        )
        runtime_status_artifact = refresh_runtime_status_payload(
            json.loads(Path(args.runtime_status_artifact).read_text(encoding="utf-8")),
            max_heartbeat_age_s=args.max_heartbeat_age_s,
        )
        result = SdkAgentSysIdSmokeReadinessChecker().check(
            doctor_artifact=doctor_artifact,
            hold_damping_artifact=hold_damping_artifact,
            runtime_status_artifact=runtime_status_artifact,
        )
        result_payload = result.to_json()
        readiness_allowed = result_payload.get("agent_sysid_smoke_allowed") is True
        payload = {
            "status": "ok" if readiness_allowed else "blocked",
            **result_payload,
        }
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="readiness",
        )
        _emit(payload, as_json=args.as_json)
        return 0 if readiness_allowed else 3

    if args.command == "sysid" and args.sysid_command == "sdk-handshake-plan":
        result = SdkHandshakePlanner().plan(model=args.model, interface=args.interface)
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "online-id" and args.online_command == "policy":
        payload = {"status": "ok", **OnlineIdentificationPolicy.default().to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "online-id" and args.online_command == "audit":
        result = OnlineIdentificationAuditor.default().record(
            OnlineAuditRequest(
                update=ParameterUpdate(
                    name=args.parameter,
                    value=args.value,
                    source=args.source,
                ),
                window_start_s=args.window_start,
                window_end_s=args.window_end,
                residual_before=args.residual_before,
                residual_after=args.residual_after,
                saturation_status=args.saturation_status,
                rollback_target=args.rollback_target,
                output_path=Path(args.output),
            )
        )
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "release" and args.release_command == "status":
        payload = {"status": "ok", **release_status()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "release" and args.release_command == "notes":
        payload = {"status": "ok", **release_notes()}
        return _emit(payload, as_json=args.as_json)

    parser.error("unsupported command")
    return 2


def _repo_default_path(path_text: str) -> str:
    """Resolve built-in repo-relative defaults without changing custom paths."""

    path = Path(path_text)
    if path.is_absolute():
        return str(path)
    repo_path = REPO_ROOT / path
    if repo_path.exists():
        return str(repo_path)
    return path_text


def _motion_profile_catalog(*, include_description: bool) -> list[dict[str, object]]:
    profiles: list[dict[str, object]] = []
    for name, profile in sorted(MOTION_PROFILES.items()):
        item: dict[str, object] = {
            "name": name,
            "runtime_backend": profile.get("runtime_backend"),
            "owner": profile.get("owner"),
            "start_pose_policy": profile.get("start_pose_policy"),
            "supported_motion_kinds": profile.get("supported_motion_kinds"),
            "status": profile.get("status", "available"),
        }
        if include_description:
            item["description"] = str(profile.get("description"))
            item["readiness"] = profile.get("readiness")
            item["legacy_entrypoints"] = profile.get("legacy_entrypoints", [])
            item["removed_entrypoints"] = profile.get("removed_entrypoints", [])
            item["preferred_entrypoints"] = profile.get("preferred_entrypoints", [])
        profiles.append(item)
    return profiles


def _motion_surface_catalog() -> dict[str, object]:
    return {
        "submit_schema": MOTION_COMMAND_SURFACE,
        "supported_submit_kinds": [
            "joint-trajectory",
            "joint-intent",
            "eef-delta",
            "eef-twist",
            "eef-pose",
            "joint-jog",
        ],
        "hardware_not_implemented_yet": ["joint-jog"],
        "reserved_contract_kinds": {
            "eef-pose": "absolute EEF pose runtime command; requires a mature pose adapter with live reference limiting",
            "joint-jog": "teleop/manual jog command contract; rejected until deadman and jog backend are configured",
        },
        "eef_control_policy": {
            "agent_input_space": ["eef-delta", "eef-twist", "eef-pose"],
            "runtime_execution_policy": "continuous_owner_servo_or_reviewed_planned_path",
            "servo_path": (
                "EEF delta/twist/pose frames are consumed by a mature backend "
                "inside the long-lived runtime; every tick is limited from live "
                "state and fallback stays in hold or damping."
            ),
            "planned_path": (
                "absolute or multi-waypoint EEF goals may be reviewed into "
                "joint-trajectory commands, but that is a planned path rather "
                "than the realtime EEF servo path."
            ),
            "disconnected_takeover_allowed": False,
            "bumpless_switch_required": True,
            "warmup_policy": (
                "start EEF backend with target=current FK pose, run zero-command "
                "warmup, then switch owner only after q_meas/fault/jitter checks pass"
            ),
            "reference_limit_policy": {
                "policy": "reject_oversized_eef_reference_step",
                "default_max_linear_step_m": DEFAULT_EEF_MAX_LINEAR_STEP_M,
                "default_max_angular_step_rad": DEFAULT_EEF_MAX_ANGULAR_STEP_RAD,
                "clamping": False,
                "enforced_at": ["submit", "execute"],
            },
        },
        "formal_cli": [
            "armctrl motion compile joint-trajectory",
            "armctrl motion submit joint-trajectory",
            "armctrl motion submit joint-trajectory --compiled-command",
            "armctrl motion submit joint-intent",
            "armctrl motion submit eef-delta",
            "armctrl motion submit eef-twist",
            "armctrl motion submit eef-pose",
            "armctrl motion submit joint-jog",
        ],
    }


def _command_surface_classes() -> dict[str, object]:
    return {
        "formal": [
            "console",
            "profile",
            "runtime",
            "motion",
        ],
        "compiler": [
            "armctrl motion compile joint-trajectory",
            "armctrl sysid compile-runtime",
        ],
        "removed": [
            "armctrl sysid run ... --adapter sdk",
        ],
        "read_only_diagnostic": [
            "armctrl sysid sdk-preflight",
            "armctrl sysid sdk-doctor",
            "armctrl sysid sdk-hold-damping-check",
            "armctrl sysid sdk-arm-session",
            "armctrl sysid sdk-tiny-motion-plan",
            "armctrl sysid sdk-tiny-motion-execute-fake",
            "armctrl sysid sdk-handshake-plan",
        ],
        "legacy_compatibility_wrapper": [
            (
                "armctrl sysid sdk-agent-sysid-smoke-readiness "
                "--runtime-status-artifact <live_runtime_status.json>"
            ),
        ],
        "hardware_diagnostic_only": [
            "armctrl sysid sdk-jog-real",
            "armctrl sysid sdk-recover-startup-real",
            "armctrl sysid sdk-tiny-motion-execute-real",
        ],
        "diagnostic": [
            "see read_only_diagnostic and hardware_diagnostic_only",
        ],
        "legacy_alias": [
            "armctrl runtime submit-trajectory",
            "armctrl runtime submit-intent",
            "armctrl runtime submit-eef",
            "armctrl runtime result-check",
        ],
        "experimental_review_export": [
            "armctrl agent-flow",
            "armctrl eef",
            "armctrl lerobot",
        ],
    }


def _handle_motion_compile(args: argparse.Namespace) -> int:
    try:
        if args.motion_kind != "joint-trajectory":
            raise ValueError(f"unsupported motion compile kind: {args.motion_kind}")
        payload = _compile_joint_trajectory_command(
            source=args.source,
            owner=args.owner,
            expected_q_start=args.expected_q_start,
            q_target=args.q_target,
            q_points=args.q_point,
            duration_s=args.duration_s,
            sample_hz=args.sample_hz,
            send_hz=args.send_hz,
            start_pose_policy=args.start_pose_policy,
            max_joint_segment_delta_rad=args.max_joint_segment_delta_rad,
            max_joint_velocity_rad_s=_agent_default_joint_velocity_limit(
                owner=args.owner,
                requested_limit=args.max_joint_velocity_rad_s,
            ),
            max_joint_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
            max_tracking_error_rad=args.max_tracking_error_rad,
            tracking_error_grace_samples=args.tracking_error_grace_samples,
            tracking_error_consecutive_samples=(
                args.tracking_error_consecutive_samples
            ),
            max_tau_abs=args.max_tau_abs,
            output_dir=Path(args.output),
        )
        _emit(payload, as_json=args.as_json)
        return 0
    except ValueError as error:
        payload = {
            "status": "rejected",
            "schema": "armctrl.motion_runtime_compile.v1",
            "command_surface": MOTION_COMMAND_SURFACE,
            "motion_kind": getattr(args, "motion_kind", None),
            "source": getattr(args, "source", None),
            "owner": getattr(args, "owner", None),
            "reason": str(error),
            "movement_command_sent": False,
            "hardware_motion": False,
        }
        _emit(payload, as_json=args.as_json)
        return 3


def _handle_motion_submit(args: argparse.Namespace) -> int:
    try:
        if args.motion_kind == "joint-trajectory":
            if args.compiled_command is not None:
                payload = _submit_compiled_joint_trajectory_command(args)
                payload = _attach_output_artifact(
                    payload,
                    args.output,
                    artifact_key="motion_submit",
                )
                _emit(payload, as_json=args.as_json)
                return 0
            if args.expected_q_start is None or args.q_point is None:
                raise ValueError(
                    "joint-trajectory requires either --compiled-command or both "
                    "--expected-q-start and at least one --q-point"
                )
            payload = submit_trajectory_command(
                session_artifact_path=Path(args.session_artifact),
                owner=args.owner,
                expected_q_start=tuple(args.expected_q_start),
                q_points=tuple(tuple(point) for point in args.q_point),
                send_hz=args.send_hz,
                trajectory_sample_hz=args.trajectory_sample_hz,
                start_pose_policy=args.start_pose_policy,
                max_start_error_rad=args.max_start_error_rad,
                heartbeat_timeout_s=args.heartbeat_timeout_s,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
                output_path=None,
                max_tracking_error_rad=args.max_tracking_error_rad,
                tracking_error_grace_samples=args.tracking_error_grace_samples,
                tracking_error_consecutive_samples=(
                    args.tracking_error_consecutive_samples
                ),
                max_tau_abs=args.max_tau_abs,
                max_joint_segment_delta_rad=args.max_joint_segment_delta_rad,
                max_joint_velocity_rad_s=_agent_default_joint_velocity_limit(
                    owner=args.owner,
                    requested_limit=args.max_joint_velocity_rad_s,
                ),
                max_joint_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
            )
            payload = _annotate_motion_submit_payload(
                payload,
                motion_kind="joint-trajectory",
                legacy_equivalent="armctrl runtime submit-trajectory",
            )
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="motion_submit",
            )
            _emit(payload, as_json=args.as_json)
            return _motion_submit_exit_code(payload)

        if args.motion_kind == "joint-intent":
            payload = submit_intent_command(
                session_artifact_path=Path(args.session_artifact),
                owner=args.owner,
                expected_q_start=tuple(args.expected_q_start),
                q_target=tuple(args.q_target),
                control_period_s=args.control_period_s,
                send_hz=args.send_hz,
                start_pose_policy=args.start_pose_policy,
                max_joint_delta_rad=args.max_joint_delta_rad,
                max_joint_velocity_rad_s=args.max_joint_velocity_rad_s,
                max_joint_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
                max_start_error_rad=args.max_start_error_rad,
                heartbeat_timeout_s=args.heartbeat_timeout_s,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
                output_path=None,
                max_tracking_error_rad=args.max_tracking_error_rad,
                tracking_error_grace_samples=args.tracking_error_grace_samples,
                tracking_error_consecutive_samples=(
                    args.tracking_error_consecutive_samples
                ),
                max_tau_abs=args.max_tau_abs,
            )
            payload = _annotate_motion_submit_payload(
                payload,
                motion_kind="joint-intent",
                legacy_equivalent="armctrl runtime submit-intent",
            )
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="motion_submit",
            )
            _emit(payload, as_json=args.as_json)
            return _motion_submit_exit_code(payload)

        if args.motion_kind == "eef-delta":
            payload = submit_eef_command(
                session_artifact_path=Path(args.session_artifact),
                owner=args.owner,
                backend=args.backend,
                kind="eef_pose_delta",
                frame=args.frame,
                expected_q_start=tuple(args.expected_q_start),
                control_period_s=args.control_period_s,
                send_hz=args.send_hz,
                start_pose_policy=args.start_pose_policy,
                max_start_error_rad=args.max_start_error_rad,
                heartbeat_timeout_s=args.heartbeat_timeout_s,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
                delta_position_m=tuple(args.delta_position),
                delta_rpy_rad=tuple(args.delta_rpy),
                max_linear_step_m=args.max_linear_step_m,
                max_angular_step_rad=args.max_angular_step_rad,
                output_path=None,
            )
            payload = _annotate_motion_submit_payload(
                payload,
                motion_kind="eef-delta",
                legacy_equivalent="armctrl runtime submit-eef --kind eef_pose_delta",
            )
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="motion_submit",
            )
            _emit(payload, as_json=args.as_json)
            return _motion_submit_exit_code(payload)

        if args.motion_kind == "eef-twist":
            payload = submit_eef_command(
                session_artifact_path=Path(args.session_artifact),
                owner=args.owner,
                backend=args.backend,
                kind="eef_twist",
                frame=args.frame,
                expected_q_start=tuple(args.expected_q_start),
                control_period_s=args.control_period_s,
                send_hz=args.send_hz,
                start_pose_policy=args.start_pose_policy,
                max_start_error_rad=args.max_start_error_rad,
                heartbeat_timeout_s=args.heartbeat_timeout_s,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
                linear_mps=tuple(args.linear),
                angular_rps=tuple(args.angular),
                max_linear_step_m=args.max_linear_step_m,
                max_angular_step_rad=args.max_angular_step_rad,
                output_path=None,
            )
            payload = _annotate_motion_submit_payload(
                payload,
                motion_kind="eef-twist",
                legacy_equivalent="armctrl runtime submit-eef --kind eef_twist",
            )
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="motion_submit",
            )
            _emit(payload, as_json=args.as_json)
            return 0

        if args.motion_kind == "eef-pose":
            payload = submit_eef_command(
                session_artifact_path=Path(args.session_artifact),
                owner=args.owner,
                backend=args.backend,
                kind="eef_pose",
                frame=args.frame,
                expected_q_start=tuple(args.expected_q_start),
                control_period_s=args.control_period_s,
                send_hz=args.send_hz,
                start_pose_policy=args.start_pose_policy,
                max_start_error_rad=args.max_start_error_rad,
                heartbeat_timeout_s=args.heartbeat_timeout_s,
                max_heartbeat_age_s=args.max_heartbeat_age_s,
                position_m=tuple(args.position),
                rpy_rad=tuple(args.rpy),
                output_path=None,
            )
            payload = _annotate_motion_submit_payload(
                payload,
                motion_kind="eef-pose",
                legacy_equivalent="armctrl runtime submit-eef --kind eef_pose",
            )
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="motion_submit",
            )
            _emit(payload, as_json=args.as_json)
            return 0

        if args.motion_kind == "joint-jog":
            payload = _unsupported_motion_submit_payload(args)
            payload = _attach_output_artifact(
                payload,
                args.output,
                artifact_key="motion_submit",
            )
            _emit(payload, as_json=args.as_json)
            return 3
    except RuntimeSessionError as error:
        payload = {
            **error.payload,
            "status": "rejected",
            "schema": "armctrl.arm_runtime_submit.v1",
            "command_surface": MOTION_COMMAND_SURFACE,
            "motion_kind": args.motion_kind,
            "reason": str(error),
            "movement_command_sent": False,
        }
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="motion_submit",
        )
        _emit(payload, as_json=args.as_json)
        return 3
    except ValueError as error:
        payload = {
            "status": "rejected",
            "schema": "armctrl.arm_runtime_submit.v1",
            "command_surface": MOTION_COMMAND_SURFACE,
            "motion_kind": args.motion_kind,
            "owner": getattr(args, "owner", None),
            "reason": str(error),
            "movement_command_sent": False,
        }
        payload = _attach_output_artifact(
            payload,
            args.output,
            artifact_key="motion_submit",
        )
        _emit(payload, as_json=args.as_json)
        return 3
    raise AssertionError(f"unsupported motion kind: {args.motion_kind}")


def _annotate_motion_submit_payload(
    payload: dict[str, object],
    *,
    motion_kind: str,
    legacy_equivalent: str,
) -> dict[str, object]:
    annotated = dict(payload)
    annotated["command_surface"] = MOTION_COMMAND_SURFACE
    annotated["motion_kind"] = motion_kind
    annotated["legacy_equivalent"] = legacy_equivalent
    _annotate_motion_command_artifact(
        annotated,
        motion_kind=motion_kind,
        legacy_equivalent=legacy_equivalent,
    )
    return annotated


def _motion_submit_exit_code(payload: dict[str, object]) -> int:
    return 0 if payload.get("status") == "queued" else 3


def _annotate_motion_command_artifact(
    payload: dict[str, object],
    *,
    motion_kind: str,
    legacy_equivalent: str,
) -> None:
    artifacts = payload.get("artifacts")
    if not isinstance(artifacts, dict):
        return
    command_path_value = artifacts.get("command")
    if not isinstance(command_path_value, str):
        return
    command_path = Path(command_path_value)
    try:
        command = json.loads(command_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return
    if not isinstance(command, dict):
        return
    command["command_surface"] = MOTION_COMMAND_SURFACE
    command["motion_kind"] = motion_kind
    command["legacy_equivalent"] = legacy_equivalent
    _write_json_atomic(command_path, command)


def _unsupported_motion_submit_payload(args: argparse.Namespace) -> dict[str, object]:
    if args.motion_kind == "eef-pose":
        reason = (
            "eef-pose is a formal command kind, but absolute EEF hardware execution "
            "requires a mature continuous-owner EEF servo backend or an explicit "
            "reviewed planned path; heuristic joint fallback and disconnected "
            "sdk_cartesian takeover are forbidden"
        )
        backend = args.backend
        owner = args.owner
        command_space = "eef"
        requested_command = {
            "kind": "eef_pose",
            "frame": args.frame,
            "position_m": (
                [float(value) for value in args.position]
                if args.position is not None
                else None
            ),
            "rpy_rad": (
                [float(value) for value in args.rpy] if args.rpy is not None else None
            ),
        }
    else:
        reason = (
            "joint-jog is reserved for teleop/manual control and requires a deadman "
            "and mature jog backend before hardware execution"
        )
        backend = "not_configured"
        owner = args.owner
        command_space = "joint"
        requested_command = {
            "kind": "joint_jog",
            "joint_delta_rad": (
                [float(value) for value in args.joint_delta]
                if args.joint_delta is not None
                else None
            ),
            "velocity_rad_s": (
                [float(value) for value in args.velocity]
                if args.velocity is not None
                else None
            ),
            "deadman": bool(args.deadman),
        }
    return {
        "status": "rejected",
        "schema": "armctrl.arm_runtime_submit.v1",
        "command_surface": MOTION_COMMAND_SURFACE,
        "motion_kind": args.motion_kind,
        "owner": owner,
        "backend": backend,
        "command_space": command_space,
        "requested_command": requested_command,
        "reason": reason,
        "movement_command_sent": False,
        "runtime": {
            "single_motion_owner": True,
            "session_artifact": str(args.session_artifact),
        },
        "mature_backend_policy": {
            "no_heuristic_joint_fallback": True,
            "disconnected_takeover_allowed": False,
            "servo_path": (
                "run eef pose/twist/delta inside the long-lived runtime with "
                "target=current FK warmup and per-tick limiting"
            ),
            "planned_path": (
                "review eef waypoints into checked joint_trajectory when realtime "
                "servo is not the intended behavior"
            ),
        },
        "next_gate": "configure mature continuous-owner runtime backend before hardware execution",
    }


def _submit_compiled_joint_trajectory_command(
    args: argparse.Namespace,
) -> dict[str, object]:
    compiled_path = Path(args.compiled_command)
    compiled = _read_compiled_joint_trajectory_command(compiled_path)
    payload = submit_trajectory_command(
        session_artifact_path=Path(args.session_artifact),
        owner=str(compiled["owner"]),
        expected_q_start=compiled["expected_q_start"],
        q_points=compiled["q_points"],
        dq_points=compiled.get("dq_points"),
        ddq_points=compiled.get("ddq_points"),
        artifact_policy=compiled.get("artifact_policy"),
        send_hz=float(compiled["send_hz"]),
        trajectory_sample_hz=(
            float(compiled["trajectory_sample_hz"])
            if compiled.get("trajectory_sample_hz") is not None
            else None
        ),
        start_pose_policy=str(compiled.get("start_pose_policy", "live_hold")),
        max_start_error_rad=args.max_start_error_rad,
        heartbeat_timeout_s=args.heartbeat_timeout_s,
        max_heartbeat_age_s=args.max_heartbeat_age_s,
        max_tracking_error_rad=(
            args.max_tracking_error_rad
            if args.max_tracking_error_rad is not None
            else _optional_float(compiled.get("max_tracking_error_rad"))
        ),
        tracking_error_grace_samples=(
            args.tracking_error_grace_samples
            if args.tracking_error_grace_samples is not None
            else _optional_int(compiled.get("tracking_error_grace_samples"))
        ),
        tracking_error_consecutive_samples=(
            args.tracking_error_consecutive_samples
            if args.tracking_error_consecutive_samples is not None
            else _optional_int(compiled.get("tracking_error_consecutive_samples"))
        ),
        max_tau_abs=(
            args.max_tau_abs
            if args.max_tau_abs is not None
            else _optional_float(compiled.get("max_tau_abs"))
        ),
        max_joint_segment_delta_rad=_optional_float(
            compiled.get("max_joint_segment_delta_rad")
        ),
        max_joint_velocity_rad_s=_optional_float(
            compiled.get("max_joint_velocity_rad_s")
        ),
        max_joint_acceleration_rad_s2=_optional_float(
            compiled.get("max_joint_acceleration_rad_s2")
        ),
    )
    payload = _annotate_motion_submit_payload(
        payload,
        motion_kind="joint-trajectory",
        legacy_equivalent="armctrl motion submit joint-trajectory --compiled-command",
    )
    payload["compiled_command_artifact"] = str(compiled_path)
    _annotate_compiled_source_artifact(payload, compiled_command_artifact=str(compiled_path))
    return payload


def _agent_default_joint_velocity_limit(
    *,
    owner: str,
    requested_limit: float | None,
) -> float | None:
    if requested_limit is not None:
        return float(requested_limit)
    if str(owner) == "agent":
        return DEFAULT_AGENT_MAX_JOINT_VELOCITY_RAD_S
    return None


def _compile_joint_trajectory_command(
    *,
    source: str,
    owner: str,
    expected_q_start: Sequence[float],
    q_target: Sequence[float] | None,
    q_points: Sequence[Sequence[float]] | None,
    duration_s: float | None,
    sample_hz: float,
    send_hz: float | None,
    start_pose_policy: str,
    max_joint_segment_delta_rad: float | None,
    max_joint_velocity_rad_s: float | None,
    max_joint_acceleration_rad_s2: float | None,
    max_tracking_error_rad: float | None,
    tracking_error_grace_samples: int | None,
    tracking_error_consecutive_samples: int | None,
    max_tau_abs: float | None,
    output_dir: Path,
) -> dict[str, object]:
    q_start = _numeric_list(expected_q_start, name="expected_q_start")
    if sample_hz <= 0.0:
        raise ValueError("--sample-hz must be positive")
    effective_send_hz = float(send_hz) if send_hz is not None else float(sample_hz)
    if effective_send_hz <= 0.0:
        raise ValueError("--send-hz must be positive")
    has_target = q_target is not None
    has_waypoints = q_points is not None and len(q_points) > 0
    if has_target == has_waypoints:
        raise ValueError("provide exactly one of --q-target or one or more --q-point")
    if has_target:
        if duration_s is None or duration_s <= 0.0:
            raise ValueError("--duration-s must be positive when using --q-target")
        target = _numeric_list(q_target, name="q_target")
        if len(target) != len(q_start):
            raise ValueError("--q-target length must match --expected-q-start length")
        compiled_q, compiled_dq, compiled_ddq = _smoothstep_joint_target_points(
            q_start=q_start,
            q_target=target,
            duration_s=float(duration_s),
            sample_hz=float(sample_hz),
        )
        q_policy = "generated_smoothstep_joint_target"
        dq_policy = "derived_smoothstep_analytic"
        ddq_policy = "derived_smoothstep_analytic"
        interpolation_policy = "smoothstep_joint_target"
        source_mode = "joint_target"
    else:
        compiled_q = _numeric_matrix(q_points, name="q_point")
        if not compiled_q:
            raise ValueError("--q-point must not be empty")
        for point in compiled_q:
            if len(point) != len(q_start):
                raise ValueError("all --q-point values must match --expected-q-start length")
        compiled_dq = _finite_difference_points(
            compiled_q,
            sample_hz=float(sample_hz),
        )
        compiled_ddq = _finite_difference_points(
            compiled_dq,
            sample_hz=float(sample_hz),
        )
        q_policy = "preserved_waypoints"
        dq_policy = "derived_finite_difference"
        ddq_policy = "derived_finite_difference"
        interpolation_policy = "waypoint_finite_difference"
        source_mode = "joint_waypoints"

    joint_trajectory_safety = _compiled_joint_trajectory_safety(
        owner=owner,
        q_points=compiled_q,
        trajectory_sample_hz=float(sample_hz),
        max_joint_segment_delta_rad=max_joint_segment_delta_rad,
        max_joint_velocity_rad_s=max_joint_velocity_rad_s,
        max_joint_acceleration_rad_s2=max_joint_acceleration_rad_s2,
    )
    if joint_trajectory_safety["status"] != "pass":
        raise ValueError(
            "joint trajectory safety failed: "
            + ", ".join(str(item) for item in joint_trajectory_safety["failed_checks"])
        )
    artifact_policy = {
        "schema": "armctrl.joint_trajectory_compiler_policy.v1",
        "source": str(source),
        "source_mode": source_mode,
        "q_cmd": q_policy,
        "dq_cmd": dq_policy,
        "ddq_cmd": ddq_policy,
        "sample_hz": float(sample_hz),
        "send_hz": effective_send_hz,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    command_path = output_dir / "compiled_motion_command.json"
    manifest_path = output_dir / "compile_motion_manifest.json"
    command: dict[str, object] = {
        "schema": "armctrl.compiled_motion_command.v1",
        "command_surface": MOTION_COMMAND_SURFACE,
        "motion_kind": "joint-trajectory",
        "source": str(source),
        "owner": str(owner),
        "expected_q_start": q_start,
        "q_points": [list(point) for point in compiled_q],
        "dq_points": [list(point) for point in compiled_dq],
        "ddq_points": [list(point) for point in compiled_ddq],
        "send_hz": effective_send_hz,
        "trajectory_sample_hz": float(sample_hz),
        "start_pose_policy": str(start_pose_policy),
        "artifact_policy": artifact_policy,
        "joint_trajectory_safety": joint_trajectory_safety,
        "interpolation_policy": interpolation_policy,
        "resampling_policy": "compiled_joint_trajectory_to_runtime_send_hz",
    }
    if max_joint_segment_delta_rad is not None:
        command["max_joint_segment_delta_rad"] = float(max_joint_segment_delta_rad)
    if max_joint_velocity_rad_s is not None:
        command["max_joint_velocity_rad_s"] = float(max_joint_velocity_rad_s)
    if max_joint_acceleration_rad_s2 is not None:
        command["max_joint_acceleration_rad_s2"] = float(
            max_joint_acceleration_rad_s2
        )
    if duration_s is not None:
        command["duration_s"] = float(duration_s)
    if max_tracking_error_rad is not None:
        command["max_tracking_error_rad"] = float(max_tracking_error_rad)
        command["tracking_error_grace_samples"] = int(
            3 if tracking_error_grace_samples is None else tracking_error_grace_samples
        )
        command["tracking_error_consecutive_samples"] = int(
            3
            if tracking_error_consecutive_samples is None
            else tracking_error_consecutive_samples
        )
    if max_tau_abs is not None:
        command["max_tau_abs"] = float(max_tau_abs)
    _write_json_atomic(command_path, command)
    payload = {
        "status": "ok",
        "schema": "armctrl.motion_runtime_compile.v1",
        "command_surface": MOTION_COMMAND_SURFACE,
        "motion_kind": "joint-trajectory",
        "source": str(source),
        "owner": str(owner),
        "movement_command_sent": False,
        "hardware_motion": False,
        "sample_count": len(compiled_q),
        "sample_hz": float(sample_hz),
        "send_hz": effective_send_hz,
        "artifact_policy": artifact_policy,
        "joint_trajectory_safety": joint_trajectory_safety,
        "artifacts": {
            "compiled_command": str(command_path),
            "compile_motion_manifest": str(manifest_path),
        },
        "next_gate": "submit compiled command with armctrl motion submit joint-trajectory",
    }
    _write_json_atomic(manifest_path, payload)
    return payload


def _smoothstep_joint_target_points(
    *,
    q_start: Sequence[float],
    q_target: Sequence[float],
    duration_s: float,
    sample_hz: float,
) -> tuple[list[list[float]], list[list[float]], list[list[float]]]:
    sample_count = max(2, int(round(float(duration_s) * float(sample_hz))) + 1)
    q_points: list[list[float]] = []
    dq_points: list[list[float]] = []
    ddq_points: list[list[float]] = []
    delta = [float(target) - float(start) for start, target in zip(q_start, q_target)]
    for index in range(sample_count):
        alpha = index / (sample_count - 1)
        s = (3.0 * alpha * alpha) - (2.0 * alpha * alpha * alpha)
        ds_dt = (6.0 * alpha * (1.0 - alpha)) / float(duration_s)
        d2s_dt2 = (6.0 - (12.0 * alpha)) / (float(duration_s) * float(duration_s))
        q_points.append(
            [float(start) + step * s for start, step in zip(q_start, delta)]
        )
        dq_points.append([step * ds_dt for step in delta])
        ddq_points.append([step * d2s_dt2 for step in delta])
    return q_points, dq_points, ddq_points


def _finite_difference_points(
    points: Sequence[Sequence[float]],
    *,
    sample_hz: float,
) -> list[list[float]]:
    if not points:
        return []
    if len(points) == 1:
        return [[0.0 for _ in points[0]]]
    result: list[list[float]] = []
    for index, point in enumerate(points):
        if index == 0:
            next_point = points[1]
            result.append(
                [
                    (float(next_value) - float(value)) * float(sample_hz)
                    for value, next_value in zip(point, next_point)
                ]
            )
        elif index == len(points) - 1:
            previous = points[index - 1]
            result.append(
                [
                    (float(value) - float(previous_value)) * float(sample_hz)
                    for previous_value, value in zip(previous, point)
                ]
            )
        else:
            previous = points[index - 1]
            next_point = points[index + 1]
            result.append(
                [
                    (float(next_value) - float(previous_value)) * float(sample_hz) / 2.0
                    for previous_value, next_value in zip(previous, next_point)
                ]
            )
    return result


def _compiled_joint_trajectory_safety(
    *,
    owner: str,
    q_points: Sequence[Sequence[float]],
    trajectory_sample_hz: float,
    max_joint_segment_delta_rad: float | None,
    max_joint_velocity_rad_s: float | None,
    max_joint_acceleration_rad_s2: float | None,
) -> dict[str, object]:
    if trajectory_sample_hz <= 0.0:
        raise ValueError("trajectory_sample_hz must be positive")
    max_segment_delta = (
        None
        if max_joint_segment_delta_rad is None
        else float(max_joint_segment_delta_rad)
    )
    if max_segment_delta is not None and max_segment_delta <= 0.0:
        raise ValueError("max_joint_segment_delta_rad must be positive")
    max_velocity = (
        None if max_joint_velocity_rad_s is None else float(max_joint_velocity_rad_s)
    )
    if max_velocity is not None and max_velocity <= 0.0:
        raise ValueError("max_joint_velocity_rad_s must be positive")
    max_acceleration = (
        None
        if max_joint_acceleration_rad_s2 is None
        else float(max_joint_acceleration_rad_s2)
    )
    if max_acceleration is not None and max_acceleration <= 0.0:
        raise ValueError("max_joint_acceleration_rad_s2 must be positive")

    segment_deltas: list[list[float]] = []
    segment_abs_deltas: list[list[float]] = []
    segment_abs_velocities: list[list[float]] = []
    for previous, current in zip(q_points, q_points[1:]):
        deltas = [
            float(current_value) - float(previous_value)
            for previous_value, current_value in zip(previous, current)
        ]
        abs_deltas = [abs(value) for value in deltas]
        segment_deltas.append(deltas)
        segment_abs_deltas.append(abs_deltas)
        segment_abs_velocities.append(
            [value * float(trajectory_sample_hz) for value in abs_deltas]
        )
    segment_abs_accelerations: list[list[float]] = []
    for previous_velocity, current_velocity in zip(
        segment_abs_velocities,
        segment_abs_velocities[1:],
    ):
        segment_abs_accelerations.append(
            [
                abs(float(current_value) - float(previous_value))
                * float(trajectory_sample_hz)
                for previous_value, current_value in zip(
                    previous_velocity,
                    current_velocity,
                )
            ]
        )
    max_abs_delta = max(
        (value for segment in segment_abs_deltas for value in segment),
        default=0.0,
    )
    max_abs_velocity = max(
        (value for segment in segment_abs_velocities for value in segment),
        default=0.0,
    )
    max_abs_acceleration = max(
        (value for segment in segment_abs_accelerations for value in segment),
        default=0.0,
    )
    failed_checks: list[str] = []
    if max_segment_delta is not None and max_abs_delta > max_segment_delta:
        failed_checks.append("joint_segment_delta_within_limit")
    if max_velocity is not None and max_abs_velocity > max_velocity:
        failed_checks.append("joint_velocity_within_limit")
    if max_acceleration is not None and max_abs_acceleration > max_acceleration:
        failed_checks.append("joint_acceleration_within_limit")
    policy = (
        "reject_oversized_or_too_fast_agent_joint_trajectory"
        if str(owner) == "agent"
        else "record_only_for_non_agent_joint_trajectory"
    )
    return {
        "schema": "armctrl.joint_trajectory_safety.v1",
        "status": "pass" if not failed_checks else "fail",
        "policy": policy,
        "owner": str(owner),
        "trajectory_sample_hz": float(trajectory_sample_hz),
        "max_joint_segment_delta_rad": max_segment_delta,
        "max_joint_velocity_rad_s": max_velocity,
        "max_joint_acceleration_rad_s2": max_acceleration,
        "segment_delta_rad": segment_deltas,
        "segment_abs_delta_rad": segment_abs_deltas,
        "segment_abs_velocity_rad_s": segment_abs_velocities,
        "segment_abs_acceleration_rad_s2": segment_abs_accelerations,
        "max_segment_abs_delta_rad": max_abs_delta,
        "max_segment_abs_velocity_rad_s": max_abs_velocity,
        "max_segment_abs_acceleration_rad_s2": max_abs_acceleration,
        "failed_checks": failed_checks,
    }


def _read_compiled_joint_trajectory_command(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("compiled command artifact must contain a JSON object")
    if payload.get("schema") != "armctrl.compiled_motion_command.v1":
        raise ValueError("compiled command must use schema armctrl.compiled_motion_command.v1")
    if payload.get("command_surface") != MOTION_COMMAND_SURFACE:
        raise ValueError(f"compiled command must target {MOTION_COMMAND_SURFACE}")
    if payload.get("motion_kind") != "joint-trajectory":
        raise ValueError("compiled command must use motion_kind=joint-trajectory")
    owner = str(payload.get("owner") or payload.get("source") or "sysid")
    expected_q_start = _numeric_list(
        payload.get("expected_q_start"),
        name="expected_q_start",
    )
    q_points = _numeric_matrix(payload.get("q_points"), name="q_points")
    if not q_points:
        raise ValueError("compiled command q_points must not be empty")
    dof = len(expected_q_start)
    for point in q_points:
        if len(point) != dof:
            raise ValueError("compiled command q_points must match expected_q_start length")
    dq_points = (
        None
        if payload.get("dq_points") is None
        else _numeric_matrix(payload.get("dq_points"), name="dq_points")
    )
    if dq_points is not None and len(dq_points) != len(q_points):
        raise ValueError("compiled command dq_points length must match q_points length")
    ddq_points = (
        None
        if payload.get("ddq_points") is None
        else _numeric_matrix(payload.get("ddq_points"), name="ddq_points")
    )
    if ddq_points is not None and len(ddq_points) != len(q_points):
        raise ValueError("compiled command ddq_points length must match q_points length")
    send_hz = _positive_float(payload.get("send_hz"), name="send_hz")
    compiled = dict(payload)
    compiled["owner"] = owner
    compiled["expected_q_start"] = expected_q_start
    compiled["q_points"] = q_points
    compiled["dq_points"] = dq_points
    compiled["ddq_points"] = ddq_points
    compiled["send_hz"] = send_hz
    return compiled


def _annotate_compiled_source_artifact(
    payload: dict[str, object],
    *,
    compiled_command_artifact: str,
) -> None:
    artifacts = payload.get("artifacts")
    if not isinstance(artifacts, dict):
        return
    command_path_value = artifacts.get("command")
    if not isinstance(command_path_value, str):
        return
    command_path = Path(command_path_value)
    try:
        command = json.loads(command_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return
    if not isinstance(command, dict):
        return
    command["compiled_command_artifact"] = compiled_command_artifact
    _write_json_atomic(command_path, command)


def _optional_float(value: object) -> float | None:
    if value is None:
        return None
    return float(value)


def _optional_int(value: object) -> int | None:
    if value is None:
        return None
    return int(value)


def _numeric_list(values: object, *, name: str) -> list[float]:
    if not isinstance(values, (list, tuple)):
        raise ValueError(f"{name} must be a numeric list")
    result = [float(value) for value in values]
    if not result:
        raise ValueError(f"{name} must not be empty")
    return result


def _numeric_matrix(values: object, *, name: str) -> list[list[float]]:
    if not isinstance(values, (list, tuple)):
        raise ValueError(f"{name} must be a list of numeric lists")
    return [_numeric_list(row, name=name) for row in values]


def _positive_float(value: object, *, name: str) -> float:
    result = float(value)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _emit(payload: dict[str, object], *, as_json: bool) -> int:
    if as_json:
        print(json.dumps(payload, ensure_ascii=False))
    else:
        print(payload["status"])
    return 0


def _attach_output_artifact(
    payload: dict[str, object],
    output: str | None,
    *,
    artifact_key: str = "runtime_log",
) -> dict[str, object]:
    if output is None:
        return payload
    output_path = Path(output)
    payload_with_artifact = dict(payload)
    existing_artifacts = payload_with_artifact.get("artifacts")
    artifacts = dict(existing_artifacts) if isinstance(existing_artifacts, dict) else {}
    artifacts[artifact_key] = str(output_path)
    payload_with_artifact["artifacts"] = artifacts
    output_path.parent.mkdir(parents=True, exist_ok=True)
    _write_json_atomic(output_path, payload_with_artifact)
    return payload_with_artifact


def _serve_runtime_session_until_stopped(
    *,
    session_artifact_path: Path,
    heartbeat_period_s: float,
    max_heartbeat_age_s: float,
    backend: MotionBackend | None = None,
    runtime: ArmRuntime | None = None,
    eef_backends: Mapping[str, MotionBackend] | None = None,
    hold_tick: Callable[[dict[str, object]], None] | None = None,
) -> dict[str, object]:
    if heartbeat_period_s <= 0.0:
        raise ValueError("heartbeat_period_s must be positive")
    try:
        while True:
            payload = _read_json_retry(session_artifact_path)
            if _runtime_stop_requested(session_artifact_path):
                stopped = stop_runtime_session_from_artifact(
                    session_artifact_path=session_artifact_path,
                    max_heartbeat_age_s=max_heartbeat_age_s,
                )
                _write_json_atomic(session_artifact_path, stopped)
                _clear_runtime_stop_request(session_artifact_path)
                return stopped
            if payload.get("status") == "stopped" or payload.get("mode") == "damping":
                return payload
            payload = watchdog_tick_from_artifact(
                session_artifact_path=session_artifact_path,
                max_heartbeat_age_s=max_heartbeat_age_s,
            )
            latest = _read_json_retry(session_artifact_path)
            if latest.get("status") == "stopped" or latest.get("mode") == "damping":
                return latest
            if payload.get("status") == "faulted" or payload.get("mode") == "damping":
                _write_json_atomic(session_artifact_path, payload)
                return payload
            if backend is not None:
                for _ in range(16):
                    executed = execute_pending_runtime_commands(
                        session_artifact_path=session_artifact_path,
                        backend=backend,
                        runtime=runtime,
                        eef_backends=eef_backends,
                        max_heartbeat_age_s=max_heartbeat_age_s,
                    )
                    payload = _read_json_retry(session_artifact_path)
                    if payload.get("status") == "stopped" or payload.get("mode") == "damping":
                        return payload
                    if executed is None:
                        break
                payload = _read_json_retry(session_artifact_path)
                if payload.get("status") == "stopped" or payload.get("mode") == "damping":
                    return payload
            if payload.get("mode") == "hold_safe":
                if hold_tick is not None:
                    hold_tick(payload)
                    if _runtime_stop_requested(session_artifact_path):
                        continue
                if backend is not None:
                    latest = _read_json_retry(session_artifact_path)
                    if latest.get("status") == "stopped" or latest.get("mode") == "damping":
                        return latest
                    if latest.get("mode") == "hold_safe":
                        try:
                            state = backend.read_joint_state()
                        except Exception as error:
                            try:
                                backend.damping()
                            except Exception:
                                pass
                            faulted = dict(latest)
                            faulted["status"] = "faulted"
                            faulted["mode"] = "damping"
                            faulted["owner"] = None
                            faulted["owner_lease"] = None
                            faulted["owner_deadman"] = None
                            faulted["landing_mode"] = "damping"
                            faulted["fault_landing_mode"] = "damping"
                            faulted["error"] = {
                                "type": type(error).__name__,
                                "message": str(error),
                            }
                            faulted["readiness"] = runtime_readiness(faulted)
                            _write_json_atomic(session_artifact_path, faulted)
                            return faulted
                        payload = record_runtime_hold_tick(
                            latest,
                            q_meas=state.q_meas,
                            fault_flags=state.fault_flags,
                            max_heartbeat_age_s=max_heartbeat_age_s,
                        )
                        _write_json_atomic(session_artifact_path, payload)
                else:
                    payload = _read_json_retry(session_artifact_path)
                    if payload.get("status") == "stopped" or payload.get("mode") == "damping":
                        return payload
            payload = heartbeat_runtime_session_payload(
                payload,
                max_heartbeat_age_s=max_heartbeat_age_s,
            )
            latest = _read_json_retry(session_artifact_path)
            if latest.get("status") == "stopped" or latest.get("mode") == "damping":
                return latest
            _write_json_atomic(session_artifact_path, payload)
            time.sleep(float(heartbeat_period_s))
    except KeyboardInterrupt:
        return _interrupt_runtime_session(
            session_artifact_path=session_artifact_path,
            backend=backend,
        )


def _interrupt_runtime_session(
    *,
    session_artifact_path: Path,
    backend: MotionBackend | None,
) -> dict[str, object]:
    if backend is not None:
        try:
            backend.damping()
        except Exception:
            pass
    payload = _read_json_retry(session_artifact_path)
    interrupted = dict(payload)
    interrupted["status"] = "interrupted"
    interrupted["mode"] = "damping"
    interrupted["owner"] = None
    interrupted["owner_lease"] = None
    interrupted["owner_deadman"] = None
    interrupted["landing_mode"] = "damping"
    interrupted["fault_landing_mode"] = "damping"
    interrupted["reason"] = "keyboard_interrupt"
    interrupted["readiness"] = runtime_readiness(interrupted)
    _write_json_atomic(session_artifact_path, interrupted)
    return interrupted


def _runtime_eef_backends_from_args(
    args: argparse.Namespace,
    *,
    q_state: tuple[float, ...],
) -> dict[str, MotionBackend] | None:
    adapters = list(getattr(args, "eef_adapter", []) or [])
    if not adapters:
        return None
    result: dict[str, MotionBackend] = {}
    for adapter in adapters:
        if adapter == "moveit_servo":
            published_commands: list[dict[str, object]] = []
            result[adapter] = MoveItServoRuntimeBackend(
                q_state=q_state,
                publisher=published_commands.append,
            )
        else:  # pragma: no cover - argparse choices keep this unreachable.
            raise ValueError(f"unsupported EEF adapter: {adapter}")
    return result


def _attach_eef_adapter_manager_from_args(
    payload: dict[str, object],
    args: argparse.Namespace,
    *,
    allow_configured_adapters: bool = True,
) -> dict[str, object]:
    updated = dict(payload)
    configured_adapters = (
        tuple(getattr(args, "eef_adapter", []) or ())
        if allow_configured_adapters
        else ()
    )
    updated["eef_adapter_manager"] = eef_adapter_manager_payload(
        primary_backend=str(updated.get("backend") or "unknown"),
        configured_adapters=configured_adapters,
    )
    if getattr(args, "eef_adapter", None) and not allow_configured_adapters:
        manager = dict(updated["eef_adapter_manager"])
        manager["ignored_requested_adapters"] = list(getattr(args, "eef_adapter", []) or [])
        manager["status"] = "unconfigured"
        manager["eef_command_executable"] = False
        adapter_status = manager.get("adapter_status")
        if isinstance(adapter_status, dict):
            manager["adapter_status"] = {
                str(name): {
                    **(dict(status) if isinstance(status, dict) else {}),
                    "configured": False,
                    "executable": False,
                    "hardware_scope": "not_configured_for_real_runtime",
                }
                for name, status in adapter_status.items()
            }
        manager["reason"] = (
            "real hardware runtime requires a concrete mature EEF adapter "
            "manager; CLI fake moveit_servo adapter is only valid for offline "
            "runtime gateway rehearsal"
        )
        updated["eef_adapter_manager"] = manager
    updated["runtime_controller_manager"] = runtime_controller_manager_payload(
        backend=str(updated.get("backend") or "unknown"),
        eef_adapter_manager=updated["eef_adapter_manager"],
    )
    return updated


def _runtime_stop_request_path(session_artifact_path: Path) -> Path:
    return session_artifact_path.with_name(f"{session_artifact_path.name}.stop")


def _write_runtime_stop_request(session_artifact_path: Path) -> None:
    stop_request_path = _runtime_stop_request_path(session_artifact_path)
    stop_request_path.write_text("stop\n", encoding="utf-8")


def _runtime_stop_requested(session_artifact_path: Path) -> bool:
    return _runtime_stop_request_path(session_artifact_path).exists()


def _clear_runtime_stop_request(session_artifact_path: Path) -> None:
    try:
        _runtime_stop_request_path(session_artifact_path).unlink()
    except FileNotFoundError:
        pass


def _live_runtime_from_session_payload(
    *,
    backend: MotionBackend,
    payload: dict[str, object],
) -> ArmRuntime:
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in payload.get("safe_center") or []),
        runtime_session_id=str(payload.get("runtime_session_id")),
    )
    runtime.mark_hold_safe(
        q_hold=tuple(float(value) for value in payload.get("q_hold") or [])
    )
    return runtime


def _read_json_retry(path: Path) -> dict[str, object]:
    last_error: json.JSONDecodeError | None = None
    for _ in range(10):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as error:
            last_error = error
            time.sleep(0.005)
            continue
        if not isinstance(payload, dict):
            raise ValueError(f"{path} does not contain a JSON object")
        return payload
    assert last_error is not None
    raise last_error


def _write_json_atomic(path: Path, payload: dict[str, object]) -> None:
    tmp_path = path.with_name(f"{path.name}.{uuid4().hex}.tmp")
    tmp_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    tmp_path.replace(path)


def _runtime_session_error_payload(error: Exception) -> dict[str, object]:
    if isinstance(error, RuntimeSessionError):
        return error.payload
    return {}


def _read_agent_flow_contract_for_runtime(path: Path) -> dict[str, object]:
    contract = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(contract, dict):
        raise ValueError("agent flow contract file must contain a JSON object")
    if contract.get("schema") != "armctrl.agent_flow_plan.v1":
        raise ValueError("agent flow contract file must use schema armctrl.agent_flow_plan.v1")
    review = contract.get("review")
    if not isinstance(review, dict) or review.get("review_status") != "completed":
        raise RuntimeError("agent flow contract review is not complete")
    sim_preview = review.get("sim_preview")
    safety = sim_preview.get("safety") if isinstance(sim_preview, dict) else None
    if not isinstance(safety, dict) or safety.get("allowed") is not True:
        raise RuntimeError("agent flow contract review is not complete")
    return contract


def _compile_sysid_runtime_command(
    *,
    execution_trajectory_path: Path,
    dof: int,
    sample_hz: float,
    expected_q_start: Sequence[float],
    owner: str,
    start_pose_policy: str,
    send_hz: float | None,
    max_tracking_error_rad: float | None,
    tracking_error_grace_samples: int | None,
    tracking_error_consecutive_samples: int | None,
    max_tau_abs: float | None,
    output_dir: Path,
) -> dict[str, object]:
    if dof <= 0:
        raise ValueError("--dof must be positive")
    q_start = [float(value) for value in expected_q_start]
    if len(q_start) != dof:
        raise ValueError("--expected-q-start length must match --dof")
    if sample_hz <= 0.0:
        raise ValueError("--sample-hz must be positive")
    effective_send_hz = float(send_hz) if send_hz is not None else float(sample_hz)
    if effective_send_hz <= 0.0:
        raise ValueError("--send-hz must be positive")
    q_points = _read_sysid_execution_q_points(execution_trajectory_path, dof=dof)
    dq_points = _read_sysid_execution_dq_points(execution_trajectory_path, dof=dof)
    dq_policy = "preserved"
    if dq_points is None:
        dq_points = _derive_sysid_execution_dq_points(
            q_points,
            sample_hz=float(sample_hz),
        )
        dq_policy = "derived_finite_difference"
    ddq_points = _read_sysid_execution_ddq_points(execution_trajectory_path, dof=dof)
    ddq_policy = "preserved" if ddq_points is not None else "missing"
    artifact_policy = {
        "schema": "armctrl.sysid_runtime_compiler_policy.v1",
        "trajectory_artifact": str(execution_trajectory_path),
        "q_cmd": "preserved",
        "dq_cmd": dq_policy,
        "ddq_cmd": ddq_policy,
        "sample_hz": float(sample_hz),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    command_path = output_dir / "compiled_motion_command.json"
    manifest_path = output_dir / "compile_runtime_manifest.json"
    command: dict[str, object] = {
        "schema": "armctrl.compiled_motion_command.v1",
        "command_surface": MOTION_COMMAND_SURFACE,
        "motion_kind": "joint-trajectory",
        "source": "sysid",
        "owner": str(owner),
        "expected_q_start": q_start,
        "q_points": [list(point) for point in q_points],
        "dq_points": [list(point) for point in dq_points],
        "send_hz": effective_send_hz,
        "trajectory_sample_hz": float(sample_hz),
        "start_pose_policy": str(start_pose_policy),
        "artifact_policy": artifact_policy,
        "source_artifacts": {
            "execution_trajectory": str(execution_trajectory_path),
        },
    }
    if ddq_points is not None:
        command["ddq_points"] = [list(point) for point in ddq_points]
    if max_tracking_error_rad is not None:
        command["max_tracking_error_rad"] = float(max_tracking_error_rad)
        command["tracking_error_grace_samples"] = int(
            3 if tracking_error_grace_samples is None else tracking_error_grace_samples
        )
        command["tracking_error_consecutive_samples"] = int(
            3
            if tracking_error_consecutive_samples is None
            else tracking_error_consecutive_samples
        )
    if max_tau_abs is not None:
        command["max_tau_abs"] = float(max_tau_abs)
    _write_json_atomic(command_path, command)
    payload = {
        "status": "ok",
        "schema": "armctrl.sysid_runtime_compile.v1",
        "command_surface": MOTION_COMMAND_SURFACE,
        "motion_kind": "joint-trajectory",
        "source": "sysid",
        "owner": str(owner),
        "movement_command_sent": False,
        "hardware_motion": False,
        "sample_count": len(q_points),
        "sample_hz": float(sample_hz),
        "send_hz": effective_send_hz,
        "artifact_policy": artifact_policy,
        "artifacts": {
            "compiled_command": str(command_path),
            "compile_runtime_manifest": str(manifest_path),
            "execution_trajectory": str(execution_trajectory_path),
        },
        "next_gate": "submit compiled command with armctrl motion submit joint-trajectory",
    }
    _write_json_atomic(manifest_path, payload)
    return payload


def _read_agent_sysid_readiness(path: Path) -> dict[str, object]:
    readiness = json.loads(path.read_text(encoding="utf-8"))
    return _read_agent_sysid_readiness_from_payload(readiness)


def _read_agent_sysid_readiness_from_payload(
    readiness: dict[str, object],
) -> dict[str, object]:
    if not isinstance(readiness, dict):
        raise ValueError("readiness artifact must contain a JSON object")
    if readiness.get("schema") != "armctrl.arm_runtime_status.v1":
        raise RuntimeError(
            "real Agent runtime smoke requires live arm runtime status readiness"
        )
    runtime_readiness_payload = readiness.get("readiness")
    if (
        not isinstance(runtime_readiness_payload, dict)
        or runtime_readiness_payload.get("agent_sysid_smoke_allowed") is not True
    ):
        raise RuntimeError("live runtime readiness is not passed")
    return readiness


def _runtime_live_hold_start_pose(
    runtime_status_payload: dict[str, object],
    *,
    dof: int,
    max_start_error_rad: float = 0.02,
) -> dict[str, object]:
    q_hold = _runtime_float_list(runtime_status_payload.get("q_hold"), name="q_hold")
    q_meas = _runtime_float_list(runtime_status_payload.get("q_meas"), name="q_meas")
    if len(q_hold) != dof:
        raise ValueError("runtime q_hold length must match --dof")
    if len(q_meas) != dof:
        raise ValueError("runtime q_meas length must match --dof")
    q_error = [measured - held for measured, held in zip(q_meas, q_hold, strict=True)]
    max_abs_error = max((abs(value) for value in q_error), default=0.0)
    if max_abs_error > float(max_start_error_rad):
        raise RuntimeError(
            "current q_meas is not close to live runtime hold pose: "
            f"max_abs_error_rad={max_abs_error:.6f} > "
            f"max_error_rad={float(max_start_error_rad):.6f}"
        )
    return {
        "policy": "live_hold",
        "q_start": q_hold,
        "q_hold": q_hold,
        "q_meas": q_meas,
        "q_start_error_rad": q_error,
        "q_start_error_max_abs_rad": max_abs_error,
        "max_start_error_rad": float(max_start_error_rad),
    }


def _linear_q_points(
    *,
    q_start: Sequence[float],
    q_target: Sequence[float],
    max_joint_step_rad: float,
) -> list[list[float]]:
    start = [float(value) for value in q_start]
    target = [float(value) for value in q_target]
    if not start:
        raise ValueError("q_start must not be empty")
    if len(start) != len(target):
        raise ValueError("q_start and q_target lengths must match")
    if max_joint_step_rad <= 0.0:
        raise ValueError("max_joint_step_rad must be positive")
    max_delta = max(
        (
            abs(target_value - start_value)
            for start_value, target_value in zip(start, target, strict=True)
        ),
        default=0.0,
    )
    step_count = max(1, int(math.ceil(max_delta / float(max_joint_step_rad))))
    return [
        [
            start_value + (target_value - start_value) * (step_index / step_count)
            for start_value, target_value in zip(start, target, strict=True)
        ]
        for step_index in range(step_count + 1)
    ]


def _runtime_float_list(values: object, *, name: str) -> list[float]:
    if not isinstance(values, (list, tuple)):
        raise ValueError(f"runtime {name} must be a numeric list")
    result = [float(value) for value in values]
    if not result:
        raise ValueError(f"runtime {name} must not be empty")
    return result


def _runtime_optional_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _runtime_optional_int(value: object) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _agent_flow_contract_control_period_s(contract: dict[str, object]) -> float:
    eef = contract.get("eef")
    plan = eef.get("plan") if isinstance(eef, dict) else None
    command = plan.get("command") if isinstance(plan, dict) else None
    if not isinstance(command, dict):
        raise ValueError("agent flow contract is missing eef command")
    control_period_s = float(command["control_period_s"])
    if control_period_s <= 0.0:
        raise ValueError("agent flow control_period_s must be positive")
    return control_period_s


def _read_sysid_execution_q_points(path: Path, *, dof: int) -> list[tuple[float, ...]]:
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        rows = list(reader)
    if not rows:
        raise ValueError(f"execution trajectory has no rows: {path}")
    q_points: list[tuple[float, ...]] = []
    for row in rows:
        q_points.append(
            tuple(float(row[f"q_cmd_{joint_index + 1}"]) for joint_index in range(dof))
        )
    return q_points


def _read_sysid_execution_dq_points(
    path: Path,
    *,
    dof: int,
) -> list[tuple[float, ...]] | None:
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        rows = list(reader)
        fieldnames = set(reader.fieldnames or [])
    if not rows:
        raise ValueError(f"execution trajectory has no rows: {path}")
    required = {f"dq_cmd_{joint_index + 1}" for joint_index in range(dof)}
    if not required.issubset(fieldnames):
        return None
    return [
        tuple(float(row[f"dq_cmd_{joint_index + 1}"]) for joint_index in range(dof))
        for row in rows
    ]


def _read_sysid_execution_ddq_points(
    path: Path,
    *,
    dof: int,
) -> list[tuple[float, ...]] | None:
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        rows = list(reader)
        fieldnames = set(reader.fieldnames or [])
    if not rows:
        raise ValueError(f"execution trajectory has no rows: {path}")
    required = {f"ddq_cmd_{joint_index + 1}" for joint_index in range(dof)}
    if not required.issubset(fieldnames):
        return None
    return [
        tuple(float(row[f"ddq_cmd_{joint_index + 1}"]) for joint_index in range(dof))
        for row in rows
    ]


def _derive_sysid_execution_dq_points(
    q_points: Sequence[Sequence[float]],
    *,
    sample_hz: float,
) -> list[tuple[float, ...]]:
    if sample_hz <= 0.0:
        raise ValueError("sample_hz must be positive")
    points = [tuple(float(value) for value in point) for point in q_points]
    if not points:
        return []
    if len(points) == 1:
        return [tuple(0.0 for _ in points[0])]
    dt_s = 1.0 / float(sample_hz)
    velocities: list[tuple[float, ...]] = []
    for index, point in enumerate(points):
        if index == 0:
            left = point
            right = points[index + 1]
            divisor = dt_s
        elif index == len(points) - 1:
            left = points[index - 1]
            right = point
            divisor = dt_s
        else:
            left = points[index - 1]
            right = points[index + 1]
            divisor = 2.0 * dt_s
        velocities.append(
            tuple(
                (right_value - left_value) / divisor
                for left_value, right_value in zip(left, right, strict=True)
            )
        )
    return velocities


def _arx5_active_hold_tick(
    *,
    backend: Arx5SdkJointRuntimeBackend,
    hold_hz: float,
) -> Callable[[dict[str, object]], None]:
    def hold_tick(payload: dict[str, object]) -> None:
        q_hold = payload.get("q_hold")
        if not isinstance(q_hold, list | tuple):
            raise RuntimeError("runtime session has no q_hold for active hold")
        backend.hold_joint_position_for_duration(
            tuple(float(value) for value in q_hold),
            duration_s=1.0 / float(hold_hz),
            hold_hz=float(hold_hz),
        )

    return hold_tick


def _sdk_cartesian_active_hold_tick(
    *,
    backend: Arx5SdkCartesianRuntimeBackend,
    hold_hz: float,
) -> Callable[[dict[str, object]], None]:
    def hold_tick(payload: dict[str, object]) -> None:
        if payload.get("mode") != "hold_safe":
            return
        if hold_hz <= 0.0:
            raise RuntimeError("hold_hz must be positive")
        backend.hold()

    return hold_tick


def _diagnostic_only_sdk_payload(payload: dict[str, object]) -> dict[str, object]:
    annotated = dict(payload)
    annotated["diagnostic_only"] = True
    annotated["formal_runtime_gateway"] = False
    annotated["runtime_gateway_note"] = (
        "Bringup diagnostic only. Formal Agent/SysID/Recipe motion must attach "
        "to a live ArmRuntime session and submit joint_intent or joint_trajectory."
    )
    return annotated


def _attach_sysid_run_manifest(
    payload: dict[str, object],
    output_dir: str,
) -> dict[str, object]:
    manifest_path = Path(output_dir) / "manifest.json"
    payload_with_artifact = dict(payload)
    if "quality" not in payload_with_artifact:
        payload_with_artifact["quality"] = _sysid_run_quality(payload_with_artifact)
    existing_artifacts = payload_with_artifact.get("artifacts")
    artifacts = dict(existing_artifacts) if isinstance(existing_artifacts, dict) else {}
    artifacts["manifest"] = str(manifest_path)
    payload_with_artifact["artifacts"] = artifacts
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(
        json.dumps(payload_with_artifact, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return payload_with_artifact


def _sysid_run_quality(payload: dict[str, object]) -> dict[str, object]:
    status = payload.get("status")
    if status == "queued":
        return {
            "motion_smoke_pass": None,
            "runtime_quality_pass": None,
            "sysid_dataset_ready": False,
            "failure_class": None,
            "stage": "queued_pending_runtime_result",
        }
    if status in {"rejected", "blocked"}:
        return {
            "motion_smoke_pass": False,
            "runtime_quality_pass": False,
            "sysid_dataset_ready": False,
            "failure_class": _sysid_failure_class(payload),
            "stage": "admission",
        }
    return {
        "motion_smoke_pass": None,
        "runtime_quality_pass": None,
        "sysid_dataset_ready": False,
        "failure_class": None,
        "stage": "unknown",
    }


def _sysid_failure_class(payload: dict[str, object]) -> str:
    reason = str(payload.get("reason") or "")
    if "readiness" in reason or "runtime" in reason:
        return "readiness_failure"
    if "safety" in reason or payload.get("safety"):
        return "admission_failure"
    return "admission_failure"


def _runtime_first_sysid_readiness_artifact(
    readiness_artifact: dict[str, object],
    *,
    runtime_session_artifact_path: Path | None = None,
    max_heartbeat_age_s: float = 1.0,
) -> dict[str, object]:
    readiness_artifact = _unwrap_runtime_status_artifact(readiness_artifact)
    if (
        readiness_artifact.get("schema") != "armctrl.arm_runtime_status.v1"
        and runtime_session_artifact_path is None
    ):
        return readiness_artifact
    if runtime_session_artifact_path is not None:
        live_payload = json.loads(
            runtime_session_artifact_path.read_text(encoding="utf-8")
        )
        if live_payload.get("schema") == "armctrl.arm_runtime_session.v1":
            return refresh_runtime_status_payload(
                live_payload,
                max_heartbeat_age_s=max_heartbeat_age_s,
            )
    return refresh_runtime_status_payload(
        readiness_artifact,
        max_heartbeat_age_s=max_heartbeat_age_s,
    )


def _unwrap_runtime_status_artifact(
    payload: dict[str, object],
) -> dict[str, object]:
    if payload.get("schema") == "armctrl.runtime_console_status.v1":
        runtime = payload.get("runtime")
        if isinstance(runtime, dict):
            return runtime
    return payload


def _runtime_result_check_payload(
    *,
    result_artifact_path: Path,
    expect_owner: str | None,
    expect_mode: str | None,
    expect_sample_count: int | None,
    max_jitter_p99_ms: float | None,
    max_tracking_error_rad: float | None,
) -> dict[str, object]:
    result = json.loads(result_artifact_path.read_text(encoding="utf-8"))
    motion = result.get("motion") if isinstance(result.get("motion"), dict) else {}
    timing = result.get("timing") if isinstance(result.get("timing"), dict) else {}
    acceptance = (
        result.get("acceptance") if isinstance(result.get("acceptance"), dict) else {}
    )
    timing_gate = (
        acceptance.get("timing_gate")
        if isinstance(acceptance.get("timing_gate"), dict)
        else {}
    )
    status_publish_gate = (
        acceptance.get("status_publish_gate")
        if isinstance(acceptance.get("status_publish_gate"), dict)
        else {}
    )
    owner = result.get("owner")
    mode = result.get("mode")
    sample_count = _runtime_optional_int(motion.get("sample_count"))
    jitter_p99_ms = _runtime_optional_float(motion.get("send_jitter_ms_p99"))
    tracking = motion.get("tracking_error")
    tracking_max_abs_rad = _runtime_optional_float(
        tracking.get("max_abs_rad") if isinstance(tracking, dict) else None
    )
    checks = {
        "schema": result.get("schema") == "armctrl.arm_runtime_command_result.v1",
        "result_completed": result.get("status") == "completed",
        "motion_completed": motion.get("status") == "completed",
        "acceptance_passed": acceptance.get("status") == "pass",
        "timing_gate_passed": timing_gate.get("status") == "pass",
        "status_publish_gate_passed": status_publish_gate.get("status") == "pass",
        "owner_matches": expect_owner is None or owner == expect_owner,
        "mode_matches": expect_mode is None or mode == expect_mode,
        "sample_count_matches": (
            expect_sample_count is None or sample_count == expect_sample_count
        ),
        "jitter_p99_present": jitter_p99_ms is not None,
        "jitter_p99_within_limit": (
            max_jitter_p99_ms is None
            or (
                jitter_p99_ms is not None
                and jitter_p99_ms <= float(max_jitter_p99_ms)
            )
        ),
        "queue_latency_present": timing.get("queue_latency_s") is not None,
        "first_send_latency_present": timing.get("first_send_latency_s") is not None,
        "execution_elapsed_present": timing.get("execution_elapsed_s") is not None,
        "tracking_error_within_limit": (
            max_tracking_error_rad is None
            or (
                tracking_max_abs_rad is not None
                and tracking_max_abs_rad <= float(max_tracking_error_rad)
            )
        ),
    }
    payload = {
        "status": "pass" if all(checks.values()) else "fail",
        "schema": "armctrl.runtime_result_check.v1",
        "result_artifact": str(result_artifact_path),
        "runtime_session_id": result.get("runtime_session_id"),
        "owner": owner,
        "mode": mode,
        "sample_count": sample_count,
        "checks": checks,
        "metrics": {
            "trajectory_sample_hz": _runtime_optional_float(
                motion.get("trajectory_sample_hz")
            ),
            "runtime_send_hz": _runtime_optional_float(motion.get("runtime_send_hz")),
            "actual_send_hz": _runtime_optional_float(motion.get("actual_send_hz")),
            "send_jitter_ms_p95": _runtime_optional_float(
                motion.get("send_jitter_ms_p95")
            ),
            "send_jitter_ms_p99": jitter_p99_ms,
            "dt_min_s": _runtime_optional_float(motion.get("dt_min_s")),
            "dt_max_s": _runtime_optional_float(motion.get("dt_max_s")),
            "dt_avg_s": _runtime_optional_float(motion.get("dt_avg_s")),
            "queue_latency_s": _runtime_optional_float(timing.get("queue_latency_s")),
            "acquire_latency_s": _runtime_optional_float(
                timing.get("acquire_latency_s")
            ),
            "first_send_latency_s": _runtime_optional_float(
                timing.get("first_send_latency_s")
            ),
            "execution_elapsed_s": _runtime_optional_float(
                timing.get("execution_elapsed_s")
            ),
            "tracking_error_max_abs_rad": tracking_max_abs_rad,
        },
        "policies": {
            "resampling_policy": motion.get("resampling_policy"),
            "interpolation_policy": motion.get("interpolation_policy"),
            "landing_mode": motion.get("landing_mode"),
        },
        "acceptance": {
            "status": acceptance.get("status"),
            "timing_gate": timing_gate.get("status"),
            "status_publish_gate": status_publish_gate.get("status"),
        },
    }
    if payload["status"] != "pass":
        payload["next_gate"] = "inspect runtime result timing and safety evidence"
    return payload


def _runtime_result_artifact_path_from_args(
    *,
    result_artifact: str | None,
    run_dir: str | None,
) -> Path:
    if result_artifact is not None:
        return Path(result_artifact)
    if run_dir is None:
        raise ValueError("provide --result-artifact or --run-dir")
    results_dir = Path(run_dir) / "runtime_session_commands" / "results"
    candidates = [path for path in results_dir.glob("*.json") if path.is_file()]
    if not candidates:
        raise ValueError(f"no runtime result artifacts found under {results_dir}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def _runtime_result_check_all_payload(
    *,
    run_dir: Path | None,
    max_jitter_p99_ms: float | None,
    max_tracking_error_rad: float | None,
    required_owners: Sequence[str],
) -> dict[str, object]:
    if run_dir is None:
        raise ValueError("--all requires --run-dir")
    result_paths = _runtime_result_artifact_paths_for_run_dir(run_dir)
    results = [
        _runtime_result_check_payload(
            result_artifact_path=path,
            expect_owner=None,
            expect_mode=None,
            expect_sample_count=None,
            max_jitter_p99_ms=max_jitter_p99_ms,
            max_tracking_error_rad=max_tracking_error_rad,
        )
        for path in result_paths
    ]
    pass_count = sum(1 for result in results if result.get("status") == "pass")
    fail_count = len(results) - pass_count
    owners_present = sorted(
        str(owner) for owner in {result.get("owner") for result in results} if owner
    )
    required_owner_list = [str(owner) for owner in required_owners]
    missing_required_owners = [
        owner for owner in required_owner_list if owner not in owners_present
    ]
    has_missing_required_owners = bool(missing_required_owners)
    payload = {
        "status": "pass" if fail_count == 0 and not has_missing_required_owners else "fail",
        "schema": "armctrl.runtime_result_check_summary.v1",
        "run_dir": str(run_dir),
        "result_count": len(results),
        "pass_count": pass_count,
        "fail_count": fail_count,
        "owners": [result.get("owner") for result in results],
        "owners_present": owners_present,
        "required_owners": required_owner_list,
        "missing_required_owners": missing_required_owners,
        "results": results,
    }
    if has_missing_required_owners:
        payload["next_gate"] = "run missing runtime owner commands"
    elif fail_count:
        payload["next_gate"] = "inspect failed runtime result artifacts"
    return payload


def _runtime_result_artifact_paths_for_run_dir(run_dir: Path) -> list[Path]:
    results_dir = run_dir / "runtime_session_commands" / "results"
    candidates = sorted(
        (path for path in results_dir.glob("*.json") if path.is_file()),
        key=lambda path: path.name,
    )
    if not candidates:
        raise ValueError(f"no runtime result artifacts found under {results_dir}")
    return candidates


def _sysid_run_readiness_allowed(readiness_artifact: dict[str, object]) -> bool:
    if readiness_artifact.get("schema") != "armctrl.arm_runtime_status.v1":
        return False
    readiness = readiness_artifact.get("readiness")
    return (
        isinstance(readiness, dict)
        and readiness.get("live_hold_allowed") is True
        and readiness.get("safe_center_allowed") is True
    )


def _nonpassing_acceptance_status(payload: dict[str, object]) -> str | None:
    acceptance = payload.get("acceptance")
    if not isinstance(acceptance, dict):
        return None
    status = acceptance.get("status")
    if status is None or status == "pass":
        return None
    return str(status)


def _sysid_readiness_summary(
    *,
    readiness_artifact_path: Path,
    readiness_artifact: dict[str, object],
) -> dict[str, object]:
    readiness = (
        readiness_artifact.get("readiness")
        if isinstance(readiness_artifact.get("readiness"), dict)
        else {}
    )
    return {
        "artifact_path": str(readiness_artifact_path),
        "agent_sysid_smoke_allowed": readiness.get(
            "agent_sysid_smoke_allowed",
            readiness_artifact.get("agent_sysid_smoke_allowed"),
        ),
        "prerequisites": readiness_artifact.get("prerequisites"),
        "tiny_motion": readiness_artifact.get("tiny_motion"),
    }


def _controller_dt_from_readiness_artifact(
    readiness_artifact: dict[str, object],
) -> float | None:
    tiny_motion = readiness_artifact.get("tiny_motion")
    if not isinstance(tiny_motion, dict):
        return None
    try:
        controller_dt_s = float(tiny_motion.get("controller_dt_s"))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(controller_dt_s) or controller_dt_s <= 0.0:
        return None
    return controller_dt_s


def _sysid_solver_gate(dataset_path: Path) -> dict[str, object]:
    manifest_path = dataset_path / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("adapter") != "sdk":
        return {"status": "pass", "checks": {}}
    run_status_completed = manifest.get("run_status") == "completed"
    acceptance = manifest.get("acceptance")
    acceptance_passed = (
        isinstance(acceptance, dict) and acceptance.get("status") == "pass"
    )
    checks = {
        "run_status_completed": run_status_completed,
        "acceptance_passed": acceptance_passed,
    }
    if run_status_completed and acceptance_passed:
        return {"status": "pass", "checks": checks}
    if not run_status_completed:
        reason = f"sdk run_status is {manifest.get('run_status')}"
    else:
        reason = "sdk acceptance is not pass"
    return {
        "status": "blocked",
        "reason": reason,
        "next_gate": "review sysid smoke acceptance before solver",
        "checks": checks,
    }


def _attach_sysid_readiness_to_manifest(
    *,
    manifest_path: Path,
    readiness_summary: dict[str, object],
) -> dict[str, object]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["readiness"] = readiness_summary
    motion_runtime = manifest.get("motion_runtime")
    if isinstance(motion_runtime, dict):
        manifest["acceptance"] = build_real_motion_acceptance(
            stage="sysid_smoke",
            motion_runtime=motion_runtime,
            hardware_motion=bool(manifest.get("safety", {}).get("movement_allowed")),
            movement_command_sent=True,
            readiness_passed=readiness_summary.get("agent_sysid_smoke_allowed") is True,
        )
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return manifest


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
