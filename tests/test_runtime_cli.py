import importlib.util
import json
import subprocess
import sys
import time
from dataclasses import replace
from pathlib import Path

import pytest

from armctrl.motion_runtime import (
    ArmRuntime,
    FakeMotionBackend,
    MotionAuditSample,
    MotionExecutionResult,
    JointStateSnapshot,
    MotionMode,
)
from armctrl.cli import (
    _attach_eef_adapter_manager_from_args,
    _runtime_result_check_payload,
    _runtime_stop_request_path,
    _serve_runtime_session_until_stopped,
)
from armctrl.runtime_session import heartbeat_runtime_session_payload
from armctrl.runtime_session import ARX5_RUNTIME_START_CONFIRMATION
from armctrl.runtime_session import eef_adapter_manager_payload
from armctrl.runtime_session import record_runtime_hold_tick
from armctrl.runtime_session import refresh_runtime_status_payload
from armctrl.runtime_session import runtime_controller_manager_payload
from armctrl.runtime_session import runtime_readiness
from armctrl.runtime_session import RuntimeSessionError
from armctrl.runtime_session import start_fake_runtime_session, stop_runtime_session_from_artifact
from armctrl.runtime_session import start_arx5_runtime_session
from armctrl.runtime_session import watchdog_tick_from_artifact
from armctrl.runtime_ipc import (
    execute_pending_runtime_commands,
    runtime_command_queue_dir,
    submit_eef_command,
    submit_intent_command,
    submit_trajectory_command,
)
from armctrl.arx5_sdk_cartesian_runtime import Arx5SdkCartesianRuntimeBackend
from armctrl.moveit_servo_runtime import MoveItServoRuntimeBackend


def _passing_doctor_artifact() -> dict[str, object]:
    return {
        "schema": "armctrl.sysid_sdk_doctor.v1",
        "interface_status": {"status": "opened_read_only"},
        "controller": {"status": "initialized_read_only", "controller_dt_s": 0.002},
        "state_read": {"status": "ok", "state_read_hz": 100.0},
        "timestamp_policy": {"monotonic": True},
        "motion_commands_sent": False,
    }


def _passing_hold_damping_artifact() -> dict[str, object]:
    return {
        "schema": "armctrl.sysid_sdk_hold_damping_check.v1",
        "landing_policy": "damping_only",
        "hold": {"status": "unsupported", "method": "set_to_hold"},
        "damping": {"status": "called", "method": "set_to_damping"},
        "mode_call_sequence": [
            {"name": "hold", "method": "set_to_hold", "status": "unsupported"},
            {"name": "damping", "method": "set_to_damping", "status": "called"},
        ],
        "joint_commands_sent": False,
        "hold_damping_gate": {
            "status": "pass",
            "checks": {
                "damping_called": True,
                "landing_policy_supported": True,
                "mode_call_sequence_recorded": True,
                "mode_call_sequence_safe_order": True,
                "no_joint_commands_sent": True,
            },
            "failed_checks": [],
        },
    }


def test_cli_runtime_submit_eef_pose_delta_blocks_without_ready_adapter(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    output_artifact = tmp_path / "eef_submit.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "submit-eef",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--backend",
            "moveit_servo",
            "--kind",
            "eef_pose_delta",
            "--frame",
            "eef_link",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--delta-position",
            "0.002",
            "0.0",
            "0.0",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.0",
            "--control-period-s",
            "0.1",
            "--send-hz",
            "50",
            "--output",
            str(output_artifact),
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    queue_dir = session_artifact.parent / "runtime_session_commands" / "pending"

    assert completed.returncode == 3
    assert payload["status"] == "blocked"
    assert payload["owner"] == "agent"
    assert payload["mode"] == "agent_servo"
    assert payload["movement_command_sent"] is False
    assert payload["hardware_executable_now"] is False
    assert payload["command_space"] == "eef"
    assert payload["eef_adapter_manager"]["status"] == "unconfigured"
    assert payload["eef_adapter_manager"]["eef_command_executable"] is False
    assert "configured mature EEF backend adapter" in payload["reason"]
    assert not queue_dir.exists()


def test_cli_motion_submit_eef_delta_blocks_without_ready_adapter(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    output_artifact = tmp_path / "eef_submit.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "eef-delta",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--backend",
            "moveit_servo",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--delta-position",
            "0.002",
            "0.0",
            "0.0",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.0",
            "--control-period-s",
            "0.1",
            "--send-hz",
            "50",
            "--output",
            str(output_artifact),
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    queue_dir = session_artifact.parent / "runtime_session_commands" / "pending"

    assert completed.returncode == 3
    assert payload["status"] == "blocked"
    assert payload["movement_command_sent"] is False
    assert payload["hardware_executable_now"] is False
    assert payload["eef_adapter_manager"]["status"] == "unconfigured"
    assert payload["eef_adapter_manager"]["eef_command_executable"] is False
    assert "configured mature EEF backend adapter" in payload["reason"]
    assert not queue_dir.exists()


def test_cli_motion_submit_joint_intent_queues_runtime_command(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "joint-intent",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-target",
            "0.004",
            "0.3",
            "0.3",
            "--control-period-s",
            "0.1",
            "--send-hz",
            "50",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))

    assert payload["status"] == "queued"
    assert payload["command_surface"] == "armctrl.motion.submit.v1"
    assert payload["motion_kind"] == "joint-intent"
    assert payload["legacy_equivalent"] == "armctrl runtime submit-intent"
    assert command["kind"] == "joint_intent"
    assert command["command_surface"] == "armctrl.motion.submit.v1"
    assert command["motion_kind"] == "joint-intent"
    assert command["max_joint_velocity_rad_s"] == pytest.approx(0.25)
    assert command["joint_intent_safety"]["status"] == "pass"
    assert command["intent_trajectory_contract"]["schema"] == (
        "armctrl.joint_intent_trajectory_contract.v1"
    )
    assert command["intent_trajectory_contract"]["q_policy"] == (
        "live_hold_to_target_smoothstep"
    )
    assert command["intent_trajectory_contract"]["dq_policy"] == (
        "derived_smoothstep_analytic"
    )
    assert command["intent_trajectory_contract"]["runtime_send_hz"] == pytest.approx(50.0)
    assert command["intent_trajectory_contract"]["expected_runtime_sample_count"] == 6
    assert command["intent_trajectory_contract"]["q_points"][0] == [0.0, 0.3, 0.3]
    assert command["intent_trajectory_contract"]["q_points"][-1] == [0.004, 0.3, 0.3]
    assert len(command["intent_trajectory_contract"]["dq_points"]) == 6
    assert len(command["intent_trajectory_contract"]["ddq_points"]) == 6
    assert command["intent_trajectory_contract"]["dq_points"][0] == [0.0, 0.0, 0.0]
    assert command["intent_trajectory_contract"]["dq_points"][-1] == [0.0, 0.0, 0.0]


def test_cli_motion_submit_joint_intent_rejects_overfast_visible_step(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "joint-intent",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-target",
            "0.0",
            "0.5",
            "0.3",
            "--control-period-s",
            "0.1",
            "--send-hz",
            "50",
            "--max-joint-delta-rad",
            "0.20",
            "--max-joint-velocity-rad-s",
            "0.25",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["motion_kind"] == "joint-intent"
    assert "joint_velocity_within_limit" in payload["reason"]


def test_cli_motion_submit_joint_intent_rejects_too_abrupt_smoothstep(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "joint-intent",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-target",
            "0.01",
            "0.3",
            "0.3",
            "--control-period-s",
            "0.1",
            "--send-hz",
            "50",
            "--max-joint-delta-rad",
            "0.02",
            "--max-joint-velocity-rad-s",
            "0.25",
            "--max-joint-acceleration-rad-s2",
            "5.0",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["motion_kind"] == "joint-intent"
    assert "joint_acceleration_within_limit" in payload["reason"]


def test_cli_motion_submit_joint_intent_allows_visible_slow_sweep_with_shape_policy(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "joint-intent",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-target",
            "0.8",
            "0.3",
            "0.3",
            "--control-period-s",
            "12.0",
            "--send-hz",
            "50",
            "--max-joint-delta-rad",
            "0.85",
            "--max-joint-velocity-rad-s",
            "0.08",
            "--max-joint-acceleration-rad-s2",
            "0.05",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))

    assert payload["status"] == "queued"
    assert command["kind"] == "joint_intent"
    assert command["joint_intent_safety"]["status"] == "pass"
    assert command["max_joint_velocity_rad_s"] == pytest.approx(0.08)
    assert command["interpolation_policy"] == "smoothstep_intent_frame"
    assert command["resampling_policy"] == "intent_frame_to_runtime_send_hz"
    assert command["expected_runtime_sample_count"] == 601
    assert payload["expected_runtime_sample_count"] == 601
    assert command["intent_trajectory_contract"]["max_joint_velocity_rad_s"] == pytest.approx(0.08)
    assert command["max_joint_acceleration_rad_s2"] == pytest.approx(0.05)
    assert command["joint_intent_safety"]["max_joint_acceleration_rad_s2"] == (
        pytest.approx(0.05)
    )
    assert command["intent_trajectory_contract"]["max_joint_acceleration_rad_s2"] == (
        pytest.approx(0.05)
    )
    assert command["intent_trajectory_contract"]["max_abs_velocity_rad_s"] < 0.08
    assert (
        command["intent_trajectory_contract"]["max_abs_acceleration_rad_s2"] < 0.05
    )


def test_cli_motion_submit_joint_trajectory_queues_runtime_command(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "joint-trajectory",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.002",
            "0.3",
            "0.3",
            "--send-hz",
            "50",
            "--trajectory-sample-hz",
            "50",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))

    assert payload["status"] == "queued"
    assert payload["command_surface"] == "armctrl.motion.submit.v1"
    assert payload["motion_kind"] == "joint-trajectory"
    assert payload["legacy_equivalent"] == "armctrl runtime submit-trajectory"
    assert command["kind"] == "joint_trajectory"
    assert command["command_surface"] == "armctrl.motion.submit.v1"
    assert command["motion_kind"] == "joint-trajectory"


def test_cli_motion_submit_agent_joint_trajectory_rejects_overfast_waypoints(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "joint-trajectory",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.50",
            "0.3",
            "0.3",
            "--send-hz",
            "50",
            "--trajectory-sample-hz",
            "50",
            "--max-joint-velocity-rad-s",
            "0.25",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["motion_kind"] == "joint-trajectory"
    assert "joint_velocity_within_limit" in payload["reason"]


def test_cli_motion_submit_agent_joint_trajectory_records_safety_summary(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "joint-trajectory",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.60",
            "0.3",
            "0.3",
            "--send-hz",
            "50",
            "--trajectory-sample-hz",
            "1",
            "--max-joint-segment-delta-rad",
            "0.65",
            "--max-joint-velocity-rad-s",
            "0.75",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))

    assert payload["status"] == "queued"
    assert payload["joint_trajectory_safety"]["status"] == "pass"
    assert command["joint_trajectory_safety"]["schema"] == (
        "armctrl.joint_trajectory_safety.v1"
    )
    assert command["joint_trajectory_safety"]["max_joint_velocity_rad_s"] == (
        pytest.approx(0.75)
    )
    assert command["joint_trajectory_safety"]["max_segment_abs_velocity_rad_s"] == (
        pytest.approx(0.6)
    )
    assert command["joint_trajectory_safety"]["max_segment_abs_delta_rad"] == (
        pytest.approx(0.6)
    )


def test_cli_motion_compile_agent_joint_target_writes_shared_joint_trajectory_contract(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "compiled-agent-target"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "compile",
            "joint-trajectory",
            "--source",
            "agent",
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-target",
            "0.45",
            "0.3",
            "0.3",
            "--duration-s",
            "6.0",
            "--sample-hz",
            "50",
            "--send-hz",
            "100",
            "--max-joint-segment-delta-rad",
            "0.02",
            "--max-joint-velocity-rad-s",
            "0.25",
            "--max-joint-acceleration-rad-s2",
            "0.25",
            "--max-tracking-error-rad",
            "0.04",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(
        Path(payload["artifacts"]["compiled_command"]).read_text(encoding="utf-8")
    )

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.motion_runtime_compile.v1"
    assert payload["command_surface"] == "armctrl.motion.submit.v1"
    assert payload["motion_kind"] == "joint-trajectory"
    assert payload["source"] == "agent"
    assert payload["owner"] == "agent"
    assert payload["sample_count"] == 301
    assert command["schema"] == "armctrl.compiled_motion_command.v1"
    assert command["source"] == "agent"
    assert command["owner"] == "agent"
    assert command["expected_q_start"] == [0.0, 0.3, 0.3]
    assert command["q_points"][0] == [0.0, 0.3, 0.3]
    assert command["q_points"][-1] == [0.45, 0.3, 0.3]
    assert len(command["q_points"]) == 301
    assert len(command["dq_points"]) == 301
    assert len(command["ddq_points"]) == 301
    assert command["dq_points"][0] == pytest.approx([0.0, 0.0, 0.0])
    assert command["dq_points"][-1] == pytest.approx([0.0, 0.0, 0.0])
    assert command["tracking_error_grace_samples"] == 3
    assert command["tracking_error_consecutive_samples"] == 3
    assert command["artifact_policy"]["schema"] == "armctrl.joint_trajectory_compiler_policy.v1"
    assert command["artifact_policy"]["q_cmd"] == "generated_smoothstep_joint_target"
    assert command["artifact_policy"]["dq_cmd"] == "derived_smoothstep_analytic"
    assert command["artifact_policy"]["ddq_cmd"] == "derived_smoothstep_analytic"
    assert command["interpolation_policy"] == "smoothstep_joint_target"
    assert command["resampling_policy"] == "compiled_joint_trajectory_to_runtime_send_hz"
    assert command["joint_trajectory_safety"]["status"] == "pass"
    assert command["joint_trajectory_safety"]["max_joint_acceleration_rad_s2"] == (
        pytest.approx(0.25)
    )
    assert command["joint_trajectory_safety"][
        "max_segment_abs_acceleration_rad_s2"
    ] <= 0.25


def test_cli_motion_compile_agent_waypoints_derives_missing_dq_ddq_with_policy(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "compiled-agent-waypoints"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "compile",
            "joint-trajectory",
            "--source",
            "agent",
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.02",
            "0.3",
            "0.3",
            "--q-point",
            "0.04",
            "0.3",
            "0.3",
            "--sample-hz",
            "10",
            "--send-hz",
            "50",
            "--max-joint-velocity-rad-s",
            "0.25",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(
        Path(payload["artifacts"]["compiled_command"]).read_text(encoding="utf-8")
    )

    assert payload["status"] == "ok"
    assert command["q_points"] == [
        [0.0, 0.3, 0.3],
        [0.02, 0.3, 0.3],
        [0.04, 0.3, 0.3],
    ]
    assert command["dq_points"][0] == pytest.approx([0.2, 0.0, 0.0])
    assert command["dq_points"][1] == pytest.approx([0.2, 0.0, 0.0])
    assert command["ddq_points"][1] == pytest.approx([0.0, 0.0, 0.0])
    assert command["artifact_policy"]["q_cmd"] == "preserved_waypoints"
    assert command["artifact_policy"]["dq_cmd"] == (
        "derived_finite_difference_for_cubic_hermite_runtime"
    )
    assert command["artifact_policy"]["ddq_cmd"] == (
        "derived_finite_difference_for_safety_evidence"
    )
    assert command["interpolation_policy"] == "cubic_hermite_joint_waypoints"
    assert command["resampling_policy"] == "compiled_joint_waypoints_to_runtime_send_hz"
    assert command["runtime_trajectory_contract"]["schema"] == (
        "armctrl.joint_trajectory_contract.v1"
    )
    assert command["runtime_trajectory_contract"]["runtime_interpolator"] == (
        "cubic_hermite_joint_position_velocity"
    )


def test_cli_motion_compile_agent_waypoints_rejects_high_acceleration(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "compiled-agent-jerky-waypoints"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "compile",
            "joint-trajectory",
            "--source",
            "agent",
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.02",
            "0.3",
            "0.3",
            "--q-point",
            "0.0205",
            "0.3",
            "0.3",
            "--sample-hz",
            "10",
            "--max-joint-velocity-rad-s",
            "0.25",
            "--max-joint-acceleration-rad-s2",
            "0.5",
            "--output",
            str(output_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert "joint_acceleration_within_limit" in payload["reason"]


def test_cli_motion_compile_agent_waypoints_rejects_direction_reversal_acceleration(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "compiled-agent-reversal-waypoints"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "compile",
            "joint-trajectory",
            "--source",
            "agent",
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.02",
            "0.3",
            "0.3",
            "--q-point",
            "0.0",
            "0.3",
            "0.3",
            "--sample-hz",
            "10",
            "--max-joint-velocity-rad-s",
            "0.25",
            "--max-joint-acceleration-rad-s2",
            "0.5",
            "--output",
            str(output_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert "joint_acceleration_within_limit" in payload["reason"]


def test_runtime_revalidates_agent_joint_trajectory_safety_before_execute(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="agent",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.004, 0.3, 0.3)],
        send_hz=50.0,
        trajectory_sample_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
        max_joint_segment_delta_rad=0.02,
        max_joint_velocity_rad_s=0.25,
    )
    command_path = Path(submitted["artifacts"]["command"])
    command = json.loads(command_path.read_text(encoding="utf-8"))
    command["q_points"] = [[0.0, 0.3, 0.3], [0.80, 0.3, 0.3]]
    command.pop("joint_trajectory_safety", None)
    command_path.write_text(json.dumps(command), encoding="utf-8")

    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "rejected"
    assert "joint trajectory safety failed" in result["reason"]
    assert "joint_velocity_within_limit" in result["reason"]
    assert result["movement_command_sent"] is False
    assert not any(
        command.producer == "agent"
        and command.mode == MotionMode.TRAJECTORY_REPLAY.value
        for command in backend.joint_commands
    )


def test_runtime_revalidates_agent_joint_trajectory_direction_reversal_acceleration(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="agent",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.004, 0.3, 0.3)],
        send_hz=50.0,
        trajectory_sample_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
        max_joint_segment_delta_rad=0.1,
        max_joint_velocity_rad_s=0.25,
        max_joint_acceleration_rad_s2=0.5,
    )
    command_path = Path(submitted["artifacts"]["command"])
    command = json.loads(command_path.read_text(encoding="utf-8"))
    command["q_points"] = [
        [0.0, 0.3, 0.3],
        [0.02, 0.3, 0.3],
        [0.0, 0.3, 0.3],
    ]
    command["trajectory_sample_hz"] = 10.0
    command.pop("joint_trajectory_safety", None)
    command_path.write_text(json.dumps(command), encoding="utf-8")

    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "rejected"
    assert "joint trajectory safety failed" in result["reason"]
    assert "joint_acceleration_within_limit" in result["reason"]
    assert result["movement_command_sent"] is False


def test_runtime_revalidates_agent_joint_intent_safety_before_execute(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    submitted = submit_intent_command(
        session_artifact_path=session_artifact,
        owner="agent",
        expected_q_start=(0.0, 0.3, 0.3),
        q_target=(0.004, 0.3, 0.3),
        control_period_s=0.1,
        send_hz=50.0,
        max_joint_delta_rad=0.02,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
        max_joint_velocity_rad_s=0.25,
    )
    command_path = Path(submitted["artifacts"]["command"])
    command = json.loads(command_path.read_text(encoding="utf-8"))
    command["q_target"] = [0.80, 0.3, 0.3]
    command.pop("joint_intent_safety", None)
    command_path.write_text(json.dumps(command), encoding="utf-8")

    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "rejected"
    assert "joint intent safety failed" in result["reason"]
    assert "joint_velocity_within_limit" in result["reason"]
    assert result["movement_command_sent"] is False
    assert not any(
        command.producer == "agent"
        and command.mode == MotionMode.AGENT_SERVO.value
        for command in backend.joint_commands
    )


def test_runtime_revalidates_agent_joint_intent_acceleration_before_execute(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    submitted = submit_intent_command(
        session_artifact_path=session_artifact,
        owner="agent",
        expected_q_start=(0.0, 0.3, 0.3),
        q_target=(0.004, 0.3, 0.3),
        control_period_s=0.1,
        send_hz=50.0,
        max_joint_delta_rad=0.02,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
        max_joint_velocity_rad_s=0.25,
        max_joint_acceleration_rad_s2=3.0,
    )
    command_path = Path(submitted["artifacts"]["command"])
    command = json.loads(command_path.read_text(encoding="utf-8"))
    command["q_target"] = [0.01, 0.3, 0.3]
    command.pop("joint_intent_safety", None)
    command_path.write_text(json.dumps(command), encoding="utf-8")

    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "rejected"
    assert "joint intent safety failed" in result["reason"]
    assert "joint_acceleration_within_limit" in result["reason"]
    assert result["movement_command_sent"] is False
    assert not any(
        command.producer == "agent"
        and command.mode == MotionMode.AGENT_SERVO.value
        for command in backend.joint_commands
    )


def test_cli_motion_submit_joint_trajectory_accepts_compiled_sysid_command(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    compiled_command = tmp_path / "compiled-sysid-command.json"
    compiled_command.write_text(
        json.dumps(
            {
                "schema": "armctrl.compiled_motion_command.v1",
                "command_surface": "armctrl.motion.submit.v1",
                "motion_kind": "joint-trajectory",
                "source": "sysid",
                "owner": "sysid",
                "expected_q_start": [0.0, 0.3, 0.3],
                "q_points": [
                    [0.0, 0.3, 0.3],
                    [0.002, 0.3, 0.3],
                    [0.004, 0.3, 0.3],
                ],
                "dq_points": [
                    [0.0, 0.0, 0.0],
                    [0.1, 0.0, 0.0],
                    [0.1, 0.0, 0.0],
                ],
                "send_hz": 100.0,
                "trajectory_sample_hz": 100.0,
                "start_pose_policy": "safe_center",
                "artifact_policy": {
                    "schema": "armctrl.sysid_runtime_compiler_policy.v1",
                    "q_cmd": "preserved",
                    "dq_cmd": "preserved",
                    "ddq_cmd": "missing",
                    "sample_hz": 100.0,
                },
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "joint-trajectory",
            "--session-artifact",
            str(session_artifact),
            "--compiled-command",
            str(compiled_command),
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))

    assert payload["status"] == "queued"
    assert payload["owner"] == "sysid"
    assert payload["command_surface"] == "armctrl.motion.submit.v1"
    assert payload["motion_kind"] == "joint-trajectory"
    assert payload["compiled_command_artifact"] == str(compiled_command)
    assert command["owner"] == "sysid"
    assert command["source"] == "sysid"
    assert command["q_points"] == [
        [0.0, 0.3, 0.3],
        [0.002, 0.3, 0.3],
        [0.004, 0.3, 0.3],
    ]
    assert command["dq_points"] == [
        [0.0, 0.0, 0.0],
        [0.1, 0.0, 0.0],
        [0.1, 0.0, 0.0],
    ]
    assert command["artifact_policy"]["dq_cmd"] == "preserved"

    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=clock.monotonic(),
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=5.0,
    )
    assert result is not None
    assert result["status"] == "completed"
    assert result["owner"] == "sysid"
    assert result["mode"] == "trajectory_replay"
    result_check = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "result",
            "--result-artifact",
            payload["artifacts"]["result"],
            "--expect-owner",
            "sysid",
            "--expect-mode",
            "trajectory_replay",
            "--expect-sample-count",
            "3",
            "--max-jitter-p99-ms",
            "6",
            "--max-tracking-error-rad",
            "0.05",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    checked = json.loads(result_check.stdout)
    assert checked["status"] == "pass"


def test_cli_motion_submit_eef_delta_queues_sdk_cartesian_command(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _configure_eef_adapter_manager(session_artifact, adapter="sdk_cartesian")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "eef-delta",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--backend",
            "sdk_cartesian",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--delta-position",
            "0.001",
            "0.0",
            "0.0",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.0",
            "--start-pose-policy",
            "live_hold",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))

    assert payload["status"] == "queued"
    assert payload["command_surface"] == "armctrl.motion.submit.v1"
    assert payload["motion_kind"] == "eef-delta"
    assert payload["backend"] == "sdk_cartesian"
    assert command["kind"] == "eef_pose_delta"
    assert command["backend"] == "sdk_cartesian"
    assert command["motion_kind"] == "eef-delta"


def test_cli_motion_submit_eef_delta_live_hold_requires_safe_center(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = _configure_eef_adapter_manager(
        session_artifact,
        adapter="sdk_cartesian",
    )
    non_safe_hold = [1.2, 0.0, 0.0]
    session["q_hold"] = non_safe_hold
    session["q_meas"] = non_safe_hold
    session["last_hold_wall_time_s"] = time.time()
    heartbeat = dict(session["heartbeat"])
    heartbeat["wall_time_s"] = time.time()
    session["heartbeat"] = heartbeat
    session_artifact.write_text(
        json.dumps(session, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "eef-delta",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--backend",
            "sdk_cartesian",
            "--expected-q-start",
            "1.2",
            "0.0",
            "0.0",
            "--delta-position",
            "0.001",
            "0.0",
            "0.0",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.0",
            "--start-pose-policy",
            "live_hold",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    queue_dir = runtime_command_queue_dir(session_artifact)

    assert completed.returncode == 3
    assert payload["status"] == "blocked"
    assert payload["start_pose_policy"] == "live_hold"
    assert payload["start_pose_guard"]["safe_center_required"] is True
    assert "q_hold_close_to_safe_center" in payload["start_pose_guard"]["failed_checks"]
    assert "SAFE_CENTER" in payload["reason"]
    assert not queue_dir.exists()


def test_cli_motion_submit_eef_delta_blocks_non_executable_adapter_status(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = _configure_eef_adapter_manager(
        session_artifact,
        adapter="sdk_cartesian",
    )
    manager = dict(session["eef_adapter_manager"])
    adapter_status = dict(manager["adapter_status"])
    sdk_status = dict(adapter_status["sdk_cartesian"])
    sdk_status["executable"] = False
    sdk_status["reason"] = "adapter warmup not configured"
    adapter_status["sdk_cartesian"] = sdk_status
    manager["adapter_status"] = adapter_status
    session["eef_adapter_manager"] = manager
    session_artifact.write_text(
        json.dumps(session, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "eef-delta",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--backend",
            "sdk_cartesian",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--delta-position",
            "0.001",
            "0.0",
            "0.0",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.0",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "blocked"
    assert payload["movement_command_sent"] is False
    assert payload["hardware_executable_now"] is False
    assert "adapter_executable=False" in payload["reason"]
    assert not (session_artifact.parent / "runtime_session_commands").exists()


def test_cli_motion_submit_eef_twist_queues_moveit_servo_command(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _configure_eef_adapter_manager(
        session_artifact,
        adapter="moveit_servo",
        live_reference=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "eef-twist",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--backend",
            "moveit_servo",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--linear",
            "0.001",
            "0.0",
            "0.0",
            "--angular",
            "0.0",
            "0.0",
            "0.0",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))

    assert payload["status"] == "queued"
    assert payload["command_surface"] == "armctrl.motion.submit.v1"
    assert payload["motion_kind"] == "eef-twist"
    assert payload["backend"] == "moveit_servo"
    assert command["kind"] == "eef_twist"
    assert command["eef_command"]["linear_mps"] == [0.001, 0.0, 0.0]
    assert command["motion_kind"] == "eef-twist"


def test_cli_motion_submit_eef_pose_queues_runtime_command(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _configure_eef_adapter_manager(
        session_artifact,
        adapter="moveit_servo",
        live_reference=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "eef-pose",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--frame",
            "base_link",
            "--position",
            "0.30",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))

    assert completed.returncode == 0
    assert payload["status"] == "queued"
    assert payload["command_surface"] == "armctrl.motion.submit.v1"
    assert payload["motion_kind"] == "eef-pose"
    assert payload["command_space"] == "eef"
    assert payload["movement_command_sent"] is False
    assert command["kind"] == "eef_pose"
    assert command["eef_command"]["frame"] == "base_link"
    assert command["eef_command"]["position_m"] == [0.3, 0.0, 0.2]
    assert command["eef_command"]["rpy_rad"] == [0.0, 0.0, 0.0]
    assert command["eef_command"]["pose_reference_limiter"] == (
        "adapter_live_reference_limit"
    )
    assert payload["mature_backend_policy"]["disconnected_takeover_allowed"] is False
    assert command["mature_backend_policy"]["no_heuristic_joint_fallback"] is True


def test_cli_motion_submit_eef_pose_blocks_without_adapter_live_reference(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _configure_eef_adapter_manager(session_artifact, adapter="moveit_servo")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "eef-pose",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--frame",
            "base_link",
            "--position",
            "0.30",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "blocked"
    assert payload["movement_command_sent"] is False
    assert payload["hardware_executable_now"] is False
    assert payload["eef_adapter_manager"]["eef_command_capabilities"]["eef_pose"][
        "executable"
    ] is False
    assert "requires_live_reference" in payload["reason"]
    assert not (session_artifact.parent / "runtime_session_commands" / "pending").exists()


def test_cli_motion_submit_joint_jog_rejects_until_deadman_backend_exists(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "motion",
            "submit",
            "joint-jog",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "teleop",
            "--joint-delta",
            "0.001",
            "0.0",
            "0.0",
            "--velocity",
            "0.01",
            "0.0",
            "0.0",
            "--deadman",
            "--json",
        ],
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["command_surface"] == "armctrl.motion.submit.v1"
    assert payload["motion_kind"] == "joint-jog"
    assert payload["owner"] == "teleop"
    assert payload["requested_command"] == {
        "kind": "joint_jog",
        "joint_delta_rad": [0.001, 0.0, 0.0],
        "velocity_rad_s": [0.01, 0.0, 0.0],
        "deadman": True,
    }
    assert "requires a deadman" in payload["reason"]


def test_cli_profile_list_and_show_builtin_profiles() -> None:
    listed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "profile",
            "list",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    list_payload = json.loads(listed.stdout)
    profile_names = {profile["name"] for profile in list_payload["profiles"]}

    assert list_payload["status"] == "ok"
    assert {"lab-sysid", "lab-agent-eef", "teleop", "recipe"} <= profile_names

    shown = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "profile",
            "show",
            "lab-agent-eef",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    show_payload = json.loads(shown.stdout)

    assert show_payload["status"] == "ok"
    assert show_payload["profile"] == "lab-agent-eef"
    assert show_payload["runtime_backend"] == "arx5_sdk"
    assert show_payload["start_pose_policy"] == "controlled_eef_ready_hold"
    assert "eef-delta" in show_payload["supported_motion_kinds"]
    assert "eef-pose" in show_payload["supported_motion_kinds"]


def test_cli_console_status_exposes_runtime_profiles_and_motion_surface(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "console",
            "status",
            "--session-artifact",
            str(session_artifact),
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    profile_names = {profile["name"] for profile in payload["profiles"]}

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.runtime_console_status.v1"
    assert payload["runtime"]["schema"] == "armctrl.arm_runtime_status.v1"
    assert {"lab-sysid", "lab-agent-eef", "recipe", "teleop"} <= profile_names
    assert payload["motion_surface"]["submit_schema"] == "armctrl.motion.submit.v1"
    assert "joint-trajectory" in payload["motion_surface"]["supported_submit_kinds"]
    assert "eef-pose" not in payload["motion_surface"]["hardware_not_implemented_yet"]
    assert payload["motion_surface"]["reserved_contract_kinds"]["eef-pose"] == (
        "absolute EEF pose runtime command; requires a mature pose adapter "
        "with live reference limiting"
    )


def test_cli_console_catalog_exposes_profiles_without_runtime() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "console",
            "catalog",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    profile_names = {profile["name"] for profile in payload["profiles"]}

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.runtime_console_catalog.v1"
    assert {"lab-sysid", "lab-agent-eef", "lab-agent-joint", "recipe", "teleop"} <= profile_names
    assert payload["motion_surface"]["submit_schema"] == "armctrl.motion.submit.v1"
    assert "armctrl motion submit joint-intent" in payload["motion_surface"]["formal_cli"]
    assert payload["motion_surface"]["eef_control_policy"]["disconnected_takeover_allowed"] is False
    assert payload["motion_surface"]["eef_control_policy"]["bumpless_switch_required"] is True
    assert payload["motion_surface"]["eef_control_policy"]["reference_limit_policy"] == {
        "policy": "reject_oversized_eef_reference_step",
        "default_max_linear_step_m": 0.005,
        "default_max_angular_step_rad": 0.05,
        "clamping": False,
        "enforced_at": ["submit", "execute"],
    }
    assert payload["operator_surfaces"]["submit_motion"] == "armctrl motion submit <kind> ..."
    assert payload["legacy_policy"]["formal_control_surface"] == "motion/profile/console"
    assert payload["legacy_policy"]["sysid_run_sdk"] == (
        "parser-level removed; sysid run only accepts --adapter {fake}; "
        "use sysid compile-runtime plus motion submit joint-trajectory"
    )
    assert payload["command_classes"]["formal"] == [
        "console",
        "profile",
        "runtime",
        "motion",
    ]
    assert "armctrl sysid compile-runtime" in payload["command_classes"]["compiler"]
    assert "armctrl sysid run ... --adapter sdk" not in payload["command_classes"]["compiler"]
    assert "armctrl sysid run ... --adapter sdk" in payload["command_classes"]["removed"]
    assert "armctrl sysid sdk-doctor" in payload["command_classes"]["read_only_diagnostic"]
    assert "armctrl sysid sdk-jog-real" in payload["command_classes"]["hardware_diagnostic_only"]
    assert (
        "armctrl sysid sdk-tiny-motion-execute-real"
        in payload["command_classes"]["hardware_diagnostic_only"]
    )
    assert "armctrl sysid sdk-jog-real" not in payload["command_classes"]["formal"]
    assert "armctrl runtime submit-trajectory" in payload["command_classes"]["legacy_alias"]
    assert (
        "armctrl sysid sdk-agent-sysid-smoke-readiness "
        "--runtime-status-artifact <live_runtime_status.json>"
        in payload["command_classes"]["legacy_compatibility_wrapper"]
    )
    assert (
        "armctrl sysid sdk-agent-sysid-smoke-readiness"
        not in payload["command_classes"]["read_only_diagnostic"]
    )
    assert payload["command_classes"]["diagnostic"] == [
        "see read_only_diagnostic and hardware_diagnostic_only"
    ]


def test_cli_runtime_submit_eef_accepts_sdk_cartesian_backend(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _configure_eef_adapter_manager(session_artifact, adapter="sdk_cartesian")

    payload = submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="sdk_cartesian",
        kind="eef_pose_delta",
        frame="eef_link",
        expected_q_start=(0.0, 0.3, 0.3),
        control_period_s=0.1,
        send_hz=50.0,
        delta_position_m=(0.001, 0.0, 0.0),
        delta_rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))

    assert payload["status"] == "queued"
    assert payload["backend"] == "sdk_cartesian"
    assert payload["movement_command_sent"] is False
    assert command["backend"] == "sdk_cartesian"
    assert command["mature_backend_policy"]["selected"] == "sdk_cartesian"
    assert command["mature_backend_policy"]["no_heuristic_joint_fallback"] is True
    assert command["mature_backend_policy"]["execution_model"] == "continuous_owner_eef_servo"
    assert command["mature_backend_policy"]["disconnected_takeover_allowed"] is False
    assert payload["mature_backend_policy"]["bumpless_switch_required"] is True
    assert command["eef_reference_limit"]["status"] == "pass"
    assert command["eef_reference_limit"]["max_linear_step_m"] == 0.005


def test_cli_runtime_submit_eef_rejects_oversized_reference_step(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    with pytest.raises(ValueError, match="EEF reference limit"):
        submit_eef_command(
            session_artifact_path=session_artifact,
            owner="agent",
            backend="sdk_cartesian",
            kind="eef_pose_delta",
            frame="eef_link",
            expected_q_start=(0.0, 0.3, 0.3),
            control_period_s=0.1,
            send_hz=50.0,
            delta_position_m=(0.02, 0.0, 0.0),
            delta_rpy_rad=(0.0, 0.0, 0.0),
            max_start_error_rad=0.02,
            heartbeat_timeout_s=0.5,
            max_heartbeat_age_s=5.0,
        )


def test_cli_runtime_submit_eef_current_measured_pose_allows_cartesian_q_drift(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    session["backend"] = "sdk_cartesian"
    session["readiness_pose_policy"] = "current_measured_pose"
    session["q_hold"] = [1.5, 0.0, 0.0]
    session["q_meas"] = [1.4, 0.2, 0.1]
    session["readiness"] = runtime_readiness(session)
    session_artifact.write_text(json.dumps(session), encoding="utf-8")
    _configure_eef_adapter_manager(
        session_artifact,
        adapter="sdk_cartesian",
        primary_backend="sdk_cartesian",
    )

    payload = submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="sdk_cartesian",
        kind="eef_pose_delta",
        frame="eef_link",
        expected_q_start=(1.5, 0.0, 0.0),
        control_period_s=0.1,
        send_hz=50.0,
        start_pose_policy="current_measured_pose",
        delta_position_m=(0.001, 0.0, 0.0),
        delta_rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))

    assert session["readiness"]["agent_sysid_smoke_allowed"] is True
    assert session["readiness"]["failed_checks"] == []
    assert session["readiness"]["safe_center_allowed"] is False
    assert payload["status"] == "queued"
    assert command["start_pose_policy"] == "current_measured_pose"


def test_runtime_eef_current_measured_pose_executes_despite_cartesian_q_drift(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    session["backend"] = "sdk_cartesian"
    session["readiness_pose_policy"] = "current_measured_pose"
    session["q_hold"] = [1.458571434020996, 0.00324249267578125, 0.00820159912109375]
    session["q_meas"] = [1.458571434020996, 0.00629425048828125, 0.04444074630737305]
    session["readiness"] = runtime_readiness(session)
    session_artifact.write_text(json.dumps(session), encoding="utf-8")
    _configure_eef_adapter_manager(
        session_artifact,
        adapter="sdk_cartesian",
        primary_backend="sdk_cartesian",
    )
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    primary_backend = FakeMotionBackend()
    primary_backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    eef_adapter = FakeCartesianEefBackend(q_state=tuple(float(value) for value in session["q_meas"]))
    runtime = ArmRuntime(
        backend=primary_backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="sdk_cartesian",
        kind="eef_pose_delta",
        frame="eef_link",
        expected_q_start=tuple(float(value) for value in session["q_hold"]),
        control_period_s=0.1,
        send_hz=50.0,
        start_pose_policy="current_measured_pose",
        delta_position_m=(0.001, 0.0, 0.0),
        delta_rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=primary_backend,
        runtime=runtime,
        eef_backends={"sdk_cartesian": eef_adapter},
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "completed"
    assert result["movement_command_sent"] is True
    assert result["start_pose_policy"] == "current_measured_pose"
    assert result["motion"]["eef_command"]["command_space"] == "eef"
    assert result["eef_switch"]["status"] == "pass"
    assert result["eef_switch"]["checks"]["adapter_prepare_hook_present"] is True
    assert result["eef_switch"]["disconnected_takeover_allowed"] is False
    assert result["eef_switch"]["sdk_owner_released"] is False
    manager = result["eef_controller_manager"]
    assert manager["start_pose_policy"] == "current_measured_pose"
    assert manager["safe_center_required"] is False
    assert manager["start_q_reference"] == pytest.approx(session["q_meas"])
    updated_session = json.loads(session_artifact.read_text(encoding="utf-8"))
    assert updated_session["mode"] == "hold_safe"
    assert updated_session["owner"] is None
    assert updated_session["readiness"]["agent_sysid_smoke_allowed"] is True


class FakeCartesianEefBackend(FakeMotionBackend):
    def __init__(self, *, q_state: tuple[float, ...]) -> None:
        super().__init__()
        self._last_q = tuple(float(value) for value in q_state)

    def prepare_eef_servo_switch(self, command: dict[str, object]) -> dict[str, object]:
        return {
            "schema": "armctrl.eef_servo_switch.v1",
            "status": "pass",
            "backend": command.get("backend"),
            "policy": "adapter_declared_live_state_zero_command_warmup",
            "target_seed": "current_live_state",
            "zero_command_warmup_ticks": 1,
            "checks": {
                "sdk_owner_released": False,
                "adapter_prepare_hook_present": True,
                "target_seeded_from_current_state": True,
                "zero_command_warmup_completed": True,
            },
        }

    def execute_eef_command(
        self,
        command: dict[str, object],
        *,
        owner: str,
        watchdog=None,
        on_sample=None,
    ) -> MotionExecutionResult:
        sample = MotionAuditSample(
            sent_monotonic_s=10.0,
            q_cmd=tuple(float(value) for value in self._last_q or ()),
            dq_cmd=tuple(0.0 for _ in self._last_q or ()),
            q_meas=tuple(float(value) for value in self._last_q or ()),
            dq_meas=tuple(0.0 for _ in self._last_q or ()),
            tau_meas=tuple(0.0 for _ in self._last_q or ()),
            fault_flags=(),
            producer=owner,
            mode=MotionMode.AGENT_SERVO.value,
        )
        if on_sample is not None:
            on_sample(sample)
        return MotionExecutionResult(
            status="completed",
            producer=owner,
            mode=MotionMode.AGENT_SERVO.value,
            trajectory_sample_hz=None,
            actual_send_hz=None,
            send_jitter_ms_p95=None,
            send_jitter_ms_p99=None,
            controller_dt_s=None,
            samples=(sample,),
            landing_mode=MotionMode.HOLD.value,
        )


class NoWarmupEefBackend(FakeMotionBackend):
    def __init__(self, *, q_state: tuple[float, ...]) -> None:
        super().__init__()
        self._last_q = tuple(float(value) for value in q_state)
        self.executed = False

    def execute_eef_command(
        self,
        command: dict[str, object],
        *,
        owner: str,
        watchdog=None,
        on_sample=None,
    ) -> MotionExecutionResult:
        self.executed = True
        return MotionExecutionResult(
            status="completed",
            producer=owner,
            mode=MotionMode.AGENT_SERVO.value,
            trajectory_sample_hz=None,
            actual_send_hz=None,
            send_jitter_ms_p95=None,
            send_jitter_ms_p99=None,
            controller_dt_s=None,
            samples=(),
            landing_mode=MotionMode.HOLD.value,
        )


def test_runtime_eef_rejects_adapter_without_declared_warmup(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    _configure_eef_adapter_manager(session_artifact, adapter="moveit_servo")
    primary_backend = FakeMotionBackend()
    primary_backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    eef_adapter = NoWarmupEefBackend(
        q_state=tuple(float(value) for value in session["q_meas"])
    )
    runtime = ArmRuntime(
        backend=primary_backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="moveit_servo",
        kind="eef_pose_delta",
        frame="eef_link",
        expected_q_start=tuple(float(value) for value in session["q_hold"]),
        control_period_s=0.1,
        send_hz=50.0,
        delta_position_m=(0.001, 0.0, 0.0),
        delta_rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=primary_backend,
        runtime=runtime,
        eef_backends={"moveit_servo": eef_adapter},
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "rejected"
    assert result["movement_command_sent"] is False
    assert result["eef_switch"]["status"] == "fail"
    assert result["eef_switch"]["checks"]["adapter_prepare_hook_present"] is False
    assert eef_adapter.executed is False
    updated_session = json.loads(session_artifact.read_text(encoding="utf-8"))
    assert updated_session["mode"] == "hold_safe"
    assert updated_session["owner"] is None


class OwnerReleasingEefBackend(FakeCartesianEefBackend):
    def __init__(self, *, q_state: tuple[float, ...]) -> None:
        super().__init__(q_state=q_state)
        self.executed = False

    def prepare_eef_servo_switch(self, command: dict[str, object]) -> dict[str, object]:
        return {
            "schema": "armctrl.eef_servo_switch.v1",
            "status": "fail",
            "backend": command.get("backend"),
            "policy": "continuous_owner_bumpless_switch",
            "checks": {
                "sdk_owner_released": True,
                "target_seeded_from_current_state": False,
                "zero_command_warmup_completed": False,
            },
        }

    def execute_eef_command(
        self,
        command: dict[str, object],
        *,
        owner: str,
        watchdog=None,
        on_sample=None,
    ) -> MotionExecutionResult:
        self.executed = True
        return super().execute_eef_command(
            command,
            owner=owner,
            watchdog=watchdog,
            on_sample=on_sample,
        )


def test_cli_runtime_start_sdk_cartesian_is_diagnostic_only_without_explicit_opt_in(
    tmp_path: Path,
) -> None:
    output_artifact = tmp_path / "runtime_session.json"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "start",
            "--backend",
            "sdk_cartesian",
            "--safe-center",
            "0.0",
            "0.3",
            "0.3",
            "0.0",
            "0.0",
            "0.0",
            "--output",
            str(output_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["backend"] == "sdk_cartesian"
    assert payload["diagnostic_only"] is True
    assert payload["formal_runtime_gateway"] is False
    assert payload["reason"] == (
        "runtime start --backend sdk_cartesian is a disconnected diagnostic "
        "takeover, not the formal Agent EEF runtime path"
    )
    assert payload["movement_command_sent"] is False
    assert payload["artifacts"]["runtime_session"] == str(output_artifact)


def test_runtime_eef_submit_blocks_without_mature_backend_adapter(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    payload = submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="moveit_servo",
        kind="eef_pose_delta",
        frame="eef_link",
        expected_q_start=tuple(session["q_hold"]),
        control_period_s=0.1,
        send_hz=50.0,
        delta_position_m=(0.002, 0.0, 0.0),
        delta_rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )

    assert payload["status"] == "blocked"
    assert payload["owner"] == "agent"
    assert payload["mode"] == "agent_servo"
    assert payload["movement_command_sent"] is False
    assert payload["hardware_executable_now"] is False
    assert "configured mature EEF backend adapter" in payload["reason"]
    assert payload["eef_adapter_manager"]["eef_command_executable"] is False
    assert not (session_artifact.parent / "runtime_session_commands" / "pending").exists()
    assert len(backend.joint_commands) == 1


def test_runtime_status_exposes_unconfigured_eef_adapter_manager() -> None:
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3),
        safe_center=(0.0, 0.3, 0.3),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=1.0,
    )
    refreshed = refresh_runtime_status_payload(payload, max_heartbeat_age_s=1.0)

    manager = refreshed["eef_adapter_manager"]
    assert manager["schema"] == "armctrl.eef_adapter_manager.v1"
    assert manager["status"] == "unconfigured"
    assert manager["primary_runtime_backend"] == "fake"
    assert manager["configured_adapters"] == []
    assert manager["adapter_registry_required"] is True
    assert manager["primary_backend_fallback_allowed"] is False
    assert manager["disconnected_takeover_allowed"] is False
    assert manager["no_heuristic_joint_fallback"] is True
    assert manager["eef_command_executable"] is False
    assert manager["eef_command_capabilities"]["eef_pose_delta"]["executable"] is False
    assert manager["eef_command_capabilities"]["eef_twist"]["executable"] is False
    assert manager["eef_command_capabilities"]["eef_pose"]["executable"] is False
    assert manager["adapter_status"]["moveit_servo"]["configured"] is False
    assert manager["adapter_status"]["moveit_servo"]["executable"] is False
    assert manager["adapter_status"]["moveit_servo"]["requires_bumpless_switch"] is True
    controllers = refreshed["runtime_controller_manager"]["controllers"]
    assert controllers["joint_hold"]["status"] == "available"
    assert controllers["joint_trajectory"]["status"] == "available"
    assert controllers["joint_intent"]["status"] == "available"
    assert controllers["eef_servo"]["status"] == "needs_mature_adapter"
    assert (
        refreshed["runtime_controller_manager"]["fallback_policy"][
            "disconnected_takeover_allowed"
        ]
        is False
    )


def test_runtime_eef_adapter_manager_ignores_fake_adapter_on_real_backend() -> None:
    payload = {
        "schema": "armctrl.arm_runtime_session.v1",
        "backend": "arx5_sdk",
    }
    args = type("Args", (), {"eef_adapter": ["moveit_servo"]})()

    updated = _attach_eef_adapter_manager_from_args(
        payload,
        args,
        allow_configured_adapters=False,
    )

    manager = updated["eef_adapter_manager"]
    assert manager["status"] == "unconfigured"
    assert manager["primary_runtime_backend"] == "arx5_sdk"
    assert manager["configured_adapters"] == []
    assert manager["ignored_requested_adapters"] == ["moveit_servo"]
    assert manager["eef_command_executable"] is False
    assert manager["eef_command_capabilities"]["eef_pose_delta"]["executable"] is False
    assert manager["eef_command_capabilities"]["eef_twist"]["executable"] is False
    assert manager["eef_command_capabilities"]["eef_pose"]["executable"] is False
    assert manager["adapter_status"]["moveit_servo"]["configured"] is False
    assert manager["adapter_status"]["moveit_servo"]["executable"] is False
    assert manager["adapter_status"]["moveit_servo"]["hardware_scope"] == (
        "not_configured_for_real_runtime"
    )
    assert "offline runtime gateway rehearsal" in manager["reason"]
    controllers = updated["runtime_controller_manager"]["controllers"]
    assert controllers["joint_trajectory"]["status"] == "available"
    assert controllers["joint_intent"]["status"] == "available"
    assert controllers["eef_servo"]["status"] == "needs_mature_adapter"
    assert controllers["eef_servo"]["configured_adapters"] == []


def test_runtime_eef_adapter_manager_reports_pose_capability_separately(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = _configure_eef_adapter_manager(session_artifact, adapter="moveit_servo")

    manager = session["eef_adapter_manager"]

    assert manager["status"] == "ready"
    assert manager["eef_command_executable"] is True
    assert manager["eef_command_capabilities"]["eef_pose_delta"]["executable"] is True
    assert manager["eef_command_capabilities"]["eef_twist"]["executable"] is True
    assert manager["eef_command_capabilities"]["eef_pose"]["executable"] is False
    assert manager["eef_command_capabilities"]["eef_pose"]["reason"] == (
        "requires_live_reference"
    )
    assert manager["adapter_status"]["moveit_servo"]["command_capabilities"][
        "eef_pose"
    ]["executable"] is False
    assert (
        session["runtime_controller_manager"]["controllers"]["eef_servo"][
            "command_capabilities"
        ]["eef_pose"]["executable"]
        is False
    )


def test_runtime_eef_command_revalidates_reference_limit_before_execute(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _configure_eef_adapter_manager(
        session_artifact,
        adapter="moveit_servo",
        live_reference=True,
    )
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    primary_backend = FakeMotionBackend()
    primary_backend.send_joint_command(
        tuple(float(value) for value in session["q_hold"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    eef_adapter = FakeCartesianEefBackend(q_state=tuple(float(value) for value in session["q_hold"]))
    runtime = ArmRuntime(
        backend=primary_backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    payload = submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="moveit_servo",
        kind="eef_pose_delta",
        frame="eef_link",
        expected_q_start=tuple(session["q_hold"]),
        control_period_s=0.1,
        send_hz=50.0,
        delta_position_m=(0.002, 0.0, 0.0),
        delta_rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )
    command_path = Path(payload["artifacts"]["command"])
    command = json.loads(command_path.read_text(encoding="utf-8"))
    command["eef_command"]["delta_position_m"] = [0.02, 0.0, 0.0]
    command_path.write_text(json.dumps(command), encoding="utf-8")

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=primary_backend,
        runtime=runtime,
        eef_backends={"moveit_servo": eef_adapter},
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "rejected"
    assert result["movement_command_sent"] is False
    assert "EEF reference limit failed" in result["reason"]
    assert "linear_step_within_limit" in result["reason"]


def test_runtime_eef_pose_requires_adapter_reference_limiter_before_execute(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _configure_eef_adapter_manager(
        session_artifact,
        adapter="moveit_servo",
        live_reference=True,
    )
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    published: list[dict[str, object]] = []
    primary_backend = FakeMotionBackend()
    primary_backend.send_joint_command(
        tuple(float(value) for value in session["q_hold"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    eef_adapter = MoveItServoRuntimeBackend(
        q_state=tuple(float(value) for value in session["q_hold"]),
        current_eef_pose_6d=(0.30, 0.0, 0.20, 0.0, 0.0, 0.0),
        publisher=lambda message: published.append(message),
        monotonic=lambda: 10.0,
        sleep=lambda _duration_s: None,
    )
    runtime = ArmRuntime(
        backend=primary_backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    payload = submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="moveit_servo",
        kind="eef_pose",
        frame="base_link",
        expected_q_start=tuple(session["q_hold"]),
        control_period_s=0.1,
        send_hz=50.0,
        position_m=(0.30, 0.0, 0.20),
        rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )
    command_path = Path(payload["artifacts"]["command"])
    command = json.loads(command_path.read_text(encoding="utf-8"))
    command["eef_command"].pop("pose_reference_limiter", None)
    command_path.write_text(json.dumps(command), encoding="utf-8")

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=primary_backend,
        runtime=runtime,
        eef_backends={"moveit_servo": eef_adapter},
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "rejected"
    assert result["movement_command_sent"] is False
    assert "EEF reference limit failed" in result["reason"]
    assert "pose_reference_limiter_configured" in result["reason"]
    assert published == []


def test_runtime_eef_command_revalidates_live_hold_safe_center_before_execute(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _configure_eef_adapter_manager(session_artifact, adapter="moveit_servo")
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    primary_backend = FakeMotionBackend()
    primary_backend.send_joint_command(
        tuple(float(value) for value in session["q_hold"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=primary_backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    payload = submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="moveit_servo",
        kind="eef_pose_delta",
        frame="eef_link",
        expected_q_start=tuple(session["q_hold"]),
        control_period_s=0.1,
        send_hz=50.0,
        delta_position_m=(0.002, 0.0, 0.0),
        delta_rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )
    command_path = Path(payload["artifacts"]["command"])
    command = json.loads(command_path.read_text(encoding="utf-8"))
    non_safe_hold = [1.2, 0.0, 0.0]
    command["expected_q_start"] = non_safe_hold
    command["start_pose_guard"]["expected_q_start"] = non_safe_hold
    command["start_pose_guard"]["q_hold"] = non_safe_hold
    command["start_pose_guard"]["q_meas"] = non_safe_hold
    command["start_pose_guard"]["q_hold_to_expected_start_max_abs_rad"] = 0.0
    command["start_pose_guard"]["q_hold_to_safe_center_max_abs_rad"] = 1.2
    command_path.write_text(json.dumps(command), encoding="utf-8")
    session["q_hold"] = non_safe_hold
    session["q_meas"] = non_safe_hold
    session["last_hold_wall_time_s"] = time.time()
    heartbeat = dict(session["heartbeat"])
    heartbeat["wall_time_s"] = time.time()
    session["heartbeat"] = heartbeat
    session_artifact.write_text(
        json.dumps(session, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    runtime.mark_hold_safe(q_hold=tuple(non_safe_hold))
    moveit_adapter = MoveItServoRuntimeBackend(
        q_state=tuple(non_safe_hold),
        publisher=lambda _message: None,
        monotonic=lambda: 10.0,
        sleep=lambda _duration_s: None,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=primary_backend,
        runtime=runtime,
        eef_backends={"moveit_servo": moveit_adapter},
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "rejected"
    assert result["movement_command_sent"] is False
    assert "SAFE_CENTER" in result["reason"]
    assert result["start_pose_guard"]["safe_center_required"] is True
    assert "q_hold_close_to_safe_center" in result["start_pose_guard"]["failed_checks"]


def test_runtime_eef_command_rejects_backend_that_releases_owner_during_switch(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _configure_eef_adapter_manager(session_artifact, adapter="moveit_servo")
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    primary_backend = FakeMotionBackend()
    primary_backend.send_joint_command(
        tuple(float(value) for value in session["q_hold"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    eef_adapter = OwnerReleasingEefBackend(q_state=tuple(float(value) for value in session["q_hold"]))
    runtime = ArmRuntime(
        backend=primary_backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="moveit_servo",
        kind="eef_pose_delta",
        frame="eef_link",
        expected_q_start=tuple(session["q_hold"]),
        control_period_s=0.1,
        send_hz=50.0,
        delta_position_m=(0.002, 0.0, 0.0),
        delta_rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=primary_backend,
        runtime=runtime,
        eef_backends={"moveit_servo": eef_adapter},
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "rejected"
    assert result["movement_command_sent"] is False
    assert "EEF servo switch warmup failed" in result["reason"]
    assert eef_adapter.executed is False


def test_moveit_servo_runtime_backend_builds_twist_from_pose_delta() -> None:
    published: list[dict[str, object]] = []
    backend = MoveItServoRuntimeBackend(
        publisher=lambda message: published.append(message),
        monotonic=lambda: 10.0,
        sleep=lambda _duration_s: None,
    )

    result = backend.execute_eef_command(
        {
            "kind": "eef_pose_delta",
            "send_hz": 50.0,
            "eef_command": {
                "frame": "eef_link",
                "delta_position_m": [0.002, 0.0, 0.0],
                "delta_rpy_rad": [0.0, 0.0, 0.1],
                "control_period_s": 0.1,
            },
        },
        owner="agent",
    )

    assert result.status == "completed"
    assert result.mode == "agent_servo"
    assert len(published) == 1
    assert published[0]["message_type"] == "geometry_msgs/msg/TwistStamped"
    assert published[0]["topic"] == "/servo_node/delta_twist_cmds"
    assert published[0]["frame_id"] == "eef_link"
    assert published[0]["twist"]["linear"] == [0.02, 0.0, 0.0]
    assert published[0]["twist"]["angular"] == [0.0, 0.0, 1.0]


def test_moveit_servo_runtime_backend_builds_pose_reference() -> None:
    published: list[dict[str, object]] = []
    backend = MoveItServoRuntimeBackend(
        current_eef_pose_6d=(0.30, 0.0, 0.20, 0.0, 0.0, 0.0),
        publisher=lambda message: published.append(message),
        monotonic=lambda: 10.0,
        sleep=lambda _duration_s: None,
    )

    result = backend.execute_eef_command(
        {
            "kind": "eef_pose",
            "send_hz": 50.0,
            "eef_command": {
                "frame": "base_link",
                "position_m": [0.30, 0.0, 0.20],
                "rpy_rad": [0.0, 0.0, 0.0],
                "pose_reference_limiter": "adapter_live_reference_limit",
                "control_period_s": 0.1,
            },
        },
        owner="agent",
    )

    assert result.status == "completed"
    assert len(published) == 1
    assert published[0]["message_type"] == "geometry_msgs/msg/PoseStamped"
    assert published[0]["topic"] == "/servo_node/pose_target_cmds"
    assert published[0]["frame_id"] == "base_link"
    assert published[0]["pose"]["position_m"] == [0.3, 0.0, 0.2]
    assert published[0]["pose"]["rpy_rad"] == [0.0, 0.0, 0.0]
    assert published[0]["reference_limit_policy"] == "adapter_live_reference_limit"


def test_moveit_servo_runtime_backend_rejects_oversized_absolute_pose() -> None:
    published: list[dict[str, object]] = []
    backend = MoveItServoRuntimeBackend(
        current_eef_pose_6d=(0.20, 0.0, 0.20, 0.0, 0.0, 0.0),
        publisher=lambda message: published.append(message),
        monotonic=lambda: 10.0,
        sleep=lambda _duration_s: None,
    )

    with pytest.raises(ValueError, match="MoveIt Servo EEF reference limit failed"):
        backend.execute_eef_command(
            {
                "kind": "eef_pose",
                "send_hz": 50.0,
                "eef_reference_limit": {
                    "schema": "armctrl.eef_reference_limit.v1",
                    "status": "pass",
                    "max_linear_step_m": 0.005,
                    "max_angular_step_rad": 0.05,
                    "pose_reference_limiter": "adapter_live_reference_limit",
                },
                "eef_command": {
                    "frame": "base_link",
                    "position_m": [0.260, 0.0, 0.20],
                    "rpy_rad": [0.0, 0.0, 0.0],
                    "pose_reference_limiter": "adapter_live_reference_limit",
                    "control_period_s": 0.1,
                },
            },
            owner="agent",
        )

    assert published == []


class FakeSdkEefState:
    def __init__(self, pose: list[float] | None = None) -> None:
        self._pose = list(pose or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.gripper_pos = 0.0
        self.gripper_vel = 0.0
        self.gripper_torque = 0.0
        self.timestamp = 0.0

    def pose_6d(self) -> list[float]:
        return self._pose


class FakeSdkJointState:
    def pos(self) -> list[float]:
        return [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]

    def vel(self) -> list[float]:
        return [0.0] * 6

    def torque(self) -> list[float]:
        return [0.0] * 6


class FakeSdkCartesianController:
    def __init__(self) -> None:
        self.current_pose = [0.20, 0.0, 0.20, 0.0, 0.0, 0.0]
        self.commands: list[list[float]] = []

    def get_eef_state(self) -> FakeSdkEefState:
        return FakeSdkEefState(self.current_pose)

    def set_eef_cmd(self, command: FakeSdkEefState) -> None:
        self.commands.append(list(command.pose_6d()))
        self.current_pose = list(command.pose_6d())

    def get_joint_state(self) -> FakeSdkJointState:
        return FakeSdkJointState()

    def get_timestamp(self) -> float:
        return 10.0

    def get_controller_config(self):
        return type("ControllerConfig", (), {"default_preview_time": 0.04})()


class FakeSdkCartesianModule:
    EEFState = FakeSdkEefState


def test_arx5_sdk_cartesian_backend_executes_limited_absolute_eef_pose() -> None:
    controller = FakeSdkCartesianController()
    clock = ManualClock()
    backend = Arx5SdkCartesianRuntimeBackend(
        arx5_module=FakeSdkCartesianModule(),
        controller=controller,
        controller_dt_s=0.002,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    result = backend.execute_eef_command(
        {
            "kind": "eef_pose",
            "send_hz": 50.0,
            "eef_command": {
                "frame": "base_link",
                "position_m": [0.205, 0.0, 0.20],
                "rpy_rad": [0.0, 0.0, 0.0],
                "pose_reference_limiter": "adapter_live_reference_limit",
                "control_period_s": 0.1,
            },
        },
        owner="agent",
    )

    assert result.status == "completed"
    assert len(controller.commands) == 6
    assert controller.commands[0] == pytest.approx([0.20, 0.0, 0.20, 0.0, 0.0, 0.0])
    assert controller.commands[-1] == pytest.approx([0.205, 0.0, 0.20, 0.0, 0.0, 0.0])
    assert result.landing_mode == "hold"


def test_arx5_sdk_cartesian_backend_rejects_oversized_absolute_eef_pose() -> None:
    controller = FakeSdkCartesianController()
    clock = ManualClock()
    backend = Arx5SdkCartesianRuntimeBackend(
        arx5_module=FakeSdkCartesianModule(),
        controller=controller,
        controller_dt_s=0.002,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    with pytest.raises(ValueError, match="SDK Cartesian EEF reference limit failed"):
        backend.execute_eef_command(
            {
                "kind": "eef_pose",
                "send_hz": 50.0,
                "eef_reference_limit": {
                    "schema": "armctrl.eef_reference_limit.v1",
                    "status": "pass",
                    "max_linear_step_m": 0.005,
                    "max_angular_step_rad": 0.05,
                    "pose_reference_limiter": "adapter_live_reference_limit",
                },
                "eef_command": {
                    "frame": "base_link",
                    "position_m": [0.260, 0.0, 0.20],
                    "rpy_rad": [0.0, 0.0, 0.0],
                    "pose_reference_limiter": "adapter_live_reference_limit",
                    "control_period_s": 0.1,
                },
            },
            owner="agent",
        )

    assert controller.commands == []


def test_runtime_eef_command_executes_with_configured_moveit_backend(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _configure_eef_adapter_manager(session_artifact, adapter="moveit_servo")
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    published: list[dict[str, object]] = []
    primary_backend = FakeMotionBackend()
    primary_backend.send_joint_command(
        tuple(float(value) for value in session["q_hold"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    moveit_adapter = MoveItServoRuntimeBackend(
        q_state=tuple(float(value) for value in session["q_hold"]),
        current_eef_pose_6d=(0.30, 0.0, 0.20, 0.0, 0.0, 0.0),
        publisher=lambda message: published.append(message),
        monotonic=lambda: 10.0,
        sleep=lambda _duration_s: None,
    )
    runtime = ArmRuntime(
        backend=primary_backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="moveit_servo",
        kind="eef_pose_delta",
        frame="eef_link",
        expected_q_start=tuple(session["q_hold"]),
        control_period_s=0.1,
        send_hz=50.0,
        delta_position_m=(0.002, 0.0, 0.0),
        delta_rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=primary_backend,
        runtime=runtime,
        eef_backends={"moveit_servo": moveit_adapter},
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "completed"
    assert result["owner"] == "agent"
    assert result["mode"] == "agent_servo"
    assert result["movement_command_sent"] is True
    assert len(published) == 1
    assert result["motion"]["eef_command"]["kind"] == "eef_pose_delta"
    assert result["motion"]["eef_command"]["backend"] == "moveit_servo"
    assert result["motion"]["eef_command"]["command_space"] == "eef"
    assert result["eef_switch"]["status"] == "pass"
    assert result["eef_switch"]["policy"] == "continuous_owner_bumpless_switch"
    assert result["eef_switch"]["disconnected_takeover_allowed"] is False
    assert result["eef_switch"]["sdk_owner_released"] is False
    assert result["eef_switch"]["checks"]["publisher_configured"] is True
    manager = result["eef_controller_manager"]
    assert manager["schema"] == "armctrl.eef_controller_manager.v1"
    assert manager["status"] == "ready"
    assert manager["start_pose_policy"] == "live_hold"
    assert manager["start_pose_guard"]["status"] == "pass"
    assert manager["start_q_reference"] == pytest.approx(session["q_hold"])
    assert manager["safe_center_required"] is True
    assert manager["safe_center_reference"] == pytest.approx(session["safe_center"])
    assert manager["mode_switch_policy"] == "runtime_internal_controller_manager"
    assert manager["controller_sequence"] == [
        "joint_hold_active",
        "resolve_explicit_eef_adapter",
        "acquire_single_runtime_owner",
        "seed_eef_target_from_current_state",
        "zero_command_warmup",
        "execute_eef_servo_command",
        "release_to_hold_or_fault_to_damping",
    ]
    assert manager["passive_safe_policy"].startswith("passive/droop pose")
    assert manager["safe_position_policy"].startswith("SAFE_CENTER")
    assert manager["primary_backend_fallback_allowed"] is False
    assert manager["disconnected_takeover_allowed"] is False
    assert manager["no_heuristic_joint_fallback"] is True
    assert manager["warmup_gate"]["status"] == "pass"
    updated_session = json.loads(session_artifact.read_text(encoding="utf-8"))
    assert updated_session["mode"] == "hold_safe"
    assert updated_session["owner"] is None


def test_runtime_eef_command_executes_with_primary_runtime_and_adapter_registry(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _configure_eef_adapter_manager(session_artifact, adapter="moveit_servo")
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    primary_backend = FakeMotionBackend()
    primary_backend.send_joint_command(
        tuple(float(value) for value in session["q_hold"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=primary_backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    published: list[dict[str, object]] = []
    moveit_adapter = MoveItServoRuntimeBackend(
        q_state=tuple(float(value) for value in session["q_hold"]),
        current_eef_pose_6d=(0.30, 0.0, 0.20, 0.0, 0.0, 0.0),
        publisher=lambda message: published.append(message),
        monotonic=lambda: 10.0,
        sleep=lambda _duration_s: None,
    )
    submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="moveit_servo",
        kind="eef_pose_delta",
        frame="eef_link",
        expected_q_start=tuple(session["q_hold"]),
        control_period_s=0.1,
        send_hz=50.0,
        delta_position_m=(0.002, 0.0, 0.0),
        delta_rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=primary_backend,
        runtime=runtime,
        eef_backends={"moveit_servo": moveit_adapter},
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "completed"
    assert result["movement_command_sent"] is True
    assert len(published) == 1
    assert result["motion"]["eef_command"]["backend"] == "moveit_servo"
    assert result["motion"]["eef_command"]["runtime_backend"] == "fake"
    assert result["motion"]["eef_command"]["eef_adapter"] == "moveit_servo"
    assert result["eef_switch"]["backend"] == "moveit_servo"
    assert result["eef_switch"]["sdk_owner_released"] is False
    assert result["eef_switch"]["disconnected_takeover_allowed"] is False
    updated_session = json.loads(session_artifact.read_text(encoding="utf-8"))
    assert updated_session["mode"] == "hold_safe"
    assert updated_session["owner"] is None


def test_runtime_eef_command_rejects_primary_backend_without_adapter_registry(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    primary_backend = MoveItServoRuntimeBackend(
        q_state=tuple(float(value) for value in session["q_hold"]),
        publisher=lambda _message: None,
        monotonic=lambda: 10.0,
        sleep=lambda _duration_s: None,
    )
    runtime = ArmRuntime(
        backend=primary_backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    payload = submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="moveit_servo",
        kind="eef_pose_delta",
        frame="eef_link",
        expected_q_start=tuple(session["q_hold"]),
        control_period_s=0.1,
        send_hz=50.0,
        delta_position_m=(0.002, 0.0, 0.0),
        delta_rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )

    assert payload["status"] == "blocked"
    assert "configured mature EEF backend adapter" in payload["reason"]
    assert payload["eef_adapter_manager"]["adapter_registry_required"] is True
    assert payload["eef_adapter_manager"]["primary_backend_fallback_allowed"] is False
    assert payload["eef_adapter_manager"]["disconnected_takeover_allowed"] is False
    assert payload["movement_command_sent"] is False
    assert payload["hardware_executable_now"] is False


def test_runtime_eef_pose_executes_with_configured_adapter_registry(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _configure_eef_adapter_manager(
        session_artifact,
        adapter="moveit_servo",
        live_reference=True,
    )
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    primary_backend = FakeMotionBackend()
    primary_backend.send_joint_command(
        tuple(float(value) for value in session["q_hold"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=primary_backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    published: list[dict[str, object]] = []
    moveit_adapter = MoveItServoRuntimeBackend(
        q_state=tuple(float(value) for value in session["q_hold"]),
        current_eef_pose_6d=(0.30, 0.0, 0.20, 0.0, 0.0, 0.0),
        publisher=lambda message: published.append(message),
        monotonic=lambda: 10.0,
        sleep=lambda _duration_s: None,
    )
    submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="moveit_servo",
        kind="eef_pose",
        frame="base_link",
        expected_q_start=tuple(session["q_hold"]),
        control_period_s=0.1,
        send_hz=50.0,
        position_m=(0.30, 0.0, 0.20),
        rpy_rad=(0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=primary_backend,
        runtime=runtime,
        eef_backends={"moveit_servo": moveit_adapter},
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "completed"
    assert result["command_space"] == "eef"
    assert result["motion"]["eef_command"]["kind"] == "eef_pose"
    assert result["motion"]["eef_command"]["eef_adapter"] == "moveit_servo"
    assert result["eef_switch"]["status"] == "pass"
    assert len(published) == 1
    assert published[0]["message_type"] == "geometry_msgs/msg/PoseStamped"
    assert published[0]["reference_limit_policy"] == "adapter_live_reference_limit"


class FakeArx5RuntimeBackend(FakeMotionBackend):
    controller_dt_s = 0.002

    def __init__(self, *, q_start: tuple[float, ...]) -> None:
        super().__init__()
        self._last_q = q_start
        self.enter_count = 0

    def enter_hold_or_damping(self) -> None:
        self.enter_count += 1

    def read_joint_state(self) -> JointStateSnapshot:
        return JointStateSnapshot(
            q_meas=self._last_q or (),
            dq_meas=(0.0,) * len(self._last_q or ()),
            tau_meas=(0.0,) * len(self._last_q or ()),
            fault_flags=self.fault_flags,
        )


class LaggingReadbackBackend(FakeMotionBackend):
    def __init__(self, *, lagging_q: tuple[float, ...]) -> None:
        super().__init__()
        self._lagging_q = lagging_q

    def read_joint_state(self) -> JointStateSnapshot:
        return JointStateSnapshot(
            q_meas=self._lagging_q,
            dq_meas=(0.0,) * len(self._lagging_q),
            tau_meas=(0.0,) * len(self._lagging_q),
            fault_flags=(),
        )


class TorqueReadbackBackend(FakeMotionBackend):
    def __init__(self, *, tau_meas: tuple[float, ...]) -> None:
        super().__init__()
        self._tau_meas = tau_meas

    def read_joint_state(self) -> JointStateSnapshot:
        return JointStateSnapshot(
            q_meas=self._last_q or (),
            dq_meas=(0.0,) * len(self._last_q or ()),
            tau_meas=self._tau_meas,
            fault_flags=(),
        )


class ManualClock:
    def __init__(self) -> None:
        self.now_s = 0.0

    def monotonic(self) -> float:
        return self.now_s

    def sleep(self, seconds: float) -> None:
        self.now_s += max(0.0, seconds)


def test_start_arx5_runtime_session_recovers_and_exports_live_hold_status() -> None:
    backend = FakeArx5RuntimeBackend(q_start=(0.8, 0.0, 0.0))

    payload = start_arx5_runtime_session(
        backend=backend,
        model="X5",
        interface="can0",
        safe_center=(0.0, 0.3, 0.3),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=1.0,
    )

    assert payload["status"] == "blocked"
    assert payload["schema"] == "armctrl.arm_runtime_session.v1"
    assert payload["backend"] == "arx5_sdk"
    assert payload["model"] == "X5"
    assert payload["interface"] == "can0"
    assert payload["mode"] == "hold_safe"
    assert payload["owner"] is None
    assert payload["q_meas"] == [0.0, 0.3, 0.3]
    assert payload["q_hold"] == [0.0, 0.3, 0.3]
    assert payload["safe_center"] == [0.0, 0.3, 0.3]
    assert payload["controller_dt_s"] == 0.002
    assert payload["send_hz"] == 50.0
    assert payload["hold_hz"] == 50.0
    assert payload["hardware_motion"] is True
    assert payload["sdk_opened"] is True
    assert payload["movement_command_sent"] is True
    assert payload["hold_tick_count"] == 0
    assert payload["hold_fresh"] is False
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is False
    assert payload["readiness"]["failed_checks"] == ["hold_fresh"]
    assert payload["recovery"]["status"] == "completed"
    assert payload["recovery"]["sample_count"] > 0
    assert backend.enter_count == 1
    assert backend.hold_count >= 1
    assert backend.damping_count == 0


def test_cli_runtime_start_fake_recovers_to_hold_safe_session(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "start",
            "--backend",
            "fake",
            "--q-current",
            "1.0",
            "0.0",
            "0.0",
            "--safe-center",
            "0.0",
            "0.3",
            "0.3",
            "--send-hz",
            "50",
            "--hold-hz",
            "50",
            "--output",
            str(session_artifact),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "blocked"
    assert payload["schema"] == "armctrl.arm_runtime_session.v1"
    assert payload["backend"] == "fake"
    assert payload["mode"] == "hold_safe"
    assert payload["owner"] is None
    assert payload["q_meas"] == [0.0, 0.3, 0.3]
    assert payload["q_hold"] == [0.0, 0.3, 0.3]
    assert payload["safe_center"] == [0.0, 0.3, 0.3]
    assert payload["send_hz"] == 50.0
    assert payload["hold_hz"] == 50.0
    assert payload["hold_tick_count"] == 0
    assert payload["last_hold_wall_time_s"] is None
    assert payload["hold_fresh"] is False
    assert payload["heartbeat"]["fresh"] is True
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is False
    assert payload["readiness"]["failed_checks"] == ["hold_fresh"]
    assert payload["runtime_session_id"]
    assert payload["artifacts"]["runtime_session"] == str(session_artifact)
    assert json.loads(session_artifact.read_text(encoding="utf-8")) == payload


def test_cli_runtime_start_serve_clears_stale_stop_marker(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    stop_request = _runtime_stop_request_path(session_artifact)
    stop_request.write_text("stop\n", encoding="utf-8")

    server = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "start",
            "--backend",
            "fake",
            "--q-current",
            "0.0",
            "0.3",
            "0.3",
            "--safe-center",
            "0.0",
            "0.3",
            "0.3",
            "--serve",
            "--heartbeat-period-s",
            "0.05",
            "--max-heartbeat-age-s",
            "1.0",
            "--output",
            str(session_artifact),
            "--json",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        served = _wait_for_session_artifact(session_artifact)
        assert served["mode"] == "hold_safe"
        assert not stop_request.exists()
        assert server.poll() is None

        stop_completed = _run_runtime_stop_with_retry(session_artifact)
        assert stop_completed.returncode == 3
        _wait_for_runtime_stopped(session_artifact)
        stdout, stderr = server.communicate(timeout=8.0)
        assert stderr == ""
        assert json.loads(stdout)["status"] == "stopped"
    finally:
        if server.poll() is None:
            server.terminate()
            server.communicate(timeout=3.0)


def test_cli_runtime_start_arx5_sdk_requires_runtime_confirmation(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "start",
            "--backend",
            "arx5_sdk",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--safe-center",
            "0.0",
            "0.3",
            "0.3",
            "0.0",
            "0.0",
            "0.0",
            "--output",
            str(session_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.arm_runtime_session.v1"
    assert payload["backend"] == "arx5_sdk"
    assert payload["reason"] == "arx5 runtime start requires explicit operator confirmation"
    assert payload["requires_confirm"] == ARX5_RUNTIME_START_CONFIRMATION
    assert payload["hardware_motion"] is True
    assert payload["movement_command_sent"] is False
    assert payload["sdk_opened"] is False
    assert payload["fault_landing_mode"] == "damping"
    assert payload["next_gate"] == "confirm arx5 runtime start on the robot host"
    assert json.loads(session_artifact.read_text(encoding="utf-8")) == payload


def test_cli_runtime_start_arx5_sdk_rejects_missing_sdk_after_confirmation(
    tmp_path: Path,
) -> None:
    if importlib.util.find_spec("arx5_interface") is not None:
        pytest.skip(
            "arx5_interface is installed; missing-SDK rejection is not applicable"
        )
    session_artifact = tmp_path / "runtime_session.json"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "start",
            "--backend",
            "arx5_sdk",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--safe-center",
            "0.0",
            "0.3",
            "0.3",
            "0.0",
            "0.0",
            "0.0",
            "--confirm",
            ARX5_RUNTIME_START_CONFIRMATION,
            "--output",
            str(session_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.arm_runtime_session.v1"
    assert payload["backend"] == "arx5_sdk"
    assert payload["reason"] == "arx5_interface is not importable in this environment"
    assert payload["requires_confirm"] == ARX5_RUNTIME_START_CONFIRMATION
    assert payload["sdk_opened"] is False
    assert payload["movement_command_sent"] is False
    assert payload["next_gate"] == "run on the robot host with arx5_interface installed"
    assert json.loads(session_artifact.read_text(encoding="utf-8")) == payload


def test_cli_runtime_start_fake_serve_refreshes_heartbeat_until_stop(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"

    server = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "start",
            "--backend",
            "fake",
            "--q-current",
            "0.0",
            "0.3",
            "0.3",
            "--safe-center",
            "0.0",
            "0.3",
            "0.3",
            "--serve",
            "--heartbeat-period-s",
            "0.05",
            "--max-heartbeat-age-s",
            "1.0",
            "--output",
            str(session_artifact),
            "--json",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        first_seen = _wait_for_session_artifact(session_artifact)
        first_wall_time_s = first_seen["heartbeat"]["wall_time_s"]
        time.sleep(0.15)
        refreshed = _wait_for_session_artifact(session_artifact)

        assert refreshed["mode"] == "hold_safe"
        assert refreshed["owner"] is None
        assert refreshed["heartbeat"]["wall_time_s"] > first_wall_time_s
        assert refreshed["readiness"]["agent_sysid_smoke_allowed"] is True

        stop_completed = _run_runtime_stop_with_retry(session_artifact)

        assert stop_completed.returncode == 3
        _wait_for_runtime_stopped(session_artifact)
        stdout, stderr = server.communicate(timeout=8.0)
        assert server.returncode == 0
        served = json.loads(stdout)
        stopped = _wait_for_runtime_stopped(session_artifact)
        assert stderr == ""
        assert served["status"] == "stopped"
        assert served["mode"] == "damping"
        assert stopped["status"] == "stopped"
        assert stopped["mode"] == "damping"
        assert stopped["runtime_session_id"] == served["runtime_session_id"]
    finally:
        if server.poll() is None:
            server.terminate()
            server.communicate(timeout=3.0)


def test_runtime_serve_loop_streams_active_hold_until_stop(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3),
        safe_center=(0.0, 0.3, 0.3),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=1.0,
    )
    payload = record_runtime_hold_tick(
        payload,
        q_meas=(0.0, 0.3, 0.3),
        fault_flags=(),
        max_heartbeat_age_s=1.0,
    )
    session_artifact.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    hold_ticks: list[tuple[float, ...]] = []

    def hold_tick(current: dict[str, object]) -> None:
        hold_ticks.append(tuple(float(value) for value in current["q_hold"]))
        if len(hold_ticks) == 2:
            _runtime_stop_request_path(session_artifact).write_text(
                "stop\n",
                encoding="utf-8",
            )

    backend = FakeMotionBackend()
    backend.send_joint_command(
        (0.0, 0.3, 0.3),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )

    result = _serve_runtime_session_until_stopped(
        session_artifact_path=session_artifact,
        heartbeat_period_s=0.01,
        max_heartbeat_age_s=1.0,
        backend=backend,
        hold_tick=hold_tick,
    )

    assert hold_ticks == [(0.0, 0.3, 0.3), (0.0, 0.3, 0.3)]
    served = json.loads(session_artifact.read_text(encoding="utf-8"))
    assert served["hold_tick_count"] == 2
    assert served["last_hold_wall_time_s"] is not None
    assert served["hold_fresh"] is True
    assert result["status"] == "stopped"
    assert result["mode"] == "damping"


def test_runtime_serve_loop_lands_damping_on_keyboard_interrupt(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3),
        safe_center=(0.0, 0.3, 0.3),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=1.0,
    )
    payload = record_runtime_hold_tick(
        payload,
        q_meas=(0.0, 0.3, 0.3),
        fault_flags=(),
        max_heartbeat_age_s=1.0,
    )
    session_artifact.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    backend = FakeMotionBackend()
    backend.send_joint_command(
        (0.0, 0.3, 0.3),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )

    def interrupt(_duration_s: float) -> None:
        raise KeyboardInterrupt

    monkeypatch.setattr("armctrl.cli.time.sleep", interrupt)

    result = _serve_runtime_session_until_stopped(
        session_artifact_path=session_artifact,
        heartbeat_period_s=0.01,
        max_heartbeat_age_s=1.0,
        backend=backend,
    )
    served = json.loads(session_artifact.read_text(encoding="utf-8"))

    assert result["status"] == "interrupted"
    assert result["mode"] == "damping"
    assert result["reason"] == "keyboard_interrupt"
    assert served == result
    assert backend.damping_count >= 1


def test_runtime_status_blocks_when_hold_evidence_is_stale() -> None:
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3),
        safe_center=(0.0, 0.3, 0.3),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=1.0,
    )
    payload["heartbeat"]["wall_time_s"] = time.time()
    payload["hold_tick_count"] = 3
    payload["last_hold_wall_time_s"] = time.time() - 10.0

    refreshed = refresh_runtime_status_payload(payload, max_heartbeat_age_s=1.0)

    assert refreshed["status"] == "blocked"
    assert refreshed["hold_fresh"] is False
    assert refreshed["readiness"]["agent_sysid_smoke_allowed"] is False
    assert refreshed["readiness"]["failed_checks"] == ["hold_fresh"]


def test_runtime_serve_loop_does_not_overwrite_concurrent_stop(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3),
        safe_center=(0.0, 0.3, 0.3),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=1.0,
    )
    payload = record_runtime_hold_tick(
        payload,
        q_meas=(0.0, 0.3, 0.3),
        fault_flags=(),
        max_heartbeat_age_s=1.0,
    )
    session_artifact.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    stop_writes = 0

    def stop_during_heartbeat(
        current: dict[str, object],
        *,
        max_heartbeat_age_s: float,
    ) -> dict[str, object]:
        nonlocal stop_writes
        if stop_writes == 0:
            stopped = stop_runtime_session_from_artifact(
                session_artifact_path=session_artifact,
                max_heartbeat_age_s=1.0,
            )
            session_artifact.write_text(
                json.dumps(stopped, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            stop_writes += 1
        return heartbeat_runtime_session_payload(
            current,
            max_heartbeat_age_s=max_heartbeat_age_s,
        )

    monkeypatch.setattr(
        "armctrl.cli.heartbeat_runtime_session_payload",
        stop_during_heartbeat,
    )
    monkeypatch.setattr("armctrl.cli.time.sleep", lambda _seconds: None)

    result = _serve_runtime_session_until_stopped(
        session_artifact_path=session_artifact,
        heartbeat_period_s=0.01,
        max_heartbeat_age_s=1.0,
    )
    stopped = json.loads(session_artifact.read_text(encoding="utf-8"))

    assert result["status"] == "stopped"
    assert result["mode"] == "damping"
    assert stopped["status"] == "stopped"
    assert stopped["mode"] == "damping"
    assert stop_writes == 1


def test_runtime_serve_loop_honors_stop_request_marker(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3),
        safe_center=(0.0, 0.3, 0.3),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=1.0,
    )
    session_artifact.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    stop_request = _runtime_stop_request_path(session_artifact)
    stop_request.write_text("stop\n", encoding="utf-8")
    monkeypatch.setattr("armctrl.cli.time.sleep", lambda _seconds: None)

    result = _serve_runtime_session_until_stopped(
        session_artifact_path=session_artifact,
        heartbeat_period_s=0.01,
        max_heartbeat_age_s=1.0,
    )
    stopped = json.loads(session_artifact.read_text(encoding="utf-8"))

    assert result["status"] == "stopped"
    assert result["mode"] == "damping"
    assert stopped == result
    assert not stop_request.exists()


def test_runtime_serve_loop_lands_stale_owner_deadman_in_damping(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3),
        safe_center=(0.0, 0.3, 0.3),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=1.0,
    )
    payload["mode"] = "agent_servo"
    payload["owner"] = "agent"
    payload["owner_lease"] = {
        "schema": "armctrl.arm_runtime_owner_lease.v1",
        "runtime_session_id": payload["runtime_session_id"],
        "owner": "agent",
        "mode": "agent_servo",
        "heartbeat_timeout_s": 0.1,
        "heartbeat_wall_time_s": time.time() - 1.0,
        "acquired_wall_time_s": time.time() - 1.0,
        "landing_policy": "watchdog_to_damping_release_to_hold_safe",
    }
    session_artifact.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    monkeypatch.setattr("armctrl.cli.time.sleep", lambda _seconds: None)

    result = _serve_runtime_session_until_stopped(
        session_artifact_path=session_artifact,
        heartbeat_period_s=0.01,
        max_heartbeat_age_s=1.0,
    )
    faulted = json.loads(session_artifact.read_text(encoding="utf-8"))

    assert result["status"] == "faulted"
    assert result["mode"] == "damping"
    assert result["owner"] is None
    assert result["owner_lease"] is None
    assert result["owner_deadman"] is None
    assert result["watchdog"]["owner"] == "agent"
    assert result["watchdog"]["reason"] == "owner_heartbeat_timeout"
    assert faulted == result


def test_runtime_watchdog_holds_missed_agent_intent_before_fault_timeout(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    now_s = 100.0
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3),
        safe_center=(0.0, 0.3, 0.3),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=1.0,
    )
    payload["status"] = "ok"
    payload["mode"] = "agent_servo"
    payload["owner"] = "agent"
    payload["q_hold"] = [0.01, 0.3, 0.3]
    payload["q_meas"] = [0.01, 0.3, 0.3]
    payload["owner_lease"] = {
        "schema": "armctrl.arm_runtime_owner_lease.v1",
        "runtime_session_id": payload["runtime_session_id"],
        "owner": "agent",
        "mode": "agent_servo",
        "heartbeat_timeout_s": 1.0,
        "heartbeat_wall_time_s": now_s - 0.31,
        "acquired_wall_time_s": now_s - 0.5,
        "last_intent_wall_time_s": now_s - 0.31,
        "missed_intent_timeout_s": 0.3,
        "fault_timeout_s": 1.0,
        "missed_intent_held": False,
        "landing_policy": "missed_intent_hold_then_damping",
    }
    payload["heartbeat"] = {"wall_time_s": now_s, "age_s": 0.0, "fresh": True}
    session_artifact.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    monkeypatch.setattr("armctrl.runtime_session.time.time", lambda: now_s)

    held = watchdog_tick_from_artifact(
        session_artifact_path=session_artifact,
        max_heartbeat_age_s=1.0,
    )

    assert held["status"] == "ok"
    assert held["mode"] == "hold_safe"
    assert held["owner"] is None
    assert held["owner_lease"] is None
    assert held["q_hold"] == [0.01, 0.3, 0.3]
    assert held["watchdog"] == {
        "owner": "agent",
        "landing_mode": "hold",
        "reason": "missed_agent_intent_timeout",
        "intent_age_s": pytest.approx(0.31),
        "missed_intent_timeout_s": 0.3,
    }
    assert json.loads(session_artifact.read_text(encoding="utf-8")) == held

    held["mode"] = "agent_servo"
    held["owner"] = "agent"
    held["owner_lease"] = dict(payload["owner_lease"])
    held["owner_lease"]["heartbeat_wall_time_s"] = now_s - 1.01
    held["owner_lease"]["last_intent_wall_time_s"] = now_s - 1.01
    held["owner_lease"]["missed_intent_held"] = True
    session_artifact.write_text(json.dumps(held), encoding="utf-8")

    damping = watchdog_tick_from_artifact(
        session_artifact_path=session_artifact,
        max_heartbeat_age_s=1.0,
    )

    assert damping["status"] == "faulted"
    assert damping["mode"] == "damping"
    assert damping["owner"] is None
    assert damping["watchdog"]["reason"] == "agent_intent_fault_timeout"
    assert damping["watchdog"]["landing_mode"] == "damping"


def test_cli_runtime_status_blocks_stale_heartbeat(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "start",
            "--backend",
            "fake",
            "--q-current",
            "0.0",
            "0.3",
            "0.3",
            "--safe-center",
            "0.0",
            "0.3",
            "0.3",
            "--output",
            str(session_artifact),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    session["heartbeat"]["wall_time_s"] = time.time() - 10.0
    session_artifact.write_text(json.dumps(session), encoding="utf-8")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "status",
            "--session-artifact",
            str(session_artifact),
            "--max-heartbeat-age-s",
            "0.1",
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "blocked"
    assert payload["schema"] == "armctrl.arm_runtime_status.v1"
    assert payload["mode"] == "hold_safe"
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is False
    assert payload["readiness"]["failed_checks"] == [
        "heartbeat_fresh",
        "hold_fresh",
    ]


def test_cli_runtime_status_reports_active_owner_deadman(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _acquire_owner(
        session_artifact,
        owner="agent",
        mode="agent_servo",
        heartbeat_timeout_s=5.0,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "status",
            "--session-artifact",
            str(session_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "blocked"
    assert payload["mode"] == "agent_servo"
    assert payload["owner"] == "agent"
    assert payload["owner_deadman"]["owner"] == "agent"
    assert payload["owner_deadman"]["mode"] == "agent_servo"
    assert payload["owner_deadman"]["heartbeat_timeout_s"] == 5.0
    assert payload["owner_deadman"]["heartbeat_age_s"] >= 0.0
    assert payload["owner_deadman"]["fresh"] is True


def test_cli_runtime_recover_fake_returns_session_to_hold_safe(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    session["mode"] = "passive_safe"
    session["q_meas"] = [0.8, 0.0, 0.0]
    session["q_hold"] = [0.8, 0.0, 0.0]
    session["hold_fresh"] = False
    session["last_hold_wall_time_s"] = None
    session["readiness"] = {"agent_sysid_smoke_allowed": False}
    session_artifact.write_text(json.dumps(session), encoding="utf-8")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "recover",
            "--session-artifact",
            str(session_artifact),
            "--safe-center",
            "0.0",
            "0.3",
            "0.3",
            "--send-hz",
            "50",
            "--hold-hz",
            "50",
            "--output",
            str(session_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "blocked"
    assert payload["schema"] == "armctrl.arm_runtime_session.v1"
    assert payload["mode"] == "hold_safe"
    assert payload["owner"] is None
    assert payload["q_meas"] == [0.0, 0.3, 0.3]
    assert payload["q_hold"] == [0.0, 0.3, 0.3]
    assert payload["safe_center"] == [0.0, 0.3, 0.3]
    assert payload["recovery"]["status"] == "completed"
    assert payload["hold_tick_count"] == 0
    assert payload["hold_fresh"] is False
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is False
    assert payload["readiness"]["failed_checks"] == ["hold_fresh"]
    assert json.loads(session_artifact.read_text(encoding="utf-8")) == payload


def test_runtime_readiness_accepts_live_hold_away_from_safe_center(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    session["q_hold"] = [0.08, 0.3, 0.3]
    session["q_meas"] = [0.08, 0.3, 0.3]
    session["last_hold_wall_time_s"] = time.time()
    session["hold_fresh"] = True

    refreshed = refresh_runtime_status_payload(
        session,
        max_heartbeat_age_s=1.0,
    )

    assert refreshed["status"] == "ok"
    assert refreshed["readiness"]["agent_sysid_smoke_allowed"] is True
    assert refreshed["readiness"]["live_hold_allowed"] is True
    assert refreshed["readiness"]["safe_center_allowed"] is False
    assert "q_hold_close_to_safe_center" in refreshed["readiness"]["safe_center_failed_checks"]
    assert "q_hold_close_to_safe_center" not in refreshed["readiness"]["failed_checks"]


def test_cli_runtime_stop_fake_lands_damping_and_clears_owner(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _acquire_owner(session_artifact, owner="agent", mode="agent_servo")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "stop",
            "--session-artifact",
            str(session_artifact),
            "--output",
            str(session_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "stopped"
    assert payload["schema"] == "armctrl.arm_runtime_session.v1"
    assert payload["mode"] == "damping"
    assert payload["owner"] is None
    assert payload["owner_lease"] is None
    assert payload["landing_mode"] == "damping"
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is False
    assert "mode_hold_safe" in payload["readiness"]["failed_checks"]
    assert json.loads(session_artifact.read_text(encoding="utf-8")) == payload


def test_cli_sysid_readiness_accepts_fresh_live_runtime_status(
    tmp_path: Path,
) -> None:
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    runtime_status_artifact = tmp_path / "runtime_status.json"
    output_artifact = tmp_path / "readiness.json"
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(_passing_hold_damping_artifact()),
        encoding="utf-8",
    )
    runtime_status_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.arm_runtime_status.v1",
                "runtime_session_id": "runtime-test",
                "mode": "hold_safe",
                "owner": None,
                "q_meas": [0.0, 0.3, 0.3],
                "q_hold": [0.0, 0.3, 0.3],
                "safe_center": [0.0, 0.3, 0.3],
                "controller_dt_s": 0.002,
                "send_hz": 50.0,
                "hold_hz": 50.0,
                "hold_tick_count": 3,
                "last_hold_wall_time_s": time.time(),
                "hold_fresh": True,
                "fault_flags": [],
                "heartbeat": {
                    "fresh": True,
                    "age_s": 0.01,
                    "max_age_s": 5.0,
                    "wall_time_s": time.time(),
                },
                "readiness": {
                    "agent_sysid_smoke_allowed": True,
                    "failed_checks": [],
                },
            }
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "sdk-agent-sysid-smoke-readiness",
            "--doctor-artifact",
            str(doctor_artifact),
            "--hold-damping-artifact",
            str(hold_damping_artifact),
            "--runtime-status-artifact",
            str(runtime_status_artifact),
            "--max-heartbeat-age-s",
            "5",
            "--output",
            str(output_artifact),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["agent_sysid_smoke_allowed"] is True
    assert payload["prerequisites"]["runtime_status"] == "pass"
    assert payload["runtime_status"]["mode"] == "hold_safe"
    assert payload["runtime_status"]["runtime_session_id"] == "runtime-test"


def test_cli_runtime_owner_lease_rejects_competing_owner(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    first = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "acquire-owner",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--mode",
            "agent_servo",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--heartbeat-timeout-s",
            "0.5",
            "--max-heartbeat-age-s",
            "5",
            "--output",
            str(session_artifact),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    first_payload = json.loads(first.stdout)

    assert first_payload["status"] == "ok"
    assert first_payload["mode"] == "agent_servo"
    assert first_payload["owner"] == "agent"
    assert first_payload["owner_lease"]["owner"] == "agent"

    competing = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "acquire-owner",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "sysid",
            "--mode",
            "trajectory_replay",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--heartbeat-timeout-s",
            "0.5",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        capture_output=True,
        text=True,
    )
    competing_payload = json.loads(competing.stdout)

    assert competing.returncode == 3
    assert competing_payload["status"] == "rejected"
    assert competing_payload["reason"] == "runtime is owned by agent"
    assert competing_payload["owner"] == "agent"


def test_cli_runtime_release_owner_returns_to_hold_safe(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _acquire_owner(session_artifact, owner="sysid", mode="trajectory_replay")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "release-owner",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "sysid",
            "--output",
            str(session_artifact),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["mode"] == "hold_safe"
    assert payload["owner"] is None
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is False
    assert payload["readiness"]["failed_checks"] == ["hold_fresh"]
    assert payload["owner_lease"] is None
    assert json.loads(session_artifact.read_text(encoding="utf-8")) == payload


def test_cli_runtime_owner_heartbeat_refreshes_deadman(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _acquire_owner(
        session_artifact,
        owner="agent",
        mode="agent_servo",
        heartbeat_timeout_s=5.0,
    )
    stale_session = json.loads(session_artifact.read_text(encoding="utf-8"))
    acquired_wall_time_s = stale_session["owner_lease"]["acquired_wall_time_s"]
    stale_session["owner_lease"]["heartbeat_wall_time_s"] = time.time() - 1.0
    session_artifact.write_text(json.dumps(stale_session), encoding="utf-8")

    heartbeat = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "owner-heartbeat",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--output",
            str(session_artifact),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    heartbeat_payload = json.loads(heartbeat.stdout)

    assert heartbeat_payload["status"] == "blocked"
    assert heartbeat_payload["owner"] == "agent"
    assert heartbeat_payload["owner_lease"]["owner"] == "agent"
    assert heartbeat_payload["owner_lease"]["heartbeat_wall_time_s"] > acquired_wall_time_s
    assert heartbeat_payload["owner_lease"]["acquired_wall_time_s"] == acquired_wall_time_s
    assert heartbeat_payload["readiness"]["agent_sysid_smoke_allowed"] is False
    assert "no_owner" in heartbeat_payload["readiness"]["failed_checks"]

    watchdog = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "watchdog-tick",
            "--session-artifact",
            str(session_artifact),
            "--output",
            str(session_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )
    watchdog_payload = json.loads(watchdog.stdout)

    assert watchdog.returncode == 3
    assert watchdog_payload["status"] == "blocked"
    assert watchdog_payload["owner"] == "agent"
    assert watchdog_payload["mode"] == "agent_servo"
    assert "watchdog" not in watchdog_payload


def test_cli_runtime_watchdog_timeout_lands_damping(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _acquire_owner(
        session_artifact,
        owner="xbox",
        mode="agent_servo",
        heartbeat_timeout_s=0.1,
    )
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    session["owner_lease"]["heartbeat_wall_time_s"] = time.time() - 1.0
    session_artifact.write_text(json.dumps(session), encoding="utf-8")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "watchdog-tick",
            "--session-artifact",
            str(session_artifact),
            "--output",
            str(session_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "faulted"
    assert payload["mode"] == "damping"
    assert payload["owner"] is None
    assert payload["watchdog"]["landing_mode"] == "damping"
    assert payload["watchdog"]["reason"] == "owner_heartbeat_timeout"
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is False


def test_runtime_queue_executes_with_live_arm_runtime(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.02, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=5.0,
    )
    updated_session = json.loads(session_artifact.read_text(encoding="utf-8"))

    assert result is not None
    assert result["status"] == "completed"
    assert result["runtime_session_id"] == session["runtime_session_id"]
    assert result["timing"]["submitted_wall_time_s"] is not None
    assert result["timing"]["dequeued_wall_time_s"] >= result["timing"]["submitted_wall_time_s"]
    assert result["timing"]["owner_acquired_wall_time_s"] >= result["timing"]["dequeued_wall_time_s"]
    assert result["timing"]["first_send_wall_time_s"] >= result["timing"]["owner_acquired_wall_time_s"]
    assert result["timing"]["first_send_monotonic_s"] == 0.0
    assert result["timing"]["completed_wall_time_s"] >= result["timing"]["owner_acquired_wall_time_s"]
    assert result["timing"]["queue_latency_s"] >= 0.0
    assert result["timing"]["acquire_latency_s"] >= 0.0
    assert result["timing"]["first_send_latency_s"] >= 0.0
    assert result["timing"]["execution_elapsed_s"] == pytest.approx(0.02)
    assert result["artifacts"]["session"] == str(session_artifact)
    assert result["motion"]["runtime_send_hz"] == 50.0
    assert result["motion"]["resampling_policy"] == "none_sample_hz_matches_send_hz"
    assert result["motion"]["interpolation_policy"] == "pre_sampled_joint_positions"
    assert result["motion"]["send_period_s"]["min"] == pytest.approx(0.02)
    assert result["motion"]["send_period_s"]["max"] == pytest.approx(0.02)
    assert result["motion"]["send_period_s"]["avg"] == pytest.approx(0.02)
    assert result["motion"]["dt_min_s"] == pytest.approx(0.02)
    assert result["motion"]["dt_max_s"] == pytest.approx(0.02)
    assert result["motion"]["dt_avg_s"] == pytest.approx(0.02)
    assert result["motion"]["send_jitter_ms_summary"]["count"] == 1
    assert result["motion"]["send_jitter_ms_summary"]["max"] == pytest.approx(0.0)
    assert result["motion"]["send_jitter_ms_summary"]["buckets"] == {
        "le_0_5": 1,
        "le_1": 0,
        "le_2": 0,
        "le_5": 0,
        "gt_5": 0,
    }
    assert result["acceptance"]["status"] == "pass"
    assert result["acceptance"]["timing_gate"]["status"] == "pass"
    assert result["acceptance"]["timing_gate"]["checks"] == {
        "actual_send_hz_present": True,
        "actual_send_hz_close_to_runtime_send_hz": True,
        "send_jitter_p99_within_limit": True,
        "dt_range_within_limit": True,
        "queue_latency_present": True,
        "sample_count_matches_expected": True,
    }
    assert result["acceptance"]["timing_gate"]["metrics"]["runtime_send_hz"] == 50.0
    assert result["acceptance"]["timing_gate"]["metrics"]["sample_count"] == 2
    assert result["acceptance"]["timing_gate"]["metrics"]["expected_sample_count"] == 2
    assert updated_session["runtime_session_id"] == session["runtime_session_id"]
    assert updated_session["mode"] == "hold_safe"
    assert updated_session["owner"] is None
    assert updated_session["q_hold"] == [0.02, 0.3, 0.3]
    assert runtime.status()["q_hold"] == (0.02, 0.3, 0.3)
    result_artifact = json.loads(Path(submitted["artifacts"]["result"]).read_text())
    assert result_artifact["status"] == "completed"
    assert [
        sample["sent_monotonic_s"]
        for sample in result_artifact["motion"]["samples"]
    ] == [0.0, 0.02]


def test_cli_runtime_result_check_summarizes_passing_owner_result(
    tmp_path: Path,
    capsys,
) -> None:
    from armctrl import cli

    result_artifact = tmp_path / "runtime-result.json"
    result_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.arm_runtime_command_result.v1",
                "status": "completed",
                "owner": "sysid",
                "mode": "trajectory_replay",
                "runtime_session_id": "session-1",
                "motion": {
                    "status": "completed",
                    "sample_count": 801,
                    "trajectory_sample_hz": 100.0,
                    "runtime_send_hz": 100.0,
                    "resampling_policy": "none_sample_hz_matches_send_hz",
                    "interpolation_policy": "pre_sampled_joint_positions",
                    "actual_send_hz": 99.9,
                    "send_jitter_ms_p95": 0.4,
                    "send_jitter_ms_p99": 0.8,
                    "send_jitter_ms_summary": {"buckets": {"le_1": 800}},
                    "dt_min_s": 0.0098,
                    "dt_max_s": 0.0102,
                    "dt_avg_s": 0.01,
                    "landing_mode": "hold",
                },
                "timing": {
                    "queue_latency_s": 0.02,
                    "acquire_latency_s": 0.01,
                    "first_send_latency_s": 0.005,
                    "execution_elapsed_s": 8.0,
                },
                "acceptance": {
                    "status": "pass",
                    "timing_gate": {"status": "pass"},
                    "status_publish_gate": {"status": "pass"},
                },
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    exit_code = cli.main(
        [
            "runtime",
            "result-check",
            "--result-artifact",
            str(result_artifact),
            "--expect-owner",
            "sysid",
            "--expect-mode",
            "trajectory_replay",
            "--expect-sample-count",
            "801",
            "--max-jitter-p99-ms",
            "5",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 0
    assert payload["status"] == "pass"
    assert payload["owner"] == "sysid"
    assert payload["mode"] == "trajectory_replay"
    assert payload["sample_count"] == 801
    assert payload["checks"]["acceptance_passed"] is True
    assert payload["checks"]["owner_matches"] is True
    assert payload["checks"]["sample_count_matches"] is True
    assert payload["checks"]["jitter_p99_within_limit"] is True
    assert payload["metrics"]["send_jitter_ms_p99"] == pytest.approx(0.8)
    assert payload["metrics"]["dt_max_s"] == pytest.approx(0.0102)


def test_cli_runtime_result_check_can_find_latest_result_from_run_dir(
    tmp_path: Path,
    capsys,
) -> None:
    from armctrl import cli

    run_dir = tmp_path / "lab-run"
    results_dir = run_dir / "runtime_session_commands" / "results"
    results_dir.mkdir(parents=True)
    stale_result = results_dir / "stale.json"
    latest_result = results_dir / "latest.json"
    stale_result.write_text(
        json.dumps(
            {
                "schema": "armctrl.arm_runtime_command_result.v1",
                "status": "completed",
                "owner": "agent",
                "mode": "agent_servo",
                "motion": {
                    "status": "completed",
                    "sample_count": 2,
                    "send_jitter_ms_p99": 0.1,
                },
                "timing": {
                    "queue_latency_s": 0.01,
                    "first_send_latency_s": 0.01,
                    "execution_elapsed_s": 0.1,
                },
                "acceptance": {
                    "status": "pass",
                    "timing_gate": {"status": "pass"},
                    "status_publish_gate": {"status": "pass"},
                },
            }
        ),
        encoding="utf-8",
    )
    latest_result.write_text(
        json.dumps(
            {
                "schema": "armctrl.arm_runtime_command_result.v1",
                "status": "completed",
                "owner": "sysid",
                "mode": "trajectory_replay",
                "motion": {
                    "status": "completed",
                    "sample_count": 801,
                    "send_jitter_ms_p99": 0.2,
                },
                "timing": {
                    "queue_latency_s": 0.01,
                    "first_send_latency_s": 0.01,
                    "execution_elapsed_s": 8.0,
                },
                "acceptance": {
                    "status": "pass",
                    "timing_gate": {"status": "pass"},
                    "status_publish_gate": {"status": "pass"},
                },
            }
        ),
        encoding="utf-8",
    )
    stale_mtime = time.time() - 100.0
    latest_mtime = time.time()
    stale_result.touch()
    latest_result.touch()
    stale_result.chmod(0o644)
    latest_result.chmod(0o644)
    import os

    os.utime(stale_result, (stale_mtime, stale_mtime))
    os.utime(latest_result, (latest_mtime, latest_mtime))

    exit_code = cli.main(
        [
            "runtime",
            "result-check",
            "--run-dir",
            str(run_dir),
            "--expect-owner",
            "sysid",
            "--expect-sample-count",
            "801",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 0
    assert payload["status"] == "pass"
    assert payload["result_artifact"] == str(latest_result)
    assert payload["owner"] == "sysid"


def test_cli_runtime_result_check_all_summarizes_run_dir_results(
    tmp_path: Path,
    capsys,
) -> None:
    from armctrl import cli

    run_dir = tmp_path / "lab-run"
    results_dir = run_dir / "runtime_session_commands" / "results"
    results_dir.mkdir(parents=True)
    for name, owner, status, jitter in [
        ("agent.json", "agent", "pass", 0.4),
        ("sysid.json", "sysid", "pass", 0.8),
        ("recipe.json", "recipe", "fail", 9.0),
    ]:
        (results_dir / name).write_text(
            json.dumps(
                {
                    "schema": "armctrl.arm_runtime_command_result.v1",
                    "status": "completed",
                    "owner": owner,
                    "mode": (
                        "agent_servo" if owner == "agent" else "trajectory_replay"
                    ),
                    "motion": {
                        "status": "completed",
                        "sample_count": 2,
                        "runtime_send_hz": 50.0,
                        "actual_send_hz": 50.0,
                        "send_jitter_ms_p99": jitter,
                        "dt_max_s": 0.02,
                    },
                    "timing": {
                        "queue_latency_s": 0.01,
                        "first_send_latency_s": 0.01,
                        "execution_elapsed_s": 0.1,
                    },
                    "acceptance": {
                        "status": status,
                        "timing_gate": {"status": status},
                        "status_publish_gate": {"status": "pass"},
                    },
                }
            ),
            encoding="utf-8",
        )

    exit_code = cli.main(
        [
            "runtime",
            "result-check",
            "--run-dir",
            str(run_dir),
            "--all",
            "--max-jitter-p99-ms",
            "5",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "fail"
    assert payload["result_count"] == 3
    assert payload["pass_count"] == 2
    assert payload["fail_count"] == 1
    assert [result["owner"] for result in payload["results"]] == [
        "agent",
        "recipe",
        "sysid",
    ]
    assert payload["results"][1]["status"] == "fail"
    assert payload["next_gate"] == "inspect failed runtime result artifacts"


def test_cli_runtime_result_check_all_requires_expected_owners(
    tmp_path: Path,
    capsys,
) -> None:
    from armctrl import cli

    run_dir = tmp_path / "lab-run"
    results_dir = run_dir / "runtime_session_commands" / "results"
    results_dir.mkdir(parents=True)
    for name, owner in [("agent.json", "agent"), ("sysid.json", "sysid")]:
        (results_dir / name).write_text(
            json.dumps(
                {
                    "schema": "armctrl.arm_runtime_command_result.v1",
                    "status": "completed",
                    "owner": owner,
                    "mode": (
                        "agent_servo" if owner == "agent" else "trajectory_replay"
                    ),
                    "motion": {
                        "status": "completed",
                        "sample_count": 2,
                        "send_jitter_ms_p99": 0.2,
                    },
                    "timing": {
                        "queue_latency_s": 0.01,
                        "first_send_latency_s": 0.01,
                        "execution_elapsed_s": 0.1,
                    },
                    "acceptance": {
                        "status": "pass",
                        "timing_gate": {"status": "pass"},
                        "status_publish_gate": {"status": "pass"},
                    },
                }
            ),
            encoding="utf-8",
        )

    exit_code = cli.main(
        [
            "runtime",
            "result-check",
            "--run-dir",
            str(run_dir),
            "--all",
            "--require-owner",
            "agent",
            "--require-owner",
            "sysid",
            "--require-owner",
            "recipe",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "fail"
    assert payload["required_owners"] == ["agent", "sysid", "recipe"]
    assert payload["owners_present"] == ["agent", "sysid"]
    assert payload["missing_required_owners"] == ["recipe"]
    assert payload["next_gate"] == "run missing runtime owner commands"


def test_cli_runtime_result_check_rejects_failed_timing_gate(
    tmp_path: Path,
    capsys,
) -> None:
    from armctrl import cli

    result_artifact = tmp_path / "runtime-result-fail.json"
    result_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.arm_runtime_command_result.v1",
                "status": "completed",
                "owner": "sysid",
                "mode": "trajectory_replay",
                "motion": {
                    "status": "completed",
                    "sample_count": 801,
                    "runtime_send_hz": 100.0,
                    "actual_send_hz": 83.0,
                    "send_jitter_ms_p99": 20.0,
                    "dt_max_s": 0.021,
                },
                "timing": {
                    "queue_latency_s": 0.02,
                    "acquire_latency_s": 0.01,
                    "first_send_latency_s": 0.005,
                    "execution_elapsed_s": 8.0,
                },
                "acceptance": {
                    "status": "fail",
                    "timing_gate": {"status": "fail"},
                    "status_publish_gate": {"status": "pass"},
                },
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    exit_code = cli.main(
        [
            "runtime",
            "result-check",
            "--result-artifact",
            str(result_artifact),
            "--expect-owner",
            "sysid",
            "--expect-sample-count",
            "801",
            "--max-jitter-p99-ms",
            "5",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "fail"
    assert payload["checks"]["acceptance_passed"] is False
    assert payload["checks"]["timing_gate_passed"] is False
    assert payload["checks"]["jitter_p99_within_limit"] is False
    assert payload["next_gate"] == "inspect runtime result timing and safety evidence"


def test_cli_runtime_result_check_rejects_tracking_error_over_limit(
    tmp_path: Path,
    capsys,
) -> None:
    from armctrl import cli

    result_artifact = tmp_path / "runtime-result-tracking-fail.json"
    result_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.arm_runtime_command_result.v1",
                "status": "completed",
                "owner": "sysid",
                "mode": "trajectory_replay",
                "motion": {
                    "status": "completed",
                    "sample_count": 2,
                    "runtime_send_hz": 100.0,
                    "actual_send_hz": 100.0,
                    "send_jitter_ms_p99": 0.1,
                    "dt_min_s": 0.01,
                    "dt_max_s": 0.01,
                    "tracking_error": {"max_abs_rad": 0.032},
                },
                "timing": {
                    "queue_latency_s": 0.02,
                    "acquire_latency_s": 0.01,
                    "first_send_latency_s": 0.005,
                    "execution_elapsed_s": 0.01,
                },
                "acceptance": {
                    "status": "pass",
                    "timing_gate": {"status": "pass"},
                    "status_publish_gate": {"status": "pass"},
                },
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    exit_code = cli.main(
        [
            "runtime",
            "result-check",
            "--result-artifact",
            str(result_artifact),
            "--expect-owner",
            "sysid",
            "--max-tracking-error-rad",
            "0.02",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "fail"
    assert payload["checks"]["tracking_error_within_limit"] is False
    assert payload["metrics"]["tracking_error_max_abs_rad"] == pytest.approx(0.032)


def test_runtime_command_result_records_tracking_error_summary(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = LaggingReadbackBackend(lagging_q=(0.0, 0.3, 0.3))
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.01, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )
    result_artifact = json.loads(Path(submitted["artifacts"]["result"]).read_text())

    assert result is not None
    assert result["status"] == "completed"
    tracking = result["motion"]["tracking_error"]
    assert tracking["sample_count"] == 2
    assert tracking["max_abs_rad"] == pytest.approx(0.01)
    assert tracking["final_abs_rad"] == pytest.approx(0.01)
    assert tracking["per_joint_max_abs_rad"] == [0.01, 0.0, 0.0]
    assert tracking["final_error_rad"] == [-0.01, 0.0, 0.0]
    assert result_artifact["motion"]["tracking_error"] == tracking


def test_direct_agent_trajectory_submit_writes_shared_contract(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="agent",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.01, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
        max_joint_velocity_rad_s=1.0,
    )
    command = json.loads(Path(submitted["artifacts"]["command"]).read_text())

    assert command["artifact_policy"]["schema"] == (
        "armctrl.joint_trajectory_compiler_policy.v1"
    )
    assert command["artifact_policy"]["source"] == "agent"
    assert command["artifact_policy"]["dq_cmd"] == (
        "runtime_finite_difference_for_cubic_hermite_runtime"
    )
    assert command["runtime_trajectory_contract"]["schema"] == (
        "armctrl.joint_trajectory_contract.v1"
    )
    assert command["runtime_trajectory_contract"]["runtime_interpolator"] == (
        "cubic_hermite_joint_position_velocity"
    )
    assert command["runtime_trajectory_contract"]["runtime_revalidation_required"] is True


def test_runtime_command_tracking_error_limit_records_quality_without_online_abort(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = LaggingReadbackBackend(lagging_q=(0.0, 0.3, 0.3))
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.01, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )
    command_path = Path(submitted["artifacts"]["command"])
    command = json.loads(command_path.read_text(encoding="utf-8"))
    command["max_tracking_error_rad"] = 0.005
    command["tracking_error_grace_samples"] = 0
    command["tracking_error_consecutive_samples"] = 1
    command_path.write_text(json.dumps(command), encoding="utf-8")

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )
    result_artifact = json.loads(Path(submitted["artifacts"]["result"]).read_text())
    updated_session = json.loads(session_artifact.read_text(encoding="utf-8"))

    assert result is not None
    assert result["status"] == "completed"
    assert result["landing_mode"] == "hold"
    assert result["motion"]["error"] is None
    assert result["motion"]["tracking_error"]["max_abs_rad"] == pytest.approx(0.01)
    assert result["motion"]["tracking_error_policy"] == {
        "schema": "armctrl.tracking_error_policy.v1",
        "enabled": True,
        "max_tracking_error_rad": pytest.approx(0.005),
        "grace_samples": 0,
        "consecutive_samples": 1,
        "landing_mode_on_violation": None,
        "damping_on_tracking_error": False,
        "online_abort_on_tracking_error": False,
        "policy": "record_quality_evidence_only_result_check_decides_pass_fail",
    }
    assert result_artifact["status"] == "completed"
    check = _runtime_result_check_payload(
        result_artifact_path=Path(submitted["artifacts"]["result"]),
        expect_owner="sysid",
        expect_mode="trajectory_replay",
        expect_sample_count=2,
        max_jitter_p99_ms=None,
        max_tracking_error_rad=0.005,
    )
    assert check["status"] == "fail"
    assert check["checks"]["tracking_error_within_limit"] is False
    assert updated_session["status"] == "ok"
    assert updated_session["mode"] == "hold_safe"
    assert updated_session["owner"] is None
    assert updated_session["owner_lease"] is None
    assert backend.hold_count >= 1
    assert backend.damping_count == 0


def test_runtime_command_default_tracking_error_policy_debounces_transient_lag(
    tmp_path: Path,
) -> None:
    class StartupLagBackend(LaggingReadbackBackend):
        def __init__(self) -> None:
            super().__init__(lagging_q=(0.0, 0.3, 0.3))
            self._motion_read_count: int | None = None

        def begin_joint_trajectory(self, points) -> None:
            self._motion_read_count = 0

        def read_joint_state(self) -> JointStateSnapshot:
            if self._motion_read_count is None:
                return JointStateSnapshot(q_meas=self._last_q or self._lagging_q)
            self._motion_read_count += 1
            if self._motion_read_count <= 2:
                return JointStateSnapshot(q_meas=self._lagging_q)
            return JointStateSnapshot(q_meas=self._last_q or self._lagging_q)

    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = StartupLagBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="agent",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[
            (0.0, 0.3, 0.3),
            (0.01, 0.3, 0.3),
            (0.02, 0.3, 0.3),
            (0.03, 0.3, 0.3),
        ],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
        max_tracking_error_rad=0.005,
        max_joint_velocity_rad_s=1.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )

    assert result is not None
    assert result["status"] == "completed"
    assert result["landing_mode"] == "hold"
    assert result["motion"]["tracking_error_policy"]["grace_samples"] == 3
    assert result["motion"]["tracking_error_policy"]["consecutive_samples"] == 3
    assert result["motion"]["tracking_error"]["max_abs_rad"] == pytest.approx(0.01)
    assert backend.damping_count == 0


def test_runtime_command_result_records_tau_summary(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = TorqueReadbackBackend(tau_meas=(0.5, -1.25, 0.25))
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.01, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )
    result_artifact = json.loads(Path(submitted["artifacts"]["result"]).read_text())

    assert result is not None
    assert result["status"] == "completed"
    tau = result["motion"]["tau_meas"]
    assert tau["sample_count"] == 2
    assert tau["max_abs"] == pytest.approx(1.25)
    assert tau["final_abs"] == pytest.approx(1.25)
    assert tau["per_joint_max_abs"] == [0.5, 1.25, 0.25]
    assert tau["final_tau"] == [0.5, -1.25, 0.25]
    assert result_artifact["motion"]["tau_meas"] == tau


def test_runtime_command_tau_limit_lands_damping(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = TorqueReadbackBackend(tau_meas=(0.5, -1.25, 0.25))
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.01, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )
    command_path = Path(submitted["artifacts"]["command"])
    command = json.loads(command_path.read_text(encoding="utf-8"))
    command["max_tau_abs"] = 1.0
    command_path.write_text(json.dumps(command), encoding="utf-8")

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )
    result_artifact = json.loads(Path(submitted["artifacts"]["result"]).read_text())
    updated_session = json.loads(session_artifact.read_text(encoding="utf-8"))

    assert result is not None
    assert result["status"] == "faulted"
    assert result["landing_mode"] == "damping"
    assert result["motion"]["error"] == {
        "type": "torque_limit",
        "message": "max absolute tau exceeded",
    }
    assert result["motion"]["tau_meas"]["max_abs"] == pytest.approx(1.25)
    assert result["motion"]["max_tau_abs"] == pytest.approx(1.0)
    assert result_artifact["status"] == "faulted"
    assert updated_session["mode"] == "damping"
    assert backend.damping_count >= 1


def test_runtime_queue_records_live_hold_start_pose_policy(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.02, 0.3, 0.3)],
        send_hz=50.0,
        start_pose_policy="live_hold",
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )

    assert submitted["start_pose_policy"] == "live_hold"
    assert result is not None
    assert result["status"] == "completed"
    assert result["start_pose_policy"] == "live_hold"
    assert result["start_pose_guard"] == {
        "status": "pass",
        "policy": "live_hold",
        "expected_q_start": [0.0, 0.3, 0.3],
        "q_hold": [0.0, 0.3, 0.3],
        "q_meas": [0.0, 0.3, 0.3],
        "safe_center": [0.0, 0.3, 0.3],
        "max_start_error_rad": 0.02,
        "q_meas_to_q_hold_max_abs_rad": 0.0,
        "q_hold_to_expected_start_max_abs_rad": 0.0,
        "q_hold_to_safe_center_max_abs_rad": 0.0,
        "failed_checks": [],
    }


def test_runtime_queue_rejects_stale_live_hold_evidence(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    stale_wall_time_s = time.time() - 5.0
    session["heartbeat"]["wall_time_s"] = stale_wall_time_s
    session["last_hold_wall_time_s"] = stale_wall_time_s
    session["hold_fresh"] = False
    session_artifact.write_text(
        json.dumps(session, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    with pytest.raises(RuntimeSessionError, match="runtime live hold evidence is stale"):
        submit_trajectory_command(
            session_artifact_path=session_artifact,
            owner="recipe",
            expected_q_start=(0.0, 0.3, 0.3),
            q_points=[(0.0, 0.3, 0.3), (0.01, 0.3, 0.3)],
            send_hz=50.0,
            max_start_error_rad=0.02,
            heartbeat_timeout_s=0.5,
            max_heartbeat_age_s=5.0,
        )


def test_runtime_queue_rejects_explicit_start_policy_before_preposition(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    with pytest.raises(Exception, match="explicit_q start pose requires runtime q_hold"):
        submit_trajectory_command(
            session_artifact_path=session_artifact,
            owner="recipe",
            expected_q_start=(0.2, 0.3, 0.3),
            q_points=[(0.2, 0.3, 0.3), (0.21, 0.3, 0.3)],
            send_hz=50.0,
            start_pose_policy="explicit_q",
            max_start_error_rad=0.02,
            heartbeat_timeout_s=0.5,
            max_heartbeat_age_s=1.0,
        )


def test_cli_runtime_preposition_enables_explicit_q_owner_submit(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    explicit_start = (0.10, 0.3, 0.3)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "preposition",
            "--session-artifact",
            str(session_artifact),
            "--q-target",
            *[str(value) for value in explicit_start],
            "--send-hz",
            "50",
            "--max-joint-step-rad",
            "0.05",
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))

    preposition_result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=5.0,
    )
    updated_session = json.loads(session_artifact.read_text(encoding="utf-8"))
    updated_session["q_hold"] = list(explicit_start)
    updated_session = record_runtime_hold_tick(
        updated_session,
        q_meas=explicit_start,
        fault_flags=(),
        max_heartbeat_age_s=5.0,
    )
    session_artifact.write_text(
        json.dumps(updated_session, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    explicit_submit = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="recipe",
        expected_q_start=explicit_start,
        q_points=[explicit_start, (0.11, 0.3, 0.3)],
        send_hz=50.0,
        start_pose_policy="explicit_q",
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
    )

    assert payload["status"] == "queued"
    assert payload["schema"] == "armctrl.runtime_preposition.v1"
    assert payload["start_pose_policy"] == "live_hold"
    assert payload["q_start"] == [0.0, 0.3, 0.3]
    assert payload["q_target"] == list(explicit_start)
    assert command["owner"] == "runtime_preposition"
    assert command["start_pose_policy"] == "live_hold"
    assert command["q_points"][0] == [0.0, 0.3, 0.3]
    assert command["q_points"][-1] == list(explicit_start)
    assert preposition_result is not None
    assert preposition_result["status"] == "completed"
    assert updated_session["q_hold"] == list(explicit_start)
    assert explicit_submit["status"] == "queued"
    assert explicit_submit["start_pose_policy"] == "explicit_q"


def test_runtime_queue_holds_final_command_when_readback_lags(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = LaggingReadbackBackend(lagging_q=(0.0, 0.3, 0.3))
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.02, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )
    updated_session = json.loads(session_artifact.read_text(encoding="utf-8"))

    assert result is not None
    assert result["status"] == "completed"
    assert result["motion"]["samples"][-1]["q_cmd"] == [0.02, 0.3, 0.3]
    assert result["motion"]["samples"][-1]["q_meas"] == [0.0, 0.3, 0.3]
    assert updated_session["q_hold"] == [0.02, 0.3, 0.3]
    assert runtime.status()["q_hold"] == (0.02, 0.3, 0.3)


def test_runtime_queue_allows_repeated_trajectory_from_live_hold(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))

    first_submit = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.02, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )
    first_result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )
    first_session = json.loads(session_artifact.read_text(encoding="utf-8"))

    second_submit = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=tuple(first_session["q_hold"]),
        q_points=[
            tuple(first_session["q_hold"]),
            (first_session["q_hold"][0] + 0.01, 0.3, 0.3),
        ],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )
    second_result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )

    assert first_result is not None
    assert second_result is not None
    assert first_result["status"] == "completed"
    assert second_result["status"] == "completed"
    assert first_submit["command_id"] != second_submit["command_id"]
    assert first_session["q_hold"] == [0.02, 0.3, 0.3]
    assert second_result["motion"]["samples"][0]["q_cmd"] == first_session["q_hold"]
    assert second_result["motion"]["samples"][-1]["q_cmd"] == [0.03, 0.3, 0.3]


def test_runtime_queue_executes_pending_commands_by_submit_time_not_filename(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    queue_dir = runtime_command_queue_dir(session_artifact)
    pending_dir = queue_dir / "pending"
    result_dir = queue_dir / "results"
    pending_dir.mkdir(parents=True)
    result_dir.mkdir(parents=True)

    early_command = {
        "schema": "armctrl.arm_runtime_command.v1",
        "command_id": "z-earlier",
        "kind": "joint_trajectory",
        "owner": "sysid",
        "mode": "trajectory_replay",
        "submitted_wall_time_s": 10.0,
        "send_hz": 50.0,
        "trajectory_sample_hz": 50.0,
        "expected_q_start": [0.0, 0.3, 0.3],
        "q_points": [[0.0, 0.3, 0.3], [0.01, 0.3, 0.3]],
        "max_start_error_rad": 0.02,
        "heartbeat_timeout_s": 0.5,
        "start_pose_policy": "live_hold",
        "result_artifact": str(result_dir / "z-earlier.json"),
    }
    late_command = {
        **early_command,
        "command_id": "a-later",
        "submitted_wall_time_s": 20.0,
        "q_points": [[0.0, 0.3, 0.3], [0.02, 0.3, 0.3]],
        "result_artifact": str(result_dir / "a-later.json"),
    }
    (pending_dir / "a-later.json").write_text(
        json.dumps(late_command),
        encoding="utf-8",
    )
    (pending_dir / "z-earlier.json").write_text(
        json.dumps(early_command),
        encoding="utf-8",
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )

    assert result is not None
    assert result["command_id"] == "z-earlier"
    assert result["motion"]["samples"][-1]["q_cmd"] == [0.01, 0.3, 0.3]


def test_runtime_queue_resamples_trajectory_when_runtime_send_hz_differs(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[
            (0.0, 0.3, 0.3),
            (0.20, 0.3, 0.3),
        ],
        dq_points=[
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ],
        send_hz=50.0,
        trajectory_sample_hz=10.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )
    result_artifact = json.loads(Path(submitted["artifacts"]["result"]).read_text())

    assert result is not None
    assert result["status"] == "completed"
    assert result["motion"]["trajectory_sample_hz"] == 10.0
    assert result["motion"]["runtime_send_hz"] == 50.0
    assert result["motion"]["resampling_policy"] == "cubic_hermite_time_resample"
    assert (
        result["motion"]["interpolation_policy"]
        == "bounded_cubic_hermite_joint_position_velocity"
    )
    assert result["motion"]["sample_count"] == 6
    assert result["motion"]["samples"][0]["q_cmd"] == [0.0, 0.3, 0.3]
    assert result["motion"]["samples"][1]["q_cmd"][0] == pytest.approx(0.0208)
    assert result["motion"]["samples"][1]["dq_cmd"][0] == pytest.approx(1.92)
    assert result["motion"]["samples"][3]["q_cmd"][0] == pytest.approx(0.1296)
    assert result["motion"]["samples"][-1]["q_cmd"] == [0.2, 0.3, 0.3]
    assert result["motion"]["send_period_s"]["avg"] == pytest.approx(0.02)
    assert result_artifact["motion"]["runtime_send_hz"] == 50.0


def test_runtime_queue_throttles_owner_status_writes(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from armctrl import runtime_ipc

    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    write_count = 0
    original_write = runtime_ipc._write_json_atomic

    def counting_write(path: Path, payload: dict[str, object]) -> None:
        nonlocal write_count
        if path == session_artifact:
            write_count += 1
        original_write(path, payload)

    monkeypatch.setattr(runtime_ipc, "_write_json_atomic", counting_write)
    q_points = [(index * 0.0001, 0.3, 0.3) for index in range(801)]
    submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=q_points,
        send_hz=100.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=2.0,
        max_heartbeat_age_s=1.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )

    assert result is not None
    assert result["status"] == "completed"
    assert result["motion"]["sample_count"] == 801
    assert write_count < 50


def test_submit_trajectory_command_preserves_ddq_and_artifact_policy(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    policy = {
        "schema": "armctrl.sysid_runtime_compiler_policy.v1",
        "q_cmd": "preserved",
        "dq_cmd": "preserved",
        "ddq_cmd": "preserved",
    }
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.01, 0.3, 0.3)],
        dq_points=[(0.0, 0.0, 0.0), (0.1, 0.0, 0.0)],
        ddq_points=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        artifact_policy=policy,
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )

    command = json.loads(
        Path(submitted["artifacts"]["command"]).read_text(encoding="utf-8")
    )
    assert command["dq_points"] == [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]]
    assert command["ddq_points"] == [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
    assert command["artifact_policy"] == policy


def test_runtime_queue_writes_owner_active_status_before_motion(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.02, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )
    observed: dict[str, object] = {}
    from armctrl.runtime_ipc import _execute_motion_command as original_execute_motion

    def observe_active_status(command: dict[str, object], *, runtime, **kwargs):
        observed.update(json.loads(session_artifact.read_text(encoding="utf-8")))
        return original_execute_motion(command, runtime=runtime, **kwargs)

    monkeypatch.setattr(
        "armctrl.runtime_ipc._execute_motion_command",
        observe_active_status,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )

    assert result is not None
    assert observed["mode"] == "trajectory_replay"
    assert observed["owner"] == "sysid"
    assert observed["owner_lease"]["owner"] == "sysid"
    assert observed["owner_lease"]["mode"] == "trajectory_replay"
    assert observed["owner_lease"]["acquired_wall_time_s"] >= 0.0
    assert observed["owner_deadman"]["owner"] == "sysid"
    assert observed["owner_deadman"]["mode"] == "trajectory_replay"
    assert observed["owner_deadman"]["fresh"] is True
    assert observed["readiness"]["agent_sysid_smoke_allowed"] is False
    assert "no_owner" in observed["readiness"]["failed_checks"]


def test_runtime_queue_uses_executor_progress_over_stale_owner_deadman_after_motion(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.02, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.01,
        max_heartbeat_age_s=1.0,
    )
    from armctrl.runtime_ipc import _execute_motion_command as original_execute_motion

    def age_owner_lease(command: dict[str, object], *, runtime, **kwargs):
        motion = original_execute_motion(command, runtime=runtime, **kwargs)
        active = json.loads(session_artifact.read_text(encoding="utf-8"))
        active["owner_lease"]["heartbeat_wall_time_s"] = time.time() - 1.0
        active["owner_lease"]["executor_progress_wall_time_s"] = time.time()
        session_artifact.write_text(json.dumps(active), encoding="utf-8")
        return motion

    monkeypatch.setattr(
        "armctrl.runtime_ipc._execute_motion_command",
        age_owner_lease,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )
    updated_session = json.loads(session_artifact.read_text(encoding="utf-8"))
    result_artifact = json.loads(Path(submitted["artifacts"]["result"]).read_text())

    assert result is not None
    assert result["status"] == "completed"
    assert result["landing_mode"] == "hold"
    assert "watchdog" not in result
    assert result_artifact["status"] == "completed"
    assert updated_session["status"] == "ok"
    assert updated_session["mode"] == "hold_safe"
    assert updated_session["owner"] is None
    assert updated_session["owner_lease"] is None
    assert backend.damping_count == 0


def test_runtime_queue_refreshes_owner_heartbeat_during_long_trajectory(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    monkeypatch.setattr("armctrl.runtime_ipc.time.time", clock.monotonic)
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[
            (0.0, 0.3, 0.3),
            (0.01, 0.3, 0.3),
            (0.02, 0.3, 0.3),
            (0.03, 0.3, 0.3),
        ],
        send_hz=100.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.015,
        max_heartbeat_age_s=1.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )
    updated_session = json.loads(session_artifact.read_text(encoding="utf-8"))
    result_artifact = json.loads(Path(submitted["artifacts"]["result"]).read_text())

    assert result is not None
    assert result["status"] == "completed"
    assert result["landing_mode"] == "hold"
    assert "watchdog" not in result
    assert result["status_update"]["min_period_s"] == pytest.approx(0.05)
    assert result["status_update"]["target_status_hz"] == pytest.approx(20.0)
    assert result["status_update"]["send_loop_write_policy"] == (
        "deferred_publish_outside_send_loop"
    )
    assert result_artifact["status"] == "completed"
    assert updated_session["mode"] == "hold_safe"
    assert updated_session["owner"] is None
    assert updated_session["owner_deadman"] is None


def test_runtime_queue_throttles_session_writes_during_100hz_sysid_trajectory(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    monkeypatch.setattr("armctrl.runtime_ipc.time.time", clock.monotonic)
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[
            (0.001 * index, 0.3, 0.3)
            for index in range(801)
        ],
        send_hz=100.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=1.0,
        max_heartbeat_age_s=1.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )
    result_artifact = json.loads(Path(submitted["artifacts"]["result"]).read_text())

    assert result is not None
    assert result["status"] == "completed"
    assert result["motion"]["sample_count"] == 801
    assert result["status_update"]["sample_count"] == 801
    assert result["status_update"]["session_write_count"] < 50
    assert result["status_update"]["session_write_count"] < (
        result["motion"]["sample_count"] / 10
    )
    status_publish_gate = result["acceptance"]["status_publish_gate"]
    assert status_publish_gate["status"] == "pass"
    assert status_publish_gate["checks"] == {
        "status_update_present": True,
        "session_writes_below_sample_count": True,
        "session_writes_below_10_percent_samples": True,
        "send_loop_write_policy_deferred": True,
        "target_status_hz_within_limit": True,
    }
    assert status_publish_gate["metrics"]["sample_count"] == 801
    assert status_publish_gate["metrics"]["session_write_count"] == (
        result["status_update"]["session_write_count"]
    )
    assert status_publish_gate["metrics"]["session_write_ratio"] == pytest.approx(
        result["status_update"]["session_write_count"] / 801
    )
    assert status_publish_gate["metrics"]["target_status_hz"] == pytest.approx(20.0)
    assert status_publish_gate["metrics"]["min_period_s"] == pytest.approx(0.05)
    assert status_publish_gate["metrics"]["send_loop_write_policy"] == (
        "deferred_publish_outside_send_loop"
    )
    assert result_artifact["status_update"] == result["status_update"]
    assert (
        result_artifact["acceptance"]["status_publish_gate"]
        == result["acceptance"]["status_publish_gate"]
    )


def test_runtime_send_loop_does_not_write_session_artifact_per_sample(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    monkeypatch.setattr("armctrl.runtime_ipc.time.time", clock.monotonic)
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.001 * index, 0.3, 0.3) for index in range(101)],
        send_hz=100.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=1.0,
        max_heartbeat_age_s=1.0,
    )
    from armctrl import runtime_ipc
    from armctrl.motion_runtime import MotionRuntime

    in_motion = False
    session_writes_during_motion = 0
    original_write_json_atomic = runtime_ipc._write_json_atomic
    original_execute_trajectory = MotionRuntime.execute_trajectory

    def capture_writes(path: Path, payload: dict[str, object]) -> None:
        nonlocal session_writes_during_motion
        if in_motion and Path(path) == session_artifact:
            session_writes_during_motion += 1
        original_write_json_atomic(path, payload)

    def mark_motion_window(self, *args, **kwargs):
        nonlocal in_motion
        in_motion = True
        try:
            return original_execute_trajectory(self, *args, **kwargs)
        finally:
            in_motion = False

    monkeypatch.setattr(runtime_ipc, "_write_json_atomic", capture_writes)
    monkeypatch.setattr(MotionRuntime, "execute_trajectory", mark_motion_window)

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )

    assert result is not None
    assert result["status"] == "completed"
    assert result["motion"]["sample_count"] == 101
    assert session_writes_during_motion == 0
    result_artifact = json.loads(Path(submitted["artifacts"]["result"]).read_text())
    assert result_artifact["status_update"]["session_write_count"] == 1
    assert result_artifact["status_update"]["policy"] == (
        "deferred_throttled_active_owner_status"
    )
    assert result_artifact["acceptance"]["status_publish_gate"]["status"] == "pass"
    assert result_artifact["acceptance"]["status_publish_gate"]["checks"][
        "session_writes_below_sample_count"
    ] is True


def test_active_owner_status_publish_does_not_refresh_owner_deadman(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    owner_heartbeat_wall_time_s = 123.0
    session["mode"] = "trajectory_replay"
    session["owner"] = "sysid"
    session["owner_lease"] = {
        "schema": "armctrl.arm_runtime_owner_lease.v1",
        "runtime_session_id": session["runtime_session_id"],
        "owner": "sysid",
        "mode": "trajectory_replay",
        "heartbeat_timeout_s": 10.0,
        "heartbeat_wall_time_s": owner_heartbeat_wall_time_s,
        "acquired_wall_time_s": 100.0,
        "landing_policy": "watchdog_to_damping_release_to_hold_safe",
    }
    session_artifact.write_text(json.dumps(session), encoding="utf-8")
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    from armctrl import runtime_ipc

    monkeypatch.setattr("armctrl.runtime_ipc.time.time", lambda: 456.0)

    runtime_ipc._refresh_active_owner_session(
        session_artifact_path=session_artifact,
        runtime=runtime,
        owner="sysid",
        mode="trajectory_replay",
        heartbeat_timeout_s=10.0,
        max_heartbeat_age_s=1.0,
    )
    updated = json.loads(session_artifact.read_text(encoding="utf-8"))

    assert updated["owner_lease"]["heartbeat_wall_time_s"] == owner_heartbeat_wall_time_s
    assert updated["owner_lease"]["executor_progress_wall_time_s"] == 456.0
    assert updated["owner_deadman"]["heartbeat_wall_time_s"] == owner_heartbeat_wall_time_s
    assert updated["status_publish"]["wall_time_s"] == 456.0
    assert updated["status_publish"]["policy"] == (
        "throttled_observation_not_owner_heartbeat"
    )


def test_runtime_command_result_records_signed_period_error_summary(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    monkeypatch.setattr("armctrl.runtime_ipc.time.time", clock.monotonic)
    submitted = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[
            (0.0, 0.3, 0.3),
            (0.01, 0.3, 0.3),
            (0.02, 0.3, 0.3),
        ],
        send_hz=100.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=1.0,
        max_heartbeat_age_s=1.0,
    )

    from armctrl.motion_runtime import MotionRuntime

    original_execute_trajectory = MotionRuntime.execute_trajectory

    def inject_late_second_sample(self, *args, **kwargs):
        result = original_execute_trajectory(self, *args, **kwargs)
        samples = list(result.samples)
        samples[1] = replace(samples[1], sent_monotonic_s=0.03)
        samples[2] = replace(samples[2], sent_monotonic_s=0.04)
        return replace(result, samples=tuple(samples))

    monkeypatch.setattr(MotionRuntime, "execute_trajectory", inject_late_second_sample)

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )
    result_artifact = json.loads(Path(submitted["artifacts"]["result"]).read_text())

    assert result is not None
    assert result["status"] == "completed"
    assert result["motion"]["dt_error_ms_summary"]["max"] == pytest.approx(20.0)
    assert result["motion"]["dt_error_ms_summary"]["buckets"]["late_gt_5"] == 1
    assert result["motion"]["dt_error_ms_summary"]["buckets"]["on_time_le_0_5"] == 1
    assert result_artifact["motion"]["dt_error_ms_summary"] == result["motion"][
        "dt_error_ms_summary"
    ]


def test_runtime_queue_preserves_trajectory_velocity_commands(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    backend = FakeMotionBackend()
    observed_dq: list[tuple[float, ...] | None] = []
    original_send = backend.send_joint_command

    def capture_send(*args, **kwargs):
        observed_dq.append(kwargs.get("dq"))
        return original_send(*args, **kwargs)

    backend.send_joint_command = capture_send
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.01, 0.3, 0.3)],
        dq_points=[(0.1, 0.0, 0.0), (0.1, 0.0, 0.0)],
        send_hz=100.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=1.0,
        max_heartbeat_age_s=1.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )

    assert result is not None
    assert result["status"] == "completed"
    assert observed_dq[-2:] == [(0.1, 0.0, 0.0), (0.1, 0.0, 0.0)]
    assert result["motion"]["samples"][0]["dq_cmd"] == [0.1, 0.0, 0.0]


def test_runtime_queue_records_joint_trajectory_shaping_contract(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="agent",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.006, 0.3, 0.3)],
        dq_points=[(0.06, 0.0, 0.0), (0.06, 0.0, 0.0)],
        artifact_policy={
            "schema": "armctrl.joint_trajectory_compiler_policy.v1",
            "source": "agent",
            "q_cmd": "generated_smoothstep_joint_target",
            "dq_cmd": "derived_smoothstep_analytic",
            "ddq_cmd": "derived_smoothstep_analytic",
        },
        send_hz=10.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=1.0,
        max_heartbeat_age_s=1.0,
        max_joint_segment_delta_rad=0.01,
        max_joint_velocity_rad_s=0.25,
        max_joint_acceleration_rad_s2=1.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )

    assert result is not None
    assert result["status"] == "completed"
    motion = result["motion"]
    assert motion["command_space"] == "joint"
    assert motion["joint_trajectory_safety"]["status"] == "pass"
    assert motion["max_joint_segment_delta_rad"] == pytest.approx(0.01)
    assert motion["max_joint_velocity_rad_s"] == pytest.approx(0.25)
    assert motion["joint_trajectory_safety"]["max_joint_acceleration_rad_s2"] == (
        pytest.approx(1.0)
    )
    assert motion["joint_trajectory_safety"][
        "max_segment_abs_acceleration_rad_s2"
    ] == pytest.approx(0.0)
    assert motion["q_policy"] == "generated_smoothstep_joint_target"
    assert motion["dq_policy"] == "derived_smoothstep_analytic"
    assert motion["ddq_policy"] == "derived_smoothstep_analytic"
    assert motion["runtime_interpolator"] == "cubic_hermite_joint_position_velocity"
    assert motion["runtime_velocity_source"] == "preserved_or_compiled_dq_points"


def test_runtime_queue_passes_owner_watchdog_into_motion_loop(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.02, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.01,
        max_heartbeat_age_s=1.0,
    )
    captured: dict[str, object] = {}
    from armctrl.motion_runtime import MotionRuntime

    original_execute_trajectory = MotionRuntime.execute_trajectory

    def capture_watchdog(self, *args, **kwargs):
        watchdog = kwargs.get("watchdog")
        captured["watchdog_passed"] = watchdog is not None
        active = json.loads(session_artifact.read_text(encoding="utf-8"))
        active["owner_lease"]["heartbeat_wall_time_s"] = time.time() - 1.0
        session_artifact.write_text(json.dumps(active), encoding="utf-8")
        captured["watchdog_event"] = watchdog()
        return original_execute_trajectory(self, *args, **kwargs)

    monkeypatch.setattr(MotionRuntime, "execute_trajectory", capture_watchdog)

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )

    assert result is not None
    assert captured["watchdog_passed"] is True
    assert captured["watchdog_event"]["reason"] == "owner_heartbeat_timeout"
    assert result["status"] == "faulted"


def test_runtime_queue_agent_intent_records_frequency_and_missed_intent_policy(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend.send_joint_command(
        tuple(float(value) for value in session["q_meas"]),
        producer="test_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=tuple(float(value) for value in session["safe_center"]),
        runtime_session_id=str(session["runtime_session_id"]),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=tuple(float(value) for value in session["q_hold"]))
    observed_owner_leases: list[dict[str, object]] = []
    from armctrl import runtime_ipc

    original_refresh = runtime_ipc._refresh_active_owner_session

    def capture_active_owner_refresh(*args, **kwargs):
        observed_owner_leases.append(
            json.loads(session_artifact.read_text(encoding="utf-8"))["owner_lease"]
        )
        return original_refresh(*args, **kwargs)

    monkeypatch.setattr(
        runtime_ipc,
        "_refresh_active_owner_session",
        capture_active_owner_refresh,
    )
    submitted = submit_intent_command(
        session_artifact_path=session_artifact,
        owner="agent",
        expected_q_start=(0.0, 0.3, 0.3),
        q_target=(0.01, 0.3, 0.3),
        control_period_s=0.1,
        send_hz=50.0,
        max_joint_delta_rad=0.02,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=1.0,
        max_heartbeat_age_s=1.0,
        max_joint_acceleration_rad_s2=10.0,
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=1.0,
    )
    result_artifact = json.loads(Path(submitted["artifacts"]["result"]).read_text())

    assert result is not None
    assert result["status"] == "completed"
    assert result["owner"] == "agent"
    assert result["motion"]["mode"] == "agent_servo"
    assert result["motion"]["agent_intent_hz"] == pytest.approx(10.0)
    assert result["motion"]["runtime_send_hz"] == 50.0
    assert result["motion"]["interpolation_policy"] == "smoothstep_intent_frame"
    assert result["motion"]["max_joint_velocity_rad_s"] == pytest.approx(0.25)
    assert result["motion"]["max_joint_acceleration_rad_s2"] == pytest.approx(10.0)
    assert result["motion"]["joint_intent_safety"]["status"] == "pass"
    assert result["motion"]["joint_intent_safety"][
        "max_joint_acceleration_rad_s2"
    ] == pytest.approx(10.0)
    assert result["motion"]["joint_intent_safety"][
        "max_abs_acceleration_rad_s2"
    ] == pytest.approx(6.0)
    assert result["motion"]["intent_trajectory_contract"]["schema"] == (
        "armctrl.joint_intent_trajectory_contract.v1"
    )
    assert result["motion"]["intent_trajectory_contract"]["q_policy"] == (
        "live_hold_to_target_smoothstep"
    )
    assert result["motion"]["intent_trajectory_contract"]["dq_policy"] == (
        "derived_smoothstep_analytic"
    )
    assert result["motion"]["intent_trajectory_contract"]["ddq_policy"] == (
        "derived_smoothstep_analytic_not_commanded"
    )
    assert result["motion"]["intent_trajectory_contract"][
        "max_abs_acceleration_rad_s2"
    ] == pytest.approx(6.0)
    assert result["motion"]["intent_trajectory_contract"][
        "expected_runtime_sample_count"
    ] == 6
    for contract_q, sample in zip(
        result["motion"]["intent_trajectory_contract"]["q_points"],
        result["motion"]["samples"],
        strict=True,
    ):
        assert contract_q == pytest.approx(sample["q_cmd"])
    assert result["motion"]["intent_trajectory_contract"]["dq_points"][0] == [
        0.0,
        0.0,
        0.0,
    ]
    assert result["motion"]["intent_trajectory_contract"]["dq_points"][-1] == [
        0.0,
        0.0,
        0.0,
    ]
    assert len(result["motion"]["intent_trajectory_contract"]["ddq_points"]) == 6
    assert result["motion"]["resampling_policy"] == "intent_frame_to_runtime_send_hz"
    assert result["motion"]["missed_intent_policy"] == "hold_then_damping"
    assert result["motion"]["missed_intent_timeout_s"] == pytest.approx(0.3)
    assert result["motion"]["fault_timeout_s"] == pytest.approx(1.0)
    assert result["motion"]["sample_count"] == 6
    assert result["motion"]["samples"][0]["q_cmd"] == [0.0, 0.3, 0.3]
    assert result["motion"]["samples"][-1]["q_cmd"] == [0.01, 0.3, 0.3]
    assert result_artifact["motion"]["missed_intent_policy"] == "hold_then_damping"
    assert result_artifact["motion"]["intent_trajectory_contract"] == (
        result["motion"]["intent_trajectory_contract"]
    )
    assert observed_owner_leases
    assert observed_owner_leases[0]["landing_policy"] == "missed_intent_hold_then_damping"
    assert observed_owner_leases[0]["last_intent_wall_time_s"] is not None
    assert observed_owner_leases[0]["missed_intent_timeout_s"] == pytest.approx(0.3)
    assert observed_owner_leases[0]["fault_timeout_s"] == pytest.approx(1.0)
    assert observed_owner_leases[0]["missed_intent_held"] is False


def test_cli_runtime_serve_executes_queued_trajectory_and_returns_to_hold(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"

    server = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "start",
            "--backend",
            "fake",
            "--q-current",
            "0.0",
            "0.3",
            "0.3",
            "--safe-center",
            "0.0",
            "0.3",
            "0.3",
            "--serve",
            "--heartbeat-period-s",
            "0.02",
            "--max-heartbeat-age-s",
            "1.0",
            "--output",
            str(session_artifact),
            "--json",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        _wait_for_session_artifact(session_artifact)
        submit = subprocess.run(
            [
                sys.executable,
                "-m",
                "armctrl.cli",
                "runtime",
                "submit-trajectory",
                "--session-artifact",
                str(session_artifact),
                "--owner",
                "sysid",
                "--expected-q-start",
                "0.0",
                "0.3",
                "0.3",
                "--send-hz",
                "50",
                "--q-point",
                "0.00",
                "0.30",
                "0.30",
                "--q-point",
                "0.01",
                "0.30",
                "0.30",
                "--output",
                str(tmp_path / "submit.json"),
                "--json",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        submitted = json.loads(submit.stdout)

        assert submitted["status"] == "queued"
        assert submitted["owner"] == "sysid"
        result = _wait_for_runtime_command_result(
            Path(submitted["artifacts"]["result"])
        )
        session = _wait_for_session_artifact(session_artifact)

        assert result["status"] == "completed"
        assert result["owner"] == "sysid"
        assert result["mode"] == "trajectory_replay"
        assert result["motion"]["status"] == "completed"
        assert result["motion"]["sample_count"] == 2
        assert result["motion"]["samples"][0]["q_cmd"] == [0.0, 0.3, 0.3]
        assert result["motion"]["samples"][0]["q_meas"] == [0.0, 0.3, 0.3]
        assert result["motion"]["samples"][0]["fault_flags"] == []
        assert result["motion"]["samples"][0]["producer"] == "sysid"
        assert result["motion"]["samples"][0]["mode"] == "trajectory_replay"
        assert result["motion"]["samples"][1]["q_cmd"] == [0.01, 0.3, 0.3]
        assert result["landing_mode"] == "hold"
        assert session["mode"] == "hold_safe"
        assert session["owner"] is None
        assert session["q_hold"] == [0.01, 0.3, 0.3]
        assert session["q_meas"] == [0.01, 0.3, 0.3]
        assert session["readiness"]["agent_sysid_smoke_allowed"] is True

        subprocess.run(
            [
                sys.executable,
                "-m",
                "armctrl.cli",
                "runtime",
                "stop",
                "--session-artifact",
                str(session_artifact),
                "--output",
                str(session_artifact),
                "--json",
            ],
            capture_output=True,
            text=True,
        )
        _wait_for_runtime_stopped(session_artifact)
        stdout, stderr = server.communicate(timeout=8.0)
        assert server.returncode == 0
        assert stderr == ""
        assert json.loads(stdout)["status"] == "stopped"
    finally:
        if server.poll() is None:
            server.terminate()
            server.communicate(timeout=3.0)


def test_cli_runtime_start_fake_serve_executes_eef_with_configured_adapter(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"

    server = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "start",
            "--backend",
            "fake",
            "--q-current",
            "0.0",
            "0.3",
            "0.3",
            "--safe-center",
            "0.0",
            "0.3",
            "0.3",
            "--serve",
            "--eef-adapter",
            "moveit_servo",
            "--heartbeat-period-s",
            "0.02",
            "--max-heartbeat-age-s",
            "1.0",
            "--output",
            str(session_artifact),
            "--json",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        served = _wait_for_session_artifact(session_artifact)
        assert served["mode"] == "hold_safe"
        served_status = refresh_runtime_status_payload(
            served,
            max_heartbeat_age_s=1.0,
        )
        assert served_status["eef_adapter_manager"]["status"] == "ready"
        assert served_status["eef_adapter_manager"]["configured_adapters"] == [
            "moveit_servo"
        ]
        assert served_status["eef_adapter_manager"]["adapter_status"]["moveit_servo"][
            "configured"
        ] is True
        assert served_status["eef_adapter_manager"]["adapter_status"]["moveit_servo"][
            "executable"
        ] is True
        assert served_status["eef_adapter_manager"]["adapter_status"]["moveit_servo"][
            "hardware_scope"
        ] == "fake_rehearsal"
        assert (
            served_status["eef_adapter_manager"]["primary_backend_fallback_allowed"]
            is False
        )
        assert (
            served_status["eef_adapter_manager"]["disconnected_takeover_allowed"]
            is False
        )
        assert served_status["eef_adapter_manager"]["eef_command_executable"] is True
        assert (
            served_status["runtime_controller_manager"]["controllers"]["eef_servo"][
                "status"
            ]
            == "available"
        )
        assert (
            served_status["runtime_controller_manager"]["controllers"]["joint_trajectory"][
                "contract"
            ]
            == "q_points/dq_points/ddq_points/sample_hz/send_hz"
        )

        submit_completed = subprocess.run(
            [
                sys.executable,
                "-m",
                "armctrl.cli",
                "motion",
                "submit",
                "eef-delta",
                "--session-artifact",
                str(session_artifact),
                "--owner",
                "agent",
                "--backend",
                "moveit_servo",
                "--expected-q-start",
                "0.0",
                "0.3",
                "0.3",
                "--delta-position",
                "0.001",
                "0.0",
                "0.0",
                "--delta-rpy",
                "0.0",
                "0.0",
                "0.0",
                "--control-period-s",
                "0.1",
                "--send-hz",
                "50",
                "--output",
                str(tmp_path / "eef_submit.json"),
                "--json",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        submit_payload = json.loads(submit_completed.stdout)
        result_path = Path(submit_payload["artifacts"]["result"])
        result = _wait_for_runtime_command_result(result_path)

        assert result["status"] == "completed"
        assert result["owner"] == "agent"
        assert result["mode"] == "agent_servo"
        assert result["command_space"] == "eef"
        assert result["motion"]["eef_command"]["backend"] == "moveit_servo"
        assert result["motion"]["eef_command"]["runtime_backend"] == "fake"
        assert result["motion"]["eef_command"]["eef_adapter"] == "moveit_servo"
        assert (
            result["motion"]["eef_command"]["adapter_resolution"]
            == "configured_adapter_registry"
        )
        assert result["motion"]["sample_count"] > 0

        stop_completed = _run_runtime_stop_with_retry(session_artifact)
        assert stop_completed.returncode == 3
        _wait_for_runtime_stopped(session_artifact)
        stdout, stderr = server.communicate(timeout=8.0)
        assert server.returncode == 0
        assert stderr == ""
        assert json.loads(stdout)["status"] == "stopped"
    finally:
        if server.poll() is None:
            server.terminate()
            server.communicate(timeout=3.0)


def test_runtime_serve_loop_drains_ready_queue_before_sleep(
    tmp_path: Path,
    monkeypatch,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3),
        safe_center=(0.0, 0.3, 0.3),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=1.0,
    )
    payload = record_runtime_hold_tick(
        payload,
        q_meas=(0.0, 0.3, 0.3),
        fault_flags=(),
        max_heartbeat_age_s=1.0,
    )
    session_artifact.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    backend = FakeMotionBackend()
    backend.send_joint_command(
        (0.0, 0.3, 0.3),
        producer="runtime_serve_bootstrap",
        mode=MotionMode.HOLD,
        monotonic_s=0.0,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=(0.0, 0.3, 0.3),
    )
    runtime.mark_hold_safe(q_hold=(0.0, 0.3, 0.3))
    first = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.0, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )
    second = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="recipe",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.0, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )

    def request_stop_instead_of_sleep(_duration_s: float) -> None:
        _runtime_stop_request_path(session_artifact).write_text(
            "stop\n",
            encoding="utf-8",
        )

    monkeypatch.setattr("armctrl.cli.time.sleep", request_stop_instead_of_sleep)

    stopped = _serve_runtime_session_until_stopped(
        session_artifact_path=session_artifact,
        heartbeat_period_s=0.5,
        max_heartbeat_age_s=1.0,
        backend=backend,
        runtime=runtime,
    )

    assert stopped["status"] == "stopped"
    assert Path(first["artifacts"]["result"]).exists()
    assert Path(second["artifacts"]["result"]).exists()
    assert json.loads(Path(first["artifacts"]["result"]).read_text())["status"] == (
        "completed"
    )
    assert json.loads(Path(second["artifacts"]["result"]).read_text())["status"] == (
        "completed"
    )


def test_cli_runtime_submit_trajectory_rejects_busy_owner(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    _acquire_owner(session_artifact, owner="agent", mode="agent_servo")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "submit-trajectory",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "sysid",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.0",
            "0.3",
            "0.3",
            "--json",
        ],
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["reason"] == "runtime is owned by agent"
    assert payload["movement_command_sent"] is False


def test_cli_runtime_submit_trajectory_records_tracking_limit(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "submit-trajectory",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "sysid",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--send-hz",
            "100",
            "--max-tracking-error-rad",
            "0.04",
            "--max-tau-abs",
            "2.5",
            "--q-point",
            "0.0",
            "0.3",
            "0.3",
            "--q-point",
            "0.01",
            "0.3",
            "0.3",
            "--output",
            str(tmp_path / "trajectory_submit.json"),
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))

    assert payload["status"] == "queued"
    assert command["kind"] == "joint_trajectory"
    assert command["max_tracking_error_rad"] == 0.04
    assert command["max_tau_abs"] == 2.5


def test_runtime_submit_retries_transient_session_read_race(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from armctrl import runtime_ipc

    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    original_read_text = Path.read_text
    attempts = 0

    def flaky_read_text(path: Path, *args, **kwargs):
        nonlocal attempts
        if path == session_artifact and attempts == 0:
            attempts += 1
            raise FileNotFoundError(path)
        return original_read_text(path, *args, **kwargs)

    monkeypatch.setattr(runtime_ipc.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(Path, "read_text", flaky_read_text)

    payload = submit_trajectory_command(
        session_artifact_path=session_artifact,
        owner="sysid",
        expected_q_start=(0.0, 0.3, 0.3),
        q_points=[(0.0, 0.3, 0.3), (0.01, 0.3, 0.3)],
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=1.0,
    )

    assert attempts == 1
    assert payload["status"] == "queued"


def test_cli_runtime_submit_intent_queues_agent_servo_command(tmp_path: Path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "submit-intent",
            "--session-artifact",
            str(session_artifact),
            "--owner",
            "agent",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--q-target",
            "0.005",
            "0.3",
            "0.3",
            "--control-period-s",
            "0.1",
            "--send-hz",
            "50",
            "--max-joint-delta-rad",
            "0.01",
            "--max-tracking-error-rad",
            "0.03",
            "--max-tau-abs",
            "1.5",
            "--output",
            str(tmp_path / "intent_submit.json"),
            "--max-heartbeat-age-s",
            "5",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(Path(payload["artifacts"]["command"]).read_text(encoding="utf-8"))

    assert payload["status"] == "queued"
    assert payload["owner"] == "agent"
    assert payload["mode"] == "agent_servo"
    assert payload["movement_command_sent"] is False
    assert command["kind"] == "joint_intent"
    assert command["q_target"] == [0.005, 0.3, 0.3]
    assert command["max_tracking_error_rad"] == 0.03
    assert command["max_tau_abs"] == 1.5


def _start_fake_hold_session(path: Path) -> None:
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "start",
            "--backend",
            "fake",
            "--q-current",
            "0.0",
            "0.3",
            "0.3",
            "--safe-center",
            "0.0",
            "0.3",
            "0.3",
            "--output",
            str(path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload = record_runtime_hold_tick(
        payload,
        q_meas=(0.0, 0.3, 0.3),
        fault_flags=(),
        max_heartbeat_age_s=1.0,
    )
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _configure_eef_adapter_manager(
    path: Path,
    *,
    adapter: str,
    primary_backend: str = "fake",
    live_reference: bool = False,
) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    manager = eef_adapter_manager_payload(
        primary_backend=primary_backend,
        configured_adapters=[adapter],
        live_reference_adapters=[adapter] if live_reference else [],
    )
    payload["eef_adapter_manager"] = manager
    payload["runtime_controller_manager"] = runtime_controller_manager_payload(
        backend=primary_backend,
        eef_adapter_manager=manager,
    )
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return payload


def _wait_for_session_artifact(path: Path) -> dict[str, object]:
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if path.exists():
            payload = _read_json_with_retry(path)
            if payload is not None:
                return payload
        time.sleep(0.02)
    raise AssertionError(f"runtime session artifact was not written: {path}")


def _wait_for_runtime_command_result(path: Path) -> dict[str, object]:
    deadline = time.time() + 8.0
    while time.time() < deadline:
        if path.exists():
            payload = _read_json_with_retry(path)
            if payload is not None:
                return payload
        time.sleep(0.02)
    raise AssertionError(f"runtime command result was not written: {path}")


def _wait_for_runtime_stopped(path: Path) -> dict[str, object]:
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if path.exists():
            payload = _read_json_with_retry(path)
            if (
                payload is not None
                and payload.get("status") == "stopped"
                and payload.get("mode") == "damping"
            ):
                return payload
        time.sleep(0.02)
    raise AssertionError(f"runtime session did not stop: {path}")


def _run_runtime_stop_with_retry(path: Path) -> subprocess.CompletedProcess[str]:
    last_completed: subprocess.CompletedProcess[str] | None = None
    for _ in range(10):
        completed = subprocess.run(
            [
                sys.executable,
                "-m",
                "armctrl.cli",
                "runtime",
                "stop",
                "--session-artifact",
                str(path),
                "--output",
                str(path),
                "--json",
            ],
            capture_output=True,
            text=True,
        )
        last_completed = completed
        if completed.returncode == 3:
            return completed
        time.sleep(0.05)
    assert last_completed is not None
    return last_completed


def _read_json_with_retry(path: Path) -> dict[str, object] | None:
    for _ in range(10):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, PermissionError):
            time.sleep(0.01)
            continue
        if not isinstance(payload, dict):
            raise AssertionError(f"{path} did not contain a JSON object")
        return payload
    return None


def _acquire_owner(
    path: Path,
    *,
    owner: str,
    mode: str,
    heartbeat_timeout_s: float = 0.5,
) -> None:
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "runtime",
            "acquire-owner",
            "--session-artifact",
            str(path),
            "--owner",
            owner,
            "--mode",
            mode,
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--heartbeat-timeout-s",
            str(heartbeat_timeout_s),
            "--max-heartbeat-age-s",
            "5",
            "--output",
            str(path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
