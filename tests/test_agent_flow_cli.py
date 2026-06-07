import json
import subprocess
import sys
from pathlib import Path

import pytest

from armctrl.agent_flow import (
    AGENT_FLOW_REAL_RUNTIME_CONFIRMATION,
    AgentFlowRealRuntimeSmokeRequest,
    AgentFlowRealRuntimeSmoker,
)
from armctrl.motion_runtime import FakeMotionBackend, JointStateSnapshot


class ManualClock:
    def __init__(self) -> None:
        self.now_s = 0.0

    def monotonic(self) -> float:
        return self.now_s

    def sleep(self, seconds: float) -> None:
        self.now_s += max(0.0, seconds)


class FakeArx5AgentBackend(FakeMotionBackend):
    controller_dt_s = 0.01

    def __init__(self) -> None:
        super().__init__()
        self.enter_hold_or_damping_count = 0

    def enter_hold_or_damping(self) -> None:
        self.enter_hold_or_damping_count += 1


class FaultingArx5AgentBackend(FakeArx5AgentBackend):
    def __init__(self, *, fault_on_read: int) -> None:
        super().__init__()
        self._fault_on_read = fault_on_read
        self._read_count = 0

    def read_joint_state(self) -> JointStateSnapshot:
        self._read_count += 1
        if self._read_count >= self._fault_on_read:
            return JointStateSnapshot(
                q_meas=self._last_q or (),
                fault_flags=("over_current",),
            )
        return super().read_joint_state()


class AbortingArx5AgentBackend(FakeArx5AgentBackend):
    def __init__(self, *, abort_on_send: int) -> None:
        super().__init__()
        self._abort_on_send = abort_on_send
        self._send_count = 0

    def send_joint_command(self, *args, **kwargs) -> None:
        self._send_count += 1
        if self._send_count >= self._abort_on_send:
            raise RuntimeError("simulated SDK send failure")
        super().send_joint_command(*args, **kwargs)


def test_cli_agent_flow_doctor_reports_available_nonhardware_paths() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "doctor",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.agent_flow_doctor.v1"
    assert payload["movement_allowed"] is False
    assert payload["read_only"] is True
    assert payload["surfaces"]["recipe"]["status"] == "available"
    assert payload["surfaces"]["eef"]["status"] == "available"
    assert payload["surfaces"]["lerobot"]["status"] == "available"
    assert payload["surfaces"]["runtime_backends"]["sdk_cartesian"] in {"available", "missing"}
    assert payload["surfaces"]["runtime_backends"]["moveit_servo"] in {
        "available",
        "missing",
        "installed_not_sourced",
    }
    assert payload["surfaces"]["runtime_backends"]["lerobot_rollout"] == "contract_available"


def test_cli_agent_flow_plan_for_lerobot_pose_delta_closes_shared_review(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "agent-flow-lerobot"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_delta",
            "--backend",
            "lerobot_rollout",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.agent_flow_plan.v1"
    assert payload["movement_allowed"] is False
    assert payload["agent_motion_contract"]["schema"] == "armctrl.agent_motion_contract.v1"
    assert payload["agent_motion_contract"]["selected_action_id"] == "eef.pose_delta"
    assert payload["recommended_path"]["schema"] == "armctrl.recommended_path.v1"
    assert payload["recommended_path"]["profile"] == "preset_then_bounded_eef_then_review"
    assert payload["recommended_path"]["primary_entrypoint"] == "armctrl agent-flow plan"
    assert payload["recommended_path"]["steps"][2]["id"] == "export_agent_session_plan"
    assert "export-agent-session-plan" in payload["recommended_path"]["steps"][2]["command"]
    assert payload["agent_motion_contract"]["lerobot_compatibility"]["status"] == "compatible"
    assert (
        payload["agent_motion_contract"]["lerobot_compatibility"]["preferred_training_action_id"]
        == "eef.pose_delta"
    )
    assert (
        payload["agent_motion_contract"]["lerobot_compatibility"]["processor_owner"]["action"]
        == "robot_action_processor"
    )
    assert payload["preset"]["recipe"]["name"] == "home"
    assert payload["eef"]["plan"]["agent_action"]["action_id"] == "eef.pose_delta"
    assert payload["runtime"]["backend"] == "lerobot_rollout"
    assert payload["runtime"]["helper_plan"]["schema"] == "armctrl.lerobot_agent_runtime_helper_plan.v1"
    assert payload["runtime"]["processor_contract"]["schema"] == "armctrl.lerobot_eef_processor_contract.v1"
    assert payload["runtime"]["helper_plan"]["runtime_boundary"] == {
        "runtime_owner": "native_lerobot_rollout",
        "armctrl_role": "processor_contract_audit_only",
        "motion_runtime_owner": False,
        "hardware_execution": "outside_armctrl",
    }
    assert payload["runtime"]["processor_contract"]["runtime_boundary"] == {
        "runtime_owner": "native_lerobot_rollout",
        "armctrl_role": "processor_contract_audit_only",
        "motion_runtime_owner": False,
        "hardware_execution": "outside_armctrl",
    }
    assert payload["review"]["review_status"] == "completed"
    assert payload["review"]["sim_preview"]["safety"]["allowed"] is True
    assert payload["artifacts"]["recipe_plan_dir"] == str(output_dir / "recipe-plan")
    assert payload["artifacts"]["eef_plan_dir"] == str(output_dir / "eef-plan")
    assert payload["artifacts"]["agent_flow_contract"] == str(output_dir / "agent_flow_plan.json")
    assert (output_dir / "agent_flow_plan.json").exists()
    assert payload["ordered_steps"][0]["id"] == "plan_recipe_preset"
    assert any(step["id"] == "review_runtime_handoff" for step in payload["ordered_steps"])

    saved = json.loads((output_dir / "agent_flow_plan.json").read_text(encoding="utf-8"))
    assert saved["schema"] == "armctrl.agent_flow_plan.v1"
    assert saved["runtime"]["backend"] == "lerobot_rollout"


def test_cli_agent_flow_plan_for_sdk_pose_absolute_exports_helper_and_preview(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "agent-flow-sdk"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_absolute",
            "--backend",
            "sdk_cartesian",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["eef"]["plan"]["agent_action"]["action_id"] == "eef.pose_absolute"
    assert payload["agent_motion_contract"]["selected_action_id"] == "eef.pose_absolute"
    assert payload["recommended_path"]["steps"][2]["id"] == "export_agent_session_plan"
    assert "--backend sdk_cartesian" in payload["recommended_path"]["steps"][2]["command"]
    assert payload["runtime"]["backend"] == "sdk_cartesian"
    assert payload["runtime"]["runner_contract"]["schema"] == "armctrl.eef_runner_contract.v1"
    assert payload["runtime"]["helper_plan"]["schema"] == "armctrl.sdk_cartesian_helper_plan.v1"
    assert payload["review"]["schema"] == "armctrl.eef_runner_preview.v1"
    assert payload["review"]["review_status"] == "completed"
    assert payload["review"]["sim_preview"]["safety"]["allowed"] is True
    assert payload["artifacts"]["agent_flow_contract"] == str(output_dir / "agent_flow_plan.json")


def test_cli_agent_flow_plan_requires_mode_specific_arguments() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_delta",
            "--backend",
            "sdk_cartesian",
            "--output",
            "runs/agent-flow-invalid",
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert "--delta-position and --delta-rpy are required for --eef-mode pose_delta" in completed.stderr


def test_cli_agent_flow_review_replays_saved_contract(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "agent-flow-review"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_delta",
            "--backend",
            "lerobot_rollout",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--output",
            str(output_dir),
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
            "agent-flow",
            "review",
            "--contract",
            str(output_dir / "agent_flow_plan.json"),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.agent_flow_review.v1"
    assert payload["movement_allowed"] is False
    assert payload["contract_path"] == str(output_dir / "agent_flow_plan.json")
    assert payload["backend"] == "lerobot_rollout"
    assert payload["recommended_path"]["profile"] == "preset_then_bounded_eef_then_review"
    assert payload["review"]["review_status"] == "completed"
    assert payload["review"]["sim_preview"]["safety"]["allowed"] is True


def test_cli_agent_flow_runtime_smoke_fake_replays_checked_intent_contract(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "agent-flow-runtime"
    runtime_log = tmp_path / "agent-runtime-smoke.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_delta",
            "--backend",
            "sdk_cartesian",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--control-period-s",
            "0.1",
            "--output",
            str(output_dir),
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
            "agent-flow",
            "runtime-smoke-fake",
            "--contract",
            str(output_dir / "agent_flow_plan.json"),
            "--q-start",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--q-target",
            "0",
            "0.302",
            "0.3",
            "0",
            "0",
            "0",
            "--send-hz",
            "50",
            "--output",
            str(runtime_log),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.agent_flow_runtime_smoke.v1"
    assert payload["movement_allowed"] is False
    assert payload["hardware_motion"] is False
    assert payload["producer"] == "agent"
    assert payload["runtime"]["backend"] == "fake"
    assert payload["runtime"]["mode"] == "agent_servo"
    assert payload["intent"]["control_period_s"] == 0.1
    assert payload["intent"]["agent_intent_hz"] == 10.0
    assert payload["intent"]["max_joint_delta_rad"] == 0.005
    assert payload["frequency_contract"] == {
        "schema": "armctrl.agent_frequency_contract.v1",
        "agent_intent_hz": 10.0,
        "agent_control_period_s": 0.1,
        "preview_sample_hz": 50.0,
        "backend_send_hz": 50.0,
        "motion_runtime_trajectory_sample_hz": 50.0,
        "actual_send_hz": 50.0,
        "interpolation_owner": "motion_runtime",
        "controller_dt_s": None,
        "missed_intent_timeout_s": 0.3,
        "missed_intent_landing_mode": "hold",
        "fault_timeout_s": 1.0,
        "fault_landing_mode": "damping",
        "missed_intent_exercised_in_this_run": True,
    }
    assert payload["motion_runtime"]["trajectory_sample_hz"] == 50.0
    assert payload["motion_runtime"]["actual_send_hz"] == 50.0
    assert payload["motion_runtime"]["sample_count"] == 6
    assert payload["motion_runtime"]["landing_mode"] == "released"
    assert payload["motion_runtime"]["tracking"] == {
        "q_cmd_delta_rad": [0.0, 0.0020000000000000018, 0.0, 0.0, 0.0, 0.0],
        "q_meas_delta_rad": [0.0, 0.0020000000000000018, 0.0, 0.0, 0.0, 0.0],
        "q_cmd_delta_max_abs_rad": 0.0020000000000000018,
        "q_meas_delta_max_abs_rad": 0.0020000000000000018,
        "max_abs_sample_tracking_error_rad": 0.0,
        "final_tracking_error_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "final_tracking_error_max_abs_rad": 0.0,
    }
    assert payload["watchdog"]["missed_intent"] == {
        "landing_mode": "hold",
        "age_s": 0.31,
        "missed_intent_timeout_s": 0.3,
    }
    assert payload["watchdog"]["policy"] == {
        "missed_intent_timeout_s": 0.3,
        "missed_intent_landing_mode": "hold",
        "fault_timeout_s": 1.0,
        "fault_landing_mode": "damping",
        "missed_intent_exercised_in_this_run": True,
    }
    assert payload["watchdog"]["backend_hold_count"] == 1
    assert payload["motion_runtime"]["samples"][-1]["q_cmd"] == [
        0.0,
        0.302,
        0.3,
        0.0,
        0.0,
        0.0,
    ]
    assert payload["artifacts"]["runtime_log"] == str(runtime_log)

    saved = json.loads(runtime_log.read_text(encoding="utf-8"))
    assert saved == payload


def test_cli_agent_flow_runtime_smoke_fake_acquires_runtime_owner_lease(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "agent-flow-runtime-lease"
    runtime_log = tmp_path / "agent-runtime-smoke.json"
    runtime_session = tmp_path / "runtime-session.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_delta",
            "--backend",
            "sdk_cartesian",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--control-period-s",
            "0.1",
            "--output",
            str(output_dir),
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
            "runtime",
            "start",
            "--backend",
            "fake",
            "--q-current",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--safe-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--output",
            str(runtime_session),
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
            "agent-flow",
            "runtime-smoke-fake",
            "--contract",
            str(output_dir / "agent_flow_plan.json"),
            "--runtime-session-artifact",
            str(runtime_session),
            "--q-start",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--q-target",
            "0",
            "0.302",
            "0.3",
            "0",
            "0",
            "0",
            "--send-hz",
            "50",
            "--output",
            str(runtime_log),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    final_session = json.loads(runtime_session.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["runtime"]["owner"] == "agent"
    assert payload["runtime"]["single_owner_runtime_session"] is True
    assert (
        payload["runtime"]["runtime_session_id"]
        == final_session["runtime_session_id"]
    )
    assert payload["runtime"]["owner_lease"]["owner"] == "agent"
    assert payload["runtime"]["owner_lease"]["mode"] == "agent_servo"
    assert final_session["mode"] == "hold_safe"
    assert final_session["owner"] is None
    assert final_session["owner_lease"] is None
    assert final_session["readiness"]["agent_sysid_smoke_allowed"] is True


def test_cli_agent_flow_runtime_smoke_fake_writes_rejected_artifact_for_unsafe_delta(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "agent-flow-runtime-reject"
    runtime_log = tmp_path / "agent-runtime-rejected.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_delta",
            "--backend",
            "sdk_cartesian",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--control-period-s",
            "0.1",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    rejected = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "runtime-smoke-fake",
            "--contract",
            str(output_dir / "agent_flow_plan.json"),
            "--q-start",
            "0",
            "0",
            "--q-target",
            "0.02",
            "0",
            "--output",
            str(runtime_log),
            "--json",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    payload = json.loads(rejected.stdout)

    assert rejected.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.agent_flow_runtime_smoke.v1"
    assert payload["movement_allowed"] is False
    assert payload["hardware_motion"] is False
    assert payload["producer"] == "agent"
    assert payload["reason"].startswith("max_joint_delta_rad exceeded")
    assert payload["artifacts"]["runtime_log"] == str(runtime_log)

    saved = json.loads(runtime_log.read_text(encoding="utf-8"))
    assert saved == payload


def test_agent_flow_real_runtime_smoke_requires_live_runtime_artifact(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "agent-flow-real-requires-runtime"
    readiness_artifact = tmp_path / "readiness.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_delta",
            "--backend",
            "sdk_cartesian",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--control-period-s",
            "0.1",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    readiness_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_agent_smoke_readiness.v1",
                "agent_sysid_smoke_allowed": True,
                "prerequisites": {"tiny_motion": "pass"},
            }
        ),
        encoding="utf-8",
    )

    def forbidden_backend_factory(**kwargs):
        raise AssertionError("backend must not be opened without live runtime")

    smoker = AgentFlowRealRuntimeSmoker(backend_factory=forbidden_backend_factory)

    with pytest.raises(RuntimeError, match="live runtime session artifact"):
        smoker.run(
            AgentFlowRealRuntimeSmokeRequest(
                contract_path=output_dir / "agent_flow_plan.json",
                readiness_artifact_path=readiness_artifact,
                model="X5",
                interface="can0",
                q_start=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
                q_target=(0.0, 0.302, 0.3, 0.0, 0.0, 0.0),
                confirm=AGENT_FLOW_REAL_RUNTIME_CONFIRMATION,
            )
        )


def test_agent_flow_real_runtime_smoke_blocks_direct_sdk_execution_with_runtime(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "agent-flow-real-blocked"
    readiness_artifact = tmp_path / "readiness.json"
    runtime_session = tmp_path / "runtime-session.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "agent-flow",
            "plan",
            "--preset",
            "home",
            "--eef-mode",
            "pose_delta",
            "--backend",
            "sdk_cartesian",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--control-period-s",
            "0.1",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    readiness_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_agent_smoke_readiness.v1",
                "agent_sysid_smoke_allowed": True,
                "prerequisites": {
                    "doctor": "pass",
                    "hold_damping": "pass",
                    "tiny_motion": "pass",
                },
                "tiny_motion": {
                    "controller_dt_s": 0.002,
                    "tracking": {
                        "q_cmd_delta_max_abs_rad": 0.002,
                        "q_meas_delta_max_abs_rad": 0.002,
                        "final_tracking_error_max_abs_rad": 0.0,
                    }
                },
            }
        ),
        encoding="utf-8",
    )

    def forbidden_backend_factory(**kwargs):
        raise AssertionError("direct SDK backend must stay disabled")

    smoker = AgentFlowRealRuntimeSmoker(backend_factory=forbidden_backend_factory)

    with pytest.raises(RuntimeError, match="live MotionRuntime IPC"):
        smoker.run(
            AgentFlowRealRuntimeSmokeRequest(
                contract_path=output_dir / "agent_flow_plan.json",
                readiness_artifact_path=readiness_artifact,
                model="X5",
                interface="can0",
                q_start=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
                q_target=(0.0, 0.302, 0.3, 0.0, 0.0, 0.0),
                confirm=AGENT_FLOW_REAL_RUNTIME_CONFIRMATION,
                runtime_session_artifact_path=runtime_session,
            )
        )


def test_cli_agent_flow_runtime_smoke_real_routes_to_gated_smoker(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    contract_path = tmp_path / "agent_flow_plan.json"
    readiness_artifact = tmp_path / "readiness.json"
    runtime_session = tmp_path / "runtime-session.json"
    output_artifact = tmp_path / "agent_real_smoke.json"
    contract_path.write_text(
        json.dumps({"schema": "armctrl.agent_flow_plan.v1"}),
        encoding="utf-8",
    )
    readiness_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_agent_smoke_readiness.v1",
                "agent_sysid_smoke_allowed": True,
            }
        ),
        encoding="utf-8",
    )
    calls: list[AgentFlowRealRuntimeSmokeRequest] = []

    class StubRealRuntimeSmoker:
        def run(self, request: AgentFlowRealRuntimeSmokeRequest) -> dict[str, object]:
            calls.append(request)
            raise AssertionError("real Agent smoke must not bypass live runtime IPC")

    monkeypatch.setattr(cli, "AgentFlowRealRuntimeSmoker", StubRealRuntimeSmoker)

    exit_code = cli.main(
        [
            "agent-flow",
            "runtime-smoke-real",
            "--contract",
            str(contract_path),
            "--readiness-artifact",
            str(readiness_artifact),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--q-start",
            "0",
            "0.3",
            "--q-target",
            "0",
            "0.302",
            "--confirm",
            AGENT_FLOW_REAL_RUNTIME_CONFIRMATION,
            "--runtime-session-artifact",
            str(runtime_session),
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert calls == []
    assert payload["status"] == "blocked"
    assert payload["schema"] == "armctrl.agent_flow_runtime_smoke_real.v1"
    assert payload["hardware_motion"] is False
    assert payload["movement_command_sent"] is False
    assert payload["runtime"]["single_owner_runtime_session"] is True
    assert payload["runtime"]["runtime_session_artifact"] == str(runtime_session)
    assert payload["reason"] == (
        "real Agent execution must be submitted through live MotionRuntime IPC; "
        "direct SDK execution is disabled"
    )
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_agent_flow_runtime_smoke_real_requires_runtime_session_artifact(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    contract_path = tmp_path / "agent_flow_plan.json"
    readiness_artifact = tmp_path / "readiness.json"
    output_artifact = tmp_path / "agent_real_smoke_requires_runtime.json"
    contract_path.write_text(
        json.dumps({"schema": "armctrl.agent_flow_plan.v1"}),
        encoding="utf-8",
    )
    readiness_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_agent_smoke_readiness.v1",
                "agent_sysid_smoke_allowed": True,
            }
        ),
        encoding="utf-8",
    )

    class ForbiddenRealRuntimeSmoker:
        def run(self, request: AgentFlowRealRuntimeSmokeRequest) -> dict[str, object]:
            raise AssertionError("real Agent smoke must not open SDK without runtime")

    monkeypatch.setattr(cli, "AgentFlowRealRuntimeSmoker", ForbiddenRealRuntimeSmoker)

    exit_code = cli.main(
        [
            "agent-flow",
            "runtime-smoke-real",
            "--contract",
            str(contract_path),
            "--readiness-artifact",
            str(readiness_artifact),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--q-start",
            "0",
            "0.3",
            "--q-target",
            "0",
            "0.302",
            "--confirm",
            AGENT_FLOW_REAL_RUNTIME_CONFIRMATION,
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.agent_flow_runtime_smoke_real.v1"
    assert payload["hardware_motion"] is False
    assert payload["movement_command_sent"] is False
    assert payload["reason"] == (
        "real Agent runtime smoke requires --runtime-session-artifact from "
        "armctrl runtime start --serve"
    )
    assert payload["runtime"]["single_owner_runtime_session"] is True
    assert payload["runtime"]["runtime_session_artifact"] is None
    assert payload["next_gate"] == (
        "start live arm runtime, recover/hold SAFE_CENTER, then attach Agent owner lease"
    )
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)
    assert json.loads(output_artifact.read_text(encoding="utf-8")) == payload


def test_cli_agent_flow_runtime_smoke_real_returns_nonzero_for_faulted_result(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    contract_path = tmp_path / "agent_flow_plan.json"
    readiness_artifact = tmp_path / "readiness.json"
    output_artifact = tmp_path / "agent_real_smoke_faulted.json"
    contract_path.write_text(
        json.dumps({"schema": "armctrl.agent_flow_plan.v1"}),
        encoding="utf-8",
    )
    readiness_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_agent_smoke_readiness.v1",
                "agent_sysid_smoke_allowed": True,
            }
        ),
        encoding="utf-8",
    )

    class StubFaultedRealRuntimeSmoker:
        def run(self, request: AgentFlowRealRuntimeSmokeRequest) -> dict[str, object]:
            return {
                "schema": "armctrl.agent_flow_runtime_smoke_real.v1",
                "run_status": "faulted",
                "hardware_motion": True,
                "movement_command_sent": True,
                "runtime": {"backend": "arx5_sdk"},
                "motion_runtime": {
                    "status": "faulted",
                    "landing_mode": "damping",
                    "fault_flags": ["over_current"],
                },
                "acceptance": {
                    "schema": "armctrl.real_motion_acceptance.v1",
                    "stage": "agent_smoke",
                    "status": "review_required",
                },
                "fault_landing_mode": "damping",
            }

    monkeypatch.setattr(cli, "AgentFlowRealRuntimeSmoker", StubFaultedRealRuntimeSmoker)

    exit_code = cli.main(
        [
            "agent-flow",
            "runtime-smoke-real",
            "--contract",
            str(contract_path),
            "--readiness-artifact",
            str(readiness_artifact),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--q-start",
            "0",
            "0.3",
            "--q-target",
            "0",
            "0.302",
            "--confirm",
            AGENT_FLOW_REAL_RUNTIME_CONFIRMATION,
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.agent_flow_runtime_smoke_real.v1"
    assert payload["hardware_motion"] is False
    assert payload["movement_command_sent"] is False
    assert payload["reason"] == (
        "real Agent runtime smoke requires --runtime-session-artifact from "
        "armctrl runtime start --serve"
    )
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_agent_flow_runtime_smoke_real_returns_nonzero_for_aborted_result(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    contract_path = tmp_path / "agent_flow_plan.json"
    readiness_artifact = tmp_path / "readiness.json"
    output_artifact = tmp_path / "agent_real_smoke_aborted.json"
    contract_path.write_text(
        json.dumps({"schema": "armctrl.agent_flow_plan.v1"}),
        encoding="utf-8",
    )
    readiness_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_agent_smoke_readiness.v1",
                "agent_sysid_smoke_allowed": True,
            }
        ),
        encoding="utf-8",
    )

    class StubAbortedRealRuntimeSmoker:
        def run(self, request: AgentFlowRealRuntimeSmokeRequest) -> dict[str, object]:
            return {
                "schema": "armctrl.agent_flow_runtime_smoke_real.v1",
                "run_status": "aborted",
                "hardware_motion": True,
                "movement_command_sent": True,
                "runtime": {"backend": "arx5_sdk"},
                "motion_runtime": {
                    "status": "aborted",
                    "landing_mode": "damping",
                    "error": {
                        "type": "RuntimeError",
                        "message": "simulated SDK send failure",
                    },
                },
                "frequency_contract": {
                    "schema": "armctrl.agent_frequency_contract.v1",
                    "agent_intent_hz": 10.0,
                    "backend_send_hz": 50.0,
                    "actual_send_hz": None,
                    "missed_intent_exercised_in_this_run": False,
                },
                "acceptance": {
                    "schema": "armctrl.real_motion_acceptance.v1",
                    "stage": "agent_smoke",
                    "status": "review_required",
                },
                "fault_landing_mode": "damping",
            }

    monkeypatch.setattr(cli, "AgentFlowRealRuntimeSmoker", StubAbortedRealRuntimeSmoker)

    exit_code = cli.main(
        [
            "agent-flow",
            "runtime-smoke-real",
            "--contract",
            str(contract_path),
            "--readiness-artifact",
            str(readiness_artifact),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--q-start",
            "0",
            "0.3",
            "--q-target",
            "0",
            "0.302",
            "--confirm",
            AGENT_FLOW_REAL_RUNTIME_CONFIRMATION,
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.agent_flow_runtime_smoke_real.v1"
    assert payload["hardware_motion"] is False
    assert payload["movement_command_sent"] is False
    assert payload["reason"] == (
        "real Agent runtime smoke requires --runtime-session-artifact from "
        "armctrl runtime start --serve"
    )
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_agent_flow_runtime_smoke_real_returns_nonzero_for_failed_acceptance(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    contract_path = tmp_path / "agent_flow_plan.json"
    readiness_artifact = tmp_path / "readiness.json"
    output_artifact = tmp_path / "agent_real_smoke_incomplete.json"
    contract_path.write_text(
        json.dumps({"schema": "armctrl.agent_flow_plan.v1"}),
        encoding="utf-8",
    )
    readiness_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_agent_smoke_readiness.v1",
                "agent_sysid_smoke_allowed": True,
            }
        ),
        encoding="utf-8",
    )

    class StubIncompleteRealRuntimeSmoker:
        def run(self, request: AgentFlowRealRuntimeSmokeRequest) -> dict[str, object]:
            return {
                "schema": "armctrl.agent_flow_runtime_smoke_real.v1",
                "run_status": "completed",
                "hardware_motion": True,
                "movement_command_sent": True,
                "runtime": {"backend": "arx5_sdk"},
                "motion_runtime": {
                    "status": "completed",
                    "landing_mode": "hold",
                    "actual_send_hz": None,
                },
                "acceptance": {
                    "schema": "armctrl.real_motion_acceptance.v1",
                    "stage": "agent_smoke",
                    "status": "incomplete",
                    "next_gate": "inspect_acceptance_checks_before_next_hardware_gate",
                },
                "fault_landing_mode": "hold",
            }

    monkeypatch.setattr(
        cli,
        "AgentFlowRealRuntimeSmoker",
        StubIncompleteRealRuntimeSmoker,
    )

    exit_code = cli.main(
        [
            "agent-flow",
            "runtime-smoke-real",
            "--contract",
            str(contract_path),
            "--readiness-artifact",
            str(readiness_artifact),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--q-start",
            "0",
            "0.3",
            "--q-target",
            "0",
            "0.302",
            "--confirm",
            AGENT_FLOW_REAL_RUNTIME_CONFIRMATION,
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.agent_flow_runtime_smoke_real.v1"
    assert payload["hardware_motion"] is False
    assert payload["movement_command_sent"] is False
    assert payload["reason"] == (
        "real Agent runtime smoke requires --runtime-session-artifact from "
        "armctrl runtime start --serve"
    )
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_agent_flow_runtime_smoke_real_rejects_missing_confirmation(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    contract_path = tmp_path / "agent_flow_plan.json"
    readiness_artifact = tmp_path / "readiness.json"
    output_artifact = tmp_path / "agent_real_smoke_rejected.json"
    contract_path.write_text(
        json.dumps({"schema": "armctrl.agent_flow_plan.v1"}),
        encoding="utf-8",
    )
    readiness_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_agent_smoke_readiness.v1",
                "agent_sysid_smoke_allowed": True,
            }
        ),
        encoding="utf-8",
    )

    class StubRealRuntimeSmoker:
        def run(self, request: AgentFlowRealRuntimeSmokeRequest) -> dict[str, object]:
            raise PermissionError("agent real runtime smoke requires explicit operator confirmation")

    monkeypatch.setattr(cli, "AgentFlowRealRuntimeSmoker", StubRealRuntimeSmoker)

    exit_code = cli.main(
        [
            "agent-flow",
            "runtime-smoke-real",
            "--contract",
            str(contract_path),
            "--readiness-artifact",
            str(readiness_artifact),
            "--interface",
            "can0",
            "--q-start",
            "0",
            "0.3",
            "--q-target",
            "0",
            "0.302",
            "--confirm",
            "not the confirmation",
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.agent_flow_runtime_smoke_real.v1"
    assert payload["hardware_motion"] is False
    assert payload["movement_command_sent"] is False
    assert payload["fault_landing_mode"] == "damping"
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)
    assert json.loads(output_artifact.read_text(encoding="utf-8")) == payload


def test_cli_agent_flow_runtime_smoke_real_rejects_failed_readiness(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    contract_path = tmp_path / "agent_flow_plan.json"
    readiness_artifact = tmp_path / "readiness.json"
    output_artifact = tmp_path / "agent_real_smoke_readiness_rejected.json"
    contract_path.write_text(
        json.dumps({"schema": "armctrl.agent_flow_plan.v1"}),
        encoding="utf-8",
    )
    readiness_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_agent_smoke_readiness.v1",
                "agent_sysid_smoke_allowed": False,
            }
        ),
        encoding="utf-8",
    )

    class StubRealRuntimeSmoker:
        def run(self, request: AgentFlowRealRuntimeSmokeRequest) -> dict[str, object]:
            raise RuntimeError("agent/sysid smoke readiness is not passed")

    monkeypatch.setattr(cli, "AgentFlowRealRuntimeSmoker", StubRealRuntimeSmoker)

    exit_code = cli.main(
        [
            "agent-flow",
            "runtime-smoke-real",
            "--contract",
            str(contract_path),
            "--readiness-artifact",
            str(readiness_artifact),
            "--interface",
            "can0",
            "--q-start",
            "0",
            "0.3",
            "--q-target",
            "0",
            "0.302",
            "--confirm",
            AGENT_FLOW_REAL_RUNTIME_CONFIRMATION,
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.agent_flow_runtime_smoke_real.v1"
    assert payload["hardware_motion"] is False
    assert payload["movement_command_sent"] is False
    assert payload["next_gate"] == (
        "start live arm runtime, recover/hold SAFE_CENTER, then attach Agent owner lease"
    )
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)
    assert json.loads(output_artifact.read_text(encoding="utf-8")) == payload
