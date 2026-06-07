import csv
import json
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

from armctrl.motion_runtime import MotionAuditSample, MotionExecutionResult, MotionMode
from armctrl.sysid import SysIdPlanRequest, trajectory_rows
from armctrl.sysid_run import (
    Arx5InterfaceCollectionBackend,
    SdkSysIdRunner,
    SDK_CONFIRMATION,
)
from armctrl.runtime_session import start_fake_runtime_session


def _passing_readiness_artifact() -> dict[str, object]:
    return {
        "schema": "armctrl.sysid_agent_smoke_readiness.v1",
        "read_only": True,
        "movement_allowed": False,
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


def _write_fake_runtime_session(path: Path) -> None:
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        safe_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=1.0,
    )
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


class RecordingBackend:
    def __init__(self) -> None:
        self.events: list[str] = []
        self.last_motion_result = MotionExecutionResult(
            status="completed",
            producer="sysid",
            mode="trajectory_replay",
            trajectory_sample_hz=20.0,
            actual_send_hz=19.8,
            send_jitter_ms_p95=1.2,
            send_jitter_ms_p99=2.3,
            controller_dt_s=0.01,
            samples=(
                MotionAuditSample(
                    sent_monotonic_s=1.0,
                    q_cmd=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
                    q_meas=(0.0, 0.299, 0.301, 0.0, 0.0, 0.0),
                    dq_meas=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    tau_meas=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    fault_flags=("simulated_warning",),
                    producer="sysid",
                    mode="trajectory_replay",
                ),
            ),
            landing_mode="released",
        )
        self.last_ramp_result = MotionExecutionResult(
            status="completed",
            producer="sysid_ramp",
            mode="trajectory_replay",
            trajectory_sample_hz=20.0,
            actual_send_hz=20.0,
            send_jitter_ms_p95=0.0,
            send_jitter_ms_p99=0.0,
            controller_dt_s=0.01,
            samples=(),
            landing_mode="released",
        )

    def enter_hold_or_damping(self) -> None:
        self.events.append("enter_hold_or_damping")

    def read_samples(self, request: SysIdPlanRequest) -> list[dict[str, str]]:
        self.events.append("read_samples")
        return [
            {
                "time_s": "0.000000",
                **{f"q_cmd_{index + 1}": "0.000000" for index in range(request.dof)},
                **{f"q_{index + 1}": "0.000000" for index in range(request.dof)},
                **{f"dq_{index + 1}": "0.000000" for index in range(request.dof)},
                **{f"tau_meas_{index + 1}": "0.000000" for index in range(request.dof)},
            }
        ]

    def enter_damping(self) -> None:
        self.events.append("enter_damping")


class PassingRecordingBackend(RecordingBackend):
    def __init__(self) -> None:
        super().__init__()
        sample = replace(self.last_motion_result.samples[0], fault_flags=())
        self.last_motion_result = replace(
            self.last_motion_result,
            samples=(sample,),
        )


class FaultedRecordingBackend(RecordingBackend):
    def __init__(self) -> None:
        super().__init__()
        self.last_motion_result = MotionExecutionResult(
            status="faulted",
            producer="sysid",
            mode="trajectory_replay",
            trajectory_sample_hz=20.0,
            actual_send_hz=20.0,
            send_jitter_ms_p95=0.0,
            send_jitter_ms_p99=0.0,
            controller_dt_s=0.01,
            samples=(
                MotionAuditSample(
                    sent_monotonic_s=1.0,
                    q_cmd=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
                    q_meas=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
                    dq_meas=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    tau_meas=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    fault_flags=(),
                    producer="sysid",
                    mode="trajectory_replay",
                ),
                MotionAuditSample(
                    sent_monotonic_s=1.05,
                    q_cmd=(0.0, 0.301, 0.3, 0.0, 0.0, 0.0),
                    q_meas=(0.0, 0.301, 0.3, 0.0, 0.0, 0.0),
                    dq_meas=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    tau_meas=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    fault_flags=("over_current",),
                    producer="sysid",
                    mode="trajectory_replay",
                ),
            ),
            landing_mode="damping",
        )

    def read_samples(self, request: SysIdPlanRequest) -> list[dict[str, str]]:
        self.events.append("read_samples")
        rows = []
        for sample_index, sample in enumerate(self.last_motion_result.samples):
            rows.append(
                {
                    "time_s": f"{sample_index / request.sample_hz:.6f}",
                    **{
                        f"q_cmd_{index + 1}": f"{sample.q_cmd[index]:.6f}"
                        for index in range(request.dof)
                    },
                    **{
                        f"q_{index + 1}": f"{sample.q_meas[index]:.6f}"
                        for index in range(request.dof)
                    },
                    **{
                        f"dq_{index + 1}": "0.000000"
                        for index in range(request.dof)
                    },
                    **{
                        f"tau_meas_{index + 1}": "0.000000"
                        for index in range(request.dof)
                    },
                }
            )
        return rows


class ManualClock:
    def __init__(self) -> None:
        self.now_s = 0.0

    def monotonic(self) -> float:
        return self.now_s

    def sleep(self, seconds: float) -> None:
        self.now_s += max(0.0, seconds)


def test_cli_sysid_run_fake_writes_raw_samples_and_manifest(tmp_path: Path) -> None:
    output_dir = tmp_path / "ident-run"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "fake",
            "--dof",
            "6",
            "--sample-hz",
            "20",
            "--duration",
            "2",
            "--amplitude",
            "0.1",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    raw_samples_path = output_dir / "raw_samples.csv"
    manifest_path = output_dir / "manifest.json"

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_run.v1"
    assert payload["adapter"] == "fake"
    assert payload["artifacts"]["raw_samples"] == str(raw_samples_path)
    assert raw_samples_path.exists()
    assert manifest_path.exists()

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema"] == "armctrl.sysid_run_manifest.v1"
    assert manifest["adapter"] == "fake"
    assert manifest["sample_count"] == 41
    assert manifest["request"]["urdf_path"] == "configs/models/X5_camera.urdf"
    assert manifest["request"]["dof"] == 6
    assert manifest["handoff"]["dataset_contract"] == "lerobot-compatible"

    with raw_samples_path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    assert rows[0]["time_s"] == "0.000000"
    assert "tau_meas_6" in rows[0]
    assert len(rows) == 41


def test_cli_sysid_run_fake_acquires_runtime_owner_lease(tmp_path: Path) -> None:
    output_dir = tmp_path / "ident-run"
    runtime_session_artifact = tmp_path / "runtime-session.json"
    safe_center = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    runtime_session_artifact.write_text(
        json.dumps(
            start_fake_runtime_session(
                q_current=safe_center,
                safe_center=safe_center,
                send_hz=50.0,
                hold_hz=50.0,
                max_joint_step_rad=0.01,
                max_heartbeat_age_s=5.0,
            ),
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
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "fake",
            "--dof",
            "6",
            "--sample-hz",
            "20",
            "--duration",
            "2",
            "--amplitude",
            "0.1",
            "--q-center",
            *[str(value) for value in safe_center],
            "--runtime-session-artifact",
            str(runtime_session_artifact),
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    released_session = json.loads(runtime_session_artifact.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["runtime"]["single_owner_runtime_session"] is True
    assert payload["runtime"]["owner"] == "sysid"
    assert payload["runtime"]["mode"] == "trajectory_replay"
    assert payload["runtime"]["release"]["mode"] == "hold_safe"
    assert payload["runtime"]["release"]["owner"] is None

    assert manifest["runtime"]["owner_lease"]["owner"] == "sysid"
    assert manifest["runtime"]["owner_lease"]["mode"] == "trajectory_replay"
    assert manifest["runtime"]["release"]["readiness"]["agent_sysid_smoke_allowed"] is True

    assert released_session["mode"] == "hold_safe"
    assert released_session["owner"] is None
    assert released_session["owner_lease"] is None
    assert released_session["readiness"]["agent_sysid_smoke_allowed"] is True


def test_cli_sysid_run_fake_rejects_busy_runtime_owner(tmp_path: Path) -> None:
    output_dir = tmp_path / "ident-run"
    runtime_session_artifact = tmp_path / "runtime-session.json"
    safe_center = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    session = start_fake_runtime_session(
        q_current=safe_center,
        safe_center=safe_center,
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=5.0,
    )
    session["mode"] = "agent_servo"
    session["owner"] = "agent"
    session["owner_lease"] = {
        "schema": "armctrl.arm_runtime_owner_lease.v1",
        "runtime_session_id": session["runtime_session_id"],
        "owner": "agent",
        "mode": "agent_servo",
        "heartbeat_timeout_s": 1.0,
    }
    runtime_session_artifact.write_text(
        json.dumps(session, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "fake",
            "--dof",
            "6",
            "--sample-hz",
            "20",
            "--duration",
            "2",
            "--amplitude",
            "0.1",
            "--q-center",
            *[str(value) for value in safe_center],
            "--runtime-session-artifact",
            str(runtime_session_artifact),
            "--output",
            str(output_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["reason"] == "runtime is owned by agent"
    assert payload["runtime"]["owner"] == "agent"
    assert not (output_dir / "raw_samples.csv").exists()
    assert json.loads(runtime_session_artifact.read_text(encoding="utf-8"))["owner"] == "agent"


def test_cli_sysid_run_sdk_is_rejected_until_runner_exists(tmp_path: Path) -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "sdk",
            "--output",
            str(tmp_path / "ident-run"),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["reason"] == "sdk sysid runner requires explicit operator confirmation"
    assert payload["requires_confirm"] == "I UNDERSTAND THIS WILL MOVE THE ARM"
    assert payload["movement_allowed"] is False
    assert payload["fault_landing_mode"] == "damping"
    assert payload["recording_starts_after_safe_state"] is True
    assert payload["next_gate"] == "run sysid sdk-handshake-plan before enabling sdk runner"
    manifest_path = tmp_path / "ident-run" / "manifest.json"
    assert payload["artifacts"]["manifest"] == str(manifest_path)
    assert json.loads(manifest_path.read_text(encoding="utf-8")) == payload


def test_cli_sysid_run_sdk_requires_smoke_readiness_after_confirm(
    tmp_path: Path,
) -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "sdk",
            "--output",
            str(tmp_path / "ident-run"),
            "--confirm",
            SDK_CONFIRMATION,
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["reason"] == (
        "sdk sysid runner requires sdk-agent-sysid-smoke-readiness artifact"
    )
    assert payload["confirm_received"] is True
    assert payload["movement_allowed"] is False
    assert payload["readiness_artifact_path"] is None
    assert payload["next_gate"] == (
        "run sysid sdk-agent-sysid-smoke-readiness after real tiny motion before sdk sysid run"
    )
    manifest_path = tmp_path / "ident-run" / "manifest.json"
    assert payload["artifacts"]["manifest"] == str(manifest_path)
    assert json.loads(manifest_path.read_text(encoding="utf-8")) == payload


def test_cli_sysid_run_sdk_requires_runtime_session_after_readiness(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    output_dir = tmp_path / "ident-run"
    readiness_artifact = tmp_path / "readiness.json"
    readiness_artifact.write_text(
        json.dumps(_passing_readiness_artifact()),
        encoding="utf-8",
    )

    class ForbiddenArx5Backend:
        def __init__(self, *args, **kwargs) -> None:
            raise AssertionError("sdk sysid must not open SDK without runtime")

    monkeypatch.setattr(cli, "Arx5InterfaceCollectionBackend", ForbiddenArx5Backend)

    exit_code = cli.main(
        [
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "sdk",
            "--dof",
            "6",
            "--sample-hz",
            "20",
            "--duration",
            "2",
            "--amplitude",
            "0.1",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--readiness-artifact",
            str(readiness_artifact),
            "--confirm",
            SDK_CONFIRMATION,
            "--output",
            str(output_dir),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.sysid_run.v1"
    assert payload["adapter"] == "sdk"
    assert payload["reason"] == (
        "sdk sysid runner requires --runtime-session-artifact from "
        "armctrl runtime start --serve"
    )
    assert payload["runtime"]["single_owner_runtime_session"] is True
    assert payload["runtime"]["runtime_session_artifact"] is None
    assert payload["movement_allowed"] is False
    assert payload["fault_landing_mode"] == "damping"
    assert payload["next_gate"] == (
        "start live arm runtime, recover/hold SAFE_CENTER, then attach SysID owner lease"
    )
    manifest_path = output_dir / "manifest.json"
    assert payload["artifacts"]["manifest"] == str(manifest_path)
    assert json.loads(manifest_path.read_text(encoding="utf-8")) == payload


def test_cli_sysid_run_sdk_rejects_failed_smoke_readiness(tmp_path: Path) -> None:
    readiness_artifact = tmp_path / "readiness.json"
    readiness_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_agent_smoke_readiness.v1",
                "agent_sysid_smoke_allowed": False,
                "prerequisites": {
                    "doctor": "pass",
                    "hold_damping": "pass",
                    "tiny_motion": "fail",
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
            "run",
            "gravity_sweep",
            "--adapter",
            "sdk",
            "--output",
            str(tmp_path / "ident-run"),
            "--confirm",
            SDK_CONFIRMATION,
            "--readiness-artifact",
            str(readiness_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["reason"] == "sdk-agent-sysid-smoke-readiness is not passed"
    assert payload["confirm_received"] is True
    assert payload["movement_allowed"] is False
    assert payload["readiness_artifact_path"] == str(readiness_artifact)
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is False
    assert payload["next_gate"] == "complete real tiny motion before sdk sysid run"
    manifest_path = tmp_path / "ident-run" / "manifest.json"
    assert payload["artifacts"]["manifest"] == str(manifest_path)
    assert json.loads(manifest_path.read_text(encoding="utf-8")) == payload


def test_cli_sysid_run_sdk_accepts_confirm_and_readiness_but_requires_runtime(
    tmp_path: Path,
) -> None:
    readiness_artifact = tmp_path / "readiness.json"
    readiness_artifact.write_text(
        json.dumps(_passing_readiness_artifact()),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "sdk",
            "--output",
            str(tmp_path / "ident-run"),
            "--confirm",
            SDK_CONFIRMATION,
            "--readiness-artifact",
            str(readiness_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["reason"] == (
        "sdk sysid runner requires --runtime-session-artifact from "
        "armctrl runtime start --serve"
    )
    assert payload["movement_allowed"] is False
    assert payload["runtime"]["single_owner_runtime_session"] is True
    assert payload["runtime"]["runtime_session_artifact"] is None
    assert payload["next_gate"] == (
        "start live arm runtime, recover/hold SAFE_CENTER, then attach SysID owner lease"
    )
    manifest_path = tmp_path / "ident-run" / "manifest.json"
    assert payload["artifacts"]["manifest"] == str(manifest_path)
    assert json.loads(manifest_path.read_text(encoding="utf-8")) == payload


def test_sdk_sysid_runner_starts_recording_after_safe_state_and_lands_damping(
    tmp_path: Path,
) -> None:
    backend = RecordingBackend()
    request = SysIdPlanRequest(
        profile_name="gravity_sweep",
        dof=6,
        sample_hz=20,
        duration_s=1,
        amplitude_rad=0.02,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path / "ident-sdk",
    )

    result = SdkSysIdRunner(backend=backend).run(
        request,
        confirm=SDK_CONFIRMATION,
    )

    manifest = json.loads((request.output_dir / "manifest.json").read_text(encoding="utf-8"))

    assert result.adapter == "sdk"
    assert backend.events == [
        "enter_hold_or_damping",
        "read_samples",
        "enter_damping",
    ]
    assert manifest["adapter"] == "sdk"
    assert manifest["safety"]["recording_starts_after_safe_state"] is True
    assert manifest["safety"]["fault_landing_mode"] == "damping"
    assert manifest["safety"]["movement_allowed"] is True
    assert manifest["motion_runtime"] == {
        "schema": "armctrl.motion_runtime_result.v1",
        "status": "completed",
        "producer": "sysid",
        "mode": "trajectory_replay",
        "trajectory_sample_hz": 20.0,
        "actual_send_hz": 19.8,
        "send_jitter_ms_p95": 1.2,
        "send_jitter_ms_p99": 2.3,
        "controller_dt_s": 0.01,
        "sample_count": 1,
        "fault_flags": ["simulated_warning"],
        "tracking": {
            "q_cmd_delta_rad": None,
            "q_meas_delta_rad": None,
            "q_cmd_delta_max_abs_rad": None,
            "q_meas_delta_max_abs_rad": None,
            "max_abs_sample_tracking_error_rad": 0.0010000000000000009,
            "final_tracking_error_rad": [
                0.0,
                -0.0010000000000000009,
                0.0010000000000000009,
                0.0,
                0.0,
                0.0,
            ],
            "final_tracking_error_max_abs_rad": 0.0010000000000000009,
        },
        "samples": [
            {
                "sent_monotonic_s": 1.0,
                "q_cmd": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
                "q_meas": [0.0, 0.299, 0.301, 0.0, 0.0, 0.0],
                "dq_meas": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "tau_meas": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "fault_flags": ["simulated_warning"],
                "producer": "sysid",
                "mode": "trajectory_replay",
            }
        ],
        "landing_mode": "released",
    }
    assert manifest["ramp_runtime"] == {
        "schema": "armctrl.motion_runtime_result.v1",
        "status": "completed",
        "producer": "sysid_ramp",
        "mode": "trajectory_replay",
        "trajectory_sample_hz": 20.0,
        "actual_send_hz": 20.0,
        "send_jitter_ms_p95": 0.0,
        "send_jitter_ms_p99": 0.0,
        "controller_dt_s": 0.01,
        "sample_count": 0,
        "fault_flags": [],
        "tracking": {
            "q_cmd_delta_rad": None,
            "q_meas_delta_rad": None,
            "q_cmd_delta_max_abs_rad": None,
            "q_meas_delta_max_abs_rad": None,
            "max_abs_sample_tracking_error_rad": None,
            "final_tracking_error_rad": None,
            "final_tracking_error_max_abs_rad": None,
        },
        "samples": [],
        "landing_mode": "released",
    }
    assert manifest["acceptance"]["schema"] == "armctrl.real_motion_acceptance.v1"
    assert manifest["acceptance"]["stage"] == "sysid_smoke"
    assert manifest["acceptance"]["status"] == "review_required"
    assert manifest["acceptance"]["next_gate"] == (
        "inspect_acceptance_checks_before_next_hardware_gate"
    )
    assert manifest["acceptance"]["checks"]["controller_dt_measured"]["status"] == "pass"
    assert manifest["acceptance"]["checks"]["actual_send_hz_measured"]["status"] == "pass"
    assert manifest["acceptance"]["checks"]["readiness_gate"]["status"] == "not_available"
    assert manifest["acceptance"]["checks"]["no_fault_flags"] == {
        "status": "fail",
        "fault_flags": ["simulated_warning"],
    }
    assert (
        manifest["acceptance"]["checks"]["q_meas_responded_to_commanded_motion"]["status"]
        == "not_applicable"
    )


def test_sdk_sysid_runner_writes_faulted_manifest_with_partial_samples(
    tmp_path: Path,
) -> None:
    backend = FaultedRecordingBackend()
    request = SysIdPlanRequest(
        profile_name="gravity_sweep",
        dof=6,
        sample_hz=20,
        duration_s=1,
        amplitude_rad=0.02,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path / "ident-sdk-faulted",
    )

    result = SdkSysIdRunner(backend=backend).run(
        request,
        confirm=SDK_CONFIRMATION,
    )

    manifest = json.loads((request.output_dir / "manifest.json").read_text(encoding="utf-8"))
    with (request.output_dir / "raw_samples.csv").open(
        "r",
        encoding="utf-8-sig",
        newline="",
    ) as handle:
        raw_rows = list(csv.DictReader(handle))

    assert result.sample_count == 2
    assert result.run_status == "faulted"
    assert manifest["run_status"] == "faulted"
    assert manifest["sample_count"] == 2
    assert len(raw_rows) == 2
    assert manifest["motion_runtime"]["status"] == "faulted"
    assert manifest["motion_runtime"]["landing_mode"] == "damping"
    assert manifest["motion_runtime"]["fault_flags"] == ["over_current"]
    assert manifest["motion_runtime"]["sample_count"] == 2
    assert manifest["acceptance"]["status"] == "review_required"
    assert manifest["acceptance"]["checks"]["runtime_completed"]["status"] == "fail"
    assert manifest["acceptance"]["checks"]["no_fault_flags"] == {
        "status": "fail",
        "fault_flags": ["over_current"],
    }
    assert backend.events == [
        "enter_hold_or_damping",
        "read_samples",
        "enter_damping",
    ]


def test_cli_sysid_run_sdk_with_runtime_blocks_until_live_queue_exists(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    readiness_artifact = tmp_path / "readiness.json"
    runtime_session = tmp_path / "runtime-session.json"
    output_dir = tmp_path / "ident-sdk-cli"
    readiness_artifact.write_text(
        json.dumps(_passing_readiness_artifact()),
        encoding="utf-8",
    )
    _write_fake_runtime_session(runtime_session)

    class ForbiddenBackendFactory:
        def __init__(self, **kwargs) -> None:
            raise AssertionError("sdk sysid must not open SDK outside runtime")

    monkeypatch.setattr(cli, "Arx5InterfaceCollectionBackend", ForbiddenBackendFactory)

    exit_code = cli.main(
        [
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "sdk",
            "--dof",
            "6",
            "--sample-hz",
            "20",
            "--duration",
            "1",
            "--amplitude",
            "0.02",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--output",
            str(output_dir),
            "--confirm",
            SDK_CONFIRMATION,
            "--readiness-artifact",
            str(readiness_artifact),
            "--runtime-session-artifact",
            str(runtime_session),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))

    assert exit_code == 0
    assert payload["status"] == "queued"
    assert payload["runtime"]["single_owner_runtime_session"] is True
    assert payload["runtime"]["runtime_session_artifact"] == str(runtime_session)
    assert payload["runtime"]["owner"] == "sysid"
    assert payload["runtime"]["mode"] == "trajectory_replay"
    assert payload["movement_allowed"] is True
    assert payload["movement_command_sent"] is False
    assert payload["runtime_command"]["status"] == "queued"
    assert Path(payload["runtime_command"]["artifacts"]["command"]).exists()
    assert payload["next_gate"] == "wait for live runtime command result artifact"
    assert manifest == payload


def test_cli_sysid_run_sdk_with_runtime_does_not_report_fake_acceptance(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    readiness_artifact = tmp_path / "readiness.json"
    runtime_session = tmp_path / "runtime-session.json"
    output_dir = tmp_path / "ident-sdk-cli-review-required"
    readiness_artifact.write_text(
        json.dumps(_passing_readiness_artifact()),
        encoding="utf-8",
    )
    _write_fake_runtime_session(runtime_session)

    class ForbiddenBackendFactory:
        def __init__(self, **kwargs) -> None:
            raise AssertionError("sdk sysid must not open SDK outside runtime")

    monkeypatch.setattr(cli, "Arx5InterfaceCollectionBackend", ForbiddenBackendFactory)

    exit_code = cli.main(
        [
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "sdk",
            "--dof",
            "6",
            "--sample-hz",
            "20",
            "--duration",
            "1",
            "--amplitude",
            "0.02",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--output",
            str(output_dir),
            "--confirm",
            SDK_CONFIRMATION,
            "--readiness-artifact",
            str(readiness_artifact),
            "--runtime-session-artifact",
            str(runtime_session),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))

    assert exit_code == 0
    assert payload["status"] == "queued"
    assert "acceptance" not in payload
    assert payload["movement_command_sent"] is False
    assert manifest == payload


def test_cli_sysid_run_sdk_with_runtime_does_not_report_fake_faults(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    readiness_artifact = tmp_path / "readiness.json"
    runtime_session = tmp_path / "runtime-session.json"
    output_dir = tmp_path / "ident-sdk-cli-faulted"
    readiness_artifact.write_text(
        json.dumps(_passing_readiness_artifact()),
        encoding="utf-8",
    )
    _write_fake_runtime_session(runtime_session)

    class ForbiddenBackendFactory:
        def __init__(self, **kwargs) -> None:
            raise AssertionError("sdk sysid must not open SDK outside runtime")

    monkeypatch.setattr(cli, "Arx5InterfaceCollectionBackend", ForbiddenBackendFactory)

    exit_code = cli.main(
        [
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "sdk",
            "--dof",
            "6",
            "--sample-hz",
            "20",
            "--duration",
            "1",
            "--amplitude",
            "0.02",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--output",
            str(output_dir),
            "--confirm",
            SDK_CONFIRMATION,
            "--readiness-artifact",
            str(readiness_artifact),
            "--runtime-session-artifact",
            str(runtime_session),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))

    assert exit_code == 0
    assert payload["status"] == "queued"
    assert "run_status" not in payload
    assert "motion_runtime" not in payload
    assert payload["movement_command_sent"] is False
    assert manifest == payload


def test_cli_sysid_postprocess_solve_blocks_faulted_sdk_manifest(
    tmp_path: Path,
    capsys,
) -> None:
    from armctrl import cli

    dataset_dir = tmp_path / "faulted-sdk-dataset"
    dataset_dir.mkdir()
    (dataset_dir / "raw_samples.csv").write_text(
        "time_s,q_cmd_1,q_1,dq_1,tau_meas_1\n0.000000,0.000000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )
    (dataset_dir / "manifest.json").write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_run_manifest.v1",
                "adapter": "sdk",
                "run_status": "faulted",
                "handoff": {"status": "blocked"},
                "acceptance": {
                    "schema": "armctrl.real_motion_acceptance.v1",
                    "stage": "sysid_smoke",
                    "status": "review_required",
                },
            }
        ),
        encoding="utf-8",
    )

    exit_code = cli.main(
        [
            "sysid",
            "postprocess",
            "--dataset",
            str(dataset_dir),
            "--solve",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "blocked"
    assert payload["schema"] == "armctrl.sysid_postprocess.v1"
    assert payload["solver_gate"] == {
        "status": "blocked",
        "reason": "sdk run_status is faulted",
        "next_gate": "review sysid smoke acceptance before solver",
        "checks": {
            "run_status_completed": False,
            "acceptance_passed": False,
        },
    }
    assert "solver" not in payload
    assert (dataset_dir / "processed" / "processed_samples.csv").exists()


def test_cli_sysid_solve_blocks_failed_sdk_acceptance(
    tmp_path: Path,
    capsys,
) -> None:
    from armctrl import cli

    dataset_dir = tmp_path / "failed-acceptance-sdk-dataset"
    dataset_dir.mkdir()
    (dataset_dir / "manifest.json").write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_run_manifest.v1",
                "adapter": "sdk",
                "run_status": "completed",
                "acceptance": {
                    "schema": "armctrl.real_motion_acceptance.v1",
                    "stage": "sysid_smoke",
                    "status": "review_required",
                },
            }
        ),
        encoding="utf-8",
    )

    exit_code = cli.main(
        [
            "sysid",
            "solve",
            "--dataset",
            str(dataset_dir),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "blocked"
    assert payload["schema"] == "armctrl.sysid_solve.v1"
    assert payload["solver_gate"] == {
        "status": "blocked",
        "reason": "sdk acceptance is not pass",
        "next_gate": "review sysid smoke acceptance before solver",
        "checks": {
            "run_status_completed": True,
            "acceptance_passed": False,
        },
    }


class FakeJointState:
    def __init__(self, dof: int) -> None:
        self._pos = [0.0] * dof
        self._vel = [0.0] * dof
        self._torque = [0.0] * dof
        self.gripper_pos = 0.0

    def pos(self) -> list[float]:
        return self._pos

    def vel(self) -> list[float]:
        return self._vel

    def torque(self) -> list[float]:
        return self._torque


class FakeController:
    def __init__(self, robot_config, controller_config, interface: str) -> None:
        self.commands: list[list[float]] = []
        self.damping_count = 0
        self.reset_home_count = 0

    def reset_to_home(self) -> None:
        self.reset_home_count += 1

    def set_joint_cmd(self, cmd: FakeJointState) -> None:
        self.commands.append(list(cmd.pos()))

    def get_joint_state(self) -> FakeJointState:
        state = FakeJointState(6)
        if self.commands:
            state.pos()[:] = self.commands[-1]
        return state

    def set_to_damping(self) -> None:
        self.damping_count += 1


class FakeRobotConfig:
    joint_dof = 6


class FakeControllerConfig:
    controller_dt = 0.0
    gravity_compensation = False
    background_send_recv = False


class FakeConfigFactory:
    def __init__(self, value) -> None:
        self._value = value

    @classmethod
    def get_instance(cls):
        return cls(cls._value)

    def get_config(self, *args):
        return self._value


class FakeRobotConfigFactory(FakeConfigFactory):
    _value = FakeRobotConfig()


class FakeControllerConfigFactory(FakeConfigFactory):
    _value = FakeControllerConfig()


class FakeArx5Module:
    RobotConfigFactory = FakeRobotConfigFactory
    ControllerConfigFactory = FakeControllerConfigFactory
    JointState = FakeJointState
    last_controller: FakeController | None = None

    @staticmethod
    def Arx5JointController(robot_config, controller_config, interface: str) -> FakeController:
        controller = FakeController(robot_config, controller_config, interface)
        FakeArx5Module.last_controller = controller
        return controller


def test_arx5_interface_backend_sends_joint_commands_and_lands_damping() -> None:
    clock = ManualClock()
    request = SysIdPlanRequest(
        profile_name="gravity_sweep",
        dof=6,
        sample_hz=2,
        duration_s=1,
        amplitude_rad=0.05,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=Path("unused"),
    )
    backend = Arx5InterfaceCollectionBackend(
        model="X5",
        interface="can0",
        arx5_module=FakeArx5Module,
        max_joint_step_rad=0.2,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    backend.enter_hold_or_damping()
    rows = backend.read_samples(request)
    backend.enter_damping()

    expected_sample_count = len(trajectory_rows(request))

    assert len(rows) == expected_sample_count
    assert backend.last_motion_result is not None
    assert backend.last_motion_result.producer == "sysid"
    assert backend.last_motion_result.trajectory_sample_hz == 2.0
    assert backend.last_motion_result.actual_send_hz == 2.0
    assert backend.last_motion_result.controller_dt_s == 0.01
    assert len(backend.last_motion_result.samples) == expected_sample_count
    assert backend.last_ramp_result is not None
    assert backend.last_ramp_result.producer == "sysid_ramp"
    assert backend.last_ramp_result.trajectory_sample_hz == 2.0
    assert backend.last_ramp_result.actual_send_hz == 2.0
    assert len(backend.last_ramp_result.samples) == 2
    assert rows[0]["q_cmd_2"] == "0.300000"
    assert rows[0]["q_2"] == "0.300000"
    assert "tau_meas_6" in rows[0]
    assert FakeArx5Module.last_controller is not None
    assert len(FakeArx5Module.last_controller.commands) > 3
    assert FakeArx5Module.last_controller.reset_home_count == 0
    for previous, current in zip(
        FakeArx5Module.last_controller.commands,
        FakeArx5Module.last_controller.commands[1:],
    ):
        assert max(abs(a - b) for a, b in zip(previous, current)) <= 0.2 + 1e-9
    assert FakeArx5Module.last_controller.damping_count == 1


def test_arx5_interface_backend_preserves_sdk_fault_flags() -> None:
    class FaultFlagJointState(FakeJointState):
        def fault_flags(self) -> list[str]:
            return ["over_current", "encoder_fault"]

    class FaultFlagController(FakeController):
        def get_joint_state(self) -> FaultFlagJointState:
            state = FaultFlagJointState(6)
            if self.commands:
                state.pos()[:] = self.commands[-1]
            return state

    class FaultFlagArx5Module(FakeArx5Module):
        @staticmethod
        def Arx5JointController(robot_config, controller_config, interface: str):
            return FaultFlagController(robot_config, controller_config, interface)

    backend = Arx5InterfaceCollectionBackend(
        model="X5",
        interface="can0",
        arx5_module=FaultFlagArx5Module,
    )

    backend.enter_hold_or_damping()
    backend.send_joint_command(
        (0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        producer="sysid",
        mode=MotionMode.TRAJECTORY_REPLAY,
        monotonic_s=0.0,
    )

    snapshot = backend.read_joint_state()

    assert snapshot.fault_flags == ("over_current", "encoder_fault")


def test_arx5_interface_backend_uses_measured_controller_dt_when_provided() -> None:
    backend = Arx5InterfaceCollectionBackend(
        model="X5",
        interface="can0",
        arx5_module=FakeArx5Module,
        controller_dt_s=0.002,
    )

    backend.enter_hold_or_damping()

    assert backend.controller_dt_s == 0.002
