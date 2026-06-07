import json
import subprocess
import sys
import time
from pathlib import Path


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

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.arm_runtime_session.v1"
    assert payload["backend"] == "fake"
    assert payload["mode"] == "hold_safe"
    assert payload["owner"] is None
    assert payload["q_meas"] == [0.0, 0.3, 0.3]
    assert payload["q_hold"] == [0.0, 0.3, 0.3]
    assert payload["safe_center"] == [0.0, 0.3, 0.3]
    assert payload["send_hz"] == 50.0
    assert payload["hold_hz"] == 50.0
    assert payload["heartbeat"]["fresh"] is True
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is True
    assert payload["runtime_session_id"]
    assert payload["artifacts"]["runtime_session"] == str(session_artifact)
    assert json.loads(session_artifact.read_text(encoding="utf-8")) == payload


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
    assert payload["readiness"]["failed_checks"] == ["heartbeat_fresh"]


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
                "fault_flags": [],
                "heartbeat": {
                    "fresh": True,
                    "age_s": 0.01,
                    "max_age_s": 1.0,
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
