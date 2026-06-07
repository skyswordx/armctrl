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
        refreshed = json.loads(session_artifact.read_text(encoding="utf-8"))

        assert refreshed["mode"] == "hold_safe"
        assert refreshed["owner"] is None
        assert refreshed["heartbeat"]["wall_time_s"] > first_wall_time_s
        assert refreshed["readiness"]["agent_sysid_smoke_allowed"] is True

        stop_completed = subprocess.run(
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

        assert stop_completed.returncode == 3
        stdout, stderr = server.communicate(timeout=3.0)
        assert server.returncode == 0
        served = json.loads(stdout)
        stopped = json.loads(session_artifact.read_text(encoding="utf-8"))
        assert stderr == ""
        assert served["status"] == "stopped"
        assert served["mode"] == "damping"
        assert served == stopped
    finally:
        if server.poll() is None:
            server.terminate()
            server.communicate(timeout=3.0)


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


def test_cli_runtime_recover_fake_returns_session_to_hold_safe(
    tmp_path: Path,
) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    _start_fake_hold_session(session_artifact)
    session = json.loads(session_artifact.read_text(encoding="utf-8"))
    session["mode"] = "passive_safe"
    session["q_meas"] = [0.8, 0.0, 0.0]
    session["q_hold"] = [0.8, 0.0, 0.0]
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
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.arm_runtime_session.v1"
    assert payload["mode"] == "hold_safe"
    assert payload["owner"] is None
    assert payload["q_meas"] == [0.0, 0.3, 0.3]
    assert payload["q_hold"] == [0.0, 0.3, 0.3]
    assert payload["safe_center"] == [0.0, 0.3, 0.3]
    assert payload["recovery"]["status"] == "completed"
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is True
    assert json.loads(session_artifact.read_text(encoding="utf-8")) == payload


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
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is True
    assert payload["owner_lease"] is None
    assert json.loads(session_artifact.read_text(encoding="utf-8")) == payload


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


def _wait_for_session_artifact(path: Path) -> dict[str, object]:
    deadline = time.time() + 3.0
    while time.time() < deadline:
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8"))
        time.sleep(0.02)
    raise AssertionError(f"runtime session artifact was not written: {path}")


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
            "--output",
            str(path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
