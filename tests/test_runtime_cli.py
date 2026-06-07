import json
import subprocess
import sys
import time
from pathlib import Path

import pytest

from armctrl.motion_runtime import (
    ArmRuntime,
    FakeMotionBackend,
    JointStateSnapshot,
    MotionMode,
)
from armctrl.cli import _runtime_stop_request_path, _serve_runtime_session_until_stopped
from armctrl.runtime_session import heartbeat_runtime_session_payload
from armctrl.runtime_session import ARX5_RUNTIME_START_CONFIRMATION
from armctrl.runtime_session import record_runtime_hold_tick
from armctrl.runtime_session import refresh_runtime_status_payload
from armctrl.runtime_session import start_fake_runtime_session, stop_runtime_session_from_artifact
from armctrl.runtime_session import start_arx5_runtime_session
from armctrl.runtime_ipc import (
    execute_pending_runtime_commands,
    submit_intent_command,
    submit_trajectory_command,
)


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
        assert served == stopped
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
        max_heartbeat_age_s=1.0,
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
            (0.10, 0.3, 0.3),
            (0.20, 0.3, 0.3),
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
    assert result["motion"]["resampling_policy"] == "linear_time_resample"
    assert result["motion"]["interpolation_policy"] == "linear_joint_position"
    assert result["motion"]["sample_count"] == 11
    assert result["motion"]["samples"][0]["q_cmd"] == [0.0, 0.3, 0.3]
    assert result["motion"]["samples"][5]["q_cmd"] == [0.1, 0.3, 0.3]
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


def test_runtime_queue_owner_heartbeat_timeout_lands_damping(
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
    assert result["status"] == "faulted"
    assert result["landing_mode"] == "damping"
    assert result["watchdog"]["reason"] == "owner_heartbeat_timeout"
    assert result_artifact["status"] == "faulted"
    assert updated_session["status"] == "faulted"
    assert updated_session["mode"] == "damping"
    assert updated_session["owner"] is None
    assert updated_session["owner_lease"] is None
    assert backend.damping_count >= 1


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
    assert result_artifact["status"] == "completed"
    assert updated_session["mode"] == "hold_safe"
    assert updated_session["owner"] is None
    assert updated_session["owner_deadman"] is None


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
    assert result["motion"]["interpolation_policy"] == "linear_intent_frame"
    assert result["motion"]["resampling_policy"] == "intent_frame_to_runtime_send_hz"
    assert result["motion"]["missed_intent_policy"] == "hold_then_damping"
    assert result["motion"]["missed_intent_timeout_s"] == pytest.approx(0.3)
    assert result["motion"]["fault_timeout_s"] == pytest.approx(1.0)
    assert result["motion"]["sample_count"] == 6
    assert result["motion"]["samples"][0]["q_cmd"] == [0.0, 0.3, 0.3]
    assert result["motion"]["samples"][-1]["q_cmd"] == [0.01, 0.3, 0.3]
    assert result_artifact["motion"]["missed_intent_policy"] == "hold_then_damping"


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
            "--output",
            str(tmp_path / "intent_submit.json"),
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
    assert command["kind"] == "intent"
    assert command["q_target"] == [0.005, 0.3, 0.3]


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


def _wait_for_session_artifact(path: Path) -> dict[str, object]:
    deadline = time.time() + 3.0
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
            "--output",
            str(path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
