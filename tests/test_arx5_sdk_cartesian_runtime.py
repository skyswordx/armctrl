from __future__ import annotations

import json
import time
from types import SimpleNamespace

import pytest

from armctrl.arx5_sdk_cartesian_runtime import Arx5SdkCartesianRuntimeBackend
from armctrl.motion_runtime import ArmRuntime
from armctrl.runtime_ipc import execute_pending_runtime_commands, submit_eef_command
from armctrl.runtime_session import start_arx5_cartesian_runtime_session, start_fake_runtime_session


class _FakeGain:
    def __init__(self, kp, kd=None, gripper_kp: float = 0.0, gripper_kd: float = 0.0) -> None:
        if isinstance(kp, int):
            self._kp = [0.0] * kp
            self._kd = [0.0] * kp
        else:
            self._kp = list(kp)
            self._kd = list(kd)
        self.gripper_kp = float(gripper_kp)
        self.gripper_kd = float(gripper_kd)

    def kp(self):
        return self._kp

    def kd(self):
        return self._kd

    def __mul__(self, scalar: float):
        return _FakeGain(
            [value * scalar for value in self._kp],
            [value * scalar for value in self._kd],
            self.gripper_kp * scalar,
            self.gripper_kd * scalar,
        )

    def __add__(self, other):
        return _FakeGain(
            [left + right for left, right in zip(self._kp, other._kp)],
            [left + right for left, right in zip(self._kd, other._kd)],
            self.gripper_kp + other.gripper_kp,
            self.gripper_kd + other.gripper_kd,
        )


class _FakeEEFState:
    def __init__(self) -> None:
        self._pose = [0.4, 0.0, 0.2, 0.0, 0.0, 0.0]
        self.gripper_pos = 0.0
        self.gripper_vel = 0.0
        self.gripper_torque = 0.0
        self.timestamp = 0.0

    def pose_6d(self):
        return self._pose


class _FakeJointState:
    gripper_pos = 0.0
    gripper_vel = 0.0
    gripper_torque = 0.0
    timestamp = 0.0

    def pos(self):
        return [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]

    def vel(self):
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def torque(self):
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class _OffsetJointState(_FakeJointState):
    def pos(self):
        return [0.2, 0.3, 0.3, 0.0, 0.0, 0.0]


class _FakeCartesianController:
    def __init__(self, *, joint_state=None) -> None:
        self.config = SimpleNamespace(
            controller_dt=0.002,
            default_preview_time=0.04,
            default_kp=[200.0, 200.0, 200.0, 120.0, 80.0, 60.0],
            default_kd=[5.0, 5.0, 5.0, 1.0, 1.0, 1.0],
            default_gripper_kp=5.0,
            default_gripper_kd=0.2,
        )
        self.gain = _FakeGain(6)
        self.commands: list[_FakeEEFState] = []
        self.gain_calls = 0
        self.damping_calls = 0
        self.joint_state = joint_state or _FakeJointState()

    def get_controller_config(self):
        return self.config

    def get_timestamp(self):
        return 10.0

    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        self.gain_calls += 1
        self.gain = gain

    def set_eef_cmd(self, command):
        self.commands.append(command)

    def get_eef_state(self):
        return _FakeEEFState()

    def get_joint_state(self):
        return self.joint_state

    def set_to_damping(self):
        self.damping_calls += 1


class _FakeSDK:
    EEFState = _FakeEEFState
    Gain = _FakeGain


def test_sdk_cartesian_pose_delta_syncs_current_eef_before_motion_command() -> None:
    controller = _FakeCartesianController()
    backend = Arx5SdkCartesianRuntimeBackend(
        arx5_module=_FakeSDK,
        controller=controller,
        sleep=lambda _duration_s: None,
        monotonic=lambda: 123.0,
        resume_gain_duration_s=0.002,
    )

    result = backend.execute_eef_command(
        {
            "kind": "eef_pose_delta",
            "send_hz": 50.0,
            "eef_command": {
                "frame": "eef_link",
                "delta_position_m": [0.002, 0.0, -0.001],
                "delta_rpy_rad": [0.0, 0.0, 0.01],
                "control_period_s": 0.1,
            },
        },
        owner="agent",
    )

    assert result.status == "completed"
    assert len(controller.commands) == 2
    assert controller.commands[0].pose_6d() == pytest.approx([0.4, 0.0, 0.2, 0.0, 0.0, 0.0])
    assert controller.commands[-1].pose_6d() == pytest.approx([0.402, 0.0, 0.199, 0.0, 0.0, 0.01])
    assert controller.commands[-1].timestamp == pytest.approx(10.04)
    assert controller.gain_calls == 1
    assert result.samples[0].q_meas == pytest.approx((0.0, 0.3, 0.3, 0.0, 0.0, 0.0))


def test_sdk_cartesian_twist_converts_to_bounded_pose_delta_without_joint_fallback() -> None:
    controller = _FakeCartesianController()
    backend = Arx5SdkCartesianRuntimeBackend(
        arx5_module=_FakeSDK,
        controller=controller,
        sleep=lambda _duration_s: None,
        monotonic=lambda: 123.0,
        resume_gain_duration_s=0.002,
    )

    result = backend.execute_eef_command(
        {
            "kind": "eef_twist",
            "send_hz": 50.0,
            "eef_command": {
                "frame": "eef_link",
                "linear_mps": [0.01, 0.0, 0.0],
                "angular_rps": [0.0, 0.0, 0.2],
                "control_period_s": 0.1,
            },
        },
        owner="agent",
    )

    assert result.status == "completed"
    assert controller.commands[-1].pose_6d() == pytest.approx([0.401, 0.0, 0.2, 0.0, 0.0, 0.02])
    assert not hasattr(controller, "set_joint_cmd")


def test_runtime_queue_executes_sdk_cartesian_eef_command(tmp_path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        safe_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=5.0,
    )
    payload["hold_fresh"] = True
    payload["last_hold_wall_time_s"] = time.time()
    payload["hold_age_s"] = 0.0
    payload["readiness"] = {"agent_sysid_smoke_allowed": True, "failed_checks": []}
    session_artifact.write_text(json.dumps(payload), encoding="utf-8")
    controller = _FakeCartesianController()
    backend = Arx5SdkCartesianRuntimeBackend(
        arx5_module=_FakeSDK,
        controller=controller,
        sleep=lambda _duration_s: None,
        monotonic=lambda: 123.0,
        resume_gain_duration_s=0.002,
    )
    runtime = ArmRuntime(
        backend=backend,
        safe_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        runtime_session_id=str(payload["runtime_session_id"]),
    )
    runtime.mark_hold_safe(q_hold=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0))
    submit_eef_command(
        session_artifact_path=session_artifact,
        owner="agent",
        backend="sdk_cartesian",
        kind="eef_pose_delta",
        frame="eef_link",
        expected_q_start=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        control_period_s=0.1,
        send_hz=50.0,
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
        max_heartbeat_age_s=5.0,
        delta_position_m=(0.001, 0.0, 0.0),
        delta_rpy_rad=(0.0, 0.0, 0.0),
    )

    result = execute_pending_runtime_commands(
        session_artifact_path=session_artifact,
        backend=backend,
        runtime=runtime,
        max_heartbeat_age_s=5.0,
    )

    assert result is not None
    assert result["status"] == "completed"
    assert result["movement_command_sent"] is True
    assert result["motion"]["eef_command"]["backend"] == "sdk_cartesian"
    assert controller.commands[-1].pose_6d() == pytest.approx([0.401, 0.0, 0.2, 0.0, 0.0, 0.0])


def test_start_arx5_cartesian_runtime_takes_over_safe_center_pose() -> None:
    backend = Arx5SdkCartesianRuntimeBackend(
        arx5_module=_FakeSDK,
        controller=_FakeCartesianController(),
        sleep=lambda _duration_s: None,
    )

    payload = start_arx5_cartesian_runtime_session(
        backend=backend,
        model="X5",
        interface="can0",
        safe_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        send_hz=50.0,
        hold_hz=50.0,
        max_start_error_rad=0.02,
        max_heartbeat_age_s=5.0,
    )

    assert payload["status"] == "ok"
    assert payload["backend"] == "sdk_cartesian"
    assert payload["mode"] == "hold_safe"
    assert payload["q_hold"] == pytest.approx([0.0, 0.3, 0.3, 0.0, 0.0, 0.0])
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is True
    assert payload["readiness"]["safe_center_allowed"] is True
    assert payload["startup_guard"]["status"] == "pass"
    assert payload["startup_guard"]["policy"] == "current_measured_cartesian_takeover"


def test_start_arx5_cartesian_runtime_takes_over_current_pose_far_from_safe_center() -> None:
    backend = Arx5SdkCartesianRuntimeBackend(
        arx5_module=_FakeSDK,
        controller=_FakeCartesianController(joint_state=_OffsetJointState()),
        sleep=lambda _duration_s: None,
    )

    payload = start_arx5_cartesian_runtime_session(
        backend=backend,
        model="X5",
        interface="can0",
        safe_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        send_hz=50.0,
        hold_hz=50.0,
        max_start_error_rad=0.02,
        max_heartbeat_age_s=5.0,
    )

    assert payload["status"] == "ok"
    assert payload["q_hold"] == pytest.approx([0.2, 0.3, 0.3, 0.0, 0.0, 0.0])
    assert payload["readiness"]["agent_sysid_smoke_allowed"] is True
    assert payload["readiness"]["safe_center_allowed"] is False
    assert payload["readiness"]["safe_center_failed_checks"] == ["q_hold_close_to_safe_center"]
    assert payload["startup_guard"]["policy"] == "current_measured_cartesian_takeover"
    assert backend.controller.damping_calls == 0
