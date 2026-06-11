import json
import subprocess
import sys
import time
from pathlib import Path

import pytest

from armctrl.motion_runtime import FakeMotionBackend, JointTrajectoryPoint, MotionRuntime
from armctrl.arx5_sdk_joint_runtime import Arx5SdkJointRuntimeBackend
from armctrl.sysid_sdk import (
    SDK_HOLD_DAMPING_CONFIRMATION,
    SDK_RECOVER_STARTUP_CONFIRMATION,
    SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
    SDK_TINY_MOTION_CONFIRMATION,
    SdkAgentSysIdSmokeReadinessChecker,
    SdkArmSession,
    SdkJogReal,
    SdkStartupRecovery,
    SdkDoctor,
    SdkHoldDampingCheck,
    SdkMeasuredStateMismatchError,
    SdkTinyMotionExecutor,
    SdkTinyMotionPlanner,
)


class ManualClock:
    def __init__(self) -> None:
        self.now_s = 0.0
        self.sleep_calls = 0
        self.raise_keyboard_interrupt_after_sleep_calls: int | None = None

    def monotonic(self) -> float:
        return self.now_s

    def sleep(self, seconds: float) -> None:
        self.sleep_calls += 1
        if (
            self.raise_keyboard_interrupt_after_sleep_calls is not None
            and self.sleep_calls >= self.raise_keyboard_interrupt_after_sleep_calls
        ):
            raise KeyboardInterrupt
        self.now_s += max(0.0, seconds)


class FakeJointState:
    def __init__(self, dof: int) -> None:
        self._pos = [0.0] * dof
        self._vel = [0.0] * dof
        self._torque = [0.0] * dof
        self.timestamp = 0.0
        self.gripper_pos = 0.0

    def pos(self) -> list[float]:
        return self._pos

    def vel(self) -> list[float]:
        return self._vel

    def torque(self) -> list[float]:
        return self._torque


class FakeDoctorController:
    damping_count = 0
    hold_count = 0
    joint_command_count = 0
    initial_pos: list[float] | None = None

    def __init__(self, robot_config, controller_config, interface: str) -> None:
        self.read_count = 0
        self.commands: list[list[float]] = []

    def get_joint_state(self) -> FakeJointState:
        self.read_count += 1
        state = FakeJointState(6)
        if self.commands:
            state.pos()[:] = self.commands[-1]
        elif FakeDoctorController.initial_pos is not None:
            state.pos()[:] = FakeDoctorController.initial_pos
        return state

    def set_joint_cmd(self, cmd) -> None:
        FakeDoctorController.joint_command_count += 1
        self.commands.append(list(cmd.pos()))

    def set_to_hold(self) -> None:
        FakeDoctorController.hold_count += 1

    def set_to_damping(self) -> None:
        FakeDoctorController.damping_count += 1


class FailingTinyMotionBackend(FakeMotionBackend):
    def __init__(self, *, fail_on_send: int) -> None:
        super().__init__()
        self._fail_on_send = fail_on_send
        self._send_count = 0

    def send_joint_command(self, *args, **kwargs) -> None:
        self._send_count += 1
        if self._send_count >= self._fail_on_send:
            raise RuntimeError("sdk send failed")
        super().send_joint_command(*args, **kwargs)


class FakeRobotConfig:
    joint_dof = 6


class FakeControllerConfig:
    controller_dt = 0.002
    gravity_compensation = False
    background_send_recv = False
    shutdown_to_passive = True


class FakeConfigFactory:
    def __init__(self, value) -> None:
        self._value = value

    @classmethod
    def get_instance(cls):
        return cls(cls._value)

    def get_config(self, *args):
        return self._value


def _reset_fake_controller_config() -> None:
    FakeControllerConfigFactory._value = FakeControllerConfig()


class FakeRobotConfigFactory(FakeConfigFactory):
    _value = FakeRobotConfig()


class FakeControllerConfigFactory(FakeConfigFactory):
    _value = FakeControllerConfig()


class FakeDoctorArx5Module:
    RobotConfigFactory = FakeRobotConfigFactory
    ControllerConfigFactory = FakeControllerConfigFactory
    JointState = FakeJointState
    last_controller: FakeDoctorController | None = None

    @staticmethod
    def Arx5JointController(robot_config, controller_config, interface: str):
        controller = FakeDoctorController(robot_config, controller_config, interface)
        FakeDoctorArx5Module.last_controller = controller
        return controller


class FailingOpenArx5Module(FakeDoctorArx5Module):
    @staticmethod
    def Arx5JointController(robot_config, controller_config, interface: str):
        raise RuntimeError("failed to open CAN interface can0")


class FailingStateReadController(FakeDoctorController):
    def get_joint_state(self) -> FakeJointState:
        self.read_count += 1
        if self.read_count >= 2:
            raise RuntimeError("state read timeout")
        return FakeJointState(6)


class FailingStateReadArx5Module(FakeDoctorArx5Module):
    last_controller: FailingStateReadController | None = None

    @staticmethod
    def Arx5JointController(robot_config, controller_config, interface: str):
        controller = FailingStateReadController(robot_config, controller_config, interface)
        FailingStateReadArx5Module.last_controller = controller
        return controller


class DampingOnlyController:
    damping_count = 0
    joint_command_count = 0
    initial_pos: list[float] | None = None

    def __init__(self, robot_config, controller_config, interface: str) -> None:
        self.commands: list[list[float]] = []

    def get_joint_state(self) -> FakeJointState:
        state = FakeJointState(6)
        if self.commands:
            state.pos()[:] = self.commands[-1]
        elif DampingOnlyController.initial_pos is not None:
            state.pos()[:] = DampingOnlyController.initial_pos
        return state

    def set_joint_cmd(self, cmd) -> None:
        DampingOnlyController.joint_command_count += 1
        self.commands.append(list(cmd.pos()))

    def set_to_damping(self) -> None:
        DampingOnlyController.damping_count += 1


class DampingOnlyArx5Module(FakeDoctorArx5Module):
    last_controller: DampingOnlyController | None = None

    @staticmethod
    def Arx5JointController(robot_config, controller_config, interface: str):
        controller = DampingOnlyController(robot_config, controller_config, interface)
        DampingOnlyArx5Module.last_controller = controller
        return controller


class FakeGain:
    def __init__(self, kp: float, kd: float, gripper_kp: float, gripper_kd: float) -> None:
        self.kp = float(kp)
        self.kd = float(kd)
        self.gripper_kp = float(gripper_kp)
        self.gripper_kd = float(gripper_kd)

    def __mul__(self, scalar: float):
        return FakeGain(
            self.kp * scalar,
            self.kd * scalar,
            self.gripper_kp * scalar,
            self.gripper_kd * scalar,
        )

    __rmul__ = __mul__

    def __add__(self, other):
        return FakeGain(
            self.kp + other.kp,
            self.kd + other.kd,
            self.gripper_kp + other.gripper_kp,
            self.gripper_kd + other.gripper_kd,
        )


class MotionGainControllerConfig(FakeControllerConfig):
    default_kp = 120.0
    default_kd = 2.5
    default_gripper_kp = 30.0
    default_gripper_kd = 1.2


class MotionGainController:
    def __init__(self, robot_config, controller_config, interface: str) -> None:
        self._controller_config = controller_config
        self._gain = FakeGain(0.0, 0.0, 0.0, 0.0)
        self.gain_history: list[FakeGain] = []
        self.commands: list[tuple[list[float], float]] = []
        self.trajectory_batches: list[list[tuple[list[float], float]]] = []
        self.velocity_commands: list[list[float]] = []
        self._current_pos = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def get_controller_config(self):
        return self._controller_config

    def get_gain(self):
        return self._gain

    def set_gain(self, gain) -> None:
        self._gain = gain
        self.gain_history.append(gain)

    def get_timestamp(self) -> float:
        return 10.0

    def get_joint_state(self) -> FakeJointState:
        state = FakeJointState(6)
        state.pos()[:] = self._current_pos
        return state

    def set_joint_cmd(self, cmd) -> None:
        q_cmd = list(cmd.pos())
        self._current_pos = q_cmd
        self.commands.append((q_cmd, float(cmd.timestamp)))
        self.velocity_commands.append(list(cmd.vel()))

    def set_joint_traj(self, trajectory) -> None:
        batch = [(list(cmd.pos()), float(cmd.timestamp)) for cmd in trajectory]
        self.trajectory_batches.append(batch)
        self.set_joint_cmd(trajectory[-1])

    def set_to_damping(self) -> None:
        pass


class MotionGainControllerConfigFactory(FakeConfigFactory):
    _value = MotionGainControllerConfig()


class MotionGainArx5Module(FakeDoctorArx5Module):
    ControllerConfigFactory = MotionGainControllerConfigFactory
    Gain = FakeGain
    last_controller: MotionGainController | None = None

    @staticmethod
    def Arx5JointController(robot_config, controller_config, interface: str):
        controller = MotionGainController(robot_config, controller_config, interface)
        MotionGainArx5Module.last_controller = controller
        return controller


class MethodStyleGain:
    def __init__(self, kp: float, kd: float, gripper_kp: float, gripper_kd: float) -> None:
        self._kp = _fake_gain_values(kp)
        self._kd = _fake_gain_values(kd)
        self._gripper_kp = float(gripper_kp)
        self._gripper_kd = float(gripper_kd)

    def kp(self) -> list[float]:
        return self._kp

    def kd(self) -> list[float]:
        return self._kd

    def gripper_kp(self) -> float:
        return self._gripper_kp

    def gripper_kd(self) -> float:
        return self._gripper_kd

    def __mul__(self, scalar: float):
        return MethodStyleGain(
            [value * scalar for value in self._kp],
            [value * scalar for value in self._kd],
            self._gripper_kp * scalar,
            self._gripper_kd * scalar,
        )

    __rmul__ = __mul__

    def __add__(self, other):
        return MethodStyleGain(
            [left + right for left, right in zip(self._kp, other.kp())],
            [left + right for left, right in zip(self._kd, other.kd())],
            self._gripper_kp + other.gripper_kp(),
            self._gripper_kd + other.gripper_kd(),
        )


def _fake_gain_values(value) -> list[float]:
    if isinstance(value, (list, tuple)):
        return [float(item) for item in value]
    return [float(value)]


class MethodStyleControllerConfig(FakeControllerConfig):
    def default_kp(self) -> float:
        return [120.0, 110.0, 100.0, 90.0, 80.0, 70.0]

    def default_kd(self) -> float:
        return [2.5, 2.4, 2.3, 2.2, 2.1, 2.0]

    def default_gripper_kp(self) -> float:
        return 30.0

    def default_gripper_kd(self) -> float:
        return 1.2


class MethodStyleControllerConfigFactory(FakeConfigFactory):
    _value = MethodStyleControllerConfig()


class MethodStyleGainArx5Module(MotionGainArx5Module):
    ControllerConfigFactory = MethodStyleControllerConfigFactory
    Gain = MethodStyleGain
    last_controller: MotionGainController | None = None

    @staticmethod
    def Arx5JointController(robot_config, controller_config, interface: str):
        controller = MotionGainController(robot_config, controller_config, interface)
        controller._gain = MethodStyleGain([0.0] * 6, [0.0] * 6, 0.0, 0.0)
        MethodStyleGainArx5Module.last_controller = controller
        return controller


def _passing_doctor_artifact() -> dict[str, object]:
    return {
        "schema": "armctrl.sysid_sdk_doctor.v1",
        "interface_status": {"interface": "can0", "status": "opened_read_only"},
        "controller": {"status": "initialized_read_only", "controller_dt_s": 0.002},
        "state_read": {"status": "ok", "state_read_hz": 100.0},
        "timestamp_policy": {"clock": "host_monotonic", "monotonic": True},
        "motion_commands_sent": False,
    }


def _passing_hold_damping_artifact() -> dict[str, object]:
    return {
        "schema": "armctrl.sysid_sdk_hold_damping_check.v1",
        "landing_policy": "hold_then_damping",
        "hold": {"status": "called"},
        "damping": {"status": "called"},
        "mode_call_sequence": [
            {"name": "hold", "method": "set_to_hold", "status": "called"},
            {"name": "damping", "method": "set_to_damping", "status": "called"},
        ],
        "joint_commands_sent": False,
    }


def _passing_damping_only_artifact() -> dict[str, object]:
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


def _passing_tiny_motion_execute_artifact() -> dict[str, object]:
    return {
        "schema": "armctrl.sysid_sdk_tiny_motion_execute.v1",
        "backend": "arx5_sdk",
        "hardware_motion": True,
        "movement_command_sent": True,
        "prerequisites": {"doctor": "pass", "hold_damping": "pass"},
        "motion_runtime": {
            "schema": "armctrl.motion_runtime_result.v1",
            "status": "completed",
            "producer": "tiny_motion",
            "mode": "trajectory_replay",
            "trajectory_sample_hz": 50.0,
            "actual_send_hz": 50.0,
            "send_jitter_ms_p95": 0.0,
            "send_jitter_ms_p99": 0.0,
            "controller_dt_s": 0.002,
            "sample_count": 2,
            "fault_flags": [],
            "samples": [
                {
                    "q_cmd": [0.0, 0.3],
                    "q_meas": [0.0, 0.3],
                    "fault_flags": [],
                },
                {
                    "q_cmd": [0.0, 0.302],
                    "q_meas": [0.0, 0.302],
                    "fault_flags": [],
                },
            ],
            "landing_mode": "hold",
        },
        "fault_landing_mode": "damping",
    }


def _passing_startup_recovery_artifact() -> dict[str, object]:
    return {
        "schema": "armctrl.sdk_startup_recovery.v1",
        "run_status": "completed",
        "hardware_motion": True,
        "movement_command_sent": True,
        "q_start": [1.312, 0.0, 0.0, -0.055, 0.0, 0.0],
        "q_target": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
        "motion_runtime": {
            "schema": "armctrl.motion_runtime_result.v1",
            "status": "completed",
            "producer": "sdk_startup_recovery",
            "mode": "trajectory_replay",
            "trajectory_sample_hz": 50.0,
            "actual_send_hz": 50.0,
            "send_jitter_ms_p95": 0.0,
            "send_jitter_ms_p99": 0.0,
            "controller_dt_s": 0.002,
            "sample_count": 133,
            "fault_flags": [],
            "samples": [],
            "landing_mode": "hold",
        },
        "fault_landing_mode": "damping",
    }


def _passing_runtime_status_artifact() -> dict[str, object]:
    now_s = time.time()
    return {
        "schema": "armctrl.arm_runtime_status.v1",
        "status": "ok",
        "mode": "hold_safe",
        "owner": None,
        "q_meas": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
        "q_hold": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
        "safe_center": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
        "hold_fresh": True,
        "last_hold_wall_time_s": now_s,
        "fault_flags": [],
        "heartbeat": {
            "fresh": True,
            "age_s": 0.01,
            "max_age_s": 5.0,
            "wall_time_s": now_s,
        },
        "readiness": {
            "agent_sysid_smoke_allowed": True,
            "failed_checks": [],
        },
    }


def _passing_session_artifact() -> dict[str, object]:
    return {
        "schema": "armctrl.sdk_arm_session.v1",
        "model": "X5",
        "interface": "can0",
        "session_status": "armed",
        "movement_allowed": True,
        "controller": {"controller_dt_s": 0.002},
        "state": {"q_meas": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]},
        "landing_policy": "hold_then_damping",
        "fault_landing_mode": "damping",
    }


def test_cli_sysid_sdk_preflight_is_read_only_and_reports_sdk_import_status() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "sdk-preflight",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_sdk_preflight.v1"
    assert payload["model"] == "X5"
    assert payload["interface"] == "can0"
    assert payload["read_only"] is True
    assert payload["movement_allowed"] is False
    assert payload["sdk"]["module"] == "arx5_interface"
    assert payload["sdk"]["status"] in {"available", "missing"}
    assert payload["next_gate"] == "sdk_runner_confirm_then_hardware_validation"


def test_sdk_doctor_reads_controller_dt_and_state_frequency_without_motion() -> None:
    clock = ManualClock()
    FakeDoctorController.damping_count = 0
    FakeDoctorController.hold_count = 0
    FakeDoctorController.joint_command_count = 0

    result = SdkDoctor(
        arx5_module=FakeDoctorArx5Module,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    ).run(
        model="X5",
        interface="can0",
        state_sample_count=3,
        state_sample_period_s=0.01,
    )

    payload = result.to_json()

    assert payload["schema"] == "armctrl.sysid_sdk_doctor.v1"
    assert payload["read_only"] is True
    assert payload["movement_allowed"] is False
    assert payload["interface_status"] == {
        "interface": "can0",
        "status": "opened_read_only",
    }
    assert payload["controller"]["controller_dt_s"] == 0.002
    assert payload["state_read"]["sample_count"] == 3
    assert payload["state_read"]["state_read_hz"] == pytest.approx(100.0)
    assert payload["state_read"]["q_meas_last"] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert payload["timestamp_policy"] == {
        "clock": "host_monotonic",
        "monotonic": True,
    }
    assert payload["measurement_window"] == {
        "requested_sample_count": 3,
        "requested_sample_period_s": 0.01,
        "observed_sample_count": 3,
        "first_read_monotonic_s": 0.0,
        "last_read_monotonic_s": 0.02,
        "duration_s": 0.02,
    }
    assert payload["doctor_gate"] == {
        "status": "pass",
        "checks": {
            "interface_opened_read_only": True,
            "controller_initialized_read_only": True,
            "controller_dt_measured": True,
            "state_read_ok": True,
            "state_read_hz_measured": True,
            "timestamps_monotonic": True,
            "no_motion_commands_sent": True,
        },
        "failed_checks": [],
    }
    assert payload["motion_commands_sent"] is False
    assert FakeDoctorArx5Module.last_controller is not None
    assert FakeDoctorArx5Module.last_controller.read_count == 3
    assert FakeDoctorController.damping_count == 0
    assert FakeDoctorController.hold_count == 0
    assert FakeDoctorController.joint_command_count == 0


def test_sdk_doctor_returns_failure_artifact_when_controller_open_fails() -> None:
    FakeDoctorController.joint_command_count = 0

    result = SdkDoctor(arx5_module=FailingOpenArx5Module).run(
        model="X5",
        interface="can0",
        state_sample_count=3,
        state_sample_period_s=0.01,
    )

    payload = result.to_json()

    assert payload["schema"] == "armctrl.sysid_sdk_doctor.v1"
    assert payload["read_only"] is True
    assert payload["movement_allowed"] is False
    assert payload["motion_commands_sent"] is False
    assert payload["interface_status"] == {
        "interface": "can0",
        "status": "open_failed",
        "reason": "failed to open CAN interface can0",
    }
    assert payload["controller"]["status"] == "init_failed"
    assert payload["controller"]["controller_dt_s"] == 0.002
    assert payload["state_read"] == {
        "status": "not_run",
        "sample_count": 0,
        "state_read_hz": None,
    }
    assert payload["timestamp_policy"] == {
        "clock": "host_monotonic",
        "monotonic": None,
    }
    assert payload["doctor_gate"]["status"] == "fail"
    assert "interface_opened_read_only" in payload["doctor_gate"]["failed_checks"]
    assert "state_read_ok" in payload["doctor_gate"]["failed_checks"]
    assert FakeDoctorController.joint_command_count == 0


def test_sdk_doctor_returns_failure_artifact_when_state_read_fails() -> None:
    clock = ManualClock()
    FakeDoctorController.joint_command_count = 0
    FailingStateReadArx5Module.last_controller = None

    result = SdkDoctor(
        arx5_module=FailingStateReadArx5Module,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    ).run(
        model="X5",
        interface="can0",
        state_sample_count=3,
        state_sample_period_s=0.01,
    )

    payload = result.to_json()

    assert payload["schema"] == "armctrl.sysid_sdk_doctor.v1"
    assert payload["interface_status"] == {
        "interface": "can0",
        "status": "opened_read_only",
    }
    assert payload["controller"]["status"] == "initialized_read_only"
    assert payload["controller"]["controller_dt_s"] == 0.002
    assert payload["state_read"] == {
        "status": "failed",
        "sample_count": 1,
        "state_read_hz": None,
        "q_meas_last": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "reason": "state read timeout",
    }
    assert payload["timestamp_policy"] == {
        "clock": "host_monotonic",
        "monotonic": True,
    }
    assert payload["doctor_gate"]["status"] == "fail"
    assert payload["doctor_gate"]["checks"]["state_read_ok"] is False
    assert payload["doctor_gate"]["checks"]["state_read_hz_measured"] is False
    assert payload["motion_commands_sent"] is False
    assert FailingStateReadArx5Module.last_controller is not None
    assert FailingStateReadArx5Module.last_controller.read_count == 2
    assert FakeDoctorController.joint_command_count == 0


def test_sdk_hold_damping_check_requires_confirm_and_never_sends_joint_commands() -> None:
    FakeDoctorController.damping_count = 0
    FakeDoctorController.hold_count = 0
    FakeDoctorController.joint_command_count = 0

    checker = SdkHoldDampingCheck(arx5_module=FakeDoctorArx5Module)

    with pytest.raises(PermissionError):
        checker.run(model="X5", interface="can0", confirm="nope")

    result = checker.run(
        model="X5",
        interface="can0",
        confirm=SDK_HOLD_DAMPING_CONFIRMATION,
    )

    payload = result.to_json()

    assert payload["schema"] == "armctrl.sysid_sdk_hold_damping_check.v1"
    assert payload["movement_allowed"] is False
    assert payload["mode_change_allowed"] is True
    assert payload["landing_policy"] == "hold_then_damping"
    assert payload["hold"]["status"] == "called"
    assert payload["damping"]["status"] == "called"
    assert payload["mode_call_sequence"] == [
        {"name": "hold", "method": "set_to_hold", "status": "called"},
        {"name": "damping", "method": "set_to_damping", "status": "called"},
    ]
    assert payload["joint_commands_sent"] is False
    assert payload["hold_damping_gate"] == {
        "status": "pass",
        "checks": {
            "damping_called": True,
            "landing_policy_supported": True,
            "mode_call_sequence_recorded": True,
            "mode_call_sequence_safe_order": True,
            "no_joint_commands_sent": True,
        },
        "failed_checks": [],
    }
    assert payload["fault_landing_mode"] == "damping"
    assert FakeDoctorController.hold_count == 1
    assert FakeDoctorController.damping_count == 1
    assert FakeDoctorController.joint_command_count == 0


def test_sdk_hold_damping_check_accepts_damping_only_sdk_contract() -> None:
    DampingOnlyController.damping_count = 0
    DampingOnlyController.joint_command_count = 0

    checker = SdkHoldDampingCheck(arx5_module=DampingOnlyArx5Module)

    result = checker.run(
        model="X5",
        interface="can0",
        confirm=SDK_HOLD_DAMPING_CONFIRMATION,
        doctor_artifact=_passing_doctor_artifact(),
    )

    payload = result.to_json()

    assert payload["landing_policy"] == "damping_only"
    assert payload["mode_change_allowed"] is True
    assert payload["hold"] == {"status": "unsupported", "method": "set_to_hold"}
    assert payload["damping"] == {"status": "called", "method": "set_to_damping"}
    assert payload["hold_damping_gate"]["status"] == "pass"
    assert payload["hold_damping_gate"]["checks"] == {
        "damping_called": True,
        "landing_policy_supported": True,
        "mode_call_sequence_recorded": True,
        "mode_call_sequence_safe_order": True,
        "no_joint_commands_sent": True,
    }
    assert DampingOnlyController.damping_count == 1
    assert DampingOnlyController.joint_command_count == 0


def test_sdk_arm_session_reads_measured_state_without_joint_commands() -> None:
    FakeDoctorController.damping_count = 0
    FakeDoctorController.hold_count = 0
    FakeDoctorController.joint_command_count = 0
    FakeDoctorController.initial_pos = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]

    result = SdkArmSession(arx5_module=FakeDoctorArx5Module).run(
        model="X5",
        interface="can0",
        confirm="I UNDERSTAND THIS WILL ARM THE SDK SESSION WITHOUT MOVING",
        doctor_artifact=_passing_doctor_artifact(),
        hold_damping_artifact=_passing_hold_damping_artifact(),
    )

    payload = result.to_json()

    assert payload["schema"] == "armctrl.sdk_arm_session.v1"
    assert payload["session_status"] == "armed"
    assert payload["movement_allowed"] is True
    assert payload["joint_commands_sent"] is False
    assert payload["state"]["q_meas"] == [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
    assert payload["controller"]["controller_dt_s"] == 0.002
    assert payload["landing_policy"] == "hold_then_damping"
    assert FakeDoctorController.joint_command_count == 0
    assert FakeDoctorController.initial_pos == [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
    FakeDoctorController.initial_pos = None


def test_sdk_jog_real_uses_session_measured_state_and_sends_limited_motion() -> None:
    clock = ManualClock()
    FakeDoctorController.damping_count = 0
    FakeDoctorController.hold_count = 0
    FakeDoctorController.joint_command_count = 0
    FakeDoctorController.initial_pos = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]

    result = SdkJogReal(
        arx5_module=FakeDoctorArx5Module,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    ).run(
        session_artifact=_passing_session_artifact(),
        joint_index=2,
        delta_rad=0.002,
        max_delta_rad=0.005,
        send_hz=50.0,
        confirm="I UNDERSTAND THIS WILL JOG THE REAL ARM",
    )

    payload = result.to_json()

    assert payload["schema"] == "armctrl.sdk_jog_real.v1"
    assert payload["run_status"] == "completed"
    assert payload["movement_command_sent"] is True
    assert payload["q_start"] == [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    assert payload["q_target"] == [0.0, 0.302, 0.3, 0.0, 0.0, 0.0]
    assert payload["motion_runtime"]["actual_send_hz"] == pytest.approx(50.0)
    assert FakeDoctorController.joint_command_count == 3
    FakeDoctorController.initial_pos = None


def test_sdk_jog_real_rejects_delta_above_limit_before_send() -> None:
    FakeDoctorController.joint_command_count = 0

    with pytest.raises(ValueError, match="delta_rad must be within"):
        SdkJogReal(arx5_module=FakeDoctorArx5Module).run(
            session_artifact=_passing_session_artifact(),
            joint_index=2,
            delta_rad=0.02,
            max_delta_rad=0.005,
            send_hz=50.0,
            confirm="I UNDERSTAND THIS WILL JOG THE REAL ARM",
        )

    assert FakeDoctorController.joint_command_count == 0


def test_sdk_startup_recovery_ramps_from_measured_droop_to_startup_pose() -> None:
    clock = ManualClock()
    _reset_fake_controller_config()
    FakeDoctorController.damping_count = 0
    FakeDoctorController.hold_count = 0
    FakeDoctorController.joint_command_count = 0
    FakeDoctorController.initial_pos = [1.312, 0.0, 0.0, -0.055, 0.0, 0.0]

    result = SdkStartupRecovery(
        arx5_module=FakeDoctorArx5Module,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    ).run(
        session_artifact=_passing_session_artifact(),
        q_target=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        send_hz=50.0,
        hold_seconds=0.0,
        max_joint_step_rad=0.01,
        confirm=SDK_RECOVER_STARTUP_CONFIRMATION,
    )

    payload = result.to_json()

    assert payload["schema"] == "armctrl.sdk_startup_recovery.v1"
    assert payload["run_status"] == "completed"
    assert payload["movement_command_sent"] is True
    assert payload["q_start"] == [1.312, 0.0, 0.0, -0.055, 0.0, 0.0]
    assert payload["q_target"] == [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    assert payload["recovery_plan"]["max_joint_step_rad"] == 0.01
    assert payload["recovery_plan"]["step_count"] == 132
    assert payload["recovery_plan"]["max_planned_step_rad"] <= 0.01
    assert FakeDoctorController.joint_command_count == 134
    assert FakeDoctorController.initial_pos == [1.312, 0.0, 0.0, -0.055, 0.0, 0.0]
    FakeDoctorController.initial_pos = None


def test_sdk_startup_recovery_streams_active_hold_after_reaching_pose() -> None:
    clock = ManualClock()
    _reset_fake_controller_config()
    FakeDoctorController.damping_count = 0
    FakeDoctorController.hold_count = 0
    FakeDoctorController.joint_command_count = 0
    FakeDoctorController.initial_pos = [0.02, 0.28, 0.31, 0.0, 0.0, 0.0]

    result = SdkStartupRecovery(
        arx5_module=FakeDoctorArx5Module,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    ).run(
        session_artifact=_passing_session_artifact(),
        q_target=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        send_hz=50.0,
        hold_seconds=0.06,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        confirm=SDK_RECOVER_STARTUP_CONFIRMATION,
    )

    payload = result.to_json()
    controller = FakeDoctorArx5Module.last_controller
    config = FakeControllerConfigFactory._value

    assert payload["run_status"] == "completed"
    assert payload["active_hold"] == {
        "requested": True,
        "duration_s": pytest.approx(0.06),
        "hold_hz": pytest.approx(50.0),
        "command_count": 3,
        "until_interrupt": False,
        "landing_mode": "hold",
    }
    assert config.shutdown_to_passive is False
    assert controller is not None
    assert controller.commands[-3:] == [
        [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
        [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
        [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
    ]
    assert FakeDoctorController.damping_count == 0
    FakeDoctorController.initial_pos = None


def test_sdk_startup_recovery_defaults_to_hold_until_interrupt() -> None:
    clock = ManualClock()
    clock.raise_keyboard_interrupt_after_sleep_calls = 7
    _reset_fake_controller_config()
    FakeDoctorController.damping_count = 0
    FakeDoctorController.hold_count = 0
    FakeDoctorController.joint_command_count = 0
    FakeDoctorController.initial_pos = [0.0, 0.29, 0.29, 0.0, 0.0, 0.0]

    with pytest.raises(KeyboardInterrupt):
        SdkStartupRecovery(
            arx5_module=FakeDoctorArx5Module,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        ).run(
            session_artifact=_passing_session_artifact(),
            q_target=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
            send_hz=50.0,
            max_joint_step_rad=0.01,
            confirm=SDK_RECOVER_STARTUP_CONFIRMATION,
        )

    controller = FakeDoctorArx5Module.last_controller
    assert controller is not None
    assert FakeControllerConfigFactory._value.shutdown_to_passive is False
    assert FakeDoctorController.joint_command_count > 4
    assert controller.commands[-1] == [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    assert FakeDoctorController.damping_count == 1
    FakeDoctorController.initial_pos = None


def test_sdk_startup_recovery_rejects_unsafe_step_limit_before_send() -> None:
    FakeDoctorController.joint_command_count = 0

    with pytest.raises(ValueError, match="max_joint_step_rad must be positive"):
        SdkStartupRecovery(arx5_module=FakeDoctorArx5Module).run(
            session_artifact=_passing_session_artifact(),
            q_target=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
            send_hz=50.0,
            max_joint_step_rad=0.0,
            confirm=SDK_RECOVER_STARTUP_CONFIRMATION,
        )

    assert FakeDoctorController.joint_command_count == 0


def test_arx5_backend_restores_motion_gain_and_timestamps_commands_before_streaming() -> None:
    clock = ManualClock()
    backend = Arx5SdkJointRuntimeBackend(
        model="X5",
        interface="can0",
        arx5_module=MotionGainArx5Module,
        controller_dt_s=0.002,
        sleep=clock.sleep,
    )

    backend.enter_hold_or_damping()
    runtime = MotionRuntime(
        backend=backend,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    result = runtime.execute_trajectory(
        [
            JointTrajectoryPoint(time_s=0.0, q=(0.9, 0.1, 0.0, 0.0, 0.0, 0.0)),
            JointTrajectoryPoint(time_s=0.02, q=(0.8, 0.2, 0.0, 0.0, 0.0, 0.0)),
        ],
        producer="sdk_startup_recovery",
        trajectory_sample_hz=50.0,
        hold_after=True,
    )

    controller = MotionGainArx5Module.last_controller
    assert result.status == "completed"
    assert controller is not None
    assert controller.gain_history
    assert controller.gain_history[-1].kp == pytest.approx(120.0)
    assert controller.gain_history[-1].kd == pytest.approx(2.5)
    assert controller.commands[0] == ([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 10.002)
    assert controller.commands[1][0] == pytest.approx(
        [0.85, 0.15, 0.0, 0.0, 0.0, 0.0]
    )
    assert controller.commands[1][1] == pytest.approx(10.01)
    assert controller.commands[2][0] == [0.8, 0.2, 0.0, 0.0, 0.0, 0.0]
    assert controller.commands[2][1] == pytest.approx(10.01)
    assert controller.trajectory_batches == []
    assert backend.sdk_stream_policy["api"] == "joint_cmd"
    assert backend.sdk_stream_policy["target_sync_before_gain_restore"] is True


def test_arx5_backend_writes_trajectory_velocity_into_sdk_joint_state() -> None:
    clock = ManualClock()
    backend = Arx5SdkJointRuntimeBackend(
        model="X5",
        interface="can0",
        arx5_module=MotionGainArx5Module,
        controller_dt_s=0.002,
        sleep=clock.sleep,
    )

    backend.enter_hold_or_damping()
    runtime = MotionRuntime(
        backend=backend,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    result = runtime.execute_trajectory(
        [
            JointTrajectoryPoint(
                time_s=0.0,
                q=(0.9, 0.1, 0.0, 0.0, 0.0, 0.0),
                dq=(-0.1, 0.1, 0.0, 0.0, 0.0, 0.0),
            ),
        ],
        producer="sysid",
        trajectory_sample_hz=100.0,
    )

    controller = MotionGainArx5Module.last_controller
    assert result.status == "completed"
    assert controller is not None
    assert controller.velocity_commands[-1] == pytest.approx(
        [-0.1, 0.1, 0.0, 0.0, 0.0, 0.0]
    )


def test_arx5_backend_accepts_method_style_sdk_gain_api() -> None:
    clock = ManualClock()
    backend = Arx5SdkJointRuntimeBackend(
        model="X5",
        interface="can0",
        arx5_module=MethodStyleGainArx5Module,
        controller_dt_s=0.002,
        sleep=clock.sleep,
    )

    backend.enter_hold_or_damping()
    runtime = MotionRuntime(
        backend=backend,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    result = runtime.execute_trajectory(
        [JointTrajectoryPoint(time_s=0.0, q=(0.9, 0.1, 0.0, 0.0, 0.0, 0.0))],
        producer="sdk_startup_recovery",
        trajectory_sample_hz=50.0,
        hold_after=True,
    )

    controller = MethodStyleGainArx5Module.last_controller
    assert result.status == "completed"
    assert controller is not None
    assert controller.gain_history
    assert list(controller.gain_history[-1].kp()) == pytest.approx(
        [120.0, 110.0, 100.0, 90.0, 80.0, 70.0]
    )
    assert controller.commands[0] == ([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 10.002)


def test_cli_sysid_sdk_arm_session_routes_and_writes_artifact(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    output_artifact = tmp_path / "session.json"
    doctor_artifact.write_text(json.dumps(_passing_doctor_artifact()), encoding="utf-8")
    hold_damping_artifact.write_text(
        json.dumps(_passing_hold_damping_artifact()),
        encoding="utf-8",
    )

    class StubArmSession:
        def run(self, **kwargs):
            class Result:
                def to_json(self) -> dict[str, object]:
                    return {
                        **_passing_session_artifact(),
                        "joint_commands_sent": False,
                    }

            return Result()

    monkeypatch.setattr(cli, "SdkArmSession", StubArmSession)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-arm-session",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--doctor-artifact",
            str(doctor_artifact),
            "--hold-damping-artifact",
            str(hold_damping_artifact),
            "--confirm",
            "I UNDERSTAND THIS WILL ARM THE SDK SESSION WITHOUT MOVING",
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 0
    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sdk_arm_session.v1"
    assert payload["artifacts"]["session"] == str(output_artifact)


def test_cli_sysid_sdk_jog_real_routes_session_artifact(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    session_artifact = tmp_path / "session.json"
    output_artifact = tmp_path / "jog.json"
    session_artifact.write_text(
        json.dumps(_passing_session_artifact()),
        encoding="utf-8",
    )
    calls: list[dict[str, object]] = []

    class StubJogReal:
        def run(self, **kwargs):
            calls.append(kwargs)

            class Result:
                def to_json(self) -> dict[str, object]:
                    return {
                        "schema": "armctrl.sdk_jog_real.v1",
                        "run_status": "completed",
                        "hardware_motion": True,
                        "movement_command_sent": True,
                        "q_start": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
                        "q_target": [0.0, 0.302, 0.3, 0.0, 0.0, 0.0],
                        "motion_runtime": {"status": "completed"},
                    }

            return Result()

    monkeypatch.setattr(cli, "SdkJogReal", StubJogReal)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-jog-real",
            "--session-artifact",
            str(session_artifact),
            "--joint-index",
            "2",
            "--delta-rad",
            "0.002",
            "--confirm",
            "I UNDERSTAND THIS WILL JOG THE REAL ARM",
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 0
    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sdk_jog_real.v1"
    assert payload["artifacts"]["jog"] == str(output_artifact)
    assert calls[0]["joint_index"] == 2
    assert calls[0]["delta_rad"] == 0.002


def test_cli_sysid_sdk_recover_startup_real_routes_session_artifact(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    session_artifact = tmp_path / "session.json"
    output_artifact = tmp_path / "startup_recovery.json"
    session_artifact.write_text(
        json.dumps(_passing_session_artifact()),
        encoding="utf-8",
    )
    calls: list[dict[str, object]] = []

    class StubStartupRecovery:
        def run(self, **kwargs):
            calls.append(kwargs)

            class Result:
                def to_json(self) -> dict[str, object]:
                    return {
                        "schema": "armctrl.sdk_startup_recovery.v1",
                        "run_status": "completed",
                        "hardware_motion": True,
                        "movement_command_sent": True,
                        "q_start": [1.312, 0.0, 0.0, -0.055, 0.0, 0.0],
                        "q_target": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
                        "motion_runtime": {"status": "completed"},
                    }

            return Result()

    monkeypatch.setattr(cli, "SdkStartupRecovery", StubStartupRecovery)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-recover-startup-real",
            "--session-artifact",
            str(session_artifact),
            "--q-target",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--max-joint-step-rad",
            "0.01",
            "--confirm",
            "I UNDERSTAND THIS WILL RECOVER THE REAL ARM TO STARTUP POSE",
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 0
    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sdk_startup_recovery.v1"
    assert payload["artifacts"]["startup_recovery"] == str(output_artifact)
    assert calls[0]["q_target"] == (0.0, 0.3, 0.3, 0.0, 0.0, 0.0)
    assert calls[0]["max_joint_step_rad"] == 0.01
    assert calls[0]["hold_seconds"] is None


def test_sdk_tiny_motion_plan_accepts_damping_only_landing_contract() -> None:
    planner = SdkTinyMotionPlanner(max_delta_rad=0.005)

    result = planner.plan(
        doctor_artifact=_passing_doctor_artifact(),
        hold_damping_artifact=_passing_damping_only_artifact(),
        joint_index=2,
        delta_rad=0.002,
        confirm=SDK_TINY_MOTION_CONFIRMATION,
    )

    payload = result.to_json()

    assert payload["tiny_motion_eligible"] is True
    assert payload["prerequisites"] == {
        "doctor": "pass",
        "hold_damping": "pass",
    }


def test_sdk_doctor_prerequisite_requires_measured_controller_dt() -> None:
    stale_doctor_artifact = _passing_doctor_artifact()
    stale_doctor_artifact["controller"] = {
        "status": "initialized_read_only",
        "controller_dt_s": None,
    }
    checker = SdkHoldDampingCheck(arx5_module=FakeDoctorArx5Module)

    with pytest.raises(RuntimeError, match="sdk doctor prerequisite failed"):
        checker.run(
            model="X5",
            interface="can0",
            confirm=SDK_HOLD_DAMPING_CONFIRMATION,
            doctor_artifact=stale_doctor_artifact,
        )


def test_sdk_doctor_prerequisite_requires_measured_state_read_hz() -> None:
    stale_doctor_artifact = _passing_doctor_artifact()
    stale_doctor_artifact["state_read"] = {
        "status": "ok",
        "sample_count": 1,
        "state_read_hz": None,
    }
    planner = SdkTinyMotionPlanner()

    with pytest.raises(RuntimeError, match="tiny motion prerequisites failed: doctor"):
        planner.plan(
            doctor_artifact=stale_doctor_artifact,
            hold_damping_artifact=_passing_hold_damping_artifact(),
            joint_index=1,
            delta_rad=0.001,
            confirm=SDK_TINY_MOTION_CONFIRMATION,
        )


def test_sdk_doctor_prerequisite_respects_failed_doctor_gate() -> None:
    failed_gate_doctor_artifact = _passing_doctor_artifact()
    failed_gate_doctor_artifact["doctor_gate"] = {
        "status": "fail",
        "checks": {
            "interface_opened_read_only": True,
            "controller_initialized_read_only": True,
            "controller_dt_measured": True,
            "state_read_ok": True,
            "state_read_hz_measured": True,
            "timestamps_monotonic": True,
            "no_motion_commands_sent": True,
        },
        "failed_checks": ["operator_marked_recheck_required"],
    }
    checker = SdkHoldDampingCheck(arx5_module=FakeDoctorArx5Module)

    with pytest.raises(RuntimeError, match="sdk doctor prerequisite failed"):
        checker.run(
            model="X5",
            interface="can0",
            confirm=SDK_HOLD_DAMPING_CONFIRMATION,
            doctor_artifact=failed_gate_doctor_artifact,
        )

    planner = SdkTinyMotionPlanner()
    with pytest.raises(RuntimeError, match="tiny motion prerequisites failed: doctor"):
        planner.plan(
            doctor_artifact=failed_gate_doctor_artifact,
            hold_damping_artifact=_passing_hold_damping_artifact(),
            joint_index=1,
            delta_rad=0.001,
            confirm=SDK_TINY_MOTION_CONFIRMATION,
        )


def test_cli_sysid_sdk_doctor_is_read_only_and_reports_measurement_contract(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    output_artifact = tmp_path / "doctor.json"

    class PassingDoctor:
        def run(self, **kwargs):
            class Result:
                def to_json(self) -> dict[str, object]:
                    return {
                        "schema": "armctrl.sysid_sdk_doctor.v1",
                        "model": "X5",
                        "interface": "can0",
                        "read_only": True,
                        "movement_allowed": False,
                        "sdk": {"module": "arx5_interface", "status": "available"},
                        "interface_status": {
                            "interface": "can0",
                            "status": "opened_read_only",
                        },
                        "controller": {
                            "status": "initialized_read_only",
                            "controller_dt_s": 0.002,
                        },
                        "state_read": {
                            "status": "ok",
                            "sample_count": 2,
                            "state_read_hz": 100.0,
                            "q_meas_last": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        },
                        "timestamp_policy": {
                            "clock": "host_monotonic",
                            "monotonic": True,
                        },
                        "motion_commands_sent": False,
                        "doctor_gate": {
                            "status": "pass",
                            "checks": {
                                "interface_opened_read_only": True,
                                "controller_initialized_read_only": True,
                                "controller_dt_measured": True,
                                "state_read_ok": True,
                                "state_read_hz_measured": True,
                                "timestamps_monotonic": True,
                                "no_motion_commands_sent": True,
                            },
                            "failed_checks": [],
                        },
                        "next_gate": "hold_damping_validation_before_tiny_motion",
                    }

            return Result()

    monkeypatch.setattr(cli, "SdkDoctor", PassingDoctor)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-doctor",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--state-sample-count",
            "2",
            "--state-sample-period",
            "0.001",
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 0
    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_sdk_doctor.v1"
    assert payload["read_only"] is True
    assert payload["movement_allowed"] is False
    assert payload["motion_commands_sent"] is False
    assert payload["interface_status"]["interface"] == "can0"
    assert payload["interface_status"]["status"] in {
        "not_checked",
        "opened_read_only",
    }
    assert payload["controller"]["controller_dt_s"] is None or isinstance(
        payload["controller"]["controller_dt_s"],
        float,
    )
    assert payload["state_read"]["status"] in {"ok", "not_run"}
    assert payload["timestamp_policy"]["clock"] == "host_monotonic"
    assert payload["doctor_gate"]["status"] in {"pass", "fail"}
    assert "controller_dt_measured" in payload["doctor_gate"]["checks"]
    assert "state_read_hz_measured" in payload["doctor_gate"]["checks"]
    assert payload["next_gate"] == "hold_damping_validation_before_tiny_motion"
    assert payload["artifacts"]["doctor"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_sysid_sdk_doctor_returns_blocked_for_failed_gate(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    output_artifact = tmp_path / "doctor_failed.json"

    class FailedDoctor:
        def run(self, **kwargs):
            class Result:
                def to_json(self) -> dict[str, object]:
                    return {
                        "schema": "armctrl.sysid_sdk_doctor.v1",
                        "model": "X5",
                        "interface": "can0",
                        "read_only": True,
                        "movement_allowed": False,
                        "sdk": {"module": "arx5_interface", "status": "available"},
                        "interface_status": {
                            "interface": "can0",
                            "status": "open_failed",
                            "reason": "failed to open CAN interface can0",
                        },
                        "controller": {
                            "status": "init_failed",
                            "controller_dt_s": 0.002,
                        },
                        "state_read": {
                            "status": "not_run",
                            "sample_count": 0,
                            "state_read_hz": None,
                        },
                        "timestamp_policy": {
                            "clock": "host_monotonic",
                            "monotonic": None,
                        },
                        "motion_commands_sent": False,
                        "doctor_gate": {
                            "status": "fail",
                            "checks": {
                                "interface_opened_read_only": False,
                                "controller_initialized_read_only": False,
                                "controller_dt_measured": True,
                                "state_read_ok": False,
                                "state_read_hz_measured": False,
                                "timestamps_monotonic": False,
                                "no_motion_commands_sent": True,
                            },
                            "failed_checks": [
                                "interface_opened_read_only",
                                "controller_initialized_read_only",
                                "state_read_ok",
                                "state_read_hz_measured",
                                "timestamps_monotonic",
                            ],
                        },
                        "next_gate": "hold_damping_validation_before_tiny_motion",
                    }

            return Result()

    monkeypatch.setattr(cli, "SdkDoctor", FailedDoctor)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-doctor",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "blocked"
    assert payload["schema"] == "armctrl.sysid_sdk_doctor.v1"
    assert payload["read_only"] is True
    assert payload["movement_allowed"] is False
    assert payload["motion_commands_sent"] is False
    assert payload["doctor_gate"]["status"] == "fail"
    assert payload["next_gate"] == "fix_sdk_doctor_gate_before_hold_damping"
    assert payload["artifacts"]["doctor"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_sysid_sdk_hold_damping_check_requires_confirm_and_sends_no_joint_commands(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    doctor_artifact = tmp_path / "doctor.json"
    output_artifact = tmp_path / "hold_damping.json"
    doctor_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_doctor.v1",
                "interface_status": {
                    "interface": "can0",
                    "status": "opened_read_only",
                },
                "controller": {
                    "status": "initialized_read_only",
                    "controller_dt_s": 0.002,
                },
                "state_read": {"status": "ok", "state_read_hz": 100.0},
                "timestamp_policy": {"clock": "host_monotonic", "monotonic": True},
                "motion_commands_sent": False,
            }
        ),
        encoding="utf-8",
    )

    class PassingHoldDampingCheck:
        def run(self, **kwargs):
            class Result:
                def to_json(self) -> dict[str, object]:
                    return {
                        "schema": "armctrl.sysid_sdk_hold_damping_check.v1",
                        "model": "X5",
                        "interface": "can0",
                        "movement_allowed": False,
                        "mode_change_allowed": True,
                        "requires_confirm": SDK_HOLD_DAMPING_CONFIRMATION,
                        "sdk": {
                            "module": "arx5_interface",
                            "status": "available",
                        },
                        "hold": {"status": "called", "method": "set_to_hold"},
                        "damping": {
                            "status": "called",
                            "method": "set_to_damping",
                        },
                        "joint_commands_sent": False,
                        "prerequisites": {"doctor": "pass"},
                        "hold_damping_gate": {
                            "status": "pass",
                            "checks": {
                                "hold_called": True,
                                "damping_called": True,
                                "no_joint_commands_sent": True,
                            },
                            "failed_checks": [],
                        },
                        "fault_landing_mode": "damping",
                        "next_gate": "tiny_motion_requires_separate_confirmation_and_limits",
                    }

            return Result()

    monkeypatch.setattr(cli, "SdkHoldDampingCheck", PassingHoldDampingCheck)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-hold-damping-check",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--confirm",
            SDK_HOLD_DAMPING_CONFIRMATION,
            "--doctor-artifact",
            str(doctor_artifact),
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 0
    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_sdk_hold_damping_check.v1"
    assert payload["movement_allowed"] is False
    assert payload["mode_change_allowed"] is True
    assert payload["requires_confirm"] == SDK_HOLD_DAMPING_CONFIRMATION
    assert payload["joint_commands_sent"] is False
    assert payload["fault_landing_mode"] == "damping"
    assert payload["next_gate"] == "tiny_motion_requires_separate_confirmation_and_limits"
    assert payload["prerequisites"]["doctor"] == "pass"
    assert payload["hold_damping_gate"]["status"] == "pass"
    assert payload["artifacts"]["hold_damping"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_sysid_sdk_hold_damping_check_rejects_failed_doctor_artifact(
    tmp_path: Path,
) -> None:
    doctor_artifact = tmp_path / "bad_doctor.json"
    output_artifact = tmp_path / "hold_damping_rejected.json"
    doctor_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_doctor.v1",
                "controller": {"status": "sdk_missing", "controller_dt_s": None},
                "state_read": {"status": "not_run", "state_read_hz": None},
                "timestamp_policy": {"clock": "host_monotonic", "monotonic": None},
                "motion_commands_sent": False,
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
            "sdk-hold-damping-check",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--confirm",
            SDK_HOLD_DAMPING_CONFIRMATION,
            "--doctor-artifact",
            str(doctor_artifact),
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
    assert payload["schema"] == "armctrl.sysid_sdk_hold_damping_check.v1"
    assert payload["movement_allowed"] is False
    assert payload["mode_change_allowed"] is False
    assert payload["joint_commands_sent"] is False
    assert payload["prerequisites"]["doctor"] == "fail"
    assert payload["reason"] == "sdk doctor prerequisite failed"
    assert payload["next_gate"] == "run sdk-doctor successfully before hold/damping"
    assert payload["artifacts"]["hold_damping"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_sysid_sdk_hold_damping_check_blocks_failed_gate(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    doctor_artifact = tmp_path / "doctor.json"
    output_artifact = tmp_path / "hold_damping_failed.json"
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )

    class FailedHoldDampingCheck:
        def run(self, **kwargs):
            class Result:
                def to_json(self) -> dict[str, object]:
                    return {
                        "schema": "armctrl.sysid_sdk_hold_damping_check.v1",
                        "model": "X5",
                        "interface": "can0",
                        "movement_allowed": False,
                        "mode_change_allowed": False,
                        "requires_confirm": SDK_HOLD_DAMPING_CONFIRMATION,
                        "sdk": {
                            "module": "arx5_interface",
                            "status": "available",
                        },
                        "hold": {"status": "unsupported", "method": "set_to_hold"},
                        "damping": {
                            "status": "called",
                            "method": "set_to_damping",
                        },
                        "joint_commands_sent": False,
                        "prerequisites": {"doctor": "pass"},
                        "hold_damping_gate": {
                            "status": "fail",
                            "checks": {
                                "hold_called": False,
                                "damping_called": True,
                                "no_joint_commands_sent": True,
                            },
                            "failed_checks": ["hold_called"],
                        },
                        "fault_landing_mode": "damping",
                        "next_gate": "tiny_motion_requires_separate_confirmation_and_limits",
                    }

            return Result()

    monkeypatch.setattr(cli, "SdkHoldDampingCheck", FailedHoldDampingCheck)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-hold-damping-check",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--confirm",
            SDK_HOLD_DAMPING_CONFIRMATION,
            "--doctor-artifact",
            str(doctor_artifact),
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "blocked"
    assert payload["schema"] == "armctrl.sysid_sdk_hold_damping_check.v1"
    assert payload["movement_allowed"] is False
    assert payload["mode_change_allowed"] is False
    assert payload["joint_commands_sent"] is False
    assert payload["prerequisites"]["doctor"] == "pass"
    assert payload["hold_damping_gate"]["status"] == "fail"
    assert payload["next_gate"] == "fix_hold_damping_gate_before_tiny_motion"
    assert payload["artifacts"]["hold_damping"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_sysid_sdk_hold_damping_check_rejects_unopened_doctor_interface(
    tmp_path: Path,
) -> None:
    doctor_artifact = tmp_path / "unopened_doctor.json"
    output_artifact = tmp_path / "hold_damping_unopened_rejected.json"
    doctor_payload = _passing_doctor_artifact()
    doctor_payload["interface_status"] = {
        "interface": "can0",
        "status": "not_checked",
        "reason": "sdk_missing",
    }
    doctor_artifact.write_text(json.dumps(doctor_payload), encoding="utf-8")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "sdk-hold-damping-check",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--confirm",
            SDK_HOLD_DAMPING_CONFIRMATION,
            "--doctor-artifact",
            str(doctor_artifact),
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
    assert payload["prerequisites"]["doctor"] == "fail"
    assert payload["reason"] == "sdk doctor prerequisite failed"
    assert payload["artifacts"]["hold_damping"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_sdk_tiny_motion_plan_requires_gates_confirm_and_small_delta() -> None:
    doctor_artifact = {
        "schema": "armctrl.sysid_sdk_doctor.v1",
        "interface_status": {"interface": "can0", "status": "opened_read_only"},
        "controller": {"status": "initialized_read_only", "controller_dt_s": 0.002},
        "state_read": {"status": "ok", "state_read_hz": 100.0},
        "timestamp_policy": {"clock": "host_monotonic", "monotonic": True},
        "motion_commands_sent": False,
    }
    hold_damping_artifact = {
        "schema": "armctrl.sysid_sdk_hold_damping_check.v1",
        "hold": {"status": "called"},
        "damping": {"status": "called"},
        "mode_call_sequence": [
            {"name": "hold", "method": "set_to_hold", "status": "called"},
            {"name": "damping", "method": "set_to_damping", "status": "called"},
        ],
        "joint_commands_sent": False,
    }
    planner = SdkTinyMotionPlanner(max_delta_rad=0.005)

    with pytest.raises(PermissionError):
        planner.plan(
            doctor_artifact=doctor_artifact,
            hold_damping_artifact=hold_damping_artifact,
            joint_index=2,
            delta_rad=0.002,
            confirm="nope",
        )

    with pytest.raises(ValueError, match="delta_rad"):
        planner.plan(
            doctor_artifact=doctor_artifact,
            hold_damping_artifact=hold_damping_artifact,
            joint_index=2,
            delta_rad=0.02,
            confirm=SDK_TINY_MOTION_CONFIRMATION,
        )

    result = planner.plan(
        doctor_artifact=doctor_artifact,
        hold_damping_artifact=hold_damping_artifact,
        joint_index=2,
        delta_rad=0.002,
        confirm=SDK_TINY_MOTION_CONFIRMATION,
    )

    payload = result.to_json()

    assert payload["schema"] == "armctrl.sysid_sdk_tiny_motion_plan.v1"
    assert payload["movement_allowed"] is False
    assert payload["tiny_motion_eligible"] is True
    assert payload["requires_confirm"] == SDK_TINY_MOTION_CONFIRMATION
    assert payload["command"] == {"joint_index": 2, "delta_rad": 0.002}
    assert payload["limits"]["max_delta_rad"] == 0.005
    assert payload["prerequisites"] == {
        "doctor": "pass",
        "hold_damping": "pass",
    }
    assert payload["next_gate"] == "tiny_motion_execute_on_target_with_runtime_logs"


def test_cli_sysid_sdk_tiny_motion_plan_reads_gate_artifacts(tmp_path: Path) -> None:
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    output_artifact = tmp_path / "tiny_motion_plan.json"
    doctor_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_doctor.v1",
                "interface_status": {
                    "interface": "can0",
                    "status": "opened_read_only",
                },
                "controller": {
                    "status": "initialized_read_only",
                    "controller_dt_s": 0.002,
                },
                "state_read": {"status": "ok", "state_read_hz": 100.0},
                "timestamp_policy": {"clock": "host_monotonic", "monotonic": True},
                "motion_commands_sent": False,
            }
        ),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_hold_damping_check.v1",
                "hold": {"status": "called"},
                "damping": {"status": "called"},
                "mode_call_sequence": [
                    {"name": "hold", "method": "set_to_hold", "status": "called"},
                    {
                        "name": "damping",
                        "method": "set_to_damping",
                        "status": "called",
                    },
                ],
                "joint_commands_sent": False,
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
            "sdk-tiny-motion-plan",
            "--doctor-artifact",
            str(doctor_artifact),
            "--hold-damping-artifact",
            str(hold_damping_artifact),
            "--joint-index",
            "2",
            "--delta-rad",
            "0.002",
            "--confirm",
            SDK_TINY_MOTION_CONFIRMATION,
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
    assert payload["schema"] == "armctrl.sysid_sdk_tiny_motion_plan.v1"
    assert payload["movement_allowed"] is False
    assert payload["tiny_motion_eligible"] is True
    assert payload["command"] == {"joint_index": 2, "delta_rad": 0.002}
    assert payload["artifacts"]["tiny_motion_plan"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_sdk_tiny_motion_executor_runs_plan_on_fake_backend_with_runtime_log() -> None:
    clock = ManualClock()
    plan_artifact = {
        "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
        "tiny_motion_eligible": True,
        "command": {"joint_index": 2, "delta_rad": 0.002},
        "limits": {"max_delta_rad": 0.005},
    }
    executor = SdkTinyMotionExecutor(
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    with pytest.raises(PermissionError):
        executor.execute(
            plan_artifact=plan_artifact,
            backend_name="fake",
            dof=6,
            q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
            confirm="nope",
        )

    result = executor.execute(
        plan_artifact=plan_artifact,
        backend_name="fake",
        dof=6,
        q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        confirm=SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
        send_hz=50.0,
    )

    payload = result.to_json()

    assert payload["schema"] == "armctrl.sysid_sdk_tiny_motion_execute.v1"
    assert payload["backend"] == "fake"
    assert payload["hardware_motion"] is False
    assert payload["movement_command_sent"] is True
    assert payload["q_start"] == [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    assert payload["q_target"] == [0.0, 0.302, 0.3, 0.0, 0.0, 0.0]
    assert payload["motion_runtime"]["actual_send_hz"] == pytest.approx(50.0)
    assert payload["motion_runtime"]["sample_count"] == 2
    assert payload["motion_runtime"]["landing_mode"] == "hold"
    assert payload["tracking"] == {
        "q_cmd_delta_rad": [0.0, 0.0020000000000000018, 0.0, 0.0, 0.0, 0.0],
        "q_meas_delta_rad": [0.0, 0.0020000000000000018, 0.0, 0.0, 0.0, 0.0],
        "q_cmd_delta_max_abs_rad": 0.0020000000000000018,
        "q_meas_delta_max_abs_rad": 0.0020000000000000018,
        "final_tracking_error_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "final_tracking_error_max_abs_rad": 0.0,
    }
    assert payload["motion_runtime"]["samples"][1]["q_cmd"] == [
        0.0,
        0.302,
        0.3,
        0.0,
        0.0,
        0.0,
    ]
    assert payload["motion_runtime"]["samples"][1]["q_meas"] == [
        0.0,
        0.302,
        0.3,
        0.0,
        0.0,
        0.0,
    ]
    assert len(executor.fake_backend.joint_commands) == 2


def test_sdk_tiny_motion_executor_reports_faulted_fake_runtime_log() -> None:
    clock = ManualClock()
    plan_artifact = {
        "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
        "tiny_motion_eligible": True,
        "command": {"joint_index": 2, "delta_rad": 0.002},
        "limits": {"max_delta_rad": 0.005},
    }
    executor = SdkTinyMotionExecutor(
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    executor.fake_backend.fault_flags = ("over_current",)

    result = executor.execute(
        plan_artifact=plan_artifact,
        backend_name="fake",
        dof=6,
        q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        confirm=SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
        send_hz=50.0,
    )

    payload = result.to_json()

    assert payload["schema"] == "armctrl.sysid_sdk_tiny_motion_execute.v1"
    assert payload["backend"] == "fake"
    assert payload["hardware_motion"] is False
    assert payload["movement_command_sent"] is True
    assert payload["run_status"] == "faulted"
    assert payload["motion_runtime"]["status"] == "faulted"
    assert payload["motion_runtime"]["sample_count"] == 1
    assert payload["motion_runtime"]["fault_flags"] == ["over_current"]
    assert payload["motion_runtime"]["landing_mode"] == "damping"
    assert payload["acceptance"]["schema"] == "armctrl.real_motion_acceptance.v1"
    assert payload["acceptance"]["stage"] == "tiny_motion"
    assert payload["acceptance"]["status"] == "review_required"
    assert payload["acceptance"]["checks"]["runtime_completed"]["status"] == "fail"
    assert payload["acceptance"]["checks"]["no_fault_flags"] == {
        "status": "fail",
        "fault_flags": ["over_current"],
    }
    assert payload["fault_landing_mode"] == "damping"
    assert executor.fake_backend.hold_count == 0
    assert executor.fake_backend.damping_count == 1


def test_sdk_tiny_motion_executor_reports_aborted_runtime_error_with_partial_log() -> None:
    clock = ManualClock()
    plan_artifact = {
        "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
        "tiny_motion_eligible": True,
        "command": {"joint_index": 2, "delta_rad": 0.002},
        "limits": {"max_delta_rad": 0.005},
    }
    executor = SdkTinyMotionExecutor(
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    executor.fake_backend = FailingTinyMotionBackend(fail_on_send=2)

    result = executor.execute(
        plan_artifact=plan_artifact,
        backend_name="fake",
        dof=6,
        q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        confirm=SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
        send_hz=50.0,
    )

    payload = result.to_json()

    assert payload["run_status"] == "aborted"
    assert payload["hardware_motion"] is False
    assert payload["movement_command_sent"] is True
    assert payload["motion_runtime"]["status"] == "aborted"
    assert payload["motion_runtime"]["landing_mode"] == "damping"
    assert payload["motion_runtime"]["sample_count"] == 1
    assert payload["motion_runtime"]["error"] == {
        "type": "RuntimeError",
        "message": "sdk send failed",
    }
    assert payload["motion_runtime"]["samples"][0]["q_cmd"] == [
        0.0,
        0.3,
        0.3,
        0.0,
        0.0,
        0.0,
    ]
    assert payload["acceptance"]["status"] == "review_required"
    assert payload["acceptance"]["checks"]["runtime_completed"]["status"] == "fail"
    assert payload["fault_landing_mode"] == "damping"
    assert executor.fake_backend.hold_count == 0
    assert executor.fake_backend.damping_count == 1


def test_sdk_tiny_motion_executor_runs_plan_on_arx5_backend_with_fake_module() -> None:
    clock = ManualClock()
    FakeDoctorController.damping_count = 0
    FakeDoctorController.hold_count = 0
    FakeDoctorController.joint_command_count = 0
    FakeDoctorController.initial_pos = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    plan_artifact = {
        "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
        "tiny_motion_eligible": True,
        "command": {"joint_index": 2, "delta_rad": 0.002},
        "limits": {"max_delta_rad": 0.005},
    }
    executor = SdkTinyMotionExecutor(
        arx5_module=FakeDoctorArx5Module,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    with pytest.raises(RuntimeError, match="tiny motion execution prerequisites failed"):
        executor.execute(
            plan_artifact=plan_artifact,
            backend_name="arx5_sdk",
            model="X5",
            interface="can0",
            dof=6,
            q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
            confirm=SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
            send_hz=50.0,
        )

    result = executor.execute(
        plan_artifact=plan_artifact,
        backend_name="arx5_sdk",
        model="X5",
        interface="can0",
        dof=6,
        q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        confirm=SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
        send_hz=50.0,
        doctor_artifact=_passing_doctor_artifact(),
        hold_damping_artifact=_passing_hold_damping_artifact(),
    )

    payload = result.to_json()

    assert payload["backend"] == "arx5_sdk"
    assert payload["hardware_motion"] is True
    assert payload["prerequisites"] == {
        "doctor": "pass",
        "hold_damping": "pass",
    }
    assert payload["q_target"] == [0.0, 0.302, 0.3, 0.0, 0.0, 0.0]
    assert payload["motion_runtime"]["actual_send_hz"] == pytest.approx(50.0)
    assert payload["motion_runtime"]["controller_dt_s"] == 0.002
    assert payload["motion_runtime"]["sample_count"] == 2
    assert payload["motion_runtime"]["landing_mode"] == "hold"
    assert payload["acceptance"]["schema"] == "armctrl.real_motion_acceptance.v1"
    assert payload["acceptance"]["stage"] == "tiny_motion"
    assert payload["acceptance"]["status"] == "pass"
    assert payload["acceptance"]["next_gate"] == "agent_sysid_smoke_readiness"
    assert payload["acceptance"]["checks"]["readiness_gate"]["status"] == "pass"
    assert payload["acceptance"]["checks"]["q_meas_responded_to_commanded_motion"][
        "status"
    ] == "pass"
    assert FakeDoctorArx5Module.last_controller is not None
    assert FakeDoctorArx5Module.last_controller.commands[-1] == [
        0.0,
        0.302,
        0.3,
        0.0,
        0.0,
        0.0,
    ]
    assert FakeDoctorController.joint_command_count == 3
    assert FakeDoctorController.hold_count == 1
    FakeDoctorController.initial_pos = None


def test_tiny_motion_execute_real_rejects_operator_q_current_mismatch_before_send() -> None:
    clock = ManualClock()
    FakeDoctorController.damping_count = 0
    FakeDoctorController.hold_count = 0
    FakeDoctorController.joint_command_count = 0
    FakeDoctorController.initial_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    plan_artifact = {
        "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
        "tiny_motion_eligible": True,
        "command": {"joint_index": 2, "delta_rad": 0.002},
        "limits": {"max_delta_rad": 0.005},
    }
    executor = SdkTinyMotionExecutor(
        arx5_module=FakeDoctorArx5Module,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    with pytest.raises(RuntimeError, match="q_current does not match measured SDK state"):
        executor.execute(
            plan_artifact=plan_artifact,
            backend_name="arx5_sdk",
            model="X5",
            interface="can0",
            dof=6,
            q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
            confirm=SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
            send_hz=50.0,
            doctor_artifact=_passing_doctor_artifact(),
            hold_damping_artifact=_passing_hold_damping_artifact(),
        )

    assert FakeDoctorController.joint_command_count == 0
    assert FakeDoctorController.damping_count == 1
    FakeDoctorController.initial_pos = None


def test_tiny_motion_execute_real_keeps_active_hold_when_sdk_hold_is_unavailable() -> None:
    clock = ManualClock()
    DampingOnlyController.damping_count = 0
    DampingOnlyController.joint_command_count = 0
    DampingOnlyController.initial_pos = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    plan_artifact = {
        "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
        "tiny_motion_eligible": True,
        "command": {"joint_index": 2, "delta_rad": 0.002},
        "limits": {"max_delta_rad": 0.005},
    }
    executor = SdkTinyMotionExecutor(
        arx5_module=DampingOnlyArx5Module,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    result = executor.execute(
        plan_artifact=plan_artifact,
        backend_name="arx5_sdk",
        model="X5",
        interface="can0",
        dof=6,
        q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        confirm=SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
        send_hz=50.0,
        doctor_artifact=_passing_doctor_artifact(),
        hold_damping_artifact=_passing_damping_only_artifact(),
    )

    payload = result.to_json()

    assert payload["motion_runtime"]["landing_mode"] == "hold"
    assert payload["fault_landing_mode"] == "damping"
    assert payload["acceptance"]["status"] == "pass"
    assert DampingOnlyController.damping_count == 0
    assert DampingOnlyController.joint_command_count == 3
    DampingOnlyController.initial_pos = None


def test_cli_sysid_sdk_tiny_motion_execute_fake_reads_plan_artifact(
    tmp_path: Path,
) -> None:
    plan_artifact = tmp_path / "tiny_plan.json"
    output_artifact = tmp_path / "tiny_execute.json"
    plan_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
                "tiny_motion_eligible": True,
                "command": {"joint_index": 2, "delta_rad": 0.002},
                "limits": {"max_delta_rad": 0.005},
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
            "sdk-tiny-motion-execute-fake",
            "--plan-artifact",
            str(plan_artifact),
            "--dof",
            "6",
            "--q-current",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--confirm",
            SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
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
    assert payload["schema"] == "armctrl.sysid_sdk_tiny_motion_execute.v1"
    assert payload["backend"] == "fake"
    assert payload["hardware_motion"] is False
    assert payload["q_target"] == [0.0, 0.302, 0.3, 0.0, 0.0, 0.0]
    assert payload["motion_runtime"]["trajectory_sample_hz"] == 50.0
    assert payload["motion_runtime"]["landing_mode"] == "hold"
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload
    assert saved_payload["motion_runtime"]["samples"][1]["q_cmd"] == [
        0.0,
        0.302,
        0.3,
        0.0,
        0.0,
        0.0,
    ]


def test_cli_sysid_sdk_tiny_motion_execute_real_routes_to_arx5_backend(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    plan_artifact = tmp_path / "tiny_plan.json"
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    plan_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
                "tiny_motion_eligible": True,
                "command": {"joint_index": 2, "delta_rad": 0.002},
                "limits": {"max_delta_rad": 0.005},
            }
        ),
        encoding="utf-8",
    )
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(_passing_hold_damping_artifact()),
        encoding="utf-8",
    )
    calls: list[dict[str, object]] = []

    class StubTinyMotionResult:
        def to_json(self) -> dict[str, object]:
            return {
                "schema": "armctrl.sysid_sdk_tiny_motion_execute.v1",
                "backend": "arx5_sdk",
                "hardware_motion": True,
                "movement_command_sent": True,
                "q_start": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
                "q_target": [0.0, 0.302, 0.3, 0.0, 0.0, 0.0],
                "motion_runtime": {"landing_mode": "hold"},
            }

    class StubTinyMotionExecutor:
        def execute(self, **kwargs):
            calls.append(kwargs)
            return StubTinyMotionResult()

    monkeypatch.setattr(cli, "SdkTinyMotionExecutor", StubTinyMotionExecutor)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-tiny-motion-execute-real",
            "--plan-artifact",
            str(plan_artifact),
            "--doctor-artifact",
            str(doctor_artifact),
            "--hold-damping-artifact",
            str(hold_damping_artifact),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--dof",
            "6",
            "--q-current",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--send-hz",
            "50",
            "--confirm",
            SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 0
    assert calls == [
        {
            "plan_artifact": json.loads(plan_artifact.read_text(encoding="utf-8")),
            "doctor_artifact": json.loads(doctor_artifact.read_text(encoding="utf-8")),
            "hold_damping_artifact": json.loads(
                hold_damping_artifact.read_text(encoding="utf-8")
            ),
            "backend_name": "arx5_sdk",
            "model": "X5",
            "interface": "can0",
            "dof": 6,
            "q_current": (0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
            "confirm": SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
            "send_hz": 50.0,
            "max_q_current_error_rad": 0.02,
        }
    ]
    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_sdk_tiny_motion_execute.v1"
    assert payload["backend"] == "arx5_sdk"
    assert payload["hardware_motion"] is True


def test_cli_sysid_sdk_tiny_motion_execute_real_rejects_q_current_mismatch(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    plan_artifact = tmp_path / "tiny_plan.json"
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    output_artifact = tmp_path / "tiny_execute_q_mismatch.json"
    plan_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
                "tiny_motion_eligible": True,
                "command": {"joint_index": 2, "delta_rad": 0.002},
                "limits": {"max_delta_rad": 0.005},
            }
        ),
        encoding="utf-8",
    )
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(_passing_hold_damping_artifact()),
        encoding="utf-8",
    )

    class MismatchTinyMotionExecutor:
        def __init__(self, *args, **kwargs) -> None:
            pass

        def execute(self, **kwargs):
            raise SdkMeasuredStateMismatchError(
                operator_q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
                measured_q_current=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                max_error_rad=0.02,
            )

    monkeypatch.setattr(cli, "SdkTinyMotionExecutor", MismatchTinyMotionExecutor)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-tiny-motion-execute-real",
            "--plan-artifact",
            str(plan_artifact),
            "--doctor-artifact",
            str(doctor_artifact),
            "--hold-damping-artifact",
            str(hold_damping_artifact),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--dof",
            "6",
            "--q-current",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--confirm",
            SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "rejected"
    assert payload["movement_command_sent"] is False
    assert payload["measured_q_current"] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert payload["operator_q_current"] == [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    assert payload["q_current_error_max_abs_rad"] == 0.3
    assert payload["q_current_max_error_rad"] == 0.02
    assert payload["fault_landing_mode"] == "damping"
    assert payload["next_gate"] == (
        "plan an explicit measured-state recovery before retrying tiny motion"
    )
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_sysid_sdk_tiny_motion_execute_real_returns_nonzero_for_faulted_result(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    plan_artifact = tmp_path / "tiny_plan.json"
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    output_artifact = tmp_path / "tiny_execute_faulted.json"
    plan_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
                "tiny_motion_eligible": True,
                "command": {"joint_index": 2, "delta_rad": 0.002},
                "limits": {"max_delta_rad": 0.005},
            }
        ),
        encoding="utf-8",
    )
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(_passing_hold_damping_artifact()),
        encoding="utf-8",
    )

    class FaultedTinyMotionResult:
        def to_json(self) -> dict[str, object]:
            return {
                "schema": "armctrl.sysid_sdk_tiny_motion_execute.v1",
                "backend": "arx5_sdk",
                "run_status": "faulted",
                "hardware_motion": True,
                "movement_command_sent": True,
                "prerequisites": {"doctor": "pass", "hold_damping": "pass"},
                "q_start": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
                "q_target": [0.0, 0.302, 0.3, 0.0, 0.0, 0.0],
                "motion_runtime": {
                    "schema": "armctrl.motion_runtime_result.v1",
                    "status": "faulted",
                    "producer": "tiny_motion",
                    "mode": "trajectory_replay",
                    "trajectory_sample_hz": 50.0,
                    "actual_send_hz": 50.0,
                    "send_jitter_ms_p95": 0.0,
                    "send_jitter_ms_p99": 0.0,
                    "controller_dt_s": 0.002,
                    "sample_count": 1,
                    "fault_flags": ["over_current"],
                    "tracking": {
                        "q_cmd_delta_max_abs_rad": None,
                        "q_meas_delta_max_abs_rad": None,
                        "final_tracking_error_max_abs_rad": 0.0,
                    },
                    "samples": [
                        {
                            "sent_monotonic_s": 0.0,
                            "q_cmd": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
                            "q_meas": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
                            "dq_meas": [],
                            "tau_meas": [],
                            "fault_flags": ["over_current"],
                            "producer": "tiny_motion",
                            "mode": "trajectory_replay",
                        }
                    ],
                    "landing_mode": "damping",
                },
                "tracking": {
                    "q_cmd_delta_max_abs_rad": None,
                    "q_meas_delta_max_abs_rad": None,
                    "final_tracking_error_max_abs_rad": 0.0,
                },
                "acceptance": {
                    "schema": "armctrl.real_motion_acceptance.v1",
                    "stage": "tiny_motion",
                    "status": "review_required",
                },
                "fault_landing_mode": "damping",
            }

    class FaultedTinyMotionExecutor:
        def execute(self, **kwargs):
            return FaultedTinyMotionResult()

    monkeypatch.setattr(cli, "SdkTinyMotionExecutor", FaultedTinyMotionExecutor)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-tiny-motion-execute-real",
            "--plan-artifact",
            str(plan_artifact),
            "--doctor-artifact",
            str(doctor_artifact),
            "--hold-damping-artifact",
            str(hold_damping_artifact),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--dof",
            "6",
            "--q-current",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--confirm",
            SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "faulted"
    assert payload["run_status"] == "faulted"
    assert payload["hardware_motion"] is True
    assert payload["movement_command_sent"] is True
    assert payload["motion_runtime"]["status"] == "faulted"
    assert payload["motion_runtime"]["landing_mode"] == "damping"
    assert payload["acceptance"]["status"] == "review_required"
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_sysid_sdk_tiny_motion_execute_real_returns_nonzero_for_failed_acceptance(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    plan_artifact = tmp_path / "tiny_plan.json"
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    output_artifact = tmp_path / "tiny_execute_incomplete.json"
    plan_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
                "tiny_motion_eligible": True,
                "command": {"joint_index": 2, "delta_rad": 0.002},
                "limits": {"max_delta_rad": 0.005},
            }
        ),
        encoding="utf-8",
    )
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(_passing_hold_damping_artifact()),
        encoding="utf-8",
    )

    class StubTinyMotionResult:
        def to_json(self) -> dict[str, object]:
            return {
                "schema": "armctrl.sysid_sdk_tiny_motion_execute.v1",
                "run_status": "completed",
                "backend": "arx5_sdk",
                "hardware_motion": True,
                "movement_command_sent": True,
                "motion_runtime": {
                    "status": "completed",
                    "landing_mode": "hold",
                    "actual_send_hz": None,
                },
                "acceptance": {
                    "schema": "armctrl.real_motion_acceptance.v1",
                    "stage": "tiny_motion",
                    "status": "incomplete",
                    "next_gate": "inspect_acceptance_checks_before_next_hardware_gate",
                },
            }

    class StubTinyMotionExecutor:
        def execute(self, **kwargs):
            return StubTinyMotionResult()

    monkeypatch.setattr(cli, "SdkTinyMotionExecutor", StubTinyMotionExecutor)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-tiny-motion-execute-real",
            "--plan-artifact",
            str(plan_artifact),
            "--doctor-artifact",
            str(doctor_artifact),
            "--hold-damping-artifact",
            str(hold_damping_artifact),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--dof",
            "6",
            "--q-current",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--send-hz",
            "50",
            "--confirm",
            SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "incomplete"
    assert payload["run_status"] == "completed"
    assert payload["acceptance"]["status"] == "incomplete"
    assert payload["acceptance"]["next_gate"] == (
        "inspect_acceptance_checks_before_next_hardware_gate"
    )
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_sysid_sdk_tiny_motion_execute_real_rejects_missing_sdk(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    plan_artifact = tmp_path / "tiny_plan.json"
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    output_artifact = tmp_path / "tiny_execute_rejected.json"
    plan_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
                "tiny_motion_eligible": True,
                "command": {"joint_index": 2, "delta_rad": 0.002},
                "limits": {"max_delta_rad": 0.005},
            }
        ),
        encoding="utf-8",
    )
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(_passing_hold_damping_artifact()),
        encoding="utf-8",
    )

    class MissingSdkExecutor:
        def execute(self, **kwargs):
            raise ModuleNotFoundError("arx5_interface")

    monkeypatch.setattr(cli, "SdkTinyMotionExecutor", MissingSdkExecutor)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-tiny-motion-execute-real",
            "--plan-artifact",
            str(plan_artifact),
            "--doctor-artifact",
            str(doctor_artifact),
            "--hold-damping-artifact",
            str(hold_damping_artifact),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--dof",
            "6",
            "--q-current",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--confirm",
            SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.sysid_sdk_tiny_motion_execute.v1"
    assert payload["backend"] == "arx5_sdk"
    assert payload["hardware_motion"] is False
    assert payload["movement_command_sent"] is False
    assert payload["reason"] == "arx5_interface is not importable in this environment"
    assert payload["fault_landing_mode"] == "damping"
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_sysid_sdk_tiny_motion_execute_real_writes_rejected_artifact_on_runtime_failure(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    plan_artifact = tmp_path / "tiny_plan.json"
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    output_artifact = tmp_path / "tiny_execute_runtime_failed.json"
    plan_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
                "tiny_motion_eligible": True,
                "command": {"joint_index": 2, "delta_rad": 0.002},
                "limits": {"max_delta_rad": 0.005},
            }
        ),
        encoding="utf-8",
    )
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(_passing_hold_damping_artifact()),
        encoding="utf-8",
    )

    class FailingTinyMotionExecutor:
        def execute(self, **kwargs):
            raise RuntimeError("sdk send failed during tiny motion")

    monkeypatch.setattr(cli, "SdkTinyMotionExecutor", FailingTinyMotionExecutor)

    exit_code = cli.main(
        [
            "sysid",
            "sdk-tiny-motion-execute-real",
            "--plan-artifact",
            str(plan_artifact),
            "--doctor-artifact",
            str(doctor_artifact),
            "--hold-damping-artifact",
            str(hold_damping_artifact),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--dof",
            "6",
            "--q-current",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--confirm",
            SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
            "--output",
            str(output_artifact),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.sysid_sdk_tiny_motion_execute.v1"
    assert payload["backend"] == "arx5_sdk"
    assert payload["hardware_motion"] == "unknown"
    assert payload["movement_command_sent"] == "unknown"
    assert payload["prerequisites"] == {
        "doctor": "pass",
        "hold_damping": "pass",
    }
    assert payload["reason"] == "tiny motion runtime failed: sdk send failed during tiny motion"
    assert payload["fault_landing_mode"] == "damping"
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_sysid_sdk_tiny_motion_execute_real_rejects_failed_hold_artifact(
    tmp_path: Path,
) -> None:
    plan_artifact = tmp_path / "tiny_plan.json"
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "bad_hold_damping.json"
    output_artifact = tmp_path / "tiny_execute_prereq_rejected.json"
    plan_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_tiny_motion_plan.v1",
                "tiny_motion_eligible": True,
                "command": {"joint_index": 2, "delta_rad": 0.002},
                "limits": {"max_delta_rad": 0.005},
            }
        ),
        encoding="utf-8",
    )
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_sdk_hold_damping_check.v1",
                "hold": {"status": "not_run"},
                "damping": {"status": "not_run"},
                "joint_commands_sent": False,
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
            "sdk-tiny-motion-execute-real",
            "--plan-artifact",
            str(plan_artifact),
            "--doctor-artifact",
            str(doctor_artifact),
            "--hold-damping-artifact",
            str(hold_damping_artifact),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--dof",
            "6",
            "--q-current",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--confirm",
            SDK_TINY_MOTION_EXECUTE_CONFIRMATION,
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
    assert payload["schema"] == "armctrl.sysid_sdk_tiny_motion_execute.v1"
    assert payload["backend"] == "arx5_sdk"
    assert payload["hardware_motion"] is False
    assert payload["movement_command_sent"] is False
    assert payload["prerequisites"] == {
        "doctor": "pass",
        "hold_damping": "fail",
    }
    assert payload["reason"] == "tiny motion execution prerequisites failed: hold_damping"
    assert payload["fault_landing_mode"] == "damping"
    assert payload["artifacts"]["runtime_log"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_sdk_agent_sysid_smoke_readiness_requires_real_tiny_motion_success() -> None:
    checker = SdkAgentSysIdSmokeReadinessChecker()

    passing = checker.check(
        doctor_artifact=_passing_doctor_artifact(),
        hold_damping_artifact=_passing_hold_damping_artifact(),
        tiny_motion_artifact=_passing_tiny_motion_execute_artifact(),
    ).to_json()

    assert passing["schema"] == "armctrl.sysid_agent_smoke_readiness.v1"
    assert passing["read_only"] is True
    assert passing["movement_allowed"] is False
    assert passing["agent_sysid_smoke_allowed"] is True
    assert passing["prerequisites"] == {
        "doctor": "pass",
        "hold_damping": "pass",
        "tiny_motion": "pass",
    }
    assert passing["tiny_motion"]["tracking"]["q_cmd_delta_max_abs_rad"] == pytest.approx(
        0.002
    )
    assert passing["tiny_motion"]["tracking"]["q_meas_delta_max_abs_rad"] == pytest.approx(
        0.002
    )
    assert passing["tiny_motion"]["tracking"]["final_tracking_error_max_abs_rad"] == 0.0
    assert passing["next_gate"] == "agent_smoke_then_sysid_smoke_on_target"

    fake_tiny_motion = _passing_tiny_motion_execute_artifact()
    fake_tiny_motion["backend"] = "fake"
    fake_tiny_motion["hardware_motion"] = False
    rejected = checker.check(
        doctor_artifact=_passing_doctor_artifact(),
        hold_damping_artifact=_passing_hold_damping_artifact(),
        tiny_motion_artifact=fake_tiny_motion,
    ).to_json()

    assert rejected["agent_sysid_smoke_allowed"] is False
    assert rejected["prerequisites"]["tiny_motion"] == "fail"
    assert (
        rejected["next_gate"]
        == "start live ArmRuntime hold session before Agent/SysID smoke"
    )

    failed_acceptance_tiny_motion = _passing_tiny_motion_execute_artifact()
    failed_acceptance_tiny_motion["acceptance"] = {
        "schema": "armctrl.real_motion_acceptance.v1",
        "stage": "tiny_motion",
        "status": "review_required",
        "checks": {
            "no_fault_flags": {
                "status": "fail",
                "fault_flags": ["over_current"],
            }
        },
        "next_gate": "inspect_acceptance_checks_before_next_hardware_gate",
    }
    failed_acceptance = checker.check(
        doctor_artifact=_passing_doctor_artifact(),
        hold_damping_artifact=_passing_hold_damping_artifact(),
        tiny_motion_artifact=failed_acceptance_tiny_motion,
    ).to_json()

    assert failed_acceptance["agent_sysid_smoke_allowed"] is False
    assert failed_acceptance["prerequisites"]["tiny_motion"] == "fail"
    assert failed_acceptance["tiny_motion"]["acceptance"]["status"] == "review_required"

    stalled_tiny_motion = _passing_tiny_motion_execute_artifact()
    stalled_tiny_motion["motion_runtime"]["samples"][-1]["q_meas"] = [0.0, 0.3]
    stalled = checker.check(
        doctor_artifact=_passing_doctor_artifact(),
        hold_damping_artifact=_passing_hold_damping_artifact(),
        tiny_motion_artifact=stalled_tiny_motion,
    ).to_json()

    assert stalled["agent_sysid_smoke_allowed"] is False
    assert stalled["prerequisites"]["tiny_motion"] == "fail"
    assert stalled["tiny_motion"]["tracking"]["q_cmd_delta_max_abs_rad"] == pytest.approx(
        0.002
    )
    assert stalled["tiny_motion"]["tracking"]["q_meas_delta_max_abs_rad"] == 0.0


def test_sdk_agent_sysid_smoke_readiness_accepts_startup_recovery_success() -> None:
    checker = SdkAgentSysIdSmokeReadinessChecker()

    passing = checker.check(
        doctor_artifact=_passing_doctor_artifact(),
        hold_damping_artifact=_passing_hold_damping_artifact(),
        startup_recovery_artifact=_passing_startup_recovery_artifact(),
    ).to_json()

    assert passing["schema"] == "armctrl.sysid_agent_smoke_readiness.v1"
    assert passing["agent_sysid_smoke_allowed"] is True
    assert passing["prerequisites"] == {
        "doctor": "pass",
        "hold_damping": "pass",
        "startup_recovery": "pass",
    }
    assert passing["startup_recovery"]["run_status"] == "completed"
    assert passing["next_gate"] == "agent_smoke_then_sysid_smoke_on_target"


def test_cli_sysid_agent_smoke_readiness_rejects_tiny_motion_artifact(
    tmp_path: Path,
) -> None:
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    tiny_motion_artifact = tmp_path / "tiny_motion_execute.json"
    runtime_status_artifact = tmp_path / "runtime_status.json"
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(_passing_hold_damping_artifact()),
        encoding="utf-8",
    )
    tiny_motion_artifact.write_text(
        json.dumps(_passing_tiny_motion_execute_artifact()),
        encoding="utf-8",
    )
    runtime_status_artifact.write_text(
        json.dumps(_passing_runtime_status_artifact()),
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
            "--tiny-motion-artifact",
            str(tiny_motion_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert completed.stdout == ""
    assert "unrecognized arguments: --tiny-motion-artifact" in completed.stderr


def test_cli_sysid_agent_smoke_readiness_rejects_startup_recovery_artifact(
    tmp_path: Path,
) -> None:
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    startup_recovery_artifact = tmp_path / "startup_recovery.json"
    runtime_status_artifact = tmp_path / "runtime_status.json"
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(_passing_hold_damping_artifact()),
        encoding="utf-8",
    )
    startup_recovery_artifact.write_text(
        json.dumps(_passing_startup_recovery_artifact()),
        encoding="utf-8",
    )
    runtime_status_artifact.write_text(
        json.dumps(_passing_runtime_status_artifact()),
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
            "--startup-recovery-artifact",
            str(startup_recovery_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert completed.stdout == ""
    assert "unrecognized arguments: --startup-recovery-artifact" in completed.stderr


def test_cli_sysid_agent_smoke_readiness_requires_runtime_status_artifact(
    tmp_path: Path,
) -> None:
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(_passing_hold_damping_artifact()),
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
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert completed.stdout == ""
    assert "the following arguments are required: --runtime-status-artifact" in completed.stderr


def test_cli_sysid_agent_smoke_readiness_accepts_live_runtime_status(
    tmp_path: Path,
) -> None:
    doctor_artifact = tmp_path / "doctor.json"
    hold_damping_artifact = tmp_path / "hold_damping.json"
    runtime_status_artifact = tmp_path / "runtime_status.json"
    output_artifact = tmp_path / "agent_sysid_readiness.json"
    doctor_artifact.write_text(
        json.dumps(_passing_doctor_artifact()),
        encoding="utf-8",
    )
    hold_damping_artifact.write_text(
        json.dumps(_passing_hold_damping_artifact()),
        encoding="utf-8",
    )
    runtime_status_artifact.write_text(
        json.dumps(_passing_runtime_status_artifact()),
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
    assert payload["schema"] == "armctrl.sysid_agent_smoke_readiness.v1"
    assert payload["agent_sysid_smoke_allowed"] is True
    assert payload["prerequisites"]["runtime_status"] == "pass"
    assert payload["runtime_status"]["readiness"]["agent_sysid_smoke_allowed"] is True
    assert payload["artifacts"]["readiness"] == str(output_artifact)

    saved_payload = json.loads(output_artifact.read_text(encoding="utf-8"))
    assert saved_payload == payload


def test_cli_sysid_sdk_handshake_plan_is_read_only_and_requires_confirmation() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "sdk-handshake-plan",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    step_names = [step["name"] for step in payload["steps"]]

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_sdk_handshake_plan.v1"
    assert payload["read_only"] is True
    assert payload["movement_allowed"] is False
    assert payload["requires_confirm"] == "I UNDERSTAND THIS WILL MOVE THE ARM"
    assert payload["fault_landing_mode"] == "damping"
    assert payload["next_gate"] == "sdk_runner_confirm_then_hardware_validation"
    assert step_names == [
        "sdk_preflight",
        "operator_confirm",
        "enter_hold_or_damping",
        "start_recording_after_safe_state",
        "fault_or_ctrl_c_to_damping",
    ]
