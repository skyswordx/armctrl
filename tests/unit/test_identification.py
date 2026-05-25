"""参数辨识模块测试。

这些测试先固定三件事：
1. 激励轨迹必须从低风险 profile 开始，并能逐级扩展到傅里叶多关节轨迹；
2. 采集数据必须有稳定 CSV/manifest 格式，方便 URDFly、Pinocchio、FIGAROH、FloBaRoID 后处理；
3. 后端必须通过协议解耦，fake 和真实 SDK 只替换硬件 I/O，不影响轨迹与数据格式。
"""

import csv
import json
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import pytest

import armctrl.cli.arx5ctl as arx5ctl_cli
from armctrl.cli.arx5ctl import main
from armctrl.identification.backends import Arx5JointRobotIO, FakeJointRobotIO
from armctrl.identification.models import TrajectoryPoint
from armctrl.identification.optimization import ExcitationScore, optimize_fourier_multisine, score_excitation_profile
from armctrl.identification.postprocess import _zero_phase_moving_average, postprocess_dataset
from armctrl.identification.recorder import DatasetRecorder
from armctrl.identification.runner import IdentificationRunner
from armctrl.identification.safety import TrajectorySafetyLimits, validate_trajectory
from armctrl.identification.solver import (
    _append_joint_affine_friction_columns,
    _base_parameter_subset_from_regressor,
    _figaroh_physical_projection_for_min_norm,
    pinocchio_regressor_scorer,
)
from armctrl.protocol.enums import CommandStatus, ErrorCode
from armctrl.protocol.errors import ArmctrlError
from armctrl.protocol.models import CommandResponse
from armctrl.identification.trajectories import (
    generate_fourier_multisine,
    generate_friction_sweep,
    generate_gravity_sweep,
)


class _FakeIdentGain:
    def __init__(self, kp_or_dof, kd=None, gripper_kp: float = 0.0, gripper_kd: float = 0.0) -> None:
        if isinstance(kp_or_dof, int):
            self._kp = [0.0] * kp_or_dof
            self._kd = [0.0] * kp_or_dof
        else:
            self._kp = list(kp_or_dof)
            self._kd = list(kd)
        self.gripper_kp = gripper_kp
        self.gripper_kd = gripper_kd

    def kp(self):
        return self._kp

    def kd(self):
        return self._kd

    def __mul__(self, scalar: float):
        return _FakeIdentGain(
            [value * scalar for value in self._kp],
            [value * scalar for value in self._kd],
            self.gripper_kp * scalar,
            self.gripper_kd * scalar,
        )

    def __add__(self, other):
        return _FakeIdentGain(
            [left + right for left, right in zip(self._kp, other._kp)],
            [left + right for left, right in zip(self._kd, other._kd)],
            self.gripper_kp + other.gripper_kp,
            self.gripper_kd + other.gripper_kd,
        )


class _FakeIdentJointState:
    def __init__(self, dof: int) -> None:
        self._pos = [0.0] * dof
        self._vel = [0.0] * dof
        self._torque = [0.0] * dof
        self.timestamp = 0.0
        self.gripper_pos = 0.0

    def pos(self):
        return self._pos

    def vel(self):
        return self._vel

    def torque(self):
        return self._torque


class _FakeIdentController:
    def __init__(self) -> None:
        self.config = SimpleNamespace(
            default_kp=[80.0, 70.0],
            default_kd=[2.0, 2.0],
            default_gripper_kp=5.0,
            default_gripper_kd=0.2,
            controller_dt=0.002,
        )
        self.gain = _FakeIdentGain(2)
        self.calls: list[str] = []
        self.joint_traj = None
        self.joint_cmds = []
        self.joint_state = _FakeIdentJointState(2)
        self.joint_state.pos()[:] = [0.25, -0.15]
        self.joint_state.timestamp = 9.9

    def get_controller_config(self):
        return self.config

    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        self.calls.append("set_gain")
        self.gain = gain

    def get_timestamp(self):
        return 10.0

    def get_joint_state(self):
        return self.joint_state

    def set_joint_cmd(self, joint_cmd):
        self.calls.append("set_joint_cmd")
        self.joint_cmds.append(joint_cmd)

    def set_joint_traj(self, joint_traj):
        self.calls.append("set_joint_traj")
        self.joint_traj = joint_traj


class _FakeIdentSDK:
    Gain = _FakeIdentGain
    JointState = _FakeIdentJointState


def test_gravity_sweep_moves_one_joint_at_a_time_with_low_velocity():
    profile = generate_gravity_sweep(dof=3, sample_hz=20.0, amplitude_rad=0.12, segment_duration_s=1.0)

    assert profile.name == "gravity_sweep"
    assert profile.points[0].q == pytest.approx((0.0, 0.0, 0.0))
    assert profile.points[-1].q == pytest.approx((0.0, 0.0, 0.0))
    for point in profile.points:
        moving_axes = [index for index, velocity in enumerate(point.dq) if abs(velocity) > 1e-9]
        assert len(moving_axes) <= 1
        assert max(abs(velocity) for velocity in point.dq) <= 0.25


def test_gravity_sweep_can_insert_static_dwell_samples():
    profile = generate_gravity_sweep(
        dof=2,
        sample_hz=10.0,
        amplitude_rad=0.12,
        segment_duration_s=2.0,
        dwell_s=0.5,
    )

    hold_points = [point for point in profile.points if point.phase.endswith("_hold")]

    assert hold_points
    assert profile.metadata["dwell_s"] == pytest.approx(0.5)
    assert all(max(abs(value) for value in point.dq) == pytest.approx(0.0) for point in hold_points)
    assert all(max(abs(value) for value in point.ddq) == pytest.approx(0.0) for point in hold_points)


def test_gravity_sweep_uses_center_pose_for_all_targets():
    profile = generate_gravity_sweep(
        dof=3,
        sample_hz=10.0,
        amplitude_rad=0.12,
        segment_duration_s=0.5,
        q_center=(0.0, 0.30, 0.30),
    )

    assert profile.metadata["q_center"] == pytest.approx((0.0, 0.30, 0.30))
    assert profile.points[0].q == pytest.approx((0.0, 0.30, 0.30))
    assert profile.points[-1].q == pytest.approx((0.0, 0.30, 0.30))
    assert min(point.q[1] for point in profile.points) >= 0.18 - 1e-12
    assert min(point.q[2] for point in profile.points) >= 0.18 - 1e-12


def test_friction_sweep_covers_positive_and_negative_velocity_per_joint():
    profile = generate_friction_sweep(dof=2, sample_hz=30.0, amplitude_rad=0.10, slow_speed_radps=0.05)

    for joint_index in range(2):
        velocities = [point.dq[joint_index] for point in profile.points]
        assert max(velocities) > 0.01
        assert min(velocities) < -0.01


def test_friction_sweep_contains_constant_velocity_plateaus():
    profile = generate_friction_sweep(
        dof=1,
        sample_hz=50.0,
        amplitude_rad=0.12,
        slow_speed_radps=0.04,
        medium_speed_radps=0.12,
    )

    plateau_points = [point for point in profile.points if "plateau" in point.phase]
    plateau_velocities = {round(point.dq[0], 3) for point in plateau_points}

    assert plateau_points
    assert {0.04, -0.04, 0.12, -0.12}.issubset(plateau_velocities)
    assert all(abs(point.ddq[0]) == pytest.approx(0.0) for point in plateau_points)


def test_fourier_multisine_has_zero_boundary_velocity_and_passes_safety():
    profile = generate_fourier_multisine(
        dof=4,
        sample_hz=50.0,
        duration_s=2.0,
        harmonics=3,
        amplitude_rad=0.08,
        seed=7,
    )
    result = validate_trajectory(
        profile,
        TrajectorySafetyLimits(
            joint_min=(-0.5, -0.5, -0.5, -0.5),
            joint_max=(0.5, 0.5, 0.5, 0.5),
            velocity_max=(0.8, 0.8, 0.8, 0.8),
            acceleration_max=(4.0, 4.0, 4.0, 4.0),
        ),
    )

    assert result.allowed
    assert profile.points[0].q == pytest.approx((0.0, 0.0, 0.0, 0.0), abs=1e-12)
    assert profile.points[-1].q == pytest.approx((0.0, 0.0, 0.0, 0.0), abs=1e-12)
    assert profile.points[0].dq == pytest.approx((0.0, 0.0, 0.0, 0.0), abs=1e-12)
    assert profile.points[-1].dq == pytest.approx((0.0, 0.0, 0.0, 0.0), abs=1e-12)


def test_fourier_multisine_uses_explicit_center_pose():
    profile = generate_fourier_multisine(
        dof=3,
        sample_hz=20.0,
        duration_s=2.0,
        harmonics=2,
        amplitude_rad=0.05,
        seed=3,
        q_center=(0.6, 0.7, 0.2),
    )

    assert profile.metadata["q_center"] == pytest.approx((0.6, 0.7, 0.2))
    assert profile.points[0].q == pytest.approx((0.6, 0.7, 0.2), abs=1e-12)
    assert profile.points[-1].q == pytest.approx((0.6, 0.7, 0.2), abs=1e-12)


def test_dataset_manifest_exports_lerobot_contract(tmp_path: Path):
    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    recorder = DatasetRecorder(tmp_path)

    manifest = recorder.write_run(
        profile=profile,
        backend_name="fake_joint",
        model="X5",
        samples=[],
        execute=False,
        urdf_path="configs/models/X5_camera.urdf",
    )

    assert manifest.lerobot_contract["schema"] == "lerobot-compatible"
    assert manifest.lerobot_contract["observation_features"]["joint_1.pos"]["unit"] == "rad"
    assert manifest.lerobot_contract["action_features"]["joint_2.pos"]["unit"] == "rad"
    assert manifest.lerobot_contract["column_map"]["observation"]["q_1"] == "joint_1.pos"
    assert manifest.lerobot_contract["column_map"]["action"]["q_cmd_2"] == "joint_2.pos"
    with (tmp_path / "lerobot_contract.json").open(encoding="utf-8") as file:
        assert json.load(file) == manifest.lerobot_contract


def test_tool_handoff_makes_figaroh_the_offline_math_target(tmp_path: Path):
    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    recorder = DatasetRecorder(tmp_path)
    runner = IdentificationRunner(FakeJointRobotIO(dof=2), sample_hz=10.0, sleep_fn=lambda _: None)
    runner.run(profile, recorder=recorder, execute=True)

    response = postprocess_dataset(
        dataset_dir=tmp_path,
        output_dir=tmp_path / "processed",
        tools=("figaroh",),
        urdf_path="configs/models/X5_camera.urdf",
        smoothing_window=3,
    )

    assert response.status.value == "completed"
    handoff = (tmp_path / "processed" / "tool_handoff.md").read_text(encoding="utf-8")
    assert "FIGAROH" in handoff
    assert "LeRobot" in handoff
    assert "fixed postprocess solver stage" in handoff
    assert "tau = Y(q, dq, ddq) * pi" in handoff


def test_optimized_fourier_multisine_improves_surrogate_condition_number():
    limits = TrajectorySafetyLimits(
        joint_min=(-0.5, -0.5, -0.5),
        joint_max=(0.5, 0.5, 0.5),
        velocity_max=(1.2, 1.2, 1.2),
        acceleration_max=(8.0, 8.0, 8.0),
    )
    baseline = generate_fourier_multisine(
        dof=3,
        sample_hz=30.0,
        duration_s=2.0,
        harmonics=3,
        amplitude_rad=0.04,
        seed=1,
    )
    optimized = optimize_fourier_multisine(
        dof=3,
        sample_hz=30.0,
        duration_s=2.0,
        harmonics=3,
        amplitude_rad=0.04,
        seed=1,
        candidate_count=6,
        safety_limits=limits,
    )

    baseline_score = score_excitation_profile(baseline)
    optimized_score = score_excitation_profile(optimized)
    assert optimized_score.condition_number <= baseline_score.condition_number
    assert optimized.metadata["optimization"]["candidate_count"] == 6
    assert optimized.metadata["optimization"]["score_mode"] == "surrogate_feature_condition"


def test_optimized_fourier_multisine_accepts_real_regressor_scorer():
    seen_seeds: list[int] = []

    def scorer(profile):
        seed = int(profile.metadata["seed"])
        seen_seeds.append(seed)
        return ExcitationScore(
            condition_number=100.0 - seed,
            rank=12,
            feature_count=18,
            sample_count=len(profile.points),
            mode="pinocchio_regressor_condition",
        )

    optimized = optimize_fourier_multisine(
        dof=2,
        sample_hz=20.0,
        duration_s=2.0,
        harmonics=2,
        amplitude_rad=0.04,
        seed=3,
        candidate_count=4,
        scorer=scorer,
    )

    assert seen_seeds == [3, 4, 5, 6]
    assert optimized.metadata["seed"] == 6
    assert optimized.metadata["optimization"]["score_mode"] == "pinocchio_regressor_condition"
    assert optimized.metadata["optimization"]["condition_number"] == pytest.approx(94.0)


def test_pinocchio_regressor_scorer_skips_when_pinocchio_unavailable():
    scorer = pinocchio_regressor_scorer(urdf_path="missing.urdf", dof=2)
    profile = generate_fourier_multisine(
        dof=2,
        sample_hz=20.0,
        duration_s=1.0,
        harmonics=2,
        amplitude_rad=0.04,
        seed=1,
    )

    score = scorer(profile)

    assert score.mode in {"pinocchio_regressor_unavailable", "pinocchio_regressor_condition"}
    assert score.sample_count == len(profile.points)


def test_runner_records_fake_backend_dataset(tmp_path: Path):
    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    backend = FakeJointRobotIO(dof=2)
    recorder = DatasetRecorder(tmp_path)
    runner = IdentificationRunner(backend=backend, sample_hz=10.0, sleep_fn=lambda _: None)

    response = runner.run(profile, recorder=recorder, execute=True)

    assert response.status.value == "completed"
    raw_csv = tmp_path / "raw_samples.csv"
    manifest = tmp_path / "manifest.json"
    assert raw_csv.is_file()
    assert manifest.is_file()
    with manifest.open(encoding="utf-8") as file:
        payload = json.load(file)
    assert payload["profile_name"] == "gravity_sweep"
    assert payload["backend_name"] == "fake_joint"
    assert payload["columns"]["q"] == ["q_1", "q_2"]
    with raw_csv.open(encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    assert rows
    assert {"q_cmd_1", "dq_cmd_1", "ddq_cmd_1", "tau_meas_1"}.issubset(rows[0])


def test_sdk_joint_backend_streams_joint_commands_from_current_state():
    backend = Arx5JointRobotIO(model="X5", interface="can0", resume_gain_duration_s=0.006, sleep_fn=lambda _: None)
    backend._sdk = _FakeIdentSDK()
    backend._controller = _FakeIdentController()
    backend.dof = 2

    response = backend.send_joint_trajectory(
        (
            TrajectoryPoint(0.0, (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)),
            TrajectoryPoint(0.1, (0.1, -0.1), (0.0, 0.0), (0.0, 0.0)),
        )
    )

    assert response.status.value == "completed"
    assert "set_joint_traj" not in backend._controller.calls
    assert backend._controller.calls[:4] == ["set_gain", "set_gain", "set_gain", "set_joint_cmd"]
    assert backend._controller.gain.kp() == pytest.approx(backend._controller.config.default_kp)
    assert backend._controller.gain.kd() == pytest.approx(backend._controller.config.default_kd)
    assert backend._controller.joint_cmds[0].pos() == pytest.approx((0.25, -0.15))
    assert backend._controller.joint_cmds[0].vel() == pytest.approx((0.0, 0.0))
    assert backend._controller.joint_cmds[0].timestamp == pytest.approx(10.002)
    assert backend._controller.joint_cmds[1].pos() == pytest.approx((0.0, 0.0))
    assert backend._controller.joint_cmds[1].timestamp == pytest.approx(10.2)
    assert backend._controller.joint_cmds[2].pos() == pytest.approx((0.1, -0.1))
    assert backend._controller.joint_cmds[2].timestamp == pytest.approx(10.3)


def test_sdk_joint_backend_completion_hold_refreshes_future_command():
    backend = Arx5JointRobotIO(model="X5", interface="can0", resume_gain_duration_s=0.006, sleep_fn=lambda _: None)
    backend._sdk = _FakeIdentSDK()
    backend._controller = _FakeIdentController()
    backend.dof = 2
    sleeps: list[float] = []

    def interrupting_sleep(seconds: float) -> None:
        sleeps.append(seconds)
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        backend.hold_joint_position_until_cancelled(
            TrajectoryPoint(1.0, (0.2, -0.1), (0.0, 0.0), (0.0, 0.0), "hold"),
            interrupting_sleep,
            update_hz=50.0,
        )

    assert sleeps == pytest.approx([0.02])
    assert backend._controller.joint_cmds[-1].pos() == pytest.approx((0.2, -0.1))
    assert backend._controller.joint_cmds[-1].vel() == pytest.approx((0.0, 0.0))
    assert backend._controller.joint_cmds[-1].timestamp > backend._controller.get_timestamp()


def test_runner_execute_resets_home_before_sending_trajectory():
    class RecordingBackend(FakeJointRobotIO):
        def __init__(self) -> None:
            super().__init__(dof=2)
            self.calls: list[str] = []

        def connect(self) -> CommandResponse:
            self.calls.append("connect")
            return super().connect()

        def reset_home(self) -> CommandResponse:
            self.calls.append("reset_home")
            return super().reset_home()

        def send_joint_trajectory(self, points) -> CommandResponse:
            self.calls.append("send_joint_trajectory")
            return super().send_joint_trajectory(points)

    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    backend = RecordingBackend()
    runner = IdentificationRunner(backend=backend, sample_hz=10.0, sleep_fn=lambda _: None)

    response = runner.run(profile, execute=True)

    assert response.status.value == "completed"
    assert backend.calls[:3] == ["connect", "reset_home", "send_joint_trajectory"]


def test_runner_streams_points_when_backend_supports_incremental_commands():
    class StreamingBackend(FakeJointRobotIO):
        def __init__(self) -> None:
            super().__init__(dof=2)
            self.calls: list[str] = []

        def connect(self) -> CommandResponse:
            self.calls.append("connect")
            return super().connect()

        def reset_home(self) -> CommandResponse:
            self.calls.append("reset_home")
            return super().reset_home()

        def begin_joint_trajectory(self, points) -> CommandResponse:
            self.calls.append(f"begin:{len(points)}")
            return CommandResponse(CommandStatus.COMPLETED, "stream prepared")

        def send_joint_command(self, point) -> CommandResponse:
            self.calls.append(f"send:{point.t_s:.1f}")
            self._last_command = point
            return CommandResponse(CommandStatus.COMPLETED, "point sent")

    profile = generate_gravity_sweep(dof=2, sample_hz=2.0, amplitude_rad=0.05, segment_duration_s=0.5)
    backend = StreamingBackend()
    runner = IdentificationRunner(backend=backend, sample_hz=2.0, sleep_fn=lambda _: None)

    response = runner.run(profile, execute=True)

    assert response.status.value == "completed"
    assert backend.calls[:3] == ["connect", "reset_home", f"begin:{len(profile.points)}"]
    assert [call for call in backend.calls if call.startswith("send:")] == [
        f"send:{point.t_s:.1f}" for point in profile.points
    ]


def test_runner_prepositions_nonzero_start_before_recording():
    class StreamingBackend(FakeJointRobotIO):
        def __init__(self) -> None:
            super().__init__(dof=2)
            self.calls: list[str] = []

        def connect(self) -> CommandResponse:
            self.calls.append("connect")
            return super().connect()

        def reset_home(self) -> CommandResponse:
            self.calls.append("reset_home")
            return super().reset_home()

        def begin_joint_trajectory(self, points) -> CommandResponse:
            self.calls.append(f"begin:{len(points)}")
            return CommandResponse(CommandStatus.COMPLETED, "stream prepared")

        def send_joint_command(self, point) -> CommandResponse:
            self.calls.append(f"send:{point.phase}:{point.t_s:.1f}:{point.q[1]:.2f}")
            self._last_command = point
            return CommandResponse(CommandStatus.COMPLETED, "point sent")

    sleeps: list[float] = []
    profile = generate_gravity_sweep(
        dof=2,
        sample_hz=2.0,
        amplitude_rad=0.05,
        segment_duration_s=0.5,
        q_center=(0.0, 0.30),
    )
    backend = StreamingBackend()
    runner = IdentificationRunner(
        backend=backend,
        sample_hz=2.0,
        sleep_fn=sleeps.append,
        preposition_settle_s=0.75,
    )

    response = runner.run(profile, execute=True)

    assert response.status.value == "completed"
    assert backend.calls[:5] == [
        "connect",
        "reset_home",
        f"begin:{len(profile.points)}",
        "send:preposition_center:0.0:0.30",
        f"begin:{len(profile.points)}",
    ]
    assert sleeps[0] == pytest.approx(0.75)
    assert response.detail["preposition_before_recording"] is True
    recorded_sends = backend.calls[5:]
    assert recorded_sends[0].startswith("send:gravity_start:0.0:0.30")


def test_runner_execute_aborts_if_reset_home_fails():
    class FailingResetBackend(FakeJointRobotIO):
        def __init__(self) -> None:
            super().__init__(dof=2)
            self.send_called = False

        def reset_home(self) -> CommandResponse:
            return CommandResponse(
                CommandStatus.FAULTED,
                "reset home failed",
                error=ArmctrlError(ErrorCode.SDK_ERROR, "reset home failed"),
            )

        def send_joint_trajectory(self, points) -> CommandResponse:
            self.send_called = True
            return super().send_joint_trajectory(points)

    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    backend = FailingResetBackend()
    runner = IdentificationRunner(backend=backend, sample_hz=10.0, sleep_fn=lambda _: None)

    response = runner.run(profile, execute=True)

    assert response.status.value == "faulted"
    assert backend.send_called is False


def test_runner_ctrl_c_damps_and_returns_cancelled():
    class InterruptingBackend(FakeJointRobotIO):
        def __init__(self) -> None:
            super().__init__(dof=2)
            self.calls: list[str] = []

        def reset_home(self) -> CommandResponse:
            self.calls.append("reset_home")
            return super().reset_home()

        def send_joint_trajectory(self, points) -> CommandResponse:
            self.calls.append("send_joint_trajectory")
            return super().send_joint_trajectory(points)

        def damping(self) -> CommandResponse:
            self.calls.append("damping")
            return super().damping()

    def interrupt_sleep(_seconds: float) -> None:
        raise KeyboardInterrupt

    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    backend = InterruptingBackend()
    runner = IdentificationRunner(backend=backend, sample_hz=10.0, sleep_fn=interrupt_sleep)

    response = runner.run(profile, execute=True)

    assert response.status.value == "cancelled"
    assert backend.calls[-1] == "damping"


def test_runner_success_holds_by_default_without_damping():
    class TrackingBackend(FakeJointRobotIO):
        def __init__(self) -> None:
            super().__init__(dof=2)
            self.damping_called = False
            self.hold_calls = 0

        def damping(self) -> CommandResponse:
            self.damping_called = True
            return super().damping()

        def hold_joint_position_until_cancelled(self, point, sleep_fn, update_hz: float) -> CommandResponse:
            self.hold_calls += 1
            return CommandResponse(
                CommandStatus.COMPLETED,
                "hold exited immediately for test",
                detail={"hold_cycles": 1, "update_hz": update_hz, "q": point.q},
            )

    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    backend = TrackingBackend()
    runner = IdentificationRunner(backend=backend, sample_hz=10.0, sleep_fn=lambda _: None)

    response = runner.run(profile, execute=True)

    assert response.status.value == "completed"
    assert backend.damping_called is False
    assert backend.hold_calls == 1
    assert response.detail["completion_hold"] is True
    assert response.detail["completion_hold_response"]["detail"]["q"] == pytest.approx(profile.points[-1].q)


def test_runner_success_can_request_damping_after_completion():
    class TrackingBackend(FakeJointRobotIO):
        def __init__(self) -> None:
            super().__init__(dof=2)
            self.damping_called = False

        def damping(self) -> CommandResponse:
            self.damping_called = True
            return super().damping()

    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    backend = TrackingBackend()
    runner = IdentificationRunner(
        backend=backend,
        sample_hz=10.0,
        sleep_fn=lambda _: None,
        damping_after=True,
    )

    response = runner.run(profile, execute=True)

    assert response.status.value == "completed"
    assert backend.damping_called is True
    assert response.detail["completion_hold"] is False


def test_runner_completion_hold_ctrl_c_damps_and_returns_completed():
    class InterruptingHoldBackend(FakeJointRobotIO):
        def __init__(self) -> None:
            super().__init__(dof=2)
            self.damping_called = False

        def hold_joint_position_until_cancelled(self, point, sleep_fn, update_hz: float) -> CommandResponse:
            raise KeyboardInterrupt

        def damping(self) -> CommandResponse:
            self.damping_called = True
            return super().damping()

    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    backend = InterruptingHoldBackend()
    runner = IdentificationRunner(backend=backend, sample_hz=10.0, sleep_fn=lambda _: None)

    response = runner.run(profile, execute=True)

    assert response.status.value == "completed"
    assert backend.damping_called is True
    assert response.detail["completion_hold_interrupted"] is True
    assert response.detail["damping_after_hold_interrupt"]["status"] == "completed"


def test_postprocess_writes_processed_csv_and_tool_handoff(tmp_path: Path):
    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    recorder = DatasetRecorder(tmp_path)
    runner = IdentificationRunner(FakeJointRobotIO(dof=2), sample_hz=10.0, sleep_fn=lambda _: None)
    runner.run(profile, recorder=recorder, execute=True)

    response = postprocess_dataset(
        dataset_dir=tmp_path,
        output_dir=tmp_path / "processed",
        tools=("pinocchio", "figaroh", "flobaroid", "urdfly"),
        urdf_path="configs/models/X5_camera.urdf",
        smoothing_window=3,
        filter_mode="zero_phase_moving_average",
    )

    assert response.status.value == "completed"
    assert (tmp_path / "processed" / "processed_samples.csv").is_file()
    assert (tmp_path / "processed" / "tool_handoff.md").is_file()
    assert (tmp_path / "processed" / "lerobot_contract.json").is_file()
    assert (tmp_path / "processed" / "pinocchio_regressor_skeleton.py").is_file()
    assert (tmp_path / "processed" / "quality_metrics.json").is_file()
    assert (tmp_path / "processed" / "quality_report.md").is_file()
    assert response.detail["quality_metrics_json"] == str(tmp_path / "processed" / "quality_metrics.json")
    with (tmp_path / "processed" / "lerobot_contract.json").open(encoding="utf-8") as file:
        assert json.load(file)["schema"] == "lerobot-compatible"
    with (tmp_path / "processed" / "quality_metrics.json").open(encoding="utf-8") as file:
        quality = json.load(file)
    assert quality["schema"] == "armctrl-ident-quality-v1"
    assert quality["preprocessing"]["filter_mode"] == "zero_phase_moving_average"
    assert quality["preprocessing"]["zero_phase"] is True
    assert quality["document_sections"]["physical_consistency"]["status"] == "not_evaluated"
    assert quality["tool_execution"]["figaroh"]["status"] == "handoff_only"
    handoff = (tmp_path / "processed" / "tool_handoff.md").read_text(encoding="utf-8")
    skeleton = (tmp_path / "processed" / "pinocchio_regressor_skeleton.py").read_text(encoding="utf-8")
    assert "Pinocchio" in handoff
    assert "URDFly" in handoff
    assert "`q_proc_* / dq_proc_* / ddq_proc_* / tau_proc_*`" in handoff
    assert 'row[f"q_proc_{index}"]' in skeleton
    assert 'row[f"dq_proc_{index}"]' in skeleton
    assert 'row[f"ddq_proc_{index}"]' in skeleton
    assert 'row[f"tau_proc_{index}"]' in skeleton
    quality_report = (tmp_path / "processed" / "quality_report.md").read_text(encoding="utf-8")
    assert "参数辨识数据质量报告" in quality_report
    assert "数据健康" in quality_report
    assert "激励充分性" in quality_report


def test_zero_phase_moving_average_is_symmetric_and_preserves_peak_center():
    values = [0.0, 0.0, 1.0, 0.0, 0.0]

    filtered = _zero_phase_moving_average(values, 3)

    assert filtered == pytest.approx(list(reversed(filtered)))
    assert filtered.index(max(filtered)) == 2
    assert filtered[1] == pytest.approx(filtered[3])


def test_postprocess_writes_solver_metrics_and_chinese_report(tmp_path: Path):
    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    recorder = DatasetRecorder(tmp_path)
    runner = IdentificationRunner(FakeJointRobotIO(dof=2), sample_hz=10.0, sleep_fn=lambda _: None)
    runner.run(profile, recorder=recorder, execute=True)

    response = postprocess_dataset(
        dataset_dir=tmp_path,
        output_dir=tmp_path / "processed",
        tools=("pinocchio", "figaroh"),
        urdf_path="configs/models/X5_camera.urdf",
        smoothing_window=3,
    )

    assert response.status.value == "completed"
    solver_metrics_path = tmp_path / "processed" / "solver_metrics.json"
    solver_report_path = tmp_path / "processed" / "solver_report_zh.md"
    solver_script_path = tmp_path / "processed" / "run_solver_stage.py"
    assert solver_metrics_path.is_file()
    assert solver_report_path.is_file()
    assert solver_script_path.is_file()
    assert response.detail["solver_metrics_json"] == str(solver_metrics_path)
    assert response.detail["solver_report_zh"] == str(solver_report_path)
    assert response.detail["solver_script"] == str(solver_script_path)
    with solver_metrics_path.open(encoding="utf-8") as file:
        solver_metrics = json.load(file)
    assert solver_metrics["schema"] == "armctrl-ident-solver-quality-v1"
    assert solver_metrics["document_gate_mapping"]["prediction_error"]["zh"] == "预测误差"
    assert solver_metrics["input_quality"]["profile_name"] == "gravity_sweep"
    assert "pinocchio" in solver_metrics["solvers"]
    assert "figaroh" in solver_metrics["solvers"]
    pinocchio = solver_metrics["solvers"]["pinocchio"]
    if pinocchio["status"] == "completed":
        gravity_only = pinocchio["runs"]["gravity_only"]
        assert gravity_only["parameter_subset"]["mode"] == "gravity_base_columns"
        assert gravity_only["parameter_subset"]["selected_parameter_count"] < gravity_only["parameter_subset"]["original_parameter_count"]
        assert gravity_only["physical_consistency_min_norm_solution"]["status"] == "not_applicable"
        assert "full_base" in pinocchio["runs"]
        full_base = pinocchio["runs"]["full_base"]
        assert full_base["parameter_subset"]["mode"] == "base_parameter_columns"
        assert full_base["parameter_subset"]["selected_parameter_count"] <= full_base["parameter_subset"]["original_parameter_count"]
        assert full_base["parameter_subset"]["source"] in {"figaroh_qr", "svd_rank_revealing_fallback"}
        assert "full_augmented" in pinocchio["runs"]
        full_augmented = pinocchio["runs"]["full_augmented"]
        assert full_augmented["parameter_subset"]["mode"] == "base_plus_joint_bias_viscous_coulomb"
        assert full_augmented["parameter_subset"]["augmented_parameter_count"] == full_augmented["regressor"]["parameter_count"]
        assert full_augmented["physical_consistency_min_norm_solution"]["status"] == "not_applicable"
    report = solver_report_path.read_text(encoding="utf-8")
    assert "参数辨识结果质量评估指标" in report
    assert "数据健康" in report
    assert "激励充分性" in report
    assert "回归矩阵条件数" in report
    assert "参数物理一致性" in report
    assert "预测误差" in report
    assert "控制收益" in report
    if pinocchio["status"] == "completed":
        assert "full_base" in report
        assert "full_augmented" in report
        assert "最小可辨识基础参数" in report
        assert "关节力矩零偏/粘滞/库仑摩擦" in report


def test_solver_base_parameter_subset_reduces_rank_deficient_regressor():
    matrix = [
        [1.0, 0.0, 1.0, 0.0],
        [0.0, 1.0, 0.0, 1.0],
        [2.0, 0.0, 2.0, 0.0],
        [0.0, 3.0, 0.0, 3.0],
    ]

    subset = _base_parameter_subset_from_regressor(matrix, source_hint="unit_test")

    assert subset["mode"] == "base_parameter_columns"
    assert subset["original_parameter_count"] == 4
    assert subset["selected_parameter_count"] == 2
    assert subset["source"] == "svd_rank_revealing_fallback"
    assert len(subset["selected_columns"]) == 2


def test_solver_augmented_friction_columns_add_bias_viscous_and_coulomb():
    y_matrix = [[1.0, 2.0], [3.0, 4.0]]
    rows = [
        {"dq_proc_1": "0.25", "dq_proc_2": "-0.5"},
        {"dq_proc_1": "0.0", "dq_proc_2": "0.75"},
    ]

    augmented, metadata = _append_joint_affine_friction_columns(y_matrix, rows=rows, dof=2)

    assert metadata["mode"] == "base_plus_joint_bias_viscous_coulomb"
    assert metadata["rigid_parameter_count"] == 2
    assert metadata["friction_parameter_count"] == 6
    assert metadata["augmented_parameter_count"] == 8
    assert augmented[0][2:] == pytest.approx([1.0, 0.25, 1.0, 0.0, 0.0, 0.0])
    assert augmented[1][2:] == pytest.approx([0.0, 0.0, 0.0, 1.0, 0.75, 1.0])


def test_figaroh_physical_projection_reports_unavailable_without_projection_backend():
    projection = _figaroh_physical_projection_for_min_norm(model=None, pi_hat=[1.0, 2.0, 3.0], active_dof=2)

    assert projection["status"] in {"skipped", "not_applicable"}
    assert projection["source"] == "figaroh_physical_consistency"


def test_postprocess_quality_metrics_flags_unreached_negative_sweep(tmp_path: Path):
    raw_path = tmp_path / "raw_samples.csv"
    fieldnames = [
        "t_s",
        "phase",
        "q_1",
        "q_2",
        "dq_1",
        "dq_2",
        "tau_meas_1",
        "tau_meas_2",
        "q_cmd_1",
        "q_cmd_2",
        "dq_cmd_1",
        "dq_cmd_2",
        "ddq_cmd_1",
        "ddq_cmd_2",
        "tau_cmd_1",
        "tau_cmd_2",
    ]
    rows = [
        {
            "t_s": index * 0.25,
            "phase": "gravity_joint_2",
            "q_1": 0.0,
            "q_2": actual,
            "dq_1": 0.0,
            "dq_2": 0.0,
            "tau_meas_1": 0.1,
            "tau_meas_2": tau,
            "q_cmd_1": 0.0,
            "q_cmd_2": command,
            "dq_cmd_1": 0.0,
            "dq_cmd_2": 0.0,
            "ddq_cmd_1": 0.0,
            "ddq_cmd_2": 0.0,
            "tau_cmd_1": 0.0,
            "tau_cmd_2": 0.0,
        }
        for index, (command, actual, tau) in enumerate(
            [
                (0.0, 0.0, 0.0),
                (0.10, 0.10, 0.5),
                (0.12, 0.12, 0.7),
                (-0.10, 0.0, -0.4),
                (-0.12, 0.0, -0.5),
            ]
        )
    ]
    with raw_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    (tmp_path / "manifest.json").write_text(
        json.dumps(
            {
                "dof": 2,
                "duration_s": 1.0,
                "sample_hz": 4.0,
                "profile_name": "gravity_sweep",
                "profile_metadata": {"amplitude_rad": 0.12},
                "raw_samples": "raw_samples.csv",
                "lerobot_contract": {"schema": "lerobot-compatible"},
            }
        ),
        encoding="utf-8",
    )

    response = postprocess_dataset(
        dataset_dir=tmp_path,
        output_dir=tmp_path / "processed",
        tools=("figaroh", "pinocchio"),
        smoothing_window=1,
    )

    assert response.status.value == "completed"
    quality = response.detail["quality_metrics"]
    assert quality["data_readiness_status"] == "fail"
    assert quality["document_sections"]["excitation"]["status"] == "fail"
    joint_2 = quality["joint_metrics"][1]
    assert joint_2["coverage_ratio"] == pytest.approx(0.5)
    assert joint_2["direction_reach"]["negative"]["status"] == "fail"
    assert joint_2["direction_reach"]["negative"]["mean_error_rad"] > 0.09


def test_postprocess_quality_metrics_flags_small_gravity_absolute_range(tmp_path: Path):
    raw_path = tmp_path / "raw_samples.csv"
    fieldnames = [
        "t_s",
        "phase",
        "q_1",
        "q_2",
        "dq_1",
        "dq_2",
        "tau_meas_1",
        "tau_meas_2",
        "q_cmd_1",
        "q_cmd_2",
        "dq_cmd_1",
        "dq_cmd_2",
        "ddq_cmd_1",
        "ddq_cmd_2",
        "tau_cmd_1",
        "tau_cmd_2",
    ]
    rows = []
    values = [-0.11, -0.055, 0.0, 0.055, 0.11]
    for index, value in enumerate(values):
        rows.append(
            {
                "t_s": index * 0.25,
                "phase": "gravity_joint_1",
                "q_1": value,
                "q_2": 0.3,
                "dq_1": 0.0,
                "dq_2": 0.0,
                "tau_meas_1": value,
                "tau_meas_2": 1.0,
                "q_cmd_1": value,
                "q_cmd_2": 0.3,
                "dq_cmd_1": 0.0,
                "dq_cmd_2": 0.0,
                "ddq_cmd_1": 0.0,
                "ddq_cmd_2": 0.0,
                "tau_cmd_1": 0.0,
                "tau_cmd_2": 0.0,
            }
        )
    with raw_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    (tmp_path / "manifest.json").write_text(
        json.dumps(
            {
                "dof": 2,
                "duration_s": 1.0,
                "sample_hz": 4.0,
                "profile_name": "gravity_sweep",
                "profile_metadata": {"amplitude_rad": 0.12},
                "raw_samples": "raw_samples.csv",
                "lerobot_contract": {"schema": "lerobot-compatible"},
            }
        ),
        encoding="utf-8",
    )

    response = postprocess_dataset(
        dataset_dir=tmp_path,
        output_dir=tmp_path / "processed",
        tools=("figaroh", "pinocchio"),
        smoothing_window=1,
    )

    assert response.status.value == "completed"
    quality = response.detail["quality_metrics"]
    assert quality["data_readiness_status"] == "fail"
    assert quality["document_sections"]["excitation"]["status"] == "fail"
    assert quality["profile_metadata"]["gravity_min_actual_range_deg"] == pytest.approx(30.0)
    assert quality["joint_metrics"][0]["excitation_status"] == "fail"
    assert quality["joint_metrics"][0]["absolute_range_status"] == "fail"
    report = (tmp_path / "processed" / "quality_report.md").read_text(encoding="utf-8")
    assert "absolute angle range" in report


def test_postprocess_quality_metrics_flags_small_fourier_absolute_range(tmp_path: Path):
    raw_path = tmp_path / "raw_samples.csv"
    fieldnames = [
        "t_s",
        "phase",
        "q_1",
        "q_2",
        "dq_1",
        "dq_2",
        "tau_meas_1",
        "tau_meas_2",
        "q_cmd_1",
        "q_cmd_2",
        "dq_cmd_1",
        "dq_cmd_2",
        "ddq_cmd_1",
        "ddq_cmd_2",
        "tau_cmd_1",
        "tau_cmd_2",
    ]
    rows = []
    values = [-0.09, -0.045, 0.0, 0.045, 0.09]
    for index, value in enumerate(values):
        rows.append(
            {
                "t_s": index * 0.25,
                "phase": "fourier_multisine",
                "q_1": value,
                "q_2": 0.3 + value,
                "dq_1": 0.01,
                "dq_2": -0.01,
                "tau_meas_1": value,
                "tau_meas_2": 1.0 + value,
                "q_cmd_1": value,
                "q_cmd_2": 0.3 + value,
                "dq_cmd_1": 0.01,
                "dq_cmd_2": -0.01,
                "ddq_cmd_1": 0.01,
                "ddq_cmd_2": -0.01,
                "tau_cmd_1": 0.0,
                "tau_cmd_2": 0.0,
            }
        )
    with raw_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    (tmp_path / "manifest.json").write_text(
        json.dumps(
            {
                "dof": 2,
                "duration_s": 1.0,
                "sample_hz": 4.0,
                "profile_name": "fourier_multisine",
                "profile_metadata": {"amplitude_rad": 0.09},
                "raw_samples": "raw_samples.csv",
                "lerobot_contract": {"schema": "lerobot-compatible"},
            }
        ),
        encoding="utf-8",
    )

    response = postprocess_dataset(
        dataset_dir=tmp_path,
        output_dir=tmp_path / "processed",
        tools=("pinocchio",),
        smoothing_window=1,
    )

    assert response.status.value == "completed"
    quality = response.detail["quality_metrics"]
    assert quality["data_readiness_status"] == "fail"
    assert quality["document_sections"]["excitation"]["status"] == "fail"
    assert quality["profile_metadata"]["dynamic_min_actual_range_deg"] == pytest.approx(60.0)
    assert quality["joint_metrics"][0]["absolute_range_status"] == "fail"


def test_cli_ident_plan_json_outputs_summary(capsys):
    code = main(
        [
            "ident-plan",
            "--adapter",
            "fake",
            "--profile",
            "fourier_multisine",
            "--dof",
            "3",
            "--duration",
            "1.0",
            "--sample-hz",
            "20",
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "completed"
    assert payload["detail"]["profile_name"] == "fourier_multisine"
    assert payload["detail"]["dof"] == 3
    assert payload["detail"]["lerobot_contract"]["schema"] == "lerobot-compatible"
    assert payload["detail"]["lerobot_contract"]["joint_names"] == ["joint_1", "joint_2", "joint_3"]


def test_cli_ident_plan_uses_field_gravity_defaults(capsys):
    code = main(
        [
            "ident-plan",
            "--adapter",
            "fake",
            "--profile",
            "gravity_sweep",
            "--dof",
            "6",
            "--sample-hz",
            "100",
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["detail"]["metadata"]["amplitude_rad"] == pytest.approx(0.55)
    assert payload["detail"]["metadata"]["segment_duration_s"] == pytest.approx(12.0)
    assert payload["detail"]["metadata"]["dwell_s"] == pytest.approx(1.0)
    assert payload["detail"]["metadata"]["q_center"] == pytest.approx((0.0, 0.30, 0.30, 0.0, 0.0, 0.0))
    assert payload["detail"]["duration_s"] == pytest.approx(312.0)


def test_cli_ident_plan_accepts_gravity_dwell_override(capsys):
    code = main(
        [
            "ident-plan",
            "--adapter",
            "fake",
            "--profile",
            "gravity_sweep",
            "--dof",
            "2",
            "--sample-hz",
            "10",
            "--amplitude",
            "0.10",
            "--duration",
            "4.0",
            "--dwell",
            "0.5",
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["detail"]["metadata"]["amplitude_rad"] == pytest.approx(0.10)
    assert payload["detail"]["metadata"]["segment_duration_s"] == pytest.approx(4.0)
    assert payload["detail"]["metadata"]["dwell_s"] == pytest.approx(0.5)


def test_cli_ident_plan_uses_field_friction_defaults(capsys):
    code = main(
        [
            "ident-plan",
            "--adapter",
            "fake",
            "--profile",
            "friction_sweep",
            "--dof",
            "6",
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    metadata = payload["detail"]["metadata"]
    assert metadata["amplitude_rad"] == pytest.approx(0.12)
    assert metadata["speed_levels_radps"] == pytest.approx((0.025, 0.06, 0.12))
    assert metadata["constant_velocity_plateaus"] is True
    assert metadata["q_center"] == pytest.approx((0.0, 0.30, 0.30, 0.0, 0.0, 0.0))


def test_cli_ident_plan_uses_field_fourier_defaults(capsys):
    code = main(
        [
            "ident-plan",
            "--adapter",
            "fake",
            "--profile",
            "fourier_multisine",
            "--dof",
            "6",
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    metadata = payload["detail"]["metadata"]
    assert metadata["duration_s"] == pytest.approx(40.0)
    assert metadata["amplitude_rad"] == pytest.approx(1.3)
    assert metadata["harmonics"] == 5
    assert metadata["q_center"] == pytest.approx((0.0, 0.30, 0.30, 0.0, 0.0, 0.0))
    assert min(payload["detail"]["planned_joint_ranges_deg"]) >= 60.0


def test_cli_ident_plan_accepts_fourier_center_pose(capsys):
    code = main(
        [
            "ident-plan",
            "--adapter",
            "fake",
            "--profile",
            "fourier_multisine",
            "--dof",
            "3",
            "--duration",
            "1.0",
            "--sample-hz",
            "20",
            "--q-center",
            "0.4",
            "0.5",
            "0.6",
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["detail"]["metadata"]["q_center"] == pytest.approx((0.4, 0.5, 0.6))


def test_cli_ident_plan_can_optimize_fourier_profile(capsys):
    code = main(
        [
            "ident-plan",
            "--adapter",
            "fake",
            "--profile",
            "fourier_multisine",
            "--dof",
            "3",
            "--duration",
            "2.0",
            "--sample-hz",
            "20",
            "--harmonics",
            "3",
            "--q-center",
            "0.1",
            "0.2",
            "0.3",
            "--optimize",
            "--candidate-count",
            "5",
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["detail"]["metadata"]["optimization"]["candidate_count"] == 5
    assert payload["detail"]["metadata"]["optimization"]["best_seed"] >= 1
    assert payload["detail"]["metadata"]["q_center"] == pytest.approx((0.1, 0.2, 0.3))


def test_cli_ident_run_fake_writes_dataset(tmp_path: Path, capsys):
    output_prefix = tmp_path / "ident-sdk"
    code = main(
        [
            "ident-run",
            "--adapter",
            "fake",
            "--profile",
            "gravity_sweep",
            "--dof",
            "2",
            "--sample-hz",
            "10",
            "--duration",
            "0.5",
            "--amplitude",
            "0.05",
            "--output",
            str(output_prefix),
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "completed"
    output_dir = Path(payload["detail"]["output_dir"])
    assert output_dir.name.startswith("ident-sdk-")
    assert (output_dir / "raw_samples.csv").is_file()
    assert (output_dir / "manifest.json").is_file()
    assert (output_dir / "lerobot_contract.json").is_file()


def test_cli_ident_run_defaults_to_hold_after_success(tmp_path: Path, capsys):
    code = main(
        [
            "ident-run",
            "--adapter",
            "fake",
            "--profile",
            "gravity_sweep",
            "--dof",
            "2",
            "--sample-hz",
            "10",
            "--duration",
            "0.5",
            "--amplitude",
            "0.05",
            "--output",
            str(tmp_path / "ident-sdk"),
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["detail"]["completion_hold"] is True
    assert payload["detail"]["damping_after_success"] is False


def test_cli_ident_run_accepts_explicit_damping_after_success(tmp_path: Path, capsys):
    code = main(
        [
            "ident-run",
            "--adapter",
            "fake",
            "--profile",
            "gravity_sweep",
            "--dof",
            "2",
            "--sample-hz",
            "10",
            "--duration",
            "0.5",
            "--amplitude",
            "0.05",
            "--output",
            str(tmp_path / "ident-sdk"),
            "--damping-after",
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["detail"]["completion_hold"] is False
    assert payload["detail"]["damping_after_success"] is True


def test_cli_ident_plan_output_directory_gets_timestamp_suffix(tmp_path: Path, capsys, monkeypatch: pytest.MonkeyPatch):
    class FixedDateTime:
        @classmethod
        def now(cls):
            return datetime(2026, 4, 26, 12, 34, 56)

    monkeypatch.setattr(arx5ctl_cli, "datetime", FixedDateTime)
    output_prefix = tmp_path / "ident-plan"

    code = main(
        [
            "ident-plan",
            "--adapter",
            "fake",
            "--profile",
            "gravity_sweep",
            "--dof",
            "2",
            "--sample-hz",
            "10",
            "--duration",
            "0.5",
            "--amplitude",
            "0.05",
            "--output",
            str(output_prefix),
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    expected_dir = (tmp_path / "ident-plan-20260426-123456").resolve()
    assert payload["detail"]["planned_trajectory_csv"] == str(expected_dir / "planned_trajectory.csv")
    assert (expected_dir / "planned_trajectory.csv").is_file()


def test_cli_ident_run_output_directory_gets_timestamp_suffix(tmp_path: Path, capsys, monkeypatch: pytest.MonkeyPatch):
    class FixedDateTime:
        @classmethod
        def now(cls):
            return datetime(2026, 4, 26, 12, 34, 56)

    monkeypatch.setattr(arx5ctl_cli, "datetime", FixedDateTime)
    output_prefix = tmp_path / "ident-sdk"

    code = main(
        [
            "ident-run",
            "--adapter",
            "fake",
            "--profile",
            "gravity_sweep",
            "--dof",
            "2",
            "--sample-hz",
            "10",
            "--duration",
            "0.5",
            "--amplitude",
            "0.05",
            "--output",
            str(output_prefix),
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    expected_dir = (tmp_path / "ident-sdk-20260426-123456").resolve()
    assert payload["detail"]["output_dir"] == str(expected_dir)
    assert (expected_dir / "raw_samples.csv").is_file()
    assert (expected_dir / "manifest.json").is_file()


def test_cli_ident_postprocess_output_directory_gets_timestamp_suffix(tmp_path: Path, capsys, monkeypatch: pytest.MonkeyPatch):
    class FixedDateTime:
        @classmethod
        def now(cls):
            return datetime(2026, 4, 26, 12, 34, 56)

    monkeypatch.setattr(arx5ctl_cli, "datetime", FixedDateTime)
    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    recorder = DatasetRecorder(tmp_path)
    runner = IdentificationRunner(FakeJointRobotIO(dof=2), sample_hz=10.0, sleep_fn=lambda _: None)
    runner.run(profile, recorder=recorder, execute=True)

    code = main(
        [
            "ident-postprocess",
            "--dataset",
            str(tmp_path),
            "--output",
            str(tmp_path / "processed"),
            "--json",
        ]
    )

    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    expected_dir = (tmp_path / "processed-20260426-123456").resolve()
    assert payload["detail"]["output_dir"] == str(expected_dir)
    assert (expected_dir / "processed_samples.csv").is_file()
    assert (expected_dir / "tool_handoff.md").is_file()
