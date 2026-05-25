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
from armctrl.identification.optimization import optimize_fourier_multisine, score_excitation_profile
from armctrl.identification.postprocess import postprocess_dataset
from armctrl.identification.recorder import DatasetRecorder
from armctrl.identification.runner import IdentificationRunner
from armctrl.identification.safety import TrajectorySafetyLimits, validate_trajectory
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

    def get_controller_config(self):
        return self.config

    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        self.calls.append("set_gain")
        self.gain = gain

    def get_timestamp(self):
        return 10.0

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


def test_friction_sweep_covers_positive_and_negative_velocity_per_joint():
    profile = generate_friction_sweep(dof=2, sample_hz=30.0, amplitude_rad=0.10, slow_speed_radps=0.05)

    for joint_index in range(2):
        velocities = [point.dq[joint_index] for point in profile.points]
        assert max(velocities) > 0.01
        assert min(velocities) < -0.01


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


def test_sdk_joint_backend_restores_motion_gain_before_sending_trajectory():
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
    assert backend._controller.calls == ["set_gain", "set_gain", "set_gain", "set_joint_traj"]
    assert backend._controller.gain.kp() == pytest.approx(backend._controller.config.default_kp)
    assert backend._controller.gain.kd() == pytest.approx(backend._controller.config.default_kd)
    assert backend._controller.joint_traj is not None
    assert backend._controller.joint_traj[0].timestamp == pytest.approx(10.2)
    assert backend._controller.joint_traj[1].timestamp == pytest.approx(10.3)


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
    )

    assert response.status.value == "completed"
    assert (tmp_path / "processed" / "processed_samples.csv").is_file()
    assert (tmp_path / "processed" / "tool_handoff.md").is_file()
    assert (tmp_path / "processed" / "lerobot_contract.json").is_file()
    assert (tmp_path / "processed" / "pinocchio_regressor_skeleton.py").is_file()
    with (tmp_path / "processed" / "lerobot_contract.json").open(encoding="utf-8") as file:
        assert json.load(file)["schema"] == "lerobot-compatible"
    handoff = (tmp_path / "processed" / "tool_handoff.md").read_text(encoding="utf-8")
    skeleton = (tmp_path / "processed" / "pinocchio_regressor_skeleton.py").read_text(encoding="utf-8")
    assert "Pinocchio" in handoff
    assert "URDFly" in handoff
    assert "`q_proc_* / dq_proc_* / ddq_proc_* / tau_proc_*`" in handoff
    assert 'row[f"q_proc_{index}"]' in skeleton
    assert 'row[f"dq_proc_{index}"]' in skeleton
    assert 'row[f"ddq_proc_{index}"]' in skeleton
    assert 'row[f"tau_proc_{index}"]' in skeleton


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
    assert payload["detail"]["metadata"]["amplitude_rad"] == pytest.approx(0.12)
    assert payload["detail"]["metadata"]["segment_duration_s"] == pytest.approx(6.0)
    assert payload["detail"]["metadata"]["dwell_s"] == pytest.approx(1.0)
    assert payload["detail"]["duration_s"] == pytest.approx(168.0)


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
