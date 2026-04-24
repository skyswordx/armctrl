"""真实 SDK adapter 的状态切换测试。"""

from types import SimpleNamespace

import pytest

import armctrl.adapters.arx5.sdk as sdk_module
from armctrl.adapters.arx5.sdk import Arx5SDKAdapter
from armctrl.calibration.models import GripperCalibration
from armctrl.calibration.store import GripperCalibrationStore
from armctrl.protocol.enums import ArmMode
from armctrl.protocol.models import MoveEEFRequest


class _FakeGain:
    """模拟 SDK 的 `Gain`，只保留本测试需要的 kp/kd 和夹爪增益。"""

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
    """模拟 SDK 的 `EEFState`，支持 `pose_6d()[:] = ...` 这种写法。"""

    def __init__(self) -> None:
        self._pose = [0.3, 0.0, 0.2, 0.0, 0.0, 0.0]
        self.gripper_pos = 0.0
        self.gripper_vel = 0.0
        self.gripper_torque = 0.0
        self.timestamp = 0.0

    def pose_6d(self):
        return self._pose


class _FakeJointState:
    """模拟 SDK 的 `JointState`，供 adapter 转成内部状态模型。"""

    gripper_pos = 0.0
    gripper_vel = 0.0
    gripper_torque = 0.0
    timestamp = 0.0

    def pos(self):
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def vel(self):
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def torque(self):
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class _FakeController:
    """模拟 SDK cartesian controller 的增益状态机。"""

    def __init__(self) -> None:
        self.config = SimpleNamespace(
            default_kp=[200.0, 200.0, 200.0, 120.0, 80.0, 60.0],
            default_kd=[5.0, 5.0, 5.0, 1.0, 1.0, 1.0],
            default_gripper_kp=5.0,
            default_gripper_kd=0.2,
            controller_dt=0.002,
            gravity_compensation=True,
        )
        self.gain = _FakeGain(6)
        self.calls: list[str] = []
        self.eef_commands: list[_FakeEEFState] = []

    def get_controller_config(self):
        return self.config

    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        self.calls.append("set_gain")
        self.gain = gain

    def set_to_damping(self):
        damping_gain = _FakeGain(6)
        damping_gain._kd = list(self.config.default_kd)
        self.set_gain(damping_gain)
        self.calls.append("set_to_damping")

    def set_eef_cmd(self, command):
        self.calls.append("set_eef_cmd")
        self.eef_commands.append(command)

    def get_timestamp(self):
        return 10.0

    def get_joint_state(self):
        return _FakeJointState()

    def get_eef_state(self):
        return _FakeEEFState()


class _FakeSDK:
    Gain = _FakeGain
    EEFState = _FakeEEFState


class _FakeRobotConfigFactory:
    @classmethod
    def get_instance(cls):
        return cls()

    def get_config(self, model):
        return SimpleNamespace(model=model, joint_dof=6, urdf_path="/sdk/models/X5.urdf")


class _FakeControllerConfigFactory:
    @classmethod
    def get_instance(cls):
        return cls()

    def get_config(self, name, joint_dof):
        return SimpleNamespace(name=name, joint_dof=joint_dof, gravity_compensation=False)


class _FakeConnectController(_FakeController):
    def __init__(self, robot_config, controller_config, interface) -> None:
        super().__init__()
        self.robot_config = robot_config
        self.controller_config = controller_config
        self.interface = interface


class _FakeConnectSDK(_FakeSDK):
    RobotConfigFactory = _FakeRobotConfigFactory
    ControllerConfigFactory = _FakeControllerConfigFactory
    last_controller: _FakeConnectController | None = None

    @classmethod
    def Arx5CartesianController(cls, robot_config, controller_config, interface):
        cls.last_controller = _FakeConnectController(robot_config, controller_config, interface)
        return cls.last_controller


def test_sdk_move_eef_ramps_default_motion_gain_after_damping():
    # SDK 的 set_to_damping 会把 kp 置零。
    # adapter 再次发送 EEF 命令前必须恢复 cartesian 默认增益，
    # 否则 set_eef_cmd 只改插值目标，真实电机仍停留在阻尼控制。
    adapter = Arx5SDKAdapter(resume_gain_duration_s=0.006, sleep_fn=lambda _: None)
    controller = _FakeController()
    adapter._sdk = _FakeSDK()
    adapter._controller = controller
    adapter._mode = ArmMode.IDLE

    damping_response = adapter.damping()
    assert damping_response.status.value == "completed"
    assert controller.gain.kp() == [0.0] * 6
    set_gain_calls_before = controller.calls.count("set_gain")

    response = adapter.move_eef(
        MoveEEFRequest(
            pose_6d=(0.31, 0.0, 0.2, 0.0, 0.0, 0.0),
            gripper_pos=0.01,
        )
    )

    assert response.status.value == "completed"
    assert controller.calls.count("set_gain") == set_gain_calls_before + 3
    assert controller.calls[-1] == "set_eef_cmd"
    assert controller.gain.kp() == pytest.approx(controller.config.default_kp)
    assert controller.gain.gripper_kp == pytest.approx(controller.config.default_gripper_kp)


def test_sdk_zero_gravity_drag_syncs_current_pose_and_sets_low_gain():
    # 进入 zero_gravity_drag 前要先把目标同步到当前姿态，
    # 再把 gain 拉到很低的 profile，而不是直接掉回纯 damping。
    adapter = Arx5SDKAdapter(resume_gain_duration_s=0.006, sleep_fn=lambda _: None)
    controller = _FakeController()
    adapter._sdk = _FakeSDK()
    adapter._controller = controller
    adapter._mode = ArmMode.IDLE

    response = adapter.zero_gravity_drag()

    assert response.status.value == "completed"
    assert response.state is not None
    assert response.state.mode.value == "zero_gravity_drag"
    assert controller.calls[0] == "set_eef_cmd"
    assert controller.gain.kp() == pytest.approx([10.0, 10.0, 10.0, 6.0, 4.0, 3.0])
    assert controller.gain.kd() == pytest.approx([0.75, 0.75, 0.75, 0.15, 0.15, 0.15])
    assert response.detail["gravity_compensation_enabled"] is True


def test_sdk_connect_prefers_project_x5_camera_urdf_by_default(monkeypatch, tmp_path):
    # X5 是当前项目的带相机主模型。
    # 调用方没有显式传 URDF 时，adapter 自动优先使用项目侧模型，避免继续读 SDK wheel 内的零 payload 版本。
    project_urdf = tmp_path / "X5_camera.urdf"
    project_urdf.write_text("<robot name='x5_camera'/>", encoding="utf-8")
    monkeypatch.setattr(Arx5SDKAdapter, "_DEFAULT_X5_CAMERA_URDF", project_urdf)
    monkeypatch.setattr(sdk_module.importlib, "import_module", lambda name: _FakeConnectSDK)

    adapter = Arx5SDKAdapter(model="X5")
    response = adapter.connect()

    assert response.status.value == "completed"
    assert _FakeConnectSDK.last_controller is not None
    assert _FakeConnectSDK.last_controller.robot_config.urdf_path == str(project_urdf)


def test_sdk_connect_keeps_sdk_urdf_for_non_x5_without_explicit_path(monkeypatch, tmp_path):
    # 只有 X5 自动使用项目相机模型，其他模型继续保持 SDK 配置。
    project_urdf = tmp_path / "X5_camera.urdf"
    project_urdf.write_text("<robot name='x5_camera'/>", encoding="utf-8")
    monkeypatch.setattr(Arx5SDKAdapter, "_DEFAULT_X5_CAMERA_URDF", project_urdf)
    monkeypatch.setattr(sdk_module.importlib, "import_module", lambda name: _FakeConnectSDK)

    adapter = Arx5SDKAdapter(model="L5")
    response = adapter.connect()

    assert response.status.value == "completed"
    assert _FakeConnectSDK.last_controller is not None
    assert _FakeConnectSDK.last_controller.robot_config.urdf_path == "/sdk/models/X5.urdf"


def test_sdk_connect_explicit_urdf_path_overrides_project_default(monkeypatch, tmp_path):
    # 显式路径优先级最高，方便 bringup 时 A/B 比较原模型、相机模型和临时模型。
    project_urdf = tmp_path / "X5_camera.urdf"
    explicit_urdf = tmp_path / "custom.urdf"
    project_urdf.write_text("<robot name='x5_camera'/>", encoding="utf-8")
    explicit_urdf.write_text("<robot name='custom'/>", encoding="utf-8")
    monkeypatch.setattr(Arx5SDKAdapter, "_DEFAULT_X5_CAMERA_URDF", project_urdf)
    monkeypatch.setattr(sdk_module.importlib, "import_module", lambda name: _FakeConnectSDK)

    adapter = Arx5SDKAdapter(model="X5", urdf_path=str(explicit_urdf))
    response = adapter.connect()

    assert response.status.value == "completed"
    assert _FakeConnectSDK.last_controller is not None
    assert _FakeConnectSDK.last_controller.robot_config.urdf_path == str(explicit_urdf.resolve())


def test_sdk_connect_applies_saved_gripper_calibration(monkeypatch, tmp_path):
    # 旧的临时 CLI 覆盖参数已经删除。
    # 现在应当由项目侧标定文件在 connect 阶段自动写入 SDK robot_config。
    project_urdf = tmp_path / "X5_camera.urdf"
    project_urdf.write_text("<robot name='x5_camera'/>", encoding="utf-8")
    monkeypatch.setattr(Arx5SDKAdapter, "_DEFAULT_X5_CAMERA_URDF", project_urdf)
    monkeypatch.setattr(sdk_module.importlib, "import_module", lambda name: _FakeConnectSDK)
    store = GripperCalibrationStore(tmp_path / "calibration")
    store.save(
        GripperCalibration(
            model="X5",
            gripper_open_readout=-3.4,
            gripper_width=0.082,
            source="test",
        )
    )

    adapter = Arx5SDKAdapter(model="X5", calibration_store=store)
    response = adapter.connect()

    assert response.status.value == "completed"
    assert _FakeConnectSDK.last_controller is not None
    assert _FakeConnectSDK.last_controller.robot_config.gripper_open_readout == pytest.approx(-3.4)
    assert _FakeConnectSDK.last_controller.robot_config.gripper_width == pytest.approx(0.082)
