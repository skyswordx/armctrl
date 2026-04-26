"""夹爪标定模块测试。

这里重点锁住三件事：
1. 标定文件能够稳定保存和读取；
2. SDK adapter / joint backend 会自动应用项目侧标定；
3. 交互式 wizard 会复用 SDK 标定流程，并把 SDK 终端打印值保存回项目配置。
"""

from types import SimpleNamespace

import pytest

import armctrl.identification.backends as backends_module
from armctrl.calibration.gripper import GripperCalibrationService, derive_gripper_motor_readout
from armctrl.calibration.models import GripperCalibration
from armctrl.calibration.store import GripperCalibrationStore
from armctrl.identification.backends import Arx5JointRobotIO


def test_gripper_calibration_store_round_trip(tmp_path):
    store = GripperCalibrationStore(tmp_path / "gripper")
    calibration = GripperCalibration(
        model="X5",
        gripper_open_readout=-3.4,
        gripper_width=0.082,
        source="unit_test",
        notes="round trip",
    )

    path = store.save(calibration)
    loaded = store.load("X5")

    assert path.is_file()
    assert loaded == calibration


def test_derive_gripper_motor_readout_matches_sdk_formula():
    # 这里直接按 SDK 源码里的换算公式做反推。
    # 只要 `joint_state.gripper_pos` 和 `robot_config` 可用，就能恢复原始电机角度读数。
    joint_state = SimpleNamespace(gripper_pos=0.041)
    robot_config = SimpleNamespace(gripper_width=0.082, gripper_open_readout=-3.4)

    readout = derive_gripper_motor_readout(joint_state, robot_config)

    assert readout == pytest.approx(-1.7)


class _FakeJointController:
    def __init__(self, robot_config, controller_config, interface) -> None:
        self.robot_config = robot_config
        self.controller_config = controller_config
        self.interface = interface
        self.calibrate_calls = 0

    def set_log_level(self, _level) -> None:
        return None

    def calibrate_gripper(self) -> None:
        self.calibrate_calls += 1

    def get_robot_config(self):
        return self.robot_config

    def get_joint_state(self):
        # 这个值按 SDK 公式反推后应得到 -1.7。
        return SimpleNamespace(gripper_pos=0.041)


class _FakeSDK:
    last_controller: _FakeJointController | None = None

    class RobotConfigFactory:
        @classmethod
        def get_instance(cls):
            return cls()

        def get_config(self, model):
            return SimpleNamespace(
                model=model,
                joint_dof=6,
                urdf_path="/sdk/models/X5.urdf",
                gripper_open_readout=3.4,
                gripper_width=0.082,
            )

    class ControllerConfigFactory:
        @classmethod
        def get_instance(cls):
            return cls()

        def get_config(self, name, joint_dof):
            return SimpleNamespace(
                controller_type=name,
                joint_dof=joint_dof,
                gravity_compensation=True,
                background_send_recv=True,
            )

    class LogLevel:
        WARNING = object()

    @classmethod
    def Arx5JointController(cls, robot_config, controller_config, interface):
        cls.last_controller = _FakeJointController(robot_config, controller_config, interface)
        return cls.last_controller


def test_gripper_calibration_wizard_saves_result_and_bootstraps_from_store(tmp_path):
    store = GripperCalibrationStore(tmp_path / "gripper")
    # 先保存一份 bootstrap 标定，模拟“这台机器如果没有负号就连不上”。
    store.save(
        GripperCalibration(
            model="X5",
            gripper_open_readout=-3.4,
            gripper_width=0.082,
            source="bootstrap",
        )
    )
    prompts = iter(["4.97768", "", "现场记录"])
    outputs: list[str] = []
    service = GripperCalibrationService(
        store=store,
        import_module=lambda name: _FakeSDK,
        input_fn=lambda prompt: next(prompts),
        output_fn=outputs.append,
    )

    response = service.run_wizard(model="X5", interface="can0")

    assert response.status.value == "completed"
    assert _FakeSDK.last_controller is not None
    assert _FakeSDK.last_controller.robot_config.gripper_open_readout == pytest.approx(-3.4)
    assert _FakeSDK.last_controller.controller_config.background_send_recv is False
    saved = store.load("X5")
    assert saved is not None
    # wizard 现在只认 SDK 终端打印出来的 fully-open 值，不再从 JointState 反推。
    assert saved.gripper_open_readout == pytest.approx(4.97768)
    assert saved.gripper_width == pytest.approx(0.082)
    assert saved.notes == "现场记录"
    assert outputs


def test_joint_backend_connect_applies_saved_gripper_calibration(monkeypatch, tmp_path):
    store = GripperCalibrationStore(tmp_path / "gripper")
    store.save(
        GripperCalibration(
            model="X5",
            gripper_open_readout=-3.4,
            gripper_width=0.082,
            source="unit_test",
        )
    )
    monkeypatch.setattr(backends_module.importlib, "import_module", lambda name: _FakeSDK)

    backend = Arx5JointRobotIO(model="X5", interface="can0", calibration_store=store)
    response = backend.connect()

    assert response.status.value == "completed"
    assert _FakeSDK.last_controller is not None
    assert _FakeSDK.last_controller.robot_config.gripper_open_readout == pytest.approx(-3.4)
    assert _FakeSDK.last_controller.robot_config.gripper_width == pytest.approx(0.082)
