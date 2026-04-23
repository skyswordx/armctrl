"""真实 SDK adapter 的状态切换测试。"""

from types import SimpleNamespace

import pytest

from armctrl.adapters.arx5.sdk import Arx5SDKAdapter
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


def test_sdk_move_eef_restores_default_motion_gain_after_damping():
    # SDK 的 set_to_damping 会把 kp 置零。
    # adapter 再次发送 EEF 命令前必须恢复 cartesian 默认增益，
    # 否则 set_eef_cmd 只改插值目标，真实电机仍停留在阻尼控制。
    adapter = Arx5SDKAdapter()
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
    assert controller.calls.count("set_gain") == set_gain_calls_before + 1
    assert controller.calls[-1] == "set_eef_cmd"
    assert controller.gain.kp() == pytest.approx(controller.config.default_kp)
    assert controller.gain.gripper_kp == pytest.approx(controller.config.default_gripper_kp)
