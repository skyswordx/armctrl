from __future__ import annotations

import time

from armctrl.protocol.enums import ArmMode, CommandStatus, DebugProfileName
from armctrl.protocol.models import (
    CommandResponse,
    DebugProfileRequest,
    EEFStateModel,
    JointStateModel,
    MoveEEFRequest,
    RobotState,
)


class FakeArx5Adapter:
    """不接硬件的 ARX5 适配器。

    这个类的作用不是“模拟完整机器人动力学”，而是：
    - 给执行器和 CLI 提供稳定的无硬件测试入口；
    - 验证命令生命周期、状态机和安全逻辑；
    - 把测试关注点放在软件协议，而不是放在物理模型精度。
    """

    name = "fake"

    def __init__(self) -> None:
        now = time.monotonic()
        # 这里的 home pose 是一组人工选定的安全、直观的教学位姿。
        # fake adapter 不做 IK，因此直接保存末端 6D 位姿即可。
        self._home_pose = (0.3, 0.0, 0.2, 0.0, 0.0, 0.0)
        self._eef = EEFStateModel(self._home_pose, timestamp=now)
        # fake joint 状态全部从零开始，目的是让测试断言尽量稳定、可预测。
        self._joint = JointStateModel(
            pos=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            vel=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            torque=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            timestamp=now,
        )
        self._mode = ArmMode.IDLE
        self.connected = True

    def connect(self) -> CommandResponse:
        # fake 环境下 connect 是幂等的。
        # 这样 CLI 多次构建 executor 也不会引入副作用。
        self.connected = True
        self._mode = ArmMode.IDLE
        return CommandResponse(CommandStatus.COMPLETED, "fake adapter connected", state=self.get_state())

    def get_state(self) -> RobotState:
        # 注意这里返回的是快照对象，而不是内部字典。
        # 这样上层不需要知道 fake / sdk 的内部存储方式。
        return RobotState(
            mode=self._mode,
            joint=self._joint,
            eef=self._eef,
            connected=self.connected,
            adapter=self.name,
        )

    def get_home_pose(self) -> tuple[float, ...]:
        return self._home_pose

    def move_eef(self, request: MoveEEFRequest) -> CommandResponse:
        # `plan_only=True` 是业务层的“只验证、不执行”语义。
        # fake adapter 也必须遵守这套契约，否则测试会和真实链路脱节。
        if not request.plan_only:
            # fake 版本直接把目标写成新的末端状态，不做轨迹插值或动力学。
            self._eef = EEFStateModel(
                pose_6d=request.pose_6d,
                gripper_pos=request.gripper_pos,
                timestamp=time.monotonic() + request.preview_time_s,
            )
            self._mode = ArmMode.TELEOP
        return CommandResponse(
            CommandStatus.COMPLETED,
            "fake eef request accepted",
            command_id=request.command_id,
            state=self.get_state(),
        )

    def apply_debug_profile(self, request: DebugProfileRequest) -> CommandResponse:
        # fake adapter 只保留对软件主链路有意义的最小行为。
        # 例如 damping / reset_home 对上层状态机有影响，需要模拟；
        # 其他 profile 则只返回“已接受”，不再硬造复杂物理效果。
        if request.name == DebugProfileName.DAMPING:
            return self.damping()
        if request.name == DebugProfileName.RESET_HOME and not request.plan_only:
            self._eef = EEFStateModel(self._home_pose, timestamp=time.monotonic())
            self._mode = ArmMode.IDLE
        return CommandResponse(
            CommandStatus.COMPLETED,
            f"fake debug profile {request.name.value} accepted",
            command_id=request.command_id,
            state=self.get_state(),
        )

    def damping(self) -> CommandResponse:
        # damping 是 fake 世界里的“停止继续发送运动目标，并进入安全落态”。
        self._mode = ArmMode.DAMPING
        return CommandResponse(CommandStatus.COMPLETED, "fake damping enabled", state=self.get_state())

    def cancel(self) -> CommandResponse:
        # 这里直接落到 damping，是对真实硬件链路的简化映射。
        self._mode = ArmMode.DAMPING
        return CommandResponse(CommandStatus.CANCELLED, "fake command cancelled", state=self.get_state())
