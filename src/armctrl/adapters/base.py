from __future__ import annotations

from typing import Protocol

from armctrl.protocol.models import (
    CommandResponse,
    DebugProfileRequest,
    MoveEEFRequest,
    RobotState,
)


class ArmAdapter(Protocol):
    # `name` 不是装饰性字段，而是状态快照和调试 UI 的关键标识。
    name: str

    # 连接阶段负责创建底层资源，但对上层只暴露统一的响应模型。
    def connect(self) -> CommandResponse:
        ...

    # 所有读状态操作都回到统一的 `RobotState`，避免 CLI / GUI 各自拼字段。
    def get_state(self) -> RobotState:
        ...

    # home pose 是 teleop 初始化积分目标的重要参考点。
    def get_home_pose(self) -> tuple[float, ...]:
        ...

    # 上层发送的是“业务请求”，适配器负责翻译成 SDK 或 fake 设备能理解的命令。
    def move_eef(self, request: MoveEEFRequest) -> CommandResponse:
        ...

    # 调试 profile 统一在这里进入硬件层，避免 CLI 直接触碰 SDK 细节。
    def apply_debug_profile(self, request: DebugProfileRequest) -> CommandResponse:
        ...

    # damping 是机械臂最常见的安全落态，所以单独抽成稳定接口。
    def damping(self) -> CommandResponse:
        ...

    # cancel 的语义不是“撤销一个 future”，而是把系统切回安全停止态。
    def cancel(self) -> CommandResponse:
        ...
