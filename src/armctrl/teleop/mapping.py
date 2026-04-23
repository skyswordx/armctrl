from __future__ import annotations

from dataclasses import dataclass, field

from armctrl.protocol.enums import DebugProfileName
from armctrl.protocol.models import TeleopCommand
from armctrl.teleop.filters import LowPassFilter

# 这里保存 Linux input 子系统里常见的轴码 -> 语义名映射。
# 统一成字符串后，后面的映射逻辑和 GUI 展示都不再依赖原始整数码。
AXIS_NAMES = {
    0: "ABS_X",
    1: "ABS_Y",
    2: "ABS_Z",
    3: "ABS_RX",
    4: "ABS_RY",
    5: "ABS_RZ",
    9: "ABS_GAS",
    10: "ABS_BRAKE",
    16: "ABS_HAT0X",
    17: "ABS_HAT0Y",
}

# 按钮同理，统一在这里做码表翻译。
BUTTON_NAMES = {
    304: "BTN_A",
    305: "BTN_B",
    307: "BTN_X",
    308: "BTN_Y",
    310: "BTN_TL",
    311: "BTN_TR",
    314: "BTN_SELECT",
    315: "BTN_START",
    316: "BTN_MODE",
    317: "BTN_THUMBL",
    318: "BTN_THUMBR",
}


@dataclass
class XboxInputEvent:
    """对 Linux input event 的轻量封装。"""

    event_type: int
    code: int
    value: int
    timestamp: float = 0.0


@dataclass
class XboxState:
    """手柄当前状态快照。

    这里采用“增量事件 -> 全量状态”的做法：
    每收到一个事件就更新一次内部状态字典，
    后续映射函数永远读取“当前完整状态”，而不是单个事件。
    """

    axes: dict[str, float] = field(default_factory=dict)
    buttons: dict[str, bool] = field(default_factory=dict)

    def update(self, event: XboxInputEvent) -> None:
        # event_type == 3 代表轴事件，event_type == 1 代表按钮事件。
        if event.event_type == 3 and event.code in AXIS_NAMES:
            self.axes[AXIS_NAMES[event.code]] = normalize_axis(event.code, event.value)
        elif event.event_type == 1 and event.code in BUTTON_NAMES:
            self.buttons[BUTTON_NAMES[event.code]] = bool(event.value)


def normalize_axis(code: int, value: int) -> float:
    """把驱动上报的原始整数归一化到 [-1, 1] 或 [0, 1] 区间。"""

    # 不同轴的原始量程并不完全一致：
    # - GAS / BRAKE 常见是 [0,255] 或 [0,1023]
    # - Z / RZ 在部分手柄上是双向轴
    # - HAT 方向键通常直接给 -1 / 0 / 1
    if code in (9, 10):
        max_value = 255.0 if 0 <= value <= 255 else 1023.0
        normalized = float(value) / max_value
    elif code in (2, 5):
        max_value = 255.0 if 0 <= value <= 255 else 1023.0
        normalized = (float(value) / max_value) * 2.0 - 1.0
    elif code in (16, 17):
        normalized = float(value)
    elif 0 <= value <= 255:
        normalized = (float(value) - 128.0) / 127.0
    else:
        normalized = float(value) / 32767.0
    return max(-1.0, min(1.0, normalized))


class XboxMapper:
    """把 `XboxState` 映射成 `TeleopCommand`。

    这里先输出速度量，再按 dt 生成“本拍增量”。
    绝对目标位姿仍由 executor 维护，摇杆回中时速度变 0，目标不会回到 0。
    """

    def __init__(
        self,
        max_translation_step_m: float | None = None,
        max_rotation_step_rad: float | None = None,
        max_gripper_step: float | None = None,
        max_translation_speed_mps: float = 0.2,
        max_rotation_speed_radps: float = 1.0,
        max_gripper_speed_mps: float = 0.04,
        default_dt_s: float = 0.01,
        deadzone: float = 0.12,
        filter_alpha: float = 0.45,
    ) -> None:
        self.default_dt_s = default_dt_s
        # 兼容旧调用方：旧参数含义是“100Hz 时每拍最大增量”。
        # 新内部语义统一转成速度，避免控制频率变化导致手感变化。
        self.max_translation_speed_mps = (
            max_translation_step_m / default_dt_s
            if max_translation_step_m is not None
            else max_translation_speed_mps
        )
        self.max_rotation_speed_radps = (
            max_rotation_step_rad / default_dt_s
            if max_rotation_step_rad is not None
            else max_rotation_speed_radps
        )
        self.max_gripper_speed_mps = (
            max_gripper_step / default_dt_s
            if max_gripper_step is not None
            else max_gripper_speed_mps
        )
        self.deadzone = deadzone
        # 每个维度单独滤波，避免一个轴的噪声污染其他控制量。
        self._filters = {
            "x": LowPassFilter(filter_alpha),
            "y": LowPassFilter(filter_alpha),
            "z": LowPassFilter(filter_alpha),
            "roll": LowPassFilter(filter_alpha),
            "pitch": LowPassFilter(filter_alpha),
            "yaw": LowPassFilter(filter_alpha),
            "gripper": LowPassFilter(filter_alpha),
        }

    def to_command(self, state: XboxState, now: float, dt_s: float | None = None) -> TeleopCommand:
        dt = self.default_dt_s if dt_s is None else float(dt_s)
        # 先处理“模式类按键”，它们优先级高于普通运动控制。
        if state.buttons.get("BTN_A", False):
            return TeleopCommand(timestamp=now, dt_s=dt, debug_profile=DebugProfileName.LOW_GAIN_PASSIVE.value)
        if state.buttons.get("BTN_B", False):
            return TeleopCommand(timestamp=now, dt_s=dt, debug_profile=DebugProfileName.COMPLIANCE_SLOW.value)
        if state.buttons.get("BTN_X", False):
            return TeleopCommand(timestamp=now, dt_s=dt, debug_profile=DebugProfileName.DAMPING.value)
        if state.buttons.get("BTN_Y", False):
            return TeleopCommand(timestamp=now, dt_s=dt, debug_profile=DebugProfileName.RESET_HOME.value)
        if state.buttons.get("BTN_SELECT", False):
            return TeleopCommand(
                timestamp=now,
                dt_s=dt,
                debug_profile=DebugProfileName.GRAVITY_COMPENSATION_STARTUP.value,
            )

        # deadman 没按住时，不应该输出运动增量。
        deadman = state.buttons.get("BTN_TR", False)
        if not deadman:
            self._reset_filters()
            return TeleopCommand(deadman=False, timestamp=now, dt_s=dt)

        # 左摇杆负责平移 x / y。
        x_velocity = self._filtered("x", -state.axes.get("ABS_Y", 0.0)) * self.max_translation_speed_mps
        y_velocity = self._filtered("y", state.axes.get("ABS_X", 0.0)) * self.max_translation_speed_mps
        # 当前设备上，右摇杆主用字段是 ABS_Z / ABS_RZ。
        # 仍保留对 ABS_RX / ABS_RY 的回退兼容，避免换设备后完全失效。
        right_y_axis = state.axes.get("ABS_RZ")
        right_x_axis = state.axes.get("ABS_Z")
        if right_y_axis is None and "ABS_RY" in state.axes:
            right_y_axis = state.axes.get("ABS_RY", 0.0)
        if right_x_axis is None and "ABS_RX" in state.axes:
            right_x_axis = state.axes.get("ABS_RX", 0.0)
        z_velocity = self._filtered("z", -(right_y_axis or 0.0)) * self.max_translation_speed_mps
        # 方向键映射到 roll / pitch，是为了补齐完整 6D pose 控制。
        roll_velocity = self._filtered("roll", state.axes.get("ABS_HAT0X", 0.0)) * self.max_rotation_speed_radps
        pitch_velocity = self._filtered("pitch", -state.axes.get("ABS_HAT0Y", 0.0)) * self.max_rotation_speed_radps
        yaw_velocity = self._filtered("yaw", right_x_axis or 0.0) * self.max_rotation_speed_radps
        # 当前这只手柄上，GAS / BRAKE 的体感方向和传统命名相反。
        # 实际控制上，以用户手感一致性优先。
        if "ABS_GAS" in state.axes or "ABS_BRAKE" in state.axes:
            gripper_axis = state.axes.get("ABS_GAS", 0.0) - state.axes.get("ABS_BRAKE", 0.0)
        else:
            gripper_axis = state.axes.get("ABS_Z", -1.0) - state.axes.get("ABS_RZ", -1.0)
        gripper_velocity = self._filtered("gripper", gripper_axis / 2.0) * self.max_gripper_speed_mps
        return TeleopCommand(
            translation_m=(x_velocity * dt, y_velocity * dt, z_velocity * dt),
            rotation_rad=(roll_velocity * dt, pitch_velocity * dt, yaw_velocity * dt),
            gripper_delta=gripper_velocity * dt,
            translation_velocity_mps=(x_velocity, y_velocity, z_velocity),
            rotation_velocity_radps=(roll_velocity, pitch_velocity, yaw_velocity),
            gripper_velocity_mps=gripper_velocity,
            dt_s=dt,
            deadman=True,
            timestamp=now,
        )

    def _filtered(self, name: str, value: float) -> float:
        # 这里先做 deadzone，再做线性重映射，最后送进低通滤波。
        # deadzone 外的重映射公式：
        #   v' = sign(v) * (|v| - dz) / (1 - dz)
        if abs(value) < self.deadzone:
            value = 0.0
        else:
            sign = 1.0 if value > 0 else -1.0
            value = sign * ((abs(value) - self.deadzone) / (1.0 - self.deadzone))
        return self._filters[name].update(value)

    def _reset_filters(self) -> None:
        # 一次性重置全部滤波器，保证重新接管时从“当前手感”重新起步。
        for axis_filter in self._filters.values():
            axis_filter.reset()
