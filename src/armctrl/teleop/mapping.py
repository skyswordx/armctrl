from __future__ import annotations

from dataclasses import dataclass, field

from armctrl.protocol.enums import DebugProfileName
from armctrl.protocol.models import TeleopCommand
from armctrl.teleop.filters import LowPassFilter

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
    event_type: int
    code: int
    value: int
    timestamp: float = 0.0


@dataclass
class XboxState:
    axes: dict[str, float] = field(default_factory=dict)
    buttons: dict[str, bool] = field(default_factory=dict)

    def update(self, event: XboxInputEvent) -> None:
        if event.event_type == 3 and event.code in AXIS_NAMES:
            self.axes[AXIS_NAMES[event.code]] = normalize_axis(event.code, event.value)
        elif event.event_type == 1 and event.code in BUTTON_NAMES:
            self.buttons[BUTTON_NAMES[event.code]] = bool(event.value)


def normalize_axis(code: int, value: int) -> float:
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
    def __init__(
        self,
        max_translation_step_m: float = 0.002,
        max_rotation_step_rad: float = 0.02,
        max_gripper_step: float = 0.0015,
        deadzone: float = 0.12,
        filter_alpha: float = 0.45,
    ) -> None:
        self.max_translation_step_m = max_translation_step_m
        self.max_rotation_step_rad = max_rotation_step_rad
        self.max_gripper_step = max_gripper_step
        self.deadzone = deadzone
        self._filters = {
            "x": LowPassFilter(filter_alpha),
            "y": LowPassFilter(filter_alpha),
            "z": LowPassFilter(filter_alpha),
            "roll": LowPassFilter(filter_alpha),
            "pitch": LowPassFilter(filter_alpha),
            "yaw": LowPassFilter(filter_alpha),
            "gripper": LowPassFilter(filter_alpha),
        }

    def to_command(self, state: XboxState, now: float) -> TeleopCommand:
        if state.buttons.get("BTN_A", False):
            return TeleopCommand(timestamp=now, debug_profile=DebugProfileName.LOW_GAIN_PASSIVE.value)
        if state.buttons.get("BTN_B", False):
            return TeleopCommand(timestamp=now, debug_profile=DebugProfileName.COMPLIANCE_SLOW.value)
        if state.buttons.get("BTN_X", False):
            return TeleopCommand(timestamp=now, debug_profile=DebugProfileName.DAMPING.value)
        if state.buttons.get("BTN_Y", False):
            return TeleopCommand(timestamp=now, debug_profile=DebugProfileName.RESET_HOME.value)
        if state.buttons.get("BTN_SELECT", False):
            return TeleopCommand(
                timestamp=now,
                debug_profile=DebugProfileName.GRAVITY_COMPENSATION_STARTUP.value,
            )

        deadman = state.buttons.get("BTN_TR", False)
        if not deadman:
            self._reset_filters()
            return TeleopCommand(deadman=False, timestamp=now)

        x = self._filtered("x", -state.axes.get("ABS_Y", 0.0)) * self.max_translation_step_m
        y = self._filtered("y", state.axes.get("ABS_X", 0.0)) * self.max_translation_step_m
        right_y_axis = state.axes.get("ABS_RZ")
        right_x_axis = state.axes.get("ABS_Z")
        if right_y_axis is None and "ABS_RY" in state.axes:
            right_y_axis = state.axes.get("ABS_RY", 0.0)
        if right_x_axis is None and "ABS_RX" in state.axes:
            right_x_axis = state.axes.get("ABS_RX", 0.0)
        z = self._filtered("z", -(right_y_axis or 0.0)) * self.max_translation_step_m
        roll = self._filtered("roll", state.axes.get("ABS_HAT0X", 0.0)) * self.max_rotation_step_rad
        pitch = self._filtered("pitch", -state.axes.get("ABS_HAT0Y", 0.0)) * self.max_rotation_step_rad
        yaw = self._filtered("yaw", right_x_axis or 0.0) * self.max_rotation_step_rad
        if "ABS_GAS" in state.axes or "ABS_BRAKE" in state.axes:
            gripper_axis = state.axes.get("ABS_GAS", 0.0) - state.axes.get("ABS_BRAKE", 0.0)
        else:
            gripper_axis = state.axes.get("ABS_Z", -1.0) - state.axes.get("ABS_RZ", -1.0)
        gripper = self._filtered("gripper", gripper_axis / 2.0) * self.max_gripper_step
        return TeleopCommand(
            translation_m=(x, y, z),
            rotation_rad=(roll, pitch, yaw),
            gripper_delta=gripper,
            deadman=True,
            timestamp=now,
        )

    def _filtered(self, name: str, value: float) -> float:
        if abs(value) < self.deadzone:
            value = 0.0
        else:
            sign = 1.0 if value > 0 else -1.0
            value = sign * ((abs(value) - self.deadzone) / (1.0 - self.deadzone))
        return self._filters[name].update(value)

    def _reset_filters(self) -> None:
        for axis_filter in self._filters.values():
            axis_filter.reset()
