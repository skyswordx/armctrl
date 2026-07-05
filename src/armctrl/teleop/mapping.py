from __future__ import annotations

from dataclasses import dataclass, field

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


@dataclass(frozen=True)
class TeleopRuntimeCommand:
    """Runtime-facing teleop command contract.

    This stays independent of the old feature/subsystem executor. Motion commands
    become eef_twist inputs; profile requests become runtime-owned teach/teleop
    profile commands.
    """

    deadman: bool
    timestamp: float
    dt_s: float
    linear_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_rps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gripper_velocity_mps: float = 0.0
    profile: str | None = None
    reason: str | None = None

    @property
    def has_motion(self) -> bool:
        return any(abs(value) > 0.0 for value in self.linear_mps + self.angular_rps)

    def to_eef_twist_payload(self, *, frame: str = "eef_link") -> dict[str, object]:
        return {
            "kind": "eef_twist",
            "frame": frame,
            "linear_mps": list(self.linear_mps),
            "angular_rps": list(self.angular_rps),
            "control_period_s": float(self.dt_s),
            "deadman": bool(self.deadman),
            "source": "teleop_xbox",
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
    """Map an Xbox-like Linux input snapshot to runtime-owned commands."""

    def __init__(
        self,
        *,
        max_translation_speed_mps: float = 0.2,
        max_rotation_speed_radps: float = 1.0,
        max_gripper_speed_mps: float = 0.04,
        default_dt_s: float = 0.02,
        deadzone: float = 0.12,
        filter_alpha: float = 0.45,
        deadman_release_profile: str = "zero_gravity_drag",
    ) -> None:
        self.max_translation_speed_mps = float(max_translation_speed_mps)
        self.max_rotation_speed_radps = float(max_rotation_speed_radps)
        self.max_gripper_speed_mps = float(max_gripper_speed_mps)
        self.default_dt_s = float(default_dt_s)
        self.deadzone = float(deadzone)
        self.deadman_release_profile = str(deadman_release_profile)
        self._filters = {
            "x": LowPassFilter(filter_alpha),
            "y": LowPassFilter(filter_alpha),
            "z": LowPassFilter(filter_alpha),
            "roll": LowPassFilter(filter_alpha),
            "pitch": LowPassFilter(filter_alpha),
            "yaw": LowPassFilter(filter_alpha),
            "gripper": LowPassFilter(filter_alpha),
        }

    def to_runtime_command(
        self,
        state: XboxState,
        *,
        now: float,
        dt_s: float | None = None,
    ) -> TeleopRuntimeCommand:
        dt = self.default_dt_s if dt_s is None else float(dt_s)
        if state.buttons.get("BTN_A", False):
            return TeleopRuntimeCommand(
                deadman=False,
                timestamp=now,
                dt_s=dt,
                profile="teleop",
                reason="BTN_A restores teleop gain",
            )
        if state.buttons.get("BTN_X", False):
            return TeleopRuntimeCommand(
                deadman=False,
                timestamp=now,
                dt_s=dt,
                profile="damping",
                reason="BTN_X requests damping",
            )

        deadman = state.buttons.get("BTN_TR", False)
        if not deadman:
            self._reset_filters()
            return TeleopRuntimeCommand(
                deadman=False,
                timestamp=now,
                dt_s=dt,
                profile=self.deadman_release_profile,
                reason="deadman released",
            )

        right_y_axis = state.axes.get("ABS_RZ")
        right_x_axis = state.axes.get("ABS_Z")
        if right_y_axis is None and "ABS_RY" in state.axes:
            right_y_axis = state.axes.get("ABS_RY", 0.0)
        if right_x_axis is None and "ABS_RX" in state.axes:
            right_x_axis = state.axes.get("ABS_RX", 0.0)
        if "ABS_GAS" in state.axes or "ABS_BRAKE" in state.axes:
            gripper_axis = state.axes.get("ABS_GAS", 0.0) - state.axes.get("ABS_BRAKE", 0.0)
        else:
            gripper_axis = state.axes.get("ABS_Z", -1.0) - state.axes.get("ABS_RZ", -1.0)
        return TeleopRuntimeCommand(
            deadman=True,
            timestamp=now,
            dt_s=dt,
            linear_mps=(
                self._filtered("x", -state.axes.get("ABS_Y", 0.0))
                * self.max_translation_speed_mps,
                self._filtered("y", state.axes.get("ABS_X", 0.0))
                * self.max_translation_speed_mps,
                self._filtered("z", -(right_y_axis or 0.0))
                * self.max_translation_speed_mps,
            ),
            angular_rps=(
                self._filtered("roll", state.axes.get("ABS_HAT0X", 0.0))
                * self.max_rotation_speed_radps,
                self._filtered("pitch", -state.axes.get("ABS_HAT0Y", 0.0))
                * self.max_rotation_speed_radps,
                self._filtered("yaw", right_x_axis or 0.0)
                * self.max_rotation_speed_radps,
            ),
            gripper_velocity_mps=(
                self._filtered("gripper", gripper_axis / 2.0)
                * self.max_gripper_speed_mps
            ),
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

