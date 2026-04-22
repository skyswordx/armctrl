from __future__ import annotations

import json
import queue
import struct
import threading
import time
from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from armctrl.daemon.executor import ArmCommandExecutor
from armctrl.protocol.enums import ErrorCode
from armctrl.protocol.errors import ArmctrlError
from armctrl.protocol.models import CommandResponse, TeleopCommand
from armctrl.teleop.mapping import (
    AXIS_NAMES,
    BUTTON_NAMES,
    XboxInputEvent,
    XboxMapper,
    XboxState,
)

LINUX_INPUT_EVENT = struct.Struct("llHHi")

AXIS_ORDER = (
    "ABS_X",
    "ABS_Y",
    "ABS_Z",
    "ABS_RZ",
    "ABS_GAS",
    "ABS_BRAKE",
    "ABS_HAT0X",
    "ABS_HAT0Y",
)
BUTTON_ORDER = (
    "BTN_A",
    "BTN_B",
    "BTN_X",
    "BTN_Y",
    "BTN_TL",
    "BTN_TR",
    "BTN_SELECT",
    "BTN_START",
    "BTN_MODE",
    "BTN_THUMBL",
    "BTN_THUMBR",
)
CONTROL_HINTS = (
    "RB / BTN_TR：按住后才发送末端 jog，松开进入 damping。",
    "左摇杆上下：末端 x 方向小步移动。",
    "左摇杆左右：末端 y 方向小步移动。",
    "右摇杆上下：末端 z 方向小步移动。",
    "右摇杆左右：末端 yaw 小步旋转。",
    "方向键左右：末端 roll 小步旋转。",
    "方向键上下：末端 pitch 小步旋转。",
    "左右扳机：夹爪开合小步控制。",
    "X / BTN_X：切到 damping。",
    "A / BTN_A：请求 low_gain_passive，需要维护权限。",
    "B / BTN_B：请求 compliance_slow，需要维护权限。",
    "Y / BTN_Y：请求 reset_home，需要维护权限。",
    "SELECT / BTN_SELECT：请求 gravity_compensation_startup，仅用于启动前检查。",
)
CONTROLLER_FIELDS = (
    ("source", "输入设备"),
    ("last_event_name", "最近事件"),
    ("last_event_type", "事件类型"),
    ("last_event_code", "事件码"),
    ("last_event_raw", "原始值"),
    ("last_event_normalized", "归一化值"),
    ("axis_ABS_X", "左摇杆 X"),
    ("axis_ABS_Y", "左摇杆 Y"),
    ("axis_ABS_Z", "右摇杆 X"),
    ("axis_ABS_RZ", "右摇杆 Y"),
    ("axis_ABS_GAS", "右扳机"),
    ("axis_ABS_BRAKE", "左扳机"),
    ("axis_ABS_HAT0X", "方向键 X"),
    ("axis_ABS_HAT0Y", "方向键 Y"),
    ("button_BTN_A", "按钮 A"),
    ("button_BTN_B", "按钮 B"),
    ("button_BTN_X", "按钮 X"),
    ("button_BTN_Y", "按钮 Y"),
    ("button_BTN_TL", "按钮 LB"),
    ("button_BTN_TR", "按钮 RB"),
    ("button_BTN_SELECT", "按钮 SELECT"),
    ("button_BTN_START", "按钮 START"),
    ("button_BTN_MODE", "按钮 MODE"),
    ("button_BTN_THUMBL", "左摇杆按下"),
    ("button_BTN_THUMBR", "右摇杆按下"),
)
CONTROL_FIELDS = (
    ("phase", "阶段"),
    ("response_command_id", "命令 ID"),
    ("command_deadman", "deadman"),
    ("command_debug_profile", "调试配置"),
    ("command_translation_x", "命令 x"),
    ("command_translation_y", "命令 y"),
    ("command_translation_z", "命令 z"),
    ("command_roll", "命令 roll"),
    ("command_pitch", "命令 pitch"),
    ("command_yaw", "命令 yaw"),
    ("command_gripper_delta", "命令夹爪增量"),
    ("response_status", "响应状态"),
    ("response_message", "响应消息"),
    ("response_error_code", "错误码"),
    ("response_error_message", "错误信息"),
    ("response_error_detail", "错误详情"),
    ("state_adapter", "适配器"),
    ("state_connected", "已连接"),
    ("state_mode", "机械臂模式"),
    ("state_timestamp", "状态时间戳"),
    ("state_eef_x", "EEF x"),
    ("state_eef_y", "EEF y"),
    ("state_eef_z", "EEF z"),
    ("state_eef_roll", "EEF roll"),
    ("state_eef_pitch", "EEF pitch"),
    ("state_eef_yaw", "EEF yaw"),
    ("state_gripper_pos", "夹爪位置"),
    ("joint_pos_1", "J1 位置"),
    ("joint_pos_2", "J2 位置"),
    ("joint_pos_3", "J3 位置"),
    ("joint_pos_4", "J4 位置"),
    ("joint_pos_5", "J5 位置"),
    ("joint_pos_6", "J6 位置"),
    ("joint_vel_1", "J1 速度"),
    ("joint_vel_2", "J2 速度"),
    ("joint_vel_3", "J3 速度"),
    ("joint_vel_4", "J4 速度"),
    ("joint_vel_5", "J5 速度"),
    ("joint_vel_6", "J6 速度"),
    ("joint_torque_1", "J1 力矩"),
    ("joint_torque_2", "J2 力矩"),
    ("joint_torque_3", "J3 力矩"),
    ("joint_torque_4", "J4 力矩"),
    ("joint_torque_5", "J5 力矩"),
    ("joint_torque_6", "J6 力矩"),
)


def iter_linux_input_events(device: str | Path) -> Iterator[XboxInputEvent]:
    with Path(device).open("rb") as handle:
        while True:
            chunk = handle.read(LINUX_INPUT_EVENT.size)
            if not chunk:
                return
            sec, usec, event_type, code, value = LINUX_INPUT_EVENT.unpack(chunk)
            yield XboxInputEvent(event_type, code, value, sec + usec / 1_000_000.0)


def iter_jsonl_events(path: str | Path) -> Iterator[XboxInputEvent]:
    import json

    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            payload = json.loads(line)
            yield XboxInputEvent(
                event_type=int(payload["type"]),
                code=int(payload["code"]),
                value=int(payload["value"]),
                timestamp=float(payload.get("timestamp", 0.0)),
            )


class TeleopDashboardSink(Protocol):
    def push(self, controller_snapshot: dict[str, object], control_snapshot: dict[str, object]) -> None: ...

    def close(self) -> None: ...


@dataclass
class XboxDebugRunner:
    executor: ArmCommandExecutor
    mapper: XboxMapper
    events: Iterable[XboxInputEvent]
    rate_hz: float = 100.0
    max_events: int | None = None
    source: str = "unknown"
    dashboard: TeleopDashboardSink | None = None
    stop_event: threading.Event | None = None

    def run(self) -> CommandResponse:
        state = XboxState()
        last_response = self.executor.state()
        last_event: XboxInputEvent | None = None
        last_command: TeleopCommand | None = None
        period_s = 1.0 / self.rate_hz
        processed = 0
        event_queue: queue.Queue[XboxInputEvent] = queue.Queue()
        reader_done = threading.Event()
        reader_errors: list[BaseException] = []

        def read_events() -> None:
            try:
                for event in self.events:
                    event_queue.put(event)
            except BaseException as exc:
                reader_errors.append(exc)
            finally:
                reader_done.set()

        threading.Thread(target=read_events, daemon=True).start()
        stop_requested = False
        next_tick = time.monotonic()
        try:
            while True:
                if self.stop_event is not None and self.stop_event.is_set():
                    break
                had_event = False
                tick_timestamp: float | None = None
                while True:
                    try:
                        event = event_queue.get_nowait()
                    except queue.Empty:
                        break
                    had_event = True
                    state.update(event)
                    last_event = event
                    tick_timestamp = event.timestamp or tick_timestamp or time.monotonic()
                    processed += 1
                    if self.max_events is not None and processed >= self.max_events:
                        stop_requested = True
                        break

                if reader_errors:
                    raise reader_errors[0]
                if had_event or state.buttons.get("BTN_TR", False):
                    now = tick_timestamp or time.monotonic()
                    command = self.mapper.to_command(state, now)
                    last_command = command
                    last_response = self.executor.handle_teleop(command)
                    self._push_snapshot(last_event, state, last_command, last_response, "event" if had_event else "tick")

                if stop_requested:
                    break
                if reader_done.is_set() and event_queue.empty() and not state.buttons.get("BTN_TR", False):
                    break

                next_tick += period_s
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.monotonic()
        finally:
            final_response = self.executor.damping()
            self._push_snapshot(last_event, state, last_command, final_response, "shutdown")
            if self.dashboard is not None:
                self.dashboard.close()
        return final_response

    def _push_snapshot(
        self,
        event: XboxInputEvent | None,
        state: XboxState,
        command: TeleopCommand | None,
        response: CommandResponse,
        phase: str,
    ) -> None:
        if self.dashboard is None:
            return
        self.dashboard.push(
            build_controller_snapshot(self.source, event, state),
            build_control_snapshot(command, response, phase),
        )


def build_controller_snapshot(
    source: str,
    event: XboxInputEvent | None,
    state: XboxState,
) -> dict[str, object]:
    snapshot: dict[str, object] = {
        "source": source,
        "last_event_name": event_label(event) if event is not None else None,
        "last_event_type": event.event_type if event is not None else None,
        "last_event_code": event.code if event is not None else None,
        "last_event_raw": event.value if event is not None else None,
        "last_event_normalized": None,
    }
    if event is not None and event.event_type == 3:
        axis_name = event_label(event)
        if axis_name in state.axes:
            snapshot["last_event_normalized"] = rounded(state.axes[axis_name])
    for axis_name in AXIS_ORDER:
        snapshot[f"axis_{axis_name}"] = rounded(state.axes.get(axis_name, 0.0))
    for button_name in BUTTON_ORDER:
        snapshot[f"button_{button_name}"] = bool(state.buttons.get(button_name, False))
    return snapshot


def build_control_snapshot(
    command: TeleopCommand | None,
    response: CommandResponse,
    phase: str,
) -> dict[str, object]:
    snapshot = build_response_snapshot(response, phase)
    snapshot.update(
        {
            "command_deadman": command.deadman if command is not None else False,
            "command_debug_profile": command.debug_profile if command is not None else None,
            "command_translation_x": rounded(command.translation_m[0]) if command is not None else 0.0,
            "command_translation_y": rounded(command.translation_m[1]) if command is not None else 0.0,
            "command_translation_z": rounded(command.translation_m[2]) if command is not None else 0.0,
            "command_roll": rounded(command.rotation_rad[0]) if command is not None else 0.0,
            "command_pitch": rounded(command.rotation_rad[1]) if command is not None else 0.0,
            "command_yaw": rounded(command.rotation_rad[2]) if command is not None else 0.0,
            "command_gripper_delta": rounded(command.gripper_delta) if command is not None else 0.0,
        }
    )
    return snapshot


def build_response_snapshot(
    response: CommandResponse,
    phase: str,
) -> dict[str, object]:
    pose_6d = response.state.eef.pose_6d if response.state is not None else (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_pos = response.state.joint.pos if response.state is not None else (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_vel = response.state.joint.vel if response.state is not None else (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_torque = response.state.joint.torque if response.state is not None else (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    snapshot: dict[str, object] = {
        "phase": phase,
        "response_command_id": response.command_id,
        "command_deadman": False,
        "command_debug_profile": None,
        "command_translation_x": 0.0,
        "command_translation_y": 0.0,
        "command_translation_z": 0.0,
        "command_roll": 0.0,
        "command_pitch": 0.0,
        "command_yaw": 0.0,
        "command_gripper_delta": 0.0,
        "response_status": response.status.value,
        "response_message": response.message,
        "response_error_code": response.error.code.value if response.error is not None else None,
        "response_error_message": response.error.message if response.error is not None else None,
        "response_error_detail": response.error.detail if response.error is not None else None,
        "state_adapter": response.state.adapter if response.state is not None else None,
        "state_connected": response.state.connected if response.state is not None else False,
        "state_mode": response.state.mode.value if response.state is not None else None,
        "state_timestamp": rounded(response.state.eef.timestamp) if response.state is not None else 0.0,
        "state_eef_x": rounded(pose_6d[0]),
        "state_eef_y": rounded(pose_6d[1]),
        "state_eef_z": rounded(pose_6d[2]),
        "state_eef_roll": rounded(pose_6d[3]),
        "state_eef_pitch": rounded(pose_6d[4]),
        "state_eef_yaw": rounded(pose_6d[5]),
        "state_gripper_pos": rounded(response.state.eef.gripper_pos) if response.state is not None else 0.0,
    }
    for index in range(6):
        snapshot[f"joint_pos_{index + 1}"] = rounded(joint_pos[index]) if index < len(joint_pos) else 0.0
        snapshot[f"joint_vel_{index + 1}"] = rounded(joint_vel[index]) if index < len(joint_vel) else 0.0
        snapshot[f"joint_torque_{index + 1}"] = rounded(joint_torque[index]) if index < len(joint_torque) else 0.0
    return snapshot


class TkTeleopDashboard:
    def __init__(self, source: str, rate_hz: float) -> None:
        try:
            import tkinter as tk
            from tkinter import ttk
        except Exception as exc:
            raise ArmctrlError(
                ErrorCode.INVALID_REQUEST,
                f"tkinter dashboard unavailable: {exc}",
            ) from exc

        self.tk = tk
        self.ttk = ttk
        self.root = tk.Tk()
        self.root.title("armctrl xbox teleop")
        self.root.geometry("1500x920")
        self.root.minsize(1280, 780)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.queue: queue.Queue[tuple[dict[str, object], dict[str, object]] | None] = queue.Queue()
        self.stop_event = threading.Event()
        self.closed = False
        self.controller_vars = {key: tk.StringVar(value="-") for key, _ in CONTROLLER_FIELDS}
        self.control_vars = {key: tk.StringVar(value="-") for key, _ in CONTROL_FIELDS}
        self.refresh_ms = hz_to_period_ms(rate_hz)
        self.summary_var = tk.StringVar(value=f"输入设备：{source}    UI 刷新频率：{rate_hz:.1f} Hz")
        self.root.protocol("WM_DELETE_WINDOW", self._on_window_close)
        self._build_layout()
        self.root.after(self.refresh_ms, self._pump)

    def _build_layout(self) -> None:
        outer = self.ttk.Frame(self.root, padding=12)
        outer.grid(row=0, column=0, sticky="nsew")
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=1)
        outer.rowconfigure(1, weight=1)

        summary = self.ttk.Label(
            outer,
            textvariable=self.summary_var,
            anchor="w",
            justify="left",
        )
        summary.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))

        controller_frame = self.ttk.LabelFrame(outer, text="遥控器输入")
        controller_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        control_frame = self.ttk.LabelFrame(outer, text="控制输出")
        control_frame.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        help_frame = self.ttk.LabelFrame(outer, text="按键与通道说明")
        help_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        outer.rowconfigure(1, weight=1)
        self._build_field_grid(controller_frame, CONTROLLER_FIELDS, self.controller_vars, pairs_per_row=2)
        self._build_field_grid(control_frame, CONTROL_FIELDS, self.control_vars, pairs_per_row=2)

        hint_label = self.ttk.Label(
            help_frame,
            text="\n".join(CONTROL_HINTS),
            justify="left",
            anchor="w",
        )
        hint_label.grid(row=0, column=0, sticky="ew", padx=8, pady=8)

    def _build_field_grid(
        self,
        parent,
        fields: tuple[tuple[str, str], ...],
        vars_map,
        *,
        pairs_per_row: int,
    ) -> None:
        for column in range(pairs_per_row * 2):
            parent.columnconfigure(column, weight=1)
        for index, (key, label) in enumerate(fields):
            row = index // pairs_per_row
            pair_column = index % pairs_per_row
            label_column = pair_column * 2
            value_column = label_column + 1
            self.ttk.Label(parent, text=label, anchor="w").grid(
                row=row,
                column=label_column,
                sticky="w",
                padx=(8, 4),
                pady=4,
            )
            self.ttk.Label(
                parent,
                textvariable=vars_map[key],
                anchor="w",
                justify="left",
            ).grid(
                row=row,
                column=value_column,
                sticky="ew",
                padx=(0, 8),
                pady=4,
            )

    def push(self, controller_snapshot: dict[str, object], control_snapshot: dict[str, object]) -> None:
        if self.closed:
            return
        self.queue.put((controller_snapshot, control_snapshot))

    def close(self) -> None:
        self.queue.put(None)

    def run(self) -> None:
        self.root.mainloop()

    def _pump(self) -> None:
        if self.closed:
            return
        while True:
            try:
                item = self.queue.get_nowait()
            except queue.Empty:
                break
            if item is None:
                self._shutdown()
                return
            controller_snapshot, control_snapshot = item
            self._apply_snapshot(self.controller_vars, controller_snapshot)
            self._apply_snapshot(self.control_vars, control_snapshot)
        self.root.after(self.refresh_ms, self._pump)

    def _apply_snapshot(self, vars_map, snapshot: dict[str, object]) -> None:
        for key, value in snapshot.items():
            if key in vars_map:
                vars_map[key].set(format_ui_value(value))

    def _on_window_close(self) -> None:
        self.stop_event.set()
        self._shutdown()

    def _shutdown(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.stop_event.set()
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            return


def create_tk_dashboard(source: str, rate_hz: float) -> TkTeleopDashboard:
    return TkTeleopDashboard(source=source, rate_hz=rate_hz)


def show_response_dashboard(response: CommandResponse, source: str) -> None:
    dashboard = create_tk_dashboard(source=source, rate_hz=0.0)
    dashboard.push(
        build_controller_snapshot(source, None, XboxState()),
        build_response_snapshot(response, phase=source.split(":", 1)[0]),
    )
    dashboard.run()


def format_ui_value(value: object) -> str:
    if value is None:
        return "-"
    if isinstance(value, bool):
        return "ON" if value else "OFF"
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def rounded(value: float) -> float:
    return round(float(value), 6)


def hz_to_period_ms(rate_hz: float) -> int:
    if rate_hz <= 0:
        return 40
    return max(1, int(round(1000.0 / rate_hz)))


def event_label(event: XboxInputEvent | None) -> str | None:
    if event is None:
        return None
    if event.event_type == 3:
        return AXIS_NAMES.get(event.code, f"ABS_{event.code}")
    if event.event_type == 1:
        return BUTTON_NAMES.get(event.code, f"BTN_{event.code}")
    return f"EV_{event.event_type}:{event.code}"


def load_events(device: str | None, event_jsonl: str | None) -> Iterable[XboxInputEvent]:
    if event_jsonl:
        return iter_jsonl_events(event_jsonl)
    if device:
        return iter_linux_input_events(device)
    default_device = Path("/dev/input/event0")
    if not default_device.exists():
        raise FileNotFoundError("no Xbox device passed and /dev/input/event0 does not exist")
    return iter_linux_input_events(default_device)
