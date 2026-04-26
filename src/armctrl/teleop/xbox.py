"""Xbox 遥操作输入、采样和 GUI 调试面板。

这层专门解决三个频率域的问题：
1. 输入读取：事件驱动，不限频；
2. 控制发送：固定 `rate_hz`；
3. UI 刷新：固定 `ui_hz`。

三者分离之后，手柄驱动抖动不会直接拖慢控制发送，
控制发送节拍也不会把 GUI 刷屏行为带进终端或日志。
"""

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

# 这里规定 GUI 中轴与按钮的展示顺序。
# 顺序固定有两个好处：
# 1. 人眼容易形成肌肉记忆；
# 2. fake / sdk / event-jsonl 三种输入源可以共享同一面板布局。
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
    "RB / BTN_TR：按住后才发送末端 jog，松开进入 zero_gravity_drag。",
    "左摇杆上下：末端 x 方向小步移动。",
    "左摇杆左右：末端 y 方向小步移动。",
    "右摇杆上下：末端 z 方向小步移动。",
    "右摇杆左右：末端 yaw 小步旋转。",
    "方向键左右：末端 roll 小步旋转。",
    "方向键上下：末端 pitch 小步旋转。",
    "左右扳机：夹爪开合小步控制。",
    "X / BTN_X：切到 damping。",
    "A / BTN_A：恢复 teleop 默认增益。",
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
    ("command_dt_s", "控制 dt"),
    ("command_velocity_x", "速度 x m/s"),
    ("command_velocity_y", "速度 y m/s"),
    ("command_velocity_z", "速度 z m/s"),
    ("command_angular_velocity_roll", "角速度 roll rad/s"),
    ("command_angular_velocity_pitch", "角速度 pitch rad/s"),
    ("command_angular_velocity_yaw", "角速度 yaw rad/s"),
    ("command_gripper_velocity", "夹爪速度 m/s"),
    ("command_translation_x", "本拍 dx m"),
    ("command_translation_y", "本拍 dy m"),
    ("command_translation_z", "本拍 dz m"),
    ("command_roll", "本拍 droll rad"),
    ("command_pitch", "本拍 dpitch rad"),
    ("command_yaw", "本拍 dyaw rad"),
    ("command_gripper_delta", "本拍夹爪增量 m"),
    ("target_eef_x", "目标 EEF x"),
    ("target_eef_y", "目标 EEF y"),
    ("target_eef_z", "目标 EEF z"),
    ("target_eef_roll", "目标 EEF roll"),
    ("target_eef_pitch", "目标 EEF pitch"),
    ("target_eef_yaw", "目标 EEF yaw"),
    ("target_gripper_pos", "目标夹爪位置"),
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
    """从 Linux `/dev/input/event*` 设备持续读取原始事件。"""

    # `struct input_event` 的格式由内核 ABI 决定。
    # 这里直接按固定结构拆包，避免额外依赖第三方 joystick 库。
    with Path(device).open("rb") as handle:
        while True:
            chunk = handle.read(LINUX_INPUT_EVENT.size)
            if not chunk:
                return
            sec, usec, event_type, code, value = LINUX_INPUT_EVENT.unpack(chunk)
            yield XboxInputEvent(event_type, code, value, sec + usec / 1_000_000.0)


def iter_jsonl_events(path: str | Path) -> Iterator[XboxInputEvent]:
    """从录制的 JSONL 回放手柄事件。

    这个入口主要服务测试、离线调参与问题复现。
    """

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
    """调试输出面板协议。

    这里故意只要求 `push` 和 `close` 两个最小方法，
    这样终端、Tk、未来 WebSocket 面板都可以无痛替换。
    """

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
        # `state` 保存当前完整手柄状态，
        # `last_response` 保存最近一次控制侧返回，
        # `last_command` 则保存最近一次由 mapper 生成的控制命令。
        state = XboxState()
        last_response = self.executor.state()
        last_event: XboxInputEvent | None = None
        last_command: TeleopCommand | None = None
        # 控制发送节拍严格由 `rate_hz` 推导：
        #   period_s = 1 / rate_hz
        period_s = 1.0 / self.rate_hz
        processed = 0
        event_queue: queue.Queue[XboxInputEvent] = queue.Queue()
        reader_done = threading.Event()
        reader_errors: list[BaseException] = []

        def read_events() -> None:
            # 输入线程只负责“收事件并入队”，不做控制计算。
            # 这样可以保持输入读取为事件驱动，不被固定控制周期卡住。
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
                # 一次控制拍内可能收到多个输入事件。
                # 这里采用“事件合并后只发送一次控制命令”的策略，
                # 可以避免 burst 输入把控制链打成高频抖动。
                if had_event or state.buttons.get("BTN_TR", False):
                    now = tick_timestamp or time.monotonic()
                    command = self.mapper.to_command(state, now, dt_s=period_s)
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
                    # 如果处理耗时已经吃掉一个周期，就重新对齐到当前时间，
                    # 防止累计误差导致控制循环越跑越慢。
                    next_tick = time.monotonic()
        finally:
            # 不论 teleop 是正常结束、报错还是用户关闭窗口，都强制落到 damping。
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
        # GUI 层永远只接收已经整理好的快照，不直接读取业务对象。
        # 这样界面层无需理解 executor / adapter / mapper 内部实现。
        self.dashboard.push(
            build_controller_snapshot(self.source, event, state),
            build_control_snapshot(command, response, phase),
        )


def build_controller_snapshot(
    source: str,
    event: XboxInputEvent | None,
    state: XboxState,
) -> dict[str, object]:
    """提取手柄输入快照，供 GUI 或日志面板直接显示。"""

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
    # 这里用固定字段全集输出，而不是只输出“当前出现过的字段”。
    # 好处是 GUI 面板不抖动，测试也能稳定断言字段存在性。
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
    """把命令层和响应层合并成一张控制面板快照。"""

    snapshot = build_response_snapshot(response, phase)
    snapshot.update(
        {
            "command_deadman": command.deadman if command is not None else False,
            "command_debug_profile": command.debug_profile if command is not None else None,
            "command_dt_s": rounded(command.dt_s) if command is not None else 0.0,
            "command_velocity_x": rounded(command.translation_velocity_mps[0]) if command is not None else 0.0,
            "command_velocity_y": rounded(command.translation_velocity_mps[1]) if command is not None else 0.0,
            "command_velocity_z": rounded(command.translation_velocity_mps[2]) if command is not None else 0.0,
            "command_angular_velocity_roll": rounded(command.rotation_velocity_radps[0]) if command is not None else 0.0,
            "command_angular_velocity_pitch": rounded(command.rotation_velocity_radps[1]) if command is not None else 0.0,
            "command_angular_velocity_yaw": rounded(command.rotation_velocity_radps[2]) if command is not None else 0.0,
            "command_gripper_velocity": rounded(command.gripper_velocity_mps) if command is not None else 0.0,
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
    """从统一响应对象里展开 GUI 所需字段。

    这里的设计重点不是“最省代码”，而是“字段固定、界面稳定、脚本好取值”。
    """

    pose_6d = response.state.eef.pose_6d if response.state is not None else (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_pos = response.state.joint.pos if response.state is not None else (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_vel = response.state.joint.vel if response.state is not None else (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_torque = response.state.joint.torque if response.state is not None else (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    target_pose_6d = response.detail.get("target_pose_6d", (None, None, None, None, None, None))
    snapshot: dict[str, object] = {
        "phase": phase,
        "response_command_id": response.command_id,
        "command_deadman": False,
        "command_debug_profile": None,
        "command_dt_s": 0.0,
        "command_velocity_x": 0.0,
        "command_velocity_y": 0.0,
        "command_velocity_z": 0.0,
        "command_angular_velocity_roll": 0.0,
        "command_angular_velocity_pitch": 0.0,
        "command_angular_velocity_yaw": 0.0,
        "command_gripper_velocity": 0.0,
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
        "target_eef_x": rounded(target_pose_6d[0]) if target_pose_6d[0] is not None else None,
        "target_eef_y": rounded(target_pose_6d[1]) if target_pose_6d[1] is not None else None,
        "target_eef_z": rounded(target_pose_6d[2]) if target_pose_6d[2] is not None else None,
        "target_eef_roll": rounded(target_pose_6d[3]) if target_pose_6d[3] is not None else None,
        "target_eef_pitch": rounded(target_pose_6d[4]) if target_pose_6d[4] is not None else None,
        "target_eef_yaw": rounded(target_pose_6d[5]) if target_pose_6d[5] is not None else None,
        "target_gripper_pos": rounded(response.detail["target_gripper_pos"])
        if "target_gripper_pos" in response.detail
        else None,
    }
    for index in range(6):
        # 关节数组在 UI 上拆成显式字段，避免前端再写一次索引逻辑。
        snapshot[f"joint_pos_{index + 1}"] = rounded(joint_pos[index]) if index < len(joint_pos) else 0.0
        snapshot[f"joint_vel_{index + 1}"] = rounded(joint_vel[index]) if index < len(joint_vel) else 0.0
        snapshot[f"joint_torque_{index + 1}"] = rounded(joint_torque[index]) if index < len(joint_torque) else 0.0
    return snapshot


class TkTeleopDashboard:
    """Tk 调试面板。

    这里仍然采用最朴素的 Tk 方案，原因很现实：
    - Python 标准库自带；
    - 在 bringup 机器上依赖最少；
    - 对“固定位置刷新数值面板”这类需求足够稳定。
    """

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
        # 布局刻意分成三块：
        # - 左侧：手柄输入；
        # - 右侧：控制输出；
        # - 底部：按键说明。
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
        # 双列 label/value 栅格能在字段较多时维持较高信息密度。
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
        # 业务线程只入队，不直接碰 Tk 控件。
        # 这是 Tk 线程模型的关键约束：控件更新必须在主线程完成。
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
        # UI 刷新频率由 `ui_hz` 决定，和控制发送频率解耦。
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
    """工厂函数，方便 CLI 侧和测试替换面板实现。"""
    return TkTeleopDashboard(source=source, rate_hz=rate_hz)


def show_response_dashboard(response: CommandResponse, source: str) -> None:
    # 非 teleop 命令也复用同一张 GUI，只是输入面板保持空状态。
    dashboard = create_tk_dashboard(source=source, rate_hz=0.0)
    dashboard.push(
        build_controller_snapshot(source, None, XboxState()),
        build_response_snapshot(response, phase=source.split(":", 1)[0]),
    )
    dashboard.run()


def format_ui_value(value: object) -> str:
    """把不同类型的值规整成适合面板显示的字符串。"""

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
    # UI 和日志都统一保留 6 位，便于对齐阅读，也足够覆盖当前调试精度。
    return round(float(value), 6)


def hz_to_period_ms(rate_hz: float) -> int:
    # `after()` 需要毫秒整数。
    # 当频率非法或为 0 时，退回到保守的 40ms 轮询。
    if rate_hz <= 0:
        return 40
    return max(1, int(round(1000.0 / rate_hz)))


def event_label(event: XboxInputEvent | None) -> str | None:
    """把原始事件码翻译成 UI 和日志可读的名称。"""

    if event is None:
        return None
    if event.event_type == 3:
        return AXIS_NAMES.get(event.code, f"ABS_{event.code}")
    if event.event_type == 1:
        return BUTTON_NAMES.get(event.code, f"BTN_{event.code}")
    return f"EV_{event.event_type}:{event.code}"


def load_events(device: str | None, event_jsonl: str | None) -> Iterable[XboxInputEvent]:
    """选择事件来源。

    优先级：
    1. JSONL 回放；
    2. 显式设备；
    3. 默认 `/dev/input/event0`。
    """

    if event_jsonl:
        return iter_jsonl_events(event_jsonl)
    if device:
        return iter_linux_input_events(device)
    default_device = Path("/dev/input/event0")
    if not default_device.exists():
        raise FileNotFoundError("no Xbox device passed and /dev/input/event0 does not exist")
    return iter_linux_input_events(default_device)
