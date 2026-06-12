from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import struct
import time
from typing import Iterable, Iterator, Sequence

from armctrl.runtime_ipc import (
    DEFAULT_EEF_MAX_ANGULAR_STEP_RAD,
    DEFAULT_EEF_MAX_LINEAR_STEP_M,
    submit_eef_command,
    submit_teleop_profile_command,
)
from armctrl.teleop.mapping import XboxInputEvent, XboxMapper, XboxState


@dataclass(frozen=True)
class XboxRuntimeSmokeRequest:
    session_artifact_path: Path
    expected_q_start: tuple[float, ...]
    source: Path
    source_kind: str = "jsonl"
    owner: str = "teleop"
    backend: str = "sdk_cartesian"
    frame: str = "eef_link"
    send_hz: float = 50.0
    max_events: int | None = None
    max_heartbeat_age_s: float = 1.0
    heartbeat_timeout_s: float = 0.5
    max_start_error_rad: float = 0.02
    start_pose_policy: str = "live_hold"
    max_linear_step_m: float = DEFAULT_EEF_MAX_LINEAR_STEP_M
    max_angular_step_rad: float = DEFAULT_EEF_MAX_ANGULAR_STEP_RAD
    output_path: Path | None = None


class XboxRuntimeSmoker:
    """Submit Xbox teleop intent into the runtime-owned queue.

    This deliberately does not execute SDK/CAN commands. The long-lived runtime
    process remains the only hardware owner.
    """

    def run(self, request: XboxRuntimeSmokeRequest) -> dict[str, object]:
        if request.send_hz <= 0.0:
            raise ValueError("send_hz must be positive")
        events = (
            _read_input_device_events(request.source)
            if request.source_kind == "device"
            else _read_jsonl_events(request.source)
        )
        mapper = XboxMapper(default_dt_s=1.0 / float(request.send_hz))
        state = XboxState()
        queued: list[dict[str, object]] = []
        submitted_count = 0
        last_profile: str | None = None
        started_wall_time_s = time.time()
        for event_index, event in enumerate(events):
            if request.max_events is not None and event_index >= request.max_events:
                break
            state.update(event)
            command = mapper.to_runtime_command(
                state,
                now=event.timestamp or time.time(),
                dt_s=1.0 / float(request.send_hz),
            )
            if command.profile is not None and command.profile != last_profile:
                queued_payload = submit_teleop_profile_command(
                    session_artifact_path=request.session_artifact_path,
                    owner=request.owner,
                    backend=request.backend,
                    profile=command.profile,
                    expected_q_start=request.expected_q_start,
                    start_pose_policy=request.start_pose_policy,
                    max_start_error_rad=request.max_start_error_rad,
                    heartbeat_timeout_s=request.heartbeat_timeout_s,
                    max_heartbeat_age_s=request.max_heartbeat_age_s,
                )
                queued.append(_queued_summary(queued_payload, input_event=event))
                submitted_count += int(queued_payload.get("status") == "queued")
                last_profile = command.profile
                continue
            if command.deadman and command.has_motion:
                queued_payload = submit_eef_command(
                    session_artifact_path=request.session_artifact_path,
                    owner=request.owner,
                    backend=request.backend,
                    kind="eef_twist",
                    frame=request.frame,
                    expected_q_start=request.expected_q_start,
                    control_period_s=command.dt_s,
                    send_hz=request.send_hz,
                    start_pose_policy=request.start_pose_policy,
                    max_start_error_rad=request.max_start_error_rad,
                    heartbeat_timeout_s=request.heartbeat_timeout_s,
                    max_heartbeat_age_s=request.max_heartbeat_age_s,
                    linear_mps=command.linear_mps,
                    angular_rps=command.angular_rps,
                    max_linear_step_m=request.max_linear_step_m,
                    max_angular_step_rad=request.max_angular_step_rad,
                )
                queued.append(_queued_summary(queued_payload, input_event=event))
                submitted_count += int(queued_payload.get("status") == "queued")
                last_profile = None
        payload = {
            "status": "ok" if submitted_count == len(queued) else "blocked",
            "schema": "armctrl.teleop_xbox_runtime_smoke.v1",
            "source_kind": request.source_kind,
            "source": str(request.source),
            "owner": request.owner,
            "backend": request.backend,
            "command_surface": "runtime-owned teleop input adapter",
            "sdk_can_singleton": True,
            "movement_command_sent": False,
            "runtime_queue_submissions": queued,
            "submitted_count": submitted_count,
            "observed_event_count": len(queued),
            "started_wall_time_s": started_wall_time_s,
            "completed_wall_time_s": time.time(),
            "next_gate": "serve the long-lived runtime queue and inspect result artifacts",
        }
        if request.output_path is not None:
            request.output_path.parent.mkdir(parents=True, exist_ok=True)
            request.output_path.write_text(
                json.dumps(payload, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            payload["artifacts"] = {"teleop_xbox_runtime_smoke": str(request.output_path)}
        return payload


def _queued_summary(payload: dict[str, object], *, input_event: XboxInputEvent) -> dict[str, object]:
    return {
        "status": payload.get("status"),
        "command_id": payload.get("command_id"),
        "owner": payload.get("owner"),
        "mode": payload.get("mode"),
        "command_space": payload.get("command_space"),
        "backend": payload.get("backend"),
        "profile": payload.get("profile"),
        "artifacts": payload.get("artifacts"),
        "input_event": {
            "event_type": input_event.event_type,
            "code": input_event.code,
            "value": input_event.value,
            "timestamp": input_event.timestamp,
        },
        "reason": payload.get("reason"),
    }


def _read_jsonl_events(path: Path) -> Iterator[XboxInputEvent]:
    with path.open(encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                payload = json.loads(stripped)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid JSONL event at line {line_number}: {error}") from error
            yield XboxInputEvent(
                event_type=int(payload["event_type"]),
                code=int(payload["code"]),
                value=int(payload["value"]),
                timestamp=float(payload.get("timestamp", 0.0)),
            )


def _read_input_device_events(path: Path) -> Iterator[XboxInputEvent]:
    # Linux input_event on the target n100d is 64-bit timeval + type/code/value.
    event_struct = struct.Struct("llHHI")
    with path.open("rb", buffering=0) as device:
        while True:
            data = device.read(event_struct.size)
            if not data:
                break
            if len(data) != event_struct.size:
                continue
            sec, usec, event_type, code, value = event_struct.unpack(data)
            yield XboxInputEvent(
                event_type=int(event_type),
                code=int(code),
                value=int(value),
                timestamp=float(sec) + float(usec) / 1_000_000.0,
            )


def write_jsonl_events(path: Path, events: Sequence[XboxInputEvent]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        for event in events:
            file.write(
                json.dumps(
                    {
                        "event_type": event.event_type,
                        "code": event.code,
                        "value": event.value,
                        "timestamp": event.timestamp,
                    },
                    sort_keys=True,
                )
                + "\n"
            )

