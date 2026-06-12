from __future__ import annotations

import json
import time

import pytest

from armctrl.runtime_session import (
    eef_adapter_manager_payload,
    runtime_controller_manager_payload,
    start_fake_runtime_session,
)
from armctrl.teleop.mapping import XboxMapper, XboxState
from armctrl.teleop.mapping import XboxInputEvent
from armctrl.teleop.runtime_smoke import (
    XboxRuntimeSmokeRequest,
    XboxRuntimeSmoker,
    write_jsonl_events,
)


def test_deadman_release_requests_zero_gravity_drag_profile() -> None:
    mapper = XboxMapper()
    command = mapper.to_runtime_command(
        XboxState(axes={"ABS_Y": -1.0}, buttons={"BTN_TR": False}),
        now=1.0,
    )

    assert command.deadman is False
    assert command.profile == "zero_gravity_drag"
    assert command.has_motion is False


def test_right_bumper_enables_eef_twist_positive_x() -> None:
    mapper = XboxMapper(max_translation_speed_mps=0.2)
    command = mapper.to_runtime_command(
        XboxState(axes={"ABS_Y": -1.0}, buttons={"BTN_TR": True}),
        now=1.0,
        dt_s=0.02,
    )

    assert command.deadman is True
    assert command.profile is None
    assert command.linear_mps[0] == pytest.approx(0.2)
    assert command.to_eef_twist_payload()["linear_mps"][0] == pytest.approx(0.2)


def test_a_button_requests_runtime_owned_teleop_profile() -> None:
    mapper = XboxMapper()
    command = mapper.to_runtime_command(
        XboxState(buttons={"BTN_A": True}),
        now=1.0,
    )

    assert command.profile == "teleop"
    assert command.deadman is False


def test_xbox_runtime_smoke_submits_twist_then_drag_profile(tmp_path) -> None:
    session_artifact = tmp_path / "runtime_session.json"
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        safe_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=5.0,
    )
    payload["hold_fresh"] = True
    payload["last_hold_wall_time_s"] = time.time()
    payload["hold_age_s"] = 0.0
    payload["readiness"] = {"agent_sysid_smoke_allowed": True, "failed_checks": []}
    payload["eef_adapter_manager"] = eef_adapter_manager_payload(
        primary_backend="fake",
        configured_adapters=["sdk_cartesian"],
    )
    payload["runtime_controller_manager"] = runtime_controller_manager_payload(
        backend="fake",
        eef_adapter_manager=payload["eef_adapter_manager"],
    )
    session_artifact.write_text(json.dumps(payload), encoding="utf-8")
    events_path = tmp_path / "xbox_events.jsonl"
    write_jsonl_events(
        events_path,
        [
            XboxInputEvent(event_type=1, code=311, value=1, timestamp=1.0),
            XboxInputEvent(event_type=3, code=1, value=-32767, timestamp=1.1),
            XboxInputEvent(event_type=1, code=311, value=0, timestamp=1.2),
        ],
    )

    result = XboxRuntimeSmoker().run(
        XboxRuntimeSmokeRequest(
            session_artifact_path=session_artifact,
            expected_q_start=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
            source=events_path,
            max_heartbeat_age_s=5.0,
        )
    )

    assert result["status"] == "ok"
    assert result["submitted_count"] == 2
    assert result["runtime_queue_submissions"][0]["command_space"] == "eef"
    assert result["runtime_queue_submissions"][1]["command_space"] == "teleop"
    assert result["runtime_queue_submissions"][1]["profile"] == "zero_gravity_drag"
