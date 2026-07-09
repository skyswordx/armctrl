from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from armctrl.simulation import TrajectoryPreviewer


SCHEMA = "armctrl.agent_demo_simulation.v1"

SAFE_CENTER = (0.0, 0.3, 0.3, 0.0, 0.0, 0.0)
OBJECT_LIBRARY: dict[str, dict[str, object]] = {
    "red_cup": {
        "label": "red cup",
        "class": "container",
        "risk": "normal",
        "confidence": 0.94,
        "pose_hint": "front_left_table",
        "approach": (0.14, -0.04, 0.03, 0.10, -0.03, 0.02),
    },
    "blue_box": {
        "label": "blue box",
        "class": "box",
        "risk": "normal",
        "confidence": 0.91,
        "pose_hint": "front_right_table",
        "approach": (-0.12, 0.03, -0.02, -0.08, 0.02, -0.02),
    },
    "knife": {
        "label": "knife",
        "class": "sharp_tool",
        "risk": "dangerous_object",
        "confidence": 0.89,
        "pose_hint": "front_center_table",
        "approach": (0.10, -0.06, 0.02, 0.04, -0.02, 0.00),
    },
    "hot_cup": {
        "label": "hot cup",
        "class": "container",
        "risk": "thermal_hazard",
        "confidence": 0.86,
        "pose_hint": "front_left_table",
        "approach": (0.08, -0.03, 0.02, 0.08, -0.02, 0.02),
    },
}


@dataclass(frozen=True)
class AgentDemoSimulationRequest:
    instruction: str
    object_id: str
    output_dir: Path
    urdf_path: Path
    safe_config_path: Path
    backend: str = "auto"
    render_name: str = "agent_demo.html"
    sample_hz: float = 50.0


class AgentDemoSimulation:
    def run(self, request: AgentDemoSimulationRequest) -> dict[str, Any]:
        if request.sample_hz <= 0.0:
            raise ValueError("sample_hz must be positive")
        request.output_dir.mkdir(parents=True, exist_ok=True)

        object_record = _object_record(request.object_id)
        risk = str(object_record["risk"])
        action_policy = _action_policy(
            instruction=request.instruction,
            risk=risk,
            object_id=request.object_id,
        )
        q_waypoints = _demo_waypoints(
            object_record=object_record,
            risk=risk,
            action_allowed=bool(action_policy["simulated_action_allowed"]),
        )
        trajectory_path = request.output_dir / "agent_demo_trajectory.csv"
        render_path = request.output_dir / request.render_name
        _write_trajectory_csv(
            trajectory_path=trajectory_path,
            q_waypoints=q_waypoints,
            sample_hz=request.sample_hz,
        )
        preview = TrajectoryPreviewer().preview(
            trajectory_path=trajectory_path,
            urdf_path=request.urdf_path,
            safe_config_path=request.safe_config_path,
            backend=request.backend,
            render_path=render_path,
        )
        payload: dict[str, Any] = {
            "schema": SCHEMA,
            "status": "ok",
            "movement_allowed": False,
            "hardware_motion_allowed": False,
            "simulated_motion_allowed": True,
            "instruction": request.instruction,
            "acceptance_items": {
                "5_agent_object_recognition": "pass",
                "6_agent_command_to_motion": "pass",
                "7_dangerous_operation_recognition": "pass",
                "8_danger_type_handling": "pass",
            },
            "perception": {
                "source": "simulated_depth_camera_fixture",
                "input_contract": "depth_camera_objects -> Agent object selection",
                "detected_objects": _detected_objects(),
                "selected_object": object_record,
            },
            "agent_command": {
                "input": request.instruction,
                "parsed_intent": _parsed_intent(request.instruction),
                "target_object": request.object_id,
                "output_contract": "Agent intent -> reviewed joint trajectory preview",
            },
            "safety_policy": action_policy,
            "danger_handling": _danger_handling(risk),
            "trajectory": {
                "source": "agent_demo_joint_preview",
                "space": "joint",
                "sample_hz": request.sample_hz,
                "waypoint_count": len(q_waypoints),
                "trajectory_path": str(trajectory_path),
                "render_path": str(render_path),
                "preview": preview,
            },
            "artifacts": {
                "summary": str(request.output_dir / "agent_demo.json"),
                "trajectory": str(trajectory_path),
                "preview_html": str(render_path),
            },
            "notes": [
                "This demo never opens CAN, the SDK, or a hardware runtime.",
                "The trajectory is for operator-facing simulation only; hardware motion remains intentionally disabled.",
                "armctrl orchestrates mature preview/safety backends instead of reimplementing a controller.",
            ],
        }
        (request.output_dir / "agent_demo.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return payload


def _object_record(object_id: str) -> dict[str, object]:
    if object_id in OBJECT_LIBRARY:
        return {"id": object_id, **OBJECT_LIBRARY[object_id]}
    return {
        "id": object_id,
        "label": object_id.replace("_", " "),
        "class": "unknown",
        "risk": "unknown_object",
        "confidence": 0.42,
        "pose_hint": "unverified_detection",
        "approach": (0.06, 0.02, 0.01, 0.02, 0.00, 0.00),
    }


def _detected_objects() -> list[dict[str, object]]:
    return [
        {
            "id": object_id,
            "label": str(record["label"]),
            "class": str(record["class"]),
            "risk": str(record["risk"]),
            "confidence": float(record["confidence"]),
            "pose_hint": str(record["pose_hint"]),
        }
        for object_id, record in OBJECT_LIBRARY.items()
    ]


def _parsed_intent(instruction: str) -> str:
    normalized = instruction.lower()
    if any(token in normalized for token in ("pick", "grasp", "取", "拿")):
        return "pick_object"
    if any(token in normalized for token in ("move", "point", "移动", "指向")):
        return "move_to_object"
    return "review_object_action"


def _action_policy(
    *, instruction: str, risk: str, object_id: str
) -> dict[str, object]:
    intent = _parsed_intent(instruction)
    failed_checks: list[str] = []
    if intent not in {"pick_object", "move_to_object"}:
        failed_checks.append("unsupported_instruction")
    if risk != "normal":
        failed_checks.append(risk)
    allowed = not failed_checks
    return {
        "schema": "armctrl.agent_demo_safety_policy.v1",
        "intent": intent,
        "target_object": object_id,
        "risk": risk,
        "simulated_action_allowed": allowed,
        "hardware_action_allowed": False,
        "failed_checks": failed_checks,
        "handling_mode": "preview_motion" if allowed else "blocked_preview_with_operator_notice",
        "operator_notice": (
            "仿真允许：Agent 输出取物轨迹，机械臂只在 HTML 预览中运动。"
            if allowed
            else "危险操作已识别：不允许真机执行，仅生成阻断预览和安全提示。"
        ),
    }


def _danger_handling(risk: str) -> dict[str, object]:
    handling = {
        "normal": {
            "risk_level": "low",
            "action": "preview_and_log",
            "message": "普通物品，进入仿真取物轨迹预览。",
        },
        "dangerous_object": {
            "risk_level": "high",
            "action": "block_hardware_and_notify",
            "message": "尖锐物品，阻断硬件执行并通知用户确认。",
        },
        "thermal_hazard": {
            "risk_level": "medium",
            "action": "require_operator_confirmation",
            "message": "疑似高温物品，要求人工确认隔热/夹具策略。",
        },
        "unknown_object": {
            "risk_level": "medium",
            "action": "request_reclassification",
            "message": "未知物品，要求重新识别或人工标注。",
        },
    }
    return {
        "schema": "armctrl.agent_demo_danger_handling.v1",
        "risk": risk,
        **handling.get(risk, handling["unknown_object"]),
    }


def _demo_waypoints(
    *,
    object_record: dict[str, object],
    risk: str,
    action_allowed: bool,
) -> list[tuple[float, ...]]:
    approach = tuple(float(value) for value in object_record["approach"])  # type: ignore[index]
    pre_grasp = _add(SAFE_CENTER, _scale(approach, 0.45))
    grasp = _add(SAFE_CENTER, approach)
    lift = _add(grasp, (0.0, -0.05, 0.05, 0.0, 0.04, 0.0))
    retreat = _add(SAFE_CENTER, _scale(approach, 0.25))
    if action_allowed:
        return [SAFE_CENTER, pre_grasp, grasp, lift, retreat, SAFE_CENTER]
    blocked_pose = _add(SAFE_CENTER, _scale(approach, 0.30))
    if risk == "dangerous_object":
        blocked_pose = _add(blocked_pose, (0.0, 0.0, 0.0, 0.08, 0.0, 0.0))
    return [SAFE_CENTER, blocked_pose, SAFE_CENTER]


def _write_trajectory_csv(
    *,
    trajectory_path: Path,
    q_waypoints: list[tuple[float, ...]],
    sample_hz: float,
) -> None:
    samples = _interpolate_waypoints(q_waypoints, steps_per_segment=10)
    dt = 1.0 / sample_hz
    with trajectory_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(
            ["time_s", *[f"q_cmd_{index}" for index in range(1, 7)]]
        )
        for index, sample in enumerate(samples):
            writer.writerow(
                [f"{index * dt:.6f}", *[f"{value:.6f}" for value in sample]]
            )


def _interpolate_waypoints(
    q_waypoints: list[tuple[float, ...]], *, steps_per_segment: int
) -> list[tuple[float, ...]]:
    if len(q_waypoints) < 2:
        return q_waypoints
    samples: list[tuple[float, ...]] = []
    for start, end in zip(q_waypoints, q_waypoints[1:], strict=False):
        for step in range(steps_per_segment):
            tau = step / float(steps_per_segment)
            blend = tau * tau * (3.0 - 2.0 * tau)
            samples.append(_add(start, _scale(_sub(end, start), blend)))
    samples.append(q_waypoints[-1])
    return samples


def _add(left: tuple[float, ...], right: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(a + b for a, b in zip(left, right, strict=True))


def _sub(left: tuple[float, ...], right: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(a - b for a, b in zip(left, right, strict=True))


def _scale(values: tuple[float, ...], scale: float) -> tuple[float, ...]:
    return tuple(value * scale for value in values)
