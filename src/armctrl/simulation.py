from __future__ import annotations

import csv
from dataclasses import dataclass
import html
import importlib
import importlib.util
import json
from pathlib import Path
import tempfile
from typing import Iterable
import xml.etree.ElementTree as ET

from armctrl.workspace import (
    WorkspaceSafetyConfig,
    evaluate_workspace_fk_clearance,
    evaluate_workspace_zones,
    link_frame_positions,
)


@dataclass(frozen=True)
class BackendSpec:
    name: str
    role: str
    modules: tuple[str, ...]
    runtime: str


BACKENDS: tuple[BackendSpec, ...] = (
    BackendSpec(
        name="pinocchio_coal",
        role="lightweight URDF geometry collision checks",
        modules=("pinocchio", "coal"),
        runtime="python",
    ),
    BackendSpec(
        name="mujoco",
        role="contact and dynamics simulation preview",
        modules=("mujoco",),
        runtime="python",
    ),
    BackendSpec(
        name="moveit",
        role="ROS planning-scene state validity and collision oracle",
        modules=("rclpy", "moveit_msgs", "moveit_configs_utils"),
        runtime="ros2_moveit",
    ),
    BackendSpec(
        name="figaroh",
        role="SysID excitation optimization and identification handoff",
        modules=("figaroh",),
        runtime="python",
    ),
)


class SimulationDoctor:
    def run(self) -> dict[str, object]:
        return {
            "schema": "armctrl.simulation_doctor.v1",
            "movement_allowed": False,
            "backends": [_backend_status(spec) for spec in BACKENDS],
            "notes": [
                "doctor is read-only and does not open CAN, instantiate the SDK, or move hardware",
                "armctrl uses mature backends as safety oracles instead of reimplementing simulators",
            ],
        }


class TrajectoryPreviewer:
    def preview(
        self,
        *,
        trajectory_path: Path,
        urdf_path: Path,
        safe_config_path: Path,
        backend: str = "auto",
        render_path: Path | None = None,
    ) -> dict[str, object]:
        rows = _read_trajectory_rows(trajectory_path)
        q_samples = _q_samples_from_rows(rows)
        config = WorkspaceSafetyConfig.from_yaml(safe_config_path)
        backend_selection = _evaluate_backend_chain(
            config,
            requested_backend=backend,
            urdf_path=urdf_path,
            q_samples=q_samples,
        )
        selected_backend = str(backend_selection["selected"])
        backend_result = dict(backend_selection["result"])
        clearance = evaluate_workspace_fk_clearance(
            urdf_path,
            config,
            samples=q_samples,
        )
        zones = evaluate_workspace_zones(
            urdf_path,
            config,
            samples=q_samples,
        )
        safety_allowed = (
            backend_result["status"] == "pass"
            and clearance.status == "pass"
            and zones.status == "pass"
        )
        payload: dict[str, object] = {
            "schema": "armctrl.trajectory_preview.v1",
            "movement_allowed": False,
            "trajectory_path": str(trajectory_path),
            "urdf_path": str(urdf_path),
            "safe_config_path": str(safe_config_path),
            "backend": {
                "requested": backend,
                "selected": selected_backend,
                "result": backend_result,
                "attempts": backend_selection["attempts"],
            },
            "safety": {
                "allowed": safety_allowed,
                "clearance_check": {
                    "status": clearance.status,
                    "method": clearance.method,
                    "violations": clearance.violations,
                },
                "zone_check": {
                    "status": zones.status,
                    "method": zones.method,
                    "violations": zones.violations,
                },
            },
            "trajectory_metrics": _trajectory_metrics(q_samples),
            "workspace_metrics": _workspace_metrics(
                urdf_path,
                config=config,
                q_samples=q_samples,
            ),
        }
        if render_path is not None:
            render_format = _write_trajectory_render(
                output_path=render_path,
                preview=payload,
                urdf_path=urdf_path,
                config=config,
                q_samples=q_samples,
            )
            payload["render"] = {
                "status": "written",
                "path": str(render_path),
                "format": render_format,
            }
        return payload


def _backend_status(spec: BackendSpec) -> dict[str, object]:
    if spec.name == "moveit":
        return _moveit_backend_status(spec)
    module_statuses = [
        {
            "module": module,
            "importable": importlib.util.find_spec(module) is not None,
        }
        for module in spec.modules
    ]
    importable = all(status["importable"] for status in module_statuses)
    return {
        "name": spec.name,
        "role": spec.role,
        "runtime": spec.runtime,
        "status": "available" if importable else "missing",
        "modules": module_statuses,
    }


def _moveit_backend_status(spec: BackendSpec) -> dict[str, object]:
    module_statuses = [
        {
            "module": module,
            "importable": importlib.util.find_spec(module) is not None,
        }
        for module in spec.modules
    ]
    importable = all(status["importable"] for status in module_statuses)
    installed_ros = _installed_ros_distribution()
    if importable:
        status = "available"
    elif installed_ros is not None:
        status = "installed_not_sourced"
    else:
        status = "missing"
    payload: dict[str, object] = {
        "name": spec.name,
        "role": spec.role,
        "runtime": spec.runtime,
        "status": status,
        "modules": module_statuses,
    }
    if installed_ros is not None:
        payload["ros_distro"] = installed_ros
        payload["source_hint"] = f"source /opt/ros/{installed_ros}/setup.bash"
    return payload


def _installed_ros_distribution() -> str | None:
    for distro in ("jazzy", "iron", "humble", "rolling"):
        if Path(f"/opt/ros/{distro}").exists():
            return distro
    return None


def _evaluate_backend_chain(
    config: WorkspaceSafetyConfig,
    *,
    requested_backend: str,
    urdf_path: Path,
    q_samples: list[tuple[float, ...]],
) -> dict[str, object]:
    if requested_backend != "auto":
        result = _run_backend_check(
            selected_backend=requested_backend,
            urdf_path=urdf_path,
            q_samples=q_samples,
            allowed_collision_pairs=config.allowed_collision_pairs,
        )
        return {
            "selected": requested_backend,
            "result": result,
            "attempts": [{"backend": requested_backend, "result": result}],
        }
    attempts: list[dict[str, object]] = []
    for backend in config.simulation_backend_preference:
        if backend == "urdf_fk_fallback":
            result = _run_backend_check(
                selected_backend=backend,
                urdf_path=urdf_path,
                q_samples=q_samples,
                allowed_collision_pairs=config.allowed_collision_pairs,
            )
            attempts.append({"backend": backend, "result": result})
            return {
                "selected": backend,
                "result": result,
                "attempts": attempts,
            }
        spec = next((item for item in BACKENDS if item.name == backend), None)
        if spec is None:
            continue
        if not all(importlib.util.find_spec(module) is not None for module in spec.modules):
            continue
        result = _run_backend_check(
            selected_backend=backend,
            urdf_path=urdf_path,
            q_samples=q_samples,
            allowed_collision_pairs=config.allowed_collision_pairs,
        )
        attempts.append({"backend": backend, "result": result})
        if result["status"] in {"pass", "fail"}:
            return {
                "selected": backend,
                "result": result,
                "attempts": attempts,
            }
    result = _run_backend_check(
        selected_backend="urdf_fk_fallback",
        urdf_path=urdf_path,
        q_samples=q_samples,
        allowed_collision_pairs=config.allowed_collision_pairs,
    )
    attempts.append({"backend": "urdf_fk_fallback", "result": result})
    return {
        "selected": "urdf_fk_fallback",
        "result": result,
        "attempts": attempts,
    }


def _run_backend_check(
    *,
    selected_backend: str,
    urdf_path: Path,
    q_samples: list[tuple[float, ...]],
    allowed_collision_pairs: tuple[tuple[str, str], ...],
) -> dict[str, object]:
    if selected_backend == "pinocchio_coal":
        return _pinocchio_coal_check(
            urdf_path=urdf_path,
            q_samples=q_samples,
            allowed_collision_pairs=allowed_collision_pairs,
        )
    if selected_backend == "mujoco":
        return _mujoco_trajectory_check(
            urdf_path=urdf_path,
            q_samples=q_samples,
            allowed_collision_pairs=allowed_collision_pairs,
        )
    if selected_backend == "moveit":
        return {
            "status": "not_invoked",
            "method": "moveit_planning_scene_adapter_pending_ros_runtime",
            "reason": "MoveIt is available only inside a ROS planning-scene runtime; use sim doctor to verify modules first.",
        }
    return {
        "status": "pass",
        "method": "urdf_fk_fallback",
        "reason": "No mature simulation backend was importable; using conservative FK workspace gates only.",
    }


def _pinocchio_coal_check(
    *,
    urdf_path: Path,
    q_samples: list[tuple[float, ...]],
    allowed_collision_pairs: tuple[tuple[str, str], ...] = (),
) -> dict[str, object]:
    try:
        import numpy as np
        import pinocchio as pin
    except ModuleNotFoundError as error:
        return {
            "status": "not_available",
            "method": "pinocchio_coal",
            "reason": f"module not importable: {error.name}",
        }
    try:
        with tempfile.TemporaryDirectory(prefix="armctrl-pinocchio-") as temp_dir:
            prepared_urdf = _prepare_urdf_for_native_geometry(
                urdf_path,
                output_dir=Path(temp_dir),
            )
            model, collision_model, _visual_model = pin.buildModelsFromUrdf(
                str(prepared_urdf)
            )
            collision_model.addAllCollisionPairs()
            data = model.createData()
            collision_data = pin.GeometryData(collision_model)
            ignored_collisions: set[tuple[str, str]] = set()
            for sample_index, sample in enumerate(q_samples):
                q = np.array(sample, dtype=float)
                pin.computeCollisions(
                    model,
                    data,
                    collision_model,
                    collision_data,
                    q,
                    False,
                )
                for pair_index, result in enumerate(collision_data.collisionResults):
                    if result.isCollision():
                        pair = collision_model.collisionPairs[pair_index]
                        first_name = collision_model.geometryObjects[pair.first].name
                        second_name = collision_model.geometryObjects[pair.second].name
                        if _is_allowed_collision_pair(
                            first_name,
                            second_name,
                            allowed_collision_pairs,
                        ):
                            ignored_collisions.add(
                                tuple(
                                    sorted(
                                        (
                                            _strip_pinocchio_suffix(first_name),
                                            _strip_pinocchio_suffix(second_name),
                                        )
                                    )
                                )
                            )
                            continue
                        return {
                            "status": "fail",
                            "method": "pinocchio_coal",
                            "violation": {
                                "sample_index": sample_index,
                                "collision_pair_index": pair_index,
                                "first": first_name,
                                "second": second_name,
                            },
                        }
        return {
            "status": "pass",
            "method": "pinocchio_coal",
            "checked_samples": len(q_samples),
            "ignored_allowed_collision_pairs": [
                {"first": first, "second": second}
                for first, second in sorted(ignored_collisions)
            ],
        }
    except Exception as error:  # pragma: no cover - depends on native geometry stack
        return {
            "status": "not_evaluated",
            "method": "pinocchio_coal",
            "reason": str(error),
        }


def _prepare_urdf_for_native_geometry(
    urdf_path: Path,
    *,
    output_dir: Path,
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if filename is None or filename.startswith("package://"):
            continue
        path = Path(filename)
        if not path.is_absolute():
            path = urdf_path.parent / path
        mesh.attrib["filename"] = str(path.resolve())
    output_path = output_dir / urdf_path.name
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


def _is_allowed_collision_pair(
    first_name: str,
    second_name: str,
    allowed_collision_pairs: tuple[tuple[str, str], ...],
) -> bool:
    first = _strip_pinocchio_suffix(first_name)
    second = _strip_pinocchio_suffix(second_name)
    pair = frozenset((first, second))
    return any(
        pair == frozenset(allowed_pair)
        for allowed_pair in allowed_collision_pairs
    )


def _strip_pinocchio_suffix(name: str) -> str:
    suffix = "_0"
    if name.endswith(suffix):
        return name[: -len(suffix)]
    return name


def _mujoco_trajectory_check(
    *,
    urdf_path: Path,
    q_samples: list[tuple[float, ...]],
    allowed_collision_pairs: tuple[tuple[str, str], ...] = (),
) -> dict[str, object]:
    try:
        mujoco = importlib.import_module("mujoco")
    except ModuleNotFoundError as error:
        return {
            "status": "not_available",
            "method": "mujoco",
            "reason": f"module not importable: {error.name}",
        }
    try:
        model = mujoco.MjModel.from_xml_path(str(urdf_path))
        data = mujoco.MjData(model)
        dof = len(q_samples[0]) if q_samples else 0
        if dof > int(model.nq):
            return {
                "status": "not_evaluated",
                "method": "mujoco_trajectory_rollout",
                "reason": f"trajectory dof {dof} exceeds MuJoCo qpos dimension {model.nq}",
            }
        max_contact_count = 0
        raw_max_contact_count = 0
        first_contact: dict[str, object] | None = None
        ignored_contacts: set[tuple[str, str]] = set()
        for sample_index, sample in enumerate(q_samples):
            data.qpos[:dof] = sample
            mujoco.mj_forward(model, data)
            raw_contact_count = int(getattr(data, "ncon", 0))
            raw_max_contact_count = max(raw_max_contact_count, raw_contact_count)
            unallowed_contacts = _mujoco_unallowed_contacts(
                model,
                data,
                mujoco,
                contact_count=raw_contact_count,
                allowed_collision_pairs=allowed_collision_pairs,
            )
            for contact_pair in _mujoco_allowed_contacts(
                model,
                data,
                mujoco,
                contact_count=raw_contact_count,
                allowed_collision_pairs=allowed_collision_pairs,
            ):
                ignored_contacts.add(contact_pair)
            contact_count = len(unallowed_contacts)
            max_contact_count = max(max_contact_count, contact_count)
            if unallowed_contacts and first_contact is None:
                first_contact = unallowed_contacts[0]
    except Exception as error:  # pragma: no cover - depends on MuJoCo/native meshes
        return {
            "status": "not_evaluated",
            "method": "mujoco_trajectory_rollout",
            "reason": str(error),
        }
    status = "fail" if max_contact_count > 0 else "pass"
    result: dict[str, object] = {
        "status": status,
        "method": "mujoco_trajectory_rollout",
        "checked_samples": len(q_samples),
        "qpos_dimension": int(model.nq),
        "max_contact_count": max_contact_count,
        "raw_max_contact_count": raw_max_contact_count,
        "ignored_allowed_collision_pairs": [
            {"first": first, "second": second}
            for first, second in sorted(ignored_contacts)
        ],
        "qpos_range_rad": _qpos_ranges(q_samples),
    }
    if first_contact is not None:
        result["first_contact"] = first_contact
    return result


def _mujoco_unallowed_contacts(
    model: object,
    data: object,
    mujoco: object,
    *,
    contact_count: int,
    allowed_collision_pairs: tuple[tuple[str, str], ...],
) -> list[dict[str, object]]:
    contacts: list[dict[str, object]] = []
    for contact_index in range(contact_count):
        contact = _mujoco_contact_pair(
            model,
            data,
            mujoco,
            contact_index=contact_index,
        )
        if contact is None:
            continue
        first = str(contact["body1_name"])
        second = str(contact["body2_name"])
        if _is_allowed_collision_pair(first, second, allowed_collision_pairs):
            continue
        contacts.append(contact)
    return contacts


def _mujoco_allowed_contacts(
    model: object,
    data: object,
    mujoco: object,
    *,
    contact_count: int,
    allowed_collision_pairs: tuple[tuple[str, str], ...],
) -> list[tuple[str, str]]:
    contacts: list[tuple[str, str]] = []
    for contact_index in range(contact_count):
        contact = _mujoco_contact_pair(
            model,
            data,
            mujoco,
            contact_index=contact_index,
        )
        if contact is None:
            continue
        first = str(contact["body1_name"])
        second = str(contact["body2_name"])
        if _is_allowed_collision_pair(first, second, allowed_collision_pairs):
            contacts.append(
                tuple(sorted((_normalize_mujoco_body_name(first), _normalize_mujoco_body_name(second))))
            )
    return contacts


def _mujoco_contact_pair(
    model: object,
    data: object,
    mujoco: object,
    *,
    contact_index: int,
) -> dict[str, object] | None:
    try:
        contact = data.contact[contact_index]
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        body1 = int(model.geom_bodyid[geom1])
        body2 = int(model.geom_bodyid[geom2])
    except Exception:
        return None
    body1_name = _normalize_mujoco_body_name(
        _mujoco_body_name(model, mujoco, body1)
    )
    body2_name = _normalize_mujoco_body_name(
        _mujoco_body_name(model, mujoco, body2)
    )
    return {
        "geom1": geom1,
        "geom2": geom2,
        "geom1_name": _mujoco_geom_name(model, mujoco, geom1),
        "geom2_name": _mujoco_geom_name(model, mujoco, geom2),
        "body1": body1,
        "body2": body2,
        "body1_name": body1_name,
        "body2_name": body2_name,
    }


def _mujoco_geom_name(model: object, mujoco: object, geom_id: int) -> str | None:
    try:
        return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
    except Exception:
        return None


def _mujoco_body_name(model: object, mujoco: object, body_id: int) -> str | None:
    try:
        return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    except Exception:
        return None


def _normalize_mujoco_body_name(name: str | None) -> str:
    if name in {None, "world"}:
        return "base_link"
    return name


def _qpos_ranges(q_samples: list[tuple[float, ...]]) -> dict[str, float]:
    if not q_samples:
        return {}
    dof = len(q_samples[0])
    return {
        f"joint_{joint_index + 1}": (
            max(sample[joint_index] for sample in q_samples)
            - min(sample[joint_index] for sample in q_samples)
        )
        for joint_index in range(dof)
    }


def _write_trajectory_svg(
    *,
    output_path: Path,
    preview: dict[str, object],
    urdf_path: Path,
    config: WorkspaceSafetyConfig,
    q_samples: list[tuple[float, ...]],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    safety = preview["safety"]
    assert isinstance(safety, dict)
    allowed = safety["allowed"] is True
    status_color = "#15803d" if allowed else "#b91c1c"
    status_text = "PASS" if allowed else "WARNING"
    reasons = _preview_warning_reasons(safety)
    joint_paths = _joint_svg_paths(q_samples)
    eef_path = _eef_side_svg_path(urdf_path, config=config, q_samples=q_samples)
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="960" height="540" viewBox="0 0 960 540">',
        '<rect width="960" height="540" fill="#fafafa"/>',
        f'<text x="32" y="42" font-family="Arial" font-size="24" font-weight="700" fill="{status_color}">{status_text}: armctrl trajectory preview</text>',
        f'<text x="32" y="72" font-family="Arial" font-size="13" fill="#334155">{html.escape(str(preview["trajectory_path"]))}</text>',
        '<rect x="32" y="100" width="896" height="180" fill="#ffffff" stroke="#cbd5e1"/>',
        '<text x="48" y="126" font-family="Arial" font-size="15" font-weight="700" fill="#0f172a">Joint command traces</text>',
        *joint_paths,
        '<rect x="32" y="310" width="896" height="150" fill="#ffffff" stroke="#cbd5e1"/>',
        '<text x="48" y="336" font-family="Arial" font-size="15" font-weight="700" fill="#0f172a">End-effector side-view z trace</text>',
        eef_path,
        f'<text x="48" y="498" font-family="Arial" font-size="14" fill="{status_color}">{html.escape("; ".join(reasons) if reasons else "all configured gates passed")}</text>',
        '</svg>',
    ]
    output_path.write_text("\n".join(svg), encoding="utf-8")


def _write_trajectory_render(
    *,
    output_path: Path,
    preview: dict[str, object],
    urdf_path: Path,
    config: WorkspaceSafetyConfig,
    q_samples: list[tuple[float, ...]],
) -> str:
    suffix = output_path.suffix.lower()
    if suffix in {".html", ".htm"}:
        _write_trajectory_animation_html(
            output_path=output_path,
            preview=preview,
            urdf_path=urdf_path,
            config=config,
            q_samples=q_samples,
        )
        return "html"
    _write_trajectory_svg(
        output_path=output_path,
        preview=preview,
        urdf_path=urdf_path,
        config=config,
        q_samples=q_samples,
    )
    return "svg"


def _write_trajectory_animation_html(
    *,
    output_path: Path,
    preview: dict[str, object],
    urdf_path: Path,
    config: WorkspaceSafetyConfig,
    q_samples: list[tuple[float, ...]],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    safety = preview["safety"]
    assert isinstance(safety, dict)
    allowed = safety["allowed"] is True
    status_text = "PASS" if allowed else "WARNING"
    status_color = "#15803d" if allowed else "#b91c1c"
    reasons = _preview_warning_reasons(safety)
    frames = link_frame_positions(urdf_path, samples=q_samples)
    selected_indices = _animation_sample_indices(len(q_samples))
    link_order = list(frames[0]) if frames else []
    payload = {
        "title": "URDF kinematic animation",
        "status": status_text,
        "statusColor": status_color,
        "trajectoryPath": str(preview["trajectory_path"]),
        "urdfPath": str(urdf_path),
        "urdfXml": urdf_path.read_text(encoding="utf-8"),
        "urdfDirectoryUrl": _directory_file_url(urdf_path.parent),
        "warningReasons": reasons,
        "qSamples": [list(q_samples[index]) for index in selected_indices],
        "linkOrder": link_order,
        "linkFrames": [
            {
                name: list(position)
                for name, position in frames[index].items()
                if not config.simulation_link_frames
                or name in config.simulation_link_frames
                or name == link_order[0]
            }
            for index in selected_indices
        ],
        "allowedBoxes": [
            {
                "name": box.name,
                "min": list(box.min_m),
                "max": list(box.max_m),
            }
            for box in config.allowed_workspace_boxes
        ],
        "forbiddenBoxes": [
            {
                "name": box.name,
                "min": list(box.min_m),
                "max": list(box.max_m),
            }
            for box in config.forbidden_workspace_boxes
        ],
        "workspaceMinZ": config.workspace_min_m[2],
    }
    data_json = json.dumps(payload, ensure_ascii=True).replace("</", "<\\/")
    reasons_text = html.escape("; ".join(reasons) if reasons else "all configured gates passed")
    html_doc = _URDF_ANIMATION_HTML_TEMPLATE
    html_doc = html_doc.replace("__TITLE__", "armctrl URDF trajectory preview")
    html_doc = html_doc.replace("__STATUS__", status_text)
    html_doc = html_doc.replace("__STATUS_COLOR__", status_color)
    html_doc = html_doc.replace(
        "__TRAJECTORY_PATH__",
        html.escape(str(preview["trajectory_path"])),
    )
    html_doc = html_doc.replace("__REASONS__", reasons_text)
    html_doc = html_doc.replace("__PREVIEW_DATA__", data_json)
    output_path.write_text(html_doc, encoding="utf-8")


def _animation_sample_indices(sample_count: int, *, max_frames: int = 300) -> list[int]:
    if sample_count <= 0:
        return []
    if sample_count <= max_frames:
        return list(range(sample_count))
    return sorted(
        {
            round(index * (sample_count - 1) / (max_frames - 1))
            for index in range(max_frames)
        }
    )


def _directory_file_url(path: Path) -> str:
    return path.resolve().as_uri().rstrip("/") + "/"


_URDF_ANIMATION_HTML_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>__TITLE__</title>
  <style>
    body { margin: 0; font-family: Arial, sans-serif; background: #f8fafc; color: #0f172a; }
    header { padding: 18px 22px 10px; background: #ffffff; border-bottom: 1px solid #cbd5e1; }
    h1 { margin: 0 0 6px; font-size: 22px; color: __STATUS_COLOR__; }
    .meta { font-size: 13px; color: #475569; overflow-wrap: anywhere; }
    #viewer { width: 100vw; height: calc(100vh - 126px); display: block; }
    #hud { position: fixed; left: 16px; bottom: 14px; right: 16px; display: flex; gap: 12px; align-items: center; }
    #hud > div { background: rgba(255,255,255,0.92); border: 1px solid #cbd5e1; padding: 10px 12px; border-radius: 8px; box-shadow: 0 8px 24px rgba(15,23,42,0.12); }
    input[type="range"] { width: min(54vw, 560px); }
    button { border: 1px solid #94a3b8; background: #ffffff; color: #0f172a; padding: 7px 10px; border-radius: 6px; cursor: pointer; }
    button:hover { background: #f1f5f9; }
    .warn { color: __STATUS_COLOR__; font-weight: 700; }
  </style>
  <script type="importmap">
    {
      "imports": {
        "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
        "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
      }
    }
  </script>
</head>
<body>
  <header>
    <h1>__STATUS__: URDF kinematic animation</h1>
    <div class="meta">Trajectory: __TRAJECTORY_PATH__</div>
    <div class="meta warn">__REASONS__</div>
  </header>
  <canvas id="viewer"></canvas>
  <div id="hud">
    <div><button id="play">Pause</button></div>
    <div>Frame <span id="frame">0</span>/<span id="frameCount">0</span></div>
    <div><input id="slider" type="range" min="0" max="0" value="0"></div>
  </div>
  <script type="module">
    import * as THREE from 'three';
    import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

    const previewData = __PREVIEW_DATA__;
    const canvas = document.getElementById('viewer');
    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf8fafc);
    const camera = new THREE.PerspectiveCamera(48, 1, 0.02, 20);
    camera.position.set(1.2, -1.4, 0.9);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0.2, 0, 0.24);
    controls.update();

    scene.add(new THREE.HemisphereLight(0xffffff, 0x94a3b8, 2.4));
    const dir = new THREE.DirectionalLight(0xffffff, 1.1);
    dir.position.set(1, -1, 2);
    scene.add(dir);
    scene.add(new THREE.GridHelper(1.6, 16, 0x94a3b8, 0xcbd5e1));
    scene.add(new THREE.AxesHelper(0.18));

    const robotGroup = new THREE.Group();
    scene.add(robotGroup);
    let urdfRobot = null;
    let useUrdfRobot = false;

    function addBox(box, color, opacity) {
      const min = box.min;
      const max = box.max;
      const size = new THREE.Vector3(max[0] - min[0], max[1] - min[1], max[2] - min[2]);
      const center = new THREE.Vector3((max[0] + min[0]) / 2, (max[1] + min[1]) / 2, (max[2] + min[2]) / 2);
      const geom = new THREE.BoxGeometry(size.x, size.y, size.z);
      const mat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false });
      const mesh = new THREE.Mesh(geom, mat);
      mesh.position.copy(center);
      scene.add(mesh);
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geom), new THREE.LineBasicMaterial({ color }));
      edges.position.copy(center);
      scene.add(edges);
    }

    previewData.allowedBoxes.forEach(box => addBox(box, 0x16a34a, 0.045));
    previewData.forbiddenBoxes.forEach(box => addBox(box, 0xdc2626, 0.12));

    function sphere(position, radius, color) {
      const mesh = new THREE.Mesh(
        new THREE.SphereGeometry(radius, 20, 12),
        new THREE.MeshStandardMaterial({ color, roughness: 0.55 })
      );
      mesh.position.fromArray(position);
      return mesh;
    }

    function cylinderBetween(start, end, color) {
      const a = new THREE.Vector3().fromArray(start);
      const b = new THREE.Vector3().fromArray(end);
      const delta = new THREE.Vector3().subVectors(b, a);
      const length = delta.length();
      if (length < 1e-6) return null;
      const mesh = new THREE.Mesh(
        new THREE.CylinderGeometry(0.018, 0.018, length, 16),
        new THREE.MeshStandardMaterial({ color, roughness: 0.45 })
      );
      mesh.position.copy(a).addScaledVector(delta, 0.5);
      mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), delta.normalize());
      return mesh;
    }

    async function tryLoadUrdfRobot() {
      try {
        const module = await import('https://unpkg.com/urdf-loader@0.12.6/src/URDFLoader.js');
        const URDFLoader = module.default || module.URDFLoader;
        const loader = new URDFLoader();
        loader.workingPath = previewData.urdfDirectoryUrl;
        urdfRobot = loader.parse(previewData.urdfXml);
        urdfRobot.traverse(child => {
          if (child.isMesh) {
            child.castShadow = false;
            child.receiveShadow = false;
          }
        });
        scene.add(urdfRobot);
        useUrdfRobot = true;
      } catch (error) {
        const note = document.createElement('div');
        note.className = 'meta';
        note.textContent = 'URDFLoader mesh view unavailable in this browser context; showing FK skeleton fallback.';
        document.querySelector('header').appendChild(note);
      }
    }

    function setUrdfJoints(index) {
      if (!urdfRobot) return;
      const q = previewData.qSamples[index] || [];
      q.forEach((value, jointIndex) => {
        const names = [`joint${jointIndex + 1}`, `joint_${jointIndex + 1}`];
        names.forEach(name => {
          if (urdfRobot.joints && urdfRobot.joints[name]) {
            urdfRobot.joints[name].setJointValue(value);
          }
        });
      });
    }

    function drawFrame(index) {
      robotGroup.clear();
      if (useUrdfRobot) {
        setUrdfJoints(index);
        robotGroup.visible = false;
      } else {
        robotGroup.visible = true;
      }
      const frames = previewData.linkFrames[index] || {};
      const points = [[0, 0, 0]];
      previewData.linkOrder.forEach(name => {
        if (frames[name]) points.push(frames[name]);
      });
      points.forEach((point, pointIndex) => {
        robotGroup.add(sphere(point, pointIndex === points.length - 1 ? 0.032 : 0.024, pointIndex === points.length - 1 ? 0xf97316 : 0x2563eb));
        if (pointIndex > 0) {
          const link = cylinderBetween(points[pointIndex - 1], point, 0x334155);
          if (link) robotGroup.add(link);
        }
      });
      frameLabel.textContent = String(index);
      slider.value = String(index);
    }

    const slider = document.getElementById('slider');
    const frameLabel = document.getElementById('frame');
    const frameCount = document.getElementById('frameCount');
    const playButton = document.getElementById('play');
    const lastFrame = Math.max(0, previewData.linkFrames.length - 1);
    slider.max = String(lastFrame);
    frameCount.textContent = String(lastFrame);
    let frame = 0;
    let playing = true;
    slider.addEventListener('input', () => { frame = Number(slider.value); drawFrame(frame); });
    playButton.addEventListener('click', () => {
      playing = !playing;
      playButton.textContent = playing ? 'Pause' : 'Play';
    });

    function resize() {
      const width = canvas.clientWidth || window.innerWidth;
      const height = canvas.clientHeight || Math.max(300, window.innerHeight - 126);
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    }
    window.addEventListener('resize', resize);
    resize();
    await tryLoadUrdfRobot();
    drawFrame(0);

    let previous = 0;
    function animate(now) {
      requestAnimationFrame(animate);
      if (playing && now - previous > 80 && lastFrame > 0) {
        frame = (frame + 1) % (lastFrame + 1);
        drawFrame(frame);
        previous = now;
      }
      controls.update();
      renderer.render(scene, camera);
    }
    requestAnimationFrame(animate);
  </script>
</body>
</html>
"""


def _preview_warning_reasons(safety: dict[str, object]) -> list[str]:
    reasons: list[str] = []
    for check_name in ("clearance_check", "zone_check"):
        check = safety.get(check_name)
        if not isinstance(check, dict) or check.get("status") == "pass":
            continue
        violations = check.get("violations", [])
        if isinstance(violations, list) and violations:
            first = violations[0]
            if isinstance(first, dict):
                reasons.append(str(first.get("check", check_name)))
                continue
        reasons.append(check_name)
    return reasons


def _joint_svg_paths(q_samples: list[tuple[float, ...]]) -> list[str]:
    if not q_samples:
        return []
    colors = ("#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2")
    dof = len(q_samples[0])
    all_values = [value for sample in q_samples for value in sample]
    min_value = min(all_values)
    max_value = max(all_values)
    if max_value == min_value:
        max_value = min_value + 1.0
    paths: list[str] = []
    for joint_index in range(dof):
        points = []
        for sample_index, sample in enumerate(q_samples):
            x = 48 + sample_index * 848 / max(1, len(q_samples) - 1)
            y = 260 - (sample[joint_index] - min_value) * 112 / (max_value - min_value)
            points.append(f"{x:.2f},{y:.2f}")
        color = colors[joint_index % len(colors)]
        paths.append(
            f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="2"/>'
        )
        paths.append(
            f'<text x="{48 + joint_index * 90}" y="276" font-family="Arial" font-size="11" fill="{color}">joint_{joint_index + 1}</text>'
        )
    return paths


def _eef_side_svg_path(
    urdf_path: Path,
    *,
    config: WorkspaceSafetyConfig,
    q_samples: list[tuple[float, ...]],
) -> str:
    frames = link_frame_positions(urdf_path, samples=q_samples)
    values = [
        positions.get("eef_link", positions.get("link6", (0.0, 0.0, 0.0)))[2]
        for positions in frames
    ]
    if not values:
        return '<text x="48" y="390" font-family="Arial" font-size="12" fill="#64748b">no FK samples</text>'
    min_z = min(min(values), config.workspace_min_m[2])
    max_z = max(values)
    if max_z == min_z:
        max_z = min_z + 1.0
    points = []
    for index, value in enumerate(values):
        x = 48 + index * 848 / max(1, len(values) - 1)
        y = 438 - (value - min_z) * 82 / (max_z - min_z)
        points.append(f"{x:.2f},{y:.2f}")
    clearance_y = 438 - (config.workspace_min_m[2] - min_z) * 82 / (max_z - min_z)
    return "\n".join(
        [
            f'<line x1="48" y1="{clearance_y:.2f}" x2="896" y2="{clearance_y:.2f}" stroke="#ef4444" stroke-dasharray="6 4"/>',
            f'<polyline points="{" ".join(points)}" fill="none" stroke="#0f766e" stroke-width="3"/>',
            f'<text x="48" y="{clearance_y - 6:.2f}" font-family="Arial" font-size="11" fill="#ef4444">workspace min z</text>',
        ]
    )


def _read_trajectory_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _q_samples_from_rows(rows: Iterable[dict[str, str]]) -> list[tuple[float, ...]]:
    samples: list[tuple[float, ...]] = []
    for row in rows:
        q_keys = sorted(
            (key for key in row if key.startswith("q_cmd_")),
            key=lambda key: int(key.removeprefix("q_cmd_")),
        )
        samples.append(tuple(float(row[key]) for key in q_keys))
    return samples


def _trajectory_metrics(q_samples: list[tuple[float, ...]]) -> dict[str, object]:
    if not q_samples:
        return {"sample_count": 0, "joint_ranges_rad": {}}
    dof = len(q_samples[0])
    ranges: dict[str, float] = {}
    for joint_index in range(dof):
        values = [sample[joint_index] for sample in q_samples]
        ranges[f"joint_{joint_index + 1}"] = max(values) - min(values)
    return {
        "sample_count": len(q_samples),
        "joint_ranges_rad": ranges,
    }


def _workspace_metrics(
    urdf_path: Path,
    *,
    config: WorkspaceSafetyConfig,
    q_samples: list[tuple[float, ...]],
) -> dict[str, object]:
    frame_positions = link_frame_positions(urdf_path, samples=q_samples)
    frame_z: dict[str, list[float]] = {}
    for positions in frame_positions:
        for link_name, position in positions.items():
            if config.simulation_link_frames and link_name not in config.simulation_link_frames:
                continue
            frame_z.setdefault(link_name, []).append(position[2])
    return {
        "link_frame_z_range_m": {
            link_name: {
                "min": min(values),
                "max": max(values),
            }
            for link_name, values in frame_z.items()
        }
    }
