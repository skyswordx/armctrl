from __future__ import annotations

import csv
from dataclasses import dataclass
import html
import importlib
import importlib.util
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
            _write_trajectory_svg(
                output_path=render_path,
                preview=payload,
                urdf_path=urdf_path,
                config=config,
                q_samples=q_samples,
            )
            payload["render"] = {
                "status": "written",
                "path": str(render_path),
                "format": "svg",
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
