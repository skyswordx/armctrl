from __future__ import annotations

import csv
from dataclasses import dataclass
import importlib.util
from pathlib import Path
from typing import Iterable

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


BACKENDS: tuple[BackendSpec, ...] = (
    BackendSpec(
        name="pinocchio_coal",
        role="lightweight URDF geometry collision checks",
        modules=("pinocchio", "coal"),
    ),
    BackendSpec(
        name="mujoco",
        role="contact and dynamics simulation preview",
        modules=("mujoco",),
    ),
    BackendSpec(
        name="moveit",
        role="ROS planning-scene state validity and collision oracle",
        modules=("moveit_commander",),
    ),
    BackendSpec(
        name="figaroh",
        role="SysID excitation optimization and identification handoff",
        modules=("figaroh",),
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
        return {
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


def _backend_status(spec: BackendSpec) -> dict[str, object]:
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
        "status": "available" if importable else "missing",
        "modules": module_statuses,
    }


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
) -> dict[str, object]:
    if selected_backend == "pinocchio_coal":
        return _pinocchio_coal_check(urdf_path=urdf_path, q_samples=q_samples)
    if selected_backend == "mujoco":
        return _mujoco_load_check(urdf_path=urdf_path)
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
        model, collision_model, _visual_model = pin.buildModelsFromUrdf(str(urdf_path))
        collision_model.addAllCollisionPairs()
        data = model.createData()
        collision_data = pin.GeometryData(collision_model)
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
                    return {
                        "status": "fail",
                        "method": "pinocchio_coal",
                        "violation": {
                            "sample_index": sample_index,
                            "collision_pair_index": pair_index,
                        },
                    }
        return {
            "status": "pass",
            "method": "pinocchio_coal",
            "checked_samples": len(q_samples),
        }
    except Exception as error:  # pragma: no cover - depends on native geometry stack
        return {
            "status": "not_evaluated",
            "method": "pinocchio_coal",
            "reason": str(error),
        }


def _mujoco_load_check(*, urdf_path: Path) -> dict[str, object]:
    try:
        import mujoco
    except ModuleNotFoundError as error:
        return {
            "status": "not_available",
            "method": "mujoco",
            "reason": f"module not importable: {error.name}",
        }
    try:
        mujoco.MjModel.from_xml_path(str(urdf_path))
    except Exception as error:  # pragma: no cover - depends on MuJoCo/native meshes
        return {
            "status": "not_evaluated",
            "method": "mujoco",
            "reason": str(error),
        }
    return {
        "status": "pass",
        "method": "mujoco_load",
        "reason": "MuJoCo loaded the URDF model; contact rollout is reserved for a calibrated MJCF scene.",
    }


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
