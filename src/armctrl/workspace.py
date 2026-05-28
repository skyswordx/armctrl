from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class WorkspaceDecision:
    status: str
    violations: list[dict[str, object]]
    method: str


@dataclass(frozen=True)
class WorkspaceSafetyConfig:
    workspace_min_m: tuple[float, float, float]
    workspace_max_m: tuple[float, float, float]

    @classmethod
    def from_yaml(cls, path: Path) -> "WorkspaceSafetyConfig":
        return cls(
            workspace_min_m=_read_float_triplet(path, "workspace_min_m"),
            workspace_max_m=_read_float_triplet(path, "workspace_max_m"),
        )


def evaluate_workspace_clearance(
    config: WorkspaceSafetyConfig,
    *,
    q_center: tuple[float, ...],
    amplitude_rad: float,
) -> WorkspaceDecision:
    # Conservative proxy until a full FK/collision model is restored.
    # The X5 safe center keeps joint2 and joint3 around 0.3 rad; lower joint2
    # values previously correlated with the end-effector dipping toward the table.
    min_clearance_proxy = q_center[1] - amplitude_rad
    required_proxy = 0.1
    violations: list[dict[str, object]] = []
    if min_clearance_proxy < required_proxy:
        violations.append(
            {
                "check": "min_clearance_proxy",
                "value": min_clearance_proxy,
                "minimum": required_proxy,
                "workspace_min_z_m": config.workspace_min_m[2],
            }
        )
    return WorkspaceDecision(
        status="fail" if violations else "pass",
        violations=violations,
        method="joint2_clearance_proxy",
    )


@dataclass(frozen=True)
class UrdfJointFrame:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]


def evaluate_workspace_fk_clearance(
    urdf_path: Path,
    config: WorkspaceSafetyConfig,
    *,
    samples: list[tuple[float, ...]],
) -> WorkspaceDecision:
    chain = _serial_joint_chain(urdf_path)
    violations: list[dict[str, object]] = []
    minimum_z = config.workspace_min_m[2]
    for sample_index, q_sample in enumerate(samples):
        frame_positions = _forward_kinematics(chain, q_sample)
        for link_name, position in frame_positions.items():
            z_m = position[2]
            if z_m < minimum_z:
                violations.append(
                    {
                        "check": "fk_min_z",
                        "sample_index": sample_index,
                        "link": link_name,
                        "z_m": z_m,
                        "minimum_z_m": minimum_z,
                    }
                )
                break
    return WorkspaceDecision(
        status="fail" if violations else "pass",
        violations=violations,
        method="urdf_fk_frame_clearance",
    )


def _read_float_triplet(path: Path, key: str) -> tuple[float, float, float]:
    prefix = f"{key}:"
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped.startswith(prefix):
            continue
        _, raw_value = stripped.split(":", maxsplit=1)
        values = raw_value.strip().removeprefix("[").removesuffix("]").split(",")
        triplet = tuple(float(value.strip()) for value in values)
        if len(triplet) != 3:
            raise ValueError(f"{key} must contain exactly three values")
        return triplet
    raise KeyError(f"{key} not found in {path}")


def _serial_joint_chain(path: Path) -> tuple[UrdfJointFrame, ...]:
    root = ET.parse(path).getroot()
    joints_by_parent: dict[str, list[UrdfJointFrame]] = {}
    child_links: set[str] = set()
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        origin = joint.find("origin")
        axis = joint.find("axis")
        frame = UrdfJointFrame(
            name=joint.attrib["name"],
            joint_type=joint.attrib.get("type", "fixed"),
            parent=parent.attrib["link"],
            child=child.attrib["link"],
            xyz=_parse_vector(origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0"),
            rpy=_parse_vector(origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"),
            axis=_parse_vector(axis.attrib.get("xyz", "0 0 1") if axis is not None else "0 0 1"),
        )
        joints_by_parent.setdefault(frame.parent, []).append(frame)
        child_links.add(frame.child)

    root_links = [
        link.attrib["name"]
        for link in root.findall("link")
        if link.attrib["name"] not in child_links
    ]
    if not root_links:
        raise ValueError(f"URDF has no root link: {path}")
    chain: list[UrdfJointFrame] = []
    current_link = root_links[0]
    while current_link in joints_by_parent:
        children = joints_by_parent[current_link]
        if len(children) != 1:
            raise ValueError("workspace FK clearance currently expects a serial chain")
        joint = children[0]
        chain.append(joint)
        current_link = joint.child
    return tuple(chain)


def _forward_kinematics(
    chain: tuple[UrdfJointFrame, ...],
    q_sample: tuple[float, ...],
) -> dict[str, tuple[float, float, float]]:
    transform = _identity()
    positions: dict[str, tuple[float, float, float]] = {}
    active_index = 0
    for joint in chain:
        transform = _matmul(transform, _origin_transform(joint.xyz, joint.rpy))
        if joint.joint_type != "fixed":
            q = q_sample[active_index] if active_index < len(q_sample) else 0.0
            transform = _matmul(transform, _axis_rotation(joint.axis, q))
            active_index += 1
        positions[joint.child] = (transform[0][3], transform[1][3], transform[2][3])
    return positions


def _parse_vector(raw: str) -> tuple[float, float, float]:
    values = tuple(float(value) for value in raw.split())
    if len(values) != 3:
        raise ValueError(f"expected 3-vector, got {raw!r}")
    return values


def _identity() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _origin_transform(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> list[list[float]]:
    roll, pitch, yaw = rpy
    rotation = _matmul3(_rotation_z(yaw), _matmul3(_rotation_y(pitch), _rotation_x(roll)))
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], xyz[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], xyz[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _axis_rotation(axis: tuple[float, float, float], angle: float) -> list[list[float]]:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm == 0.0:
        raise ValueError("joint axis must be non-zero")
    x, y, z = x / norm, y / norm, z / norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return [
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s, 0.0],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s, 0.0],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_x(angle: float) -> list[list[float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _rotation_y(angle: float) -> list[list[float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def _rotation_z(angle: float) -> list[list[float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def _matmul3(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def _matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][k] * b[k][col] for k in range(4)) for col in range(4)]
        for row in range(4)
    ]
