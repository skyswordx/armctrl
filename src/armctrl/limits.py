from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class JointLimit:
    name: str
    lower: float
    upper: float


@dataclass(frozen=True)
class LimitDecision:
    status: str
    violations: list[dict[str, object]]


@dataclass(frozen=True)
class UrdfJointLimits:
    joints: tuple[JointLimit, ...]

    @classmethod
    def from_urdf(cls, path: Path) -> "UrdfJointLimits":
        root = ET.parse(path).getroot()
        joints: list[JointLimit] = []
        for joint in root.findall("joint"):
            if joint.attrib.get("type") == "fixed":
                continue
            limit = joint.find("limit")
            if limit is None:
                continue
            lower = limit.attrib.get("lower")
            upper = limit.attrib.get("upper")
            if lower is None or upper is None:
                continue
            joints.append(
                JointLimit(
                    name=joint.attrib["name"],
                    lower=float(lower),
                    upper=float(upper),
                )
            )
        return cls(joints=tuple(joints))


def evaluate_joint_limits(
    limits: UrdfJointLimits,
    *,
    q_center: tuple[float, ...],
    amplitude_rad: float,
) -> LimitDecision:
    violations: list[dict[str, object]] = []
    for index, (joint, center) in enumerate(zip(limits.joints, q_center), start=1):
        planned_lower = center - amplitude_rad
        planned_upper = center + amplitude_rad
        if planned_lower < joint.lower or planned_upper > joint.upper:
            violations.append(
                {
                    "joint_index": index,
                    "joint_name": joint.name,
                    "planned_lower": planned_lower,
                    "planned_upper": planned_upper,
                    "urdf_lower": joint.lower,
                    "urdf_upper": joint.upper,
                }
            )
    return LimitDecision(
        status="fail" if violations else "pass",
        violations=violations,
    )
