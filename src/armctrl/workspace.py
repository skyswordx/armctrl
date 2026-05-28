from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class WorkspaceDecision:
    status: str
    violations: list[dict[str, object]]


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
