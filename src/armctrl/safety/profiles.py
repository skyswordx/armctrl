from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotionLimits:
    workspace_min: tuple[float, float, float] = (0.05, -0.45, 0.02)
    workspace_max: tuple[float, float, float] = (0.75, 0.45, 0.65)
    max_translation_step_m: float = 0.005
    max_rotation_step_rad: float = 0.05
    min_gripper_pos: float = 0.0
    max_gripper_pos: float = 0.12
    max_gripper_step: float = 0.003

    def contains_position(self, xyz: tuple[float, float, float]) -> bool:
        return all(
            self.workspace_min[index] <= xyz[index] <= self.workspace_max[index]
            for index in range(3)
        )

    def clamp_gripper_pos(self, position: float) -> float:
        return max(self.min_gripper_pos, min(self.max_gripper_pos, position))
