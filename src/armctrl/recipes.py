from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class RecipeStep:
    kind: str
    description: str
    target: tuple[float, ...] | None = None

    def to_json(self) -> dict[str, object]:
        payload: dict[str, object] = {
            "kind": self.kind,
            "description": self.description,
        }
        if self.target is not None:
            payload["target"] = list(self.target)
        return payload


@dataclass(frozen=True)
class Recipe:
    name: str
    summary: str
    moves_hardware: bool
    steps: tuple[RecipeStep, ...]

    def to_json(self) -> dict[str, object]:
        return {
            "name": self.name,
            "summary": self.summary,
            "moves_hardware": self.moves_hardware,
        }


class RecipeCatalog:
    def __init__(self, recipes: Iterable[Recipe]) -> None:
        self._recipes = {recipe.name: recipe for recipe in recipes}

    @classmethod
    def default(cls) -> "RecipeCatalog":
        return cls(
            [
                Recipe(
                    name="damping",
                    summary="Enter passive damping through the hardware backend.",
                    moves_hardware=True,
                    steps=(RecipeStep("mode", "request damping mode"),),
                ),
                Recipe(
                    name="hold-current",
                    summary="Hold the current measured pose when supported by the backend.",
                    moves_hardware=True,
                    steps=(RecipeStep("mode", "hold current measured pose"),),
                ),
                Recipe(
                    name="home",
                    summary="Move to the conservative X5 safe center.",
                    moves_hardware=True,
                    steps=(
                        RecipeStep(
                            "joint_target",
                            "move to safe center",
                            (0.0, 0.30, 0.30, 0.0, 0.0, 0.0),
                        ),
                    ),
                ),
                Recipe(
                    name="observe-front",
                    summary="Point the arm to a front observation posture.",
                    moves_hardware=True,
                    steps=(RecipeStep("joint_target", "front observation posture"),),
                ),
                Recipe(
                    name="pregrasp-table",
                    summary="Prepare a conservative table pregrasp posture.",
                    moves_hardware=True,
                    steps=(RecipeStep("joint_target", "table pregrasp posture"),),
                ),
                Recipe(
                    name="retreat-safe",
                    summary="Retreat from the workspace into a safe posture.",
                    moves_hardware=True,
                    steps=(RecipeStep("joint_target", "safe retreat posture"),),
                ),
            ]
        )

    def list_recipes(self) -> list[Recipe]:
        return [self._recipes[name] for name in sorted(self._recipes)]

    def get(self, name: str) -> Recipe:
        try:
            return self._recipes[name]
        except KeyError as exc:
            raise KeyError(f"unknown recipe: {name}") from exc
