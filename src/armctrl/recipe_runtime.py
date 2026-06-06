from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path

from armctrl.limits import UrdfJointLimits, evaluate_joint_limit_samples
from armctrl.recipes import Recipe, RecipeCatalog
from armctrl.simulation import TrajectoryPreviewer
from armctrl.workspace import WorkspaceSafetyConfig, evaluate_workspace_fk_clearance


DEFAULT_RECIPE_START = (0.0, 0.30, 0.30, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class RecipePlanRequest:
    recipe_name: str
    start_joints: tuple[float, ...]
    sample_hz: float
    duration_s: float
    urdf_path: str
    safe_config_path: str
    output_dir: Path
    render_path: Path | None = None


@dataclass(frozen=True)
class RecipeEefSeedRequest:
    plan_dir: Path


@dataclass(frozen=True)
class RecipeAgentPresetContractRequest:
    plan_dir: Path


class RecipePlanner:
    def __init__(self, catalog: RecipeCatalog | None = None) -> None:
        self._catalog = catalog or RecipeCatalog.default()

    def write_plan(self, request: RecipePlanRequest) -> dict[str, object]:
        recipe = self._catalog.get(request.recipe_name)
        request.output_dir.mkdir(parents=True, exist_ok=True)
        trajectory_path = request.output_dir / "planned_trajectory.csv"
        preview_path = request.output_dir / "trajectory_preview.json"
        manifest_path = request.output_dir / "manifest.json"
        eef_seed_path = request.output_dir / "eef_seed.json"
        render_path = request.render_path

        q_samples = recipe_joint_samples(recipe, request=request)
        rows = _rows_from_samples(q_samples, sample_hz=request.sample_hz)
        with trajectory_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

        preview = TrajectoryPreviewer().preview(
            trajectory_path=trajectory_path,
            urdf_path=Path(request.urdf_path),
            safe_config_path=Path(request.safe_config_path),
            render_path=render_path,
        )
        preview_path.write_text(
            json.dumps(preview, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        limits = evaluate_joint_limit_samples(
            UrdfJointLimits.from_urdf(Path(request.urdf_path)),
            samples=q_samples,
        )
        workspace = evaluate_workspace_fk_clearance(
            Path(request.urdf_path),
            WorkspaceSafetyConfig.from_yaml(Path(request.safe_config_path)),
            samples=q_samples,
        )
        step = _evaluate_joint_steps(
            q_samples,
            max_joint_step_rad=WorkspaceSafetyConfig.from_yaml(
                Path(request.safe_config_path)
            ).max_joint_step_rad,
        )
        simulation_status = "pass" if preview["safety"]["allowed"] is True else "fail"
        allowed = (
            limits.status == "pass"
            and workspace.status == "pass"
            and step.status == "pass"
            and simulation_status == "pass"
        )
        artifacts = {
            "planned_trajectory": str(trajectory_path),
            "trajectory_preview": str(preview_path),
            "manifest": str(manifest_path),
            "eef_seed": str(eef_seed_path),
        }
        if render_path is not None:
            artifacts["trajectory_render"] = str(render_path)
        eef_seed = _eef_seed_payload(
            plan_dir=request.output_dir,
            recipe=recipe,
            trajectory_path=trajectory_path,
            final_joints=q_samples[-1],
        )
        eef_seed_path.write_text(
            json.dumps(eef_seed, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        manifest = {
            "schema": "armctrl.recipe_plan_manifest.v1",
            "recipe": recipe.to_json(),
            "request": {
                "start_joints": list(request.start_joints),
                "sample_hz": request.sample_hz,
                "duration_s": request.duration_s,
                "urdf_path": request.urdf_path,
                "safe_config_path": request.safe_config_path,
            },
            "safety": {
                "allowed": allowed,
                "checks": {
                    "urdf_limit_check": {
                        "status": limits.status,
                        "violations": limits.violations,
                    },
                    "workspace_clearance_check": {
                        "status": workspace.status,
                        "method": workspace.method,
                        "violations": workspace.violations,
                    },
                    "trajectory_step_check": {
                        "status": step.status,
                        "violations": step.violations,
                    },
                    "simulation_check": {
                        "status": simulation_status,
                        "backend": preview["backend"]["selected"],
                        "zone_check": preview["safety"]["zone_check"],
                        "clearance_check": preview["safety"]["clearance_check"],
                    },
                },
            },
            "artifacts": artifacts,
        }
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return {
            "recipe": recipe.to_json(),
            "steps": [step.to_json() for step in recipe.steps],
            "artifacts": artifacts,
            "simulation_gate": preview["safety"],
            "handoff": _recipe_handoff_payload(
                plan_dir=request.output_dir,
                recipe=recipe,
                trajectory_path=trajectory_path,
                final_joints=q_samples[-1],
            ),
            "agent_runtime_profile": _recipe_agent_runtime_profile(
                plan_dir=request.output_dir,
            ),
            "artifact_safety": {
                "allowed": allowed,
                "urdf_limit_check": {
                    "status": limits.status,
                    "violation_count": len(limits.violations),
                },
                "workspace_clearance_check": {
                    "status": workspace.status,
                    "method": workspace.method,
                    "violation_count": len(workspace.violations),
                },
                "trajectory_step_check": {
                    "status": step.status,
                    "violation_count": len(step.violations),
                },
                "simulation_check": {
                    "status": simulation_status,
                    "backend": preview["backend"]["selected"],
                    "violation_count": len(
                        preview["safety"]["zone_check"]["violations"]
                    )
                    + len(preview["safety"]["clearance_check"]["violations"]),
                },
            },
        }


class RecipeEefSeedExporter:
    def export(self, request: RecipeEefSeedRequest) -> dict[str, object]:
        manifest_path = request.plan_dir / "manifest.json"
        trajectory_path = request.plan_dir / "planned_trajectory.csv"
        missing = [
            str(path) for path in (manifest_path, trajectory_path) if not path.exists()
        ]
        if missing:
            raise FileNotFoundError(
                "missing recipe plan artifacts required for eef seed export: "
                + ", ".join(missing)
            )
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        final_joints = _read_final_joint_sample(trajectory_path)
        return _eef_seed_payload(
            plan_dir=request.plan_dir,
            recipe=manifest["recipe"],
            trajectory_path=trajectory_path,
            final_joints=final_joints,
        )


class RecipeAgentPresetContractExporter:
    def export(self, request: RecipeAgentPresetContractRequest) -> dict[str, object]:
        manifest_path = request.plan_dir / "manifest.json"
        trajectory_path = request.plan_dir / "planned_trajectory.csv"
        eef_seed_path = request.plan_dir / "eef_seed.json"
        missing = [
            str(path)
            for path in (manifest_path, trajectory_path, eef_seed_path)
            if not path.exists()
        ]
        if missing:
            raise FileNotFoundError(
                "missing recipe plan artifacts required for agent preset contract export: "
                + ", ".join(missing)
            )
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        eef_seed = json.loads(eef_seed_path.read_text(encoding="utf-8"))
        final_joints = tuple(eef_seed["final_joints"])
        handoff = _recipe_handoff_payload(
            plan_dir=request.plan_dir,
            recipe=manifest["recipe"],
            trajectory_path=trajectory_path,
            final_joints=final_joints,
        )
        return {
            "schema": "armctrl.recipe_agent_preset_contract.v1",
            "movement_allowed": False,
            "plan_dir": str(request.plan_dir),
            "recipe": manifest["recipe"],
            "safety_summary": {
                "allowed": manifest["safety"]["allowed"],
                **manifest["safety"]["checks"],
            },
            "agent_runtime_profile": _recipe_agent_runtime_profile(
                plan_dir=request.plan_dir
            ),
            "eef_seed": eef_seed,
            "handoff": handoff,
            "required_artifacts": {
                "manifest": str(manifest_path),
                "planned_trajectory": str(trajectory_path),
                "eef_seed": str(eef_seed_path),
            },
            "ordered_steps": [
                {
                    "id": "export_eef_seed",
                    "command": handoff["suggested_cli"]["export_eef_seed"],
                    "depends_on": [],
                    "parallel_safe_with": [],
                },
                {
                    "id": "eef_synthesize_preview_from_recipe",
                    "command": handoff["suggested_cli"][
                        "eef_synthesize_preview_from_recipe"
                    ],
                    "depends_on": ["export_eef_seed"],
                    "parallel_safe_with": [],
                },
            ],
            "next_steps": [
                handoff["suggested_cli"]["export_eef_seed"],
                handoff["suggested_cli"]["eef_synthesize_preview_from_recipe"],
            ],
            "notes": [
                "This is a non-hardware preset-action handoff contract for Agent callers.",
                "It consolidates the reviewed recipe posture, safety summary, and EEF seed into one machine-readable surface.",
            ],
        }


def recipe_joint_samples(
    recipe: Recipe,
    *,
    request: RecipePlanRequest,
) -> list[tuple[float, ...]]:
    target = _recipe_joint_target(recipe, start_joints=request.start_joints)
    sample_count = int(round(request.duration_s * request.sample_hz)) + 1
    if sample_count <= 1:
        return [target]
    samples: list[tuple[float, ...]] = []
    for sample_index in range(sample_count):
        alpha = _smoothstep(sample_index / (sample_count - 1))
        samples.append(
            tuple(
                start + (goal - start) * alpha
                for start, goal in zip(request.start_joints, target)
            )
        )
    return samples


def _recipe_joint_target(
    recipe: Recipe,
    *,
    start_joints: tuple[float, ...],
) -> tuple[float, ...]:
    for step in recipe.steps:
        if step.kind == "joint_target" and step.target is not None:
            return step.target
    return start_joints


def _rows_from_samples(
    samples: list[tuple[float, ...]],
    *,
    sample_hz: float,
) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for sample_index, sample in enumerate(samples):
        row = {"time_s": f"{sample_index / sample_hz:.6f}"}
        for joint_index, value in enumerate(sample, start=1):
            row[f"q_cmd_{joint_index}"] = f"{value:.6f}"
        rows.append(row)
    return rows


def _read_final_joint_sample(trajectory_path: Path) -> tuple[float, ...]:
    with trajectory_path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)
        rows = list(reader)
    if not rows:
        raise ValueError("recipe planned trajectory must contain at least one sample")
    final_row = rows[-1]
    return tuple(float(final_row[f"q_cmd_{joint_index}"]) for joint_index in range(1, 7))


def _eef_seed_payload(
    *,
    plan_dir: Path,
    recipe: Recipe | dict[str, object],
    trajectory_path: Path,
    final_joints: tuple[float, ...],
) -> dict[str, object]:
    start_joint_tokens = " ".join(f"{value:.6f}" for value in final_joints)
    recipe_payload = recipe.to_json() if isinstance(recipe, Recipe) else recipe
    return {
        "schema": "armctrl.recipe_eef_seed.v1",
        "movement_allowed": False,
        "plan_dir": str(plan_dir),
        "recipe": recipe_payload,
        "source_trajectory": str(trajectory_path),
        "final_joints": list(final_joints),
        "suggested_cli": {
            "eef_synthesize_preview": (
                "uv run armctrl eef synthesize-preview "
                f"--plan-dir <eef-plan-dir> --start-joints {start_joint_tokens} --json"
            )
        },
        "notes": [
            "This export is a non-hardware handoff from a reviewed recipe posture into later EEF preview work.",
            "Use the final recipe joint sample as the start state when the next bounded EEF motion should assume the recipe has already been executed.",
        ],
    }


def _recipe_handoff_payload(
    *,
    plan_dir: Path,
    recipe: Recipe | dict[str, object],
    trajectory_path: Path,
    final_joints: tuple[float, ...],
) -> dict[str, object]:
    return {
        "recipe_plan_dir": str(plan_dir),
        "eef_seed_artifact": str(plan_dir / "eef_seed.json"),
        "eef_seed": _eef_seed_payload(
            plan_dir=plan_dir,
            recipe=recipe,
            trajectory_path=trajectory_path,
            final_joints=final_joints,
        ),
        "suggested_cli": {
            "export_eef_seed": (
                f"uv run armctrl recipe export-eef-seed --plan-dir {plan_dir} --json"
            ),
            "eef_synthesize_preview_from_recipe": (
                "uv run armctrl eef synthesize-preview "
                f"--plan-dir <eef-plan-dir> --recipe-plan-dir {plan_dir} --json"
            ),
        },
    }


def _recipe_agent_runtime_profile(*, plan_dir: Path) -> dict[str, object]:
    return {
        "schema": "armctrl.agent_runtime_profile.v1",
        "profile": "recipe_preset_to_eef_preview",
        "movement_allowed": False,
        "recipe_plan_dir": str(plan_dir),
        "preferred_next_surface": "armctrl.eef.synthesize_preview",
        "review_chain": "shared_simulation_preview",
        "required_artifacts": {
            "eef_seed": str(plan_dir / "eef_seed.json"),
            "planned_trajectory": str(plan_dir / "planned_trajectory.csv"),
            "manifest": str(plan_dir / "manifest.json"),
        },
    }


def _smoothstep(value: float) -> float:
    clamped = max(0.0, min(1.0, value))
    return clamped * clamped * (3.0 - 2.0 * clamped)


def _evaluate_joint_steps(
    q_samples: list[tuple[float, ...]],
    *,
    max_joint_step_rad: float,
) -> _StepDecision:
    violations: list[dict[str, object]] = []
    for sample_index, (previous, current) in enumerate(zip(q_samples, q_samples[1:]), start=1):
        for joint_index, (previous_q, current_q) in enumerate(zip(previous, current), start=1):
            step = abs(current_q - previous_q)
            if step <= max_joint_step_rad:
                continue
            violations.append(
                {
                    "sample_index": sample_index,
                    "joint": joint_index,
                    "value": step,
                    "maximum": max_joint_step_rad,
                }
            )
    return _StepDecision(
        status="fail" if violations else "pass",
        violations=violations,
    )


@dataclass(frozen=True)
class _StepDecision:
    status: str
    violations: list[dict[str, object]]
