import json
import subprocess
import sys
from pathlib import Path

from armctrl.recipes import RecipeCatalog
from armctrl.safety import SafetyGate


def _project_armctrl_executable() -> str:
    script_dir = Path(sys.executable).resolve().parent
    if sys.platform == "win32":
        return str(script_dir / "armctrl.exe")
    return str(script_dir / "armctrl")


def test_catalog_lists_bounded_agent_recipes() -> None:
    catalog = RecipeCatalog.default()

    names = [recipe.name for recipe in catalog.list_recipes()]

    assert names == [
        "damping",
        "hold-current",
        "home",
        "observe-front",
        "pregrasp-table",
        "retreat-safe",
    ]


def test_safety_gate_blocks_hardware_recipes_without_plan_only() -> None:
    catalog = RecipeCatalog.default()
    recipe = catalog.get("home")

    result = SafetyGate().evaluate(recipe, plan_only=False)

    assert result.allowed is False
    assert result.reason == "hardware recipe requires an execution backend"


def test_cli_lists_recipes_as_json() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "recipe", "list", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_catalog.v1"
    assert payload["recipes"][0]["name"] == "damping"
    assert payload["recipes"][0]["risk_level"] == "hardware"


def test_cli_dry_run_returns_plan_and_safety_gate() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "recipe", "plan", "home", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["plan_only"] is True
    assert payload["schema"] == "armctrl.recipe_plan.v1"
    assert payload["recipe"]["name"] == "home"
    assert payload["safety"]["allowed"] is True
    assert payload["safety"]["required_backend"] == "arx5-interface"
    assert payload["safety"]["dry_run"] is True
    assert payload["safety"]["movement_allowed"] is False
    assert payload["safety"]["risk_explanation"] == [
        "recipe would move hardware when execution is enabled",
        "current response is a dry-run preview only",
        "planned joint targets must pass backend, limit, and workspace checks before execution",
    ]
    assert payload["steps"][0]["kind"] == "joint_target"


def test_packaged_cli_entrypoint_lists_recipes() -> None:
    completed = subprocess.run(
        [_project_armctrl_executable(), "recipe", "list", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["recipes"][-1]["name"] == "retreat-safe"


def test_cli_execute_rejects_without_backend() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "recipe", "execute", "home", "--json"],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.recipe_execution.v1"
    assert payload["recipe"]["name"] == "home"
    assert payload["safety"]["allowed"] is False
    assert payload["safety"]["required_backend"] == "arx5-interface"
    assert payload["executor"]["status"] == "blocked"
    assert payload["executor"]["safety_gate_required"] is True


def test_cli_recipe_plan_can_write_simulation_gated_artifacts(tmp_path: Path) -> None:
    output_dir = tmp_path / "recipe-plan"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--output",
            str(output_dir),
            "--render",
            "trajectory_preview.html",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    manifest_path = output_dir / "manifest.json"
    trajectory_path = output_dir / "planned_trajectory.csv"
    preview_path = output_dir / "trajectory_preview.json"
    eef_seed_path = output_dir / "eef_seed.json"
    render_path = output_dir / "trajectory_preview.html"

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_plan.v1"
    assert payload["artifacts"]["manifest"] == str(manifest_path)
    assert payload["artifacts"]["planned_trajectory"] == str(trajectory_path)
    assert payload["artifacts"]["trajectory_preview"] == str(preview_path)
    assert payload["artifacts"]["eef_seed"] == str(eef_seed_path)
    assert payload["artifacts"]["trajectory_render"] == str(render_path)
    assert payload["agent_runtime_profile"]["schema"] == "armctrl.agent_runtime_profile.v1"
    assert payload["agent_runtime_profile"]["profile"] == "recipe_preset_to_eef_preview"
    assert payload["agent_runtime_profile"]["recipe_plan_dir"] == str(output_dir)
    assert payload["agent_runtime_profile"]["preferred_next_surface"] == "armctrl.eef.synthesize_preview"
    assert payload["artifact_safety"]["allowed"] is True
    assert payload["artifact_safety"]["simulation_check"]["status"] == "pass"
    assert any("recipe export-eef-seed" in step for step in payload["next_steps"])
    assert any("recipe execute home --backend sim" in step for step in payload["next_steps"])
    assert manifest_path.exists()
    assert trajectory_path.exists()
    assert preview_path.exists()
    assert eef_seed_path.exists()
    assert render_path.exists()

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema"] == "armctrl.recipe_plan_manifest.v1"
    assert manifest["recipe"]["name"] == "home"
    assert manifest["safety"]["allowed"] is True
    assert manifest["safety"]["checks"]["simulation_check"]["status"] == "pass"
    eef_seed = json.loads(eef_seed_path.read_text(encoding="utf-8"))
    assert eef_seed["schema"] == "armctrl.recipe_eef_seed.v1"
    assert eef_seed["final_joints"] == [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]


def test_cli_recipe_export_eef_seed_rejects_missing_plan_artifacts(
    tmp_path: Path,
) -> None:
    missing_dir = tmp_path / "missing-recipe-plan"
    missing_dir.mkdir()

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "export-eef-seed",
            "--plan-dir",
            str(missing_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.recipe_eef_seed.v1"
    assert payload["error"]["code"] == "missing_recipe_plan_artifacts"


def test_cli_recipe_export_eef_seed_reads_final_joint_sample(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "recipe-plan-seed"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "export-eef-seed",
            "--plan-dir",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_eef_seed.v1"
    assert payload["recipe"]["name"] == "home"
    assert payload["final_joints"] == [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    assert "--start-joints 0.000000 0.300000 0.300000 0.000000 0.000000 0.000000" in payload["suggested_cli"]["eef_synthesize_preview"]


def test_cli_recipe_export_agent_preset_contract_rejects_missing_plan_artifacts(
    tmp_path: Path,
) -> None:
    missing_dir = tmp_path / "missing-recipe-agent-contract"
    missing_dir.mkdir()

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "export-agent-preset-contract",
            "--plan-dir",
            str(missing_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.recipe_agent_preset_contract.v1"
    assert payload["error"]["code"] == "missing_recipe_plan_artifacts"


def test_cli_recipe_export_agent_preset_contract_collects_handoff_surface(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "recipe-agent-contract"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "export-agent-preset-contract",
            "--plan-dir",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_agent_preset_contract.v1"
    assert payload["movement_allowed"] is False
    assert payload["recipe"]["name"] == "home"
    assert payload["agent_runtime_profile"]["profile"] == "recipe_preset_to_eef_preview"
    assert payload["eef_seed"]["schema"] == "armctrl.recipe_eef_seed.v1"
    assert payload["safety_summary"]["allowed"] is True
    assert payload["safety_summary"]["simulation_check"]["status"] == "pass"
    assert payload["required_artifacts"]["eef_seed"].endswith("eef_seed.json")
    assert payload["ordered_steps"][0]["id"] == "export_eef_seed"
    assert payload["ordered_steps"][1]["depends_on"] == ["export_eef_seed"]
    assert any("recipe export-eef-seed" in step for step in payload["next_steps"])
    assert any("--recipe-plan-dir" in step for step in payload["next_steps"])


def test_cli_recipe_execute_accepts_sim_backend_after_safety_preview(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "recipe-exec"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "execute",
            "home",
            "--backend",
            "sim",
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_execution.v1"
    assert payload["executor"]["status"] == "ready"
    assert payload["executor"]["hardware_backend"] == "sim"
    assert payload["simulation_gate"]["allowed"] is True
    assert payload["artifacts"]["manifest"] == str(output_dir / "manifest.json")
    assert payload["handoff"]["recipe_plan_dir"] == str(output_dir)
    assert payload["agent_runtime_profile"]["schema"] == "armctrl.agent_runtime_profile.v1"
    assert payload["agent_runtime_profile"]["profile"] == "recipe_preset_to_eef_preview"
    assert payload["agent_runtime_profile"]["recipe_plan_dir"] == str(output_dir)
    assert payload["handoff"]["eef_seed"]["schema"] == "armctrl.recipe_eef_seed.v1"
    assert payload["handoff"]["eef_seed"]["final_joints"] == [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    assert any("recipe export-eef-seed" in step for step in payload["next_steps"])
    assert any("--recipe-plan-dir" in step for step in payload["next_steps"])


def test_cli_recipe_status_reports_no_hardware_session() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "recipe", "status", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_executor_status.v1"
    assert payload["executor"]["state"] == "idle"
    assert payload["executor"]["hardware_backend"] == "not_configured"


def test_cli_recipe_cancel_is_safe_when_no_hardware_session() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "recipe", "cancel", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_executor_cancel.v1"
    assert payload["executor"]["state"] == "idle"
    assert payload["executor"]["action"] == "no_active_session"
