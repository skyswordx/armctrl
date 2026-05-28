import json
import subprocess
import sys

from armctrl.recipes import RecipeCatalog
from armctrl.safety import SafetyGate


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
        ["armctrl", "recipe", "list", "--json"],
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
