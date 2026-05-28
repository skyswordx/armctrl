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
    assert payload["recipes"][0]["name"] == "damping"


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
    assert payload["recipe"]["name"] == "home"
    assert payload["safety"]["allowed"] is True
    assert payload["steps"][0]["kind"] == "joint_target"
