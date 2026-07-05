from pathlib import Path


def test_codex_skill_limits_agent_to_recipe_cli() -> None:
    skill_path = Path(".codex/skills/armctrl-agent-recipes/SKILL.md")

    text = skill_path.read_text(encoding="utf-8")

    assert "uv run armctrl recipe list --json" in text
    assert "uv run armctrl recipe plan <name> --json" in text
    assert "uv run armctrl recipe status --json" in text
    assert "uv run armctrl recipe cancel --json" in text
    assert "uv run armctrl recipe execute <name> --backend sim --output <dir> --json" in text
    assert "Do not call raw SDK" in text
    assert "Do not call arx5-interface directly" in text
    assert "recipe execute" in text
