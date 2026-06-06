from pathlib import Path


def test_codex_motion_skill_covers_recipe_and_eef_cli() -> None:
    skill_path = Path(".codex/skills/armctrl-agent-motion/SKILL.md")

    text = skill_path.read_text(encoding="utf-8")

    assert "uv run armctrl agent-flow plan \\" in text
    assert "uv run python scripts/agent_cli_sim_experiment.py \\" in text
    assert "acceptance_status: pass" in text
    assert "--eef-mode pose_delta" in text
    assert "--backend lerobot_rollout" in text
    assert "uv run armctrl recipe list --json" in text
    assert "uv run armctrl recipe plan <name> --json" in text
    assert "uv run armctrl recipe export-agent-preset-contract --plan-dir <dir> --json" in text
    assert "uv run armctrl eef doctor --json" in text
    assert "uv run armctrl eef plan-pose" in text
    assert "uv run armctrl eef plan-twist" in text
    assert "uv run armctrl eef plan-delta-pose" in text
    assert "agent_action" in text
    assert "eef.pose_absolute" in text
    assert "eef.pose_delta" in text
    assert "eef.twist" in text
    assert "uv run armctrl eef runtime-plan --plan-dir <dir> --model X5 --interface can0 --json" in text
    assert "uv run armctrl eef export-lerobot-action --plan-dir <dir> --json" in text
    assert "uv run armctrl eef export-sdk-cartesian" in text
    assert "uv run armctrl eef export-moveit-servo" in text
    assert "uv run armctrl eef export-runtime-bridge --plan-dir <dir> --json" in text
    assert "uv run armctrl eef export-runner-contract --plan-dir <dir> --json" in text
    assert "uv run armctrl eef export-agent-runtime-contract \\" in text
    assert "uv run armctrl eef preview-runner \\" in text
    assert "uv run armctrl eef sample-runner \\" in text
    assert "uv run armctrl eef export-sdk-helper-plan \\" in text
    assert "uv run armctrl eef export-moveit-helper-plan \\" in text
    assert "uv run armctrl lerobot agent-runtime-helper-plan \\" in text
    assert "uv run armctrl lerobot processor-helper-preview \\" in text
    assert "uv run python scripts/sdk_cartesian_contract_helper_sample.py \\" in text
    assert "uv run python scripts/lerobot_agent_runtime_helper_sample.py \\" in text
    assert "uv run python scripts/lerobot_processor_contract_helper_sample.py \\" in text
    assert "uv run armctrl eef synthesize-preview --plan-dir <dir> --json" in text
    assert "uv run armctrl eef stage-trajectory --plan-dir <dir> --trajectory <joint_csv> --json" in text
    assert "--eef-plan-dir <dir>" in text
    assert "uv run armctrl lerobot export-processor-contract --eef-plan-dir <dir> --json" in text
    assert "uv run armctrl lerobot preview-rollout \\" in text
    assert "uv run armctrl lerobot stage-rollout-trajectory" in text
    assert "uv run armctrl lerobot review-rollout" in text
    assert "uv run armctrl lerobot config-plan rollout" in text
    assert "uv run armctrl eef review --plan-dir <dir> --json" in text
    assert "Do not call raw SDK motion methods." in text
    assert "Do not call arx5-interface directly." in text
    assert "Do not bypass `armctrl recipe`, `armctrl eef`, or the shared simulation review chain." in text


def test_codex_motion_skill_defines_agent_action_protocol() -> None:
    skill_path = Path(".codex/skills/armctrl-agent-motion/SKILL.md")

    text = skill_path.read_text(encoding="utf-8")

    assert "## Agent Action Protocol" in text
    assert "Agents must choose exactly one intent class" in text
    assert "`preset.apply`" in text
    assert "`eef.pose_delta`" in text
    assert "`eef.pose_absolute`" in text
    assert "`eef.twist`" in text
    assert "`rollout.prepare`" in text
    assert "`preset.apply` -> `uv run armctrl agent-flow plan`" in text
    assert "`eef.pose_delta` -> `uv run armctrl eef plan-delta-pose`" in text
    assert "`rollout.prepare` -> `uv run armctrl lerobot export-processor-contract`" in text
    assert "Read `recommended_path` first" in text
    assert "Minimum Agent Report" in text
    assert "Agent EEF is the upstream action semantic" in text
    assert "LeRobot processors are the downstream adapter semantic" in text
    assert "robot_action_processor" in text
    assert "robot_observation_processor" in text
    assert "Classify the request into one Agent Action Protocol intent" in text
    assert "Treat that session plan as the top-level Agent artifact" in text
    assert "Report the Minimum Agent Report fields" in text
    assert "Do not convert Agent intent into native LeRobot command fields yourself" in text
