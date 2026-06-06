---
name: armctrl-agent-recipes
description: Use when an Agent needs to inspect or plan bounded ARX5/X5 arm actions through armctrl.
---

# armctrl Agent Recipes

Use this skill when an Agent needs a robot action plan from `armctrl`.

## Allowed Commands

List bounded recipes:

```bash
uv run armctrl recipe list --json
```

Preview a recipe:

```bash
uv run armctrl recipe plan <name> --json
```

Check the current execution gate:

```bash
uv run armctrl recipe execute <name> --json
```

At this stage `recipe execute` is expected to return `rejected` unless a
tested hardware backend has been added.

For a non-hardware preview that still returns structured handoff artifacts:

```bash
uv run armctrl recipe execute <name> --backend sim --output <dir> --json
```

Treat the returned `handoff` and `next_steps` fields as the default Agent
checklist into later EEF planning. Prefer the returned
`agent_runtime_profile` when you want one stable schema describing the intended
handoff role of the preset-action bundle.

If the Agent wants one single preset-action handoff artifact instead of
re-reading `manifest.json`, `eef_seed.json`, and the preview response
separately, export the Agent preset contract:

```bash
uv run armctrl recipe export-agent-preset-contract --plan-dir <dir> --json
```

This stays non-hardware and consolidates the reviewed preset posture, safety
summary, EEF seed, required artifacts, and suggested next steps for the later
EEF preview/runtime bridge.
Prefer the returned `ordered_steps` over inventing your own parallelization;
those steps make the required sequencing explicit.

Inspect executor status:

```bash
uv run armctrl recipe status --json
```

Cancel an active recipe session:

```bash
uv run armctrl recipe cancel --json
```

At this stage cancel is expected to be safe when no hardware session exists.

## Hard Boundaries

- Do not call raw SDK motion methods.
- Do not call arx5-interface directly.
- Do not create ad hoc joint commands.
- Do not bypass `armctrl recipe`.
- Treat any non-JSON output or nonzero exit code as a failed action.

## Expected Flow

1. Run `uv run armctrl recipe list --json`.
2. Choose one listed recipe by name.
3. Run `uv run armctrl recipe plan <name> --json`.
4. Report the plan and safety gate to the user.
5. Run `uv run armctrl recipe status --json` before any execution discussion.
6. If a non-hardware closure step is enough, prefer
   `uv run armctrl recipe execute <name> --backend sim --output <dir> --json`
   and consume its `handoff` / `next_steps`.
7. Only consider hardware-facing `recipe execute` when the user explicitly asks
   for execution and the command returns an allowed safety decision.
8. Use `uv run armctrl recipe cancel --json` for cleanup/status recovery only.
