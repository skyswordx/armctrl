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
5. Only consider `recipe execute` when the user explicitly asks for execution
   and the command returns an allowed safety decision.
