# armctrl

`armctrl` is the Roboclaw ARX5/X5 control and system-identification shell.

This clean rebuild branch intentionally starts small. It keeps only the project
entry points, model/config assets, vendor references, and package wiring needed
to rebuild the system with explicit boundaries.

## Current Entry Points

- `ROADMAP.md`: architecture boundaries, milestones, and release track.
- `CHANGELOG.md`: user-visible changes and release notes.
- `docs/README.md`: indexed references and historical material.

## Boundary

`armctrl` should provide:

- safety gates around planned robot motion;
- bounded Agent action recipes;
- SysID run orchestration and data handoff;
- parameter bundle validation and rollout checks.

`armctrl` should reuse, not replace:

- ARX5 SDK / `arx5-interface` for hardware sessions and low-level control;
- Pinocchio for deterministic dynamics regressors;
- FIGAROH for mature identification math and excitation tooling;
- LeRobot ARX5 integrations for data collection, training, and policy rollout.

## Development

```bash
uv sync --extra dev
uv run pytest -q
```

## Recipe Preview

The clean rebuild starts with plan-only Agent recipes. These commands do not
connect to hardware:

```bash
uv run armctrl recipe list --json
uv run armctrl recipe plan home --json
```

The first clean milestone is `v0.2.0`: rebuild the governance, safety, and
recipe contracts before reintroducing hardware-moving code.
