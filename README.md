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
uv run armctrl recipe execute home --json
```

`recipe execute` currently returns a structured `rejected` response until an
execution backend is rebuilt and verified.

## SysID Preview

SysID is also plan-only in this branch. The planner exposes the intended
handoff instead of reimplementing Pinocchio or FIGAROH:

```bash
uv run armctrl sysid plan gravity_sweep --json
uv run armctrl sysid plan friction_sweep --json
uv run armctrl sysid plan fourier_multisine --json
```

Write a reviewable offline plan:

```bash
uv run armctrl sysid plan gravity_sweep \
  --dof 6 \
  --sample-hz 100 \
  --duration 10 \
  --amplitude 0.1 \
  --q-center 0 0.3 0.3 0 0 0 \
  --urdf-path configs/models/X5_camera.urdf \
  --output runs/ident-plan-preview \
  --json
```

This writes `planned_trajectory.csv` and `manifest.json`. URDF joint-limit
checks are evaluated from `--urdf-path`; workspace clearance is still marked
`not_evaluated` until the table/workcell model is rebuilt.

## Online Identification Policy

Online identification is currently a shadow-mode policy contract. It allows
only conservative residual parameters and forbids direct online updates to
mass, center of mass, or inertia:

```bash
uv run armctrl online-id policy --json
```

## Codex Skill

The project-local Codex skill is stored at:

```text
.codex/skills/armctrl-agent-recipes/SKILL.md
```

It only allows Agents to inspect and plan bounded recipes through
`uv run armctrl recipe ...`.

The first clean milestone is `v0.2.0`: rebuild the governance, safety, and
recipe contracts before reintroducing hardware-moving code.
