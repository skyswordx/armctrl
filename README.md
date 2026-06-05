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
- Pinocchio + coal/hpp-fcl, MuJoCo, and MoveIt as mature simulation/safety
  oracles when available;
- FIGAROH for mature identification math and excitation tooling;
- LeRobot ARX5 integrations for data collection, training, and policy rollout.

## Development

```bash
uv sync --extra dev
uv run pytest -q
```

On the Linux robot workstation, install the native LeRobot ARX5 integration
surface for non-hardware CLI planning and import checks:

```bash
uv sync --extra dev --extra lerobot
```

Install optional mature simulation preview dependencies on Linux targets:

```bash
uv sync --extra dev --extra sim
```

Check the current clean rebuild release status:

```bash
uv run armctrl release status --json
uv run armctrl release notes --json
```

Version `0.6.0-rc.3` means safety-space config, simulation backend discovery,
SysID trajectory preview artifacts, Agent recipes, SDK smoke-runner contracts,
and the LeRobot planning bridge are present. Hardware execution, native LeRobot
record/rollout on hardware, and n100d mature backend validation are still
explicitly marked pending. The JSON includes local
verification commands plus separate `deferred_validation` buckets for hardware,
external tools, and geometry upgrades. Release notes are generated from the same
status surface, so the repository does not need another release-note document.

## Simulation Safety Preview

`armctrl` is a thin safety orchestration shell. It does not reimplement MoveIt,
MuJoCo, Pinocchio/coal, or FIGAROH. It converts planned motion into inputs that
mature backends can inspect, then gates motion from their results.

Read-only backend check:

```bash
uv run armctrl sim doctor --json
```

The doctor reports importability for:

- `pinocchio_coal`: lightweight URDF geometry collision checks;
- `mujoco`: contact and dynamics preview;
- `moveit`: ROS planning-scene state validity and collision oracle;
- `figaroh`: SysID excitation optimization and identification handoff.

`configs/models/X5_camera.urdf` references project-local X5 STL assets under
`configs/models/meshes/`, plus the D435 reference mesh under
`configs/models/reference/realsense2_description/`. Pinocchio/coal preview
rewrites relative mesh paths into a temporary URDF with absolute paths before
loading native geometry, so it does not depend on SDK wheel-relative mesh lookup.

On ROS 2 MoveIt hosts, source the ROS environment before running doctor or
preview commands:

```bash
source /opt/ros/jazzy/setup.bash
uv run armctrl sim doctor --json
```

Preview an already generated trajectory without hardware:

```bash
uv run armctrl sim preview \
  --trajectory runs/ident-plan-preview/planned_trajectory.csv \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --json
```

`configs/x5.safe.yaml` defines named allowed and forbidden workspace boxes,
simulation backend preference, and the distal link frames used by the fallback
FK gate. The fallback is clearly labeled; it exists only for conservative local
checks when mature backends are unavailable.

## Recipe Preview

The clean rebuild starts with plan-only Agent recipes. These commands do not
connect to hardware:

```bash
uv run armctrl recipe list --json
uv run armctrl recipe plan home --json
uv run armctrl recipe status --json
uv run armctrl recipe execute home --json
uv run armctrl recipe cancel --json
```

`recipe execute` currently returns a structured `rejected` response until an
execution backend is rebuilt and verified. `recipe status` and `recipe cancel`
are safe with no configured hardware session and expose the command executor
boundary Agents must use. `recipe plan` is always a dry-run preview: the JSON
response includes `movement_allowed: false` and a `risk_explanation` list before
any future backend is allowed to move joints.

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

This writes `planned_trajectory.csv`, `trajectory_preview.json`, and
`manifest.json`. URDF joint-limit checks are evaluated from `--urdf-path`;
workspace clearance and named allowed/forbidden spaces are evaluated through the
simulation preview gate from `--safe-config`. When mature backends are
importable, the preview selects them by config preference; otherwise it records
`urdf_fk_fallback` and keeps the decision machine-readable.

Run the fake data path:

```bash
uv run armctrl sysid run gravity_sweep \
  --adapter fake \
  --dof 6 \
  --sample-hz 100 \
  --duration 10 \
  --amplitude 0.1 \
  --q-center 0 0.3 0.3 0 0 0 \
  --output runs/ident-fake \
  --json
```

The fake runner writes `raw_samples.csv` and a run `manifest.json`.
`--adapter sdk` remains rejected from the public CLI unless the exact operator
confirmation is present and a verified backend is configured. Internally the SDK
runner skeleton is tested with an injected backend: it enters hold/damping before
recording and always lands in damping on completion or failure.

Check the SDK environment without moving hardware:

```bash
uv run armctrl sysid sdk-preflight \
  --model X5 \
  --interface can0 \
  --json
```

This command is read-only. It checks `arx5_interface` importability and records
the requested model/interface labels, but does not open CAN or instantiate robot
objects.

Preview the required SDK collection handshake before any real runner exists:

```bash
uv run armctrl sysid sdk-handshake-plan \
  --model X5 \
  --interface can0 \
  --json
```

This is also read-only. It fixes the future hardware sequence as: preflight,
explicit operator confirmation, enter hold/damping, start recording only after a
safe state, and land Ctrl-C or faults in damping.

Postprocess a dataset:

```bash
uv run armctrl sysid postprocess \
  --dataset runs/ident-fake \
  --solve \
  --json
```

This writes `processed/processed_samples.csv`,
`processed/quality_metrics.json`, `processed/quality_report.md`, and, with
`--solve`, the fixed solver handoff artifacts.

Run the fixed solver handoff stage:

```bash
uv run armctrl sysid solve \
  --dataset runs/ident-fake \
  --json
```

This writes `processed/solver_metrics.json` and
`processed/solver_report_zh.md`. The clean rebuild currently checks processed
data, records Pinocchio/FIGAROH module availability, and performs a fake-data
residual smoke test. When Pinocchio is importable, it also builds the joint
torque regressor from the dataset URDF and reports matrix rank plus effective
condition number, then solves a least-squares parameter vector and reports
prediction RMSE. Physical-consistency and base-parameter gates are always present
in `solver_metrics.json`; `sysid solve` marks them `not_evaluated` until FIGAROH
or manual-review evidence is imported.

Gate a candidate parameter package:

```bash
uv run armctrl sysid package \
  --dataset runs/ident-fake \
  --json
```

`sysid package` refuses to write `processed/parameter_package.json` unless the
solver metrics include Pinocchio condition and prediction-error evidence plus
FIGAROH/base-parameter and physical-consistency evidence. When the gate passes,
the candidate package records package version, a SHA-256 signature over solver
evidence, a rollback target, and an A/B validation checklist that must pass
before activation on hardware.

Import external solver evidence from FIGAROH or manual review:

```bash
uv run armctrl sysid figaroh-handoff \
  --dataset runs/ident-fake \
  --output runs/ident-fake/figaroh-handoff \
  --json

uv run armctrl sysid adapt-figaroh-evidence \
  --input figaroh-report.json \
  --output figaroh-evidence.json \
  --json

uv run armctrl sysid import-evidence \
  --dataset runs/ident-fake \
  --evidence figaroh-evidence.json \
  --json
```

`figaroh-handoff` writes `figaroh_handoff.json` with processed data paths, URDF
metadata, required FIGAROH outputs, and the import command to run afterward. The
evidence file must use schema `armctrl.external_solver_evidence.v1` and include
`physical_consistency` plus `figaroh_base_parameters`. This keeps FIGAROH-owned
checks outside `armctrl` while still making package gates machine-readable.

## LeRobot Planning Bridge

LeRobot owns ARX5 record, train, and rollout through its native CLI plus
`lerobot-robot-arx5` / `lerobot-teleoperator-arx5`. `armctrl` only provides
read-only checks, command plans, and dataset metadata bridges:

```bash
uv run armctrl lerobot doctor \
  --model X5 \
  --robot-interface can0 \
  --teleop-interface can1 \
  --json

uv run armctrl lerobot config-plan record \
  --model X5 \
  --robot-interface can0 \
  --teleop-interface can1 \
  --dataset-repo-id circlemoon/arx5-test \
  --task "pick cube" \
  --episodes 10 \
  --json

uv run armctrl lerobot config-plan train \
  --dataset-repo-id circlemoon/arx5-test \
  --policy act \
  --output-dir outputs/train/act_arx5_test \
  --job-name act_arx5_test \
  --json

uv run armctrl lerobot config-plan rollout \
  --model X5 \
  --robot-interface can0 \
  --policy-path outputs/train/act_arx5_test/checkpoints/last/pretrained_model \
  --json
```

These commands do not execute native LeRobot, open CAN, connect cameras, or move
hardware. They return the external `lerobot-record`, `lerobot-train`, or
`lerobot-rollout` command to review and run outside `armctrl`.

Bridge a LeRobot dataset to an `armctrl` SysID parameter package:

```bash
uv run armctrl lerobot export-metadata \
  --dataset-repo-id circlemoon/arx5-test \
  --parameter-bundle runs/sysid/processed/parameter_package.json \
  --safe-config configs/x5.safe.yaml \
  --output runs/lerobot/arx5-test.metadata.json \
  --json
```

## Online Identification Policy

Online identification is currently a shadow-mode policy contract. It allows
only conservative residual parameters and forbids direct online updates to
mass, center of mass, or inertia:

```bash
uv run armctrl online-id policy --json
```

Record a shadow-mode online update audit:

```bash
uv run armctrl online-id audit \
  --parameter joint_2.viscous_friction \
  --value 0.12 \
  --source rolling_residual_window \
  --window-start 12.0 \
  --window-end 18.0 \
  --residual-before 0.8 \
  --residual-after 0.5 \
  --saturation-status pass \
  --rollback-target previous_parameter_bundle \
  --output runs/online-id-audit.jsonl \
  --json
```

The audit log is append-only JSONL and still requires manual promotion before
any shadow update can affect control.

## Codex Skill

The project-local Codex skill is stored at:

```text
.codex/skills/armctrl-agent-recipes/SKILL.md
```

It only allows Agents to inspect and plan bounded recipes through
`uv run armctrl recipe ...`.

The first clean milestone is `v0.2.0`: rebuild the governance, safety, and
recipe contracts before reintroducing hardware-moving code.
