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

Version `0.6.0-rc.25` means safety-space config, simulation backend discovery,
SysID trajectory preview artifacts, Agent recipes, EEF runtime-plan/review
contracts, backend trajectory staging, Agent CLI simulation experiment,
SDK smoke-runner contracts, and the LeRobot planning bridge are present.
Hardware execution, native LeRobot record/rollout on hardware, and
n100d mature backend validation are still explicitly marked pending. The JSON includes local
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
  --render runs/ident-plan-preview/trajectory_preview.svg \
  --json
```

Render an interactive URDF-FK animation instead of the compact SVG report:

```bash
uv run armctrl sim preview \
  --trajectory runs/ident-plan-preview/planned_trajectory.csv \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --render runs/ident-plan-preview/trajectory_preview.html \
  --json
```

`configs/x5.safe.yaml` defines named allowed and forbidden workspace boxes,
simulation backend preference, and the distal link frames used by the fallback
FK gate. The fallback is clearly labeled; it exists only for conservative local
checks when mature backends are unavailable.

When `--backend mujoco` is selected, preview loads the model, writes each
trajectory sample into MuJoCo `qpos`, calls `mj_forward`, and reports contact
counts plus qpos ranges. This keeps MuJoCo as the simulator while `armctrl`
only performs orchestration and gate reporting.

`--render <path.svg>` writes a lightweight SVG preview of joint traces and
end-effector side-view clearance. Unsafe trajectories are still rendered, but
the SVG is marked with `WARNING` and the first gate reasons so the operator or
Agent can inspect why execution was blocked.

`--render <path.html>` writes a browser-openable 3D URDF kinematic animation
using a mature Three.js-style viewer surface. The safety decision still comes
from the configured simulation gate; the HTML is an operator/Agent preview
artifact, not a replacement for MoveIt, MuJoCo, or Pinocchio/coal checks.
Dangerous trajectories are rendered too, with `WARNING` and the first gate
reasons shown in the page header.

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

When `--output` is provided, `recipe plan` writes a joint-space preview bundle
through the same simulation safety chain used by SysID planning. The current
clean rebuild also allows `recipe execute --backend sim` as a non-hardware gate:
it writes preview artifacts, evaluates limits/workspace/simulation checks, and
returns `ok` only when the simulated gate passes. The execution response now
also carries a structured `handoff` plus `next_steps`, so Agents can treat the
simulated preset action as a machine-readable bridge into later EEF work rather
than just a binary preview result. Both `recipe plan --output` and
`recipe execute --backend sim` now also emit an `agent_runtime_profile`
declaring that this bundle is meant to seed later bounded EEF preview work.

If an Agent wants to treat a reviewed recipe posture as the start state for the
next bounded EEF motion, export an EEF seed from the recipe plan bundle:

```bash
uv run armctrl recipe export-eef-seed \
  --plan-dir runs/recipe-plan \
  --json
```

This returns the final joint sample from `planned_trajectory.csv` together with
a suggested `eef synthesize-preview --start-joints ...` command.
The `recipe plan --output` JSON response now also includes `next_steps` for this
handoff plus the local `recipe execute --backend sim` preview path.
The same plan bundle now writes `eef_seed.json`, so Agents can discover the
handoff artifact directly from `artifacts.eef_seed`.
If an Agent wants one single preset-action handoff artifact instead of reading
`manifest.json`, `eef_seed.json`, and the preview response separately, export
the Agent preset contract:

```bash
uv run armctrl recipe export-agent-preset-contract \
  --plan-dir runs/recipe-plan \
  --json
```

This stays non-hardware. It consolidates the reviewed preset posture, safety
summary, EEF seed, required artifacts, and suggested next steps into one JSON
surface for later bounded EEF planning.
The returned contract now also includes `ordered_steps`, which makes the
sequence dependency explicit for Agent callers instead of leaving it as an
implicit reading of `next_steps`.
If you want that handoff to stay fully artifact-driven, `eef synthesize-preview`
can now read the start state directly from the recipe plan bundle:

```bash
uv run armctrl eef synthesize-preview \
  --plan-dir runs/eef-plan \
  --recipe-plan-dir runs/recipe-plan \
  --json
```

## EEF Planning

Realtime end-effector control is intentionally not executed directly inside
`armctrl`. Instead, `armctrl` exposes a thin plan-only contract so Agents can
request bounded EEF intent while mature backends remain the execution owners.

Agent motion starts from a small action protocol, not backend-native commands:

- `preset.apply`: named recipe posture through `agent-flow plan` or `recipe plan`.
- `eef.pose_delta`: preferred Agent and LeRobot-aligned EEF action.
- `eef.pose_absolute`: bounded absolute EEF target.
- `eef.twist`: bounded EEF velocity/twist request.
- `rollout.prepare`: LeRobot rollout or processor handoff after an EEF plan exists.

The rule of thumb is: Agent EEF is the upstream action semantic, while LeRobot
`robot_action_processor` / `robot_observation_processor` are the downstream
adapter semantic. Agent callers should read `recommended_path` first, then
`ordered_steps`, then `next_steps`, and stop on failed gates instead of
inventing backend-specific recovery commands.

To replay a fixed Agent-style CLI experiment without hardware, run:

```bash
uv run python scripts/agent_cli_sim_experiment.py \
  --output runs/agent-cli-sim-acceptance \
  --json
```

The script calls only public CLI commands in order: `agent-flow doctor`,
an off-center EEF absolute preview, `recipe plan home`, a visibly large EEF
absolute preview, `export-agent-session-plan`, EEF preview/review, LeRobot
processor preview, and `agent-flow review`. The acceptance result is written to
`agent_cli_sim_experiment.json` and must report `acceptance_status: pass`
before treating the Agent motion contract as healthy. The generated
`offcenter_review.html`, `trajectory_preview.html`, and `review_preview.html`
artifacts are intended for visual inspection of the off-center, recenter, and
large EEF motions.

If an Agent wants one single non-hardware entry point that composes a reviewed
preset posture, bounded EEF intent, runtime helper handoff, and shared review,
use:

```bash
uv run armctrl agent-flow plan \
  --preset home \
  --eef-mode pose_delta \
  --backend lerobot_rollout \
  --delta-position 0.002 0.000 -0.003 \
  --delta-rpy 0 0 0.02 \
  --output runs/agent-flow-smoke \
  --json
```

This is a thin orchestration surface over existing `recipe`, `eef`,
`lerobot`, and shared review contracts. It does not execute hardware and does
not introduce a new motion backend.

The default Agent-facing motion vocabulary is:

- `eef.pose_absolute`
- `eef.pose_delta`
- `eef.twist`

For LeRobot-aligned flows, prefer `eef.pose_delta` as the default action
representation. `armctrl` keeps the upstream action in EEF space, then exports
machine-readable contracts so LeRobot-side `robot_action_processor` and
`robot_observation_processor` can adapt those actions into rollout-native
commands without changing the shared safety/review path.

If an Agent needs one single non-hardware session contract for realtime EEF
loops, export:

```bash
uv run armctrl eef export-agent-session-plan \
  --plan-dir runs/eef-plan \
  --json
```

This top-level session plan keeps the Agent action in EEF space, identifies the
mature runtime owner, attaches the backend helper plan when needed, and keeps
the reviewed joint-trajectory return path machine-readable.

Check the top-level non-hardware entry surface before planning:

```bash
uv run armctrl agent-flow doctor --json
```

Replay the saved top-level contract after planning:

```bash
uv run armctrl agent-flow review \
  --contract runs/agent-flow-smoke/agent_flow_plan.json \
  --json
```

Read-only backend availability:

```bash
uv run armctrl eef doctor --json

uv run armctrl eef runtime-plan \
  --backend sdk_cartesian \
  --model X5 \
  --interface can0 \
  --json
```

Plan a Cartesian twist command without executing hardware:

```bash
uv run armctrl eef plan-twist \
  --frame eef_link \
  --linear 0 0 -0.02 \
  --angular 0 0 0 \
  --backend moveit_servo \
  --json
```

Plan an absolute EEF pose intent for a lightweight differential-IK backend:

```bash
uv run armctrl eef plan-pose \
  --frame eef_link \
  --position 0.40 0.00 0.20 \
  --rpy 0 0 0 \
  --backend pink \
  --json
```

These commands are plan-only. They return `movement_allowed: false`, backend
ownership notes, and safety-space constraints that must be honored before any
future hardware execution path is enabled.
They also now return an Agent-facing `agent_action` surface:

- pose plans use `action_id: eef.pose_absolute`
- twist plans use `action_id: eef.twist`

That vocabulary is the stable upstream intent for Agent callers. Backend
surfaces such as `PoseStamped`, `TwistStamped`, or `EEFState.pose_6d` remain
under `backend_handoff` and bridge/export artifacts.

If an Agent wants a direct delta-pose intent instead of velocities, use:

```bash
uv run armctrl eef plan-delta-pose \
  --frame eef_link \
  --delta-position 0.002 0.000 -0.003 \
  --delta-rpy 0 0 0.02 \
  --backend sdk_cartesian \
  --json
```

This returns `agent_action.action_id: eef.pose_delta`. On the backend side,
`sdk_cartesian` maps that to `EEFState.pose_6d_delta`, while `moveit_servo`
maps the same delta intent to a reviewed `TwistStamped` handoff so the Agent
vocabulary stays stable above mature runtime owners.

If you want the optional local EEF preview helper to use Pink on a Linux target,
install it with:

```bash
uv sync --extra dev --extra eef
```

`eef plan-pose` now performs a direct workspace-box gate on the requested end
effector target and can write `eef_plan.json` plus `manifest.json` when
`--output <dir>` is provided. `eef plan-twist` keeps execution deferred but
uses the configured translation/rotation step limits together with a control
period to reject overly aggressive single-step commands before any backend is
invoked. Both commands also write a machine-readable `backend_request.json`
handoff artifact so a mature backend such as MoveIt Servo or Pink can own the
actual IK/servo synthesis while `armctrl` stays in the orchestration layer.
The plan payloads now also include a LeRobot-aligned cartesian action mapping,
so Agent-side EEF commands and future `lerobot-rollout` policy actions can
converge on one motion vocabulary instead of inventing separate formats.

When the robot workstation already uses the native ARX5 SDK cartesian
controller, keep that backend boundary explicit:

```bash
uv run armctrl eef plan-pose \
  --frame eef_link \
  --position 0.40 0.00 0.20 \
  --rpy 0 0 0 \
  --backend sdk_cartesian \
  --json
```

This still does not execute hardware. It emits an `EEFState.pose_6d`-style
handoff for the mature `arx5_interface` cartesian controller while preserving
the shared safety and simulation review flow.

`eef runtime-plan` is the operator/Agent-facing entry point for mature backend
ownership. It does not launch hardware itself, but it tells the caller which
runtime surface owns execution next:

- `sdk_cartesian`: native `arx5_interface` cartesian controller contract
- `moveit_servo`: ROS 2 MoveIt Servo runtime contract
- `lerobot_rollout`: native `lerobot-rollout` policy runtime contract

All three point back to the same `eef plan` artifacts and the same
`backend_joint_trajectory.csv` review path before any future execution gate.
When `--plan-dir <dir>` is provided, `eef runtime-plan` can infer the backend
from `backend_request.json` and carry the matching review contract forward, so
Agents do not need to restate the backend manually after `eef plan --output`.
For `sdk_cartesian` and `moveit_servo` plans, the same response now also
includes a backend-specific bridge preview plus a short `next_steps` list for
staging and review, so Agents can treat `runtime-plan` as a compact handoff
checklist instead of reconstructing one from multiple commands. The same
response now also includes an `agent_runtime_profile` that declares this bundle
as an `eef_runtime_handoff` toward a mature runtime owner.
If the caller explicitly wants a different mature owner for the same plan
artifact, `runtime-plan` also accepts an explicit `--backend` override together
with `--plan-dir`, so the same bounded EEF intent can be handed to a rollout
surface such as `lerobot_rollout` without regenerating the plan bundle.

If the next owner is a LeRobot policy or an Agent surface that already speaks
LeRobot-style cartesian actions, export the same EEF plan as a LeRobot-friendly
action contract instead of inventing another format:

```bash
uv run armctrl eef export-lerobot-action \
  --plan-dir runs/eef-plan \
  --json
```

This returns an ordered `feature_order` plus `action_vector`/`action_dict`
pairing for either `cartesian_pose_absolute` or `cartesian_delta`, while still
keeping `movement_allowed: false` and the shared review requirement intact.

If the next owner is a programmatic ARX5 SDK cartesian integration, export the
same plan as an SDK-structured bridge artifact:

```bash
uv run armctrl eef export-sdk-cartesian \
  --plan-dir runs/eef-plan \
  --model X5 \
  --interface can0 \
  --json
```

This writes a non-hardware request surface around `Arx5CartesianController` and
`EEFState`, preserving `armctrl` as the orchestration layer while making the
SDK-side request explicit and machine-readable.

If the next owner is a ROS 2 MoveIt Servo session, export the same EEF plan as
an explicit ROS-message bridge artifact:

```bash
uv run armctrl eef export-moveit-servo \
  --plan-dir runs/eef-plan \
  --json
```

For twist plans this writes a `geometry_msgs/msg/TwistStamped`-style contract;
for pose plans it writes a `geometry_msgs/msg/PoseStamped`-style contract. In
both cases it carries the frame id plus ROS environment hint while still
leaving realtime servo execution outside `armctrl`.

When a mature backend has already synthesized a joint-space result, stage it
into the conventional review location instead of inventing another file layout:

```bash
uv run armctrl eef stage-trajectory \
  --plan-dir runs/eef-plan \
  --trajectory path/to/backend_joint_trajectory.csv \
  --json
```

`stage-trajectory` validates the required joint CSV columns from
`backend_review_contract.json`, copies the file into
`<dir>/backend_joint_trajectory.csv`, and then points the caller back to the
shared `eef review` gate.

The same plan directory can be carried into a native LeRobot rollout plan so the
policy/runtime contract is explicit about which cartesian action schema it is
expected to follow:

```bash
uv run armctrl lerobot config-plan rollout \
  --model X5 \
  --robot-interface can0 \
  --policy-path outputs/train/act_arx5/checkpoints/last/pretrained_model \
  --eef-plan-dir runs/eef-plan \
  --json
```

This still does not execute hardware. It simply attaches the exported EEF
LeRobot action bridge to the rollout plan so Agent-side policy integration stays
aligned with the `eef` motion vocabulary.

The rollout plan also carries the same backend review contract and points to the
same conventional joint-trajectory review artifact, so an Agent does not need
to invent a second rollout-specific file layout.
It now also includes the native rollout runtime owner plus rollout-specific
`next_steps`, so the same JSON response can act as a compact operator/Agent
handoff checklist.
For programmatic integrations, the rollout payload also includes a
`processor_bridge` contract that points the caller at LeRobot's
`robot_action_processor` / `robot_observation_processor` adaptation surface
while preserving the exported EEF LeRobot action vocabulary.

If a rollout helper wants that handoff as a single explicit artifact instead of
reconstructing it from the rollout plan and the runner contract, export the
processor contract directly:

```bash
uv run armctrl lerobot export-processor-contract \
  --eef-plan-dir runs/eef-plan \
  --json
```

This emits the EEF-shaped LeRobot action contract, the recommended processor
hook names, and the reviewed joint-trajectory return path that closes the loop
back into `armctrl`.
If the plan directory does not yet contain `eef plan --output` artifacts, the
command now fails as structured JSON with `missing_eef_plan_artifacts` instead
of a Python traceback, so Agent callers can recover cleanly.

If an Agent only wants "the right bridge export for this plan" without choosing
between the backend-specific export commands up front, use the unified export
surface:

```bash
uv run armctrl eef export-runtime-bridge \
  --plan-dir runs/eef-plan \
  --json
```

This is a thin dispatcher over the existing SDK, MoveIt Servo, and LeRobot
bridge exporters, not a new execution backend.

If a backend helper needs an explicit return-path contract into the shared
review chain, export the runner contract too:

```bash
uv run armctrl eef export-runner-contract \
  --plan-dir runs/eef-plan \
  --json
```

This contract tells the helper which mature-owner bridge artifact to consume,
which joint CSV to emit, which columns are required, and which `stage` /
`review` commands close the loop back into `armctrl`. It now also returns
recommended `next_steps`, so an Agent can treat it as an execution checklist
instead of reconstructing preflight and closure commands by hand. It also emits
the same `agent_runtime_profile` shape used by `runtime-plan`, so callers can
recognize the handoff role without branching on which command produced it.

If an Agent or helper loop wants one stable contract for streaming a sequence of
EEF action frames into a mature runtime owner, export the Agent runtime
contract:

```bash
uv run armctrl eef export-agent-runtime-contract \
  --plan-dir runs/eef-plan \
  --json
```

This stays non-hardware. It keeps the Agent-facing action vocabulary
(`eef.pose_absolute`, `eef.pose_delta`, `eef.twist`) separate from the backend
session surface, then names the mature runtime session contract and the shared
review return path in one artifact.
The returned contract now also includes `ordered_steps`, so Agent callers can
see that staging must happen before review instead of inferring that from free
text alone.

If that Agent/runtime handoff should specifically land on a LeRobot rollout-side
processor helper, prefer the CLI-native helper plan:

```bash
uv run armctrl lerobot agent-runtime-helper-plan \
  --agent-runtime-contract runs/eef-plan/eef_agent_runtime_contract.json \
  --json
```

This sample stays non-hardware too. It turns the Agent runtime contract into a
processor-oriented session plan for `robot_action_processor` /
`robot_observation_processor`, while preserving the shared `review-rollout`
return path.
The helper payload also includes `ordered_steps`, so export, preview, and
review sequencing stays machine-readable for Agent callers.

If you want the lower-level repo-local script version instead, use:

```bash
uv run python scripts/lerobot_agent_runtime_helper_sample.py \
  --agent-runtime-contract runs/eef-plan/eef_agent_runtime_contract.json \
  --json
```

This script stays non-hardware too and mirrors the same helper-plan boundary.

If you want the closest repo-local sample of a LeRobot-side helper that reads a
serialized processor contract and immediately closes the loop through the shared
preview/review chain, prefer the CLI-native helper preview:

```bash
uv run armctrl lerobot processor-helper-preview \
  --processor-contract runs/eef-plan/lerobot_processor_contract.json \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --json
```

This stays non-hardware as well. It consumes the exported LeRobot processor
contract, synthesizes a conservative reviewed trajectory through
`preview-rollout`, and reports the shared simulation gate result.
Its helper payload also includes `ordered_steps`, so preview and review remain
explicitly ordered for downstream Agent orchestration.

If you want the lower-level repo-local script version instead, use:

```bash
uv run python scripts/lerobot_processor_contract_helper_sample.py \
  --processor-contract runs/eef-plan/lerobot_processor_contract.json \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --json
```

This script stays non-hardware as well and mirrors the same helper-preview
boundary.

If an Agent wants a generic non-hardware closure step that stays explicitly
bound to that mature-backend runner contract, use:

```bash
uv run armctrl eef preview-runner \
  --plan-dir runs/eef-plan \
  --json
```

This helper is still non-hardware. It exports the runner contract, synthesizes
`backend_joint_trajectory.csv`, and immediately reuses the shared `eef review`
simulation gate so the contract and the preview stay in one loop.

If you want the closest repo-local sample of an external helper actually
consuming the serialized runner contract artifact, use:

```bash
uv run armctrl eef sample-runner \
  --runner-contract runs/eef-plan/eef_runner_contract.json \
  --json
```

Export the runner contract with `--output` first, then point `sample-runner` at
that JSON file. This keeps the integration boundary honest: the helper reads
only the contract artifact, derives the conventional review path from it, and
closes back into the shared simulation gate without inventing a new layout.

If you want a thinner helper aimed specifically at the ARX5 SDK cartesian
runtime, prefer the CLI-native helper plan:

```bash
uv run armctrl eef export-sdk-helper-plan \
  --runner-contract runs/eef-plan/eef_runner_contract.json \
  --json
```

This stays non-hardware and converts the serialized runner contract into an
SDK-shaped session plan while preserving the shared review return path.

If you want the lower-level repo-local sample instead, use:

```bash
uv run python scripts/sdk_cartesian_contract_helper_sample.py \
  --runner-contract runs/eef-plan/eef_runner_contract.json \
  --json
```

This script stays outside `armctrl` runtime ownership. It simply converts the
serialized runner contract into an SDK-shaped session plan so a future mature
helper can follow the same contract without guessing field names or artifact
paths.

If you want the same style of helper for a ROS 2 MoveIt Servo runtime, prefer
the CLI-native helper plan:

```bash
uv run armctrl eef export-moveit-helper-plan \
  --runner-contract runs/eef-plan/eef_runner_contract.json \
  --json
```

This stays non-hardware and converts the serialized runner contract into a
MoveIt Servo session plan while preserving the shared review return path.

If you want the lower-level repo-local sample instead, use:

```bash
uv run python scripts/moveit_servo_contract_helper_sample.py \
  --runner-contract runs/eef-plan/eef_runner_contract.json \
  --json
```

This script also stays outside `armctrl` runtime ownership. It converts the
serialized runner contract into a MoveIt Servo session plan artifact that names
the expected topic, service, and message surfaces without starting ROS or
publishing commands.

If an Agent wants a local non-hardware closure step before involving a mature
runtime owner, `armctrl` can also synthesize a conservative preview trajectory
and immediately feed it into the same review chain:

```bash
uv run armctrl eef synthesize-preview \
  --plan-dir runs/eef-plan \
  --json
```

This helper is still non-hardware. It only writes
`backend_joint_trajectory.csv`, records whether the synthesis path was degraded,
and then runs the existing `eef review` simulation gate on the result.
When a bounded EEF motion should start from a previously reviewed recipe posture,
pass `--recipe-plan-dir <recipe_dir>` so the preview starts from that recipe's
`eef_seed.json` instead of the default safe-center joints.
When `pink` is not importable in the current environment, the synthesis result
stays explicit about the downgrade through `synthesis.degraded: true`.

If a mature rollout-side backend or bridge later produces a reviewed
joint-space trajectory, keep the LeRobot-facing safety gate on the same shared
simulation chain instead of inventing a separate one:

```bash
uv run armctrl lerobot review-rollout \
  --eef-plan-dir runs/eef-plan \
  --trajectory runs/eef-plan/backend_joint_trajectory.csv \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --json
```

`lerobot review-rollout` is just a LeRobot-facing wrapper over the existing
`eef review` path, so the same limits, workspace gate, and simulation preview
remain authoritative.

If you want to keep that handoff explicit on the LeRobot side, stage the
rollout-produced joint CSV through the rollout wrapper first:

```bash
uv run armctrl lerobot stage-rollout-trajectory \
  --eef-plan-dir runs/eef-plan \
  --trajectory path/to/rollout_joint_trajectory.csv \
  --json
```

This reuses the same underlying EEF staging contract and still lands the file
at `<dir>/backend_joint_trajectory.csv` for shared review.

If an Agent needs a LeRobot-facing non-hardware closure step before a native
rollout helper exists, `armctrl` can also combine the exported EEF action
contract, the processor handoff, and the existing preview synthesizer into one
shared safety-gated dry run:

```bash
uv run armctrl lerobot preview-rollout \
  --eef-plan-dir runs/eef-plan \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --json
```

This helper is still non-hardware. It does not execute LeRobot rollout. It
only exports the processor contract, synthesizes a conservative joint preview
into `backend_joint_trajectory.csv`, and reuses the same simulation review
chain as `eef review` / `lerobot review-rollout`.

After a mature backend turns that request into a joint-space trajectory,
`armctrl` can review the resulting motion through the same simulation preview
chain used by recipes and SysID:

```bash
uv run armctrl eef plan-pose \
  --frame eef_link \
  --position 0.40 0.00 0.20 \
  --rpy 0 0 0 \
  --backend pink \
  --output runs/eef-plan \
  --json

uv run armctrl eef review \
  --plan-dir runs/eef-plan \
  --trajectory runs/eef-plan/backend_joint_trajectory.csv \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --render runs/eef-plan/trajectory_preview.html \
  --json
```

`eef review` remains non-hardware. It simply checks that the backend-generated
joint trajectory passes the shared limits, workspace, and simulation gate
before any future execution surface is allowed to use it.
When `eef plan --output <dir>` is used, `armctrl` also writes
`backend_review_contract.json` and reserves the conventional backend output path
`<dir>/backend_joint_trajectory.csv`. If a mature backend writes or
`stage-trajectory` installs the reviewed joint trajectory there,
`armctrl eef review --plan-dir <dir>` can auto-discover it without an explicit
`--trajectory` flag.

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
  --render \
  --json
```

This writes `planned_trajectory.csv`, `trajectory_preview.json`,
`trajectory_preview.svg` when `--render` is present, and `manifest.json`.
URDF joint-limit checks are evaluated from `--urdf-path`;
workspace clearance and named allowed/forbidden spaces are evaluated through the
simulation preview gate from `--safe-config`. When mature backends are
importable, the preview selects them by config preference; otherwise it records
`urdf_fk_fallback` and keeps the decision machine-readable.

Use a mature OED backend instead of armctrl's conservative smoke fallback:

```bash
uv run armctrl sysid plan fourier_multisine \
  --dof 6 \
  --sample-hz 100 \
  --duration 40 \
  --q-center 0 0.3 0.3 0 0 0 \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output runs/ident-figaroh-plan \
  --render trajectory_preview.html \
  --json \
  --trajectory-command uv run python scripts/x5_figaroh_oed.py
```

The external command receives `ARMCTRL_FIGAROH_REQUEST` and must write a CSV to
`ARMCTRL_CANDIDATE_TRAJECTORY`. `armctrl` then imports that CSV, records the
generator command in `manifest.json`, scores the trajectory when Pinocchio is
available, and runs the same safety preview gates before any hardware path can
consume it.

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

The project-local Codex skills are stored at:

```text
.codex/skills/armctrl-agent-recipes/SKILL.md
.codex/skills/armctrl-agent-motion/SKILL.md
```

`armctrl-agent-recipes` only allows Agents to inspect and plan bounded preset
recipes through `uv run armctrl recipe ...`.

`armctrl-agent-motion` covers the broader non-hardware motion workflow through
`uv run armctrl recipe ...` plus `uv run armctrl eef ...`, including:

- EEF planning
- Agent-facing EEF vocabulary with `eef.pose_absolute` / `eef.pose_delta` /
  `eef.twist`
- unified non-hardware Agent session planning for realtime EEF loops
- LeRobot-friendly EEF action export
- mature runtime-owner inspection
- backend joint-trajectory review through the shared simulation gate

The first clean milestone is `v0.2.0`: rebuild the governance, safety, and
recipe contracts before reintroducing hardware-moving code.
