---
name: armctrl-agent-motion
description: Use when an Agent needs to plan bounded ARX5/X5 motion through armctrl recipe or EEF contracts without bypassing safety review.
---

# armctrl Agent Motion

Use this skill when an Agent needs to work with bounded robot motion through
`armctrl`.

This skill covers two motion surfaces:

- `recipe`: bounded preset joint-space actions
- `eef`: bounded end-effector intent, mature backend handoff, and simulation review

## Agent Action Protocol

Agents must choose exactly one intent class before calling `armctrl`:

- `preset.apply`: move toward a named, bounded recipe posture.
- `eef.pose_delta`: request a small end-effector pose delta; this is the preferred LeRobot-aligned action semantic.
- `eef.pose_absolute`: request a bounded absolute end-effector pose.
- `eef.twist`: request a bounded end-effector velocity/twist intent.
- `rollout.prepare`: prepare a LeRobot rollout-side processor or policy handoff from an already reviewed EEF plan.

Route those intents through this table:

- `preset.apply` -> `uv run armctrl agent-flow plan` when followed by EEF work, otherwise `uv run armctrl recipe plan <name> --output <dir> --json`.
- `eef.pose_delta` -> `uv run armctrl eef plan-delta-pose`, with bounded delta arguments plus `--output <dir> --json`, then `uv run armctrl eef export-agent-session-plan --plan-dir <dir> --json`.
- `eef.pose_absolute` -> `uv run armctrl eef plan-pose`, with bounded target arguments plus `--output <dir> --json`, then `uv run armctrl eef export-agent-session-plan --plan-dir <dir> --json`.
- `eef.twist` -> `uv run armctrl eef plan-twist`, with bounded twist arguments plus `--output <dir> --json`, then `uv run armctrl eef export-agent-session-plan --plan-dir <dir> --json`.
- `rollout.prepare` -> `uv run armctrl lerobot export-processor-contract`, with `--eef-plan-dir <dir> --json`, or `uv run armctrl lerobot config-plan rollout ... --eef-plan-dir <dir> --json` when a policy path is already selected.

Protocol invariants:

- Always request `--json`.
- Read `recommended_path` first, then `ordered_steps`, then `next_steps`.
- Treat `movement_allowed: false` as normal for planning and preview contracts.
- Stop on nonzero exit, non-JSON output, missing artifacts, failed review gates, or any `rejected` / `faulted` status.
- Do not execute hardware from this skill; it is a planning, handoff, preview, and review protocol.
- Agent EEF is the upstream action semantic; LeRobot processors are the downstream adapter semantic.
- LeRobot-side `robot_action_processor` and `robot_observation_processor` own conversion between EEF-shaped Agent actions and rollout-native actions/observations.

Minimum Agent Report:

- chosen intent class and `agent_action.action_id`
- plan directory and key artifacts
- mature runtime owner or rollout processor owner
- `recommended_path` profile and ordered next steps
- safety/review status, including the first blocking reason when blocked

If the Agent wants one single non-hardware entry point that stitches reviewed
recipe posture, EEF intent, runtime helper handoff, and shared review together,
prefer:

```bash
uv run armctrl agent-flow plan \
  --preset home \
  --eef-mode pose_delta \
  --backend lerobot_rollout \
  --delta-position 0.002 0.000 -0.003 \
  --delta-rpy 0 0 0.02 \
  --output <dir> \
  --json
```

This does not add a new motion backend. It is a thin orchestration surface over
the existing `recipe`, `eef`, and shared review contracts.
Treat the returned `agent_motion_contract` as the top-level motion policy for
Agents: stay in `eef.pose_absolute`, `eef.pose_delta`, or `eef.twist`, and keep
backend-native command objects behind exported helper contracts.
For LeRobot-aligned flows, prefer `eef.pose_delta` as the default action
representation; the returned contract now marks that as the preferred training
action and names `robot_action_processor` / `robot_observation_processor` as
the adaptation owners.

Before planning, you can inspect the top-level non-hardware surface:

```bash
uv run armctrl agent-flow doctor --json
```

To replay the fixed Agent CLI simulation experiment for off-center EEF preview,
recenter, and visibly large EEF control without touching hardware, run:

```bash
uv run python scripts/agent_cli_sim_experiment.py \
  --output runs/agent-cli-sim-acceptance \
  --json
```

Treat `acceptance_status: pass` as the quick health check for the public CLI
contract. The script intentionally calls only public CLI commands in order,
writes per-step JSON artifacts, and renders HTML previews for the off-center,
recenter, and large EEF motions.

After planning, you can replay the saved contract instead of reconstructing the
review chain manually:

```bash
uv run armctrl agent-flow review --contract <agent_flow_plan_json> --json
```

## Allowed Commands

List bounded recipes:

```bash
uv run armctrl recipe list --json
```

Preview a preset recipe:

```bash
uv run armctrl recipe plan <name> --json
```

Preview a recipe through the simulation gate:

```bash
uv run armctrl recipe plan <name> --output <dir> --json
```

When `--output <dir>` is used, inspect the returned `next_steps` field before
inventing a follow-up command; it already points to the supported EEF seed and
simulation-preview handoffs.

Export the final recipe joint sample as an EEF start-state seed:

```bash
uv run armctrl recipe export-eef-seed --plan-dir <dir> --json
```

If the Agent wants one consolidated preset-action handoff artifact before
moving into EEF planning, export the Agent preset contract:

```bash
uv run armctrl recipe export-agent-preset-contract --plan-dir <dir> --json
```

This keeps the recipe safety summary, EEF seed, and required artifacts on one
machine-readable surface.
Prefer the returned `ordered_steps` whenever present; they make the intended
step ordering explicit and help avoid running dependent commands in parallel.

Inspect EEF backend availability:

```bash
uv run armctrl eef doctor --json
```

Plan a bounded end-effector pose intent:

```bash
uv run armctrl eef plan-pose \
  --frame eef_link \
  --position <x> <y> <z> \
  --rpy <roll> <pitch> <yaw> \
  --backend <pink|sdk_cartesian> \
  --output <dir> \
  --json
```

Treat the returned `agent_action` field as the stable Agent-facing vocabulary
surface. For pose plans it uses `action_id: eef.pose_absolute`.

Plan a bounded end-effector delta/twist intent:

```bash
uv run armctrl eef plan-twist \
  --frame eef_link \
  --linear <vx> <vy> <vz> \
  --angular <wx> <wy> <wz> \
  --backend <moveit_servo|sdk_cartesian> \
  --output <dir> \
  --json
```

For twist plans the returned `agent_action` uses `action_id: eef.twist`.
Backend-specific command surfaces such as `PoseStamped`, `TwistStamped`, or
`EEFState.pose_6d` belong under `backend_handoff` and should not replace the
Agent vocabulary.

Plan a bounded end-effector delta-pose intent:

```bash
uv run armctrl eef plan-delta-pose \
  --frame eef_link \
  --delta-position 0.002 0.000 -0.003 \
  --delta-rpy 0 0 0.02 \
  --backend <sdk_cartesian|moveit_servo> \
  --output <dir> \
  --json
```

This returns `agent_action.action_id: eef.pose_delta`. On the backend side,
`sdk_cartesian` maps that to `EEFState.pose_6d_delta`, while `moveit_servo`
maps it to a reviewed `TwistStamped` handoff derived from the same delta intent.

Inspect the mature runtime owner after a plan exists:

```bash
uv run armctrl eef runtime-plan --plan-dir <dir> --model X5 --interface can0 --json
```

Prefer the returned `agent_runtime_profile` when you want one stable schema for
what kind of handoff this EEF bundle represents; use `next_steps` as the
default execution checklist around that profile.

Export the EEF plan as a LeRobot-friendly cartesian action contract:

```bash
uv run armctrl eef export-lerobot-action --plan-dir <dir> --json
```

Export the same EEF plan as an ARX5 SDK cartesian bridge artifact:

```bash
uv run armctrl eef export-sdk-cartesian \
  --plan-dir <dir> \
  --model X5 \
  --interface can0 \
  --json
```

Export an EEF plan as a MoveIt Servo bridge artifact:

```bash
uv run armctrl eef export-moveit-servo \
  --plan-dir <dir> \
  --json
```

Or let `armctrl` resolve the mature runtime bridge export in one step:

```bash
uv run armctrl eef export-runtime-bridge \
  --plan-dir <dir> \
  --json
```

Short form for Agent command indexes:
`uv run armctrl eef export-runtime-bridge --plan-dir <dir> --json`

If a backend helper needs an explicit "consume this bridge, emit that reviewed
joint CSV" contract, export the runner handoff directly:

```bash
uv run armctrl eef export-runner-contract \
  --plan-dir <dir> \
  --json
```

Short form for Agent command indexes:
`uv run armctrl eef export-runner-contract --plan-dir <dir> --json`

Treat the returned `next_steps` as the default Agent checklist for preflight,
optional local preview closure, and the reviewed trajectory return path. The
returned `agent_runtime_profile` should match the runtime-plan profile shape, so
an Agent can classify the handoff without branching on command origin.

If the Agent or a helper loop wants one stable contract for streaming realtime
EEF action frames into a mature runtime owner, export the Agent runtime
contract:

```bash
uv run armctrl eef export-agent-runtime-contract \
  --plan-dir <dir> \
  --json
```

This keeps the Agent vocabulary (`eef.pose_absolute`, `eef.pose_delta`,
`eef.twist`) separate from backend-native session details while still naming the
shared `stage-trajectory` / `review` return path.
Prefer the returned `ordered_steps` whenever present; they make the intended
ordering explicit and help avoid staging/review races.

If the Agent wants one top-level non-hardware session artifact instead of
manually combining the Agent runtime contract with backend helper plans, use:

```bash
uv run armctrl eef export-agent-session-plan \
  --plan-dir <dir> \
  --json
```

This is the preferred entrypoint for Agent-driven realtime EEF loops. It keeps
the Agent action in EEF space, identifies the mature runtime owner, attaches a
backend helper plan when one exists, and preserves the shared review return
path in one machine-readable contract.
For LeRobot-aligned flows, this top-level session plan preserves
`eef.pose_delta` as the preferred training action and still leaves
`robot_action_processor` / `robot_observation_processor` as the adaptation
owners.

If the Agent/runtime helper specifically targets a LeRobot rollout-side
processor boundary, prefer the CLI-native helper plan:

```bash
uv run armctrl lerobot agent-runtime-helper-plan \
  --agent-runtime-contract <agent_runtime_contract_json> \
  --json
```

This stays non-hardware and converts the exported EEF Agent runtime contract
into a processor-oriented session plan for `robot_action_processor` /
`robot_observation_processor`, while preserving the shared rollout review path.
Prefer the returned `ordered_steps`; they make export, preview, and review
sequencing explicit for Agent callers.

If you need the lower-level repo-local sample instead, the repo also carries a
thin script that consumes the Agent runtime contract directly:

```bash
uv run python scripts/lerobot_agent_runtime_helper_sample.py \
  --agent-runtime-contract <agent_runtime_contract_json> \
  --json
```

This script stays non-hardware and mirrors the same helper-plan boundary.

If the Agent or an external helper wants the closest repo-local sample of a
LeRobot-side helper that consumes the serialized processor contract and closes
the loop through the shared preview/review chain, prefer the CLI-native helper
preview:

```bash
uv run armctrl lerobot processor-helper-preview \
  --processor-contract <lerobot_processor_contract_json> \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --json
```

This also stays non-hardware and reuses `preview-rollout` rather than starting
native LeRobot execution.
Prefer the returned `ordered_steps`; they keep preview ahead of review instead
of leaving that dependency implicit.

If you need the lower-level repo-local sample instead, use:

```bash
uv run python scripts/lerobot_processor_contract_helper_sample.py \
  --processor-contract <lerobot_processor_contract_json> \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --json
```

This script also stays non-hardware and mirrors the same helper-preview
boundary.

If the Agent wants that closure step to stay explicitly aligned with the mature
runtime handoff contract, use:

```bash
uv run armctrl eef preview-runner \
  --plan-dir <dir> \
  --json
```

This wraps the runner contract, writes a conservative reviewed joint preview,
and reuses the shared simulation gate without running hardware.

If the Agent or an external helper wants to prove it can consume a serialized
runner contract artifact directly, use:

```bash
uv run armctrl eef sample-runner \
  --runner-contract <runner_contract_json> \
  --json
```

This is the closest repo-local sample of how a mature backend helper should
read `armctrl.eef_runner_contract.v1` and close the loop back into reviewed
joint-space artifacts.

If the next helper specifically targets the ARX5 SDK cartesian runtime, the
preferred CLI-native helper plan is:

```bash
uv run armctrl eef export-sdk-helper-plan \
  --runner-contract <runner_contract_json> \
  --json
```

This stays non-hardware and converts the serialized runner contract into an
SDK-shaped session plan while preserving the shared review return path.

If you need the lower-level repo-local sample instead, use:

```bash
uv run python scripts/sdk_cartesian_contract_helper_sample.py \
  --runner-contract <runner_contract_json> \
  --json
```

This script does not execute hardware. It only turns the serialized runner
contract into an SDK-shaped session plan artifact that a future mature helper
can follow.

If the next helper instead targets a ROS 2 MoveIt Servo runtime, use:

```bash
uv run armctrl eef export-moveit-helper-plan \
  --runner-contract <runner_contract_json> \
  --json
```

This stays non-hardware and converts the serialized runner contract into a
MoveIt Servo session plan while preserving the shared review return path.

If you need the lower-level repo-local sample instead, use:

```bash
uv run python scripts/moveit_servo_contract_helper_sample.py \
  --runner-contract <runner_contract_json> \
  --json
```

This script also stays non-hardware. It only converts the serialized runner
contract into a MoveIt Servo session plan artifact with the expected topic,
service, and message surfaces for a future mature helper.

If the Agent wants one more non-hardware closure step before any mature runtime
owner is involved, synthesize a conservative preview trajectory and send it
through the same review gate:

```bash
uv run armctrl eef synthesize-preview \
  --plan-dir <dir> \
  --recipe-plan-dir <recipe_dir> \
  --json
```

Short form for Agent command indexes:
`uv run armctrl eef synthesize-preview --plan-dir <dir> --json`

Stage a mature-backend joint trajectory into the shared review location:

```bash
uv run armctrl eef stage-trajectory \
  --plan-dir <dir> \
  --trajectory <joint_csv> \
  --json
```

Carry that EEF action contract into a native LeRobot rollout plan:

```bash
uv run armctrl lerobot config-plan rollout \
  --model X5 \
  --robot-interface can0 \
  --policy-path <policy_dir> \
  --eef-plan-dir <dir> \
  --json
```

The rollout plan response includes the native runtime owner, the shared review
artifact contract, rollout-specific next steps for staging and review, and a
processor-bridge hint for programmatic `robot_action_processor` /
`robot_observation_processor` integrations.

If an Agent or runtime helper wants that LeRobot processor handoff as one
explicit artifact, export it directly:

```bash
uv run armctrl lerobot export-processor-contract \
  --eef-plan-dir <dir> \
  --json
```

If `<dir>` is not a real `eef plan --output` bundle, treat a
`missing_eef_plan_artifacts` JSON rejection as a hard stop and regenerate the
plan first.

If an Agent wants a LeRobot-shaped non-hardware closure step before any native
rollout helper exists, use the rollout preview helper:

```bash
uv run armctrl lerobot preview-rollout \
  --eef-plan-dir <dir> \
  --json
```

This reuses the EEF preview synthesizer plus the shared simulation review chain
and keeps `movement_allowed: false`.

Review a LeRobot rollout trajectory through the same shared simulation gate:

```bash
uv run armctrl lerobot review-rollout \
  --eef-plan-dir <dir> \
  --trajectory <joint_csv> \
  --json
```

Stage a rollout-produced joint trajectory into the shared review path first:

```bash
uv run armctrl lerobot stage-rollout-trajectory \
  --eef-plan-dir <dir> \
  --trajectory <joint_csv> \
  --json
```

Review the backend-generated joint trajectory through the shared simulation gate:

```bash
uv run armctrl eef review --plan-dir <dir> --json
```

If the backend wrote the reviewed joint trajectory to the conventional path
`<dir>/backend_joint_trajectory.csv`, the review command can auto-discover it.
Otherwise pass `--trajectory <path>`.

Inspect executor status and cleanup:

```bash
uv run armctrl recipe status --json
uv run armctrl recipe cancel --json
```

## Hard Boundaries

- Do not call raw SDK motion methods.
- Do not call arx5-interface directly.
- Do not create ad hoc joint commands.
- Do not invent a separate EEF protocol outside `armctrl eef`.
- Do not bypass `armctrl recipe`, `armctrl eef`, or the shared simulation review chain.
- Treat any non-JSON output or nonzero exit code as a failed action.
- Do not claim hardware execution is available unless `armctrl` explicitly reports it.

## Expected Flow

1. Classify the request into one Agent Action Protocol intent: `preset.apply`,
   `eef.pose_delta`, `eef.pose_absolute`, `eef.twist`, or `rollout.prepare`.
2. Run the matching route from the protocol table near the top of this skill.
   Prefer `uv run armctrl agent-flow plan ... --json` when the user request
   combines a preset posture and a later EEF or LeRobot handoff.
3. For any EEF intent, write a plan bundle with `--output <dir> --json`, then
   export `uv run armctrl eef export-agent-session-plan --plan-dir <dir> --json`.
   Treat that session plan as the top-level Agent artifact.
4. For LeRobot-aligned behavior, keep the Agent action as `eef.pose_delta` and
   export `uv run armctrl lerobot export-processor-contract --eef-plan-dir <dir> --json`.
   Do not convert Agent intent into native LeRobot command fields yourself;
   `robot_action_processor` / `robot_observation_processor` own that adapter.
5. If a mature backend or helper produces a joint-space trajectory, stage it
   through `uv run armctrl eef stage-trajectory --plan-dir <dir> --trajectory <joint_csv> --json`
   or the LeRobot wrapper `uv run armctrl lerobot stage-rollout-trajectory --eef-plan-dir <dir> --trajectory <joint_csv> --json`.
6. Run the shared review gate: `uv run armctrl eef review --plan-dir <dir> --json`
   or `uv run armctrl lerobot review-rollout --eef-plan-dir <dir> --trajectory <joint_csv> --json`.
7. Read `recommended_path`, `ordered_steps`, `next_steps`, `movement_allowed`,
   and review status before proposing any next action.
8. Report the Minimum Agent Report fields. Include the selected intent, plan
   directory, runtime/processor owner, artifacts, review status, and first
   blocking reason when blocked.
9. Only discuss hardware execution once the mature backend owner and shared
   simulation review are both clear and `armctrl` explicitly exposes an
   execution gate for that path.
