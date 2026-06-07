# Lab Hardware Validation Checklist

This is the short lab-facing checklist for the current armctrl-clean branch.

Mainline target:

`power on passive/droop -> runtime start arx5_sdk --serve -> recover to SAFE_CENTER -> active hold -> status -> Agent/SysID/Recipe submit queued owner command -> runtime consumes queue -> runtime stop`

Important semantics:

- Droop/passive pose can be physically safe at rest, but it is not the Agent/SysID start pose.
- `SAFE_CENTER="0.0 0.3 0.3 0.0 0.0 0.0"` is a controlled hover pose and must be continuously held.
- Readiness must mean the live runtime is fresh and still holding near `SAFE_CENTER`, not that an older command once reached it.
- Real Agent/SysID/Recipe CLIs must not open SDK/CAN directly. They now require a runtime artifact and submit queued owner commands for the live runtime to execute.

Stop immediately on unexpected sag, collision risk, abnormal sound, high current, non-empty fault flags, or any `status` other than the expected value.

## 0. Setup

Run on the robot host from the armctrl-clean checkout:

```bash
export RUN_DIR="runs/lab-runtime-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$RUN_DIR"
export SAFE_CENTER="0.0 0.3 0.3 0.0 0.0 0.0"
```

Expected observation:

- No robot motion yet.
- `$RUN_DIR` exists and will contain all artifacts for this attempt.

## 1. Start Long-Lived Runtime

This opens SDK/CAN once, reads measured pose, recovers to `SAFE_CENTER`, then continuously streams hold commands. Keep this terminal open.

```bash
uv run armctrl runtime start \
  --backend arx5_sdk \
  --model X5 \
  --interface can0 \
  --safe-center $SAFE_CENTER \
  --send-hz 50 \
  --hold-hz 50 \
  --max-joint-step-rad 0.01 \
  --max-heartbeat-age-s 1.0 \
  --serve \
  --confirm "I UNDERSTAND THIS WILL START THE REAL ARM RUNTIME" \
  --output "$RUN_DIR/runtime_session.json" \
  --json
```

Expected observation:

- The arm should move slowly and continuously from the measured passive/droop pose toward `SAFE_CENTER`.
- There should be no first-frame jump; the recovery starts from freshly measured `q_meas`.
- After reaching `SAFE_CENTER`, the arm should keep holding and must not drop.
- The command does not return while serving. Use another terminal for status/attach checks.

## 2. Check Runtime Status

In a second terminal:

```bash
uv run armctrl runtime status \
  --session-artifact "$RUN_DIR/runtime_session.json" \
  --max-heartbeat-age-s 1.0 \
  --output "$RUN_DIR/runtime_status.json" \
  --json
```

Expected artifact checks:

- `status == "ok"`
- `mode == "hold_safe"`
- `owner == null`
- `hold_hz == 50.0`
- `hold_tick_count > 0`
- `last_hold_wall_time_s` is present and recent
- `hold_fresh == true`
- `readiness.agent_sysid_smoke_allowed == true`
- `readiness.failed_checks == []`
- `q_meas` is close to `q_hold`, and `q_hold` is close to `safe_center`.

If `readiness.failed_checks` contains `hold_fresh`, do not start Agent, SysID,
or Recipe. It means the live serving runtime has not recently refreshed active
hold evidence. Check that terminal 1 is still running, that the arm is not in
damping, and that `runtime_session.json` is still being updated.

## 3. Agent Attach Gate

Generate an Agent contract. This is non-hardware planning:

```bash
export AGENT_PLAN_DIR="$RUN_DIR/agent-flow-plan"
uv run armctrl agent-flow plan \
  --preset home \
  --eef-mode pose_delta \
  --backend sdk_cartesian \
  --delta-position 0.002 0.000 0.000 \
  --delta-rpy 0.0 0.0 0.0 \
  --control-period-s 0.1 \
  --output "$AGENT_PLAN_DIR" \
  --json
```

Now verify that real Agent execution refuses to bypass the long-lived runtime:

```bash
uv run armctrl agent-flow runtime-smoke-real \
  --contract "$AGENT_PLAN_DIR/agent_flow_plan.json" \
  --readiness-artifact "$RUN_DIR/runtime_status.json" \
  --runtime-session-artifact "$RUN_DIR/runtime_session.json" \
  --model X5 \
  --interface can0 \
  --q-start $SAFE_CENTER \
  --q-target 0.0 0.302 0.3 0.0 0.0 0.0 \
  --send-hz 50 \
  --max-joint-delta-rad 0.005 \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM WITH AGENT INTENT" \
  --output "$RUN_DIR/agent_runtime_attach.json" \
  --json
```

Expected result:

- `status == "queued"`
- `movement_command_sent == false` in the submit artifact; motion is sent only by the serving runtime.
- `runtime.owner == "agent"` and `runtime.mode == "agent_servo"`.
- A runtime command result artifact should appear under the session command queue.
- The arm should make only the requested small intent motion, then return to active hold.

Inspect the live runtime result, not only the submit artifact:

```bash
RESULT=$(ls -t "$RUN_DIR/runtime_session_commands/results"/*.json | head -1)
python -m json.tool "$RESULT" | sed -n '1,180p'
```

Expected runtime result checks:

- `status == "completed"`
- `owner == "agent"`
- `motion.resampling_policy == "intent_frame_to_runtime_send_hz"`
- `motion.interpolation_policy == "linear_intent_frame"`
- `motion.actual_send_hz` is present
- `motion.send_jitter_ms_p95`, `motion.send_jitter_ms_p99`, and `motion.send_jitter_ms_summary` are present
- `motion.dt_min_s`, `motion.dt_max_s`, and `motion.dt_avg_s` are present
- `acceptance.timing_gate.status == "pass"`
- `acceptance.status_publish_gate.status == "pass"`

If this returns `blocked` or `rejected`, inspect `runtime_status.json` first; readiness must be fresh live hold at `SAFE_CENTER`.

## 4. SysID Attach Gate

Verify the same no-direct-SDK rule for SysID:

```bash
uv run armctrl sysid run gravity_sweep \
  --adapter sdk \
  --model X5 \
  --interface can0 \
  --dof 6 \
  --sample-hz 100 \
  --duration 8 \
  --amplitude 0.05 \
  --q-center $SAFE_CENTER \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output "$RUN_DIR/ident-sdk-gravity-smoke" \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" \
  --readiness-artifact "$RUN_DIR/runtime_status.json" \
  --runtime-session-artifact "$RUN_DIR/runtime_session.json" \
  --json
```

Expected result:

- `status == "queued"`
- `movement_command_sent == false` in the submit artifact; motion is sent only by the serving runtime.
- `runtime.owner == "sysid"`
- `runtime.mode == "trajectory_replay"`
- A runtime command result artifact should appear under the session command queue.
- The arm should execute the safety-reviewed SysID trajectory, then return to active hold.

Inspect the live runtime result:

```bash
RESULT=$(ls -t "$RUN_DIR/runtime_session_commands/results"/*.json | head -1)
python -m json.tool "$RESULT" | sed -n '1,220p'
```

Expected runtime result checks:

- `status == "completed"`
- `owner == "sysid"`
- `motion.sample_count == 801` for `--duration 8 --sample-hz 100`
- `motion.trajectory_sample_hz == 100.0`
- `motion.runtime_send_hz == 100.0`
- `motion.resampling_policy == "none_sample_hz_matches_send_hz"`
- `motion.interpolation_policy == "pre_sampled_joint_positions"`
- `motion.actual_send_hz`, `motion.send_jitter_ms_p95`, `motion.send_jitter_ms_p99`, and `motion.send_jitter_ms_summary.buckets` are present
- `motion.dt_min_s`, `motion.dt_max_s`, and `motion.dt_avg_s` are present
- `timing.queue_latency_s`, `timing.acquire_latency_s`, `timing.first_send_latency_s`, and `timing.execution_elapsed_s` are present
- `acceptance.timing_gate.status == "pass"`
- `acceptance.status_publish_gate.status == "pass"`

Run the SysID command a second time with a different output directory:

```bash
uv run armctrl sysid run gravity_sweep \
  --adapter sdk \
  --model X5 \
  --interface can0 \
  --dof 6 \
  --sample-hz 100 \
  --duration 8 \
  --amplitude 0.05 \
  --q-center $SAFE_CENTER \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output "$RUN_DIR/ident-sdk-gravity-smoke-repeat" \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" \
  --readiness-artifact "$RUN_DIR/runtime_status.json" \
  --runtime-session-artifact "$RUN_DIR/runtime_session.json" \
  --json
```

The second run must also queue and complete from live `q_hold`. It must not be rejected because `q_hold` no longer exactly equals the historical `SAFE_CENTER`.

If this returns `blocked` or `rejected`, inspect `runtime_status.json` and the generated SysID plan safety result before retrying.

## 5. Recipe Attach Gate

Verify the same queued-owner rule for Recipe. First generate the reviewed plan:

```bash
export RECIPE_PLAN_DIR="$RUN_DIR/recipe-home-plan"
uv run armctrl recipe plan home \
  --sample-hz 50 \
  --duration 0.1 \
  --output "$RECIPE_PLAN_DIR" \
  --json
```

Then submit it to the live runtime queue:

```bash
uv run armctrl recipe runtime-submit \
  --plan-dir "$RECIPE_PLAN_DIR" \
  --runtime-session-artifact "$RUN_DIR/runtime_session.json" \
  --output "$RUN_DIR/recipe_runtime_submit.json" \
  --json
```

Expected result:

- `status == "queued"`
- `movement_command_sent == false` in the submit artifact; motion is sent only by the serving runtime.
- `runtime.owner == "recipe"`
- `runtime.mode == "trajectory_replay"`
- A runtime command result artifact should appear under the session command queue.
- The arm should execute the reviewed Recipe trajectory, then return to active hold.

Do not pass `--runtime-session-artifact` to `recipe runtime-smoke-fake`; that command is pure fake/local validation only.

## 6. Stop Runtime

Stop from the second terminal:

```bash
uv run armctrl runtime stop \
  --session-artifact "$RUN_DIR/runtime_session.json" \
  --max-heartbeat-age-s 1.0 \
  --output "$RUN_DIR/runtime_stop.json" \
  --json
```

Expected observation:

- The serving terminal should exit.
- The runtime artifact should show stopped/damping state.
- The arm may become passive after damping; be ready for gravity.

## Appendix. Bringup Diagnostic Only

The old `sysid sdk-*` commands are diagnostic tools for SDK bringup and API auditing. They are not the mature multi-source control path.

Use them only when debugging low-level SDK/CAN behavior:

```bash
uv run armctrl sysid sdk-doctor \
  --model X5 \
  --interface can0 \
  --state-sample-count 20 \
  --state-sample-period 0.01 \
  --output "$RUN_DIR/sdk_doctor.json" \
  --json
```

```bash
uv run armctrl sysid sdk-hold-damping-check \
  --model X5 \
  --interface can0 \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --confirm "I UNDERSTAND THIS WILL CHANGE THE ARM CONTROL MODE" \
  --output "$RUN_DIR/hold_damping.json" \
  --json
```

```bash
uv run armctrl sysid sdk-arm-session \
  --model X5 \
  --interface can0 \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --hold-damping-artifact "$RUN_DIR/hold_damping.json" \
  --confirm "I UNDERSTAND THIS WILL ARM THE SDK SESSION WITHOUT MOVING" \
  --output "$RUN_DIR/session.json" \
  --json
```

Do not use tiny motion as the normal way to recover from droop to `SAFE_CENTER`. Recovery and hold belong to the long-lived runtime.
