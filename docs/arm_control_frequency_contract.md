# Arm Control Frequency Contract

This note hands off the frequency and timing context needed to build a unified real motion-control backend for ARX X5.
It combines the current `armctrl-clean` SysID/OED work with context read from Codex thread `019e9bd8-11c1-78d1-bd01-2e063ff80956`.

## Source Context

- Current SysID/OED branch: `codex/armctrl-clean-rebuild` in `D:\repo\Roboclaw\references\projects\armctrl-clean`.
- Referenced thread file: `C:\Users\c1rcLEmoon\.codex\sessions\2026\06\06\rollout-2026-06-06T15-31-32-019e9bd8-11c1-78d1-bd01-2e063ff80956.jsonl`.
- That thread focused on the older `armctrl` real SDK side: `Arx5JointController`, `set_joint_cmd`, `set_joint_traj`, damping/fault landing, teleop send rate, UI rate, and controller timestamp handling.
- Current branch focused on safe SysID planning/review: FIGAROH/Pinocchio OED, execution-trajectory resampling, safety gates, URDF/FK/collision preview, and offline hardware-smoke readiness.

## Frequency Vocabulary

Use these names consistently. Most previous bugs came from mixing these layers.

| Name | Meaning | Current target | Owner |
| --- | --- | ---: | --- |
| `planning_sample_hz` / `oed_sample_hz` | Low-rate optimizer grid used by FIGAROH/OED to design the mathematical trajectory. | Usually `20 Hz` for Fourier/OED scans | OED backend / armctrl handoff |
| `trajectory_sample_hz` / `execution_sample_hz` | High-rate trajectory artifact after interpolation/resampling; this is what safety gates inspect and what a real backend should replay. | `100 Hz` in current SysID contract | armctrl safety layer |
| `sdk_send_hz` | Rate at which armctrl sends setpoints to the SDK if replaying point-by-point. | Start with `100 Hz`; measure real jitter | real motion backend |
| `controller_dt` | SDK/controller internal tick, read from `controller.get_controller_config().controller_dt`; old thread observed default/fallback `0.002 s`. | Treat as measured, often `500 Hz` internal | SDK |
| `record_sample_hz` | Dataset logging/sample rate for SysID observations. Should align with executed trajectory unless intentionally oversampled. | `100 Hz` for current SysID | recorder/backend |
| `ui_hz` | UI/dashboard refresh. Must not control motion timing. | `50 Hz` or lower | UI only |
| input event rate | Gamepad/operator event rate. Must not control motion timing directly. | event-driven | input layer |

## Non-Negotiable Separation

- Do not derive OED velocity limits from `max_joint_step_rad * planning_sample_hz`.
- `max_joint_step_rad` is an execution-layer safety gate on the high-rate trajectory.
- OED receives physical/profile velocity and acceleration limits.
- SDK execution receives a fully checked high-rate trajectory, not an optimizer waypoint list.
- UI/input timing must never slow down or speed up the real command loop.

## Current SysID Timing Contract

Current `configs/x5.safe.yaml` intent:

| Profile | Planning | Execution | Step gate | Velocity intent | Acceleration intent |
| --- | ---: | ---: | ---: | --- | --- |
| `gravity_sweep` | plan-only or simple generated profile | `100 Hz` | `0.015 rad/sample` | `[1.0, 0.8, 0.8, 1.0, 1.0, 1.0] rad/s` | `[2.0, 1.5, 1.5, 2.0, 2.0, 2.0] rad/s^2` |
| `friction_sweep` | segmented profile with velocity plateaus | `100 Hz` | `0.018 rad/sample` | `[1.8, 1.2, 1.2, 2.0, 2.0, 2.0] rad/s` | `[25, 25, 25, 25, 25, 25] rad/s^2` |
| `fourier_multisine` | FIGAROH/OED, typically `20 Hz` | `100 Hz` | `0.020 rad/sample` | `[2.0, 1.4, 1.4, 2.0, 2.0, 2.0] rad/s` | `[30, 30, 30, 30, 30, 30] rad/s^2` |

Known Fourier candidate evidence:

- FIGAROH base-regressor condition: `68.43`.
- Pinocchio effective condition: `77.57`.
- Execution trajectory samples: `196`.
- Max joint step: about `0.018476 rad`.
- Max velocity: about `1.84275 rad/s`.
- Max acceleration: about `17.8125 rad/s^2`.
- Pinocchio/coal collision check: pass for the configured URDF collision model and allowed adjacent pairs.
- Gate state: `hardware_smoke_plan_ready`, but `movement_allowed=false` and `hardware_execution_eligible=false` until gravity/friction smoke tests are run first.

## Real Backend Requirements

The unified real backend should accept a checked trajectory artifact with:

- `time_s`: monotonic, aligned to `1 / execution_sample_hz`.
- `q`: commanded joint positions for all active joints.
- Optional `dq` and `ddq`: if absent, compute for logging only; do not invent dynamics semantics silently.
- Metadata: `planning_sample_hz`, `execution_sample_hz`, `profile`, `max_joint_step_rad`, velocity/acceleration limits, interpolation method, URDF path, safe config path, and gate results.

The backend should expose at least:

- `prepare()`: connect SDK, read `controller_dt`, read current state, verify dof/model/interface.
- `enter_safe_hold_or_damping()`: put the robot in a known state before recording.
- `arm_motion_gains()`: ramp gains from damping/zero-kp into tracking using `controller_dt`.
- `execute_trajectory()`: replay checked trajectory with timestamps and jitter logging.
- `hold_last_sample()`: hold final command until explicit damping/cancel if required.
- `damping()`: always available, used on Ctrl-C/fault.
- `read_sample()`: record actual q/dq/current/torque/state timestamp at the contract sample rate.

## SDK Timestamp Guidance From Referenced Thread

The old SDK backend used these timing ideas:

- Read `controller_dt = controller.get_controller_config().controller_dt`, falling back to `0.002 s` only if missing/invalid.
- Before motion, send a sync command at current measured joint state with timestamp `controller.get_timestamp() + controller_dt`.
- For repeated hold, use lookahead `max(0.04, controller_dt * 5.0)`.
- For gain recovery from damping, ramp over `resume_gain_duration_s / controller_dt` steps.
- On Ctrl-C or any backend fault, request damping immediately and report that damping was requested.

These should be treated as SDK integration constraints, not as SysID trajectory-design constraints.

## Execution Loop Contract

For a point-by-point SDK backend at `100 Hz`:

1. Load the already gated `execution_trajectory.csv`.
2. Verify `dt = 0.01 s`, monotonic `time_s`, and no missing samples.
3. Read controller timestamp and current joint state before first command.
4. Optionally preposition/hold to safe start pose before recording.
5. Start recorder only after safe state is reached.
6. Send each trajectory point according to absolute trajectory time, not according to loop iteration count alone.
7. For each command, log target time, actual send time, controller timestamp, command q, measured q, current/torque, and error/fault flags.
8. If loop overruns, record the overrun and avoid sending stale bursts.
9. On completion, either hold last sample or enter damping according to the profile/run mode.

## Safety Gates Before SDK Motion

The real backend must not be the first component to discover an unsafe trajectory.

Required gates before `execute_trajectory()`:

- Human confirmation string for real SDK motion.
- URDF joint position limits.
- Profile safe joint ranges.
- Execution max joint step.
- Execution velocity and acceleration limits.
- Workspace/table/base keepout checks.
- Pinocchio/coal or another mature collision/state-validity backend.
- HTML/URDF preview available for human visual review.
- For Fourier full run: gravity and friction smoke must already be reviewed.

The backend should refuse execution if any upstream gate is missing or stale.

## Agent/Recipe/LeRobot Alignment

The same backend should serve all motion producers:

- Agent recipe commands produce bounded plans and hand them to the same safety/preview/gate path.
- SysID produces planned and execution trajectories, then passes only execution trajectories to hardware.
- LeRobot rollout should be treated as another trajectory producer. It must pass through the same safe trajectory adapter before SDK execution.
- Teleop is different because it is online; it still uses a fixed `rate_hz` command loop and must enforce per-tick delta/velocity/workspace guards.

## Open Questions For The Real-Control Thread

- Does the current SDK prefer `set_joint_traj` with full trajectory handoff, or `set_joint_cmd` at `100 Hz` with timestamps?
- What is the measured stable `sdk_send_hz` on n100d over SocketCAN/USB-CAN under load?
- Does SDK internally interpolate between timestamped commands at `controller_dt`, and how far ahead should timestamps be scheduled for non-hold motion?
- What is the maximum observed send jitter before X5 tracking becomes unstable?
- Can current/torque feedback be sampled at `100 Hz` reliably while sending commands?
- Should D435 geometry be added as a simplified collision primitive? Current URDF includes D435i mass merged into `link6`, but not a separate camera collision body.

## Suggested First Milestones

1. Implement a read-only SDK timing doctor: connect, read `controller_dt`, timestamp monotonicity, state update rate, and CAN interface labels without motion.
2. Implement fake-backend timing tests for a `100 Hz` trajectory replay loop, including overrun accounting.
3. Implement SDK hold-only smoke: no trajectory, just enter safe hold/damping with timestamp lookahead and jitter logging.
4. Implement one-point or two-point tiny motion smoke behind confirmation, with max step far below normal SysID.
5. Implement gravity/friction smoke replay from checked `execution_trajectory.csv`.
6. Only after those pass, enable Fourier 68.43 replay as a hardware smoke candidate.

## Handoff Summary

The real motion backend should be a timing-accurate replay and safety-enforcement adapter, not a new trajectory optimizer. SysID/OED owns trajectory design, armctrl owns safety gating and artifact contracts, and the SDK owns low-level motor/CAN execution. The core engineering task is to keep `planning_sample_hz`, `execution_sample_hz`, `sdk_send_hz`, `controller_dt`, `record_sample_hz`, and `ui_hz` separate, measured, and logged.
