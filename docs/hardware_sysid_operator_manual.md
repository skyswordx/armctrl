# ARX5/X5 真机 SysID 与 Agent 操作手册

本文面向 n100d 目标机和 X5 真机验证。所有命令默认在目标机执行：

```bash
cd ~/Roboclaw/references/projects/armctrl-clean-n100d
```

## 0A. Runtime-first real-motion startup sequence

This section is the current operator-grade path for real hardware. It is
runtime-first: one long-lived runtime opens SDK/CAN once, recovers from the
fresh measured passive/droop pose to `SAFE_CENTER`, then continuously holds the
arm while Agent/SysID/Recipe submit queued owner commands.

Do not run Agent or SysID hardware smoke unless live runtime status proves
`mode == "hold_safe"`, `owner == null`, heartbeat is fresh, `q_meas` is close to
`q_hold`, and `q_hold` is close to `SAFE_CENTER`. Historical recovery artifacts
do not prove current readiness.

The formal startup path is now:

```text
power on passive/droop
  -> armctrl runtime start --backend arx5_sdk --serve
  -> runtime status live readiness
  -> Agent/SysID/Recipe submit queued owner command
  -> runtime consumes queue and releases back to hold_safe
  -> runtime stop
```

The old `sysid sdk-*` and `sdk-tiny-motion-*` commands are Bringup Diagnostic
Only. They are useful for low-level SDK/CAN audits, but they are not the mature
multi-source control path and must not be used to prove live Agent/SysID
readiness.

```bash
RUN_DIR="runs/real-motion-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$RUN_DIR"
SAFE_CENTER="0 0.30 0.30 0 0 0"  # active startup recovery target
```

Start the long-lived runtime and keep this terminal open:

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

- The arm moves slowly from the freshly measured passive/droop pose to `SAFE_CENTER`.
- After reaching `SAFE_CENTER`, the runtime keeps streaming hold commands and the arm must not drop.
- The command keeps running while serving. Use a second terminal for status and submit commands.

Check live readiness from the second terminal:

```bash
uv run armctrl runtime status \
  --session-artifact "$RUN_DIR/runtime_session.json" \
  --max-heartbeat-age-s 1.0 \
  --output "$RUN_DIR/runtime_status.json" \
  --json
```

Required live readiness checks:

- `status == "ok"`
- `mode == "hold_safe"`
- `owner == null`
- `readiness.agent_sysid_smoke_allowed == true`
- `readiness.failed_checks == []`
- `q_meas` is close to `q_hold`, and `q_hold` is close to `safe_center`.

Agent/SysID/Recipe real paths must use `--runtime-session-artifact
"$RUN_DIR/runtime_session.json"` and submit/acquire a runtime owner. They must
not open SDK/CAN directly.

Stop the runtime explicitly when the lab run is over:

```bash
uv run armctrl runtime stop \
  --session-artifact "$RUN_DIR/runtime_session.json" \
  --max-heartbeat-age-s 1.0 \
  --output "$RUN_DIR/runtime_stop.json" \
  --json
```

Expected observation:

- The serving terminal exits.
- The runtime artifact shows `status == "stopped"` and `mode == "damping"`.
- The arm may become passive after damping; be ready for gravity.

## 0B. Bringup Diagnostic Only: legacy sysid sdk-* gates

The following commands are for SDK bringup and API auditing only. Use them when
debugging low-level SDK/CAN behavior, not as the normal Agent/SysID/Recipe
control path.

Read-only SDK doctor. This must not send motion commands:

```bash
uv run armctrl sysid sdk-doctor \
  --model X5 \
  --interface can0 \
  --state-sample-count 20 \
  --state-sample-period 0.01 \
  --output "$RUN_DIR/sdk_doctor.json" \
  --json
```

Required artifact checks:

- `schema == "armctrl.sysid_sdk_doctor.v1"`
- `read_only == true`
- `movement_allowed == false`
- `doctor_gate.status == "pass"`
- `controller.controller_dt_s` is measured
- `state_read.state_read_hz` is measured
- `measurement_window` records requested sample count/period and observed monotonic read duration
- `timestamp_policy.monotonic == true`
- `motion_commands_sent == false`

Extracting the measured current joint state is useful for audit, but do not use
it as an operator-guessed `--q-current` for startup. Recovery reads the real SDK
state again immediately before motion:

```bash
MEASURED_Q_CURRENT="$(python3 - <<'PY'
import json
import os
from pathlib import Path
payload = json.loads((Path(os.environ["RUN_DIR"]) / "sdk_doctor.json").read_text())
q = payload.get("state_read", {}).get("q_meas_last")
if not isinstance(q, list) or len(q) != 6:
    raise SystemExit("sdk_doctor.json does not contain state_read.q_meas_last")
print(" ".join(f"{float(v):.9f}" for v in q))
PY
)"
echo "$MEASURED_Q_CURRENT"
```

Expected observation:

- This prints six measured joint angles from the SDK.
- If the arm is physically horizontal/folded, these values are allowed to differ from `SAFE_CENTER`.
- Do not overwrite `MEASURED_Q_CURRENT` with `SAFE_CENTER`; `SAFE_CENTER` is the active startup target, not the measured droop pose.

Hold/damping mode check. This may change controller mode, but must not send
joint trajectory commands:

```bash
uv run armctrl sysid sdk-hold-damping-check \
  --model X5 \
  --interface can0 \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --confirm "I UNDERSTAND THIS WILL CHANGE THE ARM CONTROL MODE" \
  --output "$RUN_DIR/hold_damping.json" \
  --json
```

Required artifact checks:

- `schema == "armctrl.sysid_sdk_hold_damping_check.v1"`
- `prerequisites.doctor == "pass"`
- `mode_change_allowed == true`
- `joint_commands_sent == false`
- `damping.status == "called"`
- `landing_policy` is either `hold_then_damping` or `damping_only`
- For `arx5-interface==0.1.2` on n100d, `set_to_hold` is not exposed by the Python SDK; `hold.status == "unsupported"` plus `damping.status == "called"` is the expected `damping_only` contract, not a CAN/power failure.

Arm the SDK session. This reads the measured pose and records the controller
contract without sending joint commands:

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

Recover from the measured droop/rest pose to the active startup pose. This is the
first real movement step:

```bash
uv run armctrl sysid sdk-recover-startup-real \
  --session-artifact "$RUN_DIR/session.json" \
  --q-target $SAFE_CENTER \
  --safe-config configs/x5.safe.yaml \
  --send-hz 50 \
  --confirm "I UNDERSTAND THIS WILL RECOVER THE REAL ARM TO STARTUP POSE" \
  --output "$RUN_DIR/startup_recovery.json" \
  --json
```

Expected observation:

- The arm should move continuously and slowly from the measured rest pose toward `SAFE_CENTER`.
- The first command is the freshly measured SDK pose; there should be no first-frame jump to `SAFE_CENTER`.
- Per-sample joint motion is bounded by `configs/x5.safe.yaml` `max_joint_step_rad` unless explicitly overridden.
- On any abnormal sound, sag, collision risk, or fault, stop and inspect the artifact; runtime faults should land in damping.

Refresh the session after recovery before running `sdk-jog-real`. This does not
move the arm; it updates `state.q_meas` so the jog gate starts from the recovered
startup pose rather than the pre-recovery droop pose:

```bash
uv run armctrl sysid sdk-arm-session \
  --model X5 \
  --interface can0 \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --hold-damping-artifact "$RUN_DIR/hold_damping.json" \
  --confirm "I UNDERSTAND THIS WILL ARM THE SDK SESSION WITHOUT MOVING" \
  --output "$RUN_DIR/session_after_recovery.json" \
  --json
```

Optional tiny motion diagnostic. Use this only if you specifically want a
single-joint SDK bringup check after startup recovery:

```bash
uv run armctrl sysid sdk-tiny-motion-plan \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --hold-damping-artifact "$RUN_DIR/hold_damping.json" \
  --joint-index 2 \
  --delta-rad 0.002 \
  --max-delta-rad 0.005 \
  --confirm "I UNDERSTAND THIS WILL MOVE ONE JOINT A TINY AMOUNT" \
  --output "$RUN_DIR/tiny_motion_plan.json" \
  --json
```

```bash
uv run armctrl sysid sdk-tiny-motion-execute-real \
  --plan-artifact "$RUN_DIR/tiny_motion_plan.json" \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --hold-damping-artifact "$RUN_DIR/hold_damping.json" \
  --model X5 \
  --interface can0 \
  --dof 6 \
  --q-current $MEASURED_Q_CURRENT \
  --send-hz 50 \
  --max-q-current-error-rad 0.02 \
  --confirm "I UNDERSTAND THIS WILL EXECUTE THE TINY MOTION PLAN" \
  --output "$RUN_DIR/tiny_motion_real.json" \
  --json
```

If this returns `status == "rejected"` with
`reason` containing `q_current does not match measured SDK state`, no joint
command was sent. Keep the artifact and use `sdk-recover-startup-real`; do not
rerun tiny motion with a guessed pose.

Required real tiny-motion artifact checks:

- `schema == "armctrl.sysid_sdk_tiny_motion_execute.v1"`
- `backend == "arx5_sdk"`
- `run_status == "completed"`
- `hardware_motion == true`
- `movement_command_sent == true`
- `motion_runtime.status == "completed"`
- `motion_runtime.landing_mode == "hold"`
- `motion_runtime.controller_dt_s` is measured
- `motion_runtime.actual_send_hz`, `send_jitter_ms_p95`, and `send_jitter_ms_p99` are recorded
- `motion_runtime.fault_flags == []`
- `acceptance.schema == "armctrl.real_motion_acceptance.v1"`
- `acceptance.stage == "tiny_motion"`
- `acceptance.status == "pass"`
- `acceptance.next_gate == "agent_sysid_smoke_readiness"`

Legacy readiness compatibility check. For the runtime-first path, prefer
`armctrl runtime status` from section 0A. If you still need the old
`sdk-agent-sysid-smoke-readiness` wrapper for compatibility, feed it the live
runtime status artifact. Do not use startup recovery or tiny-motion history to
prove current hover readiness:

```bash
uv run armctrl sysid sdk-agent-sysid-smoke-readiness \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --hold-damping-artifact "$RUN_DIR/hold_damping.json" \
  --runtime-status-artifact "$RUN_DIR/runtime_status.json" \
  --output "$RUN_DIR/agent_sysid_readiness.json" \
  --json
```

Required readiness checks:

- `schema == "armctrl.sysid_agent_smoke_readiness.v1"`
- `status == "ok"`
- `agent_sysid_smoke_allowed == true`
- `prerequisites.doctor == "pass"`
- `prerequisites.hold_damping == "pass"`
- `prerequisites.runtime_status == "pass"`
- `runtime_status.mode == "hold_safe"`
- `runtime_status.owner == null`
- `runtime_status.readiness.agent_sysid_smoke_allowed == true`

Only after live runtime readiness passes may the operator prepare Agent real
smoke or SysID smoke. Real control sources must still use
`--runtime-session-artifact "$RUN_DIR/runtime_session.json"` so the serving
runtime owns SDK/CAN and executes the queued command.

## 0. 当前能力边界

`armctrl` clean rebuild 当前已经具备：

- fake SysID 数据链路；
- SysID planned trajectory 生成；
- URDF joint limit 检查；
- URDF frame-level table clearance 检查；
- SDK handshake/preflight；
- 最小 `arx5_interface` joint-control SDK runner 接线；
- SDK runner 不自动 `reset_to_home`；计划阶段会检查相邻采样点关节步长，确认后从当前姿态按 `max_joint_step_rad` 限步过渡到计划轨迹第一帧；
- Ctrl-C/fault 通过 `finally` 尝试落到 damping；
- 后处理与 solver handoff；
- Agent recipe CLI 安全门。

仍需人工确认：

- USB-CAN 硬件链路是否稳定；
- 机械臂底座是否固定；
- 末端下方是否无桌面/障碍物；
- 真机 Ctrl-C 后是否确实进入 damping；
- 运动幅度是否适合当前摆放。

## 1. USB 转 CAN 启动

先看 USB 设备：

```bash
ls /dev/ttyACM*
```

如果只有一个 USB-CAN，通常是 `/dev/ttyACM0`。设置 CAN：

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe slcan

sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
ip link show can0
```

如果已有旧 can0：

```bash
sudo ip link set can0 down || true
sudo pkill slcand || true
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
ip link show can0
```

`-s8` 通常表示 1Mbit/s。若官方设备要求其他速率，以官方说明为准。

## 2. 环境与只读握手

同步代码和依赖：

```bash
git switch codex/armctrl-clean-rebuild
git pull
uv sync --extra dev --extra lerobot
```

只读检查，不打开 CAN，不移动硬件：

```bash
uv run armctrl release status --json

uv run armctrl sysid sdk-preflight \
  --model X5 \
  --interface can0 \
  --json

uv run armctrl sysid sdk-handshake-plan \
  --model X5 \
  --interface can0 \
  --json

uv run armctrl sim doctor --json
```

预期：

- `sdk-preflight.status == ok`
- `sdk.status == available`
- `movement_allowed == false`
- `requires_confirm == "I UNDERSTAND THIS WILL MOVE THE ARM"`
- `sim doctor` 只读检查 `pinocchio_coal`、`mujoco`、`moveit`、`figaroh` 的可用性，不打开 CAN，不实例化 SDK，不移动硬件。

## 3. 安全中心位与安全配置

默认安全中心位：

```bash
SAFE_CENTER="0 0.30 0.30 0 0 0"
```

含义是 6 个关节角，单位 rad。当前经验上 `joint2=0.30`、`joint3=0.30` 能让躯干接近水平伸直且略抬，避免末端一开始下探到桌面。

安全配置文件：

```bash
configs/x5.safe.yaml
```

关键字段：

```yaml
safety:
  workspace_min_m: [0.05, -0.45, 0.02]
  workspace_max_m: [0.75, 0.45, 0.65]
  allowed_workspace_boxes:
    - name: main_body_sweep_volume
      min_m: [-0.35, -0.45, 0.02]
      max_m: [0.75, 0.45, 0.65]
  forbidden_workspace_boxes:
    - name: table_surface
      min_m: [-1.0, -1.0, -0.20]
      max_m: [1.0, 1.0, 0.02]
    - name: base_keepout
      min_m: [-0.08, -0.08, -0.05]
      max_m: [0.08, 0.08, 0.15]
  simulation:
    backend_preference: [pinocchio_coal, mujoco, moveit, urdf_fk_fallback]
    link_frames: [link5, link6, eef_link]
    min_clearance_m: 0.02
  max_sysid_duration_s: 60.0
  max_sysid_sample_hz: 100.0
  max_sysid_amplitude_rad: 0.25
  max_joint_step_rad: 0.01
  settle_before_record_s: 0.5
```

`max_joint_step_rad` 有两层作用：

- `sysid plan` 会检查 planned trajectory 中相邻采样点的最大关节步长，过大则 `trajectory_step_check=fail`；
- `sysid run --adapter sdk` 会用同一个阈值从当前姿态限步过渡到轨迹第一帧，过渡阶段不记录数据。

修改原则：

- 桌面更高时，提高 `workspace_min_m[2]`；
- 末端可能碰桌时，降低 `max_sysid_amplitude_rad`；
- 机器抖动或电流偏高时，降低 `max_sysid_amplitude_rad`，或提高 `sample-hz` / 降低 `max_joint_step_rad` 让每一拍更小；
- 第一帧过渡太快时，降低 `max_joint_step_rad`；
- 首次真机 smoke 建议 `amplitude <= 0.05`、`duration <= 8`。

## 4. 先生成 plan，不动机械臂

```bash
uv run armctrl sysid plan gravity_sweep \
  --dof 6 \
  --sample-hz 100 \
  --duration 8 \
  --amplitude 0.05 \
  --q-center $SAFE_CENTER \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output runs/plan-gravity-smoke \
  --json
```

检查：

```bash
cat runs/plan-gravity-smoke/manifest.json
cat runs/plan-gravity-smoke/trajectory_preview.json
```

必须满足：

- `safety.allowed == true`
- `urdf_limit_check.status == pass`
- `sysid_parameter_check.status == pass`
- `trajectory_step_check.status == pass`
- `workspace_clearance_check.status == pass`
- `simulation_check.status == pass`
- `trajectory_preview.json.backend.selected` 记录实际使用的成熟后端，或在后端缺失时明确标成 `urdf_fk_fallback`

如果 fail，不要运行真机。先减小 `--amplitude` 或调整 `--q-center` / `configs/x5.safe.yaml`。

## 4A. MotionRuntime tiny motion 阶梯门禁

本节用于进入 Agent/SysID 真机 smoke 前的最小真实运动验收。它不替代后面的 SysID gravity smoke；它只证明 SDK doctor、hold/damping、tiny joint motion 和 readiness artifact 都按顺序通过。任一步 `status=rejected`、`movement_command_sent=unknown`、`hardware_motion=unknown`、`fault_flags` 非空，或者 Ctrl-C 后没有进入 damping，都必须停止。

真实运动 CLI 会把 `acceptance.status != "pass"` 当作硬 gate：即使 `run_status == "completed"`，只要 acceptance 是 `incomplete` 或 `review_required`，命令也必须返回非零，顶层 `status` 映射为该 acceptance 状态。不要只看 `run_status` 判断能否进入下一 gate。

在 MotionRuntime 执行路径里，`fault_flags` 非空必须表现为 `motion_runtime.status == "faulted"` 且 `motion_runtime.landing_mode == "damping"`；如果 artifact 显示 fault 后仍继续发送后续点，应视为运行时安全缺陷，不允许进入下一 gate。

建议新开一个独立目录保存产物：

```bash
RUN_DIR="runs/real-tiny-motion-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$RUN_DIR"
```

只读 SDK doctor，不发送运动命令：

```bash
uv run armctrl sysid sdk-doctor \
  --model X5 \
  --interface can0 \
  --state-sample-count 20 \
  --state-sample-period 0.01 \
  --output "$RUN_DIR/sdk_doctor.json" \
  --json
```

必须满足：

- `schema == "armctrl.sysid_sdk_doctor.v1"`
- `read_only == true`
- `movement_allowed == false`
- `interface_status.status == "opened_read_only"`
- `controller.controller_dt_s` 有实测值
- `state_read.status == "ok"`
- `measurement_window` 记录请求采样次数/周期以及实际 monotonic 读取时间窗
- `timestamp_policy.monotonic == true`
- `motion_commands_sent == false`

只允许进入 hold/damping 模式检查，不发送 joint command：

```bash
uv run armctrl sysid sdk-hold-damping-check \
  --model X5 \
  --interface can0 \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --confirm "I UNDERSTAND THIS WILL CHANGE THE ARM CONTROL MODE" \
  --output "$RUN_DIR/hold_damping.json" \
  --json
```

必须满足：

- `schema == "armctrl.sysid_sdk_hold_damping_check.v1"`
- `prerequisites.doctor == "pass"`
- `mode_change_allowed == true`
- `joint_commands_sent == false`
- `damping.status == "called"`
- `landing_policy` 为 `hold_then_damping` 或 `damping_only`
- n100d 当前 `arx5-interface==0.1.2` 的 Python SDK 不暴露 `set_to_hold`；`hold.status == "unsupported"` 且 `damping.status == "called"` 是预期的 `damping_only` 合同，不是 CAN/电源故障。

生成 tiny motion plan。首轮只允许单关节、极小幅度，推荐从 `0.002 rad` 开始，不要超过 `0.005 rad`：

```bash
uv run armctrl sysid sdk-tiny-motion-plan \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --hold-damping-artifact "$RUN_DIR/hold_damping.json" \
  --joint-index 2 \
  --delta-rad 0.002 \
  --max-delta-rad 0.005 \
  --confirm "I UNDERSTAND THIS WILL MOVE ONE JOINT A TINY AMOUNT" \
  --output "$RUN_DIR/tiny_motion_plan.json" \
  --json
```

先用 fake backend 走同一 MotionRuntime 路径。这里仍然不会动真机：

```bash
uv run armctrl sysid sdk-tiny-motion-execute-fake \
  --plan-artifact "$RUN_DIR/tiny_motion_plan.json" \
  --dof 6 \
  --q-current $SAFE_CENTER \
  --send-hz 50 \
  --confirm "I UNDERSTAND THIS WILL EXECUTE THE TINY MOTION PLAN" \
  --output "$RUN_DIR/tiny_motion_fake.json" \
  --json
```

fake artifact 必须满足 `backend == "fake"`、`hardware_motion == false`、`motion_runtime.landing_mode == "hold"`、`motion_runtime.actual_send_hz` 合理。fake 通过不代表真机通过。

真机 tiny motion 只能在人工确认当前机械臂实际姿态接近 `$SAFE_CENTER`、底座固定、手在急停附近后执行。如果当前姿态不是 `$SAFE_CENTER`，必须把 `--q-current` 改成实际读到的安全姿态，不要为了凑命令假填：

```bash
uv run armctrl sysid sdk-tiny-motion-execute-real \
  --plan-artifact "$RUN_DIR/tiny_motion_plan.json" \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --hold-damping-artifact "$RUN_DIR/hold_damping.json" \
  --model X5 \
  --interface can0 \
  --dof 6 \
  --q-current $SAFE_CENTER \
  --send-hz 50 \
  --confirm "I UNDERSTAND THIS WILL EXECUTE THE TINY MOTION PLAN" \
  --output "$RUN_DIR/tiny_motion_real.json" \
  --json
```

真机 tiny motion 必须满足：

- `schema == "armctrl.sysid_sdk_tiny_motion_execute.v1"`
- `backend == "arx5_sdk"`
- `hardware_motion == true`
- `movement_command_sent == true`
- `prerequisites.doctor == "pass"`
- `prerequisites.hold_damping == "pass"`
- `motion_runtime.status == "completed"`
- `motion_runtime.landing_mode == "hold"`
- `motion_runtime.controller_dt_s` 有实测值
- `motion_runtime.actual_send_hz`、`send_jitter_ms_p95`、`send_jitter_ms_p99` 被记录
- `motion_runtime.samples[*].q_cmd` 与 `q_meas` 有极小、受控变化
- `fault_flags` 为空

最后运行 Agent/SysID smoke readiness。这个命令仍然只读 artifact，不运动：

```bash
uv run armctrl sysid sdk-agent-sysid-smoke-readiness \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --hold-damping-artifact "$RUN_DIR/hold_damping.json" \
  --tiny-motion-artifact "$RUN_DIR/tiny_motion_real.json" \
  --output "$RUN_DIR/agent_sysid_readiness.json" \
  --json
```

只有 `agent_sysid_smoke_allowed == true` 时，才允许继续设计 Agent smoke 或 SysID smoke。若为 `false`，不要进入后续真机运动；先看 `prerequisites` 和 `tiny_motion` 摘要。

### 4A.1 Agent real runtime smoke

Agent 真机 smoke 是 `10 Hz` intent 到 MotionRuntime 后端插值/限幅的最小验收，不是让 Agent 直接控制 SDK，也不是 LeRobot rollout loop。它必须复用上面的 `$RUN_DIR/sdk_doctor.json`、`$RUN_DIR/hold_damping.json`、`$RUN_DIR/tiny_motion_real.json` 和 `$RUN_DIR/agent_sysid_readiness.json`。

先生成一个已经通过 review 的 Agent contract。首轮只允许 `eef.pose_delta` 极小位移，`control_period_s=0.1` 表示 Agent intent 为 `10 Hz`：

```bash
AGENT_PLAN_DIR="$RUN_DIR/agent-flow-real-smoke"

uv run armctrl agent-flow plan \
  --preset home \
  --eef-mode pose_delta \
  --backend sdk_cartesian \
  --delta-position 0.002 0.000 -0.003 \
  --delta-rpy 0.0 0.0 0.02 \
  --control-period-s 0.1 \
  --output "$AGENT_PLAN_DIR" \
  --json

uv run armctrl agent-flow review \
  --contract "$AGENT_PLAN_DIR/agent_flow_plan.json" \
  --json
```

只有 review artifact 显示 `review_status == "completed"`、仿真/碰撞/限幅都通过，且 readiness artifact 显示 `agent_sysid_smoke_allowed == true` 时，才允许执行 Agent real runtime smoke。`--q-start` 必须填写真实读到的当前安全姿态；`--q-target` 只能比 `--q-start` 小幅变化，首轮单关节变化不要超过 `0.002 rad`，并保留 `--max-joint-delta-rad 0.005`：

```bash
uv run armctrl agent-flow runtime-smoke-real \
  --contract "$AGENT_PLAN_DIR/agent_flow_plan.json" \
  --readiness-artifact "$RUN_DIR/agent_sysid_readiness.json" \
  --model X5 \
  --interface can0 \
  --q-start $SAFE_CENTER \
  --q-target 0.0 0.302 0.3 0.0 0.0 0.0 \
  --send-hz 50 \
  --max-joint-delta-rad 0.005 \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM WITH AGENT INTENT" \
  --output "$RUN_DIR/agent_runtime_smoke_real.json" \
  --json
```

Agent real runtime smoke 必须满足：

- `schema == "armctrl.agent_flow_runtime_smoke_real.v1"`
- `hardware_motion == true`
- `movement_command_sent == true`
- `runtime.backend == "arx5_sdk"`
- `readiness.agent_sysid_smoke_allowed == true`
- `motion_runtime.mode == "agent_servo"`
- `motion_runtime.controller_dt_s` 有实测值
- `motion_runtime.actual_send_hz`、`send_jitter_ms_p95`、`send_jitter_ms_p99` 被记录
- `frequency_contract.schema == "armctrl.agent_frequency_contract.v1"`
- `frequency_contract.agent_intent_hz == 10.0`，当 `control_period_s == 0.1`
- `frequency_contract.backend_send_hz == 50.0`，首轮按 50 Hz 后端发送验证
- `frequency_contract.actual_send_hz` 与 `motion_runtime.actual_send_hz` 一致
- `frequency_contract.missed_intent_exercised_in_this_run == false`
- `motion_runtime.samples[*].q_cmd` 与 `q_meas` 有极小、受控变化
- `watchdog.policy.missed_intent_timeout_s == 0.3`
- `watchdog.policy.missed_intent_exercised_in_this_run == false`，因为真机单帧 smoke 不应故意等到断帧；missed-frame hold 必须由 fake smoke 或后续 streaming driver 测试验证
- `fault_flags` 为空
- `fault_landing_mode` 或 `motion_runtime.landing_mode` 为 `hold` / `damping` 中的预期安全落点
- `acceptance.schema == "armctrl.real_motion_acceptance.v1"`
- `acceptance.stage == "agent_smoke"`
- `acceptance.status == "pass"`
- `acceptance.checks.readiness_gate.status == "pass"`
- `acceptance.checks.q_meas_responded_to_commanded_motion.status == "pass"`
- `acceptance.next_gate == "sysid_smoke"`

如果输出 `status=rejected`，或 `hardware_motion == "unknown"`、`movement_command_sent == "unknown"`，不要重试加大幅度；先检查 rejected artifact 的 `next_gate`、SDK 状态、readiness、急停和机械臂实际姿态。如果 `acceptance.status != "pass"`，同样不能进入下一 gate；优先看 `acceptance.checks` 中的 `fail` / `not_available` 项。fake/preview 通过仍然不能写成“Agent 真机已打通”。

## 5. 真机 gravity smoke

确认：

- 底座已固定；
- 末端下方无桌面；
- 手在实体急停/断电附近；
- 已经知道 Ctrl-C 后应该落 damping；
- 首轮只跑小幅 smoke；
- `$RUN_DIR/agent_sysid_readiness.json` 已经存在，且 `agent_sysid_smoke_allowed == true`；
- 如果 4A.1 Agent real runtime smoke 输出 `status=rejected`、`hardware_motion=unknown` 或 fault，不要进入 SysID smoke。

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
  --output runs/ident-sdk-gravity-smoke \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" \
  --readiness-artifact "$RUN_DIR/agent_sysid_readiness.json" \
  --json
```

SysID smoke 输出和 `runs/ident-sdk-gravity-smoke/manifest.json` 必须满足：

- `adapter == "sdk"`
- `run_status == "completed"`
- `safety.movement_allowed == true`
- `readiness.agent_sysid_smoke_allowed == true`
- `motion_runtime.mode == "trajectory_replay"`
- `motion_runtime.controller_dt_s` 有实测值
- `motion_runtime.actual_send_hz`、`send_jitter_ms_p95`、`send_jitter_ms_p99` 被记录
- `motion_runtime.tracking.final_tracking_error_max_abs_rad` 被记录
- `acceptance.schema == "armctrl.real_motion_acceptance.v1"`
- `acceptance.stage == "sysid_smoke"`
- `acceptance.status == "pass"`
- `acceptance.checks.no_fault_flags.status == "pass"`
- `acceptance.checks.readiness_gate.status == "pass"`

如果 `acceptance.status == "review_required"`，通常说明存在 fault flag、q_cmd/q_meas 没有响应、或某个安全门控证据失败；如果是 `incomplete`，通常说明 `controller_dt_s`、send Hz、jitter 或 tracking 证据缺失。两种情况都不能进入更大幅度 SysID，也不能把结果当作已验收真机运动；CLI 也应返回非零，顶层 `status` 映射为该 acceptance 状态。

如果运行产物中 `run_status == "faulted"`，CLI 必须返回非零退出码并把顶层 `status` 映射为 `"faulted"`。这表示 SDK runner 成功写出了故障产物，不表示这次运动验收通过。此时 `raw_samples.csv` 可能只有 fault 发生前的 partial samples，必须保留这些数据用于复盘，但不能进入 solver 或下一轮加幅度。

如果 tiny motion 或 SysID runtime 产物中 `run_status == "aborted"` / `motion_runtime.status == "aborted"`，必须确认 `motion_runtime.error`、`motion_runtime.landing_mode == "damping"` 和已有 partial `samples`。这表示普通运行时异常已被转成可审计失败产物；它仍然不是验收通过，不能继续 Agent/SysID smoke，也不要重试加大幅度。

停止条件：

- joint2 电流异常；
- 末端下探接近桌面；
- 轨迹明显朝错误方向；
- 机械臂抖动强烈；
- Ctrl-C 后未进入 damping。

任一出现都停止，不加幅度。

## 6. 后处理与 solver

```bash
uv run armctrl sysid postprocess \
  --dataset runs/ident-sdk-gravity-smoke \
  --solve \
  --json
```

`--solve` 会读取 `manifest.json` 的 solver gate。若 `adapter == "sdk"` 且 `run_status != "completed"`，或 `acceptance.status != "pass"`，CLI 必须返回 `status == "blocked"`、退出码 `3`，只保留 postprocess 复盘产物，不进入 solver。

solver 通过 gate 后，`processed/solver_metrics.json` 必须保留 `source_run` 摘要，`processed/solver_report_zh.md` 也必须显示 `source_run.*` 关键字段。operator 需要确认 `source_run.adapter == "sdk"`、`source_run.run_status == "completed"`、`source_run.acceptance == "sysid_smoke/pass"`，并核对 `actual_send_hz`、`controller_dt_s`、`final_tracking_error_max_abs_rad`、`fault_flags`、`landing_mode`，再讨论任何参数求解结论。

查看产物：

```bash
find runs/ident-sdk-gravity-smoke \
  -name "quality_report.md" \
  -o -name "solver_report_zh.md" \
  -o -name "solver_metrics.json"
```

smoke 阶段主要看：

- 是否有 `raw_samples.csv`；
- 后处理是否完成；
- solver 是否能进入固定阶段；
- `data_health` 是否 pass。

小幅 smoke 不要求辨识结果好，它只验证安全运动和数据链路。

## 7. 逐步加幅度

只有 smoke 稳定，才逐步尝试：

```bash
uv run armctrl sysid run gravity_sweep \
  --adapter sdk \
  --model X5 \
  --interface can0 \
  --dof 6 \
  --sample-hz 100 \
  --duration 12 \
  --amplitude 0.10 \
  --q-center $SAFE_CENTER \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output runs/ident-sdk-gravity-a010 \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" \
  --readiness-artifact "$RUN_DIR/agent_sysid_readiness.json" \
  --json
```

再后处理：

```bash
uv run armctrl sysid postprocess \
  --dataset runs/ident-sdk-gravity-a010 \
  --solve \
  --json
```

不建议直接超过 `0.25 rad`。当前 `x5.safe.yaml` 默认也会拦截更大的幅度。

## 8. friction 与 Fourier

friction 首次：

```bash
uv run armctrl sysid plan friction_sweep \
  --dof 6 --sample-hz 100 --duration 12 --amplitude 0.05 \
  --q-center $SAFE_CENTER \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output runs/plan-friction-smoke \
  --json
```

plan pass 后再 run。Fourier 风险最高，必须最后，并且从 `0.05 rad` 开始。当前 clean rebuild 的轨迹生成仍偏 smoke/handoff，不是成熟 FIGAROH 优化轨迹；要做高质量完整动力学辨识，应先接 FIGAROH 最优轨迹。

## 9. 模拟 Agent 调用

Agent 只能通过受限 CLI：

```bash
uv run armctrl recipe list --json
uv run armctrl recipe plan home --json
uv run armctrl recipe status --json
uv run armctrl recipe execute home --json
uv run armctrl recipe cancel --json
```

当前 `recipe execute` 若无 verified backend 应返回 `rejected`。合格 Agent 行为：

1. 解析 JSON；
2. 看到 `status=rejected` 后停止；
3. 汇报 `safety` / `executor` / `reason`；
4. 不直接调用 `arx5_interface`；
5. 不自造 joint command。

Recipe 侧如果要验证“已审查轨迹是否能被本地 MotionRuntime 按时间戳 replay”，先跑 fake runtime smoke，不要直接找 SDK，也不要传 `--runtime-session-artifact`：

```bash
RECIPE_PLAN_DIR="$RUN_DIR/recipe-home-plan"

uv run armctrl recipe plan home \
  --sample-hz 50 \
  --duration 0.1 \
  --output "$RECIPE_PLAN_DIR" \
  --json

uv run armctrl recipe runtime-smoke-fake \
  --plan-dir "$RECIPE_PLAN_DIR" \
  --output "$RUN_DIR/recipe_runtime_smoke_fake.json" \
  --json
```

`recipe_runtime_smoke_fake.json` 必须满足：

- `schema == "armctrl.recipe_runtime_smoke.v1"`
- `hardware_motion == false`
- `runtime.backend == "fake"`
- `runtime.owner == "motion_runtime"`
- `motion_runtime.producer == "recipe"`
- `motion_runtime.mode == "trajectory_replay"`
- `motion_runtime.actual_send_hz` 被记录
- `motion_runtime.landing_mode == "hold"`

这一步只证明 Recipe 的 checked joint trajectory 可以被本地 fake MotionRuntime 按时间戳 replay 并落到 hold。它不能证明 arx5 SDK、MoveIt Servo、LeRobot 真机 runtime 或 live owner queue 已经可用。

如果要把 Recipe 接到 live runtime，必须走 queued owner submit：

```bash
uv run armctrl recipe runtime-submit \
  --plan-dir "$RECIPE_PLAN_DIR" \
  --runtime-session-artifact "$RUN_DIR/runtime_session.json" \
  --output "$RUN_DIR/recipe_runtime_submit.json" \
  --json
```

`recipe_runtime_submit.json` 必须满足：

- `schema == "armctrl.recipe_runtime_submit.v1"`
- `status == "queued"`
- `movement_command_sent == false`
- `runtime.owner == "recipe"`
- `runtime.mode == "trajectory_replay"`
- `runtime_command.artifacts.command` 指向 live runtime queue command

真正的运动只允许由正在 `--serve` 的 runtime 消费 queue 后发送。

SysID Agent 调用也必须先 plan：

```bash
uv run armctrl sysid plan gravity_sweep \
  --dof 6 --sample-hz 100 --duration 8 --amplitude 0.05 \
  --q-center $SAFE_CENTER \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output runs/agent-plan-gravity-smoke \
  --json
```

只有 plan pass，且人类明确给出确认 token，Agent 才能建议运行 SDK runner。

## 9A. MoveIt Servo helper 边界

如果 Agent/EEF 选择 `moveit_servo`，`armctrl` 仍然只生成 contract、preview、review 和 helper plan，不启动 ROS 2、不发布 Servo command，也不拥有 MoveIt Servo 的实时循环：

```bash
uv run armctrl eef plan-twist \
  --frame eef_link \
  --linear 0.01 0.0 0.0 \
  --angular 0.0 0.0 0.02 \
  --control-period-s 0.1 \
  --backend moveit_servo \
  --output "$RUN_DIR/eef-moveit-smoke" \
  --json

uv run armctrl eef export-runner-contract \
  --plan-dir "$RUN_DIR/eef-moveit-smoke" \
  --output "$RUN_DIR/eef_moveit_runner_contract.json" \
  --json

uv run armctrl eef export-moveit-helper-plan \
  --runner-contract "$RUN_DIR/eef_moveit_runner_contract.json" \
  --output "$RUN_DIR/moveit_helper_plan.json" \
  --json
```

`moveit_helper_plan.json` 必须满足：

- `runtime_boundary.runtime_owner == "ros2_moveit_servo"`
- `runtime_boundary.armctrl_role == "contract_preview_audit_only"`
- `runtime_boundary.motion_runtime_owner == false`
- `frequency_contract.agent_intent_hz == 10.0`，当 `control_period_s=0.1`
- `frequency_contract.command_publish_hz == "runtime_configured"`
- `frequency_contract.actual_send_hz == "measure_in_runtime_artifact"`

这表示 Agent 仍然只表达低频 intent；ROS 2 / MoveIt Servo 的 command publish 频率、servo loop 频率、实际发送频率必须由真正的 MoveIt runtime 配置和日志证明，不能由 armctrl helper plan 伪造。

## 10. 常见故障解释

`unrecognized arguments: --confirm`

- 代码不是最新；
- 先 `git pull`。

`arx5_interface is not importable`

- 不是 Linux 目标环境，或未安装 SDK；
- 在 n100d 执行 `uv sync --extra dev --extra lerobot`。

`planned trajectory did not pass safety checks`

- plan 已经被安全门挡住；
- 查看 `runs/.../manifest.json`；
- 减小幅度或调整安全中心位。

`over current detected`

- 可能撞桌、近限位、姿态力矩过大或底层控制器保护；
- 立即停止；
- 降低幅度，抬高安全空间 z 下界，重新 plan。

## 11. 完整 smoke 验证清单

```bash
ls /dev/ttyACM*
ip link show can0
uv run armctrl sysid sdk-preflight --model X5 --interface can0 --json
uv run armctrl sysid sdk-handshake-plan --model X5 --interface can0 --json
uv run armctrl sim doctor --json
uv run armctrl sysid plan gravity_sweep --dof 6 --sample-hz 100 --duration 8 --amplitude 0.05 --q-center $SAFE_CENTER --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --output runs/plan-gravity-smoke --json
uv run armctrl sysid run gravity_sweep --adapter sdk --model X5 --interface can0 --dof 6 --sample-hz 100 --duration 8 --amplitude 0.05 --q-center $SAFE_CENTER --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --output runs/ident-sdk-gravity-smoke --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" --readiness-artifact "$RUN_DIR/agent_sysid_readiness.json" --json
uv run armctrl sysid postprocess --dataset runs/ident-sdk-gravity-smoke --solve --json
```
