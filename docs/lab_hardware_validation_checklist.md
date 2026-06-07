# 实验室真机验证 Checklist

本清单用于整机实验室现场执行最小真机门控：

`doctor -> hold/damping -> arm session -> startup recovery -> readiness -> Agent smoke -> SysID smoke`

它不是完整 operator manual；细节以 `docs/hardware_sysid_operator_manual.md` 为准。

任何一步返回非零、`status != "ok"`、`fault_flags` 非空、`acceptance.status != "pass"`，或操作员感觉姿态、声音、电流异常，都必须停止。不要通过加大幅度或反复重试来“撞过去”。

## 0. 现场准备

- 机器人底座固定，工作空间清空，急停/断电手段在手边。
- 当前关节姿态必须由真实 SDK 读取，不要凭肉眼或印象填写 `--q-current`。
- `SAFE_CENTER` 是受控启动姿态目标，不是当前姿态。
- 如果机械臂处于无力下垂/搭下来的静止安全姿态，正式恢复流程是 `sdk-recover-startup-real`。

```bash
export RUN_DIR=runs/lab-hardware-$(date +%Y%m%d-%H%M%S)
mkdir -p "$RUN_DIR"
export SAFE_CENTER="0.0 0.3 0.3 0.0 0.0 0.0"
```

## 1. SDK Doctor

只读，不允许运动：

```bash
uv run armctrl sysid sdk-doctor \
  --model X5 \
  --interface can0 \
  --state-sample-count 20 \
  --state-sample-period 0.01 \
  --output "$RUN_DIR/sdk_doctor.json" \
  --json
```

预期现象：

- 终端会看到 SDK 加载动态库、绑定 `can0`、解析 URDF/KDL 的日志。
- 机械臂不应发生任何可见运动。
- artifact 中必须是 `movement_allowed == false` 且 `motion_commands_sent == false`。
- 正常通过时顶层 `status == "ok"`，`doctor_gate.status == "pass"`。

可选：保存 doctor 读到的当前姿态，供诊断命令使用。

```bash
export MEASURED_Q_CURRENT="$(python3 - <<'PY'
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

## 2. Hold/Damping Gate

只允许模式切换，不允许 joint command：

```bash
uv run armctrl sysid sdk-hold-damping-check \
  --model X5 \
  --interface can0 \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --confirm "I UNDERSTAND THIS WILL CHANGE THE ARM CONTROL MODE" \
  --output "$RUN_DIR/hold_damping.json" \
  --json
```

必须确认：

- `status == "ok"`
- `movement_allowed == false`
- `joint_commands_sent == false`
- `landing_policy` 为 `hold_then_damping` 或 `damping_only`
- `hold_damping_gate.status == "pass"`
- 如果现场 SDK 没有 `set_to_hold`，`hold.status == "unsupported"` 且 `damping.status == "called"` 是允许的 `damping_only` 合同，不是 CAN/电源故障。

## 3. Arm Session

读取实测姿态，不发送 joint command：

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

预期现象：

- 机械臂不应出现轨迹运动。
- `state.q_meas` 是现场实测姿态；它可以和 `SAFE_CENTER` 差很多。
- 正常通过时 `session_status == "armed"`。

## 4. Startup Recovery

从实测姿态恢复到启动姿态。默认命令会到位后一直主动保持，适合现场观察稳定性；你需要按 `Ctrl-C` 才会退出保持。

```bash
uv run armctrl sysid sdk-recover-startup-real \
  --session-artifact "$RUN_DIR/session.json" \
  --q-target $SAFE_CENTER \
  --safe-config configs/x5.safe.yaml \
  --send-hz 50 \
  --hold-hz 50 \
  --confirm "I UNDERSTAND THIS WILL RECOVER THE REAL ARM TO STARTUP POSE" \
  --output "$RUN_DIR/startup_recovery_hold.json" \
  --json
```

预期现象：

- 机械臂从当前实测姿态缓慢、连续地进入 `SAFE_CENTER`，不是突然跳到目标位姿。
- 到达后持续出力保持；不应无力下坠。
- 按 `Ctrl-C` 后会尝试显式进入 damping。
- 如果出现异响、下坠、碰撞风险或 fault，立即急停/断电并检查。

如果你要继续执行后续 gate 并需要 artifact，使用有限保持版本；这会保持 10 秒后正常返回并写出 `startup_recovery.json`：

```bash
uv run armctrl sysid sdk-recover-startup-real \
  --session-artifact "$RUN_DIR/session.json" \
  --q-target $SAFE_CENTER \
  --safe-config configs/x5.safe.yaml \
  --send-hz 50 \
  --hold-hz 50 \
  --hold-seconds 10 \
  --confirm "I UNDERSTAND THIS WILL RECOVER THE REAL ARM TO STARTUP POSE" \
  --output "$RUN_DIR/startup_recovery.json" \
  --json
```

必须确认：

- `status == "ok"`
- `run_status == "completed"`
- `movement_command_sent == true`
- `motion_runtime.status == "completed"`
- `motion_runtime.actual_send_hz` 和 jitter 有记录
- `active_hold.requested == true`
- 有限保持版本中 `active_hold.command_count > 0`

## 5. Readiness

readiness 直接接受 startup recovery artifact；不要把额外诊断步骤混入主线 gate。

```bash
uv run armctrl sysid sdk-agent-sysid-smoke-readiness \
  --doctor-artifact "$RUN_DIR/sdk_doctor.json" \
  --hold-damping-artifact "$RUN_DIR/hold_damping.json" \
  --startup-recovery-artifact "$RUN_DIR/startup_recovery.json" \
  --output "$RUN_DIR/agent_sysid_readiness.json" \
  --json
```

预期现象：

- 只读 artifact，不连接硬件，不应有任何机械臂动作。
- 正常通过时 `status == "ok"` 且 `agent_sysid_smoke_allowed == true`。

## 6. Agent Real Smoke

先生成 Agent contract；这一步不连接硬件：

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

执行真实 Agent single-frame smoke：

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

必须确认：

- `status == "ok"`
- `run_status == "completed"`
- `frequency_contract.agent_intent_hz == 10.0`
- `frequency_contract.backend_send_hz == 50.0`
- `motion_runtime.mode == "agent_servo"`
- `motion_runtime.fault_flags == []`
- `acceptance.status == "pass"`

## 7. SysID Smoke

只有 Agent smoke 通过后，才运行短 SysID smoke。首轮不要扩大幅度：

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
  --readiness-artifact "$RUN_DIR/agent_sysid_readiness.json" \
  --json
```

必须确认 `runs/.../manifest.json` 中：

- `run_status == "completed"`
- `readiness.agent_sysid_smoke_allowed == true`
- `motion_runtime.mode == "trajectory_replay"`
- `motion_runtime.trajectory_sample_hz == 100.0`
- `motion_runtime.actual_send_hz`、jitter、`controller_dt_s` 有记录
- `fault_flags` 为空
- `acceptance.status == "pass"`

只有这些都通过后，才讨论更大幅度 SysID、postprocess 或 solver。

## Appendix. Optional Tiny Motion Diagnostic

tiny motion 只用于首次 SDK bringup 诊断，不是主线 gate。不要用它把下垂姿态恢复到启动姿态。

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
  --confirm "I UNDERSTAND THIS WILL EXECUTE THE TINY MOTION PLAN" \
  --output "$RUN_DIR/tiny_motion_real.json" \
  --json
```

预期现象：

- 只允许一个关节发生极小变化，首轮约 `0.002 rad`。
- 不应出现突然下坠、连续摆动、撞限位、异响、电流尖峰或 unresolved fault。
- 如果顶层 `status` 是 `faulted`、`aborted`、`incomplete`、`review_required` 或 `rejected`，立即停止。
