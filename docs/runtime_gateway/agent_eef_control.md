# Agent EEF 控制收束规范

## 目标判断

Agent EEF 的正式目标不是把 EEF 命令先离线转成一整段 `joint_trajectory` 再重放，也不是让一个新的 CLI 进程关闭 joint runtime 后重新打开 `sdk_cartesian`。正式目标是：

```text
ArmRuntime 长驻持有 SDK/CAN
  -> joint_hold_controller 保底持续 hold
  -> joint_trajectory_controller 用于 SysID / Recipe / preposition
  -> joint_intent_controller 用于 Agent joint 低频 intent
  -> eef_servo_controller 用于 Agent/Teleop EEF twist/delta/pose servo
  -> controller switch 只能在 runtime 内部 warmup + zero command + target=current FK 后发生
```

因此，`Agent` 是控制源，`EEF` 是 command space。Agent 可以发 joint intent，也可以发 EEF delta/twist/pose；但真实 SDK/CAN owner 只能是长驻 runtime。

## 成熟实践映射

- MoveIt Servo 风格：EEF 实时控制应接收 `Twist` / `Pose` / `JointJog` 这类输入，由 servo loop 每 tick 基于当前 robot state 计算小步 joint delta 或 velocity，并处理限速、限位、奇异和碰撞风险。
- ros2_control 风格：控制器切换必须由长驻 controller manager/runtime 管理。硬件 interface 不应在两个 CLI 进程之间释放和重开。
- Cartesian impedance / reference limiting 风格：Agent/Teleop 给出的 EEF target 不应离当前 pose 太远。runtime 每帧只允许小步 reference 更新，避免误差和接触风险放大。

这些实践给 armctrl 的结论是：我们不复制 `feature/subsystem`，只吸收它证明过的现象事实，即连续 owner + 人类闭环 EEF 输入可以很自然。实现边界要对齐成熟 servo/controller-manager 模式。

## 两条 EEF 路径

### EEF Servo Path

```text
Agent/Teleop -> eef_twist / eef_delta / eef_pose
  -> runtime 读取 fresh q_meas / FK / Jacobian 或成熟 backend state
  -> 每 tick 计算小步 joint delta / velocity / Cartesian reference
  -> 限幅、限速、关节限位、workspace、奇异、碰撞 gate
  -> 同一个 SDK owner 连续发送
  -> stale/deadman -> hold 或 damping
```

这是 Agent EEF 想要的“像遥控器一样丝滑”的路径。它要求 backend 在长驻 runtime 内执行，不能通过 `joint runtime stop -> sdk_cartesian runtime start` 接力实现。

当前 runtime 已把 reference limiting 作为 EEF Servo Path 的硬合同：`eef_delta` / `eef_twist` 默认每 tick 最多推进 `0.005 m` 线位移和 `0.05 rad` 角位移。超过限制的 command 在 submit 阶段拒绝；执行阶段还会重新计算一次，防止 pending queue JSON 被手工篡改后绕过 safety gate。策略是 reject，不做静默 clamp。

当前实现已经补上 runtime 内部 adapter manager seam：`execute_pending_runtime_commands` 可以接收 primary runtime backend 与 `eef_backends` registry。primary backend 仍负责 live readiness、owner lease、hold/release、watchdog 和 SDK/CAN singleton；EEF command 只按 command 中声明的 mature adapter 名称分发给 `moveit_servo` 或 `sdk_cartesian` 这类 adapter。artifact 必须同时记录 `runtime_backend` 与 `eef_adapter`，用于审计是否仍是同一个 runtime 在持有运动所有权。

如果 EEF command 声明了 adapter，但长驻 runtime status 没有证明对应 adapter 已配置且该 command kind 在 `eef_command_capabilities` 中可执行，submit 端必须 `blocked`，并且不能写入 pending queue。`eef_command_executable=true` 只表示“至少一种 EEF command 可走 adapter”，不能代表 `eef_pose_delta`、`eef_twist`、`eef_pose` 全部可真机执行。尤其 `eef_pose` 必须要求 adapter 具备 live reference/FK/SDK EEF state，不能因为 fake publisher 或 adapter 名称存在就放行。serve 端仍保留二次 rejected 防线，允许的失败形态是“缺少 mature backend adapter 或缺少该 command kind 的 capability”；不允许退化为 heuristic joint fallback，也不允许为了执行 EEF command 自动关闭/重开 SDK/CAN。

### EEF Planned Path

```text
Recipe/低频目标 -> eef_pose waypoint
  -> IK / planner / collision review
  -> checked joint_trajectory
  -> runtime trajectory_replay
```

这是非实时、可审查、可复现的路径，适合 Recipe、低频固定 pose、批处理预览。它不能替代 EEF Servo Path。

## 禁止的切换方式

禁止把下列方式作为正式架构：

```text
arx5_sdk joint runtime hold
  -> stop runtime / release SDK owner / enter damping or passive
  -> open sdk_cartesian runtime
  -> hope cartesian backend catches the arm
```

这会制造 hold 空窗。机械臂在切换瞬间可能下落，之前实验室里“到 SAFE_CENTER 后切 EEF 立刻掉下来”的现象就是这个风险。

## 正式切换策略

EEF servo backend 进入 active owner 前必须满足：

```text
HOLD_SAFE_JOINT or HOLD_EEF_READY
  -> preposition to SAFE_CENTER or AGENT_EEF_HOME
  -> joint_hold remains active
  -> initialize EEF backend with target=current FK pose
  -> run zero-command warmup for N ticks
  -> verify q_meas / fault_flags / jitter / tracking
  -> switch owner to agent_eef only after checks pass
```

失败策略：

```text
EEF warmup failed -> stay in joint_hold
EEF command stale -> hold current pose
deadman timeout -> hold or damping according to policy
backend fault -> fallback joint_hold or damping
```

## 当前 CLI 语义

正式入口：

```bash
armctrl console catalog --json
armctrl console status --session-artifact "$RUN_DIR/runtime_session.json" --json
armctrl motion submit joint-intent ...
armctrl motion submit joint-trajectory ...
armctrl motion submit eef-delta ...
armctrl motion submit eef-twist ...
armctrl motion submit eef-pose ...   # contract exists, hardware execution rejected until mature backend
armctrl motion submit joint-jog ...  # contract exists, hardware execution rejected until deadman backend
```

当前 `eef-delta` / `eef-twist` submit 只证明 Agent EEF command 能进入同一个 owner/readiness/queue/artifact 表面。真机执行必须由 mature backend 在同一长驻 runtime 内消费。

runtime 执行 EEF command 前必须产出并通过 `armctrl.eef_servo_switch.v1` warmup gate：

```json
{
  "status": "pass",
  "policy": "continuous_owner_bumpless_switch",
  "target_seed": "current_fk_pose",
  "disconnected_takeover_allowed": false,
  "sdk_owner_released": false,
  "checks": {
    "target_seeded_from_current_state": true,
    "zero_command_warmup_completed": true,
    "live_q_read_before_switch": true,
    "live_q_read_after_switch": true
  }
}
```

如果 backend 在 warmup 中释放 SDK owner、没有 fresh state、没有完成 zero-command warmup，runtime 必须拒绝该 EEF command，且不能调用 backend 的 `execute_eef_command`。

`scripts/lab_agent_runtime_smoke.sh start-eef` 已降级为 diagnostic-only。它代表旧的 disconnected `sdk_cartesian` takeover，对比实验时必须显式设置：

```bash
ARMCTRL_ALLOW_DISCONNECTED_EEF_TAKEOVER=1 scripts/lab_agent_runtime_smoke.sh start-eef
```

不要把它当作正式 Agent EEF 验收路径。

无硬件预演入口使用 fake runtime 加 MoveIt Servo-style adapter：

```bash
ARMCTRL_BACKEND=fake scripts/lab_agent_runtime_smoke.sh init
ARMCTRL_BACKEND=fake scripts/lab_agent_runtime_smoke.sh start
scripts/lab_agent_runtime_smoke.sh run-eef
scripts/lab_agent_runtime_smoke.sh check-eef
```

该路径会让 `runtime start --backend fake --serve` 自动注册 `--eef-adapter moveit_servo`。
它只能证明 EEF command surface、owner lease、adapter registry、result artifact 和 release 语义；
不能证明真实 ARX5 EEF 运动质量，也不能作为真机 EEF backend 已成熟的证据。

## 等待真机验证前的完成标准

- `console catalog` 明确 Agent EEF 需要 continuous-owner servo 或 reviewed planned path。
- EEF command artifact 明确记录 `disconnected_takeover_allowed=false`、`bumpless_switch_required=true`、`no_heuristic_joint_fallback=true`。
- EEF command artifact 明确记录 `eef_reference_limit`，并证明 submit 与 execute 都执行同一套 per-tick reference limit。
- EEF runtime result artifact 必须记录 `eef_switch`：warmup 是否通过、backend 是否仍持有 SDK owner、是否从 fresh state 初始化 target。
- EEF runtime result artifact 必须记录 `eef_controller_manager.start_pose_guard`、`start_q_reference`、`safe_center_required` 和 `safe_center_reference`，避免把 passive/droop、live hold、SAFE_CENTER 和 current measured takeover 混成同一种启动姿态。
- 现场脚本默认不再启动 disconnected `sdk_cartesian` takeover。
- `run-eef` 默认使用 `start_pose_policy=live_hold`，从 runtime 正在持续 hold 的受控姿态发出 EEF command。
- 无硬件测试覆盖 catalog、submit artifact、fake `moveit_servo` adapter 消费、`eef_switch` gate、脚本文案和 Bash parse。

## 真机验证边界

在进入正式 Agent EEF 真机验收前，本地只能证明 gateway contract 和 fake/runtime queue 语义；不能证明真实 EEF 运动质量。真机阶段需要验证：

- Agent joint intent / joint trajectory 仍走 `arx5_sdk` long-lived runtime。
- Agent EEF command 从 live hold 发起时，必须能在 result artifact 中看到 `start_q_reference=q_hold` 且 `safe_center_required=true`；如果使用 `current_measured_pose`，则必须明确 `safe_center_required=false`，只能作为成熟 Cartesian adapter 的 current-pose takeover。
- EEF command 通过 `eef_switch.status=pass`，且 backend 没有释放 SDK owner。
- EEF delta/twist/pose 的实际运动是连续、小步、可停止的，stale/deadman 后回 hold 或 damping。
- `start-eef` 只能作为手动诊断入口，不能作为正式通过标准。
