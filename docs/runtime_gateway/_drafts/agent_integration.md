# Agent 集成规范

## 定位

Agent 当前没有像 SysID 一样的真机 golden path。因此阶段 1 不能把现有 Agent smoke
误当作正式控制入口保护，也不能把 Agent 集成扩张成自研控制器。

Agent 的定位是低频 action / intent source：

```text
Agent decision
  -> Agent action
  -> ArmCommand
  -> runtime gateway
  -> mature backend or runtime joint executor
```

Agent 不应直接承担 SDK send frequency、电机内环频率或 SysID trajectory sample
frequency。

## 阶段 1 支持范围

阶段 1 只支持：

- `joint_intent` fake 验证。
- `joint_intent` runtime submit。
- `joint_trajectory` fake 验证。
- `joint_trajectory` runtime submit，作为未来 Agent joint waypoints 的基础。
- `ArmCommand` contract simulation。

阶段 1 不支持：

- EEF 真机执行。
- LeRobot rollout 真机执行。
- Teleop / Xbox streaming。
- Agent 直接输出 100Hz 或 SDK controller dt 的电机控制。
- 把 `agent-flow runtime-smoke-real` 扩成正式万能控制入口。

## `agent-flow runtime-smoke-real`

当前 `agent-flow runtime-smoke-real` 只能作为 temporary smoke：

```text
Agent submit
  -> live runtime readiness
  -> owner=agent
  -> mode=agent_servo
  -> bounded joint q_target intent
  -> release back to hold
```

它验证的是 Agent 能通过 runtime owner path 接管并释放，不验证完整 EEF 控制。

未来正式入口应基于 `ArmCommand`，而不是继续把 smoke 命令扩张成万能控制入口。

## EEF Agent action

Agent 可以表达 EEF action，但阶段 1 只进入 contract/review：

- `eef_pose_delta`。
- `eef_pose_waypoints`。
- `eef_twist`。

没有成熟 EEF backend adapter 时，必须返回不可真机执行。禁止使用 heuristic preview
作为真机执行依据。

## 频率分层

Agent 频率应明确分层：

- Agent decision/tool frequency：不可靠，不进入实时控制。
- Agent intent frequency：默认 10Hz 级别。
- Runtime send frequency：由 runtime 或成熟 controller 统一管理，例如 50Hz/100Hz。
- SDK controller dt：底层参考，不由 Agent 直接控制。

Agent 给“想去哪/怎么动”的 action，runtime 或成熟后端负责插值、限幅、连续发送、
watchdog 和 landing。
