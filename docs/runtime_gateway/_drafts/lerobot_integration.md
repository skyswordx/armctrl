# LeRobot 集成规范

## 定位

LeRobot 负责 dataset、policy、rollout、action chunk 和 processor 生态。`armctrl`
负责 SDK/CAN 单 owner、runtime readiness、安全 gate 和 artifact。

LeRobot 不应作为绕过 runtime 的硬件执行入口。

## 推荐集成路径

推荐路径：

```text
LeRobot policy rollout
  -> robot_action_processor
  -> ArmCommand
  -> live runtime readiness
  -> owner lease
  -> mature backend adapter
  -> robot
```

不推荐路径：

```text
LeRobot native rollout
  -> open ARX5 SDK/CAN directly
  -> robot
```

即使 native LeRobot rollout 本身可以跑硬件，在 integrated armctrl control 目标下也不
允许绕过 runtime ownership 直接抢 SDK/CAN。

## 阶段 1 范围

阶段 1 不实现 LeRobot 真机集成。

阶段 1 只保留：

- LeRobot processor/action contract 语义分析。
- LeRobot 到 `ArmCommand` 的接口设计。
- 不允许 native rollout 被误认为 armctrl runtime path。

## 阶段 2 候选范围

- 实现 `robot_action_processor -> ArmCommand`。
- 明确 LeRobot action space：joint、EEF、delta、waypoints。
- 让 LeRobot policy action 通过 runtime owner path 执行。
- 保留 LeRobot rollout/action chunk 的成熟机制，不在 `armctrl` 内重写 rollout。

## Artifact 要求

LeRobot 集成进入真实运动前，必须记录：

- LeRobot policy/action metadata。
- processor 输入输出。
- 生成的 `ArmCommand`。
- runtime owner/result artifact。
- action frequency 与 runtime send frequency 的关系。
- missed action / stale action / timeout landing 行为。
