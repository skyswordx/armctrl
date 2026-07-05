# EEF Backend 边界规范

## 原则

EEF command kinds 写入 `ArmCommand` contract 是为了统一接口，不是为了让
`armctrl` 自研 EEF controller。

EEF 真实执行必须通过成熟 backend boundary：

- MoveIt Servo。
- ARX5 SDK Cartesian controller。
- reviewed IK-to-joint-trajectory adapter。

## `eef_twist` / `eef_pose_delta` / `eef_pose_waypoints`

首选 backend：MoveIt Servo。

可选 backend：ARX5 SDK Cartesian controller，但必须先证明：

- API 与 SDK 版本匹配。
- command frequency、hold、fault landing 行为清楚。
- 不会绕过 runtime SDK/CAN ownership。
- 能产出 tracking/timing/fault artifact。
- 安全 gate 能覆盖 workspace、joint limit、collision、velocity、acceleration。

离线 preview/review 可以使用：

- Pink/Pinocchio。
- Pinocchio/coal。
- MoveIt state validity / collision。
- MuJoCo。

Heuristic fallback 只允许作为 degraded preview 或开发提示，绝不能作为真机可执行依据。

## 第一阶段 EEF scope

阶段 1 只做：

- 解析和校验 EEF command contract。
- 解析 live runtime readiness 和 expected start state。
- 执行静态安全检查。
- 调用成熟后端做 preview/review，如果环境可用。
- 如果没有配置成熟 EEF backend adapter，则明确返回不可真机执行。

阶段 1 不做：

- 实现新的 IK solver。
- 实现新的 Cartesian servo loop。
- 绕过成熟 backend adapter，直接向 SDK/CAN 发送 EEF command。
- MoveIt Servo 真机集成。
- ARX5 SDK Cartesian 真机集成。

## 后端选择标准

进入真机执行前，EEF backend 必须满足：

- 能在 live runtime owner lease 下执行。
- 不直接由 source 打开 SDK/CAN。
- 有明确频率契约。
- 有 hold/fault/timeout landing 行为。
- 有 safety gate 输入与输出 artifact。
- 有 q_cmd/q_meas 或 EEF command/feedback 的 tracking evidence。
