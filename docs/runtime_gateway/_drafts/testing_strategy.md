# Runtime Gateway 测试策略

## 原则

本次重构采用最小但高信号的测试。测试保护 runtime 架构不变量和 golden path
行为，不覆盖每个 CLI 拼写、每个参数组合或每个 lab script 分支。

## 必须测试

- `ArmCommand` schema 解析和 normalization。
- Runtime owner 互斥。
- Live readiness guard：command 不能用历史 artifact 证明机械臂当前仍被 hold。
- `joint_trajectory` 通过 fake backend 执行。
- SysID candidate replay equivalence smoke，保护 golden path。
- Fault、stale owner heartbeat、timeout landing 行为。

## 不要求测试

- 每个 CLI 参数组合。
- 每个 shell script 分支。
- 纯 cosmetic artifact 字段。
- Bringup diagnostic one-off command。
- 纯文档/示例，除非它定义了稳定 operator contract。

## Lab script

Lab script 是 operator workflow，不是架构本体。必要时可以做语法检查或 smoke test，
但不应占据主要实现时间。

## 验收证据

阶段 1 的测试和验证必须能证明：

- SysID golden path 未被破坏。
- `joint_trajectory` 已经成为通用 owner path。
- Agent 阶段 1 入口不会绕过 runtime。
- EEF 没有成熟 backend adapter 时不会误动真机。
- Legacy SDK direct path 不再作为正式运动入口。
