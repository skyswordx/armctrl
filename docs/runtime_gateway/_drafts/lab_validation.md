# Lab 真机验证规范

## 目标

实验室验证用于证明 runtime gateway 重构没有破坏今天已经跑通的 SysID golden path，
并逐步验证新的 `ArmCommand` 入口。

## SysID golden path 复现

必须保留并可复现：

- slow Fourier candidate。
- reduced Fourier candidate。
- full Fourier candidate。
- runtime start/status/hold。
- `armctrl sysid run ... --adapter sdk ...` operator-facing 入口。
- `scripts/lab_fourier_sysid.sh` operator workflow。
- `runtime result-check`。

同一 reviewed candidate trajectory 在重构前后应能比较：

- `sample_count`。
- `trajectory_sample_hz`。
- `runtime_send_hz`。
- `actual_send_hz`。
- send jitter。
- tracking error。
- `q_cmd` / `q_meas`。
- `dq_cmd` / `dq_meas`。
- `tau_meas`。
- fault flags。
- landing mode。

## 通过标准

阶段 1 重构不算通过，除非：

- 新 `joint_trajectory` owner path 可以复现今天 SysID golden path。
- 机械臂从 live `HOLD_SAFE` 出发。
- 执行后回 active hold。
- fault/Ctrl-C/timeout landing 可预测。
- Artifact 足以审计频率、tracking、fault 和数据质量。

## 禁止事项

- 为了文档或架构整洁删除今天能跑的 SysID lab interface。
- 在没有同等替代入口并完成回归前，大幅改名或改变 lab operator 命令。
- 用 Agent/EEF 重构污染 SysID 离线 OED/轨迹格式。
- 把 native LeRobot rollout 或 legacy SDK direct path 当作 integrated armctrl lab path。
