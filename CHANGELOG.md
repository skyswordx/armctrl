# Changelog

本文记录 `armctrl` 的用户可见变更。

## [Unreleased]

### Added

- 新建 clean rebuild 分支入口，只保留 roadmap、changelog、文档索引、vendor 和 uv 配置作为重构基线。
- 明确 `armctrl` 与 ARX5 SDK、Pinocchio、FIGAROH、LeRobot ARX5 集成的职责边界。
- 新增第一版 plan-only Agent recipe catalog 和 JSON CLI 预览入口。
- 新增结构化 recipe execute 拒绝语义，防止在没有后端和安全合同前误触硬件执行。
- 新增 plan-only SysID planner，输出 gravity/friction/Fourier profile 与 Pinocchio/FIGAROH/LeRobot handoff 合同。
- 新增 `sysid plan --output` 离线产物生成，写出 `planned_trajectory.csv` 和 `manifest.json`。
- 新增轻量 URDF joint limit 检查，`sysid plan --output` 会在 stdout 和 manifest 中记录 pass/fail。
- 新增 workspace/table clearance 的第一版保守 proxy 检查，读取 `configs/x5.safe.yaml` 并在 stdout/manifest 中记录 pass/fail。
- 新增 `sysid run --adapter fake`，可生成 `raw_samples.csv` 和 run manifest，用于恢复无硬件数据链路。
- 新增 `sysid postprocess`，从 fake/raw dataset 生成 `processed_samples.csv`、`quality_metrics.json` 和 `quality_report.md`。
- 新增 `sysid solve`，从 processed dataset 生成 `solver_metrics.json` 和 `solver_report_zh.md`，并记录 Pinocchio/FIGAROH 可用性。
- 新增 `sysid postprocess --solve`，让后处理完成后可以直接运行固定 solver 阶段并返回 solver 产物。
- 新增可选 Pinocchio regressor rank/condition 指标；无 Pinocchio 环境时保持结构化降级。
- 新增可选 Pinocchio least-squares prediction RMSE 指标，用于验证 `Y*pi -> tau_pred -> residual` 链路。
- 新增 `sysid package` 质量门；缺少 Pinocchio、FIGAROH/base-parameter 或物理一致性证据时拒绝生成候选参数包。
- 新增 `sysid import-evidence`，把 FIGAROH/外部物理一致性和基础参数证据导入 solver metrics，供参数包质量门消费。
- 新增 `sysid adapt-figaroh-evidence`，把 FIGAROH-style report 规范化为 `armctrl.external_solver_evidence.v1`。
- 新增 `online-id audit`，以 append-only JSONL 记录 shadow 在线辨识更新窗口、残差变化、饱和检查、回滚目标和人工确认要求。
- 新增 `recipe status` / `recipe cancel` 和显式 recipe command executor gate，Agent skill 只能通过受限 recipe CLI 触达动作。
- 新增 `release status`，机器可读标注 `0.5.0` contracts complete / hardware pending 状态。
- 新增 `sysid sdk-preflight`，只读检查 `arx5_interface` 导入状态和目标 model/interface，不打开 CAN、不移动硬件。
- 新增 `sysid sdk-handshake-plan`，只读固定未来 SDK 采集前的确认、hold/damping、记录时序和 Ctrl-C/fault 落态契约。

## [0.5.0] - Contract Complete, Hardware Pending

### Status

- Clean rebuild contracts through v0.5.0 are implemented and tested.
- Hardware SDK runner, full FK/table collision, and n100d Pinocchio/FIGAROH validation remain pending.
- 新增在线辨识 shadow-mode policy，限制在线更新只覆盖 torque bias、摩擦项和小幅 gravity residual。
- 新增项目内 Codex skill: `.codex/skills/armctrl-agent-recipes/SKILL.md`，限制 Agent 只能通过 recipe CLI 查看和规划动作。

### Changed

- 停止在旧实验实现上继续叠加功能，后续按 `v0.2.0` 到 `v0.5.0` milestone 重新实现。
- SysID execute 仍保持 rejected；workspace clearance 目前是 joint2 proxy，不是完整 FK/table collision model。
- `sysid run --adapter sdk` 仍保持 rejected，等待真实 SDK runner、安全落态和真机验证。
- 当前 postprocess 覆盖基础文件合同和 data health；预测误差、物理一致性等进入固定 solver stage 后继续补齐。

## [0.1.0] - Historical Baseline

### Added

- 历史 `develop`/`feature/subsystem` 分支包含初始 SDK adapter、安全检查、teleop、SysID 轨迹生成、采集、后处理和 solver 报告实验。

### Known Gaps

- 历史实现边界不清，容易重复 LeRobot、FIGAROH 和 ARX5 SDK 的成熟能力。
- 历史文档较多，当前工作以 `ROADMAP.md` 为主线。
