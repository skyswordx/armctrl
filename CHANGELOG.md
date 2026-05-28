# Changelog

本文件记录 `armctrl` 的用户可见变更。

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
- 新增在线辨识 shadow-mode policy，限制在线更新只覆盖 torque bias、摩擦项和小幅 gravity residual。
- 新增项目内 Codex skill：`.codex/skills/armctrl-agent-recipes/SKILL.md`，限制 Agent 只能通过 recipe CLI 查看和规划动作。

### Changed

- 停止在旧实验实现上继续叠加功能，后续按 `v0.2.0` 到 `v0.5.0` 小 milestone 重新实现。
- SysID execute 仍保持 rejected；workspace clearance 目前是 joint2 proxy，不是完整 FK/table collision model。
- `sysid run --adapter sdk` 仍保持 rejected，等待真实 SDK runner、安全落态和真机验证。

## [0.1.0] - Historical Baseline

### Added

- 历史 `develop`/`feature/subsystem` 分支包含初始 SDK adapter、安全检查、teleop、SysID 轨迹生成、采集、后处理和 solver 报告实验。

### Known Gaps

- 历史实现边界不清，容易重复 LeRobot、FIGAROH 和 ARX5 SDK 的成熟能力。
- 历史文档较多，当前工作以 `ROADMAP.md` 为主线。
