# Changelog

本文记录 `armctrl` 的用户可见变更。

## [Unreleased]

### Added

- 新增 `armctrl lerobot doctor`，只读检查 LeRobot、ARX5 LeRobot 插件和 `arx5_interface` 导入状态，不打开 CAN、不连接硬件。
- 新增 `armctrl lerobot config-plan record/train/rollout`，把采集、训练、推理收敛为原生 LeRobot CLI 命令计划，`armctrl` 只输出 JSON 合同且不执行。
- 新增 `armctrl lerobot export-metadata`，写出 LeRobot 数据集、SysID 参数包和安全配置之间的 metadata bridge。
- 将 release 状态推进为 `0.6.0-rc.1`，标注 LeRobot 非硬件合同完成，真实 record/rollout 仍等待硬件验证。

## [0.6.0-rc.1] - LeRobot Planning Bridge

### Status

- `armctrl` 现在能为 LeRobot record/train/rollout 生成可审查命令计划，但不重写 LeRobot Robot/Teleoperator，也不在该层直接执行硬件动作。
- `armctrl lerobot doctor` 是只读环境检查；目标 Linux 主机上的插件安装/导入验证和真实 record/rollout 仍列为 deferred validation。
- `armctrl lerobot export-metadata` 提供数据集到 SysID 参数包、安全配置的桥接文件，用于后续训练、推理与参数版本追溯。

### Added

- 新增 `armctrl lerobot doctor`、`config-plan` 和 `export-metadata` 三个 CLI 合同。
- 新增 LeRobot CLI 合同测试，覆盖 doctor、record/train/rollout 计划和 metadata bridge。

### Changed

- `ROADMAP.md` 的 v0.6.0 从抽象 LeRobot 集成改为可执行的分阶段 CLI 合同：先 doctor/config-plan/metadata，再目标 Linux 非硬件验证，最后真实硬件 record/rollout。
- `release status` / `release notes` 升级到 `0.6.0-rc.1`，并把 LeRobot 真实硬件 record/rollout 标为 deferred。

## [0.5.0] - Contract Complete, Hardware Pending

### Added

- 新建 clean rebuild 分支入口，只保留 roadmap、changelog、文档索引、vendor 和 uv 配置作为重构基线。
- 明确 `armctrl` 与 ARX5 SDK、Pinocchio、FIGAROH、LeRobot ARX5 集成的职责边界。
- 新增第一版 plan-only Agent recipe catalog 和 JSON CLI 预览入口。
- 新增 recipe plan 的机器可读 dry-run、movement_allowed 和 risk_explanation 字段。
- 新增结构化 recipe execute 拒绝语义，防止在没有后端和安全合同前误触硬件执行。
- 新增 plan-only SysID planner，输出 gravity/friction/Fourier profile 与 Pinocchio/FIGAROH/LeRobot handoff 合同。
- 新增 `sysid plan --output` 离线产物生成，写出 `planned_trajectory.csv` 和 `manifest.json`。
- 新增轻量 URDF joint limit 检查，`sysid plan --output` 会在 stdout 和 manifest 中记录 pass/fail。
- 新增 workspace/table clearance 的第一版保守 proxy 检查，读取 `configs/x5.safe.yaml` 并在 stdout/manifest 中记录 pass/fail。
- 新增 URDF frame-level FK/table clearance 检查，替换旧 joint2 proxy，并在 manifest 中记录 `urdf_fk_frame_clearance`。
- 新增 `sysid run --adapter fake`，可生成 `raw_samples.csv` 和 run manifest，用于恢复无硬件数据链路。
- 新增可注入 SDK SysID runner 骨架，测试覆盖 hold/damping 后开始记录以及完成/故障落 damping；公共 CLI 仍默认拒绝真硬件。
- 新增 `sysid postprocess`，从 fake/raw dataset 生成 `processed_samples.csv`、`quality_metrics.json` 和 `quality_report.md`。
- 新增 `sysid solve`，从 processed dataset 生成 `solver_metrics.json` 和 `solver_report_zh.md`，并记录 Pinocchio/FIGAROH 可用性。
- 新增 `sysid postprocess --solve`，让后处理完成后可以直接运行固定 solver 阶段并返回 solver 产物。
- 新增可选 Pinocchio regressor rank/condition 指标；无 Pinocchio 环境时保持结构化降级。
- 新增可选 Pinocchio least-squares prediction RMSE 指标，用于验证 `Y*pi -> tau_pred -> residual` 链路。
- 新增 solver 固定输出物理一致性和 FIGAROH 基础参数 gate 字段；未导入外部证据前为 `not_evaluated`。
- 新增 `sysid package` 质量门；缺少 Pinocchio、FIGAROH/base-parameter 或物理一致性证据时拒绝生成候选参数包。
- 新增候选参数包版本、SHA-256 签名、回滚目标和上线前 A/B 验证记录。
- 新增 `sysid import-evidence`，把 FIGAROH/外部物理一致性和基础参数证据导入 solver metrics，供参数包质量门消费。
- 新增 `sysid figaroh-handoff`，为外部 FIGAROH 运行生成 processed 数据、URDF 和后续导入命令清单。
- 新增 `sysid adapt-figaroh-evidence`，把 FIGAROH-style report 规范化为 `armctrl.external_solver_evidence.v1`。
- 新增 `online-id audit`，以 append-only JSONL 记录 shadow 在线辨识更新窗口、残差变化、饱和检查、回滚目标和人工确认要求。
- 新增 `recipe status` / `recipe cancel` 和显式 recipe command executor gate，Agent skill 只能通过受限 recipe CLI 触达动作。
- 新增 `release status`，机器可读标注 `0.5.0` contracts complete / hardware pending 状态。
- 新增 release status 的本地验证命令和 deferred validation 分类，区分硬件、外部工具和几何升级缺口。
- 新增 `release notes`，从同一状态面生成 v0.5.0 clean rebuild 发布说明。
- 新增 `sysid sdk-preflight`，只读检查 `arx5_interface` 导入状态和目标 model/interface，不打开 CAN、不移动硬件。
- 新增 `sysid sdk-handshake-plan`，只读固定未来 SDK 采集前的确认、hold/damping、记录时序和 Ctrl-C/fault 落态契约。

### Status

- Clean rebuild contracts through v0.5.0 are implemented and tested.
- Hardware SDK runner, full FK/table collision, and n100d Pinocchio/FIGAROH validation remain pending.
- 新增在线辨识 shadow-mode policy，限制在线更新只覆盖 torque bias、摩擦项和小幅 gravity residual。
- 新增项目内 Codex skill: `.codex/skills/armctrl-agent-recipes/SKILL.md`，限制 Agent 只能通过 recipe CLI 查看和规划动作。

### Changed

- 停止在旧实验实现上继续叠加功能，后续按 `v0.2.0` 到 `v0.5.0` milestone 重新实现。
- SysID execute 仍保持 rejected；workspace clearance 已升级为 URDF frame-level FK/table clearance，但仍不是完整 mesh/body collision model。
- `sysid run --adapter sdk` 仍保持 rejected，等待真实 SDK runner、安全落态和真机验证。
- 当前 postprocess 覆盖基础文件合同和 data health；预测误差、物理一致性等进入固定 solver stage 后继续补齐。

## [0.1.0] - Historical Baseline

### Added

- 历史 `develop`/`feature/subsystem` 分支包含初始 SDK adapter、安全检查、teleop、SysID 轨迹生成、采集、后处理和 solver 报告实验。

### Known Gaps

- 历史实现边界不清，容易重复 LeRobot、FIGAROH 和 ARX5 SDK 的成熟能力。
- 历史文档较多，当前工作以 `ROADMAP.md` 为主线。
