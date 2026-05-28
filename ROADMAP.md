# armctrl Roadmap

本文是 ARX5/X5 运控、系统辨识和 Agent recipe 重构的当前事实源。

## Project Boundary

`armctrl` 是 Roboclaw 内部面向 ARX5/X5 的安全执行与系统辨识外壳。它不做第二套 LeRobot，不做第二套 FIGAROH，也不重写 ARX5 SDK。

- ARX5 SDK / `arx5-interface`: 底层控制器、CAN 通信、IK、动力学和硬件会话。
- Pinocchio: 可复现的本地回归矩阵构建、rank/condition/prediction error 校验。
- FIGAROH: 成熟的机器人辨识数学，包括激励轨迹、基础参数和物理一致性工具。
- `lerobot-robot-arx5` / `lerobot-teleoperator-arx5`: LeRobot Robot/Teleoperator 接入、数采、训练和 policy rollout。
- `armctrl`: 安全门、动作 recipe、辨识采集编排、数据交接、参数包、上线校验和 Agent CLI 命令。

## Release Track

Current package version: `0.5.0`.

Release readiness: `contracts_complete_hardware_pending`.

### v0.2.0 - Governance And Safety Boundary

- [x] 保持 `README.md`、`ROADMAP.md`、`CHANGELOG.md`、`docs/README.md` 为唯一当前入口。
- [x] 定义第一版 Agent recipe catalog 和 JSON 请求/响应契约。
- [x] 定义 safety gate 的 plan-only / execute 边界。
- [x] 所有硬件运动入口默认先 dry-run，并能解释计划轨迹风险。

### v0.3.0 - Offline SysID Loop

- [x] 定义 plan-only SysID profile 和 Pinocchio/FIGAROH/LeRobot handoff 合同。
- [x] `sysid plan --output` 写出 `planned_trajectory.csv` 和 `manifest.json`。
- [x] `sysid plan --output` 评估 URDF joint limit 并在 stdout/manifest 标注 pass/fail。
- [x] `sysid plan --output` 评估 URDF frame-level FK/table clearance 并在 stdout/manifest 标注 pass/fail。
- [x] `sysid run --adapter fake` 生成 `raw_samples.csv` 和 run manifest。
- [x] `sysid postprocess` 输出清洗数据、质量指标和 Markdown 报告。
- [x] `sysid solve` 输出固定 solver 阶段产物、Pinocchio/FIGAROH 可用性和 fake 数据残差冒烟检查。
- [x] `sysid postprocess --solve` 支持后处理后立即运行固定 solver 阶段。
- [x] `sysid plan` 在实机运动前使用 URDF frame-level FK/table clearance 拒绝不安全 planned trajectory。
- [ ] `sysid plan` 升级到 mesh/body collision model，覆盖连杆几何而不只检查 link frame。
- [x] `sysid sdk-preflight` 只读检查 SDK 环境，不打开 CAN、不实例化硬件对象。
- [x] `sysid sdk-handshake-plan` 只读固定实机采集前确认、hold/damping、记录时序和 Ctrl-C/fault 落态契约。
- [ ] `sysid run` 使用 ARX5 SDK 采集 gravity、friction、Fourier profile，并有 hold、damping、Ctrl-C 落态。
- [x] `sysid solve` 接入可选 Pinocchio regressor，输出 rank 和条件数；缺依赖时结构化降级。
- [x] `sysid solve` 用 Pinocchio regressor 做 least-squares 回代并输出 prediction RMSE；缺依赖时结构化降级。
- [ ] `sysid solve` 输出基础参数和物理一致性指标。
- [x] FIGAROH/外部工具证据可通过 `sysid import-evidence` 导入，并被 package gate 消费。
- [x] `sysid adapt-figaroh-evidence` 将 FIGAROH-style report 规范化为 armctrl external evidence。
- [ ] 本地不重写 FIGAROH 最优轨迹或物理一致性内部实现，只继续补官方工具输出适配器。
- [x] `sysid package` 只有质量门通过时才生成候选参数包，否则结构化拒绝。
- [x] 候选参数包加入版本号、签名、回滚目标和上线 A/B 验证记录。

### v0.4.0 - Conservative Online Identification

- [x] 在线估计第一版只允许更新 torque bias、viscous friction、Coulomb friction 和小幅 gravity residual。
- [x] 质量、质心、惯量等完整刚体参数继续保持离线辨识。
- [x] 在线更新先进入 shadow mode，不直接影响控制命令。
- [x] 每次在线更新必须记录数据窗口、残差变化、饱和检查、回滚目标和人工确认。

### v0.5.0 - Agent Recipe And CLI Skill

- [x] 增加 recipe registry: `home`、`damping`、`hold-current`、`observe-front`、`pregrasp-table`、`retreat-safe`。
- [x] 所有可能动硬件的 recipe 都支持 plan-only。
- [x] 增加 JSON CLI: 列出 recipe、dry-run、执行、取消、查看状态。
- [x] 编写 Codex CLI skill，只允许调用这些受限命令。
- [x] 测试证明 Agent 路径不能绕过 safety gate 和 command executor。

### v0.6.0 - LeRobot Integration

- [ ] 在目标 Linux 主机验证 `lerobot-robot-arx5` 和 `lerobot-teleoperator-arx5`。
- [ ] `armctrl.compat.lerobot` 只保留为数据集 metadata/export，除非明确需要 safety bridge。
- [ ] 如果 policy action 必须经过 `armctrl`，只加薄 safety bridge，不重写完整 LeRobot Robot。
- [ ] 说明 LeRobot record/train/rollout 产物与 `armctrl` SysID 数据集、参数包之间的关系。

## Rules

- 一个重构主题一条分支。
- 一个可解释行为或文档边界一个小提交。
- 用户可见变更必须更新 `CHANGELOG.md`。
- release candidate 需要 tag 和简短 release note。
- 声称完成前必须记录测试或无硬件验证命令。
