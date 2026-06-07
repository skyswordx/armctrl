# armctrl 文档索引

当前 clean 分支只保留少量主线文档，避免文档越写越散。新文档如果没有放进这个索引，默认不算项目入口。

## 必读入口

1. `../README.md`：项目定位、CLI 总入口和当前能力边界。
2. `../ROADMAP.md`：系统边界、milestone 和后续推进顺序。
3. `../CHANGELOG.md`：用户可见变更、离线证据和版本日志。
4. `docs/arm_control_frequency_contract.md`：Agent、EEF、LeRobot、SysID 与真实 SDK 后端的控制频率契约。
5. `docs/hardware_sysid_operator_manual.md`：n100d/真机连接后的安全操作流程。
6. `docs/lab_hardware_validation_checklist.md`：实验室现场按 gate 执行 doctor、hold/damping、tiny motion、Agent smoke、SysID smoke 的短清单。

## SysID 主线文档

1. `docs/x5_sysid_optimal_excitation_design.md`：从验收反推 X5 gravity/friction/Fourier 激励轨迹如何设计，覆盖安全空间、频率分层、OED 坑点和当前 Fourier 候选。
2. `docs/identification_result_quality_metrics.md`：从数据健康、激励质量、求解、物理一致性、预测和控制收益判断 SysID 结果是否合格。

## 外部参考原则

clean rebuild 分支不直接携带历史长文档和官方 PDF 快照。需要查历史材料时，从旧分支或 vendor upstream 显式取回，并在本文登记用途。

长期有效的设计决策优先进入 `ROADMAP.md`，用户可见变更进入 `CHANGELOG.md`，可执行的真机步骤进入 operator manual，SysID 经验沉淀进入上面的两篇 SysID 主文档。

## 文档规则

- 不新增未索引顶层文档。
- 新实现先更新 roadmap/changelog，再迭代码。
- 实验记录必须标注数据来源、硬件状态、是否上真机、是否可复现。
- 危险轨迹、失败轨迹和 rejected candidate 也要保留 gate 原因，避免后续 Agent 误认为“没有产物就是没试过”。
