# SysID 结果质量与验收指标

本文回答一个核心问题：跑完 X5 SysID 后，怎样判断数据、轨迹、求解结果和最终参数包是否值得继续推进。它不是数学论文，而是给操作者、Agent 和外部专家共同使用的验收指南。

## 快速结论

不要因为某一个数字好看就接受 SysID 结果。完整验收要分层看：

| 层级 | 需要回答的问题 | 典型拒绝原因 |
| --- | --- | --- |
| 数据健康 | 时间戳、状态、命令、电流/力矩是否可信 | 缺列、NaN、时间跳变、反馈全零或饱和 |
| 执行跟踪 | 真机是否真的按命令运动 | actual q 几乎没动，或 tracking error 大于命令幅度 |
| 激励质量 | 轨迹是否激发了独立动力学方向 | rank 异常、base condition 高、关节范围太小 |
| 安全证据 | 轨迹是否留在配置的物理空间内 | 限位、桌面、keepout、碰撞、step、速度、加速度失败 |
| 求解质量 | 回归是否稳定，残差是否可信 | 病态最小二乘、过拟合、优化器失败且未复核 |
| 物理一致性 | 参数是否像一台真实机器人 | 负质量、不合法惯量、质心离谱 |
| 预测质量 | 参数能否预测未参与拟合的数据 | held-out torque/current 误差高或残差有结构 |
| 控制收益 | 参数是否改善真实控制 | 电流更大、振荡、掉臂、过补偿或不可回滚 |

当前最重要的原则：OED 条件数好只说明“轨迹设计有潜力”，不等于已经完成 SysID。真正验收要等真机数据、solver、物理一致性和控制 A/B test 都通过。

## 最小产物清单

任何候选轨迹、数据集或参数包要进入评审，至少应有这些产物：

- `manifest.json`：profile、URDF、安全配置、频率元数据、gate 结果和产物路径。
- `planned_trajectory.csv`：低频 OED/规划轨迹或外部候选轨迹。
- `execution_trajectory.csv`：高频执行轨迹，真机应按这个时间网格重放。
- `trajectory_preview.json`：FK/仿真/碰撞 gate 结果。
- `preview.html`：URDF 动画，用于人工视觉终审。
- `joint_q.svg`、`joint_dq.svg`、`joint_ddq.svg`：位置、速度、加速度曲线。
- `review_summary.json` 或 solver metrics JSON：机器可读验收状态。
- `review_report.md` 或 solver report：人类可读解释。
- 真机数据集：raw samples、processed samples、q/q_cmd、dq、current/torque、timestamp、fault 状态。

缺少产物不代表完全没用，但只能作为 debugging evidence，不能被提升为最终 SysID 结论。

## 数据健康

先看数据，再看数学。坏数据会让任何高级求解器产出漂亮但错误的结果。

| 指标 | 好结果 | 为什么重要 |
| --- | --- | --- |
| sample count | 接近 `duration_s * sample_hz + 1` | 缺样会污染导数和回归矩阵 |
| timestamp | 严格递增，`dt` 稳定 | jitter 和时间跳变会破坏 dq/ddq |
| unit | q 用 rad，gripper 用 m，current/torque 明确单位 | 单位错误会直接产生荒谬参数 |
| required columns | q、q_cmd、dq、dq_cmd、ddq_cmd、current/torque | 缺少物理信号无法解算 |
| NaN/inf | 无 | 数值求解会静默崩坏 |
| feedback variation | 电流/力矩随姿态和运动变化 | 全零反馈不是辨识数据 |
| fault flags | accepted samples 内没有 fault/damping | 故障段应剔除 |

Smoke 数据可以激励不足，但仍必须通过基本数据健康检查。

## 执行跟踪

机器人必须真的执行了命令，否则后处理只是在拟合想象中的轨迹。

建议检查：

- 每个关节 actual range。
- 每个关节 commanded range。
- `actual_range / commanded_range`。
- RMS tracking error 和 max tracking error。
- tracking error 是否在高加速度或速度反向处明显恶化。

常见解释：

| 现象 | 含义 |
| --- | --- |
| q range 接近 0 | 没有真正使能运动、SDK 未执行或处于 damping |
| q range 远小于 q_cmd | 控制器未跟踪、限位/增益/状态有问题 |
| q 跟随但 current 为 0 | 电流/力矩反馈没有接入日志 |
| q 跟随但 current 突刺 | 大阶跃、碰撞、负载、重力补偿或轨迹过激 |
| tracking error 与 q/dq 强相关 | 模型、摩擦或控制器仍不匹配 |

对 X5 而言，几度的小幅 gravity smoke 能证明 pipeline 活着，但不能证明完整动力学可辨识。

## 激励质量

激励质量回答：这份数据是否包含足够独立的信息，能够解出动力学参数。

| 指标 | 验收解释 |
| --- | --- |
| joint range | gravity 应覆盖有意义姿态；Fourier 不应只有几度抖动 |
| velocity coverage | friction 需要正/负速度平台和过零点 |
| acceleration coverage | Fourier 需要平滑但非平凡的加速度 |
| base regressor rank | X5 full 参数可到 60，base rank 约 `36` 正常 |
| FIGAROH base condition | 当前 Fourier/OED 主验收指标 |
| Pinocchio effective condition | 辅助诊断，不应压过 FIGAROH base condition |
| singular values | 最小奇异值过小表示某些参数方向没被激发 |

FIGAROH base condition 经验阈值：

| FIGAROH base condition | 判断 |
| ---: | --- |
| `< 100` | 优秀 OED 候选 |
| `100 - 200` | 可调查，但噪声敏感 |
| `200 - 1000` | 通常不足以最终验收 |
| `> 1000` | 拒绝完整动力学辨识 |
| `1e6+` | 基本不可用，容易出现负质量/负惯量 |

注意：`rank 36/60` 对 6DoF 机械臂不是自动失败。真正的问题是 base space 内条件数高、覆盖弱或奇异值塌陷。

## 安全与平滑 gate

数学上好的轨迹仍可能真机危险。安全 gate 应检查 execution trajectory，而不是只检查 planned trajectory。

必查项：

- URDF joint limits。
- profile-specific safe joint ranges。
- `max_joint_step_rad`。
- velocity limit。
- acceleration limit。
- workspace allowed boxes。
- table/base keepout。
- Pinocchio/coal 或其他成熟碰撞/state-validity 后端。
- HTML URDF 视觉预览。

平滑判断：

| 现象 | 结论 |
| --- | --- |
| q 连续、dq 连续、ddq 峰值圆滑 | 可进入人工视觉复核 |
| ddq 有单点尖峰 | 可能是插值或优化毛刺，应拒绝或重采样 |
| max step 超过门限 | 执行频率/速度/step gate 不匹配 |
| velocity 合法但 step 失败 | 100 Hz 执行频率下相邻点太大，应提高执行频率或降低速度 |
| 碰撞 pass 但 HTML 肉眼可疑 | 优先暂停，补 collision geometry 或 frame 检查 |

对桌面级 X5，`100 Hz` 下 `0.01 - 0.02 rad/sample` 是合理的执行层检查区间。继续放大 step 红线不是首选；如果需要更高动态，应优先实测 SDK send rate 并提升执行频率。

## 求解质量

求解质量关注从数据到参数的稳定性。

至少记录：

- 使用的 URDF 和安全配置版本。
- 滤波模式、截止频率、是否 zero-phase。
- 求导方法。
- torque/current 映射方法。
- regressor 维度、rank、condition。
- 是否使用 base parameter 降维。
- regularization 或 robust fitting 设置。
- train/validation split。
- 每关节残差和整体残差。

如果 IPOPT 或 OED 返回 `max_iterations_exceeded`，不能自动判死刑，也不能自动放行。应看：

- constraint violation 是否为 0 或足够小。
- 目标函数是否已经达到验收区间。
- q/dq/ddq 是否平滑。
- 安全 gate 是否通过。
- stdout tail 中 `obj/inf_pr/inf_du/alpha` 是否显示仍在发散。

当前 Fourier 候选属于“condition 已达标、安全 gate 已达标，但成熟后端未返回 clean optimal label，因此需要离线收敛/视觉终审”的状态。

## 物理一致性

残差低不够。参数必须像一台真实机器人。

检查：

- link mass 全部为正。
- inertia 矩阵正定或满足所选参数化的物理一致性。
- COM 位于或接近真实连杆包络。
- payload mass 和 COM 与真实末端设置一致。
- 使用 `X5_camera.urdf` 时，`link6` 包含 D435i 负载质量。
- 参数量级与 CAD/URDF 尺度一致。
- 重复采集/重复解算的 base parameters 相近。

若出现负质量、不可能的惯量或多次解算差异巨大，即使训练残差低，也应拒绝。

## 预测质量

最终模型需要预测未参与拟合的数据。

核心形式：

```text
tau_pred = Y(q, dq, ddq) * pi
residual = tau_meas - tau_pred
```

推荐指标：

| 指标 | 含义 |
| --- | --- |
| RMSE per joint | 每关节绝对预测误差 |
| NRMSE per joint | 相对 torque/current 范围的归一化误差 |
| R2 / explained variance | 模型解释了多少信号方差 |
| validation RMSE | held-out 数据误差 |
| residual correlation | 残差是否仍与 q/dq/ddq 有结构相关 |

训练误差好但验证误差差，通常意味着过拟合、激励弱、滤波/求导问题或 torque/current 噪声大。

## 控制收益

最终验收不是表格，而是真实控制是否变好。

至少比较：

- SDK 默认参数。
- 旧项目参数，如果存在。
- 新辨识参数包。

控制验收观察：

| 检查 | 好结果 |
| --- | --- |
| 静态保持 | 已知姿态下 holding current/torque 更低 |
| 多姿态重力补偿 | 安全空间内保持误差更小 |
| 拖动手感 | 更均匀、更轻，但不过补偿 |
| 姿态切换 | 无 jerk、无突然下坠、无电流突刺 |
| 重复性 | 多次 enable/disable 后行为一致 |

新参数不能直接成为默认生产配置。必须经过 controlled rollout、A/B test 和 rollback 预案。

## 常见失败诊断

| 现象 | 可能原因 | 行动 |
| --- | --- | --- |
| `data_readiness_status=fail` | 缺列、时间戳或单位错误 | 先修 recorder |
| actual q 接近不动 | 没执行、未使能或 damping | 查 SDK 状态和 enable 流程 |
| current 高突刺 | 碰撞、阻挡、大阶跃、负载或补偿异常 | 停机检查物理环境 |
| condition `1e9` | 激励太小或约束把关键关节绑死 | 重做 OED 轨迹 |
| IPOPT restoration/dual infeasible | 硬约束与低条件数目标冲突 | 放松硬约束或移到安全 gate |
| 120s timeout/约束暴涨 | 频率层混淆，执行 step 被用作 OED 限速 | 拆分 planning/execution/gate |
| `max_joint_step` fail | 轨迹对当前执行频率太快 | 提高执行频率或降低速度 |
| rank 低于预期 | 运动维度缺失或 regressor 生成坏了 | 查 q/dq/ddq 和 regressor |
| 负质量 | 病态回归或 torque 数据差 | 拒绝并改进数据/OED |
| 离线好、控制差 | torque scale、payload、符号约定或控制器集成问题 | 做 A/B 和隔离实验 |

## 当前 X5 验收状态

截至当前 clean 分支，最优离线 Fourier 候选是“可以进入离线终审和分阶段真机 smoke 准备”的状态：

- FIGAROH base condition：`68.43`，达成 `<100` 目标。
- rank：`36`，正常。
- execution trajectory gate：通过。
- FK/table clearance：当前配置下通过。
- Pinocchio/coal collision：当前 URDF 几何下通过。
- q/dq/ddq：需要作为人工视觉终审的一部分继续检查。
- optimizer label：仍需离线复核，因为成熟后端未给出完全 clean optimal label。
- hardware state：不能直接执行 full Fourier；必须先过 gravity 和 friction smoke。

这意味着 OED 设计阶段已经很强，但完整 SysID 还没结束。

## 最终验收清单

推广任何参数包前，逐项确认：

- 数据有有效 timestamp、单位和必需列。
- 真机运行没有未解释的 fault、damping、碰撞或过流。
- actual q 有意义地跟踪 q_cmd。
- torque/current 非零、未饱和，并与姿态/运动相关。
- gravity、friction、Fourier 数据都参与完整动力学验收。
- Fourier/OED 的 FIGAROH base condition 低于目标。
- solver 报告 base rank、condition 和残差。
- 物理一致性通过。
- held-out prediction error 可接受。
- 控制 A/B test 显示改善且无不安全行为。
- 参数包有版本号、manifest 和 rollback 路径。

## 推荐最终报告结构

一份完整 SysID 报告应包含：

1. 硬件设置：机器人、负载、URDF、D435i 质量、安装、桌面高度、CAN interface。
2. 轨迹设置：profile、q center、amplitude、频率、安全配置。
3. 安全证据：gate、碰撞后端、HTML 视觉复核、fault log。
4. 数据健康：sample count、dt statistics、columns、q/q_cmd tracking。
5. 激励证据：range、rank、condition、singular values。
6. 求解证据：后端、滤波、回归方法、残差。
7. 物理一致性：mass、inertia、COM 检查。
8. 验证：held-out prediction error。
9. 控制 rollout：A/B 结果、操作者记录、rollback 结论。

如果某一节缺失，应明确写出缺口，不要用一个总的 pass/fail 掩盖。
