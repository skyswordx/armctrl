# 参数辨识结果质量评估指标

本文档用于判断一次机械臂参数辨识数据和辨识结果是否值得进入下一步：FIGAROH 离线求解、参数物理一致性检查，以及上机重力补偿验证。

## 结论先行

一次参数辨识不能只看最小二乘残差是否小。可接受的结果至少要同时满足：

- 数据健康：时间戳、列、单位、采样频率、力矩反馈都可信。
- 激励充分：每个待辨识自由度都有足够姿态覆盖，回归矩阵不病态。
- 参数合理：质量为正、惯量正定、质心和惯量数量级符合机构尺寸。
- 预测有效：未参与拟合的数据上，力矩预测误差仍然低。
- 控制收益：上机重力补偿后，保持误差、电流/力矩需求、拖动手感实际改善。

`gravity_sweep` 主要用于低风险 bringup、重力项和末端 payload 影响验证。它不能单独证明完整刚体动力学参数已经辨识充分；完整动力学还需要 `friction_sweep` 和 `fourier_multisine`。

## 1. 数据健康指标

采集完成后先检查数据本身。

| 指标       | 判断方式                                                              | 建议阈值                     |
| ---------- | --------------------------------------------------------------------- | ---------------------------- |
| 样本数     | `raw_samples.csv` 行数是否符合 `duration_s * sample_hz`           | 误差不应明显偏离             |
| 时间戳     | `t_s` 是否严格单调，采样间隔是否稳定                                | `dt` 抖动越小越好          |
| 列完整性   | `q_* / dq_* / tau_meas_* / q_cmd_* / dq_cmd_* / ddq_cmd_*` 是否齐全 | 不允许缺列                   |
| 数值完整性 | 是否存在 NaN、空值、无穷值                                            | 不允许                       |
| 实际跟踪   | `q` 是否跟随 `q_cmd`                                              | RMS error 应明显小于激励幅度 |
| 力矩反馈   | `tau_meas` 是否非零、非饱和、非纯噪声                               | 应随姿态/运动变化            |

如果实际关节位置几乎不动，或力矩反馈几乎全零，这批数据不能用于参数辨识，只能算通信或流程 smoke test。

## 2. 激励充分性指标

激励充分性决定参数能否被辨识出来。

| 指标            | 含义                                  | 判断                                                      |
| --------------- | ------------------------------------- | --------------------------------------------------------- |
| 关节角覆盖范围  | 每个关节实际扫过的角度范围            | gravity 数据建议至少覆盖约 8 到 15 度，再根据安全边界调整 |
| 命令/实际覆盖比 | `actual q range / q_cmd range`      | 过低说明执行器没有充分跟踪                                |
| 回归矩阵 rank   | `Y(q,dq,ddq)` 有效秩                | rank 不足时参数不可辨识                                   |
| 条件数          | `cond(Y)` 或 base regressor 条件数  | 越小越好；极大值表示参数强耦合                            |
| 奇异值谱        | 最小奇异值是否接近 0                  | 接近 0 表示激励退化                                       |
| 多数据集覆盖    | gravity、friction、multisine 是否互补 | 完整动力学需要组合数据                                    |

注意：当前 `armctrl` 内部的 Fourier 轨迹评分仍是 surrogate feature condition，不等价于真实动力学回归矩阵条件数。真实条件数应交给 FIGAROH、Pinocchio 或 URDFly 生成 regressor 后再计算。

## 3. 参数物理合理性指标

FIGAROH 求解后必须检查物理一致性。

| 指标           | 好结果表现                          |
| -------------- | ----------------------------------- |
| link mass      | 全部为正，数量级符合机械臂尺寸      |
| inertia        | 惯量矩阵正定，主惯量为正            |
| center of mass | 质心位置落在合理连杆范围内          |
| payload 参数   | 与末端真实负载尺寸和重量相符        |
| 参数方差       | 标准差/置信区间不要过大             |
| 重复一致性     | 多次采集得到的 base parameters 接近 |

如果出现负质量、非正定惯量、质心远离连杆，即使训练误差低，也不能直接上机使用。

## 4. 预测误差指标

离线验证的核心是用辨识参数预测力矩：

```text
tau_pred = Y(q, dq, ddq) * pi
residual = tau_meas - tau_pred
```

建议至少输出：

| 指标                    | 含义                               | 经验判断                             |
| ----------------------- | ---------------------------------- | ------------------------------------ |
| RMSE per joint          | 每个关节力矩均方根误差             | 越低越好                             |
| NRMSE                   | 归一化误差，方便跨关节比较         | 低于 10% 到 20% 通常可作为不错的起点 |
| R2 / explained variance | 力矩变化解释率                     | 大于 0.8 可用，大于 0.9 较好         |
| validation RMSE         | 未参与拟合数据上的误差             | 应接近训练误差                       |
| residual correlation    | 残差是否还随 `q/dq/ddq` 系统变化 | 理想情况应接近随机噪声               |

训练集误差低但验证集误差高，通常说明激励不足、过拟合、滤波策略不合理，或力矩反馈质量不足。

## 5. 上机控制收益指标

最终必须用控制效果验收。

| 对比项            | 期望变化                   |
| ----------------- | -------------------------- |
| 静止保持电流/力矩 | 新参数低于默认参数         |
| 多姿态保持误差    | 新参数下误差更小           |
| 拖动手感          | 更轻、更均匀，无明显过补偿 |
| 姿态切换          | 无震荡、无急跳、无异常下坠 |
| 重复测试          | 多次开关补偿效果一致       |

推荐对比三组：SDK 默认参数、旧项目参数、新辨识参数。

## 2026-05-25 远端机械臂数据判断

远端机器：

```text
host: 172.19.122.197
repo: /home/circlemoon/Roboclaw/references/projects/armctrl
branch: feature/subsystem
commit: b6ed292
```

检查到两批 `gravity_sweep` 数据：

- `runs/ident-steam-gravity-20260525-151350`
- `runs/ident-steam-gravity-20260525-150431`

### 最新数据：`ident-steam-gravity-20260525-151350`

基本信息：

| 项                    | 值                                                             |
| --------------------- | -------------------------------------------------------------- |
| profile               | `gravity_sweep`                                              |
| dof                   | 6                                                              |
| sample_hz             | 100 Hz                                                         |
| duration              | 48 s                                                           |
| sample_count          | 4801                                                           |
| commanded amplitude   | 0.03 rad                                                       |
| commanded total range | 0.06 rad, about 3.44 deg                                       |
| timestamp quality     | `dt_mean=0.01000s`, `dt_min=0.01000s`, `dt_max=0.01000s` |

关节覆盖：

| Joint | Actual q range | Actual q range deg | Command q range deg | 判断               |
| ----- | -------------: | -----------------: | ------------------: | ------------------ |
| 1     |   0.045395 rad |           2.60 deg |            3.44 deg | 偏小，但有运动     |
| 2     |   0.027084 rad |           1.55 deg |            3.44 deg | 明显不足           |
| 3     |   0.012970 rad |           0.74 deg |            3.44 deg | 严重不足           |
| 4     |   0.053406 rad |           3.06 deg |            3.44 deg | 接近命令，但仍太小 |
| 5     |   0.056458 rad |           3.23 deg |            3.44 deg | 接近命令，但仍太小 |
| 6     |   0.049973 rad |           2.86 deg |            3.44 deg | 接近命令，但仍太小 |

力矩变化：

| Joint | tau range |  tau std | 判断       |
| ----- | --------: | -------: | ---------- |
| 1     |  3.762052 | 0.505425 | 有明显变化 |
| 2     |  4.853330 | 0.538711 | 有明显变化 |
| 3     |  6.317946 | 0.653266 | 有明显变化 |
| 4     |  0.428659 | 0.058111 | 变化较小   |
| 5     |  0.216193 | 0.030791 | 变化很小   |
| 6     |  0.234831 | 0.032595 | 变化很小   |

判断：

- 这批数据的采样时序和文件链路是健康的。
- `manifest.json`、`raw_samples.csv`、`lerobot_contract.json`、`processed_samples.csv`、`tool_handoff.md` 都已生成。
- 但轨迹激励确实太小。命令总覆盖只有约 3.44 度，实际覆盖更小，尤其 joint 2 和 joint 3 不充分。
- 这批数据可以作为 bringup / pipeline smoke test，也可以初步观察低幅重力项趋势。
- 这批数据不适合作为最终参数辨识数据，尤其不适合判断完整动力学参数有效性。

### 早一批数据：`ident-steam-gravity-20260525-150431`

这批数据更弱：

| Joint | Actual q range deg | tau range | 判断            |
| ----- | -----------------: | --------: | --------------- |
| 1     |           0.00 deg |  0.000000 | 无有效运动/反馈 |
| 2     |           0.00 deg |  0.000000 | 无有效运动/反馈 |
| 3     |           0.00 deg |  0.000000 | 无有效运动/反馈 |
| 4     |           0.00 deg |  0.007456 | 基本无效        |
| 5     |           0.00 deg |  0.007456 | 基本无效        |
| 6     |           0.00 deg |  0.007456 | 基本无效        |

判断：这批不能用于辨识，只能说明命令流程或采样流程曾经跑过。

## 下一轮采集建议

下一轮建议仍从 `gravity_sweep` 开始，但增大覆盖、降低速度并加入目标姿态驻留。CLI 的 `gravity_sweep` 默认值已经调整为 `0.12 rad / 6.0 s / 1.0 s dwell`；下面命令显式写出参数，方便现场确认。

这个选择不是全局最优动力学激励轨迹。公开文献中，完整动力学辨识通常使用有限 Fourier 或改进 Fourier 轨迹，并在关节位置、速度、加速度约束下优化回归矩阵条件数、奇异值谱或 Fisher 信息矩阵。当前阶段只针对重力项和 payload 的低风险数据采集，所以优先选择准静态、单关节、大覆盖、带驻留的扫描，减少动态项和控制跟踪抖动对力矩数据的污染。后续完整动力学再交给 FIGAROH/Pinocchio/URDFly 生成真实 regressor 后做条件数优化。

```bash
uv run arx5ctl ident-plan \
  --adapter fake \
  --profile gravity_sweep \
  --dof 6 \
  --amplitude 0.12 \
  --duration 6.0 \
  --dwell 1.0 \
  --sample-hz 100 \
  --json
```

如果预览和安全检查都没问题，再真机采集：

```bash
uv run arx5ctl ident-run \
  --adapter sdk \
  --model X5 \
  --interface can0 \
  --profile gravity_sweep \
  --dof 6 \
  --amplitude 0.12 \
  --duration 6.0 \
  --dwell 1.0 \
  --sample-hz 100 \
  --output runs/ident-steam-gravity \
  --execute \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" \
  --json
```

如果 `0.12 rad` 仍有明显抖动，不要继续加大幅度，先检查增益恢复、轨迹跟踪、机械间隙、线缆/负载干涉、CAN 时延和 SDK 插值执行状态。如果它稳定但覆盖仍不足，再考虑围绕安全中心姿态设置 `--q-center`，或进入完整动力学阶段的优化轨迹。

建议的递进顺序：

1. `gravity_sweep --amplitude 0.12 --duration 6.0 --dwell 1.0`
2. 如果各关节实际覆盖达到约 8 到 15 度且运动平滑，再进入后处理和 FIGAROH 重力项验证
3. 重力数据健康后，再跑 `friction_sweep`
4. 最后再跑带 `--q-center` 和 `--optimize` 的 `fourier_multisine`

每次采集后立即计算：

- 每个关节 actual q range
- 每个关节 `actual q range / q_cmd range`
- 每个关节 `q - q_cmd` RMS
- 每个关节 `tau_meas` range 和 std
- 后处理是否生成 `processed_samples.csv` 与 `lerobot_contract.json`

只有当各关节覆盖足够、跟踪正常、力矩反馈有变化后，才值得把数据交给 FIGAROH 做正式辨识。

## 参考依据

- Swevers 等人的经典工作把机器人最优激励与辨识放在同一流程中，使用五项 Fourier 轨迹，并以 observation/regressor 的条件数等指标评价激励质量。
- Park 的 Fourier-based optimal excitation trajectory 方法使用 Fourier series 与多项式组合，优化 Fourier 系数以降低测量扰动对辨识的敏感性，并满足关节/笛卡尔空间运动约束。
- 近年的机械系统最优激励研究也强调：参考轨迹需要在物理约束和测量噪声下最大化可辨识信息，而不是单纯增大幅度。
- 多篇机器人动力学辨识论文采用有限 Fourier series 或五阶 Fourier series，并以 observation matrix 条件数、D-optimality、奇异值谱或 Fisher 信息矩阵作为优化目标。

可检索的公开入口：

- Swevers et al., "Optimal Robot Excitation and Identification", IEEE Transactions on Robotics and Automation.
- Park, "Fourier-based optimal excitation trajectories for the dynamic identification of robots", Robotica, 2006.
- "Optimal excitation trajectories for mechanical systems identification", Automatica, 2021.
- "An Analytical Approach for Dealing With Explicit Physical Constraints in Excitation Optimization Problems of Dynamic Identification", HKUST research portal.

本文当前给出的 `gravity_sweep --amplitude 0.12 --duration 6.0 --dwell 1.0` 不是完整动力学的全局最优 Fourier 激励，而是针对重力项/末端 payload 的现场安全折中：覆盖比上一轮足够大，速度和加速度又低，并通过驻留样本降低动态项污染。
