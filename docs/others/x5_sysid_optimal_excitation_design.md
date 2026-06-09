# X5 SysID 最优激励轨迹设计

本文从“最后怎样算验收通过”反推 X5 的 SysID 轨迹应该如何设计。目标读者可以不懂机器人动力学，但需要知道：为什么不能随便让机械臂大幅乱摆，为什么一个看起来安全的轨迹可能完全不能辨识参数，以及为什么 armctrl 必须把 OED、执行频率、仿真安全门控和真机 SDK 执行分开。

## 最终目标

X5 SysID 不是为了生成一段“看起来动得很厉害”的轨迹，而是为了得到一个可版本化、可回滚、能改善真实控制效果的动力学参数包。

完整 SysID 至少需要三类数据，风险从低到高：

| 阶段 | 主要用途 | 能辨识什么 | 不能单独证明什么 |
| --- | --- | --- | --- |
| `gravity_sweep` | 低速姿态覆盖、重力方向和负载 sanity check | 重力项、符号约定、相机/末端负载影响 | 摩擦和惯性参数 |
| `friction_sweep` | 正反向速度平台、过零行为 | 库伦摩擦、黏滞摩擦、跟踪行为 | 完整刚体动力学 |
| `fourier_multisine` | 同时激励位置、速度、加速度 | 惯性、科氏/离心、重力、摩擦耦合项 | 不自动证明真机安全 |

验收顺序也应该按这个风险顺序推进：先让最慢、最可解释的重力采集过关，再做摩擦，最后才执行 Fourier/OED 候选。

## 我们一路踩过的坑

这些经验是当前设计的核心约束，后续 Agent 不应该重新踩一遍。

- X5 的默认安全中心应使用 `q_center = [0, 0.30, 0.30, 0, 0, 0]`。这个姿态下 joint2 和 joint3 微微抬起，躯干大体水平，避免一启动末端向下越过底座水平面并撞到桌面。
- 之前 joint2 出现约 `34 A` 过流，更像是“重力负载 + 轨迹动态冲击 + 桌面/限位阻挡”叠加，不应简单归因于底座是否固定。底座不稳会放大风险，但被挡住或大阶跃才是过流的直接触发因素之一。
- 只检查 URDF joint limit 不够。一个轨迹可以完全在关节限位内，但末端或手腕穿过桌面、底座 keepout 或相机碰撞体，仍然会在真机上暴雷。
- 把 `q2 - q3` 强行约束在极窄范围内可以让机械臂“看起来安全”，但会让肩肘运动高度相关，破坏 OED 对 36 个基参数方向的解耦。Fourier OED 不应使用窄硬绑定；安全应主要由 FK/碰撞/工作空间 gate 承担。
- `max_joint_step_rad` 是执行层相邻点检查，不能用 `max_joint_step_rad * planning_sample_hz` 推导 OED 内部速度上限。这个错误曾把 1 秒 OED 拉伸成约 24 秒，采样点和 IPOPT 约束数量暴涨。
- `configs/models/X5_camera.urdf` 已把 D435i 相机质量并入 `link6` 惯量，`eef_link` 仍作为运动学参考。相机几何目前还不是独立碰撞体，这是后续安全建模缺口。
- HTML 预览必须渲染真实 URDF mesh，而不是只画骨架。骨架预览不足以肉眼判断相机、腕部、桌面、底座附近的空间风险。

## 频率分层

SysID 的很多错误来自把所有“频率”混成一个值。应固定使用下面的术语：

| 层级 | 含义 | 当前建议 |
| --- | --- | ---: |
| OED/planning sample rate | FIGAROH/OED 优化器内部低频采样网格 | 常用 `20 Hz` |
| execution trajectory rate | armctrl 插值后用于安全 gate 和真机重放的轨迹频率 | 当前 `100 Hz` |
| SDK send rate | 真实后端逐点发送 setpoint 的频率 | 从 `100 Hz` 起步并实测 jitter |
| controller internal dt | SDK/驱动内部控制周期 | 必须从 SDK/日志测量，不要猜 |
| record sample rate | q/dq/current/torque 数据记录频率 | 通常与执行轨迹对齐，当前 `100 Hz` |
| UI/input rate | UI 刷新或手柄事件频率 | 不能驱动运动时序 |

设计原则：

- OED 使用物理/配置速度和加速度限制。
- 执行轨迹由 OED 低频轨迹重采样得到。
- `max_joint_step_rad`、速度、加速度、碰撞、桌面等 gate 检查执行轨迹。
- 真机 SDK 只能接收已经通过 gate 的高频执行轨迹。
- UI、Agent 决策频率、LeRobot rollout 频率不能直接等同于电机/SDK 执行频率。

## 安全空间

统一安全空间配置位于 `configs/x5.safe.yaml`。它应该约束所有会让机械臂运动的入口：Agent recipe、EEF 命令、SysID、LeRobot rollout bridge。

关键概念：

- `allowed_workspace_boxes`：关键 link frame 允许出现的空间。
- `forbidden_workspace_boxes`：桌面、底座、环境障碍物 keepout。
- `simulation.link_frames`：当前重点检查 `link5`、`link6`、`eef_link` 等关键 frame。
- `max_joint_step_rad`：执行轨迹相邻采样点最大关节跳变。
- profile-specific `oed_velocity_limits_rad_s` 和 `oed_acceleration_limits_rad_s2`：OED 内部速度/加速度物理约束。
- `simulation.backend_preference`：优先使用 Pinocchio/coal、MuJoCo、MoveIt 等成熟后端，失败时明确标注 fallback。

armctrl 的边界应保持很薄：

```text
Agent recipe / SysID trajectory / LeRobot rollout
  -> armctrl 统一安全空间配置
  -> 转换成成熟后端输入
  -> MoveIt / MuJoCo / Pinocchio-coal / FIGAROH 检查或优化
  -> armctrl 汇总 gate 结果
  -> pass 后才允许进入 SDK 执行
```

armctrl 不应该自己重写 MoveIt、MuJoCo、Pinocchio 或 FIGAROH。它负责配置、转换、审查、记录证据和阻止危险执行。

## OED 轨迹设计规则

一条有效的 Fourier/OED 轨迹需要同时满足“可辨识”和“可执行”：

- 优先使用 FIGAROH base regressor condition。X5 6DoF 的 full inertial parameter 可到 60 维，但 rank 约 `36/60` 是正常的基参数维度，不应把 full 60 维矩阵的病态条件数当成唯一判断。
- 轨迹要覆盖足够独立的姿态、速度和加速度方向，让重力、摩擦、惯性、科氏/离心项能被分开。
- OED 内部要使用 profile 速度/加速度约束，不能使用 URDF 中明显占位的 `1000 rad/s`。
- 不要用过窄 `q2/q3` 硬关系把 shoulder/elbow 绑死。Fourier 的安全性应通过工作空间、桌面、碰撞和执行层 gate 处理。
- 生成低频 `planned_trajectory.csv` 后，必须生成高频 `execution_trajectory.csv` 并重新计算/验证 q/dq/ddq。
- 轨迹要足够平滑。最大加速度可以较大，但 q/dq/ddq 曲线不能有毛刺或锯齿状数值尖峰。

FIGAROH base condition 的经验判断：

| FIGAROH base condition | 判断 |
| ---: | --- |
| `< 100` | 优秀候选，可进入离线可视化终审和真机 smoke 准备 |
| `100 - 200` | 可调查，但对电流/力矩噪声更敏感 |
| `200 - 1000` | 通常不足以作为最终 SysID 轨迹 |
| `> 1000` | 拒绝用于完整动力学辨识 |

## 当前最优 Fourier 候选状态

当前已得到一个离线质量很高的候选，但它不是“自动上真机”的许可。

关键证据：

- FIGAROH base regressor condition：`68.43`。
- Pinocchio effective condition：约 `77.57`。
- rank：`36`，符合 X5 基参数问题预期。
- execution samples：`196`。
- max joint step：约 `0.018476 rad`。
- max velocity：约 `1.84275 rad/s`。
- max acceleration：约 `17.8125 rad/s^2`。
- FK/table clearance：当前配置下通过，最小 EEF clearance 约 `0.0921 m`。
- Pinocchio/coal collision：在当前 URDF collision model 下通过。
- next gate：`review_optimizer_convergence_offline` / `hardware_smoke_plan_ready`，而不是直接真机执行。

这说明 OED 设计阶段已经进入可审查状态，但还没有得到真实采集数据和最终动力学参数。

## 三类 profile 怎么设计

### `gravity_sweep`

重力采集是最低风险入口，重点是低速、可解释、可观察。

建议：

- 从 `q_center = [0, 0.30, 0.30, 0, 0, 0]` 开始。
- 真机 smoke 先用小幅度，例如 `0.05 rad`，确认方向、电流、记录列和停止路径。
- 扩大到验证过的安全幅度，例如当前 plan-only 验证过的 `0.30 rad`，再收集更有用的姿态覆盖。
- 允许 joint2/joint3 使用保守关系，但不要把 gravity-only 数据当作完整动力学辨识。

### `friction_sweep`

摩擦采集需要每个关节有正向/反向速度平台和过零区域。

建议：

- 使用分段梯形或近似匀速段，而不是高加速度乱摆。
- 重点观察速度反向时的电流变化、粘滞/库伦摩擦特征和跟踪误差。
- 风险应高于 gravity，但低于 Fourier。
- friction 通过后，才有资格把 Fourier 的动态电流变化解释为惯性/耦合，而不是纯粹摩擦噪声。

### `fourier_multisine`

Fourier/OED 是完整动力学辨识的主力，也是风险最高的阶段。

建议：

- 用 FIGAROH/Pinocchio 等成熟后端生成或评价轨迹。
- 以 FIGAROH base condition 为主指标，Pinocchio effective condition 作为辅助诊断。
- 不把安全空间硬塞成过窄 joint relation；用 workspace/collision/FK gate 做最终安全判定。
- 必须有 q/dq/ddq 图、HTML URDF 动画、FK/table clearance 和 collision evidence。
- 未通过 gravity/friction 真机 smoke 前，不执行 full Fourier。

## 离线工作流

推荐顺序：

1. 只读检查环境和后端依赖。
2. 生成 plan-only gravity/friction/Fourier 轨迹。
3. 检查 `manifest.json`、`trajectory_preview.json`、`preview.html`。
4. 对 OED 检查 `figaroh_trajectory_request.json`、IPOPT stdout tail、rank、condition、safety gates。
5. 只 freeze 同时通过 base condition 和安全 gate 的候选。
6. 运行 `armctrl sysid review-candidate` 生成 q/dq/ddq 图、FK clearance、HTML 动画和 hardware smoke plan。
7. 在显式真机命令和人工确认前保持 `movement_allowed=false`。

示例：plan-only 重力 smoke。

```bash
SAFE_CENTER="0 0.30 0.30 0 0 0"

uv run armctrl sysid plan gravity_sweep \
  --dof 6 \
  --sample-hz 100 \
  --duration 8 \
  --amplitude 0.05 \
  --q-center $SAFE_CENTER \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output runs/plan-gravity-smoke \
  --json
```

示例：离线复核 frozen Fourier 候选。

```bash
uv run armctrl sysid review-candidate \
  --plan-dir runs/x5-fourier-best-candidate-metric-contract-safe-freeze-20260606 \
  --output runs/x5-fourier-best-candidate-metric-contract-safe-freeze-20260606/offline_review_cli \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --json
```

## 真机递进顺序

不要从最优 Fourier 候选开始。真机阶段必须从最不意外的运动开始：

1. `sdk-handshake-plan`：只读确认 SDK、model、interface 和安全合同。
2. safe hold / damping：确认 Ctrl-C、fault、watchdog 都能落到安全状态。
3. tiny gravity smoke：验证方向、记录、current/torque 数据列、停止路径。
4. wider gravity smoke：验证姿态覆盖和负载影响。
5. friction smoke：验证正反向速度平台和过零电流。
6. Fourier 68.43 smoke：手握急停，在前面全部通过后执行。

真机执行必须记录：SDK send jitter、controller timestamp、q/q_cmd tracking、current/torque、fault/damping 行为。如果后端不能稳定维持所需发送频率，不要静默拉伸轨迹；应明确报告 timing failure。

## 当前缺口

- D435i 质量已进入 `link6`，但相机几何还未作为独立碰撞体建模。
- MoveIt planning scene 尚未完整接入；当前轻量碰撞 oracle 主要是 Pinocchio/coal。
- n100d 上真实 SDK send rate、jitter 和控制器 timestamp 需要实测。
- 当前 safety gate 还主要基于 q/dq/ddq、workspace、collision、FK；电流/力矩预测式 gate 是后续增强项。
- 最终 solver 验收仍依赖真实采集的 torque/current 质量。

## 轨迹进入真机 smoke 的验收清单

- `planned_trajectory.csv` 和 `execution_trajectory.csv` 都存在。
- execution sample rate 明确，时间戳和采样点数量正确。
- URDF limit、profile safe range、max step、velocity、acceleration、workspace、table、collision gate 全部通过。
- HTML preview 渲染真实 X5 URDF mesh，并能显示危险轨迹 warning。
- Fourier 使用 FIGAROH base condition 作为主指标，且 rank 正常。
- q/dq/ddq 曲线平滑，没有数值毛刺。
- gravity 和 friction 真机 smoke 已通过。
- 仍然只有显式 `--confirm "I UNDERSTAND THIS WILL MOVE THE ARM"` 的 SDK 命令才允许真实运动。
