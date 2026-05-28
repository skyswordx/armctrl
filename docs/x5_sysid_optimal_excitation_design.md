# X5 SysID 最优激励轨迹设计

> 状态：工程版第一阶段  
> 目标：把 Fourier 激励从“调大幅值”升级为“带约束的非线性优化问题”。

## 核心判断

成熟的动力学参数辨识激励轨迹不是固定模板，而是一个 optimal experiment design 问题：

```text
minimize   cond(Y(q, dq, ddq)) 或 Fisher information 相关指标
variable   Fourier 系数 a_k, b_k
subject to 关节位置/速度/加速度限制
           起止零速度、零加速度
           工作空间、避障、自碰、力矩/电流限制
           机型专用安全姿态族
```

当前代码已经把“Fourier 系数作为变量”和“关节空间安全约束”接进 `optimize_fourier_multisine()`。有 SciPy 时使用 SLSQP 做约束非线性优化；没有 SciPy 时退回多 seed 候选筛选，避免远端环境因为可选优化器缺失而不可用。

## X5 的实机安全约束

实机验证表明，`0 0.30 0.30 0 0 0` 是当前蒸汽机械臂的安全水平姿态：

- joint2 微微抬起 link2；
- joint2/joint3 相近时，躯干近似水平；
- 从高中心姿态大幅摆动会让末端下探到桌面/底座平面，触发过流。

因此 X5 默认 Fourier 现在使用：

- `q_center = (0, 0.30, 0.30, 0, 0, 0)`
- `amplitude = 1.20 rad`
- `positive_only_joint_indices = (joint2, joint3)`
- `joint2 - joint3` 关系约束：`[-0.08, 0.08] rad`

注意：这不是把 joint2/joint3 永久硬编码成同一信号。优化器会用这个关系作为安全可行域，先把初始 Fourier 系数修复到可行域，再在系数空间里继续优化。

## 已实现能力

1. `generate_fourier_multisine()` 支持显式 Fourier 系数。

   这让优化器可以真正输出一组系数，再用同一生成器重建轨迹，而不是只能随机生成。

2. `optimize_fourier_multisine()` 支持关节关系约束。

   目前形式是 `(left, right, min_delta, max_delta)`，用于表达类似 `joint2 - joint3` 的安全姿态族。

3. SLSQP 非线性优化后端。

   SciPy 可导入时，把 Fourier 系数作为变量，目标函数最小化条件数，同时通过不等式约束限制关节位置、速度、加速度和关节关系。

4. 后处理报告会检查 planned trajectory。

   `ident-postprocess` 的 `quality_report.md` 会显示 planned trajectory 是否越过模型限位，并额外标出 profile 中声明的关节关系约束。

## 仍然缺失的成熟约束

当前实现还不是最终论文级 OED，主要缺三类约束：

1. FK/table clearance 约束。

   上次过流的直接原因是末端下探到桌面/底座水平面。只靠关节限位无法判断末端是否撞桌。下一步应使用 URDF/Pinocchio FK 计算末端和关键 link 的高度，加入 `z > table_z + margin`。

2. torque/current proxy 约束。

   现在限制的是位置、速度、加速度，但没有预测 joint2 抗重力电流是否会飙高。下一步应利用 URDF/CAD 参数或已辨识参数粗算 `tau(q,dq,ddq)`，并约束每关节力矩/电流上限。

3. 真实 regressor 条件数闭环。

   CLI 已支持 `--optimize-regressor --urdf-path ...` 用 Pinocchio scorer 评分候选，但 SLSQP 当前默认只对代理特征矩阵优化。要进一步成熟，需要让 Pinocchio regressor scorer 参与连续优化，或采用两阶段流程：SLSQP 先优化代理目标，再用 Pinocchio 对候选做最终排序。

## 推荐上机顺序

先只做规划，不动机器：

```bash
uv run arx5ctl ident-plan --adapter fake --model X5 \
  --profile fourier_multisine --dof 6 --sample-hz 100 \
  --duration 40 --amplitude 1.2 \
  --optimize --candidate-count 24 \
  --output runs/plan-x5-fourier \
  --json
```

检查输出中的：

- `status == completed`
- `metadata.q_center == [0, 0.30, 0.30, 0, 0, 0]`
- `metadata.optimization.joint_relation_constraints`
- `planned_joint_ranges_deg`
- planned CSV 中 joint2/joint3 不低于安全中心，且二者差值在 `[-0.08, 0.08] rad`

再做低风险试跑：

```bash
uv run arx5ctl ident-run --adapter sdk --model X5 --interface can0 \
  --profile fourier_multisine --dof 6 --sample-hz 100 \
  --duration 40 --amplitude 0.6 \
  --optimize --candidate-count 24 \
  --output runs/ident-sdk-fourier-probe \
  --execute --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" \
  --json
```

确认没有下探、撞桌、过流，再放大到正式幅度：

```bash
uv run arx5ctl ident-run --adapter sdk --model X5 --interface can0 \
  --profile fourier_multisine --dof 6 --sample-hz 100 \
  --duration 40 --amplitude 1.2 \
  --optimize --candidate-count 24 \
  --output runs/ident-sdk-fourier \
  --execute --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" \
  --json
```

后处理：

```bash
uv run arx5ctl ident-postprocess \
  --dataset <runs/ident-sdk-fourier-...> \
  --tool pinocchio --tool figaroh --tool urdfly \
  --urdf-path configs/models/X5_camera.urdf \
  --filter-mode zero_phase_moving_average \
  --smoothing-window 5 \
  --json
```

## 参考依据

- Park, [Fourier-based optimal excitation trajectories for the dynamic identification of robots](https://cir.nii.ac.jp/crid/1360574093683863680), Robotica, 2006：Fourier series + polynomial/boundary functions 的轨迹参数化思路。
- Wu et al., [Closed-Loop Dynamic Parameter Identification of Robot Manipulators Using Modified Fourier Series](https://journals.sagepub.com/doi/10.5772/45818), 2012：modified Fourier series 用于闭环动力学参数辨识。
- Sensors 2022, [A Two-Step Method for Dynamic Parameter Identification of Indy7 Collaborative Robot Manipulator](https://www.mdpi.com/1424-8220/22/24/9708)：使用有限 Fourier series 生成激励轨迹，并以 observation/regressor matrix 条件数评价轨迹质量。
- Sensors 2019, [Dynamic Parameter Identification for a Manipulator with Joint Torque Sensors Based on an Improved Experimental Design](https://www.mdpi.com/1424-8220/19/10/2248)：把起止零速度/零加速度、关节位置/速度/加速度约束和条件数收敛作为实验设计约束。
- FIGAROH/Pinocchio 思路：用真实动力学 regressor 评价轨迹，而不是只看命令角度覆盖。
