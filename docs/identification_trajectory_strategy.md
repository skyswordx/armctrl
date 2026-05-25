# 参数辨识轨迹策略

本文记录 `gravity_sweep`、`friction_sweep`、`fourier_multisine` 三类轨迹的当前推荐默认值，以及为什么这样改。

> 日期：2026-05-25  
> 背景：上一轮 gravity/friction/fourier 数据都能采集，但 friction 和 fourier 激励不足；fourier 正常结束后自动 damping 导致机械臂从中立位塌下。

## 收尾安全策略

辨识采集正常完成后默认保持 hold，不再自动进入 damping。原因是 Fourier 轨迹本身会回到中心位且末端速度、加速度为零；如果成功结束后立刻 `set_to_damping()`，机械臂会失去主动保持力矩，表现为从中立位置塌下去。

实现上不是简单“不调用 damping”，而是在采集数据和 manifest 写盘后，继续按最终姿态周期性下发零速关节命令。因此命令会停在 hold 状态里，直到操作者按 Ctrl-C。按 Ctrl-C 后会请求 damping 并退出；这时数据已经保存。

Ctrl-C 和故障路径仍然请求 damping。这两个路径代表人工急停或异常，优先让控制器离开持续运动状态。

需要恢复旧行为时，在 `ident-run` 里显式加：

```bash
--damping-after
```

## Gravity Sweep

目标是隔离重力项，所以轨迹应尽量准静态，降低速度、加速度和摩擦项对力矩的混入。当前默认：

- 中心位：`0 0.30 0.30 0 0 0`
- 幅度：`0.12 rad`
- 单段时长：`6.0 s`
- 端点静止采样：`1.0 s`

这样会生成每关节正/负方向的慢速扫描和 dwell 样本。重力数据优先看姿态覆盖、静止段力矩一致性，以及 gravity-only regressor 的预测误差。

## Friction Sweep

摩擦项需要分离速度相关项，所以当前轨迹改成“平滑到边界，再用匀速平台穿过中心”的结构。当前默认：

- 中心位：`0 0.30 0.30 0 0 0`
- 幅度：`0.12 rad`
- 速度层级：`0.025, 0.06, 0.12 rad/s`
- 每个关节、每个速度层级都覆盖正向和反向匀速段

分析时优先截取 phase 名包含 `plateau` 的样本。低速层用于 Coulomb/Stribeck 附近行为，中高速层用于 viscous friction 斜率。

## Fourier Multisine

完整动力学辨识需要持续激励并改善回归矩阵条件数。当前默认：

- 中心位：`0 0.30 0.30 0 0 0`
- 时长：`20.0 s`
- 幅度：`0.08 rad`
- 谐波数：`5`
- 仍使用五次包络让起点和终点满足零速度、零加速度

如果显式给短时长但不指定幅度，CLI 会按 `min(0.08, 0.02 * duration^2)` 自动缩小默认幅度，避免短测试时加速度超限。正式上机建议仍使用 `--optimize --candidate-count 24`，让候选 Fourier 轨迹按条件数/激励评分择优。

## 下一轮推荐执行顺序

先跑 gravity，再跑 friction，最后跑 fourier。每条 `ident-run` 正常结束后会保持 hold；确认机械臂稳定在最终姿态后按 Ctrl-C 退出 hold，程序会请求 damping。或者在命令里显式加 `--damping-after` 恢复旧行为。

```bash
uv run arx5ctl ident-run --adapter sdk --model X5 --interface can0 \
  --profile gravity_sweep --dof 6 --sample-hz 100 \
  --output runs/ident-sdk-gravity \
  --execute --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" --json

uv run arx5ctl ident-run --adapter sdk --model X5 --interface can0 \
  --profile friction_sweep --dof 6 --sample-hz 100 \
  --output runs/ident-sdk-friction \
  --execute --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" --json

uv run arx5ctl ident-run --adapter sdk --model X5 --interface can0 \
  --profile fourier_multisine --dof 6 --sample-hz 100 \
  --optimize --candidate-count 24 \
  --output runs/ident-sdk-fourier \
  --execute --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" --json
```

每次采完后运行：

```bash
uv run arx5ctl ident-postprocess \
  --dataset <runs/ident-sdk-...> \
  --tool pinocchio --tool figaroh \
  --urdf-path configs/models/X5_camera.urdf \
  --json
```

## 参考依据

- Park, *Fourier-based optimal excitation trajectories for the dynamic identification of robots*, Robotica, 2006: <https://www.cambridge.org/core/journals/robotica/article/abs/fourierbased-optimal-excitation-trajectories-for-the-dynamic-identification-of-robots/C828D37408E35D40FF4E5070E129960D>. 该文使用 Fourier series + polynomial/boundary 条件，并优化 Fourier 系数以降低测量扰动敏感性。
- *A Two-Step Method for Dynamic Parameter Identification of Indy7 Collaborative Robot Manipulator*, Sensors, 2022: <https://pmc.ncbi.nlm.nih.gov/articles/PMC9783800/>. 文中报告 `20 s` Fourier excitation trajectory，并使用 condition number 评价激励质量；这支撑当前 Fourier 默认时长和“先摩擦、再动力学”的两阶段流程。
- 多篇 friction identification 文献把关节摩擦建模为 Coulomb + viscous/Stribeck 项，并强调在不同速度下测量电流/力矩；匀速平台可以避免加速度项污染摩擦估计。
- 准静态标定文献通常在多个姿态停留并采样 stationary samples；这对应 gravity sweep 的慢速扫描加 dwell。
