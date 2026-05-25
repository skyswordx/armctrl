# 辨识轨迹抖动问题定位与修复复盘

## 背景

在蒸汽机械臂上执行 `ident-run --profile gravity_sweep --adapter sdk --execute` 时，机械臂出现明显抖动，最新一版轨迹参数下抖动比之前更剧烈，需要通过 `Ctrl+C` 急停。急停后 damping 生效，说明安全落态链路可用。

同时，实机上的遥控 IK 逆解控制、直觉 joint 空间控制都没有出现同等级抖动。这一点非常关键：它说明问题不能简单归因于机械臂本体、CAN 通信、关节硬件或“只要运动就会抖”，而应优先怀疑辨识执行链路和稳定控制链路之间的差异。

## 现象判断

这次抖动不应视为正常的参数辨识激励。对于重力项辨识，当前轨迹本来应是准静态、低速、低加速度的扫描；如果在这种设定下仍然剧烈抖动，继续增加幅值只会放大风险。

因此当时停止了“调大轨迹强度”的方向，转为排查：

- 轨迹本身是否存在速度、加速度突变；
- 辨识链路是否和稳定 teleop 链路使用了不同控制器、不同 SDK API、不同 target 同步策略；
- SDK 内部插值器是否会在批量轨迹切换时引入旧目标或瞬态；
- `reset_to_home()` 后是否真的已经完全稳定在 home；
- 下发命令的节拍是否符合 SDK 官方示例。

## 关键证据

### 1. 远端数据表明轨迹幅值不是唯一解释

早期数据集里，`amplitude_rad=0.03` 时各关节实际运动范围很小，不足以完成高质量辨识。但后续 `0.08 rad / 2s` 或更强轨迹下，抖动并没有按“平滑准静态轨迹”的预期改善，反而更明显。

典型数据特征是：

- `q_cmd` 范围并不夸张；
- 实际 `q` 和 `dq` 却出现较明显不稳定；
- 初始阶段存在 `q_cmd` 与实测 `q` 不一致的迹象。

这更像是控制目标接管、SDK 插值器状态、gain 恢复或命令下发方式的问题，而不是单纯“轨迹太大”。

### 2. Teleop 链路有目标同步保护

稳定的 teleop 链路在从 damping、zero gravity drag 或其他模式接管前，会先把 SDK 内部目标同步到当前实测状态，再恢复 motion gain。

相关代码在：

- `src/armctrl/adapters/arx5/sdk.py`
- `_prepare_teleop_takeover()`
- `_sync_eef_target_to_current_state()`

这个逻辑的意义是：恢复刚度时，控制器看到的参考点就是机械臂当前所在的位置，不会突然追逐历史残留目标。

### 3. 旧辨识链路缺少同等保护

旧辨识链路是：

1. `IdentificationRunner.run()`
2. `backend.reset_home()`
3. `backend.send_joint_trajectory(profile.points)`
4. `Arx5JointRobotIO.send_joint_trajectory()`
5. 一次性构造整条 `joint_traj`
6. 调用 SDK `controller.set_joint_traj(joint_traj)`

这里有三个风险点：

- 轨迹默认以 `q0=0` 为基线，未显式读取当前实测关节作为接管起点；
- `reset_to_home()` 后立刻覆盖新轨迹，没有额外确认机械臂已经稳定；
- 批量 `set_joint_traj()` 与 teleop 的逐周期命令方式不同。

### 4. SDK 官方 joint control 示例偏向周期性 `set_joint_cmd`

SDK 的 `python/examples/test_joint_control.py` 使用的是循环：

```python
cmd = arx5.JointState(robot_config.joint_dof)
cmd.pos()[0:4] = easeInOutQuad(...) * target_joint_poses[0:4]
arx5_joint_controller.set_joint_cmd(cmd)
time.sleep(controller_config.controller_dt)
```

也就是说，稳定 joint 控制示例更接近“控制周期内持续发送当前目标”，而不是把 100Hz 的密集轨迹一次性塞给 `set_joint_traj()`。

### 5. SDK `set_joint_traj()` 会混入旧 interpolator 状态

SDK `Arx5JointController::set_joint_traj()` 会先取当前 interpolator 在 `start_time - 0.1s`、`start_time - 0.05s`、`start_time` 的三个状态，再拼接新轨迹，并重新计算速度。

这意味着如果当前 interpolator 里还残留 reset/home/上一模式目标，新轨迹不是从一个完全干净的“当前实测状态”开始，而是会和旧插值状态发生拼接。

对于稀疏 waypoint，这可能是合理的；但对我们之前那种 100Hz 密集轨迹，它更容易形成不可预期的切换瞬态。

### 6. `reset_to_home()` 末段还有 0.5s waypoint

SDK 的 `reset_to_home()` 内部最后会 `override_waypoint(get_timestamp(), target_state)`，把最终归零目标放到未来约 0.5s。函数返回时，并不等价于“机械臂已经完全静止且 interpolator 已清空”。

如果辨识 runner 紧接着调用 `set_joint_traj()`，就可能在 home 末段插值还没有完全结束时覆盖轨迹。

## 根因判断

本次抖动的主因不是“轨迹数学形式不够高级”，而是辨识执行路径和稳定控制路径不一致：

- 稳定链路：先同步目标到当前实测状态，再按控制节拍发送命令；
- 旧辨识链路：reset 后直接批量下发整条密集 `set_joint_traj()`；
- SDK 批量轨迹接口会拼接旧 interpolator 状态；
- 轨迹起点默认 `q0=0`，没有把当前实测关节状态作为接管基线；
- 因此机械臂可能在恢复 gain 或切换插值器时追逐旧目标/错误目标，表现为明显抖动。

这解释了为什么加大轨迹强度会让情况更糟：真正被放大的不是“有效辨识激励”，而是接管瞬态、插值器切换和追踪误差。

## 修改方法

修复 commit：

- `2c553f4 Reduce identification shake by streaming joint commands`

核心改动包括三部分。

### 1. Joint backend 增加流式执行接口

在 `src/armctrl/identification/backends.py` 中，为 `JointRobotIO` 增加可选语义：

- `begin_joint_trajectory(points)`
- `send_joint_command(point)`

`send_joint_trajectory(points)` 仍保留兼容入口，但 SDK backend 现在内部优先走流式命令。

### 2. SDK backend 先同步当前关节目标

新增 `_sync_joint_target_to_current_state()`：

1. 读取 `controller.get_joint_state()`；
2. 构造一个 `JointState`，位置等于当前实测关节角；
3. 速度和力矩置零；
4. timestamp 设置为 `now + controller_dt`；
5. 调用 `controller.set_joint_cmd(sync_cmd)`；
6. 等待一个 `controller_dt`。

这样恢复 motion gain 后，SDK 内部参考目标先对齐到当前实测关节状态，避免立刻追逐旧目标。

### 3. Runner 改为按采样节拍逐点发送

旧逻辑是在开始执行时一次性发送整条轨迹。新逻辑变成：

1. `run()` 开始阶段只调用 `begin_joint_trajectory()`；
2. `_collect_samples()` 内按 `point.t_s` 等待；
3. 到达每个采样时刻后调用 `send_joint_command(point)`；
4. 再读取该时刻样本。

这样 SDK 收到的是按时间推进的 `set_joint_cmd()`，更接近官方稳定 joint control 示例，也减少 `set_joint_traj()` 批量拼接旧 interpolator 状态的风险。

### 4. 故障时请求 damping

如果逐点发送过程中出现异常，runner 会请求 `backend.damping()`，并返回 `CommandStatus.FAULTED`，避免因为中途异常把机械臂留在活动控制状态。

`Ctrl+C` 的 `KeyboardInterrupt` 路径仍然返回 `CommandStatus.CANCELLED`，并请求 damping。

## 测试覆盖

新增和更新的单测覆盖了：

- SDK backend 不再调用 `set_joint_traj()`；
- SDK backend 会先用当前实测关节状态发送同步命令；
- 后续轨迹点通过 `set_joint_cmd()` 发送；
- runner 支持 `begin_joint_trajectory()` + `send_joint_command()` 的增量执行模式；
- 原有 fake backend 兼容旧 `send_joint_trajectory()`；
- `Ctrl+C` 仍然会请求 damping。

本地验证：

```bash
uv run --project . --with pytest pytest tests/unit/test_identification.py -q
uv run --project . --with pytest pytest tests/unit/test_executor_and_cli.py -q
python -m compileall D:\repo\Roboclaw\references\projects\armctrl\src D:\repo\Roboclaw\references\projects\armctrl\tests
```

结果：

- `tests/unit/test_identification.py`：24 passed
- `tests/unit/test_executor_and_cli.py`：16 passed
- `compileall`：通过

远端实机初步反馈：

- n100d 拉取后运行，机械臂“不再很抖动”。

## 后续验证建议

这次修复解决的是“辨识执行链路抖动”的主问题，但还不能直接说明参数辨识结果已经足够好。建议下一步按以下顺序推进。

### 1. 先做低强度安全复测

使用低幅值、长周期、带 dwell 的 gravity sweep：

```bash
uv run arx5ctl ident-run \
  --adapter sdk \
  --profile gravity_sweep \
  --sample-hz 100 \
  --amplitude 0.08 \
  --duration 6.0 \
  --dwell 1.0 \
  --execute \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" \
  --json
```

观察重点：

- 是否仍有明显关节抖动；
- `Ctrl+C` 是否仍能快速落到 damping；
- 各关节实际运动范围是否覆盖到足够角度；
- `q_cmd` 与 `q` 的跟踪误差是否显著下降；
- `dq` 是否平滑，没有尖峰。

### 2. 再评估辨识数据质量

采集后运行：

```bash
uv run arx5ctl ident-postprocess \
  --dataset runs/ident-steam-gravity-YYYYMMDD-HHMMSS \
  --tool figaroh \
  --tool pinocchio \
  --tool urdfly \
  --json
```

重点看：

- 轨迹覆盖范围；
- 最大速度和速度尖峰；
- tracking error；
- 力矩信号是否有可解释变化；
- 回归矩阵条件数；
- 残差 RMSE；
- 交叉验证误差；
- 物理参数是否满足质量、质心、惯量的合理性约束。

### 3. 逐步提高激励，而不是一次到位

建议顺序：

1. `0.08 rad / 6s / 1s dwell`
2. `0.10 rad / 6s / 1s dwell`
3. `0.12 rad / 6s / 1s dwell`

如果某一级开始出现明显抖动或跟踪误差尖峰，不要继续加大幅值，应先分析该级数据。

### 4. 后续仍应最大化复用 FIGAROH

armctrl 的职责应保持在：

- 安全执行；
- 数据采集；
- LeRobot 风格数据契约；
- manifest 和 postprocess handoff；
- 实机 bringup 防护。

动力学回归、base parameter、参数物理一致性约束和优化求解，应该继续交给 FIGAROH / Pinocchio / URDFly 等成熟工具，不在 armctrl 里重造一套系统辨识数学栈。

## 经验总结

这次最重要的判断是：抖动不是通过“更强的辨识轨迹”解决的，而是通过对齐稳定控制链路解决的。

排查顺序应该记住：

1. 先比较稳定链路和故障链路；
2. 优先怀疑状态切换、目标同步、控制模式和 SDK API 差异；
3. 不要在根因不明时增加激励；
4. 用实机反馈验证假设；
5. 保留 `Ctrl+C -> damping` 作为软件急停第一层；
6. 后续所有强度提升都必须看 tracking error、速度尖峰和力矩残差，而不是只看“有没有动起来”。
