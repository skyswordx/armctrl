# Agent 侧机械臂控制频率契约

本文用于把当前会话的 Agent/EEF/LeRobot/SysID 控制频率背景交接给真实运控后端会话 `019e9bd8-11c1-78d1-bd01-2e063ff80956`。目标不是定义电机内环，而是明确：Agent 能以什么频率表达意图，armctrl 生成什么频率的轨迹，真实 SDK/MoveIt/LeRobot 后端应该以什么方式接管执行。

## 来源边界

- 当前仓库：`D:\repo\Roboclaw\references\projects\armctrl-clean`，分支 `codex/armctrl-clean-rebuild`。
- 旧会话本地记录：`C:\Users\c1rcLEmoon\.codex\sessions\2026\06\06\rollout-2026-06-06T15-31-32-019e9bd8-11c1-78d1-bd01-2e063ff80956.jsonl`。
- 旧会话结论：`armctrl-clean` 的 Agent/recipe/EEF 当前主要生成 contract、preview、review 和 helper plan，不直接驱动真机；SysID SDK run 会创建 `Arx5JointController` 并逐点调用 `set_joint_cmd`；旧 `armctrl` teleop 真机路径默认命令循环约 `100 Hz`，UI 约 `50 Hz`。
- 当前代码事实：`EefTwistRequest.control_period_s = 0.1`，`EefDeltaPoseRequest.control_period_s = 0.1`，Agent/EEF preview 默认 `sample_hz = 50.0`，LeRobot rollout preview 默认 `sample_hz = 50.0`，SysID OED execution 默认 `100 Hz`。

## 一句话结论

Agent 不应该直接以电机内环频率控制机械臂。Agent 应该以低频、带安全约束的 EEF intent frame 表达“想往哪里动”；真实运控后端负责把 intent 插值、限幅、碰撞检查、看门狗、hold/damping 以及 SDK/MoveIt/LeRobot 执行频率统一起来。

## 频率分层

| 层级 | 含义 | 当前建议/事实 | 所属模块 |
| --- | --- | ---: | --- |
| `agent_decision_hz` | LLM/Agent 决策或工具调用频率，可能不稳定、有抖动 | 不作为实时控制频率 | Agent runner |
| `agent_intent_hz` | Agent EEF 意图帧频率，例如 pose delta/twist 每帧持续多久 | 当前默认 `0.1 s`，即 `10 Hz` | `armctrl eef` / `agent-flow` |
| `preview_sample_hz` | armctrl 生成仿真/预览/安全检查轨迹的采样率 | 当前默认 `50 Hz` | EEF/recipe/LeRobot preview |
| `trajectory_sample_hz` | 已通过安全门控、可交给真实后端重放的 joint trajectory 频率 | Agent 路径建议先 `50 Hz`；SysID 当前 `100 Hz` | armctrl safety layer |
| `sdk_send_hz` | 真实后端向 SDK 发送 setpoint 的频率 | 未完全实测；SysID 现有路径按 `sample_hz` 逐点发送，常用 `100 Hz` | real SDK backend |
| `controller_dt` | SDK/controller 内部周期 | 旧实现曾读取 SDK，fallback `0.002 s`；clean SysID 当前显式配置 `0.01 s` | arx5_interface / controller |
| `record_sample_hz` | 数据记录频率 | SysID 通常 `100 Hz`；Agent runtime 需由后端定义 | recorder |
| `ui_hz` | UI/可视化刷新率 | 旧 teleop UI 约 `50 Hz`，不可驱动运动时序 | UI |

这些名字需要在真实后端日志里分开记录。过去的混乱点就是把 optimizer 采样、preview 采样、SDK 发送、控制器内环、UI 刷新混成了一个“频率”。

## Agent 侧当前契约

当前 Agent 能表达的动作协议是：

- `preset.apply`：执行一个预设姿态/动作 recipe 的计划。
- `eef.pose_absolute`：给定 EEF 绝对位姿目标。
- `eef.pose_delta`：给定 EEF 增量位姿，当前更适合作为 LeRobot/Agent 统一 action。
- `eef.twist`：给定 EEF 线速度/角速度，并用 `control_period_s` 定义该 intent 的持续时间。
- `rollout.prepare`：准备 LeRobot rollout 的桥接/处理器契约。

当前 `eef.pose_delta` 和 `eef.twist` 默认 `control_period_s = 0.1`。这意味着 Agent 侧初始设计应该按 `10 Hz` 意图帧思考，而不是让 Agent 直接产生 `100 Hz` 或 `500 Hz` 命令流。

安全步长来自 `configs/x5.safe.yaml`：

- 通用 `max_translation_step_m = 0.005`。
- 通用 `max_rotation_step_rad = 0.05`。
- Xbox/manual 更保守：`0.002 m/frame`，`0.02 rad/frame`。

因此在 `10 Hz` Agent intent 下，通用上限大约对应：

- 平移速度：`0.005 / 0.1 = 0.05 m/s`。
- 姿态角速度：`0.05 / 0.1 = 0.5 rad/s`。

这只是 Agent intent gate 的名义速度，不等于 SDK 电机速度上限。真实后端仍必须做二次限幅、插值和碰撞检查。

## 真实后端应该怎么接

真实后端建议做成统一 `MotionBackend`，不要让 Agent、SysID、LeRobot 各自绕过安全门控直接打 SDK：

```text
Agent / recipe / LeRobot / SysID
  -> armctrl 统一安全空间配置
  -> preview / simulation / collision / gate
  -> checked intent or checked trajectory
  -> real MotionBackend
  -> arx5_interface SDK / MoveIt Servo / LeRobot native runtime
  -> robot
```

后端至少需要支持：

- `prepare()`：连接 SDK，读取 `controller_dt`、当前关节状态、接口和模型。
- `enter_safe_hold_or_damping()`：进入已知安全状态后才允许执行或记录。
- `execute_intent_frame()`：接收 `10 Hz` Agent EEF intent，内部插值到后端伺服频率。
- `execute_trajectory()`：接收已检查 joint trajectory，按时间戳重放。
- `hold_last_sample()`：正常结束后保持末端或关节目标，避免默认倒下。
- `damping()`：Ctrl-C、fault、watchdog timeout 都必须落到 damping。
- `read_sample()`：以记录频率采集 q/dq/current/torque/fault/timestamp。

## Agent 实时控制建议

第一版真实 Agent 控制不要追求“LLM 高频实时”。推荐：

- Agent runner 输出 `10 Hz` 的 `eef.pose_delta` 或 `eef.twist` intent。
- 后端内部用 `50 Hz` 生成局部平滑 EEF/joint trajectory，并通过 MoveIt Servo、Pink/Pinocchio IK 或 SDK Cartesian backend 执行。
- 如果最终使用 SDK joint command 重放，先以 `50 Hz` Agent 轨迹验证，再评估是否需要升到 `100 Hz`。
- 如果使用 MoveIt Servo，Agent 仍只输出低频 intent，MoveIt Servo 负责实时 servo 和 state validity。
- 如果 Agent 超过 `0.3 s` 没有新 intent，后端进入 hold；超过更长 watchdog 阈值或出现 fault，则 damping。

不要把 Agent 输出直接绑定到 `controller_dt` 或 CAN 发送周期。Agent 帧可能延迟、丢帧、重复或被工具调用阻塞，必须由真实后端吸收这些非实时性。

## SysID 与 Agent 的区别

SysID 是计划好的轨迹重放，不是 Agent 在线控制：

- SysID OED/trajectory 设计可以低频规划，但执行轨迹当前默认重采样到 `100 Hz`。
- SysID SDK run 当前按 `sample_hz` 逐点下发关节命令并记录数据。
- SysID 需要 `record_sample_hz` 与 `trajectory_sample_hz` 对齐，便于后处理和 solver。
- Agent 控制更像在线 intent streaming，需要 watchdog、hold、deadman 和局部 replanning。

因此不要用 SysID 的 `100 Hz` 直接要求 Agent 也 `100 Hz` 决策。Agent 的高层频率可以低，后端的执行和记录频率可以高。

## LeRobot 对接建议

LeRobot 侧应优先使用 processor/adapter，而不是让 policy 绕过 armctrl 安全层：

- 训练/采集可以继续使用 `lerobot-robot-arx5` / `teleoperator-arx5` 原生生态。
- Agent/LeRobot rollout 在 armctrl 中统一成 `eef.pose_delta` 或 checked joint trajectory。
- `robot_action_processor` 负责把 policy action 转成 armctrl 可审查的 EEF/joint action。
- `robot_observation_processor` 负责把 q/dq/eef/camera 等观测转成 LeRobot schema。
- 真机 rollout 前必须经过同一套 safety config、preview、collision gate 和 watchdog。

## SDK 侧需要实测的问题

真实运控会话下一步最应该先做 read-only timing doctor：

- `arx5_interface` 推荐的连续命令发送频率是多少？
- `controller.get_controller_config().controller_dt` 在 n100d/X5 上实际是多少？
- SDK 是更偏好 `set_joint_traj` 一次性交轨迹，还是 `set_joint_cmd` 定时逐点发？
- `set_joint_cmd` 是否需要 SDK timestamp lookahead？lookahead 应该是 `controller_dt * N` 还是固定 `0.04 s`？
- 100 Hz 发送时，SocketCAN/USB-CAN 的 p95/p99 jitter 是多少？
- 一边发命令一边读电流/力矩，100 Hz 记录是否稳定？
- SDK fault、过流、Ctrl-C 时 damping 请求是否能在一个控制周期内发出？

这些问题没有实测前，文档里不能声称真实 Agent 后端已经打通。

## 最小验证路线

1. `doctor`：只连接/读取 SDK 配置，不运动，输出 `controller_dt`、timestamp 单调性、状态更新频率。
2. fake backend：用同一接口回放 `50 Hz` 和 `100 Hz` 轨迹，验证 overrun/jitter 统计。
3. hold-only smoke：真机只进入 hold/damping，不走轨迹。
4. tiny motion smoke：极小关节或 EEF delta，带确认字符串、watchdog、Ctrl-C damping。
5. Agent EEF smoke：`10 Hz` `eef.pose_delta` intent，由后端插值到 `50 Hz` 或 SDK 推荐频率。
6. SysID gravity/friction smoke：只执行已经通过 preview/collision/gate 的短轨迹。
7. Fourier/SysID 大激励：必须在 smoke、仿真、碰撞、限位、人工预览都通过后再启用。

## 交接给真实运控端的接口要求

真实后端接收的输入应至少包含：

- `schema`：动作或轨迹契约版本。
- `producer`：`agent`、`recipe`、`lerobot`、`sysid`。
- `frame`：例如 `eef_link`。
- `control_period_s`：Agent intent 持续时间。
- `trajectory_sample_hz`：若输入为 joint trajectory。
- `q` / `dq` / `ddq`：若输入为轨迹。
- `gate_results`：安全空间、URDF limit、workspace/table/base keepout、collision、max step、velocity、acceleration。
- `watchdog`：missed-frame、timeout、fault landing 策略。

输出至少包含：

- `actual_send_hz`、`send_jitter_ms_p95`、`send_jitter_ms_p99`。
- `controller_dt_s`、`timestamp_policy`。
- `q_cmd`、`q_meas`、`dq_meas`、`current`、`tau`、`fault_flags`。
- `landing_mode`：`hold`、`damping`、`faulted_to_damping`。

## 当前可运行的非真机检查

```powershell
Set-Location D:\repo\Roboclaw\references\projects\armctrl-clean
.\.venv\Scripts\python.exe scripts\agent_cli_sim_experiment.py --output runs\agent-cli-large-acceptance --json
```

当前已验证过的 Agent CLI 仿真实验目标是：`movement_allowed=false`、生成 EEF/recipe/LeRobot contract、生成 HTML preview、所有 CLI step 返回 `ok`。这只能证明接口和安全预览链路可用，不能证明 SDK 真机执行频率已经满足要求。

## 给下一个会话的核心判断

真实运控端的第一性原则是：Agent 负责意图，armctrl 负责安全契约和门控，成熟后端负责实时执行。不要把 `10 Hz Agent intent`、`50 Hz preview trajectory`、`100 Hz SysID replay`、`controller_dt`、`UI 50 Hz` 混成一个频率，也不要让任一上层 producer 绕过同一套 gate 直接打 SDK。

| `1e6+` | Essentially unusable; parameters can become physically impossible |

Important lesson: rank `36/60` is not automatically a failure for a 6-DoF arm. The failure is a high condition number or weak coverage inside the identifiable base-parameter space.

# SysID Complete Acceptance Contract
## Safety And Smoothness Gates

A mathematically good trajectory can still be unsafe. Safety gates apply to the execution trajectory, not just the planned trajectory.

Required checks:

- URDF joint limits.
- Profile-specific safe joint ranges.
- `max_joint_step_rad`.
- Velocity limit.
- Acceleration limit.
- Workspace allowed boxes.
- Table and base keepout boxes.
- Pinocchio/coal or another mature collision/state-validity backend.
- Visual URDF preview.

Smoothness interpretation:

| Observation | Verdict |
| --- | --- |
| Low condition, smooth q/dq/ddq, safety pass | Good offline candidate |
| Low condition, acceleration spikes | Review interpolation/OED artifacts before hardware |
| Good planned trajectory, execution step fails | Resampling/execution frequency mismatch |
| Workspace pass but camera could hit | Add camera collision primitive; mass alone is not collision geometry |

## Solver Quality

The solver stage should report:

- Regressor shape and whether it is full or base-parameter reduced.
- Rank.
- Condition number.
- Regularization or filtering settings.
- Least-squares residuals.
- Per-joint prediction error.
- Parameter covariance or uncertainty if available.
- Whether FIGAROH, Pinocchio, or another backend was actually invoked.

A solver status of "installed but not invoked" is not a solved identification result. It is only a handoff readiness state.

`max_iterations_exceeded` is not always fatal for OED. If constraint violation is zero, the trajectory is safe, and FIGAROH base condition is already below target, the candidate can move to offline visual review. It should still be labeled honestly as "optimizer convergence requires review", not "perfectly solved".

## Physical Consistency

A low residual is not enough. The solved parameters must describe a plausible robot.

Check:

- Link masses are positive.
- Inertia matrices are positive definite or physically consistent under the chosen parameterization.
- COM is inside or near the physical link envelope.
- Payload mass and COM match the real end-effector setup.
- Link6 includes D435i payload mass if using `X5_camera.urdf`.
- No parameter has absurd magnitude relative to CAD/URDF scale.
- Repeated runs produce similar base parameters.

Reject if the solution contains negative mass, impossible inertia, or wildly inconsistent repeated estimates, even if training residual is low.

## Prediction Quality

The identified model should predict torque/current, especially on data not used for fitting.

Core equations:

```text
tau_pred = Y(q, dq, ddq) * pi
residual = tau_meas - tau_pred
```

Recommended metrics:

| Metric | Meaning |
| --- | --- |
| RMSE per joint | Absolute prediction error |
| NRMSE per joint | Error normalized by torque/current range |
| R2 / explained variance | How much signal variance the model explains |
| Validation RMSE | Error on held-out data |
| Residual correlation | Whether residuals still depend on q/dq/ddq |

Good training error with bad validation error usually means overfitting, weak excitation, bad filtering, or noisy torque/current feedback.

## Control Benefit

The final acceptance is not a spreadsheet. It is whether the robot behaves better.

Compare at least:

- SDK default parameters.
- Previous project parameters, if any.
- New identified parameter bundle.

Control acceptance observations:

| Check | Good result |
| --- | --- |
| Static hold | Lower current/torque demand in known postures |
| Multi-pose gravity compensation | Smaller holding error across safe workspace |
| Drag feel | More uniform and lighter feel, without overcompensation |
| Pose switching | No jerk, no sudden drop, no current spike |
| Repeatability | Similar result after multiple enable/disable cycles |

Never promote a new parameter bundle directly to normal operation. Use controlled rollout, A/B test, and rollback.

## Common Failure Diagnoses

| Symptom | Likely cause | Action |
| --- | --- | --- |
| `data_readiness_status=fail` | Missing/invalid data columns or timestamps | Fix recorder before solver |
| Actual q ranges near zero | Motion did not execute or gains stayed in damping | Check SDK enable/gain/state |
| Current spikes to high values | Collision, blocked motion, too large step, payload/gravity load | Stop, inspect physical setup, reduce motion |
| Regressor condition `1e9` | Excitation too small or constraints overcoupled joints | Redesign trajectory |
| IPOPT restoration/dual infeasible | Hard constraints conflict with low-condition objective | Relax or move constraints to safety gate/soft penalty |
| Execution `max_joint_step` fails | Trajectory too fast for execution rate | Increase execution rate or reduce velocity/step |
| Rank lower than expected | Missing motion dimensions or broken regressor | Inspect q/dq/ddq coverage |
| Negative masses | Ill-conditioned solve or bad torque data | Reject and improve data/OED |
| Good offline model, bad control | Torque scaling, payload mismatch, sign convention, or controller integration issue | A/B test and isolate |

## Current X5 Acceptance State

The best known offline Fourier candidate is promising but not a final parameter result:

- FIGAROH base condition: `68.43`, target passed.
- Rank: `36`, normal for the base-parameter problem.
- Execution trajectory gates: pass.
- FK/table clearance: pass with current configured frames.
- Pinocchio/coal collision: pass under current URDF geometry.
- Optimizer label: still requires offline convergence review because the mature backend did not return a clean "optimal solution found".
- Hardware state: not automatically executable; gravity and friction smoke should run first.

This means the OED design phase is in good shape. It does not mean the physical robot has already produced valid SysID parameters.

## Acceptance Checklist

Use this checklist before promoting any result.

- Data has valid timestamps, units, and required columns.
- Hardware run had no unresolved fault, damping event, collision, or overcurrent during accepted samples.
- Actual q tracks q_cmd with meaningful coverage.
- Torque/current feedback is nonzero, unsaturated, and correlated with motion/posture.
- Gravity, friction, and Fourier datasets are all represented for full dynamics.
- FIGAROH base condition is below the target for Fourier/OED.
- Solver reports base-parameter rank and condition.
- Physical consistency checks pass.
- Prediction error is acceptable on held-out data.
- Control A/B test shows improvement without unsafe behavior.
- Parameter bundle is versioned and rollback is available.

## Recommended Report Structure

A final SysID report should have:

1. Hardware setup: robot, payload, URDF, camera mass, mount, table height, CAN interface.
2. Trajectory setup: profiles, q center, amplitudes, frequencies, safety config.
3. Safety evidence: gates, collision backend, visual review, fault log.
4. Data health: sample counts, dt statistics, columns, q/q_cmd tracking.
5. Excitation evidence: ranges, rank, condition, singular values.
6. Solver evidence: backend, filtering, regression method, residuals.
7. Physical consistency: mass, inertia, COM checks.
8. Validation: held-out prediction error.
9. Control rollout: A/B result, operator notes, rollback decision.

If any section is missing, state the gap explicitly rather than hiding it behind a pass/fail summary.