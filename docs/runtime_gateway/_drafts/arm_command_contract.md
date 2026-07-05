# ArmCommand Contract

## 目标

所有会产生真实运动的 source 都必须通过统一 `ArmCommand` contract 进入 runtime
gateway。Source-specific CLI 可以保留，但真实运动前必须先编译成这个共享 contract。

## 第一版 command kind

- `joint_trajectory`：带时间戳的 joint waypoints，用于 reviewed trajectory replay。
- `joint_intent`：有边界的低频 joint target 或 joint delta，面向 Agent 类 source。
- `eef_pose_delta`：有边界的末端 pose delta。
- `eef_pose_waypoints`：带时间戳的末端 pose waypoints。
- `eef_twist`：有边界的 Cartesian velocity command。

第一阶段真实硬件执行只要求支持：

- `joint_trajectory`。
- `joint_intent`。

EEF command kinds 写入 contract 是为了避免未来继续长出 ad-hoc 接口，但这不
意味着 `armctrl` 要实现本地 EEF controller。EEF 真实执行必须通过成熟 backend
边界，例如 MoveIt Servo、ARX5 SDK Cartesian controller，或者 reviewed
IK-to-joint-trajectory adapter。

## 通用字段

每个 `ArmCommand` 至少应表达：

- `schema`：contract schema version。
- `command_id`：唯一 command id。
- `source`：`sysid`、`agent`、`recipe`、`lerobot`、`teleop` 等。
- `owner`：runtime owner lease 名称。
- `kind`：command kind。
- `expected_start`：期望起点，可以是 `live_hold` 或显式 state。
- `readiness_policy`：live readiness 要求。
- `safety_policy`：需要执行的 safety gate。
- `frequency_policy`：输入频率、执行频率、record 频率的关系。
- `landing_policy`：正常结束、fault、timeout、Ctrl-C 的 landing 行为。
- `artifact_policy`：需要保存的证据字段。

## `joint_trajectory`

`joint_trajectory` 应对齐 ros2_control `JointTrajectoryController` 风格语义，而不是
随意自定义一套控制器。

阶段 1 中，SysID 的 `joint_trajectory` 已经是第一个真实硬件 golden path。它的
backend 不是替换为新的控制器，而是：

```text
ArmCommand(kind=joint_trajectory, owner=sysid)
  -> ArmRuntime trajectory_replay
  -> ArmRuntime 独占持有的 arx5_sdk backend
```

也就是说，`joint_trajectory` contract 约束的是 source 到 runtime 的语义；
ARX5 SDK 仍是阶段 1 的正式硬件 backend。禁止的是 source 在 runtime 外直接打开
SDK/CAN，而不是禁止 runtime 使用 ARX5 SDK。

应表达：

- `joint_names`。
- `points[].time_from_start_s`。
- `points[].q`。
- `points[].dq`，可选。
- `points[].ddq`，可选。
- `trajectory_sample_hz`。
- `runtime_send_hz`。
- `start_pose_policy`。
- `max_start_error_rad`。
- `max_tracking_error_rad`，可选。
- `max_tau_abs`，可选。

当前阶段真实 backend 是 `ArmRuntime` 内的 `trajectory_replay`。

## `joint_intent`

`joint_intent` 是低频、有边界的 joint setpoint intent，不是实时电机控制。

应表达：

- `q_target` 或 `q_delta`。
- `control_period_s`。
- `agent_intent_hz`。
- `runtime_send_hz`。
- `max_joint_delta_rad`。
- `max_start_error_rad`。
- `missed_intent_policy`。
- `fault_timeout_s`。

当前阶段真实 backend 是 `ArmRuntime` 的 `agent_servo` 插值/限幅/发送。

## EEF command

`eef_pose_delta`、`eef_pose_waypoints`、`eef_twist` 第一阶段只进入 contract/review
层，不承诺真机执行。

它们必须表达：

- `frame`。
- pose / waypoint / twist payload。
- `control_period_s` 或 waypoint timestamps。
- workspace / collision / joint limit safety policy。
- mature backend adapter 要求。

没有成熟 backend adapter 时，必须明确返回不可真机执行，不能退化为 heuristic 真机控制。
