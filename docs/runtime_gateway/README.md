# armctrl Runtime Gateway 文档入口

当前文档集只保留一条成熟执行主线和一个最小 Agent 可执行入口：

- `joint_trajectory`：SysID 已经基本跑通，是当前 golden path。
- `joint_intent`：Agent 侧最小真机入口，低频输入，由 runtime 负责限幅、插值、hold/release。
- `eef_pose_delta` / `eef_twist`：Agent EEF command space。当前 MVP 只允许提交到 runtime queue；没有成熟 EEF backend adapter 时必须 rejected。
- `eef_pose` / `joint_jog`：正式 command contract 已保留，但当前硬件执行必须 rejected。`eef_pose` 等待成熟 absolute pose backend；`joint_jog` 等待 teleop deadman 和 jog backend。

当前正式操作表面是三组命令：

- `armctrl motion submit ...`：给 Agent、Recipe、Teleop、SysID compiler 使用的统一 motion command surface。
- `armctrl profile list/show`：统一描述 `lab-sysid`、`lab-agent-eef`、`lab-agent-joint`、`recipe`、`teleop` 的默认 owner、backend、start pose policy 和 motion kind。
- `armctrl console catalog`：不需要 live runtime 的人类中控配置入口。它汇总 profile catalog、正式 motion submit kinds、operator surfaces 和 legacy policy。
- `armctrl console status`：给人类中控 UI / operator 面板读取的统一 live status。它包装 live runtime status、profile catalog 和 motion surface；作为 readiness 输入时会解包成 `armctrl.arm_runtime_status.v1`。

最小中控命令：

```bash
armctrl console catalog --json
armctrl console status --session-artifact "$RUN_DIR/runtime_session.json" --json
armctrl profile show lab-sysid --json
armctrl motion submit joint-intent ...
armctrl motion result --run-dir "$RUN_DIR" --json
```

## 当前有效文档

- [`sysid_integration.md`](sysid_integration.md)：当前主规范。它描述 SysID 从离线轨迹、
  lab operator CLI、runtime submit、ARX5 SDK backend 到 result artifact 的完整链路。

## 当前架构判断

SysID 这一路已经基本是新架构的第一个可用 golden path：

```text
SysID/OED 离线规划
  -> reviewed execution_trajectory.csv
  -> armctrl sysid run --adapter sdk
  -> motion surface: joint-trajectory
  -> live runtime readiness
  -> runtime queue
  -> owner=sysid
  -> ArmRuntime trajectory_replay
  -> ArmRuntime 持有 ARX5 SDK/CAN
  -> runtime result artifact / result-check
```

所以当前不是推倒重来，也不是把 ARX5 SDK 换成别的控制器。当前要做的是：

- 把已经跑通的 SysID 行为 contract 化。
- 澄清 `--adapter sdk` 的真实语义：SysID operator CLI 编译并提交 `motion submit joint-trajectory` 语义，runtime backend 是 `arx5_sdk`。
- 固化 `execution_trajectory.csv -> runtime command` 的 compiler 边界。
- 固化 `q/dq/ddq` 证据语义：`q_cmd` 必须保留；`dq_cmd` 优先保留，缺失时只能显式派生并记录；
  `ddq_cmd` 有则保留，无则标注为 missing，不伪装成 OED 原始输出。
- 串联 OED/preview/CSV/runtime result/result-check 证据链。
- 清理或标注仍可能绕过 runtime 的 legacy direct SDK path。

Agent 是控制源；joint / EEF 是 command space，不应该和 Agent 并列。当前 Agent 侧已可执行的是
joint-space runtime gateway：`joint_intent` 是最小真机入口；`joint_trajectory` 只作为 Agent
waypoint/trajectory submit 能力保留，不能抢 SysID golden path 的优先级：

```text
Agent low-frequency intent
  -> ArmCommand(kind=joint_intent, owner=agent)
  -> live runtime readiness
  -> runtime agent_servo interpolation/send
  -> release back to hold

Agent checked waypoints
  -> ArmCommand(kind=joint_trajectory, owner=agent)
  -> live runtime readiness
  -> runtime trajectory_replay
  -> release back to hold
```

正式 Agent/SysID runtime gateway 的 readiness 只接受 live runtime status 证据。operator 可以使用
`armctrl console status`，内部会解包为 `armctrl.arm_runtime_status.v1`；也可以直接使用
`armctrl runtime status`。旧的
`sdk-agent-sysid-smoke-readiness` 只能作为 bringup diagnostic 参考，不能作为正式运动准入凭证。

SysID 交付证据必须显式分三层质量标签：

- `motion_smoke_pass`：证明 runtime 接受并执行了运动，正常 release/landing，无明显危险现象。
- `runtime_quality_pass`：证明 sample count、actual send hz、jitter、tracking、q_cmd/q_meas、tau/fault evidence 达标。
- `sysid_dataset_ready`：证明该 run 的数据足以交给 SysID/OED 分析 agent 做后续参数辨识；slow/reduced smoke 不能自动宣称为 full dataset ready。

EEF / Cartesian action 当前只能生成 contract、review、fake preview。没有成熟 backend adapter
之前，不允许把 EEF action 悄悄退化成 heuristic joint 真机控制。
runtime 已接受 `eef_pose_delta` / `eef_twist` command kind，用于证明 Agent 可以表达 EEF action
并走同一个 owner/readiness/queue 入口。首选成熟后端是 MoveIt Servo；在 `moveit_servo`
adapter 未接通前，serve 端必须 rejected，并产出可审计 result artifact，不能发送任何 joint command。

现场脚本入口：

- `scripts/lab_fourier_sysid.sh`：SysID slow/reduced/full Fourier 真机流程。
- `scripts/lab_agent_runtime_smoke.sh`：Agent `joint-intent` / `joint-trajectory` / `eef-delta` runtime gateway smoke。无硬件环境可用 `ARMCTRL_BACKEND=fake` 预演，真机环境默认使用 `arx5_sdk`。

这些脚本现在应尽量只编排正式入口：`console status`、`motion submit`、`motion result`。SysID 的 `sysid run`
保留为 operator-facing compiler，因为它还负责 Fourier candidate、CSV、OED evidence 和 manifest 串联。

Agent 明天实验室前的最小预演入口：

Terminal 1:

```bash
ARMCTRL_BACKEND=fake scripts/lab_agent_runtime_smoke.sh init
ARMCTRL_BACKEND=fake scripts/lab_agent_runtime_smoke.sh start
```

Terminal 2:

```bash
scripts/lab_agent_runtime_smoke.sh status before_agent
scripts/lab_agent_runtime_smoke.sh plan
scripts/lab_agent_runtime_smoke.sh run-intent
scripts/lab_agent_runtime_smoke.sh check-intent
```

其中 `plan` 只生成 EEF/Cartesian contract 供 review；真实 smoke 运动仍是受限的
joint-space `joint_intent`，不会把 EEF action heuristic fallback 成真机关节控制。

无硬件环境的验收只证明 queue/owner/release/artifact 语义，不证明真实 timing gate。真实 timing、
tracking、jitter、hold 稳定性仍必须在实验室长驻 `--serve` runtime 下验收。

## 暂存草稿

其他模块文档已暂存到 [`_drafts/`](_drafts/)。

这些草稿只作为后续参考，不作为当前实现依据。等 SysID 主链路和 Agent joint-space
最小入口收口后，再逐个恢复 EEF、LeRobot、测试策略和通用 ArmCommand 文档。恢复时应避免每个文档各自写
一套阶段路线图，而是围绕同一个 runtime gateway 主线补充接口边界。

## 当前非目标

- 不重写 SysID/OED/Fourier 离线优化。
- 不把 `armctrl` 扩张成 MoveIt、ros2_control、LeRobot 或 ARX5 SDK controller 的替代品。
- 不做 EEF 真机执行、Teleop/Xbox streaming、LeRobot rollout 真机集成。
- 不允许真实运动绕过长驻 ArmRuntime 直接打开 SDK/CAN。
