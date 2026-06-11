# armctrl Runtime Gateway 文档入口

当前文档集只保留一条成熟执行主线和一个最小 Agent 可执行入口：

- `joint_trajectory`：SysID 已经基本跑通，是当前 golden path。
- `joint_intent`：Agent 侧最小真机入口，低频输入，由 runtime 负责限幅、插值、hold/release。
- `eef_pose_delta` / `eef_twist` / `eef_pose`：Agent EEF command space。正式目标是 continuous-owner EEF servo；没有成熟 in-runtime backend adapter 时必须 rejected。
- `joint_jog`：正式 command contract 已保留，等待 teleop deadman 和 jog backend。当前硬件执行必须 rejected。

当前正式操作表面是三组命令：

- `armctrl motion compile joint-trajectory`：给 Agent、Recipe、preposition 等非 SysID source 使用的通用 joint trajectory compiler。它把 joint target / joint waypoints 规范化为 `q_points / dq_points / ddq_points / sample_hz / send_hz` compiled command。
- `armctrl motion submit ...`：给 Agent、Recipe、Teleop、SysID compiler 使用的统一 motion command surface。
- `armctrl profile list/show`：统一描述 `lab-sysid`、`lab-agent-eef`、`lab-agent-joint`、`recipe`、`teleop` 的默认 owner、backend、start pose policy 和 motion kind。
- `armctrl console catalog`：不需要 live runtime 的人类中控配置入口。它汇总 profile catalog、正式 motion submit kinds、operator surfaces 和 legacy policy。
- `armctrl console status`：给人类中控 UI / operator 面板读取的统一 live status。它包装 live runtime status、profile catalog 和 motion surface；作为 readiness 输入时会解包成 `armctrl.arm_runtime_status.v1`。

最小中控命令：

```bash
armctrl console catalog --json
armctrl console status --session-artifact "$RUN_DIR/runtime_session.json" --json
armctrl profile show lab-sysid --json
armctrl motion compile joint-trajectory --source agent ...
armctrl sysid compile-runtime --execution-trajectory <execution_trajectory.csv> ...
armctrl motion submit joint-trajectory --compiled-command <compiled_motion_command.json> ...
armctrl motion submit joint-intent ...
armctrl motion result --run-dir "$RUN_DIR" --json
```

## 当前有效文档

- [`sysid_integration.md`](sysid_integration.md)：当前主规范。它描述 SysID 从离线轨迹、
  lab operator CLI、runtime submit、ARX5 SDK backend 到 result artifact 的完整链路。
- [`agent_eef_control.md`](agent_eef_control.md)：Agent EEF 收束规范。它描述 EEF Servo Path / EEF Planned Path、禁止断 SDK owner 切换、以及等待真机验证前的 contract 要求。
- [`goal_agent_eef_ready_for_lab.md`](goal_agent_eef_ready_for_lab.md)：可直接粘贴给 goal 模式的执行提示词，用于无硬件环境继续收束到待真机验证状态。

## 当前架构判断

SysID 这一路已经基本是新架构的第一个可用 golden path：

```text
SysID/OED 离线规划
  -> reviewed execution_trajectory.csv
  -> armctrl sysid compile-runtime
  -> armctrl motion submit joint-trajectory --compiled-command ...
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
- 移除 `sysid run --adapter sdk` 作为 CLI schema 的一部分；`sysid run` 现在只接受 `--adapter {fake}`，正式链路必须显式走 `sysid compile-runtime` + `motion submit joint-trajectory`，runtime backend 才是 `arx5_sdk`。
- 固化 `execution_trajectory.csv -> runtime command` 的 compiler 边界。
- 固化 `q/dq/ddq` 证据语义：`q_cmd` 必须保留；`dq_cmd` 优先保留，缺失时只能显式派生并记录；
  `ddq_cmd` 有则保留，无则标注为 missing，不伪装成 OED 原始输出。
- 串联 OED/preview/CSV/runtime result/result-check 证据链。
- 清理或标注仍可能绕过 runtime 的 legacy direct SDK path。

Agent 是控制源；joint / EEF 是 command space，不应该和 Agent 并列。当前 Agent 侧已可执行的是
joint-space runtime gateway：`joint_intent` 是最小真机入口；`joint_trajectory` 必须接入
SysID 已经打磨出来的下游 golden path，而不是在脚本或 Agent 里另起一套 q-point 生成/执行链路：

```text
Agent low-frequency intent
  -> ArmCommand(kind=joint_intent, owner=agent)
  -> live runtime readiness
  -> runtime agent_servo interpolation/send
  -> release back to hold

Agent checked waypoints
  -> armctrl motion compile joint-trajectory --source agent
  -> compiled_motion_command.json(q_points/dq_points/ddq_points + artifact_policy)
  -> armctrl motion submit joint-trajectory --compiled-command ...
  -> live runtime readiness
  -> joint_trajectory_safety: segment delta / velocity guard + artifact
  -> runtime trajectory_replay
  -> release back to hold
```

Agent `joint_intent` / `joint_trajectory` 是可见 joint-space 能力，不是裸 q-point replay。
Agent joint target / joint waypoints 先编译成统一 JointTrajectory contract；缺失 `dq/ddq`
时必须显式派生并写入 `artifact_policy`。手写 `owner=agent` trajectory 默认使用
`max_joint_velocity_rad_s` guard；现场脚本还会显式传入 `max_joint_segment_delta_rad`。
`joint_intent` 使用低频 target + runtime 50 Hz smoothstep
整形，必须同时检查总跨度和平均速度，避免“肌无力 smoke”和过激 step 两种失败模式。
`joint_intent` 的 command/result artifact 还必须写出等价短 horizon `q_points/dq_points/ddq_points`，
让 Agent 低频 intent 和 SysID/Recipe 的 `joint_trajectory` 共享同一套轨迹质量审计语义。
SysID `--compiled-command` 不默认套 Agent guard，因为它的安全与辨识质量证据来自
`sysid compile-runtime`、offline review、`q/dq/ddq` artifact policy 和 result-check。

Runtime 执行前必须重新计算 Agent `joint_intent` 和 `joint_trajectory` safety。即使命令文件
已经进入 runtime queue，serve 进程也不能只信 submit 阶段写入的 artifact；如果 `owner=agent`
的 pending command 缺失 safety artifact，runtime 会回退到默认 Agent 速度阈值并在执行前
拒绝过快 intent/trajectory。这个复核不替代 SysID compiler/review/result-check 证据链。

`max_tracking_error_rad` 是执行质量和数据可用性 gate，不是在线急停阈值。普通 tracking lag
必须记录到 result artifact，并由 `armctrl motion result` / `runtime result-check` 判定
`runtime_quality_pass` 或 `sysid_dataset_ready` 是否失败；它不能默认在运动中途触发 damping，也不能把
hold-capable 的受控状态降级成 passive/damping 掉臂。只有 fault flags、watchdog/deadman timeout、
发送异常、torque limit 等硬故障才应进入 damping。

正式 Agent/SysID runtime gateway 的 readiness 只接受 live runtime status 证据。operator 可以使用
`armctrl console status`，内部会解包为 `armctrl.arm_runtime_status.v1`；也可以直接使用
`armctrl runtime status`。旧的
`sdk-agent-sysid-smoke-readiness` 只能作为 bringup diagnostic 参考，不能作为正式运动准入凭证。

SysID 交付证据必须显式分三层质量标签：

- `motion_smoke_pass`：证明 runtime 接受并执行了运动，正常 release/landing，无明显危险现象。
- `runtime_quality_pass`：证明 sample count、actual send hz、jitter、tracking、q_cmd/q_meas、tau/fault evidence 达标。
- `sysid_dataset_ready`：证明该 run 的数据足以交给 SysID/OED 分析 agent 做后续参数辨识；slow/reduced smoke 不能自动宣称为 full dataset ready。

EEF / Cartesian action 当前已支持 `eef_pose_delta`、`eef_twist`、`eef_pose` 进入同一个
runtime owner/readiness/queue 入口。没有成熟 backend adapter 之前，不允许把 EEF action
悄悄退化成 heuristic joint 真机控制。首选成熟后端是 MoveIt Servo；在 `moveit_servo`
adapter 未接通前，submit 端必须 `blocked` 且不得写 pending command；serve 端仍保留
二次 rejected 防线，不能发送任何 joint command。
absolute `eef_pose` 只能作为 mature adapter 的 pose reference handoff；必须带
`adapter_live_reference_limit` 证据，不能被当作一帧大步 joint/Cartesian jump。

当前代码层已经有 runtime 内部 EEF adapter manager seam：`execute_pending_runtime_commands`
使用 primary runtime backend 持有 owner/hold/watchdog，同时可通过 `eef_backends` registry
把 EEF command 分发给声明的 mature adapter。result artifact 必须区分 `runtime_backend`
和 `eef_adapter`。真机阶段仍不得自动为 EEF command 另开一个 SDK/CAN owner；没有配置 adapter
时应明确 `blocked` / `rejected`，不能留下“queued 但无 result”的现场假象。

现场脚本入口：

- `scripts/lab_fourier_sysid.sh`：SysID slow/reduced/full Fourier 真机流程。
- `scripts/lab_agent_runtime_smoke.sh`：Agent `joint-intent` / `joint-trajectory` / `eef-delta` runtime gateway smoke。无硬件环境可用 `ARMCTRL_BACKEND=fake` 预演，真机环境默认使用 `arx5_sdk`。
  fake 预演会在 `runtime start --serve` 时注册 `--eef-adapter moveit_servo`，因此 `run-eef`
  会被 runtime serve 消费并产出 EEF result artifact，而不是停留在 queued 状态。真实
  `arx5_sdk` runtime 若尚未暴露 `eef_adapter_manager.eef_command_executable=true`，
  `run-eef` 应在提交前被脚本 blocked；这是正确安全门，不是真机 EEF 验收失败。

这些脚本现在只应编排正式入口：`console status`、`sysid compile-runtime`、
`motion submit`、`motion result`。SysID 的 `sysid run --adapter sdk`
不再作为 lab 入口或迁移护栏保留；parser 会以 `--adapter {fake}` 的 invalid choice 拒绝它。
operator 必须使用显式 compiler + motion surface。Fourier candidate、CSV、OED evidence 和 manifest
串联必须由 `sysid compile-runtime` 与正式 `motion submit` 链路承担。

CLI 迁移表：

| 旧入口 | 新入口 | 状态 |
| --- | --- | --- |
| `armctrl sysid run ... --adapter sdk` | `armctrl sysid compile-runtime` + `armctrl motion submit joint-trajectory --compiled-command ...` | removed from parser；`sysid run` 只接受 `--adapter {fake}` |
| `armctrl runtime submit-trajectory` | `armctrl motion submit joint-trajectory` | legacy alias |
| `armctrl runtime submit-intent` | `armctrl motion submit joint-intent` | legacy alias |
| `armctrl runtime submit-eef` | `armctrl motion submit eef-delta/eef-twist` | legacy alias |
| `armctrl runtime result-check` | `armctrl motion result` | legacy alias |
| `armctrl sysid sdk-preflight/doctor/hold-damping/arm-session` | `armctrl console status` / diagnostic checklist | read-only diagnostic only |
| `armctrl sysid sdk-jog-real/recover-startup-real/tiny-motion-execute-real` | `armctrl runtime start` / `armctrl motion submit ...` | hardware diagnostic only; may open SDK/CAN directly and is not a formal control path |

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
scripts/lab_agent_runtime_smoke.sh run-eef
scripts/lab_agent_runtime_smoke.sh check-eef
```

其中 `plan` 只生成 EEF/Cartesian contract 供 review。fake `run-eef` 用
MoveIt Servo-style adapter 验证 command surface、owner lease、adapter registry 和 result artifact；
它不证明真实 EEF 运动质量，也不会把 EEF action heuristic fallback 成真机关节控制。

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
