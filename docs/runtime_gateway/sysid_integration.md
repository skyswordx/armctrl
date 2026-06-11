# SysID 集成规范

## 定位

SysID 是阶段 1 的保护对象，不是清理对象。

阶段 1 的目标不是重写 SysID，而是把今天已经跑通的 lab SysID 真机链路沉淀成
通用 `joint_trajectory` golden path：

```text
SysID reviewed execution trajectory
  -> ArmCommand(kind=joint_trajectory, owner=sysid)
  -> live runtime readiness (console status unwraps to armctrl.arm_runtime_status.v1)
  -> single owner lease
  -> runtime trajectory_replay
  -> SDK/CAN singleton backend
  -> runtime result artifact + SysID dataset evidence
  -> release back to active hold
```

一句话原则：

```text
保留 SysID 离线与 lab 证据语义，真实运动入口显式 ArmCommand 化；
保护 SysID 离线 OED/轨迹/求解，不为了 runtime 重构而碰算法层。
```

当前判断：单看 SysID 这一条，从离线轨迹规划到真机执行器执行，已经基本是新架构的
第一个可用 golden path。它的事实链路是：

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

因此阶段 1 不应把 SysID 推倒重来。剩余问题主要是接口语义和证据边界收口：

- `--adapter sdk` 这个名字容易误导为 SysID CLI 自己直连 SDK；它不再作为真实运动入口保留。
  当前代码在 parser 层拒绝 `--adapter sdk`：`sysid run` 只接受 `--adapter {fake}`。
  真实语义必须由 `sysid compile-runtime` 与 `motion submit joint-trajectory` 显式表达，
  runtime backend 为 `arx5_sdk`。
- `execution_trajectory.csv -> runtime command` 这步应显式成为 SysID runtime compiler。
- `ArmCommand(kind=joint_trajectory, owner=sysid)` / `motion_kind=joint-trajectory` 应从行为事实固化为 contract/artifact。
- OED/preview/CSV/runtime result/result-check 之间需要一个稳定 manifest 串联。
- 任何仍在 runtime 外直接打开 SDK/CAN 的真实运动路径，都应清理、禁用或标注为 diagnostic。

ARX5 SDK 下游与重构不冲突。阶段 1 的正式下游就是 `ArmRuntime` 独占持有的
ARX5 SDK backend。要淘汰的不是 ARX5 SDK，而是 source-specific CLI 绕过
runtime ownership 直接控制 SDK/CAN。

## 阶段 1 范围

阶段 1 分为四层，每层处理策略不同。

阶段 1 的交付物不是“把 SysID 全部重构完”，而是把真实运动入口收束到一个可审计、
可复现、可回归的最小闭环：

```text
reviewed SysID artifact
  -> SysID runtime compiler
  -> ArmCommand(kind=joint_trajectory, owner=sysid)
  -> live ArmRuntime queue
  -> single owner trajectory_replay
  -> runtime result + SysID dataset evidence
```

因此阶段 1 的边界可以用三句话判定：

- 离线 OED / 轨迹优化 / 仿真评审不动。
- 现场 operator 命令和 slow/reduced/full 语义保留。
- 真实运动执行从旧 SysID SDK direct path 收束到 runtime owner path。

### 1. SysID 离线层：冻结保护

以下内容不属于 runtime gateway 重构范围，阶段 1 不修改：

- OED / Fourier candidate search。
- Rank / condition / metric optimization。
- 轨迹目标函数、约束构造、优化器参数。
- `planned_trajectory.csv` 生成逻辑。
- `execution_trajectory.csv` 生成逻辑。
- q/dq/ddq 曲线与 SVG/HTML 可视化。
- `preview.html` / mesh review / collision review 产物。
- solver / quality metrics / identification report。
- SysID/OED agent 与仿真分析 agent 的上下文契约。

这些模块可以在 SysID/OED 专门任务中独立演进，但不能被 runtime gateway 重构顺手改动。
特别禁止：

- 为了适配 Agent/EEF 新入口而修改 SysID 轨迹格式。
- 为了 runtime 架构洁癖改 OED 评价指标。
- 为了减少工作量跳过 reviewed `execution_trajectory.csv`。
- 用真机 runtime 的临时需求反向污染离线优化/辨识模块。

### 2. Lab operator surface：保留流程，移除旧运动入口

今天已经跑通的 lab SysID 真机验证流程是 operator golden workflow。阶段 1 不应破坏
slow/reduced/full candidate、现场脚本、证据目录和 result-check 语义；但旧的
`sysid run --adapter sdk` 真实运动入口必须移除，避免继续给人“SysID CLI 可以自己开 SDK/CAN”
的心智模型。

阶段 1 应保留：

- `armctrl sysid compile-runtime` 作为新的 SysID runtime compiler 入口。
- `scripts/lab_fourier_sysid.sh` 现场实验流程脚本。
- slow / reduced / full Fourier candidate 的实验语义。
- `console status` 生成的 live readiness artifact；底层兼容 `runtime status`。
- `motion result` 对 owner、mode、sample count、jitter、tracking、landing 的检查；底层兼容 `runtime result-check`。
- 真机执行后的 runtime result artifact 与 SysID dataset evidence。

移除旧入口的原因不是否定今天跑通的 SysID 链路，而是区分两件事：
现场可复现实验路径应保留；真实运动的命令表面必须统一到 runtime-owned
`joint_trajectory`，不能继续保留一个名字上暗示 direct SDK replay 的入口。

这一层的保留是强约束。阶段 1 允许脚本流程稳定，但脚本内部必须只使用正式入口：

```text
armctrl runtime start
armctrl console status
armctrl sysid compile-runtime ...
armctrl motion submit joint-trajectory --compiled-command ...
armctrl motion result ...
scripts/lab_fourier_sysid.sh <init|start|status|preposition|run|check|stop>
```

`armctrl sysid compile-runtime` 是正式 compiler surface。`sysid run --adapter sdk`
不再并行承担真实运动，也不再承担迁移 payload 职责；它已从 parser 可接受 schema 中移除。
`command_surface=armctrl.motion.submit.v1` 与 `motion_kind=joint-trajectory`
才是正式路径。同一个 slow/reduced/full candidate 的真机回归必须通过新入口完成。

### 3. SysID 真机提交层：一步到位 ArmCommand 化

真实执行语义应一步到位收敛到 `ArmCommand`：

```text
armctrl sysid compile-runtime ...
  -> 读取 reviewed execution_trajectory.csv
  -> 保留/派生 q/dq/ddq policy
  -> 输出 compiled_motion_command.json
  -> 不读取 runtime session
  -> 不发送硬件命令

armctrl motion submit joint-trajectory --compiled-command ...
  -> 要求 live runtime status/readiness
  -> 提交到 live runtime queue

armctrl sysid run ... --adapter sdk ...
  -> parser invalid choice: --adapter {fake}
  -> 不进入真实运动 handler
  -> 不读取 runtime session 做准入
  -> 不生成或提交 runtime command
  -> 不发送 SDK/CAN command
  -> 不写旧入口 manifest
```

当前代码已经选择 parser-level removed 方向：`sysid run` 的 `--adapter` 只允许 `fake`，
`--adapter sdk` 由 argparse 直接拒绝，不再读取 readiness、规划轨迹、写旧 manifest
或提交 runtime command。阶段 1 要做的是把这个选择同步到
文档、脚本和 release/status 表面，而不是重做 SysID 离线层。

`armctrl sysid compile-runtime` 与 `armctrl motion submit joint-trajectory --compiled-command`
是正式入口。它们必须用同一 slow/reduced/full candidate 证明执行证据可复现。

阶段 1 中 `armctrl sysid run ... --adapter sdk ...` 不再有真实职责：

- parser 直接报 `invalid choice: sdk`。
- `sysid run` 帮助信息只展示 `--adapter {fake}`。
- 不再校验真实运动 confirm。
- 不再写旧入口 manifest。
- 不再产出会被误读为 runtime evidence 的迁移 payload。
- `armctrl.sysid_run` 模块只保留 offline/fake runner；ARX5 SDK joint backend 位于
  `armctrl.arx5_sdk_joint_runtime`，作为 runtime/diagnostic backend 使用，不再挂在 SysID run 模块下。

正式替代入口是 `sysid compile-runtime` 与
`motion submit joint-trajectory --compiled-command`。旧入口因此也不再负责：

- 直接打开 SDK/CAN。
- 读取 live runtime status 作为准入。
- 生成或读取 SysID planning/review artifact。
- 编译或提交 `joint_trajectory` command。
- 在 CLI 进程里循环发送关节命令。
- 自己实现 hold / damping / Ctrl-C landing。
- 用历史 artifact 证明 readiness。
- 私有化 trajectory replay、jitter、tracking、fault 判断。

换句话说，`sysid run --adapter sdk` 在阶段 1 后既不是兼容执行入口，也不是迁移护栏命令；
它是已移除的 schema。真正执行必须显式走 compiler + motion submit。

### 4. SysID 真机执行层：抽象为通用 joint_trajectory executor

SysID 真机执行层应成为通用 `joint_trajectory` executor 的第一个生产用例。

应抽象和复用：

- `owner=sysid` 的 single owner lease。
- 从 `HOLD_SAFE` / live `q_hold` 出发的 readiness guard。
- reviewed high-frequency execution trajectory replay。
- q/dq 可选输入的执行语义。
- SDK/CAN singleton backend。
- timing / jitter / tracking / tau / fault / landing artifact。
- 正常结束回 active hold。
- fault / Ctrl-C / timeout 进入可预测 landing。

不应保留为 SysID-only 特例：

- SysID 私有 SDK/CAN 打开逻辑。
- SysID 私有 readiness 规则。
- SysID 私有 owner/fault/landing 规则。
- SysID 私有 trajectory replay executor。
- SysID 私有 result-check 语义。

## 阶段 1 迁移顺序

阶段 1 应按以下顺序实施，避免一边改接口一边破坏今天能跑的实验链路。

### Step 0：冻结基线

冻结今天已经验证过的 SysID lab 证据：

- 当前可运行的 `scripts/lab_fourier_sysid.sh`。
- slow/reduced/full candidate 路径和语义。
- 关键真机 result artifact。
- 关键 preview / HTML / CSV / dataset evidence。
- 当前 `runtime result-check` 阈值和输出字段。

这些证据作为阶段 1 回归基线。后续任何“更干净”的入口都必须能解释它和这批证据的关系。

### Step 1：定义 SysID 使用的最小 `ArmCommand`

阶段 1 不需要一次性实现所有 command kind。SysID 只需要最小 `joint_trajectory`：

```yaml
schema: armctrl.arm_command.v1
command_id: <uuid>
source: sysid
owner: sysid
kind: joint_trajectory
expected_start:
  policy: live_hold
  max_start_error_rad: <float>
frequency_policy:
  trajectory_sample_hz: <float>
  runtime_send_hz: <float>
  resampling_policy: none | runtime_interpolate | reject_if_mismatch
trajectory:
  joint_names: [...]
  points:
    - time_from_start_s: 0.0
      q: [...]
      dq: [...]        # optional but preferred for SysID evidence
      ddq: [...]       # optional
safety_policy:
  safe_config: <path or digest>
  max_joint_step_rad: <float>
  max_velocity_rad_s: <float>
  max_accel_rad_s2: <float>
landing_policy:
  normal: hold
  fault: damping
artifact_policy:
  trajectory_artifact: execution_trajectory.csv
  q_cmd: preserved
  dq_cmd: preserved | derived_finite_difference
  ddq_cmd: preserved | missing
  record_q_cmd: true
  record_q_meas: true
  record_dq_cmd: true
  record_dq_meas: true
  record_tau_meas: true
```

这里的重点不是 YAML 格式本身，而是字段语义必须固定：SysID 给 reviewed
joint trajectory，runtime 负责 owner、readiness、发送频率、tracking 和 landing。

### Step 2：实现 SysID runtime compiler

新增或明确一个内部编译步骤：

```text
SysID reviewed artifacts
  -> load execution_trajectory.csv
  -> validate columns and sample count
  -> align start to live q_hold if policy allows
  -> preserve q/dq/ddq when present
  -> derive dq only when artifact explicitly缺失 and policy records fallback
  -> leave ddq missing when absent; do not fake it as OED output
  -> emit ArmCommand(kind=joint_trajectory, owner=sysid)
```

编译器只做格式转换、起点对齐和证据链接，不重新做 OED、不重新规划、不改变 candidate 的
辨识语义。

当前实现中，`armctrl sysid compile-runtime` 这个 runtime compiler 会把
`execution_trajectory.csv` 编译为 `joint_trajectory` command，并在 command 与 queued
manifest 中写入 `artifact_policy`：

- `q_cmd=preserved`：`q_cmd_*` 是 SysID/OED 输出到执行轨迹的核心数据，必须原样保留。
- `dq_cmd=preserved`：当 CSV 自带完整 `dq_cmd_*` 列时使用。
- `dq_cmd=derived_finite_difference`：当 CSV 缺失 `dq_cmd_*` 时，为 runtime 平滑插值和证据对齐显式派生。
- `ddq_cmd=preserved`：当 CSV 自带完整 `ddq_cmd_*` 列时使用。
- `ddq_cmd=missing`：当 CSV 缺失 `ddq_cmd_*` 时只记录缺失，不伪造加速度。

这一步不能反向污染 OED/离线规划层。也就是说，派生出来的 `dq_cmd` 是 runtime compiler
证据策略，不代表原始 candidate 或 OED solver 曾经输出过速度；缺失的 `ddq_cmd` 也不能在后处理里被当作原始优化证据。

### Step 3：移除旧 operator CLI 的真实运动语义

`armctrl sysid run ... --adapter sdk ...` 不再保持真实运动入口。正式提交流程统一为：

```text
sysid compile-runtime
  -> compiler
  -> ArmCommand / motion_kind=joint-trajectory
  -> submit runtime queue
  -> return queued command artifact
```

如果 runtime 不存在、heartbeat stale、当前 mode 不是 `hold_safe`、已有 owner active、
或者 live q_meas 与 q_hold 超阈值，则直接拒绝，不允许退回旧 SDK direct execution。
正式 SysID runtime gateway 接受 `console status` 或 `runtime status` 产生的 live runtime status evidence；
`console status` 只是人类中控表面，内部会解包为 `armctrl.arm_runtime_status.v1`。
正式 SysID runtime gateway 不接受历史 `sdk-agent-sysid-smoke-readiness`、startup recovery artifact
或 tiny-motion artifact 作为准入凭证；这些 artifact 只能用于 bringup diagnostic 和故障复盘。

### Step 4：复用 motion result / runtime result-check

SysID 不再拥有独立 result-check 语义。operator-facing 检查入口应优先使用 `armctrl motion result`，
底层复用 runtime result-check 语义。阶段 1 的检查应统一落在 runtime result 上：

- owner 必须是 `sysid`。
- mode 必须是 `trajectory_replay`。
- sample count 必须符合 candidate 预期。
- actual send hz / jitter 必须在阈值内。
- tracking error 必须在阈值内。
- landing 必须符合 policy。
- fault flags 必须可解释。
- result artifact 必须可链接回 SysID planning/review evidence。

### Step 5：保留 lab script，但减少脚本“聪明程度”

`scripts/lab_fourier_sysid.sh` 应继续作为 operator workflow，但它不应承载架构逻辑。
脚本只负责：

- 创建 run dir。
- 启动/停止 runtime。
- 调用 console status。
- 调用 preposition。
- 调用 sysid run。
- 调用 motion result。
- 打印下一步 operator 提示。

脚本不应自己判断复杂 readiness、不应自己修补轨迹、不应绕过 runtime queue。

## SysID 阶段 1 接口边界

### 输入

SysID 阶段 1 的输入分为三类。

离线输入：

- reviewed Fourier candidate。
- `planned_trajectory.csv`。
- `execution_trajectory.csv`。
- `trajectory_preview.json`。
- `preview.html`。
- OED / rank / condition / quality metrics。

现场输入：

- live runtime session artifact。
- live console status / runtime status readiness artifact。
- operator confirm 字符串。
- safe config。
- candidate 类型：slow / reduced / full。

控制输入：

- trajectory sample hz。
- runtime send hz。
- expected sample count。
- max start error。
- max tracking error。
- landing policy。

### 输出

SysID 阶段 1 的输出必须同时服务两类消费者。

给 operator / runtime 的输出：

- queued command artifact。
- runtime result artifact。
- result-check artifact。
- fault / landing summary。
- 下一步 gate 提示。

给 SysID/OED 分析 agent 的输出：

- planning/review artifact 索引。
- runtime execution artifact 索引。
- q_cmd/q_meas 对齐数据。
- dq_cmd/dq_meas 对齐数据。
- tau_meas。
- tracking error summary。
- timing/jitter summary。
- candidate 语义：slow / reduced / full。
- 是否可用于后续辨识的质量判定。

## 频率与轨迹语义

SysID 阶段 1 必须显式区分四个频率：

- `trajectory_sample_hz`：`execution_trajectory.csv` 的采样频率。
- `runtime_send_hz`：runtime 向 SDK/backend 发送 setpoint 的频率。
- `actual_send_hz`：真实执行中测得的发送频率。
- `record_hz`：用于 SysID dataset/evidence 的记录频率。

阶段 1 不允许把它们混成一个值。推荐策略：

- 如果 `trajectory_sample_hz == runtime_send_hz`：按 reviewed samples replay，不重采样。
- 如果 `trajectory_sample_hz < runtime_send_hz`：runtime 可做 time-based interpolation，但 artifact
  必须记录 interpolation policy。
- 如果 `trajectory_sample_hz > runtime_send_hz`：默认拒绝或显式降采样，不能静默丢点。
- 如果 candidate 自带 dq/ddq：保留并记录；runtime 是否使用由 backend 能力决定。
- 如果 candidate 缺 dq/ddq：可以派生用于 evidence，但必须标记为 derived，不可伪装为 OED 原始输出。

SysID 的辨识质量依赖实际执行轨迹，而不是只依赖 planning trajectory。因此 artifact 必须保留
planned、commanded、measured 三条链路。

## Safety 与 admission gate

SysID 阶段 1 的 safety 不应私有化，应走 runtime gateway 的分层 gate。

提交前静态 gate：

- joint limit。
- safe joint range。
- max joint step。
- velocity / acceleration。
- candidate 起点与 live hold pose 的关系。
- safe config digest 或路径记录。

提交时 live gate：

- runtime heartbeat fresh。
- runtime mode 为 `hold_safe`。
- no owner active。
- q_meas fresh。
- q_meas 接近 q_hold。
- q_hold 接近 command expected_start。
- watchdog / landing policy 已配置。

执行时 runtime gate：

- owner heartbeat。
- send jitter。
- tracking error。
- fault flags。
- tau/current 异常。
- Ctrl-C / process exit landing。

任何 gate 失败时，SysID CLI 的职责是返回 rejected/blocked artifact，而不是绕过 runtime
改用旧 SDK direct path。

## 数据质量分级

阶段 1 应把“能动”和“可用于辨识”分开判断。

`motion_smoke_pass`：

- runtime 接受 command。
- 轨迹执行完成。
- 无 fault。
- landing 正确。
- 肉眼无明显危险现象。

`runtime_quality_pass`：

- sample count 符合预期。
- actual send hz 接近 runtime send hz。
- jitter 在阈值内。
- tracking error 在阈值内。
- q_cmd/q_meas 数据完整。
- tau_meas 数据完整或明确说明不可用。

`sysid_dataset_ready`：

- planning/review evidence 完整。
- runtime execution evidence 完整。
- commanded/measured 时间轴可对齐。
- full 或约定 candidate 执行完整。
- tracking error、jitter、fault、tau 异常未破坏辨识假设。
- 数据导出路径可交给 SysID/OED 分析 agent。

一个 run 可以 `motion_smoke_pass=true`，但 `sysid_dataset_ready=false`。这不是失败含糊，
而是必要分级：实验室安全链路和辨识数据质量不是同一个验收层。

## 失败归因规则

阶段 1 需要避免所有问题都被归咎于“轨迹不好”或“runtime 不稳”。失败应先按证据归类：

- `readiness_failure`：runtime 不 live、heartbeat stale、mode 不对、owner occupied。
- `admission_failure`：起点不匹配、safe gate 失败、sample hz/send hz policy 不允许。
- `runtime_timing_failure`：actual_send_hz、jitter、queue latency、first send latency 异常。
- `tracking_failure`：q_cmd/q_meas 超阈值，但 timing 正常。
- `trajectory_quality_failure`：offline candidate 自身 step/velocity/acceleration/smoothness 不适合真机。
- `backend_failure`：SDK/CAN fault、模式切换失败、底层接口异常。
- `operator_abort`：Ctrl-C、stop、deadman、人工中止。

每类失败都必须有 artifact 字段支撑，而不是只写一句自然语言 reason。

## Slow / Reduced / Full 的阶段 1 语义

这三个不是普通参数，而是 lab 风险分级和回归分级。

`slow`：

- 目标：安全和丝滑 smoke。
- 作用：确认 runtime、preposition、trajectory replay、hold、artifact 链路正常。
- 验收重点：没有异响/明显顿挫，tracking 和 jitter 在阈值内，结束回 hold。
- 不能用于证明最终辨识效果，只证明“真机链路可安全进入下一层”。

`reduced`：

- 目标：短时辨识 smoke。
- 作用：用较短 execution trajectory 验证 SysID 数据记录和 runtime result evidence。
- 验收重点：sample count、q/dq/tau、tracking、landing 与预期一致。
- 可以用于检查数据格式和 runtime evidence，但不应作为 OED full candidate 的最终质量结论。

`full`：

- 目标：完整候选轨迹真机执行。
- 作用：验证 OED candidate 是否能形成可用于后续辨识分析的数据。
- 验收重点：执行完整性、数据质量、tracking、jitter、fault flags、tau 异常。
- 只有 full 通过 `sysid_dataset_ready`，才应进入后续参数辨识分析。

阶段 1 重构不能删除这三层语义。可以改善脚本或入口，但必须保留同等风险分级。

slow / reduced / full 的幅度、时长、降速策略可以为了 lab 安全临时调整，但必须记录：

- 原始 candidate 来源。
- 修改了哪些幅度/时长/速度参数。
- 修改原因。
- 修改后是否仍保留 OED 语义。
- 修改后数据是否只用于 smoke，还是可用于正式辨识。

## Readiness 与起点语义

阶段 1 必须以 live runtime readiness 为准。

允许：

- `console status` 或 `runtime status` 生成 live readiness artifact。
- `sysid run` 使用 live `q_hold` / `q_meas` 推导 effective start。
- candidate trajectory 以 live hold pose 对齐后再 submit。

禁止：

- 用历史 startup recovery artifact 证明当前仍在 hold。
- 把 operator 手写 `SAFE_CENTER` 当作当前真实起点。
- 跳过 live runtime status 直接 submit。
- 在 runtime 外重新打开 SDK/CAN 读取状态并自行执行。

起点处理必须遵守以下优先级：

1. 当前 live `q_meas` 是事实来源。
2. runtime `q_hold` 是受控悬停目标。
3. SysID candidate 的 q_center / start pose 是计划参考。
4. operator 环境变量里的 `SAFE_CENTER` 只能作为配置输入，不能覆盖 live 状态。

如果 live `q_meas` 不接近 `q_hold`，SysID 不应自己 recovery；应拒绝并要求 runtime
先 recovery/hold。SysID 的第一条轨迹点也不应假设机械臂“历史上到过 SAFE_CENTER”。

## Artifact 要求

SysID 阶段 1 必须保留两类证据。

SysID planning/review evidence：

- `planned_trajectory.csv`。
- `execution_trajectory.csv`。
- `trajectory_preview.json`。
- `preview.html`。
- q/dq/ddq 曲线。
- safety gate summary。
- OED / rank / condition / quality metrics。

Runtime execution evidence：

- `ArmCommand` 或等价 command payload，必须带 `command_surface=armctrl.motion.submit.v1`
  和 `motion_kind=joint-trajectory`。
- runtime session id。
- owner。
- mode。
- `sample_count`。
- `trajectory_sample_hz`。
- `runtime_send_hz`。
- `actual_send_hz`。
- send jitter。
- tracking error。
- `q_cmd` / `q_meas`。
- `dq_cmd` / `dq_meas`。
- `tau_meas`。
- fault flags。
- landing mode。
- queue latency / first send latency / execution elapsed。

后续 SysID/OED 分析 agent 应主要消费 planning/review evidence 和 runtime execution
evidence，而不是从操作者聊天记录里恢复上下文。

阶段 1 还应新增一个 SysID run manifest，把两类证据串起来：

```yaml
schema: armctrl.sysid_runtime_manifest.v1
profile: fourier_multisine
candidate_label: slow | reduced | full
candidate_source: <path>
planning_artifacts:
  planned_trajectory: <path>
  execution_trajectory: <path>
  preview_html: <path>
  metrics: <path>
runtime_artifacts:
  session: <path>
  status_before: <path>
  command: <path>
  result: <path>
  result_check: <path>
quality:
  motion_smoke_pass: <bool>
  runtime_quality_pass: <bool>
  sysid_dataset_ready: <bool>
  failure_class: <string|null>
notes:
  amplitude_scale: <float|null>
  time_scale: <float|null>
  interpolation_policy: <string>
```

这个 manifest 是交接给 SysID/OED 分析 agent 的主入口。

## 阶段 1 验收标准

阶段 1 不算完成，除非满足：

- `armctrl sysid run ... --adapter sdk ...` 不再可用于 lab 真实运动；parser 层拒绝 `sdk`，
  `sysid run` 只接受 `--adapter {fake}`。
- `armctrl sysid compile-runtime` 输出的 compiled command 标注 `command_surface=armctrl.motion.submit.v1`、
  `motion_kind=joint-trajectory`，说明 SysID 已收束到正式 motion surface。
- `scripts/lab_fourier_sysid.sh` 或同等脚本仍能复现 slow/reduced/full。
- 内部真实执行已经通过 runtime owner path，而不是 SysID 直接开 SDK/CAN。
- 同一 reviewed candidate trajectory 可以通过新 `joint_trajectory` path 执行。
- `motion result` 能验证 owner=`sysid`、mode=`trajectory_replay`、sample count、jitter、
  tracking、landing。
- 不再产生旧入口 manifest；执行质量证据只能来自 compiled command、motion submit、
  runtime result 与 motion result。
- SysID 离线 OED/轨迹/求解模块未被 runtime 重构改动。

更具体地，阶段 1 验收分三档：

### A. 架构验收

- `sysid run --adapter sdk` 不再直接打开 SDK/CAN 做真实运动。
- 所有真实运动都通过 live runtime session 提交。
- owner lease 互斥生效。
- readiness 只接受 live console/runtime status 解包后的 runtime status。
- `motion result` 消费 runtime result，而不是 SysID 私有 result。

### B. Lab 回归验收

- slow 可以从 live hold 出发并回到 active hold。
- reduced 可以完整执行并产出 runtime evidence。
- full 可以完整执行或给出明确失败归因。
- `scripts/lab_fourier_sysid.sh` 仍能指导 operator 复现。
- Ctrl-C / fault / timeout landing 可预测。

### C. 数据交付验收

- manifest 能链接 planning/review evidence 和 runtime execution evidence。
- q_cmd/q_meas、dq_cmd/dq_meas、tau_meas 可被后续 agent 读取。
- full 数据是否可用于辨识有明确 `sysid_dataset_ready` 判定。
- 如果使用 slow/reduced/缩放版轨迹，只能按 manifest 中的质量标签使用，不能误当正式 full OED 数据。

## Legacy SysID SDK direct path 处理规则

如果现有 SysID 代码中存在 runtime 外直接打开 SDK/CAN 的路径，处理规则如下：

- 如果是今天 lab golden path 的必要 operator interface：阶段 1 不删除，优先改内部实现为 runtime submit。
- 如果是 bringup/diagnostic 命令：保留但明确标注为 diagnostic，不进入正式控制链路。
- 如果是旧实验路径且会绕过 runtime ownership：禁用真实运动或删除。

禁止为了架构洁癖破坏今天的可复现实验链路。任何删除或重命名 SysID lab interface
的操作，都必须先提供同等可复制命令，并用同一 slow/reduced/full candidate 完成回归验证。

## 阶段 1 不做事项

为了避免再次漂移，SysID 阶段 1 明确不做：

- 不重写 OED/Fourier 搜索。
- 不重写 SysID 仿真评审。
- 不新增自研 joint trajectory controller。
- 不把 Agent/EEF 的接口需求塞进 SysID CSV 格式。
- 不因为某个 lab 脚本不优雅就删除它。
- 不为每个 CLI 参数组合写大量测试。
- 不把 slow/reduced 的 smoke 成功宣传成 full dataset ready。
- 不允许真实运动 fallback 到 legacy SDK direct path。

阶段 1 的工程美学应该是“保护已验证真机链路，把真实运动责任迁到 runtime”，而不是
“趁机把 SysID 全部重做一遍”。
