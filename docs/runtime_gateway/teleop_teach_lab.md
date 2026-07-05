# Teleop / 零力拖拽示教验收说明

本文档描述当前 Teleop/Teach MVP 的真机前准备和现场验收方式。它的定位是把 `feature/subsystem` 中已经实机确认过的 Xbox 输入语义与 `zero_gravity_drag` 关节增益配置，收束到当前 `armctrl runtime gateway`，而不是恢复旧的 direct SDK/CAN 控制链。

## 架构边界

正式链路：

```text
Xbox / JSONL input
  -> armctrl teleop xbox-runtime-smoke
  -> eef_twist 或 teleop_profile runtime command
  -> live runtime readiness
  -> owner=teleop
  -> sdk_cartesian in-runtime adapter
  -> ARX5 SDK/CAN singleton
  -> result artifact
```

关键不变量：

- `arx5_sdk --serve --eef-adapter sdk_cartesian` 是唯一真实 SDK/CAN owner。
- `teleop-profile` 只切 runtime 内的 profile，不直接打开 SDK。
- `zero_gravity_drag` 是低刚度/低阻尼 teach mode，不是 passive droop。
- Xbox deadman 按住时才提交 EEF twist；deadman 松开时提交 `zero_gravity_drag`。
- `A` 键语义是恢复 `teleop` profile。
- `X` 键语义是 damping 请求，现场需谨慎使用。

## 现场命令

Terminal 1：启动长驻 runtime。

```bash
cd ~/Roboclaw/references/projects/armctrl-clean
scripts/lab_teleop_teach_smoke.sh init
scripts/lab_teleop_teach_smoke.sh start
```

预期现象：

- SDK/CAN 打开一次。
- 机械臂 recovery 到 `SAFE_CENTER` 并持续 hold。
- `runtime_session.json` 中有 `eef_adapter_manager`，且 `teleop_profile` 对 `sdk_cartesian` 可执行。

Terminal 2：检查状态。

```bash
cd ~/Roboclaw/references/projects/armctrl-clean
scripts/lab_teleop_teach_smoke.sh status before_teach
```

预期 JSON：

- `mode=hold_safe`
- `owner=null`
- `hold_fresh=true`
- `eef_adapter_manager.configured_adapters` 包含 `sdk_cartesian`

进入零力拖拽示教：

```bash
scripts/lab_teleop_teach_smoke.sh teach-on
scripts/lab_teleop_teach_smoke.sh check
```

预期现象：

- 机械臂不应主动跑轨迹。
- 人手轻推时应比正常 hold 更容易拖动。
- 它不应像 passive/damping 一样直接垂落。
- result artifact 中 `owner=teleop`、`kind=teleop_profile`、`profile=zero_gravity_drag`、`landing_mode=hold`。

恢复正常 Teleop 增益：

```bash
scripts/lab_teleop_teach_smoke.sh teleop-on
scripts/lab_teleop_teach_smoke.sh check
```

预期现象：

- EEF target 会先同步当前 pose。
- 关节增益恢复到 SDK 默认 teleop profile。

JSONL 假手柄输入预演：

```bash
scripts/lab_teleop_teach_smoke.sh xbox-jsonl
scripts/lab_teleop_teach_smoke.sh check
```

预期语义：

- 第一段事件模拟 `RB` deadman + 左摇杆前推，提交一个 `eef_twist`。
- 第二段事件模拟 deadman 松开，提交 `zero_gravity_drag`。
- 这只证明输入映射、runtime queue、owner/artifact 链路，不证明真实手柄设备、人体操作手感或运动质量。

真实 Xbox 设备输入：

```bash
ls /dev/input/by-id/
scripts/lab_teleop_teach_smoke.sh xbox-device /dev/input/eventX
scripts/lab_teleop_teach_smoke.sh check
```

操作建议：

- 先只短按 `RB` 并轻推左摇杆。
- 不要一开始测试大幅 roll/pitch/yaw。
- 松开 `RB` 后应切入 `zero_gravity_drag`，不应掉臂。

停止 runtime：

```bash
scripts/lab_teleop_teach_smoke.sh stop
```

## 验收判定

阶段 A：无硬件/fake 预演通过。

- `ARMCTRL_BACKEND=fake scripts/lab_teleop_teach_smoke.sh start`
- `teach-on` 可 queue profile command。
- `xbox-jsonl` 可 queue `eef_twist` 和 `zero_gravity_drag`。
- result artifact 可审计。

阶段 B：真实 Teach profile 通过。

- `teach-on` 后无主动运动。
- 人手可拖动，但不是 passive 掉落。
- `teleop-on` 可恢复正常 hold/servo 手感。

阶段 C：真实 Xbox 输入通过。

- deadman 按住才运动。
- deadman 松开回到 `zero_gravity_drag`。
- 整个过程中 SDK/CAN 没有被第二个 source 打开。
- 所有动作都有 runtime result artifact。

## 当前非目标

- 不实现完整 GUI。
- 不实现成熟 joint-jog controller。
- 不把 Teleop 变成独立 SDK daemon。
- 不用 JSONL smoke 证明真实手柄手感。
- 不用 `zero_gravity_drag` 替代安全 hold 或 SysID preposition。

