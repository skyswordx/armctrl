# armctrl 机械臂控制模块总 Wiki

> 状态：设计草案
> 适用仓库：`armctrl`
> 当前路径：`~/Roboclaw/references/projects/armctrl`
> 目标硬件：ARX X5 机械臂
> 当前优先级：先完成硬件验证，再实现服务化控制层

## 1. 结论

`armctrl` 应作为 `Roboclaw` 的独立机械臂运控模块。
它不负责视觉算法、任务级推理和 OpenClaw 对话流程。
它只负责把 ARX X5 的底层能力整理成稳定、可测试、可审计的控制入口。

推荐架构是：

```text
OpenClaw / Roboclaw Task Orchestrator
  -> armctrl Python interface
      -> arx5ctl
          -> arx5d
              -> ARX5 SDK / arx5-interface
                  -> CAN / EtherCAT-CAN
                      -> ARX X5 firmware
```

其中 `Python interface`（Python 接口）面向 OpenClaw 和 Roboclaw 业务代码。
`arx5ctl` 是给人、脚本和 agent 用的薄控制面。
`arx5d` 是常驻控制进程，独占 SDK 控制器对象，维护模式、状态、安全检查、命令生命周期和事件日志。
`ARX5 SDK`（方舟机械臂 SDK）继续承担和固件通信、控制器、求解器、轨迹接口、重力补偿与模型相关能力。
`armctrl` 的代码重点是封装、校验、编排和调试入口，不能重写 SDK 已经实现得更好的 IK、动力学、轨迹覆盖、CAN 通信和控制器逻辑。

## 2. Deep Research 依据

本轮采用 `Deep Research`（深度调研）方式，先读本地源码和文档，再核对外部主来源。
结论主要依据下列资料：

| 来源 | 可信度 | 采用内容 |
|---|---:|---|
| `vendor/real_stanford_arx5_sdk/python/arx5_interface.pyi` | 高 | SDK 暴露的类型、控制器、求解器、轨迹和状态接口 |
| `vendor/real_stanford_arx5_sdk/python/examples/` | 高 | 关节控制、末端控制、遥操作、轨迹调度、IK 和动力学示例 |
| `vendor/real_stanford_arx5_sdk/README.md` | 高 | 安全提示、pip wheel、CAN 设置、示例入口 |
| `docs/bringup/bringup_code_assessment.md` | 高 | 当前 bringup 代码的定位和长期结构建议 |
| `docs/interviewed/reference_projects_and_sjtu_roboclaw_notes.md` | 中高 | LeRobot、ARX5 插件、参考项目边界 |
| `Roboclaw/docs/architecture/OVERVIEW.md` | 高 | OpenClaw、本地服务层、设备层边界 |
| `Roboclaw/docs/architecture/openclaw-arx5-d435i-integration-design.md` | 高 | Roboclaw 中 ARX5 Control Service 的系统定位 |
| https://github.com/real-stanford/arx5-sdk | 高 | 上游 SDK 公开说明 |
| https://pypi.org/project/arx5-interface/ | 高 | `arx5-interface==0.1.2` wheel 分发状态 |
| https://huggingface.co/docs/lerobot/integrate_hardware | 高 | LeRobot 自定义硬件接口模式 |
| https://pypi.org/project/arx5-common/ | 中 | ARX5 LeRobot 插件共用工具包信息 |
| https://pypi.org/project/lerobot-robot-arx5/ | 中 | ARX5 follower robot 插件信息 |
| https://pypi.org/project/lerobot-teleoperator-arx5/ | 中 | ARX5 teleoperator 插件信息 |

当前未做实机复测。
涉及硬件动作的结论只作为软件架构和执行计划，不等同于已通过真机验证。

## 3. 需求整理

### 3.0 本次修正判断

前一版蓝图方向是对的，但执行计划还需要加三条硬约束。
第一，所有底层控制能力优先复用 SDK。
自研代码只做协议、配置、安全、状态机、日志和用户输入适配。
第二，bringup 阶段已经暴露过“脚本层逻辑继续膨胀、局部重写 SDK 行为”的问题，后续不能沿着这个方向扩展。
第三，调试入口要正式纳入设计：Xbox 手柄、零重力/被动拖动、重力补偿、增益 profile、轨迹调度和阻抗/柔顺实验都应进入维护或调试模式，而不是混进 agent 默认能力。

### 3.1 对 OpenClaw 的接口

OpenClaw 不应直接 import `arx5_interface`。
它应通过 `armctrl.client` 或 `arx5ctl` 访问控制能力。
这一层需要提供：

- `health()`：检查 daemon、SDK、硬件连接、当前模式。
- `state()`：返回关节、末端、夹爪、错误码、最近命令。
- `move_eef()`：提交末端目标或短轨迹。
- `run_recipe()`：执行观察位、预抓取位、撤退位等受约束动作。
- `gripper()`：控制夹爪开合。
- `cancel()`：取消当前命令。
- `damping()`：进入阻尼或被动安全态。

返回值必须是结构化 `JSON`（JavaScript 对象表示法）兼容对象，不把 SDK 原生日志直接透传给 OpenClaw。

### 3.2 内部运行组件

`arx5d` 是唯一能创建和持有 SDK 控制器的进程。
所有真实写命令都必须经过它。
它内部建议拆成以下组件：

| 组件 | 职责 |
|---|---|
| `SessionManager` | 创建 SDK controller，绑定 `model`、`interface_name`、配置和日志级别 |
| `ModeManager` | 管理 `idle`、`cartesian`、`joint`、`maintenance`、`damping`、`fault` |
| `StateStore` | 缓存关节状态、末端状态、夹爪状态、活动命令和错误 |
| `SafetyGuard` | 检查工作空间、速度、步长、关节限位、夹爪范围、超时和模式权限 |
| `MotionRequestBuilder` | 只做目标规范化、timestamp、profile 选择和 SDK 调用准备，不重写 IK 或轨迹控制 |
| `CommandExecutor` | 串行执行命令，处理取消、超时、失败和最终状态 |
| `RecipeRegistry` | 管理观察位、预抓取位、撤退位、复位位等命名动作 |
| `DebugProfileManager` | 管理重力补偿、低增益、阻尼、柔顺/阻抗实验等调试 profile |
| `EventLogger` | 记录命令、状态迁移、安全拒绝、人工确认和异常 |

### 3.3 业务逻辑和控制需求

基础控制路径：

```text
EEF target
  -> frame / unit / schema normalization
  -> mode check
  -> safety precheck
  -> SDK IK / trajectory request
  -> command execution
  -> state polling
  -> completion / abort / fault
```

`EEF`（末端执行器）目标优先使用末端位姿表示。
SDK 已确认提供 `Arx5CartesianController.set_eef_cmd()`、`set_eef_traj()`、`Arx5Solver.inverse_kinematics()`、`multi_trial_ik()`、`forward_kinematics()`、`inverse_dynamics()`、`Gain`、`reset_to_home()`、`set_to_damping()` 等接口。
`armctrl` 不应自己实现 IK、动力学或轨迹插值控制器。
它只应在 SDK 调用前做输入校验、profile 选择、日志记录和错误规整。

当前本地类型文件没有显示“直接按末端力反馈控制”的高级 API。
因此“用力控过去”不能写成已确认能力。
第一阶段应实现为命名 `compliance profile`（柔顺参数配置）：限制速度、步长和增益，必要时调用阻尼态；真正的末端力控、扭矩控制或阻抗控制必须单独上机验证。

### 3.4 VLA 输出

`VLA`（视觉语言动作模型）不应直接输出 SDK 命令。
推荐边界是：

```text
VLA output
  -> Roboclaw task layer validates intent
  -> Vision / calibration converts target to base-frame pose
  -> armctrl receives EEFGoal
  -> SafetyGuard checks
  -> MotionPlanner executes
```

`armctrl` 只接受已经落到机械臂基座坐标系下的末端目标。
如果目标来自相机，必须携带标定版本、置信度和来源。
低置信目标只能进入观察位、靠近位或人工确认流程，不能直接执行接触动作。

### 3.5 遥操作输出

`teleoperation`（遥操作）入口可以来自 Xbox 手柄、键盘、SpaceMouse、leader arm 或 LeRobot。
它不应绕过 `arx5d`。
推荐使用两类输入：

- 增量末端速度：适合 Xbox 手柄、键盘和 SpaceMouse，必须做频率限制和低通滤波。
- 绝对末端点：适合 leader arm 或策略输出，必须做每步最大位移裁剪。

遥操作默认进入 `cartesian` 模式。
退出时必须进入 `damping` 或安全停止状态。

### 3.6 Xbox 手柄调试入口

Xbox 手柄应作为正式调试输入，而不是一次性脚本。
建议第一版使用 Linux `evdev`（Linux 输入事件）读取 `/dev/input/event*`，保留 `pygame`（跨平台游戏输入库）作为后备方案。
手柄输入层只产生归一化 `TeleopCommand`，不直接调用 SDK。

建议默认映射：

| 输入 | 输出 |
|---|---|
| 左摇杆 | 末端 x / y 增量 |
| LT / RT | 末端 z 下降 / 上升 |
| 右摇杆 | yaw / pitch 或 roll / yaw，具体由 profile 选择 |
| LB | deadman enable，未按下不发送运动 |
| RB | 低速精细模式 |
| A | 切换 `damping` |
| B | `cancel` 当前命令 |
| X | 夹爪关闭 |
| Y | 夹爪打开 |
| Start + A | 进入调试 profile，必须二次确认 |

手柄调试必须有四个保护：deadman、输入超时自动阻尼、最大步长裁剪、模式白名单。
不得允许手柄直接修改 `kp / kd / gravity_compensation`。
这类操作只能通过命名 debug profile 触发，并写入事件日志。

## 4. SDK 能力映射

| 需求 | SDK 已见接口 | `armctrl` 应补的部分 |
|---|---|---|
| 状态读取 | `get_joint_state()`、`get_eef_state()`、`get_timestamp()` | 统一状态模型、缓存、错误码 |
| 回零 | `reset_to_home()`、`get_home_pose()` | 命令生命周期、超时、状态确认 |
| 阻尼 | `set_to_damping()` | 安全态语义、恢复条件 |
| 关节控制 | `Arx5JointController`、`set_joint_cmd()`、`set_joint_traj()` | 权限、限位、维护模式 |
| 末端控制 | `Arx5CartesianController`、`set_eef_cmd()`、`set_eef_traj()` | 末端目标模型、插值、工作空间检查 |
| IK / FK | `Arx5Solver.inverse_kinematics()`、`multi_trial_ik()`、`forward_kinematics()` | IK 失败处理、候选选择、日志 |
| 模型力矩 | `inverse_dynamics()`、`ControllerConfig.gravity_compensation` | 是否进入业务主链路需实机验证 |
| 柔顺 / 增益 | `Gain`、`set_gain()`、`get_gain()` | 命名 profile、权限和人工确认 |
| 校准 | `calibrate_gripper()`、`calibrate_joint()` | 维护模式，不对 agent 默认开放 |
| 轨迹调度 | `set_eef_traj()`、`set_joint_traj()`、SDK 内部 `interpolator_.override_traj` | 构造目标序列和 timestamp，不重写轨迹控制器 |
| 重力补偿 | `ControllerConfig.gravity_compensation`、`inverse_dynamics()` | 只作为启动前配置或 debug profile，不运行时随意切换 |
| 零重力 / 被动拖动 | `set_to_damping()`、低/零 `Gain`、`shutdown_to_passive` | 命名调试模式，必须实机验证后才能宣称“零重力” |
| 遥操作 | `keyboard_teleop.py`、`spacemouse_teleop.py`、`teach_replay.py` | 输入适配层、统一安全执行入口 |

### 4.1 复用优先规则

执行计划生成代码时必须遵守下面的规则：

| 规则 | 说明 |
|---|---|
| SDK 已有能力不重写 | IK、FK、动力学、轨迹覆盖、控制器、CAN 通信、增益设置优先调用 SDK |
| 自研代码只做边界 | 协议模型、配置加载、安全检查、状态机、日志、权限、输入设备适配由 `armctrl` 实现 |
| fake 只模拟协议 | fake adapter 只用于测试命令生命周期，不伪造真实控制算法 |
| wrapper 保持薄 | adapter 只包装 SDK 调用和异常，不改写 SDK 的运动语义 |
| 硬件逻辑必须可旁路测试 | 无硬件测试不 import SDK，不打开 CAN，不创建真实 controller |
| 调试能力显式命名 | 重力补偿、低增益、阻尼、被动拖动、阻抗实验都必须是 profile，不允许散落在脚本参数里 |

禁止事项：

- 不自己写 CAN 通信。
- 不自己写 IK / FK。
- 不自己写重力补偿力矩。
- 不复制 SDK 内部插值器。
- 不在 CLI、Xbox 输入层或 OpenClaw 工具层创建 SDK controller。
- 不把 bringup 脚本里的临时逻辑提升为长期 API。

### 4.2 调试模式和高级能力分级

调试能力应分成三级：

| 等级 | 能力 | 入口 | 默认开放 |
|---|---|---|---:|
| L1 | `damping`、状态读取、小步长末端 jog、夹爪 jog | Xbox / CLI | 是，需 deadman |
| L2 | 重力补偿启动配置、低增益被动拖动、轨迹调度对比 | CLI / debug profile | 否，需人工确认 |
| L3 | 阻抗/柔顺实验、力矩相关实验、校准、增益修改 | maintenance profile | 否，需上机测试记录 |

`gravity_compensation` 是 `ControllerConfig` 字段，通常应在 controller 创建前配置。
运行中临时切换可能需要重建 controller 或进入维护流程，不能做成手柄快捷键。
`set_to_damping()` 和低/零 `Gain` 可以用于被动拖动或类似零重力的调试体验，但当前文档不能把它表述成已经验证的“真零重力模式”。
阻抗控制如果只是通过 `kp / kd` 和轨迹 profile 近似实现，应命名为 `compliance profile`，不要命名成已确认的 SDK 原生阻抗控制。

## 5. 运行时分层

### 5.1 对外层

`armctrl.client` 是 OpenClaw / Roboclaw 用的 Python 包。
它只处理请求模型、连接、错误解析和返回值。
它不持有 SDK controller。

建议接口形态：

```python
from armctrl.client import ArmctrlClient

client = ArmctrlClient(endpoint="http://127.0.0.1:8765")
state = client.state()
client.move_eef(target_pose=[x, y, z, roll, pitch, yaw], gripper=0.04)
```

### 5.2 控制面

`arx5ctl` 是命令行入口。
它用于人工调试、agent 工具调用和脚本自动化。

推荐命令：

```bash
arx5ctl health --json
arx5ctl state --json
arx5ctl mode --json
arx5ctl move-eef --pose '[x,y,z,r,p,y]' --gripper 0.04 --json
arx5ctl recipe run go-observe --json
arx5ctl gripper --width 0.04 --json
arx5ctl damping --json
arx5ctl cancel --command-id <id> --json
```

### 5.3 守护进程

`arx5d` 是唯一硬件会话持有者。
第一版可以用 Python 实现。
协议优先使用本地 `HTTP`（超文本传输协议）和 JSON；如果后续需要更低开销，可换成 `UDS`（Unix 域套接字）或 `gRPC`（远程过程调用协议）。
协议字段必须先稳定，内部实现可以后换。

## 6. 命令模型

### 6.1 请求模型

```json
{
  "command_id": "cmd_20260422_0001",
  "type": "move_eef",
  "frame": "base",
  "target_pose": [0.32, 0.0, 0.24, 0.0, 1.57, 0.0],
  "gripper": 0.04,
  "profile": "safe_default",
  "source": "openclaw",
  "timeout_s": 5.0,
  "metadata": {
    "calibration_id": "handeye_2026_04_xx",
    "confidence": 0.82
  }
}
```

### 6.2 响应模型

```json
{
  "ok": true,
  "command_id": "cmd_20260422_0001",
  "state": "completed",
  "mode": "cartesian",
  "result": {
    "eef_pose": [0.32, 0.0, 0.24, 0.0, 1.57, 0.0],
    "gripper": 0.04
  },
  "safety": {
    "workspace": "passed",
    "delta": "passed",
    "mode": "passed"
  },
  "error": null
}
```

### 6.3 命令状态

| 状态 | 说明 |
|---|---|
| `received` | 已接收，未检查 |
| `rejected` | 参数或安全检查失败 |
| `accepted` | 已通过检查，排队执行 |
| `running` | 正在执行 |
| `completed` | 正常结束 |
| `cancelled` | 被取消 |
| `timeout` | 超时停止 |
| `faulted` | SDK、通信或安全异常 |

## 7. 安全策略

所有运动命令至少经过以下检查：

- `model` 必须匹配实机，不能把 `L5`、`X5` 混用。
- `interface_name` 必须来自配置，不由 agent 自由传入。
- 末端目标必须在工作空间内。
- 单步末端位移、姿态变化和夹爪变化必须有限制。
- 关节目标必须在 `RobotConfig.joint_pos_min / joint_pos_max` 内。
- 速度不能超过 `RobotConfig.joint_vel_max` 和 profile 限制。
- 高级模式必须要求人工确认或维护权限。
- VLA 低置信目标不得直接进入接触动作。
- 遥操作必须有死区、滤波、频率限制和退出阻尼。
- 任何异常都要优先进入 `damping` 或 `fault`。

SDK README 明确提醒：SDK 内部限位并不足以覆盖所有危险情况，尤其在奇异位形或输入噪声较大时，调用方仍需先做安全检查。
所以 `SafetyGuard` 是主系统必需组件，不是可选封装。

## 8. Recipe 设计

agent 默认不直接拿到高自由度运动能力。
推荐先定义少量 recipe：

| recipe | 用途 | 默认开放 |
|---|---|---:|
| `go_home_safe` | 回到安全初始位 | 是 |
| `go_observe` | 移动到观察位，配合 D435i 采样 | 是 |
| `go_pregrasp` | 到达抓取前位姿 | 是 |
| `retreat_safe` | 从目标附近撤退 | 是 |
| `reset_task_posture` | 任务失败后回到中性位 | 是 |
| `surface_approach_soft` | 低速柔顺靠近 | 否，需验证 |
| `calibrate_gripper` | 夹爪校准 | 否，维护模式 |
| `calibrate_joint` | 关节校准 | 否，维护模式 |

Recipe 应版本化。
每个 recipe 需要声明输入、坐标系、最大速度、超时、允许模式、失败回退和验收标准。

## 9. 与 Roboclaw 的边界

`Roboclaw` 继续负责：

- OpenClaw 工具包装和任务编排。
- D435i 视觉桥接。
- eye-in-hand 标定治理。
- VLA 输出解释。
- 任务级成功/失败判断。

`armctrl` 负责：

- ARX5 控制会话。
- 末端、关节、夹爪动作。
- 安全检查。
- 命令状态。
- 运动 recipe。
- 遥操作和 LeRobot 兼容入口的控制侧适配。

视觉坐标转换不放进 `armctrl`。
`armctrl` 只消费基座坐标系下的目标位姿，并记录目标来源和标定版本。

## 10. LeRobot 关系

LeRobot 的价值在数据采集、遥操作、训练和策略评测。
它不替代 `arx5d`。

已有 `arx5-common`、`lerobot-robot-arx5`、`lerobot-teleoperator-arx5` 可作为参考或后续依赖。
它们覆盖 follower / teleoperator 插件的一部分工作，但没有覆盖 agent-safe daemon、recipe、统一安全状态机和 OpenClaw 控制面。

推荐做法：

```text
LeRobot policy / teleoperator
  -> armctrl lerobot_compat
      -> same core control layer
          -> arx5d
              -> ARX5 SDK
```

这样 LeRobot、OpenClaw 和人工 CLI 共用同一个安全执行入口。

## 11. 推荐仓库结构

当前仓库还没有正式 `src/armctrl` 包。
后续建议结构如下：

```text
armctrl/
  pyproject.toml
  src/armctrl/
    __init__.py
    client.py
    protocol/
      models.py
      errors.py
    daemon/
      app.py
      session.py
      executor.py
      state_store.py
    adapters/
      arx5/
        sdk.py
        solver.py
        fake.py
    safety/
      guard.py
      limits.py
      profiles.py
    planning/
      eef_goal.py
      trajectory_request.py
    recipes/
      registry.py
      builtin.py
    cli/
      arx5ctl.py
    teleop/
      xbox.py
      keyboard.py
      spacemouse.py
      mapping.py
    lerobot_compat/
      robot.py
      teleoperator.py
  tests/
    unit/
    integration/
  configs/
    x5.safe.yaml
  docs/
    bringup/
    interviewed/
```

`scripts/bringup/` 如果后续恢复，应只保留硬件验证脚本，不继续扩成主架构。

## 12. 执行计划

### 12.0 Agent 实现守则

实现前先读取本文件、`README.md`、`vendor/real_stanford_arx5_sdk/python/arx5_interface.pyi` 和 SDK 示例。
代码生成必须遵守以下约束：

- SDK 是底层控制能力来源，`armctrl` 只实现协议、配置、安全、状态机、日志、输入设备适配和 OpenClaw 控制面。
- 不重写 SDK 的 IK、FK、动力学、轨迹覆盖、CAN 通信、controller 创建、增益设置和固件交互。
- SDK adapter 只能是薄边界：延迟导入 `arx5_interface`、转换数据结构、规整异常、记录调用结果。
- `FakeAdapter` 只模拟协议、状态和生命周期，不模拟真实机械臂运动学、动力学、轨迹控制或力控。
- CLI、OpenClaw 工具层、Xbox 输入层和 LeRobot 兼容层都不能创建 SDK controller，也不能直接调用 SDK。
- 高级控制能力先作为 debug 或 maintenance profile，不进入 agent 默认动作集合。
- 每个阶段必须先有无硬件测试，再做上机验证；没有验证的模式不能在文档或代码中宣称安全可用。

### 12.1 工程基线

目标：先建立可维护的 Python 项目边界，避免后续代码散落成脚本集合。

- 创建/修改文件：`pyproject.toml`、`src/armctrl/__init__.py`、`tests/`、`configs/x5.safe.yaml`、`docs/README.md`。
- 必须复用的 SDK 能力：本阶段不调用 SDK，只记录 SDK commit 和导入验证命令。
- 禁止 agent 自研的内容：不写临时 bringup 脚本代替包结构，不把 `.venv`、本机 CAN 配置或硬件日志纳入 Git。
- 单元测试/无硬件验证：`uv run pytest`、`uv run ruff check`；若还未启用 ruff，先用 `python -m compileall src tests`。
- 硬件验证：只做环境导入检查，不运动。
- 退出条件：`armctrl` 可作为包安装，协议层测试不 import `arx5_interface`，`git submodule status --recursive` 记录清楚。

### 12.2 协议模型和错误模型

目标：先固定 OpenClaw、CLI、daemon 和 teleop 共用的数据契约。

- 创建/修改文件：`src/armctrl/protocol/models.py`、`src/armctrl/protocol/errors.py`、`src/armctrl/protocol/enums.py`、`tests/unit/test_protocol_models.py`。
- 必须复用的 SDK 能力：只引用 SDK 公开概念名，如 `EEFState`、`JointState`、`Gain`，不 import SDK。
- 禁止 agent 自研的内容：不在协议层编码 IK 结果、轨迹插值结果或控制器内部状态。
- 单元测试/无硬件验证：请求、响应、状态、错误码可 JSON 序列化；非法枚举和缺字段能被拒绝。
- 硬件验证：不做。
- 退出条件：`MoveEEFRequest`、`RunRecipeRequest`、`TeleopCommand`、`DebugProfileRequest`、`CommandResponse` 和 `RobotState` schema 稳定。

### 12.3 SDK 适配层

目标：把真实 SDK 调用集中到唯一边界，其他层只能依赖 adapter 协议。

- 创建/修改文件：`src/armctrl/adapters/base.py`、`src/armctrl/adapters/arx5/sdk.py`、`src/armctrl/adapters/arx5/fake.py`、`tests/unit/test_arx5_adapter_fake.py`。
- 必须复用的 SDK 能力：`Arx5CartesianController`、`Arx5JointController`、`Arx5Solver`、`set_eef_cmd()`、`set_eef_traj()`、`set_joint_cmd()`、`set_joint_traj()`、`set_to_damping()`、`reset_to_home()`、`set_gain()`、`get_gain()`、`ControllerConfig.gravity_compensation`。
- 禁止 agent 自研的内容：不实现 IK、FK、逆动力学、插值器、轨迹覆盖、CAN 读写、controller 重连策略的隐式重试。
- 单元测试/无硬件验证：fake adapter 覆盖状态读取、命令接受、命令失败、异常映射；SDK adapter import 必须延迟到运行时。
- 硬件验证：只在维护环境中验证 `health`、`state`、`damping`，不做运动。
- 退出条件：除了 `adapters/arx5/sdk.py` 和 `daemon/session.py`，全仓不直接 import `arx5_interface`。

### 12.4 安全层和 profile 配置

目标：任何运动、维护和调试命令进入执行器前都经过可测试安全检查。

- 创建/修改文件：`src/armctrl/safety/limits.py`、`src/armctrl/safety/profiles.py`、`src/armctrl/safety/guard.py`、`tests/unit/test_safety_guard.py`。
- 必须复用的 SDK 能力：只把 SDK 的控制模式能力映射成权限和 profile，不复制 SDK 控制逻辑。
- 禁止 agent 自研的内容：不在安全层生成轨迹，不用启发式绕过 SDK 限位，不给 agent 开放任意 `kp/kd/gravity_compensation` 参数。
- 单元测试/无硬件验证：工作空间、关节范围、单步位移、速度、频率、deadman、模式权限、VLA 置信度、维护命令权限均有测试。
- 硬件验证：用 plan-only 或 fake adapter 验证拒绝路径；真机只观察拒绝命令不会运动。
- 退出条件：默认 profile 只允许 L0/L1；重力补偿、低增益、阻尼、被动拖动、柔顺实验必须有命名 debug profile 和维护权限。

### 12.5 末端目标构造和 SDK 调用准备

目标：把 VLA、遥操作、CLI 或 recipe 的目标点转换成 SDK 可接收的命令请求。

- 创建/修改文件：`src/armctrl/planning/eef_goal.py`、`src/armctrl/planning/trajectory_request.py`、`tests/unit/test_eef_goal.py`。
- 必须复用的 SDK 能力：需要可达性预检查时调用 `Arx5Solver.multi_trial_ik()`；执行轨迹时调用 `set_eef_traj()` 或 `set_eef_cmd()`。
- 禁止 agent 自研的内容：不写自研 IK，不复制 SDK 内部插值器，不写自研模型控制，不在 Python 层实现低级力控。
- 单元测试/无硬件验证：单点目标、多点目标、timestamp、frame、profile、safety precheck 和 solver 失败路径均可测试。
- 硬件验证：先跑 plan-only，再做毫米级小步长 `move-eef`，全程保留 `cancel` 和 `damping`。
- 退出条件：构造层只输出 SDK 调用所需数据和结构化错误，不直接控制硬件。

### 12.6 daemon、session 和命令执行器

目标：用常驻进程独占 SDK controller，统一状态、模式、命令生命周期和错误处理。

- 创建/修改文件：`src/armctrl/daemon/session.py`、`src/armctrl/daemon/executor.py`、`src/armctrl/daemon/state_store.py`、`src/armctrl/daemon/app.py`、`tests/unit/test_command_lifecycle.py`。
- 必须复用的 SDK 能力：controller 创建、damping、reset、状态读取和运动调用都经 adapter 进入 SDK。
- 禁止 agent 自研的内容：不在 daemon 外创建 controller，不让多个进程同时持有硬件控制权，不用线程绕过命令状态机。
- 单元测试/无硬件验证：`received -> accepted -> running -> completed/rejected/timeout/cancelled/faulted` 全路径可在 fake adapter 下测试。
- 硬件验证：先验证 daemon 启动、状态读取、阻尼、取消，再验证最小运动。
- 退出条件：所有入口都通过 executor；异常会进入 `faulted` 或结构化失败状态，硬件默认回到阻尼或安全停止。

### 12.7 client、CLI 和 OpenClaw 接入

目标：给人、脚本、OpenClaw 和 agent 提供稳定控制面。

- 创建/修改文件：`src/armctrl/client.py`、`src/armctrl/cli/arx5ctl.py`、`tests/unit/test_client_cli_contract.py`；Roboclaw 侧建议新增 `src/openclaw/communication/armctrl_client.py`、`src/openclaw/controller/arm_motion_controller.py`。
- 必须复用的 SDK 能力：不直接复用 SDK；只复用 daemon 暴露的能力。
- 禁止 agent 自研的内容：CLI、OpenClaw 工具层和 agent 工具不得 import SDK，不得绕过 daemon 发运动命令。
- 单元测试/无硬件验证：`health --json`、`state --json`、`move-eef --plan-only`、`run-recipe`、`cancel`、`damping` 的 schema 固定。
- 硬件验证：只验证 `health`、`state`、`damping`、`cancel` 和小步长 `move-eef`。
- 退出条件：OpenClaw 只看见 `health/state/move_eef/run_recipe/gripper/cancel/damping`，VLA 输出必须先经任务层和坐标转换。

### 12.8 recipe 层

目标：让 agent 优先调用受约束、可回退、可审计的动作。

- 创建/修改文件：`src/armctrl/recipes/registry.py`、`src/armctrl/recipes/builtin.py`、`tests/unit/test_recipe_registry.py`、`configs/x5.safe.yaml`。
- 必须复用的 SDK 能力：recipe 展开后仍走 `move_eef`、`set_eef_traj()`、`damping`、`cancel` 等既有路径。
- 禁止 agent 自研的内容：不把 recipe 写成自由 Python 脚本，不允许 recipe 修改底层 controller 参数。
- 单元测试/无硬件验证：`go_observe`、`go_pregrasp`、`retreat_safe`、`open_gripper`、`close_gripper` 可查找、可校验、可超时、可失败回退。
- 硬件验证：先验证只读和 plan-only，再验证低速安全位姿切换。
- 退出条件：recipe 必须声明 profile、输入范围、超时、失败动作和日志字段。

### 12.9 Xbox 遥操作和调试 profile

目标：把 Xbox 手柄做成正式调试入口，用同一安全链路支持学习、调试和低速遥操作。

- 创建/修改文件：`src/armctrl/teleop/xbox.py`、`src/armctrl/teleop/mapping.py`、`src/armctrl/teleop/filters.py`、`tests/unit/test_xbox_mapping.py`、`tests/unit/test_teleop_rate_limit.py`、`tests/unit/test_debug_profiles.py`。
- 必须复用的 SDK 能力：手柄层只生成 `TeleopCommand`；实际运动仍走 daemon、SafetyGuard、adapter 和 SDK。
- 禁止 agent 自研的内容：手柄层不调用 SDK，不切换 `gravity_compensation`，不直接写 `Gain`，不创建“零重力”控制器；ARX5 专属的 gain profile 应集中在 adapter 侧单独模块维护。
- 单元测试/无硬件验证：deadman、死区、低通滤波、轴映射、步长限制、输入超时、退出阻尼、profile 权限均可测试。
- 硬件验证：按 `state -> damping -> deadman -> 低速 jog -> 超时阻尼 -> cancel` 顺序验证。
- 退出条件：A/B/X/Y 等按键只能请求命名 debug profile；重力补偿和低增益实验必须进入维护流程并记录结果。

### 12.10 LeRobot 兼容和数据采集

目标：复用 `armctrl` 控制面支持 LeRobot action、teleoperator 和数据采集，不让策略绕过安全层。

- 创建/修改文件：`src/armctrl/lerobot_compat/robot.py`、`src/armctrl/lerobot_compat/teleoperator.py`、`tests/unit/test_lerobot_mapping.py`。
- 必须复用的 SDK 能力：LeRobot action 只映射到 `move_eef`、`run_recipe` 或受限 teleop 命令。
- 禁止 agent 自研的内容：不复制 LeRobot ARX5 插件已有的硬件驱动逻辑，不让 policy 直接操作 SDK controller。
- 单元测试/无硬件验证：action shape、坐标系、单位、速率限制、失败回退和状态采样 schema 可测试。
- 硬件验证：等 core control layer 稳定后再做低速策略回放。
- 退出条件：LeRobot 兼容层可以替换输入来源，但不能替换 SafetyGuard、daemon 和 adapter。

### 12.11 硬件验证记录

目标：在 N100D 和实机上逐步验证最小安全链，所有结论都有命令和观察记录。

验证顺序：

1. `uv` 环境、`arx5_interface` 导入和 ROS2 环境共存。
2. CAN 接口识别和只读状态查询。
3. `arx5d` fake adapter 生命周期。
4. `arx5d` 真 SDK adapter 的 `health/state/damping/cancel`。
5. 默认不运动的 plan-only 验证。
6. 毫米级 `move-eef` 小步长验证。
7. recipe 的 plan-only 和低速安全位姿验证。
8. Xbox deadman、输入超时、退出阻尼和低速 jog。
9. `gravity_compensation`、低增益、阻尼、被动拖动、柔顺 profile 在维护模式下逐项验证。

每次硬件验证记录：

- commit、配置文件、SDK commit、系统环境、CAN 接口名。
- 命令、输出 JSON、日志路径、人工观察结果。
- 是否触发安全拒绝、故障、阻尼、取消或急停。
- 是否允许进入下一阶段；不允许时记录阻塞原因。

## 13. 近期优先级

短期不要先做完整 LeRobot 插件，也不要先做复杂力控。
当前已完成第一条可测试主链：

- `src/armctrl/protocol`：请求、响应、状态、错误码和 JSON 序列化。
- `src/armctrl/adapters/arx5/fake.py`：无硬件 fake adapter。
- `src/armctrl/adapters/arx5/sdk.py`：延迟导入 `arx5_interface` 的 SDK adapter。
- `src/armctrl/safety`：末端、遥操作和 debug profile 安全检查。
- `src/armctrl/daemon/executor.py`：命令执行器和 deadman 松开进入阻尼。
- `src/armctrl/cli/arx5ctl.py`：`health/state/damping/cancel/move-eef/debug-profile/teleop-xbox`。
- `src/armctrl/teleop/xbox.py`：Linux input event 和 JSONL 回放。

下一阶段建议按以下顺序推进：

1. 在 N100D 上运行 fake adapter 和 JSONL 回放验证。
2. 用 `arx5ctl health/state --adapter sdk` 做只读 SDK 验证。
3. 用 `arx5ctl damping/cancel --adapter sdk` 验证安全态切换。
4. 找到 Xbox 的 `/dev/input/by-id/` 事件设备，先用 fake adapter 读真实手柄事件。
5. 进入实机低速 Xbox jog，只允许 `RB` deadman、毫米级末端 jog、松开后阻尼。
6. 维护模式下逐项验证 `reset_home`，并验证 `teleop <-> zero_gravity_drag <-> damping` 切换。
7. 稳定后再补 `src/armctrl/client.py`、OpenClaw 接入和 daemon 服务化。

这个顺序先固定边界、测试和安全策略，再逐步引入真实硬件和高级控制。

## 14. 未确认事项

- SDK 是否提供稳定的末端力控或阻抗控制 API：本地类型文件未确认。
- Xbox 手柄第一版使用 `evdev` 还是 `pygame`：建议先在 N100D 上用 `evtest` 或等价工具确认设备事件。
- 低增益 + 重力补偿能否满足“零重力”调试体验：必须实机验证，未验证前只能称为被动/低增益调试 profile。
- `arx5d` 第一版使用 HTTP 还是 UDS：建议先 HTTP/JSON，若延迟或部署不合适再换。
- LeRobot ARX5 插件是否直接作为依赖：应等 core control layer 稳定后再决定。
- VLA 输出的坐标系和置信字段：应由 Roboclaw 视觉与任务层定义。
- MoveIt 2 是否进入机械臂主链路：当前不建议进入第一阶段。

## 15. 验收基线

软件层最低验收：

- 所有无硬件单元测试通过。
- lint 或 `compileall` 验证通过。
- fake adapter 覆盖命令生命周期。
- 安全层可独立测试。
- CLI 输出稳定 JSON。
- OpenClaw 侧不直接 import SDK。
- 适配层没有自研 IK、动力学、插值器或 CAN 通信。
- Xbox 输入层没有 SDK 调用，只产生受限遥操作命令。
- debug profile 只能通过维护权限启用，不能由 agent 默认调用。

硬件层最低验收：

- `arx5-interface` 可导入。
- `rclpy` 和 ROS2 Jazzy 环境可并存。
- CAN 接口可识别。
- `state` 可读取。
- `damping` 可触发。
- 最小末端小步长动作可执行并可取消。
- Xbox 手柄 deadman 松开后停止发送运动并进入安全状态。
- 重力补偿和低增益 profile 只在维护流程中验证和启用。

只有这些基线成立后，才进入 VLA 真实目标、遥操作采集、LeRobot 策略评测和高级柔顺 profile。
