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

需要注意：当前本地类型文件没有显示“直接按末端力闭环控制”的高级 API。
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
      debug_profiles.py
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

### Phase 0：冻结当前文档和硬件基线

目标：把当前可确认事实固定下来，避免实现阶段反复改边界。

交付物：

- 更新本文件。
- 更新 `docs/README.md` 对本文件的说明。
- 记录本地 SDK commit：`vendor/real_stanford_arx5_sdk` 当前为 `8612790d6823916e7e394661e4e7abfe355ceb64`。
- 保留当前 `.venv` 验证方式，不把 `.venv` 纳入 Git。

验证：

```bash
git status --short
git submodule status --recursive
python - <<'PY'
import sys
import arx5_interface
print(sys.executable)
print(arx5_interface.__file__)
PY
```

### Phase 1：建立 Python 包骨架

目标：让 `armctrl` 从文档仓变成可测试 Python 包。

文件：

- 创建 `pyproject.toml`
- 创建 `src/armctrl/__init__.py`
- 创建 `src/armctrl/protocol/models.py`
- 创建 `tests/unit/test_protocol_models.py`

验收：

- 能运行 `uv run pytest tests/unit/test_protocol_models.py`
- 请求和响应模型能序列化为 JSON。
- 不 import `arx5_interface` 时也能跑协议层测试。

### Phase 2：实现 SDK 适配层

目标：把 SDK 调用集中到一个边界内。
这一阶段最重要的是“薄封装”，不是重写 SDK。

文件：

- 创建 `src/armctrl/adapters/arx5/sdk.py`
- 创建 `src/armctrl/adapters/arx5/fake.py`
- 创建 `tests/unit/test_arx5_adapter_fake.py`

验收：

- fake adapter 可返回固定关节和末端状态。
- SDK adapter 只在运行时导入 `arx5_interface`。
- 单元测试不需要真机。
- adapter 内不实现 IK、FK、动力学、插值器或 CAN 通信。
- SDK 异常会被转换成 `armctrl.protocol.errors` 中的结构化错误。

### Phase 3：实现安全层

目标：任何运动命令进入执行前都能被纯函数检查。

文件：

- 创建 `src/armctrl/safety/limits.py`
- 创建 `src/armctrl/safety/profiles.py`
- 创建 `src/armctrl/safety/guard.py`
- 创建 `tests/unit/test_safety_guard.py`

验收：

- 超出工作空间的 EEF 目标被拒绝。
- 单步位移过大的遥操作目标被裁剪或拒绝。
- 维护命令在非维护模式下被拒绝。
- `VLA` 低置信目标不能直接执行接触 recipe。

### Phase 4：实现 EEF 目标构造层

目标：把目标末端点转换成 SDK 可接收的 `EEFState` 或短 `EEFState` 序列。
这一层只负责目标规范化、timestamp、profile 和 safety precheck，不实现自研 IK 或自研轨迹控制器。

文件：

- 创建 `src/armctrl/planning/eef_goal.py`
- 创建 `src/armctrl/planning/trajectory_request.py`
- 创建 `tests/unit/test_eef_goal.py`

验收：

- 输入单个末端点，输出带 timestamp 的 `EEFState` 请求数据。
- 多点请求只生成 SDK `set_eef_traj()` 所需的数据结构，不复制 SDK 内部插值器。
- 如需 IK 预检查，只调用 `Arx5Solver.multi_trial_ik()`，失败时返回结构化错误。
- 每个请求点都满足 `SafetyGuard`。

### Phase 5：实现 daemon 和执行器

目标：让所有命令通过同一常驻进程执行。

文件：

- 创建 `src/armctrl/daemon/session.py`
- 创建 `src/armctrl/daemon/state_store.py`
- 创建 `src/armctrl/daemon/executor.py`
- 创建 `src/armctrl/daemon/app.py`
- 创建 `tests/unit/test_command_lifecycle.py`

验收：

- 命令状态按 `received -> accepted -> running -> completed` 转换。
- 拒绝命令返回 `rejected`。
- 超时命令返回 `timeout`。
- fake adapter 下可跑完整生命周期测试。

### Phase 6：实现 client 和 CLI

目标：提供给 OpenClaw、人工和 agent 的稳定入口。

文件：

- 创建 `src/armctrl/client.py`
- 创建 `src/armctrl/cli/arx5ctl.py`
- 创建 `tests/unit/test_client_cli_contract.py`

验收：

- `arx5ctl health --json` 返回固定 schema。
- `arx5ctl state --json` 返回关节、末端、模式和错误字段。
- CLI 不直接创建 SDK controller，只调用 client。

### Phase 7：实现 recipe 层

目标：让 agent 优先调用受约束动作。

文件：

- 创建 `src/armctrl/recipes/registry.py`
- 创建 `src/armctrl/recipes/builtin.py`
- 创建 `configs/x5.safe.yaml`
- 创建 `tests/unit/test_recipe_registry.py`

验收：

- `go_observe`、`go_pregrasp`、`retreat_safe` 可查找。
- 每个 recipe 有 profile、超时、失败回退。
- recipe 展开后仍经过 SafetyGuard。

### Phase 8：接入 Roboclaw

目标：Roboclaw 只通过 client 或 CLI 调用 `armctrl`。

Roboclaw 侧建议文件：

- `src/openclaw/communication/armctrl_client.py`
- `src/openclaw/controller/arm_motion_controller.py`
- `tests/unit/test_armctrl_client_contract.py`

验收：

- OpenClaw 工具层不 import `arx5_interface`。
- 工具只调用 `health`、`state`、`move_eef`、`run_recipe`、`gripper`、`cancel`、`damping`。
- VLA 输出必须先经过 Roboclaw 的任务层和坐标转换，再进入 `armctrl`。

### Phase 9：Xbox 遥操作、调试 profile 和 LeRobot 兼容层

目标：复用同一安全入口支持 Xbox 手柄调试、采集、遥操作和策略评测。

文件：

- 创建 `src/armctrl/teleop/xbox.py`
- 创建 `src/armctrl/teleop/mapping.py`
- 创建 `src/armctrl/teleop/keyboard.py`
- 创建 `src/armctrl/teleop/spacemouse.py`
- 创建 `src/armctrl/safety/debug_profiles.py`
- 创建 `src/armctrl/lerobot_compat/robot.py`
- 创建 `src/armctrl/lerobot_compat/teleoperator.py`
- 创建 `tests/unit/test_xbox_mapping.py`
- 创建 `tests/unit/test_teleop_rate_limit.py`
- 创建 `tests/unit/test_debug_profiles.py`

验收：

- Xbox 手柄输入只生成 `TeleopCommand`，不直接调用 SDK。
- 遥操作输入有 deadman、死区、滤波、步长限制和输入超时。
- 重力补偿、低增益、阻尼、被动拖动、柔顺/阻抗实验通过命名 debug profile 触发。
- `gravity_compensation` 只在 controller 创建前配置，运行中切换必须走维护流程。
- LeRobot action 映射到 `move_eef` 或 recipe，不绕过 SafetyGuard。
- 退出遥操作后进入阻尼或安全停止状态。

### Phase 10：硬件验证

目标：在 N100D 和实机上验证最小安全链。

顺序：

1. 环境导入验证。
2. CAN 接口验证。
3. 状态读取验证。
4. 默认不运动的 plan-only 验证。
5. 最小安全动作验证。
6. `arx5d` fake adapter 验证。
7. `arx5d` 真 SDK adapter 验证。
8. `arx5ctl state` 真机验证。
9. `arx5ctl move-eef` 小步长验证。
10. `damping` 和 `cancel` 验证。
11. Xbox 手柄 deadman、超时阻尼、低速 jog 验证。
12. `gravity_compensation` / 低增益 / 阻尼 profile 在维护模式下逐项验证。

每次硬件验证都要记录：

- commit
- 配置文件
- SDK 版本
- 接口名
- 命令
- 输出 JSON
- 人工观察结果
- 是否触发安全拒绝或故障

## 13. 近期优先级

短期不要先做完整 LeRobot 插件，也不要先做复杂力控。
建议先做：

1. `src/armctrl/protocol`：定请求、响应、状态、错误码。
2. `src/armctrl/safety`：纯函数安全检查。
3. `src/armctrl/adapters/arx5/fake.py`：无硬件测试基座。
4. `src/armctrl/daemon/executor.py`：命令生命周期。
5. `src/armctrl/client.py` 和 `arx5ctl`：OpenClaw 调用入口。
6. `src/armctrl/teleop/xbox.py`：手柄输入映射和 deadman 机制。
7. 真机只验证 `health`、`state`、`damping`、小步长 `move_eef` 和 Xbox 低速 jog。

这样做能先把边界、测试和安全策略固定下来，再逐步引入真实硬件和高级控制。

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
- fake adapter 覆盖命令生命周期。
- 安全层可独立测试。
- CLI 输出稳定 JSON。
- OpenClaw 侧不直接 import SDK。
- 适配层没有自研 IK、动力学、插值器或 CAN 通信。
- Xbox 输入层没有 SDK 调用，只产生受限遥操作命令。

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
