# armctrl 机械臂控制子模块蓝图

- `canonical_id`: `project.armctrl.arm_control_subsystem_blueprint`
- `project_id`: `armctrl`
- `review_status`: `draft`
- `updated_at`: `2026-04-14`
- `source_path`: `D:/repo/Roboclaw`

## 摘要

结合 `Roboclaw` 当前蓝图与 `ARX5 SDK` 的能力边界，`armctrl` 应作为独立机械臂控制仓库存在，采用“控制内核、agent 控制面、LeRobot 兼容层分离”的三层目标。推荐形态不是让上层直接通过一次性 CLI 或 Python 脚本连接机械臂，而是建立一个长期持有硬件会话的本地守护进程 `arx5d`，由它独占 SDK 控制器对象、维护状态机、执行安全检查，并通过一个薄控制面 `arx5ctl` 向人或 agent 暴露结构化能力；当任务进入数据采集、模仿学习和策略部署阶段，再通过 LeRobot 兼容层接入 `Robot` / `Teleoperator` 生态。

这样做的核心收益有三点。第一，实时性敏感部分继续留在 SDK 的 C++ 控制内核与后台线程中，避免“每次命令都重新起进程、重新建控制器、重新连总线”的额外抖动。第二，上层接口可以稳定下来，未来即便内部实现从 Python 守护进程迁到 C++ 守护进程，CLI 与 agent 协议也不需要重写。第三，`Roboclaw` 总体架构里要求的本地服务封装、安全治理、状态查询、视觉桥接和任务编排边界，都可以在这一层落地。

## 一、子模块在整个系统中的位置

`Roboclaw` 的总蓝图已经明确：上层 `OpenClaw / agent` 只负责任务编排，不直接碰底层实时控制；本地服务层负责机械臂控制、视觉桥接、安全策略和配置治理；设备与中间件层继续复用 `ARX5 SDK` 与 `realsense-ros`。在这个前提下，机械臂控制子模块的职责不是“成为整个机器人框架”，而是成为整套系统里唯一合法的 ARX5 动作入口。

因此它的边界应当收敛为四件事：一是绑定和管理 ARX5 硬件会话；二是把底层原语整理成稳定的动作接口；三是在动作进入硬件前执行统一安全检查；四是以状态、事件和错误码形式向上提供可追溯反馈。它不负责视觉算法，不负责任务级多步推理，也不负责最终的人机交互体验。

## 二、设计原则

| 原则 | 设计含义 |
|---|---|
| 单一硬件入口 | 机械臂真实动作只允许通过一个守护进程进入 SDK |
| 数据面与控制面分离 | 高频控制留在 SDK/daemon 内，CLI 只承担命令和查询 |
| 能力分层暴露 | 先暴露基础动作原语，再暴露受约束 recipe，不直接把高风险控制模式交给 agent |
| 模式显式切换 | 关节模式、笛卡尔模式、维护模式、阻尼模式必须是显式状态而不是隐式副作用 |
| 安全优先于灵活 | 对 agent 来说，少暴露一些自由度比把不稳定能力裸露出来更合适 |
| 可替换实现 | 先用 SDK 官方 pybind/wheel 起步，但协议层和进程边界要允许后续替换为 C++ 实现 |

## 三、三层目标

### 3.1 Core Control Layer

`core control layer` 是 `armctrl` 的底座，负责直接接入 `ARX5 SDK`，并提供长期持有硬件会话的 `arx5d`。这一层的主要对象包括 `SessionManager`、`ModeManager`、`SafetyGuard`、`CommandExecutor`、`StateStore` 和 `RecipeRegistry`。它必须保证真实动作只从一个受控入口进入硬件，不让上层 agent 或 LeRobot 策略直接持有 SDK 控制器对象。

这一层的核心输出不是“给人看的 CLI”，而是稳定的内部命令协议、结构化状态、错误码、安全检查结果和动作生命周期事件。后续如果 Python 守护进程成为瓶颈，优先替换这一层内部实现，而不是改变上层协议。

### 3.2 Agent CLI Layer

`agent CLI layer` 面向人、脚本和 agent。它包含 `arx5ctl`、thin client、可审计 JSON 输出和动作确认策略。它不承担控制逻辑，不做高频闭环，只负责把上层意图映射到 `core control layer` 的受控命令。

这一层默认开放低风险能力，例如 `health`、`state`、`mode`、`home`、`damping`、`move-eef`、`gripper`、`cancel` 和少量 recipe。`move-joint`、校准、增益修改、柔顺 / 阻抗参数和原始 streaming 不应作为默认 agent 工具直接暴露。

### 3.3 LeRobot Compatibility Layer

`LeRobot compatibility layer` 面向数据采集、遥操作、模仿学习、策略训练和策略部署。它应实现或适配 LeRobot 的 `Robot` / `Teleoperator` 接口，把 `get_observation()`、`send_action()`、`get_action()` 映射到 `core control layer`，并复用 LeRobot 对数据集、processor、teleoperation、record / train / eval 的生态能力。

这一层不应替代 `core control layer`。LeRobot 的接口适合学习循环里的 observation / action 标准化，但不天然覆盖本项目需要的 agent 安全策略、动作确认、recipe 管理、长期 daemon 生命周期和故障回退。因此推荐把它作为兼容层，而不是把 `armctrl` 直接设计成 LeRobot 插件的薄包装。

### 3.4 三层关系

```text
LeRobot / policy / teleoperation
    -> LeRobot compatibility layer
        -> core control layer
            -> ARX5 SDK
                -> ARX X5

OpenClaw / agent / scripts
    -> agent CLI layer
        -> core control layer
            -> ARX5 SDK
                -> ARX X5
```

这意味着 LeRobot 和 agent 是两条上层入口，二者都不直接越过 `core control layer` 操作机械臂。

## 四、推荐分层

### 4.1 四层运行结构

| 层级 | 组件 | 主要职责 | 对上暴露 |
|---|---|---|---|
| L0 | `ARX5 SDK` | 电机通信、求解器、控制器、后台线程 | 原生 Python/C++ 接口 |
| L1 | `arx5d` 控制守护进程 | 生命周期管理、状态机、安全检查、命令执行、状态缓存 | 本地 RPC / 本地 TCP / UDS |
| L2 | `arx5ctl` 控制面 | 面向人和 agent 的薄 CLI，参数解析、输出 JSON、确认策略 | CLI 命令 |
| L3 | `Roboclaw` 任务层 | 任务编排、视觉调用、动作 recipe 选择、结果汇总 | Tool / Skill / HTTP API |

### 4.2 为什么不推荐“直接 py-first CLI”

直接让 CLI 进程每次调用都创建 SDK 控制器对象，会同时带来几类问题：硬件连接重复建立，状态不可持续，安全上下文无法继承，执行中的动作难以中止，命令之间缺少会话语义。即便 SDK 内部控制环本身很快，外层这种一次性进程模型也会把延迟、抖动和可靠性问题重新引进来。

因此，推荐的运行方式是 `arx5d` 常驻，`arx5ctl` 瞬时。前者保有真实控制状态，后者只是命令前端。

## 五、运行形态

### 5.1 第一阶段建议

- `arx5d`: Python 守护进程，内部直接使用官方 `arx5-interface` pybind/wheel
- `arx5ctl`: Python CLI，负责本地调用守护进程并输出 JSON
- `Roboclaw`: 继续按现有蓝图使用 Python 编排层，调用 `arx5ctl` 或直接调本地服务接口
- `LeRobot compatibility layer`: 先作为可选适配层，不进入最小控制闭环的关键路径

这个阶段的重点是先把“硬件连接、状态、安全、命令接口”打稳，而不是过早追求语言纯度。

### 5.1.1 Bringup 分支与阶段目标

当前阶段应落在 `feature/hardware-bringup`，而不是提前进入 `agent CLI layer` 或 `LeRobot compatibility layer` 的开发。这个分支的目标不是把框架做满，而是先证明四件事：目标机环境正常、官方 SDK 可用、机械臂控制通路打通、D435i 链路正常。

更具体地说，bringup 阶段应至少完成以下验证项：一是 Ubuntu 24.04 + Intel x86 主机上的 Python 运行环境、依赖和 USB / 网卡基础条件正常；二是 `arx5-interface` 可安装、可导入、可创建 controller；三是 ARX5 本体能够完成状态读取与最小安全动作；四是 D435i 能稳定枚举并输出 RGB / depth / IMU 基础流。只有这四项成立之后，再进入 CLI 协议、recipe 和上层 agent 工具开发，后续定位问题才不会把“环境问题”和“框架问题”混在一起。

### 5.1.2 Bringup 期的最小交付物

`feature/hardware-bringup` 分支内的交付重点应收敛在 bringup runbook、上游依赖登记、最小检查脚本和安全配置，而不是提前铺开完整模块树。推荐优先落四类内容：一类是环境与接线检查文档；二类是 `arx5-interface` 与相机链路的最小验证脚本；三类是最保守的 bringup 配置；四类是失败场景与恢复步骤记录。

换句话说，这个分支首先要产出“能否动起来”的确定性，而不是“接口看起来很完整”的表面进展。对 `armctrl` 来说，bringup 阶段的价值在于把后续所有软件层建立在真实可控的硬件基础上。

### 5.2 第二阶段演进

当后续需要进一步降低尾延迟、压缩故障面或纳入更多高频控制模式时，可以把 `arx5d` 从 Python 实现替换为 C++ 守护进程，但保持同一套命令协议和同一组 CLI。

换句话说，当前推荐的是“先固定边界，再替换内部实现”，而不是一开始就把所有层级绑死在某种语言选择上。

### 5.3 Bringup 阶段的依赖边界

bringup 期必须以官方 SDK 作为主线，而不是让多个第三方封装同时进入运行闭环。对当前项目，更稳妥的顺序是：运行时优先使用 `arx5-interface`，源码锚点保留 `real-stanford/arx5-sdk`，系统侧继续借助 `librealsense` / `realsense-ros` 打通 D435i；`umi-arx`、`umi-on-legs`、`arx5-common` 与 LeRobot ARX5 插件则作为参考实现，不直接进入最小控制通路。

这样划分的原因很直接：bringup 阶段首先要回答“官方链路能否工作”，而不是“哪种封装最优雅”。如果一开始就把官方 SDK、自定义 daemon、LeRobot 插件和其他参考仓一起绑进主链路，后续一旦失败，很难区分问题到底来自硬件、环境、SDK 还是上层封装。把运行依赖、源码锚点和参考实现明确分层，能显著降低排障成本。

## 六、子模块内部模块划分

### 6.1 进程内组件

| 组件 | 职责 |
|---|---|
| `SessionManager` | 初始化 SDK、绑定接口、加载配置、维护会话状态 |
| `ModeManager` | 管理 `idle / joint / cartesian / maintenance / damping / fault` 模式 |
| `SafetyGuard` | 工作空间、步长、速度、姿态、夹爪开度、超时、模式权限检查 |
| `CommandExecutor` | 执行基础动作、recipe、校准、回零、阻尼切换 |
| `StateStore` | 缓存当前关节状态、末端状态、最近命令、错误码、健康状态 |
| `EventLogger` | 记录状态迁移、动作请求、异常、手动确认与超时事件 |
| `RecipeRegistry` | 管理受约束动作序列，例如观察位、预抓取位、撤退位 |

### 6.2 SDK 控制器持有策略

`ARX5 SDK` 当前同时提供 `Arx5JointController` 和 `Arx5CartesianController`。这两类控制器都可读状态，但写命令域不同。为了避免同一机械臂在同一时刻被多个控制器对象并发持有，推荐 `arx5d` 在任意时刻只保有一个活动控制器，并把控制器切换作为显式模式切换。

推荐策略如下：

- 默认执行模式使用 `cartesian`，服务视觉引导、观察位、预抓取位等任务
- 维护动作和校准动作进入 `maintenance`，必要时切到 `joint` 控制器
- `home`、`damping`、`state` 属于跨模式公共能力，但执行前后要保证模式一致性

这套设计比“同时持有 joint/cartesian 两个控制器对象”更稳，更利于管理接口占用、会话一致性和异常恢复。

## 七、生命周期

### 7.1 守护进程生命周期

| 阶段 | 说明 | 允许的动作 |
|---|---|---|
| `bootstrapping` | 读取配置、检查依赖、检查接口、建立日志 | 只允许健康检查 |
| `binding` | 创建 SDK 控制器对象、绑定总线或网卡接口 | 不允许外部动作 |
| `ready` | 硬件可用但尚未执行任务 | 允许查询、home、进入执行模式 |
| `executing` | 正在执行基础动作或 recipe | 允许查询、取消、阻尼降级 |
| `maintenance` | 校准、调参、调试期 | 仅允许维护类命令 |
| `damping` | 进入阻尼或被动安全态 | 允许查询、恢复、关闭 |
| `fault` | 安全检查失败、SDK 异常、通信超时 | 允许查询、人工恢复、关闭 |
| `shutting_down` | 回收资源、写入日志、释放硬件会话 | 不接受新动作 |

### 7.2 动作级生命周期

每个动作请求都应走一条统一流水线：

1. 接收命令并生成 `command_id`
2. 做模式检查与参数校验
3. 做安全检查与工作空间检查
4. 写入 `accepted / rejected`
5. 进入执行
6. 持续更新 `running / completed / aborted / faulted`
7. 记录最终状态与关键 telemetry

这样即便上层最终通过 CLI 调用，依然具备服务式可追踪性。

## 八、输入与输出设计

### 8.1 配置输入

控制子模块启动时至少需要以下配置域：

| 配置域 | 关键字段 |
|---|---|
| 机械臂 | `model`, `interface_name`, `controller_mode`, `robot_config_override` |
| 安全 | `workspace_limits`, `joint_delta_max`, `eef_delta_max`, `velocity_scale`, `command_timeout` |
| 模式 | `default_mode`, `allow_maintenance`, `allow_direct_joint_motion`, `allow_direct_cartesian_motion` |
| recipe | `home_pose_name`, `observation_pose_name`, `pregrasp_profiles` |
| LeRobot | `enable_lerobot_adapter`, `control_mode`, `observation_features`, `action_features`, `camera_bindings` |
| 审计 | `log_level`, `event_log_path`, `record_last_n_commands` |

### 8.2 运行时命令输入

推荐把输入分成三组。

第一组是只读查询命令，包括 `health`、`state`、`mode`、`last-error`、`active-command`。这类命令可以直接对 agent 开放。

第二组是基础动作原语，包括 `home`、`damping`、`move-joint`、`move-eef`、`set-gripper`、`cancel`。这类命令属于标准控制面，但需要统一走安全检查。

第三组是受约束 recipe，包括 `go-observe`、`go-pregrasp`、`retreat-safe`、`reset-task-posture` 等预定义动作序列。它们对 agent 更友好，因为参数面更小、风险更可控。

### 8.3 输出结构

所有命令输出建议使用稳定 JSON，而不是混合日志文本。最小返回结构建议如下：

```json
{
  "ok": true,
  "command_id": "cmd_20260413_001",
  "state": "completed",
  "mode": "cartesian",
  "timestamp": 1770000000.123,
  "result": {
    "joint_pos": [],
    "eef_pose": [],
    "gripper_pos": 0.0
  },
  "safety": {
    "workspace_check": "passed",
    "delta_check": "passed"
  },
  "error": null
}
```

如果命令失败，错误应当结构化返回，不要只把 Python 异常原样喷给上层。

## 九、CLI 设计

### 9.1 CLI 的角色

`arx5ctl` 不应成为控制逻辑承载体，而应保持为一个薄前端。它负责四件事：解析参数、调用 `arx5d`、在危险动作时执行确认策略、把结果稳定打印为 JSON。

### 9.2 建议命令集

| 命令 | 用途 | 对 agent 的建议 |
|---|---|---|
| `arx5ctl health` | 查询守护进程与硬件状态 | 默认开放 |
| `arx5ctl state` | 返回关节、末端、夹爪、模式、错误态 | 默认开放 |
| `arx5ctl home` | 回零或回到安全初始位 | 允许，但建议带确认策略 |
| `arx5ctl damping` | 进入阻尼 / 安全态 | 默认开放 |
| `arx5ctl move-joint` | 关节空间移动 | 仅在明确需要时开放 |
| `arx5ctl move-eef` | 末端位姿移动 | 推荐作为主要动作原语 |
| `arx5ctl gripper` | 夹爪开合 | 开放 |
| `arx5ctl recipe run <name>` | 执行受约束动作序列 | 推荐开放 |
| `arx5ctl cancel` | 取消当前动作 | 默认开放 |
| `arx5ctl calibrate-*` | 维护类动作 | 不直接对 agent 开放 |

### 9.3 CLI 与 agent 的边界

对 agent 来说，推荐优先暴露 `state`、`move-eef`、`gripper`、`recipe run`、`home`、`damping` 六类能力。`move-joint` 可以保留给工程调试或特定场景，不作为默认主入口。`calibrate-*`、增益修改、原始维护命令不宜直接开放给 agent。

## 十、不同层级功能如何分层

### 10.1 底层原语层

这一层是对子模块最稳定的接口承诺，应尽量小而硬，包括：

- 状态读取
- 回零
- 阻尼
- 关节运动
- 末端位姿运动
- 夹爪运动
- 取消 / 超时回退

原语层是以后所有 recipe、视觉闭环和任务编排的共同基座。

### 10.2 recipe 层

这层用于承载“设计好的动作序列”，但不应写死在 agent prompt 里，而应写成受版本管理的配置或代码对象。推荐先做三类：

| recipe 类型 | 作用 |
|---|---|
| 姿态类 | `go_home_safe`, `go_observe`, `go_rest` |
| 操作前后处理类 | `pregrasp_setup`, `retreat_vertical`, `post_task_reset` |
| 采集类 | `scan_small_arc`, `capture_pose_set` |

recipe 层适合暴露给 agent，因为它把高自由度动作压缩成了有限的、经过验证的操作模板。

### 10.3 高级控制模式层

柔顺、阻抗、力补偿、扭矩控制、示教回放这类能力应视为高级模式，而不是默认 agent 接口。原因很简单：它们更接近控制器参数空间，安全面大，语义边界窄，对上下文误解非常敏感。

推荐策略是：

- 第一阶段不把柔顺 / 阻抗控制直接暴露给 agent
- 如果后续确实需要，把它封装成少量命名 profile，例如 `surface_approach_soft`、`peg_insert_compliant`
- 上层只选择 profile，不直接改 `kp / kd / torque` 等低层参数

这样既保留能力扩展空间，又避免把控制器内部细节泄漏给语言模型。

### 10.4 LeRobot 接口层

LeRobot 接口层负责把机械臂状态转换成 `observation_features`，把策略输出或遥操作输入转换成 `action_features`，并在发送动作前经过 `core control layer` 的安全检查。它的功能边界应聚焦学习系统，不承接 agent 的自然语言任务编排。

如果现有 `lerobot-robot-arx5`、`lerobot-teleoperator-arx5`、`arx5-common` 已满足基础接入，可以优先复用其接口命名、feature schema 和 processor 思路；如果它们缺少 daemon、安全状态机或 recipe 管理，则在 `armctrl` 内补齐，而不是直接把这些责任推给 LeRobot 插件。

## 十一、为什么推荐“daemon + pybind”，而不是一开始就全 C++

从系统工程角度看，当前最稀缺的不是极限性能，而是边界稳定性。官方 SDK 已经把高频控制、求解器、底层通信封装在 C++ 内核里，并通过 pybind 暴露可用接口。此时最合理的做法，是用这个边界先做一个可靠控制面，而不是为了语言统一再重造一层完整控制内核。

因此当前阶段推荐：

- 实时控制依赖 SDK
- 进程生命周期、安全、协议和状态机由 `arx5d` 管
- CLI 和上层任务编排只看结构化接口

后续如果测得 Python 守护进程本身在你的真实负载下成为瓶颈，再把 `arx5d` 下沉为 C++ 也不迟。这个迁移的前提是现在就把协议面定清楚。

## 十二、与 Roboclaw 总体蓝图的对齐

这套方案与 `Roboclaw` 现有文档是一致的。`Roboclaw` 已经明确把机械臂控制服务定义为本地服务层能力，把 `OpenClaw` 定位为上层任务入口，而不是直接操作 SDK。当前需要做的，不是推翻这个蓝图，而是把“机械臂控制服务”从一个概念名字，落成一个具体到进程、命令、状态机和安全边界的子模块。

对齐后的系统关系应为：

```text
OpenClaw / Agent
    -> Roboclaw Task Orchestrator
        -> arx5ctl / local RPC client
            -> arx5d
                -> ARX5 SDK
                    -> ARX X5
```

视觉链路则继续平行存在：

```text
OpenClaw / Agent
    -> Roboclaw Task Orchestrator
        -> Vision Bridge Service
            -> realsense-ros / D435i
```

二者在任务层汇合，不在控制内核层耦合。

## 十三、建议的实现顺序

### 阶段 A：打通控制内核边界

- 在目标机上验证 `ARX5 SDK` 安装、导入、接口权限和最小动作链
- 明确 `model`、`interface_name`、安全配置的加载方式
- 固定守护进程与 CLI 的命令协议

### 阶段 B：建立最小守护进程

- 实现 `arx5d`
- 先支持 `health`、`state`、`home`、`damping`、`move-eef`、`gripper`
- 补齐状态缓存、超时回退、错误码与事件日志

### 阶段 C：建立 recipe 层

- 先定义少量观察位、预抓取位、撤退位
- 验证 recipe 是否比直接暴露底层动作更适合 agent

### 阶段 D：补 LeRobot compatibility layer

- 评估并复用现有 `lerobot-robot-arx5`、`lerobot-teleoperator-arx5`、`arx5-common`
- 将 LeRobot 的 `get_observation()` / `send_action()` / `get_action()` 映射到 `core control layer`
- 验证 teleoperation、record、policy eval 与 agent CLI 能否共用同一个安全执行入口

### 阶段 E：再考虑高级模式

- 关节空间直接控制
- 校准维护模式
- 柔顺 / 力补偿 / 阻抗 profile

## 十四、当前建议结论

当前最合适的方案不是“全 py-first 直连 CLI”，也不是“现在就全面迁往 C++ 框架”，而是：

- 用官方 `ARX5 SDK` 的 pybind 作为当前控制内核边界
- 把真实硬件控制收敛到常驻进程 `arx5d`
- 用 `arx5ctl` 暴露稳定、结构化、可审计的控制面
- 把 LeRobot 作为兼容层，服务数据采集、遥操作、训练和策略评测
- 对 agent 主要开放基础状态、末端动作、夹爪动作和受约束 recipe
- 把柔顺、阻抗、校准、调参等高风险能力保留在维护层或命名 profile 层

这条路线既保留了系统工程上的清晰边界，也保留了未来向更低延迟实现演进的空间。
