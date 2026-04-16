# armctrl 参考项目与 SJTU RoboClaw 调研记录

- `canonical_id`: `project.armctrl.reference_projects_and_sjtu_roboclaw_notes`
- `project_id`: `armctrl`
- `review_status`: `draft`
- `updated_at`: `2026-04-14`
- `source_path`: `D:/repo/Roboclaw`

## 调研问题

本轮调研围绕一个具体架构问题展开：`armctrl` 是否应从 `Roboclaw` 系统仓中解耦为独立机械臂控制仓库，并在未来以 submodule 形式接入。对应的工程问题包括：机械臂执行层是否有类似 `px4ctrl` 的独立控制包范式；agent 系统应如何调用机械臂能力；已有机器人项目如何处理控制层、任务层、LLM/agent 层之间的边界。

## 总体结论

公开项目里存在多种“类似 `px4ctrl` 的机械臂控制层设计范式”，但没有一个像 `px4ctrl` 在飞控生态里那样统一的机械臂领域标准仓库。对本项目更有价值的是架构模式，而不是照搬某个仓库。

当前最适合 `armctrl` 的边界是：独立仓库负责 ARX5 机械臂控制、状态、安全、recipe、agent CLI / daemon 控制面，以及 LeRobot compatibility layer；`Roboclaw` 系统仓继续负责 D435i 系统集成、视觉桥接、hand-eye 标定、OpenClaw / agent 编排和整体系统 glue code。未来 `Roboclaw` 通过 submodule 引入 `armctrl`，并通过本地 RPC、CLI 或 thin client 调用它。

本轮新增发现是：ARX5 生态里已经存在 `arx5-common`、`lerobot-robot-arx5`、`lerobot-teleoperator-arx5` 这三个 PyPI 包。它们不是完整的 agent-safe 机械臂执行层，但已经覆盖了 LeRobot follower / leader 插件和一部分低层 ARX5 包装逻辑。因此，`armctrl` 不应重复造一套 LeRobot 插件命名和 feature schema；更合理的做法是评估复用或兼容这些包，同时在本仓补齐安全执行层、daemon、CLI、recipe 和 Roboclaw submodule 边界。

从当前开发阶段看，`armctrl` 的第一优先级仍然是 bringup，而不是先做完整框架。也就是说，当前最重要的判断不是“哪套 agent CLI 最优雅”，而是“官方 SDK 能否在目标机上稳定跑通、控制链路能否闭合、D435i 链路是否正常”。因此 bringup 分支应明确落在 `feature/hardware-bringup`，并以最小控制通路为交付目标。

## Bringup 阶段的依赖管理结论

bringup 阶段不建议把所有参考仓都做成运行依赖。更稳妥的做法是把上游信息分成三层管理。

第一层是运行依赖层，只放真正进入 bringup 闭环的组件。对当前项目，这一层的核心是 `arx5-interface`、`arx5-sdk` 和 D435i 对应的 `librealsense` / `realsense-ros`。其中 `arx5-interface` 适合在 Ubuntu 24.04 + Intel x86 上快速验证安装、导入、controller 创建和最小动作链，`arx5-sdk` 则作为最权威的源码与示例来源。

第二层是源码锚点层，用来固定可追溯的官方来源。对 ARX5 来说，`real-stanford/arx5-sdk` 应作为 bringup 阶段必须保留的官方源码锚点，因为后续无论是排查控制器创建、查 pybind 接口、还是核对安全提示和 ZMQ 示例，都需要一个稳定的上游参考。

第三层是参考实现层，用来借鉴部署方式、系统组织和接口边界，但不直接进入最小控制通路。当前最值得看的两个仓库是 `real-stanford/umi-arx` 和 `real-stanford/umi-on-legs`。前者更像 ARX5 的最小部署环境，适合借鉴 bringup 组织方式；后者更像完整系统集成，适合在后续系统工程收口时参考。`arx5-common`、`lerobot-robot-arx5` 和 `lerobot-teleoperator-arx5` 仍然更适合放在 LeRobot compatibility 阶段，而不是现在的 bringup 主线。

从仓库治理角度，bringup 期不建议把 `umi-arx`、`umi-on-legs` 和 LeRobot ARX5 插件都纳入 `armctrl` 的硬依赖。更合适的方式是在项目文档中维护一张上游依赖登记表，记录名称、类型、来源 URL、固定版本或 commit、用途以及当前阶段是否启用。这样既保留了参考信息，又不会把第三方项目直接卷进最小验证闭环。

## 参考项目对照

| 项目 | 链接 | 对 `armctrl` 的启发 | 不建议照搬的部分 |
|---|---|---|---|
| LeRobot | `https://github.com/huggingface/lerobot` / `https://huggingface.co/docs/lerobot/main/integrate_hardware` | 提供 `Robot` / `Teleoperator` 接口、数据采集、控制管线、策略训练与推理生态；适合作为 `LeRobot compatibility layer` 的目标接口 | 不应替代本项目的 daemon、安全状态机和 agent CLI 控制面 |
| ARX5 LeRobot 插件 | `https://pypi.org/project/arx5-common/` / `https://pypi.org/project/lerobot-robot-arx5/` / `https://pypi.org/project/lerobot-teleoperator-arx5/` | 已有 ARX5 follower、leader / teleoperator 与 common utilities，可直接参考 feature schema、control mode、processor 和 gripper 处理 | 规模较小，偏 LeRobot 插件；不覆盖 agent 安全执行层、长期 daemon、CLI、recipe 和系统级故障恢复 |
| MINT-SJTU RoboClaw | `https://github.com/MINT-SJTU/RoboClaw` | 说明 RoboClaw 论文代码更偏 embodied workflow、command builder、session 和执行服务封装，而不是单独机械臂底层控制包 | 不应把它的任务/数据/LeRobot 工作流直接塞进 `armctrl` 控制仓 |
| FrankaPy | `https://github.com/iamlab-cmu/frankapy` | 借鉴“上层 Python API / 客户端 + 底层 Franka Interface 服务”的分离思路 | 不照搬 Franka 专用接口和硬件假设 |
| Polymetis | `https://github.com/facebookresearch/fairo/tree/main/polymetis` | 借鉴机器人接口服务化、控制策略与 RPC 风格边界 | 不在第一阶段重建完整高频策略框架 |
| TidyBot2 | `https://github.com/jimmyyhwu/tidybot2` | 借鉴 `arm_server.py` / `arm_client.py` 这类隔离机械臂执行进程的系统组织方式 | 不把完整家务机器人任务栈引入机械臂控制仓 |
| MoveIt Servo | `https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html` | 后续连续伺服、视觉闭环、遥操作可借鉴 servo layer | 第一阶段不把 MoveIt Servo 作为基础依赖 |
| xArm ROS2 / xArm Python SDK | `https://github.com/xArm-Developer/xarm_ros2` / `https://github.com/xArm-Developer/xArm-Python-SDK` | 借鉴厂商 SDK、ROS 包、上层应用三者分仓的生态边界 | 不引入 xArm 具体 API 形态 |
| Stretch AI | `https://github.com/hello-robot/stretch_ai` | 借鉴 agent / embodied AI 层与底层机器人接口分离 | 不把 agent prompt 和机器人控制逻辑混进控制仓 |
| ROS MCP Server | `https://github.com/robotmcp/ros-mcp-server` | 说明 robot-to-agent 工具服务器是一类上层桥接模式 | 该层更适合放在 `Roboclaw` 或 agent gateway，不适合成为 `armctrl` 内核 |

## LeRobot 调研补充

LeRobot 的 `Bring Your Own Hardware` 文档明确说明，它提供 `Robot` 基类，用于把自定义硬件接入 LeRobot 的工具链，覆盖数据采集、控制管线、策略训练和推理。该文档要求自定义机器人实现 `observation_features`、`action_features`、`connect()`、`disconnect()`、`get_observation()`、`send_action()` 等接口，并支持以独立可安装 Python package 的形式通过 `lerobot_robot_`、`lerobot_camera_`、`lerobot_teleoperator_` 前缀被 CLI 自动发现。

这说明 LeRobot 做的事情确实与我们此前规划的“统一机器人接口”部分重合。区别在于，LeRobot 的接口中心是学习系统中的 observation / action 循环，而 `armctrl` 还需要面向 agent 的安全执行层，包括动作确认、recipe、状态机、故障恢复和单一硬件入口。因此，推荐的关系是：`armctrl` 以 `core control layer` 为底座，向上提供两类入口，一类是面向 agent 的 `agent CLI layer`，另一类是面向数据采集和策略学习的 `LeRobot compatibility layer`。

## ARX5 现有开源包覆盖度判断

### `arx5-common`

`arx5-common` 是 PyPI 上发布的 ARX5 LeRobot 插件共用工具包，版本 `0.1.1`，发布时间为 `2026-01-23`。本地下载源码包后可以看到它提供 `ARX5Arm`、`ARX5ArmConfig`、`ARXControlMode`、数据处理和 processor 相关工具。

从源码看，`ARX5Arm` 已经直接包装 `arx5_interface`，支持 `connect()`、`disconnect()`、`reset_to_home()`、`configure()`、`get_state()`、`get_observation()`、`send_command()`、`interpolate_to_position()`。它在连接时会创建 `Arx5JointController` 或 `Arx5CartesianController`，设置 `controller_dt = 0.01`、`gravity_compensation = True`、`background_send_recv = True`，并执行 `reset_to_home()`。这已经覆盖了“LeRobot 插件内部如何调用 ARX5 SDK”的基础层。

但它仍不是完整的 `armctrl`。它没有独立 daemon，没有 agent CLI，没有长期状态机，没有系统级错误码，没有动作审计，也没有面向 OpenClaw / agent 的 recipe 权限层。它可以作为 `sdk_adapters/arx5` 和 `LeRobot compatibility layer` 的重要参考或依赖。

### `lerobot-robot-arx5`

`lerobot-robot-arx5` 是 PyPI 上发布的 ARX5 follower robot 插件，版本 `0.1.1`，发布时间为 `2026-01-23`。它实现了 `ARX5Follower`，继承 LeRobot 的 `Robot` 接口，并提供 `observation_features`、`action_features`、`connect()`、`get_observation()`、`send_action()`、`disconnect()` 等方法。

从源码看，它支持 joint controller 与 cartesian controller 两种模式。joint 模式下 action 是各关节与夹爪位置；cartesian 模式下 action 是 `eef.x`、`eef.y`、`eef.z`、`eef.roll`、`eef.pitch`、`eef.yaw`、`eef.gripper`。它还支持摄像头配置，并通过 `max_relative_target` 对每步动作做相对目标裁剪。

这已经覆盖了“把 ARX5 作为 LeRobot follower 接入数据采集 / 策略部署”的关键需求。它不覆盖“给 agent 暴露安全 CLI / daemon / recipe”的需求。

### `lerobot-teleoperator-arx5`

`lerobot-teleoperator-arx5` 是 PyPI 上发布的 ARX5 leader / teleoperator 插件，版本 `0.1.1`，发布时间为 `2026-01-23`。它实现了 `ARX5Leader`，继承 LeRobot 的 `Teleoperator` 接口，并通过 `get_action()` 读取 leader arm 的位置、速度、力矩和 EEF pose。

这对遥操作采集非常有价值：如果未来有 leader-follower 双臂或希望把一只 ARX5 当示教输入，它可以直接作为参考。但对当前 “单 ARX5 + OpenClaw agent + D435i” 的执行层来说，它不是必要核心依赖。

### 对“是否已有仓库覆盖我们需求”的回答

截至本轮调研，ARX5 方向已经有以下可复用开源组件：

1. `real-stanford/arx5-sdk` 覆盖底层 SDK、C++/Python pybind、示例、ZMQ server/client 参考。
2. `arx5-common` 覆盖 LeRobot ARX5 插件共用低层包装。
3. `lerobot-robot-arx5` 覆盖 LeRobot follower robot 接口。
4. `lerobot-teleoperator-arx5` 覆盖 LeRobot leader / teleoperator 接口。

但没有发现一个已经完整覆盖 `armctrl` 需求的仓库。原因是我们的需求同时包含四类东西：ARX5 SDK 接入、agent-safe daemon / CLI、动作 recipe / 生命周期状态机、LeRobot compatibility layer。现有包基本覆盖第 1 类和第 4 类的一部分，第 2 类和第 3 类仍需要由 `armctrl` 自己设计和实现。

## ARX5 现有模块信息卡片

本节记录截至 `2026-04-14` 已核验到的 ARX5 相关开源模块。除 `real-stanford/arx5-sdk` 为 GitHub 仓库外，`arx5-common`、`lerobot-robot-arx5`、`lerobot-teleoperator-arx5` 当前以 PyPI 包形式发布；其 `PKG-INFO` 与 `pyproject.toml` 中未看到明确的 GitHub / homepage URL，因此暂按 PyPI 源码包记录。

| 模块 | 来源 | 当前核验版本 | 主要职责 | 与 `armctrl` 的关系 |
|---|---|---|---|---|
| `arx5-sdk` | `https://github.com/real-stanford/arx5-sdk` | 本地 submodule 记录为 `fa331080252aed4479a97c6f0eb76a1932839b0e`；README 当前显示有 `2026.03.20` pip 安装更新 | ARX5 底层 C++ / Python 控制 SDK，提供 `Arx5JointController`、`Arx5CartesianController`、`Arx5Solver`、ZMQ 示例、CAN / EtherCAT-CAN 设置说明 | 必须依赖的硬件执行内核；`armctrl` 不应重写底层控制器 |
| `arx5-interface` | `https://pypi.org/project/arx5-interface/` | 调研时核验到 `0.1.2` | ARX5 SDK 的 pip wheel / pybind 分发形式，降低 Ubuntu 24.04 / Python 环境接入成本 | 第一阶段优先作为 `sdk_adapters/arx5` 的底层依赖，替代手工 conda + 源码编译路径 |
| `umi-arx` | `https://github.com/real-stanford/umi-arx` | 本轮核验到公开仓存在，定位为 “Minimal UMI deployment environment for ARX5 robot arm” | 最小部署与 bringup 组织参考 | 适合借鉴 bringup 结构和部署方式，但不应在当前阶段作为主运行依赖 |
| `umi-on-legs` | `https://github.com/real-stanford/umi-on-legs` | 本轮核验到公开仓存在，仓内直接引用 `arx5-sdk` | 完整系统集成参考 | 适合借鉴系统组织与后续集成方式，不应替代当前官方 bringup 主链路 |
| `arx5-common` | `https://pypi.org/project/arx5-common/` | `0.1.1` | ARX5 LeRobot 插件共用工具，包含 `ARX5Arm`、配置类型、processor、dataset frame 构造工具 | 可复用或参考其 `ARX5Arm` 包装方式，但需要在 `armctrl` 补 daemon、安全、recipe 和 agent CLI |
| `lerobot-robot-arx5` | `https://pypi.org/project/lerobot-robot-arx5/` | `0.1.1` | LeRobot follower robot 插件，实现 `ARX5Follower`，支持 `Robot` 接口、camera 配置、joint / cartesian action features | 可作为 `LeRobot compatibility layer` 的直接参考或依赖；不应作为 agent 执行层替代品 |
| `lerobot-teleoperator-arx5` | `https://pypi.org/project/lerobot-teleoperator-arx5/` | `0.1.1` | LeRobot leader / teleoperator 插件，实现 `ARX5Leader`，读取 leader arm 状态并生成 action | 对 leader-follower 示教和遥操作采集有价值；当前单臂 agent 执行链可暂不作为核心依赖 |

## Bringup 阶段建议的上游登记方式

为避免 bringup 期把“官方依赖”“参考实现”“后续兼容层”混成一团，建议后续把上游条目按如下口径登记。

| 名称 | 类型 | 建议形态 | 当前阶段用途 |
|---|---|---|---|
| `arx5-interface` | runtime | Python 运行依赖 | 首选安装与导入验证 |
| `arx5-sdk` | official-source | 固定 commit 的官方源码锚点 | 查接口、示例与安全边界 |
| `librealsense` / `realsense-ros` | runtime / official-source | 继续沿用 `Roboclaw` 现有参考仓 | D435i 链路 bringup |
| `umi-arx` | reference-only | 文档登记或临时 clone | 借鉴最小部署环境 |
| `umi-on-legs` | reference-only | 文档登记或临时 clone | 借鉴完整系统集成 |
| `arx5-common` / `lerobot-robot-arx5` / `lerobot-teleoperator-arx5` | reference-only | 文档登记，必要时在 LeRobot 阶段引入 | 后续兼容层对齐 |

### `arx5-sdk`

`arx5-sdk` 是当前最底层、最权威的 ARX5 控制来源。其 README 说明该 SDK 支持无 ROS 运行、Python 接口、C++ 多线程控制，并提供 joint controller、cartesian controller、keyboard / SpaceMouse teleoperation、teach-replay、ZMQ server / client 等示例。它还明确提示：默认安全限位不足以覆盖奇异位形附近或高噪声输入场景，用户应在发送 joint / EEF 控制信号前自行做安全检查。

对 `armctrl` 来说，这意味着底层控制器不应重写，核心工作应放在“怎样安全地持有 SDK 会话、怎样限制上层输入、怎样组织动作生命周期和错误恢复”上。`arx5-sdk` 里的 ZMQ 示例可作为最小服务化参考，但它不是完整的生产控制服务。

### `arx5-interface`

`arx5-interface` 是 ARX5 SDK 的 pip 分发路径。此前源码编译路径需要 conda / cmake / pybind / KDL 等依赖，而 `arx5-interface` 使第一阶段在目标机上验证 import、controller 创建和最小动作链更简单。对 Ubuntu 24.04 + Intel x86 平台，建议优先验证 `pip install arx5-interface`，只有当 wheel 不满足需求或需要改 C++ 源码时，再回到源码编译。

`armctrl` 第一阶段可以把 `arx5-interface` 作为默认后端，在 `sdk_adapters/arx5` 中只写轻包装。这样能避免一开始把仓库做成复杂的 C++ 构建项目。

### `arx5-common`

`arx5-common` 的源码包显示，它面向 LeRobot 插件提供共用 ARX5 工具。核心类 `ARX5Arm` 直接导入 `arx5_interface`，在 `connect()` 中根据 `ARXControlMode` 创建 `Arx5JointController` 或 `Arx5CartesianController`，设置 `controller_dt = 0.01`、`gravity_compensation = True`、`background_send_recv = True`，并执行 `reset_to_home()`。它提供 `get_state()`、`get_observation()`、`send_command()`、`interpolate_to_position()` 等方法。

它已经把 ARX5 SDK 与 LeRobot 插件之间的低层重复代码整理出来，尤其是 joint / EEF 两种 action 形态、leader gripper 缩放、观测聚合和 processor 工具。`armctrl` 可以借鉴或依赖这部分，但不能只用它结束设计，因为它没有长期 daemon、没有外部 CLI 协议、没有状态机和 recipe 权限层。

### `lerobot-robot-arx5`

`lerobot-robot-arx5` 实现了 `ARX5Follower`，继承 LeRobot `Robot` 接口。它定义了 `observation_features`、`action_features`、`connect()`、`configure()`、`reset()`、`get_observation()`、`send_action()`、`disconnect()`。它支持两种控制模式：joint controller 模式下 action 是 `shoulder_pan.pos` 到 `gripper.pos`；cartesian controller 模式下 action 是 `eef.x`、`eef.y`、`eef.z`、`eef.roll`、`eef.pitch`、`eef.yaw`、`eef.gripper`。

该插件已经解决了“ARX5 如何作为 LeRobot follower 参与 record / policy eval”的大部分问题。它还提供 `max_relative_target` 对动作步长做裁剪，这一点对安全有启发。不过这种裁剪仍是学习接口里的局部安全措施，不等同于完整的 agent 执行安全层。`armctrl` 应避免重复实现完全相同的 LeRobot feature schema，但应把动作最终落到统一的 `core control layer`。

### `lerobot-teleoperator-arx5`

`lerobot-teleoperator-arx5` 实现了 `ARX5Leader`，继承 LeRobot `Teleoperator` 接口。它通过 `ARX5Arm` 读取 leader arm 的 joint position、velocity、effort 与 EEF pose，并通过 `get_action()` 输出给 follower 或数据采集管线。它还会把 leader arm 配置到 damping 与较低阻尼增益，适合人工拖动示教。

该模块适合未来双臂 leader-follower 或实体示教采集，但不适合作为当前单臂 agent 执行层的核心。当前可以把它列为 `LeRobot compatibility layer` 的参考实现和后续可选依赖。

### 综合接入判断

`armctrl` 不应从零实现所有 ARX5 + LeRobot 生态能力。更稳妥的做法是：

1. 以 `arx5-interface` / `arx5-sdk` 作为硬件执行内核。
2. 参考或复用 `arx5-common` 的 ARX5 wrapper、control mode、feature key 与 processor 思路。
3. 在 LeRobot 兼容层优先对齐 `lerobot-robot-arx5` 和 `lerobot-teleoperator-arx5` 的接口形态。
4. 将本项目新增价值集中在 `arx5d`、`arx5ctl`、`SafetyGuard`、`RecipeRegistry`、状态机、错误码、动作审计和 Roboclaw submodule 边界。

这样可以避免重复造 LeRobot 插件，同时保留 `armctrl` 作为 agent-safe 控制仓的必要性。

## SJTU RoboClaw 代码仓观察

调研对象为 `MINT-SJTU/RoboClaw`：`https://github.com/MINT-SJTU/RoboClaw`。

从公开仓库结构与 README 看，该项目的中心不是某一个“机械臂底层控制器”，而是围绕 `LeRobot` 生态组织的 embodied workflow。它的 `README` 明确给出两类主要用法：一类是直接在命令行运行 `roboclaw` 进入 workflow；另一类是通过 UI 启动控制台。仓库内部出现 `roboclaw/embodied/executor.py`、`roboclaw/embodied/command/builder.py`、`roboclaw/embodied/lock.py`、`roboclaw/embodied/service/*` 等结构，说明它更偏“执行服务、命令构造、会话管理、数据/训练工作流”的系统，而不是只提供机械臂伺服控制层。

该仓库里的 `Executor` 设计值得参考。它不是让上层把底层控制命令散落在各处，而是通过 executor 统一接受请求、获取锁、加载 dataset / model / robot、调用 command builder、执行操作，并在结束后释放锁。这种“执行入口集中化”的思路，与 `armctrl` 中建议的 `arx5d` 很接近：真实机械臂动作应通过一个集中入口进入，而不是让每个上层任务直接创建控制器对象。

它的 `CommandBuilder` 也值得借鉴。`CommandBuilder` 根据 session、model、robot、dataset 等上下文组合执行命令，而不是把命令字符串散落在 UI 或外层逻辑里。对应到 `armctrl`，我们不应让 agent 拼接任意底层指令，而应通过 `RecipeRegistry` 或 command schema 生成受约束动作，例如 `go_observe`、`pregrasp_setup`、`retreat_safe`。

不过，SJTU RoboClaw 的执行层偏向论文系统工作流，不等同于 ARX5 机械臂底层执行层。它面向的是完整 embodied pipeline，包括模型、数据集、机器人接口、UI / CLI workflow。`armctrl` 则应更窄：只处理 ARX5 机械臂控制、状态、安全和动作 recipe。换言之，SJTU RoboClaw 可以作为“上层任务执行器如何组织”的参考，不适合作为“机械臂控制仓库该包含什么”的直接模板。

## 对 `armctrl` 的架构影响

本轮参考项目搜索后，`armctrl` 的仓库边界更明确：

```text
armctrl/
  src/armctrl/
    sdk_adapters/arx5/
    daemon/
    cli/
    protocol/
    safety/
    recipes/
    state/
    config/
    lerobot_compat/
  tests/
  configs/
  docs/
```

`Roboclaw` 系统仓只应消费它：

```text
Roboclaw/
  references/projects/armctrl         # submodule
  src/openclaw/communication          # 调 armctrl client / CLI
  src/openclaw/controller             # 任务编排，不直接持有 SDK 对象
  vision bridge / calibration         # 继续留在 Roboclaw 系统层
```

## Agent 场景下的接口分层

agent 不应直接访问底层 SDK，也不应直接控制高频闭环。推荐对 agent 开放三类能力：

第一类是只读状态能力，包括 `health`、`state`、`mode`、`active-command`、`last-error`。这类能力低风险，可作为默认工具暴露。

第二类是受安全检查约束的基础动作，包括 `home`、`damping`、`move-eef`、`gripper`、`cancel`。这些能力需要经过 `SafetyGuard`，并统一返回结构化 JSON。

第三类是 recipe 能力，包括 `go-observe`、`pregrasp-setup`、`retreat-safe`、`scan-small-arc` 等。对 agent 来说，recipe 比裸的 `move-joint` 更适合默认开放，因为它减少了自由度并保留了工程约束。

不建议默认暴露的能力包括 `calibrate-joint`、`calibrate-gripper`、增益修改、柔顺 / 阻抗参数、扭矩控制、原始 joint streaming。这些能力应归入维护模式或命名 profile，并需要人工确认或更高权限。

LeRobot 侧则不走 agent CLI，而是通过 compatibility layer 进入同一个 core control layer。这样可以让 `lerobot-record`、teleoperation 和 policy eval 与 agent tool 共享同一个安全执行入口。

## submodule 接入建议

`armctrl` 应先独立成仓，再由 `Roboclaw` 以 submodule 引入。建议挂载路径为：

```text
D:/repo/Roboclaw/references/projects/armctrl
```

在 `Roboclaw` 侧，只保留 thin client 或 CLI wrapper，不复制 `armctrl` 的控制逻辑。后续如果 `armctrl` 从 Python daemon 演进到 C++ daemon，只要协议兼容，`Roboclaw` 侧无需大改。

## 当前记录的边界说明

本轮对 SJTU RoboClaw 的本地 `git clone` 因网络连接 GitHub 超时失败，因此源码细节主要依据 GitHub 页面、raw 文件、README 和仓库结构观察得出。已核验到的方向足够支撑架构层判断，但后续如果要逐文件分析其 `service/session`、`executor`、`command` 实现，建议在网络稳定时重新浅克隆仓库做源码级复核。

## 参考链接

- `https://github.com/MINT-SJTU/RoboClaw`
- `https://github.com/MINT-SJTU/RoboClaw/tree/main/roboclaw`
- `https://github.com/MINT-SJTU/RoboClaw/tree/main/roboclaw/embodied`
- `https://raw.githubusercontent.com/MINT-SJTU/RoboClaw/main/README.md`
- `https://raw.githubusercontent.com/MINT-SJTU/RoboClaw/main/roboclaw/embodied/executor.py`
- `https://raw.githubusercontent.com/MINT-SJTU/RoboClaw/main/roboclaw/embodied/command/builder.py`
- `https://github.com/iamlab-cmu/frankapy`
- `https://iamlab-cmu.github.io/frankapy/`
- `https://github.com/facebookresearch/fairo/tree/main/polymetis`
- `https://facebookresearch.github.io/fairo/polymetis/`
- `https://github.com/jimmyyhwu/tidybot2`
- `https://www.jimmyyhwu.com/tidybot2/`
- `https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html`
- `https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo`
- `https://github.com/xArm-Developer/xarm_ros2`
- `https://github.com/xArm-Developer/XArm-Python-SDK`
- `https://github.com/hello-robot/stretch_ai`
- `https://github.com/robotmcp/ros-mcp-server`
- `https://github.com/huggingface/lerobot`
- `https://huggingface.co/docs/lerobot/main/integrate_hardware`
- `https://github.com/real-stanford/umi-arx`
- `https://github.com/real-stanford/umi-on-legs`
- `https://pypi.org/project/arx5-common/`
- `https://pypi.org/project/arx5-interface/`
- `https://pypi.org/project/lerobot-robot-arx5/`
- `https://pypi.org/project/lerobot-teleoperator-arx5/`

## ARX5 官方资料与旧版 X5 参数补充（2026-04-14）

本轮补充调研的重点不是再找新的二手转载，而是把 ARX 官方公开仓库里已经放出的手册、旧版说明和当前 `arx5-sdk` 控制配置收拢起来，回答三个更直接的问题：第一，你手上的机械臂是否更像“旧版 X5”；第二，旧版 X5 公开可核验到哪些参数；第三，当前 `arx5-sdk` 的 `X5` 配置是否可以直接套用到旧版 X5。

### 旧版 X5 的识别线索

现有公开资料里，`ARXroboticsX/ARX_X5` 的新版单臂 ROS2 话题说明已经把两代 X5 区分开：`X5（2023）` 被标注为“标准单臂”，`launch` 文件走 `v1` 目录，夹爪是“单轨二指夹爪”；`X5（2025）` 被标注为 `AC one` 上的机械臂，`launch` 文件用 `v2` 标识，夹爪是“双轨二指夹爪”。这意味着如果你手上的设备是独立桌面单臂、不是 AC one 机体上的新版臂，且软件资料更接近 `open_single_arm.launch.py` 一类命名，那么它更大概率属于旧版 `X5（2023）` 这一支，而不是 `AC one / X5（2025）`。这里仍然是基于官方文档结构做的判断，不等于已经坐实你这台实物的具体出厂版本；最终仍应结合夹爪结构、控制箱、launch 命名和实机串号再核一次。来源：`ARX_X5/00-readme/01-X5&AC one-单臂ROS2话题说明 .pdf`，原始链接：`https://github.com/ARXroboticsX/ARX_X5/tree/main/00-readme`

### 旧版 X5 目前能坐实的参数

下表只保留这轮从官方 PDF 和 `arx5-sdk` 源码里明确核到的内容，不把未坐实的商品页参数混进来。

| 参数项 | 当前可核验值 | 主要来源 | 备注 |
|---|---|---|---|
| 机械臂型号 | `X5` | `01-python-单臂X5-SDK.pdf` / `03-ROS2-单臂X5-SDK.pdf` | 旧版单臂资料均以 `X5` 命名 |
| 供电 | `DC24V` | `01-python-单臂X5-SDK.pdf` / `03-ROS2-单臂X5-SDK.pdf` / `组装&环境配置手册.pdf` | 三份手册一致 |
| 单臂 bringup 通信 | `USB2CAN`（旧版手册）/ `ARX 通信模块`（新版组装手册） | 旧版 Python/ROS2 手册；新版组装手册 | 说明接口方案存在代际变化 |
| 夹爪开合范围 | `0-80 mm` | 旧版 Python/ROS2 手册；新版组装手册 | 一致 |
| 夹爪反馈/控制方式 | `位置 / 速度 / 扭矩` | 旧版 Python/ROS2 手册；新版组装手册 | 一致 |
| 末端接口 | `xt30 2+2` | 旧版 Python/ROS2 手册；新版组装手册 | 可作为末端供电/扩展线索 |
| 夹爪重量 | `约 585 g`（旧版）/ `约 510 g`（新版组装手册） | 旧版 Python/ROS2 手册；新版组装手册 | 不同文档有差异 |
| 最大夹持力 | `2NM`（旧版 Python 手册）/ `10NM`（旧版 ROS1/ROS2 手册）/ `2N`（新版组装手册） | 三份官方 PDF | 明显冲突，不能直接当成定值 |
| ROS2 控制模式 | `0 力矩清零`、`1 复位`、`2 阻尼`、`3 重力补偿`、`4 末端位姿控制`、`5 关节控制` | `03-ROS2-单臂X5-SDK.pdf` | 旧版 ROS2 控制语义 |
| 旧版 ROS2 末端控制范围 | `x:[0,0.5]`、`y:[-0.5,0.5]`、`z:[-0.5,0.5]`、`roll/pitch/yaw: ±1.3 rad`、`gripper: 0-5 对应 0-80 mm` | `03-ROS2-单臂X5-SDK.pdf` | 这是旧版 ROS2 话题层给出的控制边界，不等于当前 `arx5-sdk` 全部边界 |
| 旧版 ROS2 关节限位 | `j1:[-3.14,2.6]`、`j2:[-3.6,0.1]`、`j3:[-1.57,1.57]`、`j4:[-1.3,1.3]`、`j5:[-1.57,1.57]`、`j6:[-2.1,2.1]` | `03-ROS2-单臂X5-SDK.pdf` | 与当前 `arx5-sdk` 的 `X5` 配置差异很大 |
| 当前 `arx5-sdk` 的 `X5` 控制配置 | `joint_pos_min=[-3.14,-0.05,-0.1,-1.6,-1.57,-2]`、`joint_pos_max=[2.618,3.50,3.20,1.55,1.57,2]`、`joint_vel_max=[5.0,5.0,5.5,5.5,5.0,5.0]`、`joint_torque_max=[30,40,30,15,10,10]`、`gripper_width=0.088 m`、`gripper_open_readout=5.03` | `real-stanford/arx5-sdk/include/app/config.h` | 更像当前 SDK 的控制模型，不应在未确认代际前直接套给旧版实机 |

这里最需要强调的是：旧版 `ARX_X5` ROS2 手册给出的关节限位，与 `real-stanford/arx5-sdk` 当前 `config.h` 里的 `X5` 配置明显不一致，尤其是第 2、3、4 关节范围差异很大。这个冲突不适合由文档作者替资料做裁决，只能先把它当成“版本差异或控制定义差异”的明确风险项。对你这个项目，工程上更稳妥的做法不是直接相信其中任何一份，而是在 bringup 期先用只读状态脚本核实零位、回读范围、软限位行为，再决定是否沿用当前 `arx5-sdk` 的 `X5` 模型，还是要按旧版手册另做 profile。

### 旧版 X5 的使用手册集合

这次已经把官方公开资料保存到本地目录：

`D:/repo/eediy_wiki/modules/private_projects/projects/armctrl/arx5_official_materials`

其中和你当前场景最相关的文件有：

| 本地文件 | 用途 | 原始来源 |
|---|---|---|
| `D:/repo/eediy_wiki/modules/private_projects/projects/armctrl/arx5_official_materials/ARX_X5/00-readme/旧版-readme/00-配置CAN手册.pdf` | 旧版 USB2CAN / udev / can 启动流程 | `https://github.com/ARXroboticsX/ARX_X5/tree/main/00-readme/%E6%97%A7%E7%89%88-readme` |
| `D:/repo/eediy_wiki/modules/private_projects/projects/armctrl/arx5_official_materials/ARX_X5/00-readme/旧版-readme/01-python-单臂X5-SDK.pdf` | 旧版 Python SDK bringup、硬件清单、异常处理 | 同上 |
| `D:/repo/eediy_wiki/modules/private_projects/projects/armctrl/arx5_official_materials/ARX_X5/00-readme/旧版-readme/03-ROS2-单臂X5-SDK.pdf` | 旧版 ROS2 bringup、控制模式、末端范围、关节限位 | 同上 |
| `D:/repo/eediy_wiki/modules/private_projects/projects/armctrl/arx5_official_materials/ARX_X5/00-readme/旧版-readme/06-X5-单臂ROS2话题说明.pdf` | 旧版 ROS2 话题使用说明与 `rqt` 调试入口 | 同上 |
| `D:/repo/eediy_wiki/modules/private_projects/projects/armctrl/arx5_official_materials/ARX_X5/00-readme/组装&环境配置手册.pdf` | 新版组装与环境配置说明，可用于比对代际差异 | `https://github.com/ARXroboticsX/ARX_X5/tree/main/00-readme` |
| `D:/repo/eediy_wiki/modules/private_projects/projects/armctrl/arx5_official_materials/ARX_X5/00-readme/ARX-CAN手册.pdf` | 新版 CAN 绑定说明 | 同上 |
| `D:/repo/eediy_wiki/modules/private_projects/projects/armctrl/arx5_official_materials/ARX_X5/00-readme/01-X5&AC one-单臂ROS2话题说明 .pdf` | 同时区分 `X5(2023)` 与 `X5(2025)` 的新版 ROS2 话题说明 | 同上 |
| `D:/repo/eediy_wiki/modules/private_projects/projects/armctrl/arx5_official_materials/real-stanford_arx5-sdk/README.md` | 当前官方 SDK 使用说明、安装方式、安全边界 | `https://github.com/real-stanford/arx5-sdk` |
| `D:/repo/eediy_wiki/modules/private_projects/projects/armctrl/arx5_official_materials/real-stanford_arx5-sdk/config.h` | 当前 `X5`/`L5` 控制配置源 | `https://raw.githubusercontent.com/real-stanford/arx5-sdk/main/include/app/config.h` |

### 对当前项目最有用的结论

如果你下一步的目标是把旧版 X5 在 `Ubuntu 24.04 + ROS2 Jazzy + uv` 的 bringup 先跑通，那么最应该优先相信的是“当前 `arx5-sdk` 的安装方式”和“旧版 `ARX_X5` 手册的设备代际线索”，而不是把旧版 PDF 的所有控制边界原样照搬进新脚本。更具体地说：

第一，当前软件安装链路优先沿用 `real-stanford/arx5-sdk` / `arx5-interface`，因为这是你已经在 N100D 上跑通 import 的链路，也是当前维护中的 SDK 主线。

第二，旧版实机识别优先看三件事：是不是标准单臂、launch 是否更接近 `v1`、夹爪是不是单轨二指结构。如果这三条都更像旧版，就不要默认把 `AC one / X5(2025)` 的假设套进去。

第三，旧版 ROS2 手册给出的 `mode` 语义、`rqt` 调试习惯和小步长验证建议仍然很有参考价值，尤其是“除夹爪外，角度先不要超过 ±0.1 验证链路”和“只保留一个控制终端”的操作边界，这和我们当前 `check_arx5_state.py`、`check_arx5_min_motion.py` 的 bringup 安全策略是相容的。

第四，参数冲突本身就是结论的一部分。夹爪力、夹爪重量、关节限位在不同官方资料之间都存在差异，说明这条产品线的文档并没有完全标准化。对 `armctrl` 来说，这不是“再找一份手册就能解决”的问题，而是 bringup 阶段必须把软限位、零位和夹爪开口做成现场核验项。

### 本轮新增参考链接

- `https://github.com/ARXroboticsX/ARX_X5`
- `https://github.com/ARXroboticsX/ARX_X5/tree/main/00-readme`
- `https://github.com/ARXroboticsX/ARX_X5/tree/main/00-readme/%E6%97%A7%E7%89%88-readme`
- `https://github.com/ARXroboticsX/ARX_all_in_one_readme`
- `https://github.com/real-stanford/arx5-sdk`
- `https://raw.githubusercontent.com/real-stanford/arx5-sdk/main/include/app/config.h`

## USB-CAN 连接正确性排查记录（2026-04-15）

本次现场现象是：运行 `check_arx5_state.py` 时，尝试 `can0`、`can1`、`can2` 等接口名均失败，系统提示找不到对应 CAN 接口；进一步确认后，设备实际只枚举成了 `/dev/ttyACM0`，尚未挂载或转换成 Linux SocketCAN 网络接口 `can0`。这说明问题首先不在 `check_arx5_state.py` 的 ARX5 SDK 调用层，而在 USB-CAN 适配器尚未完成系统侧接口初始化。

判断链路应按以下顺序进行。第一步不直接猜 `can0`，而是先用 `ip a` 或 `ip -details link show type can` 查看系统当前是否已经存在 `can*` 网络接口；如果不存在，再用 `ls /dev/ttyACM*` 和插拔时的 `dmesg -w` 判断 USB-CAN 被内核识别成了哪类设备。若出现的是 `/dev/ttyACM0` / `/dev/ttyACM1`，通常说明适配器走的是 `SLCAN` 路径，此时它还只是串口设备，并不会自动变成 `can0`。若插上后直接出现 `can0` / `can1`，通常才是 `candleLight` 这类直接暴露 SocketCAN 网口的适配器。若出现的是 `enx...` 或 `eth...`，则更接近 EtherCAT-CAN 或 USB 网卡路径，传给 SDK 的 interface 也不应写成 `can0`。

对本次 `/dev/ttyACM0` 场景，正确处理逻辑是先把串口桥接为 SocketCAN 接口，再运行 ARX5 状态脚本。最小手动命令为：

```bash
sudo apt install can-utils net-tools
ls /dev/ttyACM*
sudo slcand -o -f -s8 /dev/ttyACM0 can0
sudo ip link set up can0
ip -details link show can0
```

确认 `can0` 已经出现在 `ip a` 之后，再运行：

```bash
python scripts/bringup/check_arx5_state.py --model X5 --interface can0 --json
```

如果插拔后 `/dev/ttyACM0` 序号变化，临时命令会失效；稳定做法是参考官方 `ARX-CAN手册.pdf` 和旧版 `00-配置CAN手册.pdf`，用 `udev` 规则把 USB-CAN 的 serial 绑定到固定符号链接，例如 `/dev/arxcan0`，再由 `slcand` 挂成固定的 `can0`。这也是官方文档里要求先执行 `search.sh`、修改 `arx_can.rules`、再执行 `set.sh` / CAN 启动脚本的原因。

这条排查逻辑对 `armctrl` 的设计也有直接影响：`check_arx5_import.py` 只验证 Python 包和动态库；`check_arx5_state.py --dry-run` 只验证 SDK 配置加载；真正的 `check_arx5_state.py` 需要系统中已经存在可用的 CAN / EtherCAT-CAN interface。因此后续 bringup 脚本应补一个更靠前的 `check_can_interface.py` 或 `check_transport.py`，专门输出当前检测到的 `/dev/ttyACM*`、`can*`、`enx*`，并明确告诉用户下一步该走 `SLCAN -> slcand -> can0`、`candleLight -> ip link set up`，还是 EtherCAT-CAN interface 路径，避免把底层接口未初始化误判为 ARX5 SDK 不兼容。
