# armctrl

## 这个目录做什么

`armctrl` 是机械臂控制子系统的 Python 包入口。
它把真实 SDK、fake 适配器、安全检查、CLI、手柄映射和 GUI 调试面板整理成一套统一接口。

## 主要组件

- `adapters/`：硬件适配层，负责把 fake 或真实 SDK 翻译成统一接口。
- `calibration/`：项目侧标定层，负责保存和应用夹爪等标定结果。
- `protocol/`：全工程共享的数据模型、枚举和错误对象。
- `daemon/`：应用服务层，负责把命令、安全检查和 adapter 执行串起来。
- `safety/`：运动限位、调试 profile 权限和请求合法性检查。
- `teleop/`：Xbox 输入解析、增量控制映射和调试 UI。
- `identification/`：参数辨识激励轨迹、硬件采集、数据集导出和离线工具交接。
- `cli/`：`arx5ctl` 进程入口。

## 实现思路

这个包采用很克制的分层：

1. `protocol` 定义数据契约；
2. `calibration` 保存项目经验参数，并在连接 SDK 前统一应用；
3. `adapters` 连接外部世界；
4. `adapters/arx5/control_profiles.py` 负责 ARX5 专属的 gain profile 定义；
5. `daemon/executor.py` 作为统一入口调度；
6. `safety/profiles.py` 统一保存运动限值和 debug profile 元信息；
7. `cli` 和 `teleop` 负责与人交互。
8. `identification` 负责关节空间辨识数据链路，与实时 teleop 分开。

这样做的好处是 bringup 阶段即便硬件、输入设备或 UI 形态变化，
中间的数据契约和执行骨架仍然稳定。

## 这轮调试记录

- X5 带 D435i 时，不能再直接沿用 SDK wheel 内的原始 `X5.urdf`。
  当前 `sdk` 适配器和 `identification` 后端都会优先选项目侧 `configs/models/X5_camera.urdf`。
- SDK 运行日志已经确认：运动学末端使用 `eef_link`，逆动力学最后一节使用 `link6`。
  所以 payload 只能并入 `link6 <inertial>`，写在 `eef_link` 上不会进入重力补偿。
- `damping` 是安全阻尼态，不是保持姿态的重力补偿态。
  从 `damping` 恢复 teleop 时需要先渐变恢复增益，再发送新的末端命令。
- 当前 teleop 接管新增了一条更严格的安全语义：
  从任意非 `teleop` 状态恢复时，第一拍先把当前实测末端/夹爪状态同步成新的 teleop 基线，
  并下发一条“保持当前位置”的命令；第二拍起才开始累加摇杆增量。
- 当前 deadman 松手后的默认落态已经改成 `zero_gravity_drag`。
  它会保留启动时已有的重力补偿，并把增益降到很低，手感比纯 `damping` 更轻。
  真正的退出安全态仍然保留为 `damping`。
- `teleop` 和 `zero_gravity_drag` 这两套 ARX5 专属增益 profile
  已经从 `sdk.py` 的流程代码中抽到 `adapters/arx5/control_profiles.py`。
  `sdk.py` 现在只负责状态切换、目标同步和 ramp，不再同时承担参数仓库职责。
- 目前项目里凡是涉及 X5 动力学模型的路径，都尽量走同一份项目侧 URDF，
  避免 teleop、辨识采集、离线分析各自读到不同模型。
- 夹爪方向和开口宽度的修正，也已经从“启动时临时传参”改成项目侧标定文件。
  当前约定路径是 `configs/calibration/gripper/<model>.json`，
  `sdk` 适配器和 `identification` 后端都会在 connect 前自动读取它。

## 建议阅读顺序

1. `protocol/models.py`
2. `adapters/base.py`
3. `daemon/executor.py`
4. `safety/guard.py`
5. `teleop/mapping.py`
6. `teleop/xbox.py`
7. `cli/arx5ctl.py`
8. `identification/trajectories.py`
9. `identification/optimization.py`
10. `identification/runner.py`
