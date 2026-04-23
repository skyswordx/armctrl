# adapters/arx5

## 这个目录做什么

这里放 ARX5 机械臂的两套实现：

- `fake.py`：离线仿真适配器；
- `sdk.py`：真实方舟 SDK 适配器。

## 组件说明

- `fake.py`
  - 用纯 Python 状态模拟机械臂反馈；
  - 适合单测、GUI 联调、手柄映射调试；
  - 不依赖 CAN、驱动或 SDK 安装。
- `sdk.py`
  - 动态导入 `arx5_interface`；
  - 复用 SDK 的控制器、状态对象和调试接口；
  - 把 SDK 结果转换成 armctrl 的统一响应。

## 实现思路

这个目录的关键原则是：优先复用 SDK 已有能力。
尤其是重力补偿、控制器参数、状态时间戳和命令对象，
都应该由 SDK 负责。
本项目只负责边界转换、统一错误和安全接入。

## 最近确认的调试事实

- X5 带相机时，`sdk.py` 默认会把 `robot_config.urdf_path` 指到项目侧 `configs/models/X5_camera.urdf`。
  这是为了让 SDK 控制器直接读取带 payload 的模型，不再沿用 wheel 内原始空载模型。
- SDK 运行时会显示 `Using eef_link for kinematics and link6 for inverse dynamics`。
  这个细节决定了 payload 不能只写在 `eef_link`，必须并入 `link6 <inertial>`。
- `set_to_damping()` 之后控制器会落到零刚度阻尼态。
  如果此时直接恢复 teleop 命令，机械臂容易出现“命令发了但不明显动”或恢复瞬间抖动，
  所以 `sdk.py` 里专门加了按 `controller_dt` 渐变恢复增益的逻辑。
