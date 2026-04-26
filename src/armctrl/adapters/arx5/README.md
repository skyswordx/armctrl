# adapters/arx5

## 这个目录做什么

这里放 ARX5 机械臂的两套实现：

- `fake.py`：离线仿真适配器；
- `sdk.py`：真实方舟 SDK 适配器。
- `control_profiles.py`：ARX5 专属 gain profile 定义。

## 组件说明

- `fake.py`
  - 用纯 Python 状态模拟机械臂反馈；
  - 适合单测、GUI 联调、手柄映射调试；
  - 不依赖 CAN、驱动或 SDK 安装。
- `sdk.py`
  - 动态导入 `arx5_interface`；
  - 复用 SDK 的控制器、状态对象和调试接口；
  - 把 SDK 结果转换成 armctrl 的统一响应。
- `control_profiles.py`
  - 保存 `teleop`、`zero_gravity_drag` 这两套运行时 gain 定义；
  - 支持“统一缩放 + 逐关节乘子”的组合表达；
  - 让 `sdk.py` 只负责模式切换流程，不再同时硬编码 profile 参数。

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
- 项目现在显式保留了 `teleop` profile。
  它的语义不是“另一套调参”，而是明确复用 SDK 默认 MIT 增益，
  给按钮映射、状态切换和测试一个统一的“恢复标准操控手感”入口。
- 项目现在额外定义了 `zero_gravity_drag` profile。
  它不会关闭已有的重力补偿，而是先把目标同步到当前实测姿态，再把增益降到很低，
  用来替代 deadman 松手后的默认落态。
- `zero_gravity_drag` 现在不再只有一个全局 `kd_scale`。
  它保留统一极低阻尼，同时允许对 `joint2 / joint3` 这类主承载关节单独再降一档，
  用来修正 shoulder / elbow 手拖时明显比 wrist 更难动的问题。
- `sdk.py` 里 teleop 恢复默认增益的判断，现在按“当前模式是否已经是 `teleop`”处理，
  不再用“kp 是否为零”猜测当前是不是 damping。
  这是为了兼容非零低增益的 `zero_gravity_drag` profile，
  也就是修掉“第一次 RB 松开有效，第二次状态切换不对”的问题。
- 真正的 `damping` 仍然保留，给显式安全停机、`X` 键和 teleop 退出使用。
- 某些近期批次的 X5 夹爪读数方向和 SDK 默认值相反。
  现在不再通过临时 CLI 参数覆盖。
  项目会在 connect 前自动读取 `configs/calibration/gripper/<model>.json`，
  把保存好的 `gripper_open_readout` 和 `gripper_width` 写回 SDK `robot_config`。
  如果还没有标定文件，可以先用 `arx5ctl gripper-calibration-set` 写 bootstrap 值，
  再用 `arx5ctl gripper-calibration-wizard` 走完整校准流程。
