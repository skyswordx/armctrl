# calibration

## 这个目录做什么

这里放项目侧的标定模块。
当前先实现夹爪标定，
把 SDK 原生交互流程和项目配置文件接起来。

## 主要组件

- `models.py`
  - 定义 `GripperCalibration`；
  - 定义把标定结果应用到 SDK `RobotConfig` 的辅助函数。
- `store.py`
  - 负责 `configs/calibration/gripper/<model>.json` 的读写。
- `gripper.py`
  - 复用 SDK `calibrate_gripper()`；
  - 在终端补充 SDK 打印值和开口宽度输入；
  - 保存最终标定结果。

## 实现思路

项目层不重写 SDK 的夹爪标定算法。
真正的“闭合后设零点、张开后读取电机角度”仍然由 SDK 完成。
`armctrl` 只负责两件事：

1. 把标定结果保存成项目配置，避免每次启动都临时传参数；
2. 在 teleop、health、identification 这些链路连接 SDK 前自动应用这份配置。

## 当前约束

- Python 绑定没有直接暴露底层 CAN 原始电机消息；
- 可以用 SDK 源码里的换算公式从 `JointState` 反推出电机读数；
- 但 wizard 流程里不再拿这个反推值做最终保存。

这轮排障确认过两个关键细节：

1. `Fully-open joint position readout` 以 SDK 终端打印值为准；
2. wizard 构造 joint controller 时要关闭 `background_send_recv`，
   否则旧夹爪配置可能在校准刚结束时触发一次 SDK 合法性检查，导致还没保存新值就先进入 emergency。

这样可以不改 vendor SDK，也能把项目侧标定闭环做完整。
