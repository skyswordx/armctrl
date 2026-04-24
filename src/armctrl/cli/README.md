# cli

## 这个目录做什么

这里是命令行接口层。
用户直接运行的是 `arx5ctl`，
它负责把命令行参数转换成内部请求并调用执行器。

## 主要组件

- `arx5ctl.py`：唯一入口文件，包含参数解析、适配器创建、命令分发和结果输出。

## 实现思路

CLI 不直接实现控制逻辑。
它只做三件事：

1. 参数解析；
2. 组装 `executor`；
3. 按 `--json` 或 `--gui` 输出结果。

这样可以保证控制逻辑仍集中在 `daemon`、`safety` 和 `teleop`，
CLI 只是一个薄壳。

## 当前额外职责

除了普通控制命令，CLI 现在还承担项目侧夹爪标定入口：

- `gripper-calibration-show`
- `gripper-calibration-set`
- `gripper-calibration-clear`
- `gripper-calibration-wizard`

这些命令不直接改 vendor SDK 文件，
而是维护项目里的标定配置，再由 adapter / backend 在连接 SDK 前自动应用。
