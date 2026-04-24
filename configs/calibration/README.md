# calibration configs

这里保存项目侧标定文件。

当前已约定：

- 夹爪标定路径：`configs/calibration/gripper/<model>.json`

这些文件由 `arx5ctl` 的夹爪标定命令生成和修改，
普通运行命令会在创建 SDK controller 前自动读取并应用。

这样做是为了替代过去那种直接在命令行临时覆盖 SDK 参数的方式。
