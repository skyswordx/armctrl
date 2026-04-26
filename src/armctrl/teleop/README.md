# teleop

## 这个目录做什么

这里放手柄输入到机械臂增量控制的完整链路。
包括滤波、按键映射、事件读取、控制循环和 GUI 调试面板。

## 主要组件

- `filters.py`：低通滤波器。
- `mapping.py`：`XboxState` 到 `TeleopCommand` 的映射。
- `xbox.py`：Linux event / JSONL 输入、控制循环、快照提取、Tk GUI。

## 实现思路

这个目录把三件事明确拆开：

1. 输入设备层：读取原始事件；
2. 映射层：把手柄语义转换成 6D 增量和调试 profile；
3. 运行层：按固定频率发送控制并刷新界面。

输入读取、控制频率和 UI 频率解耦，是这个目录最重要的工程决定。

另外当前 teleop 会话在“模式切换 -> 重新接管”这一步采用两拍语义：

1. 第一拍只做接管同步，把当前实测末端/夹爪状态覆盖成新的 teleop 基线；
2. 第二拍开始才把摇杆增量累加到这条基线上。

这样做是为了让手柄层、executor 积分状态和 SDK 内部控制目标先重新对齐，
避免从 `damping`、`zero_gravity_drag` 或其他 maintenance profile 切回时出现跳变。
