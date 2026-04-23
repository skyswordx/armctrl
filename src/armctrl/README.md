# armctrl

## 这个目录做什么

`armctrl` 是机械臂控制子系统的 Python 包入口。
它把真实 SDK、fake 适配器、安全检查、CLI、手柄映射和 GUI 调试面板整理成一套统一接口。

## 主要组件

- `adapters/`：硬件适配层，负责把 fake 或真实 SDK 翻译成统一接口。
- `protocol/`：全工程共享的数据模型、枚举和错误对象。
- `daemon/`：应用服务层，负责把命令、安全检查和 adapter 执行串起来。
- `safety/`：运动限位、调试 profile 权限和请求合法性检查。
- `teleop/`：Xbox 输入解析、增量控制映射和调试 UI。
- `cli/`：`arx5ctl` 进程入口。

## 实现思路

这个包采用很克制的分层：

1. `protocol` 定义数据契约；
2. `adapters` 连接外部世界；
3. `daemon/executor.py` 作为统一入口调度；
4. `safety` 在执行前做拒绝和约束；
5. `cli` 和 `teleop` 负责与人交互。

这样做的好处是 bringup 阶段即便硬件、输入设备或 UI 形态变化，
中间的数据契约和执行骨架仍然稳定。

## 建议阅读顺序

1. `protocol/models.py`
2. `adapters/base.py`
3. `daemon/executor.py`
4. `safety/guard.py`
5. `teleop/mapping.py`
6. `teleop/xbox.py`
7. `cli/arx5ctl.py`
