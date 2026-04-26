# safety

## 这个目录做什么

这里负责所有“执行前阻断”逻辑，以及和安全检查直接相关的静态 profile 元信息。
机械臂真正动起来之前，先在这一层决定请求是否允许通过。

## 主要组件

- `profiles.py`
  - `MotionLimits`：运动限位；
  - `DebugProfile` / `DebugProfileRegistry`：调试 profile 的权限与运行语义元信息。
- `guard.py`：统一安全校验入口。

## 实现思路

安全层和 adapter 分开有两个原因：

1. fake 和 sdk 都要共享同一套安全规则；
2. 安全拒绝应该发生在命令进入硬件前，而不是出错后补救。

当前主要覆盖平移步长、工作空间、夹爪范围和调试 profile 权限。
`profiles.py` 现在同时承担 motion limits 和 debug profile 元信息，
不再拆成两个小文件来回跳。
