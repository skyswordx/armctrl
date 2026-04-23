# adapters

## 这个目录做什么

这一层是硬件边界。
上层只认识 `ArmAdapter` 协议，不直接接触具体 SDK。

## 主要组件

- `base.py`：定义统一适配器协议。
- `arx5/`：ARX5 机械臂的具体实现。

## 实现思路

这里采用 `Adapter`（适配器）模式。
`executor` 只依赖抽象协议，
因此可以在 `fake` 和 `sdk` 之间切换而不改业务层。

## 扩展方式

如果以后要接别的机械臂：

1. 新建一个子目录；
2. 实现 `ArmAdapter` 规定的方法；
3. 在 CLI 或工厂逻辑里注册；
4. 复用现有 `protocol`、`safety` 和 `executor`。
