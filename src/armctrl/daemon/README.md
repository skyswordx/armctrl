# daemon

## 这个目录做什么

这里放应用服务层。
当前核心是 `executor.py`，
负责把“请求对象 -> 安全校验 -> 适配器执行 -> 统一响应”串成完整链路。

## 主要组件

- `executor.py`：命令执行器，处理 health、state、move、teleop、debug profile、cancel。

## 实现思路

`ArmCommandExecutor` 不关心底层是 fake 还是真 SDK。
它只关心两件事：

1. 这个请求安不安全；
2. 这个请求该调用 adapter 的哪个统一方法。

teleop 的“目标积分”也放在这里，
因为它属于业务执行语义，不属于手柄映射层。
