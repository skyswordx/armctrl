# tests

## 这个目录做什么

这里放 armctrl 的自动化验收。
测试目标不是覆盖率数字本身，
而是把关键工程约束固定下来，避免 bringup 后期反复回退。

## 子目录

- `unit/`：单元测试，覆盖协议、执行器、安全层、手柄映射和 GUI 快照。
- `fixtures/`：测试数据样本，例如手柄事件回放文件。

## 建议用法

常用命令：

- `uv run pytest`
- `uv run pytest tests/unit/test_xbox_mapping.py`
