# tests/unit

## 这组测试覆盖什么

- `test_protocol_models.py`：协议与模型契约。
- `test_fake_adapter_and_safety.py`：fake adapter 与安全拒绝。
- `test_executor_and_cli.py`：执行器和 CLI 行为。
- `test_xbox_mapping.py`：手柄映射、控制拍和 GUI 快照。
- `test_identification.py`：参数辨识轨迹、采集、数据集和 CLI。
- `test_model_payload.py`：项目侧 URDF payload 是否合并进 SDK 逆动力学可读取的 link。

## 阅读建议

如果想快速理解系统边界，可以先读：

1. `test_protocol_models.py`
2. `test_executor_and_cli.py`
3. `test_xbox_mapping.py`

测试文件本身已经加了大量中文注释，
可以直接当成“系统行为说明书”阅读。
