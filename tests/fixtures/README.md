# tests/fixtures

## 这个目录做什么

这里保存测试和回放用的静态样本。

## 当前内容

- `xbox_sample.jsonl`：录制好的手柄事件序列。

## 用途

这个文件让 teleop 链路可以在没有真实手柄、没有蓝牙和没有 `/dev/input/event*` 的情况下复现。
它既能用于单元测试，也能用于 CLI 的离线联调。
