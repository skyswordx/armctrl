# armctrl 文档索引

当前分支只保留少量权威入口，避免继续产生找不到主线的长文档。

## 必读入口

1. `../README.md`: 项目定位和开发入口。
2. `../ROADMAP.md`: 系统边界、milestone 和发布轨道。
3. `../CHANGELOG.md`: 用户可见变更和发版日志。
4. `docs/README.md`: 本文档索引。

## 外部参考

clean rebuild 分支不直接携带历史长文档和官方 PDF 快照。需要查历史材料时，从旧分支或 vendor upstream 显式取回，并在本文登记用途。

长期有效的决策优先写入 `ROADMAP.md`，用户可见变化写入 `CHANGELOG.md`。

## 文档规则

- 不新增未索引顶层文档。
- 新实现先更新 roadmap/changelog，再迭代代码。
- 实验记录必须明确标注数据来源、硬件状态和是否可复现。
