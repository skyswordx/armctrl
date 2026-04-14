# armctrl 分支策略

## 主干职责

### `main`

- 作为发布分支使用
- 不直接推送
- 仅通过 Pull Request 合并
- 用于承接来自 `develop` 的稳定发布内容

### `develop`

- 作为日常集成分支使用
- 不直接推送
- 仅通过 Pull Request 合并
- 必须保持所有功能处于开启状态，不保留为测试临时关闭的逻辑

## 工作分支

### `feature/*`

- 从 `develop` 拉出
- 命名格式：`feature/<topic>`
- 允许自由推送
- 功能完成并通过测试后，向 `develop` 提交 Pull Request

### `hotfix/*`

- 从 `develop` 拉出
- 命名格式：`hotfix/<topic>`
- 不回退已进入 `develop` 的提交
- 修复完成后，向 `develop` 提交 Pull Request

## 当前阶段建议分支

- `feature/hardware-bringup`：验证硬件接线、控制通路与最小动作链
- `feature/agent-cli`：建立 `arx5ctl` / agent 工具入口
- `feature/lerobot-compat`：补齐 LeRobot 兼容层
- `feature/perf-hardening`：做性能优化、稳定性与安全加固

## 合并原则

- 日常开发：`feature/*` -> `develop`
- 问题修复：`hotfix/*` -> `develop`
- 发布集成：`develop` -> `main`
