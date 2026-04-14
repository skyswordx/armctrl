# armctrl

`armctrl` 是面向 ARX5 机械臂控制的独立仓库，定位为 `Roboclaw` 未来以 submodule 引入的控制侧代码仓。

## 当前定位

- `main`：发布分支，仅接收经 Pull Request 合并的变更
- `develop`：开发主分支，仅接收经 Pull Request 合并的变更
- `feature/*`：功能开发分支，可自由提交，完成后合并回 `develop`
- `hotfix/*`：问题修复分支，从 `develop` 拉出，修复后合并回 `develop`

## 当前阶段分支规划

- `feature/hardware-bringup`
- `feature/agent-cli`
- `feature/lerobot-compat`
- `feature/perf-hardening`

## 目标分层

- `core control layer`
- `agent CLI layer`
- `LeRobot compatibility layer`

详细分支策略见 `docs/branching_strategy.md`。
