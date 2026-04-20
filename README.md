
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

## 当前 Bringup 状态

当前硬件 bringup 工作分支为 `feature/hardware-bringup`。N100D 已作为第一台正式 bringup 主机，当前状态如下：

| 项目            | 当前值                                                                                   |
| --------------- | ---------------------------------------------------------------------------------------- |
| 主机            | N100D /`skyswordx`                                                                     |
| 仓库路径        | `/home/circlemoon/work/armctrl`                                                        |
| 分支            | `feature/hardware-bringup`                                                             |
| OS              | Ubuntu 24.04.4 LTS                                                                       |
| 架构            | `x86_64`                                                                               |
| 系统 Python     | Python 3.12.3                                                                            |
| ROS2            | Jazzy，`/opt/ros/jazzy/setup.bash`                                                     |
| Python 环境管理 | `uv 0.11.6`                                                                            |
| venv            | `.venv`，由 `uv venv --python /usr/bin/python3.12 --system-site-packages .venv` 创建 |
| ARX5 Python SDK | `arx5-interface==0.1.2`                                                                |

已验证通过：

- `.venv` 内 Python 指向 `/home/circlemoon/work/armctrl/.venv/bin/python`
- `rclpy` 可从 ROS2 Jazzy 系统路径导入
- `arx5_interface` 可从 `.venv` 导入

## N100D uv + ROS2 启动流程

N100D 上 ROS2 Jazzy 使用系统 Python 包，因此 `armctrl` 的 bringup 环境必须基于系统 Python 创建，并开启 `--system-site-packages`，否则后续使用 `rclpy` 时容易找不到 ROS2 包。

首次初始化：

```bash
cd ~/work/armctrl

curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"

uv venv --python /usr/bin/python3.12 --system-site-packages .venv
source .venv/bin/activate

uv pip install -U pip setuptools wheel
uv pip install arx5-interface
```

每次进入 bringup 环境：

```bash
cd ~/work/armctrl

export PATH="$HOME/.local/bin:$PATH"
export http_proxy=${http_proxy:-http://127.0.0.1:7890}
export https_proxy=${https_proxy:-http://127.0.0.1:7890}
export all_proxy=${all_proxy:-http://127.0.0.1:7890}

source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
```

环境验证：

```bash
python - <<'PY'
import sys
print("python_executable:", sys.executable)
print("python_version:", sys.version)

import rclpy
print("rclpy:", rclpy.__file__)

import arx5_interface
print("arx5_interface:", arx5_interface.__file__)
PY
```

注意：不要在 `source /opt/ros/jazzy/setup.bash` 前启用 `set -u`。ROS2 setup 脚本会访问部分未预先定义的环境变量，`set -u` 会导致类似 `AMENT_TRACE_SETUP_FILES: unbound variable` 的错误。bringup 脚本建议使用 `set -eo pipefail`。

## Hardware Bringup 脚本布局规划

当前阶段已经先落地三个最小脚本，用于验证 ARX5 SDK 在 N100D 上的 Python 导入、配置加载、状态读取入口以及最小运动安全门。该阶段不直接实现完整 daemon / CLI，而是先用小脚本确认官方 SDK 能否在目标机上稳定工作。

已实现脚本：

| 脚本                                         | 默认行为                                                                                                   |                            是否连接硬件 | 是否发送运动命令 |
| -------------------------------------------- | ---------------------------------------------------------------------------------------------------------- | --------------------------------------: | ---------------: |
| `scripts/bringup/check_arx5_import.py`     | 检查 `arx5_interface` 必需符号，可选检查 `rclpy`                                                       |                                      否 |               否 |
| `scripts/bringup/check_arx5_state.py`      | 按官方示例加载 `RobotConfigFactory` 与 `ControllerConfigFactory`；加 `--dry-run` 时不创建 controller | `--dry-run` 否；默认会创建 controller |               否 |
| `scripts/bringup/check_arx5_min_motion.py` | 只输出 plan，不运动；必须 `--execute` 后才进入人工确认                                                   |                     仅 `--execute` 后 |         仅确认后 |

当前已在 N100D 上完成无运动验证：

```bash
python -m py_compile scripts/bringup/*.py
python scripts/bringup/check_arx5_import.py --check-ros --json
python scripts/bringup/check_arx5_state.py --model X5 --interface can0 --dry-run --json
python scripts/bringup/check_arx5_min_motion.py --model X5 --interface can0 --json
```

其中 `check_arx5_state.py --dry-run --json` 已处理 SDK 原生 stdout/stderr 日志，输出可以被 `python -m json.tool` 正常解析；`check_arx5_min_motion.py` 在不带 `--execute` 时固定为 `plan_only`，不会实例化控制器或发送命令。

下一步接线与上电后，先运行状态读取，不运行运动脚本：

```bash
python scripts/bringup/check_arx5_state.py --model X5 --interface can0 --json
```

只有在状态读取稳定、机械臂处于安全空间、急停和供电状态确认无误后，才考虑最小运动测试。执行最小运动测试必须显式传入 `--execute`，并输入确认短语 `I UNDERSTAND THIS WILL MOVE THE ARM`；默认命令不会运动机械臂。

建议布局如下：

```text
armctrl/
  docs/
    bringup/
      n100d_ubuntu2404_jazzy.md
      arx5_interface_smoke_test.md
  scripts/
    bringup/
      check_env.py
      check_arx5_import.py
      check_arx5_state.py
      check_arx5_min_motion.py
  src/
    armctrl/
      sdk_adapters/
        arx5/
          README.md
```

各部分职责：

| 路径                                         | 职责                                                         |
| -------------------------------------------- | ------------------------------------------------------------ |
| `docs/bringup/`                            | 记录 N100D 环境、接线、执行命令、成功标准和失败现象          |
| `scripts/bringup/check_env.py`             | 检查 Python、ROS2、代理、基础系统环境                        |
| `scripts/bringup/check_arx5_import.py`     | 只验证 `arx5_interface` 可导入，不触碰硬件动作             |
| `scripts/bringup/check_arx5_state.py`      | 基于官方示例创建 controller 并读取状态，验证通信链路         |
| `scripts/bringup/check_arx5_min_motion.py` | 在人工确认后执行最小安全动作，用于验证控制链路               |
| `src/armctrl/sdk_adapters/arx5/`           | 后续沉淀正式 adapter；bringup 脚本跑通后再把稳定逻辑迁入此处 |

阶段边界：

1. `check_arx5_import.py` 只验证 Python 包和动态库是否可用。
2. `check_arx5_state.py` 只验证 controller 创建、硬件连接和状态读取，不发送运动命令。
3. `check_arx5_min_motion.py` 必须默认要求人工确认，并且只允许很小的、可回退的安全动作。
4. 只有当状态读取与最小动作都稳定后，才把稳定代码迁入 `src/armctrl/sdk_adapters/arx5/`，再进入 `arx5d` / `arx5ctl` 设计。

这样组织的好处是：bringup 分支可以快速验证硬件，同时为未来合并到 `develop` 保留清晰边界。脚本中的临时探测逻辑不会直接污染生产模块，真正稳定的 SDK 包装逻辑再逐步沉淀到 `src/armctrl/`。
