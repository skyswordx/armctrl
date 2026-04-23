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

## 当前可用的 `armctrl` 包和 Xbox 调试入口

当前仓库已经包含最小可用的 Python 包骨架、fake adapter、SDK adapter、命令执行器、安全检查、Xbox 输入映射和 `arx5ctl` 命令。
默认命令使用 `fake` adapter，不会连接硬件。

```bash
sudo slcand -o -f -s8 /dev/ttyACM0 can0 && sudo ip link set up can0


```

无硬件验证：

```bash
cd ~/Roboclaw/references/projects/armctrl

uv run --with pytest pytest tests/unit -v
uv run python -m compileall src tests
uv run arx5ctl health --adapter fake --json
uv run arx5ctl teleop-xbox --adapter fake \
  --event-jsonl tests/fixtures/xbox_sample.jsonl \
  --max-events 20 \
  --json
```

`teleop-xbox` 默认打开 `tkinter` GUI。
其他命令可显式加 `--gui`，把单次 `CommandResponse` 显示到同一套调试窗口里。
左侧面板显示遥控器输入字段，包括最近事件、摇杆、扳机和按钮状态。
右侧面板显示 Python 控制输出字段，包括 deadman、末端 jog、`roll/pitch/yaw`、夹爪增量、响应状态、EEF 位姿、joint 状态和错误详情。
底部会显示按键与通道说明。
输入读取是事件驱动，不限频。
控制发送默认 `100 Hz`，UI 刷新默认 `50 Hz`。
显式加 `--json` 时进入无界面机器输出模式，便于测试和脚本验收。

Xbox 手柄默认映射：

| 输入                    | 行为                                                        |
| ----------------------- | ----------------------------------------------------------- |
| `RB / BTN_TR`         | deadman，按住才发送末端 jog                                 |
| 左摇杆上下              | 末端 `x` 小步长 jog                                       |
| 左摇杆左右              | 末端 `y` 小步长 jog                                       |
| 右摇杆上下              | 末端 `z` 小步长 jog                                       |
| 右摇杆左右              | 末端 yaw 小步长 jog                                         |
| 方向键左右              | 末端 `roll` 小步长 jog                                    |
| 方向键上下              | 末端 `pitch` 小步长 jog                                   |
| 左右扳机                | 夹爪开合小步长                                              |
| `X / BTN_X`           | 进入 SDK damping                                            |
| `A / BTN_A`           | 请求 `low_gain_passive`，需要维护权限                     |
| `B / BTN_B`           | 请求 `compliance_slow`，需要维护权限                      |
| `Y / BTN_Y`           | 请求 `reset_home`，需要维护权限                           |
| `SELECT / BTN_SELECT` | 请求 `gravity_compensation_startup`，仅用于启动前配置检查 |

实机调试前先确认手柄事件设备：

```bash
ls -l /dev/input/by-id/
cat /proc/bus/input/devices
```

如果插拔手柄后再次出现 `Permission denied`，通常是因为 `/dev/input/event*` 是内核重新创建的设备节点，之前对旧 `event10` 做过的临时 ACL 不会跟着新节点迁移。
先重新定位当前事件设备：

```bash
ls -l /dev/input/by-id/
grep -B4 -A8 -i "gamepad\|xbox\|zikway\|gamesir" /proc/bus/input/devices
```

临时修复当前这一次插入的设备：

```bash
sudo setfacl -m u:$USER:rw /dev/input/event10
```

把 `/dev/input/event10` 换成当前实际设备。
这个命令只对当前节点有效，拔插或重启后需要重做。

长期方案一：把当前用户加入 `input` 组。
这适合本地 bringup 主机，但该组可以读取所有输入设备事件，包括键盘和鼠标。

```bash
sudo usermod -aG input "$USER"
newgrp input
id -nG | tr ' ' '\n' | grep '^input$'
```

如果 `newgrp input` 后仍然打不开设备，退出当前登录会话后重新登录。

长期方案二：为这只手柄写 udev 规则。
你当前 `evtest` 看到的设备是 `Zikway HID gamepad`，`vendor=3537`、`product=1041`，可先用这条规则：

```bash
cat <<'EOF' | sudo tee /etc/udev/rules.d/99-armctrl-gamepad.rules
SUBSYSTEM=="input", KERNEL=="event*", ATTRS{idVendor}=="3537", ATTRS{idProduct}=="1041", TAG+="uaccess", GROUP="input", MODE="0660"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔手柄后验证：

```bash
ls -l /dev/input/by-id/
getfacl /dev/input/event10
uv run arx5ctl teleop-xbox --adapter fake --device /dev/input/event10 --gui
```

同样把 `event10` 换成重新定位到的实际设备。

实机只读检查：

```bash
uv run arx5ctl health --adapter sdk --model X5 --interface can0 --json
uv run arx5ctl state --adapter sdk --model X5 --interface can0 --json
```

实机 Xbox 低速 jog 需要显式执行确认：

```bash
uv run arx5ctl teleop-xbox \
  --adapter sdk \
  --model X5 \
  --interface can0 \
  --device /dev/input/by-id/<xbox-event-device> \
  --rate-hz 100 \
  --ui-hz 50 \
  --execute \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM"
```

如果要从手柄触发 `low_gain_passive`、`compliance_slow` 或 `reset_home`，还必须显式开启维护权限：

```bash
uv run arx5ctl teleop-xbox \
  --adapter sdk \
  --model X5 \
  --interface can0 \
  --device /dev/input/by-id/<xbox-event-device> \
  --rate-hz 100 \
  --ui-hz 50 \
  --execute \
  --maintenance \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM"
```

`gravity_compensation_startup` 不能在 controller 已创建后运行时切换。
它只应通过启动配置或 plan-only 检查处理。

## 参数辨识链路

当前仓库已经增加参数辨识辅助模块，职责是：

- 生成三层安全激励轨迹；
- 通过关节空间后端执行轨迹并采集 `q / dq / tau`；
- 写出 `planned_trajectory.csv`、`raw_samples.csv`、`manifest.json`；
- 生成 `processed_samples.csv` 和面向 `URDFly`、`Pinocchio`、`FIGAROH`、`FloBaRoID` 的交接说明。

三类激励轨迹：

- `gravity_sweep`：静态/准静态单关节扫描，先看重力项和末端 payload。
- `friction_sweep`：单关节正反向慢速/中速扫描，先估计摩擦。
- `fourier_multisine`：多关节有限傅里叶轨迹，默认带五次包络，起止速度和加速度回零。
- `fourier_multisine --optimize`：从多个傅里叶候选里选择代理观测矩阵条件数最低的一条。

先只做轨迹预览：

```bash
uv run arx5ctl ident-plan \
  --adapter fake \
  --profile gravity_sweep \
  --dof 6 \
  --sample-hz 100 \
  --output runs/ident-preview \
  --json
```

预览第三层优化后的傅里叶轨迹：

```bash
uv run arx5ctl ident-plan \
  --adapter fake \
  --profile fourier_multisine \
  --dof 6 \
  --sample-hz 100 \
  --duration 12 \
  --harmonics 5 \
  --optimize \
  --candidate-count 24 \
  --output runs/ident-fourier-preview \
  --json
```

`--optimize` 当前使用代理特征矩阵条件数，不直接代表真实动力学回归矩阵条件数。
它用于先筛掉明显差的傅里叶候选。
最终论文级轨迹优化仍应接入 `Pinocchio computeJointTorqueRegressor` 或 `URDFly` 生成的真实回归矩阵。

用 fake 后端走完整数据链路：

```bash
uv run arx5ctl ident-run \
  --adapter fake \
  --profile friction_sweep \
  --dof 6 \
  --sample-hz 100 \
  --output runs/ident-fake \
  --json
```

真实 ARX5 用 SDK 关节控制器采集：

```bash
uv run arx5ctl ident-run \
  --adapter sdk \
  --model X5 \
  --interface can0 \
  --profile gravity_sweep \
  --sample-hz 100 \
  --output runs/ident-sdk \
  --execute \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" \
  --json
```

对采集结果做后处理并生成外部工具交接文件：

```bash
uv run arx5ctl ident-postprocess \
  --dataset runs/ident-fake \
  --tool pinocchio \
  --tool figaroh \
  --tool flobaroid \
  --tool urdfly \
  --json
```

输出目录约定：

- `planned_trajectory.csv`：准备执行的关节轨迹。
- `raw_samples.csv`：原始采样数据。
- `manifest.json`：数据集元信息、列名、URDF 路径和工具提示。
- `processed/processed_samples.csv`：平滑和差分后的离线分析数据。
- `processed/tool_handoff.md`：离线辨识工具交接说明。

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
