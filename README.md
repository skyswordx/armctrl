# armctrl

`armctrl` 是 `Roboclaw` 机械臂控制子系统的独立 Python 包。
当前重点是 ARX5 / X5 机械臂的 bringup、手柄遥操作、夹爪标定、参数辨识采集。

## 项目结构

```text
armctrl/
├── configs/
│   ├── calibration/gripper/      # 项目侧夹爪标定文件
│   ├── models/                   # 项目侧 URDF，当前主用 X5_camera.urdf
│   └── x5.safe.yaml              # 预留安全配置
├── docs/                         # bringup、蓝图、设计文档
├── src/armctrl/
│   ├── protocol/                 # 统一数据模型、错误码、枚举
│   ├── safety/                   # 运动限位、debug profile 权限检查
│   ├── calibration/              # 项目侧标定存储与交互流程
│   ├── adapters/arx5/            # fake / sdk 适配器与 ARX5 gain profile
│   ├── daemon/                   # 命令执行器，负责状态机与安全入口
│   ├── teleop/                   # Xbox 输入映射、控制循环、GUI 调试面板
│   ├── identification/           # 激励轨迹、采集、后处理
│   └── cli/                      # arx5ctl 命令行入口
├── tests/                        # 单元测试
└── vendor/real_stanford_arx5_sdk # 参考 SDK 源码快照
```

## 运行前提

当前代码路径默认面向 Linux bringup 主机。
最少前提如下：

- Python `>= 3.12`
- `uv`
- `arx5-interface` 已安装到当前虚拟环境
- 真实机械臂场景需要：
  - `SocketCAN` 接口，例如 `can0`
  - 游戏手柄事件设备，例如 `/dev/input/event10`
  - 当前用户对对应 `event` 设备有读权限

当前仓库的 `pyproject.toml` 里声明的开发依赖只有：

- `pytest>=8.2`

## 环境准备

```bash
cd ~/Roboclaw/references/projects/armctrl

uv venv --python 3.12 .venv
source .venv/bin/activate

uv pip install -e ".[dev]"
uv pip install arx5-interface
```

如果你的环境里 `python3.12` 是系统 Python，也可以显式写成：

```bash
uv venv --python /usr/bin/python3.12 .venv
```

## CAN 启用

如果机械臂控制器通过串口转 `SocketCAN` 暴露到主机，先把 `can0` 拉起来：

```bash
sudo slcand -o -f -s8 /dev/ttyACM0 can0
sudo ip link set up can0
```

检查：

```bash
ip -details link show can0
```

## 手柄权限

先定位事件设备：

```bash
ls -l /dev/input/by-id/
cat /proc/bus/input/devices
```

临时给当前用户开权限：

```bash
sudo setfacl -m u:$USER:rw /dev/input/event10
```

把 `/dev/input/event10` 换成当前真实设备。

## 连接与基础检查

无硬件自检：

```bash
uv run pytest -q
uv run python -m compileall src tests
uv run arx5ctl health --adapter fake --json
```

真实硬件只读检查：

```bash
uv run arx5ctl health --adapter sdk --model X5 --interface can0 --json
uv run arx5ctl state --adapter sdk --model X5 --interface can0 --json
```

## 模型与 URDF

X5 模型的 URDF 优先级是：

1. CLI 显式传入 `--urdf-path`
2. 项目侧 `configs/models/X5_camera.urdf`
3. SDK 自带默认模型

当前项目默认假设：

- X5 带末端相机 payload
- 逆动力学要读取 `link6` 的惯量，而不是只看 `eef_link`

## 夹爪标定

项目不再支持每次启动时临时传 `--gripper-open-readout` / `--gripper-width`。
现在统一走项目侧标定文件：

- `configs/calibration/gripper/X5.json`

查看当前标定：

```bash
uv run arx5ctl gripper-calibration-show --model X5 --json
```

### 1. Bootstrap 标定

如果当前夹爪方向已经错到 controller 起不来，先写一份 bootstrap 值：

```bash
uv run arx5ctl gripper-calibration-set \
  --model X5 \
  --open-readout -3.4 \
  --width 0.082
```

### 2. SDK 交互式校准

```bash
uv run arx5ctl gripper-calibration-wizard \
  --model X5 \
  --interface can0
```

这个命令会复用 SDK 自己的 `calibrate_gripper()`，并在同一次流程里把结果保存到
`configs/calibration/gripper/X5.json`：

- 提示你把夹爪完全闭合并回车
- 在闭合位置设零点
- 提示你把夹爪完全张开并回车
- SDK 在终端打印 `Fully-open joint position readout: ...`
- 把这个数值输入 wizard 后续提示
- 如有需要，再输入真实开口宽度
- `armctrl` 立即持久化保存，后续 `health`、`teleop-xbox`、`identification` 默认自动读取

### 3. 当前推荐做法

如果 controller 因夹爪方向错误连不上，先写一份 bootstrap 值起机；
起机后优先再跑一次上面的 wizard。

只有在你已经从 SDK 日志拿到权威值、但这次不想重新跑 wizard 时，
才需要手工执行 `gripper-calibration-set`。

例如：

```bash
uv run arx5ctl gripper-calibration-set \
  --model X5 \
  --open-readout 4.97768 \
  --width 0.082 \
  --notes "from sdk calibrate stdout 2026-04-26"
```

清除标定：

```bash
uv run arx5ctl gripper-calibration-clear --model X5
```

## 启用机械臂与遥操作

### 只连通，不运动

```bash
uv run arx5ctl health \
  --adapter sdk \
  --model X5 \
  --interface can0 \
  --gravity-compensation \
  --json
```

### Xbox GUI 遥操作

```bash
uv run arx5ctl teleop-xbox \
  --adapter sdk \
  --model X5 \
  --interface can0 \
  --gravity-compensation \
  --device /dev/input/event10 \
  --gui \
  --execute \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM"
```

当前 teleop 接管语义：

- 从 `damping`、`zero_gravity_drag`、`idle` 等非 `teleop` 状态切回时，第一拍先同步当前实测末端/夹爪状态；
- 这一拍会先发送一条“保持当前位置”的 teleop 命令；
- 第二拍开始，摇杆增量才真正累加到新的 teleop 基线上。

如果需要从手柄触发维护型 profile，再加：

```bash
--maintenance
```

## 手柄控制语义

- `RB / BTN_TR`
  - 按住：进入 `teleop`
  - 松开：进入 `zero_gravity_drag`
- 左摇杆：末端 `x / y`
- 右摇杆：末端 `z / yaw`
- 方向键：末端 `roll / pitch`
- 左右扳机：夹爪开合
- `X / BTN_X`：进入 `damping`
- `A / BTN_A`：恢复 `teleop` 默认增益
- `Y / BTN_Y`：请求 `reset_home`，需要 `--maintenance`

## 模式说明

- `teleop`
  - 正常遥操作状态
  - 使用 SDK 默认控制增益
- `zero_gravity_drag`
  - 松开 `RB` 后进入
  - 先同步当前目标，再切到低增益拖动 profile
  - 这套 gain profile 已经从 `sdk.py` 流程代码中抽到 `src/armctrl/adapters/arx5/control_profiles.py`
- `damping`
  - 真正的安全阻尼态
  - `X` 键和进程退出都会落到这里

## 参数辨识链路

轨迹预览：

```bash
uv run arx5ctl ident-plan \
  --adapter fake \
  --profile gravity_sweep \
  --dof 6 \
  --sample-hz 100 \
  --json
```

真实采集：

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

后处理：

```bash
uv run arx5ctl ident-postprocess \
  --dataset runs/ident-sdk \
  --tool pinocchio \
  --tool figaroh \
  --tool flobaroid \
  --tool urdfly \
  --json
```

## 常用命令汇总

```bash
# fake
uv run arx5ctl health --adapter fake --json
uv run arx5ctl teleop-xbox --adapter fake --device /dev/input/event10 --gui

# sdk 只读
uv run arx5ctl health --adapter sdk --model X5 --interface can0 --json
uv run arx5ctl state --adapter sdk --model X5 --interface can0 --json

# sdk 遥操作
uv run arx5ctl teleop-xbox \
  --adapter sdk \
  --model X5 \
  --interface can0 \
  --gravity-compensation \
  --device /dev/input/event10 \
  --gui \
  --execute \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM"
```

## 验证

```bash
uv run pytest -q
uv run python -m compileall src tests
```

当前这次重构至少覆盖了：

- `SafetyGuard` 与 debug profile 元信息整合
- ARX5 gain profile 从 `sdk.py` 抽离
- `RB` 多次按下/松开时 `teleop <-> zero_gravity_drag` 状态切换回归测试
