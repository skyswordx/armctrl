# ARX5/X5 真机 SysID 与 Agent 操作手册

本文面向 n100d 目标机和 X5 真机验证。所有命令默认在目标机执行：

```bash
cd ~/Roboclaw/references/projects/armctrl-clean-n100d
```

## 0. 当前能力边界

`armctrl` clean rebuild 当前已经具备：

- fake SysID 数据链路；
- SysID planned trajectory 生成；
- URDF joint limit 检查；
- URDF frame-level table clearance 检查；
- SDK handshake/preflight；
- 最小 `arx5_interface` joint-control SDK runner 接线；
- SDK runner 不自动 `reset_to_home`；计划阶段会检查相邻采样点关节步长，确认后从当前姿态按 `max_joint_step_rad` 限步过渡到计划轨迹第一帧；
- Ctrl-C/fault 通过 `finally` 尝试落到 damping；
- 后处理与 solver handoff；
- Agent recipe CLI 安全门。

仍需人工确认：

- USB-CAN 硬件链路是否稳定；
- 机械臂底座是否固定；
- 末端下方是否无桌面/障碍物；
- 真机 Ctrl-C 后是否确实进入 damping；
- 运动幅度是否适合当前摆放。

## 1. USB 转 CAN 启动

先看 USB 设备：

```bash
ls /dev/ttyACM*
```

如果只有一个 USB-CAN，通常是 `/dev/ttyACM0`。设置 CAN：

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe slcan

sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
ip link show can0
```

如果已有旧 can0：

```bash
sudo ip link set can0 down || true
sudo pkill slcand || true
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
ip link show can0
```

`-s8` 通常表示 1Mbit/s。若官方设备要求其他速率，以官方说明为准。

## 2. 环境与只读握手

同步代码和依赖：

```bash
git switch codex/armctrl-clean-rebuild
git pull
uv sync --extra dev --extra lerobot
```

只读检查，不打开 CAN，不移动硬件：

```bash
uv run armctrl release status --json

uv run armctrl sysid sdk-preflight \
  --model X5 \
  --interface can0 \
  --json

uv run armctrl sysid sdk-handshake-plan \
  --model X5 \
  --interface can0 \
  --json

uv run armctrl sim doctor --json
```

预期：

- `sdk-preflight.status == ok`
- `sdk.status == available`
- `movement_allowed == false`
- `requires_confirm == "I UNDERSTAND THIS WILL MOVE THE ARM"`
- `sim doctor` 只读检查 `pinocchio_coal`、`mujoco`、`moveit`、`figaroh` 的可用性，不打开 CAN，不实例化 SDK，不移动硬件。

## 3. 安全中心位与安全配置

默认安全中心位：

```bash
SAFE_CENTER="0 0.30 0.30 0 0 0"
```

含义是 6 个关节角，单位 rad。当前经验上 `joint2=0.30`、`joint3=0.30` 能让躯干接近水平伸直且略抬，避免末端一开始下探到桌面。

安全配置文件：

```bash
configs/x5.safe.yaml
```

关键字段：

```yaml
safety:
  workspace_min_m: [0.05, -0.45, 0.02]
  workspace_max_m: [0.75, 0.45, 0.65]
  allowed_workspace_boxes:
    - name: main_body_sweep_volume
      min_m: [-0.35, -0.45, 0.02]
      max_m: [0.75, 0.45, 0.65]
  forbidden_workspace_boxes:
    - name: table_surface
      min_m: [-1.0, -1.0, -0.20]
      max_m: [1.0, 1.0, 0.02]
    - name: base_keepout
      min_m: [-0.08, -0.08, -0.05]
      max_m: [0.08, 0.08, 0.15]
  simulation:
    backend_preference: [pinocchio_coal, mujoco, moveit, urdf_fk_fallback]
    link_frames: [link5, link6, eef_link]
    min_clearance_m: 0.02
  max_sysid_duration_s: 60.0
  max_sysid_sample_hz: 100.0
  max_sysid_amplitude_rad: 0.25
  max_joint_step_rad: 0.01
  settle_before_record_s: 0.5
```

`max_joint_step_rad` 有两层作用：

- `sysid plan` 会检查 planned trajectory 中相邻采样点的最大关节步长，过大则 `trajectory_step_check=fail`；
- `sysid run --adapter sdk` 会用同一个阈值从当前姿态限步过渡到轨迹第一帧，过渡阶段不记录数据。

修改原则：

- 桌面更高时，提高 `workspace_min_m[2]`；
- 末端可能碰桌时，降低 `max_sysid_amplitude_rad`；
- 机器抖动或电流偏高时，降低 `max_sysid_amplitude_rad`，或提高 `sample-hz` / 降低 `max_joint_step_rad` 让每一拍更小；
- 第一帧过渡太快时，降低 `max_joint_step_rad`；
- 首次真机 smoke 建议 `amplitude <= 0.05`、`duration <= 8`。

## 4. 先生成 plan，不动机械臂

```bash
uv run armctrl sysid plan gravity_sweep \
  --dof 6 \
  --sample-hz 100 \
  --duration 8 \
  --amplitude 0.05 \
  --q-center $SAFE_CENTER \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output runs/plan-gravity-smoke \
  --json
```

检查：

```bash
cat runs/plan-gravity-smoke/manifest.json
cat runs/plan-gravity-smoke/trajectory_preview.json
```

必须满足：

- `safety.allowed == true`
- `urdf_limit_check.status == pass`
- `sysid_parameter_check.status == pass`
- `trajectory_step_check.status == pass`
- `workspace_clearance_check.status == pass`
- `simulation_check.status == pass`
- `trajectory_preview.json.backend.selected` 记录实际使用的成熟后端，或在后端缺失时明确标成 `urdf_fk_fallback`

如果 fail，不要运行真机。先减小 `--amplitude` 或调整 `--q-center` / `configs/x5.safe.yaml`。

## 5. 真机 gravity smoke

确认：

- 底座已固定；
- 末端下方无桌面；
- 手在实体急停/断电附近；
- 已经知道 Ctrl-C 后应该落 damping；
- 首轮只跑小幅 smoke。

```bash
uv run armctrl sysid run gravity_sweep \
  --adapter sdk \
  --model X5 \
  --interface can0 \
  --dof 6 \
  --sample-hz 100 \
  --duration 8 \
  --amplitude 0.05 \
  --q-center $SAFE_CENTER \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output runs/ident-sdk-gravity-smoke \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" \
  --json
```

停止条件：

- joint2 电流异常；
- 末端下探接近桌面；
- 轨迹明显朝错误方向；
- 机械臂抖动强烈；
- Ctrl-C 后未进入 damping。

任一出现都停止，不加幅度。

## 6. 后处理与 solver

```bash
uv run armctrl sysid postprocess \
  --dataset runs/ident-sdk-gravity-smoke \
  --solve \
  --json
```

查看产物：

```bash
find runs/ident-sdk-gravity-smoke \
  -name "quality_report.md" \
  -o -name "solver_report_zh.md" \
  -o -name "solver_metrics.json"
```

smoke 阶段主要看：

- 是否有 `raw_samples.csv`；
- 后处理是否完成；
- solver 是否能进入固定阶段；
- `data_health` 是否 pass。

小幅 smoke 不要求辨识结果好，它只验证安全运动和数据链路。

## 7. 逐步加幅度

只有 smoke 稳定，才逐步尝试：

```bash
uv run armctrl sysid run gravity_sweep \
  --adapter sdk \
  --model X5 \
  --interface can0 \
  --dof 6 \
  --sample-hz 100 \
  --duration 12 \
  --amplitude 0.10 \
  --q-center $SAFE_CENTER \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output runs/ident-sdk-gravity-a010 \
  --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" \
  --json
```

再后处理：

```bash
uv run armctrl sysid postprocess \
  --dataset runs/ident-sdk-gravity-a010 \
  --solve \
  --json
```

不建议直接超过 `0.25 rad`。当前 `x5.safe.yaml` 默认也会拦截更大的幅度。

## 8. friction 与 Fourier

friction 首次：

```bash
uv run armctrl sysid plan friction_sweep \
  --dof 6 --sample-hz 100 --duration 12 --amplitude 0.05 \
  --q-center $SAFE_CENTER \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output runs/plan-friction-smoke \
  --json
```

plan pass 后再 run。Fourier 风险最高，必须最后，并且从 `0.05 rad` 开始。当前 clean rebuild 的轨迹生成仍偏 smoke/handoff，不是成熟 FIGAROH 优化轨迹；要做高质量完整动力学辨识，应先接 FIGAROH 最优轨迹。

## 9. 模拟 Agent 调用

Agent 只能通过受限 CLI：

```bash
uv run armctrl recipe list --json
uv run armctrl recipe plan home --json
uv run armctrl recipe status --json
uv run armctrl recipe execute home --json
uv run armctrl recipe cancel --json
```

当前 `recipe execute` 若无 verified backend 应返回 `rejected`。合格 Agent 行为：

1. 解析 JSON；
2. 看到 `status=rejected` 后停止；
3. 汇报 `safety` / `executor` / `reason`；
4. 不直接调用 `arx5_interface`；
5. 不自造 joint command。

SysID Agent 调用也必须先 plan：

```bash
uv run armctrl sysid plan gravity_sweep \
  --dof 6 --sample-hz 100 --duration 8 --amplitude 0.05 \
  --q-center $SAFE_CENTER \
  --urdf-path configs/models/X5_camera.urdf \
  --safe-config configs/x5.safe.yaml \
  --output runs/agent-plan-gravity-smoke \
  --json
```

只有 plan pass，且人类明确给出确认 token，Agent 才能建议运行 SDK runner。

## 10. 常见故障解释

`unrecognized arguments: --confirm`

- 代码不是最新；
- 先 `git pull`。

`arx5_interface is not importable`

- 不是 Linux 目标环境，或未安装 SDK；
- 在 n100d 执行 `uv sync --extra dev --extra lerobot`。

`planned trajectory did not pass safety checks`

- plan 已经被安全门挡住；
- 查看 `runs/.../manifest.json`；
- 减小幅度或调整安全中心位。

`over current detected`

- 可能撞桌、近限位、姿态力矩过大或底层控制器保护；
- 立即停止；
- 降低幅度，抬高安全空间 z 下界，重新 plan。

## 11. 完整 smoke 验证清单

```bash
ls /dev/ttyACM*
ip link show can0
uv run armctrl sysid sdk-preflight --model X5 --interface can0 --json
uv run armctrl sysid sdk-handshake-plan --model X5 --interface can0 --json
uv run armctrl sim doctor --json
uv run armctrl sysid plan gravity_sweep --dof 6 --sample-hz 100 --duration 8 --amplitude 0.05 --q-center $SAFE_CENTER --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --output runs/plan-gravity-smoke --json
uv run armctrl sysid run gravity_sweep --adapter sdk --model X5 --interface can0 --dof 6 --sample-hz 100 --duration 8 --amplitude 0.05 --q-center $SAFE_CENTER --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --output runs/ident-sdk-gravity-smoke --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" --json
uv run armctrl sysid postprocess --dataset runs/ident-sdk-gravity-smoke --solve --json
```
