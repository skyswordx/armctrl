# Changelog

本文记录 `armctrl` 的用户可见变更。

## [Unreleased]

### Added

- Added profile-aware SysID OED safety settings: Fourier multisine now gets a wider offline OED amplitude envelope while gravity/friction can keep conservative hardware bring-up constraints.
- Added FIGAROH base-regressor diagnostics to the external OED handoff path so `manifest.json` and OED scan summaries can distinguish FIGAROH base-regressor condition from Pinocchio full-regressor effective condition.
- Added IPOPT print-level propagation and iteration-log tail extraction for SysID OED scans, making `obj/inf_pr/inf_du/alpha` evidence available when `print_level >= 5`.
- Added first-class OED scan evidence artifacts: `oed_scan_attempts.json` flattens per-attempt safety/OED/optimizer fields for external review, and `oed_scan_report.md` gives a compact Markdown table.
- Added `--attempt-timeout-s` and `--ipopt-print-level` to `scripts/x5_oed_scan.py`, so long FIGAROH/IPOPT attempts can be bounded without wrapping the mature backend command in a shell-specific `timeout`.
- Added `representative_ipopt_stdout.txt` for OED scans when a diagnostic attempt exposes IPOPT stdout, plus last-iteration `objective/inf_pr/inf_du` extraction from IPOPT iteration tails for timeout cases without a final solver summary.
- Added SysID frequency layering artifacts: `planned_trajectory.csv` remains the low-rate FIGAROH/OED plan, while `execution_trajectory.csv` is resampled at the configured high-rate rollout frequency for final safety gates.
- Added configurable X5 profile OED velocity and acceleration limits for Fourier, friction, and gravity profiles, avoiding URDF placeholder velocities such as `1000 rad/s` while keeping the values scan-tunable.

### Changed

- Relaxed the X5 joint2/joint3 hard relation constraint for `fourier_multisine`; the narrow `q2-q3` band remains available for conservative gravity/friction probes but no longer locks the full-body OED search space.
- Added profile-specific SysID execution step gates: global Agent/recipe motion remains at `0.01 rad/sample`, while Fourier/friction/gravity SysID plans can use `0.02/0.018/0.015 rad/sample` respectively at the 100 Hz execution layer.
- Added execution-trajectory velocity and acceleration gates for SysID plans, using profile OED limits after resampling rather than trusting the low-rate planned trajectory alone.
- Changed execution trajectory resampling from linear interpolation to SciPy cubic splines when enough OED waypoints are available, reducing artificial acceleration spikes before safety gates run.
- Relaxed only the Fourier/OED acceleration safety envelope to `30 rad/s^2` per joint after WSL scan evidence showed low-condition candidates around `25.6 rad/s^2`; gravity and friction remain on their conservative profile limits.
- Updated the OED quality gate to prefer FIGAROH base-regressor condition when the mature backend reports it, with Pinocchio full-regressor condition retained as a fallback diagnostic.
- WSL evidence from `runs/oed-scan-profile-relaxed-smoke-20260605-v4` shows the relaxed Fourier request removes `joint_relation_constraints`, raises the Fourier profile amplitude ceiling to `0.8 rad`, and captures `oed_scan_summary.json`, `oed_scan_attempts.json`, `oed_scan_report.md`, and `representative_ipopt_stdout.txt`. The bounded `amplitude=0.3, n_wps=5, stack_reps=1, sample_hz=20` smoke still timed out after 120s without a candidate, but the captured IPOPT iteration log moved objective from `1.049e5` to `2.263e4`, kept `inf_pr=0`, and reduced `inf_du` from `1.13e2` to `1.04e1`; the current bottleneck is runtime/problem size, not the earlier `q2-q3` hard-lock restoration failure.
- OED scan reproduction should now use armctrl's built-in per-attempt timeout plumbing; `trajectory_command` should point directly at the FIGAROH wrapper.
- Removed the incorrect `max_joint_step_rad * planning_sample_hz` OED velocity derivation. `max_joint_step_rad` is now an execution-trajectory safety gate, not a FIGAROH velocity limit.
- WSL evidence from `runs/oed-scan-frequency-layering-smoke-20260606` shows the same `duration=1, amplitude=0.3, n_wps=5, stack_reps=1, sample_hz=20` smoke now keeps `effective_duration_s=1.0`, `effective_sample_count=21`, `execution_sample_count=101`, and reduces IPOPT inequality constraints from `5796` to `276`. The attempt returned `status=ok` with rank `36` and Pinocchio full-regressor condition about `167.43`, but OED quality still fails because optimizer convergence is not yet clean and the external safety gate is not passing.
- WSL evidence from `runs/oed-scan-fourier-accel30-focused-200-20260606` found the current best Fourier candidate at `duration=1.0`, `amplitude=0.5`, `n_wps=5`, `stack_reps=1`, `seed=3`, and `ipopt_max_iterations=200`: safety gates all pass, rank is `36`, base-regressor condition is about `106.28`, execution max step is about `0.0196 rad`, max velocity about `1.95 rad/s`, and max acceleration about `25.51 rad/s^2`. The remaining OED quality gap is optimizer convergence labeling (`optimizer_dual_infeasible`), not execution safety.

### Reproduce

```bash
cd /home/circlemoon/armctrl-clean-oed
/home/circlemoon/.local/bin/uv run pytest \
  tests/test_sysid_oed_scan.py \
  tests/test_sysid_trajectory_backend.py \
  tests/test_x5_figaroh_oed.py \
  tests/test_simulation_safety.py -q
/home/circlemoon/.local/bin/uv run python scripts/x5_oed_scan.py \
  --output runs/oed-scan-profile-relaxed-smoke-20260605-v4 \
  --duration 1 \
  --amplitude 0.3 \
  --n-wps 5 \
  --stack-reps 1 \
  --seed 1 \
  --sample-hz 20 \
  --condition-number-threshold 1000 \
  --ipopt-max-iterations 30 \
  --ipopt-print-level 5 \
  --attempt-timeout-s 120 \
  --trajectory-command /home/circlemoon/.local/bin/uv run python scripts/x5_figaroh_oed.py
```

- Added `armctrl sysid plan --candidate-trajectory` so FIGAROH/Pinocchio or other mature OED backends can provide the trajectory while `armctrl` only imports it, runs safety gates, scores the regressor when available, and records the source in `manifest.json`.
- Added `armctrl sysid plan --trajectory-command ...` so an external FIGAROH/OED wrapper can consume `ARMCTRL_FIGAROH_REQUEST`, write `ARMCTRL_CANDIDATE_TRAJECTORY`, and then hand the generated CSV back to the same safety-gated planning path.
- Added `scripts/x5_figaroh_oed.py` and `armctrl.x5_figaroh_oed` as the X5 FIGAROH OED wrapper contract: it reads the armctrl request, attempts to construct a FIGAROH optimal trajectory run, converts successful `T_F/P_F` results into candidate CSV, and fails explicitly instead of producing fake OED output when the mature backend path is incomplete.
- Added structured external OED command diagnostics: successful wrapper stdout JSON is recorded in `manifest.json`, and failed wrapper runs return `status=faulted` with command argv, exit code, stdout/stderr, and parsed stdout JSON instead of a Python traceback.
- Added `armctrl eef doctor` for non-hardware discovery of MoveIt Servo and Pink backend availability.
- Added plan-only `armctrl eef plan-twist` and `armctrl eef plan-pose` contracts so Agents can express bounded end-effector intent without bypassing safety gates.
- Added `backend_request.json` handoff artifacts for EEF plans so mature backends can own Twist/Pose execution semantics outside `armctrl`.
- Added `armctrl eef review` so mature backend joint trajectories can be checked through the shared simulation preview chain before any future hardware execution path.
- Added `sdk_cartesian` as an explicit EEF backend contract for ARX5 SDK cartesian control, plus LeRobot-aligned cartesian action metadata in EEF plan artifacts.
- Added `backend_review_contract.json` plus a conventional `backend_joint_trajectory.csv` review path so EEF backend outputs can be auto-discovered by `armctrl eef review`.
- Added `armctrl eef runtime-plan` so Agents can query the mature runtime owner for `sdk_cartesian`, `moveit_servo`, or `lerobot_rollout` without `armctrl` pretending to be the executor.
- Added `eef runtime-plan --plan-dir` inference so Agent flows can carry backend/runtime/review context forward directly from EEF plan artifacts.
- Added `armctrl eef export-lerobot-action --plan-dir <dir>` so an EEF plan can be converted into a LeRobot-friendly cartesian action contract with ordered features and values for Agent/runtime reuse.
- Added `armctrl eef export-sdk-cartesian --plan-dir <dir>` so an EEF plan can be converted into a programmatic ARX5 SDK cartesian bridge artifact without making armctrl own controller execution.
- Added `armctrl eef export-moveit-servo --plan-dir <dir>` so a twist-style EEF plan can be converted into a MoveIt Servo bridge artifact using the standard ROS `TwistStamped` surface.
- Added `armctrl eef stage-trajectory --plan-dir <dir> --trajectory <joint_csv>` so mature backend output can be validated, copied into the conventional review path, and then passed through the existing shared simulation gate.
- Added backend-specific `bridge_artifact_preview` and `next_steps` hints on `armctrl eef runtime-plan --plan-dir <dir>` for `sdk_cartesian` and `moveit_servo`, so Agents can consume one compact mature-backend handoff checklist.
- Added explicit `--backend` override support on `armctrl eef runtime-plan --plan-dir <dir>`, including LeRobot preview support, so one bounded EEF plan can be handed to a different mature runtime owner without regenerating artifacts.
- Added `armctrl eef export-runtime-bridge --plan-dir <dir>` as a unified, backend-agnostic bridge-export surface that delegates to the existing SDK, MoveIt Servo, or LeRobot exporters.
- Added `armctrl eef export-runner-contract --plan-dir <dir>` as a non-executing handoff contract for backend helpers that consume a mature-owner bridge artifact and emit a reviewed joint trajectory CSV back into the shared simulation gate.
- Added `armctrl eef export-agent-runtime-contract --plan-dir <dir>` as a non-hardware handoff contract for Agent/helper loops that stream stable EEF action frames into SDK, MoveIt Servo, or LeRobot mature runtime owners while keeping the shared review return path explicit.
- Added `scripts/lerobot_agent_runtime_helper_sample.py` as a non-hardware sample that consumes `armctrl.eef_agent_runtime_contract.v1` for `lerobot_rollout` and turns it into a processor-oriented helper plan around `robot_action_processor` / `robot_observation_processor`.
- Added `scripts/lerobot_processor_contract_helper_sample.py` as a non-hardware sample that consumes `armctrl.lerobot_eef_processor_contract.v1` and immediately closes the loop through `preview-rollout` plus the shared simulation review gate.
- Added `armctrl recipe export-agent-preset-contract --plan-dir <dir>` as a non-hardware Agent-facing preset-action handoff artifact that consolidates recipe safety summary, EEF seed, required artifacts, and next-step guidance into one JSON contract.
- Added explicit `ordered_steps` sequencing metadata to the Agent-facing recipe preset and EEF runtime contracts so dependent commands no longer need to be inferred from free-text `next_steps`.
- Added explicit `ordered_steps` sequencing metadata to LeRobot processor contracts and both LeRobot helper samples so Agent callers can keep export, preview, staging, and review steps serialized.
- Added CLI-native `armctrl lerobot agent-runtime-helper-plan` and `armctrl lerobot processor-helper-preview` so Agent callers can stay on the main JSON CLI surface instead of depending on repo-local sample scripts as their primary interface.
- Added CLI-native `armctrl eef export-sdk-helper-plan` and `armctrl eef export-moveit-helper-plan` so SDK Cartesian and MoveIt Servo backend handoff planning can also stay on the main JSON CLI surface.
- Added optional `--eef-plan-dir` on `armctrl lerobot config-plan rollout` so rollout planning can carry the exported EEF LeRobot action bridge and keep policy/runtime action semantics aligned.
- Added `armctrl lerobot stage-rollout-trajectory --eef-plan-dir <dir> --trajectory <joint_csv>` as a LeRobot-facing wrapper over the shared EEF trajectory staging contract.
- Added runtime-owner and rollout-specific `next_steps` guidance to `armctrl lerobot config-plan rollout`, so it mirrors the compact handoff ergonomics of `eef runtime-plan`.
- Added a `processor_bridge` contract to `armctrl lerobot config-plan rollout`, pointing programmatic integrations at LeRobot's processor-based rollout adaptation surface while reusing the exported EEF action vocabulary.
- Added `armctrl lerobot review-rollout` so LeRobot-facing rollout validation can reuse the existing EEF simulation review chain instead of introducing a separate safety path.
- Added the project-local `armctrl-agent-motion` Codex skill so Agents can follow the recipe + EEF + review workflow without bypassing safety contracts.
- Added simulation-gated recipe artifact coverage for `recipe plan --output` and `recipe execute --backend sim`.

- 新增 `armctrl sim doctor`，只读检测 `pinocchio_coal`、`mujoco`、`moveit`、`figaroh` 成熟后端可用性，不打开 CAN，不实例化 SDK，不移动硬件。
- 新增安全空间 DSL：`configs/x5.safe.yaml` 现在支持 `allowed_workspace_boxes`、`forbidden_workspace_boxes` 和 `simulation.backend_preference`，用于统一约束 Agent recipe、SysID 和后续 LeRobot rollout safety bridge。
- 新增 SysID 轨迹仿真安全预览：`sysid plan --output` 现在写出 `trajectory_preview.json`，并把 `simulation_check` 纳入 `manifest.json` 与 stdout safety gate。
- 新增 `armctrl sim preview`，可对已生成的 `planned_trajectory.csv` 做无硬件安全预览；成熟后端缺失时明确标记 `urdf_fk_fallback`，不会伪装成完整碰撞仿真。
- 将 release 状态推进为 `0.6.0-rc.3`，标记 safety-space config、simulation doctor 和 SysID trajectory preview gate 已完成本地非硬件验证。
- 新增项目侧 X5 STL mesh 资产，`configs/models/X5_camera.urdf` 可被 Pinocchio/coal 直接加载几何模型，不再依赖 SDK wheel 内部相对路径。
- 新增 `sim` optional extra：Linux 目标主机可通过 `uv sync --extra dev --extra sim` 安装 MuJoCo 预览依赖。
- 新增 ROS 2 MoveIt/Jazzy doctor 检测：未 source ROS 环境但 `/opt/ros/jazzy` 存在时，报告 `installed_not_sourced` 和 `source /opt/ros/jazzy/setup.bash` 提示。
- 新增 X5 显式 allowed collision pairs：仅忽略 n100d Pinocchio/coal 实测出的相邻装配 mesh 重叠对，其他碰撞仍保持 hard gate。
- 新增 MuJoCo trajectory rollout：`sim preview --backend mujoco` 现在按 `planned_trajectory.csv` 逐帧设置 `qpos`、调用 `mj_forward` 并报告 contact/qpos 指标，不再只是证明 URDF 可加载。
- 新增 CLI SVG 可视化：`sim preview --render path.svg` 与 `sysid plan --render` 会渲染轨迹预览；危险轨迹也会出图并带 `WARNING` 与 gate 原因。
- 新增 CLI HTML 动画可视化：`sim preview --render path.html` 会生成可交互 URDF-FK 轨迹动画；被 gate 拒绝的危险轨迹也会渲染并标出 `WARNING` 与原因。

- 新增 `armctrl sysid run --adapter sdk` 的最小 `arx5_interface` 真机 smoke runner：通过显式 `--confirm` 后才会构造 SDK joint controller，并在采集完成、故障或 Ctrl-C 路径中尝试落到 damping。
- 新增 `docs/hardware_sysid_operator_manual.md`，收束 n100d 上机步骤：USB-CAN、SDK handshake、SysID plan、SDK smoke run、postprocess/solve、Agent recipe 模拟调用和安全配置调参。
- 新增 SysID 上机安全参数 gate：`configs/x5.safe.yaml` 现在限制最大 duration、sample rate、amplitude、planned trajectory 相邻采样关节步长、首帧过渡关节步长和记录前 settle 合同。
- 新增 SDK runner 首帧限步过渡：从当前关节角逐步移动到 planned trajectory 第一帧，过渡阶段不写入采样数据。
- 新增 `armctrl lerobot doctor`，只读检查 LeRobot、ARX5 LeRobot 插件和 `arx5_interface` 导入状态，不打开 CAN、不连接硬件。
- 新增 `armctrl lerobot config-plan record/train/rollout`，把采集、训练、推理收敛为原生 LeRobot CLI 命令计划，`armctrl` 只输出 JSON 合同且不执行。
- 新增 `armctrl lerobot export-metadata`，写出 LeRobot 数据集、SysID 参数包和安全配置之间的 metadata bridge。
- 新增 `lerobot` optional extra，目标 Linux 主机可通过 `uv sync --extra dev --extra lerobot` 安装 LeRobot ARX5 插件并进行无硬件导入验证。
- 将 release 状态推进为 `0.6.0-rc.2`，标注 SDK smoke runner 已完成非硬件验证，真实硬件动作仍等待 n100d 上机验证。

### Fixed

- 修复 `armctrl sysid run --adapter sdk` 的确认参数契约：CLI 现在接收 `--confirm`，并在确认后进入 SDK runner 或结构化安全拒绝，而不是由 argparse 报 unknown argument。

## [0.6.0-rc.1] - LeRobot Planning Bridge

### Status

- `armctrl` 现在能为 LeRobot record/train/rollout 生成可审查命令计划，但不重写 LeRobot Robot/Teleoperator，也不在该层直接执行硬件动作。
- `armctrl lerobot doctor` 是只读环境检查；目标 Linux 主机上的插件安装/导入验证和真实 record/rollout 仍列为 deferred validation。
- `armctrl lerobot export-metadata` 提供数据集到 SysID 参数包、安全配置的桥接文件，用于后续训练、推理与参数版本追溯。

### Added

- 新增 `armctrl lerobot doctor`、`config-plan` 和 `export-metadata` 三个 CLI 合同。
- 新增 `lerobot` optional extra，包含 `lerobot`、`lerobot-robot-arx5` 和 `lerobot-teleoperator-arx5`。
- 新增 LeRobot CLI 合同测试，覆盖 doctor、record/train/rollout 计划和 metadata bridge。

### Changed

- `ROADMAP.md` 的 v0.6.0 从抽象 LeRobot 集成改为可执行的分阶段 CLI 合同：先 doctor/config-plan/metadata，再目标 Linux 非硬件验证，最后真实硬件 record/rollout。
- `release status` / `release notes` 升级到 `0.6.0-rc.1`，并把 LeRobot 真实硬件 record/rollout 标为 deferred。

## [0.5.0] - Contract Complete, Hardware Pending

### Added

- 新建 clean rebuild 分支入口，只保留 roadmap、changelog、文档索引、vendor 和 uv 配置作为重构基线。
- 明确 `armctrl` 与 ARX5 SDK、Pinocchio、FIGAROH、LeRobot ARX5 集成的职责边界。
- 新增第一版 plan-only Agent recipe catalog 和 JSON CLI 预览入口。
- 新增 recipe plan 的机器可读 dry-run、movement_allowed 和 risk_explanation 字段。
- 新增结构化 recipe execute 拒绝语义，防止在没有后端和安全合同前误触硬件执行。
- 新增 plan-only SysID planner，输出 gravity/friction/Fourier profile 与 Pinocchio/FIGAROH/LeRobot handoff 合同。
- 新增 `sysid plan --output` 离线产物生成，写出 `planned_trajectory.csv` 和 `manifest.json`。
- 新增轻量 URDF joint limit 检查，`sysid plan --output` 会在 stdout 和 manifest 中记录 pass/fail。
- 新增 workspace/table clearance 的第一版保守 proxy 检查，读取 `configs/x5.safe.yaml` 并在 stdout/manifest 中记录 pass/fail。
- 新增 URDF frame-level FK/table clearance 检查，替换旧 joint2 proxy，并在 manifest 中记录 `urdf_fk_frame_clearance`。
- 新增 `sysid run --adapter fake`，可生成 `raw_samples.csv` 和 run manifest，用于恢复无硬件数据链路。
- 新增可注入 SDK SysID runner 骨架，测试覆盖 hold/damping 后开始记录以及完成/故障落 damping；公共 CLI 仍默认拒绝真硬件。
- 新增 `sysid postprocess`，从 fake/raw dataset 生成 `processed_samples.csv`、`quality_metrics.json` 和 `quality_report.md`。
- 新增 `sysid solve`，从 processed dataset 生成 `solver_metrics.json` 和 `solver_report_zh.md`，并记录 Pinocchio/FIGAROH 可用性。
- 新增 `sysid postprocess --solve`，让后处理完成后可以直接运行固定 solver 阶段并返回 solver 产物。
- 新增可选 Pinocchio regressor rank/condition 指标；无 Pinocchio 环境时保持结构化降级。
- 新增可选 Pinocchio least-squares prediction RMSE 指标，用于验证 `Y*pi -> tau_pred -> residual` 链路。
- 新增 solver 固定输出物理一致性和 FIGAROH 基础参数 gate 字段；未导入外部证据前为 `not_evaluated`。
- 新增 `sysid package` 质量门；缺少 Pinocchio、FIGAROH/base-parameter 或物理一致性证据时拒绝生成候选参数包。
- 新增候选参数包版本、SHA-256 签名、回滚目标和上线前 A/B 验证记录。
- 新增 `sysid import-evidence`，把 FIGAROH/外部物理一致性和基础参数证据导入 solver metrics，供参数包质量门消费。
- 新增 `sysid figaroh-handoff`，为外部 FIGAROH 运行生成 processed 数据、URDF 和后续导入命令清单。
- 新增 `sysid adapt-figaroh-evidence`，把 FIGAROH-style report 规范化为 `armctrl.external_solver_evidence.v1`。
- 新增 `online-id audit`，以 append-only JSONL 记录 shadow 在线辨识更新窗口、残差变化、饱和检查、回滚目标和人工确认要求。
- 新增 `recipe status` / `recipe cancel` 和显式 recipe command executor gate，Agent skill 只能通过受限 recipe CLI 触达动作。
- 新增 `release status`，机器可读标注 `0.5.0` contracts complete / hardware pending 状态。
- 新增 release status 的本地验证命令和 deferred validation 分类，区分硬件、外部工具和几何升级缺口。
- 新增 `release notes`，从同一状态面生成 v0.5.0 clean rebuild 发布说明。
- 新增 `sysid sdk-preflight`，只读检查 `arx5_interface` 导入状态和目标 model/interface，不打开 CAN、不移动硬件。
- 新增 `sysid sdk-handshake-plan`，只读固定未来 SDK 采集前的确认、hold/damping、记录时序和 Ctrl-C/fault 落态契约。

### Status

- Clean rebuild contracts through v0.5.0 are implemented and tested.
- Hardware SDK runner, full FK/table collision, and n100d Pinocchio/FIGAROH validation remain pending.
- 新增在线辨识 shadow-mode policy，限制在线更新只覆盖 torque bias、摩擦项和小幅 gravity residual。
- 新增项目内 Codex skill: `.codex/skills/armctrl-agent-recipes/SKILL.md`，限制 Agent 只能通过 recipe CLI 查看和规划动作。

### Changed

- 停止在旧实验实现上继续叠加功能，后续按 `v0.2.0` 到 `v0.5.0` milestone 重新实现。
- SysID execute 仍保持 rejected；workspace clearance 已升级为 URDF frame-level FK/table clearance，但仍不是完整 mesh/body collision model。
- `sysid run --adapter sdk` 仍保持 rejected，等待真实 SDK runner、安全落态和真机验证。
- 当前 postprocess 覆盖基础文件合同和 data health；预测误差、物理一致性等进入固定 solver stage 后继续补齐。

## [0.1.0] - Historical Baseline

### Added

- 历史 `develop`/`feature/subsystem` 分支包含初始 SDK adapter、安全检查、teleop、SysID 轨迹生成、采集、后处理和 solver 报告实验。

### Known Gaps

- 历史实现边界不清，容易重复 LeRobot、FIGAROH 和 ARX5 SDK 的成熟能力。
- 历史文档较多，当前工作以 `ROADMAP.md` 为主线。
