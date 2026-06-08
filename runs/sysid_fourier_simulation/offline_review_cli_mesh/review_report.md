# X5 SysID Fourier 离线硬件放行审查

- gate_state: `hardware_smoke_plan_ready`
- movement_allowed: `false`
- hardware_execution_eligible: `false`
- 结论: 允许进入 hardware smoke plan

## 条件数

- FIGAROH base condition: `68.43362806845255`
- Pinocchio effective condition: `77.56547530743994`
- rank: `36`
- primary metric: `figaroh_base_regressor`

## 优化器

- status: `fail`
- reason: `review_optimizer_convergence_offline`
- constraint_violation_unscaled: `0.0`

## 轨迹平滑性

- status: `pass`
- max_joint_step_rad: `0.01847600000000002`
- max_velocity_rad_s: `1.842749999999998`
- max_acceleration_rad_s2: `17.812500000000036`

## 空间/碰撞预览

- clearance status: `pass`
- min_clearance_m: `0.09210931930410017`
- execution gate: `pass`

## 审查材料

- q plot: `runs/x5-fourier-best-candidate-metric-contract-safe-freeze-20260606/offline_review_cli_mesh/joint_q.svg`
- dq plot: `runs/x5-fourier-best-candidate-metric-contract-safe-freeze-20260606/offline_review_cli_mesh/joint_dq.svg`
- ddq plot: `runs/x5-fourier-best-candidate-metric-contract-safe-freeze-20260606/offline_review_cli_mesh/joint_ddq.svg`
- preview html: `runs/x5-fourier-best-candidate-metric-contract-safe-freeze-20260606/offline_review_cli_mesh/trajectory_preview.html`
- hardware smoke plan: `runs/x5-fourier-best-candidate-metric-contract-safe-freeze-20260606/offline_review_cli_mesh/hardware_smoke_plan.md`
