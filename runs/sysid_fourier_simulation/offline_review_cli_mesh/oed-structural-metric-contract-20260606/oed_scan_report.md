# X5 SysID OED Scan Report

- Profile: `fourier_multisine`
- Attempts: `1`
- URDF: `configs/models/X5_camera.urdf`
- Safe config: `configs/x5.safe.yaml`

## Best Attempt

- Attempt: `attempt-001`
- Safety allowed: `True`
- Condition metric: `figaroh_base_regressor`
- Condition number: `68.43362806845255`
- FIGAROH base condition: `68.43362806845255`
- Pinocchio effective condition: `77.56547530743994`
- Optimizer status: `fail`
- Optimizer reason: `max_iterations_exceeded`
- Rank: `36`

## Best Diagnostic Attempt

- Attempt: `attempt-001`
- Failure kind: `optimizer_dual_infeasible`

## Attempts

| attempt | status | safety | OED gate | optimizer | metric | condition | max step | max vel | max accel | rank | failure |
| --- | --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| attempt-001 | ok | True | fail | fail | figaroh_base_regressor | 68.43362806845255 | 0.01847600000000002 | 1.8427499999999999 | 17.812499999999975 | 36 | optimizer_dual_infeasible |
