# SysID / OED Handoff: Lab Fourier Runtime Run

This folder contains artifacts copied from the n100d real-arm validation run:

- Remote project: `~/Roboclaw/references/projects/armctrl-clean-runtime-d371aaf`
- Remote run dir: `runs/lab-fourier-sysid-20260608-154946`
- Local archive: `D:/repo/Roboclaw/references/projects/armctrl-clean/runs/remote_lab_artifacts/lab-fourier-sysid-20260608-154946-artifacts.tgz`
- Local extracted data: `D:/repo/Roboclaw/references/projects/armctrl-clean/runs/remote_lab_artifacts/lab-fourier-sysid-20260608-154946`

## High-Level Meaning

The lab run used the live `ArmRuntime` path, not the old direct SDK SysID path:

1. Runtime opened SDK/CAN once.
2. Runtime recovered/held the arm.
3. SysID commands were queued into the live runtime.
4. Runtime acquired the single motion owner lease.
5. Runtime streamed `q_cmd` at 100 Hz and recorded `q_meas`, `dq_meas`, and `tau_meas`.

For identification quality analysis, do not assume the robot exactly followed `q_cmd`. Use the measured samples in `derived_sysid_csv`.

## Configuration Semantics

### `ident-fourier-reduced-0p25-slow4x`

Purpose: gentle real-arm smoke of the reduced Fourier trajectory.

Meaning:

- Uses the reduced 0.25-rad candidate shape.
- Time-stretched to about 8.8 s.
- Same overall reduced path family, much lower velocity and acceleration.
- Best for checking runtime stability, hold/release behavior, CAN/SDK timing, and gross mechanical safety.
- Not the main OED-quality trajectory; excitation is weaker/slower than the full candidate.

Observed execution:

- Samples: `881`
- Actual send rate: about `100.003 Hz`
- P99 send jitter: about `2.04 ms`
- Max tracking error: about `0.024 rad`
- RMS tracking error: about `0.0074 rad`

Primary measured-data CSV:

`derived_sysid_csv/ident-fourier-reduced-0p25-slow4x_b9973ba1_samples.csv`

### `ident-fourier-reduced-0p25-normal`

Purpose: normal-duration reduced-amplitude smoke.

Meaning:

- Uses the reduced 0.25-rad candidate shape.
- Runs at about 1.95 s / 100 Hz.
- Useful as a medium-excitation validation set.
- It is smoother than full in tracking quality, but still visually has nonuniform speed because the trajectory is an excitation path, not a constant-speed path.

Observed execution:

- Samples: `196`
- Actual send rate: about `100.009 Hz`
- P99 send jitter: about `0.125 ms`
- Max tracking error: about `0.028 rad`
- RMS tracking error: about `0.0086 rad`

Primary measured-data CSV:

`derived_sysid_csv/ident-fourier-reduced-0p25-normal_72c1056b_samples.csv`

### `ident-fourier-full`

Purpose: execute the offline OED-selected full Fourier trajectory on the real arm.

Meaning:

- This is the important candidate from the offline OED/simulation review.
- The lab full execution trajectory is byte-for-byte identical to the previously reviewed offline execution trajectory:

`D:/repo/Roboclaw/references/projects/armctrl-clean/runs/x5-fourier-best-candidate-metric-contract-safe-freeze-20260606/offline_review_cli_mesh/oed-structural-metric-contract-20260606/attempt-001/execution_trajectory.csv`

- The lab full execution file is:

`runs/lab-fourier-sysid-20260608-154946/ident-fourier-full/execution_trajectory.csv`

- SHA256 for both files:

`24ac41641084a56b65fd7b50d4962f825754b9a7231baaaae0ef0699b99ebc86`

Important safety-parameter note:

- The CLI was changed from `--amplitude 1.0` to `--amplitude 0.8` only to satisfy `fourier_multisine.max_sysid_amplitude_rad = 0.8` in `configs/x5.safe.yaml`.
- In the `--candidate-trajectory` path, `amplitude_rad` is a safety/metadata parameter. It does not multiply or shrink the CSV candidate.
- Therefore the full lab `q_cmd` trajectory was not reduced by 20 percent.

Observed execution:

- Samples: `196`
- Actual send rate: about `100.008 Hz`
- P99 send jitter: about `2.66 ms`
- Max tracking error: about `0.072 rad`
- RMS tracking error: about `0.0191 rad`

Primary measured-data CSV:

`derived_sysid_csv/ident-fourier-full_385f291a_samples.csv`

There is also an earlier full command result:

`derived_sysid_csv/4e6e1eaa-02ba-4dd9-a5d3-1c1221501372_4e6e1eaa_samples.csv`

It has similar tracking quality and can be used as a repeatability reference, but `ident-fourier-full_385f291a_samples.csv` is the labeled primary full run.

## Why Full Looks Nonuniform On The Real Arm

The full OED trajectory is not a constant-speed trajectory. It is a dynamic excitation trajectory. Its joint-space speed norm varies strongly:

- Speed norm min: about `0.025 rad/s`
- Speed norm max: about `3.78 rad/s`
- Speed norm mean: about `1.82 rad/s`
- Max joint acceleration: roughly `[19.05, 13.22, 12.42, 19.05, 17.81, 18.0] rad/s^2`

The HTML preview can make the motion look smoother or more uniform than real hardware. On the real arm, finite tracking bandwidth, gravity load, gear backlash/compliance, and torque limits make high-acceleration segments more visible.

The runtime did not resample or distort full:

- `resampling_policy`: `none_sample_hz_matches_send_hz`
- `interpolation_policy`: `pre_sampled_joint_positions`
- `trajectory_sample_hz`: `100 Hz`
- `runtime_send_hz`: `100 Hz`

## Data Columns For Analysis

The derived CSV files contain one row per runtime sample:

- `sent_monotonic_s`: monotonic send timestamp.
- `q_cmd_1..6`: commanded joint positions sent by runtime.
- `dq_cmd_1..6`: commanded joint velocities when available/derived from the trajectory.
- `q_meas_1..6`: SDK-measured joint positions.
- `dq_meas_1..6`: SDK-measured joint velocities.
- `tau_meas_1..6`: SDK-measured joint torque/effort signal.
- `q_error_1..6`: `q_meas - q_cmd`.
- `fault_flags`: SDK/runtime fault flags.

For rigid-body parameter identification, prefer measured signals:

- Use `q_meas` as the realized joint trajectory.
- Use `dq_meas` if it is stable enough; otherwise smooth/filter `q_meas` and derive `dq/ddq`.
- Estimate `ddq` from filtered measured trajectories, not raw finite differences without smoothing.
- Use `tau_meas` as the target torque/effort signal, after checking offset/noise/filtering assumptions.

Do not use `q_cmd` alone as the realized trajectory for final identification.

## Suggested Analysis Checklist

1. Verify execution integrity.

- `sample_count` equals expected count.
- `actual_send_hz` is near 100 Hz.
- `send_jitter_ms_p99` is acceptable.
- `fault_flags` are empty.
- `landing_mode` is `hold`.

2. Quantify tracking quality.

- Compute max, MAE, and RMS of `q_meas - q_cmd`.
- Report per-joint metrics.
- Flag full as tracking-warning if max error remains around `0.07 rad`.

3. Recompute excitation quality from measured data.

- Build the regressor using `q_meas`, filtered `dq_meas`, and estimated `ddq_meas`.
- Recompute rank and condition number.
- Compare measured-trajectory condition number against the offline command-trajectory condition number.

4. Fit and validate dynamics.

- Fit inertial/base parameters using measured data.
- Report torque prediction residuals: RMS, max, and per-joint residuals.
- Validate on a separate run if possible, for example reduced normal versus full.

5. Decide if the full run is usable.

- Usable for preliminary real-hardware identification if measured-regressor rank/condition and torque residuals are acceptable.
- Not sufficient if the measured regressor becomes ill-conditioned, if acceleration estimation is noisy, or if torque residuals are dominated by tracking/transient effects.

## External Practice Context

Mature robot-control stacks do not assume the robot exactly follows the command trajectory. ROS joint trajectory controllers track desired versus actual state and enforce path/goal tolerances. MoveIt Servo also explicitly recommends command smoothing because insufficiently smooth inputs can increase actuator wear and may prevent some robots from moving.

Relevant references:

- ROS 2 `joint_trajectory_controller`: https://docs.ros.org/en/jazzy/p/joint_trajectory_controller/doc/userdoc.html
- ROS 2 `FollowJointTrajectory` tolerance semantics: https://docs.ros.org/en/ros2_packages/rolling/api/control_msgs/action/FollowJointTrajectory.html
- MoveIt Servo smoothing: https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html

Practical implication for this dataset:

- Reduced and slow runs are good runtime and safety evidence.
- Full is the OED-relevant excitation, but its measured tracking error must be included in the identification-quality report.
- The final SysID/OED verdict should be based on measured trajectory rank/condition and torque prediction residuals, not on visual smoothness or command trajectory condition number alone.

