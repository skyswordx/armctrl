# ARX5 Bringup Teleop Notes

## Scope

This note explains three things for the current `feature/hardware-bringup` branch:

1. why the existing `check_arx5_min_motion.py` script moves the arm in a small and controlled way,
2. why the current branch uses the Python SDK control path instead of wiring MoveIt into bringup immediately,
3. how to use the new joint GUI and cartesian keyboard teleop tools.

## Compatibility Conclusion

The current bringup environment uses `arx5-interface==0.1.2`.
The official materials under `docs/arx5_official_materials/` and the latest upstream `real-stanford/arx5-sdk` examples are compatible with this wheel at the API level.

The upstream SDK examples are also tracked in this repo as a git submodule:

- `vendor/real_stanford_arx5_sdk`

Initialize or refresh it with:

```bash
git submodule update --init --recursive
```

The main matching points are:

- `RobotConfigFactory.get_instance().get_config(...)`
- `ControllerConfigFactory.get_instance().get_config(...)`
- `Arx5JointController(...)`
- `Arx5CartesianController(...)`
- `JointState(...).pos()`
- `EEFState().pose_6d()`
- `controller.set_joint_cmd(...)`
- `controller.set_eef_cmd(...)`
- `controller.reset_to_home()`

The official wheel does not install the example scripts into `site-packages`.
It only installs the controller library, URDF models, and the typed Python interface.
That is why this repo vendors the bringup-facing tools directly under `scripts/bringup/` instead of trying to import `keyboard_teleop.py` from the wheel.

## Why Not MoveIt In This Branch

The ROS2 PDFs describe a different stack:

- a separate `arx_x5_controller` workspace,
- `rqt` publishing to that workspace's topics,
- launch files such as `open_single_arm.launch.py` and `v2_single_arm.launch.py`.

Those launch files and controller nodes are not present in this repo.
The only control path already verified on real hardware in this branch is the Python SDK path.

Because of that, this branch keeps the control loop close to the verified path:

- joint commands go through `Arx5JointController`,
- end-effector commands go through `Arx5CartesianController`,
- cartesian teleop adds an `Arx5Solver.multi_trial_ik(...)` precheck before sending the next target.

This keeps the number of moving parts small while bringup is still focused on first hardware validation.

## How `check_arx5_min_motion.py` Works

The script does not jump to a hard-coded pose.
It reads the measured joint state first, copies that state into a writable `JointState` command object, and only adds a tiny delta on one selected joint.

The control flow is:

1. build a joint controller with background send/recv enabled,
2. read the current joint state from the real arm,
3. copy the current state into `cmd`,
4. change only `target_pos[joint_index] += delta`,
5. linearly interpolate from current pose to target pose over `steps`,
6. call `controller.set_joint_cmd(cmd)` on every interpolation step,
7. optionally interpolate back to the original pose.

The arm moves because `set_joint_cmd(...)` updates the target joint vector seen by the SDK controller.
With background send/recv enabled, the SDK communication thread keeps pushing the latest commanded state to the hardware.

The script still keeps three safety gates:

- default mode is `plan_only`,
- motion only happens with `--execute`,
- motion still asks for the confirmation phrase unless `--yes` is passed.

## New Tools

### `scripts/bringup/arx5_joint_slider_gui.py`

Purpose:

- simple GUI for direct joint control during bringup,
- sliders are initialized from the measured joint state,
- the GUI can sync from current state, send the current target once, reset home, or switch to damping.

Notes:

- requires `--execute` to arm motion,
- uses `tkinter`,
- `--reset-home-on-startup` matches the official examples and keeps the GUI target on SDK home joints plus fully open gripper while the home reset finishes,
- on a headless machine, use the keyboard teleop instead.

Run:

```bash
cd ~/work/armctrl
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
python scripts/bringup/arx5_joint_slider_gui.py --model X5 --interface can0 --execute
```

### `scripts/bringup/arx5_keyboard_cartesian_teleop.py`

Purpose:

- terminal cartesian teleop for end-effector position and orientation,
- adapted from the official `keyboard_teleop.py` control idea,
- starts from the measured pose instead of resetting home on startup,
- checks IK before accepting each new pose target.
- `--reset-home-on-startup` matches the official examples and keeps the target on SDK home pose plus fully open gripper while the home reset finishes.

Key map:

- arrow up / down: `x+ / x-`
- arrow left / right: `y+ / y-`
- page up / page down: `z+ / z-`
- `q / a`: `roll+ / roll-`
- `w / s`: `pitch+ / pitch-`
- `e / d`: `yaw+ / yaw-`
- `r / f`: open / close gripper
- `Space`: `reset_to_home()` and keep the target on SDK home pose plus fully open gripper until you command something else
- `c`: sync target pose from current measured state
- `x` or `Esc`: quit

Run:

```bash
cd ~/work/armctrl
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
python scripts/bringup/arx5_keyboard_cartesian_teleop.py --model X5 --interface can0 --execute
```

## Verification Checklist

Before any motion test:

- `check_arx5_state.py` can read joint and end-effector state cleanly,
- the arm is in a clear workspace,
- emergency stop and power-off path are confirmed,
- only one control terminal is actively commanding the arm.

Recommended no-motion checks:

```bash
python -m py_compile scripts/bringup/*.py
python scripts/bringup/check_arx5_import.py --json
python scripts/bringup/check_arx5_import.py --json --check-tk
python scripts/bringup/check_arx5_import.py --json --check-curses
python scripts/bringup/check_arx5_min_motion.py --model X5 --interface can0 --json
python scripts/bringup/arx5_joint_slider_gui.py --model X5 --interface can0 --json
python scripts/bringup/arx5_keyboard_cartesian_teleop.py --model X5 --interface can0 --json
```
