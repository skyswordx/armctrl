# X5 SysID Hardware Smoke Plan

This artifact is a non-executing plan. It does not open CAN, import the SDK, or move hardware.

- movement_allowed: false
- hardware_execution_eligible: false
- hardware_smoke_plan_allowed: true
- gate_state: `hardware_smoke_plan_ready`

## 1. Gravity Smoke

Purpose: verify joint sign, SDK communication, damping landing, logs, and current baseline with low-speed small/mid amplitude motion.

Command draft:

```bash
uv run armctrl sysid run gravity_sweep --adapter sdk --model X5 --interface can0 --dof 6 --sample-hz 100 --duration 8 --amplitude 0.05 --q-center 0 0.30 0.30 0 0 0 --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --output runs/ident-sdk-gravity-smoke --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" --json
```

Fallback: Ctrl-C/fault must land in damping; stop if current spikes, table contact, or sign mismatch appears.

## 2. Friction Smoke

Purpose: verify positive/negative velocity data quality and current envelope before dynamic excitation.

Command draft:

```bash
uv run armctrl sysid run friction_sweep --adapter sdk --model X5 --interface can0 --dof 6 --sample-hz 100 --duration 10 --amplitude 0.08 --q-center 0 0.30 0.30 0 0 0 --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --output runs/ident-sdk-friction-smoke --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" --json
```

Fallback: stop before Fourier if current, velocity tracking, or endpoint clearance is abnormal.

## 3. Fourier Reduced Smoke

Purpose: run a reduced-risk version of the reviewed Fourier candidate only after gravity and friction smoke pass.

Command draft:

```bash
uv run armctrl sysid plan fourier_multisine --candidate-trajectory <reduced_execution_trajectory.csv> --dof 6 --sample-hz 100 --duration 1 --amplitude 0.25 --q-center 0 0.30 0.30 0 0 0 --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --output runs/ident-sdk-fourier-reduced-plan --json
```

Fallback: keep full Fourier blocked if reduced smoke shows over-current, contact, or unexpected posture.

## 4. Fourier Full Collection

Purpose: collect the reviewed low-condition OED candidate only after the first three gates pass and are recorded.

Command draft:

```bash
uv run armctrl sysid run fourier_multisine --adapter sdk --model X5 --interface can0 --dof 6 --sample-hz 100 --duration 1 --amplitude 0.50 --q-center 0 0.30 0.30 0 0 0 --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --output runs/ident-sdk-fourier-full --confirm "I UNDERSTAND THIS WILL MOVE THE ARM" --json
```

Observe: joint2 current, over-current logs, table/base clearance, end-effector dip, tracking error, and damping/fault landing.
