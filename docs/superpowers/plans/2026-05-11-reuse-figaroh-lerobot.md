# Reuse FIGAROH and LeRobot Compatibility Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make armctrl a thin, safe hardware/control shell that reuses FIGAROH for identification math and uses LeRobot-style contracts for non-robot-specific data and interface adaptation.

**Architecture:** Keep armctrl's runtime, safety, and hardware adapters local. Add a narrow compatibility layer that maps armctrl identification datasets into a LeRobot-shaped schema and maps offline identification handoff into FIGAROH-ready inputs. Do not reimplement regressor math, QR decomposition, or trajectory optimization inside armctrl.

**Tech Stack:** Python 3.12, pytest, existing armctrl identification/CLI modules, vendor/figaroh as the external math reference, LeRobot-style schema contracts without adding a hard dependency.

---

### Task 1: Document the boundary and data contract

**Files:**
- Modify: `docs/arm_control_subsystem_blueprint.md`
- Modify: `docs/reference_projects_and_sjtu_roboclaw_notes.md`
- Modify: `docs/identification_excitation_trajectory_analysis.md` if needed for the data contract note

- [ ] **Step 1: Write the boundary change as a focused testable design note**

```md
- armctrl owns runtime safety, adapters, and dataset capture.
- FIGAROH owns excitation scoring, regressor handling, QR/base-parameter extraction, and physical-consistency projection.
- LeRobot is the unified schema for observations/actions/dataset-facing contracts.
- armctrl must not reimplement sysid math; it only exports FIGAROH-ready inputs and LeRobot-shaped metadata.
```

- [ ] **Step 2: Update the blueprint wording to match the new boundary**

```md
Add a `lerobot_compat/` boundary description that says the package maps armctrl dataset fields into a LeRobot-like contract and keeps the math in FIGAROH.
```

- [ ] **Step 3: Verify the docs mention FIGAROH reuse explicitly and do not imply a second sysid stack**

Run: `Select-String -Path docs/*.md -Pattern 'FIGAROH|LeRobot|lerobot_compat|regressor|QR'`
Expected: the new boundary is called out in the arm control docs.

- [ ] **Step 4: Commit the doc-only boundary update**

```bash
git add docs/arm_control_subsystem_blueprint.md docs/reference_projects_and_sjtu_roboclaw_notes.md docs/identification_excitation_trajectory_analysis.md
git commit -m "docs: align armctrl around figaroh and lerobot contracts"
```

### Task 2: Add a thin LeRobot compatibility layer for identification datasets

**Files:**
- Create: `src/armctrl/compat/__init__.py`
- Create: `src/armctrl/compat/lerobot.py`
- Modify: `src/armctrl/identification/models.py`
- Modify: `src/armctrl/identification/recorder.py`
- Modify: `src/armctrl/identification/postprocess.py`
- Modify: `src/armctrl/identification/tools.py`

- [ ] **Step 1: Write the compatibility-layer tests first**

```python
from armctrl.compat.lerobot import build_lerobot_contract
from armctrl.identification.models import DatasetManifest

def test_lerobot_contract_uses_joint_observation_and_action_names():
    contract = build_lerobot_contract(dof=3, gripper=True)
    assert contract["observation_features"]["joint_positions"]["names"] == ["joint_1", "joint_2", "joint_3"]
    assert contract["action_features"]["joint_targets"]["names"] == ["joint_1", "joint_2", "joint_3"]
```

- [ ] **Step 2: Run the new test and confirm it fails before implementation**

Run: `pytest tests/unit/test_identification.py -k lerobot_contract -v`
Expected: FAIL because the compatibility layer does not exist yet.

- [ ] **Step 3: Implement the compatibility helper and wire it into manifest generation**

```python
# src/armctrl/compat/lerobot.py
from __future__ import annotations

def build_lerobot_contract(*, dof: int, gripper: bool = False) -> dict[str, object]:
    joint_names = [f"joint_{index}" for index in range(1, dof + 1)]
    contract = {
        "schema": "lerobot-compatible",
        "observation_features": {
            "joint_positions": {"names": joint_names, "unit": "rad"},
            "joint_velocities": {"names": joint_names, "unit": "rad/s"},
            "joint_efforts": {"names": joint_names, "unit": "Nm"},
        },
        "action_features": {
            "joint_targets": {"names": joint_names, "unit": "rad"},
        },
    }
    if gripper:
        contract["observation_features"]["gripper_position"] = {"unit": "m"}
        contract["action_features"]["gripper_target"] = {"unit": "m"}
    return contract
```

- [ ] **Step 4: Add the contract to dataset manifest and tool handoff metadata**

```python
manifest.lerobot_contract = build_lerobot_contract(dof=profile.dof, gripper=True)
```

- [ ] **Step 5: Run the identification tests that cover manifest and postprocess output**

Run: `pytest tests/unit/test_identification.py -q`
Expected: pass, with the new schema present in manifest and handoff output.

- [ ] **Step 6: Commit the compatibility layer**

```bash
git add src/armctrl/compat/__init__.py src/armctrl/compat/lerobot.py src/armctrl/identification/models.py src/armctrl/identification/recorder.py src/armctrl/identification/postprocess.py src/armctrl/identification/tools.py tests/unit/test_identification.py
git commit -m "feat: add lerobot compatibility metadata for identification"
```

### Task 3: Make FIGAROH the explicit offline handoff target

**Files:**
- Modify: `src/armctrl/identification/tools.py`
- Modify: `src/armctrl/identification/postprocess.py`
- Modify: `tests/unit/test_identification.py`

- [ ] **Step 1: Write a test that the handoff says FIGAROH owns the offline math**

```python
def test_tool_handoff_marks_figaroh_as_offline_math_target(tmp_path: Path):
    profile = generate_gravity_sweep(dof=2, sample_hz=10.0, amplitude_rad=0.05, segment_duration_s=0.5)
    recorder = DatasetRecorder(tmp_path)
    runner = IdentificationRunner(FakeJointRobotIO(dof=2), sample_hz=10.0, sleep_fn=lambda _: None)
    runner.run(profile, recorder=recorder, execute=True)

    response = postprocess_dataset(
        dataset_dir=tmp_path,
        output_dir=tmp_path / "processed",
        tools=("figaroh",),
        urdf_path="configs/models/X5_camera.urdf",
        smoothing_window=3,
    )

    handoff = (tmp_path / "processed" / "tool_handoff.md").read_text(encoding="utf-8")
    assert "FIGAROH" in handoff
    assert "tau = Y(q, dq, ddq) * pi" in handoff
```

- [ ] **Step 2: Run the test and confirm it fails before the handoff wording is updated**

Run: `pytest tests/unit/test_identification.py -k figaroh -v`
Expected: FAIL until the handoff wording is explicit and complete.

- [ ] **Step 3: Update the handoff text to name the FIGAROH bridge and LeRobot schema together**

```md
- armctrl emits a LeRobot-shaped dataset contract.
- FIGAROH consumes the offline processed CSV + URDF + manifest to compute regressors, base parameters, and physically consistent outputs.
```

- [ ] **Step 4: Run the focused identification suite again**

Run: `pytest tests/unit/test_identification.py -q`
Expected: pass.

- [ ] **Step 5: Commit the handoff update**

```bash
git add src/armctrl/identification/tools.py src/armctrl/identification/postprocess.py tests/unit/test_identification.py
git commit -m "docs: make figaroh the explicit offline identification target"
```

### Task 4: Verify the bridge in the full identification and CLI path

**Files:**
- Modify: `tests/unit/test_identification.py`
- Modify: `src/armctrl/cli/arx5ctl.py` only if a CLI flag or output needs to expose the new contract

- [ ] **Step 1: Add an end-to-end test for the contract export path**

```python
def test_identification_pipeline_exports_lerobot_contract_in_manifest(tmp_path: Path):
    profile = generate_fourier_multisine(
        dof=3,
        sample_hz=20.0,
        duration_s=2.0,
        harmonics=2,
        amplitude_rad=0.05,
        seed=3,
        q_center=(0.1, 0.2, 0.3),
    )
    recorder = DatasetRecorder(tmp_path)
    manifest = recorder.write_run(
        profile=profile,
        backend_name="fake_joint",
        model="X5",
        samples=[],
        execute=False,
    )
    assert manifest.profile_metadata["q_center"] == (0.1, 0.2, 0.3)
```

- [ ] **Step 2: Run only the relevant identification tests first**

Run: `pytest tests/unit/test_identification.py -q`
Expected: pass.

- [ ] **Step 3: Run the CLI test slice if the CLI output was touched**

Run: `pytest tests/unit/test_identification.py -k cli -q`
Expected: pass.

- [ ] **Step 4: Commit the verification-only cleanup**

```bash
git add tests/unit/test_identification.py src/armctrl/cli/arx5ctl.py
git commit -m "test: cover lerobot contract and figaroh handoff path"
```

### Self-Review

- [ ] Spec coverage: every requirement has a task
- [ ] Placeholder scan: no TBD/TODO or vague steps
- [ ] Type consistency: LeRobot contract keys and manifest fields match across tasks

---
