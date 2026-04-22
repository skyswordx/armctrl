# Xbox Arm Debug Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a tested `armctrl` package where an Xbox controller can safely drive ARX5 end-effector jog commands through the same protocol, safety, adapter and CLI path used by OpenClaw.

**Architecture:** The package keeps SDK access behind `ArmAdapter`. `FakeArx5Adapter` supports no-hardware tests and command lifecycle checks; `Arx5SDKAdapter` delay-imports `arx5_interface` and only calls SDK controller methods. Xbox input maps to `TeleopCommand`; `ArmCommandExecutor` validates it with `SafetyGuard` before sending an EEF command to the adapter.

**Tech Stack:** Python 3.12, dataclasses, argparse CLI, stdlib Linux input parsing, pytest, optional ARX5 SDK runtime import.

---

### Task 1: Project Baseline

**Files:**
- Create: `pyproject.toml`
- Create: `src/armctrl/__init__.py`
- Create: `src/armctrl/protocol/enums.py`
- Create: `src/armctrl/protocol/errors.py`
- Create: `src/armctrl/protocol/models.py`
- Test: `tests/unit/test_protocol_models.py`

- [ ] **Step 1: Write protocol tests**

```python
from armctrl.protocol.enums import CommandStatus
from armctrl.protocol.models import CommandResponse, EEFStateModel, TeleopCommand


def test_teleop_command_rejects_wrong_vector_length():
    try:
        TeleopCommand(translation_m=(0.0, 0.0), rotation_rad=(0.0, 0.0, 0.0))
    except ValueError as exc:
        assert "translation_m" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_response_json_roundtrip():
    response = CommandResponse(status=CommandStatus.COMPLETED, message="ok")
    payload = response.to_json()
    assert '"completed"' in payload
    assert CommandResponse.from_json(payload).status is CommandStatus.COMPLETED


def test_eef_state_has_six_dof_pose():
    state = EEFStateModel(pose_6d=(0.3, 0.0, 0.2, 0.0, 0.0, 0.0))
    assert state.pose_6d[0] == 0.3
```

- [ ] **Step 2: Run protocol tests to verify failure**

Run: `uv run --with pytest pytest tests/unit/test_protocol_models.py -v`
Expected: FAIL because `armctrl` package does not exist.

- [ ] **Step 3: Implement dataclass protocol models**

Create enum types, structured error payloads, `EEFStateModel`, `JointStateModel`, `RobotState`, `MoveEEFRequest`, `TeleopCommand`, `DebugProfileRequest` and `CommandResponse`. Keep serialization in stdlib JSON and do not import `arx5_interface`.

- [ ] **Step 4: Run protocol tests**

Run: `uv run --with pytest pytest tests/unit/test_protocol_models.py -v`
Expected: PASS.

### Task 2: Adapter and Safety Core

**Files:**
- Create: `src/armctrl/adapters/base.py`
- Create: `src/armctrl/adapters/arx5/fake.py`
- Create: `src/armctrl/adapters/arx5/sdk.py`
- Create: `src/armctrl/safety/profiles.py`
- Create: `src/armctrl/safety/debug_profiles.py`
- Create: `src/armctrl/safety/guard.py`
- Test: `tests/unit/test_fake_adapter_and_safety.py`

- [ ] **Step 1: Write adapter and safety tests**

```python
from armctrl.adapters.arx5.fake import FakeArx5Adapter
from armctrl.protocol.enums import DebugProfileName
from armctrl.protocol.models import DebugProfileRequest, MoveEEFRequest, TeleopCommand
from armctrl.safety.debug_profiles import DebugProfileRegistry
from armctrl.safety.guard import SafetyGuard
from armctrl.safety.profiles import MotionLimits


def test_fake_adapter_plan_only_does_not_move():
    adapter = FakeArx5Adapter()
    before = adapter.get_state().eef.pose_6d
    response = adapter.move_eef(MoveEEFRequest(pose_6d=(0.31, 0.0, 0.2, 0.0, 0.0, 0.0), plan_only=True))
    assert response.status.value == "completed"
    assert adapter.get_state().eef.pose_6d == before


def test_safety_rejects_large_teleop_step():
    guard = SafetyGuard(MotionLimits(max_translation_step_m=0.01))
    command = TeleopCommand(translation_m=(0.2, 0.0, 0.0), rotation_rad=(0.0, 0.0, 0.0), deadman=True)
    result = guard.validate_teleop(command)
    assert not result.allowed


def test_debug_profile_requires_maintenance():
    registry = DebugProfileRegistry.default()
    guard = SafetyGuard(MotionLimits(), registry)
    request = DebugProfileRequest(name=DebugProfileName.LOW_GAIN_PASSIVE, maintenance=False, confirm=False)
    result = guard.validate_debug_profile(request)
    assert not result.allowed
```

- [ ] **Step 2: Run tests to verify failure**

Run: `uv run --with pytest pytest tests/unit/test_fake_adapter_and_safety.py -v`
Expected: FAIL because modules do not exist.

- [ ] **Step 3: Implement thin adapter contract and safety checks**

`Arx5SDKAdapter` delay-imports SDK inside `connect()`. It creates `RobotConfigFactory`, `ControllerConfigFactory` and `Arx5CartesianController`, then calls SDK methods such as `set_eef_cmd()`, `set_to_damping()` and `reset_to_home()`. `FakeArx5Adapter` only stores state for tests and does not implement kinematics.

- [ ] **Step 4: Run adapter and safety tests**

Run: `uv run --with pytest pytest tests/unit/test_fake_adapter_and_safety.py -v`
Expected: PASS.

### Task 3: Executor and CLI

**Files:**
- Create: `src/armctrl/daemon/executor.py`
- Create: `src/armctrl/cli/arx5ctl.py`
- Test: `tests/unit/test_executor_and_cli.py`

- [ ] **Step 1: Write executor and CLI tests**

```python
import json

from armctrl.adapters.arx5.fake import FakeArx5Adapter
from armctrl.daemon.executor import ArmCommandExecutor
from armctrl.protocol.models import TeleopCommand
from armctrl.cli.arx5ctl import main


def test_executor_teleop_moves_fake_adapter_when_deadman_active():
    adapter = FakeArx5Adapter()
    executor = ArmCommandExecutor(adapter)
    response = executor.handle_teleop(TeleopCommand(translation_m=(0.001, 0.0, 0.0), rotation_rad=(0.0, 0.0, 0.0), deadman=True))
    assert response.status.value == "completed"
    assert adapter.get_state().eef.pose_6d[0] > 0.3


def test_cli_health_json(capsys):
    code = main(["health", "--adapter", "fake", "--json"])
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["status"] == "completed"
```

- [ ] **Step 2: Run tests to verify failure**

Run: `uv run --with pytest pytest tests/unit/test_executor_and_cli.py -v`
Expected: FAIL because executor and CLI do not exist.

- [ ] **Step 3: Implement executor and CLI commands**

Add `health`, `state`, `damping`, `move-eef`, `debug-profile`, `list-debug-profiles` and `teleop-xbox`. Real SDK movement requires `--adapter sdk --execute --confirm "I UNDERSTAND THIS WILL MOVE THE ARM"`; default execution uses fake adapter or plan-only behavior.

- [ ] **Step 4: Run executor and CLI tests**

Run: `uv run --with pytest pytest tests/unit/test_executor_and_cli.py -v`
Expected: PASS.

### Task 4: Xbox Mapping and Debug Runner

**Files:**
- Create: `src/armctrl/teleop/filters.py`
- Create: `src/armctrl/teleop/mapping.py`
- Create: `src/armctrl/teleop/xbox.py`
- Test: `tests/unit/test_xbox_mapping.py`

- [ ] **Step 1: Write Xbox mapping tests**

```python
from armctrl.teleop.mapping import XboxMapper, XboxState


def test_deadman_required_for_motion():
    mapper = XboxMapper()
    state = XboxState(axes={"ABS_Y": -1.0}, buttons={"BTN_TR": False})
    command = mapper.to_command(state, now=1.0)
    assert not command.deadman
    assert command.translation_m == (0.0, 0.0, 0.0)


def test_right_bumper_enables_slow_x_jog():
    mapper = XboxMapper(max_translation_step_m=0.002)
    state = XboxState(axes={"ABS_Y": -1.0}, buttons={"BTN_TR": True})
    command = mapper.to_command(state, now=1.0)
    assert command.deadman
    assert command.translation_m[0] > 0.0


def test_x_button_requests_damping_profile():
    mapper = XboxMapper()
    state = XboxState(buttons={"BTN_X": True})
    command = mapper.to_command(state, now=1.0)
    assert command.debug_profile == "damping"
```

- [ ] **Step 2: Run tests to verify failure**

Run: `uv run --with pytest pytest tests/unit/test_xbox_mapping.py -v`
Expected: FAIL because mapping modules do not exist.

- [ ] **Step 3: Implement Xbox mapping and event parsing**

Map Linux Xbox events into normalized axes/buttons. Use right bumper `BTN_TR` as deadman. Map left stick to X/Y jog, right stick Y to Z jog, right stick X to yaw, triggers to gripper, X button to damping, Y button to reset-home request.

- [ ] **Step 4: Run Xbox tests**

Run: `uv run --with pytest pytest tests/unit/test_xbox_mapping.py -v`
Expected: PASS.

### Task 5: End-to-End Verification

**Files:**
- Create: `configs/x5.safe.yaml`
- Modify: `README.md`
- Modify: `docs/subsystem/arm_control_subsystem_blueprint.md`

- [ ] **Step 1: Add safe config and usage docs**

Document `arx5ctl teleop-xbox --adapter fake --event-jsonl tests/fixtures/xbox_sample.jsonl --json` and the real-hardware command requiring `--adapter sdk --execute --confirm`.

- [ ] **Step 2: Run full no-hardware verification**

Run: `uv run --with pytest pytest -v`
Expected: PASS.

Run: `uv run python -m compileall src tests`
Expected: PASS.

Run: `uv run arx5ctl health --adapter fake --json`
Expected: JSON with status `completed`.

Run: `uv run arx5ctl teleop-xbox --adapter fake --event-jsonl tests/fixtures/xbox_sample.jsonl --max-events 20 --json`
Expected: JSON lines showing accepted teleop commands and a final damping/cancel status.

- [ ] **Step 3: Confirm SDK isolation**

Run: `rg -n "import arx5_interface|from arx5_interface" src tests`
Expected: only `src/armctrl/adapters/arx5/sdk.py` may import the SDK, and it must do so inside a method.
