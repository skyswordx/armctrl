import json
import subprocess
import sys


def test_cli_release_status_reports_simulation_safety_rc_and_hardware_pending() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "release", "status", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    milestone_ids = [milestone["id"] for milestone in payload["milestones"]]

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.release_status.v1"
    assert payload["version"] == "0.6.0-rc.3"
    assert payload["branch"] == "codex/armctrl-clean-rebuild"
    assert (
        payload["release_readiness"]
        == "simulation_safety_preview_nonhardware_verified"
    )
    assert milestone_ids == [
        "v0.2.0",
        "v0.3.0",
        "v0.4.0",
        "v0.5.0",
        "v0.6.0-rc.3",
    ]
    assert payload["milestones"][-1]["status"] == "rc_nonhardware_verified_remote_backend_pending"
    assert "real_sdk_runner_hardware_validation" in payload["hardware_pending"]
    assert "sdk_gravity_smoke_on_target_linux" in payload["hardware_pending"]
    assert "native_lerobot_record_on_target_linux" in payload["hardware_pending"]
    assert "native_lerobot_rollout_on_target_linux" in payload["hardware_pending"]
    assert "n100d_pinocchio_figaroh_validation" in payload["hardware_pending"]
    assert "n100d_mujoco_moveit_pinocchio_coal_doctor" in payload["hardware_pending"]
    assert payload["verification"]["local_commands"] == [
        "uv sync --extra dev --extra sim",
        "uv run pytest -q",
        "uv run python -m compileall src tests",
        "uv run armctrl release status --json",
        "uv run armctrl sim doctor --json",
        "uv run armctrl sysid plan gravity_sweep --dof 6 --sample-hz 100 --duration 2 --amplitude 0.05 --q-center 0 0.30 0.30 0 0 0 --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --output runs/plan-preview-smoke --json",
        "uv sync --extra dev --extra lerobot",
        "uv run armctrl lerobot doctor --model X5 --robot-interface can0 --teleop-interface can1 --json",
        "uv run armctrl sysid run gravity_sweep --adapter sdk --duration 8 --amplitude 0.5 --q-center 0 0.30 0.30 0 0 0 --output runs/tmp-confirm-check --confirm 'I UNDERSTAND THIS WILL MOVE THE ARM' --json",
    ]
    assert payload["verification"]["test_count"] == 77
    assert payload["deferred_validation"]["requires_hardware"] == [
        "real_sdk_runner_hardware_validation",
        "native_lerobot_record_on_target_linux",
        "native_lerobot_rollout_on_target_linux",
        "sdk_preflight_on_target_linux",
        "sdk_handshake_plan_on_target_linux",
        "sdk_gravity_smoke_on_target_linux",
        "hold_damping_ctrl_c_hardware_landing",
        "hardware_ab_control_benefit_test",
    ]
    assert payload["deferred_validation"]["requires_external_tool"] == [
        "n100d_pinocchio_figaroh_validation",
        "n100d_mujoco_moveit_pinocchio_coal_doctor",
    ]


def test_cli_release_notes_reports_v060_rc_summary() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "release", "notes", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.release_notes.v1"
    assert payload["version"] == "0.6.0-rc.3"
    assert payload["title"] == "armctrl 0.6.0-rc.3 simulation safety preview"
    assert "simulation_safety_preview_nonhardware_verified" in payload["summary"]
    assert payload["sections"]["included"] == [
        "governance and safety boundary",
        "offline SysID loop contracts",
        "conservative online identification policy",
        "Agent recipe CLI skill",
        "LeRobot doctor, native command plans, and dataset metadata bridge",
        "SDK gravity smoke runner with confirmation, safety gates, and damping landing path",
        "Safety-space config plus simulation doctor and SysID trajectory preview gate",
    ]
    assert "real_sdk_runner_hardware_validation" in payload["sections"]["deferred"]
    assert payload["markdown"].startswith(
        "# armctrl 0.6.0-rc.3 simulation safety preview"
    )
