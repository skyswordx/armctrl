import json
import subprocess
import sys

from armctrl.sysid import SysIdPlanner


def test_sysid_planner_exposes_three_profiles_with_external_tool_boundaries() -> None:
    planner = SysIdPlanner.default()

    profiles = planner.list_profiles()

    assert [profile.name for profile in profiles] == [
        "fourier_multisine",
        "friction_sweep",
        "gravity_sweep",
    ]
    assert profiles[0].tool_boundary["trajectory_optimization"] == "FIGAROH"
    assert profiles[0].tool_boundary["regressor_solver"] == "Pinocchio"


def test_sysid_plan_rejects_execute_and_returns_handoff_contract() -> None:
    planner = SysIdPlanner.default()

    plan = planner.plan("gravity_sweep", execute=False)

    assert plan.schema == "armctrl.sysid_plan.v1"
    assert plan.profile.name == "gravity_sweep"
    assert plan.safety.allowed is True
    assert plan.handoff["dataset_contract"] == "lerobot-compatible"
    assert plan.handoff["solver_backends"] == ["pinocchio", "figaroh"]


def test_sysid_execute_is_rejected_until_hardware_runner_exists() -> None:
    planner = SysIdPlanner.default()

    plan = planner.plan("fourier_multisine", execute=True)

    assert plan.safety.allowed is False
    assert plan.safety.reason == "sysid execution runner is not implemented in clean rebuild"


def test_cli_sysid_plan_outputs_json_contract() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "sysid", "plan", "friction_sweep", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_plan.v1"
    assert payload["profile"]["name"] == "friction_sweep"
    assert payload["handoff"]["solver_backends"] == ["pinocchio", "figaroh"]


def test_cli_sysid_plan_marks_profile_parameter_limits_as_safety_fail(
    tmp_path,
) -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "gravity_sweep",
            "--duration",
            "120",
            "--amplitude",
            "0.5",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--output",
            str(tmp_path / "plan"),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["artifact_safety"]["allowed"] is False
    assert payload["artifact_safety"]["sysid_parameter_check"]["status"] == "fail"
    assert payload["artifact_safety"]["sysid_parameter_check"]["violation_count"] == 2


def test_cli_sysid_plan_marks_large_inter_sample_joint_steps_as_safety_fail(
    tmp_path,
) -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "gravity_sweep",
            "--sample-hz",
            "4",
            "--duration",
            "1",
            "--amplitude",
            "0.20",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--output",
            str(tmp_path / "plan"),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["safety"]["allowed"] is False
    assert payload["safety"]["reason"] == "planned trajectory exceeds max joint step"
    assert payload["artifact_safety"]["allowed"] is False
    assert payload["artifact_safety"]["trajectory_step_check"]["status"] == "fail"
    assert payload["artifact_safety"]["trajectory_step_check"]["violation_count"] > 0
