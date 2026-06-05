import json
import subprocess
import sys
from pathlib import Path

import yaml

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
            "0.6",
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
    candidate_path = tmp_path / "large_step_candidate.csv"
    candidate_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.010000,0.020000,0.300000,0.300000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "gravity_sweep",
            "--sample-hz",
            "100",
            "--duration",
            "0.01",
            "--amplitude",
            "0.20",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--candidate-trajectory",
            str(candidate_path),
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


def test_cli_sysid_plan_uses_profile_specific_joint_step_gate(
    tmp_path,
) -> None:
    safe_config = yaml.safe_load(
        Path("configs/x5.safe.yaml").read_text(encoding="utf-8")
    )
    safe_config["safety"]["sysid"]["profile_overrides"]["fourier_multisine"][
        "max_joint_step_rad"
    ] = 0.02
    safe_config["safety"]["sysid"]["profile_overrides"]["gravity_sweep"][
        "max_joint_step_rad"
    ] = 0.01
    safe_config_path = tmp_path / "x5.step.safe.yaml"
    safe_config_path.write_text(
        yaml.safe_dump(safe_config, sort_keys=False),
        encoding="utf-8",
    )
    candidate_path = tmp_path / "candidate.csv"
    candidate_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.010000,0.015000,0.300000,0.300000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )

    fourier = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "fourier_multisine",
            "--sample-hz",
            "100",
            "--duration",
            "0.01",
            "--amplitude",
            "0.02",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--candidate-trajectory",
            str(candidate_path),
            "--safe-config",
            str(safe_config_path),
            "--output",
            str(tmp_path / "fourier"),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    gravity = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "gravity_sweep",
            "--sample-hz",
            "100",
            "--duration",
            "0.01",
            "--amplitude",
            "0.02",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--candidate-trajectory",
            str(candidate_path),
            "--safe-config",
            str(safe_config_path),
            "--output",
            str(tmp_path / "gravity"),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    fourier_payload = json.loads(fourier.stdout)
    gravity_payload = json.loads(gravity.stdout)
    assert fourier_payload["artifact_safety"]["trajectory_step_check"]["status"] == "pass"
    assert gravity_payload["artifact_safety"]["trajectory_step_check"]["status"] == "fail"


def test_cli_sysid_plan_rejects_execution_velocity_limit_violation(
    tmp_path,
) -> None:
    safe_config = yaml.safe_load(
        Path("configs/x5.safe.yaml").read_text(encoding="utf-8")
    )
    safe_config["safety"]["sysid"]["profile_overrides"]["fourier_multisine"][
        "max_joint_step_rad"
    ] = 0.02
    safe_config["safety"]["sysid"]["profile_overrides"]["fourier_multisine"][
        "oed_velocity_limits_rad_s"
    ] = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    safe_config_path = tmp_path / "x5.velocity.safe.yaml"
    safe_config_path.write_text(
        yaml.safe_dump(safe_config, sort_keys=False),
        encoding="utf-8",
    )
    candidate_path = tmp_path / "fast_candidate.csv"
    candidate_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.010000,0.015000,0.300000,0.300000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "fourier_multisine",
            "--sample-hz",
            "100",
            "--duration",
            "0.01",
            "--amplitude",
            "0.02",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--candidate-trajectory",
            str(candidate_path),
            "--safe-config",
            str(safe_config_path),
            "--output",
            str(tmp_path / "plan"),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    manifest = json.loads(
        (tmp_path / "plan" / "manifest.json").read_text(encoding="utf-8")
    )
    assert payload["safety"]["allowed"] is False
    assert payload["safety"]["reason"] == "planned trajectory exceeds velocity limit"
    assert payload["artifact_safety"]["trajectory_velocity_check"]["status"] == "fail"
    assert manifest["safety"]["checks"]["trajectory_velocity_check"]["violations"][0][
        "check"
    ] == "oed_velocity_limits_rad_s"


def test_cli_sysid_plan_rejects_execution_acceleration_limit_violation(
    tmp_path,
) -> None:
    safe_config = yaml.safe_load(
        Path("configs/x5.safe.yaml").read_text(encoding="utf-8")
    )
    safe_config["safety"]["sysid"]["profile_overrides"]["fourier_multisine"][
        "max_joint_step_rad"
    ] = 0.02
    safe_config["safety"]["sysid"]["profile_overrides"]["fourier_multisine"][
        "oed_velocity_limits_rad_s"
    ] = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
    safe_config["safety"]["sysid"]["profile_overrides"]["fourier_multisine"][
        "oed_acceleration_limits_rad_s2"
    ] = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    safe_config_path = tmp_path / "x5.accel.safe.yaml"
    safe_config_path.write_text(
        yaml.safe_dump(safe_config, sort_keys=False),
        encoding="utf-8",
    )
    candidate_path = tmp_path / "jerky_candidate.csv"
    candidate_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.010000,0.001000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.020000,0.010000,0.300000,0.300000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "fourier_multisine",
            "--sample-hz",
            "100",
            "--duration",
            "0.02",
            "--amplitude",
            "0.02",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--candidate-trajectory",
            str(candidate_path),
            "--safe-config",
            str(safe_config_path),
            "--output",
            str(tmp_path / "plan"),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    manifest = json.loads(
        (tmp_path / "plan" / "manifest.json").read_text(encoding="utf-8")
    )
    assert payload["safety"]["allowed"] is False
    assert payload["safety"]["reason"] == "planned trajectory exceeds acceleration limit"
    assert payload["artifact_safety"]["trajectory_acceleration_check"]["status"] == "fail"
    assert manifest["safety"]["checks"]["trajectory_acceleration_check"][
        "violations"
    ][0]["check"] == "oed_acceleration_limits_rad_s2"
