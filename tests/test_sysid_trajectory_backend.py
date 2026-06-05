import csv
import json
import sys
import types
import subprocess
from pathlib import Path

import pytest
import yaml

from armctrl.sysid import SysIdPlanRequest, trajectory_rows
from armctrl.sysid_trajectory_backend import plan_sysid_trajectory


def _request(profile: str, tmp_path: Path) -> SysIdPlanRequest:
    return SysIdPlanRequest(
        profile_name=profile,
        dof=6,
        sample_hz=100,
        duration_s=2,
        amplitude_rad=0.05,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path,
    )


def _write_candidate(path: Path) -> None:
    path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.010000,0.010000,0.310000,0.310000,0.000000,0.000000,0.000000\n"
        "0.020000,0.020000,0.320000,0.320000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )


def _write_relation_violation_candidate(path: Path) -> None:
    path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.500000,0.250000,0.000000,0.000000,0.000000\n"
        "0.010000,0.000000,0.500000,0.250000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )


def _write_oed_safe_config(path: Path) -> None:
    safe_config = yaml.safe_load(Path("configs/x5.safe.yaml").read_text(encoding="utf-8"))
    safe_config["safety"]["sysid"]["oed"] = {
        "n_wps": 7,
        "stack_reps": 3,
        "ipopt_max_iterations": 900,
        "ipopt_print_level": 7,
        "condition_number_threshold": 250.0,
        "random_seed": 42,
    }
    path.write_text(yaml.safe_dump(safe_config, sort_keys=False), encoding="utf-8")


def test_fourier_backend_prefers_figaroh_oed_and_labels_fallback(
    tmp_path: Path,
) -> None:
    plan = plan_sysid_trajectory(_request("fourier_multisine", tmp_path))

    assert plan.backend["requested"] == "figaroh_optimal_trajectory"
    assert plan.backend["profile_role"] == "oed"
    assert plan.backend["implementation_boundary"] == (
        "armctrl orchestrates; FIGAROH owns optimal excitation math"
    )
    assert plan.backend["fallback"]["name"] == "deterministic_smoke_probe"
    assert plan.backend["fallback"]["profile_role"] == "smoke_test"
    assert plan.backend["fallback"]["oed_valid"] is False
    assert plan.backend["oed_valid"] is False
    assert plan.backend["hardware_execution_eligible"] is False
    assert plan.backend["next_gate"] == (
        "improve_figaroh_oed_until_regressor_quality_gate_passes"
    )
    assert plan.backend["oed_quality_gate"]["status"] == "fail"
    assert "external_figaroh_oed_not_run" in plan.backend["oed_quality_gate"]["reasons"]
    assert plan.backend["figaroh_vendor_reference"]["class"] == (
        "figaroh.optimal.BaseOptimalTrajectory"
    )
    assert Path(plan.backend["figaroh_vendor_reference"]["source_path"]).exists()
    assert plan.backend["dependency_status"]["figaroh"]["status"] in {
        "available",
        "vendored_source",
    }
    assert plan.backend["figaroh_vendor_reference"]["objective"] == (
        "base regressor condition number"
    )


def test_sysid_plan_manifest_records_trajectory_backend_boundary(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "plan"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "fourier_multisine",
            "--dof",
            "6",
            "--sample-hz",
            "100",
            "--duration",
            "2",
            "--amplitude",
            "0.05",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))

    assert payload["trajectory_backend"]["requested"] == "figaroh_optimal_trajectory"
    assert payload["trajectory_backend"]["oed_valid"] is False
    assert manifest["trajectory_backend"] == payload["trajectory_backend"]
    assert manifest["handoff"]["trajectory_optimizer"] == "figaroh"
    assert Path(manifest["trajectory_backend"]["artifacts"]["figaroh_config"]).exists()
    score = manifest["trajectory_backend"]["regressor_score"]
    assert score["backend"] == "pinocchio"
    assert score["status"] in {"computed", "not_evaluated", "failed"}
    if score["status"] == "not_evaluated":
        assert score["reason"]


def test_planned_trajectory_is_not_silent_joint1_only_placeholder(
    tmp_path: Path,
) -> None:
    plan = plan_sysid_trajectory(_request("fourier_multisine", tmp_path))
    rows = plan.rows

    ranges = []
    for joint_index in range(6):
        values = [float(row[f"q_cmd_{joint_index + 1}"]) for row in rows]
        ranges.append(max(values) - min(values))

    assert ranges[0] > 0.0
    assert sum(value > 0.0 for value in ranges[1:]) >= 2


def test_cli_writes_backend_handoff_config_next_to_trajectory(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "plan"

    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "gravity_sweep",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    with (output_dir / "planned_trajectory.csv").open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    figaroh_config = json.loads(
        (output_dir / "figaroh_trajectory_request.json").read_text(encoding="utf-8")
    )

    assert len(rows) == 1001
    assert figaroh_config["schema"] == "armctrl.figaroh_optimal_trajectory_request.v1"
    assert figaroh_config["profile"] == "gravity_sweep"
    assert figaroh_config["model"]["active_joints"] == [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    ]
    assert figaroh_config["constraints"]["safe_config_path"] == "configs/x5.safe.yaml"


def test_figaroh_handoff_embeds_numeric_safety_limits(tmp_path: Path) -> None:
    request = _request("fourier_multisine", tmp_path)
    request = SysIdPlanRequest(
        profile_name=request.profile_name,
        dof=request.dof,
        sample_hz=20,
        duration_s=request.duration_s,
        amplitude_rad=0.05,
        q_center=request.q_center,
        urdf_path=request.urdf_path,
        safe_config_path=request.safe_config_path,
        output_dir=tmp_path,
    )

    plan_sysid_trajectory(request)
    figaroh_config = json.loads(
        (tmp_path / "figaroh_trajectory_request.json").read_text(encoding="utf-8")
    )

    constraints = figaroh_config["constraints"]
    assert constraints["max_joint_step_rad"] == 0.01
    assert constraints["derived_velocity_limit_rad_s"] == 0.2
    assert constraints["joint_limits_rad"][0] == [-0.05, 0.05]
    assert constraints["joint_limits_rad"][1] == [0.25, 0.35]
    assert constraints["velocity_limits_rad_s"][0] == [-0.2, 0.2]
    assert constraints["velocity_limits_rad_s"][5] == [-0.2, 0.2]
    assert constraints["effort_limits_nm"][2] == [-30.0, 30.0]
    timing = figaroh_config["figaroh"]["timing"]
    assert timing["execution_sample_hz"] == 20.0
    assert timing["execution_sample_period_s"] == 0.05
    assert timing["n_wps"] == 5
    assert timing["stack_reps"] == 1
    assert timing["requested_duration_s"] == 2.0
    assert timing["segment_duration_s"] >= 0.5
    assert timing["effective_duration_s"] >= timing["requested_duration_s"]
    assert figaroh_config["figaroh"]["optimizer"]["random_seed"] == 1


def test_figaroh_handoff_records_effective_execution_sample_count(
    tmp_path: Path,
) -> None:
    request = _request("fourier_multisine", tmp_path)
    request = SysIdPlanRequest(
        profile_name=request.profile_name,
        dof=request.dof,
        sample_hz=20,
        duration_s=1,
        amplitude_rad=0.1,
        q_center=request.q_center,
        urdf_path=request.urdf_path,
        safe_config_path=request.safe_config_path,
        output_dir=tmp_path,
    )

    plan_sysid_trajectory(request)
    figaroh_config = json.loads(
        (tmp_path / "figaroh_trajectory_request.json").read_text(encoding="utf-8")
    )

    timing = figaroh_config["figaroh"]["timing"]
    assert timing["duration_adjusted_for_safety"] is True
    assert timing["effective_duration_s"] == 8.0
    assert figaroh_config["sampling"]["requested_sample_count"] == 21
    assert figaroh_config["sampling"]["effective_sample_count"] == 161
    assert figaroh_config["sampling"]["sample_count"] == 161


def test_figaroh_handoff_uses_safe_config_oed_timing_and_quality_gate(
    tmp_path: Path,
) -> None:
    safe_config_path = tmp_path / "x5.oed.safe.yaml"
    _write_oed_safe_config(safe_config_path)
    request = _request("fourier_multisine", tmp_path)
    request = SysIdPlanRequest(
        profile_name=request.profile_name,
        dof=request.dof,
        sample_hz=20,
        duration_s=12,
        amplitude_rad=0.01,
        q_center=request.q_center,
        urdf_path=request.urdf_path,
        safe_config_path=str(safe_config_path),
        output_dir=tmp_path,
    )

    plan = plan_sysid_trajectory(request)
    figaroh_config = json.loads(
        (tmp_path / "figaroh_trajectory_request.json").read_text(encoding="utf-8")
    )

    timing = figaroh_config["figaroh"]["timing"]
    assert timing["n_wps"] == 7
    assert timing["stack_reps"] == 3
    assert timing["requested_segment_duration_s"] == 4.0
    assert timing["waypoint_duration_s"] == pytest.approx(4.0 / 6.0)
    assert timing["effective_duration_s"] == 12.0
    assert figaroh_config["figaroh"]["optimizer"]["ipopt_max_iterations"] == 900
    assert figaroh_config["figaroh"]["optimizer"]["ipopt_print_level"] == 7
    assert figaroh_config["figaroh"]["optimizer"]["random_seed"] == 42
    assert figaroh_config["figaroh"]["quality_gate"][
        "condition_number_threshold"
    ] == 250.0
    assert plan.backend["oed_quality_gate"]["condition_number_threshold"] == 250.0


def test_fallback_smoke_trajectory_uses_effective_timing_contract(
    tmp_path: Path,
) -> None:
    request = _request("fourier_multisine", tmp_path)
    request = SysIdPlanRequest(
        profile_name=request.profile_name,
        dof=request.dof,
        sample_hz=20,
        duration_s=1,
        amplitude_rad=0.1,
        q_center=request.q_center,
        urdf_path=request.urdf_path,
        safe_config_path=request.safe_config_path,
        output_dir=tmp_path,
    )

    plan = plan_sysid_trajectory(request)

    assert len(plan.rows) == 161
    assert plan.rows[-1]["time_s"] == "8.000000"
    assert plan.backend["fallback"]["sample_count"] == 161


def test_external_candidate_is_resampled_to_execution_sample_rate(
    tmp_path: Path,
) -> None:
    candidate_path = tmp_path / "coarse_figaroh_candidate.csv"
    candidate_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.100000,0.100000,0.400000,0.400000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )
    base_request = _request("fourier_multisine", tmp_path)
    request = SysIdPlanRequest(
        profile_name=base_request.profile_name,
        dof=base_request.dof,
        sample_hz=20,
        duration_s=0.1,
        amplitude_rad=base_request.amplitude_rad,
        q_center=base_request.q_center,
        urdf_path=base_request.urdf_path,
        safe_config_path=base_request.safe_config_path,
        output_dir=tmp_path,
        candidate_trajectory_path=candidate_path,
    )

    plan = plan_sysid_trajectory(request)

    assert [row["time_s"] for row in plan.rows] == [
        "0.000000",
        "0.050000",
        "0.100000",
    ]
    assert plan.rows[1]["q_cmd_1"] == "0.050000"
    assert plan.rows[1]["q_cmd_2"] == "0.350000"
    source = plan.backend["candidate_source"]
    assert source["raw_sample_count"] == 2
    assert source["sample_count"] == 3
    assert source["resampled_to_sample_hz"] == 20.0


def test_external_candidate_rejects_duration_off_execution_grid(
    tmp_path: Path,
) -> None:
    candidate_path = tmp_path / "off_grid_figaroh_candidate.csv"
    candidate_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.075000,0.100000,0.400000,0.400000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )
    base_request = _request("fourier_multisine", tmp_path)
    request = SysIdPlanRequest(
        profile_name=base_request.profile_name,
        dof=base_request.dof,
        sample_hz=20,
        duration_s=0.075,
        amplitude_rad=base_request.amplitude_rad,
        q_center=base_request.q_center,
        urdf_path=base_request.urdf_path,
        safe_config_path=base_request.safe_config_path,
        output_dir=tmp_path,
        candidate_trajectory_path=candidate_path,
    )

    with pytest.raises(ValueError, match="execution sample period"):
        plan_sysid_trajectory(request)


def test_cli_sysid_plan_uses_external_candidate_trajectory(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "plan"
    candidate_path = tmp_path / "figaroh_candidate.csv"
    _write_candidate(candidate_path)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "gravity_sweep",
            "--dof",
            "6",
            "--sample-hz",
            "100",
            "--duration",
            "0.02",
            "--amplitude",
            "0.05",
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
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    with (output_dir / "planned_trajectory.csv").open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    assert rows[1]["q_cmd_1"] == "0.010000"
    assert payload["trajectory_backend"]["selected"] == "external_candidate_trajectory"
    assert manifest["trajectory_backend"]["candidate_source"]["path"] == str(candidate_path)
    assert manifest["trajectory_backend"]["fallback"]["status"] == "not_used"
    assert manifest["trajectory_backend"]["regressor_score"]["backend"] == "pinocchio"


def test_cli_sysid_plan_runs_external_oed_command_then_imports_candidate(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "plan"
    command_path = tmp_path / "write_candidate.py"
    command_path.write_text(
        "from pathlib import Path\n"
        "import json\n"
        "import os\n"
        "request_path = Path(os.environ['ARMCTRL_FIGAROH_REQUEST'])\n"
        "candidate_path = Path(os.environ['ARMCTRL_CANDIDATE_TRAJECTORY'])\n"
        "request = json.loads(request_path.read_text(encoding='utf-8'))\n"
        "assert request['figaroh']['class'] == 'figaroh.optimal.BaseOptimalTrajectory'\n"
        "candidate_path.write_text(\n"
        "    'time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\\n'\n"
        "    '0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\\n'\n"
        "    '0.010000,0.020000,0.330000,0.330000,0.010000,0.000000,0.000000\\n',\n"
        "    encoding='utf-8',\n"
        ")\n",
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
            "--dof",
            "6",
            "--sample-hz",
            "100",
            "--duration",
            "0.01",
            "--amplitude",
            "0.05",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--output",
            str(output_dir),
            "--json",
            "--trajectory-command",
            sys.executable,
            str(command_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    with (output_dir / "planned_trajectory.csv").open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    assert rows[1]["q_cmd_1"] == "0.020000"
    assert payload["trajectory_backend"]["selected"] == "external_oed_command"
    assert manifest["trajectory_backend"]["candidate_source"]["generated_by"][
        "command_argv"
    ] == [sys.executable, str(command_path)]
    assert manifest["trajectory_backend"]["candidate_source"]["generated_by"][
        "exit_code"
    ] == 0
    assert manifest["trajectory_backend"]["fallback"]["status"] == "not_used"


def test_cli_sysid_plan_records_external_oed_command_stdout_json(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "plan"
    command_path = tmp_path / "write_candidate_with_status.py"
    command_path.write_text(
        "from pathlib import Path\n"
        "import json\n"
        "import os\n"
        "candidate_path = Path(os.environ['ARMCTRL_CANDIDATE_TRAJECTORY'])\n"
        "candidate_path.write_text(\n"
        "    'time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\\n'\n"
        "    '0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\\n',\n"
        "    encoding='utf-8',\n"
        ")\n"
        "print(json.dumps({'status': 'ok', 'backend': 'figaroh'}))\n",
        encoding="utf-8",
    )

    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "fourier_multisine",
            "--dof",
            "6",
            "--output",
            str(output_dir),
            "--json",
            "--trajectory-command",
            sys.executable,
            str(command_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    generated_by = manifest["trajectory_backend"]["candidate_source"]["generated_by"]
    assert generated_by["stdout_json"]["status"] == "ok"
    assert generated_by["stdout_json"]["backend"] == "figaroh"


def test_cli_sysid_plan_extracts_figaroh_json_after_ipopt_log(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "plan"
    command_path = tmp_path / "write_candidate_after_ipopt_log.py"
    command_path.write_text(
        "from pathlib import Path\n"
        "import json\n"
        "import os\n"
        "candidate_path = Path(os.environ['ARMCTRL_CANDIDATE_TRAJECTORY'])\n"
        "candidate_path.write_text(\n"
        "    'time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\\n'\n"
        "    '0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\\n',\n"
        "    encoding='utf-8',\n"
        ")\n"
        "print('Number of Iterations....: 200')\n"
        "print('Objective...............:   1.230000e-01    1.750000e+05')\n"
        "print('Dual infeasibility......:   2.400000e+00    3.500000e+06')\n"
        "print('Constraint violation....:   0.000000e+00    0.000000e+00')\n"
        "print('EXIT: Maximum Number of Iterations Exceeded.')\n"
        "print(json.dumps({'status': 'ok', 'backend': 'figaroh'}))\n",
        encoding="utf-8",
    )

    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "fourier_multisine",
            "--dof",
            "6",
            "--output",
            str(output_dir),
            "--json",
            "--trajectory-command",
            sys.executable,
            str(command_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    backend = manifest["trajectory_backend"]
    generated_by = backend["candidate_source"]["generated_by"]
    assert generated_by["stdout_json"]["status"] == "ok"
    assert generated_by["optimizer_convergence"]["status"] == "fail"
    assert generated_by["optimizer_convergence"]["reason"] == "max_iterations_exceeded"
    assert generated_by["optimizer_diagnostics"] == {
        "iterations": 200,
        "objective_scaled": 0.123,
        "objective_unscaled": 175000.0,
        "dual_infeasibility_scaled": 2.4,
        "dual_infeasibility_unscaled": 3500000.0,
        "constraint_violation_scaled": 0.0,
        "constraint_violation_unscaled": 0.0,
    }
    assert backend["oed_quality_gate"]["status"] == "fail"
    assert "optimizer_not_converged" in backend["oed_quality_gate"]["reasons"]
    assert backend["hardware_execution_eligible"] is False


def test_figaroh_handoff_omits_x5_joint_relation_constraints_for_fourier_oed(
    tmp_path: Path,
) -> None:
    plan_sysid_trajectory(_request("fourier_multisine", tmp_path))
    figaroh_config = json.loads(
        (tmp_path / "figaroh_trajectory_request.json").read_text(encoding="utf-8")
    )

    constraints = figaroh_config["constraints"]
    assert constraints["joint_relation_constraints"] == []
    assert constraints["profile_safety"]["max_sysid_amplitude_rad"] == 0.8


def test_figaroh_handoff_keeps_x5_joint_relation_constraints_for_gravity(
    tmp_path: Path,
) -> None:
    plan_sysid_trajectory(_request("gravity_sweep", tmp_path))
    figaroh_config = json.loads(
        (tmp_path / "figaroh_trajectory_request.json").read_text(encoding="utf-8")
    )

    constraints = figaroh_config["constraints"]
    assert constraints["joint_relation_constraints"] == [
        {
            "name": "x5_joint2_joint3_parallel_band",
            "left_joint": 2,
            "right_joint": 3,
            "min_delta_rad": -0.08,
            "max_delta_rad": 0.08,
        }
    ]


def test_cli_sysid_plan_rejects_joint_relation_constraint_violation(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "plan"
    candidate_path = tmp_path / "relation_violation_candidate.csv"
    _write_relation_violation_candidate(candidate_path)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "gravity_sweep",
            "--dof",
            "6",
            "--sample-hz",
            "100",
            "--duration",
            "0.01",
            "--amplitude",
            "0.05",
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
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    relation_check = manifest["safety"]["checks"]["joint_relation_check"]
    assert payload["safety"]["allowed"] is False
    assert payload["safety"]["reason"] == "planned trajectory violates joint relation constraints"
    assert relation_check["status"] == "fail"
    assert relation_check["violations"][0]["constraint"] == "x5_joint2_joint3_parallel_band"


def test_cli_sysid_plan_does_not_use_joint_relation_gate_for_fourier_oed(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "plan"
    candidate_path = tmp_path / "relation_violation_candidate.csv"
    _write_relation_violation_candidate(candidate_path)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "fourier_multisine",
            "--dof",
            "6",
            "--sample-hz",
            "100",
            "--duration",
            "0.01",
            "--amplitude",
            "0.05",
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
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    relation_check = manifest["safety"]["checks"]["joint_relation_check"]
    assert relation_check["status"] == "pass"
    assert relation_check["constraints"] == []
    assert payload["safety"]["reason"] != "planned trajectory violates joint relation constraints"


def test_cli_sysid_plan_reports_external_oed_command_failure_as_json(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "plan"
    command_path = tmp_path / "fail_candidate.py"
    command_path.write_text(
        "import json\n"
        "import sys\n"
        "print(json.dumps({'status': 'failed', 'reason': 'cyipopt_missing'}))\n"
        "sys.exit(1)\n",
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
            "--dof",
            "6",
            "--output",
            str(output_dir),
            "--json",
            "--trajectory-command",
            sys.executable,
            str(command_path),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    assert completed.returncode == 1
    assert "Traceback" not in completed.stderr
    assert payload["status"] == "faulted"
    assert payload["error"]["code"] == "trajectory_command_failed"
    assert payload["error"]["detail"]["stdout_json"]["reason"] == "cyipopt_missing"


def test_trajectory_rows_does_not_write_backend_artifacts(tmp_path: Path) -> None:
    rows = trajectory_rows(_request("gravity_sweep", tmp_path))

    assert rows
    assert not (tmp_path / "figaroh_trajectory_request.json").exists()


def test_pinocchio_regressor_score_computes_rank_with_available_backend(
    tmp_path: Path,
    monkeypatch,
) -> None:
    module = types.ModuleType("pinocchio")

    class FakeModel:
        nq = 6
        nv = 6

        def createData(self):
            return types.SimpleNamespace(jointTorqueRegressor=None)

    def build_model_from_urdf(_path: str):
        return FakeModel()

    def compute_joint_torque_regressor(_model, _data, q, v, a):
        rows = []
        for joint_index in range(6):
            row = [0.0] * 12
            row[joint_index] = 1.0 + float(q[joint_index])
            row[joint_index + 6] = 0.5 * float(v[joint_index]) + float(a[joint_index])
            rows.append(row)
        return rows

    module.buildModelFromUrdf = build_model_from_urdf
    module.computeJointTorqueRegressor = compute_joint_torque_regressor
    monkeypatch.setitem(sys.modules, "pinocchio", module)

    plan = plan_sysid_trajectory(_request("fourier_multisine", tmp_path))

    score = plan.backend["regressor_score"]
    assert score["status"] == "computed"
    assert score["backend"] == "pinocchio"
    assert score["row_count"] == len(plan.rows) * 6
    assert score["column_count"] == 12
    assert score["rank"] > 0
    assert score["effective_condition_number"] > 0.0


def test_external_oed_base_regressor_score_is_preferred_for_quality_gate(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "plan"
    command_path = tmp_path / "write_candidate_with_base_score.py"
    command_path.write_text(
        "from pathlib import Path\n"
        "import json\n"
        "import os\n"
        "candidate_path = Path(os.environ['ARMCTRL_CANDIDATE_TRAJECTORY'])\n"
        "candidate_path.write_text(\n"
        "    'time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\\n'\n"
        "    '0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\\n',\n"
        "    encoding='utf-8',\n"
        ")\n"
        "print(json.dumps({\n"
        "    'status': 'ok',\n"
        "    'base_regressor_score': {\n"
        "        'status': 'computed',\n"
        "        'condition_number': 42.0,\n"
        "        'row_count': 198,\n"
        "        'column_count': 36,\n"
        "        'base_parameter_count': 36\n"
        "    }\n"
        "}))\n",
        encoding="utf-8",
    )

    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "fourier_multisine",
            "--dof",
            "6",
            "--output",
            str(output_dir),
            "--json",
            "--trajectory-command",
            sys.executable,
            str(command_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    backend = manifest["trajectory_backend"]
    assert backend["base_regressor_score"] == {
        "status": "computed",
        "condition_number": 42.0,
        "row_count": 198,
        "column_count": 36,
        "base_parameter_count": 36,
    }
    assert backend["oed_quality_gate"]["status"] == "pass"
    assert backend["oed_quality_gate"]["primary_metric"] == "figaroh_base_regressor"
