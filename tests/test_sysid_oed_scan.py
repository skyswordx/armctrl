import json
from pathlib import Path

import pytest
import yaml

from scripts.x5_oed_freeze_candidate import freeze_candidate
from scripts.x5_oed_followup_plan import build_followup_plan
from armctrl.sysid_oed_scan import OedScanRequest, OedScanRunner
from armctrl.sysid_trajectory_backend import TrajectoryCommandError


def test_oed_scan_writes_candidate_safe_configs_and_summary(tmp_path: Path) -> None:
    request = OedScanRequest(
        profile_name="fourier_multisine",
        dof=6,
        sample_hz=20.0,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path,
        durations_s=(1.0,),
        amplitudes_rad=(0.01, 0.02),
        n_wps_values=(5,),
        stack_reps_values=(1,),
        random_seed_values=(10, 11),
        ipopt_max_iterations=300,
        ipopt_print_level=5,
        condition_number_threshold=500.0,
    )

    result = OedScanRunner().run(request)

    summary_path = tmp_path / "oed_scan_summary.json"
    attempts_path = tmp_path / "oed_scan_attempts.json"
    report_path = tmp_path / "oed_scan_report.md"
    ipopt_stdout_path = tmp_path / "representative_ipopt_stdout.txt"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    flattened = json.loads(attempts_path.read_text(encoding="utf-8"))
    report = report_path.read_text(encoding="utf-8")
    assert result["schema"] == "armctrl.sysid_oed_scan.v1"
    assert summary == result
    assert len(flattened["attempts"]) == 4
    assert flattened["attempts"][0]["attempt_id"] == "attempt-001"
    assert flattened["attempts"][0]["status"] == "ok"
    assert "| attempt-001 | ok |" in report
    assert len(result["attempts"]) == 4
    first = result["attempts"][0]
    assert first["status"] == "ok"
    assert first["safety_allowed"] is True
    assert first["timing_contract"]["planning_sample_hz"] == 20.0
    assert first["timing_contract"]["execution_sample_hz"] == 100.0
    assert first["oed_quality_gate"]["condition_number_threshold"] == 500.0

    safe_config = yaml.safe_load(
        Path(first["safe_config"]).read_text(encoding="utf-8")
    )
    assert safe_config["safety"]["sysid"]["oed"] == {
        "n_wps": 5,
        "stack_reps": 1,
        "random_seed": 10,
        "ipopt_max_iterations": 300,
        "ipopt_print_level": 5,
        "condition_number_threshold": 500.0,
    }
    assert result["artifacts"] == {
        "summary": str(summary_path),
        "attempts": str(attempts_path),
        "report": str(report_path),
        "representative_ipopt_stdout": None,
    }
    assert result["best_attempt"]["attempt_id"] in {
        attempt["attempt_id"] for attempt in result["attempts"]
    }
    assert result["target_condition"]["target_condition_number"] == 100.0
    assert result["target_condition"]["best_condition_metric"] == (
        result["best_attempt"]["condition_metric"]
    )
    assert result["target_condition"]["best_condition_number"] == (
        result["best_attempt"]["condition_number"]
    )
    assert result["target_condition"]["best_attempt_id"] == (
        result["best_attempt"]["attempt_id"]
    )
    assert result["target_condition"]["status"] in {"not_evaluated", "not_met", "met"}
    assert result["target_condition"]["next_gate"] in {
        "run_structural_oed_scan",
        "continue_structural_oed_search",
        "freeze_reproducible_candidate",
    }


def test_oed_scan_target_condition_marks_numeric_near_miss(tmp_path: Path) -> None:
    class NumericNearMissPlanner:
        def write_plan(self, _request):
            class Plan:
                def to_json(self):
                    return {
                        "trajectory_backend": {
                            "oed_valid": False,
                            "hardware_execution_eligible": False,
                            "oed_quality_gate": {
                                "status": "fail",
                                "reasons": ["optimizer_not_converged"],
                            },
                            "condition_number": 106.28,
                            "rank": 36,
                            "timing_contract": {},
                            "sampling_contract": {},
                        },
                        "safety": {"allowed": True, "reason": "ok"},
                        "artifacts": {},
                    }

            return Plan()

    request = OedScanRequest(
        profile_name="fourier_multisine",
        dof=6,
        sample_hz=20.0,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path,
        durations_s=(1.0,),
        amplitudes_rad=(0.5,),
        n_wps_values=(5,),
        stack_reps_values=(1,),
        random_seed_values=(3,),
        ipopt_max_iterations=500,
        ipopt_print_level=5,
        condition_number_threshold=500.0,
    )

    result = OedScanRunner(planner=NumericNearMissPlanner()).run(request)

    assert result["target_condition"] == {
        "target_condition_number": 100.0,
        "status": "not_met",
        "best_condition_metric": "pinocchio_effective_regressor",
        "best_condition_number": 106.28,
        "best_attempt_id": "attempt-001",
        "next_gate": "continue_structural_oed_search",
    }


def test_oed_scan_target_condition_uses_figaroh_base_condition(
    tmp_path: Path,
) -> None:
    class MixedMetricPlanner:
        def write_plan(self, _request):
            class Plan:
                def to_json(self):
                    return {
                        "trajectory_backend": {
                            "oed_valid": False,
                            "hardware_execution_eligible": False,
                            "oed_quality_gate": {
                                "status": "fail",
                                "reasons": ["base_regressor_condition_too_high"],
                                "primary_metric": "figaroh_base_regressor",
                            },
                            "condition_number": 106.28,
                            "rank": 36,
                            "base_regressor_score": {
                                "status": "computed",
                                "condition_number": 150.84,
                                "base_parameter_count": 36,
                            },
                            "regressor_score": {
                                "status": "computed",
                                "effective_condition_number": 106.28,
                                "rank": 36,
                            },
                            "timing_contract": {},
                            "sampling_contract": {},
                            "execution_trajectory": {
                                "max_joint_step_rad": 0.0196,
                                "max_velocity_rad_s": 1.95,
                                "max_acceleration_rad_s2": 25.51,
                            },
                            "candidate_source": {
                                "generated_by": {
                                    "optimizer_convergence": {
                                        "status": "fail",
                                        "reason": "dual_infeasible",
                                    }
                                }
                            },
                        },
                        "safety": {"allowed": True, "reason": "ok"},
                        "artifacts": {},
                    }

            return Plan()

    request = OedScanRequest(
        profile_name="fourier_multisine",
        dof=6,
        sample_hz=20.0,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path,
        durations_s=(1.0,),
        amplitudes_rad=(0.5,),
        n_wps_values=(5,),
        stack_reps_values=(1,),
        random_seed_values=(3,),
        ipopt_max_iterations=500,
        ipopt_print_level=5,
        condition_number_threshold=500.0,
    )

    result = OedScanRunner(planner=MixedMetricPlanner()).run(request)
    flattened = json.loads(
        (tmp_path / "oed_scan_attempts.json").read_text(encoding="utf-8")
    )

    assert result["best_attempt"]["condition_metric"] == "figaroh_base_regressor"
    assert result["best_attempt"]["condition_number"] == 150.84
    assert result["best_attempt"]["pinocchio_effective_condition_number"] == 106.28
    assert result["best_attempt"]["motion_summary"] == {
        "max_joint_step_rad": 0.0196,
        "max_velocity_rad_s": 1.95,
        "max_acceleration_rad_s2": 25.51,
    }
    assert result["best_attempt"]["optimizer_status"] == "fail"
    assert result["best_attempt"]["optimizer_reason"] == "dual_infeasible"
    assert result["target_condition"] == {
        "target_condition_number": 100.0,
        "status": "not_met",
        "best_condition_metric": "figaroh_base_regressor",
        "best_condition_number": 150.84,
        "best_attempt_id": "attempt-001",
        "next_gate": "continue_structural_oed_search",
    }
    assert flattened["attempts"][0]["condition_metric"] == "figaroh_base_regressor"
    assert flattened["attempts"][0]["condition_number"] == 150.84
    assert flattened["attempts"][0]["pinocchio_effective_condition_number"] == 106.28
    assert flattened["attempts"][0]["max_joint_step_rad"] == 0.0196
    assert flattened["attempts"][0]["max_velocity_rad_s"] == 1.95
    assert flattened["attempts"][0]["max_acceleration_rad_s2"] == 25.51
    assert flattened["attempts"][0]["optimizer_status"] == "fail"
    assert flattened["attempts"][0]["optimizer_reason"] == "dual_infeasible"


def test_oed_scan_records_trajectory_command_fault_and_continues(
    tmp_path: Path,
) -> None:
    class FaultingPlanner:
        def __init__(self) -> None:
            self.calls = 0

        def write_plan(self, _request):
            self.calls += 1
            raise TrajectoryCommandError(
                {
                    "exit_code": 1,
                    "stdout_json": {"status": "failed", "reason": "restoration_failed"},
                    "optimizer_convergence": {
                        "status": "fail",
                        "reason": "restoration_failed",
                    },
                    "optimizer_diagnostics": {
                        "constraint_violation_unscaled": 0.0,
                        "dual_infeasibility_unscaled": (
                            3916.7 if self.calls == 1 else 92408.9
                        ),
                    },
                }
            )

    request = OedScanRequest(
        profile_name="fourier_multisine",
        dof=6,
        sample_hz=20.0,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path,
        durations_s=(1.0, 2.0),
        amplitudes_rad=(0.02,),
        n_wps_values=(5,),
        stack_reps_values=(1,),
        random_seed_values=(10,),
        ipopt_max_iterations=300,
        ipopt_print_level=7,
        condition_number_threshold=500.0,
        trajectory_command_argv=("python", "scripts/x5_figaroh_oed.py"),
        attempt_timeout_s=120.0,
    )

    result = OedScanRunner(planner=FaultingPlanner()).run(request)

    assert len(result["attempts"]) == 2
    assert all(attempt["status"] == "faulted" for attempt in result["attempts"])
    assert result["best_attempt"] is None
    assert result["best_diagnostic_attempt"] == {
        "attempt_id": "attempt-001",
        "status": "faulted",
        "failure_classification": {
            "kind": "optimizer_dual_infeasible",
            "next_action": "tune IPOPT scaling/initialization or reduce objective ill-conditioning before changing hardware safety limits",
        },
        "optimizer_diagnostics": {
            "constraint_violation_unscaled": 0.0,
            "dual_infeasibility_unscaled": 3916.7,
        },
        "parameters": {
            "duration_s": 1.0,
            "amplitude_rad": 0.02,
            "n_wps": 5,
            "stack_reps": 1,
            "random_seed": 10,
            "ipopt_max_iterations": 300,
            "ipopt_print_level": 7,
            "condition_number_threshold": 500.0,
            "attempt_timeout_s": 120.0,
        },
        "output_dir": str(tmp_path / "attempt-001"),
    }
    assert result["attempts"][0]["error"]["code"] == "trajectory_command_failed"
    assert result["attempts"][0]["error"]["detail"]["optimizer_convergence"][
        "reason"
    ] == "restoration_failed"
    assert result["attempts"][0]["failure_classification"] == {
        "kind": "optimizer_dual_infeasible",
        "next_action": "tune IPOPT scaling/initialization or reduce objective ill-conditioning before changing hardware safety limits",
    }


def test_oed_scan_classifies_figaroh_cubic_spline_infeasible_fault(
    tmp_path: Path,
) -> None:
    class FaultingPlanner:
        def write_plan(self, _request):
            raise TrajectoryCommandError(
                {
                    "exit_code": 1,
                    "stdout_json": {
                        "status": "failed",
                        "reason": "figaroh_oed_failed",
                        "message": "FIGAROH results did not include T_F/P_F segments",
                    },
                    "optimizer_convergence": {
                        "status": "not_evaluated",
                        "reason": "optimizer_exit_not_reported",
                    },
                    "stderr": (
                        "WARNING:figaroh.utils.cubic_spline:"
                        "Joint vel idx_v 1 limits violated!\n"
                        "WARNING:figaroh.utils.cubic_spline:"
                        "FAILED to generate a feasible cubic spline\n"
                    ),
                }
            )

    request = OedScanRequest(
        profile_name="fourier_multisine",
        dof=6,
        sample_hz=20.0,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path,
        durations_s=(2.0,),
        amplitudes_rad=(0.45,),
        n_wps_values=(7,),
        stack_reps_values=(1,),
        random_seed_values=(2,),
        ipopt_max_iterations=300,
        ipopt_print_level=5,
        condition_number_threshold=500.0,
        trajectory_command_argv=("python", "scripts/x5_figaroh_oed.py"),
        attempt_timeout_s=300.0,
    )

    result = OedScanRunner(planner=FaultingPlanner()).run(request)

    assert result["attempts"][0]["failure_classification"] == {
        "kind": "figaroh_cubic_spline_infeasible",
        "next_action": "repair FIGAROH waypoint initialization or relax OED velocity limits before adding seeds or IPOPT iterations",
    }
    assert result["best_diagnostic_attempt"]["failure_classification"] == (
        result["attempts"][0]["failure_classification"]
    )


def test_oed_scan_classifies_missing_cyipopt_dependency(
    tmp_path: Path,
) -> None:
    class FaultingPlanner:
        def write_plan(self, _request):
            raise TrajectoryCommandError(
                {
                    "exit_code": 1,
                    "stdout_json": {
                        "status": "failed",
                        "reason": "figaroh_oed_failed",
                        "message": "FIGAROH results did not include T_F/P_F segments",
                    },
                    "optimizer_convergence": {
                        "status": "not_evaluated",
                        "reason": "optimizer_exit_not_reported",
                    },
                    "stderr": (
                        "ERROR:figaroh.tools.robotipopt.RobotIPOPTSolver:"
                        "Error during optimization: cyipopt is required for IPOPT "
                        "optimization. Install with: pip install cyipopt\n"
                    ),
                }
            )

    request = OedScanRequest(
        profile_name="fourier_multisine",
        dof=6,
        sample_hz=20.0,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path,
        durations_s=(2.0,),
        amplitudes_rad=(0.45,),
        n_wps_values=(7,),
        stack_reps_values=(1,),
        random_seed_values=(2,),
        ipopt_max_iterations=60,
        ipopt_print_level=5,
        condition_number_threshold=500.0,
        trajectory_command_argv=("python", "scripts/x5_figaroh_oed.py"),
        attempt_timeout_s=180.0,
    )

    result = OedScanRunner(planner=FaultingPlanner()).run(request)

    assert result["attempts"][0]["failure_classification"] == {
        "kind": "figaroh_ipopt_dependency_missing",
        "next_action": "install or select a WSL/workstation environment with cyipopt before judging OED convergence",
    }


def test_oed_scan_classifies_cyipopt_jacobian_contract_fault(
    tmp_path: Path,
) -> None:
    class FaultingPlanner:
        def write_plan(self, _request):
            raise TrajectoryCommandError(
                {
                    "exit_code": 1,
                    "stdout_json": {
                        "status": "failed",
                        "reason": "figaroh_oed_failed",
                        "message": "FIGAROH results did not include T_F/P_F segments",
                    },
                    "optimizer_convergence": {
                        "status": "not_evaluated",
                        "reason": "optimizer_exit_not_reported",
                    },
                    "stderr": (
                        "ERROR:cyipopt:b'Invalid number of indices returned "
                        "from jacobian'\n"
                        "EXIT: Invalid number in NLP function or derivative "
                        "detected.\n"
                    ),
                }
            )

    request = OedScanRequest(
        profile_name="fourier_multisine",
        dof=6,
        sample_hz=20.0,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path,
        durations_s=(2.0,),
        amplitudes_rad=(0.45,),
        n_wps_values=(7,),
        stack_reps_values=(1,),
        random_seed_values=(2,),
        ipopt_max_iterations=60,
        ipopt_print_level=5,
        condition_number_threshold=500.0,
        trajectory_command_argv=("python", "scripts/x5_figaroh_oed.py"),
        attempt_timeout_s=180.0,
    )

    result = OedScanRunner(planner=FaultingPlanner()).run(request)

    assert result["attempts"][0]["failure_classification"] == {
        "kind": "figaroh_cyipopt_jacobian_contract",
        "next_action": "repair the armctrl FIGAROH/cyipopt jacobian adapter before changing seeds or OED timing",
    }


def test_oed_scan_writes_representative_ipopt_stdout_artifact(
    tmp_path: Path,
) -> None:
    class FaultingPlanner:
        def write_plan(self, _request):
            raise TrajectoryCommandError(
                {
                    "exit_code": 1,
                    "stdout": (
                        "iter    objective    inf_pr   inf_du\n"
                        "   0  1.0e+05 0.00e+00 1.0e+02\n"
                        "EXIT: Maximum Number of Iterations Exceeded.\n"
                    ),
                    "optimizer_convergence": {
                        "status": "fail",
                        "reason": "max_iterations_exceeded",
                    },
                    "optimizer_diagnostics": {
                        "iterations": 1,
                        "constraint_violation_unscaled": 0.0,
                        "dual_infeasibility_unscaled": 100.0,
                        "iteration_log_tail": [
                            "   0  1.0e+05 0.00e+00 1.0e+02",
                            "   1  2.5e+04 0.00e+00 8.0e+00",
                        ],
                    },
                }
            )

    request = OedScanRequest(
        profile_name="fourier_multisine",
        dof=6,
        sample_hz=20.0,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path,
        durations_s=(1.0,),
        amplitudes_rad=(0.02,),
        n_wps_values=(5,),
        stack_reps_values=(1,),
        ipopt_max_iterations=300,
        ipopt_print_level=5,
        condition_number_threshold=500.0,
        trajectory_command_argv=("python", "scripts/x5_figaroh_oed.py"),
    )

    result = OedScanRunner(planner=FaultingPlanner()).run(request)

    stdout_path = tmp_path / "representative_ipopt_stdout.txt"
    flattened = json.loads(
        (tmp_path / "oed_scan_attempts.json").read_text(encoding="utf-8")
    )
    assert result["artifacts"]["representative_ipopt_stdout"] == str(stdout_path)
    assert flattened["attempts"][0]["optimizer_last_iter_objective"] == 25000.0
    assert flattened["attempts"][0]["optimizer_last_iter_inf_pr"] == 0.0
    assert flattened["attempts"][0]["optimizer_last_iter_inf_du"] == 8.0
    assert "EXIT: Maximum Number of Iterations Exceeded." in stdout_path.read_text(
        encoding="utf-8"
    )


def test_oed_scan_surfaces_trajectory_command_diagnostics(
    tmp_path: Path,
) -> None:
    class FakePlan:
        def to_json(self):
            return {
                "safety": {"allowed": True, "reason": "plan-only sysid preview"},
                "artifacts": {"manifest": "attempt/manifest.json"},
                "trajectory_backend": {
                    "oed_valid": False,
                    "hardware_execution_eligible": False,
                    "condition_number": 1234.0,
                    "rank": 36,
                    "timing_contract": {"execution_sample_hz": 20.0},
                    "sampling_contract": {"sample_count": 10},
                    "oed_quality_gate": {
                        "status": "fail",
                        "reasons": ["optimizer_not_converged"],
                    },
                    "candidate_source": {
                        "generated_by": {
                            "optimizer_convergence": {
                                "status": "fail",
                                "reason": "max_iterations_exceeded",
                            },
                            "optimizer_diagnostics": {
                                "iterations": 200,
                                "dual_infeasibility_unscaled": 3500000.0,
                                "iteration_log_tail": [
                                    " 199  1.0e+05 0.0e+00 2.0e+02",
                                    " 200  1.0e+05 0.0e+00 2.1e+02",
                                ],
                            },
                        }
                    },
                },
            }

    class FakePlanner:
        def write_plan(self, _request):
            return FakePlan()

    request = OedScanRequest(
        profile_name="fourier_multisine",
        dof=6,
        sample_hz=20.0,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path,
        durations_s=(1.0,),
        amplitudes_rad=(0.02,),
        n_wps_values=(5,),
        stack_reps_values=(1,),
        ipopt_max_iterations=300,
        ipopt_print_level=7,
        condition_number_threshold=500.0,
        trajectory_command_argv=("python", "scripts/x5_figaroh_oed.py"),
    )

    result = OedScanRunner(planner=FakePlanner()).run(request)

    assert result["attempts"][0]["trajectory_command"]["optimizer_diagnostics"][
        "dual_infeasibility_unscaled"
    ] == 3500000.0
    assert result["attempts"][0]["trajectory_command"]["optimizer_diagnostics"][
        "iteration_log_tail"
    ][-1].startswith(" 200")
    assert result["attempts"][0]["failure_classification"]["kind"] == (
        "optimizer_dual_infeasible"
    )


def test_oed_scan_passes_attempt_timeout_to_plan_request(tmp_path: Path) -> None:
    class CapturingPlanner:
        def __init__(self) -> None:
            self.requests = []

        def write_plan(self, request):
            self.requests.append(request)
            raise TrajectoryCommandError(
                {
                    "exit_code": "timeout",
                    "timeout_s": 12.5,
                    "optimizer_convergence": {
                        "status": "fail",
                        "reason": "trajectory_command_timeout",
                    },
                    "optimizer_diagnostics": {},
                }
            )

    planner = CapturingPlanner()
    request = OedScanRequest(
        profile_name="fourier_multisine",
        dof=6,
        sample_hz=20.0,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path="configs/models/X5_camera.urdf",
        safe_config_path="configs/x5.safe.yaml",
        output_dir=tmp_path,
        durations_s=(1.0,),
        amplitudes_rad=(0.02,),
        n_wps_values=(5,),
        stack_reps_values=(1,),
        ipopt_max_iterations=300,
        ipopt_print_level=5,
        condition_number_threshold=500.0,
        trajectory_command_argv=("python", "scripts/x5_figaroh_oed.py"),
        attempt_timeout_s=12.5,
    )

    result = OedScanRunner(planner=planner).run(request)

    assert planner.requests[0].trajectory_command_timeout_s == 12.5
    assert result["attempts"][0]["parameters"]["attempt_timeout_s"] == 12.5
    assert result["attempts"][0]["failure_classification"]["kind"] == (
        "optimizer_timeout"
    )


def test_freeze_candidate_copies_best_planned_trajectory_with_provenance(
    tmp_path: Path,
) -> None:
    attempt_dir = tmp_path / "attempt-002"
    attempt_dir.mkdir()
    planned = attempt_dir / "planned_trajectory.csv"
    execution = attempt_dir / "execution_trajectory.csv"
    planned.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )
    execution.write_text(planned.read_text(encoding="utf-8"), encoding="utf-8")
    (attempt_dir / "manifest.json").write_text(
        json.dumps(
            {
                "profile": {"name": "fourier_multisine"},
                "trajectory_backend": {
                    "condition_number": 106.27694211039746,
                    "rank": 36,
                    "base_regressor_score": {
                        "status": "computed",
                        "condition_number": 150.83617311561034,
                        "rank": 36,
                    },
                    "regressor_score": {
                        "status": "computed",
                        "effective_condition_number": 106.27694211039746,
                        "rank": 36,
                    },
                    "oed_quality_gate": {"status": "pass"},
                },
                "safety": {"allowed": True},
                "artifacts": {
                    "planned_trajectory": str(planned),
                    "execution_trajectory": str(execution),
                },
            }
        ),
        encoding="utf-8",
    )
    output_dir = tmp_path / "frozen"

    result = freeze_candidate(attempt_dir=attempt_dir, output_dir=output_dir)

    recommended = output_dir / "recommended_candidate.csv"
    manifest = json.loads(
        (output_dir / "best_candidate_manifest.json").read_text(encoding="utf-8")
    )
    assert result["schema"] == "armctrl.x5_oed_frozen_candidate.v1"
    assert recommended.read_text(encoding="utf-8") == planned.read_text(
        encoding="utf-8"
    )
    assert manifest["source_attempt"] == str(attempt_dir)
    assert manifest["source_planned_trajectory"] == str(planned)
    assert manifest["source_execution_trajectory"] == str(execution)
    assert manifest["condition_number"] == 150.83617311561034
    assert manifest["base_regressor_condition_number"] == 150.83617311561034
    assert manifest["pinocchio_effective_condition_number"] == 106.27694211039746
    assert manifest["rank"] == 36
    assert manifest["safety_allowed"] is True
    assert manifest["target_condition_number"] == 100.0
    assert manifest["target_condition_status"] == "fail"
    assert manifest["target_condition_metric"] == "figaroh_base_regressor"
    assert manifest["target_condition_margin"] == pytest.approx(50.83617311561034)
    assert manifest["next_gate"] == "continue_structural_oed_search"
    assert manifest["replay_hint"]["candidate_trajectory"] == str(recommended)


def test_freeze_candidate_blocks_hardware_next_gate_when_oed_gate_failed(
    tmp_path: Path,
) -> None:
    attempt_dir = tmp_path / "attempt-001"
    attempt_dir.mkdir()
    planned = attempt_dir / "planned_trajectory.csv"
    execution = attempt_dir / "execution_trajectory.csv"
    planned.write_text("time_s,q_cmd_1\n0.0,0.0\n", encoding="utf-8")
    execution.write_text("time_s,q_cmd_1\n0.0,0.0\n", encoding="utf-8")
    (attempt_dir / "manifest.json").write_text(
        json.dumps(
            {
                "trajectory_backend": {
                    "condition_number": 68.4,
                    "rank": 36,
                    "base_regressor_score": {
                        "status": "computed",
                        "condition_number": 68.4,
                        "base_parameter_count": 36,
                    },
                    "regressor_score": {
                        "status": "computed",
                        "effective_condition_number": 77.5,
                        "rank": 36,
                    },
                    "oed_quality_gate": {
                        "status": "fail",
                        "reasons": ["optimizer_not_converged"],
                    },
                },
                "safety": {"allowed": True},
                "artifacts": {"execution_trajectory": str(execution)},
            }
        ),
        encoding="utf-8",
    )

    manifest = freeze_candidate(
        attempt_dir=attempt_dir,
        output_dir=tmp_path / "frozen",
    )

    assert manifest["target_condition_status"] == "pass"
    assert manifest["oed_quality_status"] == "fail"
    assert manifest["next_gate"] == "review_optimizer_convergence_offline"


def test_freeze_candidate_resolves_repo_relative_execution_artifact(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    attempt_dir = tmp_path / "runs" / "scan" / "attempt-002"
    attempt_dir.mkdir(parents=True)
    planned = attempt_dir / "planned_trajectory.csv"
    execution = attempt_dir / "execution_trajectory.csv"
    planned.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )
    execution.write_text(planned.read_text(encoding="utf-8"), encoding="utf-8")
    (attempt_dir / "manifest.json").write_text(
        json.dumps(
            {
                "trajectory_backend": {"rank": 36, "condition_number": 150.0},
                "safety": {"allowed": True},
                "artifacts": {
                    "execution_trajectory": (
                        "runs/scan/attempt-002/execution_trajectory.csv"
                    ),
                },
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.chdir(tmp_path)

    freeze_candidate(attempt_dir=attempt_dir, output_dir=tmp_path / "frozen")

    manifest = json.loads(
        (tmp_path / "frozen" / "best_candidate_manifest.json").read_text(
            encoding="utf-8"
        )
    )
    assert manifest["source_execution_trajectory"] == str(execution.resolve())


def test_oed_followup_plan_builds_replay_and_focused_scan_commands(
    tmp_path: Path,
) -> None:
    frozen_dir = tmp_path / "frozen"
    frozen_dir.mkdir()
    candidate = frozen_dir / "recommended_candidate.csv"
    candidate.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )
    manifest = frozen_dir / "best_candidate_manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema": "armctrl.x5_oed_frozen_candidate.v1",
                "recommended_candidate": str(candidate),
                "condition_number": 106.27694211039746,
                "base_regressor_condition_number": 150.83617311561034,
                "rank": 36,
                "safety_allowed": True,
            }
        ),
        encoding="utf-8",
    )
    output_dir = tmp_path / "followup"

    result = build_followup_plan(
        manifest_path=manifest,
        output_dir=output_dir,
        seeds=(2, 3, 4),
        amplitudes=(0.45, 0.50, 0.55),
    )

    saved = json.loads((output_dir / "followup_plan.json").read_text(encoding="utf-8"))
    assert result == saved
    assert result["schema"] == "armctrl.x5_oed_followup_plan.v1"
    assert result["baseline"]["condition_number"] == 150.83617311561034
    assert result["baseline"]["target_condition_metric"] == "figaroh_base_regressor"
    assert result["baseline"]["safety_allowed"] is True
    assert result["baseline"]["target_condition_status"] == "fail"
    assert result["baseline"]["next_gate"] == "continue_structural_oed_search"
    assert result["host_contract"]["heavy_oed_scan"]["allowed_hosts"] == [
        "local_wsl",
        "workstation",
    ]
    assert result["host_contract"]["heavy_oed_scan"]["disallowed_hosts"] == [
        {
            "host": "n100d",
            "reason": "memory_constrained_for_figaroh_ipopt_oed_scan",
        }
    ]
    assert result["host_contract"]["n100d_role"] == (
        "lightweight_replay_hardware_collection_postprocess_solver"
    )
    assert result["warm_start_status"] == "not_supported_by_current_figaroh_wrapper"
    assert "--candidate-trajectory" in result["commands"]["replay_plan"]
    assert str(candidate) in result["commands"]["replay_plan"]
    structural_scans = result["commands"]["structural_scans"]
    assert list(structural_scans) == [
        "A_duration2_nwps7_stack1",
        "B_duration2_nwps9_stack1",
        "C_duration3_nwps9_stack1",
        "D_duration2_nwps7_stack2",
    ]
    assert "--duration 2 " in structural_scans["A_duration2_nwps7_stack1"]
    assert "--n-wps 7 " in structural_scans["A_duration2_nwps7_stack1"]
    assert "--stack-reps 1 " in structural_scans["A_duration2_nwps7_stack1"]
    assert "--duration 2 " in structural_scans["B_duration2_nwps9_stack1"]
    assert "--n-wps 9 " in structural_scans["B_duration2_nwps9_stack1"]
    assert "--stack-reps 1 " in structural_scans["B_duration2_nwps9_stack1"]
    assert "--duration 3 " in structural_scans["C_duration3_nwps9_stack1"]
    assert "--n-wps 9 " in structural_scans["C_duration3_nwps9_stack1"]
    assert "--stack-reps 1 " in structural_scans["C_duration3_nwps9_stack1"]
    assert "--duration 2 " in structural_scans["D_duration2_nwps7_stack2"]
    assert "--n-wps 7 " in structural_scans["D_duration2_nwps7_stack2"]
    assert "--stack-reps 2 " in structural_scans["D_duration2_nwps7_stack2"]
    assert all("--seed 2 3 4" in command for command in structural_scans.values())
    assert all(
        "--amplitude 0.45 0.5 0.55" in command
        for command in structural_scans.values()
    )
    assert "focused_scan" not in result["commands"]
    assert result["scan_strategy"] == {
        "mode": "structural_parameterization_search",
        "rejected_mode": "seed_or_iteration_only_lottery",
        "combos": [
            {"name": "A", "duration_s": 2.0, "n_wps": 7, "stack_reps": 1},
            {"name": "B", "duration_s": 2.0, "n_wps": 9, "stack_reps": 1},
            {"name": "C", "duration_s": 3.0, "n_wps": 9, "stack_reps": 1},
            {"name": "D", "duration_s": 2.0, "n_wps": 7, "stack_reps": 2},
        ],
        "amplitude_rad": [0.45, 0.5, 0.55],
        "seed": [2, 3, 4],
        "target_acceleration_rad_s2": 20.0,
    }
    assert result["acceptance"]["target_condition_number"] == 100.0
    assert result["acceptance"]["target_condition_metric"] == "figaroh_base_regressor"
