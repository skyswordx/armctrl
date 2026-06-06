import json
from pathlib import Path

import pytest
import yaml

from scripts.x5_oed_freeze_candidate import freeze_candidate
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
    assert manifest["condition_number"] == 106.27694211039746
    assert manifest["base_regressor_condition_number"] == 150.83617311561034
    assert manifest["pinocchio_effective_condition_number"] == 106.27694211039746
    assert manifest["rank"] == 36
    assert manifest["safety_allowed"] is True
    assert manifest["replay_hint"]["candidate_trajectory"] == str(recommended)


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
