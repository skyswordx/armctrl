import json
from pathlib import Path

import yaml

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
        ipopt_max_iterations=300,
        condition_number_threshold=500.0,
    )

    result = OedScanRunner().run(request)

    summary_path = tmp_path / "oed_scan_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert result["schema"] == "armctrl.sysid_oed_scan.v1"
    assert summary == result
    assert len(result["attempts"]) == 2
    first = result["attempts"][0]
    assert first["status"] == "ok"
    assert first["safety_allowed"] is True
    assert first["timing_contract"]["execution_sample_hz"] == 20.0
    assert first["oed_quality_gate"]["condition_number_threshold"] == 500.0

    safe_config = yaml.safe_load(
        Path(first["safe_config"]).read_text(encoding="utf-8")
    )
    assert safe_config["safety"]["sysid"]["oed"] == {
        "n_wps": 5,
        "stack_reps": 1,
        "ipopt_max_iterations": 300,
        "condition_number_threshold": 500.0,
    }
    assert result["best_attempt"]["attempt_id"] in {
        attempt["attempt_id"] for attempt in result["attempts"]
    }


def test_oed_scan_records_trajectory_command_fault_and_continues(
    tmp_path: Path,
) -> None:
    class FaultingPlanner:
        def write_plan(self, _request):
            raise TrajectoryCommandError(
                {
                    "exit_code": 1,
                    "stdout_json": {"status": "failed", "reason": "restoration_failed"},
                    "optimizer_convergence": {
                        "status": "fail",
                        "reason": "restoration_failed",
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
        ipopt_max_iterations=300,
        condition_number_threshold=500.0,
        trajectory_command_argv=("python", "scripts/x5_figaroh_oed.py"),
    )

    result = OedScanRunner(planner=FaultingPlanner()).run(request)

    assert len(result["attempts"]) == 2
    assert all(attempt["status"] == "faulted" for attempt in result["attempts"])
    assert result["best_attempt"] is None
    assert result["attempts"][0]["error"]["code"] == "trajectory_command_failed"
    assert result["attempts"][0]["error"]["detail"]["optimizer_convergence"][
        "reason"
    ] == "restoration_failed"
