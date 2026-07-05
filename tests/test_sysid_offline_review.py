import csv
import json
import subprocess
import sys
from pathlib import Path

import yaml

from armctrl.sysid_review import SysIdOfflineReviewRequest, SysIdOfflineReviewer


def _write_execution_trajectory(path: Path, *, unsafe: bool = False) -> None:
    rows = [
        [0.00, 0.00, 0.30, 0.30, 0.00, 0.00, 0.00],
        [0.01, 0.006, 0.306, 0.294, 0.006, -0.006, 0.006],
        [0.02, 0.012, 0.312, 0.288, 0.012, -0.012, 0.012],
        [0.03, 0.018, 0.318, 0.282, 0.018, -0.018, 0.018],
    ]
    if unsafe:
        rows[-1][2] = 3.0
        rows[-1][3] = 1.0
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "time_s",
                "q_cmd_1",
                "q_cmd_2",
                "q_cmd_3",
                "q_cmd_4",
                "q_cmd_5",
                "q_cmd_6",
            ]
        )
        writer.writerows(rows)


def _write_candidate_plan(plan_dir: Path, *, unsafe: bool = False) -> None:
    plan_dir.mkdir(parents=True)
    trajectory_path = plan_dir / "execution_trajectory.csv"
    _write_execution_trajectory(trajectory_path, unsafe=unsafe)
    manifest = {
        "schema": "armctrl.ident_plan_manifest.v1",
        "profile": {"name": "fourier_multisine"},
        "request": {
            "dof": 6,
            "sample_hz": 100.0,
            "duration_s": 0.03,
            "amplitude_rad": 0.50,
            "q_center": [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
            "urdf_path": "configs/models/X5_camera.urdf",
            "safe_config_path": "configs/x5.safe.yaml",
        },
        "safety": {
            "allowed": not unsafe,
            "reason": "plan-only sysid preview",
            "checks": {
                "trajectory_step_check": {"status": "pass", "violations": []},
                "trajectory_velocity_check": {"status": "pass", "violations": []},
                "trajectory_acceleration_check": {"status": "pass", "violations": []},
                "workspace_clearance_check": {"status": "pass", "violations": []},
            },
        },
        "trajectory_backend": {
            "condition_metric": "figaroh_base_regressor",
            "condition_number": 68.43,
            "figaroh_base_condition_number": 68.43,
            "pinocchio_effective_condition_number": 77.56,
            "rank": 36,
            "oed_quality_gate": {
                "status": "fail",
                "reasons": ["optimizer_not_converged"],
                "primary_metric": "figaroh_base_regressor",
            },
            "candidate_source": {
                "generated_by": {
                    "optimizer_convergence": {
                        "status": "fail",
                        "reason": "max_iterations_exceeded",
                        "constraint_violation_unscaled": 0.0,
                    }
                }
            },
        },
        "artifacts": {"execution_trajectory": str(trajectory_path)},
    }
    (plan_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _write_frozen_candidate(plan_dir: Path) -> None:
    plan_dir.mkdir(parents=True)
    recommended_path = plan_dir / "recommended_candidate.csv"
    execution_path = plan_dir / "execution_trajectory.csv"
    _write_execution_trajectory(recommended_path, unsafe=True)
    _write_execution_trajectory(execution_path)
    manifest = {
        "schema": "armctrl.x5_oed_frozen_candidate.v1",
        "recommended_candidate": str(recommended_path),
        "source_execution_trajectory": str(execution_path),
        "condition_number": 68.43,
        "base_regressor_condition_number": 68.43,
        "pinocchio_effective_condition_number": 77.56,
        "rank": 36,
        "oed_quality_status": "fail",
        "safety_allowed": True,
        "target_condition_metric": "figaroh_base_regressor",
        "target_condition_status": "pass",
        "next_gate": "review_optimizer_convergence_offline",
    }
    (plan_dir / "best_candidate_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def test_offline_review_writes_visual_artifacts_and_keeps_hardware_execution_blocked(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "candidate"
    output_dir = tmp_path / "review"
    _write_candidate_plan(plan_dir)

    result = SysIdOfflineReviewer().review(
        SysIdOfflineReviewRequest(
            plan_dir=plan_dir,
            output_dir=output_dir,
            urdf_path=Path("configs/models/X5_camera.urdf"),
            safe_config_path=Path("configs/x5.safe.yaml"),
        )
    )

    assert result["schema"] == "armctrl.sysid_offline_review.v1"
    assert result["movement_allowed"] is False
    assert result["hardware_execution_eligible"] is False
    assert result["gate_state"] == "hardware_smoke_plan_ready"
    assert result["condition"]["metric"] == "figaroh_base_regressor"
    assert result["condition"]["figaroh_base_condition_number"] == 68.43
    assert result["optimizer"]["reason"] == "max_iterations_exceeded"
    assert result["smoothness"]["status"] == "pass"
    assert result["clearance"]["status"] == "pass"
    assert result["execution_gate"]["status"] == "pass"
    assert Path(result["artifacts"]["q_plot"]).exists()
    assert Path(result["artifacts"]["dq_plot"]).exists()
    assert Path(result["artifacts"]["ddq_plot"]).exists()
    assert Path(result["artifacts"]["preview_html"]).exists()
    assert Path(result["artifacts"]["hardware_smoke_plan"]).exists()
    assert "hardware_execution_eligible: false" in (
        output_dir / "hardware_smoke_plan.md"
    ).read_text(encoding="utf-8")
    hardware_plan = (output_dir / "hardware_smoke_plan.md").read_text(
        encoding="utf-8"
    )
    assert "armctrl sysid compile-runtime" in hardware_plan
    assert "armctrl motion submit joint-trajectory" in hardware_plan
    assert "sysid run gravity_sweep --adapter sdk" not in hardware_plan
    assert "sysid run friction_sweep --adapter sdk" not in hardware_plan
    assert "sysid run fourier_multisine --adapter sdk" not in hardware_plan
    assert "--adapter sdk" not in hardware_plan
    assert "FIGAROH base condition" in (
        output_dir / "review_report.md"
    ).read_text(encoding="utf-8")


def test_offline_review_clearance_failure_blocks_hardware_smoke_plan_ready(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "candidate"
    output_dir = tmp_path / "review"
    _write_candidate_plan(plan_dir, unsafe=True)

    result = SysIdOfflineReviewer().review(
        SysIdOfflineReviewRequest(
            plan_dir=plan_dir,
            output_dir=output_dir,
            urdf_path=Path("configs/models/X5_camera.urdf"),
            safe_config_path=Path("configs/x5.safe.yaml"),
        )
    )

    assert result["gate_state"] == "offline_candidate"
    assert result["hardware_execution_eligible"] is False
    assert result["hardware_smoke_plan_allowed"] is False
    assert result["clearance"]["status"] == "fail"
    assert "clearance_failed" in result["gate_reasons"]
    assert "不允许进入 hardware smoke plan" in (
        output_dir / "review_report.md"
    ).read_text(encoding="utf-8")


def test_cli_sysid_review_candidate_is_read_only_and_outputs_artifacts(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "candidate"
    output_dir = tmp_path / "review"
    _write_candidate_plan(plan_dir)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "review-candidate",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(output_dir),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["movement_allowed"] is False
    assert payload["hardware_execution_eligible"] is False
    assert payload["gate_state"] == "hardware_smoke_plan_ready"
    assert Path(payload["artifacts"]["review_report"]).exists()
    assert Path(payload["artifacts"]["review_summary"]).exists()


def test_review_accepts_manifest_paths_relative_to_current_working_directory(
    tmp_path: Path,
    monkeypatch,
) -> None:
    plan_dir = tmp_path / "candidate"
    output_dir = tmp_path / "review"
    _write_candidate_plan(plan_dir)
    manifest_path = plan_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    cwd_relative = plan_dir / "execution_trajectory.csv"
    manifest["artifacts"]["execution_trajectory"] = str(cwd_relative)
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    monkeypatch.chdir(Path.cwd())

    result = SysIdOfflineReviewer().review(
        SysIdOfflineReviewRequest(
            plan_dir=plan_dir,
            output_dir=output_dir,
            urdf_path=Path("configs/models/X5_camera.urdf"),
            safe_config_path=Path("configs/x5.safe.yaml"),
        )
    )

    assert result["trajectory_path"] == str(cwd_relative)
    assert result["execution_gate"]["status"] == "pass"


def test_offline_review_accepts_frozen_candidate_manifest(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "frozen"
    output_dir = tmp_path / "review"
    _write_frozen_candidate(plan_dir)

    result = SysIdOfflineReviewer().review(
        SysIdOfflineReviewRequest(
            plan_dir=plan_dir,
            output_dir=output_dir,
            urdf_path=Path("configs/models/X5_camera.urdf"),
            safe_config_path=Path("configs/x5.safe.yaml"),
        )
    )

    assert result["trajectory_path"] == str(plan_dir / "execution_trajectory.csv")
    assert result["condition"]["metric"] == "figaroh_base_regressor"
    assert result["condition"]["figaroh_base_condition_number"] == 68.43
    assert result["gate_state"] == "hardware_smoke_plan_ready"
    assert result["optimizer"]["reason"] == "review_optimizer_convergence_offline"


def test_condition_below_target_does_not_bypass_optimizer_and_visual_review(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "candidate"
    output_dir = tmp_path / "review"
    _write_candidate_plan(plan_dir)
    manifest_path = plan_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["safety"]["allowed"] = False
    manifest["safety"]["checks"]["trajectory_step_check"] = {
        "status": "fail",
        "violations": [{"check": "max_joint_step_rad"}],
    }
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    result = SysIdOfflineReviewer().review(
        SysIdOfflineReviewRequest(
            plan_dir=plan_dir,
            output_dir=output_dir,
            urdf_path=Path("configs/models/X5_camera.urdf"),
            safe_config_path=Path("configs/x5.safe.yaml"),
        )
    )

    assert result["condition"]["target_status"] == "pass"
    assert result["gate_state"] == "offline_candidate"
    assert result["hardware_smoke_plan_allowed"] is False
    assert result["hardware_execution_eligible"] is False
    assert "execution_gate_failed" in result["gate_reasons"]


def test_review_uses_safe_config_override_to_detect_unsafe_execution_step(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "candidate"
    output_dir = tmp_path / "review"
    safe_config = yaml.safe_load(
        Path("configs/x5.safe.yaml").read_text(encoding="utf-8")
    )
    safe_config["safety"]["sysid"]["profile_overrides"]["fourier_multisine"][
        "max_joint_step_rad"
    ] = 0.005
    safe_config_path = tmp_path / "x5.strict.safe.yaml"
    safe_config_path.write_text(
        yaml.safe_dump(safe_config, sort_keys=False),
        encoding="utf-8",
    )
    _write_candidate_plan(plan_dir)

    result = SysIdOfflineReviewer().review(
        SysIdOfflineReviewRequest(
            plan_dir=plan_dir,
            output_dir=output_dir,
            urdf_path=Path("configs/models/X5_camera.urdf"),
            safe_config_path=safe_config_path,
        )
    )

    assert result["execution_gate"]["trajectory_step_check"]["status"] == "fail"
    assert result["gate_state"] == "offline_candidate"
    assert "execution_gate_failed" in result["gate_reasons"]
