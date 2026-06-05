import csv
import json
import os
from pathlib import Path

import numpy as np
import yaml

from armctrl.x5_figaroh_oed import (
    _apply_request_limits_to_robot_model,
    _active_joint_indices,
    _write_figaroh_config,
    main,
    run_oed,
)


def _write_request(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "schema": "armctrl.figaroh_optimal_trajectory_request.v1",
                "profile": "fourier_multisine",
                "model": {
                    "urdf_path": "configs/models/X5_camera.urdf",
                    "dof": 6,
                    "active_joints": [
                        "joint_1",
                        "joint_2",
                        "joint_3",
                        "joint_4",
                        "joint_5",
                        "joint_6",
                    ],
                },
                "sampling": {
                    "sample_hz": 100.0,
                    "duration_s": 2.0,
                    "sample_count": 201,
                },
                "constraints": {
                    "safe_config_path": "configs/x5.safe.yaml",
                    "joint_limits_rad": [
                        [-0.05, 0.05],
                        [0.25, 0.35],
                        [0.25, 0.35],
                        [-0.05, 0.05],
                        [-0.05, 0.05],
                        [-0.05, 0.05],
                    ],
                    "velocity_limits_rad_s": [[-0.2, 0.2] for _ in range(6)],
                    "effort_limits_nm": [
                        [-100.0, 100.0],
                        [-100.0, 100.0],
                        [-30.0, 30.0],
                        [-100.0, 100.0],
                        [-100.0, 100.0],
                        [-100.0, 100.0],
                    ],
                },
                "figaroh": {
                    "timing": {
                        "n_wps": 5,
                        "stack_reps": 1,
                        "segment_duration_s": 2.0,
                        "effective_duration_s": 2.0,
                    },
                },
            }
        ),
        encoding="utf-8",
    )


def test_run_oed_writes_candidate_csv_from_injected_optimizer(tmp_path: Path) -> None:
    request_path = tmp_path / "figaroh_request.json"
    candidate_path = tmp_path / "candidate.csv"
    _write_request(request_path)

    class FakeOptimizer:
        def solve(self, stack_reps: int = 2):
            assert stack_reps == 1
            return {
                "T_F": [np.asarray([0.0, 0.01])],
                "P_F": [
                    np.asarray(
                        [
                            [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
                            [0.01, 0.31, 0.32, 0.0, 0.0, 0.0],
                        ]
                    )
                ],
                "final_regressor_shape": (12, 36),
            }

    result = run_oed(
        request_path=request_path,
        candidate_path=candidate_path,
        optimizer_factory=lambda _request: FakeOptimizer(),
    )

    with candidate_path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    assert result["status"] == "ok"
    assert result["schema"] == "armctrl.x5_figaroh_oed_result.v1"
    assert result["source_backend"] == "figaroh"
    assert result["candidate_trajectory"] == str(candidate_path)
    assert rows[1]["q_cmd_1"] == "0.010000"
    assert rows[1]["q_cmd_3"] == "0.320000"
    assert result["stack_reps"] == 1


def test_main_rejects_missing_oed_environment(monkeypatch, capsys) -> None:
    monkeypatch.delenv("ARMCTRL_FIGAROH_REQUEST", raising=False)
    monkeypatch.delenv("ARMCTRL_CANDIDATE_TRAJECTORY", raising=False)

    exit_code = main([])
    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 2
    assert payload["status"] == "failed"
    assert payload["reason"] == "missing_environment"


def test_figaroh_config_uses_legacy_required_sections(tmp_path: Path) -> None:
    request_path = tmp_path / "figaroh_request.json"
    candidate_path = tmp_path / "candidate.csv"
    _write_request(request_path)
    request = json.loads(request_path.read_text(encoding="utf-8"))

    config_path = _write_figaroh_config(request, candidate_path=candidate_path)
    config_text = config_path.read_text(encoding="utf-8")

    assert "robot_params:" in config_text
    assert "problem_params:" in config_text
    assert "processing_params:" in config_text
    assert "tls_params:" in config_text
    assert "trajectory_params:" in config_text
    assert "q_lim_def:" in config_text
    assert "is_joint_torques: true" in config_text


def test_figaroh_config_uses_armctrl_safety_limits(tmp_path: Path) -> None:
    request_path = tmp_path / "figaroh_request.json"
    candidate_path = tmp_path / "candidate.csv"
    _write_request(request_path)
    request = json.loads(request_path.read_text(encoding="utf-8"))

    config_path = _write_figaroh_config(request, candidate_path=candidate_path)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    robot_params = config["identification"]["robot_params"][0]

    assert robot_params["q_lim_def"][0] == [-0.05, 0.05]
    assert robot_params["q_lim_def"][1] == [0.25, 0.35]
    assert robot_params["dq_lim_def"][0] == [-0.2, 0.2]
    assert robot_params["dq_lim_def"][5] == [-0.2, 0.2]
    trajectory_params = config["identification"]["trajectory_params"][0]
    assert trajectory_params["n_wps"] == 5
    assert trajectory_params["t_s"] == 0.5


def test_run_oed_uses_request_stack_reps_and_offsets_segment_times(
    tmp_path: Path,
) -> None:
    request_path = tmp_path / "figaroh_request.json"
    candidate_path = tmp_path / "candidate.csv"
    _write_request(request_path)
    request = json.loads(request_path.read_text(encoding="utf-8"))
    request["figaroh"]["timing"]["stack_reps"] = 2
    request_path.write_text(json.dumps(request), encoding="utf-8")

    class FakeOptimizer:
        def solve(self, stack_reps: int = 2):
            assert stack_reps == 2
            return {
                "T_F": [np.asarray([0.0, 0.5]), np.asarray([0.0, 0.5])],
                "P_F": [
                    np.asarray(
                        [
                            [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
                            [0.01, 0.31, 0.31, 0.0, 0.0, 0.0],
                        ]
                    ),
                    np.asarray(
                        [
                            [0.01, 0.31, 0.31, 0.0, 0.0, 0.0],
                            [0.02, 0.32, 0.32, 0.0, 0.0, 0.0],
                        ]
                    ),
                ],
            }

    run_oed(
        request_path=request_path,
        candidate_path=candidate_path,
        optimizer_factory=lambda _request: FakeOptimizer(),
    )

    with candidate_path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    assert [row["time_s"] for row in rows] == [
        "0.000000",
        "0.500000",
        "1.000000",
    ]


def test_apply_request_limits_to_robot_model_updates_pinocchio_arrays() -> None:
    class FakeModel:
        names = ["universe", "joint1", "joint2", "joint3"]
        idx_qs = [0, 0, 1, 2]
        idx_vs = [0, 0, 1, 2]
        lowerPositionLimit = np.asarray([-10.0, 0.0, 0.0])
        upperPositionLimit = np.asarray([10.0, 3.14, 3.14])
        velocityLimit = np.asarray([1000.0, 1000.0, 10.0])
        effortLimit = np.asarray([100.0, 100.0, 30.0])

        def getJointId(self, name: str) -> int:
            return self.names.index(name)

    request = {
        "model": {"active_joints": ["joint1", "joint2", "joint3"]},
        "constraints": {
            "joint_limits_rad": [[-0.5, 0.5], [0.2, 0.8], [0.3, 0.9]],
            "velocity_limits_rad_s": [[-0.2, 0.2], [-0.3, 0.3], [-0.4, 0.4]],
            "effort_limits_nm": [[-80.0, 80.0], [-70.0, 70.0], [-20.0, 20.0]],
        },
    }

    _apply_request_limits_to_robot_model(FakeModel(), request)

    assert FakeModel.lowerPositionLimit.tolist() == [-0.5, 0.2, 0.3]
    assert FakeModel.upperPositionLimit.tolist() == [0.5, 0.8, 0.9]
    assert FakeModel.velocityLimit.tolist() == [0.2, 0.3, 0.4]
    assert FakeModel.effortLimit.tolist() == [80.0, 70.0, 20.0]


def test_active_joint_indices_follow_pinocchio_model_index_arrays() -> None:
    class FakeModel:
        names = ["universe", "joint1", "joint2", "joint3"]
        idx_qs = [0, 0, 1, 2]
        idx_vs = [0, 0, 1, 2]

        def getJointId(self, name: str) -> int:
            return self.names.index(name)

    idx_q, idx_v = _active_joint_indices(
        FakeModel(),
        ["joint1", "joint2", "joint3"],
    )

    assert idx_q == [0, 1, 2]
    assert idx_v == [0, 1, 2]
