import csv
import json
import os
from pathlib import Path

import numpy as np
import pytest
import yaml

from armctrl.x5_figaroh_oed import (
    X5ConstraintContractManager,
    X5JointRelationConstraintManager,
    X5TrajectoryIPOPTProblem,
    _apply_request_limits_to_robot_model,
    _active_joint_indices,
    _request_ipopt_max_iterations,
    _velocity_feasible_initial_waypoints,
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
                    "optimizer": {
                        "ipopt_max_iterations": 200,
                        "ipopt_print_level": 5,
                    },
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
        idx_b = list(range(36))
        last_base_regressor_condition_number = 42.0
        last_base_regressor_shape = (12, 36)

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
    assert result["base_regressor_score"] == {
        "status": "computed",
        "condition_number": 42.0,
        "row_count": 12,
        "column_count": 36,
        "base_parameter_count": 36,
    }


def test_velocity_feasible_initial_waypoints_limit_adjacent_delta() -> None:
    waypoints = _velocity_feasible_initial_waypoints(
        wp_init=[0.0, 0.3],
        joint_limits_rad=[[-0.5, 0.5], [0.0, 0.6]],
        velocity_limits_rad_s=[[-1.0, 1.0], [-0.8, 0.8]],
        waypoint_duration_s=0.25,
        n_wps=7,
    )

    assert waypoints.shape == (2, 7)
    np.testing.assert_allclose(waypoints[:, 0], [0.0, 0.3])
    max_delta = np.max(np.abs(np.diff(waypoints, axis=1)), axis=1)
    np.testing.assert_allclose(max_delta <= np.asarray([0.1125, 0.09]) + 1e-12, True)
    assert np.all(waypoints[0] >= -0.5)
    assert np.all(waypoints[0] <= 0.5)
    assert np.all(waypoints[1] >= 0.0)
    assert np.all(waypoints[1] <= 0.6)


def test_run_oed_reports_optimizer_initialization_strategy(tmp_path: Path) -> None:
    request_path = tmp_path / "figaroh_request.json"
    candidate_path = tmp_path / "candidate.csv"
    _write_request(request_path)

    class FakeOptimizer:
        last_initialization_strategy = "armctrl_velocity_feasible"

        def solve(self, stack_reps: int = 1):
            return {
                "T_F": [np.asarray([0.0, 0.01])],
                "P_F": [
                    np.asarray(
                        [
                            [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
                            [0.01, 0.31, 0.31, 0.0, 0.0, 0.0],
                        ]
                    )
                ],
            }

    result = run_oed(
        request_path=request_path,
        candidate_path=candidate_path,
        optimizer_factory=lambda _request: FakeOptimizer(),
    )

    assert result["initialization"] == {
        "strategy": "armctrl_velocity_feasible",
    }


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


def test_figaroh_config_uses_armctrl_oed_knobs(tmp_path: Path) -> None:
    request_path = tmp_path / "figaroh_request.json"
    candidate_path = tmp_path / "candidate.csv"
    _write_request(request_path)
    request = json.loads(request_path.read_text(encoding="utf-8"))
    request["figaroh"]["timing"].update(
        {
            "n_wps": 7,
            "stack_reps": 3,
            "waypoint_duration_s": 0.25,
            "segment_duration_s": 1.5,
        }
    )
    request["figaroh"]["optimizer"] = {
        "ipopt_max_iterations": 900,
        "ipopt_print_level": 7,
    }

    config_path = _write_figaroh_config(request, candidate_path=candidate_path)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    trajectory_params = config["identification"]["trajectory_params"][0]
    assert trajectory_params["n_wps"] == 7
    assert trajectory_params["freq"] == 100.0
    assert trajectory_params["t_s"] == 0.25
    assert trajectory_params["ipopt_max_iterations"] == 900
    assert trajectory_params["ipopt_print_level"] == 7
    assert _request_ipopt_max_iterations(request) == 900


def test_run_oed_seeds_numpy_from_request_optimizer(
    tmp_path: Path,
    monkeypatch,
) -> None:
    request_path = tmp_path / "figaroh_request.json"
    candidate_path = tmp_path / "candidate.csv"
    _write_request(request_path)
    request = json.loads(request_path.read_text(encoding="utf-8"))
    request["figaroh"]["optimizer"] = {
        "ipopt_max_iterations": 900,
        "random_seed": 42,
    }
    request_path.write_text(json.dumps(request), encoding="utf-8")
    captured = {}
    monkeypatch.setattr(
        "armctrl.x5_figaroh_oed.np.random.seed",
        lambda seed: captured.setdefault("seed", seed),
    )

    class FakeOptimizer:
        def solve(self, stack_reps: int = 1):
            return {
                "T_F": [np.asarray([0.0, 0.01])],
                "P_F": [
                    np.asarray(
                        [
                            [0.0, 0.3, 0.3, 0.0, 0.0, 0.0],
                            [0.01, 0.31, 0.31, 0.0, 0.0, 0.0],
                        ]
                    )
                ],
            }

    run_oed(
        request_path=request_path,
        candidate_path=candidate_path,
        optimizer_factory=lambda _request: FakeOptimizer(),
    )

    assert captured["seed"] == 42


def test_x5_ipopt_problem_applies_request_max_iterations(monkeypatch) -> None:
    captured = {}

    class FakeConfig:
        def __init__(self) -> None:
            self.tolerance = None
            self.acceptable_tolerance = None
            self.max_iterations = None
            self.print_level = None
            self.custom_options = {}

        @classmethod
        def for_trajectory_optimization(cls):
            return cls()

    class FakeSolver:
        def __init__(self, _problem, config) -> None:
            captured["max_iterations"] = config.max_iterations
            captured["print_level"] = config.print_level

        def solve(self):
            return False, {"status": "forced_stop"}

    monkeypatch.setattr("armctrl.x5_figaroh_oed.IPOPTConfig", FakeConfig)
    monkeypatch.setattr("armctrl.x5_figaroh_oed.RobotIPOPTSolver", FakeSolver)
    problem = object.__new__(X5TrajectoryIPOPTProblem)
    problem.ipopt_max_iterations = 900
    problem.ipopt_print_level = 7
    problem.logger = type(
        "Logger",
        (),
        {"error": lambda self, _message: None},
    )()

    success, result = problem.solve_with_waypoints([[0.0]])

    assert success is False
    assert result["status"] == "forced_stop"
    assert captured["max_iterations"] == 900
    assert captured["print_level"] == 7


def test_x5_ipopt_problem_returns_cyipopt_dense_jacobian_buffer() -> None:
    class FakeFigarohProblem:
        def jacobian(self, _x):
            return np.asarray(
                [
                    [1.0, 2.0, 3.0],
                    [4.0, 5.0, 6.0],
                ]
            )

    class Problem(X5TrajectoryIPOPTProblem, FakeFigarohProblem):
        pass

    jacobian = Problem().jacobian(np.asarray([0.0, 0.1, 0.2]))

    assert jacobian.shape == (6,)
    assert jacobian.flags.c_contiguous
    assert jacobian.tolist() == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_x5_constraint_contract_manager_repairs_missing_waypoint_positions() -> None:
    class FakeCB:
        act_idxq = [0, 1]
        lower_q = [-1.0, 0.0]
        upper_q = [1.0, 1.0]

    class FakeBaseManager:
        CB = FakeCB()
        n_wps = 3
        freq = 20.0

        def get_variable_bounds(self):
            return [-1.0, 0.0, -1.0, 0.0], [1.0, 1.0, 1.0, 1.0]

        def get_constraint_bounds(self, _ns):
            return (
                [-1.0, 0.0, -1.0, 0.0, -0.5],
                [1.0, 1.0, 1.0, 1.0, 0.5],
            )

        def evaluate_constraints(
            self,
            _ns,
            _x,
            _opt_cb,
            _tps,
            _vel_wps,
            _acc_wps,
            _wp_init,
        ):
            return np.asarray([0.25])

    manager = X5ConstraintContractManager(FakeBaseManager())
    values = manager.evaluate_constraints(
        10,
        np.asarray([0.1, 0.2, 0.3, 0.4]),
        {},
        None,
        None,
        None,
        np.asarray([0.0, 0.0]),
    )

    assert values.tolist() == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.25])


def test_joint_relation_constraint_manager_appends_relation_bounds() -> None:
    class FakeCB:
        act_idxq = [0, 1, 2]
        act_idxv = [0, 1, 2]

        def get_full_config(self, _freq, _tps, _wps, _vel_wps, _acc_wps):
            return (
                np.asarray([[0.0], [0.01]]),
                np.asarray(
                    [
                        [0.0, 0.30, 0.30],
                        [0.0, 0.34, 0.29],
                    ]
                ),
                np.zeros((2, 3)),
                np.zeros((2, 3)),
            )

    class FakeBaseManager:
        CB = FakeCB()
        n_wps = 2
        freq = 100.0

        def get_variable_bounds(self):
            return [0.0], [1.0]

        def get_constraint_bounds(self, _ns):
            return [0.0], [1.0]

        def evaluate_constraints(
            self,
            _ns,
            _x,
            _opt_cb,
            _tps,
            _vel_wps,
            _acc_wps,
            _wp_init,
        ):
            return np.asarray([0.5])

    manager = X5JointRelationConstraintManager(
        FakeBaseManager(),
        [
            {
                "name": "x5_joint2_joint3_parallel_band",
                "left_joint": 2,
                "right_joint": 3,
                "min_delta_rad": -0.08,
                "max_delta_rad": 0.08,
            }
        ],
    )

    lower, upper = manager.get_constraint_bounds(2)
    values = manager.evaluate_constraints(
        2,
        np.asarray([0.0, 0.0, 0.0]),
        {},
        None,
        None,
        None,
        np.asarray([0.0, 0.30, 0.30]),
    )

    assert lower == [0.0, -0.08, -0.08]
    assert upper == [1.0, 0.08, 0.08]
    assert values.tolist() == pytest.approx([0.5, 0.0, 0.05])


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
