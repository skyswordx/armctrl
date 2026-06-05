from __future__ import annotations

import csv
import json
import os
from pathlib import Path
import sys
from typing import Any, Callable, Sequence

import numpy as np


OptimizerFactory = Callable[[dict[str, Any]], Any]
IPOPTConfig: Any | None = None
RobotIPOPTSolver: Any | None = None


class FigarohOedError(RuntimeError):
    pass


class X5TrajectoryIPOPTProblem:
    """Mixin that keeps FIGAROH's problem math but lets armctrl set IPOPT knobs."""

    ipopt_max_iterations = 200

    def solve_with_waypoints(self, wps):
        try:
            self._initial_wps = wps
            if IPOPTConfig is None or RobotIPOPTSolver is None:
                raise FigarohOedError("FIGAROH IPOPT solver classes are not loaded")
            config = IPOPTConfig.for_trajectory_optimization()
            config.tolerance = 1e-3
            config.acceptable_tolerance = 1e-2
            config.max_iterations = int(self.ipopt_max_iterations)
            config.print_level = 3
            config.custom_options = {
                b"mu_strategy": b"adaptive",
            }
            solver = RobotIPOPTSolver(self, config)
            success, results = solver.solve()
            if not success:
                self.logger.error("Optimization failed")
                return False, results

            X_opt = results["x_opt"]
            wps_X = np.reshape(np.array(X_opt), (self.n_wps - 1, self.n_joints))
            final_waypoint = wps_X[-1, :]
            results.update(
                {
                    "t_f": self.opt_cb["t_f"],
                    "p_f": self.opt_cb["p_f"],
                    "v_f": self.opt_cb["v_f"],
                    "a_f": self.opt_cb["a_f"],
                    "iter_data": {
                        "iterations": self.iteration_data["iterations"],
                        "obj_values": self.iteration_data["obj_values"],
                        "solve_time": results["solve_time"],
                        "status": results["status"],
                        "final_waypoint": final_waypoint,
                    },
                }
            )
            return True, results
        except Exception as exc:
            self.logger.error(f"Error in IPOPT solve: {exc}")
            return False, {"error": str(exc)}


class X5JointRelationConstraintManager:
    """Append X5-specific joint relation constraints to FIGAROH constraints."""

    def __init__(
        self,
        base_manager: Any,
        relation_constraints: list[dict[str, Any]],
    ) -> None:
        self._base_manager = base_manager
        self._relation_constraints = relation_constraints
        self.CB = base_manager.CB
        self.n_wps = base_manager.n_wps
        self.freq = base_manager.freq

    def get_variable_bounds(self):
        return self._base_manager.get_variable_bounds()

    def get_constraint_bounds(self, Ns: int):
        lower, upper = self._base_manager.get_constraint_bounds(Ns)
        for _sample_index in range(Ns):
            for constraint in self._relation_constraints:
                lower.append(float(constraint["min_delta_rad"]))
                upper.append(float(constraint["max_delta_rad"]))
        return lower, upper

    def evaluate_constraints(
        self,
        Ns: int,
        X: np.ndarray,
        opt_cb: dict[str, Any],
        tps,
        vel_wps,
        acc_wps,
        wp_init,
    ) -> np.ndarray:
        base_constraints = self._base_manager.evaluate_constraints(
            Ns,
            X,
            opt_cb,
            tps,
            vel_wps,
            acc_wps,
            wp_init,
        )
        relation_values = self._evaluate_joint_relation_constraints(
            X,
            tps,
            vel_wps,
            acc_wps,
            wp_init,
        )
        return np.concatenate((base_constraints, relation_values), axis=None)

    def _evaluate_joint_relation_constraints(
        self,
        X: np.ndarray,
        tps,
        vel_wps,
        acc_wps,
        wp_init,
    ) -> np.ndarray:
        X = np.asarray(X)
        wps_X = np.reshape(X, (self.n_wps - 1, len(self.CB.act_idxq)))
        wps = np.vstack((wp_init, wps_X)).transpose()
        _t_f, p_f, _v_f, _a_f = self.CB.get_full_config(
            self.freq,
            tps,
            wps,
            vel_wps,
            acc_wps,
        )
        values: list[float] = []
        for row in p_f:
            for constraint in self._relation_constraints:
                left_active_index = int(constraint["left_joint"]) - 1
                right_active_index = int(constraint["right_joint"]) - 1
                left_q_index = int(self.CB.act_idxq[left_active_index])
                right_q_index = int(self.CB.act_idxq[right_active_index])
                values.append(float(row[left_q_index]) - float(row[right_q_index]))
        return np.asarray(values, dtype=float)


def run_oed(
    *,
    request_path: Path,
    candidate_path: Path,
    optimizer_factory: OptimizerFactory | None = None,
) -> dict[str, Any]:
    request = json.loads(request_path.read_text(encoding="utf-8"))
    _validate_request(request)
    optimizer = (
        optimizer_factory(request)
        if optimizer_factory is not None
        else _build_figaroh_optimizer(request, candidate_path=candidate_path)
    )
    stack_reps = _request_stack_reps(request)
    results = optimizer.solve(stack_reps=stack_reps)
    rows = _rows_from_figaroh_results(results, dof=int(request["model"]["dof"]))
    _write_candidate_rows(candidate_path, rows)
    return {
        "schema": "armctrl.x5_figaroh_oed_result.v1",
        "status": "ok",
        "source_backend": "figaroh",
        "request": str(request_path),
        "candidate_trajectory": str(candidate_path),
        "sample_count": len(rows),
        "stack_reps": stack_reps,
        "final_regressor_shape": _jsonable_shape(results.get("final_regressor_shape")),
    }


def main(argv: Sequence[str] | None = None) -> int:
    _ = argv
    request_text = os.environ.get("ARMCTRL_FIGAROH_REQUEST")
    candidate_text = os.environ.get("ARMCTRL_CANDIDATE_TRAJECTORY")
    if not request_text or not candidate_text:
        _emit(
            {
                "schema": "armctrl.x5_figaroh_oed_result.v1",
                "status": "failed",
                "reason": "missing_environment",
                "required_environment": [
                    "ARMCTRL_FIGAROH_REQUEST",
                    "ARMCTRL_CANDIDATE_TRAJECTORY",
                ],
            }
        )
        return 2
    try:
        result = run_oed(
            request_path=Path(request_text),
            candidate_path=Path(candidate_text),
        )
    except Exception as exc:
        _emit(
            {
                "schema": "armctrl.x5_figaroh_oed_result.v1",
                "status": "failed",
                "reason": "figaroh_oed_failed",
                "message": str(exc),
            }
        )
        return 1
    _emit(result)
    return 0


def _build_figaroh_optimizer(
    request: dict[str, Any],
    *,
    candidate_path: Path,
) -> Any:
    global IPOPTConfig, RobotIPOPTSolver
    _ensure_vendor_figaroh_on_path()
    try:
        from figaroh.optimal.base_optimal_trajectory import (
            BaseOptimalTrajectory,
            BaseTrajectoryIPOPTProblem,
        )
        from figaroh.tools.robotipopt import (
            IPOPTConfig as FigarohIPOPTConfig,
            RobotIPOPTSolver as FigarohRobotIPOPTSolver,
        )
        from figaroh.tools.load_robot import load_robot
    except Exception as exc:  # pragma: no cover - environment dependent.
        raise FigarohOedError(
            "FIGAROH optimal trajectory modules are not importable: "
            f"{type(exc).__name__}: {exc}"
        ) from exc
    IPOPTConfig = FigarohIPOPTConfig
    RobotIPOPTSolver = FigarohRobotIPOPTSolver
    x5_problem_cls = type(
        "X5FigarohTrajectoryIPOPTProblem",
        (X5TrajectoryIPOPTProblem, BaseTrajectoryIPOPTProblem),
        {},
    )

    class X5OptimalTrajectory(BaseOptimalTrajectory):
        def create_ipopt_problem(
            self,
            n_joints,
            n_wps,
            Ns,
            tps,
            vel_wps,
            acc_wps,
            wp_init,
            vel_wp_init,
            acc_wp_init,
            W_stack,
        ):
            problem = x5_problem_cls(
                self,
                n_joints,
                n_wps,
                Ns,
                tps,
                vel_wps,
                acc_wps,
                wp_init,
                vel_wp_init,
                acc_wp_init,
                W_stack,
                problem_name="X5TrajectoryOptimization",
            )
            problem.ipopt_max_iterations = _request_ipopt_max_iterations(request)
            return problem

    config_path = _write_figaroh_config(request, candidate_path=candidate_path)
    urdf_path = _resolve_path(str(request["model"]["urdf_path"]))
    try:
        robot = load_robot(str(urdf_path), package_dirs=str(urdf_path.parent))
        _apply_request_limits_to_robot_model(robot.model, request)
        optimizer = X5OptimalTrajectory(
            robot,
            active_joints=list(request["model"]["active_joints"]),
            config_file=str(config_path),
        )
        idx_q, idx_v = _active_joint_indices(
            robot.model,
            list(request["model"]["active_joints"]),
        )
        optimizer.identif_config["active_joints"] = list(request["model"]["active_joints"])
        optimizer.identif_config["act_idxq"] = idx_q
        optimizer.identif_config["act_idxv"] = idx_v
        optimizer.initialize()
        relation_constraints = _request_joint_relation_constraints(request)
        if relation_constraints:
            optimizer.constraint_manager = X5JointRelationConstraintManager(
                optimizer.constraint_manager,
                relation_constraints,
            )
        return optimizer
    except Exception as exc:  # pragma: no cover - real FIGAROH integration path.
        raise FigarohOedError(
            "failed to construct X5 FIGAROH optimizer from URDF/config: "
            f"{type(exc).__name__}: {exc}"
        ) from exc


def _write_figaroh_config(
    request: dict[str, Any],
    *,
    candidate_path: Path,
) -> Path:
    config_path = candidate_path.with_name("x5_figaroh_oed_config.yaml")
    sample_hz = float(request["sampling"]["sample_hz"])
    duration_s = float(request["sampling"]["duration_s"])
    dof = int(request["model"]["dof"])
    motion_limits = _request_motion_limits(request, dof=dof)
    timing = _request_timing(request, sample_hz=sample_hz, duration_s=duration_s)
    config = {
        "identification": {
            "active_joints": list(request["model"]["active_joints"]),
            "robot_params": [
                {
                    "q_lim_def": motion_limits["joint_limits_rad"],
                    "dq_lim_def": motion_limits["velocity_limits_rad_s"],
                    "effort_lim_def": motion_limits["effort_limits_nm"],
                    "fv": [0.0 for _ in range(dof)],
                    "fs": [0.0 for _ in range(dof)],
                    "Ia": [0.0 for _ in range(dof)],
                    "offset": [0.0 for _ in range(dof)],
                    "Iam6": 0.0,
                    "fvm6": 0.0,
                    "fsm6": 0.0,
                    "reduction_ratio": [1.0 for _ in range(dof)],
                    "ratio_essential": [1.0 for _ in range(dof)],
                }
            ],
            "problem_params": [
                {
                    "is_external_wrench": False,
                    "is_joint_torques": True,
                    "force_torque": [],
                    "external_wrench_offsets": [],
                    "has_friction": False,
                    "has_actuator_inertia": False,
                    "has_joint_offset": False,
                    "has_coupled_wrist": False,
                }
            ],
            "processing_params": [
                {
                    "ts": 1.0 / sample_hz,
                    "cut_off_frequency_butterworth": min(10.0, sample_hz / 4.0),
                }
            ],
            "tls_params": [
                {
                    "mass_load": 0.0,
                    "which_body_loaded": "",
                }
            ],
            "trajectory_params": [
                {
                    "n_wps": timing["n_wps"],
                    "freq": sample_hz,
                    "t_s": timing["waypoint_duration_s"],
                    "soft_lim": 0.05,
                    "max_attempts": 1000,
                    "ipopt_max_iterations": _request_ipopt_max_iterations(request),
                    "x5_joint_relation_constraints": _request_joint_relation_constraints(
                        request
                    ),
                }
            ],
        }
    }
    config_path.parent.mkdir(parents=True, exist_ok=True)
    _write_minimal_yaml(config_path, config)
    return config_path


def _request_stack_reps(request: dict[str, Any]) -> int:
    timing = request.get("figaroh", {}).get("timing", {})
    if not isinstance(timing, dict):
        return 1
    return max(1, int(timing.get("stack_reps", 1)))


def _request_ipopt_max_iterations(request: dict[str, Any]) -> int:
    optimizer = request.get("figaroh", {}).get("optimizer", {})
    if not isinstance(optimizer, dict):
        return 200
    return max(1, int(optimizer.get("ipopt_max_iterations", 200)))


def _request_timing(
    request: dict[str, Any],
    *,
    sample_hz: float,
    duration_s: float,
) -> dict[str, Any]:
    timing = request.get("figaroh", {}).get("timing", {})
    if not isinstance(timing, dict):
        timing = {}
    n_wps = max(2, int(timing.get("n_wps", 5)))
    stack_reps = max(1, int(timing.get("stack_reps", 1)))
    segment_duration_s = float(
        timing.get("segment_duration_s", duration_s / stack_reps)
    )
    waypoint_duration_s = float(
        timing.get("waypoint_duration_s", segment_duration_s / (n_wps - 1))
    )
    return {
        "execution_sample_hz": float(timing.get("execution_sample_hz", sample_hz)),
        "n_wps": n_wps,
        "stack_reps": stack_reps,
        "segment_duration_s": segment_duration_s,
        "waypoint_duration_s": waypoint_duration_s,
    }


def _request_motion_limits(request: dict[str, Any], *, dof: int) -> dict[str, list[list[float]]]:
    constraints = request.get("constraints", {})
    if not isinstance(constraints, dict):
        constraints = {}
    return {
        "joint_limits_rad": _limit_pairs(
            constraints.get("joint_limits_rad"),
            dof=dof,
            default=[-3.14159, 3.14159],
            label="joint_limits_rad",
        ),
        "velocity_limits_rad_s": _limit_pairs(
            constraints.get("velocity_limits_rad_s"),
            dof=dof,
            default=[-1.0, 1.0],
            label="velocity_limits_rad_s",
        ),
        "effort_limits_nm": _limit_pairs(
            constraints.get("effort_limits_nm"),
            dof=dof,
            default=[-100.0, 100.0],
            label="effort_limits_nm",
        ),
    }


def _request_joint_relation_constraints(request: dict[str, Any]) -> list[dict[str, Any]]:
    constraints = request.get("constraints", {})
    if not isinstance(constraints, dict):
        return []
    raw_value = constraints.get("joint_relation_constraints", []) or []
    if not isinstance(raw_value, list):
        raise FigarohOedError("joint_relation_constraints must be a list")
    parsed: list[dict[str, Any]] = []
    for index, raw_constraint in enumerate(raw_value):
        if not isinstance(raw_constraint, dict):
            raise FigarohOedError(
                f"joint_relation_constraints[{index}] must be a mapping"
            )
        parsed.append(
            {
                "name": str(raw_constraint.get("name", f"joint_relation_{index + 1}")),
                "left_joint": int(raw_constraint["left_joint"]),
                "right_joint": int(raw_constraint["right_joint"]),
                "min_delta_rad": float(raw_constraint["min_delta_rad"]),
                "max_delta_rad": float(raw_constraint["max_delta_rad"]),
            }
        )
    return parsed


def _limit_pairs(
    raw_value: object,
    *,
    dof: int,
    default: list[float],
    label: str,
) -> list[list[float]]:
    if raw_value is None:
        return [list(default) for _ in range(dof)]
    if not isinstance(raw_value, list) or len(raw_value) != dof:
        raise FigarohOedError(f"{label} must contain {dof} [lower, upper] pairs")
    pairs: list[list[float]] = []
    for index, raw_pair in enumerate(raw_value):
        if not isinstance(raw_pair, list) or len(raw_pair) != 2:
            raise FigarohOedError(f"{label}[{index}] must be [lower, upper]")
        lower = float(raw_pair[0])
        upper = float(raw_pair[1])
        if lower > upper:
            raise FigarohOedError(f"{label}[{index}] lower is greater than upper")
        pairs.append([lower, upper])
    return pairs


def _apply_request_limits_to_robot_model(model: Any, request: dict[str, Any]) -> None:
    active_joints = list(request["model"]["active_joints"])
    motion_limits = _request_motion_limits(request, dof=len(active_joints))
    idx_q, idx_v = _active_joint_indices(model, active_joints)
    lower_position = getattr(model, "lowerPositionLimit")
    upper_position = getattr(model, "upperPositionLimit")
    velocity_limit = getattr(model, "velocityLimit")
    effort_limit = getattr(model, "effortLimit")
    for index, (q_idx, v_idx) in enumerate(zip(idx_q, idx_v)):
        q_lower, q_upper = motion_limits["joint_limits_rad"][index]
        dq_lower, dq_upper = motion_limits["velocity_limits_rad_s"][index]
        tau_lower, tau_upper = motion_limits["effort_limits_nm"][index]
        lower_position[q_idx] = q_lower
        upper_position[q_idx] = q_upper
        velocity_limit[v_idx] = max(abs(dq_lower), abs(dq_upper))
        effort_limit[v_idx] = max(abs(tau_lower), abs(tau_upper))


def _active_joint_indices(model: Any, active_joints: list[str]) -> tuple[list[int], list[int]]:
    idx_q: list[int] = []
    idx_v: list[int] = []
    for joint_name in active_joints:
        resolved_name = _resolve_model_joint_name(model, joint_name)
        joint_id = int(model.getJointId(resolved_name))
        idx_q.append(int(model.idx_qs[joint_id]))
        idx_v.append(int(model.idx_vs[joint_id]))
    return idx_q, idx_v


def _resolve_model_joint_name(model: Any, joint_name: str) -> str:
    names = set(str(name) for name in getattr(model, "names", []))
    if joint_name in names:
        return joint_name
    compact = joint_name.replace("_", "")
    if compact in names:
        return compact
    raise FigarohOedError(f"active joint '{joint_name}' not found in URDF model")


def _rows_from_figaroh_results(
    results: dict[str, Any],
    *,
    dof: int,
) -> list[dict[str, str]]:
    time_segments = results.get("T_F", [])
    position_segments = results.get("P_F", [])
    if not time_segments or not position_segments:
        raise FigarohOedError("FIGAROH results did not include T_F/P_F segments")
    rows: list[dict[str, str]] = []
    time_offset = 0.0
    last_time_s: float | None = None
    for times, positions in zip(time_segments, position_segments):
        times_array = np.asarray(times, dtype=float).reshape(-1)
        q_array = np.asarray(positions, dtype=float)
        if q_array.ndim != 2:
            raise FigarohOedError("FIGAROH P_F segment must be a 2D matrix")
        if q_array.shape[0] != times_array.shape[0]:
            raise FigarohOedError("FIGAROH T_F and P_F segment lengths differ")
        if q_array.shape[1] < dof:
            raise FigarohOedError("FIGAROH P_F segment has fewer joints than request")
        local_start = float(times_array[0])
        local_end = float(times_array[-1])
        for time_s, q in zip(times_array, q_array):
            absolute_time_s = time_offset + (float(time_s) - local_start)
            if last_time_s is not None and absolute_time_s <= last_time_s + 1e-9:
                continue
            row = {"time_s": f"{absolute_time_s:.6f}"}
            for joint_index in range(dof):
                row[f"q_cmd_{joint_index + 1}"] = f"{float(q[joint_index]):.6f}"
            rows.append(row)
            last_time_s = absolute_time_s
        time_offset += max(0.0, local_end - local_start)
    if not rows:
        raise FigarohOedError("FIGAROH generated no trajectory samples")
    return rows


def _write_candidate_rows(path: Path, rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _validate_request(request: dict[str, Any]) -> None:
    if request.get("schema") != "armctrl.figaroh_optimal_trajectory_request.v1":
        raise FigarohOedError("unsupported request schema")
    model = request.get("model")
    if not isinstance(model, dict):
        raise FigarohOedError("request model must be an object")
    if int(model.get("dof", 0)) <= 0:
        raise FigarohOedError("request model.dof must be positive")
    if len(model.get("active_joints", [])) != int(model["dof"]):
        raise FigarohOedError("active_joints length must match dof")


def _ensure_vendor_figaroh_on_path() -> None:
    vendor_src = Path(__file__).resolve().parents[2] / "vendor" / "figaroh" / "src"
    if vendor_src.exists() and str(vendor_src) not in sys.path:
        sys.path.insert(0, str(vendor_src))


def _resolve_path(path_text: str) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    return Path.cwd() / path


def _jsonable_shape(shape: object) -> list[int] | None:
    if shape is None:
        return None
    return [int(value) for value in shape]


def _write_minimal_yaml(path: Path, value: dict[str, Any]) -> None:
    try:
        import yaml
    except Exception as exc:  # pragma: no cover - dependency guard.
        raise FigarohOedError("PyYAML is required to write FIGAROH config") from exc
    path.write_text(
        yaml.safe_dump(value, sort_keys=False),
        encoding="utf-8",
    )


def _emit(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, ensure_ascii=False))


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
