"""有限傅里叶激励轨迹优化。

这里把成熟 SysID optimal excitation 的核心约束放进同一个问题：
- 有限傅里叶系数是优化变量；
- 关节位置、速度、加速度是硬约束；
- 关节间关系约束用于表达 X5 实机 table-safe 姿态族；
- 目标函数最小化代理观测矩阵条件数，或外部 Pinocchio scorer 的真实回归矩阵条件数。

SciPy 可用时走 SLSQP 约束非线性优化；不可用时退回到安全候选筛选。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

from armctrl.identification.models import ExcitationProfile, TrajectoryPoint
from armctrl.identification.safety import TrajectorySafetyLimits, validate_trajectory
from armctrl.identification.trajectories import generate_fourier_multisine


@dataclass(frozen=True)
class ExcitationScore:
    """激励轨迹评分结果。

    `condition_number` 越小，代理特征矩阵越不病态。
    `rank` 是特征矩阵在当前阈值下的数值秩。
    """

    condition_number: float
    rank: int
    feature_count: int
    sample_count: int
    mode: str = "surrogate_feature_condition"


JointRelationConstraint = tuple[int, int, float, float]


def optimize_fourier_multisine(
    *,
    dof: int,
    sample_hz: float,
    duration_s: float,
    harmonics: int,
    amplitude_rad: float,
    seed: int,
    candidate_count: int,
    safety_limits: TrajectorySafetyLimits | None = None,
    q_center: tuple[float, ...] | None = None,
    q0: tuple[float, ...] | None = None,
    coupled_joint_groups: tuple[tuple[int, ...], ...] = (),
    positive_only_joint_indices: tuple[int, ...] = (),
    joint_relation_constraints: tuple[JointRelationConstraint, ...] = (),
    use_nonlinear_optimizer: bool = True,
    scorer: Callable[[ExcitationProfile], ExcitationScore] | None = None,
) -> ExcitationProfile:
    """优化有限傅里叶轨迹，让激励尽量强且仍满足安全约束。"""

    if candidate_count <= 0:
        raise ValueError("candidate_count must be positive")
    best_profile: ExcitationProfile | None = None
    best_score: ExcitationScore | None = None
    rejected_candidates = 0
    candidate_records: list[tuple[ExcitationProfile, ExcitationScore]] = []
    for offset in range(candidate_count):
        candidate_seed = seed + offset
        candidate = generate_fourier_multisine(
            dof=dof,
            sample_hz=sample_hz,
            duration_s=duration_s,
            harmonics=harmonics,
            amplitude_rad=amplitude_rad,
            seed=candidate_seed,
            q_center=q_center,
            q0=q0,
            coupled_joint_groups=coupled_joint_groups,
            positive_only_joint_indices=positive_only_joint_indices,
        )
        if joint_relation_constraints:
            candidate = _relation_feasible_initial_profile(candidate, joint_relation_constraints)
        validation = _validate_with_relation_constraints(
            candidate,
            safety_limits,
            joint_relation_constraints,
        )
        if not validation.allowed:
            rejected_candidates += 1
            continue
        score = scorer(candidate) if scorer is not None else score_excitation_profile(candidate)
        candidate_records.append((candidate, score))
        if best_score is None or score.condition_number < best_score.condition_number:
            best_profile = candidate
            best_score = score
    if best_profile is None or best_score is None:
        raise ValueError("no safe fourier candidate found")
    nonlinear_result = (
        _optimize_with_slsqp(
            baseline=best_profile,
            baseline_score=best_score,
            scorer=scorer,
            safety_limits=safety_limits,
            joint_relation_constraints=joint_relation_constraints,
        )
        if use_nonlinear_optimizer and scorer is None
        else None
    )
    optimizer_backend = "seeded_candidate_search"
    nonlinear_status = None
    if nonlinear_result is not None:
        nonlinear_profile, nonlinear_score, nonlinear_status = nonlinear_result
        if (
            nonlinear_profile is not None
            and nonlinear_score is not None
            and nonlinear_score.condition_number <= best_score.condition_number
        ):
            best_profile = nonlinear_profile
            best_score = nonlinear_score
            optimizer_backend = "scipy_slsqp"
    metadata = dict(best_profile.metadata)
    metadata["optimization"] = {
        "score_mode": best_score.mode,
        "optimizer_backend": optimizer_backend,
        "candidate_count": candidate_count,
        "rejected_candidates": rejected_candidates,
        "best_seed": metadata.get("seed"),
        "condition_number": best_score.condition_number,
        "rank": best_score.rank,
        "feature_count": best_score.feature_count,
        "joint_relation_constraints": _serialize_joint_relation_constraints(joint_relation_constraints),
    }
    if nonlinear_status is not None:
        metadata["optimization"]["nonlinear_status"] = nonlinear_status
    return ExcitationProfile(
        name=best_profile.name,
        description=best_profile.description + " 已按观测矩阵条件数和安全约束优化。",
        sample_hz=best_profile.sample_hz,
        points=best_profile.points,
        metadata=metadata,
    )


def score_excitation_profile(profile: ExcitationProfile) -> ExcitationScore:
    """用代理特征矩阵评分一条激励轨迹。"""

    matrix = _surrogate_feature_matrix(profile.points)
    condition_number, rank = _condition_number(matrix)
    return ExcitationScore(
        condition_number=condition_number,
        rank=rank,
        feature_count=len(matrix[0]) if matrix else 0,
        sample_count=len(matrix),
    )


def _surrogate_feature_matrix(points: tuple[TrajectoryPoint, ...]) -> list[list[float]]:
    # 代理特征不宣称等于真实刚体动力学回归矩阵。
    # 它只把参数辨识常见相关量放进同一张矩阵：
    #   q, dq, ddq, sin(q), cos(q), dq², q*dq, q*ddq
    # 这样优化至少能避免“所有候选都只激励很窄一组状态”的明显坏情况。
    matrix: list[list[float]] = []
    for point in points:
        row = [1.0]
        row.extend(point.q)
        row.extend(point.dq)
        row.extend(point.ddq)
        row.extend(math.sin(value) for value in point.q)
        row.extend(math.cos(value) for value in point.q)
        row.extend(value * value for value in point.dq)
        row.extend(point.q[index] * point.dq[index] for index in range(point.dof))
        row.extend(point.q[index] * point.ddq[index] for index in range(point.dof))
        matrix.append(row)
    return _standardize_columns(matrix)


def _validate_with_relation_constraints(
    profile: ExcitationProfile,
    safety_limits: TrajectorySafetyLimits | None,
    joint_relation_constraints: tuple[JointRelationConstraint, ...],
):
    validation = validate_trajectory(profile, safety_limits)
    if not validation.allowed:
        return validation
    from armctrl.protocol.enums import ErrorCode
    from armctrl.protocol.errors import ValidationResult

    for point_index, point in enumerate(profile.points):
        for left, right, min_delta, max_delta in joint_relation_constraints:
            delta = point.q[left] - point.q[right]
            if delta < min_delta or delta > max_delta:
                return ValidationResult.reject(
                    ErrorCode.SAFETY_REJECTED,
                    "trajectory joint relation outside configured range",
                    {
                        "point_index": point_index,
                        "left_joint_index": left,
                        "right_joint_index": right,
                        "delta_rad": delta,
                        "min_delta_rad": min_delta,
                        "max_delta_rad": max_delta,
                    },
                )
    return validation


def _relation_feasible_initial_profile(
    profile: ExcitationProfile,
    joint_relation_constraints: tuple[JointRelationConstraint, ...],
) -> ExcitationProfile:
    coefficient_payload = profile.metadata.get("coefficients")
    if not coefficient_payload:
        return profile
    coefficients = [
        [tuple(float(value) for value in coefficient) for coefficient in joint_coefficients]
        for joint_coefficients in coefficient_payload
    ]
    for left, right, min_delta, max_delta in joint_relation_constraints:
        if min_delta <= 0.0 <= max_delta:
            averaged = []
            for left_item, right_item in zip(coefficients[left], coefficients[right]):
                harmonic = left_item[0]
                averaged.append((harmonic, 0.5 * (left_item[1] + right_item[1]), 0.5 * (left_item[2] + right_item[2])))
            coefficients[left] = averaged
            coefficients[right] = list(averaged)
    metadata = profile.metadata
    return generate_fourier_multisine(
        dof=profile.dof,
        sample_hz=profile.sample_hz,
        duration_s=float(metadata["duration_s"]),
        harmonics=int(metadata["harmonics"]),
        amplitude_rad=float(metadata["amplitude_rad"]),
        seed=int(metadata.get("seed", 1)),
        q_center=tuple(float(value) for value in metadata.get("q_center", profile.points[0].q)),
        coupled_joint_groups=tuple(tuple(int(value) for value in group) for group in metadata.get("coupled_joint_groups", ())),
        positive_only_joint_indices=tuple(int(value) for value in metadata.get("positive_only_joint_indices", ())),
        coefficients=coefficients,
    )


def _serialize_joint_relation_constraints(
    constraints: tuple[JointRelationConstraint, ...],
) -> list[dict[str, float | int]]:
    return [
        {
            "left": left,
            "right": right,
            "min_delta_rad": min_delta,
            "max_delta_rad": max_delta,
        }
        for left, right, min_delta, max_delta in constraints
    ]


def _optimize_with_slsqp(
    *,
    baseline: ExcitationProfile,
    baseline_score: ExcitationScore,
    scorer: Callable[[ExcitationProfile], ExcitationScore] | None,
    safety_limits: TrajectorySafetyLimits | None,
    joint_relation_constraints: tuple[JointRelationConstraint, ...],
) -> tuple[ExcitationProfile | None, ExcitationScore | None, dict[str, object]] | None:
    try:
        import numpy as np
        from scipy.optimize import minimize
    except ImportError:
        return None

    coefficient_payload = baseline.metadata.get("coefficients")
    if not coefficient_payload:
        return None
    dof = baseline.dof
    harmonics = int(baseline.metadata["harmonics"])
    duration_s = float(baseline.metadata["duration_s"])
    amplitude_rad = float(baseline.metadata["amplitude_rad"])
    sample_hz = float(baseline.sample_hz)
    optimization_sample_hz = min(sample_hz, 20.0)
    seed = int(baseline.metadata.get("seed", 1))
    q_center = tuple(float(value) for value in baseline.metadata.get("q_center", baseline.points[0].q))
    coupled_joint_groups = tuple(tuple(int(value) for value in group) for group in baseline.metadata.get("coupled_joint_groups", ()))
    positive_only_joint_indices = tuple(int(value) for value in baseline.metadata.get("positive_only_joint_indices", ()))
    x0 = np.asarray(
        [float(value) for joint in coefficient_payload for _, a, b in joint for value in (a, b)],
        dtype=float,
    )
    if x0.size == 0:
        return None
    coefficient_bound = max(amplitude_rad, max(abs(value) for value in x0) * 1.5, 1e-3)
    bounds = [(-coefficient_bound, coefficient_bound) for _ in range(int(x0.size))]

    def profile_from_x(x_values) -> ExcitationProfile:
        return _profile_from_x_values(
            x_values,
            dof=dof,
            sample_hz=optimization_sample_hz,
            duration_s=duration_s,
            harmonics=harmonics,
            amplitude_rad=amplitude_rad,
            seed=seed,
            q_center=q_center,
            coupled_joint_groups=coupled_joint_groups,
            positive_only_joint_indices=positive_only_joint_indices,
        )

    def full_rate_profile_from_x(x_values) -> ExcitationProfile:
        return _profile_from_x_values(
            x_values,
            dof=dof,
            sample_hz=sample_hz,
            duration_s=duration_s,
            harmonics=harmonics,
            amplitude_rad=amplitude_rad,
            seed=seed,
            q_center=q_center,
            coupled_joint_groups=coupled_joint_groups,
            positive_only_joint_indices=positive_only_joint_indices,
        )

    def objective(x_values) -> float:
        profile = profile_from_x(x_values)
        score = scorer(profile) if scorer is not None else score_excitation_profile(profile)
        range_reward = _joint_range_reward(profile)
        if not math.isfinite(score.condition_number):
            return 1e12
        return math.log10(max(score.condition_number, 1.0)) - 0.03 * range_reward

    constraints = _build_slsqp_constraints(
        safety_limits=safety_limits,
        joint_relation_constraints=joint_relation_constraints,
        profile_from_x=profile_from_x,
    )
    result = minimize(
        objective,
        x0,
        method="SLSQP",
        bounds=bounds,
        constraints=constraints,
        options={"maxiter": 15, "ftol": 1e-3, "disp": False},
    )
    status = {
        "success": bool(result.success),
        "message": str(result.message),
        "iterations": int(result.nit),
        "optimization_sample_hz": float(optimization_sample_hz),
        "initial_condition_number": float(baseline_score.condition_number),
    }
    profile = full_rate_profile_from_x(result.x)
    validation = _validate_with_relation_constraints(profile, safety_limits, joint_relation_constraints)
    if not validation.allowed:
        status["accepted_feasible_iterate"] = False
        status["rejected_reason"] = validation.error.message if validation.error else "slsqp iterate failed safety validation"
        return None, None, status
    score = scorer(profile) if scorer is not None else score_excitation_profile(profile)
    status["final_condition_number"] = float(score.condition_number)
    status["accepted_feasible_iterate"] = bool(score.condition_number <= baseline_score.condition_number)
    return profile, score, status


def _profile_from_x_values(
    x_values,
    *,
    dof: int,
    sample_hz: float,
    duration_s: float,
    harmonics: int,
    amplitude_rad: float,
    seed: int,
    q_center: tuple[float, ...],
    coupled_joint_groups: tuple[tuple[int, ...], ...],
    positive_only_joint_indices: tuple[int, ...],
) -> ExcitationProfile:
        coefficients = _vector_to_coefficients(x_values, dof=dof, harmonics=harmonics)
        return generate_fourier_multisine(
            dof=dof,
            sample_hz=sample_hz,
            duration_s=duration_s,
            harmonics=harmonics,
            amplitude_rad=amplitude_rad,
            seed=seed,
            q_center=q_center,
            coupled_joint_groups=coupled_joint_groups,
            positive_only_joint_indices=positive_only_joint_indices,
            coefficients=coefficients,
        )


def _vector_to_coefficients(x_values, *, dof: int, harmonics: int):
    coefficients = []
    cursor = 0
    for _ in range(dof):
        joint = []
        for harmonic in range(1, harmonics + 1):
            joint.append((float(harmonic), float(x_values[cursor]), float(x_values[cursor + 1])))
            cursor += 2
        coefficients.append(tuple(joint))
    return tuple(coefficients)


def _build_slsqp_constraints(
    *,
    safety_limits: TrajectorySafetyLimits | None,
    joint_relation_constraints: tuple[JointRelationConstraint, ...],
    profile_from_x,
) -> list[dict[str, object]]:
    if safety_limits is None and not joint_relation_constraints:
        return []

    def all_margins(x_values):
        profile = profile_from_x(x_values)
        margins = []
        if safety_limits is not None:
            for joint_index, limit in enumerate(safety_limits.joint_min):
                margins.append(_min_q(profile, joint_index) - limit)
            for joint_index, limit in enumerate(safety_limits.joint_max):
                margins.append(limit - _max_q(profile, joint_index))
            for joint_index, limit in enumerate(safety_limits.velocity_max):
                margins.append(limit - _max_abs_dq(profile, joint_index))
            for joint_index, limit in enumerate(safety_limits.acceleration_max):
                margins.append(limit - _max_abs_ddq(profile, joint_index))
        for left, right, min_delta, max_delta in joint_relation_constraints:
            margins.append(_min_joint_delta(profile, left, right) - min_delta)
            margins.append(max_delta - _max_joint_delta(profile, left, right))
        return margins

    return [{"type": "ineq", "fun": all_margins}]


def _min_q(profile: ExcitationProfile, joint_index: int) -> float:
    return min(point.q[joint_index] for point in profile.points)


def _max_q(profile: ExcitationProfile, joint_index: int) -> float:
    return max(point.q[joint_index] for point in profile.points)


def _max_abs_dq(profile: ExcitationProfile, joint_index: int) -> float:
    return max(abs(point.dq[joint_index]) for point in profile.points)


def _max_abs_ddq(profile: ExcitationProfile, joint_index: int) -> float:
    return max(abs(point.ddq[joint_index]) for point in profile.points)


def _min_joint_delta(profile: ExcitationProfile, left: int, right: int) -> float:
    return min(point.q[left] - point.q[right] for point in profile.points)


def _max_joint_delta(profile: ExcitationProfile, left: int, right: int) -> float:
    return max(point.q[left] - point.q[right] for point in profile.points)


def _joint_range_reward(profile: ExcitationProfile) -> float:
    ranges = []
    for joint_index in range(profile.dof):
        values = [point.q[joint_index] for point in profile.points]
        ranges.append(max(values) - min(values))
    return sum(ranges) / max(len(ranges), 1)


def _standardize_columns(matrix: list[list[float]]) -> list[list[float]]:
    if not matrix:
        return []
    row_count = len(matrix)
    column_count = len(matrix[0])
    means = [sum(row[column] for row in matrix) / row_count for column in range(column_count)]
    variances = [
        sum((row[column] - means[column]) ** 2 for row in matrix) / row_count
        for column in range(column_count)
    ]
    scales = [math.sqrt(variance) if variance > 1e-18 else 1.0 for variance in variances]
    return [
        [(row[column] - means[column]) / scales[column] for column in range(column_count)]
        for row in matrix
    ]


def _condition_number(matrix: list[list[float]]) -> tuple[float, int]:
    # 条件数来自 XᵀX 的特征值：
    #   cond(X) = sqrt(lambda_max / lambda_min)
    # 这里用 Jacobi 方法求对称矩阵特征值，避免为了一个评分函数给项目新增 numpy 依赖。
    if not matrix:
        return math.inf, 0
    gram = _gram_matrix(matrix)
    eigenvalues = _jacobi_eigenvalues(gram)
    positive = sorted(value for value in eigenvalues if value > 1e-10)
    rank = len(positive)
    if not positive:
        return math.inf, 0
    smallest = positive[0]
    largest = positive[-1]
    return math.sqrt(largest / smallest), rank


def _gram_matrix(matrix: list[list[float]]) -> list[list[float]]:
    column_count = len(matrix[0])
    gram = [[0.0 for _ in range(column_count)] for _ in range(column_count)]
    for row in matrix:
        for left in range(column_count):
            left_value = row[left]
            for right in range(left, column_count):
                gram[left][right] += left_value * row[right]
    for left in range(column_count):
        for right in range(left):
            gram[left][right] = gram[right][left]
    return gram


def _jacobi_eigenvalues(matrix: list[list[float]], *, max_sweeps: int = 80, tolerance: float = 1e-10) -> list[float]:
    # Jacobi 旋转适合这里的小型对称矩阵。
    # 它比手写完整 SVD 简单，而且不会引入第三方依赖。
    size = len(matrix)
    values = [row[:] for row in matrix]
    for _ in range(max_sweeps):
        pivot_row = 0
        pivot_col = 1 if size > 1 else 0
        max_offdiag = 0.0
        for row in range(size):
            for col in range(row + 1, size):
                offdiag = abs(values[row][col])
                if offdiag > max_offdiag:
                    max_offdiag = offdiag
                    pivot_row = row
                    pivot_col = col
        if max_offdiag < tolerance:
            break
        app = values[pivot_row][pivot_row]
        aqq = values[pivot_col][pivot_col]
        apq = values[pivot_row][pivot_col]
        if abs(apq) < tolerance:
            continue
        tau = (aqq - app) / (2.0 * apq)
        tangent = math.copysign(1.0, tau) / (abs(tau) + math.sqrt(1.0 + tau * tau)) if tau != 0.0 else 1.0
        cosine = 1.0 / math.sqrt(1.0 + tangent * tangent)
        sine = tangent * cosine
        for index in range(size):
            if index not in (pivot_row, pivot_col):
                aip = values[index][pivot_row]
                aiq = values[index][pivot_col]
                values[index][pivot_row] = cosine * aip - sine * aiq
                values[pivot_row][index] = values[index][pivot_row]
                values[index][pivot_col] = sine * aip + cosine * aiq
                values[pivot_col][index] = values[index][pivot_col]
        values[pivot_row][pivot_row] = cosine * cosine * app - 2.0 * sine * cosine * apq + sine * sine * aqq
        values[pivot_col][pivot_col] = sine * sine * app + 2.0 * sine * cosine * apq + cosine * cosine * aqq
        values[pivot_row][pivot_col] = 0.0
        values[pivot_col][pivot_row] = 0.0
    return [max(0.0, values[index][index]) for index in range(size)]
