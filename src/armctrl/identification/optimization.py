"""有限傅里叶激励轨迹优化。

这里实现的是“工程可用的第一层优化”：
- 用多个随机种子生成候选有限傅里叶轨迹；
- 对每条轨迹做关节位置、速度、加速度安全预检查；
- 用轻量代理特征矩阵估计观测条件数；
- 选择条件数最低的候选。

真正的论文级优化应把这里的代理评分替换成外部工具生成的真实回归矩阵：
`Y(q, dq, ddq)`。
当前模块先把优化接口、元数据和安全筛选做稳，方便后面接 Pinocchio 或 URDFly。
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
    scorer: Callable[[ExcitationProfile], ExcitationScore] | None = None,
) -> ExcitationProfile:
    """在多个候选有限傅里叶轨迹中选择代理条件数最低的一条。"""

    if candidate_count <= 0:
        raise ValueError("candidate_count must be positive")
    best_profile: ExcitationProfile | None = None
    best_score: ExcitationScore | None = None
    rejected_candidates = 0
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
        )
        validation = validate_trajectory(candidate, safety_limits)
        if not validation.allowed:
            rejected_candidates += 1
            continue
        score = scorer(candidate) if scorer is not None else score_excitation_profile(candidate)
        if best_score is None or score.condition_number < best_score.condition_number:
            best_profile = candidate
            best_score = score
    if best_profile is None or best_score is None:
        raise ValueError("no safe fourier candidate found")
    metadata = dict(best_profile.metadata)
    metadata["optimization"] = {
        "score_mode": best_score.mode,
        "candidate_count": candidate_count,
        "rejected_candidates": rejected_candidates,
        "best_seed": metadata.get("seed"),
        "condition_number": best_score.condition_number,
        "rank": best_score.rank,
        "feature_count": best_score.feature_count,
    }
    return ExcitationProfile(
        name=best_profile.name,
        description=best_profile.description + " 已按代理观测矩阵条件数从候选集中选优。",
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
