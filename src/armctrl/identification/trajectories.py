"""安全激励轨迹生成。

轨迹分三层：
1. `gravity_sweep`：静态/准静态重力扫描，一次主要移动一个关节；
2. `friction_sweep`：正反向慢速/中速扫描，用于摩擦项估计；
3. `fourier_multisine`：多关节有限傅里叶轨迹，用于耦合动力学辨识。

傅里叶轨迹默认乘五次平滑包络，使起止位置、速度、加速度回到零。
这对应工程上常用的“改进傅里叶 / 傅里叶 + 五次多项式混合”思路，
目标是减少起止速度和加速度不连续带来的振动与跟踪误差。
"""

from __future__ import annotations

import math
import random
from collections.abc import Sequence

from armctrl.identification.models import ExcitationProfile, TrajectoryPoint


def _zeros(dof: int) -> tuple[float, ...]:
    return tuple(0.0 for _ in range(dof))


def _base_q(dof: int, q0: tuple[float, ...] | None) -> tuple[float, ...]:
    if q0 is None:
        return _zeros(dof)
    if len(q0) != dof:
        raise ValueError(f"q0 must contain {dof} values")
    return tuple(float(value) for value in q0)


def _center_q(
    dof: int,
    q_center: tuple[float, ...] | None,
    q0: tuple[float, ...] | None,
) -> tuple[float, ...]:
    if q_center is not None and q0 is not None:
        if tuple(float(value) for value in q_center) != tuple(float(value) for value in q0):
            raise ValueError("q_center and q0 must match when both are provided")
    source = q_center if q_center is not None else q0
    if source is None:
        return _zeros(dof)
    if len(source) != dof:
        raise ValueError(f"q_center must contain {dof} values")
    return tuple(float(value) for value in source)


def _sample_times(duration_s: float, sample_hz: float) -> list[float]:
    # 末尾显式包含 duration_s，保证轨迹最后一个点能回到安全边界条件。
    if duration_s <= 0:
        raise ValueError("duration_s must be positive")
    if sample_hz <= 0:
        raise ValueError("sample_hz must be positive")
    step_count = max(1, int(round(duration_s * sample_hz)))
    return [min(index / sample_hz, duration_s) for index in range(step_count)] + [duration_s]


def _smoothstep5(s: float) -> tuple[float, float, float]:
    # 五次 smoothstep：
    #   h(s)  = 10s³ - 15s⁴ + 6s⁵
    #   h'(s) = 30s² - 60s³ + 30s⁴
    #   h''(s)= 60s - 180s² + 120s³
    # 它在 s=0 和 s=1 处一阶、二阶导数都为 0。
    h = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
    dh = 30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4
    ddh = 60.0 * s - 180.0 * s**2 + 120.0 * s**3
    return h, dh, ddh


def _zero_boundary_envelope(s: float) -> tuple[float, float, float]:
    # 用五次 smoothstep 构造零边界包络：
    #   e(s) = 4h(s)(1-h(s))
    # 乘 4 是为了让包络峰值回到 1。
    # 因为 h'(0)=h'(1)=h''(0)=h''(1)=0，
    # 所以 e 在起止点的位置、速度和加速度扰动也都是 0。
    h, dh, ddh = _smoothstep5(s)
    envelope = 4.0 * h * (1.0 - h)
    denvelope = 4.0 * dh * (1.0 - 2.0 * h)
    ddenvelope = 4.0 * (ddh * (1.0 - 2.0 * h) - 2.0 * dh * dh)
    return envelope, denvelope, ddenvelope


def _quintic_segment(
    start_q: tuple[float, ...],
    end_q: tuple[float, ...],
    start_t_s: float,
    duration_s: float,
    sample_hz: float,
    phase: str,
    skip_first: bool,
) -> list[TrajectoryPoint]:
    # 单段五次插值用于低风险扫描。
    # 它不是最优激励，但足够平滑，适合 bringup 和静态/准静态数据采集。
    points: list[TrajectoryPoint] = []
    times = _sample_times(duration_s, sample_hz)
    if skip_first:
        times = times[1:]
    delta = tuple(end_q[index] - start_q[index] for index in range(len(start_q)))
    for local_t in times:
        s = min(max(local_t / duration_s, 0.0), 1.0)
        h, dh, ddh = _smoothstep5(s)
        q = tuple(start_q[index] + delta[index] * h for index in range(len(start_q)))
        dq = tuple(delta[index] * dh / duration_s for index in range(len(start_q)))
        ddq = tuple(delta[index] * ddh / (duration_s * duration_s) for index in range(len(start_q)))
        points.append(TrajectoryPoint(start_t_s + local_t, q, dq, ddq, phase))
    return points


def _dwell_segment(
    q: tuple[float, ...],
    start_t_s: float,
    duration_s: float,
    sample_hz: float,
    phase: str,
) -> list[TrajectoryPoint]:
    if duration_s <= 0:
        return []
    return [
        TrajectoryPoint(start_t_s + local_t, q, _zeros(len(q)), _zeros(len(q)), phase)
        for local_t in _sample_times(duration_s, sample_hz)[1:]
    ]


def _constant_velocity_segment(
    start_q: tuple[float, ...],
    end_q: tuple[float, ...],
    joint_index: int,
    start_t_s: float,
    speed_radps: float,
    sample_hz: float,
    phase: str,
) -> list[TrajectoryPoint]:
    distance = end_q[joint_index] - start_q[joint_index]
    duration_s = abs(distance) / max(abs(speed_radps), 1e-6)
    signed_speed = math.copysign(abs(speed_radps), distance)
    points: list[TrajectoryPoint] = []
    for local_t in _sample_times(duration_s, sample_hz)[1:]:
        q_values = list(start_q)
        q_values[joint_index] = start_q[joint_index] + signed_speed * local_t
        dq_values = [0.0] * len(start_q)
        dq_values[joint_index] = signed_speed
        points.append(
            TrajectoryPoint(
                start_t_s + local_t,
                tuple(q_values),
                tuple(dq_values),
                _zeros(len(start_q)),
                phase,
            )
        )
    return points


def generate_gravity_sweep(
    *,
    dof: int = 6,
    sample_hz: float = 100.0,
    amplitude_rad: float = 0.20,
    segment_duration_s: float = 4.0,
    dwell_s: float = 0.0,
    q_center: tuple[float, ...] | None = None,
    q0: tuple[float, ...] | None = None,
) -> ExcitationProfile:
    """生成静态/准静态重力扫描轨迹。"""

    base = _center_q(dof, q_center, q0)
    points = [TrajectoryPoint(0.0, base, _zeros(dof), _zeros(dof), "gravity_start")]
    current_q = base
    current_t = 0.0
    for joint_index in range(dof):
        for signed_amplitude in (amplitude_rad, 0.0, -amplitude_rad, 0.0):
            target = list(base)
            target[joint_index] = base[joint_index] + signed_amplitude
            phase = f"gravity_joint_{joint_index + 1}"
            segment = _quintic_segment(
                current_q,
                tuple(target),
                current_t,
                segment_duration_s,
                sample_hz,
                phase,
                skip_first=True,
            )
            points.extend(segment)
            current_q = tuple(target)
            current_t = points[-1].t_s
            dwell = _dwell_segment(
                current_q,
                current_t,
                dwell_s,
                sample_hz,
                f"{phase}_hold",
            )
            if dwell:
                points.extend(dwell)
                current_t = points[-1].t_s
    return ExcitationProfile(
        name="gravity_sweep",
        description="静态/准静态重力扫描：一次主要移动一个关节，速度和加速度尽量小。",
        sample_hz=sample_hz,
        points=tuple(points),
        metadata={
            "layer": "low",
            "amplitude_rad": amplitude_rad,
            "segment_duration_s": segment_duration_s,
            "dwell_s": dwell_s,
            "q_center": base,
            "recommended_use": "先辨识重力项和末端 payload 影响。",
        },
    )


def generate_friction_sweep(
    *,
    dof: int = 6,
    sample_hz: float = 100.0,
    amplitude_rad: float = 0.12,
    slow_speed_radps: float = 0.025,
    medium_speed_radps: float = 0.06,
    fast_speed_radps: float = 0.12,
    q0: tuple[float, ...] | None = None,
) -> ExcitationProfile:
    """生成摩擦辨识扫描轨迹。"""

    base = _base_q(dof, q0)
    points = [TrajectoryPoint(0.0, base, _zeros(dof), _zeros(dof), "friction_start")]
    current_q = base
    current_t = 0.0
    for joint_index in range(dof):
        for speed_label, speed in (
            ("slow", slow_speed_radps),
            ("medium", medium_speed_radps),
            ("fast", fast_speed_radps),
        ):
            for start_offset, end_offset, direction_label in (
                (-amplitude_rad, amplitude_rad, "positive"),
                (amplitude_rad, -amplitude_rad, "negative"),
            ):
                start_target = list(base)
                start_target[joint_index] = base[joint_index] + start_offset
                approach = _quintic_segment(
                    current_q,
                    tuple(start_target),
                    current_t,
                    max(1.0, amplitude_rad / 0.12),
                    sample_hz,
                    f"friction_joint_{joint_index + 1}_{speed_label}_{direction_label}_approach",
                    skip_first=True,
                )
                points.extend(approach)
                current_q = tuple(start_target)
                current_t = points[-1].t_s
                end_target = list(base)
                end_target[joint_index] = base[joint_index] + end_offset
                plateau = _constant_velocity_segment(
                    current_q,
                    tuple(end_target),
                    joint_index,
                    current_t,
                    speed,
                    sample_hz,
                    f"friction_joint_{joint_index + 1}_{speed_label}_{direction_label}_plateau",
                )
                points.extend(plateau)
                current_q = tuple(end_target)
                current_t = points[-1].t_s
    return ExcitationProfile(
        name="friction_sweep",
        description="摩擦辨识扫描：每个关节覆盖正反向慢速和中速运动。",
        sample_hz=sample_hz,
        points=tuple(points),
        metadata={
            "layer": "medium",
            "amplitude_rad": amplitude_rad,
            "slow_speed_radps": slow_speed_radps,
            "medium_speed_radps": medium_speed_radps,
            "fast_speed_radps": fast_speed_radps,
            "speed_levels_radps": (slow_speed_radps, medium_speed_radps, fast_speed_radps),
            "constant_velocity_plateaus": True,
            "q_center": base,
            "recommended_use": "估计库伦摩擦和粘性摩擦。",
        },
    )


def _normalised_fourier_coefficients(
    *,
    dof: int,
    harmonics: int,
    amplitude_rad: float,
    duration_s: float,
    seed: int,
) -> list[list[tuple[float, float, float]]]:
    # 每个关节生成一组 (k, a, b)。
    # 先随机，再按粗采样的最大幅值归一化，保证姿态幅值不超过 amplitude_rad。
    rng = random.Random(seed)
    omega = 2.0 * math.pi / duration_s
    all_coefficients: list[list[tuple[float, float, float]]] = []
    for joint_index in range(dof):
        raw = []
        for harmonic in range(1, harmonics + 1):
            phase_bias = 0.3 * joint_index
            a = rng.uniform(-1.0, 1.0) / harmonic
            b = rng.uniform(-1.0, 1.0) / harmonic
            raw.append((float(harmonic), a + 0.05 * math.sin(phase_bias), b))
        max_abs = 0.0
        for sample_index in range(200):
            t = duration_s * sample_index / 199.0
            value = sum(
                a * math.sin(harmonic * omega * t) + b * math.cos(harmonic * omega * t)
                for harmonic, a, b in raw
            )
            max_abs = max(max_abs, abs(value))
        scale = amplitude_rad / max(max_abs, 1e-9)
        all_coefficients.append([(harmonic, a * scale, b * scale) for harmonic, a, b in raw])
    return all_coefficients


def _coerce_fourier_coefficients(
    coefficients: Sequence[Sequence[Sequence[float]]] | None,
    *,
    dof: int,
    harmonics: int,
) -> tuple[tuple[tuple[float, float, float], ...], ...] | None:
    if coefficients is None:
        return None
    if len(coefficients) != dof:
        raise ValueError(f"coefficients must contain {dof} joint coefficient sets")
    coerced: list[tuple[tuple[float, float, float], ...]] = []
    for joint_index, joint_coefficients in enumerate(coefficients):
        if len(joint_coefficients) != harmonics:
            raise ValueError(f"coefficients[{joint_index}] must contain {harmonics} harmonics")
        joint_values = []
        for harmonic_index, item in enumerate(joint_coefficients):
            if len(item) != 3:
                raise ValueError("each coefficient entry must be (harmonic, a, b)")
            harmonic, a, b = (float(item[0]), float(item[1]), float(item[2]))
            expected_harmonic = float(harmonic_index + 1)
            if abs(harmonic - expected_harmonic) > 1e-9:
                raise ValueError("coefficient harmonics must be ordered from 1 to harmonics")
            joint_values.append((harmonic, a, b))
        coerced.append(tuple(joint_values))
    return tuple(coerced)


def generate_fourier_multisine(
    *,
    dof: int = 6,
    sample_hz: float = 100.0,
    duration_s: float = 12.0,
    harmonics: int = 5,
    amplitude_rad: float = 0.12,
    seed: int = 1,
    q_center: tuple[float, ...] | None = None,
    q0: tuple[float, ...] | None = None,
    coupled_joint_groups: tuple[tuple[int, ...], ...] = (),
    positive_only_joint_indices: tuple[int, ...] = (),
    coefficients: Sequence[Sequence[Sequence[float]]] | None = None,
) -> ExcitationProfile:
    """生成带五次包络的有限傅里叶多关节激励轨迹。"""

    base = _center_q(dof, q_center, q0)
    omega = 2.0 * math.pi / duration_s
    explicit_coefficients = _coerce_fourier_coefficients(coefficients, dof=dof, harmonics=harmonics)
    active_coefficients = explicit_coefficients or tuple(
        tuple(joint_coefficients)
        for joint_coefficients in _normalised_fourier_coefficients(
            dof=dof,
            harmonics=harmonics,
            amplitude_rad=amplitude_rad,
            duration_s=duration_s,
            seed=seed,
        )
    )
    points: list[TrajectoryPoint] = []
    for t in _sample_times(duration_s, sample_hz):
        s = min(max(t / duration_s, 0.0), 1.0)
        envelope, denvelope_ds, ddenvelope_ds = _zero_boundary_envelope(s)
        denvelope_dt = denvelope_ds / duration_s
        ddenvelope_dt = ddenvelope_ds / (duration_s * duration_s)
        signals: list[float] = []
        signal_ds: list[float] = []
        signal_dds: list[float] = []
        for joint_index in range(dof):
            signal = 0.0
            signal_d = 0.0
            signal_dd = 0.0
            for harmonic, a, b in active_coefficients[joint_index]:
                kw = harmonic * omega
                signal += a * math.sin(kw * t) + b * math.cos(kw * t)
                signal_d += a * kw * math.cos(kw * t) - b * kw * math.sin(kw * t)
                signal_dd += -a * kw * kw * math.sin(kw * t) - b * kw * kw * math.cos(kw * t)
            signals.append(signal)
            signal_ds.append(signal_d)
            signal_dds.append(signal_dd)
        for group in coupled_joint_groups:
            if not group:
                continue
            representative = group[0]
            for joint_index in group[1:]:
                signals[joint_index] = signals[representative]
                signal_ds[joint_index] = signal_ds[representative]
                signal_dds[joint_index] = signal_dds[representative]
        for joint_index in positive_only_joint_indices:
            signals[joint_index] = 0.5 * (signals[joint_index] + amplitude_rad)
            signal_ds[joint_index] = 0.5 * signal_ds[joint_index]
            signal_dds[joint_index] = 0.5 * signal_dds[joint_index]
        q_values: list[float] = []
        dq_values: list[float] = []
        ddq_values: list[float] = []
        for joint_index in range(dof):
            signal = signals[joint_index]
            signal_d = signal_ds[joint_index]
            signal_dd = signal_dds[joint_index]
            q_values.append(base[joint_index] + envelope * signal)
            dq_values.append(denvelope_dt * signal + envelope * signal_d)
            ddq_values.append(ddenvelope_dt * signal + 2.0 * denvelope_dt * signal_d + envelope * signal_dd)
        points.append(TrajectoryPoint(t, tuple(q_values), tuple(dq_values), tuple(ddq_values), "fourier_multisine"))
    return ExcitationProfile(
        name="fourier_multisine",
        description="多关节有限傅里叶轨迹：覆盖耦合动力学，并用五次包络平滑起止边界。",
        sample_hz=sample_hz,
        points=tuple(points),
        metadata={
            "layer": "high",
            "amplitude_rad": amplitude_rad,
            "duration_s": duration_s,
            "harmonics": harmonics,
            "seed": seed,
            "q_center": base,
            "coupled_joint_groups": coupled_joint_groups,
            "positive_only_joint_indices": positive_only_joint_indices,
            "coefficient_source": "explicit" if explicit_coefficients is not None else "seeded_random_normalized",
            "coefficients": [
                [[harmonic, a, b] for harmonic, a, b in joint_coefficients]
                for joint_coefficients in active_coefficients
            ],
            "boundary_mode": "quintic_envelope_zero_velocity_acceleration",
            "recommended_use": "让回归矩阵观测条件更好，真实硬件上应放在最后阶段。",
        },
    )
