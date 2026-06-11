"""Safety-gated motion runtime primitives for real and fake arm backends."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
import math
from uuid import uuid4
from time import monotonic as default_monotonic
from time import sleep as default_sleep
from typing import Callable, Iterable, Protocol, Sequence


class MotionMode(str, Enum):
    HOLD = "hold"
    DAMPING = "damping"
    AGENT_SERVO = "agent_servo"
    TRAJECTORY_REPLAY = "trajectory_replay"


class ArmRuntimeMode(str, Enum):
    PASSIVE_SAFE = "passive_safe"
    HOLD_SAFE = "hold_safe"


class MotionModeError(RuntimeError):
    """Raised when two producers try to own the runtime at the same time."""


class ArmRuntimeError(RuntimeError):
    """Raised when a live arm runtime ownership or state invariant is violated."""


@dataclass(frozen=True)
class JointTrajectoryPoint:
    time_s: float
    q: tuple[float, ...]
    dq: tuple[float, ...] | None = None


@dataclass(frozen=True)
class JointIntentFrame:
    q_start: tuple[float, ...]
    q_target: tuple[float, ...]
    control_period_s: float
    max_joint_delta_rad: float | None = None
    max_joint_velocity_rad_s: float | None = None
    max_joint_acceleration_rad_s2: float | None = None


@dataclass(frozen=True)
class JointCommandRecord:
    q: tuple[float, ...]
    sent_monotonic_s: float
    producer: str
    mode: str


@dataclass(frozen=True)
class JointStateSnapshot:
    q_meas: tuple[float, ...]
    dq_meas: tuple[float, ...] = ()
    tau_meas: tuple[float, ...] = ()
    fault_flags: tuple[str, ...] = ()


@dataclass(frozen=True)
class MotionAuditSample:
    sent_monotonic_s: float
    q_cmd: tuple[float, ...]
    dq_cmd: tuple[float, ...]
    q_meas: tuple[float, ...]
    dq_meas: tuple[float, ...]
    tau_meas: tuple[float, ...]
    fault_flags: tuple[str, ...]
    producer: str
    mode: str


@dataclass(frozen=True)
class MotionExecutionResult:
    status: str
    producer: str
    mode: str
    trajectory_sample_hz: float | None
    actual_send_hz: float | None
    send_jitter_ms_p95: float | None
    send_jitter_ms_p99: float | None
    controller_dt_s: float | None
    samples: tuple[MotionAuditSample, ...]
    landing_mode: str
    error: dict[str, str] | None = None


@dataclass
class _TrackingErrorGate:
    max_error_rad: float | None
    grace_samples: int = 0
    consecutive_samples: int = 1
    _sample_count: int = 0
    _consecutive_exceeded: int = 0

    def __post_init__(self) -> None:
        if self.grace_samples < 0:
            raise ValueError("tracking_error_grace_samples must be non-negative")
        if self.consecutive_samples <= 0:
            raise ValueError("tracking_error_consecutive_samples must be positive")

    def exceeded(self, sample: MotionAuditSample) -> bool:
        self._sample_count += 1
        if not _tracking_error_exceeded(
            sample,
            max_tracking_error_rad=self.max_error_rad,
        ):
            self._consecutive_exceeded = 0
            return False
        if self._sample_count <= self.grace_samples:
            return False
        self._consecutive_exceeded += 1
        return self._consecutive_exceeded >= self.consecutive_samples


class MotionBackend(Protocol):
    def send_joint_command(
        self,
        q: tuple[float, ...],
        *,
        producer: str,
        mode: MotionMode,
        monotonic_s: float,
        trajectory_time_s: float | None = None,
        dq: tuple[float, ...] | None = None,
    ) -> None:
        ...

    def hold(self) -> str | None:
        ...

    def damping(self) -> None:
        ...

    def read_joint_state(self) -> JointStateSnapshot:
        ...


@dataclass
class FakeMotionBackend:
    """In-memory backend used to verify timing and safety semantics."""

    joint_commands: list[JointCommandRecord] = field(default_factory=list)
    hold_count: int = 0
    damping_count: int = 0
    fault_flags: tuple[str, ...] = ()
    _last_q: tuple[float, ...] | None = None

    def send_joint_command(
        self,
        q: tuple[float, ...],
        *,
        producer: str,
        mode: MotionMode,
        monotonic_s: float,
        trajectory_time_s: float | None = None,
        dq: tuple[float, ...] | None = None,
    ) -> None:
        q_tuple = tuple(float(value) for value in q)
        self._last_q = q_tuple
        self.joint_commands.append(
            JointCommandRecord(
                q=q_tuple,
                sent_monotonic_s=float(monotonic_s),
                producer=producer,
                mode=mode.value,
            )
        )

    def hold(self) -> str:
        self.hold_count += 1
        return MotionMode.HOLD.value

    def damping(self) -> None:
        self.damping_count += 1

    def read_joint_state(self) -> JointStateSnapshot:
        return JointStateSnapshot(q_meas=self._last_q or (), fault_flags=self.fault_flags)


class _ModeToken:
    def __init__(self, runtime: MotionRuntime, owner: str, mode: MotionMode) -> None:
        self._runtime = runtime
        self._owner = owner
        self._mode = mode
        self._released = False

    def release(self) -> None:
        if self._released:
            return
        self._released = True
        self._runtime._release_mode(owner=self._owner, mode=self._mode)


class MotionRuntime:
    def __init__(
        self,
        *,
        backend: MotionBackend,
        monotonic: Callable[[], float] = default_monotonic,
        sleep: Callable[[float], None] = default_sleep,
    ) -> None:
        self._backend = backend
        self._monotonic = monotonic
        self._sleep = sleep
        self._mode = MotionMode.HOLD
        self._owner: str | None = None
        self._last_intent_frame_s: float | None = None
        self._stale_intent_held = False

    @property
    def mode(self) -> MotionMode:
        return self._mode

    def acquire_mode(self, mode: MotionMode, *, producer: str) -> _ModeToken:
        if self._owner is not None and self._owner != producer:
            raise MotionModeError(
                f"motion runtime is owned by {self._owner}; {producer} cannot acquire {mode.value}"
            )
        self._owner = producer
        self._mode = mode
        return _ModeToken(self, producer, mode)

    def execute_trajectory(
        self,
        trajectory: Iterable[JointTrajectoryPoint],
        *,
        producer: str,
        trajectory_sample_hz: float,
        hold_after: bool = False,
        watchdog: Callable[[], dict[str, object] | None] | None = None,
        on_sample: Callable[[MotionAuditSample], None] | None = None,
        max_tracking_error_rad: float | None = None,
        tracking_error_grace_samples: int = 0,
        tracking_error_consecutive_samples: int = 1,
        max_tau_abs: float | None = None,
    ) -> MotionExecutionResult:
        points = list(trajectory)
        _validate_trajectory_inputs(
            points,
            trajectory_sample_hz=float(trajectory_sample_hz),
        )
        token = self.acquire_mode(MotionMode.TRAJECTORY_REPLAY, producer=producer)
        sent_times: list[float] = []
        samples: list[MotionAuditSample] = []
        tracking_gate = _TrackingErrorGate(
            max_error_rad=max_tracking_error_rad,
            grace_samples=int(tracking_error_grace_samples),
            consecutive_samples=int(tracking_error_consecutive_samples),
        )
        landing_mode = "released"
        try:
            begin_trajectory = getattr(self._backend, "begin_joint_trajectory", None)
            if callable(begin_trajectory):
                begin_trajectory(points)
            start_s = self._monotonic()
            for point in points:
                watchdog_event = watchdog() if watchdog is not None else None
                if watchdog_event is not None:
                    self._damping()
                    return _motion_execution_result(
                        status="faulted",
                        producer=producer,
                        mode=MotionMode.TRAJECTORY_REPLAY,
                        trajectory_sample_hz=float(trajectory_sample_hz),
                        sent_times=sent_times,
                        expected_period_s=1.0 / float(trajectory_sample_hz),
                        controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                        samples=samples,
                        landing_mode=MotionMode.DAMPING.value,
                        error=_watchdog_error(watchdog_event),
                    )
                target_s = start_s + float(point.time_s)
                now_s = self._monotonic()
                if now_s < target_s:
                    self._sleep(target_s - now_s)
                sent_s = self._monotonic()
                q_cmd = tuple(point.q)
                dq_cmd = (
                    tuple(point.dq)
                    if point.dq is not None
                    else tuple(0.0 for _ in q_cmd)
                )
                self._backend.send_joint_command(
                    q_cmd,
                    producer=producer,
                    mode=MotionMode.TRAJECTORY_REPLAY,
                    monotonic_s=sent_s,
                    trajectory_time_s=float(point.time_s),
                    dq=dq_cmd,
                )
                state = self._backend.read_joint_state()
                sent_times.append(sent_s)
                sample = MotionAuditSample(
                    sent_monotonic_s=sent_s,
                    q_cmd=q_cmd,
                    dq_cmd=dq_cmd,
                    q_meas=state.q_meas,
                    dq_meas=state.dq_meas,
                    tau_meas=state.tau_meas,
                    fault_flags=state.fault_flags,
                    producer=producer,
                    mode=MotionMode.TRAJECTORY_REPLAY.value,
                )
                samples.append(sample)
                if on_sample is not None:
                    on_sample(sample)
                if state.fault_flags:
                    self._damping()
                    return _motion_execution_result(
                        status="faulted",
                        producer=producer,
                        mode=MotionMode.TRAJECTORY_REPLAY,
                        trajectory_sample_hz=float(trajectory_sample_hz),
                        sent_times=sent_times,
                        expected_period_s=1.0 / float(trajectory_sample_hz),
                        controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                        samples=samples,
                        landing_mode=MotionMode.DAMPING.value,
                    )
                if tracking_gate.exceeded(sample):
                    landing_mode = self._hold()
                    return _motion_execution_result(
                        status="aborted",
                        producer=producer,
                        mode=MotionMode.TRAJECTORY_REPLAY,
                        trajectory_sample_hz=float(trajectory_sample_hz),
                        sent_times=sent_times,
                        expected_period_s=1.0 / float(trajectory_sample_hz),
                        controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                        samples=samples,
                        landing_mode=landing_mode,
                        error=_tracking_error_limit_error(
                            debounced=(
                                tracking_gate.grace_samples > 0
                                or tracking_gate.consecutive_samples > 1
                            ),
                        ),
                    )
                if _tau_limit_exceeded(sample, max_tau_abs=max_tau_abs):
                    self._damping()
                    return _motion_execution_result(
                        status="faulted",
                        producer=producer,
                        mode=MotionMode.TRAJECTORY_REPLAY,
                        trajectory_sample_hz=float(trajectory_sample_hz),
                        sent_times=sent_times,
                        expected_period_s=1.0 / float(trajectory_sample_hz),
                        controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                        samples=samples,
                        landing_mode=MotionMode.DAMPING.value,
                        error=_tau_limit_error(),
                    )
                watchdog_event = watchdog() if watchdog is not None else None
                if watchdog_event is not None:
                    self._damping()
                    return _motion_execution_result(
                        status="faulted",
                        producer=producer,
                        mode=MotionMode.TRAJECTORY_REPLAY,
                        trajectory_sample_hz=float(trajectory_sample_hz),
                        sent_times=sent_times,
                        expected_period_s=1.0 / float(trajectory_sample_hz),
                        controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                        samples=samples,
                        landing_mode=MotionMode.DAMPING.value,
                        error=_watchdog_error(watchdog_event),
                    )
            if hold_after:
                landing_mode = self._hold()
            else:
                token.release()
            return _motion_execution_result(
                status="completed",
                producer=producer,
                mode=MotionMode.TRAJECTORY_REPLAY,
                trajectory_sample_hz=float(trajectory_sample_hz),
                sent_times=sent_times,
                expected_period_s=1.0 / float(trajectory_sample_hz),
                controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                samples=samples,
                landing_mode=landing_mode,
            )
        except Exception as exc:
            self._damping()
            return _motion_execution_result(
                status="aborted",
                producer=producer,
                mode=MotionMode.TRAJECTORY_REPLAY,
                trajectory_sample_hz=float(trajectory_sample_hz),
                sent_times=sent_times,
                expected_period_s=1.0 / float(trajectory_sample_hz),
                controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                samples=samples,
                landing_mode=MotionMode.DAMPING.value,
                error=_motion_error(exc),
            )
        except BaseException:
            self._damping()
            raise

    def execute_intent_frame(
        self,
        intent: JointIntentFrame,
        *,
        producer: str,
        send_hz: float,
        hold_after: bool = False,
        watchdog: Callable[[], dict[str, object] | None] | None = None,
        on_sample: Callable[[MotionAuditSample], None] | None = None,
        max_tracking_error_rad: float | None = None,
        tracking_error_grace_samples: int = 0,
        tracking_error_consecutive_samples: int = 1,
        max_tau_abs: float | None = None,
    ) -> MotionExecutionResult:
        if send_hz <= 0.0:
            raise ValueError("send_hz must be positive")
        points = _intent_frame_points(intent, send_hz=send_hz)
        token = self.acquire_mode(MotionMode.AGENT_SERVO, producer=producer)
        sent_times: list[float] = []
        samples: list[MotionAuditSample] = []
        tracking_gate = _TrackingErrorGate(
            max_error_rad=max_tracking_error_rad,
            grace_samples=int(tracking_error_grace_samples),
            consecutive_samples=int(tracking_error_consecutive_samples),
        )
        landing_mode = "released"
        try:
            begin_trajectory = getattr(self._backend, "begin_joint_trajectory", None)
            if callable(begin_trajectory):
                begin_trajectory(points)
            start_s = self._monotonic()
            self._last_intent_frame_s = start_s
            self._stale_intent_held = False
            for point in points:
                watchdog_event = watchdog() if watchdog is not None else None
                if watchdog_event is not None:
                    self._damping()
                    return _motion_execution_result(
                        status="faulted",
                        producer=producer,
                        mode=MotionMode.AGENT_SERVO,
                        trajectory_sample_hz=float(send_hz),
                        sent_times=sent_times,
                        expected_period_s=1.0 / float(send_hz),
                        controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                        samples=samples,
                        landing_mode=MotionMode.DAMPING.value,
                        error=_watchdog_error(watchdog_event),
                    )
                target_s = start_s + float(point.time_s)
                now_s = self._monotonic()
                if now_s < target_s:
                    self._sleep(target_s - now_s)
                sent_s = self._monotonic()
                q_cmd = tuple(point.q)
                dq_cmd = (
                    tuple(point.dq)
                    if point.dq is not None
                    else tuple(0.0 for _ in q_cmd)
                )
                self._backend.send_joint_command(
                    q_cmd,
                    producer=producer,
                    mode=MotionMode.AGENT_SERVO,
                    monotonic_s=sent_s,
                    trajectory_time_s=float(point.time_s),
                    dq=dq_cmd,
                )
                state = self._backend.read_joint_state()
                sent_times.append(sent_s)
                sample = MotionAuditSample(
                    sent_monotonic_s=sent_s,
                    q_cmd=q_cmd,
                    dq_cmd=dq_cmd,
                    q_meas=state.q_meas,
                    dq_meas=state.dq_meas,
                    tau_meas=state.tau_meas,
                    fault_flags=state.fault_flags,
                    producer=producer,
                    mode=MotionMode.AGENT_SERVO.value,
                )
                samples.append(sample)
                if on_sample is not None:
                    on_sample(sample)
                if state.fault_flags:
                    self._damping()
                    return _motion_execution_result(
                        status="faulted",
                        producer=producer,
                        mode=MotionMode.AGENT_SERVO,
                        trajectory_sample_hz=float(send_hz),
                        sent_times=sent_times,
                        expected_period_s=1.0 / float(send_hz),
                        controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                        samples=samples,
                        landing_mode=MotionMode.DAMPING.value,
                    )
                if tracking_gate.exceeded(sample):
                    landing_mode = self._hold()
                    return _motion_execution_result(
                        status="aborted",
                        producer=producer,
                        mode=MotionMode.AGENT_SERVO,
                        trajectory_sample_hz=float(send_hz),
                        sent_times=sent_times,
                        expected_period_s=1.0 / float(send_hz),
                        controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                        samples=samples,
                        landing_mode=landing_mode,
                        error=_tracking_error_limit_error(
                            debounced=(
                                tracking_gate.grace_samples > 0
                                or tracking_gate.consecutive_samples > 1
                            ),
                        ),
                    )
                if _tau_limit_exceeded(sample, max_tau_abs=max_tau_abs):
                    self._damping()
                    return _motion_execution_result(
                        status="faulted",
                        producer=producer,
                        mode=MotionMode.AGENT_SERVO,
                        trajectory_sample_hz=float(send_hz),
                        sent_times=sent_times,
                        expected_period_s=1.0 / float(send_hz),
                        controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                        samples=samples,
                        landing_mode=MotionMode.DAMPING.value,
                        error=_tau_limit_error(),
                    )
                watchdog_event = watchdog() if watchdog is not None else None
                if watchdog_event is not None:
                    self._damping()
                    return _motion_execution_result(
                        status="faulted",
                        producer=producer,
                        mode=MotionMode.AGENT_SERVO,
                        trajectory_sample_hz=float(send_hz),
                        sent_times=sent_times,
                        expected_period_s=1.0 / float(send_hz),
                        controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                        samples=samples,
                        landing_mode=MotionMode.DAMPING.value,
                        error=_watchdog_error(watchdog_event),
                    )
            if hold_after:
                landing_mode = self._hold()
            else:
                token.release()
            return _motion_execution_result(
                status="completed",
                producer=producer,
                mode=MotionMode.AGENT_SERVO,
                trajectory_sample_hz=float(send_hz),
                sent_times=sent_times,
                expected_period_s=1.0 / float(send_hz),
                controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                samples=samples,
                landing_mode=landing_mode,
            )
        except Exception as exc:
            self._damping()
            return _motion_execution_result(
                status="aborted",
                producer=producer,
                mode=MotionMode.AGENT_SERVO,
                trajectory_sample_hz=float(send_hz),
                sent_times=sent_times,
                expected_period_s=1.0 / float(send_hz),
                controller_dt_s=getattr(self._backend, "controller_dt_s", None),
                samples=samples,
                landing_mode=MotionMode.DAMPING.value,
                error=_motion_error(exc),
            )
        except BaseException:
            self._damping()
            raise

    def mark_intent_frame(self, *, producer: str) -> None:
        if self._owner is not None and self._owner != producer:
            raise MotionModeError(
                f"motion runtime is owned by {self._owner}; {producer} cannot mark intent"
            )
        self._owner = producer
        self._mode = MotionMode.AGENT_SERVO
        self._last_intent_frame_s = self._monotonic()
        self._stale_intent_held = False

    def watchdog_tick(
        self,
        *,
        missed_intent_timeout_s: float,
        fault_timeout_s: float,
    ) -> dict[str, str] | None:
        if self._last_intent_frame_s is None:
            return None
        age_s = self._monotonic() - self._last_intent_frame_s
        if age_s >= fault_timeout_s:
            self._damping()
            return {"landing_mode": MotionMode.DAMPING.value}
        if not self._stale_intent_held and age_s >= missed_intent_timeout_s:
            landing_mode = self._hold()
            self._stale_intent_held = True
            return {"landing_mode": landing_mode}
        return None

    def _release_mode(self, *, owner: str, mode: MotionMode) -> None:
        if self._owner == owner and self._mode == mode:
            self._owner = None
            self._mode = MotionMode.HOLD

    def _hold(self) -> str:
        landing_mode = self._backend.hold()
        if landing_mode not in {MotionMode.HOLD.value, MotionMode.DAMPING.value}:
            landing_mode = MotionMode.HOLD.value
        self._owner = None
        self._mode = MotionMode(landing_mode)
        return landing_mode

    def _damping(self) -> None:
        self._backend.damping()
        self._owner = None
        self._mode = MotionMode.DAMPING


@dataclass(frozen=True)
class ArmOwnerLease:
    owner: str
    mode: str
    runtime_session_id: str
    _runtime: ArmRuntime
    _released: bool = False

    def release(self) -> None:
        if self._released:
            return
        object.__setattr__(self, "_released", True)
        self._runtime.release_owner(owner=self.owner)

    def heartbeat(self) -> None:
        if self._released:
            raise ArmRuntimeError("owner lease is already released")
        self._runtime.owner_heartbeat(owner=self.owner)


class ArmRuntime:
    """Long-lived arm state machine around MotionRuntime and one hardware backend."""

    def __init__(
        self,
        *,
        backend: MotionBackend,
        safe_center: tuple[float, ...],
        passive_safe_q: tuple[float, ...] | None = None,
        runtime_session_id: str | None = None,
        monotonic: Callable[[], float] = default_monotonic,
        sleep: Callable[[float], None] = default_sleep,
    ) -> None:
        self._backend = backend
        self._safe_center = tuple(float(value) for value in safe_center)
        self._passive_safe_q = (
            tuple(float(value) for value in passive_safe_q)
            if passive_safe_q is not None
            else None
        )
        self._runtime_session_id = runtime_session_id or str(uuid4())
        self._monotonic = monotonic
        self._sleep = sleep
        self._mode: str = ArmRuntimeMode.PASSIVE_SAFE.value
        self._owner: str | None = None
        self._owner_mode: MotionMode | None = None
        self._owner_heartbeat_timeout_s: float | None = None
        self._last_owner_heartbeat_s: float | None = None
        self._last_heartbeat_s: float = self._monotonic()
        self._q_hold: tuple[float, ...] | None = None

    def arm_from_passive(self, *, max_passive_error_rad: float = 0.05) -> None:
        if self._passive_safe_q is None:
            self._last_heartbeat_s = self._monotonic()
            return
        q_meas = self._read_q_meas()
        _raise_if_not_close(
            q_meas,
            self._passive_safe_q,
            max_error_rad=float(max_passive_error_rad),
            message="current q_meas is not close to passive-safe droop pose",
        )
        self._last_heartbeat_s = self._monotonic()

    def recover_to_safe(
        self,
        *,
        send_hz: float,
        max_joint_step_rad: float,
    ) -> MotionExecutionResult:
        q_start = self._read_q_meas()
        trajectory = _linear_recovery_trajectory(
            q_start=q_start,
            q_target=self._safe_center,
            send_hz=float(send_hz),
            max_joint_step_rad=float(max_joint_step_rad),
        )
        motion = MotionRuntime(
            backend=self._backend,
            monotonic=self._monotonic,
            sleep=self._sleep,
        )
        result = motion.execute_trajectory(
            trajectory,
            producer="runtime_recovery",
            trajectory_sample_hz=float(send_hz),
            hold_after=True,
        )
        if result.status == "completed":
            self.mark_hold_safe(q_hold=self._safe_center)
        else:
            self._mode = MotionMode.DAMPING.value
            self._owner = None
        return result

    def mark_hold_safe(self, *, q_hold: tuple[float, ...]) -> None:
        self._q_hold = tuple(float(value) for value in q_hold)
        self._owner = None
        self._owner_mode = None
        self._owner_heartbeat_timeout_s = None
        self._last_owner_heartbeat_s = None
        self._mode = ArmRuntimeMode.HOLD_SAFE.value
        self._last_heartbeat_s = self._monotonic()

    def acquire_owner(
        self,
        *,
        owner: str,
        mode: MotionMode,
        expected_q_start: tuple[float, ...],
        max_start_error_rad: float,
        heartbeat_timeout_s: float,
    ) -> ArmOwnerLease:
        if self._owner is not None:
            raise ArmRuntimeError(f"runtime is owned by {self._owner}")
        if self._mode != ArmRuntimeMode.HOLD_SAFE.value:
            raise ArmRuntimeError(f"runtime must be hold_safe before acquire, got {self._mode}")
        if heartbeat_timeout_s <= 0.0:
            raise ValueError("heartbeat_timeout_s must be positive")
        q_meas = self._read_q_meas()
        _raise_if_not_close(
            q_meas,
            tuple(float(value) for value in expected_q_start),
            max_error_rad=float(max_start_error_rad),
            message="current q_meas is not close to expected start pose",
        )
        self._owner = str(owner)
        self._owner_mode = mode
        self._mode = mode.value
        self._owner_heartbeat_timeout_s = float(heartbeat_timeout_s)
        self._last_owner_heartbeat_s = self._monotonic()
        self._last_heartbeat_s = self._last_owner_heartbeat_s
        return ArmOwnerLease(
            owner=str(owner),
            mode=mode.value,
            runtime_session_id=self._runtime_session_id,
            _runtime=self,
        )

    def owner_heartbeat(self, *, owner: str) -> None:
        if self._owner != owner:
            raise ArmRuntimeError(f"runtime is not owned by {owner}")
        now_s = self._monotonic()
        self._last_owner_heartbeat_s = now_s
        self._last_heartbeat_s = now_s

    def release_owner(self, *, owner: str) -> None:
        q_meas = self._read_q_meas()
        self._release_owner_to_hold(owner=owner, q_hold=q_meas)

    def _release_owner_to_hold(
        self,
        *,
        owner: str,
        q_hold: tuple[float, ...],
    ) -> None:
        if self._owner != owner:
            raise ArmRuntimeError(f"runtime is not owned by {owner}")
        self.mark_hold_safe(q_hold=q_hold)
        self._backend.hold()

    def watchdog_tick(self) -> dict[str, str] | None:
        if self._owner is None or self._owner_heartbeat_timeout_s is None:
            return None
        last_heartbeat_s = self._last_owner_heartbeat_s
        if last_heartbeat_s is None:
            return None
        if self._monotonic() - last_heartbeat_s < self._owner_heartbeat_timeout_s:
            return None
        owner = self._owner
        self._backend.damping()
        self._owner = None
        self._owner_mode = None
        self._owner_heartbeat_timeout_s = None
        self._last_owner_heartbeat_s = None
        self._mode = MotionMode.DAMPING.value
        self._last_heartbeat_s = self._monotonic()
        return {
            "owner": owner,
            "landing_mode": MotionMode.DAMPING.value,
            "reason": "owner_heartbeat_timeout",
        }

    def execute_owner_trajectory(
        self,
        trajectory: Iterable[JointTrajectoryPoint],
        *,
        owner: str,
        trajectory_sample_hz: float,
        watchdog: Callable[[], dict[str, object] | None] | None = None,
        on_sample: Callable[[MotionAuditSample], None] | None = None,
        max_tracking_error_rad: float | None = None,
        tracking_error_grace_samples: int = 0,
        tracking_error_consecutive_samples: int = 1,
        max_tau_abs: float | None = None,
    ) -> MotionExecutionResult:
        self._raise_if_not_owned(owner=owner, mode=MotionMode.TRAJECTORY_REPLAY)
        motion = MotionRuntime(
            backend=self._backend,
            monotonic=self._monotonic,
            sleep=self._sleep,
        )
        result = motion.execute_trajectory(
            trajectory,
            producer=owner,
            trajectory_sample_hz=trajectory_sample_hz,
            hold_after=False,
            watchdog=watchdog,
            on_sample=on_sample,
            max_tracking_error_rad=max_tracking_error_rad,
            tracking_error_grace_samples=tracking_error_grace_samples,
            tracking_error_consecutive_samples=tracking_error_consecutive_samples,
            max_tau_abs=max_tau_abs,
        )
        return self._finish_owner_motion(owner=owner, result=result)

    def execute_owner_intent_frame(
        self,
        intent: JointIntentFrame,
        *,
        owner: str,
        send_hz: float,
        watchdog: Callable[[], dict[str, object] | None] | None = None,
        on_sample: Callable[[MotionAuditSample], None] | None = None,
        max_tracking_error_rad: float | None = None,
        tracking_error_grace_samples: int = 0,
        tracking_error_consecutive_samples: int = 1,
        max_tau_abs: float | None = None,
    ) -> MotionExecutionResult:
        self._raise_if_not_owned(owner=owner, mode=MotionMode.AGENT_SERVO)
        motion = MotionRuntime(
            backend=self._backend,
            monotonic=self._monotonic,
            sleep=self._sleep,
        )
        result = motion.execute_intent_frame(
            intent,
            producer=owner,
            send_hz=send_hz,
            hold_after=False,
            watchdog=watchdog,
            on_sample=on_sample,
            max_tracking_error_rad=max_tracking_error_rad,
            tracking_error_grace_samples=tracking_error_grace_samples,
            tracking_error_consecutive_samples=tracking_error_consecutive_samples,
            max_tau_abs=max_tau_abs,
        )
        return self._finish_owner_motion(owner=owner, result=result)

    def status(self) -> dict[str, object]:
        q_meas = self._read_q_meas()
        now_s = self._monotonic()
        fault_flags = self._backend.read_joint_state().fault_flags
        return {
            "schema": "armctrl.arm_runtime_status.v1",
            "runtime_session_id": self._runtime_session_id,
            "mode": self._mode,
            "owner": self._owner,
            "q_meas": q_meas,
            "q_hold": self._q_hold,
            "safe_center": self._safe_center,
            "controller_dt_s": getattr(self._backend, "controller_dt_s", None),
            "heartbeat_monotonic_s": self._last_heartbeat_s,
            "heartbeat_age_s": max(0.0, now_s - self._last_heartbeat_s),
            "fault_flags": tuple(fault_flags),
        }

    def _raise_if_not_owned(self, *, owner: str, mode: MotionMode) -> None:
        if self._owner != owner:
            raise ArmRuntimeError(f"runtime is not owned by {owner}")
        if self._owner_mode != mode or self._mode != mode.value:
            raise ArmRuntimeError(
                f"runtime owner mode must be {mode.value}, got {self._mode}"
            )

    def _finish_owner_motion(
        self,
        *,
        owner: str,
        result: MotionExecutionResult,
    ) -> MotionExecutionResult:
        if result.status == "completed":
            final_q_cmd = (
                result.samples[-1].q_cmd
                if result.samples
                else self._read_q_meas()
            )
            self._release_owner_to_hold(owner=owner, q_hold=final_q_cmd)
            return replace(result, landing_mode=MotionMode.HOLD.value)
        if result.landing_mode == MotionMode.HOLD.value:
            q_hold = (
                result.samples[-1].q_meas
                if result.samples and result.samples[-1].q_meas
                else self._read_q_meas()
            )
            self._release_owner_to_hold(owner=owner, q_hold=q_hold)
            return replace(result, landing_mode=MotionMode.HOLD.value)
        self._owner = None
        self._owner_mode = None
        self._owner_heartbeat_timeout_s = None
        self._last_owner_heartbeat_s = None
        self._mode = MotionMode.DAMPING.value
        self._last_heartbeat_s = self._monotonic()
        return result

    def _read_q_meas(self) -> tuple[float, ...]:
        return tuple(float(value) for value in self._backend.read_joint_state().q_meas)


def _actual_send_hz(sent_times: Sequence[float]) -> float | None:
    if len(sent_times) < 2:
        return None
    elapsed_s = sent_times[-1] - sent_times[0]
    if elapsed_s <= 0.0:
        return None
    return (len(sent_times) - 1) / elapsed_s


def _raise_if_not_close(
    q_meas: tuple[float, ...],
    q_expected: tuple[float, ...],
    *,
    max_error_rad: float,
    message: str,
) -> None:
    if len(q_meas) != len(q_expected):
        raise ArmRuntimeError(
            f"{message}: length mismatch {len(q_meas)} != {len(q_expected)}"
        )
    if max_error_rad < 0.0:
        raise ValueError("max_error_rad must be non-negative")
    max_abs_error = max(
        (abs(measured - expected) for measured, expected in zip(q_meas, q_expected)),
        default=0.0,
    )
    if max_abs_error > max_error_rad:
        raise ArmRuntimeError(
            f"{message}: max_abs_error_rad={max_abs_error:.6f} "
            f"> max_error_rad={max_error_rad:.6f}"
        )


def _linear_recovery_trajectory(
    *,
    q_start: tuple[float, ...],
    q_target: tuple[float, ...],
    send_hz: float,
    max_joint_step_rad: float,
) -> list[JointTrajectoryPoint]:
    if send_hz <= 0.0:
        raise ValueError("send_hz must be positive")
    if max_joint_step_rad <= 0.0:
        raise ValueError("max_joint_step_rad must be positive")
    if len(q_start) != len(q_target):
        raise ValueError("q_start and q_target lengths must match")
    max_delta = max(
        (abs(target - start) for start, target in zip(q_start, q_target)),
        default=0.0,
    )
    step_count = max(1, int(math.ceil(max_delta / max_joint_step_rad)))
    return [
        JointTrajectoryPoint(
            time_s=step_index / float(send_hz),
            q=tuple(
                start + (target - start) * (step_index / step_count)
                for start, target in zip(q_start, q_target)
            ),
        )
        for step_index in range(step_count + 1)
    ]


def _motion_execution_result(
    *,
    status: str,
    producer: str,
    mode: MotionMode,
    trajectory_sample_hz: float,
    sent_times: Sequence[float],
    expected_period_s: float,
    controller_dt_s: float | None,
    samples: Sequence[MotionAuditSample],
    landing_mode: str,
    error: dict[str, str] | None = None,
) -> MotionExecutionResult:
    return MotionExecutionResult(
        status=status,
        producer=producer,
        mode=mode.value,
        trajectory_sample_hz=trajectory_sample_hz,
        actual_send_hz=_actual_send_hz(sent_times),
        send_jitter_ms_p95=_send_jitter_ms_percentile(
            sent_times,
            expected_period_s=expected_period_s,
            percentile=0.95,
        ),
        send_jitter_ms_p99=_send_jitter_ms_percentile(
            sent_times,
            expected_period_s=expected_period_s,
            percentile=0.99,
        ),
        controller_dt_s=controller_dt_s,
        samples=tuple(samples),
        landing_mode=landing_mode,
        error=error,
    )


def _motion_error(exc: Exception) -> dict[str, str]:
    return {"type": exc.__class__.__name__, "message": str(exc)}


def _watchdog_error(event: dict[str, object]) -> dict[str, str]:
    reason = str(event.get("reason") or "watchdog_timeout")
    return {"type": "watchdog", "message": reason}


def _tracking_error_limit_error(*, debounced: bool = False) -> dict[str, str]:
    message = (
        "max tracking error exceeded after debounce"
        if debounced
        else "max tracking error exceeded"
    )
    return {"type": "tracking_error", "message": message}


def _tau_limit_error() -> dict[str, str]:
    return {"type": "torque_limit", "message": "max absolute tau exceeded"}


def _tracking_error_exceeded(
    sample: MotionAuditSample,
    *,
    max_tracking_error_rad: float | None,
) -> bool:
    if max_tracking_error_rad is None:
        return False
    threshold = float(max_tracking_error_rad)
    if threshold <= 0.0:
        return False
    errors = (
        abs(float(measured) - float(commanded))
        for commanded, measured in zip(sample.q_cmd, sample.q_meas, strict=False)
    )
    return any(error > threshold for error in errors)


def _tau_limit_exceeded(
    sample: MotionAuditSample,
    *,
    max_tau_abs: float | None,
) -> bool:
    if max_tau_abs is None:
        return False
    threshold = float(max_tau_abs)
    if threshold <= 0.0:
        return False
    return any(abs(float(value)) > threshold for value in sample.tau_meas)


def _validate_trajectory_inputs(
    points: Sequence[JointTrajectoryPoint],
    *,
    trajectory_sample_hz: float,
) -> None:
    if not math.isfinite(trajectory_sample_hz) or trajectory_sample_hz <= 0.0:
        raise ValueError("trajectory_sample_hz must be positive")
    previous_time_s: float | None = None
    expected_dof: int | None = None
    for index, point in enumerate(points):
        time_s = float(point.time_s)
        if not math.isfinite(time_s):
            raise ValueError("trajectory timestamps must be finite")
        if time_s < 0.0 or (
            previous_time_s is not None and time_s < previous_time_s
        ):
            raise ValueError("trajectory timestamps must be nonnegative and monotonic")
        q = tuple(point.q)
        if expected_dof is None:
            expected_dof = len(q)
        elif len(q) != expected_dof:
            raise ValueError("trajectory q vectors must have consistent length")
        if point.dq is not None and len(tuple(point.dq)) != expected_dof:
            raise ValueError("trajectory dq vectors must match q length")
        for value in q:
            if not math.isfinite(float(value)):
                raise ValueError("trajectory q values must be finite")
        if point.dq is not None:
            for value in point.dq:
                if not math.isfinite(float(value)):
                    raise ValueError("trajectory dq values must be finite")
        previous_time_s = time_s


def _intent_frame_points(
    intent: JointIntentFrame,
    *,
    send_hz: float,
) -> list[JointTrajectoryPoint]:
    if intent.control_period_s <= 0.0:
        raise ValueError("control_period_s must be positive")
    q_start = tuple(float(value) for value in intent.q_start)
    q_target = tuple(float(value) for value in intent.q_target)
    if len(q_start) != len(q_target):
        raise ValueError("q_start and q_target must have the same length")
    observed_delta_rad = max(
        (abs(target - start) for start, target in zip(q_start, q_target)),
        default=0.0,
    )
    if intent.max_joint_delta_rad is not None:
        max_joint_delta_rad = float(intent.max_joint_delta_rad)
        if max_joint_delta_rad <= 0.0:
            raise ValueError("max_joint_delta_rad must be positive")
        if observed_delta_rad > max_joint_delta_rad:
            raise ValueError(
                "max_joint_delta_rad exceeded: "
                f"{observed_delta_rad:.6g} rad > {max_joint_delta_rad:.6g} rad"
            )
    if intent.max_joint_velocity_rad_s is not None:
        max_joint_velocity_rad_s = float(intent.max_joint_velocity_rad_s)
        if max_joint_velocity_rad_s <= 0.0:
            raise ValueError("max_joint_velocity_rad_s must be positive")
        observed_velocity_rad_s = observed_delta_rad / float(intent.control_period_s)
        if observed_velocity_rad_s > max_joint_velocity_rad_s:
            raise ValueError(
                "max_joint_velocity_rad_s exceeded: "
                f"{observed_velocity_rad_s:.6g} rad/s > "
                f"{max_joint_velocity_rad_s:.6g} rad/s"
            )
    if intent.max_joint_acceleration_rad_s2 is not None:
        max_joint_acceleration_rad_s2 = float(intent.max_joint_acceleration_rad_s2)
        if max_joint_acceleration_rad_s2 <= 0.0:
            raise ValueError("max_joint_acceleration_rad_s2 must be positive")
        observed_acceleration_rad_s2 = (
            6.0 * observed_delta_rad / (float(intent.control_period_s) ** 2)
        )
        if observed_acceleration_rad_s2 > max_joint_acceleration_rad_s2:
            raise ValueError(
                "max_joint_acceleration_rad_s2 exceeded: "
                f"{observed_acceleration_rad_s2:.6g} rad/s^2 > "
                f"{max_joint_acceleration_rad_s2:.6g} rad/s^2"
            )
    interval_count = max(1, int(round(float(intent.control_period_s) * send_hz)))
    points: list[JointTrajectoryPoint] = []
    deltas = tuple(target - start for start, target in zip(q_start, q_target))
    for index in range(interval_count + 1):
        linear_ratio = index / interval_count
        ratio = linear_ratio * linear_ratio * (3.0 - 2.0 * linear_ratio)
        ratio_derivative = 6.0 * linear_ratio * (1.0 - linear_ratio)
        q_cmd = tuple(
            start + delta * ratio for start, delta in zip(q_start, deltas)
        )
        points.append(
            JointTrajectoryPoint(
                time_s=index / send_hz,
                q=q_cmd,
                dq=tuple(
                    delta * ratio_derivative / float(intent.control_period_s)
                    for delta in deltas
                ),
            )
        )
    return points


def _send_jitter_ms_percentile(
    sent_times: Sequence[float],
    *,
    expected_period_s: float,
    percentile: float,
) -> float | None:
    if len(sent_times) < 2:
        return None
    jitters_ms = [
        abs((sent_times[index] - sent_times[index - 1]) - expected_period_s) * 1000.0
        for index in range(1, len(sent_times))
    ]
    if not jitters_ms:
        return None
    jitters_ms.sort()
    percentile_index = max(0, int(percentile * (len(jitters_ms) - 1)))
    return jitters_ms[percentile_index]
