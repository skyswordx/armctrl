import pytest

from armctrl.motion_runtime import (
    ArmRuntime,
    ArmRuntimeError,
    ArmRuntimeMode,
    FakeMotionBackend,
    JointStateSnapshot,
    JointIntentFrame,
    JointTrajectoryPoint,
    MotionMode,
    MotionModeError,
    MotionRuntime,
)


class ManualClock:
    def __init__(self) -> None:
        self.now_s = 0.0

    def monotonic(self) -> float:
        return self.now_s

    def sleep(self, seconds: float) -> None:
        self.now_s += max(0.0, seconds)


class InterruptingMotionBackend(FakeMotionBackend):
    def send_joint_command(self, *args, **kwargs) -> None:
        raise KeyboardInterrupt()


class FailingSendBackend(FakeMotionBackend):
    def __init__(self, *, fail_on_send: int) -> None:
        super().__init__()
        self._fail_on_send = fail_on_send
        self._send_count = 0

    def send_joint_command(self, *args, **kwargs) -> None:
        self._send_count += 1
        if self._send_count >= self._fail_on_send:
            raise RuntimeError("sdk send failed")
        super().send_joint_command(*args, **kwargs)


class CapturingVelocityBackend(FakeMotionBackend):
    def __init__(self) -> None:
        super().__init__()
        self.dq_commands: list[tuple[float, ...] | None] = []

    def send_joint_command(self, *args, **kwargs) -> None:
        self.dq_commands.append(kwargs.get("dq"))
        super().send_joint_command(*args, **kwargs)


class FaultingReadBackend(FakeMotionBackend):
    def __init__(self, *, fault_on_read: int) -> None:
        super().__init__()
        self._fault_on_read = fault_on_read
        self._read_count = 0

    def read_joint_state(self) -> JointStateSnapshot:
        self._read_count += 1
        if self._read_count >= self._fault_on_read:
            return JointStateSnapshot(
                q_meas=self._last_q or (),
                fault_flags=("over_current",),
            )
        return super().read_joint_state()


def test_execute_trajectory_replays_timestamped_points_and_reports_send_metrics():
    clock = ManualClock()
    backend = FakeMotionBackend()
    runtime = MotionRuntime(backend=backend, monotonic=clock.monotonic, sleep=clock.sleep)
    trajectory = [
        JointTrajectoryPoint(time_s=0.0, q=(0.0, 0.0)),
        JointTrajectoryPoint(time_s=0.01, q=(0.01, 0.0)),
        JointTrajectoryPoint(time_s=0.02, q=(0.02, 0.0)),
    ]

    result = runtime.execute_trajectory(
        trajectory,
        producer="sysid",
        trajectory_sample_hz=100.0,
        hold_after=True,
    )

    assert [command.q for command in backend.joint_commands] == [
        (0.0, 0.0),
        (0.01, 0.0),
        (0.02, 0.0),
    ]
    assert [round(command.sent_monotonic_s, 6) for command in backend.joint_commands] == [
        0.0,
        0.01,
        0.02,
    ]
    assert result.status == "completed"
    assert result.producer == "sysid"
    assert result.mode == MotionMode.TRAJECTORY_REPLAY.value
    assert result.trajectory_sample_hz == 100.0
    assert result.actual_send_hz == pytest.approx(100.0)
    assert result.send_jitter_ms_p95 == pytest.approx(0.0)
    assert result.send_jitter_ms_p99 == pytest.approx(0.0)
    assert result.controller_dt_s is None
    assert [sample.q_cmd for sample in result.samples] == [
        (0.0, 0.0),
        (0.01, 0.0),
        (0.02, 0.0),
    ]
    assert [sample.q_meas for sample in result.samples] == [
        (0.0, 0.0),
        (0.01, 0.0),
        (0.02, 0.0),
    ]
    assert [sample.fault_flags for sample in result.samples] == [(), (), ()]
    assert result.landing_mode == "hold"
    assert backend.hold_count == 1
    assert runtime.mode == MotionMode.HOLD


def test_execute_trajectory_preserves_velocity_commands_for_backend():
    clock = ManualClock()
    backend = CapturingVelocityBackend()
    runtime = MotionRuntime(backend=backend, monotonic=clock.monotonic, sleep=clock.sleep)
    trajectory = [
        JointTrajectoryPoint(time_s=0.0, q=(0.0, 0.0), dq=(0.1, 0.0)),
        JointTrajectoryPoint(time_s=0.01, q=(0.01, 0.0), dq=(0.1, 0.0)),
    ]

    result = runtime.execute_trajectory(
        trajectory,
        producer="sysid",
        trajectory_sample_hz=100.0,
    )

    assert result.status == "completed"
    assert backend.dq_commands == [(0.1, 0.0), (0.1, 0.0)]


def test_execute_trajectory_rejects_nonpositive_sample_hz_before_sending():
    backend = FakeMotionBackend()
    runtime = MotionRuntime(backend=backend)

    with pytest.raises(ValueError, match="trajectory_sample_hz"):
        runtime.execute_trajectory(
            [JointTrajectoryPoint(time_s=0.0, q=(0.0,))],
            producer="sysid",
            trajectory_sample_hz=0.0,
        )

    assert backend.joint_commands == []
    assert backend.damping_count == 0
    assert runtime.mode == MotionMode.HOLD


def test_execute_trajectory_rejects_nonmonotonic_timestamps_before_sending():
    backend = FakeMotionBackend()
    runtime = MotionRuntime(backend=backend)

    with pytest.raises(ValueError, match="monotonic"):
        runtime.execute_trajectory(
            [
                JointTrajectoryPoint(time_s=0.02, q=(0.02,)),
                JointTrajectoryPoint(time_s=0.01, q=(0.01,)),
            ],
            producer="sysid",
            trajectory_sample_hz=100.0,
        )

    assert backend.joint_commands == []
    assert backend.damping_count == 0
    assert runtime.mode == MotionMode.HOLD


def test_execute_intent_frame_interpolates_agent_10hz_frame_to_50hz_backend():
    clock = ManualClock()
    backend = FakeMotionBackend()
    runtime = MotionRuntime(backend=backend, monotonic=clock.monotonic, sleep=clock.sleep)

    result = runtime.execute_intent_frame(
        JointIntentFrame(
            q_start=(0.0, 0.0),
            q_target=(0.1, 0.05),
            control_period_s=0.1,
        ),
        producer="agent",
        send_hz=50.0,
        hold_after=True,
    )

    expected_q = [
        (0.0, 0.0),
        (0.0104, 0.0052),
        (0.0352, 0.0176),
        (0.0648, 0.0324),
        (0.0896, 0.0448),
        (0.1, 0.05),
    ]

    assert [round(command.sent_monotonic_s, 6) for command in backend.joint_commands] == [
        0.0,
        0.02,
        0.04,
        0.06,
        0.08,
        0.1,
    ]
    assert len(backend.joint_commands) == 6
    for command, expected in zip(backend.joint_commands, expected_q, strict=True):
        assert command.q == pytest.approx(expected)
        assert command.producer == "agent"
        assert command.mode == MotionMode.AGENT_SERVO.value
    assert result.status == "completed"
    assert result.producer == "agent"
    assert result.mode == MotionMode.AGENT_SERVO.value
    assert result.trajectory_sample_hz == 50.0
    assert result.actual_send_hz == pytest.approx(50.0)
    assert result.send_jitter_ms_p95 == pytest.approx(0.0)
    assert [sample.mode for sample in result.samples] == [
        MotionMode.AGENT_SERVO.value
    ] * 6
    assert result.landing_mode == "hold"
    assert backend.hold_count == 1
    assert runtime.mode == MotionMode.HOLD


def test_execute_intent_frame_respects_existing_mode_owner():
    clock = ManualClock()
    runtime = MotionRuntime(
        backend=FakeMotionBackend(),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    token = runtime.acquire_mode(MotionMode.TRAJECTORY_REPLAY, producer="sysid")

    with pytest.raises(MotionModeError, match="owned by sysid"):
        runtime.execute_intent_frame(
            JointIntentFrame(
                q_start=(0.0,),
                q_target=(0.1,),
                control_period_s=0.1,
            ),
            producer="agent",
            send_hz=50.0,
        )

    token.release()
    assert runtime.mode == MotionMode.HOLD


def test_execute_intent_frame_rejects_joint_delta_above_gate():
    runtime = MotionRuntime(backend=FakeMotionBackend())

    with pytest.raises(ValueError, match="max_joint_delta_rad"):
        runtime.execute_intent_frame(
            JointIntentFrame(
                q_start=(0.0, 0.0),
                q_target=(0.02, 0.0),
                control_period_s=0.1,
                max_joint_delta_rad=0.005,
            ),
            producer="agent",
            send_hz=50.0,
        )


def test_mode_owner_rejects_competing_producers_until_released():
    clock = ManualClock()
    runtime = MotionRuntime(
        backend=FakeMotionBackend(),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    token = runtime.acquire_mode(MotionMode.AGENT_SERVO, producer="agent")

    with pytest.raises(MotionModeError, match="owned by agent"):
        runtime.execute_trajectory(
            [JointTrajectoryPoint(time_s=0.0, q=(0.0,))],
            producer="sysid",
            trajectory_sample_hz=100.0,
        )

    token.release()
    assert runtime.mode == MotionMode.HOLD


def test_watchdog_holds_stale_agent_intent_then_damps_after_fault_timeout():
    clock = ManualClock()
    backend = FakeMotionBackend()
    runtime = MotionRuntime(backend=backend, monotonic=clock.monotonic, sleep=clock.sleep)

    runtime.mark_intent_frame(producer="agent")
    clock.now_s = 0.31
    hold_event = runtime.watchdog_tick(missed_intent_timeout_s=0.3, fault_timeout_s=1.0)

    assert hold_event["landing_mode"] == "hold"
    assert backend.hold_count == 1
    assert runtime.mode == MotionMode.HOLD

    clock.now_s = 1.01
    damping_event = runtime.watchdog_tick(missed_intent_timeout_s=0.3, fault_timeout_s=1.0)

    assert damping_event["landing_mode"] == "damping"
    assert backend.damping_count == 1
    assert runtime.mode == MotionMode.DAMPING


def test_execute_trajectory_lands_damping_and_stops_on_fault_flags():
    clock = ManualClock()
    backend = FaultingReadBackend(fault_on_read=2)
    runtime = MotionRuntime(backend=backend, monotonic=clock.monotonic, sleep=clock.sleep)

    result = runtime.execute_trajectory(
        [
            JointTrajectoryPoint(time_s=0.0, q=(0.0, 0.0)),
            JointTrajectoryPoint(time_s=0.01, q=(0.01, 0.0)),
            JointTrajectoryPoint(time_s=0.02, q=(0.02, 0.0)),
        ],
        producer="sysid",
        trajectory_sample_hz=100.0,
        hold_after=True,
    )

    assert result.status == "faulted"
    assert result.landing_mode == "damping"
    assert result.actual_send_hz == pytest.approx(100.0)
    assert [command.q for command in backend.joint_commands] == [
        (0.0, 0.0),
        (0.01, 0.0),
    ]
    assert [sample.fault_flags for sample in result.samples] == [
        (),
        ("over_current",),
    ]
    assert backend.hold_count == 0
    assert backend.damping_count == 1
    assert runtime.mode == MotionMode.DAMPING


def test_execute_trajectory_lands_damping_on_watchdog_timeout():
    clock = ManualClock()
    backend = FakeMotionBackend()
    runtime = MotionRuntime(backend=backend, monotonic=clock.monotonic, sleep=clock.sleep)
    watchdog_calls = 0

    def watchdog() -> dict[str, object] | None:
        nonlocal watchdog_calls
        watchdog_calls += 1
        if watchdog_calls >= 2:
            return {
                "reason": "owner_heartbeat_timeout",
                "landing_mode": MotionMode.DAMPING.value,
            }
        return None

    result = runtime.execute_trajectory(
        [
            JointTrajectoryPoint(time_s=0.0, q=(0.0, 0.0)),
            JointTrajectoryPoint(time_s=0.01, q=(0.01, 0.0)),
            JointTrajectoryPoint(time_s=0.02, q=(0.02, 0.0)),
        ],
        producer="sysid",
        trajectory_sample_hz=100.0,
        hold_after=True,
        watchdog=watchdog,
    )

    assert result.status == "faulted"
    assert result.landing_mode == "damping"
    assert result.error == {
        "type": "watchdog",
        "message": "owner_heartbeat_timeout",
    }
    assert [command.q for command in backend.joint_commands] == [(0.0, 0.0)]
    assert len(result.samples) == 1
    assert backend.hold_count == 0
    assert backend.damping_count == 1
    assert runtime.mode == MotionMode.DAMPING


def test_execute_trajectory_tracking_error_aborts_to_hold_not_damping():
    class LaggingReadBackend(FakeMotionBackend):
        def read_joint_state(self) -> JointStateSnapshot:
            return JointStateSnapshot(q_meas=(0.0, 0.0))

    clock = ManualClock()
    backend = LaggingReadBackend()
    runtime = MotionRuntime(backend=backend, monotonic=clock.monotonic, sleep=clock.sleep)

    result = runtime.execute_trajectory(
        [
            JointTrajectoryPoint(time_s=0.0, q=(0.0, 0.0)),
            JointTrajectoryPoint(time_s=0.01, q=(0.01, 0.0)),
            JointTrajectoryPoint(time_s=0.02, q=(0.02, 0.0)),
        ],
        producer="sysid",
        trajectory_sample_hz=100.0,
        hold_after=True,
        max_tracking_error_rad=0.005,
    )

    assert result.status == "aborted"
    assert result.landing_mode == "hold"
    assert result.error == {
        "type": "tracking_error",
        "message": "max tracking error exceeded",
    }
    assert [command.q for command in backend.joint_commands] == [
        (0.0, 0.0),
        (0.01, 0.0),
    ]
    assert backend.hold_count == 1
    assert backend.damping_count == 0
    assert runtime.mode == MotionMode.HOLD


def test_execute_trajectory_returns_aborted_result_on_send_exception():
    clock = ManualClock()
    backend = FailingSendBackend(fail_on_send=2)
    runtime = MotionRuntime(backend=backend, monotonic=clock.monotonic, sleep=clock.sleep)

    result = runtime.execute_trajectory(
        [
            JointTrajectoryPoint(time_s=0.0, q=(0.0, 0.0)),
            JointTrajectoryPoint(time_s=0.01, q=(0.01, 0.0)),
            JointTrajectoryPoint(time_s=0.02, q=(0.02, 0.0)),
        ],
        producer="sysid",
        trajectory_sample_hz=100.0,
        hold_after=True,
    )

    assert result.status == "aborted"
    assert result.landing_mode == "damping"
    assert result.error == {"type": "RuntimeError", "message": "sdk send failed"}
    assert [command.q for command in backend.joint_commands] == [(0.0, 0.0)]
    assert [sample.q_cmd for sample in result.samples] == [(0.0, 0.0)]
    assert backend.hold_count == 0
    assert backend.damping_count == 1
    assert runtime.mode == MotionMode.DAMPING


def test_execute_intent_frame_lands_damping_and_stops_on_fault_flags():
    clock = ManualClock()
    backend = FaultingReadBackend(fault_on_read=2)
    runtime = MotionRuntime(backend=backend, monotonic=clock.monotonic, sleep=clock.sleep)

    result = runtime.execute_intent_frame(
        JointIntentFrame(
            q_start=(0.0, 0.0),
            q_target=(0.1, 0.05),
            control_period_s=0.1,
        ),
        producer="agent",
        send_hz=50.0,
        hold_after=True,
    )

    assert result.status == "faulted"
    assert result.landing_mode == "damping"
    assert result.actual_send_hz == pytest.approx(50.0)
    assert len(backend.joint_commands) == 2
    assert backend.hold_count == 0
    assert backend.damping_count == 1
    assert runtime.mode == MotionMode.DAMPING


def test_execute_intent_frame_tracking_error_aborts_to_hold_not_damping():
    class LaggingReadBackend(FakeMotionBackend):
        def read_joint_state(self) -> JointStateSnapshot:
            return JointStateSnapshot(q_meas=(0.0, 0.0))

    clock = ManualClock()
    backend = LaggingReadBackend()
    runtime = MotionRuntime(backend=backend, monotonic=clock.monotonic, sleep=clock.sleep)

    result = runtime.execute_intent_frame(
        JointIntentFrame(
            q_start=(0.0, 0.0),
            q_target=(0.1, 0.0),
            control_period_s=0.1,
        ),
        producer="agent",
        send_hz=50.0,
        hold_after=True,
        max_tracking_error_rad=0.005,
    )

    assert result.status == "aborted"
    assert result.landing_mode == "hold"
    assert result.error == {
        "type": "tracking_error",
        "message": "max tracking error exceeded",
    }
    assert backend.hold_count == 1
    assert backend.damping_count == 0
    assert runtime.mode == MotionMode.HOLD


def test_execute_intent_frame_lands_damping_on_watchdog_timeout():
    clock = ManualClock()
    backend = FakeMotionBackend()
    runtime = MotionRuntime(backend=backend, monotonic=clock.monotonic, sleep=clock.sleep)
    watchdog_calls = 0

    def watchdog() -> dict[str, object] | None:
        nonlocal watchdog_calls
        watchdog_calls += 1
        if watchdog_calls >= 2:
            return {
                "reason": "owner_heartbeat_timeout",
                "landing_mode": MotionMode.DAMPING.value,
            }
        return None

    result = runtime.execute_intent_frame(
        JointIntentFrame(
            q_start=(0.0, 0.0),
            q_target=(0.1, 0.05),
            control_period_s=0.1,
        ),
        producer="agent",
        send_hz=50.0,
        hold_after=True,
        watchdog=watchdog,
    )

    assert result.status == "faulted"
    assert result.landing_mode == "damping"
    assert result.error == {
        "type": "watchdog",
        "message": "owner_heartbeat_timeout",
    }
    assert len(backend.joint_commands) == 1
    assert len(result.samples) == 1
    assert backend.hold_count == 0
    assert backend.damping_count == 1
    assert runtime.mode == MotionMode.DAMPING


def test_execute_trajectory_lands_damping_on_keyboard_interrupt():
    clock = ManualClock()
    backend = InterruptingMotionBackend()
    runtime = MotionRuntime(backend=backend, monotonic=clock.monotonic, sleep=clock.sleep)

    with pytest.raises(KeyboardInterrupt):
        runtime.execute_trajectory(
            [JointTrajectoryPoint(time_s=0.0, q=(0.0,))],
            producer="sysid",
            trajectory_sample_hz=100.0,
        )

    assert backend.damping_count == 1
    assert runtime.mode == MotionMode.DAMPING


def test_arm_runtime_recovers_from_passive_droop_to_live_hold_session():
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend._last_q = (1.3, 0.0, 0.0, -0.05, 0.0, 0.0)
    runtime = ArmRuntime(
        backend=backend,
        safe_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        passive_safe_q=(1.3, 0.0, 0.0, -0.05, 0.0, 0.0),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    runtime.arm_from_passive()
    result = runtime.recover_to_safe(send_hz=50.0, max_joint_step_rad=0.5)
    status = runtime.status()

    assert result.status == "completed"
    assert status["schema"] == "armctrl.arm_runtime_status.v1"
    assert status["mode"] == ArmRuntimeMode.HOLD_SAFE.value
    assert status["owner"] is None
    assert status["q_hold"] == pytest.approx((0.0, 0.3, 0.3, 0.0, 0.0, 0.0))
    assert status["q_meas"] == pytest.approx((0.0, 0.3, 0.3, 0.0, 0.0, 0.0))
    assert status["heartbeat_age_s"] == pytest.approx(0.0)
    assert status["runtime_session_id"]
    assert backend.hold_count == 1


def test_arm_runtime_requires_fresh_pose_before_owner_acquire():
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend._last_q = (0.0, 0.3, 0.3, 0.0, 0.0, 0.0)
    runtime = ArmRuntime(
        backend=backend,
        safe_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0))

    lease = runtime.acquire_owner(
        owner="agent",
        mode=MotionMode.AGENT_SERVO,
        expected_q_start=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
    )

    assert lease.owner == "agent"
    assert runtime.status()["mode"] == MotionMode.AGENT_SERVO.value
    assert runtime.status()["owner"] == "agent"

    with pytest.raises(ArmRuntimeError, match="owned by agent"):
        runtime.acquire_owner(
            owner="sysid",
            mode=MotionMode.TRAJECTORY_REPLAY,
            expected_q_start=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
            max_start_error_rad=0.02,
            heartbeat_timeout_s=0.5,
        )

    lease.release()
    backend._last_q = (0.5, 0.0, 0.0, 0.0, 0.0, 0.0)

    with pytest.raises(ArmRuntimeError, match="current q_meas is not close"):
        runtime.acquire_owner(
            owner="sysid",
            mode=MotionMode.TRAJECTORY_REPLAY,
            expected_q_start=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
            max_start_error_rad=0.02,
            heartbeat_timeout_s=0.5,
        )

    assert runtime.status()["mode"] == ArmRuntimeMode.HOLD_SAFE.value
    assert runtime.status()["owner"] is None


def test_arm_runtime_deadman_timeout_lands_damping():
    clock = ManualClock()
    backend = FakeMotionBackend()
    backend._last_q = (0.0, 0.3, 0.3, 0.0, 0.0, 0.0)
    runtime = ArmRuntime(
        backend=backend,
        safe_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    runtime.mark_hold_safe(q_hold=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0))
    runtime.acquire_owner(
        owner="xbox",
        mode=MotionMode.AGENT_SERVO,
        expected_q_start=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        max_start_error_rad=0.02,
        heartbeat_timeout_s=0.5,
    )

    clock.now_s = 0.49
    assert runtime.watchdog_tick() is None
    clock.now_s = 0.51
    event = runtime.watchdog_tick()

    assert event == {
        "owner": "xbox",
        "landing_mode": "damping",
        "reason": "owner_heartbeat_timeout",
    }
    assert runtime.status()["mode"] == MotionMode.DAMPING.value
    assert runtime.status()["owner"] is None
    assert backend.damping_count == 1
