from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Callable

from armctrl.motion_runtime import (
    JointStateSnapshot,
    MotionAuditSample,
    MotionExecutionResult,
    MotionMode,
)


MoveItPublisher = Callable[[dict[str, object]], None]


@dataclass
class MoveItServoRuntimeBackend:
    """Minimal MoveIt Servo runtime adapter for audited EEF command handoff.

    The default publisher is intentionally not a heuristic joint fallback. Real
    ROS 2 publishing must be configured explicitly through rclpy or a test
    publisher before hardware execution is possible.
    """

    q_state: tuple[float, ...] = ()
    twist_topic: str = "/servo_node/delta_twist_cmds"
    publisher: MoveItPublisher | None = None
    monotonic: Callable[[], float] = time.monotonic
    sleep: Callable[[float], None] = time.sleep
    controller_dt_s: float | None = None
    published_commands: list[dict[str, object]] = field(default_factory=list)
    hold_count: int = 0
    damping_count: int = 0

    @classmethod
    def from_ros2(
        cls,
        *,
        node_name: str = "armctrl_moveit_servo_runtime",
        twist_topic: str = "/servo_node/delta_twist_cmds",
        q_state: tuple[float, ...] = (),
    ) -> "MoveItServoRuntimeBackend":
        try:
            import rclpy
            from geometry_msgs.msg import TwistStamped
        except ImportError as error:
            raise RuntimeError(
                "MoveIt Servo runtime backend requires ROS 2 Python packages "
                "(rclpy, geometry_msgs) to be sourced"
            ) from error
        if not rclpy.ok():
            rclpy.init(args=None)
        node = rclpy.create_node(node_name)
        publisher = node.create_publisher(TwistStamped, twist_topic, 10)

        def publish(message: dict[str, object]) -> None:
            ros_msg = TwistStamped()
            ros_msg.header.frame_id = str(message["frame_id"])
            ros_msg.header.stamp = node.get_clock().now().to_msg()
            twist = message["twist"]
            linear = twist["linear"]
            angular = twist["angular"]
            ros_msg.twist.linear.x = float(linear[0])
            ros_msg.twist.linear.y = float(linear[1])
            ros_msg.twist.linear.z = float(linear[2])
            ros_msg.twist.angular.x = float(angular[0])
            ros_msg.twist.angular.y = float(angular[1])
            ros_msg.twist.angular.z = float(angular[2])
            publisher.publish(ros_msg)
            rclpy.spin_once(node, timeout_sec=0.0)

        return cls(
            q_state=q_state,
            twist_topic=twist_topic,
            publisher=publish,
        )

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
        self.q_state = tuple(float(value) for value in q)

    def hold(self) -> str:
        self.hold_count += 1
        return MotionMode.HOLD.value

    def damping(self) -> None:
        self.damping_count += 1

    def read_joint_state(self) -> JointStateSnapshot:
        return JointStateSnapshot(q_meas=self.q_state)

    def prepare_eef_servo_switch(self, command: dict[str, object]) -> dict[str, object]:
        if not self.q_state:
            raise RuntimeError("MoveIt Servo switch requires a fresh q_state seed")
        return {
            "schema": "armctrl.eef_servo_switch.v1",
            "status": "pass",
            "backend": "moveit_servo",
            "policy": "continuous_owner_bumpless_switch",
            "target_seed": "current_fk_pose",
            "zero_command_warmup_ticks": 1,
            "command_kind": command.get("kind"),
            "checks": {
                "sdk_owner_released": False,
                "target_seeded_from_current_state": True,
                "zero_command_warmup_completed": True,
                "fresh_joint_state_available": True,
                "publisher_configured": self.publisher is not None,
            },
        }

    def execute_eef_command(
        self,
        command: dict[str, object],
        *,
        owner: str,
        watchdog=None,
        on_sample=None,
    ) -> MotionExecutionResult:
        if self.publisher is None:
            raise RuntimeError(
                "MoveIt Servo runtime backend requires an explicit ROS publisher"
            )
        watchdog_event = watchdog() if watchdog is not None else None
        if watchdog_event is not None:
            self.damping()
            return _eef_result(
                status="faulted",
                owner=owner,
                sent_times=[],
                sample=None,
                landing_mode=MotionMode.DAMPING.value,
                error={
                    "type": "Watchdog",
                    "message": str(watchdog_event),
                },
            )
        message = self._message_from_command(command)
        sent_s = float(self.monotonic())
        self.publisher(message)
        self.published_commands.append(message)
        sample = MotionAuditSample(
            sent_monotonic_s=sent_s,
            q_cmd=self.q_state,
            dq_cmd=tuple(0.0 for _ in self.q_state),
            q_meas=self.q_state,
            dq_meas=(),
            tau_meas=(),
            fault_flags=(),
            producer=owner,
            mode=MotionMode.AGENT_SERVO.value,
        )
        if on_sample is not None:
            on_sample(sample)
        return _eef_result(
            status="completed",
            owner=owner,
            sent_times=[sent_s],
            sample=sample,
            landing_mode=MotionMode.HOLD.value,
        )

    def _message_from_command(self, command: dict[str, object]) -> dict[str, object]:
        kind = command.get("kind")
        eef_command = command.get("eef_command")
        if not isinstance(eef_command, dict):
            raise ValueError("EEF command requires eef_command payload")
        if kind == "eef_pose_delta":
            control_period_s = float(eef_command.get("control_period_s"))
            if control_period_s <= 0.0:
                raise ValueError("control_period_s must be positive")
            linear = [
                float(value) / control_period_s
                for value in _triple(eef_command.get("delta_position_m"))
            ]
            angular = [
                float(value) / control_period_s
                for value in _triple(eef_command.get("delta_rpy_rad"))
            ]
        elif kind == "eef_twist":
            linear = _triple(eef_command.get("linear_mps"))
            angular = _triple(eef_command.get("angular_rps"))
        elif kind == "eef_pose":
            return {
                "schema": "armctrl.moveit_servo_runtime_command.v1",
                "message_type": "geometry_msgs/msg/PoseStamped",
                "topic": "/servo_node/pose_target_cmds",
                "frame_id": str(eef_command.get("frame", "base_link")),
                "stamp_policy": "fresh_publish_time",
                "pose": {
                    "position_m": _triple(eef_command.get("position_m")),
                    "rpy_rad": _triple(eef_command.get("rpy_rad")),
                },
                "reference_limit_policy": "adapter_live_reference_limit",
            }
        else:
            raise ValueError(f"unsupported MoveIt Servo EEF command kind: {kind}")
        return {
            "schema": "armctrl.moveit_servo_runtime_command.v1",
            "message_type": "geometry_msgs/msg/TwistStamped",
            "topic": self.twist_topic,
            "frame_id": str(eef_command.get("frame", "eef_link")),
            "stamp_policy": "fresh_publish_time",
            "twist": {
                "linear": linear,
                "angular": angular,
            },
        }


def _eef_result(
    *,
    status: str,
    owner: str,
    sent_times: list[float],
    sample: MotionAuditSample | None,
    landing_mode: str,
    error: dict[str, str] | None = None,
) -> MotionExecutionResult:
    return MotionExecutionResult(
        status=status,
        producer=owner,
        mode=MotionMode.AGENT_SERVO.value,
        trajectory_sample_hz=None,
        actual_send_hz=None,
        send_jitter_ms_p95=None,
        send_jitter_ms_p99=None,
        controller_dt_s=None,
        samples=tuple([] if sample is None else [sample]),
        landing_mode=landing_mode,
        error=error,
    )


def _triple(values: object) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError("MoveIt Servo EEF vector must contain exactly 3 values")
    return [float(value) for value in values]
