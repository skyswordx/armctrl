from __future__ import annotations

import importlib

from armctrl.protocol.enums import ArmMode, CommandStatus, DebugProfileName, ErrorCode
from armctrl.protocol.errors import ArmctrlError
from armctrl.protocol.models import (
    CommandResponse,
    DebugProfileRequest,
    EEFStateModel,
    JointStateModel,
    MoveEEFRequest,
    RobotState,
)


class Arx5SDKAdapter:
    name = "sdk"

    def __init__(
        self,
        model: str = "X5",
        interface: str = "can0",
        gravity_compensation: bool | None = None,
        log_level: str = "WARNING",
    ) -> None:
        self.model = model
        self.interface = interface
        self.gravity_compensation = gravity_compensation
        self.log_level = log_level
        self._sdk = None
        self._controller = None
        self._mode = ArmMode.DISCONNECTED
        self._last_error: ArmctrlError | None = None

    def connect(self) -> CommandResponse:
        try:
            self._sdk = importlib.import_module("arx5_interface")
            robot_config = self._sdk.RobotConfigFactory.get_instance().get_config(self.model)
            controller_config = self._sdk.ControllerConfigFactory.get_instance().get_config(
                "cartesian_controller",
                robot_config.joint_dof,
            )
            if self.gravity_compensation is not None:
                controller_config.gravity_compensation = self.gravity_compensation
            self._controller = self._sdk.Arx5CartesianController(
                robot_config,
                controller_config,
                self.interface,
            )
            if hasattr(self._sdk, "LogLevel") and hasattr(self._controller, "set_log_level"):
                level = getattr(self._sdk.LogLevel, self.log_level, None)
                if level is not None:
                    self._controller.set_log_level(level)
            self._mode = ArmMode.IDLE
            return CommandResponse(CommandStatus.COMPLETED, "sdk adapter connected", state=self.get_state())
        except ModuleNotFoundError:
            self._last_error = ArmctrlError(
                ErrorCode.SDK_UNAVAILABLE,
                "arx5_interface is not importable in this environment",
            )
            return CommandResponse(CommandStatus.FAULTED, "sdk unavailable", error=self._last_error)
        except Exception as exc:
            self._last_error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            self._mode = ArmMode.FAULTED
            return CommandResponse(CommandStatus.FAULTED, "sdk connect failed", error=self._last_error)

    def _require_controller(self):
        if self._controller is None:
            response = self.connect()
            if response.status is not CommandStatus.COMPLETED:
                raise self._last_error or ArmctrlError(ErrorCode.SDK_ERROR, "SDK controller unavailable")
        return self._controller

    def get_state(self) -> RobotState:
        if self._controller is None:
            zero_joint = JointStateModel(
                pos=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                vel=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                torque=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            )
            zero_eef = EEFStateModel((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            return RobotState(self._mode, zero_joint, zero_eef, False, self.name, self._last_error)
        controller = self._controller
        joint_state = controller.get_joint_state()
        eef_state = controller.get_eef_state()
        joint = JointStateModel(
            pos=tuple(float(value) for value in joint_state.pos()),
            vel=tuple(float(value) for value in joint_state.vel()),
            torque=tuple(float(value) for value in joint_state.torque()),
            gripper_pos=float(joint_state.gripper_pos),
            gripper_vel=float(joint_state.gripper_vel),
            gripper_torque=float(joint_state.gripper_torque),
            timestamp=float(joint_state.timestamp),
        )
        eef = EEFStateModel(
            pose_6d=tuple(float(value) for value in eef_state.pose_6d()),
            gripper_pos=float(eef_state.gripper_pos),
            gripper_vel=float(eef_state.gripper_vel),
            gripper_torque=float(eef_state.gripper_torque),
            timestamp=float(eef_state.timestamp),
        )
        return RobotState(self._mode, joint, eef, True, self.name, self._last_error)

    def get_home_pose(self) -> tuple[float, ...]:
        controller = self._require_controller()
        return tuple(float(value) for value in controller.get_home_pose())

    def move_eef(self, request: MoveEEFRequest) -> CommandResponse:
        if request.plan_only:
            return CommandResponse(
                CommandStatus.COMPLETED,
                "sdk eef request validated as plan-only",
                command_id=request.command_id,
                state=self.get_state(),
            )
        try:
            controller = self._require_controller()
            eef_cmd = self._sdk.EEFState()
            eef_cmd.pose_6d()[:] = request.pose_6d
            eef_cmd.gripper_pos = request.gripper_pos
            eef_cmd.timestamp = controller.get_timestamp() + request.preview_time_s
            controller.set_eef_cmd(eef_cmd)
            self._mode = ArmMode.TELEOP
            return CommandResponse(
                CommandStatus.COMPLETED,
                "sdk eef command sent",
                command_id=request.command_id,
                state=self.get_state(),
            )
        except Exception as exc:
            self._last_error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            self._mode = ArmMode.FAULTED
            return CommandResponse(CommandStatus.FAULTED, "sdk eef command failed", error=self._last_error)

    def apply_debug_profile(self, request: DebugProfileRequest) -> CommandResponse:
        if request.plan_only:
            return CommandResponse(
                CommandStatus.COMPLETED,
                f"sdk debug profile {request.name.value} validated as plan-only",
                command_id=request.command_id,
                state=self.get_state(),
            )
        try:
            controller = self._require_controller()
            if request.name == DebugProfileName.DAMPING:
                return self.damping()
            if request.name == DebugProfileName.RESET_HOME:
                controller.reset_to_home()
                self._mode = ArmMode.IDLE
            elif request.name == DebugProfileName.LOW_GAIN_PASSIVE:
                gain = controller.get_gain()
                controller.set_gain(gain * 0.2)
                self._mode = ArmMode.MAINTENANCE
            elif request.name == DebugProfileName.COMPLIANCE_SLOW:
                gain = controller.get_gain()
                controller.set_gain(gain * 0.5)
                self._mode = ArmMode.MAINTENANCE
            elif request.name == DebugProfileName.GRAVITY_COMPENSATION_STARTUP:
                raise ArmctrlError(
                    ErrorCode.INVALID_REQUEST,
                    "gravity_compensation must be configured before controller creation",
                )
            return CommandResponse(
                CommandStatus.COMPLETED,
                f"sdk debug profile {request.name.value} applied",
                command_id=request.command_id,
                state=self.get_state(),
            )
        except ArmctrlError as exc:
            return CommandResponse(CommandStatus.REJECTED, exc.message, command_id=request.command_id, error=exc)
        except Exception as exc:
            self._last_error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            self._mode = ArmMode.FAULTED
            return CommandResponse(CommandStatus.FAULTED, "sdk debug profile failed", error=self._last_error)

    def damping(self) -> CommandResponse:
        try:
            controller = self._require_controller()
            controller.set_to_damping()
            self._mode = ArmMode.DAMPING
            return CommandResponse(CommandStatus.COMPLETED, "sdk damping enabled", state=self.get_state())
        except Exception as exc:
            self._last_error = ArmctrlError(ErrorCode.SDK_ERROR, str(exc))
            self._mode = ArmMode.FAULTED
            return CommandResponse(CommandStatus.FAULTED, "sdk damping failed", error=self._last_error)

    def cancel(self) -> CommandResponse:
        response = self.damping()
        if response.status is CommandStatus.COMPLETED:
            return CommandResponse(CommandStatus.CANCELLED, "sdk command cancelled via damping", state=self.get_state())
        return response
