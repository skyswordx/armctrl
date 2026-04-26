"""夹爪标定服务。

这部分把三件事收在一起：
1. 创建 SDK `Arx5JointController`；
2. 复用 SDK 原生 `calibrate_gripper()` 完成交互式零点设置；
3. 把结果转换成项目侧持久化标定文件。

这里没有重写 SDK 的底层标定流程。
项目层只是补了一层“保存结果”和“后续自动应用”。
"""

from __future__ import annotations

import importlib
from collections.abc import Callable

from armctrl.protocol.enums import CommandStatus, ErrorCode
from armctrl.protocol.errors import ArmctrlError
from armctrl.protocol.models import CommandResponse

from .models import GripperCalibration, apply_gripper_calibration
from .store import GripperCalibrationStore


def derive_gripper_motor_readout(joint_state, robot_config) -> float:
    """按 SDK 公式反推出当前夹爪电机角度读数。

    SDK 内部公式是：
    `gripper_pos = angle_actual_rad / gripper_open_readout * gripper_width`

    因此反推有：
    `angle_actual_rad = gripper_pos / gripper_width * gripper_open_readout`

    这个反推只适合做离线分析或诊断。
    夹爪标定 wizard 结束后不能再依赖这个值做最终保存，
    因为 SDK 可能会在恢复后台线程后立刻按旧配置做一次位置合法性检查，
    这时 `JointState` 已经不再代表“刚刚 fully-open 时 SDK 打印出来的权威原始读数”。
    """

    width = float(robot_config.gripper_width)
    if abs(width) <= 1e-12:
        raise ArmctrlError(ErrorCode.INVALID_REQUEST, "robot_config.gripper_width must not be zero during calibration")
    return float(joint_state.gripper_pos) / width * float(robot_config.gripper_open_readout)


def _parse_width_text(text: str, default_width_m: float) -> float:
    # 终端标定时人工经常会顺手输入 `82` 这种毫米值。
    # 这里做一个很保守的兼容：数值大于 1 时视为毫米，其他按米。
    raw = text.strip()
    if not raw:
        return float(default_width_m)
    value = float(raw)
    if value > 1.0:
        return value / 1000.0
    return value


def _parse_required_float(text: str, *, field_name: str) -> float:
    """解析必须输入的浮点数。

    这里专门给 wizard 里“抄 SDK 终端打印值”的场景使用。
    标定结果如果允许空值继续保存，后续自动启动仍然会读到错误配置，
    所以这里宁可直接报错，也不悄悄沿用旧值。
    """

    raw = text.strip()
    if not raw:
        raise ArmctrlError(ErrorCode.INVALID_REQUEST, f"{field_name} is required")
    return float(raw)


class GripperCalibrationService:
    """复用 SDK 的夹爪标定服务。"""

    def __init__(
        self,
        *,
        store: GripperCalibrationStore | None = None,
        import_module: Callable[[str], object] = importlib.import_module,
        input_fn: Callable[[str], str] = input,
        output_fn: Callable[[str], None] | None = None,
    ) -> None:
        self.store = store or GripperCalibrationStore()
        self._import_module = import_module
        self._input_fn = input_fn
        self._output_fn = output_fn or (lambda message: print(message))

    def show(self, model: str) -> CommandResponse:
        calibration = self.store.load(model)
        if calibration is None:
            return CommandResponse(
                CommandStatus.COMPLETED,
                f"no gripper calibration saved for model {model}",
                detail={
                    "model": model,
                    "path": str(self.store.path_for_model(model)),
                    "calibration": None,
                },
            )
        return CommandResponse(
            CommandStatus.COMPLETED,
            f"loaded gripper calibration for model {model}",
            detail={
                "model": model,
                "path": str(self.store.path_for_model(model)),
                "calibration": calibration.to_dict(),
            },
        )

    def set(
        self,
        *,
        model: str,
        gripper_open_readout: float,
        gripper_width: float,
        interface: str | None = None,
        notes: str = "",
        source: str = "manual_set",
    ) -> CommandResponse:
        calibration = GripperCalibration(
            model=model,
            gripper_open_readout=gripper_open_readout,
            gripper_width=gripper_width,
            closed_motor_readout=0.0,
            interface=interface,
            source=source,
            notes=notes,
        )
        path = self.store.save(calibration)
        return CommandResponse(
            CommandStatus.COMPLETED,
            f"saved gripper calibration for model {model}",
            detail={"model": model, "path": str(path), "calibration": calibration.to_dict()},
        )

    def clear(self, model: str) -> CommandResponse:
        removed = self.store.clear(model)
        return CommandResponse(
            CommandStatus.COMPLETED,
            f"cleared gripper calibration for model {model}" if removed else f"no gripper calibration to clear for model {model}",
            detail={"model": model, "path": str(self.store.path_for_model(model)), "removed": removed is not None},
        )

    def run_wizard(self, *, model: str, interface: str) -> CommandResponse:
        try:
            sdk = self._import_module("arx5_interface")
        except ModuleNotFoundError:
            error = ArmctrlError(ErrorCode.SDK_UNAVAILABLE, "arx5_interface is not importable in this environment")
            return CommandResponse(CommandStatus.FAULTED, "sdk unavailable", error=error)

        try:
            robot_config = sdk.RobotConfigFactory.get_instance().get_config(model)
            saved = self.store.load(model)
            # 这里优先把项目里已有的标定值先应用回去。
            # 原因是某些机器如果当前方向符号就是错的，控制器构造阶段就会先被 SDK 拒绝。
            # 已经保存过的项目标定，正好就是下一次连上控制器的 bootstrap 条件。
            apply_gripper_calibration(robot_config, saved)

            controller_config = sdk.ControllerConfigFactory.get_instance().get_config("joint_controller", robot_config.joint_dof)
            controller_config.gravity_compensation = False
            # 这里显式关掉后台收发线程。
            # 这轮排障已经确认，若 SDK 在 calibrate_gripper() 返回后立刻恢复后台线程，
            # 旧的夹爪配置可能马上触发一次 position sanity check，
            # 导致我们还没来得及把新标定值持久化，控制器就先进 emergency。
            if hasattr(controller_config, "background_send_recv"):
                controller_config.background_send_recv = False
            controller = sdk.Arx5JointController(robot_config, controller_config, interface)
        except Exception as exc:
            error = ArmctrlError(
                ErrorCode.SDK_ERROR,
                (
                    f"failed to create SDK joint controller for gripper calibration: {exc}. "
                    "If gripper direction or width is already wrong enough to block startup, "
                    "save a bootstrap project calibration first with gripper-calibration-set."
                ),
            )
            return CommandResponse(CommandStatus.FAULTED, "gripper calibration bootstrap failed", error=error)

        # 下面直接复用 SDK 原生交互流程。
        # SDK 自己会在终端里提示“闭合 -> 回车 -> 张开 -> 回车”，
        # 并在闭合位置调用 reset_zero_readout。
        controller.calibrate_gripper()

        runtime_robot_config = controller.get_robot_config()
        default_width_m = float(runtime_robot_config.gripper_width)
        self._output_fn(
            "SDK 已完成零点校准。"
            " 请把 SDK 刚才打印的 `Fully-open joint position readout: ...` 数值输入下面这个提示。"
            " 这里不再从 JointState 反推，避免旧配置恢复后把标定值算错。"
        )
        open_readout_text = self._input_fn(
            "请输入 SDK 终端打印的 fully-open joint position readout："
        )
        open_readout = _parse_required_float(
            open_readout_text,
            field_name="fully-open joint position readout",
        )
        width_text = self._input_fn(
            "请输入完全张开时的真实开口宽度，单位米；如果直接回车则沿用当前值。"
            "也可以直接输入毫米整数，例如 82："
        )
        calibrated_width_m = _parse_width_text(width_text, default_width_m)

        notes = self._input_fn("可选备注，直接回车跳过：").strip()
        calibration = GripperCalibration(
            model=model,
            gripper_open_readout=open_readout,
            gripper_width=calibrated_width_m,
            closed_motor_readout=0.0,
            source="sdk_calibrate_gripper",
            interface=interface,
            notes=notes,
        )
        path = self.store.save(calibration)
        return CommandResponse(
            CommandStatus.COMPLETED,
            f"saved gripper calibration for model {model}",
            detail={
                "model": model,
                "path": str(path),
                "calibration": calibration.to_dict(),
                "wizard": {
                    "sdk_default_gripper_width": default_width_m,
                    "sdk_reported_open_motor_readout": open_readout,
                    "closed_motor_readout_after_zero": 0.0,
                },
            },
        )
