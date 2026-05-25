"""辨识轨迹执行与采集。"""

from __future__ import annotations

import time
from collections.abc import Callable

from armctrl.identification.backends import JointRobotIO
from armctrl.identification.models import ExcitationProfile, JointSample
from armctrl.identification.recorder import DatasetRecorder
from armctrl.identification.safety import TrajectorySafetyLimits, validate_trajectory
from armctrl.protocol.enums import CommandStatus, ErrorCode
from armctrl.protocol.errors import ArmctrlError
from armctrl.protocol.models import CommandResponse


class IdentificationRunner:
    """执行轨迹并采集关节状态。

    runner 不知道具体机械臂型号。
    它只按 `JointRobotIO` 协议调用后端，保证未来接 Piper 时不用重写采集主流程。
    """

    def __init__(
        self,
        backend: JointRobotIO,
        *,
        sample_hz: float = 100.0,
        safety_limits: TrajectorySafetyLimits | None = None,
        sleep_fn: Callable[[float], None] = time.sleep,
        damping_after: bool = True,
        reset_home_before_execute: bool = True,
    ) -> None:
        self.backend = backend
        self.sample_hz = float(sample_hz)
        self.safety_limits = safety_limits
        self.sleep_fn = sleep_fn
        self.damping_after = damping_after
        # 当前 CLI 生成的辨识轨迹默认都以 q0=0 为基线。
        # 如果实机起始姿态和这条基线偏差很大，直接下发第一段轨迹会有明显风险。
        # 因此 runner 在真实执行前默认先调用一次后端的 `reset_home()`。
        self.reset_home_before_execute = reset_home_before_execute

    def run(
        self,
        profile: ExcitationProfile,
        *,
        recorder: DatasetRecorder | None = None,
        execute: bool = False,
        urdf_path: str | None = None,
    ) -> CommandResponse:
        validation = validate_trajectory(profile, self.safety_limits)
        if not validation.allowed:
            return CommandResponse(
                CommandStatus.REJECTED,
                validation.error.message if validation.error else "trajectory rejected",
                error=validation.error,
                detail=profile.summary(),
            )
        connect_response = self.backend.connect()
        if connect_response.status is not CommandStatus.COMPLETED:
            return connect_response
        try:
            if execute:
                if self.reset_home_before_execute:
                    # 这里把“执行前自动回零”放在 runner，而不是散落到 CLI 或具体后端里。
                    # 好处是 fake / sdk / 未来 Piper 后端都共用同一条安全语义。
                    reset_response = self.backend.reset_home()
                    if reset_response.status is not CommandStatus.COMPLETED:
                        return reset_response
                send_response = self._begin_execution(profile)
                if send_response.status is not CommandStatus.COMPLETED:
                    return send_response
            samples = self._collect_samples(profile, execute=execute)
        except KeyboardInterrupt:
            damping_response = self.backend.damping() if execute else None
            detail = profile.summary()
            detail.update(
                {
                    "backend_name": self.backend.name,
                    "execute": execute,
                    "interrupted": True,
                    "damping_after_interrupt": damping_response.to_dict() if damping_response else None,
                }
            )
            return CommandResponse(CommandStatus.CANCELLED, "identification run interrupted; damping requested", detail=detail)
        except Exception as exc:
            damping_response = self.backend.damping() if execute else None
            detail = profile.summary()
            detail.update(
                {
                    "backend_name": self.backend.name,
                    "execute": execute,
                    "damping_after_fault": damping_response.to_dict() if damping_response else None,
                }
            )
            return CommandResponse(
                CommandStatus.FAULTED,
                "identification run faulted; damping requested",
                error=ArmctrlError(ErrorCode.SDK_ERROR, str(exc)),
                detail=detail,
            )
        manifest = None
        if recorder is not None:
            manifest = recorder.write_run(
                profile=profile,
                backend_name=self.backend.name,
                model=self.backend.model,
                samples=samples,
                execute=execute,
                urdf_path=urdf_path,
            )
        if execute and self.damping_after:
            self.backend.damping()
        detail = profile.summary()
        detail.update(
            {
                "backend_name": self.backend.name,
                "sample_count": len(samples),
                "execute": execute,
                "reset_home_before_execute": bool(execute and self.reset_home_before_execute),
                "output_dir": str(recorder.output_dir) if recorder else None,
                "manifest": manifest.to_dict() if manifest else None,
            }
        )
        return CommandResponse(CommandStatus.COMPLETED, "identification run completed", detail=detail)

    def _begin_execution(self, profile: ExcitationProfile) -> CommandResponse:
        begin = getattr(self.backend, "begin_joint_trajectory", None)
        if callable(begin):
            return begin(profile.points)
        return self.backend.send_joint_trajectory(profile.points)

    def _send_point_if_supported(self, point) -> CommandResponse | None:
        send_point = getattr(self.backend, "send_joint_command", None)
        if not callable(send_point):
            return None
        return send_point(point)

    def _collect_samples(self, profile: ExcitationProfile, *, execute: bool) -> list[JointSample]:
        samples: list[JointSample] = []
        start_monotonic = time.monotonic()
        for point in profile.points:
            if execute:
                target_monotonic = start_monotonic + point.t_s
                wait_s = target_monotonic - time.monotonic()
                if wait_s > 0:
                    self.sleep_fn(wait_s)
                send_response = self._send_point_if_supported(point)
                if send_response is not None and send_response.status is not CommandStatus.COMPLETED:
                    raise RuntimeError(send_response.message)
            # plan-only 采集也走 read_sample。
            # fake 后端会返回命令值，SDK 后端只有 execute=True 时才应在实机上使用。
            samples.append(self.backend.read_sample(point, point.phase))
        return samples
