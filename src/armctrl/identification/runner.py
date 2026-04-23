"""辨识轨迹执行与采集。"""

from __future__ import annotations

import time
from collections.abc import Callable

from armctrl.identification.backends import JointRobotIO
from armctrl.identification.models import ExcitationProfile, JointSample
from armctrl.identification.recorder import DatasetRecorder
from armctrl.identification.safety import TrajectorySafetyLimits, validate_trajectory
from armctrl.protocol.enums import CommandStatus
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
    ) -> None:
        self.backend = backend
        self.sample_hz = float(sample_hz)
        self.safety_limits = safety_limits
        self.sleep_fn = sleep_fn
        self.damping_after = damping_after

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
        if execute:
            send_response = self.backend.send_joint_trajectory(profile.points)
            if send_response.status is not CommandStatus.COMPLETED:
                return send_response
        samples = self._collect_samples(profile, execute=execute)
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
                "output_dir": str(recorder.output_dir) if recorder else None,
                "manifest": manifest.to_dict() if manifest else None,
            }
        )
        return CommandResponse(CommandStatus.COMPLETED, "identification run completed", detail=detail)

    def _collect_samples(self, profile: ExcitationProfile, *, execute: bool) -> list[JointSample]:
        samples: list[JointSample] = []
        start_monotonic = time.monotonic()
        for point in profile.points:
            if execute:
                target_monotonic = start_monotonic + point.t_s
                wait_s = target_monotonic - time.monotonic()
                if wait_s > 0:
                    self.sleep_fn(wait_s)
            # plan-only 采集也走 read_sample。
            # fake 后端会返回命令值，SDK 后端只有 execute=True 时才应在实机上使用。
            samples.append(self.backend.read_sample(point, point.phase))
        return samples

