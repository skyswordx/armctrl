"""ARX5 控制 profile 定义。

这里专门放“ARX5 适配器如何构造不同 gain profile”的逻辑。
目的很直接：

1. 不把模式参数硬编码散落在 `sdk.py`；
2. 把“调试 profile 元数据”和“ARX5 具体 gain 构造”分开；
3. 让 teleop / zero-gravity / maintenance 三类模式切换有单独的事实来源。

这里仍然只复用 SDK 已有的 `Gain`、`default_kp`、`default_kd`。
项目层不自己重新定义一套 MIT 控制器。
"""

from __future__ import annotations

from dataclasses import dataclass

from armctrl.protocol.enums import ArmMode, DebugProfileName


@dataclass(frozen=True)
class Arx5GainProfile:
    """ARX5 增益 profile。

    这些字段的语义是“相对 SDK 默认增益的缩放比例”，
    不是一组脱离 SDK 的绝对控制参数。
    """

    name: DebugProfileName
    label: str
    target_mode: ArmMode
    kp_scale: float
    kd_scale: float
    gripper_kp_scale: float
    gripper_kd_scale: float
    sync_eef_target: bool = True


class Arx5GainProfileRegistry:
    """ARX5 增益 profile 注册表。"""

    def __init__(self, profiles: list[Arx5GainProfile]) -> None:
        self._profiles = {profile.name: profile for profile in profiles}

    @classmethod
    def default(cls) -> "Arx5GainProfileRegistry":
        # 这里保留当前已经在实机上调过的一组比例。
        # 只是把它们从 `sdk.py` 的流程代码里移出来，
        # 不在这次重构里继续改数值。
        return cls(
            [
                Arx5GainProfile(
                    name=DebugProfileName.ZERO_GRAVITY_DRAG,
                    label="gravity compensation + low-gain hand guiding",
                    target_mode=ArmMode.ZERO_GRAVITY_DRAG,
                    kp_scale=0.0,
                    kd_scale=0.00001,
                    gripper_kp_scale=0.0,
                    gripper_kd_scale=0.0,
                    sync_eef_target=True,
                ),
                Arx5GainProfile(
                    name=DebugProfileName.LOW_GAIN_PASSIVE,
                    label="low-gain passive check",
                    target_mode=ArmMode.MAINTENANCE,
                    kp_scale=0.2,
                    kd_scale=0.2,
                    gripper_kp_scale=0.2,
                    gripper_kd_scale=0.2,
                    sync_eef_target=True,
                ),
                Arx5GainProfile(
                    name=DebugProfileName.COMPLIANCE_SLOW,
                    label="slow compliance profile",
                    target_mode=ArmMode.MAINTENANCE,
                    kp_scale=0.5,
                    kd_scale=0.5,
                    gripper_kp_scale=0.5,
                    gripper_kd_scale=0.5,
                    sync_eef_target=True,
                ),
            ]
        )

    def get(self, name: DebugProfileName | str) -> Arx5GainProfile:
        return self._profiles[DebugProfileName(name)]

    def build_gain(self, sdk_module, controller_config, name: DebugProfileName | str):
        profile = self.get(name)
        return sdk_module.Gain(
            [float(value) * profile.kp_scale for value in controller_config.default_kp],
            [float(value) * profile.kd_scale for value in controller_config.default_kd],
            float(controller_config.default_gripper_kp) * profile.gripper_kp_scale,
            float(controller_config.default_gripper_kd) * profile.gripper_kd_scale,
        )
