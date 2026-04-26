"""ARX5 控制 profile 定义。

这里专门放“ARX5 适配器如何构造不同 gain profile”的逻辑。
目的很直接：

1. 不把模式参数硬编码散落在 `sdk.py`；
2. 把“调试 profile 元数据”和“ARX5 具体 gain 构造”分开；
3. 让 teleop / zero-gravity 两类运行时增益切换有单独的事实来源。

这里仍然只复用 SDK 已有的 `Gain`、`default_kp`、`default_kd`。
项目层不自己重新定义一套 MIT 控制器。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TypeAlias

from armctrl.protocol.enums import ArmMode, DebugProfileName


@dataclass(frozen=True)
class JointScaleProfile:
    """逐关节增益缩放定义。

    这里保留两层语义：

    1. `uniform`
       先对整条默认增益向量做统一缩放。
       这延续了当前项目“尽量复用 SDK 默认参数，再做相对缩放”的思路。
    2. `per_joint`
       可选的逐关节二次乘子。
       它不是一组绝对 `kp/kd`，而是给已经整体缩放后的向量再做局部修正。

    这样做的好处是：
    - 不复制 SDK 默认值；
    - X5 / L5 / X7 不同自由度仍能共用同一套构造逻辑；
    - 可以只对 shoulder / elbow 这种主承载关节单独微调手感。
    """

    uniform: float = 1.0
    per_joint: tuple[float, ...] | None = None

    @classmethod
    def coerce(cls, value: "JointScaleInput") -> "JointScaleProfile":
        # 这里做一个兼容层，避免把现有所有调用点都一起打碎：
        # - 传 float：保持旧语义，表示统一缩放；
        # - 传 tuple/list：表示逐关节乘子，统一缩放默认取 1；
        # - 传 JointScaleProfile：直接复用。
        if isinstance(value, JointScaleProfile):
            return value
        if isinstance(value, (tuple, list)):
            return cls(uniform=1.0, per_joint=tuple(float(item) for item in value))
        return cls(uniform=float(value))

    def scale_vector(self, defaults: list[float] | tuple[float, ...]) -> list[float]:
        scaled = [float(value) * self.uniform for value in defaults]
        if self.per_joint is None:
            return scaled
        if len(self.per_joint) != len(scaled):
            raise ValueError(
                f"per_joint scale length {len(self.per_joint)} does not match controller dof {len(scaled)}"
            )
        return [scaled[index] * self.per_joint[index] for index in range(len(scaled))]


JointScaleInput: TypeAlias = float | tuple[float, ...] | list[float] | JointScaleProfile


@dataclass(frozen=True)
class Arx5GainProfile:
    """ARX5 增益 profile。

    这些字段的语义是“相对 SDK 默认增益的缩放比例”，
    不是一组脱离 SDK 的绝对控制参数。
    """

    name: DebugProfileName
    label: str
    target_mode: ArmMode
    kp_scale: JointScaleInput
    kd_scale: JointScaleInput
    gripper_kp_scale: float
    gripper_kd_scale: float
    sync_eef_target: bool = True

    def __post_init__(self) -> None:
        # 这里统一把外部传入的 `float / tuple / JointScaleProfile` 规范化。
        # 这样 default profile、测试自定义 profile 和后续配置文件加载都能共用同一套入口。
        object.__setattr__(self, "kp_scale", JointScaleProfile.coerce(self.kp_scale))
        object.__setattr__(self, "kd_scale", JointScaleProfile.coerce(self.kd_scale))
        object.__setattr__(self, "gripper_kp_scale", float(self.gripper_kp_scale))
        object.__setattr__(self, "gripper_kd_scale", float(self.gripper_kd_scale))


class Arx5GainProfileRegistry:
    """ARX5 增益 profile 注册表。"""

    def __init__(self, profiles: list[Arx5GainProfile]) -> None:
        self._profiles = {profile.name: profile for profile in profiles}

    def has(self, name: DebugProfileName | str) -> bool:
        return DebugProfileName(name) in self._profiles

    @classmethod
    def default(cls) -> "Arx5GainProfileRegistry":
        # 这里保留当前已经在实机上调过的一组比例。
        # 只是把它们从 `sdk.py` 的流程代码里移出来，
        # 不在这次重构里继续改数值。
        return cls(
            [
                Arx5GainProfile(
                    name=DebugProfileName.TELEOP,
                    label="sdk default teleop gain",
                    target_mode=ArmMode.TELEOP,
                    # `teleop` 的语义就是完整复用 SDK 默认 MIT 增益。
                    # 这里显式保留一条 profile，而不是让 `sdk.py` 私下拼默认值，
                    # 这样“恢复标准操控手感”也成为可见、可测试、可复用的统一入口。
                    kp_scale=1.0,
                    kd_scale=1.0,
                    gripper_kp_scale=1.0,
                    gripper_kd_scale=1.0,
                    sync_eef_target=True,
                ),
                Arx5GainProfile(
                    name=DebugProfileName.ZERO_GRAVITY_DRAG,
                    label="gravity compensation + low-gain hand guiding",
                    target_mode=ArmMode.ZERO_GRAVITY_DRAG,
                    kp_scale=0.0,
                    # 当前实机体验上，大部分关节在 `1e-5` 全局阻尼下已经比较顺手，
                    # 但 shoulder / elbow（joint2 / joint3）在前伸、后折这两类姿态里仍偏难拖。
                    # 这里先保留统一极低阻尼，再单独把 joint2 / joint3 的阻尼再降一档。
                    # X5 默认 cartesian kd 顺序 = [J1, J2, J3, J4, J5, J6] = [5, 5, 5, 1, 1, 1]
                    kd_scale=JointScaleProfile(
                        uniform=0.00001,
                        per_joint=(0.3, 0.3, 0.3, 1.5, 1.0, 1.0),
                    ),
                    gripper_kp_scale=0.0,
                    gripper_kd_scale=0.0,
                    sync_eef_target=True,
                ),
            ]
        )

    def get(self, name: DebugProfileName | str) -> Arx5GainProfile:
        return self._profiles[DebugProfileName(name)]

    def build_gain(self, sdk_module, controller_config, name: DebugProfileName | str):
        profile = self.get(name)
        return sdk_module.Gain(
            profile.kp_scale.scale_vector(controller_config.default_kp),
            profile.kd_scale.scale_vector(controller_config.default_kd),
            float(controller_config.default_gripper_kp) * profile.gripper_kp_scale,
            float(controller_config.default_gripper_kd) * profile.gripper_kd_scale,
        )
