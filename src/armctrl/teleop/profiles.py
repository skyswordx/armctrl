from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class JointScaleProfile:
    uniform: float = 1.0
    per_joint: tuple[float, ...] | None = None

    def scale_vector(self, defaults: list[float] | tuple[float, ...]) -> list[float]:
        scaled = [float(value) * self.uniform for value in defaults]
        if self.per_joint is None:
            return scaled
        if len(self.per_joint) != len(scaled):
            raise ValueError(
                f"per_joint scale length {len(self.per_joint)} does not match controller dof {len(scaled)}"
            )
        return [
            value * self.per_joint[index]
            for index, value in enumerate(scaled)
        ]


@dataclass(frozen=True)
class RuntimeGainProfile:
    name: str
    label: str
    kp_scale: JointScaleProfile
    kd_scale: JointScaleProfile
    gripper_kp_scale: float
    gripper_kd_scale: float
    sync_eef_target: bool = True


class RuntimeGainProfileRegistry:
    def __init__(self, profiles: list[RuntimeGainProfile]) -> None:
        self._profiles = {profile.name: profile for profile in profiles}

    @classmethod
    def default(cls) -> "RuntimeGainProfileRegistry":
        return cls(
            [
                RuntimeGainProfile(
                    name="teleop",
                    label="sdk default teleop gain",
                    kp_scale=JointScaleProfile(1.0),
                    kd_scale=JointScaleProfile(1.0),
                    gripper_kp_scale=1.0,
                    gripper_kd_scale=1.0,
                    sync_eef_target=True,
                ),
                RuntimeGainProfile(
                    name="zero_gravity_drag",
                    label="gravity compensation + low-gain hand guiding",
                    kp_scale=JointScaleProfile(0.0),
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

    def names(self) -> tuple[str, ...]:
        return tuple(sorted(self._profiles))

    def get(self, name: str) -> RuntimeGainProfile:
        try:
            return self._profiles[str(name)]
        except KeyError as error:
            raise ValueError(
                f"unsupported teleop profile {name!r}; expected one of {', '.join(self.names())}"
            ) from error

    def build_gain(self, sdk_module, controller_config, name: str):
        profile = self.get(name)
        return sdk_module.Gain(
            profile.kp_scale.scale_vector(controller_config.default_kp),
            profile.kd_scale.scale_vector(controller_config.default_kd),
            float(controller_config.default_gripper_kp) * profile.gripper_kp_scale,
            float(controller_config.default_gripper_kd) * profile.gripper_kd_scale,
        )

