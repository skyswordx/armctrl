from __future__ import annotations

from dataclasses import dataclass
import importlib.util


@dataclass(frozen=True)
class SdkPreflightResult:
    model: str
    interface: str
    sdk: dict[str, str]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": "armctrl.sysid_sdk_preflight.v1",
            "model": self.model,
            "interface": self.interface,
            "read_only": True,
            "movement_allowed": False,
            "sdk": self.sdk,
            "next_gate": "sdk_runner_confirm_then_hardware_validation",
            "notes": [
                "preflight only checks importability and requested session labels",
                "it does not open CAN, instantiate hardware objects, or send motion commands",
            ],
        }


class SdkPreflight:
    def run(self, *, model: str, interface: str) -> SdkPreflightResult:
        return SdkPreflightResult(
            model=model,
            interface=interface,
            sdk=_module_status("arx5_interface"),
        )


@dataclass(frozen=True)
class SdkHandshakeStep:
    name: str
    purpose: str
    movement_allowed: bool

    def to_json(self) -> dict[str, object]:
        return {
            "name": self.name,
            "purpose": self.purpose,
            "movement_allowed": self.movement_allowed,
        }


@dataclass(frozen=True)
class SdkHandshakePlanResult:
    model: str
    interface: str
    steps: tuple[SdkHandshakeStep, ...]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": "armctrl.sysid_sdk_handshake_plan.v1",
            "model": self.model,
            "interface": self.interface,
            "read_only": True,
            "movement_allowed": False,
            "requires_confirm": "I UNDERSTAND THIS WILL MOVE THE ARM",
            "fault_landing_mode": "damping",
            "steps": [step.to_json() for step in self.steps],
            "next_gate": "sdk_runner_confirm_then_hardware_validation",
            "notes": [
                "handshake planning is read-only and does not import or instantiate the SDK",
                "real collection must enter a verified hold or damping state before recording",
                "faults and Ctrl-C must land in damping during hardware validation",
            ],
        }


class SdkHandshakePlanner:
    def plan(self, *, model: str, interface: str) -> SdkHandshakePlanResult:
        return SdkHandshakePlanResult(
            model=model,
            interface=interface,
            steps=(
                SdkHandshakeStep(
                    name="sdk_preflight",
                    purpose="check SDK availability and requested labels without opening CAN",
                    movement_allowed=False,
                ),
                SdkHandshakeStep(
                    name="operator_confirm",
                    purpose="require explicit human confirmation before any future motion command",
                    movement_allowed=False,
                ),
                SdkHandshakeStep(
                    name="enter_hold_or_damping",
                    purpose="initialize the SDK controller without reset-to-home before collection",
                    movement_allowed=False,
                ),
                SdkHandshakeStep(
                    name="start_recording_after_safe_state",
                    purpose="start logs only after the safe state is reached and verified",
                    movement_allowed=False,
                ),
                SdkHandshakeStep(
                    name="fault_or_ctrl_c_to_damping",
                    purpose="route interruption and controller faults to damping",
                    movement_allowed=False,
                ),
            ),
        )


def _module_status(module_name: str) -> dict[str, str]:
    spec = importlib.util.find_spec(module_name)
    return {
        "module": module_name,
        "status": "available" if spec is not None else "missing",
    }
