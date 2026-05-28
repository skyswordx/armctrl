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
            "next_gate": "real_sdk_runner_pending",
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


def _module_status(module_name: str) -> dict[str, str]:
    spec = importlib.util.find_spec(module_name)
    return {
        "module": module_name,
        "status": "available" if spec is not None else "missing",
    }
