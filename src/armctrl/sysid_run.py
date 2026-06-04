from __future__ import annotations

from dataclasses import dataclass
import csv
import json
from typing import Protocol

from armctrl.sysid import SysIdPlanRequest, SysIdPlanner, trajectory_rows

SDK_CONFIRMATION = "I UNDERSTAND THIS WILL MOVE THE ARM"


@dataclass(frozen=True)
class SysIdRunResult:
    schema: str
    adapter: str
    sample_count: int
    artifacts: dict[str, str]

    def to_json(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "adapter": self.adapter,
            "sample_count": self.sample_count,
            "artifacts": self.artifacts,
        }


class FakeSysIdRunner:
    def run(self, request: SysIdPlanRequest) -> SysIdRunResult:
        request.output_dir.mkdir(parents=True, exist_ok=True)
        raw_samples_path = request.output_dir / "raw_samples.csv"
        manifest_path = request.output_dir / "manifest.json"
        plan = SysIdPlanner.default().write_plan(request)
        raw_rows = _raw_sample_rows(request)

        with raw_samples_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(raw_rows[0]))
            writer.writeheader()
            writer.writerows(raw_rows)

        manifest = {
            "schema": "armctrl.sysid_run_manifest.v1",
            "adapter": "fake",
            "profile": plan.profile.to_json(),
            "request": {
                "dof": request.dof,
                "sample_hz": request.sample_hz,
                "duration_s": request.duration_s,
                "amplitude_rad": request.amplitude_rad,
                "q_center": list(request.q_center),
                "urdf_path": request.urdf_path,
                "safe_config_path": request.safe_config_path,
            },
            "sample_count": len(raw_rows),
            "handoff": plan.handoff,
            "artifacts": {
                "planned_trajectory": plan.artifacts["planned_trajectory"],
                "raw_samples": str(raw_samples_path),
                "manifest": str(manifest_path),
            },
            "plan_safety": plan.artifact_safety,
        }
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return SysIdRunResult(
            schema="armctrl.sysid_run.v1",
            adapter="fake",
            sample_count=len(raw_rows),
            artifacts={
                "planned_trajectory": plan.artifacts["planned_trajectory"],
                "raw_samples": str(raw_samples_path),
                "manifest": str(manifest_path),
            },
        )


class SdkCollectionBackend(Protocol):
    def enter_hold_or_damping(self) -> None:
        ...

    def read_samples(self, request: SysIdPlanRequest) -> list[dict[str, str]]:
        ...

    def enter_damping(self) -> None:
        ...


class SdkSysIdRunner:
    def __init__(self, *, backend: SdkCollectionBackend) -> None:
        self._backend = backend

    def run(self, request: SysIdPlanRequest, *, confirm: str) -> SysIdRunResult:
        if confirm != SDK_CONFIRMATION:
            raise PermissionError("sdk sysid runner requires explicit operator confirmation")
        request.output_dir.mkdir(parents=True, exist_ok=True)
        raw_samples_path = request.output_dir / "raw_samples.csv"
        manifest_path = request.output_dir / "manifest.json"
        plan = SysIdPlanner.default().write_plan(request)

        self._backend.enter_hold_or_damping()
        try:
            raw_rows = self._backend.read_samples(request)
        finally:
            self._backend.enter_damping()

        with raw_samples_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(raw_rows[0]))
            writer.writeheader()
            writer.writerows(raw_rows)

        manifest = {
            "schema": "armctrl.sysid_run_manifest.v1",
            "adapter": "sdk",
            "profile": plan.profile.to_json(),
            "request": {
                "dof": request.dof,
                "sample_hz": request.sample_hz,
                "duration_s": request.duration_s,
                "amplitude_rad": request.amplitude_rad,
                "q_center": list(request.q_center),
                "urdf_path": request.urdf_path,
                "safe_config_path": request.safe_config_path,
            },
            "sample_count": len(raw_rows),
            "handoff": plan.handoff,
            "artifacts": {
                "planned_trajectory": plan.artifacts["planned_trajectory"],
                "raw_samples": str(raw_samples_path),
                "manifest": str(manifest_path),
            },
            "plan_safety": plan.artifact_safety,
            "safety": {
                "requires_confirm": SDK_CONFIRMATION,
                "movement_allowed": True,
                "recording_starts_after_safe_state": True,
                "fault_landing_mode": "damping",
            },
        }
        manifest_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return SysIdRunResult(
            schema="armctrl.sysid_run.v1",
            adapter="sdk",
            sample_count=len(raw_rows),
            artifacts={
                "planned_trajectory": plan.artifacts["planned_trajectory"],
                "raw_samples": str(raw_samples_path),
                "manifest": str(manifest_path),
            },
        )


def _raw_sample_rows(request: SysIdPlanRequest) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for planned in trajectory_rows(request):
        row = dict(planned)
        for joint_index in range(request.dof):
            q_cmd = float(planned[f"q_cmd_{joint_index + 1}"])
            row[f"q_{joint_index + 1}"] = f"{q_cmd:.6f}"
            row[f"dq_{joint_index + 1}"] = "0.000000"
            row[f"tau_meas_{joint_index + 1}"] = "0.000000"
        rows.append(row)
    return rows


class SdkSysIdRunnerGate:
    def evaluate(self, *, adapter: str, confirm: str | None) -> dict[str, object]:
        if confirm != SDK_CONFIRMATION:
            return self.reject_without_confirmation(adapter=adapter)
        return self.reject_without_backend(adapter=adapter)

    def reject_without_confirmation(self, *, adapter: str) -> dict[str, object]:
        return {
            "status": "rejected",
            "schema": "armctrl.sysid_run.v1",
            "adapter": adapter,
            "reason": "sdk sysid runner requires explicit operator confirmation",
            "requires_confirm": SDK_CONFIRMATION,
            "movement_allowed": False,
            "fault_landing_mode": "damping",
            "recording_starts_after_safe_state": True,
            "next_gate": "run sysid sdk-handshake-plan before enabling sdk runner",
        }

    def reject_without_backend(self, *, adapter: str) -> dict[str, object]:
        return {
            "status": "rejected",
            "schema": "armctrl.sysid_run.v1",
            "adapter": adapter,
            "reason": "real sdk sysid runner is not implemented in this clean rebuild",
            "requires_confirm": SDK_CONFIRMATION,
            "confirm_received": True,
            "movement_allowed": False,
            "fault_landing_mode": "damping",
            "recording_starts_after_safe_state": True,
            "next_gate": "implement and verify an arx5_interface SdkCollectionBackend before moving hardware",
        }
