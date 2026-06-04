from __future__ import annotations

from dataclasses import dataclass
import csv
import importlib
import json
import time
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
        if not (plan.artifact_safety or {}).get("allowed", False):
            raise RuntimeError("planned trajectory did not pass safety checks")

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


class Arx5InterfaceCollectionBackend:
    def __init__(
        self,
        *,
        model: str,
        interface: str,
        arx5_module: object | None = None,
        max_joint_step_rad: float = 0.01,
        sleep=time.sleep,
    ) -> None:
        self._model = model
        self._interface = interface
        self._arx5 = arx5_module
        self._max_joint_step_rad = max_joint_step_rad
        self._sleep = sleep
        self._controller = None

    def enter_hold_or_damping(self) -> None:
        arx5 = self._load_arx5()
        robot_config = arx5.RobotConfigFactory.get_instance().get_config(self._model)
        controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "joint_controller",
            robot_config.joint_dof,
        )
        controller_config.controller_dt = 0.01
        controller_config.gravity_compensation = True
        controller_config.background_send_recv = True
        self._controller = arx5.Arx5JointController(
            robot_config,
            controller_config,
            self._interface,
        )

    def read_samples(self, request: SysIdPlanRequest) -> list[dict[str, str]]:
        if self._controller is None:
            raise RuntimeError("sdk controller is not initialized")
        rows: list[dict[str, str]] = []
        planned_rows = trajectory_rows(request)
        if planned_rows:
            self._ramp_to_first_target(request, planned_rows[0])
        for planned in planned_rows:
            cmd = self._joint_state_from_plan(request, planned)
            self._controller.set_joint_cmd(cmd)
            self._sleep(1.0 / request.sample_hz)
            rows.append(self._row_from_observation(request, planned))
        return rows

    def enter_damping(self) -> None:
        if self._controller is not None:
            self._controller.set_to_damping()

    def _ramp_to_first_target(
        self,
        request: SysIdPlanRequest,
        first_planned: dict[str, str],
    ) -> None:
        if self._controller is None:
            raise RuntimeError("sdk controller is not initialized")
        current_state = self._controller.get_joint_state()
        current = [
            float(current_state.pos()[joint_index])
            for joint_index in range(request.dof)
        ]
        target = [
            float(first_planned[f"q_cmd_{joint_index + 1}"])
            for joint_index in range(request.dof)
        ]
        max_delta = max(abs(end - start) for start, end in zip(current, target))
        if max_delta == 0.0:
            return
        step_limit = max(self._max_joint_step_rad, 1e-6)
        step_count = int(max_delta / step_limit)
        if max_delta % step_limit:
            step_count += 1
        for step_index in range(1, step_count + 1):
            ratio = step_index / step_count
            q_cmd = tuple(
                start + (end - start) * ratio
                for start, end in zip(current, target)
            )
            cmd = self._joint_state_from_positions(request, q_cmd)
            self._controller.set_joint_cmd(cmd)
            self._sleep(1.0 / request.sample_hz)

    def _load_arx5(self):
        if self._arx5 is None:
            self._arx5 = importlib.import_module("arx5_interface")
        return self._arx5

    def _joint_state_from_plan(
        self,
        request: SysIdPlanRequest,
        planned: dict[str, str],
    ):
        arx5 = self._load_arx5()
        cmd = arx5.JointState(request.dof)
        for joint_index in range(request.dof):
            cmd.pos()[joint_index] = float(planned[f"q_cmd_{joint_index + 1}"])
        cmd.gripper_pos = 0.0
        return cmd

    def _joint_state_from_positions(
        self,
        request: SysIdPlanRequest,
        q_cmd: tuple[float, ...],
    ):
        arx5 = self._load_arx5()
        cmd = arx5.JointState(request.dof)
        for joint_index, joint_position in enumerate(q_cmd):
            cmd.pos()[joint_index] = joint_position
        cmd.gripper_pos = 0.0
        return cmd

    def _row_from_observation(
        self,
        request: SysIdPlanRequest,
        planned: dict[str, str],
    ) -> dict[str, str]:
        if self._controller is None:
            raise RuntimeError("sdk controller is not initialized")
        joint_state = self._controller.get_joint_state()
        row = dict(planned)
        for joint_index in range(request.dof):
            row[f"q_{joint_index + 1}"] = f"{float(joint_state.pos()[joint_index]):.6f}"
            row[f"dq_{joint_index + 1}"] = f"{float(joint_state.vel()[joint_index]):.6f}"
            row[f"tau_meas_{joint_index + 1}"] = f"{float(joint_state.torque()[joint_index]):.6f}"
        return row


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

    def reject_sdk_unavailable(self, *, adapter: str) -> dict[str, object]:
        return {
            "status": "rejected",
            "schema": "armctrl.sysid_run.v1",
            "adapter": adapter,
            "reason": "arx5_interface is not importable in this environment",
            "requires_confirm": SDK_CONFIRMATION,
            "confirm_received": True,
            "movement_allowed": False,
            "fault_landing_mode": "damping",
            "recording_starts_after_safe_state": True,
            "next_gate": "install arx5_interface on the target Linux host before running sdk collection",
        }

    def reject_unsafe_plan(self, *, adapter: str) -> dict[str, object]:
        return {
            "status": "rejected",
            "schema": "armctrl.sysid_run.v1",
            "adapter": adapter,
            "reason": "planned trajectory did not pass safety checks",
            "requires_confirm": SDK_CONFIRMATION,
            "confirm_received": True,
            "movement_allowed": False,
            "fault_landing_mode": "damping",
            "recording_starts_after_safe_state": True,
            "next_gate": "run sysid plan with smaller amplitude or safer q-center and inspect manifest safety checks",
        }
