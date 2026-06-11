from __future__ import annotations

from dataclasses import dataclass
import csv
import json

from armctrl.sysid import SysIdPlanRequest, SysIdPlanner, trajectory_rows


@dataclass(frozen=True)
class SysIdRunResult:
    schema: str
    adapter: str
    sample_count: int
    artifacts: dict[str, str]
    run_status: str = "completed"

    def to_json(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "adapter": self.adapter,
            "run_status": self.run_status,
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
            "run_status": "completed",
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
            "runtime_policy": {
                "formal_real_motion_entrypoint": (
                    "armctrl sysid compile-runtime + "
                    "armctrl motion submit joint-trajectory"
                ),
                "sysid_run_runtime_session_artifact": "unsupported",
                "reason": (
                    "sysid run is offline/fake only and must not acquire a live "
                    "runtime owner or mutate runtime_session.json"
                ),
            },
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
