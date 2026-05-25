"""Identification dataset writer."""

from __future__ import annotations

import csv
import json
from pathlib import Path

from armctrl.compat.lerobot import build_lerobot_contract
from armctrl.identification.models import (
    DatasetManifest,
    ExcitationProfile,
    JointSample,
    TrajectoryPoint,
    vector_column_names,
)


class DatasetRecorder:
    """Write trajectory and sample results into a stable dataset directory."""

    def __init__(self, output_dir: str | Path) -> None:
        self.output_dir = Path(output_dir).expanduser().resolve()

    def write_trajectory(self, profile: ExcitationProfile) -> Path:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        path = self.output_dir / "planned_trajectory.csv"
        with path.open("w", encoding="utf-8", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=self._trajectory_fieldnames(profile.dof))
            writer.writeheader()
            for point in profile.points:
                writer.writerow(self._trajectory_row(point))
        return path

    def write_run(
        self,
        *,
        profile: ExcitationProfile,
        backend_name: str,
        model: str,
        samples: list[JointSample],
        execute: bool,
        urdf_path: str | None = None,
    ) -> DatasetManifest:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        planned_path = self.write_trajectory(profile)
        raw_path = self.output_dir / "raw_samples.csv"
        with raw_path.open("w", encoding="utf-8", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=self._sample_fieldnames(profile.dof))
            writer.writeheader()
            for sample in samples:
                writer.writerow(self._sample_row(sample))
        manifest = DatasetManifest(
            profile_name=profile.name,
            backend_name=backend_name,
            model=model,
            dof=profile.dof,
            sample_hz=profile.sample_hz,
            duration_s=profile.duration_s,
            raw_samples=raw_path.name,
            planned_trajectory=planned_path.name,
            urdf_path=urdf_path,
            execute=execute,
            columns={
                "q": vector_column_names("q", profile.dof),
                "dq": vector_column_names("dq", profile.dof),
                "tau_meas": vector_column_names("tau_meas", profile.dof),
                "q_cmd": vector_column_names("q_cmd", profile.dof),
                "dq_cmd": vector_column_names("dq_cmd", profile.dof),
                "ddq_cmd": vector_column_names("ddq_cmd", profile.dof),
                "tau_cmd": vector_column_names("tau_cmd", profile.dof),
            },
            lerobot_contract=build_lerobot_contract(dof=profile.dof, gripper=True),
            profile_metadata=profile.metadata,
            notes=[
                "tau_meas comes from the backend torque estimate.",
                "Offline regressors and base-parameter extraction are delegated to external tools.",
                "The manifest stores a LeRobot-shaped contract so the dataset can be consumed through a unified interface.",
            ],
            tool_hints={
                "pinocchio": "Use computeJointTorqueRegressor(model, data, q, dq, ddq) to build the regressor.",
                "urdfly": "Generate symbolic regressor code from URDF, then read raw/processed CSV.",
                "figaroh": "Map manifest, URDF, and processed CSV into FIGAROH identification configuration.",
                "flobaroid": "Convert processed_samples.csv into FloBaRoID tables and use its optimization/parameter export flow.",
            },
        )
        with (self.output_dir / "manifest.json").open("w", encoding="utf-8") as file:
            json.dump(manifest.to_dict(), file, ensure_ascii=False, sort_keys=True, indent=2)
        with (self.output_dir / "lerobot_contract.json").open("w", encoding="utf-8") as file:
            json.dump(manifest.lerobot_contract, file, ensure_ascii=False, sort_keys=True, indent=2)
        return manifest

    def _trajectory_fieldnames(self, dof: int) -> list[str]:
        return ["t_s", "phase"] + vector_column_names("q_cmd", dof) + vector_column_names("dq_cmd", dof) + vector_column_names("ddq_cmd", dof)

    def _sample_fieldnames(self, dof: int) -> list[str]:
        return (
            ["t_s", "monotonic_s", "source_timestamp_s", "phase"]
            + vector_column_names("q", dof)
            + vector_column_names("dq", dof)
            + vector_column_names("tau_meas", dof)
            + vector_column_names("q_cmd", dof)
            + vector_column_names("dq_cmd", dof)
            + vector_column_names("ddq_cmd", dof)
            + vector_column_names("tau_cmd", dof)
        )

    def _trajectory_row(self, point: TrajectoryPoint) -> dict[str, float | str]:
        row: dict[str, float | str] = {"t_s": point.t_s, "phase": point.phase}
        for prefix, values in (("q_cmd", point.q), ("dq_cmd", point.dq), ("ddq_cmd", point.ddq)):
            for index, value in enumerate(values, start=1):
                row[f"{prefix}_{index}"] = value
        return row

    def _sample_row(self, sample: JointSample) -> dict[str, float | str]:
        row: dict[str, float | str] = {
            "t_s": sample.t_s,
            "monotonic_s": sample.monotonic_s,
            "source_timestamp_s": sample.source_timestamp_s,
            "phase": sample.phase,
        }
        for prefix, values in (
            ("q", sample.q),
            ("dq", sample.dq),
            ("tau_meas", sample.tau_meas),
            ("q_cmd", sample.q_cmd),
            ("dq_cmd", sample.dq_cmd),
            ("ddq_cmd", sample.ddq_cmd),
            ("tau_cmd", sample.tau_cmd),
        ):
            for index, value in enumerate(values, start=1):
                row[f"{prefix}_{index}"] = value
        return row
