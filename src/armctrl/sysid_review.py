from __future__ import annotations

import csv
from dataclasses import dataclass
import html
import json
from pathlib import Path
from typing import Any

import numpy as np

from armctrl.sysid import (
    _evaluate_trajectory_derivative_limits,
    _evaluate_trajectory_steps,
)
from armctrl.sysid_trajectory_backend import read_sysid_profile_safety
from armctrl.simulation import TrajectoryPreviewer
from armctrl.workspace import WorkspaceSafetyConfig, link_frame_positions


@dataclass(frozen=True)
class SysIdOfflineReviewRequest:
    plan_dir: Path
    output_dir: Path
    urdf_path: Path
    safe_config_path: Path
    trajectory_path: Path | None = None


class SysIdOfflineReviewer:
    def review(self, request: SysIdOfflineReviewRequest) -> dict[str, Any]:
        manifest_path, manifest = _read_candidate_manifest(request.plan_dir)
        profile_name = str(manifest.get("profile", {}).get("name", "fourier_multisine"))
        dof = int(manifest.get("request", {}).get("dof", 6))
        trajectory_path = _resolve_execution_trajectory_path(
            request,
            manifest=manifest,
        )
        request.output_dir.mkdir(parents=True, exist_ok=True)
        rows = _read_rows(trajectory_path)
        q_matrix = _q_matrix(rows, dof=dof)
        times = _time_vector(rows)
        dq_matrix = _derivative_matrix(q_matrix, times)
        ddq_matrix = _derivative_matrix(dq_matrix, times)
        profile_safety = read_sysid_profile_safety(
            request.safe_config_path,
            profile_name=profile_name,
        )
        step_decision = _evaluate_trajectory_steps(
            rows,
            dof=dof,
            max_joint_step_rad=float(profile_safety["max_joint_step_rad"]),
        )
        velocity_decision = _evaluate_trajectory_derivative_limits(
            rows,
            dof=dof,
            limits=[float(value) for value in profile_safety["oed_velocity_limits_rad_s"]],
            derivative_order=1,
            check_name="oed_velocity_limits_rad_s",
        )
        acceleration_decision = _evaluate_trajectory_derivative_limits(
            rows,
            dof=dof,
            limits=[
                float(value)
                for value in profile_safety["oed_acceleration_limits_rad_s2"]
            ],
            derivative_order=2,
            check_name="oed_acceleration_limits_rad_s2",
        )
        execution_gate = _execution_gate(
            manifest,
            step_decision=step_decision,
            velocity_decision=velocity_decision,
            acceleration_decision=acceleration_decision,
        )
        preview_html_path = request.output_dir / "trajectory_preview.html"
        preview = TrajectoryPreviewer().preview(
            trajectory_path=trajectory_path,
            urdf_path=request.urdf_path,
            safe_config_path=request.safe_config_path,
            render_path=preview_html_path,
        )
        clearance = _clearance_summary(
            request.urdf_path,
            WorkspaceSafetyConfig.from_yaml(request.safe_config_path),
            q_matrix=q_matrix,
            preview=preview,
        )
        smoothness = _smoothness_summary(
            rows=rows,
            q_matrix=q_matrix,
            dq_matrix=dq_matrix,
            ddq_matrix=ddq_matrix,
            acceleration_limit=max(
                abs(float(value))
                for value in profile_safety["oed_acceleration_limits_rad_s2"]
            ),
        )
        condition = _condition_summary(manifest)
        optimizer = _optimizer_summary(manifest)
        gate_state, gate_reasons = _review_gate_state(
            condition=condition,
            optimizer=optimizer,
            smoothness=smoothness,
            clearance=clearance,
            execution_gate=execution_gate,
        )
        artifacts = {
            "review_summary": str(request.output_dir / "review_summary.json"),
            "review_report": str(request.output_dir / "review_report.md"),
            "hardware_smoke_plan": str(request.output_dir / "hardware_smoke_plan.md"),
            "q_plot": str(request.output_dir / "joint_q.svg"),
            "dq_plot": str(request.output_dir / "joint_dq.svg"),
            "ddq_plot": str(request.output_dir / "joint_ddq.svg"),
            "preview_json": str(request.output_dir / "trajectory_preview.json"),
            "preview_html": str(preview_html_path),
        }
        _write_joint_plot(
            Path(artifacts["q_plot"]),
            times=times,
            matrix=q_matrix,
            title="X5 SysID execution trajectory q",
            unit="rad",
        )
        _write_joint_plot(
            Path(artifacts["dq_plot"]),
            times=times,
            matrix=dq_matrix,
            title="X5 SysID execution trajectory dq",
            unit="rad/s",
        )
        _write_joint_plot(
            Path(artifacts["ddq_plot"]),
            times=times,
            matrix=ddq_matrix,
            title="X5 SysID execution trajectory ddq",
            unit="rad/s^2",
        )
        result: dict[str, Any] = {
            "schema": "armctrl.sysid_offline_review.v1",
            "movement_allowed": False,
            "hardware_execution_eligible": False,
            "hardware_smoke_plan_allowed": gate_state == "hardware_smoke_plan_ready",
            "gate_state": gate_state,
            "gate_reasons": gate_reasons,
            "plan_dir": str(request.plan_dir),
            "trajectory_path": str(trajectory_path),
            "profile": profile_name,
            "condition": condition,
            "optimizer": optimizer,
            "smoothness": smoothness,
            "clearance": clearance,
            "execution_gate": execution_gate,
            "simulation_preview": preview,
            "artifacts": artifacts,
            "next_gate": (
                "review hardware_smoke_plan.md and run gravity/friction smoke first"
                if gate_state == "hardware_smoke_plan_ready"
                else "fix offline review failures before hardware smoke planning"
            ),
        }
        Path(artifacts["preview_json"]).write_text(
            json.dumps(preview, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        Path(artifacts["hardware_smoke_plan"]).write_text(
            _hardware_smoke_plan_markdown(result),
            encoding="utf-8",
        )
        Path(artifacts["review_report"]).write_text(
            _review_report_markdown(result),
            encoding="utf-8",
        )
        Path(artifacts["review_summary"]).write_text(
            json.dumps(result, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return result


def _resolve_execution_trajectory_path(
    request: SysIdOfflineReviewRequest,
    *,
    manifest: dict[str, Any],
) -> Path:
    if request.trajectory_path is not None:
        return request.trajectory_path
    artifacts = manifest.get("artifacts", {})
    if isinstance(artifacts, dict) and isinstance(artifacts.get("execution_trajectory"), str):
        path = Path(artifacts["execution_trajectory"])
        if path.is_absolute() or path.exists():
            return path
        return request.plan_dir / path
    for key in ("source_execution_trajectory", "recommended_candidate"):
        raw_path = manifest.get(key)
        if not isinstance(raw_path, str):
            continue
        path = Path(raw_path)
        if path.is_absolute() or path.exists():
            return path
        return request.plan_dir / path
    return request.plan_dir / "execution_trajectory.csv"


def _read_candidate_manifest(plan_dir: Path) -> tuple[Path, dict[str, Any]]:
    manifest_path = plan_dir / "manifest.json"
    if manifest_path.exists():
        return manifest_path, json.loads(manifest_path.read_text(encoding="utf-8"))
    frozen_manifest_path = plan_dir / "best_candidate_manifest.json"
    if frozen_manifest_path.exists():
        raw = json.loads(frozen_manifest_path.read_text(encoding="utf-8"))
        return frozen_manifest_path, _manifest_from_frozen_candidate(raw)
    raise FileNotFoundError(
        f"expected manifest.json or best_candidate_manifest.json in {plan_dir}"
    )


def _manifest_from_frozen_candidate(raw: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema": raw.get("schema"),
        "profile": {"name": "fourier_multisine"},
        "request": {
            "dof": 6,
            "safe_config_path": "configs/x5.safe.yaml",
            "urdf_path": "configs/models/X5_camera.urdf",
        },
        "safety": {
            "allowed": bool(raw.get("safety_allowed", False)),
            "reason": raw.get("next_gate", "review_optimizer_convergence_offline"),
            "checks": {},
        },
        "trajectory_backend": {
            "condition_metric": raw.get(
                "target_condition_metric",
                "figaroh_base_regressor",
            ),
            "condition_number": raw.get("condition_number"),
            "figaroh_base_condition_number": raw.get(
                "base_regressor_condition_number"
            ),
            "pinocchio_effective_condition_number": raw.get(
                "pinocchio_effective_condition_number"
            ),
            "rank": raw.get("rank"),
            "oed_quality_gate": {
                "status": raw.get("oed_quality_status", "not_reported"),
                "reasons": [str(raw.get("next_gate", "review_optimizer_convergence_offline"))],
                "primary_metric": raw.get(
                    "target_condition_metric",
                    "figaroh_base_regressor",
                ),
            },
            "candidate_source": {
                "generated_by": {
                    "optimizer_convergence": {
                        "status": "fail",
                        "reason": raw.get(
                            "next_gate",
                            "review_optimizer_convergence_offline",
                        ),
                        "constraint_violation_unscaled": 0.0,
                    }
                }
            },
        },
        "artifacts": {
            "execution_trajectory": raw.get("source_execution_trajectory")
            or raw.get("recommended_candidate"),
        },
        "source_execution_trajectory": raw.get("source_execution_trajectory"),
        "recommended_candidate": raw.get("recommended_candidate"),
    }


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise ValueError(f"trajectory has no rows: {path}")
    return rows


def _time_vector(rows: list[dict[str, str]]) -> np.ndarray:
    return np.asarray([float(row["time_s"]) for row in rows], dtype=float)


def _q_matrix(rows: list[dict[str, str]], *, dof: int) -> np.ndarray:
    return np.asarray(
        [
            [float(row[f"q_cmd_{joint_index + 1}"]) for joint_index in range(dof)]
            for row in rows
        ],
        dtype=float,
    )


def _derivative_matrix(matrix: np.ndarray, times: np.ndarray) -> np.ndarray:
    if matrix.size == 0:
        return matrix
    if len(times) <= 1:
        return np.zeros_like(matrix)
    return np.gradient(matrix, times, axis=0, edge_order=1)


def _execution_gate(
    manifest: dict[str, Any],
    *,
    step_decision: object,
    velocity_decision: object,
    acceleration_decision: object,
) -> dict[str, Any]:
    manifest_safety = manifest.get("safety", {})
    manifest_allowed = (
        bool(manifest_safety.get("allowed", False))
        if isinstance(manifest_safety, dict)
        else False
    )
    checks = {
        "trajectory_step_check": {
            "status": step_decision.status,
            "violations": step_decision.violations,
        },
        "trajectory_velocity_check": {
            "status": velocity_decision.status,
            "violations": velocity_decision.violations,
        },
        "trajectory_acceleration_check": {
            "status": acceleration_decision.status,
            "violations": acceleration_decision.violations,
        },
    }
    status = (
        "pass"
        if manifest_allowed and all(check["status"] == "pass" for check in checks.values())
        else "fail"
    )
    return {
        "status": status,
        "manifest_safety_allowed": manifest_allowed,
        **checks,
    }


def _clearance_summary(
    urdf_path: Path,
    config: WorkspaceSafetyConfig,
    *,
    q_matrix: np.ndarray,
    preview: dict[str, Any],
) -> dict[str, Any]:
    q_samples = [tuple(float(value) for value in row) for row in q_matrix.tolist()]
    frames = link_frame_positions(urdf_path, samples=q_samples)
    tracked_links = tuple(config.simulation_link_frames or ("link5", "link6", "eef_link"))
    link_z: dict[str, list[float]] = {}
    for frame in frames:
        for link_name in tracked_links:
            if link_name in frame:
                link_z.setdefault(link_name, []).append(float(frame[link_name][2]))
    link_ranges = {
        name: {
            "min_z_m": min(values),
            "max_z_m": max(values),
            "min_clearance_to_workspace_min_m": min(values) - config.workspace_min_m[2],
        }
        for name, values in link_z.items()
    }
    safety = preview.get("safety", {})
    clearance_check = safety.get("clearance_check", {}) if isinstance(safety, dict) else {}
    zone_check = safety.get("zone_check", {}) if isinstance(safety, dict) else {}
    status = (
        "pass"
        if clearance_check.get("status") == "pass" and zone_check.get("status") == "pass"
        else "fail"
    )
    min_clearance = min(
        (
            float(range_payload["min_clearance_to_workspace_min_m"])
            for range_payload in link_ranges.values()
        ),
        default=None,
    )
    return {
        "status": status,
        "min_clearance_m": min_clearance,
        "link_frame_z_range_m": link_ranges,
        "clearance_check": clearance_check,
        "zone_check": zone_check,
    }


def _smoothness_summary(
    *,
    rows: list[dict[str, str]],
    q_matrix: np.ndarray,
    dq_matrix: np.ndarray,
    ddq_matrix: np.ndarray,
    acceleration_limit: float,
) -> dict[str, Any]:
    max_step = float(np.max(np.abs(np.diff(q_matrix, axis=0)))) if len(q_matrix) > 1 else 0.0
    max_velocity = float(np.max(np.abs(dq_matrix))) if dq_matrix.size else 0.0
    max_acceleration = float(np.max(np.abs(ddq_matrix))) if ddq_matrix.size else 0.0
    status = "pass" if max_acceleration <= acceleration_limit * 1.05 else "fail"
    return {
        "status": status,
        "sample_count": len(rows),
        "max_joint_step_rad": max_step,
        "max_velocity_rad_s": max_velocity,
        "max_acceleration_rad_s2": max_acceleration,
        "acceleration_review_limit_rad_s2": acceleration_limit,
    }


def _condition_summary(manifest: dict[str, Any]) -> dict[str, Any]:
    backend = manifest.get("trajectory_backend", {})
    if not isinstance(backend, dict):
        backend = {}
    condition = backend.get("condition_number")
    figaroh_base = backend.get("figaroh_base_condition_number")
    pinocchio_effective = backend.get("pinocchio_effective_condition_number")
    target = 100.0
    return {
        "metric": backend.get("condition_metric"),
        "condition_number": condition,
        "figaroh_base_condition_number": figaroh_base,
        "pinocchio_effective_condition_number": pinocchio_effective,
        "rank": backend.get("rank"),
        "target_condition_number": target,
        "target_status": (
            "pass"
            if isinstance(condition, int | float) and float(condition) <= target
            else "fail"
        ),
    }


def _optimizer_summary(manifest: dict[str, Any]) -> dict[str, Any]:
    backend = manifest.get("trajectory_backend", {})
    if not isinstance(backend, dict):
        backend = {}
    source = backend.get("candidate_source", {})
    generated_by = source.get("generated_by", {}) if isinstance(source, dict) else {}
    convergence = (
        generated_by.get("optimizer_convergence", {})
        if isinstance(generated_by, dict)
        else {}
    )
    diagnostics = (
        generated_by.get("optimizer_diagnostics", {})
        if isinstance(generated_by, dict)
        else {}
    )
    return {
        "status": convergence.get("status", "not_reported"),
        "reason": convergence.get("reason", "optimizer_convergence_not_reported"),
        "constraint_violation_unscaled": convergence.get(
            "constraint_violation_unscaled",
            diagnostics.get("constraint_violation_unscaled"),
        ),
        "diagnostics": diagnostics,
        "oed_quality_gate": backend.get("oed_quality_gate"),
    }


def _review_gate_state(
    *,
    condition: dict[str, Any],
    optimizer: dict[str, Any],
    smoothness: dict[str, Any],
    clearance: dict[str, Any],
    execution_gate: dict[str, Any],
) -> tuple[str, list[str]]:
    reasons: list[str] = []
    if condition.get("target_status") != "pass":
        reasons.append("condition_target_not_met")
    if condition.get("rank") not in {36, "36"}:
        reasons.append("rank_not_confirmed")
    if optimizer.get("constraint_violation_unscaled") not in {0, 0.0, None}:
        reasons.append("optimizer_constraint_violation_nonzero")
    if smoothness.get("status") != "pass":
        reasons.append("smoothness_failed")
    if clearance.get("status") != "pass":
        reasons.append("clearance_failed")
    if execution_gate.get("status") != "pass":
        reasons.append("execution_gate_failed")
    if reasons:
        return "offline_candidate", reasons
    return "hardware_smoke_plan_ready", ["requires_gravity_and_friction_smoke_before_full_fourier"]


def _write_joint_plot(
    output_path: Path,
    *,
    times: np.ndarray,
    matrix: np.ndarray,
    title: str,
    unit: str,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    width = 960
    height = 420
    left = 64
    top = 48
    plot_width = 840
    plot_height = 280
    values = matrix.flatten()
    min_value = float(np.min(values)) if values.size else 0.0
    max_value = float(np.max(values)) if values.size else 1.0
    if min_value == max_value:
        min_value -= 1.0
        max_value += 1.0
    min_time = float(times[0]) if len(times) else 0.0
    max_time = float(times[-1]) if len(times) else 1.0
    if min_time == max_time:
        max_time = min_time + 1.0
    colors = ("#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2")
    polylines: list[str] = []
    for joint_index in range(matrix.shape[1] if matrix.ndim == 2 else 0):
        points = []
        for sample_index, time_s in enumerate(times):
            x = left + (float(time_s) - min_time) * plot_width / (max_time - min_time)
            y = top + plot_height - (
                (float(matrix[sample_index, joint_index]) - min_value)
                * plot_height
                / (max_value - min_value)
            )
            points.append(f"{x:.2f},{y:.2f}")
        color = colors[joint_index % len(colors)]
        polylines.append(
            f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="2"/>'
        )
        polylines.append(
            f'<text x="{left + joint_index * 112}" y="372" font-family="Arial" font-size="12" fill="{color}">joint_{joint_index + 1}</text>'
        )
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#ffffff"/>
  <text x="{left}" y="28" font-family="Arial" font-size="18" fill="#0f172a">{html.escape(title)}</text>
  <text x="{left}" y="396" font-family="Arial" font-size="12" fill="#475569">time_s {min_time:.3f} to {max_time:.3f}; unit: {html.escape(unit)}</text>
  <line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_height}" stroke="#94a3b8"/>
  <line x1="{left}" y1="{top + plot_height}" x2="{left + plot_width}" y2="{top + plot_height}" stroke="#94a3b8"/>
  <text x="8" y="{top + 12}" font-family="Arial" font-size="11" fill="#475569">{max_value:.3f}</text>
  <text x="8" y="{top + plot_height}" font-family="Arial" font-size="11" fill="#475569">{min_value:.3f}</text>
  {"".join(polylines)}
</svg>
"""
    output_path.write_text(svg, encoding="utf-8")


def _review_report_markdown(result: dict[str, Any]) -> str:
    allowed_text = (
        "允许进入 hardware smoke plan"
        if result["hardware_smoke_plan_allowed"]
        else "不允许进入 hardware smoke plan"
    )
    condition = result["condition"]
    smoothness = result["smoothness"]
    clearance = result["clearance"]
    execution = result["execution_gate"]
    return "\n".join(
        [
            "# X5 SysID Fourier 离线硬件放行审查",
            "",
            f"- gate_state: `{result['gate_state']}`",
            f"- movement_allowed: `{str(result['movement_allowed']).lower()}`",
            f"- hardware_execution_eligible: `{str(result['hardware_execution_eligible']).lower()}`",
            f"- 结论: {allowed_text}",
            "",
            "## 条件数",
            "",
            f"- FIGAROH base condition: `{condition.get('figaroh_base_condition_number')}`",
            f"- Pinocchio effective condition: `{condition.get('pinocchio_effective_condition_number')}`",
            f"- rank: `{condition.get('rank')}`",
            f"- primary metric: `{condition.get('metric')}`",
            "",
            "## 优化器",
            "",
            f"- status: `{result['optimizer'].get('status')}`",
            f"- reason: `{result['optimizer'].get('reason')}`",
            f"- constraint_violation_unscaled: `{result['optimizer'].get('constraint_violation_unscaled')}`",
            "",
            "## 轨迹平滑性",
            "",
            f"- status: `{smoothness['status']}`",
            f"- max_joint_step_rad: `{smoothness['max_joint_step_rad']}`",
            f"- max_velocity_rad_s: `{smoothness['max_velocity_rad_s']}`",
            f"- max_acceleration_rad_s2: `{smoothness['max_acceleration_rad_s2']}`",
            "",
            "## 空间/碰撞预览",
            "",
            f"- clearance status: `{clearance['status']}`",
            f"- min_clearance_m: `{clearance['min_clearance_m']}`",
            f"- execution gate: `{execution['status']}`",
            "",
            "## 审查材料",
            "",
            f"- q plot: `{result['artifacts']['q_plot']}`",
            f"- dq plot: `{result['artifacts']['dq_plot']}`",
            f"- ddq plot: `{result['artifacts']['ddq_plot']}`",
            f"- preview html: `{result['artifacts']['preview_html']}`",
            f"- hardware smoke plan: `{result['artifacts']['hardware_smoke_plan']}`",
            "",
        ]
    )


def _hardware_smoke_plan_markdown(result: dict[str, Any]) -> str:
    allowed = str(result["hardware_smoke_plan_allowed"]).lower()
    return "\n".join(
        [
            "# X5 SysID Hardware Smoke Plan",
            "",
            "This artifact is a non-executing plan. It does not open CAN, import the SDK, or move hardware.",
            "",
            f"- movement_allowed: false",
            f"- hardware_execution_eligible: false",
            f"- hardware_smoke_plan_allowed: {allowed}",
            f"- gate_state: `{result['gate_state']}`",
            "",
            "## 1. Gravity Smoke",
            "",
            "Purpose: verify joint sign, SDK communication, damping landing, logs, and current baseline with a reviewed low-speed execution trajectory.",
            "",
            "Command draft (compile reviewed CSV, then submit through the live runtime owner):",
            "",
            "```bash",
            "uv run armctrl sysid compile-runtime --execution-trajectory <gravity_execution_trajectory.csv> --dof 6 --sample-hz 100 --expected-q-start 0 0.30 0.30 0 0 0 --owner sysid --start-pose-policy live_hold --send-hz 100 --max-tracking-error-rad 0.04 --output runs/ident-runtime-gravity-smoke/compile --json",
            "uv run armctrl motion submit joint-trajectory --session-artifact \"$RUN_DIR/runtime_session.json\" --compiled-command runs/ident-runtime-gravity-smoke/compile/compiled_motion_command.json --max-heartbeat-age-s 1.0 --heartbeat-timeout-s 3.0 --max-tracking-error-rad 0.04 --output runs/ident-runtime-gravity-smoke/runtime_submit.json --json",
            "```",
            "",
            "Fallback: Ctrl-C/fault must land in damping; stop if current spikes, table contact, or sign mismatch appears.",
            "",
            "## 2. Friction Smoke",
            "",
            "Purpose: verify positive/negative velocity data quality and current envelope before dynamic excitation.",
            "",
            "Command draft (compile reviewed CSV, then submit through the live runtime owner):",
            "",
            "```bash",
            "uv run armctrl sysid compile-runtime --execution-trajectory <friction_execution_trajectory.csv> --dof 6 --sample-hz 100 --expected-q-start 0 0.30 0.30 0 0 0 --owner sysid --start-pose-policy live_hold --send-hz 100 --max-tracking-error-rad 0.04 --output runs/ident-runtime-friction-smoke/compile --json",
            "uv run armctrl motion submit joint-trajectory --session-artifact \"$RUN_DIR/runtime_session.json\" --compiled-command runs/ident-runtime-friction-smoke/compile/compiled_motion_command.json --max-heartbeat-age-s 1.0 --heartbeat-timeout-s 3.0 --max-tracking-error-rad 0.04 --output runs/ident-runtime-friction-smoke/runtime_submit.json --json",
            "```",
            "",
            "Fallback: stop before Fourier if current, velocity tracking, or endpoint clearance is abnormal.",
            "",
            "## 3. Fourier Reduced Smoke",
            "",
            "Purpose: run a reduced-risk version of the reviewed Fourier candidate only after gravity and friction smoke pass.",
            "",
            "Command draft:",
            "",
            "```bash",
            "uv run armctrl sysid compile-runtime --execution-trajectory <reduced_execution_trajectory.csv> --dof 6 --sample-hz 100 --expected-q-start 0 0.30 0.30 0 0 0 --owner sysid --start-pose-policy live_hold --send-hz 100 --max-tracking-error-rad 0.06 --output runs/ident-runtime-fourier-reduced/compile --json",
            "uv run armctrl motion submit joint-trajectory --session-artifact \"$RUN_DIR/runtime_session.json\" --compiled-command runs/ident-runtime-fourier-reduced/compile/compiled_motion_command.json --max-heartbeat-age-s 1.0 --heartbeat-timeout-s 3.0 --max-tracking-error-rad 0.06 --output runs/ident-runtime-fourier-reduced/runtime_submit.json --json",
            "```",
            "",
            "Fallback: keep full Fourier blocked if reduced smoke shows over-current, contact, or unexpected posture.",
            "",
            "## 4. Fourier Full Collection",
            "",
            "Purpose: collect the reviewed low-condition OED candidate only after the first three gates pass and are recorded.",
            "",
            "Command draft (compile reviewed CSV, then submit through the live runtime owner):",
            "",
            "```bash",
            "uv run armctrl sysid compile-runtime --execution-trajectory <full_execution_trajectory.csv> --dof 6 --sample-hz 100 --expected-q-start 0 0.30 0.30 0 0 0 --owner sysid --start-pose-policy live_hold --send-hz 100 --max-tracking-error-rad 0.08 --output runs/ident-runtime-fourier-full/compile --json",
            "uv run armctrl motion submit joint-trajectory --session-artifact \"$RUN_DIR/runtime_session.json\" --compiled-command runs/ident-runtime-fourier-full/compile/compiled_motion_command.json --max-heartbeat-age-s 1.0 --heartbeat-timeout-s 3.0 --max-tracking-error-rad 0.08 --output runs/ident-runtime-fourier-full/runtime_submit.json --json",
            "```",
            "",
            "Observe: joint2 current, over-current logs, table/base clearance, end-effector dip, tracking error, and damping/fault landing.",
            "",
        ]
    )
