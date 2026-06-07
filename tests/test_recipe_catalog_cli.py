import json
import subprocess
import sys
from pathlib import Path

import pytest

from armctrl.recipes import RecipeCatalog
from armctrl.runtime_session import start_fake_runtime_session
from armctrl.safety import SafetyGate


def _project_armctrl_executable() -> str:
    script_dir = Path(sys.executable).resolve().parent
    if sys.platform == "win32":
        return str(script_dir / "armctrl.exe")
    return str(script_dir / "armctrl")


def test_catalog_lists_bounded_agent_recipes() -> None:
    catalog = RecipeCatalog.default()

    names = [recipe.name for recipe in catalog.list_recipes()]

    assert names == [
        "damping",
        "hold-current",
        "home",
        "observe-front",
        "pregrasp-table",
        "retreat-safe",
    ]


def test_safety_gate_blocks_hardware_recipes_without_plan_only() -> None:
    catalog = RecipeCatalog.default()
    recipe = catalog.get("home")

    result = SafetyGate().evaluate(recipe, plan_only=False)

    assert result.allowed is False
    assert result.reason == "hardware recipe requires an execution backend"


def test_cli_lists_recipes_as_json() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "recipe", "list", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_catalog.v1"
    assert payload["recipes"][0]["name"] == "damping"
    assert payload["recipes"][0]["risk_level"] == "hardware"


def test_cli_dry_run_returns_plan_and_safety_gate() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "recipe", "plan", "home", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["plan_only"] is True
    assert payload["schema"] == "armctrl.recipe_plan.v1"
    assert payload["recipe"]["name"] == "home"
    assert payload["safety"]["allowed"] is True
    assert payload["safety"]["required_backend"] == "arx5-interface"
    assert payload["safety"]["dry_run"] is True
    assert payload["safety"]["movement_allowed"] is False
    assert payload["safety"]["risk_explanation"] == [
        "recipe would move hardware when execution is enabled",
        "current response is a dry-run preview only",
        "planned joint targets must pass backend, limit, and workspace checks before execution",
    ]
    assert payload["steps"][0]["kind"] == "joint_target"


def test_packaged_cli_entrypoint_lists_recipes() -> None:
    completed = subprocess.run(
        [_project_armctrl_executable(), "recipe", "list", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["recipes"][-1]["name"] == "retreat-safe"


def test_cli_execute_rejects_without_backend() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "recipe", "execute", "home", "--json"],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.recipe_execution.v1"
    assert payload["recipe"]["name"] == "home"
    assert payload["safety"]["allowed"] is False
    assert payload["safety"]["required_backend"] == "arx5-interface"
    assert payload["executor"]["status"] == "blocked"
    assert payload["executor"]["safety_gate_required"] is True


def test_cli_recipe_plan_can_write_simulation_gated_artifacts(tmp_path: Path) -> None:
    output_dir = tmp_path / "recipe-plan"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--output",
            str(output_dir),
            "--render",
            "trajectory_preview.html",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    manifest_path = output_dir / "manifest.json"
    trajectory_path = output_dir / "planned_trajectory.csv"
    preview_path = output_dir / "trajectory_preview.json"
    eef_seed_path = output_dir / "eef_seed.json"
    render_path = output_dir / "trajectory_preview.html"

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_plan.v1"
    assert payload["artifacts"]["manifest"] == str(manifest_path)
    assert payload["artifacts"]["planned_trajectory"] == str(trajectory_path)
    assert payload["artifacts"]["trajectory_preview"] == str(preview_path)
    assert payload["artifacts"]["eef_seed"] == str(eef_seed_path)
    assert payload["artifacts"]["trajectory_render"] == str(render_path)
    assert payload["agent_runtime_profile"]["schema"] == "armctrl.agent_runtime_profile.v1"
    assert payload["agent_runtime_profile"]["profile"] == "recipe_preset_to_eef_preview"
    assert payload["agent_runtime_profile"]["recipe_plan_dir"] == str(output_dir)
    assert payload["agent_runtime_profile"]["preferred_next_surface"] == "armctrl.eef.synthesize_preview"
    assert payload["artifact_safety"]["allowed"] is True
    assert payload["artifact_safety"]["simulation_check"]["status"] == "pass"
    assert any("recipe export-eef-seed" in step for step in payload["next_steps"])
    assert any("recipe execute home --backend sim" in step for step in payload["next_steps"])
    assert manifest_path.exists()
    assert trajectory_path.exists()
    assert preview_path.exists()
    assert eef_seed_path.exists()
    assert render_path.exists()

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema"] == "armctrl.recipe_plan_manifest.v1"
    assert manifest["recipe"]["name"] == "home"
    assert manifest["safety"]["allowed"] is True
    assert manifest["safety"]["checks"]["simulation_check"]["status"] == "pass"
    eef_seed = json.loads(eef_seed_path.read_text(encoding="utf-8"))
    assert eef_seed["schema"] == "armctrl.recipe_eef_seed.v1"
    assert eef_seed["final_joints"] == [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]


def test_cli_recipe_export_eef_seed_rejects_missing_plan_artifacts(
    tmp_path: Path,
) -> None:
    missing_dir = tmp_path / "missing-recipe-plan"
    missing_dir.mkdir()

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "export-eef-seed",
            "--plan-dir",
            str(missing_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.recipe_eef_seed.v1"
    assert payload["error"]["code"] == "missing_recipe_plan_artifacts"


def test_cli_recipe_export_eef_seed_reads_final_joint_sample(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "recipe-plan-seed"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "export-eef-seed",
            "--plan-dir",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_eef_seed.v1"
    assert payload["recipe"]["name"] == "home"
    assert payload["final_joints"] == [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    assert "--start-joints 0.000000 0.300000 0.300000 0.000000 0.000000 0.000000" in payload["suggested_cli"]["eef_synthesize_preview"]


def test_cli_recipe_export_agent_preset_contract_rejects_missing_plan_artifacts(
    tmp_path: Path,
) -> None:
    missing_dir = tmp_path / "missing-recipe-agent-contract"
    missing_dir.mkdir()

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "export-agent-preset-contract",
            "--plan-dir",
            str(missing_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.recipe_agent_preset_contract.v1"
    assert payload["error"]["code"] == "missing_recipe_plan_artifacts"


def test_cli_recipe_export_agent_preset_contract_collects_handoff_surface(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "recipe-agent-contract"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "export-agent-preset-contract",
            "--plan-dir",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_agent_preset_contract.v1"
    assert payload["movement_allowed"] is False
    assert payload["recipe"]["name"] == "home"
    assert payload["agent_runtime_profile"]["profile"] == "recipe_preset_to_eef_preview"
    assert payload["eef_seed"]["schema"] == "armctrl.recipe_eef_seed.v1"
    assert payload["safety_summary"]["allowed"] is True
    assert payload["safety_summary"]["simulation_check"]["status"] == "pass"
    assert payload["required_artifacts"]["eef_seed"].endswith("eef_seed.json")
    assert payload["ordered_steps"][0]["id"] == "export_eef_seed"
    assert payload["ordered_steps"][1]["depends_on"] == ["export_eef_seed"]
    assert any("recipe export-eef-seed" in step for step in payload["next_steps"])
    assert any("--recipe-plan-dir" in step for step in payload["next_steps"])


def test_cli_recipe_execute_accepts_sim_backend_after_safety_preview(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "recipe-exec"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "execute",
            "home",
            "--backend",
            "sim",
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_execution.v1"
    assert payload["executor"]["status"] == "ready"
    assert payload["executor"]["hardware_backend"] == "sim"
    assert payload["simulation_gate"]["allowed"] is True
    assert payload["artifacts"]["manifest"] == str(output_dir / "manifest.json")
    assert payload["handoff"]["recipe_plan_dir"] == str(output_dir)
    assert payload["agent_runtime_profile"]["schema"] == "armctrl.agent_runtime_profile.v1"
    assert payload["agent_runtime_profile"]["profile"] == "recipe_preset_to_eef_preview"
    assert payload["agent_runtime_profile"]["recipe_plan_dir"] == str(output_dir)
    assert payload["handoff"]["eef_seed"]["schema"] == "armctrl.recipe_eef_seed.v1"
    assert payload["handoff"]["eef_seed"]["final_joints"] == [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    assert any("recipe export-eef-seed" in step for step in payload["next_steps"])
    assert any("--recipe-plan-dir" in step for step in payload["next_steps"])


def test_cli_recipe_runtime_smoke_fake_replays_checked_plan_with_motion_runtime(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "recipe-runtime-smoke-plan"
    runtime_log = tmp_path / "recipe_runtime_smoke.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--sample-hz",
            "50",
            "--duration",
            "0.1",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "runtime-smoke-fake",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(runtime_log),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(runtime_log.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_runtime_smoke.v1"
    assert payload["movement_allowed"] is False
    assert payload["hardware_motion"] is False
    assert payload["runtime"]["backend"] == "fake"
    assert payload["runtime"]["owner"] == "motion_runtime"
    assert payload["recipe_plan_dir"] == str(plan_dir)
    assert payload["safety"]["allowed"] is True
    assert payload["motion_runtime"]["producer"] == "recipe"
    assert payload["motion_runtime"]["mode"] == "trajectory_replay"
    assert payload["motion_runtime"]["trajectory_sample_hz"] == 50.0
    assert payload["motion_runtime"]["actual_send_hz"] == 50.0
    assert payload["motion_runtime"]["send_jitter_ms_p95"] == pytest.approx(0.0)
    assert payload["motion_runtime"]["landing_mode"] == "hold"
    assert payload["motion_runtime"]["sample_count"] == 6
    assert payload["artifacts"]["runtime_log"] == str(runtime_log)
    assert written == payload


def test_cli_recipe_runtime_smoke_fake_rejects_runtime_session_artifact(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "recipe-runtime-smoke-plan"
    runtime_session_artifact = tmp_path / "runtime-session.json"
    safe_center = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    original_session = start_fake_runtime_session(
        q_current=safe_center,
        safe_center=safe_center,
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=5.0,
    )
    runtime_session_artifact.write_text(
        json.dumps(original_session, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--sample-hz",
            "50",
            "--duration",
            "0.1",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "runtime-smoke-fake",
            "--plan-dir",
            str(plan_dir),
            "--runtime-session-artifact",
            str(runtime_session_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    unchanged_session = json.loads(runtime_session_artifact.read_text(encoding="utf-8"))

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.recipe_runtime_smoke.v1"
    assert payload["movement_command_sent"] is False
    assert payload["reason"] == (
        "recipe runtime-smoke-fake is pure fake; use recipe runtime-submit "
        "to enqueue a live runtime owner command"
    )
    assert payload["next_gate"] == "run armctrl recipe runtime-submit --runtime-session-artifact"
    assert unchanged_session == original_session


def test_cli_recipe_runtime_submit_queues_live_runtime_owner(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "recipe-runtime-submit-plan"
    submit_log = tmp_path / "recipe_runtime_submit.json"
    runtime_session_artifact = tmp_path / "runtime-session.json"
    safe_center = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    runtime_session_artifact.write_text(
        json.dumps(
            start_fake_runtime_session(
                q_current=safe_center,
                safe_center=safe_center,
                send_hz=50.0,
                hold_hz=50.0,
                max_joint_step_rad=0.01,
                max_heartbeat_age_s=5.0,
            ),
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--sample-hz",
            "50",
            "--duration",
            "0.1",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "runtime-submit",
            "--plan-dir",
            str(plan_dir),
            "--runtime-session-artifact",
            str(runtime_session_artifact),
            "--output",
            str(submit_log),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    command = json.loads(Path(payload["runtime_command"]["artifacts"]["command"]).read_text(encoding="utf-8"))
    written = json.loads(submit_log.read_text(encoding="utf-8"))

    assert payload["status"] == "queued"
    assert payload["schema"] == "armctrl.recipe_runtime_submit.v1"
    assert payload["movement_command_sent"] is False
    assert payload["runtime"]["single_owner_runtime_session"] is True
    assert payload["runtime"]["owner"] == "recipe"
    assert payload["runtime"]["mode"] == "trajectory_replay"
    assert payload["runtime_command"]["sample_count"] == 6
    assert payload["runtime_command"]["send_hz"] == 50.0
    assert payload["start_pose_policy"] == "live_hold"
    assert command["owner"] == "recipe"
    assert command["kind"] == "trajectory"
    assert command["start_pose_policy"] == "live_hold"
    assert command["q_points"][0] == safe_center
    assert written == payload


def test_cli_recipe_runtime_submit_explicit_q_requires_preposition(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "recipe-runtime-submit-explicit-plan"
    runtime_session_artifact = tmp_path / "runtime-session.json"
    safe_center = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    explicit_start = [0.1, 0.3, 0.3, 0.0, 0.0, 0.0]
    runtime_session_artifact.write_text(
        json.dumps(
            start_fake_runtime_session(
                q_current=safe_center,
                safe_center=safe_center,
                send_hz=50.0,
                hold_hz=50.0,
                max_joint_step_rad=0.01,
                max_heartbeat_age_s=5.0,
            ),
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--sample-hz",
            "50",
            "--duration",
            "0.1",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    manifest_path = plan_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["request"]["start_joints"] = explicit_start
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    trajectory_path = plan_dir / "planned_trajectory.csv"
    lines = trajectory_path.read_text(encoding="utf-8").splitlines()
    header = lines[0]
    rows = lines[1:]
    patched_rows = []
    for row in rows:
        values = row.split(",")
        patched_rows.append(
            ",".join([values[0], *[f"{value:.9f}" for value in explicit_start]])
        )
    trajectory_path.write_text(
        "\n".join([header, *patched_rows]) + "\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "runtime-submit",
            "--plan-dir",
            str(plan_dir),
            "--runtime-session-artifact",
            str(runtime_session_artifact),
            "--start-pose-policy",
            "explicit_q",
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.recipe_runtime_submit.v1"
    assert payload["start_pose_policy"] == "explicit_q"
    assert payload["reason"] == (
        "explicit_q start pose requires runtime q_hold to be pre-positioned"
    )
    assert payload["start_pose_guard"]["policy"] == "explicit_q"
    assert payload["start_pose_guard"]["failed_checks"] == ["q_hold_close_to_explicit_q"]


def test_cli_recipe_runtime_submit_rejects_busy_runtime_owner(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "recipe-runtime-submit-plan"
    runtime_session_artifact = tmp_path / "runtime-session.json"
    safe_center = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    session = start_fake_runtime_session(
        q_current=safe_center,
        safe_center=safe_center,
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=5.0,
    )
    session["mode"] = "agent_servo"
    session["owner"] = "agent"
    session["owner_lease"] = {
        "schema": "armctrl.arm_runtime_owner_lease.v1",
        "runtime_session_id": session["runtime_session_id"],
        "owner": "agent",
        "mode": "agent_servo",
        "heartbeat_timeout_s": 1.0,
    }
    runtime_session_artifact.write_text(
        json.dumps(session, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--sample-hz",
            "50",
            "--duration",
            "0.1",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "runtime-submit",
            "--plan-dir",
            str(plan_dir),
            "--runtime-session-artifact",
            str(runtime_session_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["reason"] == "runtime is owned by agent"
    assert payload["runtime"]["owner"] == "recipe"
    busy_session = json.loads(runtime_session_artifact.read_text(encoding="utf-8"))
    assert busy_session["owner"] == "agent"


def test_cli_recipe_runtime_smoke_fake_rejects_runtime_session_even_when_busy(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "recipe-runtime-smoke-plan"
    runtime_session_artifact = tmp_path / "runtime-session.json"
    safe_center = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    session = start_fake_runtime_session(
        q_current=safe_center,
        safe_center=safe_center,
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=5.0,
    )
    session["mode"] = "agent_servo"
    session["owner"] = "agent"
    session["owner_lease"] = {
        "schema": "armctrl.arm_runtime_owner_lease.v1",
        "runtime_session_id": session["runtime_session_id"],
        "owner": "agent",
        "mode": "agent_servo",
        "heartbeat_timeout_s": 1.0,
    }
    runtime_session_artifact.write_text(
        json.dumps(session, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--sample-hz",
            "50",
            "--duration",
            "0.1",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "runtime-smoke-fake",
            "--plan-dir",
            str(plan_dir),
            "--runtime-session-artifact",
            str(runtime_session_artifact),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["reason"] == (
        "recipe runtime-smoke-fake is pure fake; use recipe runtime-submit "
        "to enqueue a live runtime owner command"
    )
    busy_session = json.loads(runtime_session_artifact.read_text(encoding="utf-8"))
    assert busy_session["owner"] == "agent"


def test_cli_recipe_status_reports_no_hardware_session() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "recipe", "status", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_executor_status.v1"
    assert payload["executor"]["state"] == "idle"
    assert payload["executor"]["hardware_backend"] == "not_configured"


def test_cli_recipe_cancel_is_safe_when_no_hardware_session() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "recipe", "cancel", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.recipe_executor_cancel.v1"
    assert payload["executor"]["state"] == "idle"
    assert payload["executor"]["action"] == "no_active_session"
