from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "lab_agent_runtime_smoke.sh"
FOURIER_SCRIPT = ROOT / "scripts" / "lab_fourier_sysid.sh"


def test_lab_agent_runtime_smoke_script_documents_runtime_agent_flow() -> None:
    assert SCRIPT.exists()

    text = SCRIPT.read_text(encoding="utf-8")
    assert "scripts/lab_agent_runtime_smoke.sh start" in text
    assert "scripts/lab_agent_runtime_smoke.sh plan  # optional EEF contract review" in text
    assert "scripts/lab_agent_runtime_smoke.sh run" in text
    assert "scripts/lab_agent_runtime_smoke.sh run-eef" in text
    assert "scripts/lab_agent_runtime_smoke.sh check-eef" in text
    assert "armctrl motion submit joint-intent" in text
    assert "armctrl motion submit joint-trajectory" in text
    assert "armctrl motion compile joint-trajectory" in text
    assert "armctrl motion submit eef-delta" in text
    assert "armctrl motion result" in text
    assert "armctrl console status" in text
    assert "continuous-owner servo" in text
    assert "ARMCTRL_ALLOW_DISCONNECTED_EEF_TAKEOVER=1" in text
    assert "--start-pose-policy live_hold" in text
    assert "motion kind=joint-intent" in text
    assert "owner=agent" in text
    assert "Visible-large profile v10" in text
    assert "joint1 +3.00 rad over 45.0 s" in text
    assert "avoids joint2/joint3 droop-sensitive motion" in text
    assert "default EEF delta smoke uses +750 mm" in text
    assert 'SMOKE_PROFILE_VERSION_CURRENT="10"' in text
    assert 'AGENT_Q_TARGET_DEFAULT="3.00 0.3 0.3 0.0 0.0 0.0"' in text
    assert 'AGENT_DELTA_POSITION_DEFAULT="0.750 0.000 0.000"' in text
    assert 'AGENT_EEF_BACKEND_DEFAULT="${ARMCTRL_AGENT_EEF_BACKEND:-moveit_servo}"' in text
    assert "start_args+=(--eef-adapter moveit_servo)" in text
    assert "fake rehearsal registers the MoveIt Servo-style adapter" in text
    assert "--max-linear-step-m" in text
    assert "stretches it into a 50 Hz" in text
    assert "trajectory_q_point_args" not in text
    run_trajectory_section = text.split("  run-trajectory)", maxsplit=1)[1].split(
        "  run-eef)", maxsplit=1
    )[0]
    assert "--compiled-command" in run_trajectory_section
    assert "--q-point" not in run_trajectory_section


def test_lab_agent_runtime_smoke_script_is_bash_parseable() -> None:
    bash = shutil.which("bash")
    if bash is None:
        return

    root_for_bash = _path_for_bash(ROOT)
    completed = subprocess.run(
        [
            bash,
            "-lc",
            f"cd {root_for_bash} && bash -n scripts/lab_agent_runtime_smoke.sh",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr


def test_lab_fourier_sysid_script_uses_formal_status_and_result_surface() -> None:
    assert FOURIER_SCRIPT.exists()

    text = FOURIER_SCRIPT.read_text(encoding="utf-8")
    assert "armctrl console status" in text
    assert "armctrl motion result" in text
    assert "armctrl motion submit joint-trajectory" in text
    assert "--compiled-command" in text
    assert "armctrl sysid compile-runtime" in text
    assert "compile-* uses the SysID compiler surface" in text
    assert "armctrl sysid run --adapter sdk is intentionally not used" in text
    assert "armctrl sysid run fourier_multisine" not in text
    assert "console_status_${label}.json" in text


def _path_for_bash(path: Path) -> str:
    path_text = str(path)
    if len(path_text) >= 2 and path_text[1] == ":":
        drive = path_text[0].lower()
        rest = path_text[2:].replace("\\", "/").lstrip("/")
        return f"/mnt/{drive}/{rest}"
    return path_text.replace("\\", "/")
