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
    assert "armctrl motion submit eef-delta" in text
    assert "armctrl motion result" in text
    assert "armctrl console status" in text
    assert "current measured EEF pose" in text
    assert "motion kind=joint-intent" in text
    assert "owner=agent" in text
    assert "Agent intent is 10 Hz" in text
    assert "runtime sends at 50 Hz" in text


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
    assert "armctrl sysid run fourier_multisine" in text
    assert "owns Fourier candidate compilation and SysID evidence" in text
    assert "console_status_${label}.json" in text


def _path_for_bash(path: Path) -> str:
    path_text = str(path)
    if len(path_text) >= 2 and path_text[1] == ":":
        drive = path_text[0].lower()
        rest = path_text[2:].replace("\\", "/").lstrip("/")
        return f"/mnt/{drive}/{rest}"
    return path_text.replace("\\", "/")
