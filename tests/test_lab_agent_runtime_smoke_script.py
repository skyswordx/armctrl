from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "lab_agent_runtime_smoke.sh"


def test_lab_agent_runtime_smoke_script_documents_runtime_agent_flow() -> None:
    assert SCRIPT.exists()

    text = SCRIPT.read_text(encoding="utf-8")
    assert "scripts/lab_agent_runtime_smoke.sh start" in text
    assert "scripts/lab_agent_runtime_smoke.sh plan" in text
    assert "scripts/lab_agent_runtime_smoke.sh run" in text
    assert "runtime-smoke-real" in text
    assert "--runtime-session-artifact" in text
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


def _path_for_bash(path: Path) -> str:
    path_text = str(path)
    if len(path_text) >= 2 and path_text[1] == ":":
        drive = path_text[0].lower()
        rest = path_text[2:].replace("\\", "/").lstrip("/")
        return f"/mnt/{drive}/{rest}"
    return path_text.replace("\\", "/")
