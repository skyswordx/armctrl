import csv
import json
import subprocess
import sys
from pathlib import Path


def _create_fake_dataset(output_dir: Path) -> None:
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "fake",
            "--dof",
            "6",
            "--sample-hz",
            "20",
            "--duration",
            "2",
            "--amplitude",
            "0.1",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def test_cli_sysid_postprocess_writes_processed_samples_and_quality(tmp_path: Path) -> None:
    dataset_dir = tmp_path / "ident-run"
    _create_fake_dataset(dataset_dir)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "postprocess",
            "--dataset",
            str(dataset_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    processed_csv = dataset_dir / "processed" / "processed_samples.csv"
    quality_json = dataset_dir / "processed" / "quality_metrics.json"
    quality_report = dataset_dir / "processed" / "quality_report.md"

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_postprocess.v1"
    assert payload["artifacts"]["processed_samples"] == str(processed_csv)
    assert processed_csv.exists()
    assert quality_json.exists()
    assert quality_report.exists()

    metrics = json.loads(quality_json.read_text(encoding="utf-8"))
    assert metrics["schema"] == "armctrl.sysid_quality.v1"
    assert metrics["sample_count"] == 41
    assert metrics["data_health"]["status"] == "pass"
    assert metrics["handoff"]["solver_backends"] == ["pinocchio", "figaroh"]

    with processed_csv.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    assert len(rows) == 41
    assert "q_proc_6" in rows[0]
    assert "tau_proc_6" in rows[0]
