import json
import subprocess
import sys
from pathlib import Path


def test_cli_sysid_adapt_figaroh_evidence_writes_armctrl_evidence(
    tmp_path: Path,
) -> None:
    figaroh_report = tmp_path / "figaroh-report.json"
    evidence_path = tmp_path / "armctrl-evidence.json"
    figaroh_report.write_text(
        json.dumps(
            {
                "schema": "figaroh.identification.report.v1",
                "base_parameters": {
                    "status": "available",
                    "parameter_count": 36,
                    "names": ["p0", "p1"],
                },
                "physical_consistency": {
                    "status": "pass",
                    "mass_positive": True,
                    "inertia_positive_definite": True,
                },
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "adapt-figaroh-evidence",
            "--input",
            str(figaroh_report),
            "--output",
            str(evidence_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.figarohevidence_adapter.v1"
    assert payload["artifacts"]["external_evidence"] == str(evidence_path)
    assert evidence["schema"] == "armctrl.external_solver_evidence.v1"
    assert evidence["source"] == "figaroh"
    assert evidence["physical_consistency"]["status"] == "pass"
    assert evidence["figaroh_base_parameters"]["status"] == "available"
    assert evidence["figaroh_base_parameters"]["parameter_count"] == 36


def test_adapted_figaroh_evidence_can_be_imported_into_solver_metrics(
    tmp_path: Path,
) -> None:
    dataset_dir = tmp_path / "ident-run"
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
            "--output",
            str(dataset_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "postprocess",
            "--dataset",
            str(dataset_dir),
            "--solve",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    figaroh_report = tmp_path / "figaroh-report.json"
    evidence_path = tmp_path / "armctrl-evidence.json"
    figaroh_report.write_text(
        json.dumps(
            {
                "schema": "figaroh.identification.report.v1",
                "base_parameters": {
                    "status": "available",
                    "parameter_count": 36,
                },
                "physical_consistency": {"status": "pass"},
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "adapt-figaroh-evidence",
            "--input",
            str(figaroh_report),
            "--output",
            str(evidence_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "import-evidence",
            "--dataset",
            str(dataset_dir),
            "--evidence",
            str(evidence_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    metrics = json.loads(
        (dataset_dir / "processed" / "solver_metrics.json").read_text(
            encoding="utf-8"
        )
    )

    assert metrics["physical_consistency"]["status"] == "pass"
    assert metrics["figaroh_base_parameters"]["parameter_count"] == 36


def test_cli_sysid_figaroh_handoff_writes_external_tool_package(
    tmp_path: Path,
) -> None:
    dataset_dir = tmp_path / "ident-run"
    handoff_dir = tmp_path / "figaroh-handoff"
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
            "--output",
            str(dataset_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "postprocess",
            "--dataset",
            str(dataset_dir),
            "--solve",
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
            "sysid",
            "figaroh-handoff",
            "--dataset",
            str(dataset_dir),
            "--output",
            str(handoff_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    handoff = json.loads((handoff_dir / "figaroh_handoff.json").read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.figaroh_handoff.v1"
    assert payload["artifacts"]["figaroh_handoff"] == str(handoff_dir / "figaroh_handoff.json")
    assert handoff["dataset"]["processed_samples"].endswith("processed_samples.csv")
    assert handoff["dataset"]["solver_metrics"].endswith("solver_metrics.json")
    assert handoff["model"]["urdf_path"] == "configs/models/X5_camera.urdf"
    assert handoff["expected_outputs"]["report_schema"] == "figaroh.identification.report.v1"
    assert handoff["armctrl_import_command"][0:3] == [
        "armctrl",
        "sysid",
        "adapt-figaroh-evidence",
    ]
