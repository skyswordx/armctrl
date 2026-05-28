import json
import subprocess
import sys
from pathlib import Path


def _create_solved_fake_dataset(output_dir: Path) -> None:
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
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "postprocess",
            "--dataset",
            str(output_dir),
            "--solve",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def test_cli_sysid_import_evidence_merges_external_figaroh_and_physical_gates(
    tmp_path: Path,
) -> None:
    dataset_dir = tmp_path / "ident-run"
    _create_solved_fake_dataset(dataset_dir)
    evidence_path = tmp_path / "external-evidence.json"
    evidence_path.write_text(
        json.dumps(
            {
                "schema": "armctrl.external_solver_evidence.v1",
                "source": "figaroh",
                "physical_consistency": {
                    "status": "pass",
                    "mass_positive": True,
                    "inertia_positive_definite": True,
                },
                "figaroh_base_parameters": {
                    "status": "available",
                    "parameter_count": 36,
                    "basis": "external",
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

    payload = json.loads(completed.stdout)
    metrics = json.loads(
        (dataset_dir / "processed" / "solver_metrics.json").read_text(
            encoding="utf-8"
        )
    )

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_external_evidence.v1"
    assert payload["evidence"]["source"] == "figaroh"
    assert metrics["physical_consistency"]["status"] == "pass"
    assert metrics["figaroh_base_parameters"]["status"] == "available"
    assert metrics["external_evidence"]["schema"] == "armctrl.external_solver_evidence.v1"


def test_imported_external_evidence_removes_figaroh_package_gate_gaps(
    tmp_path: Path,
) -> None:
    dataset_dir = tmp_path / "ident-run"
    _create_solved_fake_dataset(dataset_dir)
    evidence_path = tmp_path / "external-evidence.json"
    evidence_path.write_text(
        json.dumps(
            {
                "schema": "armctrl.external_solver_evidence.v1",
                "source": "figaroh",
                "physical_consistency": {"status": "pass"},
                "figaroh_base_parameters": {"status": "available"},
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

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "package",
            "--dataset",
            str(dataset_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert completed.returncode == 3
    assert "physical_consistency" not in payload["quality_gate"]["missing"]
    assert "figaroh_base_parameters" not in payload["quality_gate"]["missing"]
    assert "pinocchio_regressor_condition" in payload["quality_gate"]["missing"]


def test_cli_sysid_import_evidence_accepts_utf8_bom_json(tmp_path: Path) -> None:
    dataset_dir = tmp_path / "ident-run"
    _create_solved_fake_dataset(dataset_dir)
    evidence_path = tmp_path / "external-evidence-bom.json"
    evidence_path.write_text(
        "\ufeff"
        + json.dumps(
            {
                "schema": "armctrl.external_solver_evidence.v1",
                "source": "figaroh",
                "physical_consistency": {"status": "pass"},
                "figaroh_base_parameters": {"status": "available"},
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

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["evidence"]["source"] == "figaroh"
