import json
import subprocess
import sys
from pathlib import Path

from armctrl.online_id import OnlineIdentificationPolicy, ParameterUpdate


def test_online_policy_only_allows_conservative_shadow_updates() -> None:
    policy = OnlineIdentificationPolicy.default()

    accepted = policy.evaluate(
        ParameterUpdate(
            name="joint_2.viscous_friction",
            value=0.12,
            source="rolling_residual_window",
        )
    )
    rejected = policy.evaluate(
        ParameterUpdate(
            name="link_2.mass",
            value=3.2,
            source="rolling_residual_window",
        )
    )

    assert accepted.allowed is True
    assert accepted.mode == "shadow"
    assert rejected.allowed is False
    assert rejected.reason == "full rigid-body parameters require offline SysID"


def test_online_policy_exports_audit_contract() -> None:
    policy = OnlineIdentificationPolicy.default()

    payload = policy.to_json()

    assert payload["schema"] == "armctrl.online_identification_policy.v1"
    assert payload["mode"] == "shadow"


def test_cli_online_audit_writes_shadow_update_record(tmp_path: Path) -> None:
    audit_log = tmp_path / "online-id-audit.jsonl"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "online-id",
            "audit",
            "--parameter",
            "joint_2.viscous_friction",
            "--value",
            "0.12",
            "--source",
            "rolling_residual_window",
            "--window-start",
            "12.0",
            "--window-end",
            "18.0",
            "--residual-before",
            "0.8",
            "--residual-after",
            "0.5",
            "--saturation-status",
            "pass",
            "--rollback-target",
            "previous_parameter_bundle",
            "--output",
            str(audit_log),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    records = [
        json.loads(line)
        for line in audit_log.read_text(encoding="utf-8").splitlines()
        if line
    ]

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.online_identification_audit.v1"
    assert payload["decision"]["allowed"] is True
    assert payload["artifacts"]["audit_log"] == str(audit_log)
    assert records[0]["mode"] == "shadow"
    assert records[0]["manual_confirmation_required"] is True
    assert records[0]["rollback_target"] == "previous_parameter_bundle"
    assert records[0]["residual_delta"] == -0.3
    assert payload["allowed_parameter_groups"] == [
        "torque_bias",
        "viscous_friction",
        "coulomb_friction",
        "gravity_residual",
    ]
    assert payload["requires_manual_promotion"] is True


def test_cli_online_policy_outputs_json() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "online-id", "policy", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.online_identification_policy.v1"
    assert payload["mode"] == "shadow"
