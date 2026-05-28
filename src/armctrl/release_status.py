from __future__ import annotations


def release_status() -> dict[str, object]:
    return {
        "schema": "armctrl.release_status.v1",
        "version": "0.5.0",
        "branch": "codex/armctrl-clean-rebuild",
        "release_readiness": "contracts_complete_hardware_pending",
        "milestones": [
            {
                "id": "v0.2.0",
                "name": "governance-and-safety-boundary",
                "status": "contract_complete",
            },
            {
                "id": "v0.3.0",
                "name": "offline-sysid-loop",
                "status": "contract_complete_hardware_pending",
            },
            {
                "id": "v0.4.0",
                "name": "conservative-online-identification",
                "status": "contract_complete",
            },
            {
                "id": "v0.5.0",
                "name": "agent-recipe-and-cli-skill",
                "status": "contract_complete",
            },
        ],
        "hardware_pending": [
            "full_fk_table_collision_model",
            "real_sdk_runner",
            "sdk_preflight_on_target_linux",
            "sdk_handshake_plan_on_target_linux",
            "hold_damping_ctrl_c_hardware_landing",
            "n100d_pinocchio_figaroh_validation",
            "hardware_ab_control_benefit_test",
        ],
        "notes": [
            "Recipe execution remains rejected until a verified hardware backend exists.",
            "Parameter packages require package-gated solver evidence before rollout.",
        ],
    }
