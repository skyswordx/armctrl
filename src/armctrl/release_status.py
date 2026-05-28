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
            "mesh_body_collision_model",
            "real_sdk_runner",
            "sdk_preflight_on_target_linux",
            "sdk_handshake_plan_on_target_linux",
            "hold_damping_ctrl_c_hardware_landing",
            "n100d_pinocchio_figaroh_validation",
            "hardware_ab_control_benefit_test",
        ],
        "verification": {
            "local_commands": [
                "uv run pytest -q",
                "uv run python -m compileall src tests",
                "uv run armctrl release status --json",
            ],
            "test_count": 50,
            "scope": "local contracts and non-hardware safety gates through v0.5.0",
        },
        "deferred_validation": {
            "requires_hardware": [
                "real_sdk_runner",
                "sdk_preflight_on_target_linux",
                "sdk_handshake_plan_on_target_linux",
                "hold_damping_ctrl_c_hardware_landing",
                "hardware_ab_control_benefit_test",
            ],
            "requires_external_tool": [
                "n100d_pinocchio_figaroh_validation",
            ],
            "requires_geometry_upgrade": [
                "mesh_body_collision_model",
            ],
        },
        "notes": [
            "Recipe plans are dry-run previews with machine-readable risk explanations.",
            "Recipe execution remains rejected until a verified hardware backend exists.",
            "Parameter packages require package-gated solver evidence before rollout.",
        ],
    }


def release_notes() -> dict[str, object]:
    status = release_status()
    included = [
        "governance and safety boundary",
        "offline SysID loop contracts",
        "conservative online identification policy",
        "Agent recipe CLI skill",
    ]
    deferred = list(status["hardware_pending"])
    title = "armctrl 0.5.0 clean rebuild"
    summary = (
        "contracts_complete_hardware_pending: local contracts through v0.5.0 "
        "are implemented, while real hardware and n100d external-tool validation "
        "remain deferred."
    )
    markdown = (
        f"# {title}\n\n"
        f"{summary}\n\n"
        "## Included\n"
        + "\n".join(f"- {item}" for item in included)
        + "\n\n## Deferred Validation\n"
        + "\n".join(f"- {item}" for item in deferred)
        + "\n"
    )
    return {
        "schema": "armctrl.release_notes.v1",
        "version": status["version"],
        "title": title,
        "summary": summary,
        "sections": {
            "included": included,
            "deferred": deferred,
        },
        "markdown": markdown,
    }
