from __future__ import annotations


def release_status() -> dict[str, object]:
    return {
        "schema": "armctrl.release_status.v1",
        "version": "0.6.0-rc.3",
        "branch": "codex/armctrl-clean-rebuild",
        "release_readiness": "simulation_safety_preview_nonhardware_verified",
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
            {
                "id": "v0.6.0-rc.3",
                "name": "simulation-safety-preview-gate",
                "status": "rc_nonhardware_verified_remote_backend_pending",
            },
        ],
        "hardware_pending": [
            "real_sdk_runner_hardware_validation",
            "native_lerobot_record_on_target_linux",
            "native_lerobot_rollout_on_target_linux",
            "sdk_preflight_on_target_linux",
            "sdk_handshake_plan_on_target_linux",
            "sdk_gravity_smoke_on_target_linux",
            "hold_damping_ctrl_c_hardware_landing",
            "n100d_pinocchio_figaroh_validation",
            "n100d_mujoco_moveit_pinocchio_coal_doctor",
            "hardware_ab_control_benefit_test",
        ],
        "verification": {
            "local_commands": [
                "uv sync --extra dev --extra sim",
                "uv run pytest -q",
                "uv run python -m compileall src tests",
                "uv run armctrl release status --json",
                "uv run armctrl sim doctor --json",
                "uv run armctrl sysid plan gravity_sweep --dof 6 --sample-hz 100 --duration 2 --amplitude 0.05 --q-center 0 0.30 0.30 0 0 0 --urdf-path configs/models/X5_camera.urdf --safe-config configs/x5.safe.yaml --output runs/plan-preview-smoke --json",
                "uv sync --extra dev --extra lerobot",
                "uv run armctrl lerobot doctor --model X5 --robot-interface can0 --teleop-interface can1 --json",
                "uv run armctrl sysid run gravity_sweep --adapter sdk --duration 8 --amplitude 0.5 --q-center 0 0.30 0.30 0 0 0 --output runs/tmp-confirm-check --confirm 'I UNDERSTAND THIS WILL MOVE THE ARM' --json",
            ],
            "test_count": 75,
            "scope": "local contracts and non-hardware simulation safety gates through v0.6.0-rc.3",
        },
        "deferred_validation": {
            "requires_hardware": [
                "real_sdk_runner_hardware_validation",
                "native_lerobot_record_on_target_linux",
                "native_lerobot_rollout_on_target_linux",
                "sdk_preflight_on_target_linux",
                "sdk_handshake_plan_on_target_linux",
                "sdk_gravity_smoke_on_target_linux",
                "hold_damping_ctrl_c_hardware_landing",
                "hardware_ab_control_benefit_test",
            ],
            "requires_external_tool": [
                "n100d_pinocchio_figaroh_validation",
                "n100d_mujoco_moveit_pinocchio_coal_doctor",
            ],
            "requires_geometry_upgrade": [],
        },
        "notes": [
            "Recipe plans are dry-run previews with machine-readable risk explanations.",
            "Recipe execution remains rejected until a verified hardware backend exists.",
            "LeRobot record/train/rollout remain native LeRobot commands; armctrl emits plans and metadata only.",
            "Install LeRobot integrations with uv sync --extra dev --extra lerobot on Linux targets.",
            "Parameter packages require package-gated solver evidence before rollout.",
            "Install MuJoCo preview support with uv sync --extra dev --extra sim on Linux targets.",
            "MoveIt 2 on ROS 2 hosts must be sourced before Python module detection, for example source /opt/ros/jazzy/setup.bash.",
            "X5_camera.urdf now has project-local STL assets so Pinocchio/coal can load native geometry without SDK wheel paths.",
            "Known adjacent X5 assembly mesh overlaps are configured as named allowed collision pairs; every other Pinocchio/coal collision remains a hard gate.",
            "MuJoCo preview now performs a qpos trajectory rollout with mj_forward and reports contact counts instead of only loading the URDF.",
            "MuJoCo rollout reuses the named allowed collision pairs; unlisted contacts still fail the safety gate.",
            "sim preview and sysid plan can render SVG trajectory previews, including unsafe warning annotations.",
            "The SDK sysid runner is wired for low-amplitude smoke collection, but hardware validation is still explicit deferred validation.",
            "SysID plans reject trajectories whose adjacent joint samples exceed max_joint_step_rad before any SDK backend is instantiated.",
            "SysID plans now write trajectory_preview.json and gate motion through the simulation safety preview before SDK execution.",
        ],
    }


def release_notes() -> dict[str, object]:
    status = release_status()
    included = [
        "governance and safety boundary",
        "offline SysID loop contracts",
        "conservative online identification policy",
        "Agent recipe CLI skill",
        "LeRobot doctor, native command plans, and dataset metadata bridge",
        "SDK gravity smoke runner with confirmation, safety gates, and damping landing path",
        "Safety-space config plus simulation doctor and SysID trajectory preview gate",
    ]
    deferred = list(status["hardware_pending"])
    title = "armctrl 0.6.0-rc.3 simulation safety preview"
    summary = (
        "simulation_safety_preview_nonhardware_verified: safety-space config, "
        "simulation backend doctor, and SysID trajectory preview gates are present; "
        "real robot movement and target-host mature backend validation remain deferred."
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
