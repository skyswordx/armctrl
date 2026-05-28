from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence

from armctrl.recipes import RecipeCatalog
from armctrl.recipe_executor import RecipeExecutor
from armctrl.release_status import release_status
from armctrl.online_id import (
    OnlineAuditRequest,
    OnlineIdentificationAuditor,
    OnlineIdentificationPolicy,
    ParameterUpdate,
)
from armctrl.safety import SafetyGate
from armctrl.sysid import SysIdPlanner, SysIdPlanRequest
from armctrl.sysid_evidence import SysIdEvidenceImporter
from armctrl.sysid_figaroh_adapter import FigarohEvidenceAdapter
from armctrl.sysid_package import SysIdPackager
from armctrl.sysid_postprocess import SysIdPostprocessor, SysIdPostprocessResult
from armctrl.sysid_run import FakeSysIdRunner, SdkSysIdRunnerGate
from armctrl.sysid_sdk import SdkHandshakePlanner, SdkPreflight
from armctrl.sysid_solve import SysIdSolver


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="armctrl")
    subparsers = parser.add_subparsers(dest="command", required=True)

    recipe_parser = subparsers.add_parser("recipe")
    recipe_subparsers = recipe_parser.add_subparsers(dest="recipe_command", required=True)

    list_parser = recipe_subparsers.add_parser("list")
    list_parser.add_argument("--json", action="store_true", dest="as_json")

    plan_parser = recipe_subparsers.add_parser("plan")
    plan_parser.add_argument("name")
    plan_parser.add_argument("--json", action="store_true", dest="as_json")

    execute_parser = recipe_subparsers.add_parser("execute")
    execute_parser.add_argument("name")
    execute_parser.add_argument("--json", action="store_true", dest="as_json")

    status_parser = recipe_subparsers.add_parser("status")
    status_parser.add_argument("--json", action="store_true", dest="as_json")

    cancel_parser = recipe_subparsers.add_parser("cancel")
    cancel_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_parser = subparsers.add_parser("sysid")
    sysid_subparsers = sysid_parser.add_subparsers(dest="sysid_command", required=True)

    sysid_plan_parser = sysid_subparsers.add_parser("plan")
    sysid_plan_parser.add_argument("profile")
    sysid_plan_parser.add_argument("--execute", action="store_true")
    sysid_plan_parser.add_argument("--dof", type=int, default=6)
    sysid_plan_parser.add_argument("--sample-hz", type=float, default=100.0)
    sysid_plan_parser.add_argument("--duration", type=float, default=10.0)
    sysid_plan_parser.add_argument("--amplitude", type=float, default=0.1)
    sysid_plan_parser.add_argument("--q-center", nargs="+", type=float)
    sysid_plan_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    sysid_plan_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    sysid_plan_parser.add_argument("--output")
    sysid_plan_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_run_parser = sysid_subparsers.add_parser("run")
    sysid_run_parser.add_argument("profile")
    sysid_run_parser.add_argument("--adapter", default="fake")
    sysid_run_parser.add_argument("--dof", type=int, default=6)
    sysid_run_parser.add_argument("--sample-hz", type=float, default=100.0)
    sysid_run_parser.add_argument("--duration", type=float, default=10.0)
    sysid_run_parser.add_argument("--amplitude", type=float, default=0.1)
    sysid_run_parser.add_argument("--q-center", nargs="+", type=float)
    sysid_run_parser.add_argument("--urdf-path", default="configs/models/X5_camera.urdf")
    sysid_run_parser.add_argument("--safe-config", default="configs/x5.safe.yaml")
    sysid_run_parser.add_argument("--output", required=True)
    sysid_run_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_postprocess_parser = sysid_subparsers.add_parser("postprocess")
    sysid_postprocess_parser.add_argument("--dataset", required=True)
    sysid_postprocess_parser.add_argument("--solve", action="store_true")
    sysid_postprocess_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_solve_parser = sysid_subparsers.add_parser("solve")
    sysid_solve_parser.add_argument("--dataset", required=True)
    sysid_solve_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_package_parser = sysid_subparsers.add_parser("package")
    sysid_package_parser.add_argument("--dataset", required=True)
    sysid_package_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_evidence_parser = sysid_subparsers.add_parser("import-evidence")
    sysid_evidence_parser.add_argument("--dataset", required=True)
    sysid_evidence_parser.add_argument("--evidence", required=True)
    sysid_evidence_parser.add_argument("--json", action="store_true", dest="as_json")

    sysid_figaroh_adapter_parser = sysid_subparsers.add_parser(
        "adapt-figaroh-evidence"
    )
    sysid_figaroh_adapter_parser.add_argument("--input", required=True)
    sysid_figaroh_adapter_parser.add_argument("--output", required=True)
    sysid_figaroh_adapter_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_sdk_preflight_parser = sysid_subparsers.add_parser("sdk-preflight")
    sysid_sdk_preflight_parser.add_argument("--model", default="X5")
    sysid_sdk_preflight_parser.add_argument("--interface", required=True)
    sysid_sdk_preflight_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    sysid_sdk_handshake_parser = sysid_subparsers.add_parser("sdk-handshake-plan")
    sysid_sdk_handshake_parser.add_argument("--model", default="X5")
    sysid_sdk_handshake_parser.add_argument("--interface", required=True)
    sysid_sdk_handshake_parser.add_argument(
        "--json",
        action="store_true",
        dest="as_json",
    )

    online_parser = subparsers.add_parser("online-id")
    online_subparsers = online_parser.add_subparsers(
        dest="online_command",
        required=True,
    )
    online_policy_parser = online_subparsers.add_parser("policy")
    online_policy_parser.add_argument("--json", action="store_true", dest="as_json")

    online_audit_parser = online_subparsers.add_parser("audit")
    online_audit_parser.add_argument("--parameter", required=True)
    online_audit_parser.add_argument("--value", type=float, required=True)
    online_audit_parser.add_argument("--source", required=True)
    online_audit_parser.add_argument("--window-start", type=float, required=True)
    online_audit_parser.add_argument("--window-end", type=float, required=True)
    online_audit_parser.add_argument("--residual-before", type=float, required=True)
    online_audit_parser.add_argument("--residual-after", type=float, required=True)
    online_audit_parser.add_argument("--saturation-status", required=True)
    online_audit_parser.add_argument("--rollback-target", required=True)
    online_audit_parser.add_argument("--output", required=True)
    online_audit_parser.add_argument("--json", action="store_true", dest="as_json")

    release_parser = subparsers.add_parser("release")
    release_subparsers = release_parser.add_subparsers(
        dest="release_command",
        required=True,
    )
    release_status_parser = release_subparsers.add_parser("status")
    release_status_parser.add_argument("--json", action="store_true", dest="as_json")

    args = parser.parse_args(argv)
    catalog = RecipeCatalog.default()

    if args.command == "recipe" and args.recipe_command == "list":
        payload = {
            "status": "ok",
            "schema": "armctrl.recipe_catalog.v1",
            "recipes": [recipe.to_json() for recipe in catalog.list_recipes()],
        }
        return _emit(payload, as_json=args.as_json)

    if args.command == "recipe" and args.recipe_command == "plan":
        recipe = catalog.get(args.name)
        safety = SafetyGate().evaluate(recipe, plan_only=True)
        payload = {
            "status": "ok",
            "schema": "armctrl.recipe_plan.v1",
            "plan_only": True,
            "recipe": recipe.to_json(),
            "safety": safety.to_json(),
            "steps": [step.to_json() for step in recipe.steps],
        }
        return _emit(payload, as_json=args.as_json)

    if args.command == "recipe" and args.recipe_command == "execute":
        recipe = catalog.get(args.name)
        safety = SafetyGate().evaluate(recipe, plan_only=False)
        executor = RecipeExecutor().evaluate(safety)
        payload = {
            "status": "rejected",
            "schema": "armctrl.recipe_execution.v1",
            "recipe": recipe.to_json(),
            "safety": safety.to_json(),
            "executor": executor.to_json(),
            "steps": [step.to_json() for step in recipe.steps],
        }
        _emit(payload, as_json=args.as_json)
        return 3

    if args.command == "recipe" and args.recipe_command == "status":
        executor = RecipeExecutor().status()
        payload = {
            "status": "ok",
            "schema": "armctrl.recipe_executor_status.v1",
            "executor": executor.to_json(),
        }
        return _emit(payload, as_json=args.as_json)

    if args.command == "recipe" and args.recipe_command == "cancel":
        executor = RecipeExecutor().cancel()
        payload = {
            "status": "ok",
            "schema": "armctrl.recipe_executor_cancel.v1",
            "executor": executor.to_json(),
        }
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "plan":
        planner = SysIdPlanner.default()
        if args.execute:
            plan = planner.plan(args.profile, execute=True)
            payload = {"status": "rejected", **plan.to_json()}
            _emit(payload, as_json=args.as_json)
            return 3
        if args.output:
            q_center = tuple(args.q_center or [0.0] * args.dof)
            if len(q_center) != args.dof:
                parser.error("--q-center length must match --dof")
            plan = planner.write_plan(
                SysIdPlanRequest(
                    profile_name=args.profile,
                    dof=args.dof,
                    sample_hz=args.sample_hz,
                    duration_s=args.duration,
                    amplitude_rad=args.amplitude,
                    q_center=q_center,
                    urdf_path=args.urdf_path,
                    safe_config_path=args.safe_config,
                    output_dir=Path(args.output),
                )
            )
        else:
            plan = planner.plan(args.profile, execute=False)
        payload = {"status": "ok", **plan.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "run":
        if args.adapter != "fake":
            payload = SdkSysIdRunnerGate().reject_without_confirmation(
                adapter=args.adapter,
            )
            _emit(payload, as_json=args.as_json)
            return 3
        q_center = tuple(args.q_center or [0.0] * args.dof)
        if len(q_center) != args.dof:
            parser.error("--q-center length must match --dof")
        result = FakeSysIdRunner().run(
            SysIdPlanRequest(
                profile_name=args.profile,
                dof=args.dof,
                sample_hz=args.sample_hz,
                duration_s=args.duration,
                amplitude_rad=args.amplitude,
                q_center=q_center,
                urdf_path=args.urdf_path,
                safe_config_path=args.safe_config,
                output_dir=Path(args.output),
            )
        )
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "postprocess":
        result = SysIdPostprocessor().run(Path(args.dataset))
        if args.solve:
            solver_result = SysIdSolver().run(Path(args.dataset))
            result = SysIdPostprocessResult(
                schema=result.schema,
                sample_count=result.sample_count,
                artifacts=result.artifacts,
                solver=solver_result.to_json(),
            )
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "solve":
        result = SysIdSolver().run(Path(args.dataset))
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "package":
        result = SysIdPackager().run(Path(args.dataset))
        payload = {"status": result.status, **result.to_json()}
        _emit(payload, as_json=args.as_json)
        return 0 if result.status == "ok" else 3

    if args.command == "sysid" and args.sysid_command == "import-evidence":
        result = SysIdEvidenceImporter().run(Path(args.dataset), Path(args.evidence))
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "adapt-figaroh-evidence":
        result = FigarohEvidenceAdapter().run(Path(args.input), Path(args.output))
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "sdk-preflight":
        result = SdkPreflight().run(model=args.model, interface=args.interface)
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "sysid" and args.sysid_command == "sdk-handshake-plan":
        result = SdkHandshakePlanner().plan(model=args.model, interface=args.interface)
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "online-id" and args.online_command == "policy":
        payload = {"status": "ok", **OnlineIdentificationPolicy.default().to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "online-id" and args.online_command == "audit":
        result = OnlineIdentificationAuditor.default().record(
            OnlineAuditRequest(
                update=ParameterUpdate(
                    name=args.parameter,
                    value=args.value,
                    source=args.source,
                ),
                window_start_s=args.window_start,
                window_end_s=args.window_end,
                residual_before=args.residual_before,
                residual_after=args.residual_after,
                saturation_status=args.saturation_status,
                rollback_target=args.rollback_target,
                output_path=Path(args.output),
            )
        )
        payload = {"status": "ok", **result.to_json()}
        return _emit(payload, as_json=args.as_json)

    if args.command == "release" and args.release_command == "status":
        payload = {"status": "ok", **release_status()}
        return _emit(payload, as_json=args.as_json)

    parser.error("unsupported command")
    return 2


def _emit(payload: dict[str, object], *, as_json: bool) -> int:
    if as_json:
        print(json.dumps(payload, ensure_ascii=False))
    else:
        print(payload["status"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
