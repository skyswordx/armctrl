from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence

from armctrl.recipes import RecipeCatalog
from armctrl.online_id import OnlineIdentificationPolicy
from armctrl.safety import SafetyGate
from armctrl.sysid import SysIdPlanner, SysIdPlanRequest


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

    online_parser = subparsers.add_parser("online-id")
    online_subparsers = online_parser.add_subparsers(
        dest="online_command",
        required=True,
    )
    online_policy_parser = online_subparsers.add_parser("policy")
    online_policy_parser.add_argument("--json", action="store_true", dest="as_json")

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
        payload = {
            "status": "rejected",
            "schema": "armctrl.recipe_execution.v1",
            "recipe": recipe.to_json(),
            "safety": safety.to_json(),
            "steps": [step.to_json() for step in recipe.steps],
        }
        _emit(payload, as_json=args.as_json)
        return 3

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

    if args.command == "online-id" and args.online_command == "policy":
        payload = {"status": "ok", **OnlineIdentificationPolicy.default().to_json()}
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
