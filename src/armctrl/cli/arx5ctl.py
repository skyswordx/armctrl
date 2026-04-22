from __future__ import annotations

import argparse
import json
import sys
import threading
from collections.abc import Sequence

from armctrl.adapters.arx5.fake import FakeArx5Adapter
from armctrl.adapters.arx5.sdk import Arx5SDKAdapter
from armctrl.daemon.executor import ArmCommandExecutor
from armctrl.protocol.enums import AdapterKind, CommandStatus, DebugProfileName, ErrorCode
from armctrl.protocol.errors import ArmctrlError
from armctrl.protocol.models import DebugProfileRequest, MoveEEFRequest
from armctrl.safety.debug_profiles import DebugProfileRegistry
from armctrl.teleop.mapping import XboxMapper
from armctrl.teleop.xbox import XboxDebugRunner, create_tk_dashboard, load_events, show_response_dashboard

MOVE_CONFIRMATION = "I UNDERSTAND THIS WILL MOVE THE ARM"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="arx5ctl")
    subparsers = parser.add_subparsers(dest="command", required=True)

    for name in ("health", "state", "damping", "cancel"):
        add_common(subparsers.add_parser(name))

    move = subparsers.add_parser("move-eef")
    add_common(move)
    move.add_argument("--pose", nargs=6, type=float, required=True)
    move.add_argument("--gripper", type=float, default=0.0)
    move.add_argument("--preview", type=float, default=0.1)
    move.add_argument("--plan-only", action="store_true")
    move.add_argument("--execute", action="store_true")
    move.add_argument("--confirm", default="")

    debug = subparsers.add_parser("debug-profile")
    add_common(debug)
    debug.add_argument("name", choices=[profile.value for profile in DebugProfileName])
    debug.add_argument("--maintenance", action="store_true")
    debug.add_argument("--plan-only", action="store_true")
    debug.add_argument("--confirm", default="")

    list_profiles = subparsers.add_parser("list-debug-profiles")
    list_profiles.add_argument("--json", action="store_true")
    list_profiles.add_argument("--pretty", action="store_true")

    teleop = subparsers.add_parser("teleop-xbox")
    add_common(teleop)
    teleop.add_argument("--device")
    teleop.add_argument("--event-jsonl")
    teleop.add_argument("--max-events", type=int)
    teleop.add_argument("--rate-hz", type=float, default=100.0)
    teleop.add_argument("--ui-hz", type=float, default=50.0)
    teleop.add_argument("--execute", action="store_true")
    teleop.add_argument("--maintenance", action="store_true")
    teleop.add_argument("--plan-only-debug-profiles", action="store_true")
    teleop.add_argument("--confirm", default="")

    return parser


def add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--adapter", choices=[kind.value for kind in AdapterKind], default=AdapterKind.FAKE.value)
    parser.add_argument("--model", default="X5")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--gravity-compensation", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--pretty", action="store_true")


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == "list-debug-profiles":
        return list_debug_profiles(args)
    if getattr(args, "json", False) and getattr(args, "gui", False):
        response = exc_to_response(ArmctrlError(ErrorCode.INVALID_REQUEST, "--json and --gui cannot be used together"))
        emit_json(response.to_dict())
        return 2

    try:
        executor = build_executor(args)
        response = dispatch(args, executor)
    except ArmctrlError as exc:
        response = exc_to_response(exc)
    except Exception as exc:
        response = exc_to_response(ArmctrlError(ErrorCode.INVALID_REQUEST, str(exc)))

    if response is None:
        return 2
    if args.command == "teleop-xbox" and response.error is None and not getattr(args, "json", False):
        return 0 if response.status in (CommandStatus.COMPLETED, CommandStatus.CANCELLED) else 2
    if getattr(args, "gui", False):
        show_response_dashboard(response, source=command_source(args))
        return 0 if response.status in (CommandStatus.COMPLETED, CommandStatus.CANCELLED) else 2
    if getattr(args, "json", False):
        emit_json(response.to_dict(), pretty=getattr(args, "pretty", False))
    else:
        emit(response.message)
    return 0 if response.status in (CommandStatus.COMPLETED, CommandStatus.CANCELLED) else 2


def build_executor(args: argparse.Namespace) -> ArmCommandExecutor:
    if args.adapter == AdapterKind.SDK.value:
        adapter = Arx5SDKAdapter(
            model=args.model,
            interface=args.interface,
            gravity_compensation=True if args.gravity_compensation else None,
        )
    else:
        adapter = FakeArx5Adapter()
    if args.adapter == AdapterKind.SDK.value and args.command in {"move-eef", "teleop-xbox"}:
        if not args.execute or args.confirm != MOVE_CONFIRMATION:
            raise ArmctrlError(
                code=ErrorCode.CONFIRMATION_REQUIRED,
                message=f"real SDK motion requires --execute --confirm '{MOVE_CONFIRMATION}'",
            )
    connect_response = adapter.connect()
    if connect_response.status is not CommandStatus.COMPLETED:
        raise connect_response.error or ArmctrlError(code=ErrorCode.SDK_ERROR, message=connect_response.message)
    return ArmCommandExecutor(
        adapter,
        maintenance=getattr(args, "maintenance", False),
        confirm_debug_profiles=getattr(args, "confirm", "") == MOVE_CONFIRMATION,
        plan_only_debug_profiles=getattr(args, "plan_only_debug_profiles", False),
    )


def dispatch(args: argparse.Namespace, executor: ArmCommandExecutor):
    if args.command == "health":
        return executor.health()
    if args.command == "state":
        return executor.state()
    if args.command == "damping":
        return executor.damping()
    if args.command == "cancel":
        return executor.cancel()
    if args.command == "move-eef":
        plan_only = args.plan_only or not args.execute
        return executor.move_eef(
            MoveEEFRequest(
                pose_6d=tuple(args.pose),
                gripper_pos=args.gripper,
                preview_time_s=args.preview,
                plan_only=plan_only,
            )
        )
    if args.command == "debug-profile":
        return executor.apply_debug_profile(
            DebugProfileRequest(
                name=DebugProfileName(args.name),
                maintenance=args.maintenance,
                confirm=args.confirm == MOVE_CONFIRMATION,
                plan_only=args.plan_only,
            )
        )
    if args.command == "teleop-xbox":
        events = load_events(args.device, args.event_jsonl)
        source = args.event_jsonl or args.device or "/dev/input/event0"
        if args.json:
            runner = XboxDebugRunner(
                executor=executor,
                mapper=XboxMapper(),
                events=events,
                rate_hz=args.rate_hz,
                max_events=args.max_events,
                source=source,
            )
            return runner.run()
        dashboard = create_tk_dashboard(source=source, rate_hz=args.ui_hz)
        runner = XboxDebugRunner(
            executor=executor,
            mapper=XboxMapper(),
            events=events,
            rate_hz=args.rate_hz,
            max_events=args.max_events,
            source=source,
            dashboard=dashboard,
            stop_event=dashboard.stop_event,
        )
        result: dict[str, object] = {}

        def worker() -> None:
            try:
                result["response"] = runner.run()
            except BaseException as exc:
                result["error"] = exc

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        dashboard.run()
        thread.join()
        if "error" in result:
            raise result["error"]
        if "response" not in result:
            raise ArmctrlError(
                ErrorCode.INVALID_REQUEST,
                "teleop dashboard exited before runner produced a response",
            )
        return result["response"]
    raise ArmctrlError(code=ErrorCode.INVALID_REQUEST, message=f"unsupported command {args.command}")


def list_debug_profiles(args: argparse.Namespace) -> int:
    profiles = [
        {
            "name": profile.name.value,
            "label": profile.label,
            "requires_maintenance": profile.requires_maintenance,
            "requires_restart": profile.requires_restart,
        }
        for profile in DebugProfileRegistry.default().list()
    ]
    if args.json:
        emit_json({"profiles": profiles}, pretty=args.pretty)
    else:
        for profile in profiles:
            emit(profile["name"])
    return 0


def emit(message: str) -> None:
    sys.stdout.write(message + "\n")


def emit_json(payload: dict, *, pretty: bool = False) -> None:
    if pretty:
        emit(json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2))
    else:
        emit(json.dumps(payload, ensure_ascii=False, sort_keys=True))


def command_source(args: argparse.Namespace) -> str:
    if args.command == "teleop-xbox":
        return args.event_jsonl or args.device or "/dev/input/event0"
    adapter = getattr(args, "adapter", "unknown")
    return f"{args.command}:{adapter}"


def exc_to_response(error: ArmctrlError):
    from armctrl.protocol.enums import CommandStatus
    from armctrl.protocol.models import CommandResponse

    return CommandResponse(CommandStatus.REJECTED, error.message, error=error)


if __name__ == "__main__":
    raise SystemExit(main())
