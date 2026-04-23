"""`arx5ctl` 命令行入口。

这个文件负责三件事：
1. 解析命令行参数；
2. 组装 adapter + executor；
3. 把结果按机器可读 JSON 或人工调试 GUI 输出。

它不直接实现控制算法，职责保持在“进程入口”和“用户接口层”。
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path

from armctrl.adapters.arx5.fake import FakeArx5Adapter
from armctrl.adapters.arx5.sdk import Arx5SDKAdapter
from armctrl.daemon.executor import ArmCommandExecutor
from armctrl.identification.backends import build_joint_backend
from armctrl.identification.optimization import optimize_fourier_multisine
from armctrl.identification.postprocess import postprocess_dataset
from armctrl.identification.recorder import DatasetRecorder
from armctrl.identification.runner import IdentificationRunner
from armctrl.identification.safety import TrajectorySafetyLimits, validate_trajectory
from armctrl.identification.trajectories import (
    generate_fourier_multisine,
    generate_friction_sweep,
    generate_gravity_sweep,
)
from armctrl.protocol.enums import AdapterKind, CommandStatus, DebugProfileName, ErrorCode
from armctrl.protocol.errors import ArmctrlError
from armctrl.protocol.models import CommandResponse, DebugProfileRequest, MoveEEFRequest
from armctrl.safety.debug_profiles import DebugProfileRegistry
from armctrl.teleop.mapping import XboxMapper
from armctrl.teleop.xbox import XboxDebugRunner, create_tk_dashboard, load_events, show_response_dashboard

MOVE_CONFIRMATION = "I UNDERSTAND THIS WILL MOVE THE ARM"


def build_parser() -> argparse.ArgumentParser:
    # `argparse` 结构和子命令一一对应，方便新同学从 CLI 直接反推业务能力边界。
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

    ident_plan = subparsers.add_parser("ident-plan")
    add_common(ident_plan)
    add_identification_profile_args(ident_plan)
    ident_plan.add_argument("--output")

    ident_run = subparsers.add_parser("ident-run")
    add_common(ident_run)
    add_identification_profile_args(ident_run)
    ident_run.add_argument("--output")
    ident_run.add_argument("--execute", action="store_true")
    ident_run.add_argument("--confirm", default="")
    ident_run.add_argument("--no-damping-after", action="store_true")

    ident_postprocess = subparsers.add_parser("ident-postprocess")
    ident_postprocess.add_argument("--dataset", required=True)
    ident_postprocess.add_argument("--output")
    ident_postprocess.add_argument("--tool", action="append", choices=["all", "pinocchio", "figaroh", "flobaroid", "urdfly"])
    ident_postprocess.add_argument("--urdf-path")
    ident_postprocess.add_argument("--smoothing-window", type=int, default=5)
    ident_postprocess.add_argument("--json", action="store_true")
    ident_postprocess.add_argument("--gui", action="store_true")
    ident_postprocess.add_argument("--pretty", action="store_true")

    return parser


def add_common(parser: argparse.ArgumentParser) -> None:
    # 这些参数是所有命令共享的运行上下文：
    # - 选 fake 还是 sdk；
    # - 选哪台模型；
    # - 走哪张 CAN 接口；
    # - 是否覆盖 SDK 默认 URDF；
    # - 结果是给脚本看还是给人看。
    parser.add_argument("--adapter", choices=[kind.value for kind in AdapterKind], default=AdapterKind.FAKE.value)
    parser.add_argument("--model", default="X5")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--urdf-path")
    parser.add_argument("--gravity-compensation", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--pretty", action="store_true")


def add_identification_profile_args(parser: argparse.ArgumentParser) -> None:
    # 参数辨识 profile 都走同一套轨迹参数。
    # 不同 profile 会只读取自己需要的字段，避免 CLI 命令膨胀成三套重复入口。
    parser.add_argument("--profile", choices=["gravity_sweep", "friction_sweep", "fourier_multisine"], required=True)
    parser.add_argument("--dof", type=int, default=6)
    parser.add_argument("--sample-hz", type=float, default=100.0)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--amplitude", type=float)
    parser.add_argument("--harmonics", type=int, default=5)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--optimize", action="store_true")
    parser.add_argument("--candidate-count", type=int, default=12)


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == "list-debug-profiles":
        return list_debug_profiles(args)
    # `--json` 和 `--gui` 分别服务机器和人，语义互斥，避免输出介质混乱。
    if getattr(args, "json", False) and getattr(args, "gui", False):
        response = exc_to_response(ArmctrlError(ErrorCode.INVALID_REQUEST, "--json and --gui cannot be used together"))
        emit_json(response.to_dict())
        return 2

    try:
        if args.command in {"ident-plan", "ident-run", "ident-postprocess"}:
            response = dispatch_identification(args)
        else:
            executor = build_executor(args)
            response = dispatch(args, executor)
    except ArmctrlError as exc:
        response = exc_to_response(exc)
    except Exception as exc:
        response = exc_to_response(ArmctrlError(ErrorCode.INVALID_REQUEST, str(exc)))

    if response is None:
        return 2
    # teleop 的 GUI 模式本身会持续显示反馈，不需要额外往终端打一份摘要。
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
    # 适配器选择严格限制在 fake / sdk 两种，避免 CLI 侧出现隐式后门。
    if args.adapter == AdapterKind.SDK.value:
        adapter = Arx5SDKAdapter(
            model=args.model,
            interface=args.interface,
            urdf_path=args.urdf_path,
            gravity_compensation=True if args.gravity_compensation else None,
        )
    else:
        adapter = FakeArx5Adapter()
    # 真实 SDK 的运动命令必须显式确认。
    # 这里把确认要求放在入口层，而不是深埋在 adapter 内部，用户体验更清晰。
    if args.adapter == AdapterKind.SDK.value and args.command in {"move-eef", "teleop-xbox"}:
        if not args.execute or args.confirm != MOVE_CONFIRMATION:
            raise ArmctrlError(
                code=ErrorCode.CONFIRMATION_REQUIRED,
                message=f"real SDK motion requires --execute --confirm '{MOVE_CONFIRMATION}'",
            )
    # executor 创建前先做一次连接验证，避免后续运行到半路才发现接口不可用。
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
    # 这里保持“一个子命令一个分支”的直白结构。
    # 虽然可以用映射表压缩代码，但当前写法对调试和新人阅读更友好。
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
            # `--json` 保持纯机器可读，不混入 GUI 或额外提示。
            runner = XboxDebugRunner(
                executor=executor,
                mapper=XboxMapper(),
                events=events,
                rate_hz=args.rate_hz,
                max_events=args.max_events,
                source=source,
            )
            return runner.run()
        # GUI 模式下，控制线程和 Tk 主循环必须分线程协作：
        # - worker 线程跑 teleop 控制循环；
        # - 主线程跑 Tk 事件循环。
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
            # 这个保护分支主要针对 GUI 被用户提前关掉、而 worker 尚未产出最终响应的情况。
            raise ArmctrlError(
                ErrorCode.INVALID_REQUEST,
                "teleop dashboard exited before runner produced a response",
            )
        return result["response"]
    raise ArmctrlError(code=ErrorCode.INVALID_REQUEST, message=f"unsupported command {args.command}")


def dispatch_identification(args: argparse.Namespace) -> CommandResponse:
    # 参数辨识命令使用关节空间后端，不复用 cartesian executor。
    # 这样可以直接使用 SDK 的 Arx5JointController 和 set_joint_traj。
    if args.command == "ident-plan":
        profile = build_identification_profile(args)
        validation = validate_trajectory(profile, TrajectorySafetyLimits.conservative(profile.dof))
        detail = profile.summary()
        if args.output:
            path = DatasetRecorder(args.output).write_trajectory(profile)
            detail["planned_trajectory_csv"] = str(path)
        if not validation.allowed:
            return CommandResponse(
                CommandStatus.REJECTED,
                validation.error.message if validation.error else "identification trajectory rejected",
                error=validation.error,
                detail=detail,
            )
        return CommandResponse(CommandStatus.COMPLETED, "identification trajectory planned", detail=detail)
    if args.command == "ident-run":
        if args.adapter == AdapterKind.SDK.value and (not args.execute or args.confirm != MOVE_CONFIRMATION):
            raise ArmctrlError(
                code=ErrorCode.CONFIRMATION_REQUIRED,
                message=f"real SDK identification requires --execute --confirm '{MOVE_CONFIRMATION}'",
            )
        profile = build_identification_profile(args)
        output_dir = args.output or default_identification_output_dir(profile.name)
        backend = build_joint_backend(
            adapter=args.adapter,
            model=args.model,
            interface=args.interface,
            dof=profile.dof,
            urdf_path=args.urdf_path,
        )
        runner = IdentificationRunner(
            backend,
            sample_hz=args.sample_hz,
            safety_limits=TrajectorySafetyLimits.conservative(profile.dof),
            damping_after=not args.no_damping_after,
        )
        # fake 后端默认执行，真实 SDK 必须显式 --execute。
        execute = args.execute or args.adapter == AdapterKind.FAKE.value
        return runner.run(
            profile,
            recorder=DatasetRecorder(output_dir),
            execute=execute,
            urdf_path=args.urdf_path,
        )
    if args.command == "ident-postprocess":
        dataset = Path(args.dataset).expanduser().resolve()
        output = Path(args.output).expanduser().resolve() if args.output else dataset / "processed"
        tools = tuple(args.tool or ["all"])
        return postprocess_dataset(
            dataset_dir=dataset,
            output_dir=output,
            tools=tools,
            urdf_path=args.urdf_path,
            smoothing_window=args.smoothing_window,
        )
    raise ArmctrlError(ErrorCode.INVALID_REQUEST, f"unsupported identification command {args.command}")


def build_identification_profile(args: argparse.Namespace):
    # 三个 profile 的默认参数按风险递增设置。
    # 用户可以用 --duration / --amplitude 覆盖，但仍会经过 safety 预检查。
    if args.profile == "gravity_sweep":
        return generate_gravity_sweep(
            dof=args.dof,
            sample_hz=args.sample_hz,
            amplitude_rad=args.amplitude if args.amplitude is not None else 0.20,
            segment_duration_s=args.duration if args.duration is not None else 4.0,
        )
    if args.profile == "friction_sweep":
        return generate_friction_sweep(
            dof=args.dof,
            sample_hz=args.sample_hz,
            amplitude_rad=args.amplitude if args.amplitude is not None else 0.15,
            slow_speed_radps=max(0.02, (args.amplitude or 0.15) / max(args.duration or 2.5, 0.5)),
        )
    if args.profile == "fourier_multisine":
        duration_s = args.duration if args.duration is not None else 12.0
        # 傅里叶轨迹加速度大致随 amplitude / duration² 增大。
        # 默认值随时长缩放，短测试不会因为默认参数直接越过安全限幅；
        # 用户显式传 --amplitude 时仍按用户值生成并交给 safety 检查。
        default_amplitude = min(0.12, 0.02 * duration_s * duration_s)
        amplitude_rad = args.amplitude if args.amplitude is not None else default_amplitude
        if args.optimize:
            return optimize_fourier_multisine(
                dof=args.dof,
                sample_hz=args.sample_hz,
                duration_s=duration_s,
                harmonics=args.harmonics,
                amplitude_rad=amplitude_rad,
                seed=args.seed,
                candidate_count=args.candidate_count,
                safety_limits=TrajectorySafetyLimits.conservative(args.dof),
            )
        return generate_fourier_multisine(
            dof=args.dof,
            sample_hz=args.sample_hz,
            duration_s=duration_s,
            harmonics=args.harmonics,
            amplitude_rad=amplitude_rad,
            seed=args.seed,
        )
    raise ArmctrlError(ErrorCode.INVALID_REQUEST, f"unsupported identification profile {args.profile}")


def default_identification_output_dir(profile_name: str) -> str:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return str(Path("runs") / "identification" / f"{timestamp}-{profile_name}")


def list_debug_profiles(args: argparse.Namespace) -> int:
    # profile 列表来自注册表，而不是手写常量，保证 CLI 与安全层看到的是同一套定义。
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
    # 显式走 stdout，方便 pytest 的 `capsys` 和 shell 管道接入。
    sys.stdout.write(message + "\n")


def emit_json(payload: dict, *, pretty: bool = False) -> None:
    # `ensure_ascii=False` 让中文错误信息和 UI 摘要保持可读。
    if pretty:
        emit(json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2))
    else:
        emit(json.dumps(payload, ensure_ascii=False, sort_keys=True))


def command_source(args: argparse.Namespace) -> str:
    # GUI 标题栏和摘要会用到这个来源字符串，便于快速分辨数据从哪条命令进来。
    if args.command == "teleop-xbox":
        return args.event_jsonl or args.device or "/dev/input/event0"
    adapter = getattr(args, "adapter", "unknown")
    return f"{args.command}:{adapter}"


def exc_to_response(error: ArmctrlError):
    # 统一把异常压平为 `CommandResponse`，这样 CLI / GUI / 测试都只处理一种结果类型。
    from armctrl.protocol.enums import CommandStatus
    from armctrl.protocol.models import CommandResponse

    return CommandResponse(CommandStatus.REJECTED, error.message, error=error)


if __name__ == "__main__":
    raise SystemExit(main())
