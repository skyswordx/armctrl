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
from armctrl.calibration.gripper import GripperCalibrationService
from armctrl.compat.lerobot import build_lerobot_contract
from armctrl.daemon.executor import ArmCommandExecutor
from armctrl.identification.backends import build_joint_backend
from armctrl.identification.optimization import optimize_fourier_multisine
from armctrl.identification.postprocess import postprocess_dataset
from armctrl.identification.recorder import DatasetRecorder
from armctrl.identification.runner import IdentificationRunner
from armctrl.identification.safety import (
    TrajectorySafetyLimits,
    model_coordinate_contract,
    model_safety_limits,
    validate_trajectory,
)
from armctrl.identification.solver import pinocchio_regressor_scorer
from armctrl.identification.trajectories import (
    generate_fourier_multisine,
    generate_friction_sweep,
    generate_gravity_sweep,
)
from armctrl.protocol.enums import AdapterKind, CommandStatus, DebugProfileName, ErrorCode
from armctrl.protocol.errors import ArmctrlError
from armctrl.protocol.models import CommandResponse, DebugProfileRequest, MoveEEFRequest
from armctrl.safety.profiles import DebugProfileRegistry
from armctrl.teleop.mapping import XboxMapper
from armctrl.teleop.xbox import XboxDebugRunner, create_tk_dashboard, load_events, show_response_dashboard

MOVE_CONFIRMATION = "I UNDERSTAND THIS WILL MOVE THE ARM"
DEFAULT_GRAVITY_SWEEP_AMPLITUDE_RAD = 0.55
DEFAULT_GRAVITY_SWEEP_SEGMENT_DURATION_S = 12.0
DEFAULT_GRAVITY_SWEEP_DWELL_S = 1.0
DEFAULT_X5_GRAVITY_SWEEP_CENTER = (0.0, 0.80, 0.85, 0.0, 0.0, 0.0)
DEFAULT_FRICTION_SWEEP_AMPLITUDE_RAD = 0.12
DEFAULT_FRICTION_SWEEP_SLOW_SPEED_RADPS = 0.025
DEFAULT_FRICTION_SWEEP_MEDIUM_SPEED_RADPS = 0.06
DEFAULT_FRICTION_SWEEP_FAST_SPEED_RADPS = 0.12
DEFAULT_FOURIER_DURATION_S = 40.0
DEFAULT_FOURIER_AMPLITUDE_RAD = 1.3
DEFAULT_X5_FOURIER_AMPLITUDE_RAD = 0.75
DEFAULT_FOURIER_HARMONICS = 5
GRIPPER_CALIBRATION_COMMANDS = {
    "gripper-calibration-show",
    "gripper-calibration-set",
    "gripper-calibration-clear",
    "gripper-calibration-wizard",
}


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
    ident_run.add_argument(
        "--damping-after",
        action="store_true",
        help="Request damping after successful completion; Ctrl-C/fault paths always request damping.",
    )
    ident_run.add_argument("--no-damping-after", action="store_false", dest="damping_after", help=argparse.SUPPRESS)
    ident_run.set_defaults(damping_after=False)

    ident_postprocess = subparsers.add_parser("ident-postprocess")
    ident_postprocess.add_argument("--dataset", required=True)
    ident_postprocess.add_argument("--output")
    ident_postprocess.add_argument("--tool", action="append", choices=["all", "pinocchio", "figaroh", "flobaroid", "urdfly"])
    ident_postprocess.add_argument("--urdf-path")
    ident_postprocess.add_argument("--smoothing-window", type=int, default=5)
    ident_postprocess.add_argument(
        "--filter-mode",
        choices=["moving_average", "zero_phase_moving_average"],
        default="zero_phase_moving_average",
    )
    ident_postprocess.add_argument("--json", action="store_true")
    ident_postprocess.add_argument("--gui", action="store_true")
    ident_postprocess.add_argument("--pretty", action="store_true")

    gripper_show = subparsers.add_parser("gripper-calibration-show")
    add_calibration_common(gripper_show)

    gripper_set = subparsers.add_parser("gripper-calibration-set")
    add_calibration_common(gripper_set)
    gripper_set.add_argument("--open-readout", type=float, required=True)
    gripper_set.add_argument("--width", type=float, required=True)
    gripper_set.add_argument("--notes", default="")

    gripper_clear = subparsers.add_parser("gripper-calibration-clear")
    add_calibration_common(gripper_clear)

    gripper_wizard = subparsers.add_parser("gripper-calibration-wizard")
    add_calibration_common(gripper_wizard)
    gripper_wizard.add_argument("--interface", default="can0")

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


def add_calibration_common(parser: argparse.ArgumentParser) -> None:
    # 标定命令不需要 fake/sdk adapter 选择，
    # 它们处理的是项目侧配置文件，以及可选的 SDK 标定流程。
    parser.add_argument("--model", default="X5")
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
    parser.add_argument("--dwell", type=float)
    parser.add_argument("--harmonics", type=int)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument(
        "--q-center",
        nargs="+",
        type=float,
        help="Center joint pose for gravity_sweep and fourier_multisine, in radians",
    )
    parser.add_argument("--optimize", action="store_true")
    parser.add_argument(
        "--optimize-regressor",
        action="store_true",
        help="When Pinocchio and a URDF are available, score Fourier candidates by the true torque regressor condition number.",
    )
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
        elif args.command in GRIPPER_CALIBRATION_COMMANDS:
            response = dispatch_gripper_calibration(args)
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


def dispatch_gripper_calibration(args: argparse.Namespace) -> CommandResponse:
    # 夹爪标定命令统一走项目侧 service。
    # 这样普通控制命令和标定维护命令就不会在 CLI 层重复各自的文件读写逻辑。
    service = GripperCalibrationService()
    if args.command == "gripper-calibration-show":
        return service.show(args.model)
    if args.command == "gripper-calibration-set":
        return service.set(
            model=args.model,
            gripper_open_readout=args.open_readout,
            gripper_width=args.width,
            interface=getattr(args, "interface", None),
            notes=args.notes,
        )
    if args.command == "gripper-calibration-clear":
        return service.clear(args.model)
    if args.command == "gripper-calibration-wizard":
        if getattr(args, "json", False):
            raise ArmctrlError(
                ErrorCode.INVALID_REQUEST,
                "gripper-calibration-wizard is interactive and does not support --json",
            )
        return service.run_wizard(model=args.model, interface=args.interface)
    raise ArmctrlError(ErrorCode.INVALID_REQUEST, f"unsupported gripper calibration command {args.command}")


def dispatch_identification(args: argparse.Namespace) -> CommandResponse:
    # 参数辨识命令使用关节空间后端，不复用 cartesian executor。
    # 这样可以直接使用 SDK 的 Arx5JointController 和 set_joint_traj。
    if args.command == "ident-plan":
        profile = build_identification_profile(args)
        validation = validate_trajectory(profile, model_safety_limits(args.model, profile.dof))
        detail = profile.summary()
        detail["lerobot_contract"] = build_lerobot_contract(dof=profile.dof, gripper=True)
        detail["coordinate_contract"] = model_coordinate_contract(args.model, profile.dof)
        if args.output:
            output_dir = timestamped_output_dir(args.output)
            path = DatasetRecorder(output_dir).write_trajectory(profile)
            detail["output_dir"] = str(output_dir)
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
        output_dir = timestamped_output_dir(args.output) if args.output else Path(default_identification_output_dir(profile.name))
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
            safety_limits=model_safety_limits(args.model, profile.dof),
            damping_after=bool(args.damping_after),
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
        output_base = Path(args.output).expanduser().resolve() if args.output else dataset / "processed"
        output = timestamped_output_dir(output_base)
        tools = tuple(args.tool or ["all"])
        return postprocess_dataset(
            dataset_dir=dataset,
            output_dir=output,
            tools=tools,
            urdf_path=args.urdf_path,
            smoothing_window=args.smoothing_window,
            filter_mode=args.filter_mode,
        )
    raise ArmctrlError(ErrorCode.INVALID_REQUEST, f"unsupported identification command {args.command}")


def build_identification_profile(args: argparse.Namespace):
    # 三个 profile 的默认参数按风险递增设置。
    # 用户可以用 --duration / --amplitude 覆盖，但仍会经过 safety 预检查。
    if args.profile == "gravity_sweep":
        q_center = tuple(args.q_center) if args.q_center is not None else _default_gravity_sweep_center(args)
        return generate_gravity_sweep(
            dof=args.dof,
            sample_hz=args.sample_hz,
            amplitude_rad=args.amplitude if args.amplitude is not None else DEFAULT_GRAVITY_SWEEP_AMPLITUDE_RAD,
            segment_duration_s=args.duration if args.duration is not None else DEFAULT_GRAVITY_SWEEP_SEGMENT_DURATION_S,
            dwell_s=args.dwell if args.dwell is not None else DEFAULT_GRAVITY_SWEEP_DWELL_S,
            q_center=q_center,
        )
    if args.profile == "friction_sweep":
        amplitude_rad = args.amplitude if args.amplitude is not None else DEFAULT_FRICTION_SWEEP_AMPLITUDE_RAD
        q_center = tuple(args.q_center) if args.q_center is not None else _default_gravity_sweep_center(args)
        return generate_friction_sweep(
            dof=args.dof,
            sample_hz=args.sample_hz,
            amplitude_rad=amplitude_rad,
            slow_speed_radps=DEFAULT_FRICTION_SWEEP_SLOW_SPEED_RADPS,
            medium_speed_radps=DEFAULT_FRICTION_SWEEP_MEDIUM_SPEED_RADPS,
            fast_speed_radps=DEFAULT_FRICTION_SWEEP_FAST_SPEED_RADPS,
            q0=q_center,
        )
    if args.profile == "fourier_multisine":
        duration_s = args.duration if args.duration is not None else DEFAULT_FOURIER_DURATION_S
        # 傅里叶轨迹加速度大致随 amplitude / duration² 增大。
        # 默认值随时长缩放，短测试不会因为默认参数直接越过安全限幅；
        # 用户显式传 --amplitude 时仍按用户值生成并交给 safety 检查。
        if args.amplitude is not None:
            amplitude_rad = args.amplitude
        elif args.duration is not None:
            amplitude_rad = min(_default_fourier_amplitude(args), 0.02 * duration_s * duration_s)
        else:
            amplitude_rad = _default_fourier_amplitude(args)
        harmonics = args.harmonics if args.harmonics is not None else DEFAULT_FOURIER_HARMONICS
        q_center = tuple(args.q_center) if args.q_center is not None else _default_fourier_center(args)
        if args.optimize:
            scorer = (
                pinocchio_regressor_scorer(urdf_path=args.urdf_path, dof=args.dof)
                if args.optimize_regressor and args.urdf_path
                else None
            )
            return optimize_fourier_multisine(
                dof=args.dof,
                sample_hz=args.sample_hz,
                duration_s=duration_s,
                harmonics=harmonics,
                amplitude_rad=amplitude_rad,
                seed=args.seed,
                candidate_count=args.candidate_count,
                safety_limits=model_safety_limits(args.model, args.dof),
                q_center=q_center,
                scorer=scorer,
            )
        return generate_fourier_multisine(
            dof=args.dof,
            sample_hz=args.sample_hz,
            duration_s=duration_s,
            harmonics=harmonics,
            amplitude_rad=amplitude_rad,
            seed=args.seed,
            q_center=q_center,
        )
    raise ArmctrlError(ErrorCode.INVALID_REQUEST, f"unsupported identification profile {args.profile}")


def _default_gravity_sweep_center(args: argparse.Namespace) -> tuple[float, ...] | None:
    if args.dof == 6 and getattr(args, "model", "X5") == "X5":
        return DEFAULT_X5_GRAVITY_SWEEP_CENTER
    return None


def _default_fourier_center(args: argparse.Namespace) -> tuple[float, ...] | None:
    if args.dof == 6 and getattr(args, "model", "X5") == "X5":
        return (0.0, 1.20, 1.20, 0.0, 0.0, 0.0)
    return _default_gravity_sweep_center(args)


def _default_fourier_amplitude(args: argparse.Namespace) -> float:
    if args.dof == 6 and getattr(args, "model", "X5") == "X5":
        return DEFAULT_X5_FOURIER_AMPLITUDE_RAD
    return DEFAULT_FOURIER_AMPLITUDE_RAD


def default_identification_output_dir(profile_name: str) -> str:
    return str(timestamped_output_dir(Path("runs") / "identification" / profile_name))


def timestamped_output_dir(base_dir: str | Path) -> Path:
    """把输出目录解析成带时间戳的唯一路径。

    CLI 侧把用户传入的 `--output` 当作目录名前缀，而不是最终落盘目录。
    这样重复运行同一条辨识命令时，不会把旧的 planned/raw/processed 文件覆盖掉。
    """

    base = Path(base_dir).expanduser()
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return (base.parent / f"{base.name}-{timestamp}").resolve()


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
