#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ -z "${ARMCTRL_UV_BIN:-}" ]]; then
  if command -v uv >/dev/null 2>&1; then
    ARMCTRL_UV_BIN="uv"
  elif [[ -x "$HOME/.local/bin/uv" ]]; then
    ARMCTRL_UV_BIN="$HOME/.local/bin/uv"
  else
    ARMCTRL_UV_BIN="uv"
  fi
fi

STATE_FILE="${ARMCTRL_LAB_AGENT_STATE:-.lab_agent_runtime_smoke.env}"
SMOKE_PROFILE_VERSION_CURRENT="10"
BACKEND_DEFAULT="${ARMCTRL_BACKEND:-arx5_sdk}"
AGENT_EEF_BACKEND_DEFAULT="${ARMCTRL_AGENT_EEF_BACKEND:-moveit_servo}"
SAFE_CENTER_DEFAULT="0.0 0.3 0.3 0.0 0.0 0.0"
AGENT_Q_TARGET_DEFAULT="3.00 0.3 0.3 0.0 0.0 0.0"
AGENT_DELTA_POSITION_DEFAULT="0.750 0.000 0.000"
AGENT_DELTA_RPY_DEFAULT="0.0 0.0 0.0"
CONTROL_PERIOD_S_DEFAULT="45.0"
SEND_HZ_DEFAULT="50"
HOLD_HZ_DEFAULT="50"
MAX_JOINT_DELTA_RAD_DEFAULT="3.05"
MAX_JOINT_VELOCITY_RAD_S_DEFAULT="0.12"
MAX_EEF_LINEAR_STEP_M_DEFAULT="0.750"
MAX_EEF_ANGULAR_STEP_RAD_DEFAULT="0.05"
MAX_TRACKING_ERROR_RAD_DEFAULT="0.08"
MAX_HEARTBEAT_AGE_S_DEFAULT="5"
EXPECT_INTENT_SAMPLE_COUNT_DEFAULT="2251"
EXPECT_TRAJECTORY_SAMPLE_COUNT_DEFAULT="2251"

usage() {
  cat <<'EOF'
Usage:
  scripts/lab_agent_runtime_smoke.sh init [run-dir]
  scripts/lab_agent_runtime_smoke.sh start
  scripts/lab_agent_runtime_smoke.sh start-eef  # diagnostic-only, opt-in
  scripts/lab_agent_runtime_smoke.sh status [label]
  scripts/lab_agent_runtime_smoke.sh plan
  scripts/lab_agent_runtime_smoke.sh run-intent
  scripts/lab_agent_runtime_smoke.sh run-trajectory
  scripts/lab_agent_runtime_smoke.sh run-eef
  scripts/lab_agent_runtime_smoke.sh check-intent
  scripts/lab_agent_runtime_smoke.sh check-trajectory
  scripts/lab_agent_runtime_smoke.sh check-eef
  scripts/lab_agent_runtime_smoke.sh back-safe
  scripts/lab_agent_runtime_smoke.sh stop
  scripts/lab_agent_runtime_smoke.sh env

Expected lab flow:
  Terminal 1:
    scripts/lab_agent_runtime_smoke.sh init
    scripts/lab_agent_runtime_smoke.sh start

  Terminal 2:
    scripts/lab_agent_runtime_smoke.sh status before_agent
    scripts/lab_agent_runtime_smoke.sh plan  # optional EEF contract review
    scripts/lab_agent_runtime_smoke.sh run-intent
    scripts/lab_agent_runtime_smoke.sh check-intent
    scripts/lab_agent_runtime_smoke.sh run-trajectory
    scripts/lab_agent_runtime_smoke.sh check-trajectory
    scripts/lab_agent_runtime_smoke.sh back-safe

  Agent EEF contract path:
    # Terminal 1 keeps the long-lived arx5_sdk runtime holding a controlled pose.
    # EEF commands are submitted only when runtime status proves a mature
    # in-runtime servo adapter is configured and executable.
    scripts/lab_agent_runtime_smoke.sh start

    # Terminal 2:
    scripts/lab_agent_runtime_smoke.sh run-eef
    scripts/lab_agent_runtime_smoke.sh check-eef

  Diagnostic-only disconnected Cartesian takeover:
    # This is not the target architecture. It intentionally stops/reopens SDK owner
    # and is only kept for comparison with old lab observations.
    ARMCTRL_ALLOW_DISCONNECTED_EEF_TAKEOVER=1 scripts/lab_agent_runtime_smoke.sh start-eef

No-hardware rehearsal:
  ARMCTRL_BACKEND=fake scripts/lab_agent_runtime_smoke.sh init
  ARMCTRL_BACKEND=fake scripts/lab_agent_runtime_smoke.sh start
  scripts/lab_agent_runtime_smoke.sh run-intent
  scripts/lab_agent_runtime_smoke.sh check-intent
  scripts/lab_agent_runtime_smoke.sh run-eef
  scripts/lab_agent_runtime_smoke.sh check-eef

Expected phenomena:
  - arx5_sdk start opens SDK/CAN once, recovers to SAFE_CENTER, then keeps active hold.
  - Agent EEF target behavior is continuous-owner servo: warm target=current FK,
    then accept small eef-delta/eef-twist/eef-pose commands without releasing SDK/CAN.
  - start-eef is diagnostic-only disconnected takeover and is not the target architecture.
  - fake start uses the same runtime queue/session contract without touching CAN.
  - fake rehearsal registers the MoveIt Servo-style adapter so run-eef is consumed
    by runtime serve. A real arx5_sdk runtime without an adapter manager should
    block run-eef before queuing; that is expected and safe.
  - plan does not move hardware; it writes an optional EEF/Cartesian Agent contract for review only.
  - run-intent submits motion kind=joint-intent, owner=agent through live runtime IPC.
  - run-trajectory submits motion kind=joint-trajectory, owner=agent through live runtime IPC.
  - run-eef submits motion kind=eef-delta only after live status reports
    eef_adapter_manager.eef_command_executable=true for AGENT_EEF_BACKEND.
  - Visible-large profile v10: the default real motion is a wide base-yaw
    sweep, joint1 +3.00 rad over 45.0 s.
  - The default EEF delta smoke uses +750 mm in eef_link x over 45.0 s. This is
    intentionally larger than the runtime's conservative 5 mm default and the
    lab script passes an explicit max-linear-step override for visible EEF validation.
  - Agent intent is low-frequency intent; this smoke stretches it into a 50 Hz
    runtime-owned smoothstep trajectory instead of sending a high-velocity step.
  - The visible joint smoke avoids joint2/joint3 droop-sensitive motion by default;
    override AGENT_Q_TARGET only when the operator explicitly wants shoulder/elbow motion.
  - Checks should report owner=agent and release back to hold.

Meaning:
  - EEF submit is wired as an Agent command space, not a separate source.
  - EEF hardware execution must use a mature backend inside the same long-lived runtime.
  - sdk_cartesian disconnected takeover is diagnostic-only; SysID SAFE_CENTER recovery stays on arx5_sdk.
  - It validates Agent-to-runtime ownership for joint_intent and joint_trajectory.
EOF
}

write_state() {
  local run_dir="$1"
  mkdir -p "$run_dir"
  cat > "$STATE_FILE" <<EOF
export RUN_DIR="$run_dir"
export LAB_AGENT_SMOKE_PROFILE_VERSION="$SMOKE_PROFILE_VERSION_CURRENT"
export ARMCTRL_BACKEND="$BACKEND_DEFAULT"
export AGENT_EEF_BACKEND="$AGENT_EEF_BACKEND_DEFAULT"
export SAFE_CENTER="$SAFE_CENTER_DEFAULT"
export AGENT_Q_TARGET="$AGENT_Q_TARGET_DEFAULT"
export AGENT_DELTA_POSITION="$AGENT_DELTA_POSITION_DEFAULT"
export AGENT_DELTA_RPY="$AGENT_DELTA_RPY_DEFAULT"
export CONTROL_PERIOD_S="$CONTROL_PERIOD_S_DEFAULT"
export SEND_HZ="$SEND_HZ_DEFAULT"
export HOLD_HZ="$HOLD_HZ_DEFAULT"
export MAX_JOINT_DELTA_RAD="$MAX_JOINT_DELTA_RAD_DEFAULT"
export MAX_JOINT_VELOCITY_RAD_S="$MAX_JOINT_VELOCITY_RAD_S_DEFAULT"
export MAX_EEF_LINEAR_STEP_M="$MAX_EEF_LINEAR_STEP_M_DEFAULT"
export MAX_EEF_ANGULAR_STEP_RAD="$MAX_EEF_ANGULAR_STEP_RAD_DEFAULT"
export MAX_TRACKING_ERROR_RAD="$MAX_TRACKING_ERROR_RAD_DEFAULT"
export MAX_HEARTBEAT_AGE_S="$MAX_HEARTBEAT_AGE_S_DEFAULT"
export EXPECT_INTENT_SAMPLE_COUNT="$EXPECT_INTENT_SAMPLE_COUNT_DEFAULT"
export EXPECT_TRAJECTORY_SAMPLE_COUNT="$EXPECT_TRAJECTORY_SAMPLE_COUNT_DEFAULT"
EOF
  echo "Wrote $STATE_FILE"
  echo "RUN_DIR=$run_dir"
  echo "ARMCTRL_BACKEND=$BACKEND_DEFAULT"
  echo "AGENT_EEF_BACKEND=$AGENT_EEF_BACKEND_DEFAULT"
}

ensure_state() {
  if [[ ! -f "$STATE_FILE" ]]; then
    echo "Missing $STATE_FILE. Run: scripts/lab_agent_runtime_smoke.sh init" >&2
    exit 2
  fi
  # shellcheck disable=SC1090
  source "$STATE_FILE"
  : "${LAB_AGENT_SMOKE_PROFILE_VERSION:=0}"
  if [[ "$LAB_AGENT_SMOKE_PROFILE_VERSION" != "$SMOKE_PROFILE_VERSION_CURRENT" ]]; then
    echo "Updating $STATE_FILE to visible smoke profile v$SMOKE_PROFILE_VERSION_CURRENT." >&2
    write_state "$RUN_DIR" >/dev/null
    # shellcheck disable=SC1090
    source "$STATE_FILE"
  fi
  : "${MAX_JOINT_VELOCITY_RAD_S:=$MAX_JOINT_VELOCITY_RAD_S_DEFAULT}"
}

ensure_plan() {
  ensure_state
  if [[ ! -f "$RUN_DIR/agent-flow-plan/agent_flow_plan.json" ]]; then
    echo "Missing Agent plan. Run: scripts/lab_agent_runtime_smoke.sh plan" >&2
    exit 2
  fi
}

runtime_status() {
  local label="${1:-status}"
  ensure_state
  "$ARMCTRL_UV_BIN" run armctrl console status \
    --session-artifact "$RUN_DIR/runtime_session.json" \
    --max-heartbeat-age-s 1.0 \
    --output "$RUN_DIR/console_status_${label}.json" \
    --json
}

require_eef_adapter_ready() {
  local status_path="$1"
  "$ARMCTRL_UV_BIN" run python - "$status_path" "$AGENT_EEF_BACKEND" <<'PY'
import json
import sys
from pathlib import Path

payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
requested = sys.argv[2]
runtime = payload.get("runtime") if isinstance(payload.get("runtime"), dict) else payload
manager = (
    runtime.get("eef_adapter_manager")
    if isinstance(runtime, dict) and isinstance(runtime.get("eef_adapter_manager"), dict)
    else {}
)
configured = manager.get("configured_adapters") if isinstance(manager, dict) else []
if not isinstance(configured, list):
    configured = []
executable = bool(manager.get("eef_command_executable")) if isinstance(manager, dict) else False
if executable and requested in [str(item) for item in configured]:
    raise SystemExit(0)
print(
    json.dumps(
        {
            "status": "blocked",
            "reason": "Agent EEF smoke requires a configured mature in-runtime EEF adapter before queuing hardware motion",
            "requested_backend": requested,
            "eef_adapter_manager": manager,
            "next_gate": "start a runtime that exposes eef_adapter_manager.eef_command_executable=true for the requested backend; fake rehearsal can use ARMCTRL_BACKEND=fake start",
        },
        ensure_ascii=False,
    ),
    file=sys.stderr,
)
raise SystemExit(3)
PY
}

runtime_stop_if_present() {
  ensure_state
  if [[ -f "$RUN_DIR/runtime_session.json" ]]; then
    "$ARMCTRL_UV_BIN" run armctrl runtime stop \
      --session-artifact "$RUN_DIR/runtime_session.json" \
      --max-heartbeat-age-s 1.0 \
      --output "$RUN_DIR/runtime_stop_auto.json" \
      --json >/dev/null 2>&1 || true
    rm -f "$RUN_DIR/runtime_session.json.stop"
    sleep 0.3
  fi
}

wait_runtime_ready() {
  local label="$1"
  local attempts="${2:-100}"
  local delay_s="${3:-0.1}"
  ensure_state
  for _ in $(seq 1 "$attempts"); do
    if [[ -f "$RUN_DIR/runtime_session.json" ]]; then
      set +e
      "$ARMCTRL_UV_BIN" run armctrl runtime status \
        --session-artifact "$RUN_DIR/runtime_session.json" \
        --max-heartbeat-age-s "$MAX_HEARTBEAT_AGE_S" \
        --output "$RUN_DIR/runtime_status_${label}.json" \
        --json >"$RUN_DIR/runtime_status_${label}.stdout.json" 2>"$RUN_DIR/runtime_status_${label}.stderr.log"
      local code=$?
      set -e
      if [[ "$code" == "0" ]]; then
        return 0
      fi
    fi
    sleep "$delay_s"
  done
  echo "Runtime did not become ready; see $RUN_DIR/runtime_status_${label}.stderr.log" >&2
  return 1
}

preposition_to() {
  local name="$1"
  local q_target="$2"
  ensure_state
  "$ARMCTRL_UV_BIN" run armctrl runtime preposition \
    --session-artifact "$RUN_DIR/runtime_session.json" \
    --q-target $q_target \
    --send-hz "$SEND_HZ" \
    --max-joint-step-rad 0.01 \
    --max-heartbeat-age-s 1.0 \
    --output "$RUN_DIR/preposition_${name}.json" \
    --json
}

cmd="${1:-help}"
case "$cmd" in
  init)
    run_dir="${2:-runs/lab-agent-runtime-$(date +%Y%m%d-%H%M%S)}"
    write_state "$run_dir"
    ;;
  start)
    if [[ ! -f "$STATE_FILE" ]]; then
      write_state "runs/lab-agent-runtime-$(date +%Y%m%d-%H%M%S)"
    fi
    ensure_state
    start_args=()
    if [[ "$ARMCTRL_BACKEND" == "fake" ]]; then
      start_args+=(--q-current $SAFE_CENTER)
      start_args+=(--eef-adapter moveit_servo)
    fi
    "$ARMCTRL_UV_BIN" run armctrl runtime start \
      --backend "$ARMCTRL_BACKEND" \
      --model X5 \
      --interface can0 \
      "${start_args[@]}" \
      --safe-center $SAFE_CENTER \
      --send-hz "$SEND_HZ" \
      --hold-hz "$HOLD_HZ" \
      --max-joint-step-rad 0.01 \
      --max-heartbeat-age-s "$MAX_HEARTBEAT_AGE_S" \
      --serve \
      --confirm "I UNDERSTAND THIS WILL START THE REAL ARM RUNTIME" \
      --output "$RUN_DIR/runtime_session.json" \
      --json
    ;;
  start-eef)
    ensure_state
    if [[ "${ARMCTRL_ALLOW_DISCONNECTED_EEF_TAKEOVER:-0}" != "1" ]]; then
      cat >&2 <<'EOF'
start-eef is diagnostic-only and disabled by default.

Target architecture:
  arx5_sdk runtime remains alive, holds a controlled EEF-ready pose, and an
  in-runtime EEF servo backend warms up from target=current FK before ownership
  switches. SDK/CAN owner must not be released between modes.

If you intentionally want the old disconnected sdk_cartesian takeover for
comparison, rerun with:
  ARMCTRL_ALLOW_DISCONNECTED_EEF_TAKEOVER=1 scripts/lab_agent_runtime_smoke.sh start-eef
EOF
      exit 2
    fi
    runtime_stop_if_present
    echo "[start-eef] Diagnostic-only disconnected sdk_cartesian takeover."
    "$ARMCTRL_UV_BIN" run armctrl runtime start \
      --backend sdk_cartesian \
      --model X5 \
      --interface can0 \
      --safe-center $SAFE_CENTER \
      --send-hz "$SEND_HZ" \
      --hold-hz "$HOLD_HZ" \
      --max-joint-step-rad 0.01 \
      --max-start-error-rad 0.02 \
      --max-heartbeat-age-s "$MAX_HEARTBEAT_AGE_S" \
      --allow-diagnostic-sdk-cartesian-takeover \
      --serve \
      --confirm "I UNDERSTAND THIS WILL START THE REAL ARM RUNTIME" \
      --output "$RUN_DIR/runtime_session.json" \
      --json
    ;;
  status)
    runtime_status "${2:-manual}"
    ;;
  plan)
    ensure_state
    "$ARMCTRL_UV_BIN" run armctrl agent-flow plan \
      --preset home \
      --eef-mode pose_delta \
      --backend sdk_cartesian \
      --delta-position $AGENT_DELTA_POSITION \
      --delta-rpy $AGENT_DELTA_RPY \
      --control-period-s "$CONTROL_PERIOD_S" \
      --output "$RUN_DIR/agent-flow-plan" \
      --json
    ;;
  run|run-intent)
    ensure_state
    runtime_status "before_agent"
    "$ARMCTRL_UV_BIN" run armctrl motion submit joint-intent \
      --session-artifact "$RUN_DIR/runtime_session.json" \
      --owner agent \
      --expected-q-start $SAFE_CENTER \
      --q-target $AGENT_Q_TARGET \
      --control-period-s "$CONTROL_PERIOD_S" \
      --send-hz "$SEND_HZ" \
      --max-joint-delta-rad "$MAX_JOINT_DELTA_RAD" \
      --max-joint-velocity-rad-s "$MAX_JOINT_VELOCITY_RAD_S" \
      --max-tracking-error-rad "$MAX_TRACKING_ERROR_RAD" \
      --max-heartbeat-age-s "$MAX_HEARTBEAT_AGE_S" \
      --output "$RUN_DIR/agent_runtime_attach.json" \
      --json
    ;;
  run-trajectory)
    ensure_state
    runtime_status "before_agent_trajectory"
    "$ARMCTRL_UV_BIN" run armctrl motion compile joint-trajectory \
      --source agent \
      --owner agent \
      --expected-q-start $SAFE_CENTER \
      --q-target $AGENT_Q_TARGET \
      --duration-s "$CONTROL_PERIOD_S" \
      --sample-hz "$SEND_HZ" \
      --send-hz "$SEND_HZ" \
      --max-joint-segment-delta-rad "$MAX_JOINT_DELTA_RAD" \
      --max-joint-velocity-rad-s "$MAX_JOINT_VELOCITY_RAD_S" \
      --max-tracking-error-rad "$MAX_TRACKING_ERROR_RAD" \
      --output "$RUN_DIR/agent-joint-trajectory-compile" \
      --json
    "$ARMCTRL_UV_BIN" run armctrl motion submit joint-trajectory \
      --session-artifact "$RUN_DIR/runtime_session.json" \
      --compiled-command "$RUN_DIR/agent-joint-trajectory-compile/compiled_motion_command.json" \
      --max-heartbeat-age-s "$MAX_HEARTBEAT_AGE_S" \
      --heartbeat-timeout-s 0.5 \
      --max-tracking-error-rad "$MAX_TRACKING_ERROR_RAD" \
      --output "$RUN_DIR/agent_runtime_trajectory_submit.json" \
      --json
    ;;
  run-eef)
    ensure_state
    runtime_status "before_agent_eef"
    require_eef_adapter_ready "$RUN_DIR/console_status_before_agent_eef.json"
    live_q_hold=$("$ARMCTRL_UV_BIN" run python - "$RUN_DIR/console_status_before_agent_eef.json" <<'PY'
import json
import sys
from pathlib import Path

payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
runtime = payload.get("runtime") if isinstance(payload.get("runtime"), dict) else payload
q_hold = runtime.get("q_hold") if isinstance(runtime, dict) else None
if not isinstance(q_hold, list) or not q_hold:
    raise SystemExit("runtime status does not contain q_hold")
print(" ".join(str(float(value)) for value in q_hold))
PY
)
    "$ARMCTRL_UV_BIN" run armctrl motion submit eef-delta \
      --session-artifact "$RUN_DIR/runtime_session.json" \
      --owner agent \
      --backend "$AGENT_EEF_BACKEND" \
      --frame eef_link \
      --expected-q-start $live_q_hold \
      --delta-position $AGENT_DELTA_POSITION \
      --delta-rpy $AGENT_DELTA_RPY \
      --control-period-s "$CONTROL_PERIOD_S" \
      --send-hz "$SEND_HZ" \
      --max-linear-step-m "$MAX_EEF_LINEAR_STEP_M" \
      --max-angular-step-rad "$MAX_EEF_ANGULAR_STEP_RAD" \
      --start-pose-policy live_hold \
      --max-heartbeat-age-s "$MAX_HEARTBEAT_AGE_S" \
      --heartbeat-timeout-s 0.5 \
      --output "$RUN_DIR/agent_runtime_eef_submit.json" \
      --json
    ;;
  check|check-intent)
    ensure_state
    "$ARMCTRL_UV_BIN" run armctrl motion result \
      --run-dir "$RUN_DIR" \
      --expect-owner agent \
      --expect-mode agent_servo \
      --expect-sample-count "$EXPECT_INTENT_SAMPLE_COUNT" \
      --max-jitter-p99-ms 5 \
      --max-tracking-error-rad "$MAX_TRACKING_ERROR_RAD" \
      --json
    ;;
  check-trajectory)
    ensure_state
    "$ARMCTRL_UV_BIN" run armctrl motion result \
      --run-dir "$RUN_DIR" \
      --expect-owner agent \
      --expect-mode trajectory_replay \
      --expect-sample-count "$EXPECT_TRAJECTORY_SAMPLE_COUNT" \
      --max-jitter-p99-ms 5 \
      --max-tracking-error-rad "$MAX_TRACKING_ERROR_RAD" \
      --json
    ;;
  check-eef)
    ensure_state
    "$ARMCTRL_UV_BIN" run python - "$RUN_DIR" <<'PY'
import json
import sys
import time
from pathlib import Path

run_dir = Path(sys.argv[1])
result_dir = run_dir / "runtime_session_commands" / "results"
deadline = time.time() + 5.0
while time.time() <= deadline:
    results = sorted(result_dir.glob("*.json"), key=lambda path: path.stat().st_mtime)
    for path in reversed(results):
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("owner") == "agent" and payload.get("mode") == "agent_servo":
            motion = payload.get("motion") if isinstance(payload.get("motion"), dict) else {}
            eef_contract = (
                motion.get("eef_command") if isinstance(motion, dict) else None
            ) or payload.get("eef_command")
            if eef_contract is not None:
                status = payload.get("status")
                print(json.dumps({"status": status, "result": str(path), "payload": payload}, ensure_ascii=False))
                if status != "completed":
                    raise SystemExit(1)
                raise SystemExit(0)
    time.sleep(0.1)
raise SystemExit("no Agent EEF runtime result artifact found")
PY
    ;;
  back-safe)
    preposition_to "safe_center" "$SAFE_CENTER"
    ;;
  stop)
    ensure_state
    "$ARMCTRL_UV_BIN" run armctrl runtime stop \
      --session-artifact "$RUN_DIR/runtime_session.json" \
      --max-heartbeat-age-s 1.0 \
      --output "$RUN_DIR/runtime_stop.json" \
      --json
    ;;
  env)
    ensure_state
    cat "$STATE_FILE"
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
