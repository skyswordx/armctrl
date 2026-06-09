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
BACKEND_DEFAULT="${ARMCTRL_BACKEND:-arx5_sdk}"
AGENT_EEF_BACKEND_DEFAULT="${ARMCTRL_AGENT_EEF_BACKEND:-sdk_cartesian}"
SAFE_CENTER_DEFAULT="0.0 0.3 0.3 0.0 0.0 0.0"
AGENT_Q_TARGET_DEFAULT="0.0 0.302 0.3 0.0 0.0 0.0"
AGENT_DELTA_POSITION_DEFAULT="0.002 0.000 0.000"
AGENT_DELTA_RPY_DEFAULT="0.0 0.0 0.0"
CONTROL_PERIOD_S_DEFAULT="0.1"
SEND_HZ_DEFAULT="50"
HOLD_HZ_DEFAULT="50"
MAX_JOINT_DELTA_RAD_DEFAULT="0.005"
MAX_TRACKING_ERROR_RAD_DEFAULT="0.08"
MAX_HEARTBEAT_AGE_S_DEFAULT="5"
EXPECT_INTENT_SAMPLE_COUNT_DEFAULT="6"
EXPECT_TRAJECTORY_SAMPLE_COUNT_DEFAULT="6"

usage() {
  cat <<'EOF'
Usage:
  scripts/lab_agent_runtime_smoke.sh init [run-dir]
  scripts/lab_agent_runtime_smoke.sh start
  scripts/lab_agent_runtime_smoke.sh start-eef
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

  Agent EEF hardware path:
    # Terminal 1; this starts the ARX5 Cartesian controller from the
    # current measured EEF pose, matching the feature/subsystem teleop path.
    scripts/lab_agent_runtime_smoke.sh start-eef

    # Terminal 2:
    scripts/lab_agent_runtime_smoke.sh run-eef
    scripts/lab_agent_runtime_smoke.sh check-eef

No-hardware rehearsal:
  ARMCTRL_BACKEND=fake scripts/lab_agent_runtime_smoke.sh init
  ARMCTRL_BACKEND=fake scripts/lab_agent_runtime_smoke.sh start
  scripts/lab_agent_runtime_smoke.sh run-intent
  scripts/lab_agent_runtime_smoke.sh check-intent

Expected phenomena:
  - arx5_sdk start opens SDK/CAN once, recovers to SAFE_CENTER, then keeps active hold.
  - start-eef opens the ARX5 Cartesian controller directly and syncs its
    internal EEF target to the current measured pose before gain recovery.
  - sdk_cartesian start does not require SAFE_CENTER; it is an Agent EEF takeover path.
  - fake start uses the same runtime queue/session contract without touching CAN.
  - plan does not move hardware; it writes an optional EEF/Cartesian Agent contract for review only.
  - run-intent submits motion kind=joint-intent, owner=agent through live runtime IPC.
  - run-trajectory submits motion kind=joint-trajectory, owner=agent through live runtime IPC.
  - run-eef submits motion kind=eef-delta, owner=agent through live runtime IPC.
  - The default real motion is intentionally tiny: joint2 +0.002 rad from live hold.
  - Agent intent is 10 Hz by default; runtime sends at 50 Hz after interpolation.
  - Checks should report owner=agent and release back to hold.

Meaning:
  - EEF submit is wired as an Agent command space, not a separate source.
  - EEF hardware execution defaults to sdk_cartesian and uses ARX5 SDK Cartesian controller.
  - sdk_cartesian is an EEF takeover backend; SysID SAFE_CENTER recovery stays on arx5_sdk.
  - It validates Agent-to-runtime ownership for joint_intent and joint_trajectory.
EOF
}

write_state() {
  local run_dir="$1"
  mkdir -p "$run_dir"
  cat > "$STATE_FILE" <<EOF
export RUN_DIR="$run_dir"
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

trajectory_q_point_args() {
  ensure_state
  "$ARMCTRL_UV_BIN" run python - "$SAFE_CENTER" "$AGENT_Q_TARGET" "$EXPECT_TRAJECTORY_SAMPLE_COUNT" <<'PY'
import sys

q_start = [float(value) for value in sys.argv[1].split()]
q_target = [float(value) for value in sys.argv[2].split()]
count = int(sys.argv[3])
if len(q_start) != len(q_target):
    raise SystemExit("SAFE_CENTER and AGENT_Q_TARGET must have the same length")
if count < 2:
    raise SystemExit("EXPECT_TRAJECTORY_SAMPLE_COUNT must be >= 2")
for index in range(count):
    alpha = index / (count - 1)
    q = [start + (target - start) * alpha for start, target in zip(q_start, q_target)]
    print("--q-point " + " ".join(f"{value:.9f}" for value in q))
PY
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
    runtime_stop_if_present
    echo "[start-eef] Start sdk_cartesian runtime from current measured EEF pose."
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
      --max-tracking-error-rad "$MAX_TRACKING_ERROR_RAD" \
      --max-heartbeat-age-s "$MAX_HEARTBEAT_AGE_S" \
      --output "$RUN_DIR/agent_runtime_attach.json" \
      --json
    ;;
  run-trajectory)
    ensure_state
    runtime_status "before_agent_trajectory"
    mapfile -t q_point_args < <(trajectory_q_point_args)
    q_point_cli=()
    for line in "${q_point_args[@]}"; do
      # shellcheck disable=SC2206
      parts=($line)
      q_point_cli+=("${parts[@]}")
    done
    "$ARMCTRL_UV_BIN" run armctrl motion submit joint-trajectory \
      --session-artifact "$RUN_DIR/runtime_session.json" \
      --owner agent \
      --expected-q-start $SAFE_CENTER \
      "${q_point_cli[@]}" \
      --send-hz "$SEND_HZ" \
      --max-heartbeat-age-s "$MAX_HEARTBEAT_AGE_S" \
      --heartbeat-timeout-s 0.5 \
      --max-tracking-error-rad "$MAX_TRACKING_ERROR_RAD" \
      --output "$RUN_DIR/agent_runtime_trajectory_submit.json" \
      --json
    ;;
  run-eef)
    ensure_state
    runtime_status "before_agent_eef"
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
      --start-pose-policy current_measured_pose \
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
