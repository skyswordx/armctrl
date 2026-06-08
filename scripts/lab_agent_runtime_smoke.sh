#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

STATE_FILE="${ARMCTRL_LAB_AGENT_STATE:-.lab_agent_runtime_smoke.env}"
SAFE_CENTER_DEFAULT="0.0 0.3 0.3 0.0 0.0 0.0"
AGENT_Q_TARGET_DEFAULT="0.0 0.302 0.3 0.0 0.0 0.0"
AGENT_DELTA_POSITION_DEFAULT="0.002 0.000 0.000"
AGENT_DELTA_RPY_DEFAULT="0.0 0.0 0.0"
CONTROL_PERIOD_S_DEFAULT="0.1"
SEND_HZ_DEFAULT="50"
HOLD_HZ_DEFAULT="50"
MAX_JOINT_DELTA_RAD_DEFAULT="0.005"
MAX_TRACKING_ERROR_RAD_DEFAULT="0.08"
EXPECT_SAMPLE_COUNT_DEFAULT="6"

usage() {
  cat <<'EOF'
Usage:
  scripts/lab_agent_runtime_smoke.sh init [run-dir]
  scripts/lab_agent_runtime_smoke.sh start
  scripts/lab_agent_runtime_smoke.sh status [label]
  scripts/lab_agent_runtime_smoke.sh plan
  scripts/lab_agent_runtime_smoke.sh run
  scripts/lab_agent_runtime_smoke.sh check
  scripts/lab_agent_runtime_smoke.sh back-safe
  scripts/lab_agent_runtime_smoke.sh stop
  scripts/lab_agent_runtime_smoke.sh env

Expected lab flow:
  Terminal 1:
    scripts/lab_agent_runtime_smoke.sh init
    scripts/lab_agent_runtime_smoke.sh start

  Terminal 2:
    scripts/lab_agent_runtime_smoke.sh status before_agent
    scripts/lab_agent_runtime_smoke.sh plan
    scripts/lab_agent_runtime_smoke.sh run
    scripts/lab_agent_runtime_smoke.sh check
    scripts/lab_agent_runtime_smoke.sh back-safe

Expected phenomena:
  - start opens SDK/CAN once, recovers to SAFE_CENTER, then keeps active hold.
  - plan does not move hardware; it writes an EEF/Cartesian Agent contract.
  - run submits owner=agent through live runtime IPC; it must not open SDK directly.
  - The default real motion is intentionally tiny: joint2 +0.002 rad from live hold.
  - Agent intent is 10 Hz by default; runtime sends at 50 Hz after interpolation.
  - check should report owner=agent, mode=agent_servo, sample_count=6, landing=hold.

Meaning:
  - This is not full Cartesian EEF execution yet.
  - It validates the Agent-to-runtime ownership path with an upstream EEF contract and
    a bounded joint-space runtime smoke command.
EOF
}

write_state() {
  local run_dir="$1"
  mkdir -p "$run_dir"
  cat > "$STATE_FILE" <<EOF
export RUN_DIR="$run_dir"
export SAFE_CENTER="$SAFE_CENTER_DEFAULT"
export AGENT_Q_TARGET="$AGENT_Q_TARGET_DEFAULT"
export AGENT_DELTA_POSITION="$AGENT_DELTA_POSITION_DEFAULT"
export AGENT_DELTA_RPY="$AGENT_DELTA_RPY_DEFAULT"
export CONTROL_PERIOD_S="$CONTROL_PERIOD_S_DEFAULT"
export SEND_HZ="$SEND_HZ_DEFAULT"
export HOLD_HZ="$HOLD_HZ_DEFAULT"
export MAX_JOINT_DELTA_RAD="$MAX_JOINT_DELTA_RAD_DEFAULT"
export MAX_TRACKING_ERROR_RAD="$MAX_TRACKING_ERROR_RAD_DEFAULT"
export EXPECT_SAMPLE_COUNT="$EXPECT_SAMPLE_COUNT_DEFAULT"
EOF
  echo "Wrote $STATE_FILE"
  echo "RUN_DIR=$run_dir"
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
  uv run armctrl runtime status \
    --session-artifact "$RUN_DIR/runtime_session.json" \
    --max-heartbeat-age-s 1.0 \
    --output "$RUN_DIR/runtime_status_${label}.json" \
    --json
}

preposition_to() {
  local name="$1"
  local q_target="$2"
  ensure_state
  uv run armctrl runtime preposition \
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
    uv run armctrl runtime start \
      --backend arx5_sdk \
      --model X5 \
      --interface can0 \
      --safe-center $SAFE_CENTER \
      --send-hz "$SEND_HZ" \
      --hold-hz "$HOLD_HZ" \
      --max-joint-step-rad 0.01 \
      --max-heartbeat-age-s 1.0 \
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
    uv run armctrl agent-flow plan \
      --preset home \
      --eef-mode pose_delta \
      --backend sdk_cartesian \
      --delta-position $AGENT_DELTA_POSITION \
      --delta-rpy $AGENT_DELTA_RPY \
      --control-period-s "$CONTROL_PERIOD_S" \
      --output "$RUN_DIR/agent-flow-plan" \
      --json
    ;;
  run)
    ensure_plan
    runtime_status "before_agent"
    uv run armctrl agent-flow runtime-smoke-real \
      --contract "$RUN_DIR/agent-flow-plan/agent_flow_plan.json" \
      --readiness-artifact "$RUN_DIR/runtime_status_before_agent.json" \
      --runtime-session-artifact "$RUN_DIR/runtime_session.json" \
      --model X5 \
      --interface can0 \
      --q-start $SAFE_CENTER \
      --q-target $AGENT_Q_TARGET \
      --send-hz "$SEND_HZ" \
      --max-joint-delta-rad "$MAX_JOINT_DELTA_RAD" \
      --max-tracking-error-rad "$MAX_TRACKING_ERROR_RAD" \
      --confirm "I UNDERSTAND THIS WILL MOVE THE ARM WITH AGENT INTENT" \
      --output "$RUN_DIR/agent_runtime_attach.json" \
      --json
    ;;
  check)
    ensure_state
    uv run armctrl runtime result-check \
      --run-dir "$RUN_DIR" \
      --expect-owner agent \
      --expect-mode agent_servo \
      --expect-sample-count "$EXPECT_SAMPLE_COUNT" \
      --max-jitter-p99-ms 5 \
      --max-tracking-error-rad "$MAX_TRACKING_ERROR_RAD" \
      --json
    ;;
  back-safe)
    preposition_to "safe_center" "$SAFE_CENTER"
    ;;
  stop)
    ensure_state
    uv run armctrl runtime stop \
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
