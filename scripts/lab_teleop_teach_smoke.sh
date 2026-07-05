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

STATE_FILE="${ARMCTRL_LAB_TELEOP_STATE:-.lab_teleop_teach_smoke.env}"
SAFE_CENTER_DEFAULT="0.0 0.3 0.3 0.0 0.0 0.0"
SEND_HZ_DEFAULT="50"
HOLD_HZ_DEFAULT="50"
MAX_HEARTBEAT_AGE_S_DEFAULT="5"
XBOX_DEVICE_DEFAULT="${ARMCTRL_XBOX_DEVICE:-/dev/input/event0}"

usage() {
  cat <<'EOF'
Usage:
  scripts/lab_teleop_teach_smoke.sh init [run-dir]
  scripts/lab_teleop_teach_smoke.sh start
  scripts/lab_teleop_teach_smoke.sh status [label]
  scripts/lab_teleop_teach_smoke.sh teach-on
  scripts/lab_teleop_teach_smoke.sh teleop-on
  scripts/lab_teleop_teach_smoke.sh xbox-jsonl
  scripts/lab_teleop_teach_smoke.sh xbox-device [/dev/input/eventX]
  scripts/lab_teleop_teach_smoke.sh check
  scripts/lab_teleop_teach_smoke.sh stop
  scripts/lab_teleop_teach_smoke.sh env

Architecture:
  This smoke keeps SDK/CAN singleton ownership inside the long-lived runtime.
  Xbox input and teach/teleop gain switches only submit runtime queue commands.
  They never open ARX5 SDK/CAN directly.

Expected lab flow:
  Terminal 1:
    scripts/lab_teleop_teach_smoke.sh init
    scripts/lab_teleop_teach_smoke.sh start

  Terminal 2:
    scripts/lab_teleop_teach_smoke.sh status before_teach
    scripts/lab_teleop_teach_smoke.sh teach-on
    scripts/lab_teleop_teach_smoke.sh check
    # hand-guide gently; arm should feel low-stiffness, not passive droop
    scripts/lab_teleop_teach_smoke.sh teleop-on
    scripts/lab_teleop_teach_smoke.sh xbox-jsonl
    scripts/lab_teleop_teach_smoke.sh xbox-device /dev/input/eventX
    scripts/lab_teleop_teach_smoke.sh check

No-hardware rehearsal:
  ARMCTRL_BACKEND=fake scripts/lab_teleop_teach_smoke.sh init
  ARMCTRL_BACKEND=fake scripts/lab_teleop_teach_smoke.sh start
  scripts/lab_teleop_teach_smoke.sh teach-on
  scripts/lab_teleop_teach_smoke.sh xbox-jsonl
  scripts/lab_teleop_teach_smoke.sh check
EOF
}

write_state() {
  local run_dir="$1"
  mkdir -p "$run_dir"
  cat > "$STATE_FILE" <<EOF
export RUN_DIR="$run_dir"
export ARMCTRL_BACKEND="${ARMCTRL_BACKEND:-arx5_sdk}"
export SAFE_CENTER="$SAFE_CENTER_DEFAULT"
export SEND_HZ="$SEND_HZ_DEFAULT"
export HOLD_HZ="$HOLD_HZ_DEFAULT"
export MAX_HEARTBEAT_AGE_S="$MAX_HEARTBEAT_AGE_S_DEFAULT"
export XBOX_DEVICE="$XBOX_DEVICE_DEFAULT"
EOF
  echo "Wrote $STATE_FILE"
  echo "RUN_DIR=$run_dir"
}

ensure_state() {
  if [[ ! -f "$STATE_FILE" ]]; then
    echo "Missing $STATE_FILE. Run: scripts/lab_teleop_teach_smoke.sh init" >&2
    exit 2
  fi
  # shellcheck disable=SC1090
  source "$STATE_FILE"
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

live_q_hold() {
  local status_path="$1"
  "$ARMCTRL_UV_BIN" run python - "$status_path" <<'PY'
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
}

submit_profile() {
  local profile="$1"
  ensure_state
  runtime_status "before_${profile}"
  local q_hold
  q_hold="$(live_q_hold "$RUN_DIR/console_status_before_${profile}.json")"
  "$ARMCTRL_UV_BIN" run armctrl motion submit teleop-profile \
    --session-artifact "$RUN_DIR/runtime_session.json" \
    --owner teleop \
    --backend sdk_cartesian \
    --profile "$profile" \
    --expected-q-start $q_hold \
    --start-pose-policy live_hold \
    --max-heartbeat-age-s "$MAX_HEARTBEAT_AGE_S" \
    --heartbeat-timeout-s 0.5 \
    --output "$RUN_DIR/teleop_profile_${profile}.json" \
    --json
}

write_sample_events() {
  ensure_state
  "$ARMCTRL_UV_BIN" run python - "$RUN_DIR/xbox_sample_events.jsonl" <<'PY'
import json
import sys
from pathlib import Path

events = [
    {"event_type": 1, "code": 311, "value": 1, "timestamp": 1.0},
    {"event_type": 3, "code": 1, "value": -32767, "timestamp": 1.1},
    {"event_type": 1, "code": 311, "value": 0, "timestamp": 1.2},
]
path = Path(sys.argv[1])
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text("\n".join(json.dumps(item, sort_keys=True) for item in events) + "\n", encoding="utf-8")
print(path)
PY
}

run_xbox_smoke() {
  local source_kind="$1"
  local source="$2"
  ensure_state
  runtime_status "before_xbox_${source_kind}"
  local q_hold
  q_hold="$(live_q_hold "$RUN_DIR/console_status_before_xbox_${source_kind}.json")"
  "$ARMCTRL_UV_BIN" run armctrl teleop xbox-runtime-smoke \
    --session-artifact "$RUN_DIR/runtime_session.json" \
    --owner teleop \
    --backend sdk_cartesian \
    --source-kind "$source_kind" \
    --source "$source" \
    --expected-q-start $q_hold \
    --send-hz "$SEND_HZ" \
    --max-heartbeat-age-s "$MAX_HEARTBEAT_AGE_S" \
    --heartbeat-timeout-s 0.5 \
    --output "$RUN_DIR/xbox_runtime_smoke_${source_kind}.json" \
    --json
}

check_results() {
  ensure_state
  "$ARMCTRL_UV_BIN" run python - "$RUN_DIR" <<'PY'
import json
import sys
from pathlib import Path

run_dir = Path(sys.argv[1])
result_dir = run_dir / "runtime_session_commands" / "results"
results = sorted(result_dir.glob("*.json"), key=lambda path: path.stat().st_mtime)
teleop_results = []
for path in results:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("owner") == "teleop":
        teleop_results.append((path, payload))
if not teleop_results:
    raise SystemExit("no teleop runtime result artifact found")
summary = []
for path, payload in teleop_results:
    summary.append(
        {
            "path": str(path),
            "status": payload.get("status"),
            "kind": payload.get("kind"),
            "command_space": payload.get("command_space"),
            "landing_mode": payload.get("landing_mode"),
            "profile": (
                payload.get("motion", {})
                .get("teleop_profile", {})
                .get("profile")
                if isinstance(payload.get("motion"), dict)
                else None
            ),
        }
    )
print(json.dumps({"status": "ok", "teleop_results": summary}, ensure_ascii=False))
if any(item["status"] != "completed" for item in summary):
    raise SystemExit(1)
PY
}

cmd="${1:-help}"
case "$cmd" in
  init)
    write_state "${2:-runs/lab-teleop-teach-$(date +%Y%m%d-%H%M%S)}"
    ;;
  start)
    if [[ ! -f "$STATE_FILE" ]]; then
      write_state "runs/lab-teleop-teach-$(date +%Y%m%d-%H%M%S)"
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
      --eef-adapter sdk_cartesian \
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
  status)
    runtime_status "${2:-manual}"
    ;;
  teach-on)
    submit_profile "zero_gravity_drag"
    ;;
  teleop-on)
    submit_profile "teleop"
    ;;
  xbox-jsonl)
    sample_path="$(write_sample_events)"
    run_xbox_smoke "jsonl" "$sample_path"
    ;;
  xbox-device)
    ensure_state
    run_xbox_smoke "device" "${2:-$XBOX_DEVICE}"
    ;;
  check)
    check_results
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

