#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

STATE_FILE="${ARMCTRL_LAB_FOURIER_STATE:-.lab_fourier_sysid.env}"
SAFE_CENTER_DEFAULT="0.0 0.3 0.3 0.0 0.0 0.0"
SLOW_CANDIDATE_DEFAULT="runs/remote_lab_artifacts/lab-fourier-sysid-20260608-154946/runs/lab-fourier-sysid-20260608-154946/ident-fourier-reduced-0p25-slow4x/execution_trajectory.csv"
REDUCED_CANDIDATE_DEFAULT="runs/remote_lab_artifacts/lab-fourier-sysid-20260608-154946/runs/lab-fourier-sysid-20260608-154946/ident-fourier-reduced-0p25-normal/execution_trajectory.csv"
FULL_CANDIDATE_DEFAULT="runs/remote_lab_artifacts/lab-fourier-sysid-20260608-154946/runs/lab-fourier-sysid-20260608-154946/ident-fourier-full/execution_trajectory.csv"
REDUCED_FIRST_Q_DEFAULT="0.07875 0.384375 0.346875 -0.05625 0.07875 0.05625"
FULL_FIRST_Q_DEFAULT="0.315 0.6375 0.4875 -0.225 0.315 0.225"

usage() {
  cat <<'EOF'
Usage:
  scripts/lab_fourier_sysid.sh init [run-dir]
  scripts/lab_fourier_sysid.sh start
  scripts/lab_fourier_sysid.sh status [label]
  scripts/lab_fourier_sysid.sh preposition-slow
  scripts/lab_fourier_sysid.sh compile-slow
  scripts/lab_fourier_sysid.sh run-slow
  scripts/lab_fourier_sysid.sh check-slow
  scripts/lab_fourier_sysid.sh preposition-reduced
  scripts/lab_fourier_sysid.sh compile-reduced
  scripts/lab_fourier_sysid.sh run-reduced
  scripts/lab_fourier_sysid.sh check-reduced
  scripts/lab_fourier_sysid.sh preposition-full
  scripts/lab_fourier_sysid.sh compile-full
  scripts/lab_fourier_sysid.sh run-full
  scripts/lab_fourier_sysid.sh check-full
  scripts/lab_fourier_sysid.sh back-safe
  scripts/lab_fourier_sysid.sh stop
  scripts/lab_fourier_sysid.sh env

Notes:
  - start runs in the foreground and holds the SDK/CAN runtime.
  - Other subcommands can be run from a second terminal; they read .lab_fourier_sysid.env.
  - status and result checks use the formal armctrl console/motion surface.
  - compile-* uses the SysID compiler surface and sends no hardware command.
  - run-* submits the compiled joint trajectory through armctrl motion submit.
  - armctrl sysid run --adapter sdk is intentionally not used by this lab script.
  - Do not run full until slow and reduced have completed smoothly.
EOF
}

write_state() {
  local run_dir="$1"
  mkdir -p "$run_dir"
  cat > "$STATE_FILE" <<EOF
export RUN_DIR="$run_dir"
export SAFE_CENTER="$SAFE_CENTER_DEFAULT"
export SLOW_CANDIDATE="$SLOW_CANDIDATE_DEFAULT"
export REDUCED_CANDIDATE="$REDUCED_CANDIDATE_DEFAULT"
export FULL_CANDIDATE="$FULL_CANDIDATE_DEFAULT"
export REDUCED_FIRST_Q="$REDUCED_FIRST_Q_DEFAULT"
export FULL_FIRST_Q="$FULL_FIRST_Q_DEFAULT"
EOF
  echo "Wrote $STATE_FILE"
  echo "RUN_DIR=$run_dir"
}

ensure_state() {
  if [[ ! -f "$STATE_FILE" ]]; then
    echo "Missing $STATE_FILE. Run: scripts/lab_fourier_sysid.sh init" >&2
    exit 2
  fi
  # shellcheck disable=SC1090
  source "$STATE_FILE"
}

ensure_file() {
  local path="$1"
  if [[ ! -f "$path" ]]; then
    echo "Missing file: $path" >&2
    exit 2
  fi
}

ensure_candidate_files() {
  ensure_file "$SLOW_CANDIDATE"
  ensure_file "$REDUCED_CANDIDATE"
  ensure_file "$FULL_CANDIDATE"
}

runtime_status() {
  local label="${1:-status}"
  ensure_state
  uv run armctrl console status \
    --session-artifact "$RUN_DIR/runtime_session.json" \
    --max-heartbeat-age-s 1.0 \
    --output "$RUN_DIR/console_status_${label}.json" \
    --json
}

preposition_to() {
  local name="$1"
  local q_target="$2"
  ensure_state
  uv run armctrl runtime preposition \
    --session-artifact "$RUN_DIR/runtime_session.json" \
    --q-target $q_target \
    --send-hz 50 \
    --max-joint-step-rad 0.01 \
    --max-heartbeat-age-s 1.0 \
    --output "$RUN_DIR/preposition_${name}.json" \
    --json
}

compile_candidate() {
  local name="$1"
  local candidate="$2"
  local expected_start="$3"
  ensure_state
  ensure_file "$candidate"
  uv run armctrl sysid compile-runtime \
    --execution-trajectory "$candidate" \
    --dof 6 \
    --sample-hz 100 \
    --expected-q-start $expected_start \
    --owner sysid \
    --start-pose-policy live_hold \
    --output "$RUN_DIR/compiled-fourier-${name}" \
    --json
}

run_candidate() {
  local name="$1"
  local candidate="$2"
  local expected_start="$3"
  local max_tracking="$4"
  ensure_state
  ensure_file "$candidate"
  local compiled_dir="$RUN_DIR/compiled-fourier-${name}"
  local compiled_command="$compiled_dir/compiled_motion_command.json"
  if [[ ! -f "$compiled_command" ]]; then
    echo "Missing compiled command: $compiled_command" >&2
    echo "Compiling ${name} now through armctrl sysid compile-runtime." >&2
    compile_candidate "$name" "$candidate" "$expected_start" >/dev/null
  fi
  uv run armctrl motion submit joint-trajectory \
    --session-artifact "$RUN_DIR/runtime_session.json" \
    --compiled-command "$compiled_command" \
    --max-heartbeat-age-s 1.0 \
    --heartbeat-timeout-s 3.0 \
    --max-tracking-error-rad "$max_tracking" \
    --output "$RUN_DIR/ident-fourier-${name}/runtime_submit.json" \
    --json
}

check_result() {
  local expected_samples="$1"
  local max_tracking="$2"
  ensure_state
  uv run armctrl motion result \
    --run-dir "$RUN_DIR" \
    --expect-owner sysid \
    --expect-mode trajectory_replay \
    --expect-sample-count "$expected_samples" \
    --max-jitter-p99-ms 5 \
    --max-tracking-error-rad "$max_tracking" \
    --json
}

cmd="${1:-help}"
case "$cmd" in
  init)
    run_dir="${2:-runs/lab-fourier-sysid-$(date +%Y%m%d-%H%M%S)}"
    write_state "$run_dir"
    ensure_state
    ensure_candidate_files
    ;;
  start)
    if [[ ! -f "$STATE_FILE" ]]; then
      write_state "runs/lab-fourier-sysid-$(date +%Y%m%d-%H%M%S)"
    fi
    ensure_state
    uv run armctrl runtime start \
      --backend arx5_sdk \
      --model X5 \
      --interface can0 \
      --safe-center $SAFE_CENTER \
      --send-hz 50 \
      --hold-hz 50 \
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
  preposition-slow|preposition-reduced)
    preposition_to "reduced_first" "$REDUCED_FIRST_Q"
    ;;
  compile-slow)
    compile_candidate "reduced-0p25-slow4x" "$SLOW_CANDIDATE" "$REDUCED_FIRST_Q"
    ;;
  run-slow)
    runtime_status "before_slow"
    run_candidate "reduced-0p25-slow4x" "$SLOW_CANDIDATE" "$REDUCED_FIRST_Q" "0.08"
    ;;
  check-slow)
    check_result "881" "0.08"
    ;;
  run-reduced)
    runtime_status "before_reduced"
    run_candidate "reduced-0p25-normal" "$REDUCED_CANDIDATE" "$REDUCED_FIRST_Q" "0.08"
    ;;
  compile-reduced)
    compile_candidate "reduced-0p25-normal" "$REDUCED_CANDIDATE" "$REDUCED_FIRST_Q"
    ;;
  check-reduced)
    check_result "196" "0.08"
    ;;
  preposition-full)
    preposition_to "full_first" "$FULL_FIRST_Q"
    ;;
  compile-full)
    compile_candidate "full" "$FULL_CANDIDATE" "$FULL_FIRST_Q"
    ;;
  run-full)
    runtime_status "before_full"
    run_candidate "full" "$FULL_CANDIDATE" "$FULL_FIRST_Q" "0.12"
    ;;
  check-full)
    check_result "196" "0.12"
    ;;
  back-safe)
    preposition_to "safe_center" "$SAFE_CENTER_DEFAULT"
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
