#!/usr/bin/env bash
set -euo pipefail

IFACE="${ARMCTRL_CAN_IFACE:-can0}"
BITRATE="${ARMCTRL_CAN_BITRATE:-1000000}"
SERIAL_DEVICE="${ARMCTRL_CAN_SERIAL:-}"
VERIFY_ONLY=0
FORCE=0
JSON=0
STRICT=0

usage() {
  cat <<'EOF'
Usage: scripts/can_bringup.sh [options]

Detect a USB serial CAN adapter, configure it as SocketCAN, and verify can0.

Options:
  --iface NAME          SocketCAN interface name, default can0
  --bitrate BPS         CAN bitrate, default 1000000
  --serial PATH         Serial device path. If omitted, auto-detect CANable/ttyACM/ttyUSB
  --verify-only         Only inspect and verify; do not change interface state
  --force               Recreate the interface even if it already exists
  --strict              Treat stale/multiple slcand ownership warnings as failure
  --json                Emit a compact JSON summary
  -h, --help            Show this help

Environment overrides:
  ARMCTRL_CAN_IFACE, ARMCTRL_CAN_BITRATE, ARMCTRL_CAN_SERIAL

Notes:
  - For CANable/SLCAN, 1000000 bps maps to slcand speed code -s8.
  - If can0 is already UP, the script verifies it and exits without restarting it
    unless --force is used.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface) IFACE="$2"; shift 2 ;;
    --bitrate) BITRATE="$2"; shift 2 ;;
    --serial) SERIAL_DEVICE="$2"; shift 2 ;;
    --verify-only) VERIFY_ONLY=1; shift ;;
    --force) FORCE=1; shift ;;
    --strict) STRICT=1; shift ;;
    --json) JSON=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

log() {
  if [[ "$JSON" != "1" ]]; then
    printf '[can_bringup] %s\n' "$*" >&2
  fi
}

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 2
  fi
}

sudo_cmd() {
  if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

slcan_speed_code() {
  case "$1" in
    10000) echo 0 ;;
    20000) echo 1 ;;
    50000) echo 2 ;;
    100000) echo 3 ;;
    125000) echo 4 ;;
    250000) echo 5 ;;
    500000) echo 6 ;;
    800000) echo 7 ;;
    1000000) echo 8 ;;
    *) echo "Unsupported SLCAN bitrate: $1" >&2; exit 2 ;;
  esac
}

detect_serial() {
  if [[ -n "$SERIAL_DEVICE" ]]; then
    printf '%s\n' "$SERIAL_DEVICE"
    return
  fi
  local candidates=()
  while IFS= read -r path; do candidates+=("$path"); done < <(
    find /dev/serial/by-id -maxdepth 1 -type l 2>/dev/null \
      | grep -Ei 'can|canable|candle|slcan|usb.*serial|openlight' \
      | sort
  )
  while IFS= read -r path; do candidates+=("$path"); done < <(
    ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | sort
  )
  if [[ "${#candidates[@]}" -eq 0 ]]; then
    echo "No CAN serial adapter found under /dev/serial/by-id, /dev/ttyACM*, or /dev/ttyUSB*" >&2
    exit 3
  fi
  printf '%s\n' "${candidates[0]}"
}

iface_exists() {
  ip link show "$IFACE" >/dev/null 2>&1
}

iface_is_up() {
  ip link show "$IFACE" 2>/dev/null | grep -q '<[^>]*UP'
}

verify_iface() {
  if ! iface_exists; then
    echo "SocketCAN interface $IFACE does not exist" >&2
    return 1
  fi
  ip -details link show "$IFACE"
}

slcand_pids_for_iface() {
  pgrep -af "slcand .*${IFACE}" 2>/dev/null || true
}

runtime_pids_for_iface() {
  pgrep -af "armctrl runtime start .*--interface ${IFACE}" 2>/dev/null || true
}

json_escape() {
  python3 -c 'import json,sys; print(json.dumps(sys.stdin.read().strip()))'
}

need_cmd ip
SERIAL_DEVICE="$(detect_serial)"

if [[ "$VERIFY_ONLY" == "1" ]]; then
  STATUS="ok"
  WARNINGS=()
  if ! verify_iface >/tmp/armctrl_can_bringup_verify.txt 2>&1; then
    STATUS="fail"
  fi
  SLCAND_TEXT="$(slcand_pids_for_iface)"
  RUNTIME_TEXT="$(runtime_pids_for_iface)"
  SLCAND_COUNT=0
  if [[ -n "$SLCAND_TEXT" ]]; then
    SLCAND_COUNT="$(printf '%s\n' "$SLCAND_TEXT" | sed '/^[[:space:]]*$/d' | wc -l | tr -d ' ')"
  fi
  RUNTIME_COUNT=0
  if [[ -n "$RUNTIME_TEXT" ]]; then
    RUNTIME_COUNT="$(printf '%s\n' "$RUNTIME_TEXT" | sed '/^[[:space:]]*$/d' | wc -l | tr -d ' ')"
  fi
  if [[ "$SLCAND_COUNT" -gt 1 ]]; then
    WARNINGS+=("multiple_slcand_processes_for_${IFACE}")
  fi
  if [[ "$RUNTIME_COUNT" -gt 0 ]]; then
    WARNINGS+=("armctrl_runtime_active_on_${IFACE}")
  fi
  if [[ "$STRICT" == "1" && "${#WARNINGS[@]}" -gt 0 && "$STATUS" == "ok" ]]; then
    STATUS="fail"
  elif [[ "${#WARNINGS[@]}" -gt 0 && "$STATUS" == "ok" ]]; then
    STATUS="warn"
  fi
  if [[ "$JSON" == "1" ]]; then
    VERIFY_TEXT="$(cat /tmp/armctrl_can_bringup_verify.txt 2>/dev/null || true)"
    WARNINGS_JSON="$(printf '%s\n' "${WARNINGS[@]:-}" | python3 -c 'import json,sys; print(json.dumps([line for line in sys.stdin.read().splitlines() if line]))')"
    printf '{"status":%s,"iface":%s,"serial":%s,"bitrate":%s,"verify":%s}\n' \
      "$(printf '%s' "$STATUS" | json_escape)" \
      "$(printf '%s' "$IFACE" | json_escape)" \
      "$(printf '%s' "$SERIAL_DEVICE" | json_escape)" \
      "$(printf '%s' "$BITRATE" | json_escape)" \
      "$(printf '%s' "$VERIFY_TEXT" | json_escape)" \
      | python3 -c 'import json,sys; data=json.load(sys.stdin); data["slcand_count"]=int(sys.argv[1]); data["runtime_count"]=int(sys.argv[2]); data["warnings"]=json.loads(sys.argv[3]); print(json.dumps(data,separators=(",",":")))' \
      "$SLCAND_COUNT" "$RUNTIME_COUNT" "$WARNINGS_JSON"
  else
    cat /tmp/armctrl_can_bringup_verify.txt 2>/dev/null || true
    if [[ "${#WARNINGS[@]}" -gt 0 ]]; then
      printf '\nWarnings:\n' >&2
      printf '  - %s\n' "${WARNINGS[@]}" >&2
      if [[ -n "$SLCAND_TEXT" ]]; then
        printf '\nslcand processes for %s:\n%s\n' "$IFACE" "$SLCAND_TEXT" >&2
      fi
      if [[ -n "$RUNTIME_TEXT" ]]; then
        printf '\narmctrl runtimes for %s:\n%s\n' "$IFACE" "$RUNTIME_TEXT" >&2
      fi
    fi
  fi
  [[ "$STATUS" == "ok" || "$STATUS" == "warn" ]]
  exit $?
fi

need_cmd slcand
need_cmd modprobe

if iface_exists && iface_is_up && [[ "$FORCE" != "1" ]]; then
  log "$IFACE already exists and is UP; verifying without restart."
  verify_iface
  exit 0
fi

SPEED_CODE="$(slcan_speed_code "$BITRATE")"
log "Using serial adapter: $SERIAL_DEVICE"
log "Configuring $IFACE at bitrate $BITRATE via slcand -s$SPEED_CODE"

sudo_cmd modprobe can
sudo_cmd modprobe can_raw
sudo_cmd modprobe slcan

if iface_exists; then
  sudo_cmd ip link set "$IFACE" down || true
fi

if [[ "$FORCE" == "1" ]]; then
  sudo_cmd pkill -f "slcand.*${IFACE}" || true
fi

if ! iface_exists || [[ "$FORCE" == "1" ]]; then
  sudo_cmd slcand -o -c -s"$SPEED_CODE" "$SERIAL_DEVICE" "$IFACE"
  sleep 0.5
fi

sudo_cmd ip link set "$IFACE" up txqueuelen 1000
verify_iface
log "CAN bringup complete."
