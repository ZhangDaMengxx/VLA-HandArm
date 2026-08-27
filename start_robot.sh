#!/usr/bin/env bash
# Interactive launcher for the NERO + Inspire Web, Bridge, and ROS2 stack.
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="$(cd -- "${ROOT_DIR}/.." && pwd)"
BRIDGE_DIR="${ROS2_WS}/robot-mcp-server/robot-bridge"
MCP_DIR="${ROS2_WS}/robot-mcp-server/mcp_server"
CONFIG_FILE="${NERO_RUNTIME_CONFIG:-${ROOT_DIR}/.nero_runtime.env}"

MODE=""
COMPONENTS=""
ENABLE_ARM="ask"
DRY_RUN=0
INTERACTIVE=1

PIDS=()
NAMES=()
LOGS=()
LAST_PID=""
LAST_LOG=""
DRIVER_MANAGED_PID=""
ARM_ENABLED_BY_LAUNCHER=0
CLEANED=0

usage() {
  cat <<'EOF'
Usage:
  ./start_robot.sh
  ./start_robot.sh --mode ros --components both
  ./start_robot.sh --mode direct --components web
  ./start_robot.sh --mode direct --components both
  ./start_robot.sh --mode mock --components all

Options:
  --mode MODE          ros | direct | mock (also accepts 1 | 2 | 3)
  --components SET     web | bridge | both | all
                       all = Web + Bridge + local MCP Server
  --enable-arm VALUE   ask | yes | no (ROS mode only; default: ask)
  --config PATH        Runtime environment file
  --dry-run            Print the selected plan without starting processes
  -h, --help           Show this help

Without arguments, the launcher uses numeric menus. Ctrl+C stops every process
started by this launcher.
EOF
}

info() { printf '[INFO] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*" >&2; }
die() { printf '[ERROR] %s\n' "$*" >&2; exit 1; }

normalize_mode() {
  case "${1,,}" in
    1|ros|ros2) printf 'ros' ;;
    2|direct) printf 'direct' ;;
    3|mock) printf 'mock' ;;
    *) return 1 ;;
  esac
}

normalize_components() {
  case "${1,,}" in
    1|web) printf 'web' ;;
    2|bridge) printf 'bridge' ;;
    3|both) printf 'both' ;;
    4|all) printf 'all' ;;
    *) return 1 ;;
  esac
}

while (($#)); do
  case "$1" in
    --mode)
      (($# >= 2)) || die "--mode requires a value"
      MODE="$(normalize_mode "$2")" || die "Unknown mode: $2"
      shift 2
      ;;
    --components)
      (($# >= 2)) || die "--components requires a value"
      COMPONENTS="$(normalize_components "$2")" || die "Unknown components: $2"
      shift 2
      ;;
    --enable-arm)
      (($# >= 2)) || die "--enable-arm requires ask, yes, or no"
      ENABLE_ARM="${2,,}"
      [[ "$ENABLE_ARM" =~ ^(ask|yes|no)$ ]] || die "--enable-arm requires ask, yes, or no"
      shift 2
      ;;
    --config)
      (($# >= 2)) || die "--config requires a path"
      CONFIG_FILE="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

if [[ -n "$MODE" && -n "$COMPONENTS" ]]; then
  INTERACTIVE=0
fi

if [[ -r "$CONFIG_FILE" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$CONFIG_FILE"
  set +a
fi

# Never leak the caller's Conda/ROS PYTHONPATH across interpreter boundaries.
BASE_PYTHONPATH="${NERO_EXTRA_PYTHONPATH:-}"

WEB_PORT="${WEB_PORT:-7860}"
BRIDGE_PORT="${BRIDGE_PORT:-9000}"
MCP_PORT="${MCP_PORT:-8000}"
WEB_HOST="${WEB_HOST:-0.0.0.0}"
BRIDGE_HOST="${BRIDGE_HOST:-127.0.0.1}"
MCP_HOST="${MCP_HOST:-127.0.0.1}"
WEB_PYTHON="${WEB_PYTHON:-${HOME}/miniconda3/envs/lerobot-v3/bin/python}"
ROS_PYTHON="${ROS_PYTHON:-${HOME}/miniconda3/envs/ros-humble/bin/python}"
DIRECT_PYTHON="${DIRECT_PYTHON:-${HOME}/miniconda3/envs/lerobot/bin/python}"
MCP_PYTHON="${MCP_PYTHON:-${DIRECT_PYTHON}}"
ROS_ENV_SCRIPT="${ROS_ENV_SCRIPT:-${ROOT_DIR}/deploy/nero_hardware_env.sh}"
PYAGXARM_ROOT="${PYAGXARM_ROOT:-${ROOT_DIR}/third_party/pyAgxArm/pyAgxArm-master}"
NERO_HAND_PORT="${NERO_HAND_PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"
NERO_CAN_CHANNEL="${NERO_CAN_CHANNEL:-can0}"
NERO_FIRMWARE="${NERO_FIRMWARE:-auto}"
BRIDGE_TOKEN="${BRIDGE_TOKEN:-}"
DRIVER_READY_TIMEOUT="${DRIVER_READY_TIMEOUT:-45}"
RUNTIME_DIR="${RUNTIME_DIR:-${ROOT_DIR}/.runtime/launcher/$(date +%Y%m%d_%H%M%S)}"

choose_mode() {
  printf '\n请选择运行模式：\n\n'
  printf '  1. ROS2（推荐）\n'
  printf '  2. Direct\n'
  printf '  3. Mock\n\n'
  local choice
  read -r -p '请输入 [1-3，默认 1]: ' choice
  choice="${choice:-1}"
  MODE="$(normalize_mode "$choice")" || die "请输入 1、2 或 3"
}

choose_components() {
  local choice
  if [[ "$MODE" == "direct" ]]; then
    printf '\n请选择 Direct 启动范围：\n\n'
    printf '  1. Web Direct\n'
    printf '  2. Bridge Direct\n'
    printf '  3. Web + Bridge Direct\n\n'
    read -r -p '请输入 [1-3，默认 1]: ' choice
    choice="${choice:-1}"
    case "$choice" in
      1|web) COMPONENTS="web" ;;
      2|bridge) COMPONENTS="bridge" ;;
      3|both) COMPONENTS="both" ;;
      *) die "Direct 模式请输入 1、2 或 3" ;;
    esac
    return
  fi

  printf '\n请选择启动范围：\n\n'
  printf '  1. 仅 Web\n'
  printf '  2. 仅 Bridge\n'
  printf '  3. Web + Bridge（默认）\n'
  printf '  4. Web + Bridge + 本地 MCP Server\n\n'
  read -r -p '请输入 [1-4，默认 3]: ' choice
  choice="${choice:-3}"
  COMPONENTS="$(normalize_components "$choice")" || die "请输入 1、2、3 或 4"
}

[[ -n "$MODE" ]] || choose_mode
[[ -n "$COMPONENTS" ]] || choose_components

wants_web() { [[ "$COMPONENTS" =~ ^(web|both|all)$ ]]; }
wants_bridge() { [[ "$COMPONENTS" =~ ^(bridge|both|all)$ ]]; }
wants_mcp() { [[ "$COMPONENTS" == "all" ]]; }

printf '\n启动计划\n'
printf '  mode:       %s\n' "$MODE"
printf '  components: %s\n' "$COMPONENTS"
printf '  config:     %s%s\n' "$CONFIG_FILE" "$([[ -r "$CONFIG_FILE" ]] && printf ' (loaded)' || true)"
printf '  runtime:    %s\n' "$RUNTIME_DIR"

if ((DRY_RUN)); then
  info "Dry run complete; no process was started."
  exit 0
fi

mkdir -p "$RUNTIME_DIR"

require_file() { [[ -r "$1" ]] || die "Required file not found: $1"; }
require_executable() { [[ -x "$1" ]] || die "Python executable not found: $1"; }

check_web_runtime() {
  local version=""
  if ! version="$(env PYTHONPATH="$BASE_PYTHONPATH" "$WEB_PYTHON" -c \
      'import lerobot; print(lerobot.__version__)' 2>/dev/null)"; then
    die "Web Python cannot import lerobot: ${WEB_PYTHON}"
  fi
  [[ "$version" == "0.6.1" ]] ||
    die "Web Python requires lerobot 0.6.1, got ${version:-unknown}: ${WEB_PYTHON}"
}

port_in_use() {
  local port="$1"
  (exec 3<>"/dev/tcp/127.0.0.1/${port}") >/dev/null 2>&1
}

check_port() {
  local name="$1" port="$2" setting="$3"
  if port_in_use "$port"; then
    die "${name} port ${port} is already in use. Stop the existing service or change ${setting} in .nero_runtime.env"
  fi
}

start_process() {
  local name="$1" logfile="$2"
  shift 2
  info "Starting ${name}; log: ${logfile}"
  setsid "$@" >>"$logfile" 2>&1 &
  LAST_PID=$!
  LAST_LOG="$logfile"
  PIDS+=("$LAST_PID")
  NAMES+=("$name")
  LOGS+=("$logfile")
  sleep 0.4
  if ! kill -0 "$LAST_PID" 2>/dev/null; then
    tail -n 30 "$logfile" >&2 || true
    die "${name} exited during startup"
  fi
}

check_last_process() {
  local name="$1"
  if ! kill -0 "$LAST_PID" 2>/dev/null; then
    tail -n 30 "$LAST_LOG" >&2 || true
    die "${name} exited before becoming ready"
  fi
}

cleanup() {
  ((CLEANED == 0)) || return
  CLEANED=1
  if ((ARM_ENABLED_BY_LAUNCHER)); then
    info "Disabling the arm enabled by this launcher..."
    local disable_output=""
    if disable_output="$(timeout 8 ros2 service call /nero/arm/set_enabled \
        std_srvs/srv/SetBool '{data: false}' 2>&1)" &&
       [[ "$disable_output" == *"success=True"* ]]; then
      info "Arm disabled."
    else
      warn "Could not confirm arm disable; use the emergency stop and verify hardware state."
    fi
    ARM_ENABLED_BY_LAUNCHER=0
  fi
  ((${#PIDS[@]})) || return
  info "Stopping processes started by this launcher..."
  local i pid
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    pid="${PIDS[$i]}"
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  done
  for _ in {1..30}; do
    local alive=0
    for pid in "${PIDS[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive=1
    done
    ((alive == 0)) && break
    sleep 0.1
  done
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
}

trap cleanup EXIT
trap 'exit 130' INT TERM

resolve_hand_port() {
  if [[ -e "$NERO_HAND_PORT" ]]; then
    return
  fi
  local candidate
  shopt -s nullglob
  for candidate in /dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM*; do
    if [[ -e "$candidate" ]]; then
      warn "Configured hand port is missing; using ${candidate}"
      NERO_HAND_PORT="$candidate"
      shopt -u nullglob
      return
    fi
  done
  shopt -u nullglob
  die "No hand serial device found. Attach it from Windows with usbipd, then rerun."
}

check_real_hardware() {
  resolve_hand_port
  [[ -r "$NERO_HAND_PORT" && -w "$NERO_HAND_PORT" ]] ||
    die "No read/write permission for ${NERO_HAND_PORT}; check the dialout group"
  if ! ip link show "$NERO_CAN_CHANNEL" >/dev/null 2>&1; then
    die "CAN interface ${NERO_CAN_CHANNEL} does not exist"
  fi
  if ! ip link show "$NERO_CAN_CHANNEL" | head -n 1 | grep -q '<[^>]*UP'; then
    if ((INTERACTIVE)); then
      read -r -p "${NERO_CAN_CHANNEL} is down. Bring it up at 1 Mbps with sudo? [y/N]: " answer
      if [[ "${answer,,}" == "y" || "${answer,,}" == "yes" ]]; then
        sudo ip link set "$NERO_CAN_CHANNEL" down || true
        sudo ip link set "$NERO_CAN_CHANNEL" up type can bitrate 1000000
      else
        die "CAN interface is down"
      fi
    else
      die "CAN interface ${NERO_CAN_CHANNEL} is down"
    fi
  fi
  export NERO_HAND_PORT NERO_CAN_CHANNEL NERO_FIRMWARE
  export INSPIRE_HAND_PORT="$NERO_HAND_PORT"
}

check_serial_not_busy() {
  if command -v fuser >/dev/null 2>&1 &&
     fuser "$NERO_HAND_PORT" >/dev/null 2>&1; then
    fuser -v "$NERO_HAND_PORT" >&2 || true
    die "Hand serial port is already in use: ${NERO_HAND_PORT}"
  fi
}

load_ros_environment() {
  require_file "$ROS_ENV_SCRIPT"
  PYTHONPATH="$BASE_PYTHONPATH"
  export PYTHONPATH
  # shellcheck disable=SC1090
  source "$ROS_ENV_SCRIPT"
}

driver_is_running() {
  pgrep -f \
    '[r]os2 launch nero_inspire_hardware hardware\.launch\.py|nero_inspire_hardware/[h]ardware_driver|/[h]ardware_driver([[:space:]]|$)' \
    >/dev/null 2>&1
}

driver_state_ready() {
  local output
  output="$(timeout 3 ros2 topic echo /nero/driver_state --field data --once \
    --no-daemon 2>/dev/null || true)"
  [[ -n "$output" ]] || return 1
  NERO_DRIVER_STATE_OUTPUT="$output" /usr/bin/python3 -c '
import json, os
import yaml
raw = os.environ["NERO_DRIVER_STATE_OUTPUT"].split("\n---", 1)[0]
value = yaml.safe_load(raw)
state = json.loads(value) if isinstance(value, str) else value
raise SystemExit(0 if state.get("ready") and state.get("accepting_commands") else 1)
' >/dev/null 2>&1
}

wait_for_driver() {
  info "Waiting for arm + hand READY (${DRIVER_READY_TIMEOUT}s timeout)..."
  local elapsed
  for ((elapsed=0; elapsed<DRIVER_READY_TIMEOUT; elapsed++)); do
    if [[ -n "$DRIVER_MANAGED_PID" ]] && ! kill -0 "$DRIVER_MANAGED_PID" 2>/dev/null; then
      tail -n 30 "${RUNTIME_DIR}/hardware.log" >&2 || true
      die "Hardware Driver exited before becoming READY"
    fi
    driver_state_ready && { info "Hardware Driver is READY and accepting commands."; return; }
    sleep 1
  done
  [[ -r "${RUNTIME_DIR}/hardware.log" ]] && tail -n 30 "${RUNTIME_DIR}/hardware.log" >&2
  die "Hardware Driver did not become READY"
}

ensure_arm_disabled() {
  info "Setting arm to the disabled startup state..."
  local disable_output=""
  if ! disable_output="$(timeout 8 ros2 service call /nero/arm/set_enabled \
      std_srvs/srv/SetBool '{data: false}' 2>&1)"; then
    printf '%s\n' "$disable_output" >&2
    die "Arm disable service call failed"
  fi
  printf '%s\n' "$disable_output"
  [[ "$disable_output" == *"success=True"* ]] ||
    die "Could not confirm the arm is disabled"
  info "Arm is disabled."
}

maybe_enable_arm() {
  local decision="$ENABLE_ARM"
  if [[ "$decision" == "ask" ]]; then
    if ((INTERACTIVE)); then
      printf '\n机械臂使能会允许后续运动。确认净空、低速且急停可用。\n'
      read -r -p '现在使能机械臂？[y/N]: ' answer
      [[ "${answer,,}" == "y" || "${answer,,}" == "yes" ]] && decision=yes || decision=no
    else
      decision=no
    fi
  fi
  if [[ "$decision" == "yes" ]]; then
    local enable_output=""
    if ! enable_output="$(ros2 service call /nero/arm/set_enabled \
        std_srvs/srv/SetBool '{data: true}' 2>&1)"; then
      printf '%s\n' "$enable_output" >&2
      die "Arm enable service call failed"
    fi
    printf '%s\n' "$enable_output"
    [[ "$enable_output" == *"success=True"* ]] || die "Hardware Driver rejected arm enable"
    ARM_ENABLED_BY_LAUNCHER=1
  else
    info "Arm remains disabled; enable later with nero_arm_enable."
  fi
}

wait_for_http() {
  local name="$1" url="$2" insecure="${3:-no}"
  local args=(-fsS)
  [[ "$insecure" == "yes" ]] && args=(-kfsS)
  for _ in {1..30}; do
    check_last_process "$name"
    curl "${args[@]}" "$url" >/dev/null 2>&1 && { info "${name} is ready: ${url}"; return; }
    sleep 0.5
  done
  die "${name} did not become ready: ${url}"
}

wait_for_web() {
  local https_url="https://127.0.0.1:${WEB_PORT}/"
  local http_url="http://127.0.0.1:${WEB_PORT}/"
  for _ in {1..30}; do
    check_last_process "Web"
    if curl -kfsS "$https_url" >/dev/null 2>&1; then
      WEB_URL="$https_url"
      info "Web is ready: ${WEB_URL}"
      return
    fi
    if curl -fsS "$http_url" >/dev/null 2>&1; then
      WEB_URL="$http_url"
      info "Web is ready: ${WEB_URL}"
      return
    fi
    sleep 0.5
  done
  die "Web did not become ready on port ${WEB_PORT}"
}

# Finish service preflight before opening hardware or offering arm enable.
if wants_web; then
  require_executable "$WEB_PYTHON"
  check_web_runtime
  check_port "Web" "$WEB_PORT" "WEB_PORT"
fi
if wants_bridge; then
  require_file "${BRIDGE_DIR}/bridge.py"
  case "$MODE" in
    ros) require_executable "$ROS_PYTHON" ;;
    direct|mock) require_executable "$DIRECT_PYTHON" ;;
  esac
  check_port "Bridge" "$BRIDGE_PORT" "BRIDGE_PORT"
fi
if wants_mcp; then
  require_file "${MCP_DIR}/app/main.py"
  require_executable "$MCP_PYTHON"
  check_port "MCP Server" "$MCP_PORT" "MCP_PORT"
fi

if [[ "$MODE" == "ros" ]]; then
  check_real_hardware
  load_ros_environment
  require_executable "$ROS_PYTHON"
  if driver_is_running; then
    warn "An existing nero_hardware_driver will be reused and not stopped on exit."
  else
    start_process "Hardware Driver" "${RUNTIME_DIR}/hardware.log" \
      ros2 launch nero_inspire_hardware hardware.launch.py \
      arm_mock:=false hand_mock:=false enable_control:=true \
      can_channel:="$NERO_CAN_CHANNEL" firmware:="$NERO_FIRMWARE" \
      hand_port:="$NERO_HAND_PORT"
    DRIVER_MANAGED_PID="$LAST_PID"
  fi
  wait_for_driver
  ensure_arm_disabled
elif [[ "$MODE" == "direct" ]]; then
  driver_is_running && die "Hardware Driver is running; stop it before Direct mode"
  check_real_hardware
  check_serial_not_busy
  [[ -d "${PYAGXARM_ROOT}/pyAgxArm" ]] || die "pyAgxArm not found: ${PYAGXARM_ROOT}"
fi

if wants_web; then
  require_executable "$WEB_PYTHON"
  check_port "Web" "$WEB_PORT" "WEB_PORT"
  web_pythonpath="$BASE_PYTHONPATH"
  [[ "$MODE" == "direct" ]] && web_pythonpath="${PYAGXARM_ROOT}${web_pythonpath:+:${web_pythonpath}}"
  start_process "Web" "${RUNTIME_DIR}/web.log" \
    env WEB_HARDWARE_BACKEND="$MODE" WEB_PORT="$WEB_PORT" WEB_HOST="$WEB_HOST" \
    WEB_HARDWARE_PYTHON="$WEB_PYTHON" NERO_HAND_PORT="$NERO_HAND_PORT" \
    NERO_CAN_CHANNEL="$NERO_CAN_CHANNEL" NERO_FIRMWARE="$NERO_FIRMWARE" \
    PYAGXARM_ROOT="$PYAGXARM_ROOT" PYTHONPATH="$web_pythonpath" \
    "$WEB_PYTHON" "${ROOT_DIR}/src/lerobot_v3/app_web.py"
  wait_for_web
fi

if wants_bridge; then
  require_file "${BRIDGE_DIR}/bridge.py"
  check_port "Bridge" "$BRIDGE_PORT" "BRIDGE_PORT"
  if [[ -z "$BRIDGE_TOKEN" ]]; then
    warn "BRIDGE_TOKEN is empty. Bridge is safe only while bound to 127.0.0.1."
  fi
  if [[ "$MODE" == "ros" ]]; then
    bridge_python="$ROS_PYTHON"
    bridge_args=(--backend ros --host "$BRIDGE_HOST" --port "$BRIDGE_PORT")
    bridge_pythonpath="$PYTHONPATH"
  elif [[ "$MODE" == "direct" ]]; then
    bridge_python="$DIRECT_PYTHON"
    bridge_args=(--backend direct --host "$BRIDGE_HOST" --port "$BRIDGE_PORT"
                 --hand-port "$NERO_HAND_PORT")
    bridge_pythonpath="${PYAGXARM_ROOT}${BASE_PYTHONPATH:+:${BASE_PYTHONPATH}}"
  else
    bridge_python="$DIRECT_PYTHON"
    bridge_args=(--backend direct --mock --host "$BRIDGE_HOST" --port "$BRIDGE_PORT")
    bridge_pythonpath="$BASE_PYTHONPATH"
  fi
  require_executable "$bridge_python"
  start_process "Robot Bridge" "${RUNTIME_DIR}/bridge.log" \
    env BRIDGE_TOKEN="$BRIDGE_TOKEN" INSPIRE_HAND_PORT="$NERO_HAND_PORT" \
    PYAGXARM_ROOT="$PYAGXARM_ROOT" PYTHONPATH="$bridge_pythonpath" \
    "$bridge_python" "${BRIDGE_DIR}/bridge.py" "${bridge_args[@]}"
  wait_for_http "Robot Bridge" "http://127.0.0.1:${BRIDGE_PORT}/health"
fi

if wants_mcp; then
  wants_bridge || die "Local MCP Server requires Bridge"
  require_file "${MCP_DIR}/app/main.py"
  require_executable "$MCP_PYTHON"
  check_port "MCP Server" "$MCP_PORT" "MCP_PORT"
  start_process "MCP Server" "${RUNTIME_DIR}/mcp.log" \
    env ROBOT_BRIDGE_URL="http://127.0.0.1:${BRIDGE_PORT}" \
    ROBOT_BRIDGE_TOKEN="$BRIDGE_TOKEN" MCP_SECURITY_MODE=lan \
    PYTHONPATH="$BASE_PYTHONPATH" \
    "$MCP_PYTHON" -m uvicorn app.main:app --app-dir "$MCP_DIR" \
    --host "$MCP_HOST" --port "$MCP_PORT"
  wait_for_http "MCP Server" "http://127.0.0.1:${MCP_PORT}/health"
fi

if [[ "$MODE" == "ros" ]]; then
  maybe_enable_arm
fi

printf '\nStartup complete.\n'
wants_web && printf '  Web:    %s\n' "$WEB_URL"
wants_bridge && printf '  Bridge: http://127.0.0.1:%s/health\n' "$BRIDGE_PORT"
wants_mcp && printf '  MCP:    http://127.0.0.1:%s/mcp\n' "$MCP_PORT"
printf '  Logs:   %s\n' "$RUNTIME_DIR"
printf '\nPress Ctrl+C to stop this stack.\n'
[[ "$MODE" == "ros" && "$COMPONENTS" =~ ^(both|all)$ ]] &&
  warn "Web and MCP are both online; do not send motion commands from both at once."
[[ "$MODE" == "direct" && "$COMPONENTS" == "both" ]] &&
  warn "Direct Web and Bridge are both online and may contend for CAN/serial when Web connects."

wait -n "${PIDS[@]}" || status=$?
status="${status:-0}"
warn "A managed process exited (status ${status}); stopping the remaining stack."
exit "$status"
