#!/usr/bin/env bash
# Source this file to prepare the ROS 2 NERO/Inspire hardware runtime.

if [[ -z "${BASH_VERSION:-}" ]]; then
  echo "nero_hardware_env.sh requires bash" >&2
  return 1 2>/dev/null || exit 1
fi

_nero_env_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export NERO_PROJECT_ROOT="$(cd -- "${_nero_env_dir}/.." && pwd)"
export NERO_ROS2_WS="$(cd -- "${NERO_PROJECT_ROOT}/.." && pwd)"

_nero_prepend_path() {
  local entry="$1"
  local current="${2:-}"
  case ":${current}:" in
    *":${entry}:"*) printf '%s' "${current}" ;;
    *)
      if [[ -n "${current}" ]]; then
        printf '%s:%s' "${entry}" "${current}"
      else
        printf '%s' "${entry}"
      fi
      ;;
  esac
}

_nero_ros_setup="/opt/ros/humble/setup.bash"
_nero_ws_setup="${NERO_ROS2_WS}/install/setup.bash"
if [[ ! -r "${_nero_ros_setup}" ]]; then
  echo "ROS Humble setup not found: ${_nero_ros_setup}" >&2
  return 1 2>/dev/null || exit 1
fi
if [[ ! -r "${_nero_ws_setup}" ]]; then
  echo "ROS workspace setup not found: ${_nero_ws_setup}" >&2
  echo "Build the workspace before loading this environment." >&2
  return 1 2>/dev/null || exit 1
fi

# ROS-generated setup files read optional variables before assigning defaults, so
# they cannot be sourced while a caller has Bash nounset enabled.
_nero_restore_nounset=0
_nero_setup_status=0
if [[ $- == *u* ]]; then
  _nero_restore_nounset=1
  set +u
fi
# shellcheck disable=SC1091
source "${_nero_ros_setup}" || _nero_setup_status=$?
if ((_nero_setup_status == 0)); then
  # shellcheck disable=SC1090
  source "${_nero_ws_setup}" || _nero_setup_status=$?
fi
((_nero_restore_nounset == 0)) || set -u
if ((_nero_setup_status != 0)); then
  echo "Failed to load the ROS environment" >&2
  return "${_nero_setup_status}" 2>/dev/null || exit "${_nero_setup_status}"
fi

export PYAGXARM_ROOT="${NERO_PROJECT_ROOT}/third_party/pyAgxArm/pyAgxArm-master"
export NERO_ROS_PYTHON_SITE="${NERO_ROS_PYTHON_SITE:-${HOME}/miniconda3/envs/ros-humble/lib/python3.10/site-packages}"
if [[ ! -d "${PYAGXARM_ROOT}/pyAgxArm" ]]; then
  echo "pyAgxArm source not found: ${PYAGXARM_ROOT}" >&2
  return 1 2>/dev/null || exit 1
fi
if [[ ! -d "${NERO_ROS_PYTHON_SITE}" ]]; then
  echo "ROS Python dependency directory not found: ${NERO_ROS_PYTHON_SITE}" >&2
  return 1 2>/dev/null || exit 1
fi

PYTHONPATH="$(_nero_prepend_path "${NERO_ROS_PYTHON_SITE}" "${PYTHONPATH:-}")"
PYTHONPATH="$(_nero_prepend_path "${PYAGXARM_ROOT}" "${PYTHONPATH}")"
export PYTHONPATH

export NERO_CAN_CHANNEL="${NERO_CAN_CHANNEL:-can0}"
export NERO_FIRMWARE="${NERO_FIRMWARE:-auto}"
export NERO_HAND_PORT="${NERO_HAND_PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"

nero_hardware_monitor() {
  ros2 launch nero_inspire_hardware hardware.launch.py \
    arm_mock:=false \
    hand_mock:=false \
    enable_control:=false \
    can_channel:="${NERO_CAN_CHANNEL}" \
    firmware:="${NERO_FIRMWARE}" \
    hand_port:="${NERO_HAND_PORT}" \
    "$@"
}

nero_hardware_control() {
  ros2 launch nero_inspire_hardware hardware.launch.py \
    arm_mock:=false \
    hand_mock:=false \
    enable_control:=true \
    can_channel:="${NERO_CAN_CHANNEL}" \
    firmware:="${NERO_FIRMWARE}" \
    hand_port:="${NERO_HAND_PORT}" \
    "$@"
}

nero_arm_enable() {
  ros2 service call /nero/arm/set_enabled std_srvs/srv/SetBool '{data: true}'
}

nero_arm_disable() {
  ros2 service call /nero/arm/set_enabled std_srvs/srv/SetBool '{data: false}'
}

unset _nero_env_dir _nero_ros_setup _nero_ws_setup
unset _nero_restore_nounset _nero_setup_status
unset -f _nero_prepend_path

echo "NERO hardware environment loaded."
echo "  monitor: nero_hardware_monitor"
echo "  control: nero_hardware_control (arm remains disabled until nero_arm_enable)"
