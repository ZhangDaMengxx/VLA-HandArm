#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
SDK_ROOT="$REPO_ROOT/third_party/OrbbecSDK"
SOURCE="$SCRIPT_DIR/orbbec_60fps_probe.cpp"
BINARY="${TMPDIR:-/tmp}/lerobot-orbbec-60fps-probe"
BACKEND="${1:-v4l2}"
DURATION="${2:-12}"

case "$BACKEND" in
  auto|v4l2|libuvc) ;;
  *)
    echo "用法: $0 [auto|v4l2|libuvc] [duration_seconds]" >&2
    exit 2
    ;;
esac

if [[ ! -f "$SDK_ROOT/lib/libOrbbecSDK.so.2.9.3" ]]; then
  echo "未找到 OrbbecSDK 2.9.3: $SDK_ROOT" >&2
  exit 2
fi

if [[ "$BACKEND" == "v4l2" ]]; then
  expected_nodes=0
  if [[ -d /sys/class/video4linux ]]; then
    for sys_node in /sys/class/video4linux/video*; do
      [[ -e "$sys_node" ]] && ((expected_nodes += 1))
    done
  fi
  for _ in {1..20}; do
    actual_nodes=0
    for dev_node in /dev/video*; do
      [[ -e "$dev_node" ]] && ((actual_nodes += 1))
    done
    [[ "$expected_nodes" -gt 0 && "$actual_nodes" -eq "$expected_nodes" ]] && break
    sleep 0.25
  done
  if [[ "$expected_nodes" -eq 0 || "$actual_nodes" -ne "$expected_nodes" ]]; then
    echo "V4L2 节点不完整: sysfs=$expected_nodes, /dev=$actual_nodes。" >&2
    echo "停止相机进程并重新执行 usbipd detach/attach，待 /dev/video* 全部出现后重试。" >&2
    exit 2
  fi
fi

if [[ ! -x "$BINARY" || "$SOURCE" -nt "$BINARY" ]]; then
  g++ -std=c++11 -pthread "$SOURCE" \
    -I"$SDK_ROOT/include" \
    -L"$SDK_ROOT/lib" \
    -Wl,-rpath,"$SDK_ROOT/lib" \
    -lOrbbecSDK \
    -o "$BINARY"
fi

exec "$BINARY" "$BACKEND" "$DURATION"
