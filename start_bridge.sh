#!/bin/bash
# 启动硬件代理。锁定带 pyserial 的解释器 —— 用错解释器会掉进 mock,
# 那样测试全是假的(2026-08-11 踩过)。
#
# 用法:
#   bash start_bridge.sh            # 真机
#   bash start_bridge.sh --mock     # 空跑
set -e

PY=/home/zhang123/miniconda3/envs/lerobot/bin/python
cd "$(dirname "$0")"

if [ ! -x "$PY" ]; then
    echo "✗ 找不到解释器: $PY"; exit 1
fi
"$PY" -c "import serial, fastapi, uvicorn" 2>/dev/null || {
    echo "✗ $PY 缺 serial/fastapi/uvicorn"; exit 1; }

if [ "$1" != "--mock" ]; then
    [ -e /dev/ttyUSB0 ] || {
        echo "✗ /dev/ttyUSB0 不存在 —— Windows 侧 usbipd 是否已转发?"; exit 1; }
    id -nG | grep -qw dialout || {
        echo "✗ 当前用户不在 dialout 组,打不开串口"; exit 1; }
fi

echo "解释器: $PY"
exec "$PY" bridge.py --host 0.0.0.0 --port 9000 "$@"
