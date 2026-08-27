#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
SDK_ROOT="$REPO_ROOT/third_party/pyorbbecsdk-2-main"
BUILD_DIR="$REPO_ROOT/.runtime/build/pyorbbecsdk"
PYTHON_BIN="${1:-$REPO_ROOT/.envs/lerobot-v3/bin/python}"

if [[ ! -x "$PYTHON_BIN" ]]; then
  PYTHON_BIN="$REPO_ROOT/../miniconda3/envs/lerobot-v3/bin/python"
fi
if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "找不到 lerobot-v3 Python；可把解释器绝对路径作为第一个参数传入。" >&2
  exit 2
fi
if [[ ! -f "$SDK_ROOT/CMakeLists.txt" ]]; then
  echo "找不到 pyorbbecsdk 源码: $SDK_ROOT" >&2
  exit 2
fi

"$PYTHON_BIN" -m pip install "pybind11==2.12.0"
PYBIND11_CMAKE_DIR="$("$PYTHON_BIN" -m pybind11 --cmakedir)"

cmake -S "$SDK_ROOT" -B "$BUILD_DIR" \
  -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE="$PYTHON_BIN" \
  -Dpybind11_DIR="$PYBIND11_CMAKE_DIR"
cmake --build "$BUILD_DIR" --parallel
cmake --install "$BUILD_DIR"

# The upstream distribution metadata pulls GUI/demo dependencies and pins an
# incompatible PyAV. The project only needs the native extension built above.
SITE_PACKAGES="$("$PYTHON_BIN" -c 'import site; print(site.getsitepackages()[0])')"
PTH_FILE="$SITE_PACKAGES/lerobot_pyorbbecsdk.pth"
"$PYTHON_BIN" -c \
  'from pathlib import Path; import sys; Path(sys.argv[1]).write_text(sys.argv[2] + "\n", encoding="utf-8")' \
  "$PTH_FILE" "$SDK_ROOT/install/lib/pyorbbecsdk"

"$PYTHON_BIN" -c 'import pyorbbecsdk as ob; print("pyorbbecsdk ready:", ob.get_version())'
