#!/usr/bin/env bash
# deploy_robot_host.sh — 在「连着机械臂的那台主机」上一键部署。
#
# 为什么脚本跑在真机那台:app_web.py 是编排器,用 subprocess 拉起本机的
# writer/reader/runner,再由 bridge 经 CAN/RS485 驱动硬件 —— 整条链锁在同一台机器。
# 开发机只要浏览器开 http://<本机IP>:7860 就行,运行时不参与任何数据通路。
#
# 用法(在目标主机上):
#   git clone git@github.com:ZhangDaMengxx/VLA-HandArm.git ~/ros2_ws/lerobotTest
#   cd ~/ros2_ws/lerobotTest && bash deploy/deploy_robot_host.sh --all
#
# 分步(推荐,出错好定位):
#   bash deploy/deploy_robot_host.sh --check     # 只体检,不改系统
#   bash deploy/deploy_robot_host.sh --env       # 建 conda + venv(不需 sudo)
#   bash deploy/deploy_robot_host.sh --ros       # colcon build(不需 sudo)
#   bash deploy/deploy_robot_host.sh --hw        # CAN/udev/dialout(**需 sudo**)
#   bash deploy/deploy_robot_host.sh --verify    # mock 链路 + 安全闸自检
#
# 幂等:重复跑不会重复装。只有 --hw 会动系统配置,且每步都先打印再执行。
set -euo pipefail

# ---- 路径:全部从脚本位置推出,不写死用户名 ----
DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$DEPLOY_DIR/.." && pwd)"          # .../lerobotTest
WS="$(cd "$REPO/.." && pwd)"                  # .../ros2_ws
ROS_DISTRO_WANT="humble"
PY_WANT="3.10"                                # 必须与 ROS Humble 的 python 一致
CONDA_ENV_NAME="lerobot"
GRADIO_VENV="$HOME/gradio_venv"
WEB_PORT="${WEB_PORT:-7860}"

# 手的 RS485 串口:udev 会做成固定别名 /dev/inspire_hand,避免插拔后 ttyUSB 号对调
HAND_SYMLINK="inspire_hand"
CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"

C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_DIM=$'\033[2m'; C_END=$'\033[0m'
ok()   { echo "${C_OK}  ✔${C_END} $*"; }
warn() { echo "${C_WARN}  !${C_END} $*"; }
err()  { echo "${C_ERR}  x${C_END} $*" >&2; }
step() { echo; echo "═══ $*"; }
run()  { echo "${C_DIM}    \$ $*${C_END}"; "$@"; }

FAILED=0

# 找 conda:非交互 shell 里 conda 常不在 PATH(它靠 .bashrc 的 init 块注入),
# 所以除了 PATH 还要翻常见安装位置。$WS/enter 是本项目开发机的装法。
find_conda() {
  if command -v conda >/dev/null 2>&1; then command -v conda; return 0; fi
  local c
  for c in "$WS/enter/bin/conda" "$HOME/miniconda3/bin/conda" \
           "$HOME/anaconda3/bin/conda" "$HOME/miniforge3/bin/conda" \
           "/opt/conda/bin/conda"; do
    [[ -x "$c" ]] && { echo "$c"; return 0; }
  done
  return 1
}

# ═════════════════════════════════════════════════════════ 体检(只读,不改系统)
do_check() {
  step "体检:确认前提,不改任何东西"

  # --- OS / ROS ---
  . /etc/os-release 2>/dev/null || true
  echo "  OS: ${PRETTY_NAME:-未知}"
  [[ "${VERSION_ID:-}" == "22.04" ]] || warn "非 Ubuntu 22.04;ROS Humble 官方只支持 22.04"

  if [[ -f "/opt/ros/$ROS_DISTRO_WANT/setup.bash" ]]; then
    ok "ROS $ROS_DISTRO_WANT 已装"
  else
    err "没找到 /opt/ros/$ROS_DISTRO_WANT — 先装 ROS2 Humble(desktop)"
    echo "      https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
    FAILED=1
  fi

  # rclpy 必须在系统 python3 里(bridge/writer/reader/runner 都靠它)
  if bash -lc "source /opt/ros/$ROS_DISTRO_WANT/setup.bash 2>/dev/null && python3 -c 'import rclpy'" 2>/dev/null; then
    ok "系统 python3 能 import rclpy"
  else
    err "系统 python3 import rclpy 失败"; FAILED=1
  fi

  # --- 工具链 ---
  for c in git colcon rosdep; do
    if command -v "$c" >/dev/null 2>&1; then ok "$c: $(command -v $c)"
    else
      case $c in
        colcon|rosdep) err "$c 未装: sudo apt install python3-colcon-common-extensions python3-rosdep"; FAILED=1;;
        *) err "$c 未装"; FAILED=1;;
      esac
    fi
  done

  if find_conda >/dev/null; then ok "conda: $(find_conda)"
  else warn "conda 未装(或不在常见位置)— --env 会提示你装 Miniconda"; fi

  # --- 系统 py3 的第三方包(runner/backend 要 numpy+yaml,手要 pyserial)---
  for m in numpy yaml serial; do
    if python3 -c "import $m" 2>/dev/null; then ok "系统 python3: $m"
    else warn "系统 python3 缺 $m → sudo apt install python3-${m/yaml/yaml}"; fi
  done

  do_check_hw_readiness
  do_check_repo_assets
}

# ---- 硬件就绪度:CAN(臂) + RS485(手) ----
do_check_hw_readiness() {
  echo "  --- 硬件 ---"

  if ip link show "$CAN_IFACE" >/dev/null 2>&1; then
    if ip -d link show "$CAN_IFACE" | grep -q 'state UP'; then
      ok "CAN $CAN_IFACE 已 UP ($(ip -d link show $CAN_IFACE | grep -oP 'bitrate \K[0-9]+' || echo '?') bps)"
    else
      warn "CAN $CAN_IFACE 存在但 DOWN → --hw 会拉起来"
    fi
  else
    warn "没有 $CAN_IFACE — USB-CAN 没插,或驱动没加载(gs_usb / slcan / peak)"
  fi

  # 串口:优先看 udev 固定别名,没有再看裸 ttyUSB
  if [[ -e "/dev/$HAND_SYMLINK" ]]; then
    ok "手串口别名 /dev/$HAND_SYMLINK → $(readlink -f /dev/$HAND_SYMLINK)"
  elif compgen -G "/dev/ttyUSB*" >/dev/null; then
    warn "有 $(ls /dev/ttyUSB* | tr '\n' ' ')但没固定别名 — 插拔顺序一变编号就对调,建议跑 --hw"
  else
    warn "没有 /dev/ttyUSB* — 手的 RS485 转换器没插?"
  fi

  # dialout 组:这个最容易漏,且现象是"代码没错但打不开串口"
  if id -nG "$USER" | grep -qw dialout; then
    ok "$USER 在 dialout 组"
  else
    warn "$USER 不在 dialout 组 → 串口会 Permission denied(--hw 会加,但**必须重新登录**才生效)"
  fi
}

# ---- 仓库自带资产:哪些 git 给不了,得单独补 ----
do_check_repo_assets() {
  echo "  --- 仓库资产 ---"

  # pyAgxArm 是厂商 SDK,.gitignore 里明确排除(第三方不入库),所以 clone 拿不到
  if [[ -d "$REPO/pyAgxArm-master/pyAgxArm-master" ]]; then
    ok "pyAgxArm SDK 就位"
  else
    err "缺 pyAgxArm SDK — 真机 --no-mock 必需,git 里没有(第三方不入库)"
    echo "      从开发机拷: rsync -av <dev>:~/ros2_ws/lerobotTest/pyAgxArm-master/ $REPO/pyAgxArm-master/"
    FAILED=1
  fi

  # 技能表引用的轨迹:已用 !sim/out/*.npz 破例入库,理应存在
  local miss=0
  for f in robot_traj_nero_inspire_rgbd.npz robot_traj_nero_inspire_rgb.npz; do
    [[ -f "$REPO/sim/out/$f" ]] || { warn "缺 sim/out/$f — trajectory 类技能会报错"; miss=1; }
  done
  [[ $miss -eq 0 ]] && ok "技能轨迹 npz 就位"

  # 装配 URDF 含绝对路径,故不入库,必须在本机重建
  if [[ -f "$REPO/sim/assets/nero_inspire_right.urdf" ]]; then
    if grep -q "$REPO" "$REPO/sim/assets/nero_inspire_right.urdf" 2>/dev/null; then
      ok "装配 URDF 已重建(路径匹配本机)"
    else
      warn "装配 URDF 里的绝对路径不是本机的 → --env 会重建"
    fi
  else
    warn "装配 URDF 未生成 → --env 会跑 build_nero_inspire.py 重建"
  fi
}

# ═══════════════════════════════════════════════ 建环境(不需 sudo,只动 $HOME)
do_env() {
  step "建 python 环境:conda lerobot + gradio venv"

  local conda_bin
  conda_bin="$(find_conda)" || {
    err "conda 未装。装 Miniconda 后重开 shell 再跑:"
    echo "      wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh"
    echo "      bash Miniconda3-latest-Linux-x86_64.sh -b -p \$HOME/miniconda3"
    echo "      \$HOME/miniconda3/bin/conda init bash"
    return 1
  }
  ok "用 conda: $conda_bin"

  # --- conda lerobot:python 必须 3.10 ---
  # 不是随便挑的版本:sim 的 ROS 脚本用这个解释器跑,靠 source humble 注入的
  # PYTHONPATH 去 import rclpy。rclpy 的 .so 是给 cp310 编的,3.11/3.12 直接 import 失败,
  # 而报错会指向别处,极难查。
  local cbase; cbase="$("$conda_bin" info --base)"
  # shellcheck disable=SC1091
  source "$cbase/etc/profile.d/conda.sh" 2>/dev/null || true

  if "$conda_bin" env list | awk '{print $1}' | grep -qx "$CONDA_ENV_NAME"; then
    local have; have="$("$conda_bin" run -n "$CONDA_ENV_NAME" python -c 'import sys;print("%d.%d"%sys.version_info[:2])' 2>/dev/null || echo '?')"
    if [[ "$have" == "$PY_WANT" ]]; then
      ok "conda env '$CONDA_ENV_NAME' 已存在,python $have"
    else
      err "conda env '$CONDA_ENV_NAME' 是 python $have,必须 $PY_WANT(rclpy ABI 要对齐)"
      echo "      重建: conda env remove -n $CONDA_ENV_NAME && 重跑本步"
      return 1
    fi
  else
    run "$conda_bin" create -y -n "$CONDA_ENV_NAME" "python=$PY_WANT"
    ok "conda env '$CONDA_ENV_NAME' 建好(python $PY_WANT)"
  fi

  echo "  装 requirements.txt(mujoco/pin/lerobot/rerun 等,较慢)"
  run "$conda_bin" run -n "$CONDA_ENV_NAME" python -m pip install -q -r "$REPO/requirements.txt"
  ok "conda 依赖装完"

  # 验证 conda python + ROS 能一起用:这是整个架构的关键假设
  if bash -lc "source /opt/ros/$ROS_DISTRO_WANT/setup.bash && '$cbase/envs/$CONDA_ENV_NAME/bin/python3' -c 'import rclpy'" 2>/dev/null; then
    ok "conda python + source humble → rclpy 可见(ABI 对齐)"
  else
    err "conda python 看不到 rclpy — 检查 conda python 是否为 $PY_WANT"
    FAILED=1
  fi

  do_env_venv
  do_env_urdf
}

# ---- 第三个环境:app_web 自己跑在这里 ----
# 为什么不塞进 conda:app_web 只 import fastapi/uvicorn/yaml,不碰 lerobot/rerun/pinocchio
# (那些是 subprocess 调的)。单独一个瘦 venv,前端崩了不牵连算法环境。
do_env_venv() {
  if [[ -x "$GRADIO_VENV/bin/python" ]]; then
    ok "web venv 已存在: $GRADIO_VENV"
  else
    run python3 -m venv "$GRADIO_VENV"
    ok "web venv 建好: $GRADIO_VENV"
  fi
  run "$GRADIO_VENV/bin/python" -m pip install -q --upgrade pip
  run "$GRADIO_VENV/bin/python" -m pip install -q fastapi uvicorn pyyaml python-multipart
  # python-multipart 是 FastAPI 收 UploadFile 表单要的,不装的话上传接口 500
  ok "web 依赖装完(fastapi/uvicorn/pyyaml/python-multipart)"
}

# ---- 重建装配 URDF ----
# 为什么必须重建:build_nero_inspire.py 把 mesh 路径写成**绝对路径**(MuJoCo 要),
# 所以生成物带着开发机的 /home/<用户> 前缀,换机必然失效。.gitignore 也因此排除它。
# 好消息:这脚本只用 stdlib(xml.etree + pathlib),系统 python3 就能跑。
do_env_urdf() {
  echo "  重建装配 URDF(mesh 用绝对路径,换机必须重建)"
  if run python3 "$REPO/sim/build_nero_inspire.py" >/dev/null 2>&1; then
    ok "URDF 重建完成: sim/assets/nero_inspire_right.urdf"
  else
    warn "URDF 重建失败 —— 单独跑看报错: python3 sim/build_nero_inspire.py"
  fi
}

# ═══════════════════════════════════════════════ colcon build(不需 sudo)
do_ros() {
  step "编 ROS2 工作区"

  [[ -d "$WS/src" ]] || { err "没找到 $WS/src — 仓库该 clone 到 <ws>/lerobotTest 下"; return 1; }

  # nero_inspire_ros2 是独立仓库(嵌套的),clone 主仓不会带上它
  if [[ -d "$WS/src/nero_inspire_ros2" ]]; then
    ok "src/nero_inspire_ros2 就位"
  else
    err "缺 src/nero_inspire_ros2 — 它是**另一个仓库**,要单独 clone:"
    echo "      git clone git@github.com:ZhangDaMengxx/VLA-HandArm-Ros.git $WS/src/nero_inspire_ros2"
    FAILED=1; return 1
  fi

  if command -v rosdep >/dev/null 2>&1; then
    [[ -d /etc/ros/rosdep/sources.list.d ]] || run sudo rosdep init || true
    run rosdep update --rosdistro="$ROS_DISTRO_WANT" || warn "rosdep update 失败(网络?),继续"
    echo "  装 src/ 声明的依赖(controller_manager / joint_trajectory_controller 等)"
    run bash -lc "source /opt/ros/$ROS_DISTRO_WANT/setup.bash && rosdep install --from-paths '$WS/src' --ignore-src -y -r" \
      || warn "rosdep install 有未解析项,继续 build 看是否够用"
  fi

  # install/ 不能从开发机拷:里面是编译产物 + 绝对路径,必须本机重编
  echo "  colcon build(install/ 是编译产物,不能拷,必须本机编)"
  run bash -lc "cd '$WS' && source /opt/ros/$ROS_DISTRO_WANT/setup.bash && colcon build --symlink-install"
  ok "colcon build 完成"
}

# ═══════════════════════════════════════════ 硬件配置(**需 sudo,会改系统**)
do_hw() {
  step "硬件配置:CAN + 串口权限 + udev"
  echo "${C_WARN}  这一步会改系统配置(需 sudo):${C_END}"
  echo "    1. 加载 CAN 内核模块,把 $CAN_IFACE 拉到 UP @ $CAN_BITRATE bps"
  echo "    2. 把 $USER 加进 dialout 组(为了开串口)"
  echo "    3. 写 /etc/udev/rules.d/99-nero-hand.rules(串口固定别名)"
  echo "    4. 写 systemd unit(开机自动拉 CAN)"
  echo "  以上都可逆,下面每步会先打印命令。"
  read -r -p "  继续? [y/N] " a; [[ "$a" == "y" || "$a" == "Y" ]] || { warn "跳过硬件配置"; return 0; }

  do_hw_can
  do_hw_serial
}

do_hw_can() {
  echo "  --- CAN ---"
  # USB-CAN 常见三类:gs_usb(candleLight/CANable)、peak_usb(PCAN)、slcan(串口式)
  run sudo modprobe can can_raw || warn "can/can_raw 模块加载失败"
  for m in gs_usb peak_usb; do sudo modprobe "$m" 2>/dev/null && ok "模块 $m 已加载" || true; done

  if ! ip link show "$CAN_IFACE" >/dev/null 2>&1; then
    err "$CAN_IFACE 不存在 — USB-CAN 没插或驱动不匹配。dmesg | tail -20 看看"
    FAILED=1; return 0
  fi

  # 已经 UP 就不动:改 bitrate 必须先 down,会打断正在跑的通信
  if ip -d link show "$CAN_IFACE" | grep -q 'state UP'; then
    ok "$CAN_IFACE 已 UP,不动它(改 bitrate 需先 down,会打断通信)"
  else
    run sudo ip link set "$CAN_IFACE" type can bitrate "$CAN_BITRATE"
    run sudo ip link set "$CAN_IFACE" up
    ok "$CAN_IFACE UP @ $CAN_BITRATE bps"
  fi

  # 手动 ip link 重启就没了,做成 systemd 才能开机自动起
  local unit=/etc/systemd/system/can-up@.service
  if [[ -f "$unit" ]]; then
    ok "systemd unit 已存在: $unit"
  else
    echo "  写 systemd unit(开机自动拉 CAN)"
    sudo tee "$unit" >/dev/null <<EOF
[Unit]
Description=Bring up CAN interface %i for NERO arm
After=network.target
[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set %i type can bitrate $CAN_BITRATE
ExecStart=/sbin/ip link set %i up
ExecStop=/sbin/ip link set %i down
[Install]
WantedBy=multi-user.target
EOF
    run sudo systemctl daemon-reload
    run sudo systemctl enable "can-up@$CAN_IFACE.service"
    ok "开机自动拉 $CAN_IFACE 已启用"
  fi
}

do_hw_serial() {
  echo "  --- 串口(手 RS485)---"

  # dialout:不在这个组,open('/dev/ttyUSB0') 直接 Permission denied
  if id -nG "$USER" | grep -qw dialout; then
    ok "$USER 已在 dialout 组"
  else
    run sudo usermod -aG dialout "$USER"
    warn "已加入 dialout 组 —— ${C_WARN}必须登出重进(或重启)才生效${C_END}"
    warn "  当前 shell 里权限还没变,现在跑真机仍会 Permission denied"
  fi

  # udev 固定别名:插拔顺序一变 ttyUSB0/1 就对调,那时"代码没错但读的是另一个设备"
  local rules=/etc/udev/rules.d/99-nero-hand.rules
  if [[ -f "$rules" ]]; then
    ok "udev 规则已存在: $rules"
  else
    # 拿第一个 USB 串口的 VID/PID 做模板 —— 只是模板,多半要按实物改
    local dev vid pid
    dev="$(ls /dev/ttyUSB* 2>/dev/null | head -1 || true)"
    if [[ -z "$dev" ]]; then
      warn "没有 /dev/ttyUSB* —— 插上 RS485 转换器后重跑 --hw 才能生成规则"
      return 0
    fi
    vid="$(udevadm info -a -n "$dev" | grep -m1 'ATTRS{idVendor}' | grep -oP '"\K[0-9a-f]+' || true)"
    pid="$(udevadm info -a -n "$dev" | grep -m1 'ATTRS{idProduct}' | grep -oP '"\K[0-9a-f]+' || true)"
    if [[ -z "$vid" || -z "$pid" ]]; then
      warn "取不到 $dev 的 VID/PID,跳过 udev。手动写规则参考:"
      echo "      udevadm info -a -n $dev | grep -E 'idVendor|idProduct|serial'"
      return 0
    fi
    echo "  用 $dev 的 VID:PID = $vid:$pid 生成规则"
    sudo tee "$rules" >/dev/null <<EOF
# 因时 RH56DFX 灵巧手的 RS485 转换器 → 固定别名 /dev/$HAND_SYMLINK
# 目的:插拔顺序变化时 ttyUSB 编号会对调,别名保证代码里的端口名始终指对设备。
# 注意:如果臂的 USB-CAN 用了同款芯片(VID:PID 相同),这条会同时匹配两个设备。
#      那时要改用序列号区分: ATTRS{serial}=="<用 udevadm 查>"
SUBSYSTEM=="tty", ATTRS{idVendor}=="$vid", ATTRS{idProduct}=="$pid", SYMLINK+="$HAND_SYMLINK", MODE="0660", GROUP="dialout"
EOF
    run sudo udevadm control --reload-rules
    run sudo udevadm trigger --subsystem-match=tty
    sleep 1
    if [[ -e "/dev/$HAND_SYMLINK" ]]; then
      ok "别名生效: /dev/$HAND_SYMLINK → $(readlink -f /dev/$HAND_SYMLINK)"
      warn "确认它指的是手而不是 USB-CAN;若指错,按规则里的注释改用 serial 区分"
    else
      warn "别名未生成 —— 重新插拔一次设备,或检查 $rules"
    fi
  fi
}

# ═══════════════════════════════════════ 写环境变量文件(把本机路径固定下来)
do_envfile() {
  local f="$WS/robot_host_env.sh"
  local cbase cb
  if cb="$(find_conda)"; then cbase="$("$cb" info --base 2>/dev/null)"; fi
  cbase="${cbase:-$HOME/miniconda3}"
  cat > "$f" <<EOF
# robot_host_env.sh — 由 deploy_robot_host.sh 生成。跑任何东西前先 source 它。
# 这些变量让代码不依赖写死的用户名/路径(见 nero_arm_bridge.py 的 _find_lerobot_site)。
source /opt/ros/$ROS_DISTRO_WANT/setup.bash
source $WS/install/setup.bash

export LEROBOT_REPO=$REPO
export LEROBOT_PY=$cbase/envs/$CONDA_ENV_NAME/bin/python3
export ROS_PYTHON=\$LEROBOT_PY
# LEROBOT_SITE 不用设:bridge 会从 LEROBOT_PY 自动推出同环境的 site-packages
export ROS_SETUP="source /opt/ros/$ROS_DISTRO_WANT/setup.bash && source $WS/install/setup.bash"
export WEB_PORT=$WEB_PORT

# 手的串口:udev 别名优先,回落到 ttyUSB0
if [ -e /dev/$HAND_SYMLINK ]; then export INSPIRE_HAND_PORT=/dev/$HAND_SYMLINK
else export INSPIRE_HAND_PORT=/dev/ttyUSB0; fi
export CAN_IFACE=$CAN_IFACE

# 启动 web:  \$GRADIO_VENV/bin/python \$LEROBOT_REPO/sim/app_web.py
export GRADIO_VENV=$GRADIO_VENV
EOF
  ok "环境变量文件: $f"
  echo "      用法: source $f"
}

# ═════════════════════════════════════════ 自检(mock 优先,不碰硬件)
do_verify() {
  step "自检:先 mock 跑通链路,再验安全闸"
  # shellcheck disable=SC1090
  local envf="$WS/robot_host_env.sh"
  [[ -f "$envf" ]] || { warn "还没生成 $envf,先跑 --env"; do_envfile; }

  echo "  1) 技能表 + 安全闸单测(纯 python,不需 ROS/硬件)"
  if run bash -lc "cd '$REPO' && python3 -m pytest sim/skills/test_schema.py sim/skills/test_runner_gates.py -q" 2>&1 | tail -3; then
    ok "技能表与安全闸单测通过"
  else
    warn "单测有失败 —— 上真机前必须先弄清楚(安全闸是最后一道防线)"
  fi

  echo "  2) runner 干跑(展开技能但不发 ROS)"
  run bash -lc "source '$envf' && python3 '$REPO/sim/skills/runner.py' --dry-run --once '{\"skill_id\":\"go_home\",\"confirmed\":true,\"assume_enabled\":true}'" 2>&1 | tail -4 \
    && ok "干跑通过" || warn "干跑失败,看上面报错"

  do_verify_bridge
}

do_verify_bridge() {
  local envf="$WS/robot_host_env.sh"
  echo "  3) bridge mock 模式(臂发正弦、手回读占位,不碰 CAN/串口)"
  local log; log="$(mktemp)"
  bash -lc "source '$envf' && timeout 6 python3 '$REPO/sim/nero_arm_bridge.py' --mock" >"$log" 2>&1 || true
  if grep -q '桥接启动' "$log"; then
    ok "bridge mock 启动成功"
    echo "      $(grep -m1 '桥接启动' "$log" | sed 's/^.*\]: //')"
  else
    err "bridge mock 没起来:"; sed -n '1,12p' "$log"; FAILED=1
  fi

  echo "  4) mock 下 /joint_states 有没有真在发(13 个关节)"
  bash -lc "source '$envf' && (timeout 8 python3 '$REPO/sim/nero_arm_bridge.py' --mock >/dev/null 2>&1 &) ; sleep 3; timeout 4 ros2 topic echo /joint_states --once" >"$log" 2>&1 || true
  if grep -q 'name:' "$log"; then
    ok "/joint_states 正常发布($(grep -c '^- ' "$log" || echo '?') 个关节名)"
  else
    warn "没收到 /joint_states —— 单独跑 bridge 再 ros2 topic list 看看"
  fi
  rm -f "$log"

  echo
  if [[ $FAILED -eq 0 ]]; then
    ok "mock 链路通了。真机步骤(按顺序,别跳):"
    echo "      a) source $WS/robot_host_env.sh"
    echo "      b) candump \$CAN_IFACE          # 确认臂在 CAN 上有回包"
    echo "      c) python3 sim/nero_arm_bridge.py --no-mock   # 只读,先不控制"
    echo "      d) 确认 /joint_states 是真实角度后,再加 --enable-control"
  else
    err "有失败项,先解决再上真机"
  fi
}

# ═══════════════════════════════════════════════════════════════ 启动提示
do_hint_run() {
  step "怎么跑起来"
  cat <<EOF
  source $WS/robot_host_env.sh

  # 终端 1:真机桥(先 --mock 验,再 --no-mock)
  python3 $REPO/sim/nero_arm_bridge.py --mock --enable-control

  # 终端 2:Web 工作台
  \$GRADIO_VENV/bin/python $REPO/sim/app_web.py

  开发机浏览器打开: http://$(hostname -I 2>/dev/null | awk '{print $1}'):$WEB_PORT

${C_WARN}  安全提醒:$WEB_PORT 在局域网上没有鉴权,同一路由下任何人打开就能下发关节指令。${C_END}
${C_WARN}  这网络不只有你的话,先加一层(nginx basic auth / 只绑内网段 / ufw 限源 IP)。${C_END}
EOF
}

# ═══════════════════════════════════════════════════════════════════ 入口
main() {
  local did=0
  for a in "$@"; do
    case "$a" in
      --check)  do_check; did=1;;
      --env)    do_env; do_envfile; did=1;;
      --ros)    do_ros; did=1;;
      --hw)     do_hw; did=1;;
      --verify) do_verify; did=1;;
      --all)    do_check; do_env; do_envfile; do_ros; do_hw; do_verify; do_hint_run; did=1;;
      -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0;;
      *) err "未知参数: $a"; exit 2;;
    esac
  done
  [[ $did -eq 1 ]] || { sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0; }

  echo
  if [[ $FAILED -eq 0 ]]; then ok "完成,没有阻塞问题"; else err "有失败项,见上面 x 标记"; exit 1; fi
}

main "$@"
