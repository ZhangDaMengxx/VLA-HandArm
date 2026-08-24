#!/usr/bin/env bash
# setup_wsl_system.sh — 需要 sudo 的系统级准备(WSL2 / Ubuntu 22.04)
#
# 做四件事,每件都幂等:
#   1. apt 源换清华镜像(备份原文件)
#   2. 装 ROS 2 Humble desktop + colcon/rosdep + 系统 python 包
#   3. rosdep 初始化(用清华 rosdistro 镜像,绕开 raw.githubusercontent 不通)
#   4. 把当前用户加进 dialout 组(串口权限)
#
# 用法:
#   sudo bash deploy/setup_wsl_system.sh
#
# 之后再跑不需要 sudo 的部分:
#   bash deploy/deploy_robot_host.sh --check --env --ros
set -euo pipefail

C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_DIM=$'\033[2m'; C_END=$'\033[0m'
ok()   { echo "${C_OK}  ✔${C_END} $*"; }
warn() { echo "${C_WARN}  !${C_END} $*"; }
err()  { echo "${C_ERR}  x${C_END} $*" >&2; }
step() { echo; echo "═══ $*"; }
run()  { echo "${C_DIM}    \$ $*${C_END}"; "$@"; }

[[ $EUID -eq 0 ]] || { err "要用 sudo 跑: sudo bash $0"; exit 1; }

# sudo 下 $USER 可能是 root,用 SUDO_USER 拿回真实用户
REAL_USER="${SUDO_USER:-$USER}"
[[ "$REAL_USER" != "root" ]] || warn "拿不到非 root 用户名,dialout 那步会跳过"

MIRROR="https://mirrors.tuna.tsinghua.edu.cn"
CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-jammy}")"
ARCH="$(dpkg --print-architecture)"
KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
# 已由外部下载并校验过指纹的 key(Open Robotics: C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654)
ROS_KEY_SRC="${ROS_KEY_SRC:-/tmp/ros.key}"

# ═════════════════════════════════════════════════════════ 1. apt 源换镜像
do_mirror() {
  step "apt 源 → 清华镜像"

  if grep -q 'mirrors.tuna.tsinghua.edu.cn' /etc/apt/sources.list 2>/dev/null; then
    ok "已是清华源,跳过"
    return 0
  fi

  # 备份带时间戳,想回滚就 cp 回去
  local bak="/etc/apt/sources.list.bak.$(date +%Y%m%d%H%M%S)"
  run cp /etc/apt/sources.list "$bak"
  ok "原源已备份: $bak"

  # security 也指向镜像:archive.ubuntu.com 和 security.ubuntu.com 在国内都慢
  cat > /etc/apt/sources.list <<EOF
# 清华 TUNA 镜像 — 由 deploy/setup_wsl_system.sh 写入
# 回滚: sudo cp $bak /etc/apt/sources.list && sudo apt update
deb $MIRROR/ubuntu/ $CODENAME main restricted universe multiverse
deb $MIRROR/ubuntu/ $CODENAME-updates main restricted universe multiverse
deb $MIRROR/ubuntu/ $CODENAME-backports main restricted universe multiverse
deb $MIRROR/ubuntu/ $CODENAME-security main restricted universe multiverse
EOF
  ok "已写入清华源($CODENAME)"
}

# ═══════════════════════════════════════════════════════ 2. ROS 2 apt 源
do_ros_repo() {
  step "ROS 2 apt 源 → 清华镜像"

  # key:raw.githubusercontent 在国内基本不通,所以由调用方预先下好放 /tmp/ros.key。
  # 指纹应为 C1CF 6E31 E6BA DE88 68B1 72B4 F42E D6FB AB17 C654 (Open Robotics)。
  if [[ -f "$KEYRING" ]]; then
    ok "keyring 已存在: $KEYRING"
  elif [[ -f "$ROS_KEY_SRC" ]]; then
    run gpg --dearmor --batch --yes -o "$KEYRING" "$ROS_KEY_SRC" 2>/dev/null \
      || run cp "$ROS_KEY_SRC" "$KEYRING"   # 已是二进制格式时 dearmor 会报错
    run chmod 0644 "$KEYRING"
    ok "ROS key 已装: $KEYRING"
  else
    err "找不到 $ROS_KEY_SRC —— 先在普通用户下拿 key:"
    echo "      curl -fsSL https://ghfast.top/https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key"
    echo "      gpg --show-keys --with-fingerprint /tmp/ros.key   # 核对指纹"
    exit 1
  fi

  local list=/etc/apt/sources.list.d/ros2.list
  echo "deb [arch=$ARCH signed-by=$KEYRING] $MIRROR/ros2/ubuntu $CODENAME main" > "$list"
  ok "已写入: $list"
}

# ═══════════════════════════════════════════════════════════ 3. 装包
do_install() {
  step "apt update + 装 ROS 2 Humble 与工具链"
  run apt-get update

  echo "  装 ROS 2 Humble desktop(约 2-3 GB,含 rviz2)"
  run env DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-desktop

  echo "  装构建工具与系统 python 包"
  # python3-serial: 灵巧手 RS485;python3-yaml/numpy: runner/backend;
  # python3-pytest: --verify 跑安全闸单测
  run env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-numpy \
    python3-yaml \
    python3-serial \
    python3-pytest \
    python3-venv \
    can-utils \
    build-essential \
    git
  # python3-venv: 没它 `python3 -m venv` 会建出**没有 pip 的空壳**
  # (ensurepip 在 Ubuntu 上被拆到这个包里),现象是 "No module named pip"
  ok "装包完成"
}

# ═════════════════════════════════════════ 4b. src/ 声明的 ROS 依赖
do_pkg_deps() {
  step "装 nero_inspire_ros2 声明的 ROS 依赖"
  # 只解析 nero_inspire_ros2:src/ 下的 so101_* 要整套 moveit_*,与本部署无关
  local src_dir; src_dir="$(dirname "$(dirname "$(readlink -f "$0")")")/../src/nero_inspire_ros2"
  src_dir="$(readlink -f "$src_dir")"
  if [[ ! -d "$src_dir" ]]; then
    warn "找不到 $src_dir,跳过"
    return 0
  fi
  # rosdep 要以普通用户跑(缓存在 ~/.ros),但它内部调 apt 需要 root —— 已经是 root 了
  run sudo -u "$REAL_USER" env ROSDISTRO_INDEX_URL="$MIRROR/rosdistro/index-v4.yaml" \
    bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths '$src_dir' --ignore-src -y -r" \
    || warn "有未解析项,colcon build 时再看是否够用"
  ok "ROS 依赖装完"
}

# ══════════════════════════════════════════════════ 4. rosdep(走镜像)
do_rosdep() {
  step "rosdep 初始化(清华 rosdistro 镜像)"

  # 默认的 rosdep 源在 raw.githubusercontent,国内基本超时。
  # 换成清华镜像:sources 列表 + index 都指过去。
  local d=/etc/ros/rosdep/sources.list.d
  mkdir -p "$d"
  cat > "$d/20-default.list" <<EOF
# 清华 rosdistro 镜像 — 由 deploy/setup_wsl_system.sh 写入
# 官方源在 raw.githubusercontent.com,国内基本超时
yaml $MIRROR/rosdistro/rosdep/osx-homebrew.yaml osx
yaml $MIRROR/rosdistro/rosdep/base.yaml
yaml $MIRROR/rosdistro/rosdep/python.yaml
yaml $MIRROR/rosdistro/rosdep/ruby.yaml
gbpdistro $MIRROR/rosdistro/releases/fuerte.yaml fuerte
EOF
  ok "rosdep 源已指向镜像"

  # ROSDISTRO_INDEX_URL 也得换,否则 rosdep update 仍去访问 raw.githubusercontent
  local prof=/etc/profile.d/rosdistro-mirror.sh
  echo "export ROSDISTRO_INDEX_URL=$MIRROR/rosdistro/index-v4.yaml" > "$prof"
  ok "已写 $prof(新 shell 生效)"

  # rosdep update 要以普通用户跑,root 跑会把缓存写到 /root 下,普通用户读不到
  if [[ "$REAL_USER" != "root" ]]; then
    run sudo -u "$REAL_USER" env ROSDISTRO_INDEX_URL="$MIRROR/rosdistro/index-v4.yaml" \
      rosdep update --rosdistro=humble || warn "rosdep update 失败(网络?),--ros 时会重试"
  else
    warn "拿不到普通用户,跳过 rosdep update —— 之后手动跑: rosdep update"
  fi
}

# ═════════════════════════════════════════════════════ 5. 串口权限
do_dialout() {
  step "串口权限:把 $REAL_USER 加进 dialout 组"
  if [[ "$REAL_USER" == "root" ]]; then
    warn "拿不到普通用户,跳过"
    return 0
  fi
  if id -nG "$REAL_USER" | grep -qw dialout; then
    ok "$REAL_USER 已在 dialout 组"
  else
    run usermod -aG dialout "$REAL_USER"
    warn "已加入 —— ${C_WARN}必须完全退出 WSL 再进才生效${C_END}"
    warn "  在 Windows PowerShell 里: wsl --shutdown"
  fi
}

main() {
  do_mirror
  do_ros_repo
  do_install
  do_rosdep
  do_pkg_deps
  do_dialout

  step "完成"
  ok "系统级准备就绪。接下来(不需 sudo):"
  cat <<EOF
    cd $(dirname "$(dirname "$(readlink -f "$0")")")
    bash deploy/deploy_robot_host.sh --check
    bash deploy/deploy_robot_host.sh --env     # lerobot-v3 主环境 + ros-humble 薄环境
    bash deploy/deploy_robot_host.sh --ros     # colcon build

  ${C_WARN}注意:如果刚被加进 dialout 组,先 wsl --shutdown 再继续。${C_END}
EOF
}

main "$@"
