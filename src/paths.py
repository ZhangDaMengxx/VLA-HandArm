"""集中路径:仓库自动定位 + 内置 assets/data/configs。所有 src 脚本从这里取路径,
不再引用第三方仓库的绝对路径。clone 到任何位置都能用。

## 2026-08-10 assets 重组

旧源码目录内的 assets/ 已合并到仓库顶层 assets/,按用途分层:
- assets/arm/, assets/hand/ — 源 URDF + mesh(STL/obj)
- assets/assembled/ — 装配体 URDF(pinocchio/MuJoCo 用,绝对路径 mesh)
- assets/viz/ — 浏览器可视化产物(glb + 相对路径 URDF)
"""
import glob
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SRC = REPO / "src"
# 兼容尚未重命名局部变量的模块；路径本身已经指向 src/。
SIM = SRC
ASSETS = REPO / "assets"
DATA = REPO / "data"
CONFIGS = REPO / "configs"
OUT = SRC / "out"

# ===== 源 URDF + mesh =====
ARM_ROOT = ASSETS / "arm"
HAND_ROOT = ASSETS / "hand"
HAND_LEGACY = ASSETS / "hand_legacy"
ARM_LEGACY = ASSETS / "arm_legacy"

NERO_URDF = ARM_ROOT / "urdf/nero_description.urdf"
NERO_FLANGE_URDF = ARM_ROOT / "urdf/nero_with_hand_flange_description.urdf"
INSPIRE_URDF = HAND_ROOT / "urdf/inspire_hand_right.urdf"

# ===== 装配体(pinocchio/MuJoCo 用,绝对路径 mesh)=====
ASSEMBLED = ASSETS / "assembled"
ASSEMBLY_URDF = ASSEMBLED / "nero_inspire_right.urdf"         # 臂+法兰+手
GRIPPER_URDF = ASSEMBLED / "nero_gripper_right.urdf"          # 臂+夹爪
HAND_ABSOLUTE_URDF = ASSEMBLED / "inspire_hand_absolute.urdf" # 手单体,绝对路径

# ===== 浏览器 viz(glb + 相对路径 URDF)=====
VIZ = ASSETS / "viz"
ARM_VIZ = VIZ / "arm"
HAND_VIZ = VIZ / "hand"
COMBO_VIZ = VIZ / "combo"

ARM_VIZ_URDF = ARM_VIZ / "nero_arm_viz.urdf"
HAND_VIZ_URDF = HAND_VIZ / "inspire_hand_right_viz.urdf"
COMBO_VIZ_URDF = COMBO_VIZ / "nero_inspire_right_viz.urdf"

# ===== dex_retargeting =====
RETARGET_CONFIG = CONFIGS / "inspire_hand_right_local.yml"
RETARGET_URDF_DIR = ASSETS  # urdf_path 相对它解析


def find_video():
    """返回 data/ 下第一个 mp4(没有则 None)。"""
    v = sorted(glob.glob(str(DATA / "*.mp4")))
    return v[0] if v else None
