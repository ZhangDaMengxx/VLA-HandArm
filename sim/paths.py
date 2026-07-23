"""集中路径:仓库自动定位 + 内置 assets/data/configs。所有 sim 脚本从这里取路径,
不再引用第三方仓库的绝对路径。clone 到任何位置都能用。
"""
import glob
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SIM = REPO / "sim"
ASSETS = REPO / "assets"
DATA = REPO / "data"
CONFIGS = REPO / "configs"

NERO_DESCRIPTION = ASSETS / "nero_description"
NERO_URDF = NERO_DESCRIPTION / "urdf" / "nero_description.urdf"
NERO_FLANGE_URDF = NERO_DESCRIPTION / "urdf" / "nero_with_hand_flange_description.urdf"
INSPIRE_URDF = ASSETS / "inspire_hand" / "inspire_hand_right.urdf"
RETARGET_CONFIG = CONFIGS / "inspire_hand_right_local.yml"
RETARGET_URDF_DIR = ASSETS          # dex_retargeting.set_default_urdf_dir;配置 urdf_path 相对它
ASSEMBLY_URDF = SIM / "assets" / "nero_inspire_right.urdf"   # build_nero_inspire.py 生成
OUT = SIM / "out"


def find_video():
    """返回 data/ 下第一个 mp4(没有则 None)。"""
    v = sorted(glob.glob(str(DATA / "*.mp4")))
    return v[0] if v else None
