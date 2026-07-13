"""把管线需要的资产内置进仓库:assets/(NERO+inspire URDF+网格)、configs/、data/(示例视频)。
一次性脚本;之后代码从这些仓库本地目录读,不再引用第三方仓库。
"""
import glob
import shutil
from pathlib import Path

REPO = Path("/home/zhang123/ros2_ws/lerobotTest")
DR = REPO / "dex-retargeting-main/dex-retargeting-main"
PKL = REPO / "pinocchio-kinematics-lite-main/pinocchio-kinematics-lite-main"

# NERO(URDF + 网格)-> assets/nero/
shutil.copytree(PKL / "src/pinocchio_kinematics_lite/assets/nero",
                REPO / "assets/nero", dirs_exist_ok=True,
                ignore=shutil.ignore_patterns("__pycache__"))

# inspire 手(URDF + 网格)-> assets/inspire_hand/
shutil.copytree(DR / "assets/robots/hands/inspire_hand",
                REPO / "assets/inspire_hand", dirs_exist_ok=True)

# retargeting 配置 -> configs/
(REPO / "configs").mkdir(exist_ok=True)
shutil.copy2(DR / "src/dex_retargeting/configs/teleop/inspire_hand_right_local.yml",
             REPO / "configs/inspire_hand_right_local.yml")

# 示例视频 -> data/
(REPO / "data").mkdir(exist_ok=True)
vids = sorted(glob.glob(str(DR / "example/vector_retargeting/data/*.mp4")))
if vids:
    shutil.copy2(vids[0], REPO / "data" / Path(vids[0]).name)

print("=== 内置资产完成 ===")
for d in ["assets/nero", "assets/inspire_hand", "configs", "data"]:
    files = [p for p in (REPO / d).rglob("*") if p.is_file()]
    size = sum(p.stat().st_size for p in files) / 1e6
    print(f"  {d:22s} {len(files):3d} files, {size:.1f} MB")
