"""把我们塞进 dex-retargeting 的文件归拢到 overlays/dex-retargeting/(只拷贝,不动原文件)。
这样一个干净自足的 git 仓库就能保住这些"嵌在第三方里"的自研内容。
"""
import shutil
from pathlib import Path

REPO = Path("/home/zhang123/ros2_ws/lerobotTest")
DR = REPO / "dex-retargeting-main/dex-retargeting-main"
VR = DR / "example/vector_retargeting"
OV = REPO / "overlays/dex-retargeting"

(OV / "src/dex_retargeting/configs/teleop").mkdir(parents=True, exist_ok=True)
(OV / "example/vector_retargeting").mkdir(parents=True, exist_ok=True)

# 配置
shutil.copy2(DR / "src/dex_retargeting/configs/teleop/inspire_hand_right_local.yml",
             OV / "src/dex_retargeting/configs/teleop/inspire_hand_right_local.yml")

# vector_retargeting 里我们加的 + 改的
ours = ["hand_robot_visualizer.py", "VISUALIZER_ARCH.md", "hand_perception.py",
        "webgl_replay.py", "webgl_server.py", "render_robot_hand_meshcat.py",
        "check_urdf_meshes.py", "inspect_pkl.py",
        "detect_from_video.py", "render_robot_hand.py"]  # 后两个是改过的原文件
for f in ours:
    src = VR / f
    if src.exists():
        shutil.copy2(src, OV / "example/vector_retargeting" / f)
    else:
        print("MISS", f)

# 实验目录(跳过 __pycache__)
exp_dst = OV / "example/vector_retargeting/exp_pip_vs_tip"
if exp_dst.exists():
    shutil.rmtree(exp_dst)
shutil.copytree(VR / "exp_pip_vs_tip", exp_dst,
                ignore=shutil.ignore_patterns("__pycache__"))

print("=== overlays 归拢完成 ===")
for p in sorted(OV.rglob("*")):
    if p.is_file():
        print(" ", p.relative_to(REPO))
