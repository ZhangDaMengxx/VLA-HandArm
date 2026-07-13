"""检测视频人手 → dex-retargeting → 保存 inspire 关节轨迹 + 手腕朝向,供装配回放。
复用 detect_from_video.py 的确切逻辑,用开合修复版配置 inspire_hand_right_local.yml。"""
import sys
import glob
import pickle
from pathlib import Path

import numpy as np
import cv2

REPO = Path("/home/zhang123/ros2_ws/lerobotTest")
DEX = REPO / "dex-retargeting-main/dex-retargeting-main"
sys.path.insert(0, str(DEX / "example/vector_retargeting"))

from dex_retargeting.retargeting_config import RetargetingConfig
from single_hand_detector import SingleHandDetector

CFG = DEX / "src/dex_retargeting/configs/teleop/inspire_hand_right_local.yml"
URDF_DIR = DEX / "assets/robots/hands"
OUT = REPO / "sim/out/hand_traj.pkl"

vids = sorted(glob.glob(str(DEX / "example/vector_retargeting/data/*.mp4")))
if not vids:
    vids = sorted(glob.glob(str(REPO / "**/*.mp4"), recursive=True))
assert vids, "no mp4 found"
video = vids[0]
print("video:", video)

RetargetingConfig.set_default_urdf_dir(str(URDF_DIR))
rt = RetargetingConfig.load_from_file(str(CFG)).build()
names = list(rt.optimizer.robot.dof_joint_names)
print("joint_names:", names)
idx = np.asarray(rt.optimizer.target_link_human_indices)
origin_i, task_i = idx[0, :], idx[1, :]

detector = SingleHandDetector(hand_type="Right", selfie=False,
                              min_detection_confidence=0.5, min_tracking_confidence=0.5)
cap = cv2.VideoCapture(video)
data, wrist = [], []
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    rgb = frame[..., ::-1]
    num, joint_pos, kp2d, wrist_rot = detector.detect(rgb)
    if num == 0:
        if data:
            data.append(data[-1]); wrist.append(wrist[-1])
        continue
    ref = joint_pos[task_i, :] - joint_pos[origin_i, :]
    q = rt.retarget(ref)
    data.append(q)
    wrist.append(wrist_rot @ detector.operator2mano)
cap.release()

data = np.asarray(data)
wrist = np.asarray(wrist)
print("frames:", len(data), "qpos shape:", data.shape)
print("qpos range per joint (deg):")
for i, n in enumerate(names):
    print(f"  {n:28s} {np.rad2deg(data[:,i].min()):7.1f} .. {np.rad2deg(data[:,i].max()):7.1f}")
OUT.parent.mkdir(parents=True, exist_ok=True)
with open(OUT, "wb") as f:
    pickle.dump(dict(data=data, wrist_rot=wrist, joint_names=names), f)
print("saved", OUT)
