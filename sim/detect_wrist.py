"""B-2 跑通:视频 → MediaPipe → 手指 retarget(dex-retargeting) + 手腕 6-DoF 估计。
产出同步的 (手关节, 手腕位姿) 轨迹,存 sim/out/full_traj.pkl,并打印手腕轨迹统计。
手腕位置是单目近似(相机系);Femto 到手后换 depth_lookup 即度量准确。
"""
import sys
import glob
import pickle
from pathlib import Path

import numpy as np
import cv2

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))   # sim/(vendored detector + estimate_wrist)

from dex_retargeting.retargeting_config import RetargetingConfig
from single_hand_detector import SingleHandDetector
from estimate_wrist import estimate_wrist_pose

CFG = REPO / "configs/inspire_hand_right_local.yml"
URDF_DIR = REPO / "assets"
OUT = REPO / "sim/out/full_traj.pkl"

vids = sorted(glob.glob(str(REPO / "data/*.mp4")))
video = vids[0]
print("video:", video)

RetargetingConfig.set_default_urdf_dir(str(URDF_DIR))
rt = RetargetingConfig.load_from_file(str(CFG), override={"low_pass_alpha": 1.0}).build()  # 关内部低通:出原始满幅度,平滑交给 SavGol
names = list(rt.optimizer.robot.dof_joint_names)
idx = np.asarray(rt.optimizer.target_link_human_indices)
origin_i, task_i = idx[0, :], idx[1, :]

det = SingleHandDetector(hand_type="Right", selfie=False,
                         min_detection_confidence=0.5, min_tracking_confidence=0.5)
cap = cv2.VideoCapture(video)
hand, wrist = [], []
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    rgb = frame[..., ::-1]
    num, joint_pos, kp2d, wrist_rot = det.detect(rgb)
    if num == 0:
        if hand:
            hand.append(hand[-1]); wrist.append(wrist[-1])
        continue
    ref = joint_pos[task_i, :] - joint_pos[origin_i, :]
    q = rt.retarget(ref)
    kp2d_px = SingleHandDetector.parse_keypoint_2d(kp2d, frame.shape)
    T = estimate_wrist_pose(joint_pos, kp2d_px, wrist_rot, det.operator2mano, frame.shape)
    hand.append(q)
    wrist.append(T)
cap.release()

hand = np.asarray(hand)
wrist = np.asarray(wrist)
pos = wrist[:, :3, 3]
zax = wrist[:, :3, 2]  # 手腕伸出轴(相机系)
mean_z = zax.mean(0); mean_z /= np.linalg.norm(mean_z)
ang = np.degrees(np.arccos(np.clip(zax @ mean_z, -1, 1)))

print(f"frames: {len(wrist)}  hand qpos: {hand.shape}")
print("wrist pos range (m, 相机系):")
for i, ax in enumerate("xyz"):
    print(f"  {ax}: {pos[:,i].min():.3f} .. {pos[:,i].max():.3f}  (span {np.ptp(pos[:,i]):.3f})")
print(f"wrist orientation swing: {ang.min():.1f}..{ang.max():.1f} deg from mean (证明朝向在跟)")

OUT.parent.mkdir(parents=True, exist_ok=True)
with open(OUT, "wb") as f:
    pickle.dump(dict(hand=hand, wrist_pose=wrist, joint_names=names), f)
print("saved", OUT)
