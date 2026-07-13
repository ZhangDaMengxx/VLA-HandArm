"""量化抖动:臂/手关节的逐帧变化 + 手腕朝向逐帧跳变,定位抖动来源。"""
import pickle
from pathlib import Path
import numpy as np

REPO = Path("/home/zhang123/ros2_ws/lerobotTest")
R = pickle.load(open(REPO / "sim/out/robot_traj.pkl", "rb"))
arm = np.asarray(R["arm"])     # (F,7) rad
hand = np.asarray(R["hand"])   # (F,12) rad
Fd = pickle.load(open(REPO / "sim/out/full_traj.pkl", "rb"))
wrist = np.asarray(Fd["wrist_pose"])  # (F,4,4)


def framestep(traj):
    d = np.abs(np.diff(traj, axis=0))
    return np.rad2deg(d.mean()), np.rad2deg(np.percentile(d, 95)), np.rad2deg(d.max())


print("逐帧 |Δ|(度):        mean    p95     max")
print("  arm  joints:      %6.2f %6.2f %6.2f" % framestep(arm))
print("  hand joints:      %6.2f %6.2f %6.2f" % framestep(hand))

ang = []
for f in range(1, len(wrist)):
    dR = wrist[f][:3, :3] @ wrist[f - 1][:3, :3].T
    ang.append(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
ang = np.array(ang)
print("  wrist orient:     %6.2f %6.2f %6.2f" % (ang.mean(), np.percentile(ang, 95), ang.max()))

# 抖动 = 高频往复。数方向反转次数(每关节 Δ 符号变化)/ 帧数
def reversal_rate(traj):
    s = np.sign(np.diff(traj, axis=0))
    rev = (np.diff(s, axis=0) != 0).sum(0)
    return (rev / (len(traj) - 2)).mean()

print("方向反转率 arm  = %.2f (越接近1越抖)" % reversal_rate(arm))
print("方向反转率 hand = %.2f" % reversal_rate(hand))
