"""B-3: full_traj.pkl → 每帧 NeroKinematics 逆解 → 本体层机器人轨迹 robot_traj.pkl。

消抖用 Savitzky-Golay(离线、对称窗=零滞后、保峰值/保幅度),取代朴素 EMA:
  1. 手腕朝向:四元数(符号对齐)上 SavGol → 平滑 IK 目标。
  2. 逆解出的臂关节:SavGol。
  3. 手指:retargeting 已关内部低通(满幅度、带抖),这里 SavGol 去抖并保幅度。
配合 detect_wrist.py 的 low_pass_alpha=1.0,即可"张开幅度回来 + 不抖"。
"""
import sys
import argparse
import pickle
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as Rot

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))   # sim/(nero_kin, wrist_stabilize)
from nero_kin import NeroKin
from wrist_stabilize import gate_outliers, attenuate_out_of_plane
NERO_URDF = REPO / "assets/nero/nero_description.urdf"

WIN, POLY = 11, 3   # SavGol 窗口(奇数帧)/ 多项式阶。窗口越大越平滑
Q_HOME_ARM = np.array([1.2635, 0.9302, 2.6464, 1.7779, 1.0898, 0.6034, -0.6634])

ap_ = argparse.ArgumentParser(description="full_traj → IK → 本体层 robot_traj")
ap_.add_argument("--out", default=str(REPO / "sim/out/robot_traj.pkl"), help="输出 pkl 路径")
ap_.add_argument("--k-null", type=float, default=0.0,
                 help="IK 零空间正则强度(往 home 姿态拉,压掉冗余自由度乱飘)。0=原纯任务空间")
ap_.add_argument("--oop-alpha", type=float, default=0.4,
                 help="出平面(手掌法向倾斜)朝向分量衰减系数。1=不衰减(基线);越小越贴参考帧,压单目深度噪声。默认0.4")
ap_.add_argument("--gate-deg", type=float, default=8.0,
                 help="残差门限(度):帧间朝向增量超此值则限幅,剔离群跳变帧。0=关。默认8")
ARGS = ap_.parse_args()


def revrate(traj):
    s = np.sign(np.diff(traj, axis=0))
    return ((np.diff(s, axis=0) != 0).sum(0) / (len(traj) - 2)).mean()


with open(REPO / "sim/out/full_traj.pkl", "rb") as f:
    T = pickle.load(f)
hand = np.asarray(T["hand"])          # (F,12) 原始满幅度(带抖)
wrist = np.asarray(T["wrist_pose"])   # (F,4,4)
names = list(T["joint_names"])
F = len(wrist)
print("frames:", F)

# 1. 手腕朝向:符号对齐 → 残差门限剔跳变 → SavGol → 出平面各向异性衰减
quats = Rot.from_matrix(wrist[:, :3, :3]).as_quat()
for i in range(1, F):
    if np.dot(quats[i - 1], quats[i]) < 0:
        quats[i] = -quats[i]
quats = gate_outliers(quats, ARGS.gate_deg)                    # 剔离群跳变帧
quats_s = savgol_filter(quats, WIN, POLY, axis=0)
quats_s /= np.linalg.norm(quats_s, axis=1, keepdims=True)
Rs = Rot.from_quat(quats_s).as_matrix()
Rs = attenuate_out_of_plane(Rs, ARGS.oop_alpha, ref=0)         # 压出平面(单目深度噪声主源)
print(f"稳定化: gate={ARGS.gate_deg}° (限幅{getattr(gate_outliers,'last_clamped',0)}帧)  "
      f"out-of-plane α={ARGS.oop_alpha}")

# 2. IK(平滑朝向,位置锚定,热启动)
kin = NeroKin(NERO_URDF)
anchor = kin.fk(Q_HOME_ARM)
aR, ap = anchor[:3, :3], anchor[:3, 3]
R0 = Rs[0]
q_raw = np.zeros((F, 7))
prev = Q_HOME_ARM.copy()
ok = 0
for f in range(F):
    Rt = (Rs[f] @ R0.T) @ aR
    Tt = np.eye(4); Tt[:3, :3] = Rt; Tt[:3, 3] = ap
    prev, good = kin.ik(Tt, prev, q_rest=Q_HOME_ARM, k_null=ARGS.k_null)
    if good:
        ok += 1
    q_raw[f] = prev

# 3. SavGol 臂关节 + 手指
q_arm = savgol_filter(q_raw, WIN, POLY, axis=0)
hand_s = np.clip(savgol_filter(hand, WIN, POLY, axis=0), 0.0, 1.55)

print(f"IK success {ok}/{F}")
print("反转率(越低越平滑):     raw    savgol")
print("  hand:  %.2f   %.2f" % (revrate(hand), revrate(hand_s)))
print("  arm :  %.2f   %.2f" % (revrate(q_raw), revrate(q_arm)))
print("手指最张开角(度,越小越张开;旧 low_pass=0.2:index21/pinky14):")
for fj in ["index_proximal_joint", "middle_proximal_joint", "ring_proximal_joint", "pinky_proximal_joint"]:
    i = names.index(fj)
    print(f"  {fj:22s} {np.rad2deg(hand_s[:, i].min()):5.1f}  (原始 {np.rad2deg(hand[:, i].min()):5.1f})")

print("臂关节摆幅(度):", np.round((q_arm.max(0) - q_arm.min(0)) * 180 / np.pi, 1))

OUT = Path(ARGS.out)
OUT.parent.mkdir(parents=True, exist_ok=True)
with open(OUT, "wb") as f:
    pickle.dump(dict(arm=q_arm, hand=hand_s, hand_joint_names=names,
                     arm_joint_names=[f"joint{i}" for i in range(1, 8)]), f)
print(f"saved {OUT}  (k_null={ARGS.k_null})")
