"""B-3 完整回放:机器人 [臂+手] 轨迹(robot_traj.pkl)在 MeshCat 循环播放。
臂由人手腕朝向驱动、手指由 retarget 驱动 —— 整条"人手视频 → 机械臂+灵巧手"同步动。
用法: python sim/replay_full.py
"""
import sys
import pickle
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

REPO = Path(__file__).resolve().parents[1]
urdf = str(REPO / "sim/assets/nero_inspire_right.urdf")

with open(REPO / "sim/out/robot_traj.pkl", "rb") as f:
    T = pickle.load(f)
arm = np.asarray(T["arm"])            # (F,7)
hand = np.asarray(T["hand"])          # (F,12)
arm_names = list(T["arm_joint_names"])
hand_names = list(T["hand_joint_names"])
F = len(arm)
print("frames:", F)

model = pin.buildModelFromUrdf(urdf)
collision = pin.buildGeomFromUrdf(model, urdf, pin.GeometryType.COLLISION)
q0 = pin.neutral(model)
arm_qidx = [model.joints[model.getJointId(n)].idx_q for n in arm_names]
hand_qidx = [model.joints[model.getJointId(n)].idx_q if model.existJointName(n) else None
             for n in hand_names]


def frame_q(fr):
    q = q0.copy()
    for k, qi in enumerate(arm_qidx):
        q[qi] = arm[fr][k]
    for k, qi in enumerate(hand_qidx):
        if qi is not None:
            q[qi] = hand[fr][k]
    return q


viz = MeshcatVisualizer(model, collision, collision)
viz.initViewer(open=False)
viz.loadViewerModel(rootNodeName="robot")
viz.display(frame_q(0))
print("MESHCAT_URL:", viz.viewer.url())
sys.stdout.flush()

dt = 1.0 / 25.0
while True:
    for fr in range(F):
        viz.display(frame_q(fr))
        time.sleep(dt)
