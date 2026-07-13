"""A-2 检视台:把真人手视频 retarget 出的 inspire 轨迹回放到 NERO+inspire 装配。
臂保持 home 姿态(法兰朝上),手指由真实轨迹驱动,MeshCat 循环动画。
用法: python sim/replay_assembly.py
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
traj_path = REPO / "sim/out/hand_traj.pkl"

Q_HOME_ARM = {
    "joint1": 1.2635, "joint2": 0.9302, "joint3": 2.6464, "joint4": 1.7779,
    "joint5": 1.0898, "joint6": 0.6034, "joint7": -0.6634,
}

with open(traj_path, "rb") as f:
    T = pickle.load(f)
data = np.asarray(T["data"])
names = list(T["joint_names"])
print("traj frames:", data.shape)

model = pin.buildModelFromUrdf(urdf)
collision = pin.buildGeomFromUrdf(model, urdf, pin.GeometryType.COLLISION)

q0 = pin.neutral(model)
for n, v in Q_HOME_ARM.items():
    if model.existJointName(n):
        q0[model.joints[model.getJointId(n)].idx_q] = v

hand_qidx = [model.joints[model.getJointId(jn)].idx_q if model.existJointName(jn) else None
             for jn in names]
print("mapped hand joints:", sum(i is not None for i in hand_qidx), "/", len(names))

viz = MeshcatVisualizer(model, collision, collision)
viz.initViewer(open=False)
viz.loadViewerModel(rootNodeName="robot")
viz.display(q0)
print("MESHCAT_URL:", viz.viewer.url())
sys.stdout.flush()

dt = 1.0 / 25.0
while True:
    for fr in range(len(data)):
        q = q0.copy()
        for k, qi in enumerate(hand_qidx):
            if qi is not None:
                q[qi] = data[fr, k]
        viz.display(q)
        time.sleep(dt)
