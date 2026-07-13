"""手势演示(阶段 A 成品):在 NERO+inspire 装配上循环展示手势预设,MeshCat 动画。
臂保持 home 姿态(手指朝上),手在预设之间平滑过渡。
用法: python sim/gesture_demo.py
"""
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

sys.path.insert(0, str(Path(__file__).parent))
from gestures import GESTURES

REPO = Path("/home/zhang123/ros2_ws/lerobotTest")
urdf = str(REPO / "sim/assets/nero_inspire_right.urdf")

Q_HOME_ARM = {
    "joint1": 1.2635, "joint2": 0.9302, "joint3": 2.6464, "joint4": 1.7779,
    "joint5": 1.0898, "joint6": 0.6034, "joint7": -0.6634,
}

model = pin.buildModelFromUrdf(urdf)
collision = pin.buildGeomFromUrdf(model, urdf, pin.GeometryType.COLLISION)

q0 = pin.neutral(model)
for n, v in Q_HOME_ARM.items():
    if model.existJointName(n):
        q0[model.joints[model.getJointId(n)].idx_q] = v


def gesture_vec(gdict):
    q = q0.copy()
    for jn, val in gdict.items():
        if model.existJointName(jn):
            q[model.joints[model.getJointId(jn)].idx_q] = val
    return q


ORDER = ["open", "point", "victory", "thumbs_up", "ok", "fist", "open"]
seq = [gesture_vec(GESTURES[g]) for g in ORDER]

viz = MeshcatVisualizer(model, collision, collision)
viz.initViewer(open=False)
viz.loadViewerModel(rootNodeName="robot")
viz.display(seq[0])
print("gestures:", ORDER)
print("MESHCAT_URL:", viz.viewer.url())
sys.stdout.flush()

while True:
    for a, b in zip(seq[:-1], seq[1:]):
        for t in np.linspace(0.0, 1.0, 25):
            viz.display(a * (1.0 - t) + b * t)
            time.sleep(1.0 / 30.0)
        time.sleep(0.7)  # 停顿展示
