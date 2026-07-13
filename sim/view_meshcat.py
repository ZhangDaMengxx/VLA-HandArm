"""MeshCat 浏览器查看器(WSL 跑、Windows 浏览器看)。
Pinocchio 加载装配 URDF 的碰撞模型;默认显示"法兰朝上"的 home 姿态(手指竖直朝上)。
用法: python sim/view_meshcat.py [urdf_path]
"""
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

REPO = Path("/home/zhang123/ros2_ws/lerobotTest")
urdf = sys.argv[1] if len(sys.argv) > 1 else str(REPO / "sim/assets/nero_inspire_right.urdf")

# NERO 7 关节初始姿态:法兰伸出轴朝世界 +z(IK 求得),平贴手指即竖直朝上
Q_HOME_ARM = {
    "joint1": 1.2635, "joint2": 0.9302, "joint3": 2.6464, "joint4": 1.7779,
    "joint5": 1.0898, "joint6": 0.6034, "joint7": -0.6634,
}

model = pin.buildModelFromUrdf(urdf)
collision = pin.buildGeomFromUrdf(model, urdf, pin.GeometryType.COLLISION)

q = pin.neutral(model)
for name, val in Q_HOME_ARM.items():
    if model.existJointName(name):
        q[model.joints[model.getJointId(name)].idx_q] = val

data = model.createData()
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)


def ftrans(n):
    return np.round(data.oMf[model.getFrameId(n)].translation, 4).tolist() if model.existFrame(n) else None


print("home pose frame check (z 越大越靠上):")
for f in ["link7", "thumb_tip", "index_tip", "pinky_tip"]:
    print(f"  {f:10s} {ftrans(f)}")

viz = MeshcatVisualizer(model, collision, collision)
viz.initViewer(open=False)
viz.loadViewerModel(rootNodeName="robot")
viz.display(q)
print("MESHCAT_URL:", viz.viewer.url())
sys.stdout.flush()

while True:
    time.sleep(3600)
