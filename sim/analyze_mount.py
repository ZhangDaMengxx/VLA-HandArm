"""分析 q=0 时 link7 坐标系与手的朝向,给出安装变换标定所需数据。"""
from pathlib import Path
import numpy as np
import pinocchio as pin

REPO = Path("/home/zhang123/ros2_ws/lerobotTest")
urdf = str(REPO / "sim/assets/nero_inspire_right.urdf")
model = pin.buildModelFromUrdf(urdf)
data = model.createData()
q = pin.neutral(model)
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)


def M(name):
    return data.oMf[model.getFrameId(name)]


link7 = M("link7")
hb = M("hand_base_link")
mtip = M("middle_tip").translation
itip = M("index_tip").translation
ptip = M("pinky_tip").translation

Rl = link7.rotation
finger_world = mtip - hb.translation
finger_world /= np.linalg.norm(finger_world)
palm_world = np.cross(itip - ptip, finger_world)
palm_world /= np.linalg.norm(palm_world)

np.set_printoptions(precision=3, suppress=True)
print("link7 axes in world (cols x,y,z):\n", Rl)
print("link7 z (tool approach, world):", Rl[:, 2])
print("finger_dir (world):", finger_world)
print("palm_normal (world):", palm_world)
print("--- expressed in link7 local frame ---")
print("finger_dir in link7:", Rl.T @ finger_world)
print("palm_normal in link7:", Rl.T @ palm_world)
print("hand-base R in link7 frame:\n", Rl.T @ hb.rotation)
