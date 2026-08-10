#!/usr/bin/env python3
"""诊断当前装配:输出法兰和手底座在世界系的位置,判断偏移。"""
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ARM_URDF = REPO / "assets/nero_description/urdf/nero_with_hand_flange_description.urdf"
ASSEMBLED = REPO / "sim/assets/nero_inspire_right.urdf"

def rpy_to_R(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def T(xyz, rpy):
    M = np.eye(4)
    M[:3, :3] = rpy_to_R(*rpy)
    M[:3, 3] = xyz
    return M

def parse_joints(urdf_path):
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0").split())] if o is not None else [0, 0, 0]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0").split())] if o is not None else [0, 0, 0]
        joints[j.get("name")] = dict(
            parent=j.find("parent").get("link"),
            child=j.find("child").get("link"),
            xyz=xyz, rpy=rpy)
    return joints

def fk_to_link(joints, target_link):
    """从 world/base_link FK 到目标 link。"""
    path = []
    current = target_link
    # 向上找到 base_link
    child_to_joint = {v["child"]: (k, v) for k, v in joints.items()}
    while current in child_to_joint:
        jname, jdata = child_to_joint[current]
        path.append((current, jdata))
        current = jdata["parent"]

    path.reverse()
    T_world = np.eye(4)
    for link, jdata in path:
        T_world = T_world @ T(np.array(jdata["xyz"]), np.array(jdata["rpy"]))
    return T_world

# 1) FK 到法兰(从臂 URDF)
arm_joints = parse_joints(ARM_URDF)
T_world_link8 = fk_to_link(arm_joints, "link8")
# link8 -> 法兰(from build_nero_inspire.py)
FLANGE_MOUNT_XYZ = np.array([0, 0, 0.016489])
FLANGE_MOUNT_RPY = np.array([0, 0, 1.570796])
T_world_flange = T_world_link8 @ T(FLANGE_MOUNT_XYZ, FLANGE_MOUNT_RPY)

print("=== 法兰在世界系的位姿(q=0,臂竖直) ===")
print(f"位置: x={T_world_flange[0,3]:.6f} y={T_world_flange[1,3]:.6f} z={T_world_flange[2,3]:.6f}")
print(f"Z轴方向(法兰圆段朝向): {T_world_flange[:3,2]}")
print()

# 2) FK 到手 base(从装配 URDF)
asm_joints = parse_joints(ASSEMBLED)
T_world_hand_base = fk_to_link(asm_joints, "base")

print("=== 手 base 在世界系的位姿 ===")
print(f"位置: x={T_world_hand_base[0,3]:.6f} y={T_world_hand_base[1,3]:.6f} z={T_world_hand_base[2,3]:.6f}")
print(f"Z轴方向: {T_world_hand_base[:3,2]}")
print()

# 3) FK 到 hand_base_link(实际几何所在)
T_world_hand_base_link = fk_to_link(asm_joints, "hand_base_link")
print("=== hand_base_link 在世界系的位姿 ===")
print(f"位置: x={T_world_hand_base_link[0,3]:.6f} y={T_world_hand_base_link[1,3]:.6f} z={T_world_hand_base_link[2,3]:.6f}")
print()

# 4) 偏移量
offset = T_world_hand_base[:3,3] - T_world_flange[:3,3]
print("=== 手 base 相对法兰的偏移 ===")
print(f"Δx={offset[0]*1000:.2f}mm  Δy={offset[1]*1000:.2f}mm  Δz={offset[2]*1000:.2f}mm")
print(f"总偏移: {np.linalg.norm(offset)*1000:.2f}mm")
print()

# 5) 当前 MOUNT 值(从 build_nero_inspire.py 读取)
print("=== 当前 MOUNT 值 ===")
import re
script = (REPO / "sim/build_nero_inspire.py").read_text()
mount_xyz = re.search(r'MOUNT_XYZ = "([^"]+)"', script).group(1)
mount_rpy = re.search(r'MOUNT_RPY = "([^"]+)"', script).group(1)
print(f"MOUNT_XYZ = \"{mount_xyz}\"")
print(f"MOUNT_RPY = \"{mount_rpy}\"")
