#!/usr/bin/env python3
"""重新计算法兰→新手 base 的挂接量，并在地面真值装配体上验证。

原理:
旧 URDF 的 base→hand_base_link joint origin rpy=(-1.57079, 0, 3.14159) 即 R_old
新 URDF 的 base→hand_base_link joint origin rpy=(0, 0, 0) 即单位阵
=> T_oldbase_newbase = (R_old, 0)  (旋转差,平移零,因 xyz 都是 0 0 0)

原挂接 T_flange_oldbase 是装配体反解出来的 = (MOUNT_XYZ, MOUNT_RPY)
新挂接 T_flange_newbase = T_flange_oldbase @ T_oldbase_newbase

最后用装配体 STL 的手掌区域验证:新手 FK(base_link)->world 后跑 ICP,残差应≤0.4mm。
"""
import numpy as np
import trimesh
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from icp import register

REPO = Path(__file__).resolve().parents[2]
ASSY = REPO / "assets/nero_description/meshes/nero_RH56DF.stl"
NEW_HAND = REPO / "assets/inspire_hand/inspire_hand_right.urdf"

# 08-04 装配体反解出的旧挂接量(for dex-urdf 手)
OLD_MOUNT_XYZ = np.array([0.000042, 0.005962, 0.002158])
OLD_MOUNT_RPY = np.array([0, 0, 1.570796])

# 旧手 base->hand_base_link joint origin (dex-urdf 约定)
OLD_BASE_JOINT_RPY = np.array([-1.57079, 0, 3.14159])


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


def T_to_urdf(M):
    """4x4 -> (xyz, rpy) for URDF origin 属性。"""
    xyz = M[:3, 3]
    R = M[:3, :3]
    # rpy: R = Rz(y) @ Ry(p) @ Rx(r)  解析反解(仅适用 |cos(p)| > 1e-6)
    if abs(R[2, 0]) < 0.999:
        p = np.arcsin(-R[2, 0])
        r = np.arctan2(R[2, 1] / np.cos(p), R[2, 2] / np.cos(p))
        y = np.arctan2(R[1, 0] / np.cos(p), R[0, 0] / np.cos(p))
    else:
        y = 0
        if R[2, 0] < 0:
            p = np.pi / 2
            r = np.arctan2(R[0, 1], R[0, 2])
        else:
            p = -np.pi / 2
            r = np.arctan2(-R[0, 1], -R[0, 2])
    return xyz, np.array([r, p, y])


# 1) 计算新挂接量
R_old = rpy_to_R(*OLD_BASE_JOINT_RPY)
T_oldbase_newbase = T(np.zeros(3), OLD_BASE_JOINT_RPY)  # (R_old, 0)

T_flange_oldbase = T(OLD_MOUNT_XYZ, OLD_MOUNT_RPY)
T_flange_newbase = T_flange_oldbase @ T_oldbase_newbase

new_xyz, new_rpy = T_to_urdf(T_flange_newbase)

print("=== 旧挂接量(for dex-urdf 手) ===")
print(f"MOUNT_XYZ = {OLD_MOUNT_XYZ}")
print(f"MOUNT_RPY = {OLD_MOUNT_RPY}")
print()
print("=== 坐标系变换(旧 base->新 base) ===")
print("R_oldbase_newbase =")
print(R_old)
print("t = 0(joint origin xyz 都是 0)")
print()
print("=== 新挂接量(for 官方 inspire 手) ===")
print(f"MOUNT_XYZ = \"{new_xyz[0]:.6f} {new_xyz[1]:.6f} {new_xyz[2]:.6f}\"")
print(f"MOUNT_RPY = \"{new_rpy[0]:.6f} {new_rpy[1]:.6f} {new_rpy[2]:.6f}\"")
print()

# 2) 重新 ICP:新手掌 mesh → 装配体手掌区,全自由度(R+t)
print("=== 重新 ICP:新手 → 装配体手掌(全自由度) ===")

# 装配体手掌区(米)
assy_m = trimesh.load_mesh(str(ASSY))
assy_m.vertices *= 0.001
mask = (assy_m.vertices[:, 2] >= 0.34) & (assy_m.vertices[:, 2] <= 0.42)
palm_assy_world = assy_m.vertices[mask]
print(f"装配体手掌区:z∈[0.34,0.42]m => {mask.sum()} 点")

# 新手 hand_base_link mesh (在其局部系)
import xml.etree.ElementTree as ET

hand_root = ET.parse(NEW_HAND).getroot()
for link in hand_root.findall("link"):
    if link.get("name") == "hand_base_link":
        vis = link.find("visual")
        mesh_el = vis.find("geometry/mesh")
        mesh_fn = mesh_el.get("filename")
        origin = vis.find("origin")
        if origin is not None:
            vis_xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            vis_rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        else:
            vis_xyz, vis_rpy = [0, 0, 0], [0, 0, 0]
        break

mesh_path = (NEW_HAND.parent / mesh_fn).resolve()
hand_mesh = trimesh.load_mesh(str(mesh_path))
print(f"新手 hand_base_link mesh: {mesh_path.name}, {len(hand_mesh.vertices)} 点")

# mesh 局部 -> hand_base_link 系
T_hbl_mesh = T(np.array(vis_xyz), np.array(vis_rpy))
hand_in_hbl = trimesh.transformations.transform_points(hand_mesh.vertices, T_hbl_mesh)

# base -> hand_base_link joint origin(新 URDF rpy=0)
hand_base_joint = next(j for j in hand_root.findall("joint") if j.find("child").get("link") == "hand_base_link")
hb_o = hand_base_joint.find("origin")
hb_xyz = [float(v) for v in hb_o.get("xyz", "0 0 0").split()]
hb_rpy = [float(v) for v in hb_o.get("rpy", "0 0 0").split()]
T_base_hbl = T(np.array(hb_xyz), np.array(hb_rpy))

# 新手掌在**新 base 系**下的坐标
hand_in_newbase = trimesh.transformations.transform_points(hand_in_hbl, T_base_hbl)

# ICP: 找 T 使得 T @ hand_in_newbase 对齐到 palm_assy_world
# T 就是 T_world_newbase,分解后得 T_flange_newbase = inv(T_world_flange) @ T
print("\nICP 新手掌(在新 base 系) → 装配体手掌(world)...")
R_icp, t_icp, rmse, q75, md = register(hand_in_newbase, palm_assy_world, n_src=6000, keep=0.75)
print(f"  rmse={rmse * 1000:.3f}mm  q75={q75 * 1000:.3f}mm  mean={md * 1000:.3f}mm")

T_world_newbase_icp = np.eye(4)
T_world_newbase_icp[:3, :3] = R_icp
T_world_newbase_icp[:3, 3] = t_icp

# FK 到 link8 + 法兰,得 T_world_flange
ARM_URDF = REPO / "assets/nero_description/urdf/nero_with_hand_flange_description.urdf"
FLANGE_MOUNT_XYZ = np.array([0, 0, 0.016489])
FLANGE_MOUNT_RPY = np.array([0, 0, 1.570796])

arm_root = ET.parse(ARM_URDF).getroot()
joints = {}
for j in arm_root.findall("joint"):
    o = j.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0").split())] if o is not None else [0, 0, 0]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0").split())] if o is not None else [0, 0, 0]
    joints[j.get("name")] = dict(
        parent=j.find("parent").get("link"), child=j.find("child").get("link"),
        xyz=xyz, rpy=rpy)

T_world_link8 = np.eye(4)
for lname in ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "link7", "link8"]:
    j = next((v for v in joints.values() if v["child"] == lname), None)
    if j:
        T_world_link8 = T_world_link8 @ T(np.array(j["xyz"]), np.array(j["rpy"]))

T_world_flange = T_world_link8 @ T(FLANGE_MOUNT_XYZ, FLANGE_MOUNT_RPY)

# T_flange_newbase = inv(T_world_flange) @ T_world_newbase_icp
T_flange_newbase_final = np.linalg.inv(T_world_flange) @ T_world_newbase_icp

new_xyz, new_rpy = T_to_urdf(T_flange_newbase_final)

print("\n=== 重新 ICP 解出的挂接量(for 官方 inspire 手) ===")
print(f"MOUNT_XYZ = \"{new_xyz[0]:.6f} {new_xyz[1]:.6f} {new_xyz[2]:.6f}\"")
print(f"MOUNT_RPY = \"{new_rpy[0]:.6f} {new_rpy[1]:.6f} {new_rpy[2]:.6f}\"")
print()

# 验证:用新挂接量 FK,再 ICP 残差
T_world_newbase_check = T_world_flange @ T_flange_newbase_final
hand_world_check = trimesh.transformations.transform_points(hand_in_newbase, T_world_newbase_check)
R2, t2, rmse2, q75_2, md2 = register(hand_world_check, palm_assy_world, n_src=6000, keep=0.75)
print(f"验证 FK+ICP: rmse={rmse2 * 1000:.3f}mm  q75={q75_2 * 1000:.3f}mm")
print(f"状态:{'✓ 通过' if rmse2 < 0.0008 else '✗ 残差仍过大'} ({'≤0.8mm 可接受' if rmse2 < 0.0008 else '>0.8mm'})")
print()
if rmse2 < 0.0008:
    print("下一步:把新挂接量写进 sim/build_nero_inspire.py 的 MOUNT_XYZ / MOUNT_RPY,重新生成 URDF。")
else:
    print("需要检查:① 新手 mesh 是否与装配体 STL 同源;② 装配体手掌区域选择是否合理。")
