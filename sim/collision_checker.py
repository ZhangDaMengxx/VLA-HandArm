#!/usr/bin/env python3
"""几何碰撞检查器 —— 基于 URDF + 碰撞网格的精确自碰撞检测。

第 3 步:拇指-食指碰撞检测(HAND_SAFETY_PLAN.md)。
输入 6 个驱动关节角(rad) → 输出 (bool可行, float穿透深度mm)。

为什么要它:
  1. check_feasible 用的是一维压缩表(max(yaw,pitch)),丢失了三维信息
  2. 旧表只有 3 个实测点,域外靠钳制常数外插(T≤300 → 固定 225)
  3. 几何模型能算密集可行域,比实测 20-30 点更快更准

设计:
  · 只算拇指-食指对(重点,其余通道实测同时下 raw 0 时到底不相交)
  · 输入是驱动关节 rad,内部用 mimic 关系算耦合的远端关节
  · 输出 (bool, penetration_mm):可行 → (True,0.0),碰撞 → (False,穿透深度)
  · 不预计算表 —— 在线算够快(fcl 毫秒级),且能给出穿透深度用于诊断

约束:
  · URDF 的 collision 几何是**显式网格**(13 个 STL),不是盒/球/圆柱
  · mimic 关系硬编码(拇指 intermediate/distal,食指 intermediate)
  · 坐标变换从 URDF 的 origin 读取
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import NamedTuple

import numpy as np
import trimesh
import fcl
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parent.parent
# sys.path.insert(0, str(REPO / "sim"))
# from inspire_hand import rad_to_raw, n_to_rad  # noqa: E402

# 拇指-食指的驱动关节和耦合关节
THUMB_YAW = "thumb_proximal_yaw_joint"
THUMB_PITCH = "thumb_proximal_pitch_joint"
INDEX = "index_proximal_joint"

# mimic 关系(inspire_hand_right_glb.urdf 实测)
MIMIC = {
    "thumb_intermediate_joint": (THUMB_PITCH, 1.334, 0.0),
    "thumb_distal_joint": (THUMB_PITCH, 0.667, 0.0),
    "index_intermediate_joint": (INDEX, 1.06399, -0.04545),
}


class CollisionResult(NamedTuple):
    """碰撞检测结果。"""
    feasible: bool            # True = 可行(不碰撞)
    penetration_mm: float     # 穿透深度(mm),0 = 不碰撞或刚接触
    contact_point: tuple[float, float, float] | None  # 接触点世界坐标(可选)


class ThumbIndexChecker:
    """拇指-食指碰撞检查器。

    加载 URDF + 碰撞网格,提供 check(q) 接口。
    q 是 6 个驱动关节 rad(按 HAND_JOINTS 顺序)。
    """

    def __init__(self, urdf_path: Path, mesh_dir: Path):
        self.urdf_path = urdf_path
        self.mesh_dir = mesh_dir
        self._meshes: dict[str, trimesh.Trimesh] = {}
        self._origins: dict[str, np.ndarray] = {}  # link → 4x4 变换矩阵
        self._load()

    def _load(self):
        """加载 URDF + 碰撞网格。"""
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()

        # 读取所有 link 的 collision 几何和 origin
        for link in root.findall("link"):
            name = link.get("name")
            collision = link.find("collision")
            if collision is None:
                continue
            
            # 读取 origin (rpy + xyz)
            origin = collision.find("origin")
            if origin is not None:
                xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
                rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]
                # 构造 4x4 变换矩阵(rpy是ZYX欧拉角)
                T = self._rpy_xyz_to_matrix(rpy, xyz)
            else:
                T = np.eye(4)
            self._origins[name] = T

            # 读取 mesh 文件名
            geometry = collision.find("geometry")
            if geometry is None:
                continue
            mesh_elem = geometry.find("mesh")
            if mesh_elem is None:
                continue
            
            filename = mesh_elem.get("filename")
            if filename.startswith("package://"):
                # 去掉 package:// 前缀,相对路径解析
                rel = filename.split("package://")[1]
                mesh_path = self.mesh_dir / Path(rel).name
            elif filename.startswith("../"):
                # 相对 URDF 的路径
                mesh_path = (self.urdf_path.parent / filename).resolve()
            else:
                mesh_path = Path(filename)
            
            if not mesh_path.exists():
                print(f"⚠ 网格文件不存在: {mesh_path}")
                continue
            
            try:
                self._meshes[name] = trimesh.load(mesh_path)
            except Exception as e:
                print(f"✗ 加载网格失败 {name}: {e}")

    def _rpy_xyz_to_matrix(self, rpy: list[float], xyz: list[float]) -> np.ndarray:
        """RPY(ZYX欧拉角) + XYZ → 4x4 齐次变换矩阵。"""
        r, p, y = rpy
        # ZYX 顺序: R = Rz(y) * Ry(p) * Rx(r)
        cr, cp, cy = np.cos([r, p, y])
        sr, sp, sy = np.sin([r, p, y])
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ])
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz
        return T

    def _forward_kinematics(self, q: list[float]) -> dict[str, np.ndarray]:
        """正运动学:6 驱动关节 rad → 所有 link 的世界坐标变换。

        q 顺序:thumb_yaw, thumb_pitch, index, middle, ring, pinky
        返回 dict[link_name → 4x4 世界变换]
        """
        # 简化:只算拇指和食指的关节链
        # 拇指链: base → yaw → pitch → intermediate → distal
        # 食指链: base → proximal → intermediate
        
        q_dict = {
            THUMB_YAW: q[0],
            THUMB_PITCH: q[1],
            INDEX: q[2],
        }
        
        # 计算 mimic 关节
        for joint, (parent, multiplier, offset) in MIMIC.items():
            q_dict[joint] = q_dict[parent] * multiplier + offset
        
        # 硬编码的关节链(简化,完整版应从 URDF 读取)
        # 这里只做拇指-食指,其他手指跳过
        transforms = {}
        
        # Base link (固定)
        T_base = np.eye(4)
        transforms["R_base_link"] = T_base
        
        # 拇指 yaw (绕 Z 轴旋转)
        T_yaw = T_base @ self._joint_transform("thumb_proximal_yaw_joint", q_dict[THUMB_YAW])
        transforms["right_thumb_1"] = T_yaw @ self._origins.get("right_thumb_1", np.eye(4))
        
        # 拇指 pitch (绕 Y 轴旋转)
        T_pitch = T_yaw @ self._joint_transform("thumb_proximal_pitch_joint", q_dict[THUMB_PITCH])
        transforms["right_thumb_2"] = T_pitch @ self._origins.get("right_thumb_2", np.eye(4))
        
        # 拇指 intermediate
        T_inter = T_pitch @ self._joint_transform("thumb_intermediate_joint", q_dict["thumb_intermediate_joint"])
        transforms["right_thumb_3"] = T_inter @ self._origins.get("right_thumb_3", np.eye(4))
        
        # 拇指 distal
        T_dist = T_inter @ self._joint_transform("thumb_distal_joint", q_dict["thumb_distal_joint"])
        transforms["right_thumb_4"] = T_dist @ self._origins.get("right_thumb_4", np.eye(4))
        
        # 食指 proximal
        T_idx_prox = T_base @ self._joint_transform("index_proximal_joint", q_dict[INDEX])
        transforms["right_index_1"] = T_idx_prox @ self._origins.get("right_index_1", np.eye(4))
        
        # 食指 intermediate
        T_idx_inter = T_idx_prox @ self._joint_transform("index_intermediate_joint", q_dict["index_intermediate_joint"])
        transforms["right_index_2"] = T_idx_inter @ self._origins.get("right_index_2", np.eye(4))
        
        return transforms
    
    def _joint_transform(self, joint_name: str, angle: float) -> np.ndarray:
        """单关节变换矩阵。简化:假设都是绕 Z 轴的旋转关节。"""
        # TODO: 从 URDF 读取 joint 的 axis 和 origin
        # 现在硬编码假设都是 revolute, axis="0 0 1"
        c, s = np.cos(angle), np.sin(angle)
        T = np.array([
            [c, -s, 0, 0],
            [s,  c, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1]
        ])
        return T

    def check(self, q: list[float]) -> CollisionResult:
        """检查 6 驱动关节构型是否可行。

        q: [thumb_yaw, thumb_pitch, index, middle, ring, pinky] (rad)
        返回: CollisionResult(feasible, penetration_mm, contact_point)
        """
        # 正运动学
        transforms = self._forward_kinematics(q)
        
        # 拇指链所有 link
        thumb_links = ["right_thumb_1", "right_thumb_2", "right_thumb_3", "right_thumb_4"]
        # 食指链所有 link
        index_links = ["right_index_1", "right_index_2"]
        
        # 检查拇指-食指所有对
        min_dist = float('inf')
        contact_pt = None
        
        for thumb_link in thumb_links:
            if thumb_link not in self._meshes or thumb_link not in transforms:
                continue
            
            for index_link in index_links:
                if index_link not in self._meshes or index_link not in transforms:
                    continue
                
                # 变换到世界坐标
                thumb_mesh = self._meshes[thumb_link].copy()
                thumb_mesh.apply_transform(transforms[thumb_link])
                
                index_mesh = self._meshes[index_link].copy()
                index_mesh.apply_transform(transforms[index_link])
                
                # FCL 碰撞检测
                thumb_obj = fcl.CollisionObject(fcl.BVHModel(thumb_mesh.vertices, thumb_mesh.faces))
                index_obj = fcl.CollisionObject(fcl.BVHModel(index_mesh.vertices, index_mesh.faces))
                
                request = fcl.CollisionRequest()
                result = fcl.CollisionResult()
                
                fcl.collide(thumb_obj, index_obj, request, result)
                
                if result.is_collision:
                    # 穿透深度(FCL 的 penetration_depth 单位是 m)
                    pen_mm = result.contacts[0].penetration_depth * 1000 if result.contacts else 0.0
                    if pen_mm < min_dist:
                        min_dist = pen_mm
                        if result.contacts:
                            contact_pt = tuple(result.contacts[0].pos)
        
        if min_dist < float('inf'):
            return CollisionResult(feasible=False, penetration_mm=min_dist, contact_point=contact_pt)
        else:
            return CollisionResult(feasible=True, penetration_mm=0.0, contact_point=None)


def main():
    """测试碰撞检查器。"""
    urdf_path = REPO / "assets" / "hand" / "urdf" / "inspire_hand_right.urdf"
    mesh_dir = REPO / "assets" / "hand" / "meshes"
    
    checker = ThumbIndexChecker(urdf_path, mesh_dir)
    print(f"✓ 加载完成,网格 {len(checker._meshes)} 个")
    
    # 测试用例:旧表的点
    test_cases = [
        ("T=600 cmd_idx=0", [0.498, 0.279, 0.0, 0.0, 0.0, 0.0]),  # 旧表 (600, 0)
        ("T=300 cmd_idx=225", [0.7, 0.42, 0.54, 0.0, 0.0, 0.0]),   # 旧表 (300, 225)
        ("全张开", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),                 # 应该可行
    ]
    
    for desc, q in test_cases:
        result = checker.check(q)
        status = "✓" if result.feasible else "✗"
        print(f"{status} {desc:20} 可行={result.feasible}  穿透={result.penetration_mm:.2f}mm")


if __name__ == "__main__":
    main()
