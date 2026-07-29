"""B-2: 人手手腕 6-DoF 位姿估计。

朝向:来自 MediaPipe(可靠,单目即可)。
位置:可插拔深度后端。
  - 现在(无 Femto):单目手掌尺度启发式 Z = f * L_metric / L_pixels(近似)。
  - Femto 到手后:传入 depth_lookup(u,v)->Z 用真实 ToF 深度(度量准确),其余不变。
输出 4x4 手腕位姿(相机系)。注意:相机系→机器人基座系的对齐是 B-3 的活。
"""
import numpy as np


def mano_wrist_orientation(mediapipe_wrist_rot, operator2mano):
    """3x3:MANO/retargeting wrist frame -> camera frame.

    This is the historical orientation used together with `joint_pos =
    keypoints @ mediapipe_wrist_rot @ operator2mano`. It exists to satisfy the
    dex-retargeting/MANO local coordinate convention for hand keypoints.
    """
    return mediapipe_wrist_rot @ operator2mano


def physical_wrist_orientation(mediapipe_wrist_rot, operator2mano):
    """3x3:physical wrist frame -> camera frame.

    Physical convention used by the arm:
      X = palm normal
      Z = wrist -> middle MCP direction
      Y = right-handed completion, Y = Z x X

    The sign of the palm normal comes from the existing right/left hand
    operator convention, but this frame is intentionally separate from the
    hand-keypoint MANO/local frame used by dex-retargeting.
    """
    mano_R = mano_wrist_orientation(mediapipe_wrist_rot, operator2mano)
    x = mano_R[:, 0].astype(float)
    z = mano_R[:, 2].astype(float)
    x /= np.linalg.norm(x) + 1e-12
    z = z - x * float(np.dot(x, z))
    z /= np.linalg.norm(z) + 1e-12
    y = np.cross(z, x)
    y /= np.linalg.norm(y) + 1e-12
    return np.stack([x, y, z], axis=1)


def depth_wrist_orientation(palm_pts, mono_R, min_planarity=0.1):
    """用深度反投的掌骨点拟合手腕 physical 朝向(相机系)。

    palm_pts: (5,3) 深度反投的 [腕, 食指MCP, 中指MCP, 环指MCP, 小指MCP](米,相机系)。
    mono_R:   (3,3) 单目 physical 朝向,仅借它给平面法向定符号(手心/手背歧义)。
    返回 (3x3, resid_mm) 或 (None, resid_mm)——点接近共线等退化时返回 None,调用方回退单目。

    约定与 physical_wrist_orientation 一致:X=手掌法向, Z=腕->中指MCP, Y=Z x X。
    法向来自 5 点 SVD 平面拟合(深度硬值),Z 来自深度反投的腕->中指MCP 向量。
    """
    pts = np.asarray(palm_pts, dtype=float)
    c = pts.mean(axis=0)
    _, S, Vt = np.linalg.svd(pts - c)
    resid_mm = float(np.abs((pts - c) @ Vt[-1]).mean()) * 1000.0
    if S[0] < 1e-9 or S[1] / S[0] < min_planarity:      # 点接近共线,法向不唯一
        return None, resid_mm
    n = Vt[-1] / (np.linalg.norm(Vt[-1]) + 1e-12)
    if float(np.dot(n, np.asarray(mono_R)[:, 0])) < 0.0:  # 符号对齐单目手掌法向
        n = -n
    x = n
    z = pts[2] - pts[0]                        # 腕(palm[0]) -> 中指MCP(kp9=palm[2])
    z = z - x * float(np.dot(x, z))
    nz = np.linalg.norm(z)
    if nz < 1e-9:
        return None, resid_mm
    z /= nz
    y = np.cross(z, x)
    y /= np.linalg.norm(y) + 1e-12
    return np.stack([x, y, z], axis=1), resid_mm


# Backward-compatible name for older callers.
wrist_orientation = mano_wrist_orientation


def hand_scale_depth(joint_pos, kp2d_px, focal_px, ref=(0, 9)):
    """单目手掌尺度估深:Z = f * L_metric / L_pixels。
    joint_pos: 21x3 米(保距);kp2d_px: 21x2 像素;ref: (腕, 中指MCP)。"""
    a, b = ref
    L_m = np.linalg.norm(joint_pos[a] - joint_pos[b])
    L_px = np.linalg.norm(kp2d_px[a] - kp2d_px[b]) + 1e-6
    return focal_px * L_m / L_px


def backproject(u, v, Z, focal_px, cx, cy):
    return np.array([(u - cx) * Z / focal_px, (v - cy) * Z / focal_px, Z])


def estimate_wrist_pose(joint_pos, kp2d_px, mediapipe_wrist_rot, operator2mano,
                        img_shape, focal_px=None, depth_lookup=None,
                        wrist_frame="physical"):
    """返回 4x4 手腕位姿(相机系)。
    depth_lookup: callable(u,v)->Z(米)。给了就用它(Femto ToF);否则单目尺度启发式。"""
    H, W = img_shape[:2]
    if focal_px is None:
        focal_px = 0.87 * W          # ~60° FOV 近似焦距(无标定时)
    cx, cy = W / 2.0, H / 2.0
    u, v = float(kp2d_px[0][0]), float(kp2d_px[0][1])   # 腕关键点像素
    if depth_lookup is not None:
        Z = float(depth_lookup(u, v))                   # Femto: 真实 ToF 深度
    else:
        Z = hand_scale_depth(joint_pos, kp2d_px, focal_px)  # 单目近似
    pos = backproject(u, v, Z, focal_px, cx, cy)
    if wrist_frame == "physical":
        R = physical_wrist_orientation(mediapipe_wrist_rot, operator2mano)
    elif wrist_frame == "mano":
        R = mano_wrist_orientation(mediapipe_wrist_rot, operator2mano)
    else:
        raise ValueError(f"unknown wrist_frame {wrist_frame!r}, expected physical/mano")
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T
