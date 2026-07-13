"""B-2: 人手手腕 6-DoF 位姿估计。

朝向:来自 MediaPipe(可靠,单目即可)。
位置:可插拔深度后端。
  - 现在(无 Femto):单目手掌尺度启发式 Z = f * L_metric / L_pixels(近似)。
  - Femto 到手后:传入 depth_lookup(u,v)->Z 用真实 ToF 深度(度量准确),其余不变。
输出 4x4 手腕位姿(相机系)。注意:相机系→机器人基座系的对齐是 B-3 的活。
"""
import numpy as np


def wrist_orientation(mediapipe_wrist_rot, operator2mano):
    """3x3:与 detect_from_video 的 wrist_rot 一致(MANO/机器人基 → 相机系)。"""
    return mediapipe_wrist_rot @ operator2mano


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
                        img_shape, focal_px=None, depth_lookup=None):
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
    R = wrist_orientation(mediapipe_wrist_rot, operator2mano)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T
