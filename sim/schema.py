"""锁定的两层数据 schema —— 整条采集→训练管线的基石。

规范层(canonical, 本体无关):真人第一视角采集的原始信息,可复用到任意臂+手。
本体层(embodiment, 每机器人一份):对规范层按 URDF 参数化 retarget 得到的 LeRobotDataset 帧。
换新臂手 = 换 URDF 重跑 canonical→embodiment,规范层一字不动。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

# ---------- 机器人本体常量(NERO 7-DoF + inspire 6 驱动关节)----------
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
HAND_ACTUATED = [
    "index_proximal_joint", "middle_proximal_joint", "ring_proximal_joint",
    "pinky_proximal_joint", "thumb_proximal_pitch_joint", "thumb_proximal_yaw_joint",
]
STATE_DIM = len(ARM_JOINTS) + len(HAND_ACTUATED)   # 13
ACTION_DIM = STATE_DIM                              # 绝对关节目标,同维


@dataclass
class CanonicalFrame:
    """规范层(本体无关)。采一次,喂多个机器人。"""
    ego_rgb: np.ndarray                     # (H,W,3) uint8   第一视角 RGB
    hand_keypoints: np.ndarray              # (21,3) float32  MANO 帧,米
    wrist_pose: np.ndarray                  # (4,4) float32   手腕 6-DoF(位置需深度)
    task: str                               # 语言指令
    timestamp: float
    ego_depth: Optional[np.ndarray] = None  # (H,W) float32 米  辅助,不喂基座 VLA


@dataclass
class EmbodimentFrame:
    """本体层(每机器人一份)= 一条 LeRobotDataset 记录。"""
    observation_images_ego: np.ndarray               # RGB → 喂 VLA
    observation_state: np.ndarray                    # (13,) [7臂+6手] 当前关节
    action: np.ndarray                               # (13,) [7臂+6手] 绝对关节目标
    task: str
    timestamp: float
    observation_images_depth: Optional[np.ndarray] = None  # 辅助,不喂基座 VLA


# LeRobotDataset 特征映射(Phase B-4 写盘时用)
LEROBOT_FEATURES = {
    "observation.images.ego":   {"dtype": "video", "shape": None, "names": ["h", "w", "c"]},
    "observation.images.depth": {"dtype": "video", "shape": None, "names": ["h", "w", "c"]},
    "observation.state":        {"dtype": "float32", "shape": (STATE_DIM,), "names": ARM_JOINTS + HAND_ACTUATED},
    "action":                   {"dtype": "float32", "shape": (ACTION_DIM,), "names": ARM_JOINTS + HAND_ACTUATED},
}


def canonical_to_embodiment(
    frame: CanonicalFrame,
    retarget_hand: Callable[[np.ndarray], np.ndarray],  # 手关键点 → 6 驱动关节
    arm_ik: Callable[[np.ndarray], np.ndarray],          # 手腕位姿 → 7 臂关节
    next_wrist_pose: Optional[np.ndarray] = None,
    next_hand_keypoints: Optional[np.ndarray] = None,
) -> EmbodimentFrame:
    """规范层一帧 → 本体层一帧。action 用"下一帧目标",无下一帧则同当前。"""
    q_arm = arm_ik(frame.wrist_pose)
    q_hand = retarget_hand(frame.hand_keypoints)
    state = np.concatenate([q_arm, q_hand]).astype(np.float32)

    if next_wrist_pose is not None and next_hand_keypoints is not None:
        a_arm = arm_ik(next_wrist_pose)
        a_hand = retarget_hand(next_hand_keypoints)
        action = np.concatenate([a_arm, a_hand]).astype(np.float32)
    else:
        action = state.copy()

    return EmbodimentFrame(
        observation_images_ego=frame.ego_rgb,
        observation_images_depth=frame.ego_depth,
        observation_state=state,
        action=action,
        task=frame.task,
        timestamp=frame.timestamp,
    )
