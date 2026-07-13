"""inspire 手势预设:6 个驱动关节 → 计算 mimic → 完整 12 关节 dict。
mimic 比例来自 inspire URDF:
  finger_intermediate = 1.06399 * proximal - 0.04545
  thumb_intermediate  = 1.334 * thumb_pitch
  thumb_distal        = 0.667 * thumb_pitch
关节范围(rad):手指近端 [0,1.47];拇指 pitch [0,0.6];拇指 yaw [0,1.308]。
"""


def _finger_inter(p):
    return max(0.0, 1.06399 * p - 0.04545)


def full12(index, middle, ring, pinky, thumb_pitch, thumb_yaw):
    """由 6 驱动关节算出完整 12 关节 dict(含 mimic)。"""
    return {
        "index_proximal_joint": index,   "index_intermediate_joint": _finger_inter(index),
        "middle_proximal_joint": middle, "middle_intermediate_joint": _finger_inter(middle),
        "ring_proximal_joint": ring,     "ring_intermediate_joint": _finger_inter(ring),
        "pinky_proximal_joint": pinky,   "pinky_intermediate_joint": _finger_inter(pinky),
        "thumb_proximal_yaw_joint": thumb_yaw,
        "thumb_proximal_pitch_joint": thumb_pitch,
        "thumb_intermediate_joint": 1.334 * thumb_pitch,
        "thumb_distal_joint": 0.667 * thumb_pitch,
    }


CURL = 1.35  # 手指近端弯曲量(接近上限 1.47)

# 第一版预设,可按视觉微调
GESTURES = {
    "open":      full12(0.0,  0.0,  0.0,  0.0,  0.00, 0.0),   # 张开
    "fist":      full12(CURL, CURL, CURL, CURL, 0.55, 0.30),  # 握拳
    "point":     full12(0.0,  CURL, CURL, CURL, 0.50, 0.20),  # 食指指点
    "victory":   full12(0.0,  0.0,  CURL, CURL, 0.50, 0.20),  # 食指+中指(V)
    "thumbs_up": full12(CURL, CURL, CURL, CURL, 0.00, 0.00),  # 点赞(拇指伸)
    "ok":        full12(0.70, 0.0,  0.0,  0.0,  0.55, 1.00),  # OK(拇指靠食指)
}
