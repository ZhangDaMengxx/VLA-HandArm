"""RobotSpec:把「一台机器人」需要的全部参数收成一个规格对象。

规范层(canonical_ds)本体无关;`derive_embodiment.py` 拿 canonical_ds + 一个 RobotSpec →
按这台机器人的 URDF/重定向配置派生出它的 LeRobotDataset。**换机器人 = 加一个 RobotSpec**,
采集与规范层一字不动。这是「一次采集、多本体复用」的落地点。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List

import numpy as np

from schema import ARM_JOINTS, HAND_ACTUATED

REPO = Path(__file__).resolve().parents[1]


@dataclass
class RobotSpec:
    name: str
    # --- 手:dex-retargeting ---
    retarget_cfg: Path              # 重定向配置(.yml)
    urdf_dir: Path                  # 配置里 urdf_path 的解析根
    hand_actuated: List[str]        # 进入 state/action 的驱动手关节(retarget 12 输出的子集)
    # --- 臂:IK ---
    arm_urdf: Path
    ee_frame: str
    q_home: np.ndarray              # (nq,) home 姿态(法兰朝向 + 位置锚点)
    arm_joint_names: List[str]      # 进入 state/action 的臂关节名
    ee_frame_correction_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    arm_position_mode: str = "fixed"     # fixed=稳定默认; relative=跟随 wrist_pose 相对位移(需先做轴向/外参验证)
    arm_position_gain: float = 1.0
    arm_position_limit_m: float = 0.05   # 相对 home 的最大末端位移半径,避免视觉跳点甩飞 IK
    # --- 稳定化 / 平滑(见 wrist_stabilize.py + build_robot_traj)---
    gate_deg: float = 25.0  # 每帧旋转超过 8 度会被限幅，快转很容易被截掉
    oop_alpha: float = 1.0  #出平面旋转只保留 40%，手心/手背翻转可能被压小
    savgol_win: int = 9   #11 帧平滑，动作会变慢、峰值会被抹掉
    savgol_poly: int = 3
    """
        - savgol_poly=2：用二次曲线，适合保留加速/减速动作，比线性平滑自然。
        - savgol_poly=3：能保留更多细节，但对噪声更敏感。
        - savgol_poly=1：更像局部直线平均，动作会更钝。
    """


    k_null: float = 0.0
    repo_id: str = ""

    def __post_init__(self):
        if not self.repo_id:
            self.repo_id = f"local/{self.name}_handdemo"

    @property
    def out_root(self) -> Path:
        return REPO / f"sim/out/lerobot_ds_{self.name}"


# ---------- 已支持的本体 ----------
NERO_INSPIRE = RobotSpec(
    name="nero_inspire",
    retarget_cfg=REPO / "configs/inspire_hand_right_local.yml",
    urdf_dir=REPO / "assets",
    hand_actuated=HAND_ACTUATED,
    arm_urdf=REPO / "assets/nero_description/urdf/nero_description.urdf",
    ee_frame="link7",
    # 视频手腕坐标系 -> NERO 末端坐标系的固定补偿。
    # 当前视频回放中手腕轴朝 +Y 横向;绕 X +90 deg 后映射到 +Z,手心更接近朝向相机。
    ee_frame_correction_rpy=(np.pi / 2.0, np.pi / 2.0, 0.0),
    q_home=np.array([1.2635, 0.9302, 2.6464, 1.7779, 1.0898, 0.6034, -0.6634]),
    arm_joint_names=ARM_JOINTS,
)

SPECS = {s.name: s for s in [NERO_INSPIRE]}


def get_spec(name: str) -> RobotSpec:
    if name not in SPECS:
        raise SystemExit(f"未知本体 '{name}';可选: {list(SPECS)}")
    return SPECS[name]
