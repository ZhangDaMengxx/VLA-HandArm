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
    # --- 稳定化 / 平滑(见 wrist_stabilize.py + build_robot_traj)---
    gate_deg: float = 8.0
    oop_alpha: float = 0.4
    savgol_win: int = 11
    savgol_poly: int = 3
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
    arm_urdf=REPO / "assets/nero/nero_description.urdf",
    ee_frame="link7",
    q_home=np.array([1.2635, 0.9302, 2.6464, 1.7779, 1.0898, 0.6034, -0.6634]),
    arm_joint_names=ARM_JOINTS,
)

SPECS = {s.name: s for s in [NERO_INSPIRE]}


def get_spec(name: str) -> RobotSpec:
    if name not in SPECS:
        raise SystemExit(f"未知本体 '{name}';可选: {list(SPECS)}")
    return SPECS[name]
