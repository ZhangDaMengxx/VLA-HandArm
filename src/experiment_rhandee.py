"""实验:把深度-单目的 18.5° 常量偏置吸收进 R_hand_ee,看 IK 能否从 38 恢复。

R_hand_ee_new = R_rel_mean^T @ R_hand_ee_old,使深度朝向经此映射后与单目路径同目标。
恢复到接近 550 = 崩溃纯是坐标系偏置(深度朝向可用,只需重推 R_hand_ee);
只恢复一部分 = 每帧 11° 抖动是真的,深度 Z 轴需再处理。
"""
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_specs import get_spec
from derive_embodiment import load_canonical, solve_arm

# measure_orient_offset.py 实测的平均 R_rel(18.5°,轴见下)
MEAN_ROTVEC = np.radians(18.5) * np.array([-0.06196926, 0.38198973, 0.92208657])
R_rel_mean = Rot.from_rotvec(MEAN_ROTVEC).as_matrix()

spec = get_spec("nero_inspire_rgbd")
R_old = np.asarray(spec.R_hand_ee, dtype=np.float64).reshape(3, 3)
spec.R_hand_ee = R_rel_mean.T @ R_old      # 吸收常量偏置

kps, wps, egos, fps = load_canonical()
print(f"canonical: {len(kps)} 帧 (当前为深度朝向)")
print("R_hand_ee 已吸收 18.5° 偏置,重跑 metric IK:")
solve_arm(wps, spec)   # 内部打印 IK success
