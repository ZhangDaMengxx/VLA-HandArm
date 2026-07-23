"""测 nero_kin:FK(q_home) 是否法兰朝上 + IK 能否从扰动初值收敛回来。"""
import sys
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from nero_kin import NeroKin

kin = NeroKin(REPO / "assets/nero_description/urdf/nero_description.urdf")
q_home = np.array([1.2635, 0.9302, 2.6464, 1.7779, 1.0898, 0.6034, -0.6634])
T = kin.fk(q_home)
print("nq:", kin.model.nq)
print("FK(q_home) pos:", np.round(T[:3, 3], 3), " approach_z:", np.round(T[:3, 2], 3))
print("(期望 pos≈[0.1,-0.1,0.5], approach_z≈[0,0,1])")

q, ok = kin.ik(T, q_init=q_home + 0.2)   # 扰动初值,求回同一位姿
T2 = kin.fk(q)
print("IK ok:", ok)
print("recovered pos:", np.round(T2[:3, 3], 3), " approach_z:", np.round(T2[:3, 2], 3))
print("pos 误差(mm):", round(np.linalg.norm(T2[:3, 3] - T[:3, 3]) * 1000, 2))
