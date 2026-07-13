"""用 NeroKinematics 逆解求初始姿态:法兰(link7)伸出轴朝世界 +z,
平贴安装的手指即竖直朝上。多随机初值重启提高求解率。打印 q_home。"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))   # sim/(nero_kin)
from nero_kin import NeroKin

np.set_printoptions(precision=4, suppress=True)
kin = NeroKin(Path(__file__).resolve().parents[1] / "assets/nero/nero_description.urdf")
LIM = np.array([[-2.70, 2.70], [-1.74, 1.74], [-2.75, 2.75], [-1.01, 2.14],
                [-2.75, 2.75], [-0.73, 0.95], [-1.57, 1.57]])

z = np.array([0., 0., 1.])
x = np.array([1., 0., 0.]); x -= z * (x @ z); x /= np.linalg.norm(x)
y = np.cross(z, x)
Rt = np.column_stack([x, y, z])

rng = np.random.default_rng(1)
targets = [np.array([0.10, -0.10, 0.50]), np.array([0.00, -0.15, 0.55]),
           np.array([0.15, 0.00, 0.48]), np.array([0.00, -0.25, 0.45]),
           np.array([0.10, 0.10, 0.52]), np.array([-0.10, -0.10, 0.50])]
best = None
tried = 0
for pt in targets:
    Tt = np.eye(4); Tt[:3, :3] = Rt; Tt[:3, 3] = pt
    for _ in range(80):
        tried += 1
        qi = LIM[:, 0] + rng.random(7) * (LIM[:, 1] - LIM[:, 0])
        q, ok_ = kin.ik(Tt, qi)
        zc = kin.fk(q)[:3, :3][:, 2]
        inlim = np.all(q >= LIM[:, 0] - 1e-3) and np.all(q <= LIM[:, 1] + 1e-3)
        if ok_ and zc[2] > 0.9 and inlim:
            best = (pt, q, zc, kin.fk(q)[:3, 3])
            break
    if best:
        break

if best:
    pt, q, zc, pos = best
    print(f"HOME FOUND (tried {tried})")
    print("  target pt:", np.round(pt, 3))
    print("  q_home   :", np.round(q, 4).tolist())
    print("  approach_z:", np.round(zc, 3), " ee_pos:", np.round(pos, 3))
else:
    print(f"no upright IK solution found (tried {tried})")
