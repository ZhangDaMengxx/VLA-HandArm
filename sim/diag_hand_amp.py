"""确认手指数据是否在平滑前后变了 + 看手指开合幅度(min proximal = 最张开)。"""
import pickle
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[1]
R = pickle.load(open(REPO / "sim/out/robot_traj.pkl", "rb"))
hand_now = np.asarray(R["hand"])
names = list(R["hand_joint_names"])

hp = REPO / "sim/out/hand_traj.pkl"
if hp.exists():
    H = pickle.load(open(hp, "rb"))
    hand_early = np.asarray(H["data"])
    same = (hand_now.shape == hand_early.shape) and np.allclose(hand_now, hand_early)
    print(f"手指数据(早期 hand_traj vs 现在 robot_traj)完全相同? {same}")
else:
    print("hand_traj.pkl 不存在,跳过比对")

print("\n各手指开合幅度(度,proximal;min=最张开,max=最握紧):")
for i, n in enumerate(names):
    if "proximal" in n and "yaw" not in n and "pitch" not in n:
        print(f"  {n:22s} min {np.rad2deg(hand_now[:, i].min()):5.1f}   max {np.rad2deg(hand_now[:, i].max()):5.1f}")
print("\n参考:URDF 里 proximal 上限约 84°(1.47rad);手势预设 open=0°(全平)。")
