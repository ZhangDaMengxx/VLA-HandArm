"""降风险探测:LeRobotDataset 能否存/读回扁平 float32 特征(规范层要用 hand_keypoints(63,)、wrist_pose(7,))+ video。
通过 → 规范层就用 LeRobotDataset 容器;失败 → 回退 npz+mp4。独立进程跑(create 与回读同进程会冲突)。
"""
import os, shutil, sys
from pathlib import Path
import numpy as np

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ROOT = Path(__file__).resolve().parents[1] / "sim/out/_probe_ds"

if "--read" in sys.argv:
    ds = LeRobotDataset("local/_probe", root=str(ROOT))
    print("READ_OK len =", len(ds), "features =", list(ds.features.keys()))
    s = ds[0]
    for k in ("observation.hand_keypoints", "observation.wrist_pose", "observation.images.ego"):
        v = s[k]
        print(f"  {k}: shape={tuple(getattr(v,'shape',()))} dtype={getattr(v,'dtype',type(v))}")
    print("  wrist_pose[0]=", np.asarray(s["observation.wrist_pose"]))
    sys.exit(0)

if ROOT.exists():
    shutil.rmtree(ROOT)

IMG = 64
features = {
    "observation.images.ego": {"dtype": "video", "shape": (IMG, IMG, 3), "names": ["height", "width", "channel"]},
    "observation.hand_keypoints": {"dtype": "float32", "shape": (63,),
                                   "names": [f"kp{i}_{a}" for i in range(21) for a in "xyz"]},
    "observation.wrist_pose": {"dtype": "float32", "shape": (7,),
                               "names": ["tx", "ty", "tz", "qx", "qy", "qz", "qw"]},
}
ds = LeRobotDataset.create(repo_id="local/_probe", fps=30, features=features,
                           root=str(ROOT), robot_type="canonical", use_videos=True,
                           metadata_buffer_size=1)
N = 5
for f in range(N):
    ds.add_frame({
        "observation.images.ego": (np.random.rand(IMG, IMG, 3) * 255).astype(np.uint8),
        "observation.hand_keypoints": np.random.rand(63).astype(np.float32),
        "observation.wrist_pose": np.array([0, 0, 0.4, 0, 0, 0, 1], np.float32),
        "task": "probe",
    })
ds.save_episode()
print("WROTE_OK", N, "frames ->", ROOT)
print("run readback: python sim/probe_canonical_feats.py --read")
