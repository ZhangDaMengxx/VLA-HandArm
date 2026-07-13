"""回读验证 LeRobotDataset(探正确的属性名)。"""
import os
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
from pathlib import Path
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ROOT = Path("/home/zhang123/ros2_ws/lerobotTest/sim/out/lerobot_ds")
ds = LeRobotDataset("local/nero_inspire_handdemo", root=str(ROOT))
print("len(ds):", len(ds))
for a in ["num_frames", "num_episodes", "total_frames", "total_episodes"]:
    print(f"  {a}:", getattr(ds, a, "N/A"))
print("features:", list(ds.features.keys()))
s = ds[0]
print("sample keys:", list(s.keys()))
for k in ["observation.state", "action", "observation.images.ego", "task"]:
    if k in s:
        v = s[k]
        print(f"  {k}: {type(v).__name__} shape={getattr(v, 'shape', None)}")
