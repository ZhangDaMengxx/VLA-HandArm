"""回读验证 LeRobotDataset(探正确的属性名)。"""
import os
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
from pathlib import Path
from lerobot.datasets.lerobot_dataset import LeRobotDataset

_base = Path(__file__).resolve().parents[1] / "sim/out"
ROOT = _base / "lerobot_ds_nero_inspire"          # 两层路径产物(默认)
if not ROOT.exists():
    ROOT = _base / "lerobot_ds"                   # 回退旧单本体路径
print("dataset:", ROOT.name)
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
