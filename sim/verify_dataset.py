"""回读验证 LeRobotDataset(探正确的属性名)。"""
import os
import argparse
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
from pathlib import Path
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ap = argparse.ArgumentParser()
ap.add_argument("--canonical", action="store_true", help="验证 canonical_ds")
args = ap.parse_args()

_base = Path(__file__).resolve().parents[1] / "sim/out"
if args.canonical:
    ROOT = _base / "canonical_ds"
    REPO_ID = "local/handdemo_canonical"
else:
    ROOT = _base / "lerobot_ds_nero_inspire"      # 两层路径产物(默认)
    REPO_ID = "local/nero_inspire_handdemo"
    if not ROOT.exists():
        ROOT = _base / "lerobot_ds"               # 回退旧单本体路径
print("dataset:", ROOT.name)
ds = LeRobotDataset(REPO_ID, root=str(ROOT))
print("len(ds):", len(ds))
for a in ["num_frames", "num_episodes", "total_frames", "total_episodes"]:
    print(f"  {a}:", getattr(ds, a, "N/A"))
print("features:", list(ds.features.keys()))
s = ds[0]
print("sample keys:", list(s.keys()))
for k in [
    "observation.state",
    "action",
    "observation.images.ego",
    "observation.hand_keypoints",
    "observation.hand_keypoints_2d",
    "observation.hand_visibility",
    "observation.wrist_pose",
    "observation.hand_estimator_id",
    "task",
]:
    if k in s:
        v = s[k]
        print(f"  {k}: {type(v).__name__} shape={getattr(v, 'shape', None)}")
