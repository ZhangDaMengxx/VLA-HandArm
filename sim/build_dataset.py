"""B-4: 本体层轨迹 + 视频 ego 帧 + 语言标签 → LeRobotDataset。
state/action = [7 臂 + 6 驱动手] = 13;ego RGB 从视频取(缩到 256)。
对齐假设:轨迹第 f 帧 = 视频第 f 帧(无前导丢检;脚本会核对帧数)。
"""
import sys
import glob
import shutil
import pickle
from pathlib import Path

import numpy as np
import cv2

sys.path.insert(0, str(Path(__file__).parent))
from schema import ARM_JOINTS, HAND_ACTUATED, STATE_DIM

import os
os.environ.setdefault("HF_HUB_OFFLINE", "1")        # 离线:不连 huggingface.co
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
from lerobot.datasets.lerobot_dataset import LeRobotDataset

REPO = Path(__file__).resolve().parents[1]
DEX = REPO / "dex-retargeting-main/dex-retargeting-main"
ROOT = REPO / "sim/out/lerobot_ds"
IMG = 256
TASK = "imitate the demonstrated hand motion"

T = pickle.load(open(REPO / "sim/out/robot_traj.pkl", "rb"))
arm = np.asarray(T["arm"])            # (F,7)
hand = np.asarray(T["hand"])          # (F,12)
hand_names = list(T["hand_joint_names"])
F = len(arm)

act_idx = [hand_names.index(n) for n in HAND_ACTUATED]
state = np.concatenate([arm, hand[:, act_idx]], axis=1).astype(np.float32)   # (F,13)
action = np.concatenate([state[1:], state[-1:]], axis=0).astype(np.float32)  # 下一帧目标

vids = sorted(glob.glob(str(REPO / "data/*.mp4")))
cap = cv2.VideoCapture(vids[0])
fps = int(round(cap.get(cv2.CAP_PROP_FPS))) or 30
nframe = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
print(f"video fps={fps} frames={nframe} traj={F}")
N = min(F, nframe)

if ROOT.exists():
    shutil.rmtree(ROOT)

features = {
    "observation.state": {"dtype": "float32", "shape": (STATE_DIM,), "names": ARM_JOINTS + HAND_ACTUATED},
    "action": {"dtype": "float32", "shape": (STATE_DIM,), "names": ARM_JOINTS + HAND_ACTUATED},
    "observation.images.ego": {"dtype": "video", "shape": (IMG, IMG, 3), "names": ["height", "width", "channel"]},
}
ds = LeRobotDataset.create(repo_id="local/nero_inspire_handdemo", fps=fps, features=features,
                           root=str(ROOT), robot_type="nero_inspire", use_videos=True,
                           metadata_buffer_size=1)   # 立即刷 episode 元数据到 parquet

for f in range(N):
    ok, frame = cap.read()
    if not ok:
        break
    img = cv2.cvtColor(cv2.resize(frame, (IMG, IMG)), cv2.COLOR_BGR2RGB)
    ds.add_frame({
        "observation.state": state[f],
        "action": action[f],
        "observation.images.ego": img,
        "task": TASK,
    })
cap.release()
ds.save_episode()
print(f"wrote {N} frames, 1 episode -> {ROOT}")

print("回读验证请用独立进程: python sim/verify_dataset.py(同进程重开会冲突)")
