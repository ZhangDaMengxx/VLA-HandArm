"""Build canonical_ds from a processed hand-observation file.

Supported input formats: .npz, .pkl/.pickle, .json.

Required fields:
  - hand_keypoints: (N,21,3) or (N,63), canonical MediaPipe/MANO order, meters
  - wrist_pose: (N,7) [tx,ty,tz,qx,qy,qz,qw] or (N,4,4)

Optional fields:
  - hand_keypoints_2d: (N,21,2) or (N,42), pixels
  - hand_visibility: (N,21)
  - fps: scalar
  - hand_estimator_id: scalar or (N,1), 0=mediapipe, 1=wilor
"""
from __future__ import annotations

import argparse
import json
import os
import pickle
import shutil
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
from lerobot.datasets.lerobot_dataset import LeRobotDataset

REPO = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = REPO / "sim/out/canonical_ds"
IMG = 256
TASK = "imitate the demonstrated hand motion"

KP_NAMES = [f"kp{i}_{a}" for i in range(21) for a in "xyz"]
KP2D_NAMES = [f"kp{i}_{a}" for i in range(21) for a in ["u", "v"]]
VIS_NAMES = [f"kp{i}_visibility" for i in range(21)]
WRIST_NAMES = ["tx", "ty", "tz", "qx", "qy", "qz", "qw"]

CANONICAL_FEATURES = {
    "observation.images.ego": {"dtype": "video", "shape": (IMG, IMG, 3),
                               "names": ["height", "width", "channel"]},
    "observation.hand_keypoints": {"dtype": "float32", "shape": (63,), "names": KP_NAMES},
    "observation.hand_keypoints_2d": {"dtype": "float32", "shape": (42,), "names": KP2D_NAMES},
    "observation.hand_visibility": {"dtype": "float32", "shape": (21,), "names": VIS_NAMES},
    "observation.wrist_pose": {"dtype": "float32", "shape": (7,), "names": WRIST_NAMES},
    "observation.hand_estimator_id": {"dtype": "float32", "shape": (1,), "names": ["estimator_id"]},
}


def _load(path: Path) -> dict:
    suffix = path.suffix.lower()
    if suffix == ".npz":
        z = np.load(path, allow_pickle=True)
        return {k: z[k] for k in z.files}
    if suffix in {".pkl", ".pickle"}:
        with open(path, "rb") as f:
            return pickle.load(f)
    if suffix == ".json":
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    raise SystemExit(f"不支持的处理结果格式: {path.suffix};请用 .npz/.pkl/.json")


def _field(data: dict, *names: str):
    for name in names:
        if name in data:
            return data[name]
    return None


def _as_kps(v) -> np.ndarray:
    if v is None:
        raise SystemExit("缺少必需字段 hand_keypoints")
    arr = np.asarray(v, dtype=np.float32)
    if arr.ndim == 2 and arr.shape[1] == 63:
        arr = arr.reshape(arr.shape[0], 21, 3)
    if arr.ndim != 3 or arr.shape[1:] != (21, 3):
        raise SystemExit("hand_keypoints 必须是 (N,21,3) 或 (N,63)")
    return arr


def _as_kp2d(v, n: int) -> np.ndarray:
    if v is None:
        return np.zeros((n, 21, 2), dtype=np.float32)
    arr = np.asarray(v, dtype=np.float32)
    if arr.ndim == 2 and arr.shape[1] == 42:
        arr = arr.reshape(arr.shape[0], 21, 2)
    if arr.ndim != 3 or arr.shape[1:] != (21, 2):
        raise SystemExit("hand_keypoints_2d 必须是 (N,21,2) 或 (N,42)")
    return arr


def _as_visibility(v, n: int) -> np.ndarray:
    if v is None:
        return np.ones((n, 21), dtype=np.float32)
    arr = np.asarray(v, dtype=np.float32)
    if arr.ndim != 2 or arr.shape[1] != 21:
        raise SystemExit("hand_visibility 必须是 (N,21)")
    return arr


def _as_wrist(v) -> np.ndarray:
    if v is None:
        raise SystemExit("缺少必需字段 wrist_pose")
    arr = np.asarray(v, dtype=np.float32)
    if arr.ndim == 3 and arr.shape[1:] == (4, 4):
        qs = Rot.from_matrix(arr[:, :3, :3]).as_quat().astype(np.float32)
        arr = np.concatenate([arr[:, :3, 3], qs], axis=1)
    if arr.ndim != 2 or arr.shape[1] != 7:
        raise SystemExit("wrist_pose 必须是 (N,7) 或 (N,4,4)")
    return arr.astype(np.float32)


def main() -> None:
    ap = argparse.ArgumentParser(description="已处理手部结果 -> canonical_ds")
    ap.add_argument("--input", required=True, help="外部处理结果 .npz/.pkl/.json")
    ap.add_argument("--fps", type=int, default=0, help="覆盖输入文件 fps;默认读文件 fps 或 30")
    ap.add_argument("--estimator-id", type=float, default=None,
                    help="覆盖估计器 id:0=mediapipe,1=wilor")
    ap.add_argument("--root", default=str(DEFAULT_ROOT),
                    help="canonical_ds 输出目录;Web 默认使用 sim/out/canonical_ds")
    args = ap.parse_args()

    data = _load(Path(args.input))
    kps = _as_kps(_field(data, "hand_keypoints", "keypoints_3d", "joints", "joints_3d"))
    n = len(kps)
    kp2d = _as_kp2d(_field(data, "hand_keypoints_2d", "keypoints_2d", "joints_2d"), n)
    vis = _as_visibility(_field(data, "hand_visibility", "visibility", "confidence"), n)
    wrist = _as_wrist(_field(data, "wrist_pose", "wrist_poses", "T_wrist", "wrist_matrix"))

    if len(wrist) != n or len(kp2d) != n or len(vis) != n:
        raise SystemExit("hand_keypoints / wrist_pose / 2d / visibility 帧数不一致")

    raw_fps = _field(data, "fps", "frame_rate")
    fps = args.fps or int(np.asarray(30 if raw_fps is None else raw_fps).reshape(-1)[0])
    est = args.estimator_id
    if est is None:
        raw_est = _field(data, "hand_estimator_id", "estimator_id")
        est = float(np.asarray(raw_est).reshape(-1)[0]) if raw_est is not None else -1.0
    est_vec = np.array([est], dtype=np.float32)

    root = Path(args.root)
    if root.exists():
        shutil.rmtree(root)
    ds = LeRobotDataset.create(repo_id="local/handdemo_canonical", fps=fps,
                               features=CANONICAL_FEATURES, root=str(root),
                               robot_type="canonical", use_videos=True,
                               metadata_buffer_size=1)
    blank = np.full((IMG, IMG, 3), 245, dtype=np.uint8)
    cv2.putText(blank, "processed hand file", (28, 128), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (90, 94, 104), 1, cv2.LINE_AA)
    for i in range(n):
        ds.add_frame({
            "observation.images.ego": blank,
            "observation.hand_keypoints": kps[i].reshape(63).astype(np.float32),
            "observation.hand_keypoints_2d": kp2d[i].reshape(42).astype(np.float32),
            "observation.hand_visibility": vis[i].astype(np.float32),
            "observation.wrist_pose": wrist[i].astype(np.float32),
            "observation.hand_estimator_id": est_vec,
            "task": TASK,
        })
    ds.save_episode()
    print(f"wrote {n} processed frames @ {fps}fps -> {root}")


if __name__ == "__main__":
    main()
