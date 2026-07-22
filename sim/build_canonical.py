"""规范层捕获:视频 → 每帧原始人手测量 → canonical_ds(LeRobotDataset,本体无关)。

规范层 = 采集到的「人做了什么」,不绑任何机器人、不做重定向、不做平滑(纯母带):
  - observation.images.ego     : 第一视角 RGB(video)
  - observation.hand_keypoints : MANO 规范系 21 点 ×3 = (63,) 米(腕在原点)
  - observation.wrist_pose     : 手腕 6-DoF = (7,) [tx,ty,tz, qx,qy,qz,qw](相机系;Femto 后换度量世界系)
  - task                       : 语言指令
  (timestamp/frame_index 由 LeRobotDataset 自动加)

换机器人只需拿这份 canonical_ds 过 `derive_embodiment.py` 按 URDF 重新 retarget,采集不重来。
对比:旧 `detect_wrist.py` 在采集时就 retarget 成 inspire 的 12 关节并丢掉原始 21 点 —— 那是有损不可逆投影,换本体就废了。

用法: python sim/build_canonical.py            (默认吃 data/ 第一个 mp4)
"""
import os
import sys
import glob
import shutil
import argparse
from pathlib import Path

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as Rot

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))   # sim/(vendored detector + estimate_wrist)
from single_hand_detector import SingleHandDetector
from estimate_wrist import estimate_wrist_pose

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ROOT = REPO / "sim/out/canonical_ds"
IMG = 256
TASK = "imitate the demonstrated hand motion"

KP_NAMES = [f"kp{i}_{a}" for i in range(21) for a in "xyz"]           # 63
WRIST_NAMES = ["tx", "ty", "tz", "qx", "qy", "qz", "qw"]             # 7

CANONICAL_FEATURES = {
    "observation.images.ego": {"dtype": "video", "shape": (IMG, IMG, 3),
                               "names": ["height", "width", "channel"]},
    "observation.hand_keypoints": {"dtype": "float32", "shape": (63,), "names": KP_NAMES},
    "observation.wrist_pose": {"dtype": "float32", "shape": (7,), "names": WRIST_NAMES},
}


def pose_to_vec(T: np.ndarray) -> np.ndarray:
    """4x4 → (7,) [平移3 + 四元数4(qx,qy,qz,qw)]。"""
    q = Rot.from_matrix(T[:3, :3]).as_quat()   # xyzw
    return np.concatenate([T[:3, 3], q]).astype(np.float32)


def main():
    ap = argparse.ArgumentParser(description="视频 → canonical_ds(本体无关规范层)")
    ap.add_argument("--video", default=None,
                    help="源视频路径;不传则取 data/ 下字母序第一个 mp4(原行为)")
    args = ap.parse_args()

    if args.video:
        video = args.video
        if not Path(video).exists():
            raise SystemExit(f"找不到视频: {video}")
    else:
        vids = sorted(glob.glob(str(REPO / "data/*.mp4")))
        if not vids:
            raise SystemExit("data/ 下没有 mp4")
        video = vids[0]
    print("video:", video)

    det = SingleHandDetector(hand_type="Right", selfie=False,
                             min_detection_confidence=0.5, min_tracking_confidence=0.5)
    cap = cv2.VideoCapture(video)
    fps = int(round(cap.get(cv2.CAP_PROP_FPS))) or 30

    if ROOT.exists():
        shutil.rmtree(ROOT)
    ds = LeRobotDataset.create(repo_id="local/handdemo_canonical", fps=fps,
                               features=CANONICAL_FEATURES, root=str(ROOT),
                               robot_type="canonical", use_videos=True,
                               metadata_buffer_size=1)

    n_frames, n_miss, last_kp, last_wp = 0, 0, None, None
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        rgb = frame[..., ::-1]
        num, joint_pos, kp2d, wrist_rot = det.detect(rgb)
        if num == 0:
            if last_kp is None:      # 起始还没有检测到手,跳过(保持逐帧自洽)
                continue
            kp_vec, wp_vec = last_kp, last_wp   # 丢检:沿用上一帧(与 detect_wrist 一致)
            n_miss += 1
        else:            
            kp2d_px = SingleHandDetector.parse_keypoint_2d(kp2d, frame.shape)
            T = estimate_wrist_pose(joint_pos, kp2d_px, wrist_rot, det.operator2mano, frame.shape)
            kp_vec = joint_pos.astype(np.float32).reshape(63)     # (21,3)→(63,) MANO 米
            wp_vec = pose_to_vec(T)
            last_kp, last_wp = kp_vec, wp_vec

        img = cv2.cvtColor(cv2.resize(frame, (IMG, IMG)), cv2.COLOR_BGR2RGB)
        ds.add_frame({
            "observation.images.ego": img,
            "observation.hand_keypoints": kp_vec,
            "observation.wrist_pose": wp_vec,
            "task": TASK,
        })
        n_frames += 1
    cap.release()
    ds.save_episode()
    print(f"wrote {n_frames} frames ({n_miss} 丢检沿用上一帧), 1 episode -> {ROOT}")
    print("回读验证(独立进程): python sim/verify_dataset.py --canonical")


if __name__ == "__main__":
    main()
