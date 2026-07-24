"""Build canonical_ds from aligned RGB-D frames.

Expected input layout:
  root/
    calibration.json
    color/frame000.png
    depth/frame000.png

The depth frames are assumed to be aligned to color. Hand 2D landmarks come from
the selected estimator, and metric 3D landmarks are lifted from depth using the
camera intrinsics.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from hand_estimators import make_hand_estimator
from estimate_wrist import estimate_wrist_pose
from single_hand_detector import SingleHandDetector

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
from lerobot.datasets.lerobot_dataset import LeRobotDataset

DEFAULT_ROOT = REPO / "sim/out/canonical_ds"
IMG = 256
TASK = "imitate the demonstrated hand motion"

KP_NAMES = [f"kp{i}_{a}" for i in range(21) for a in "xyz"]
KP2D_NAMES = [f"kp{i}_{a}" for i in range(21) for a in ["u", "v"]]
VIS_NAMES = [f"kp{i}_visibility" for i in range(21)]
WRIST_NAMES = ["tx", "ty", "tz", "qx", "qy", "qz", "qw"]
ESTIMATOR_IDS = {"mediapipe": 0.0, "wilor": 1.0}
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
]

CANONICAL_FEATURES = {
    "observation.images.ego": {"dtype": "video", "shape": (IMG, IMG, 3),
                               "names": ["height", "width", "channel"]},
    "observation.hand_keypoints": {"dtype": "float32", "shape": (63,), "names": KP_NAMES},
    "observation.hand_keypoints_2d": {"dtype": "float32", "shape": (42,), "names": KP2D_NAMES},
    "observation.hand_visibility": {"dtype": "float32", "shape": (21,), "names": VIS_NAMES},
    "observation.wrist_pose": {"dtype": "float32", "shape": (7,), "names": WRIST_NAMES},
    "observation.hand_estimator_id": {"dtype": "float32", "shape": (1,), "names": ["estimator_id"]},
}


def _load_camera(calib_path: Path, camera: str) -> tuple[dict, np.ndarray]:
    data = json.loads(calib_path.read_text(encoding="utf-8"))
    cam = data["cameras"][camera]
    intr = cam["intrinsics"]
    ext = cam["extrinsics"]
    if ext.get("direction") != "camera_to_world":
        raise SystemExit(f"{camera} extrinsics.direction must be camera_to_world")
    q_wxyz = np.asarray(ext["rotation_q_wxyz"], dtype=np.float64)
    q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=np.float64)
    T_wc = np.eye(4, dtype=np.float64)
    T_wc[:3, :3] = Rot.from_quat(q_xyzw).as_matrix()
    T_wc[:3, 3] = np.asarray(ext["translation_xyz"], dtype=np.float64)
    return intr, T_wc


def _pose_to_vec(T: np.ndarray) -> np.ndarray:
    q = Rot.from_matrix(T[:3, :3]).as_quat()
    return np.concatenate([T[:3, 3], q]).astype(np.float32)


def _depth_at(depth_m: np.ndarray, u: float, v: float, radius: int) -> tuple[float, bool]:
    h, w = depth_m.shape
    x = int(round(u))
    y = int(round(v))
    if x < 0 or x >= w or y < 0 or y >= h:
        return 0.0, False
    x0, x1 = max(0, x - radius), min(w, x + radius + 1)
    y0, y1 = max(0, y - radius), min(h, y + radius + 1)
    patch = depth_m[y0:y1, x0:x1]
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return 0.0, False
    return float(np.median(valid)), True


def _backproject(kp2d: np.ndarray, depth_m: np.ndarray, intr: dict, radius: int,
                 max_depth_delta: float) -> tuple[np.ndarray, np.ndarray]:
    fx, fy = float(intr["fx"]), float(intr["fy"])
    cx, cy = float(intr["cx"]), float(intr["cy"])
    pts = np.zeros((21, 3), dtype=np.float32)
    valid = np.zeros(21, dtype=bool)
    depths = np.zeros(21, dtype=np.float32)
    for i, (u, v) in enumerate(kp2d):
        z, ok = _depth_at(depth_m, float(u), float(v), radius)
        if not ok:
            continue
        depths[i] = z
        pts[i] = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=np.float32)
        valid[i] = True
    if max_depth_delta > 0.0 and valid.any():
        ref_ids = [i for i in [0, 5, 9, 13, 17] if valid[i]]
        ref_depth = float(np.median(depths[ref_ids] if ref_ids else depths[valid]))
        valid &= np.abs(depths - ref_depth) <= max_depth_delta
    return pts, valid


def _fill_missing_from_model(
    pts_cam: np.ndarray,
    valid: np.ndarray,
    model_rel: np.ndarray,
    T_c_hand: np.ndarray,
) -> np.ndarray:
    if valid.all():
        return pts_cam
    out = pts_cam.copy()
    if valid[0]:
        wrist_cam = out[0].astype(np.float64)
    elif valid.any():
        valid_idx = np.flatnonzero(valid)
        offsets = model_rel[valid_idx] - model_rel[0]
        wrist_candidates = out[valid_idx].astype(np.float64) - offsets.astype(np.float64)
        wrist_cam = np.median(wrist_candidates, axis=0)
        out[0] = wrist_cam.astype(np.float32)
        valid[0] = True
    else:
        wrist_cam = T_c_hand[:3, 3]
        out[0] = wrist_cam.astype(np.float32)
        valid[0] = True
    R_c_hand = T_c_hand[:3, :3]
    for i in np.flatnonzero(~valid):
        rel_cam = R_c_hand @ model_rel[i].astype(np.float64)
        out[i] = (wrist_cam + rel_cam).astype(np.float32)
    return out


def _transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return (pts.astype(np.float64) @ T[:3, :3].T + T[:3, 3]).astype(np.float32)


def _draw_debug(rgb: np.ndarray, kp2d: np.ndarray, frame_name: str, label: str,
                target_hand: str, select_region: str, lock_region: bool) -> np.ndarray:
    out = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    h, w = out.shape[:2]
    if select_region in {"left", "right"}:
        x = w // 2
        cv2.line(out, (x, 0), (x, h), (80, 180, 255), 2)
    elif select_region == "center":
        cv2.line(out, (w // 4, 0), (w // 4, h), (80, 180, 255), 2)
        cv2.line(out, (3 * w // 4, 0), (3 * w // 4, h), (80, 180, 255), 2)
    for a, b in HAND_CONNECTIONS:
        pa = tuple(np.round(kp2d[a]).astype(int))
        pb = tuple(np.round(kp2d[b]).astype(int))
        cv2.line(out, pa, pb, (30, 220, 80), 2, cv2.LINE_AA)
    for i, p in enumerate(kp2d):
        pt = tuple(np.round(p).astype(int))
        color = (0, 0, 255) if i == 0 else (255, 255, 255)
        cv2.circle(out, pt, 4, color, -1, cv2.LINE_AA)
    text = f"{frame_name} selected={target_hand}/{label} region={select_region} lock={int(lock_region)}"
    cv2.putText(out, text, (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(out, text, (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def _make_detector(hand_estimator: str, target_hand: str, max_num_hands: int):
    if hand_estimator != "mediapipe":
        return make_hand_estimator(
            hand_estimator,
            hand_type=target_hand,
            selfie=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
    return SingleHandDetector(
        hand_type=target_hand,
        selfie=False,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        max_num_hands=max_num_hands,
    )


def _region_score(wrist_xy: np.ndarray, width: int, region: str) -> float:
    x = float(wrist_xy[0])
    if region == "left":
        return -x
    if region == "right":
        return x
    if region == "center":
        return -abs(x - width * 0.5)
    return 0.0


def _in_region(wrist_xy: np.ndarray, width: int, region: str) -> bool:
    x = float(wrist_xy[0])
    if region == "left":
        return x < width * 0.5
    if region == "right":
        return x >= width * 0.5
    if region == "center":
        return width * 0.25 <= x <= width * 0.75
    return True


def _detect_target_mediapipe(detector: SingleHandDetector, rgb: np.ndarray, target_hand: str,
                             select_region: str, lock_region: bool, last_wrist: np.ndarray | None):
    num, joint_pos, kp2d_landmarks, wrist_rot = detector.detect(rgb)
    if num == 0:
        return None
    kp2d = SingleHandDetector.parse_keypoint_2d(kp2d_landmarks, rgb.shape).astype(np.float32)
    T = estimate_wrist_pose(
        joint_pos,
        kp2d,
        wrist_rot,
        detector.operator2mano,
        rgb.shape,
    )
    return {
        "keypoints_3d": joint_pos.astype(np.float32),
        "keypoints_2d": kp2d,
        "visibility": np.ones(21, dtype=np.float32),
        "wrist_pose": T.astype(np.float32),
        "label": target_hand,
        "target_hand": target_hand,
    }


def _detect_target(detector, rgb: np.ndarray, hand_estimator: str, target_hand: str,
                   select_region: str, lock_region: bool, last_wrist: np.ndarray | None):
    if hand_estimator == "mediapipe":
        return _detect_target_mediapipe(detector, rgb, target_hand, select_region, lock_region, last_wrist)
    obs = detector.detect(rgb)
    if obs is None:
        return None
    return {
        "keypoints_3d": obs.keypoints_3d.astype(np.float32),
        "keypoints_2d": obs.keypoints_2d.astype(np.float32),
        "visibility": obs.visibility.astype(np.float32) if obs.visibility is not None else np.ones(21, dtype=np.float32),
        "wrist_pose": obs.wrist_pose.astype(np.float32),
        "label": target_hand,
        "target_hand": target_hand,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="RGB-D frames -> canonical_ds with metric hand keypoints")
    ap.add_argument("--input-root", required=True, help="目录,包含 color/ depth/ calibration.json")
    ap.add_argument("--camera", default="kinect2_middle", help="calibration.json 中的相机名")
    ap.add_argument("--depth-scale", type=float, default=0.001, help="uint16 depth -> meter")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--hand-estimator", default="mediapipe", choices=["mediapipe", "wilor"])
    ap.add_argument("--target-hand", default="Right", choices=["Right", "Left"],
                    help="要重定向的人手;注意 selfie=False 时 MediaPipe handedness 会镜像过滤")
    ap.add_argument("--max-num-hands", type=int, default=1, help="MediaPipe 候选手数量;默认保持单手提取")
    ap.add_argument("--select-region", default="auto", choices=["auto", "left", "right", "center"],
                    help="第一帧目标手选择区域;后续按上一帧 wrist 最近跟踪")
    ap.add_argument("--lock-region", action="store_true",
                    help="每帧都只接受 select-region 内的候选;已有两只手视频可用于强制选左/右手")
    ap.add_argument("--debug-dir", default=None, help="保存选中手骨架预览帧的目录")
    ap.add_argument("--debug-every", type=int, default=30, help="每隔多少帧保存一张预览;默认 30")
    ap.add_argument("--depth-radius", type=int, default=2, help="深度查找半径,用非零中值")
    ap.add_argument("--max-depth-delta", type=float, default=0.25,
                    help="关键点深度离手掌参考深度超过该米数则丢弃,用相对手型 fallback")
    ap.add_argument("--hand-keypoints-source", default="mano", choices=["mano", "depth_world"],
                    help="写入 observation.hand_keypoints 的来源;retarget 默认需要 mano 局部系")
    ap.add_argument("--max-frames", type=int, default=0, help="调试用;0 表示全部帧")
    ap.add_argument("--root", default=str(DEFAULT_ROOT), help="canonical_ds 输出目录")
    args = ap.parse_args()

    src = Path(args.input_root)
    color_dir = src / "color"
    depth_dir = src / "depth"
    intr, T_wc = _load_camera(src / "calibration.json", args.camera)

    color_files = sorted(color_dir.glob("frame*.png"))
    depth_files = {p.name: p for p in depth_dir.glob("frame*.png")}
    pairs = [(c, depth_files[c.name]) for c in color_files if c.name in depth_files]
    if args.max_frames > 0:
        pairs = pairs[:args.max_frames]
    if not pairs:
        raise SystemExit(f"没有找到匹配的 color/depth 帧: {src}")
    debug_dir = Path(args.debug_dir) if args.debug_dir else None
    if debug_dir is not None:
        debug_dir.mkdir(parents=True, exist_ok=True)

    det = _make_detector(args.hand_estimator, args.target_hand, args.max_num_hands)
    root = Path(args.root)
    if root.exists():
        shutil.rmtree(root)
    ds = LeRobotDataset.create(repo_id="local/handdemo_canonical", fps=args.fps,
                               features=CANONICAL_FEATURES, root=str(root),
                               robot_type="canonical", use_videos=True,
                               metadata_buffer_size=1)
    estimator_id = np.array([ESTIMATOR_IDS[args.hand_estimator]], dtype=np.float32)

    n_frames, n_miss, n_depth_fallback = 0, 0, 0
    last = None
    last_wrist_2d = None
    for source_i, (color_path, depth_path) in enumerate(pairs):
        bgr = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
        depth_raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if bgr is None or depth_raw is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        obs = _detect_target(
            det,
            rgb,
            args.hand_estimator,
            args.target_hand,
            args.select_region,
            args.lock_region,
            last_wrist_2d,
        )
        if obs is None:
            if last is None:
                continue
            kp_out, kp2d, vis, wp_world = last
            vis = np.zeros(21, dtype=np.float32)
            n_miss += 1
        else:
            depth_m = depth_raw.astype(np.float32) * float(args.depth_scale)
            kp2d = obs["keypoints_2d"].astype(np.float32)
            pts_cam, valid = _backproject(kp2d, depth_m, intr, args.depth_radius, args.max_depth_delta)
            n_depth_fallback += int((~valid).sum())
            pts_cam = _fill_missing_from_model(pts_cam, valid.copy(), obs["keypoints_3d"], obs["wrist_pose"])
            T_c_hand = obs["wrist_pose"].astype(np.float64).copy()
            T_c_hand[:3, 3] = pts_cam[0].astype(np.float64)
            T_w_hand = T_wc @ T_c_hand
            kp_world = _transform_points(T_wc, pts_cam)
            wp_world = _pose_to_vec(T_w_hand)
            vis_raw = obs["visibility"].astype(np.float32).reshape(21)
            if args.hand_keypoints_source == "depth_world":
                kp_out = kp_world
                vis = vis_raw * valid.astype(np.float32)
            else:
                kp_out = obs["keypoints_3d"].astype(np.float32)
                vis = vis_raw
            last = (kp_out, kp2d, vis, wp_world)
            last_wrist_2d = kp2d[0].copy()
            if debug_dir is not None and (source_i % max(1, args.debug_every) == 0):
                dbg = _draw_debug(
                    rgb,
                    kp2d,
                    color_path.stem,
                    str(obs["label"]),
                    str(obs["target_hand"]),
                    args.select_region,
                    args.lock_region,
                )
                cv2.imwrite(str(debug_dir / f"{color_path.stem}.jpg"), dbg)

        img = cv2.resize(rgb, (IMG, IMG))
        ds.add_frame({
            "observation.images.ego": img,
            "observation.hand_keypoints": kp_out.reshape(63).astype(np.float32),
            "observation.hand_keypoints_2d": kp2d.reshape(42).astype(np.float32),
            "observation.hand_visibility": vis.astype(np.float32),
            "observation.wrist_pose": wp_world.astype(np.float32),
            "observation.hand_estimator_id": estimator_id,
            "task": TASK,
        })
        n_frames += 1
    ds.save_episode()
    print(f"wrote {n_frames} RGB-D frames ({n_miss} detector misses, {n_depth_fallback} keypoint depth fallbacks) -> {root}")


if __name__ == "__main__":
    main()
