"""Build canonical_ds from a processed hand-observation file.

Supported input formats: .npz, .pkl/.pickle, .json.

Required fields:
  - hand_keypoints: (N,21,3) or (N,63), canonical MediaPipe/MANO order, meters
  - wrist_pose: (N,7) [tx,ty,tz,qx,qy,qz,qw] or (N,4,4)

Optional fields:
  - hand_keypoints_2d: (N,21,2) or (N,42), pixels
  - hand_visibility: (N,21)
  - fps: scalar
  - timestamp_hw_us: (N,) strictly increasing hardware timestamps in microseconds
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
from capture_bundle import (
    WRIST_POSE_FRAMES,
    WRIST_FRAME_EPISODE0_CAMERA,
    archive_processed_input,
    record_source,
    resolve_ego_output,
    write_ego_metadata,
    write_sample_stream_index,
)
from quality_profiles import load_quality_profile

REPO = Path(__file__).resolve().parents[1]
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


def _text_scalar(value, label: str) -> str | None:
    if value is None:
        return None
    array = np.asarray(value).reshape(-1)
    if len(array) != 1:
        raise SystemExit(f"{label} 必须是单个字符串")
    item = array[0]
    if isinstance(item, bytes):
        item = item.decode("utf-8")
    return str(item)


def _resolve_wrist_pose_frame(data: dict, cli_value: str | None) -> tuple[str, str]:
    embedded = _text_scalar(
        _field(data, "wrist_pose_frame", "wrist_coordinate_frame"),
        "wrist_pose_frame",
    )
    if embedded is not None and embedded not in WRIST_POSE_FRAMES:
        raise SystemExit(
            f"输入 wrist_pose_frame={embedded!r} 不受支持;"
            f"可选值:{', '.join(sorted(WRIST_POSE_FRAMES))}"
        )
    if cli_value is not None and embedded is not None and cli_value != embedded:
        raise SystemExit(
            f"--wrist-pose-frame={cli_value} 与输入字段 wrist_pose_frame={embedded} 冲突"
        )
    if cli_value is not None:
        return cli_value, "command_line"
    if embedded is not None:
        return embedded, "processed_input.wrist_pose_frame"
    print(
        "warning:输入未声明 wrist_pose_frame;为兼容旧处理文件按 "
        f"{WRIST_FRAME_EPISODE0_CAMERA} 导入,建议在输入字段或命令行显式声明"
    )
    return WRIST_FRAME_EPISODE0_CAMERA, "compatibility_default_episode0_camera"


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
    ap.add_argument(
        "--wrist-pose-frame",
        choices=sorted(WRIST_POSE_FRAMES),
        default=None,
        help="wrist_pose 所在坐标系;默认读取输入字段,旧文件兼容为 episode0_camera",
    )
    ap.add_argument("--capture-root", default=None,
                    help="Capture Bundle 目录;不传则在 datasets/captures/ 新建")
    ap.add_argument("--root", default=None,
                    help="精确 canonical 输出目录(高级兼容入口,不自动创建 Capture)")
    ap.add_argument("--legacy-out", action="store_true",
                    help="显式写入旧 src/out/canonical_ds")
    ap.add_argument("--quality-profile", default="processed_observations_v1",
                    help="质量 profile ID 或 JSON 路径")
    args = ap.parse_args()

    try:
        root, capture = resolve_ego_output(
            capture_root=args.capture_root,
            output_root=args.root,
            legacy_out=args.legacy_out,
        )
    except (ValueError, FileNotFoundError) as error:
        raise SystemExit(str(error)) from error

    input_path = Path(args.input)
    data = _load(input_path)
    kps = _as_kps(_field(data, "hand_keypoints", "keypoints_3d", "joints", "joints_3d"))
    n = len(kps)
    kp2d = _as_kp2d(_field(data, "hand_keypoints_2d", "keypoints_2d", "joints_2d"), n)
    vis = _as_visibility(_field(data, "hand_visibility", "visibility", "confidence"), n)
    wrist = _as_wrist(_field(data, "wrist_pose", "wrist_poses", "T_wrist", "wrist_matrix"))
    wrist_pose_frame, wrist_pose_frame_declared_by = _resolve_wrist_pose_frame(
        data, args.wrist_pose_frame
    )

    if len(wrist) != n or len(kp2d) != n or len(vis) != n:
        raise SystemExit("hand_keypoints / wrist_pose / 2d / visibility 帧数不一致")

    raw_fps = _field(data, "fps", "frame_rate")
    fps = args.fps or int(np.asarray(30 if raw_fps is None else raw_fps).reshape(-1)[0])
    est = args.estimator_id
    if est is None:
        raw_est = _field(data, "hand_estimator_id", "estimator_id")
        est = float(np.asarray(raw_est).reshape(-1)[0]) if raw_est is not None else -1.0
    est_vec = np.array([est], dtype=np.float32)
    raw_timestamps_hw_us = _field(data, "timestamp_hw_us", "timestamps_hw_us")
    timestamps_hw_us = None
    if raw_timestamps_hw_us is not None:
        timestamp_array = np.asarray(raw_timestamps_hw_us).reshape(-1)
        if len(timestamp_array) != n or not np.isfinite(timestamp_array.astype(float)).all():
            raise SystemExit("timestamp_hw_us 必须是与帧数一致的有限 (N,) 数组")
        timestamps_hw_us = [int(value) for value in timestamp_array]
        if any(current <= previous for previous, current in zip(timestamps_hw_us, timestamps_hw_us[1:])):
            raise SystemExit("timestamp_hw_us 必须严格单调递增")

    if capture is not None:
        try:
            quality_profile = load_quality_profile(args.quality_profile)
        except (ValueError, FileNotFoundError) as error:
            raise SystemExit(str(error)) from error
        record_source(capture, kind="processed_hand_observations", source=input_path, config={
            "estimator_id": est,
            "fps_override": args.fps,
            "fps": fps,
            "wrist_pose_frame": wrist_pose_frame,
            "wrist_pose_frame_declared_by": wrist_pose_frame_declared_by,
            "hand_keypoints_frame": "wrist_local_mano",
        }, hardware_timestamps_available=timestamps_hw_us is not None,
            quality_profile=quality_profile)
        archive_processed_input(capture, input_path)
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
    ds.finalize()
    if capture is not None:
        write_sample_stream_index(
            capture,
            frame_count=n,
            fps=fps,
            rgb_path=None,
            pairing_basis="processed_sample_index",
            timestamps_hw_us=timestamps_hw_us,
        )
        estimator_name = {0.0: "mediapipe", 1.0: "wilor"}.get(est, f"estimator_id_{est:g}")
        write_ego_metadata(
            capture,
            estimator=estimator_name,
            source_kind="processed_hand_observations",
            wrist_pose_frame=wrist_pose_frame,
            wrist_pose_frame_declared_by=wrist_pose_frame_declared_by,
        )
    print(f"wrote {n} processed frames @ {fps}fps -> {root}")


if __name__ == "__main__":
    main()
