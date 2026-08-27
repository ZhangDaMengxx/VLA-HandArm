"""验收指标测量:逐项测可测的路线图指标,不可测的明确标注(pass=None)。

返回结构化指标 + 可写 JSON,供 app_web 右侧"数据有效性·验收"卡按本体绑定显示。

用法:
    python3 src/measure_acceptance.py --robot nero_inspire_rgbd         # 最新 Capture，报告写入 reports/
    python3 src/measure_acceptance.py --capture-root datasets/captures/capture_<id> --robot nero_gripper_rgbd
    python3 src/measure_acceptance.py --robot nero_inspire_rgbd --legacy-out  # 显式读取旧 src/out
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np

from paths import REPO, ASSEMBLY_URDF, GRIPPER_URDF
from capture_bundle import capture_for_path, read_ego_coordinate_system, resolve_data_paths
from quality_profiles import (
    evaluate_threshold,
    load_quality_profile,
    metric_spec,
    read_quality_profile_snapshot,
    threshold_text,
)
sys.path.insert(0, str(Path(__file__).resolve().parent))


def _clean(v):
    """把 numpy 标量转成干净的 Python 类型(避免 float32 的 10.6999… 尾巴)。"""
    if isinstance(v, np.bool_):
        return bool(v)
    if isinstance(v, np.integer):
        return int(v)
    if isinstance(v, np.floating):
        return round(float(v), 3)
    return v


def _metric(
    key,
    label,
    value,
    unit,
    threshold,
    passed,
    category,
    note="",
    *,
    measurement_class,
    measurement_basis,
    ground_truth_required=False,
    ground_truth_available=False,
):
    """Build one metric without conflating proxy evidence with absolute accuracy."""
    return {
        "key": key,
        "label": label,
        "value": _clean(value),
        "unit": unit,
        "threshold": threshold,
        "pass": _clean(passed),
        "category": category,
        "measurement_class": measurement_class,
        "measurement_basis": measurement_basis,
        "ground_truth_required": bool(ground_truth_required),
        "ground_truth_available": bool(ground_truth_available),
        "note": note,
    }


def _profile_metric(
    profile,
    profile_key,
    key,
    label,
    value,
    category,
    note="",
    *,
    measurement_class,
    measurement_basis,
    ground_truth_required=False,
    ground_truth_available=False,
    optional_profile_metric=False,
):
    try:
        spec = metric_spec(profile, profile_key)
    except ValueError:
        if not optional_profile_metric:
            raise
        return _metric(
            key,
            label,
            value,
            "",
            "not defined in profile snapshot",
            None,
            category,
            note,
            measurement_class=measurement_class,
            measurement_basis=measurement_basis,
            ground_truth_required=ground_truth_required,
            ground_truth_available=ground_truth_available,
        )
    declared_class = spec.get("measurement_class")
    if declared_class is not None and declared_class != measurement_class:
        raise ValueError(
            f"quality metric {profile_key!r} declares measurement_class "
            f"{declared_class!r}, producer uses {measurement_class!r}"
        )
    declared_ground_truth = spec.get("ground_truth_required")
    if declared_ground_truth is not None and declared_ground_truth != ground_truth_required:
        raise ValueError(
            f"quality metric {profile_key!r} ground_truth_required disagrees with producer"
        )
    passed = None if value is None else evaluate_threshold(value, spec)
    return _metric(
        key,
        label,
        value,
        spec["unit"],
        threshold_text(spec),
        passed,
        category,
        note,
        measurement_class=measurement_class,
        measurement_basis=measurement_basis,
        ground_truth_required=ground_truth_required,
        ground_truth_available=ground_truth_available,
    )


def _resolve_quality_profile(canonical_dir: Path, explicit: str | None):
    capture = capture_for_path(canonical_dir)
    snapshot = read_quality_profile_snapshot(capture.source) if capture is not None else None
    if explicit:
        selected = load_quality_profile(explicit)
        if snapshot is not None and selected != snapshot:
            raise ValueError(
                "explicit quality profile differs from the immutable Capture snapshot"
            )
        return selected, "capture_snapshot" if snapshot is not None else "explicit"
    if snapshot is not None:
        return snapshot, "capture_snapshot"
    # Old Capture and external/legacy roots have no device evidence. This profile
    # deliberately requires no RGB, depth, timestamp, or sync capability.
    return load_quality_profile("processed_observations_v1"), "compatibility_default_undeclared"


def _measured_source_fps(capture, stream: str):
    """Measure native stream cadence from hardware timestamps, never from declared FPS."""
    if capture is None:
        return None, None, 0, "Capture 不存在，缺少设备时间戳证据"
    stream_index = capture.source / "stream_index.parquet"
    if not stream_index.is_file():
        return None, None, 0, "缺少 source/stream_index.parquet，无法实测帧率"
    column = "rgb_timestamp_hw_us" if stream == "rgb" else "depth_timestamp_hw_us"
    try:
        import pandas as pd

        frame = pd.read_parquet(stream_index)
        if column not in frame.columns:
            return None, None, 0, f"stream_index.parquet 缺少 {column}"
        if "episode_index" not in frame.columns:
            frame = frame.assign(episode_index=0)
        interval_count = 0
        duration_us = 0.0
        max_gap_us = 0.0
        sample_count = 0
        for _, episode in frame.groupby("episode_index", sort=False):
            values = episode[column].dropna().to_numpy(dtype=np.int64)
            sample_count += len(values)
            if len(values) < 2:
                continue
            differences = np.diff(values)
            if np.any(differences <= 0):
                return None, None, sample_count, f"{column} 不是严格单调递增"
            interval_count += len(differences)
            duration_us += float(np.sum(differences))
            max_gap_us = max(max_gap_us, float(np.max(differences)))
        if interval_count == 0 or duration_us <= 0:
            return None, None, sample_count, f"{column} 有效样本不足"
        measured_fps = interval_count * 1_000_000.0 / duration_us
        return (
            measured_fps,
            max_gap_us / 1000.0,
            sample_count,
            f"基于 {column} 的设备时间戳实测；不是 acquisition.json 声明值",
        )
    except (ImportError, OSError, ValueError, TypeError):
        return None, None, 0, "无法读取 stream_index.parquet 的设备时间戳"


def _source_acquisition_metrics(canonical_dir: Path, profile):
    capture = capture_for_path(canonical_dir)
    acquisition_path = capture.source / "acquisition.json" if capture is not None else None
    acquisition = None
    if acquisition_path is not None and acquisition_path.is_file():
        acquisition = json.loads(acquisition_path.read_text(encoding="utf-8"))
    config = acquisition.get("config", {}) if isinstance(acquisition, dict) else {}
    metrics = []

    for stream in ("rgb", "depth"):
        standard = profile["acquisition"][stream]
        if not standard["required"]:
            continue
        declared_fps = config.get(f"{stream}_fps", config.get("fps"))
        min_fps = standard.get("min_fps")
        min_measured_fps = standard.get("min_measured_fps")
        measured_fps = None
        max_gap_ms = None
        sample_count = 0
        fps_note = "来自 source/acquisition.json；不是由 Ego 输出帧率反推"
        if min_measured_fps is not None:
            measured_fps, max_gap_ms, sample_count, fps_note = _measured_source_fps(
                capture, stream
            )
            declaration_pass = (
                declared_fps is not None
                and min_fps is not None
                and float(declared_fps) >= float(min_fps)
            )
            measured_pass = (
                measured_fps is not None
                and float(measured_fps) >= float(min_measured_fps)
            )
            fps_pass = bool(declaration_pass and measured_pass)
            fps = measured_fps
            fps_note += (
                f"；声明={declared_fps!r} fps，样本={sample_count}，最大帧间隔="
                f"{max_gap_ms:.3f} ms" if max_gap_ms is not None else
                f"；声明={declared_fps!r} fps，样本={sample_count}"
            )
            threshold = f"nominal>={min_fps:g}, measured>={min_measured_fps:g}"
            measurement_basis = "source_hardware_timestamp_cadence"
        else:
            fps = declared_fps
            fps_pass = None if fps is None or min_fps is None else float(fps) >= float(min_fps)
            threshold = f">={min_fps:g}" if min_fps is not None else "declared"
            measurement_basis = "source_acquisition_declaration"
        metrics.append(_metric(
            f"source_{stream}_fps",
            f"Source {stream.upper()} 帧率",
            fps,
            "fps",
            threshold,
            fps_pass,
            "source",
            fps_note,
            measurement_class="source_capability",
            measurement_basis=measurement_basis,
        ))
        width_key = "width" if stream == "rgb" else "depth_width"
        height_key = "height" if stream == "rgb" else "depth_height"
        if stream == "rgb" and "rgb_width" in config:
            width_key, height_key = "rgb_width", "rgb_height"
        width, height = config.get(width_key), config.get(height_key)
        min_width, min_height = standard.get("min_width"), standard.get("min_height")
        resolution_pass = None
        if None not in (width, height, min_width, min_height):
            resolution_pass = int(width) >= int(min_width) and int(height) >= int(min_height)
        metrics.append(_metric(
            f"source_{stream}_resolution",
            f"Source {stream.upper()} 分辨率",
            None if width is None or height is None else f"{width}x{height}",
            "px",
            (
                f">={min_width:g}x{min_height:g}"
                if min_width is not None and min_height is not None else "declared"
            ),
            resolution_pass,
            "source",
            "实际输入尺寸；Ego 训练视频可另行缩放，不用于替代 Source 能力",
            measurement_class="source_capability",
            measurement_basis="source_acquisition_declaration",
        ))

    timebase = acquisition.get("timebase", {}) if isinstance(acquisition, dict) else {}
    hardware_available = timebase.get("hardware_timestamps_available")
    timestamp_required = profile["acquisition"]["hardware_timestamps"]["required"]
    timestamp_pass = None
    if timestamp_required:
        timestamp_pass = bool(hardware_available) if hardware_available is not None else None
    metrics.append(_metric(
        "source_hardware_timestamps",
        "Source 硬件时间戳",
        hardware_available,
        "",
        "required" if timestamp_required else "not required",
        timestamp_pass,
        "source",
        "false 表示当前 Source 明确没有硬件时间；不得用 frame_index/fps 冒充",
        measurement_class="source_capability",
        measurement_basis="source_acquisition_declaration",
    ))

    sync_standard = profile["acquisition"]["rgb_depth_sync"]
    sync_value = None
    sync_note = "profile 不要求 RGB-D 硬件同步"
    if sync_standard["required"]:
        sync_note = "缺少成对 RGB/Depth 硬件时间戳，无法计算同步残差"
        if hardware_available and capture is not None:
            stream_index = capture.source / "stream_index.parquet"
            if stream_index.is_file():
                try:
                    import pandas as pd
                    values = pd.read_parquet(stream_index)["sync_error_ms"].dropna().to_numpy(float)
                    if len(values):
                        sync_value = float(np.max(values))
                        sync_note = "Source 成对硬件时间戳的最大绝对残差"
                except (ImportError, KeyError, OSError, ValueError):
                    sync_note = "无法读取 stream_index.parquet 的硬件同步残差"
    sync_pass = None
    if sync_standard["required"]:
        if hardware_available is False:
            sync_pass = False
        elif sync_value is not None:
            sync_pass = sync_value < float(sync_standard["max_error_ms"])
    metrics.append(_metric(
        "source_rgb_depth_sync",
        "Source RGB-Depth 硬件同步",
        sync_value,
        "ms",
        (
            f"<{sync_standard['max_error_ms']:g}"
            if sync_standard["required"] else "not required"
        ),
        sync_pass,
        "source",
        sync_note,
        measurement_class="timing",
        measurement_basis="paired_hardware_timestamps",
    ))
    return metrics


def _hand_type(robot: str) -> str:
    """从本体名判手类型:含 gripper→夹爪;否则→inspire 灵巧手。"""
    return "gripper" if "gripper" in robot else "inspire"


def _canonical_df(canonical_dir: Path):
    import pandas as pd, glob
    f = sorted(glob.glob(str(canonical_dir / "data/**/*.parquet"), recursive=True))
    if not f:
        raise FileNotFoundError(f"无 canonical parquet: {canonical_dir}")
    return pd.read_parquet(f[0])


# MANO/MediaPipe 手骨连接(近似,用于骨长恒定性)
BONES = [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),(0,9),(9,10),(10,11),(11,12),
         (0,13),(13,14),(14,15),(15,16),(0,17),(17,18),(18,19),(19,20)]


def measure_gripper(profile):
    """夹爪开合误差<1mm:驱动两指 prismatic 到指令行程,量指尖位移 vs 指令。返回指标列表。"""
    import pinocchio as pin
    m = pin.buildModelFromUrdf(str(GRIPPER_URDF))
    d = m.createData()
    fingers = [m.names[i] for i in range(len(m.names)) if "gripper_finger" in m.names[i]]
    tip_frames = [f.name for f in m.frames
                  if "finger" in f.name.lower() and f.type == pin.FrameType.BODY]
    rows = []
    for cmd in np.linspace(0.0, 0.05, 6):        # 指令行程 0~50mm
        q = pin.neutral(m)
        for n in fingers:
            q[m.joints[m.getJointId(n)].idx_q] = cmd
        pin.forwardKinematics(m, d, q)
        pin.updateFramePlacements(m, d)
        tips = [d.oMf[m.getFrameId(t)].translation.copy() for t in tip_frames[:2]]
        if len(tips) == 2:
            rows.append((cmd, np.linalg.norm(tips[0] - tips[1])))
    base = rows[0][1]
    max_e = 0.0
    for cmd, w in rows:
        single = 0.5 * (w - base)         # 单指行程 = 开口变化/2
        max_e = max(max_e, abs(single - cmd))
        print(f"[夹爪] cmd={cmd*1000:5.1f}mm 开口Δ={(w-base)*1000:6.2f}mm 单指行程={single*1000:5.1f}mm")
    print(f"[夹爪] 最大开合误差={max_e*1000:.3f}mm (阈值<1mm) 注:URDF运动学线性映射,非物理仿真")
    return [_profile_metric(
        profile, "gripper_open_error_mm", "gripper_open", "夹爪开合误差",
        round(max_e * 1000, 3), "embodiment",
        "URDF运动学:指令行程vs指尖位移。线性映射,非物理仿真",
        measurement_class="model_fidelity",
        measurement_basis="urdf_forward_kinematics",
    )]


def _episode_values(df):
    if "episode_index" in df.columns:
        return np.asarray(df["episode_index"].values)
    return np.zeros(len(df), dtype=int)


def _consecutive_pairs(df):
    """Mask adjacent rows that are consecutive frames in the same episode."""
    if len(df) < 2:
        return np.zeros(0, dtype=bool)
    same_episode = _episode_values(df)[1:] == _episode_values(df)[:-1]
    if "frame_index" in df.columns:
        frames = np.asarray(df["frame_index"].values)
        same_episode &= np.diff(frames) == 1
    return same_episode


def _wrist_continuity(wrist_pose, pair_mask):
    valid = pair_mask & np.isfinite(wrist_pose[1:]).all(axis=1)
    valid &= np.isfinite(wrist_pose[:-1]).all(axis=1)
    if not valid.any():
        return None, None
    position_steps = np.linalg.norm(np.diff(wrist_pose[:, :3], axis=0)[valid], axis=1)
    q0 = wrist_pose[:-1, 3:7][valid]
    q1 = wrist_pose[1:, 3:7][valid]
    n0 = np.linalg.norm(q0, axis=1)
    n1 = np.linalg.norm(q1, axis=1)
    quat_valid = (n0 > 1e-8) & (n1 > 1e-8)
    rotation_steps = None
    if quat_valid.any():
        dots = np.abs(np.sum(
            q0[quat_valid] / n0[quat_valid, None]
            * q1[quat_valid] / n1[quat_valid, None],
            axis=1,
        ))
        rotation_steps = np.degrees(2.0 * np.arccos(np.clip(dots, 0.0, 1.0)))
    return (
        float(np.percentile(position_steps, 99) * 1000),
        None if rotation_steps is None else float(np.percentile(rotation_steps, 99)),
    )


def _stationary_wrist_jitter(df, wrist_pose):
    """Return worst segment p95 radius; motion is never guessed to be stationary."""
    column = "annotation.wrist_stationary"
    if column not in df.columns:
        return None, False
    stationary = np.asarray(df[column].values, dtype=bool)
    pairs = _consecutive_pairs(df)
    segments = []
    start = None
    for index, is_stationary in enumerate(stationary):
        joins_previous = index > 0 and pairs[index - 1]
        if is_stationary and (start is None or joins_previous):
            if start is None:
                start = index
        else:
            if start is not None:
                segments.append((start, index))
                start = index if is_stationary else None
    if start is not None:
        segments.append((start, len(stationary)))
    values = []
    for first, last in segments:
        points = wrist_pose[first:last, :3]
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) < 3:
            continue
        center = np.median(points, axis=0)
        values.append(float(np.percentile(np.linalg.norm(points - center, axis=1), 95) * 1000))
    return (max(values) if values else None), True


def measure_hand_quality(df, profile):
    """Measure detection, absolute wrist accuracy, and explicitly named proxies."""
    n = len(df)
    if n == 0:
        raise ValueError("cannot measure an empty Ego dataset")
    vis = np.stack(df["observation.hand_visibility"].values)               # (N,21)
    detected_mask = vis.sum(axis=1) > 0
    detected = int(detected_mask.sum())
    kp = np.stack(df["observation.hand_keypoints"].values).reshape(n, 21, 3)
    episode_ids = _episode_values(df)
    bone_stds = []
    for episode_id in np.unique(episode_ids):
        mask = (episode_ids == episode_id) & detected_mask & np.isfinite(kp).all(axis=(1, 2))
        if mask.sum() < 2:
            continue
        episode_kp = kp[mask]
        lengths = np.array([
            [np.linalg.norm(points[a] - points[b]) for a, b in BONES]
            for points in episode_kp
        ])
        bone_stds.append(lengths.std(axis=0))
    std_l = np.concatenate(bone_stds) if bone_stds else np.array([], dtype=float)
    rate = detected / n * 100
    worst_mm = float(std_l.max() * 1000) if len(std_l) else None
    med_mm = float(np.median(std_l) * 1000) if len(std_l) else None
    detect_threshold = threshold_text(metric_spec(profile, "hand_detection_rate_percent"))
    scale_threshold = threshold_text(metric_spec(profile, "bone_length_std_max_mm"))
    print(f"[检出] {detected}/{n} = {rate:.1f}% (阈值{detect_threshold}%)")
    print(f"[稳定性代理] 骨长波动 中位std={med_mm}mm 最差骨={worst_mm}mm "
          f"(阈值{scale_threshold}mm；不代表绝对尺度精度)")

    wrist_pose = np.stack(df["observation.wrist_pose"].values).reshape(n, 7)
    gt_column = "ground_truth.wrist_pose"
    ground_truth_available = gt_column in df.columns
    absolute_error = None
    absolute_note = "缺少 ground_truth.wrist_pose；稳定性代理不能替代绝对精度"
    if ground_truth_available:
        truth = np.stack(df[gt_column].values).reshape(n, 7)
        valid = np.isfinite(wrist_pose[:, :3]).all(axis=1)
        valid &= np.isfinite(truth[:, :3]).all(axis=1)
        if valid.any():
            errors = np.linalg.norm(wrist_pose[valid, :3] - truth[valid, :3], axis=1)
            absolute_error = float(np.percentile(errors, 95) * 100)
            absolute_note = f"与逐帧真值位置比较；有效帧 {int(valid.sum())}/{n}"
        else:
            absolute_note = "ground_truth.wrist_pose 存在，但没有可比较的有限位置样本"

    jitter_mm, stationary_annotations_available = _stationary_wrist_jitter(df, wrist_pose)
    jitter_note = (
        "按 annotation.wrist_stationary 连续静止段，计算相对段内中位位置的最差 p95 半径"
        if stationary_annotations_available
        else "缺少 annotation.wrist_stationary；不会把正常运动帧猜作静止帧"
    )
    position_step_mm, rotation_step_deg = _wrist_continuity(
        wrist_pose,
        _consecutive_pairs(df),
    )

    metrics = [
        _profile_metric(
            profile, "hand_detection_rate_percent", "detect", "手部检出率",
            round(rate, 1), "canonical", "vis>0 的帧占比",
            measurement_class="detection",
            measurement_basis="visibility_coverage",
        ),
        _profile_metric(
            profile, "wrist_position_absolute_error_p95_cm", "wrist_absolute_position",
            "手腕绝对位置误差(p95)",
            None if absolute_error is None else round(absolute_error, 2),
            "canonical", absolute_note,
            measurement_class="absolute_accuracy",
            measurement_basis="ground_truth_pose_comparison",
            ground_truth_required=True,
            ground_truth_available=ground_truth_available,
            optional_profile_metric=True,
        ),
        _profile_metric(
            profile, "wrist_static_jitter_p95_mm", "wrist_static_jitter",
            "静止手腕位置抖动(p95)",
            None if jitter_mm is None else round(jitter_mm, 2),
            "canonical", jitter_note,
            measurement_class="stability_proxy",
            measurement_basis="annotated_stationary_segment_dispersion",
            optional_profile_metric=True,
        ),
        _profile_metric(
            profile, "wrist_translation_step_p99_mm", "wrist_translation_continuity",
            "手腕位置帧间步长(p99)",
            None if position_step_mm is None else round(position_step_mm, 2),
            "canonical", "仅比较同一 episode 的连续 frame_index；用于发现跳点，不代表位置精度",
            measurement_class="continuity",
            measurement_basis="consecutive_frame_pose_delta",
            optional_profile_metric=True,
        ),
        _profile_metric(
            profile, "wrist_rotation_step_p99_deg", "wrist_rotation_continuity",
            "手腕姿态帧间角度(p99)",
            None if rotation_step_deg is None else round(rotation_step_deg, 2),
            "canonical", "四元数用 abs(dot) 消除 q/-q；仅用于发现旋转跳变，不代表姿态精度",
            measurement_class="continuity",
            measurement_basis="consecutive_frame_quaternion_geodesic",
            optional_profile_metric=True,
        ),
        _profile_metric(
            profile, "bone_length_std_max_mm", "scale", "骨长稳定性代理(最差骨)",
            None if worst_mm is None else round(worst_mm, 1), "canonical",
            (
                "按 episode 分开计算骨长帧间标准差；"
                f"中位{med_mm:.1f}mm。无真值，不代表三维尺度绝对误差"
                if med_mm is not None else "有效帧不足，无法计算骨长稳定性"
            ),
            measurement_class="stability_proxy",
            measurement_basis="within_episode_bone_length_dispersion",
        ),
    ]
    return metrics


def measure_detect_scale(df, profile):
    """Compatibility wrapper for callers using the old function name."""
    return measure_hand_quality(df, profile)


def measure_sync(df, canonical_dir: Path, profile):
    """Measure internal timestamp cadence, not cross-device hardware sync."""
    info_p = canonical_dir / "meta/info.json"
    fps = json.loads(info_p.read_text())["fps"] if info_p.exists() else 30.0
    ts = df["timestamp"].values.astype(float)
    cadence_pairs = _consecutive_pairs(df)
    jitter_ms = np.abs(np.diff(ts)[cadence_pairs] - 1.0 / fps) * 1000
    mx = float(jitter_ms.max()) if len(jitter_ms) else None
    median = float(np.median(jitter_ms)) if len(jitter_ms) else None
    print(f"[时序] fps={fps} 内部帧间隔抖动 中位={median}ms 最大={mx}ms")
    return [_profile_metric(
        profile, "frame_interval_jitter_max_ms", "sync", "内部帧间隔一致性",
        None if mx is None else round(mx, 3), "canonical",
        "同一 episode 连续 frame_index 的 LeRobot timestamp 与 1/fps 一致性；"
        "不是 RGB/Depth/位姿硬件同步",
        measurement_class="timing",
        measurement_basis="dataset_timestamp_cadence",
    )]


def measure_align(df, profile):
    """Keep true RGB-D alignment accuracy separate from a depth-continuity proxy."""
    wp = np.stack(df["observation.wrist_pose"].values)                     # (N,7)
    pair_mask = _consecutive_pairs(df)
    finite = pair_mask & np.isfinite(wp[1:, 2]) & np.isfinite(wp[:-1, 2])
    p99 = None
    if finite.any():
        p99 = float(np.percentile(np.abs(np.diff(wp[:, 2]))[finite], 99) * 1000)
    print(f"[对齐] 缺少像素对应真值；腕深度连续性代理 p99={p99}mm")
    return [
        _profile_metric(
            profile, "rgb_depth_alignment_error_px", "align", "RGB/Depth 对齐误差",
            None, "canonical",
            "缺少标定靶/对应点真值，腕深度连续不能证明 RGB-D 像素对齐精度",
            measurement_class="absolute_accuracy",
            measurement_basis="rgb_depth_correspondence_ground_truth",
            ground_truth_required=True,
            ground_truth_available=False,
            optional_profile_metric=True,
        ),
        _profile_metric(
            profile, "wrist_depth_step_p99_mm", "wrist_depth_continuity",
            "手腕深度帧间步长(p99)",
            None if p99 is None else round(p99, 2), "canonical",
            "同一 episode 连续帧的 Z 变化；仅用于发现深度跳点，不代表 RGB-D 对齐精度",
            measurement_class="continuity",
            measurement_basis="consecutive_frame_wrist_depth_delta",
            optional_profile_metric=True,
        ),
    ]


def measure_retarget(df, robot, profile):
    """指尖重定向误差<1.5cm + 越限 + 跳变。robot FK 向量 vs 缩放人手向量。返回指标列表。"""
    from dex_retargeting.retargeting_config import RetargetingConfig
    from robot_specs import get_spec
    spec = get_spec(robot)
    kps = np.stack(df["observation.hand_keypoints"].values).reshape(-1, 21, 3)
    RetargetingConfig.set_default_urdf_dir(str(spec.urdf_dir))
    rt = RetargetingConfig.load_from_file(str(spec.retarget_cfg),
                                          override={"low_pass_alpha": 1.0}).build()
    opt = rt.optimizer; robot_m = opt.robot
    idx = np.asarray(opt.target_link_human_indices)
    origin_i, task_i = idx[0, :], idx[1, :]
    scaling = opt.scaling if hasattr(opt, "scaling") else 1.15
    o_links = [robot_m.get_link_index(n) for n in opt.origin_link_names]
    t_links = [robot_m.get_link_index(n) for n in opt.task_link_names]
    lower, upper = robot_m.joint_limits[:, 0], robot_m.joint_limits[:, 1]
    errs, qs = [], []
    for f in range(len(kps)):
        ref = kps[f][task_i, :] - kps[f][origin_i, :]
        q = rt.retarget(ref); qs.append(q)
        robot_m.compute_forward_kinematics(q)
        rob_vec = np.array([robot_m.get_link_pose(t)[:3, 3] - robot_m.get_link_pose(o)[:3, 3]
                            for o, t in zip(o_links, t_links)])
        errs.append(np.linalg.norm(rob_vec - ref * scaling, axis=1))
    errs = np.array(errs); qs = np.array(qs)
    med_cm = float(np.median(errs) * 100)
    over = np.maximum(lower - qs, qs - upper)
    real_viol = int((over > 1e-2).sum())
    jump_max = float(np.degrees(np.abs(np.diff(qs, axis=0)).max()))
    retarget_threshold = threshold_text(metric_spec(profile, "retarget_tip_error_median_cm"))
    print(f"[重定向] 综合中位={med_cm:.2f}cm (阈值{retarget_threshold}) "
          f"真越限={real_viol} 帧间跳变max={jump_max:.1f}°")
    return [
        _profile_metric(
            profile, "retarget_tip_error_median_cm", "retarget", "指尖重定向误差(中位)",
            round(med_cm, 2), "embodiment",
            "机器手FK向量vs缩放人手向量。拇指本体差异+单目噪声为主因",
            measurement_class="model_fidelity",
            measurement_basis="robot_fk_vs_scaled_human_vectors",
        ),
        _profile_metric(
            profile, "joint_limit_violation_count", "joint_limit", "关节越限次数",
            real_viol, "embodiment", "真越限(>0.57°);贴限位的饱和不计",
            measurement_class="safety",
            measurement_basis="robot_joint_limit_check",
        ),
        _profile_metric(
            profile, "joint_step_max_deg", "joint_jump", "关节帧间跳变(最大)",
            round(jump_max, 1), "embodiment", "单步最大关节变化;大跳来自单目输入噪声",
            measurement_class="continuity",
            measurement_basis="consecutive_frame_robot_joint_delta",
        ),
    ]


def run_all(robot: str, canonical_dir: Path, profile, profile_source: str):
    """按本体跑全部适用指标,返回 {robot, hand, metrics:[...]}。"""
    hand = _hand_type(robot)
    coordinates = read_ego_coordinate_system(
        canonical_dir,
        required=False,
        allow_legacy_schema=True,
    )
    wrist_pose_frame = (
        "undeclared"
        if coordinates is None
        else coordinates["features"]["observation.wrist_pose"]["frame"]
    )
    df = _canonical_df(canonical_dir)
    metrics = _source_acquisition_metrics(canonical_dir, profile)
    metrics += measure_hand_quality(df, profile)
    metrics += measure_sync(df, canonical_dir, profile)
    if profile["acquisition"]["depth"]["required"]:
        metrics += measure_align(df, profile)
    else:
        metrics.append(_metric(
            "align", "RGB/Depth对齐", None, "", "not applicable", None,
            "canonical", "该 quality profile 不要求 Depth，不能从 RGB 或处理结果推断对齐质量",
            measurement_class="absolute_accuracy",
            measurement_basis="rgb_depth_correspondence_ground_truth",
            ground_truth_required=True,
            ground_truth_available=False,
        ))
    if hand == "gripper":
        metrics += measure_gripper(profile)
    else:
        metrics += measure_retarget(df, robot, profile)
    # 无法测项:内参重投影(缺标定角点数据)
    metrics.append(_profile_metric(
        profile, "camera_reprojection_error_px", "reproj", "内参重投影误差",
        None, "canonical", "缺标定角点/棋盘格原始数据,无法重算。需原始标定图像",
        measurement_class="absolute_accuracy",
        measurement_basis="calibration_target_ground_truth",
        ground_truth_required=True,
        ground_truth_available=False,
    ))
    return {
        "robot": robot,
        "hand": hand,
        "quality_profile": {
            "profile_id": profile["profile_id"],
            "revision": profile["revision"],
            "status": profile.get("status"),
            "source": profile_source,
            "device_class": profile["acquisition"]["device_class"],
            "sync_mode": profile["acquisition"]["sync_mode"],
        },
        "wrist_pose_frame": wrist_pose_frame,
        "metrics": metrics,
    }


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="nero_inspire_rgbd")
    ap.add_argument("--capture-root", default=None,
                    help="Capture Bundle;不传则读取 datasets/captures/ 中最新一次")
    ap.add_argument("--canonical", default=None, help="显式 canonical LeRobotDataset 根目录")
    ap.add_argument("--dataset-root", default=None, help="显式 RobotDataset 根目录")
    ap.add_argument("--legacy-out", action="store_true", help="显式读取旧 src/out")
    ap.add_argument("--target-revision", default="target_revision_v001")
    ap.add_argument("--retarget-revision", default="retarget_v001")
    ap.add_argument("--quality-profile", default=None,
                    help="外部/旧数据显式 profile ID 或 JSON；Capture 默认读取不可变快照")
    ap.add_argument("--json", default=None,
                    help="报告输出路径;Capture 模式默认写入 reports/retargeting/")
    args = ap.parse_args()
    try:
        data_paths = resolve_data_paths(
            args.robot,
            capture_root=args.capture_root,
            canonical_root=args.canonical,
            output_root=args.dataset_root,
            legacy_out=args.legacy_out,
            target_revision=args.target_revision,
            retarget_revision=args.retarget_revision,
        )
    except (ValueError, FileNotFoundError) as error:
        raise SystemExit(str(error)) from error
    try:
        quality_profile, profile_source = _resolve_quality_profile(
            data_paths.canonical_root,
            args.quality_profile,
        )
    except (ValueError, FileNotFoundError) as error:
        raise SystemExit(str(error)) from error
    result = run_all(
        args.robot,
        data_paths.canonical_root,
        quality_profile,
        profile_source,
    )
    json_path = Path(args.json) if args.json else (
        data_paths.quality_report if data_paths.capture is not None else None
    )
    if json_path is not None:
        def _np(o):
            if isinstance(o, np.bool_):
                return bool(o)
            if isinstance(o, np.integer):
                return int(o)
            if isinstance(o, np.floating):
                return float(o)
            raise TypeError(o)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2, default=_np) + "\n",
                             encoding="utf-8")
        print(f"[验收] 写入 {json_path}")
