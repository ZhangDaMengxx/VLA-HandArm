from __future__ import annotations

import copy
import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import pytest


SRC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SRC))

from capture_bundle import create_capture, record_source  # noqa: E402
from measure_acceptance import (  # noqa: E402
    _resolve_quality_profile,
    _source_acquisition_metrics,
    measure_align,
    measure_hand_quality,
    measure_sync,
)
from quality_profiles import (  # noqa: E402
    PROFILE_ROOT,
    evaluate_threshold,
    load_quality_profile,
    metric_spec,
    read_quality_profile_snapshot,
    write_quality_profile_snapshot,
)


def _by_key(metrics: list[dict], key: str) -> dict:
    return next(metric for metric in metrics if metric["key"] == key)


def test_repository_quality_profiles_are_valid_and_versioned() -> None:
    paths = sorted(PROFILE_ROOT.glob("*.json"))
    assert {path.stem for path in paths} == {
        "ego_fixed_rgbd_60hz_v1",
        "legacy_aligned_rgbd_30hz_v1",
        "legacy_rgb_video_30hz_v1",
        "processed_observations_v1",
    }
    for path in paths:
        profile = load_quality_profile(path.stem)
        assert profile["profile_id"] == path.stem
        assert profile["schema_version"] == "1.1"
        assert profile["revision"] >= 2
        assert all(
            "measurement_class" in spec and "ground_truth_required" in spec
            for spec in profile["metrics"].values()
        )


def test_profile_threshold_comparisons_keep_strict_boundaries() -> None:
    profile = load_quality_profile("ego_fixed_rgbd_60hz_v1")
    detection = metric_spec(profile, "hand_detection_rate_percent")
    retarget = metric_spec(profile, "retarget_tip_error_median_cm")

    assert evaluate_threshold(90, detection) is True
    assert evaluate_threshold(89.9, detection) is False
    assert evaluate_threshold(0.99, retarget) is True
    assert evaluate_threshold(1.0, retarget) is False


def test_measured_fps_threshold_requires_nominal_fps() -> None:
    profile = copy.deepcopy(load_quality_profile("ego_fixed_rgbd_60hz_v1"))
    profile["acquisition"]["rgb"].pop("min_fps")
    with pytest.raises(ValueError, match="min_measured_fps requires min_fps"):
        from quality_profiles import validate_quality_profile

        validate_quality_profile(profile)


def test_capture_quality_profile_snapshot_is_immutable(tmp_path: Path) -> None:
    source = tmp_path / "source"
    profile = load_quality_profile("legacy_rgb_video_30hz_v1")
    destination = write_quality_profile_snapshot(source, profile)

    assert read_quality_profile_snapshot(source) == profile
    assert write_quality_profile_snapshot(source, profile) == destination
    changed = copy.deepcopy(profile)
    changed["metrics"]["hand_detection_rate_percent"]["value"] = 80
    with pytest.raises(ValueError, match="immutable"):
        write_quality_profile_snapshot(source, changed)


def test_record_source_rejects_profile_for_another_source_kind(tmp_path: Path) -> None:
    capture = create_capture(tmp_path, datetime(2026, 8, 20))
    source = tmp_path / "input.mp4"
    source.write_bytes(b"video")
    profile = load_quality_profile("processed_observations_v1")

    with pytest.raises(ValueError, match="does not apply"):
        record_source(
            capture,
            kind="rgb_video",
            source=source,
            config={"fps": 30},
            quality_profile=profile,
        )


def test_target_profile_does_not_accept_legacy_rgbd_capabilities(tmp_path: Path) -> None:
    capture = create_capture(tmp_path, datetime(2026, 8, 20))
    source = tmp_path / "rgbd"
    source.mkdir()
    profile = load_quality_profile("ego_fixed_rgbd_60hz_v1")
    record_source(
        capture,
        kind="aligned_rgbd",
        source=source,
        config={
            "fps": 30,
            "rgb_width": 960,
            "rgb_height": 540,
            "depth_width": 960,
            "depth_height": 540,
        },
        hardware_timestamps_available=False,
        quality_profile=profile,
    )

    metrics = _source_acquisition_metrics(capture.ego, profile)
    assert _by_key(metrics, "source_rgb_fps")["pass"] is False
    assert _by_key(metrics, "source_depth_fps")["pass"] is False
    assert _by_key(metrics, "source_rgb_resolution")["pass"] is False
    assert _by_key(metrics, "source_depth_resolution")["pass"] is True
    assert _by_key(metrics, "source_hardware_timestamps")["pass"] is False
    assert _by_key(metrics, "source_rgb_depth_sync")["pass"] is False

    acquisition = json.loads(
        (capture.source / "acquisition.json").read_text(encoding="utf-8")
    )
    assert acquisition["quality_profile"] == {
        "path": "quality_profile.json",
        "profile_id": "ego_fixed_rgbd_60hz_v1",
        "revision": 4,
    }


def test_60hz_profile_requires_measured_hardware_timestamp_cadence(tmp_path: Path) -> None:
    pd = pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    capture = create_capture(tmp_path, datetime(2026, 8, 27))
    source = tmp_path / "rgbd"
    source.mkdir()
    profile = load_quality_profile("ego_fixed_rgbd_60hz_v1")
    record_source(
        capture,
        kind="native_rgbd",
        source=source,
        config={
            "rgb_fps": 60,
            "depth_fps": 60,
            "rgb_width": 1280,
            "rgb_height": 800,
            "depth_width": 848,
            "depth_height": 480,
        },
        hardware_timestamps_available=True,
        quality_profile=profile,
    )

    missing = _source_acquisition_metrics(capture.ego, profile)
    assert _by_key(missing, "source_rgb_fps")["pass"] is False
    assert _by_key(missing, "source_depth_fps")["pass"] is False

    timestamps = np.arange(121, dtype=np.int64) * 16_667 + 1_000_000
    pd.DataFrame({
        "episode_index": np.zeros(len(timestamps), dtype=np.int32),
        "rgb_timestamp_hw_us": timestamps,
        "depth_timestamp_hw_us": timestamps + 1_000,
        "sync_error_ms": np.ones(len(timestamps)),
    }).to_parquet(capture.source / "stream_index.parquet", index=False)

    measured = _source_acquisition_metrics(capture.ego, profile)
    rgb = _by_key(measured, "source_rgb_fps")
    depth = _by_key(measured, "source_depth_fps")
    assert rgb["pass"] is True
    assert depth["pass"] is True
    assert 59.9 < rgb["value"] < 60.1
    assert rgb["measurement_basis"] == "source_hardware_timestamp_cadence"

    slow_timestamps = np.arange(121, dtype=np.int64) * 16_950 + 1_000_000
    pd.DataFrame({
        "episode_index": np.zeros(len(slow_timestamps), dtype=np.int32),
        "rgb_timestamp_hw_us": slow_timestamps,
        "depth_timestamp_hw_us": slow_timestamps + 1_000,
        "sync_error_ms": np.ones(len(slow_timestamps)),
    }).to_parquet(capture.source / "stream_index.parquet", index=False)
    slow = _source_acquisition_metrics(capture.ego, profile)
    assert _by_key(slow, "source_rgb_fps")["pass"] is False
    assert _by_key(slow, "source_depth_fps")["pass"] is False


def test_acceptance_uses_capture_snapshot_and_rejects_override(tmp_path: Path) -> None:
    capture = create_capture(tmp_path, datetime(2026, 8, 20))
    profile = load_quality_profile("legacy_rgb_video_30hz_v1")
    write_quality_profile_snapshot(capture.source, profile)

    selected, source = _resolve_quality_profile(capture.ego, None)
    assert selected == profile
    assert source == "capture_snapshot"
    with pytest.raises(ValueError, match="differs"):
        _resolve_quality_profile(capture.ego, "processed_observations_v1")


def test_internal_cadence_metric_does_not_claim_hardware_sync(tmp_path: Path) -> None:
    pd = pytest.importorskip("pandas")
    dataset = tmp_path / "robot"
    (dataset / "meta").mkdir(parents=True)
    (dataset / "meta/info.json").write_text(json.dumps({"fps": 60}), encoding="utf-8")
    frame = pd.DataFrame({"timestamp": [0.0, 1.0 / 60.0, 2.0 / 60.0]})
    profile = load_quality_profile("ego_fixed_rgbd_60hz_v1")

    metric = measure_sync(frame, dataset, profile)[0]
    assert metric["pass"] is True
    assert metric["label"] == "内部帧间隔一致性"
    assert "不是 RGB/Depth/位姿硬件同步" in metric["note"]
    assert metric["measurement_class"] == "timing"


def test_internal_cadence_ignores_episode_timestamp_reset(tmp_path: Path) -> None:
    pd = pytest.importorskip("pandas")
    dataset = tmp_path / "ego"
    (dataset / "meta").mkdir(parents=True)
    (dataset / "meta/info.json").write_text(json.dumps({"fps": 60}), encoding="utf-8")
    frame = pd.DataFrame({
        "episode_index": [0, 0, 1, 1],
        "frame_index": [0, 1, 0, 1],
        "timestamp": [0.0, 1.0 / 60.0, 0.0, 1.0 / 60.0],
    })
    profile = load_quality_profile("ego_fixed_rgbd_60hz_v1")

    metric = measure_sync(frame, dataset, profile)[0]
    assert metric["value"] == 0.0
    assert metric["pass"] is True


def _hand_frame(include_ground_truth: bool = False):
    pd = pytest.importorskip("pandas")
    rows = []
    base_keypoints = np.zeros((21, 3), dtype=float)
    base_keypoints[:, 0] = np.arange(21) * 0.01
    for episode, frame, x, quat_sign in (
        (0, 0, 0.000, 1.0),
        (0, 1, 0.001, -1.0),
        (0, 2, -0.001, 1.0),
        (1, 0, 1.000, 1.0),
        (1, 1, 1.001, 1.0),
        (1, 2, 0.999, 1.0),
    ):
        wrist = np.array([x, 0.0, 0.5, 0.0, 0.0, 0.0, quat_sign])
        row = {
            "episode_index": episode,
            "frame_index": frame,
            "observation.hand_visibility": np.ones(21),
            "observation.hand_keypoints": base_keypoints.reshape(-1),
            "observation.wrist_pose": wrist,
            "annotation.wrist_stationary": True,
        }
        if include_ground_truth:
            truth = wrist.copy()
            truth[0] -= 0.005
            row["ground_truth.wrist_pose"] = truth
        rows.append(row)
    return pd.DataFrame(rows)


def test_absolute_wrist_accuracy_is_unmeasured_without_ground_truth() -> None:
    profile = load_quality_profile("processed_observations_v1")
    metrics = measure_hand_quality(_hand_frame(), profile)

    absolute = _by_key(metrics, "wrist_absolute_position")
    assert absolute["value"] is None
    assert absolute["pass"] is None
    assert absolute["measurement_class"] == "absolute_accuracy"
    assert absolute["ground_truth_required"] is True
    assert absolute["ground_truth_available"] is False
    assert "不能替代绝对精度" in absolute["note"]

    stability = _by_key(metrics, "scale")
    assert stability["pass"] is True
    assert stability["measurement_class"] == "stability_proxy"
    assert "不代表三维尺度绝对误差" in stability["note"]


def test_wrist_ground_truth_and_proxy_metrics_remain_separate() -> None:
    profile = load_quality_profile("processed_observations_v1")
    metrics = measure_hand_quality(_hand_frame(include_ground_truth=True), profile)

    absolute = _by_key(metrics, "wrist_absolute_position")
    assert absolute["value"] == 0.5
    assert absolute["pass"] is True
    assert absolute["ground_truth_available"] is True

    jitter = _by_key(metrics, "wrist_static_jitter")
    assert jitter["measurement_basis"] == "annotated_stationary_segment_dispersion"
    assert jitter["pass"] is True
    translation = _by_key(metrics, "wrist_translation_continuity")
    assert translation["value"] < 3.0
    assert translation["pass"] is True
    rotation = _by_key(metrics, "wrist_rotation_continuity")
    assert rotation["value"] == 0.0
    assert rotation["pass"] is True


def test_rgb_depth_continuity_does_not_claim_alignment_accuracy() -> None:
    profile = load_quality_profile("legacy_aligned_rgbd_30hz_v1")
    metrics = measure_align(_hand_frame(), profile)

    alignment = _by_key(metrics, "align")
    assert alignment["value"] is None
    assert alignment["pass"] is None
    assert alignment["ground_truth_required"] is True
    assert alignment["ground_truth_available"] is False
    depth_proxy = _by_key(metrics, "wrist_depth_continuity")
    assert depth_proxy["measurement_class"] == "continuity"
    assert depth_proxy["pass"] is True


def test_schema_10_profile_snapshot_remains_readable(tmp_path: Path) -> None:
    profile = copy.deepcopy(load_quality_profile("processed_observations_v1"))
    profile["schema_version"] = "1.0"
    profile["revision"] = 1
    for spec in profile["metrics"].values():
        spec.pop("measurement_class")
        spec.pop("ground_truth_required")
    source = tmp_path / "source"
    write_quality_profile_snapshot(source, profile)
    assert read_quality_profile_snapshot(source) == profile
