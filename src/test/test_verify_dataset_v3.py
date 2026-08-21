from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest


SRC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SRC))

from capture_bundle import (  # noqa: E402
    WRIST_FRAME_EPISODE0_CAMERA,
    create_capture,
    write_ego_metadata,
    write_robot_metadata,
)
from verify_dataset import (  # noqa: E402
    StrictV3Error,
    validate_capture_bundle,
    validate_strict_lerobot_v3,
)


def _write_fixture(root: Path, *, video: bool = True) -> None:
    pd = pytest.importorskip("pandas")
    (root / "meta/episodes/chunk-000").mkdir(parents=True)
    (root / "data/chunk-000").mkdir(parents=True)
    features = {
        "observation.state": {"dtype": "float32", "shape": [1], "names": ["state"]},
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
    }
    if video:
        features["observation.images.ego"] = {
            "dtype": "video", "shape": [2, 2, 3], "names": ["height", "width", "channel"]
        }
        path = root / "videos/observation.images.ego/chunk-000/file-000.mp4"
        path.parent.mkdir(parents=True)
        path.write_bytes(b"fixture")
    (root / "meta/info.json").write_text(json.dumps({
        "codebase_version": "v3.0",
        "total_episodes": 1,
        "total_frames": 2,
        "total_tasks": 1,
        "fps": 30,
        "features": features,
        "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
        "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
    }), encoding="utf-8")
    (root / "meta/stats.json").write_text(
        json.dumps({key: {} for key in features}), encoding="utf-8"
    )
    pd.DataFrame({
        "observation.state": [[0.0], [1.0]],
        "timestamp": [0.0, 1 / 30],
        "frame_index": [0, 1],
        "episode_index": [0, 0],
        "index": [0, 1],
        "task_index": [0, 0],
    }).to_parquet(root / "data/chunk-000/file-000.parquet")
    pd.DataFrame({"task_index": [0], "task": ["test"]}).to_parquet(
        root / "meta/tasks.parquet"
    )
    pd.DataFrame({"episode_index": [0], "tasks": [["test"]], "length": [2]}).to_parquet(
        root / "meta/episodes/chunk-000/file-000.parquet"
    )


def test_strict_v3_accepts_complete_dataset(tmp_path: Path) -> None:
    _write_fixture(tmp_path)
    result = validate_strict_lerobot_v3(tmp_path)
    assert result == {
        "codebase_version": "v3.0",
        "frames": 2,
        "episodes": 1,
        "tasks": 1,
        "features": 7,
        "video_features": ["observation.images.ego"],
    }


def test_strict_v3_rejects_count_mismatch(tmp_path: Path) -> None:
    _write_fixture(tmp_path, video=False)
    info_path = tmp_path / "meta/info.json"
    info = json.loads(info_path.read_text(encoding="utf-8"))
    info["total_frames"] = 3
    info_path.write_text(json.dumps(info), encoding="utf-8")
    with pytest.raises(StrictV3Error, match="total_frames"):
        validate_strict_lerobot_v3(tmp_path)


def test_strict_v3_requires_video_for_video_feature(tmp_path: Path) -> None:
    _write_fixture(tmp_path)
    (tmp_path / "videos/observation.images.ego/chunk-000/file-000.mp4").unlink()
    with pytest.raises(StrictV3Error, match="has no chunked MP4"):
        validate_strict_lerobot_v3(tmp_path)


def test_strict_v3_rejects_anonymous_legacy_task_index(tmp_path: Path) -> None:
    pd = pytest.importorskip("pandas")
    _write_fixture(tmp_path, video=False)
    tasks = pd.DataFrame({"task_index": [0]}, index=["test"])
    tasks.to_parquet(tmp_path / "meta/tasks.parquet")
    with pytest.raises(StrictV3Error, match="task_index and task"):
        validate_strict_lerobot_v3(tmp_path)


def test_capture_bundle_validator_covers_source_lineage_sidecars_and_checksums(
    tmp_path: Path,
) -> None:
    pd = pytest.importorskip("pandas")
    capture = create_capture(tmp_path)
    _write_fixture(capture.ego)
    (capture.source / "acquisition.json").write_text("{}\n", encoding="utf-8")
    (capture.source / "quality_profile.json").write_text("{}\n", encoding="utf-8")
    pd.DataFrame({"episode_index": [0], "frame_index": [0]}).to_parquet(
        capture.source / "stream_index.parquet"
    )
    write_ego_metadata(
        capture,
        estimator="mediapipe",
        source_kind="test",
        wrist_pose_frame=WRIST_FRAME_EPISODE0_CAMERA,
        wrist_pose_frame_declared_by="test_fixture",
    )

    robot = capture.robot("robot")
    _write_fixture(robot.dataset_root, video=False)
    write_robot_metadata(capture, target_id="robot", dataset_root=robot.dataset_root)

    result = validate_capture_bundle(capture.root)
    assert result["status"] == "passed"
    assert result["ego"]["episode_sidecars"] == 1
    assert result["robots"]["robot"]["episode_sidecars"] == 1


def test_capture_bundle_validator_detects_checksum_drift(tmp_path: Path) -> None:
    pd = pytest.importorskip("pandas")
    capture = create_capture(tmp_path)
    _write_fixture(capture.ego, video=False)
    (capture.source / "acquisition.json").write_text("{}\n", encoding="utf-8")
    (capture.source / "quality_profile.json").write_text("{}\n", encoding="utf-8")
    pd.DataFrame({"episode_index": [0], "frame_index": [0]}).to_parquet(
        capture.source / "stream_index.parquet"
    )
    write_ego_metadata(
        capture,
        estimator="mediapipe",
        source_kind="test",
        wrist_pose_frame=WRIST_FRAME_EPISODE0_CAMERA,
        wrist_pose_frame_declared_by="test_fixture",
    )
    info_path = capture.ego / "meta/info.json"
    info_path.write_text(info_path.read_text(encoding="utf-8") + "\n", encoding="utf-8")

    with pytest.raises(StrictV3Error, match="checksum mismatch"):
        validate_capture_bundle(capture.root)
