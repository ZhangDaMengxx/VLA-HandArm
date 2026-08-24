from __future__ import annotations

import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SRC))

from capture_bundle import (  # noqa: E402
    CAPTURE_BUILDING,
    CAPTURE_FAILED,
    CAPTURE_READY,
    WRIST_FRAME_EPISODE0_CAMERA,
    WRIST_FRAME_SCENE_WORLD,
    CapturePaths,
    archive_aligned_rgbd,
    archive_processed_input,
    create_capture,
    discover_trajectory_npz,
    latest_capture,
    legacy_data_paths,
    open_capture,
    read_ego_coordinate_system,
    record_source,
    resolve_ego_output,
    resolve_ego_input,
    resolve_data_paths,
    write_ego_frame_mapping,
    write_ego_metadata,
    write_multisensor_source_index,
    write_robot_metadata,
)


def _ready(capture: CapturePaths) -> CapturePaths:
    (capture.ego / "meta").mkdir(parents=True, exist_ok=True)
    (capture.ego / "meta/info.json").write_text("{}", encoding="utf-8")
    write_ego_metadata(
        capture,
        estimator="mediapipe",
        source_kind="test",
        wrist_pose_frame=WRIST_FRAME_EPISODE0_CAMERA,
        wrist_pose_frame_declared_by="test_fixture",
    )
    return capture


def test_create_capture_allocates_sequence_and_contract(tmp_path: Path) -> None:
    when = datetime(2026, 8, 20, 10, 0, 0)
    first = create_capture(tmp_path, when)
    second = create_capture(tmp_path, when)

    assert first.capture_id.startswith("capture_20260820_000000_")
    assert second.capture_id.startswith("capture_20260820_000001_")
    assert first.ego == first.root / "ego"
    assert first.source.is_dir()
    manifest = json.loads(first.bundle_manifest.read_text(encoding="utf-8"))
    assert manifest["data_conventions"]["quaternion_order"] == "xyzw"
    assert manifest["data_conventions"]["canonical_wrist_pose"].endswith("qx,qy,qz,qw")
    assert manifest["datasets"]["ego"] == "ego"
    assert manifest["status"] == CAPTURE_BUILDING
    assert manifest["stages"]["ego"]["status"] == CAPTURE_BUILDING


def test_create_capture_allocates_unique_sequences_concurrently(tmp_path: Path) -> None:
    when = datetime(2026, 8, 20, 10, 0, 0)
    with ThreadPoolExecutor(max_workers=8) as pool:
        captures = list(pool.map(lambda _: create_capture(tmp_path, when), range(8)))

    sequences = sorted(int(item.capture_id.split("_")[2]) for item in captures)
    assert sequences == list(range(8))


def test_robot_paths_keep_dataset_independent_from_reports(tmp_path: Path) -> None:
    paths = create_capture(tmp_path, datetime(2026, 8, 20))
    robot = paths.robot("nero_inspire_rgbd")

    assert robot.dataset_root.relative_to(paths.root).as_posix() == (
        "robot_datasets/nero_inspire_rgbd/target_revision_v001/retarget_v001"
    )
    assert robot.trajectory_pkl.parent.relative_to(robot.dataset_root).as_posix() == (
        "exports/workbench"
    )
    assert paths.reports in robot.quality_report.parents
    assert paths.reports not in robot.dataset_root.parents


def test_latest_and_open_capture_do_not_fall_back_to_legacy(tmp_path: Path) -> None:
    assert latest_capture(tmp_path) is None
    created = create_capture(tmp_path, datetime(2026, 8, 20))
    assert latest_capture(tmp_path) is None
    _ready(created)
    assert latest_capture(tmp_path) == CapturePaths(created.root.resolve())
    assert latest_capture(tmp_path).status == CAPTURE_READY
    assert open_capture(created.capture_id, tmp_path).root == created.root.resolve()
    with pytest.raises(ValueError):
        open_capture(tmp_path / "not_a_capture", tmp_path)


def test_latest_capture_skips_newer_building_and_failed_batches(tmp_path: Path) -> None:
    ready = _ready(create_capture(tmp_path, datetime(2026, 8, 20)))
    building = create_capture(tmp_path, datetime(2026, 8, 20))
    failed = create_capture(tmp_path, datetime(2026, 8, 20))
    failed.mark_ego_failed("synthetic failure")

    assert building.status == CAPTURE_BUILDING
    assert failed.status == CAPTURE_FAILED
    assert latest_capture(tmp_path) == CapturePaths(ready.root.resolve())
    assert open_capture(building.root).status == CAPTURE_BUILDING
    assert open_capture(failed.root).status == CAPTURE_FAILED


def test_unfinished_builder_is_failed_on_normal_process_exit(tmp_path: Path) -> None:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(SRC)
    env["CAPTURE_TEST_ROOT"] = str(tmp_path)
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import os; from capture_bundle import resolve_ego_output; "
            "resolve_ego_output(captures_root=os.environ['CAPTURE_TEST_ROOT'])",
        ],
        check=True,
        env=env,
    )

    created = next(tmp_path.glob("capture_*"))
    assert open_capture(created).status == CAPTURE_FAILED
    assert latest_capture(tmp_path) is None


def test_legacy_paths_are_explicit_and_unchanged() -> None:
    legacy = legacy_data_paths("nero_inspire_rgbd")
    assert legacy.canonical_root.as_posix().endswith("src/out/canonical_ds")
    assert legacy.dataset_root.as_posix().endswith("src/out/lerobot_ds_nero_inspire_rgbd")
    assert legacy.trajectory_pkl.as_posix().endswith("src/out/robot_traj_nero_inspire_rgbd.pkl")


def test_ego_output_defaults_to_new_capture_and_legacy_is_explicit(
    tmp_path: Path,
) -> None:
    root, capture = resolve_ego_output(captures_root=tmp_path)
    assert capture is not None
    assert root == capture.ego
    assert capture.status == CAPTURE_BUILDING

    legacy_root, legacy_capture = resolve_ego_output(legacy_out=True)
    assert legacy_capture is None
    assert legacy_root.as_posix().endswith("src/out/canonical_ds")
    with pytest.raises(ValueError):
        resolve_ego_output(capture_root=capture.root, legacy_out=True)
    with pytest.raises(ValueError):
        resolve_ego_output(output_root=capture.source)


def test_ego_input_reads_latest_without_creating_or_falling_back(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        resolve_ego_input(captures_root=tmp_path)
    capture = create_capture(tmp_path, datetime(2026, 8, 20))
    _ready(capture)
    root, selected = resolve_ego_input(captures_root=tmp_path)
    assert root == capture.ego
    assert selected == CapturePaths(capture.root.resolve())

    external = tmp_path / "external"
    root, selected = resolve_ego_input(input_root=external)
    assert root == external.resolve()
    assert selected is None
    with pytest.raises(ValueError):
        resolve_ego_input(input_root=external, legacy_out=True)


def test_discover_trajectory_npz_is_scoped_to_one_capture(tmp_path: Path) -> None:
    first = _ready(create_capture(tmp_path, datetime(2026, 8, 20)))
    old = first.robot("old").trajectory_npz
    old.parent.mkdir(parents=True)
    old.write_bytes(b"old")
    second = _ready(create_capture(tmp_path, datetime(2026, 8, 20)))
    expected = second.robot("new").trajectory_npz
    expected.parent.mkdir(parents=True)
    expected.write_bytes(b"new")

    assert discover_trajectory_npz(captures_root=tmp_path) == [expected]
    assert discover_trajectory_npz(capture_root=first.root, captures_root=tmp_path) == [old]
    with pytest.raises(ValueError):
        discover_trajectory_npz(capture_root=first.root, legacy_out=True)


def test_data_paths_resolve_latest_capture_without_legacy_fallback(tmp_path: Path) -> None:
    capture = _ready(create_capture(tmp_path, datetime(2026, 8, 20)))
    paths = resolve_data_paths("nero_inspire_rgbd", captures_root=tmp_path)
    assert paths.capture == CapturePaths(capture.root.resolve())
    assert paths.canonical_root == capture.ego
    assert paths.dataset_root == capture.robot("nero_inspire_rgbd").dataset_root
    assert paths.trajectory_npz.name == "robot_traj.npz"


def test_external_data_paths_require_both_roots(tmp_path: Path) -> None:
    canonical = tmp_path / "external_ego"
    canonical.mkdir()
    with pytest.raises(ValueError):
        resolve_data_paths("robot", canonical_root=canonical, captures_root=tmp_path / "none")
    output = tmp_path / "external_robot"
    paths = resolve_data_paths(
        "robot", canonical_root=canonical, output_root=output, captures_root=tmp_path / "none"
    )
    assert paths.capture is None
    assert paths.canonical_root == canonical.resolve()
    assert paths.dataset_root == output.resolve()


def test_data_paths_reject_mixed_or_wrong_capture_roots(tmp_path: Path) -> None:
    first = create_capture(tmp_path, datetime(2026, 8, 20))
    second = create_capture(tmp_path, datetime(2026, 8, 20))
    first_robot = first.robot("robot").dataset_root
    second_robot = second.robot("robot").dataset_root

    with pytest.raises(ValueError):
        resolve_data_paths("robot", capture_root=first.root, canonical_root=first.ego)
    with pytest.raises(ValueError):
        resolve_data_paths("robot", canonical_root=first.ego, output_root=second_robot)
    with pytest.raises(ValueError):
        resolve_data_paths("robot", canonical_root=first.source, output_root=first_robot)
    with pytest.raises(ValueError):
        resolve_data_paths("robot", canonical_root=first.ego, output_root=first.source)

    paths = resolve_data_paths("robot", canonical_root=first.ego, output_root=first_robot)
    assert paths.capture == CapturePaths(first.root.resolve())


def test_metadata_records_lineage_and_checksums(tmp_path: Path) -> None:
    pd = pytest.importorskip("pandas")
    capture = create_capture(tmp_path, datetime(2026, 8, 20))
    (capture.ego / "meta").mkdir(parents=True)
    (capture.ego / "data/chunk-000").mkdir(parents=True)
    pd.DataFrame({"episode_index": [0], "frame_index": [0]}).to_parquet(
        capture.ego / "data/chunk-000/file-000.parquet"
    )
    write_ego_metadata(
        capture,
        estimator="mediapipe",
        source_kind="test",
        wrist_pose_frame=WRIST_FRAME_EPISODE0_CAMERA,
        wrist_pose_frame_declared_by="test_fixture",
    )
    assert capture.status == CAPTURE_READY

    robot = capture.robot("robot")
    (robot.dataset_root / "meta").mkdir(parents=True)
    (robot.dataset_root / "meta/info.json").write_text(
        json.dumps({"features": {"observation.state": {"shape": [2], "names": ["a", "b"]}}}),
        encoding="utf-8",
    )
    (robot.dataset_root / "data/chunk-000").mkdir(parents=True)
    pd.DataFrame({
        "episode_index": [0, 0],
        "frame_index": [0, 1],
        "observation.state": [[0.0, 1.0], [0.1, 1.1]],
        "action": [[0.1, 1.1], [0.2, 1.2]],
    }).to_parquet(robot.dataset_root / "data/chunk-000/file-000.parquet")
    write_robot_metadata(capture, target_id="robot", dataset_root=robot.dataset_root)

    relations = json.loads((capture.lineage / "dataset_relations.json").read_text(encoding="utf-8"))
    assert relations["ego"] == "ego"
    assert relations["robots"]["robot"].endswith("retarget_v001")
    assert (capture.ego / "checksums.json").is_file()
    assert (robot.dataset_root / "checksums.json").is_file()
    runtime = json.loads((capture.environment / "runtime.json").read_text(encoding="utf-8"))
    assert runtime["python"]
    assert "numpy" in runtime["dependencies"]
    assert (capture.environment / "requirements.txt").is_file()
    assert (capture.environment / "environment.lock").is_file()
    bundle = json.loads(capture.bundle_manifest.read_text(encoding="utf-8"))
    assert bundle["layout"]["environment"] == "environment"
    schema = json.loads((robot.dataset_root / "meta/robot_schema.json").read_text(encoding="utf-8"))
    assert schema["joint_order"] == ["a", "b"]
    source_ego = json.loads((robot.dataset_root / "meta/source_ego.json").read_text(encoding="utf-8"))
    assert source_ego["wrist_pose_frame"] == WRIST_FRAME_EPISODE0_CAMERA
    ego_annotation = json.loads(
        (capture.ego / "annotations/episode_000000.json").read_text(encoding="utf-8")
    )
    assert ego_annotation["review_status"] == "unreviewed"
    robot_qa = json.loads(
        (robot.dataset_root / "qa/episode_000000.json").read_text(encoding="utf-8")
    )
    assert robot_qa["automated_status"] == "passed"
    assert robot_qa["physical_checks"]["collision"]["status"] == "not_evaluated"
    bundle = json.loads(capture.bundle_manifest.read_text(encoding="utf-8"))
    assert bundle["stages"]["robots"]["robot"]["status"] == CAPTURE_READY


def test_episode_annotation_generation_preserves_human_review(tmp_path: Path) -> None:
    pd = pytest.importorskip("pandas")
    capture = create_capture(tmp_path, datetime(2026, 8, 20))
    (capture.ego / "data/chunk-000").mkdir(parents=True)
    pd.DataFrame({"episode_index": [0], "frame_index": [0]}).to_parquet(
        capture.ego / "data/chunk-000/file-000.parquet"
    )
    annotation = capture.ego / "annotations/episode_000000.json"
    annotation.parent.mkdir(parents=True)
    annotation.write_text(
        json.dumps({"episode_index": 0, "review_status": "reviewed", "notes": "human"}),
        encoding="utf-8",
    )

    write_ego_metadata(
        capture,
        estimator="mediapipe",
        source_kind="test",
        wrist_pose_frame=WRIST_FRAME_EPISODE0_CAMERA,
        wrist_pose_frame_declared_by="test_fixture",
    )

    assert json.loads(annotation.read_text(encoding="utf-8"))["notes"] == "human"


def test_coordinate_contract_uses_explicit_feature_frame_not_source_kind(tmp_path: Path) -> None:
    episode_camera = create_capture(tmp_path, datetime(2026, 8, 20))
    scene_world = create_capture(tmp_path, datetime(2026, 8, 20))

    write_ego_metadata(
        episode_camera,
        estimator="mediapipe",
        source_kind="same_source_kind",
        wrist_pose_frame=WRIST_FRAME_EPISODE0_CAMERA,
        wrist_pose_frame_declared_by="test_episode_camera",
    )
    write_ego_metadata(
        scene_world,
        estimator="mediapipe",
        source_kind="same_source_kind",
        wrist_pose_frame=WRIST_FRAME_SCENE_WORLD,
        wrist_pose_frame_declared_by="test_scene_world",
        hand_keypoints_frame=WRIST_FRAME_SCENE_WORLD,
    )

    camera_contract = read_ego_coordinate_system(episode_camera.ego)
    world_contract = read_ego_coordinate_system(scene_world.ego)
    assert camera_contract["schema_version"] == "2.0"
    assert camera_contract["features"]["observation.wrist_pose"]["frame"] == (
        WRIST_FRAME_EPISODE0_CAMERA
    )
    assert world_contract["features"]["observation.wrist_pose"]["frame"] == (
        WRIST_FRAME_SCENE_WORLD
    )
    assert world_contract["features"]["observation.hand_keypoints"]["frame"] == (
        WRIST_FRAME_SCENE_WORLD
    )
    camera_source = json.loads(
        (episode_camera.ego / "meta/source_reference.json").read_text(encoding="utf-8")
    )
    world_source = json.loads(
        (scene_world.ego / "meta/source_reference.json").read_text(encoding="utf-8")
    )
    assert camera_source["source_kind"] == world_source["source_kind"]


def test_coordinate_contract_rejects_unknown_frames(tmp_path: Path) -> None:
    capture = create_capture(tmp_path, datetime(2026, 8, 20))
    with pytest.raises(ValueError, match="wrist pose frame"):
        write_ego_metadata(
            capture,
            estimator="mediapipe",
            source_kind="test",
            wrist_pose_frame="source_kind_guess",
            wrist_pose_frame_declared_by="test",
        )


def test_robot_lineage_preserves_legacy_unknown_frame_without_guessing(tmp_path: Path) -> None:
    capture = create_capture(tmp_path, datetime(2026, 8, 20))
    meta = capture.ego / "meta"
    meta.mkdir(parents=True)
    (meta / "coordinate_system.json").write_text(
        json.dumps({"schema_version": "1.0", "wrist_pose_frame": "ambiguous_legacy"}),
        encoding="utf-8",
    )
    capture.mark_ego_ready()

    robot = capture.robot("legacy_robot")
    (robot.dataset_root / "meta").mkdir(parents=True)
    write_robot_metadata(capture, target_id="legacy_robot", dataset_root=robot.dataset_root)

    source_ego = json.loads(
        (robot.dataset_root / "meta/source_ego.json").read_text(encoding="utf-8")
    )
    assert source_ego["wrist_pose_frame"] is None
    assert source_ego["coordinate_contract_status"] == "legacy_undeclared"


def test_processed_input_is_preserved_as_source_recording(tmp_path: Path) -> None:
    capture = create_capture(tmp_path, datetime(2026, 8, 20))
    source = tmp_path / "observations.npz"
    source.write_bytes(b"processed-observations")

    archived = archive_processed_input(capture, source)

    assert archived.relative_to(capture.source).as_posix() == (
        "recordings/processed_input_000000.npz"
    )
    assert archived.read_bytes() == source.read_bytes()


def test_aligned_rgbd_source_is_archived_and_indexed_without_fake_timestamps(
    tmp_path: Path,
) -> None:
    pq = pytest.importorskip("pyarrow.parquet")
    capture = create_capture(tmp_path, datetime(2026, 8, 20))
    input_root = tmp_path / "rgbd"
    color = input_root / "color/frame007.png"
    depth = input_root / "depth/frame007.png"
    color.parent.mkdir(parents=True)
    depth.parent.mkdir(parents=True)
    color.write_bytes(b"rgb")
    depth.write_bytes(b"depth")

    record_source(
        capture,
        kind="aligned_rgbd",
        source=input_root,
        config={"fps": 30, "depth_scale": 0.001},
    )
    index_path = archive_aligned_rgbd(
        capture,
        [(color, depth)],
        fps=30,
        depth_scale=0.001,
        camera="test_camera",
    )
    write_ego_frame_mapping(capture, [0])

    table = pq.read_table(index_path).to_pylist()
    assert table == [{
        "episode_index": 0,
        "frame_index": 0,
        "source_frame_index": 0,
        "ego_frame_index": 0,
        "rgb_source_name": "frame007.png",
        "depth_source_name": "frame007.png",
        "rgb_path": "rgb_original/episode_000000/frame_000000.png",
        "depth_aligned_path": "depth/aligned_to_rgb/episode_000000/frame_000000.png",
        "rgb_timestamp_hw_us": None,
        "depth_timestamp_hw_us": None,
        "timestamp_relative_ms": 0.0,
        "timestamp_source": "fps_derived",
        "sync_error_ms": None,
        "pairing_basis": "source_filename",
    }]
    assert (capture.source / table[0]["rgb_path"]).read_bytes() == b"rgb"
    assert (capture.source / table[0]["depth_aligned_path"]).read_bytes() == b"depth"
    acquisition = json.loads((capture.source / "acquisition.json").read_text(encoding="utf-8"))
    assert acquisition["timebase"]["hardware_timestamps_available"] is False
    depth_meta = json.loads(
        (capture.source / "depth/depth_streams.json").read_text(encoding="utf-8")
    )
    assert depth_meta["raw_depth"]["available"] is False
    assert depth_meta["aligned_to_rgb"]["frame_count"] == 1


def test_multisensor_source_index_preserves_native_stream_timelines(tmp_path: Path) -> None:
    pq = pytest.importorskip("pyarrow.parquet")
    capture = create_capture(tmp_path, datetime(2026, 8, 20))
    legacy_index = capture.source / "stream_index.parquet"
    legacy_index.write_bytes(b"compatibility-view")

    streams_path, samples_path, synchronization_path = write_multisensor_source_index(
        capture,
        streams=[
            {
                "stream_id": "glasses_rgb",
                "sensor_id": "glasses_cam0",
                "modality": "rgb",
                "nominal_rate_hz": 30,
                "calibration_id": "rig_v001",
                "clock_id": "glasses_clock",
                "source_path": "recordings/glasses/session.vrs",
            },
            {
                "stream_id": "wrist_imu_right",
                "sensor_id": "wrist_right",
                "modality": "imu",
                "nominal_rate_hz": 200,
                "calibration_id": "wrist_v001",
                "clock_id": "wrist_clock",
                "source_path": "recordings/wrist_right/session.bin",
            },
        ],
        samples=[
            {
                "episode_index": 0,
                "stream_id": "glasses_rgb",
                "sample_index": 0,
                "device_timestamp_us": 1_000_000,
                "master_timestamp_us": 2_000_000,
                "timestamp_uncertainty_us": 100,
                "path": None,
                "valid": True,
            },
            {
                "episode_index": 0,
                "stream_id": "wrist_imu_right",
                "sample_index": 0,
                "device_timestamp_us": 500_000,
                "master_timestamp_us": 2_000_100,
                "timestamp_uncertainty_us": 250,
                "path": None,
                "valid": True,
            },
            {
                "episode_index": 0,
                "stream_id": "wrist_imu_right",
                "sample_index": 1,
                "device_timestamp_us": 505_000,
                "master_timestamp_us": 2_005_100,
                "timestamp_uncertainty_us": 250,
                "path": None,
                "valid": False,
            },
        ],
        master_clock="host_monotonic",
        clock_models={
            "host_monotonic": {
                "scale": 1.0,
                "offset_us": 0,
                "uncertainty_us": 0,
                "method": "identity",
            },
            "glasses_clock": {
                "scale": 1.0,
                "offset_us": 1_000_000,
                "uncertainty_us": 100,
                "method": "hardware_sync",
            },
            "wrist_clock": {
                "scale": 1.0,
                "offset_us": 1_500_100,
                "uncertainty_us": 250,
                "method": "event_fit",
            },
        },
    )

    assert legacy_index.read_bytes() == b"compatibility-view"
    assert [row["stream_id"] for row in pq.read_table(streams_path).to_pylist()] == [
        "glasses_rgb",
        "wrist_imu_right",
    ]
    samples = pq.read_table(samples_path).to_pylist()
    assert [row["stream_id"] for row in samples] == [
        "glasses_rgb",
        "wrist_imu_right",
        "wrist_imu_right",
    ]
    assert samples[-1]["valid"] is False
    sync = json.loads(synchronization_path.read_text(encoding="utf-8"))
    assert sync["master_clock"] == "host_monotonic"
    assert sync["clocks"]["wrist_clock"]["method"] == "event_fit"


@pytest.mark.parametrize(
    ("streams", "samples", "error"),
    [
        (
            [
                {
                    "stream_id": "rgb",
                    "sensor_id": "cam",
                    "modality": "rgb",
                    "nominal_rate_hz": 30,
                    "calibration_id": None,
                    "clock_id": "missing_clock",
                    "source_path": None,
                }
            ],
            [
                {
                    "episode_index": 0,
                    "stream_id": "rgb",
                    "sample_index": 0,
                    "device_timestamp_us": 1,
                    "master_timestamp_us": 1,
                    "timestamp_uncertainty_us": 0,
                    "path": None,
                    "valid": True,
                }
            ],
            "unknown clock",
        ),
        (
            [
                {
                    "stream_id": "rgb",
                    "sensor_id": "cam",
                    "modality": "rgb",
                    "nominal_rate_hz": 30,
                    "calibration_id": None,
                    "clock_id": "host",
                    "source_path": None,
                }
            ],
            [
                {
                    "episode_index": 0,
                    "stream_id": "rgb",
                    "sample_index": 0,
                    "device_timestamp_us": 10,
                    "master_timestamp_us": 10,
                    "timestamp_uncertainty_us": 0,
                    "path": "../outside.bin",
                    "valid": True,
                }
            ],
            "unsafe sample path",
        ),
        (
            [
                {
                    "stream_id": "rgb",
                    "sensor_id": "cam",
                    "modality": "rgb",
                    "nominal_rate_hz": 30,
                    "calibration_id": None,
                    "clock_id": "host",
                    "source_path": None,
                }
            ],
            [
                {
                    "episode_index": 0,
                    "stream_id": "rgb",
                    "sample_index": 0,
                    "device_timestamp_us": 10,
                    "master_timestamp_us": 10,
                    "timestamp_uncertainty_us": 0,
                    "path": None,
                    "valid": True,
                },
                {
                    "episode_index": 0,
                    "stream_id": "rgb",
                    "sample_index": 1,
                    "device_timestamp_us": None,
                    "master_timestamp_us": None,
                    "timestamp_uncertainty_us": None,
                    "path": None,
                    "valid": False,
                },
                {
                    "episode_index": 0,
                    "stream_id": "rgb",
                    "sample_index": 2,
                    "device_timestamp_us": 9,
                    "master_timestamp_us": 11,
                    "timestamp_uncertainty_us": 0,
                    "path": None,
                    "valid": True,
                },
            ],
            "device timestamps must increase",
        ),
    ],
)
def test_multisensor_source_index_rejects_invalid_contract(
    tmp_path: Path,
    streams: list[dict],
    samples: list[dict],
    error: str,
) -> None:
    pytest.importorskip("pyarrow")
    capture = create_capture(tmp_path, datetime(2026, 8, 20))
    with pytest.raises(ValueError, match=error):
        write_multisensor_source_index(
            capture,
            streams=streams,
            samples=samples,
            master_clock="host",
            clock_models={
                "host": {
                    "scale": 1.0,
                    "offset_us": 0,
                    "uncertainty_us": 0,
                    "method": "identity",
                }
            },
        )
