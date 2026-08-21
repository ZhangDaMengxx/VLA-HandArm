"""Load and inspect an Ego or Robot LeRobotDataset without changing it."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")

from capture_bundle import (
    CAPTURE_READY,
    CapturePaths,
    read_ego_coordinate_system,
    legacy_data_paths,
    resolve_capture,
    resolve_data_paths,
)
from lerobot.datasets.lerobot_dataset import LeRobotDataset


class StrictV3Error(ValueError):
    pass


def _json_object(path: Path) -> dict:
    if not path.is_file():
        raise StrictV3Error(f"missing required file: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise StrictV3Error(f"invalid JSON: {path}") from error
    if not isinstance(value, dict):
        raise StrictV3Error(f"JSON root must be an object: {path}")
    return value


def _parquet_files(root: Path, label: str) -> list[Path]:
    files = sorted(root.glob("chunk-*/file-*.parquet"))
    if not files:
        raise StrictV3Error(f"missing {label} parquet under {root}")
    return files


def validate_strict_lerobot_v3(root: Path) -> dict:
    """Validate the local LeRobot v3 storage contract without downloading anything."""
    import pyarrow.parquet as pq

    root = Path(root).expanduser().resolve()
    info = _json_object(root / "meta/info.json")
    stats = _json_object(root / "meta/stats.json")
    if info.get("codebase_version") != "v3.0":
        raise StrictV3Error(
            f"meta/info.json codebase_version must be 'v3.0', got {info.get('codebase_version')!r}"
        )
    required_info = {
        "total_episodes", "total_frames", "total_tasks", "fps", "features",
        "data_path", "video_path",
    }
    missing_info = sorted(required_info - info.keys())
    if missing_info:
        raise StrictV3Error(f"meta/info.json missing keys: {', '.join(missing_info)}")
    features = info["features"]
    if not isinstance(features, dict) or not features:
        raise StrictV3Error("meta/info.json features must be a non-empty object")
    for key in ("timestamp", "frame_index", "episode_index", "index", "task_index"):
        if key not in features:
            raise StrictV3Error(f"meta/info.json features missing LeRobot index field: {key}")

    data_files = _parquet_files(root / "data", "data")
    episode_files = _parquet_files(root / "meta/episodes", "episode metadata")
    tasks_path = root / "meta/tasks.parquet"
    if not tasks_path.is_file():
        raise StrictV3Error(f"missing required file: {tasks_path}")

    data_rows = sum(pq.ParquetFile(path).metadata.num_rows for path in data_files)
    episode_rows = sum(pq.ParquetFile(path).metadata.num_rows for path in episode_files)
    task_rows = pq.ParquetFile(tasks_path).metadata.num_rows
    expected_counts = {
        "total_frames": data_rows,
        "total_episodes": episode_rows,
        "total_tasks": task_rows,
    }
    for key, actual in expected_counts.items():
        declared = info.get(key)
        if isinstance(declared, bool) or not isinstance(declared, int) or declared != actual:
            raise StrictV3Error(f"{key}={declared!r} does not match parquet rows {actual}")

    data_columns: set[str] = set()
    for path in data_files:
        data_columns.update(pq.read_schema(path).names)
    expected_data_columns = {
        key for key, spec in features.items()
        if isinstance(spec, dict) and spec.get("dtype") != "video"
    }
    missing_columns = sorted(expected_data_columns - data_columns)
    if missing_columns:
        raise StrictV3Error(f"data parquet missing declared features: {', '.join(missing_columns)}")

    task_columns = set(pq.read_schema(tasks_path).names)
    if not {"task_index", "task"}.issubset(task_columns):
        raise StrictV3Error("meta/tasks.parquet must contain task_index and task")
    episode_columns: set[str] = set()
    episode_lengths = 0
    for path in episode_files:
        schema = pq.read_schema(path)
        episode_columns.update(schema.names)
        table = pq.read_table(path, columns=["length"])
        episode_lengths += sum(int(value.as_py()) for value in table["length"])
    if not {"episode_index", "tasks", "length"}.issubset(episode_columns):
        raise StrictV3Error(
            "meta/episodes parquet must contain episode_index, tasks, and length"
        )
    if episode_lengths != data_rows:
        raise StrictV3Error(
            f"episode length sum {episode_lengths} does not match total frames {data_rows}"
        )

    missing_stats = sorted(set(features) - set(stats))
    if missing_stats:
        raise StrictV3Error(f"meta/stats.json missing feature stats: {', '.join(missing_stats)}")
    video_keys = [
        key for key, spec in features.items()
        if isinstance(spec, dict) and spec.get("dtype") == "video"
    ]
    for key in video_keys:
        video_files = sorted((root / "videos" / key).glob("chunk-*/file-*.mp4"))
        if not video_files:
            raise StrictV3Error(f"video feature {key!r} has no chunked MP4")

    return {
        "codebase_version": "v3.0",
        "frames": data_rows,
        "episodes": episode_rows,
        "tasks": task_rows,
        "features": len(features),
        "video_features": video_keys,
    }


def _verify_checksums(root: Path, manifest_path: Path) -> int:
    manifest = _json_object(manifest_path)
    if manifest.get("algorithm") != "sha256" or not isinstance(manifest.get("files"), list):
        raise StrictV3Error(f"invalid checksum manifest: {manifest_path}")
    seen = set()
    for item in manifest["files"]:
        if not isinstance(item, dict) or not isinstance(item.get("path"), str):
            raise StrictV3Error(f"invalid checksum row in {manifest_path}")
        relative = Path(item["path"])
        if relative.is_absolute() or ".." in relative.parts or relative.as_posix() in seen:
            raise StrictV3Error(f"unsafe or duplicate checksum path: {item.get('path')!r}")
        seen.add(relative.as_posix())
        path = (root / relative).resolve()
        if not path.is_relative_to(root.resolve()) or not path.is_file():
            raise StrictV3Error(f"checksummed file is missing: {relative.as_posix()}")
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        if digest.hexdigest() != item.get("sha256") or path.stat().st_size != item.get("size"):
            raise StrictV3Error(f"checksum mismatch: {path}")
    actual = {
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file() and path.resolve() != manifest_path.resolve()
    }
    missing = sorted(actual - seen)
    if missing:
        raise StrictV3Error(f"checksum manifest omits files: {', '.join(missing)}")
    return len(seen)


def _episode_indices(root: Path) -> list[int]:
    import pyarrow.parquet as pq

    values = set()
    for path in _parquet_files(root / "meta/episodes", "episode metadata"):
        schema = set(pq.read_schema(path).names)
        if "episode_index" not in schema:
            raise StrictV3Error(f"episode metadata missing episode_index: {path}")
        values.update(int(value.as_py()) for value in pq.read_table(path, columns=["episode_index"])[0])
    return sorted(values)


def _validate_episode_sidecars(root: Path, *, robot: bool) -> int:
    episodes = _episode_indices(root)
    for episode_index in episodes:
        annotation = _json_object(root / "annotations" / f"episode_{episode_index:06d}.json")
        if annotation.get("episode_index") != episode_index:
            raise StrictV3Error(f"annotation episode index mismatch: {episode_index}")
        if annotation.get("review_status") not in {"unreviewed", "reviewed", "rejected"}:
            raise StrictV3Error(f"invalid annotation review_status: {annotation}")
        if robot:
            qa = _json_object(root / "qa" / f"episode_{episode_index:06d}.json")
            if qa.get("episode_index") != episode_index:
                raise StrictV3Error(f"RobotDataset QA episode index mismatch: {episode_index}")
            if qa.get("automated_status") not in {"passed", "failed"}:
                raise StrictV3Error(f"invalid RobotDataset automated_status: {qa}")
            physical = qa.get("physical_checks")
            if not isinstance(physical, dict):
                raise StrictV3Error(f"RobotDataset QA physical_checks missing: {episode_index}")
    return len(episodes)


def validate_capture_bundle(root: Path) -> dict:
    """Validate one complete Capture, including every declared dataset and sidecar."""
    root = Path(root).expanduser().resolve()
    paths = CapturePaths(root)
    manifest = _json_object(paths.bundle_manifest)
    if manifest.get("capture_id") != root.name:
        raise StrictV3Error("bundle.json capture_id does not match its directory")
    if manifest.get("status") != CAPTURE_READY:
        raise StrictV3Error(f"Capture status must be ready, got {manifest.get('status')!r}")
    expected_layout = {
        "environment": "environment", "source": "source", "ego_dataset": "ego",
        "robot_datasets": "robot_datasets", "lineage": "lineage", "reports": "reports",
    }
    if manifest.get("layout") != expected_layout:
        raise StrictV3Error("bundle.json layout does not match the Capture contract")
    for relative in (
        "environment/runtime.json", "environment/requirements.txt", "environment/environment.lock",
        "source/acquisition.json", "source/quality_profile.json", "source/stream_index.parquet",
        "source/retention.json", "source/checksums_original.json",
        "lineage/dataset_relations.json", "lineage/processing_runs.jsonl",
    ):
        if not (root / relative).is_file():
            raise StrictV3Error(f"Capture required file missing: {relative}")

    ego_result = validate_strict_lerobot_v3(paths.ego)
    ego_episodes = _validate_episode_sidecars(paths.ego, robot=False)
    ego_checksums = _verify_checksums(paths.ego, paths.ego / "checksums.json")
    source_checksums = _verify_checksums(paths.source, paths.source / "checksums_original.json")

    datasets = manifest.get("datasets")
    robots = datasets.get("robots") if isinstance(datasets, dict) else None
    if not isinstance(robots, dict):
        raise StrictV3Error("bundle.json datasets.robots must be an object")
    relations = _json_object(paths.lineage / "dataset_relations.json")
    if relations.get("capture_id") != paths.capture_id or relations.get("ego") != "ego":
        raise StrictV3Error("dataset_relations.json does not reference this Capture/Ego")
    if relations.get("robots") != robots:
        raise StrictV3Error("bundle.json and dataset_relations.json robot paths differ")

    robot_results = {}
    for target_id, relative_text in sorted(robots.items()):
        if not isinstance(relative_text, str):
            raise StrictV3Error(f"invalid RobotDataset path for {target_id!r}")
        relative = Path(relative_text)
        dataset_root = (root / relative).resolve()
        if relative.is_absolute() or ".." in relative.parts or not dataset_root.is_relative_to(paths.robot_datasets):
            raise StrictV3Error(f"unsafe RobotDataset path for {target_id!r}: {relative_text}")
        strict = validate_strict_lerobot_v3(dataset_root)
        episode_count = _validate_episode_sidecars(dataset_root, robot=True)
        checksum_count = _verify_checksums(dataset_root, dataset_root / "checksums.json")
        source_ego = _json_object(dataset_root / "meta/source_ego.json")
        if source_ego.get("capture_id") != paths.capture_id:
            raise StrictV3Error(f"RobotDataset {target_id!r} points to another Capture")
        robot_results[target_id] = {
            **strict, "episode_sidecars": episode_count, "checksummed_files": checksum_count,
        }

    return {
        "capture_id": paths.capture_id,
        "status": "passed",
        "ego": {**ego_result, "episode_sidecars": ego_episodes, "checksummed_files": ego_checksums},
        "source_checksummed_files": source_checksums,
        "robots": robot_results,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--canonical", action="store_true", help="验证 Ego canonical dataset")
    ap.add_argument("--robot", default="nero_inspire", help="RobotSpec 名")
    ap.add_argument("--capture-root", default=None,
                    help="Capture Bundle;不传则读取 datasets/captures/ 中最新一次")
    ap.add_argument("--root", default=None, help="显式待验证 LeRobotDataset 根目录")
    ap.add_argument("--legacy-out", action="store_true", help="显式读取旧 src/out")
    ap.add_argument("--strict-v3", action="store_true",
                    help="在官方加载前严格检查 LeRobot v3.0 文件、列和计数契约")
    ap.add_argument("--capture-bundle", action="store_true",
                    help="校验整个 Capture 的 Source/环境/Ego/Robot/血缘/annotations/QA/checksum")
    ap.add_argument("--json", default=None, help="将 Capture 校验结果写入 JSON")
    args = ap.parse_args()

    if args.capture_bundle:
        if args.root or args.legacy_out or args.canonical:
            raise SystemExit("--capture-bundle uses --capture-root and cannot combine with dataset flags")
        try:
            capture = resolve_capture(args.capture_root)
            result = validate_capture_bundle(capture.root)
        except (ValueError, FileNotFoundError, StrictV3Error) as error:
            raise SystemExit(f"Capture validation failed: {error}") from error
        print(
            f"capture-bundle: OK id={result['capture_id']} "
            f"ego_frames={result['ego']['frames']} robots={len(result['robots'])}"
        )
        if args.json:
            destination = Path(args.json).expanduser().resolve()
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(
                json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
            print("report:", destination)
        return

    # Keep RobotSpec imports out of standalone Capture/strict-v3 validators.
    # Some skill tests intentionally expose their own top-level `schema` module.
    from robot_specs import get_spec

    if args.root and (args.capture_root or args.legacy_out):
        raise SystemExit("--root cannot be combined with --capture-root or --legacy-out")

    spec = get_spec(args.robot)
    if args.root:
        root = Path(args.root).expanduser().resolve()
    elif args.legacy_out:
        legacy = legacy_data_paths(spec.name)
        root = legacy.canonical_root if args.canonical else legacy.dataset_root
    else:
        try:
            paths = resolve_data_paths(spec.name, capture_root=args.capture_root)
        except (ValueError, FileNotFoundError) as error:
            raise SystemExit(str(error)) from error
        root = paths.canonical_root if args.canonical else paths.dataset_root

    repo_id = "local/handdemo_canonical" if args.canonical else spec.repo_id
    print("dataset:", root)
    if args.canonical:
        try:
            coordinates = read_ego_coordinate_system(
                root,
                required=False,
                allow_legacy_schema=True,
            )
        except (ValueError, FileNotFoundError) as error:
            raise SystemExit(str(error)) from error
        if coordinates is None:
            print("coordinates: legacy/undeclared (no frame inferred)")
        else:
            wrist = coordinates["features"]["observation.wrist_pose"]
            hand = coordinates["features"]["observation.hand_keypoints"]
            print(
                f"coordinates: {coordinates['contract']} "
                f"wrist_pose={wrist['frame']} hand_keypoints={hand['frame']}"
            )
    if args.strict_v3:
        try:
            strict = validate_strict_lerobot_v3(root)
        except StrictV3Error as error:
            raise SystemExit(f"strict LeRobot v3 validation failed: {error}") from error
        print(
            "strict-v3: OK "
            f"frames={strict['frames']} episodes={strict['episodes']} "
            f"tasks={strict['tasks']} features={strict['features']}"
        )
    ds = LeRobotDataset(repo_id, root=str(root))
    print("len(ds):", len(ds))
    for attribute in ["num_frames", "num_episodes", "total_frames", "total_episodes"]:
        print(f"  {attribute}:", getattr(ds, attribute, "N/A"))
    print("features:", list(ds.features.keys()))
    sample = ds[0]
    print("sample keys:", list(sample.keys()))
    for key in [
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
        if key in sample:
            value = sample[key]
            print(f"  {key}: {type(value).__name__} shape={getattr(value, 'shape', None)}")


if __name__ == "__main__":
    main()
