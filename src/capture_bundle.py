"""Capture Bundle paths and small provenance manifests.

This module owns storage layout only. It deliberately does not transform hand
landmarks, poses, quaternions, timestamps, robot states, or actions.
"""
from __future__ import annotations

import atexit
import fcntl
import hashlib
import importlib.metadata
import json
import math
import os
import platform
import re
import shutil
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from quality_profiles import validate_quality_profile, write_quality_profile_snapshot


REPO = Path(__file__).resolve().parents[1]
CAPTURES_ROOT = REPO / "datasets/captures"
LEGACY_OUT_ROOT = REPO / "src/out"
CAPTURE_NAME_RE = re.compile(
    r"^capture_(?P<date>\d{8})_(?P<sequence>\d{6})_(?P<uuid>[0-9a-f-]{32,36})$"
)
TARGET_REVISION = "target_revision_v001"
RETARGET_REVISION = "retarget_v001"
CAPTURE_BUILDING = "building"
CAPTURE_READY = "ready"
CAPTURE_FAILED = "failed"
COORDINATE_SCHEMA_VERSION = "2.0"
WRIST_FRAME_EPISODE0_CAMERA = "episode0_camera"
WRIST_FRAME_SCENE_WORLD = "scene_world"
WRIST_POSE_FRAMES = frozenset({WRIST_FRAME_EPISODE0_CAMERA, WRIST_FRAME_SCENE_WORLD})
HAND_KEYPOINT_FRAMES = frozenset({"wrist_local_mano", *WRIST_POSE_FRAMES})
_REGISTERED_EGO_BUILDS: set[Path] = set()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(path)


def _write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_text(value, encoding="utf-8")
    tmp.replace(path)


def _append_jsonl(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(value, ensure_ascii=False) + "\n")


def _write_checksums(root: Path, destination: Path) -> None:
    rows = []
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        if path.resolve() == destination.resolve():
            continue
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        rows.append({
            "path": path.relative_to(root).as_posix(),
            "size": path.stat().st_size,
            "sha256": digest.hexdigest(),
        })
    _write_json(destination, {"algorithm": "sha256", "files": rows})


def _link_or_copy(source: Path, destination: Path) -> Path:
    source = source.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if source == destination.resolve():
        return destination
    destination.unlink(missing_ok=True)
    try:
        os.link(source, destination)
    except OSError:
        shutil.copy2(source, destination)
    return destination


def _timestamp_values(values: Any, count: int, label: str) -> list[int | None]:
    if values is None:
        return [None] * count
    result = [int(value) for value in values]
    if len(result) != count:
        raise ValueError(f"{label} count {len(result)} does not match frame count {count}")
    if any(current <= previous for previous, current in zip(result, result[1:])):
        raise ValueError(f"{label} must be strictly increasing")
    return result


def _write_stream_index(paths: "CapturePaths", rows: list[dict[str, Any]]) -> Path:
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as error:
        raise RuntimeError(
            "Source stream_index.parquet requires pyarrow; use the documented lerobot environment"
        ) from error

    schema = pa.schema([
        ("episode_index", pa.int32()),
        ("frame_index", pa.int64()),
        ("source_frame_index", pa.int64()),
        ("ego_frame_index", pa.int64()),
        ("rgb_source_name", pa.string()),
        ("depth_source_name", pa.string()),
        ("rgb_path", pa.string()),
        ("depth_aligned_path", pa.string()),
        ("rgb_timestamp_hw_us", pa.int64()),
        ("depth_timestamp_hw_us", pa.int64()),
        ("timestamp_relative_ms", pa.float64()),
        ("timestamp_source", pa.string()),
        ("sync_error_ms", pa.float64()),
        ("pairing_basis", pa.string()),
    ])
    destination = paths.source / "stream_index.parquet"
    destination.parent.mkdir(parents=True, exist_ok=True)
    tmp = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
    pq.write_table(pa.Table.from_pylist(rows, schema=schema), tmp)
    tmp.replace(destination)
    return destination


def _safe_component(value: str, label: str) -> str:
    if not value or value in {".", ".."} or not re.fullmatch(r"[A-Za-z0-9_.-]+", value):
        raise ValueError(f"invalid {label}: {value!r}")
    return value


def _fail_unfinished_ego_build(root: Path) -> None:
    """Convert ordinary process exits before finalization into an inspectable failure."""
    try:
        paths = CapturePaths(root)
        if paths.root.is_dir() and paths.status == CAPTURE_BUILDING:
            paths.mark_ego_failed("process exited before Ego metadata finalization")
    except (OSError, ValueError, json.JSONDecodeError):
        # The directory may be a short-lived test fixture or externally removed.
        return


@dataclass(frozen=True)
class RobotDataPaths:
    dataset_root: Path
    trajectory_pkl: Path
    trajectory_npz: Path
    quality_report: Path


@dataclass(frozen=True)
class CapturePaths:
    root: Path

    @property
    def capture_id(self) -> str:
        return self.root.name

    @property
    def bundle_manifest(self) -> Path:
        return self.root / "bundle.json"

    @property
    def source(self) -> Path:
        return self.root / "source"

    @property
    def environment(self) -> Path:
        return self.root / "environment"

    @property
    def ego(self) -> Path:
        return self.root / "ego"

    @property
    def robot_datasets(self) -> Path:
        return self.root / "robot_datasets"

    @property
    def lineage(self) -> Path:
        return self.root / "lineage"

    @property
    def reports(self) -> Path:
        return self.root / "reports"

    @property
    def original_video(self) -> Path:
        return self.source / "rgb_original/recording_000000.mp4"

    @property
    def replay_rrd(self) -> Path:
        return self.reports / "replay.rrd"

    def robot(
        self,
        target_id: str,
        target_revision: str = TARGET_REVISION,
        retarget_revision: str = RETARGET_REVISION,
    ) -> RobotDataPaths:
        target_id = _safe_component(target_id, "target_id")
        target_revision = _safe_component(target_revision, "target_revision")
        retarget_revision = _safe_component(retarget_revision, "retarget_revision")
        dataset = self.robot_datasets / target_id / target_revision / retarget_revision
        report = self.reports / "retargeting" / target_id / target_revision
        return RobotDataPaths(
            dataset_root=dataset,
            trajectory_pkl=dataset / "exports/workbench/robot_traj.pkl",
            trajectory_npz=dataset / "exports/workbench/robot_traj.npz",
            quality_report=report / f"{retarget_revision}_summary.json",
        )

    def ensure_layout(self) -> None:
        for path in (
            self.environment, self.source, self.robot_datasets, self.lineage, self.reports,
        ):
            path.mkdir(parents=True, exist_ok=True)
        if not self.bundle_manifest.exists():
            _write_json(self.bundle_manifest, {
                "bundle_schema_version": "1.0",
                "capture_id": self.capture_id,
                "created_at": _utc_now(),
                "status": CAPTURE_BUILDING,
                "stages": {
                    "ego": {"status": CAPTURE_BUILDING, "started_at": _utc_now()},
                    "robots": {},
                },
                "layout": {
                    "environment": "environment",
                    "source": "source",
                    "ego_dataset": "ego",
                    "robot_datasets": "robot_datasets",
                    "lineage": "lineage",
                    "reports": "reports",
                },
                "data_conventions": {
                    "length_unit": "m",
                    "angle_unit": "rad",
                    "canonical_wrist_pose": "tx,ty,tz,qx,qy,qz,qw",
                    "quaternion_order": "xyzw",
                    "note": "Path migration preserves the existing numerical contract.",
                },
                "datasets": {"ego": "ego", "robots": {}},
            })
        relations = self.lineage / "dataset_relations.json"
        if not relations.exists():
            _write_json(relations, {
                "capture_id": self.capture_id,
                "source": "source",
                "ego": "ego",
                "robots": {},
            })
        retention = self.source / "retention.json"
        if not retention.exists():
            _write_json(retention, {"status": "hot", "updated_at": _utc_now()})

    def update_manifest(self, updater) -> None:
        self.ensure_layout()
        data = json.loads(self.bundle_manifest.read_text(encoding="utf-8"))
        updater(data)
        data["updated_at"] = _utc_now()
        _write_json(self.bundle_manifest, data)

    @property
    def status(self) -> str:
        """Return lifecycle state, including compatibility for early manifests."""
        if not self.bundle_manifest.is_file():
            return CAPTURE_FAILED
        try:
            data = json.loads(self.bundle_manifest.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return CAPTURE_FAILED
        status = data.get("status")
        if status in {CAPTURE_BUILDING, CAPTURE_READY, CAPTURE_FAILED}:
            return status
        # Capture manifests created before lifecycle tracking had no status.
        return CAPTURE_READY if (self.ego / "meta/info.json").is_file() else CAPTURE_BUILDING

    def begin_ego_build(self) -> None:
        """Mark Ego output incomplete until metadata finalization succeeds."""
        started_at = _utc_now()

        def update(data: dict[str, Any]) -> None:
            data["status"] = CAPTURE_BUILDING
            data.pop("failure", None)
            stages = data.setdefault("stages", {})
            stages["ego"] = {"status": CAPTURE_BUILDING, "started_at": started_at}
            stages.setdefault("robots", {})

        self.update_manifest(update)
        resolved = self.root.resolve()
        if resolved not in _REGISTERED_EGO_BUILDS:
            _REGISTERED_EGO_BUILDS.add(resolved)
            atexit.register(_fail_unfinished_ego_build, resolved)

    def mark_ego_ready(self) -> None:
        completed_at = _utc_now()

        def update(data: dict[str, Any]) -> None:
            data["status"] = CAPTURE_READY
            data.pop("failure", None)
            stages = data.setdefault("stages", {})
            ego = stages.setdefault("ego", {})
            ego.update({"status": CAPTURE_READY, "completed_at": completed_at})
            stages.setdefault("robots", {})

        self.update_manifest(update)

    def mark_ego_failed(self, reason: str) -> None:
        failed_at = _utc_now()

        def update(data: dict[str, Any]) -> None:
            data["status"] = CAPTURE_FAILED
            data["failure"] = {"stage": "ego", "reason": reason, "failed_at": failed_at}
            stages = data.setdefault("stages", {})
            ego = stages.setdefault("ego", {})
            ego.update({"status": CAPTURE_FAILED, "failed_at": failed_at})
            stages.setdefault("robots", {})

        self.update_manifest(update)


@dataclass(frozen=True)
class LegacyDataPaths:
    canonical_root: Path
    dataset_root: Path
    trajectory_pkl: Path
    trajectory_npz: Path
    quality_report: Path
    original_video: Path
    replay_rrd: Path


@dataclass(frozen=True)
class DataProductPaths:
    capture: CapturePaths | None
    canonical_root: Path
    dataset_root: Path
    trajectory_pkl: Path
    trajectory_npz: Path
    quality_report: Path
    original_video: Path
    replay_rrd: Path


def capture_for_path(path: str | Path) -> CapturePaths | None:
    candidate = Path(path).expanduser().resolve()
    for parent in (candidate, *candidate.parents):
        if CAPTURE_NAME_RE.fullmatch(parent.name):
            paths = CapturePaths(parent)
            paths.ensure_layout()
            return paths
        if parent == REPO.parent:
            break
    return None


def resolve_ego_output(
    *,
    capture_root: str | Path | None = None,
    output_root: str | Path | None = None,
    legacy_out: bool = False,
    captures_root: str | Path | None = None,
) -> tuple[Path, CapturePaths | None]:
    selected = sum(bool(value) for value in (capture_root, output_root, legacy_out))
    if selected > 1:
        raise ValueError("use only one of --capture-root, --root, or --legacy-out")
    if legacy_out:
        return LEGACY_OUT_ROOT / "canonical_ds", None
    if output_root:
        root = Path(output_root).expanduser().resolve()
        capture = capture_for_path(root)
        if capture is not None and root != capture.ego.resolve():
            raise ValueError(f"Ego root inside a Capture must be {capture.ego}, got {root}")
        if capture is not None:
            capture.begin_ego_build()
        return root, capture
    paths = resolve_capture(
        capture_root,
        create=True,
        captures_root=Path(captures_root) if captures_root is not None else CAPTURES_ROOT,
    )
    paths.begin_ego_build()
    return paths.ego, paths


def resolve_ego_input(
    *,
    capture_root: str | Path | None = None,
    input_root: str | Path | None = None,
    legacy_out: bool = False,
    captures_root: str | Path | None = None,
) -> tuple[Path, CapturePaths | None]:
    """Resolve an existing Ego dataset without ever creating a Capture."""
    selected = sum(bool(value) for value in (capture_root, input_root, legacy_out))
    if selected > 1:
        raise ValueError("use only one of --capture-root, --canonical, or --legacy-out")
    if legacy_out:
        return LEGACY_OUT_ROOT / "canonical_ds", None
    if input_root:
        root = Path(input_root).expanduser().resolve()
        capture = capture_for_path(root)
        if capture is not None and root != capture.ego.resolve():
            raise ValueError(f"Ego root inside a Capture must be {capture.ego}, got {root}")
        return root, capture
    paths = resolve_capture(
        capture_root,
        captures_root=Path(captures_root) if captures_root is not None else CAPTURES_ROOT,
    )
    return paths.ego, paths


def legacy_data_paths(target_id: str) -> LegacyDataPaths:
    target_id = _safe_component(target_id, "target_id")
    traj = LEGACY_OUT_ROOT / f"robot_traj_{target_id}.pkl"
    return LegacyDataPaths(
        canonical_root=LEGACY_OUT_ROOT / "canonical_ds",
        dataset_root=LEGACY_OUT_ROOT / f"lerobot_ds_{target_id}",
        trajectory_pkl=traj,
        trajectory_npz=traj.with_suffix(".npz"),
        quality_report=LEGACY_OUT_ROOT / f"metrics_{target_id}.json",
        original_video=LEGACY_OUT_ROOT / f"replay_video_original_{target_id}.mp4",
        replay_rrd=LEGACY_OUT_ROOT / "replay.rrd",
    )


def resolve_data_paths(
    target_id: str,
    *,
    capture_root: str | Path | None = None,
    canonical_root: str | Path | None = None,
    output_root: str | Path | None = None,
    legacy_out: bool = False,
    target_revision: str = TARGET_REVISION,
    retarget_revision: str = RETARGET_REVISION,
    captures_root: str | Path | None = None,
) -> DataProductPaths:
    if legacy_out and any((capture_root, canonical_root, output_root)):
        raise ValueError("--legacy-out cannot be combined with Capture or explicit roots")
    if capture_root and any((canonical_root, output_root)):
        raise ValueError("--capture-root cannot be combined with --canonical or --output-root")
    if legacy_out:
        legacy = legacy_data_paths(target_id)
        return DataProductPaths(None, **legacy.__dict__)

    lookup_root = Path(captures_root) if captures_root is not None else CAPTURES_ROOT
    capture = resolve_capture(capture_root, captures_root=lookup_root) if capture_root else None
    canonical = Path(canonical_root).expanduser().resolve() if canonical_root else None
    output = Path(output_root).expanduser().resolve() if output_root else None

    canonical_capture = capture_for_path(canonical) if canonical is not None else None
    output_capture = capture_for_path(output) if output is not None else None
    if canonical_capture is not None and output_capture is not None:
        if canonical_capture.root != output_capture.root:
            raise ValueError("canonical and output roots belong to different Capture Bundles")
    elif (canonical_capture is None) != (output_capture is None) and canonical is not None and output is not None:
        raise ValueError("canonical and output roots must both be external or belong to one Capture")
    if capture is None:
        capture = canonical_capture or output_capture
    if capture is None and canonical is None and output is None:
        capture = resolve_capture(None, captures_root=lookup_root)

    if capture is not None:
        robot = capture.robot(target_id, target_revision, retarget_revision)
        if canonical is not None and canonical != capture.ego.resolve():
            raise ValueError(f"canonical root inside a Capture must be {capture.ego}, got {canonical}")
        if output is not None and output != robot.dataset_root.resolve():
            raise ValueError(
                f"RobotDataset root for this Capture/revision must be {robot.dataset_root}, got {output}"
            )
        return DataProductPaths(
            capture=capture,
            canonical_root=canonical or capture.ego,
            dataset_root=output or robot.dataset_root,
            trajectory_pkl=robot.trajectory_pkl if output is None else output / "exports/workbench/robot_traj.pkl",
            trajectory_npz=robot.trajectory_npz if output is None else output / "exports/workbench/robot_traj.npz",
            quality_report=robot.quality_report if output is None else output / "qa/summary.json",
            original_video=capture.original_video,
            replay_rrd=capture.replay_rrd,
        )

    if canonical is None or output is None:
        raise ValueError(
            "external datasets require both --canonical and --output-root, or use a Capture Bundle"
        )
    return DataProductPaths(
        capture=None,
        canonical_root=canonical,
        dataset_root=output,
        trajectory_pkl=output / "exports/workbench/robot_traj.pkl",
        trajectory_npz=output / "exports/workbench/robot_traj.npz",
        quality_report=output / "qa/summary.json",
        original_video=canonical / "source_video.mp4",
        replay_rrd=output / "qa/replay.rrd",
    )


def create_capture(captures_root: Path = CAPTURES_ROOT, now: datetime | None = None) -> CapturePaths:
    """Atomically allocate the next sequence for the current local date."""
    captures_root = Path(captures_root).resolve()
    captures_root.mkdir(parents=True, exist_ok=True)
    local_now = now or datetime.now().astimezone()
    date_text = local_now.strftime("%Y%m%d")
    lock_path = captures_root / ".capture-sequence.lock"
    with lock_path.open("a+", encoding="utf-8") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        used = []
        for candidate in captures_root.glob(f"capture_{date_text}_*"):
            match = CAPTURE_NAME_RE.fullmatch(candidate.name)
            if match:
                used.append(int(match.group("sequence")))
        sequence = max(used, default=-1) + 1
        while True:
            name = f"capture_{date_text}_{sequence:06d}_{uuid.uuid4()}"
            root = captures_root / name
            try:
                root.mkdir()
            except FileExistsError:
                sequence += 1
                continue
            paths = CapturePaths(root)
            paths.ensure_layout()
            return paths


def open_capture(value: str | Path, captures_root: Path = CAPTURES_ROOT) -> CapturePaths:
    raw = Path(value).expanduser()
    root = raw if raw.is_absolute() else Path(captures_root) / raw
    root = root.resolve()
    if not CAPTURE_NAME_RE.fullmatch(root.name):
        raise ValueError(f"not a capture_<date>_<sequence>_<uuid> directory: {root}")
    if not root.is_dir():
        raise FileNotFoundError(f"capture does not exist: {root}")
    paths = CapturePaths(root)
    paths.ensure_layout()
    return paths


def latest_capture(captures_root: Path = CAPTURES_ROOT) -> CapturePaths | None:
    """Return the newest complete Ego Capture; ignore building/failed batches."""
    root = Path(captures_root)
    if not root.is_dir():
        return None
    candidates = sorted(
        (
            path
            for path in root.iterdir()
            if path.is_dir()
            and CAPTURE_NAME_RE.fullmatch(path.name)
            and CapturePaths(path.resolve()).status == CAPTURE_READY
        ),
        key=lambda path: path.name,
    )
    return CapturePaths(candidates[-1].resolve()) if candidates else None


def discover_trajectory_npz(
    *,
    capture_root: str | Path | None = None,
    legacy_out: bool = False,
    captures_root: str | Path | None = None,
) -> list[Path]:
    """Find exported robot trajectories in one Capture, or in explicit legacy mode."""
    if legacy_out and capture_root:
        raise ValueError("--legacy-out cannot be combined with --capture-root")
    if legacy_out:
        return sorted(LEGACY_OUT_ROOT.glob("robot_traj_*.npz"))
    paths = resolve_capture(
        capture_root,
        captures_root=Path(captures_root) if captures_root is not None else CAPTURES_ROOT,
    )
    return sorted(
        paths.robot_datasets.glob("*/*/*/exports/workbench/robot_traj.npz")
    )


def resolve_capture(
    value: str | Path | None,
    *,
    create: bool = False,
    captures_root: Path = CAPTURES_ROOT,
) -> CapturePaths:
    if value:
        raw = Path(value).expanduser()
        candidate = raw if raw.is_absolute() else Path(captures_root) / raw
        if candidate.exists():
            return open_capture(candidate, captures_root)
        if not create:
            raise FileNotFoundError(f"capture does not exist: {candidate}")
        if not CAPTURE_NAME_RE.fullmatch(candidate.name):
            raise ValueError("new explicit capture roots must use capture_<date>_<sequence>_<uuid>")
        candidate.mkdir(parents=True)
        paths = CapturePaths(candidate.resolve())
        paths.ensure_layout()
        return paths
    if create:
        return create_capture(Path(captures_root))
    latest = latest_capture(Path(captures_root))
    if latest is None:
        raise FileNotFoundError(
            f"no ready Capture Bundle under {Path(captures_root)}; build one, pass --capture-root "
            "to inspect an incomplete batch, or pass --legacy-out explicitly"
        )
    return latest


def archive_original_video(paths: CapturePaths, source: str | Path) -> Path:
    source = Path(source).expanduser().resolve()
    suffix = source.suffix.lower() or ".mp4"
    destination = paths.source / "rgb_original" / f"recording_000000{suffix}"
    return _link_or_copy(source, destination)


def archive_processed_input(paths: CapturePaths, source: str | Path) -> Path:
    source = Path(source).expanduser().resolve()
    suffix = source.suffix.lower() or ".bin"
    return _link_or_copy(
        source,
        paths.source / "recordings" / f"processed_input_000000{suffix}",
    )


def write_sample_stream_index(
    paths: CapturePaths,
    *,
    frame_count: int,
    fps: float,
    rgb_path: str | None,
    pairing_basis: str,
    timestamps_hw_us: Any = None,
    ego_frame_indices: Any = None,
) -> Path:
    """Index a single RGB/video or processed-sample stream without inventing hardware time."""
    if frame_count < 0 or fps <= 0:
        raise ValueError("frame_count must be non-negative and fps must be positive")
    timestamps = _timestamp_values(timestamps_hw_us, frame_count, "timestamps_hw_us")
    if ego_frame_indices is None:
        ego_indices = list(range(frame_count))
    else:
        ego_indices = [None if value is None else int(value) for value in ego_frame_indices]
        if len(ego_indices) != frame_count:
            raise ValueError(
                f"ego_frame_indices count {len(ego_indices)} does not match frame count {frame_count}"
            )
    t0 = next((value for value in timestamps if value is not None), None)
    rows = []
    for index, timestamp in enumerate(timestamps):
        relative_ms = (
            (timestamp - t0) / 1000.0
            if timestamp is not None and t0 is not None
            else index * 1000.0 / fps
        )
        rows.append({
            "episode_index": 0,
            "frame_index": index,
            "source_frame_index": index,
            "ego_frame_index": ego_indices[index],
            "rgb_source_name": None,
            "depth_source_name": None,
            "rgb_path": rgb_path,
            "depth_aligned_path": None,
            "rgb_timestamp_hw_us": timestamp,
            "depth_timestamp_hw_us": None,
            "timestamp_relative_ms": relative_ms,
            "timestamp_source": "hardware" if timestamp is not None else "fps_derived",
            "sync_error_ms": None,
            "pairing_basis": pairing_basis,
        })
    return _write_stream_index(paths, rows)


def archive_aligned_rgbd(
    paths: CapturePaths,
    pairs: list[tuple[Path, Path]],
    *,
    fps: float,
    depth_scale: float,
    camera: str,
    rgb_timestamps_hw_us: Any = None,
    depth_timestamps_hw_us: Any = None,
) -> Path:
    """Archive aligned RGB-D source frames and write their exact pairing index."""
    if fps <= 0 or depth_scale <= 0:
        raise ValueError("fps and depth_scale must be positive")
    rgb_timestamps = _timestamp_values(rgb_timestamps_hw_us, len(pairs), "rgb_timestamps_hw_us")
    depth_timestamps = _timestamp_values(
        depth_timestamps_hw_us, len(pairs), "depth_timestamps_hw_us"
    )
    hardware_t0 = next(
        (value for value in (*rgb_timestamps, *depth_timestamps) if value is not None),
        None,
    )
    rows = []
    for index, (rgb_source, depth_source) in enumerate(pairs):
        frame_name = f"frame_{index:06d}"
        rgb_destination = paths.source / "rgb_original/episode_000000" / (
            frame_name + (rgb_source.suffix.lower() or ".png")
        )
        depth_destination = paths.source / "depth/aligned_to_rgb/episode_000000" / (
            frame_name + (depth_source.suffix.lower() or ".png")
        )
        _link_or_copy(rgb_source, rgb_destination)
        _link_or_copy(depth_source, depth_destination)
        rgb_timestamp = rgb_timestamps[index]
        depth_timestamp = depth_timestamps[index]
        reference_timestamp = rgb_timestamp if rgb_timestamp is not None else depth_timestamp
        relative_ms = (
            (reference_timestamp - hardware_t0) / 1000.0
            if reference_timestamp is not None and hardware_t0 is not None
            else index * 1000.0 / fps
        )
        sync_error_ms = (
            abs(rgb_timestamp - depth_timestamp) / 1000.0
            if rgb_timestamp is not None and depth_timestamp is not None
            else None
        )
        rows.append({
            "episode_index": 0,
            "frame_index": index,
            "source_frame_index": index,
            "ego_frame_index": None,
            "rgb_source_name": rgb_source.name,
            "depth_source_name": depth_source.name,
            "rgb_path": rgb_destination.relative_to(paths.source).as_posix(),
            "depth_aligned_path": depth_destination.relative_to(paths.source).as_posix(),
            "rgb_timestamp_hw_us": rgb_timestamp,
            "depth_timestamp_hw_us": depth_timestamp,
            "timestamp_relative_ms": relative_ms,
            "timestamp_source": "hardware" if reference_timestamp is not None else "fps_derived",
            "sync_error_ms": sync_error_ms,
            "pairing_basis": "hardware_timestamp" if sync_error_ms is not None else "source_filename",
        })

    _write_json(paths.source / "depth/depth_streams.json", {
        "schema_version": "1.0",
        "camera": camera,
        "depth_scale_m_per_unit": depth_scale,
        "raw_depth": {
            "available": False,
            "reason": "input contains only depth already aligned/presented with the RGB frame set",
        },
        "aligned_to_rgb": {
            "available": True,
            "path": "aligned_to_rgb/episode_000000",
            "frame_count": len(pairs),
        },
        "hardware_timestamps_available": any(
            value is not None for value in (*rgb_timestamps, *depth_timestamps)
        ),
    })
    return _write_stream_index(paths, rows)


def write_ego_frame_mapping(paths: CapturePaths, ego_frame_indices: Any) -> Path:
    """Finalize Source-to-Ego frame lineage after detection has decided kept frames."""
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as error:
        raise RuntimeError(
            "Source stream_index.parquet requires pyarrow; use the documented lerobot environment"
        ) from error

    index_path = paths.source / "stream_index.parquet"
    table = pq.read_table(index_path)
    values = [None if value is None else int(value) for value in ego_frame_indices]
    if len(values) != table.num_rows:
        raise ValueError(
            f"ego frame mapping count {len(values)} does not match source rows {table.num_rows}"
        )
    column_index = table.schema.get_field_index("ego_frame_index")
    if column_index < 0:
        raise ValueError("stream index is missing ego_frame_index")
    table = table.set_column(column_index, "ego_frame_index", pa.array(values, type=pa.int64()))
    tmp = index_path.with_name(f".{index_path.name}.{os.getpid()}.tmp")
    pq.write_table(table, tmp)
    tmp.replace(index_path)
    return index_path


def record_source(
    paths: CapturePaths,
    *,
    kind: str,
    source: str | Path,
    config: dict[str, Any],
    hardware_timestamps_available: bool = False,
    quality_profile: dict[str, Any] | None = None,
) -> None:
    source_path = Path(source).expanduser().resolve()
    profile_ref = None
    if quality_profile is not None:
        quality_profile = validate_quality_profile(quality_profile)
        if kind not in quality_profile["source_kinds"]:
            raise ValueError(
                f"quality profile {quality_profile['profile_id']!r} does not apply to source kind {kind!r}"
            )
        write_quality_profile_snapshot(paths.source, quality_profile)
        profile_ref = {
            "path": "quality_profile.json",
            "profile_id": quality_profile["profile_id"],
            "revision": quality_profile["revision"],
        }
    _write_json(paths.source / "acquisition.json", {
        "schema_version": "1.1",
        "kind": kind,
        "source_path": str(source_path),
        "recorded_at": _utc_now(),
        "config": config,
        "quality_profile": profile_ref,
        "timebase": {
            "hardware_timestamp_field": "timestamp_hw_us",
            "hardware_timestamp_unit": "us",
            "hardware_timestamps_available": hardware_timestamps_available,
            "relative_timestamp_field": "timestamp_relative_ms",
            "relative_timestamp_unit": "ms",
            "fallback": "frame_index/fps" if not hardware_timestamps_available else None,
        },
    })
    calibration = source_path / "calibration.json" if source_path.is_dir() else None
    if calibration is not None and calibration.is_file():
        destination = paths.source / "calibration/intrinsics_extrinsics.json"
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(calibration, destination)


def _validate_frame(value: str, allowed: frozenset[str], label: str) -> str:
    if value not in allowed:
        choices = ", ".join(sorted(allowed))
        raise ValueError(f"invalid {label} {value!r}; expected one of: {choices}")
    return value


def read_ego_coordinate_system(
    dataset_root: str | Path,
    *,
    required: bool = True,
    allow_legacy_schema: bool = False,
) -> dict[str, Any] | None:
    """Read and validate coordinate semantics without inferring them from paths."""
    path = Path(dataset_root).expanduser().resolve() / "meta/coordinate_system.json"
    if not path.is_file():
        if required:
            raise FileNotFoundError(f"missing Ego coordinate contract: {path}")
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid Ego coordinate contract JSON: {path}") from error
    if data.get("schema_version") != COORDINATE_SCHEMA_VERSION:
        if allow_legacy_schema and data.get("schema_version") == "1.0":
            return None
        raise ValueError(
            f"unsupported coordinate schema {data.get('schema_version')!r} in {path}; "
            f"expected {COORDINATE_SCHEMA_VERSION}"
        )
    features = data.get("features")
    if not isinstance(features, dict):
        raise ValueError(f"coordinate contract has no features object: {path}")
    wrist = features.get("observation.wrist_pose")
    keypoints = features.get("observation.hand_keypoints")
    pixels = features.get("observation.hand_keypoints_2d")
    if not isinstance(wrist, dict) or not isinstance(keypoints, dict) or not isinstance(pixels, dict):
        raise ValueError(f"coordinate contract is missing canonical feature declarations: {path}")
    _validate_frame(wrist.get("frame"), WRIST_POSE_FRAMES, "wrist pose frame")
    _validate_frame(keypoints.get("frame"), HAND_KEYPOINT_FRAMES, "hand keypoint frame")
    if pixels.get("frame") != "ego_rgb_pixels":
        raise ValueError(f"invalid 2D keypoint frame in {path}")
    if wrist.get("quaternion_order") != "xyzw":
        raise ValueError(f"unsupported wrist quaternion order in {path}")
    return data


_DIRECT_ENVIRONMENT_PACKAGES = (
    "lerobot",
    "torch",
    "torchvision",
    "torchcodec",
    "numpy",
    "pandas",
    "pyarrow",
    "av",
    "opencv-python-headless",
    "scipy",
)


def _installed_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def write_environment_snapshot(paths: CapturePaths) -> None:
    """Record the environment that generated this Capture without changing data."""
    direct = {
        name: version
        for name in _DIRECT_ENVIRONMENT_PACKAGES
        if (version := _installed_version(name)) is not None
    }
    distributions = {}
    for distribution in importlib.metadata.distributions():
        name = distribution.metadata.get("Name")
        if name:
            distributions[name.lower()] = f"{name}=={distribution.version}"
    _write_json(paths.environment / "runtime.json", {
        "generated_at": _utc_now(),
        "python": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "platform": platform.platform(),
        "machine": platform.machine(),
        "dependencies": direct,
        "offline_dataset_validation": {
            "HF_HUB_OFFLINE": os.environ.get("HF_HUB_OFFLINE") == "1",
            "HF_DATASETS_OFFLINE": os.environ.get("HF_DATASETS_OFFLINE") == "1",
        },
    })
    requirements = [f"{name}=={version}" for name, version in sorted(direct.items())]
    _write_text(
        paths.environment / "requirements.txt",
        "\n".join(requirements) + ("\n" if requirements else ""),
    )
    _write_text(
        paths.environment / "environment.lock",
        "\n".join(distributions[name] for name in sorted(distributions)) + "\n",
    )


def _dataset_episode_indices(dataset_root: Path) -> list[int]:
    """Read materialized episode IDs without loading sample payloads or videos."""
    try:
        import pyarrow.parquet as pq
    except ImportError as error:
        raise RuntimeError("episode metadata generation requires pyarrow") from error

    indices = set()
    for path in sorted((dataset_root / "data").glob("chunk-*/file-*.parquet")):
        schema_names = set(pq.read_schema(path).names)
        if "episode_index" not in schema_names:
            continue
        column = pq.read_table(path, columns=["episode_index"])["episode_index"]
        indices.update(int(value.as_py()) for value in column)
    return sorted(indices)


def _all_finite(value: Any) -> bool:
    if isinstance(value, (list, tuple)):
        return all(_all_finite(item) for item in value)
    if value is None:
        return False
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def write_episode_annotations(dataset_root: Path, *, dataset_kind: str) -> list[Path]:
    """Create non-destructive review placeholders for every materialized episode."""
    written = []
    for episode_index in _dataset_episode_indices(dataset_root):
        destination = dataset_root / "annotations" / f"episode_{episode_index:06d}.json"
        if destination.exists():
            continue
        _write_json(destination, {
            "schema_version": "1.0",
            "dataset_kind": dataset_kind,
            "episode_index": episode_index,
            "review_status": "unreviewed",
            "excluded": False,
            "issues": [],
            "frame_ranges": [],
            "reviewer": None,
            "reviewed_at": None,
            "notes": "",
        })
        written.append(destination)
    return written


def write_robot_episode_qa(dataset_root: Path) -> list[Path]:
    """Write structural per-episode QA while leaving physical checks explicitly unevaluated."""
    try:
        import pyarrow.parquet as pq
    except ImportError as error:
        raise RuntimeError("RobotDataset episode QA requires pyarrow") from error

    aggregates: dict[int, dict[str, Any]] = {}
    for path in sorted((dataset_root / "data").glob("chunk-*/file-*.parquet")):
        schema_names = set(pq.read_schema(path).names)
        if "episode_index" not in schema_names:
            continue
        columns = [
            name for name in (
                "episode_index", "frame_index", "observation.state", "action"
            )
            if name in schema_names
        ]
        for row in pq.read_table(path, columns=columns).to_pylist():
            episode_index = int(row["episode_index"])
            aggregate = aggregates.setdefault(episode_index, {
                "frame_count": 0,
                "frame_indices": set(),
                "observation.state_available": False,
                "observation.state_finite": True,
                "action_available": False,
                "action_finite": True,
            })
            aggregate["frame_count"] += 1
            if row.get("frame_index") is not None:
                aggregate["frame_indices"].add(int(row["frame_index"]))
            for column in ("observation.state", "action"):
                if column in row:
                    aggregate[f"{column}_available"] = True
                    aggregate[f"{column}_finite"] &= _all_finite(row[column])

    written = []
    for episode_index, aggregate in sorted(aggregates.items()):
        frame_count = aggregate["frame_count"]
        contiguous = aggregate["frame_indices"] == set(range(frame_count))
        automatic_checks: dict[str, dict[str, Any]] = {
            "frame_count": {"value": frame_count, "pass": frame_count > 0},
            "frame_index_contiguous": {
                "value": contiguous,
                "pass": contiguous,
            },
        }
        for column in ("observation.state", "action"):
            available = aggregate[f"{column}_available"]
            automatic_checks[f"{column}_finite"] = {
                "value": aggregate[f"{column}_finite"] if available else None,
                "pass": aggregate[f"{column}_finite"] if available else None,
                "available": available,
            }
        evaluated = [check["pass"] for check in automatic_checks.values() if check["pass"] is not None]
        destination = dataset_root / "qa" / f"episode_{episode_index:06d}.json"
        _write_json(destination, {
            "schema_version": "1.0",
            "episode_index": episode_index,
            "automated_status": "passed" if evaluated and all(evaluated) else "failed",
            "automatic_checks": automatic_checks,
            "physical_checks": {
                "joint_limits": {"status": "not_evaluated", "reason": "robot limit evidence not attached"},
                "collision": {"status": "not_evaluated", "reason": "collision model evidence not attached"},
                "fingertip_error": {"status": "not_evaluated", "reason": "retarget ground truth not attached"},
            },
        })
        written.append(destination)
    return written


def write_ego_metadata(
    paths: CapturePaths,
    *,
    estimator: str,
    source_kind: str,
    wrist_pose_frame: str,
    wrist_pose_frame_declared_by: str,
    hand_keypoints_frame: str = "wrist_local_mano",
) -> None:
    wrist_pose_frame = _validate_frame(
        wrist_pose_frame, WRIST_POSE_FRAMES, "wrist pose frame"
    )
    hand_keypoints_frame = _validate_frame(
        hand_keypoints_frame, HAND_KEYPOINT_FRAMES, "hand keypoint frame"
    )
    if not wrist_pose_frame_declared_by:
        raise ValueError("wrist_pose_frame_declared_by must not be empty")
    meta = paths.ego / "meta"
    _write_json(meta / "ego_schema.json", {
        "schema_version": "1.0",
        "keypoint_count": 21,
        "keypoint_order": "MediaPipe Hands canonical indices 0..20",
        "wrist_pose": ["tx", "ty", "tz", "qx", "qy", "qz", "qw"],
        "quaternion_order": "xyzw",
        "hand_estimator": estimator,
    })
    _write_json(meta / "coordinate_system.json", {
        "schema_version": COORDINATE_SCHEMA_VERSION,
        "contract": "ego_coordinates_v1",
        "length_unit": "m",
        "angle_unit": "rad",
        "quaternion_order": "xyzw",
        "frames": {
            "episode0_camera": {
                "type": "cartesian_3d",
                "definition": "camera frame at episode frame 0",
                "handedness": "right",
                "axes": {"x": "image_right", "y": "image_down", "z": "optical_forward"},
            },
            "scene_world": {
                "type": "cartesian_3d",
                "definition": "source calibration or tracking world; not interchangeable with episode0_camera",
                "handedness": "right",
            },
            "wrist_local_mano": {
                "type": "cartesian_3d",
                "definition": "estimator-defined wrist-local MANO/hand frame",
                "handedness": "right",
            },
            "ego_rgb_pixels": {
                "type": "image_2d",
                "origin": "top_left",
                "axes": {"u": "right", "v": "down"},
                "unit": "px",
            },
        },
        "features": {
            "observation.wrist_pose": {
                "frame": wrist_pose_frame,
                "semantics": f"T_{wrist_pose_frame}_wrist",
                "representation": ["tx", "ty", "tz", "qx", "qy", "qz", "qw"],
                "translation_unit": "m",
                "quaternion_order": "xyzw",
                "declared_by": wrist_pose_frame_declared_by,
            },
            "observation.hand_keypoints": {
                "frame": hand_keypoints_frame,
                "unit": "m",
            },
            "observation.hand_keypoints_2d": {
                "frame": "ego_rgb_pixels",
                "unit": "px",
            },
        },
        "frame_relationships": {
            "scene_world_to_episode0_camera": None,
            "note": "No transform is inferred when a source does not provide one.",
        },
        "migration_note": "Metadata-only declaration; no coordinate, matrix, quaternion, or sample value was converted.",
    })
    read_ego_coordinate_system(paths.ego)
    _write_json(meta / "source_reference.json", {
        "capture_id": paths.capture_id,
        "source": "../../source",
        "source_kind": source_kind,
    })
    _write_json(meta / "processing.json", {
        "generated_at": _utc_now(),
        "pipeline": "canonical ego",
        "storage_migration": "capture_bundle_v1",
        "numerical_contract_changed": False,
    })
    write_episode_annotations(paths.ego, dataset_kind="ego")
    write_environment_snapshot(paths)
    paths.update_manifest(lambda data: data["datasets"].update({"ego": "ego"}))
    relations_path = paths.lineage / "dataset_relations.json"
    relations = json.loads(relations_path.read_text(encoding="utf-8"))
    relations["ego"] = "ego"
    _write_json(relations_path, relations)
    _append_jsonl(paths.lineage / "processing_runs.jsonl", {
        "timestamp": _utc_now(),
        "stage": "ego",
        "output": "ego",
        "source_kind": source_kind,
        "numerical_contract_changed": False,
    })
    _write_checksums(paths.ego, paths.ego / "checksums.json")
    _write_checksums(paths.source, paths.source / "checksums_original.json")
    # Ready is written last: implicit readers never observe a partially finalized Ego dataset.
    paths.mark_ego_ready()


def write_robot_metadata(
    paths: CapturePaths,
    *,
    target_id: str,
    dataset_root: Path,
    target_revision: str = TARGET_REVISION,
    retarget_revision: str = RETARGET_REVISION,
) -> None:
    relative = dataset_root.resolve().relative_to(paths.root.resolve()).as_posix()
    meta = dataset_root / "meta"
    coordinates = read_ego_coordinate_system(
        paths.ego,
        required=False,
        allow_legacy_schema=True,
    )
    wrist_pose_frame = (
        None
        if coordinates is None
        else coordinates["features"]["observation.wrist_pose"]["frame"]
    )
    _write_json(meta / "source_ego.json", {
        "capture_id": paths.capture_id,
        "dataset_path": "../../../../ego",
        "quaternion_order": "xyzw",
        "coordinate_system": "../../../../ego/meta/coordinate_system.json",
        "wrist_pose_frame": wrist_pose_frame,
        "coordinate_contract_status": (
            "legacy_undeclared" if coordinates is None else "declared"
        ),
    })
    _write_json(meta / "robot_asset_ref.json", {
        "target_id": target_id,
        "target_revision": target_revision,
    })
    _write_json(meta / "processing.json", {
        "generated_at": _utc_now(),
        "retarget_revision": retarget_revision,
        "storage_migration": "capture_bundle_v1",
        "numerical_contract_changed": False,
    })
    info_path = meta / "info.json"
    info = json.loads(info_path.read_text(encoding="utf-8")) if info_path.exists() else {}
    state_feature = info.get("features", {}).get("observation.state", {})
    _write_json(meta / "robot_schema.json", {
        "schema_version": "1.0",
        "target_id": target_id,
        "joint_order": state_feature.get("names", []),
        "state_dimension": state_feature.get("shape", [None])[0],
        "length_unit": "m",
        "angle_unit": "rad",
    })
    _write_json(meta / "retargeting.json", {
        "implementation": "derive_embodiment.py",
        "target_revision": target_revision,
        "retarget_revision": retarget_revision,
        "numerical_contract_changed_by_storage_migration": False,
    })
    write_episode_annotations(dataset_root, dataset_kind="robot")
    write_robot_episode_qa(dataset_root)

    def update(data: dict[str, Any]) -> None:
        data["datasets"].setdefault("robots", {})[target_id] = relative
        robots = data.setdefault("stages", {}).setdefault("robots", {})
        robots[target_id] = {
            "status": CAPTURE_READY,
            "completed_at": _utc_now(),
            "target_revision": target_revision,
            "retarget_revision": retarget_revision,
        }

    paths.update_manifest(update)
    relations_path = paths.lineage / "dataset_relations.json"
    relations = json.loads(relations_path.read_text(encoding="utf-8"))
    relations.setdefault("robots", {})[target_id] = relative
    _write_json(relations_path, relations)
    _append_jsonl(paths.lineage / "processing_runs.jsonl", {
        "timestamp": _utc_now(),
        "stage": "retarget",
        "source": "ego",
        "output": relative,
        "target_id": target_id,
        "target_revision": target_revision,
        "retarget_revision": retarget_revision,
        "numerical_contract_changed": False,
    })
    _write_checksums(dataset_root, dataset_root / "checksums.json")
