#!/usr/bin/env python3
"""Persist native Gemini 336L RGB-D frames into a Capture Source stage."""
from __future__ import annotations

import argparse
import json
import os
import queue
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

try:  # PYTHONPATH=src python -m camera.capture_orbbec
    from capture_bundle import (
        CAPTURES_ROOT,
        CapturePaths,
        create_capture,
        open_capture,
        record_source,
        write_multisensor_source_index,
        write_native_rgbd_stream_index,
        write_source_checksums,
    )
    from quality_profiles import load_quality_profile
except ImportError:  # pragma: no cover - direct package layouts still use src
    raise

from .orbbec_gemini336l import (
    CadenceReport,
    CadenceStats,
    CadenceValidationError,
    Gemini336LAdapter,
    RGBDFrame,
)


@dataclass(frozen=True)
class SourceCaptureResult:
    capture_root: str
    frame_count: int
    interrupted: bool
    color_actual_hz: float
    depth_actual_hz: float
    max_sync_error_ms: float
    journal_path: str


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_bytes(payload)
    tmp.replace(path)


def _atomic_write_json(path: Path, value: dict[str, Any]) -> None:
    _atomic_write_bytes(
        path,
        (json.dumps(value, ensure_ascii=False, indent=2) + "\n").encode("utf-8"),
    )


def _clock_model(records: list[dict[str, Any]], prefix: str) -> dict[str, Any]:
    hardware_key = "rgb_timestamp_hw_us" if prefix == "color" else "depth_timestamp_hw_us"
    offsets = [
        row[f"{prefix}_timestamp_global_us"] - row[hardware_key]
        for row in records
        if row.get(f"{prefix}_timestamp_global_us") is not None
    ]
    if not offsets:
        return {
            "scale": 1.0,
            "offset_us": 0.0,
            "uncertainty_us": 0.0,
            "method": "device_hardware_timestamp_identity_no_global_timestamp",
        }
    median = float(np.median(np.asarray(offsets, dtype=np.float64)))
    uncertainty = float(np.max(np.abs(np.asarray(offsets, dtype=np.float64) - median)))
    return {
        "scale": 1.0,
        "offset_us": median,
        "uncertainty_us": uncertainty,
        "method": "per_frame_global_minus_device_timestamp_median",
    }


class Gemini336LCaptureWriter:
    """Bounded asynchronous writer; queue pressure is an acquisition failure."""

    def __init__(
        self,
        capture: CapturePaths,
        adapter: Gemini336LAdapter,
        *,
        quality_profile: str = "ego_fixed_rgbd_60hz_v1",
        queue_size: int = 120,
    ) -> None:
        if queue_size < 2:
            raise ValueError("queue_size must be at least 2")
        self.paths = capture
        self.adapter = adapter
        self.quality_profile = load_quality_profile(quality_profile)
        self.queue_size = queue_size
        self._queue: queue.Queue[RGBDFrame | None] = queue.Queue(maxsize=queue_size)
        self._records: list[dict[str, Any]] = []
        self._writer_error: BaseException | None = None
        self._writer_thread: threading.Thread | None = None
        self._journal_path = self.paths.source / "recordings/native_rgbd_frames.jsonl"
        self._rgb_dir = self.paths.source / "rgb_original/episode_000000"
        self._depth_dir = self.paths.source / "depth/raw/episode_000000"

    def _prepare(self) -> None:
        if self._journal_path.exists() or (self.paths.source / "stream_index.parquet").exists():
            raise FileExistsError(
                f"Capture Source already contains native RGB-D data: {self.paths.source}"
            )
        self._rgb_dir.mkdir(parents=True, exist_ok=True)
        self._depth_dir.mkdir(parents=True, exist_ok=True)
        self._journal_path.parent.mkdir(parents=True, exist_ok=True)
        self.paths.begin_source_capture()

    def _write_frame(self, frame: RGBDFrame, output_index: int) -> dict[str, Any]:
        stem = f"frame_{output_index:06d}"
        rgb_path = self._rgb_dir / f"{stem}.jpg"
        depth_path = self._depth_dir / f"{stem}.y16"
        _atomic_write_bytes(rgb_path, frame.color_mjpg)
        depth_le = np.asarray(frame.depth_raw, dtype="<u2", order="C")
        _atomic_write_bytes(depth_path, depth_le.tobytes(order="C"))
        record = {
            "episode_index": 0,
            "source_frame_index": output_index,
            "adapter_sequence": frame.sequence,
            "color_device_frame_index": frame.color_frame_index,
            "depth_device_frame_index": frame.depth_frame_index,
            "rgb_path": rgb_path.relative_to(self.paths.source).as_posix(),
            "depth_raw_path": depth_path.relative_to(self.paths.source).as_posix(),
            "rgb_timestamp_hw_us": frame.color_timestamp_hw_us,
            "depth_timestamp_hw_us": frame.depth_timestamp_hw_us,
            "color_timestamp_global_us": frame.color_timestamp_global_us,
            "depth_timestamp_global_us": frame.depth_timestamp_global_us,
            "color_timestamp_system_us": frame.color_timestamp_system_us,
            "depth_timestamp_system_us": frame.depth_timestamp_system_us,
            "sync_error_ms": frame.sync_error_ms,
            "pairing_basis": frame.pairing_timestamp_source,
            "depth_unit_mm": frame.depth_unit_mm,
            "received_monotonic_ns": frame.received_monotonic_ns,
        }
        with self._journal_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
            stream.flush()
        return record

    def _writer_loop(self) -> None:
        try:
            while True:
                frame = self._queue.get()
                try:
                    if frame is None:
                        return
                    self._records.append(self._write_frame(frame, len(self._records)))
                finally:
                    self._queue.task_done()
        except BaseException as error:
            self._writer_error = error

    def _start_writer(self) -> None:
        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name="gemini336l-source-writer",
            daemon=True,
        )
        self._writer_thread.start()

    def _stop_writer(self) -> None:
        thread = self._writer_thread
        if thread is None:
            return
        while thread.is_alive() and self._writer_error is None:
            try:
                self._queue.put(None, timeout=0.1)
                break
            except queue.Full:
                continue
        thread.join(timeout=30.0)
        if thread.is_alive():
            raise RuntimeError("native RGB-D writer did not stop within 30 seconds")
        if self._writer_error is not None:
            raise RuntimeError("native RGB-D writer failed") from self._writer_error

    def _enqueue(self, frame: RGBDFrame) -> None:
        if self._writer_error is not None:
            raise RuntimeError("native RGB-D writer failed") from self._writer_error
        try:
            self._queue.put(frame, timeout=0.5)
        except queue.Full as error:
            raise RuntimeError(
                f"native RGB-D writer queue is full ({self.queue_size}); refusing frame loss"
            ) from error

    def _finalize(self, *, interrupted: bool) -> SourceCaptureResult:
        if len(self._records) < 2:
            raise RuntimeError("native RGB-D capture produced fewer than two frame pairs")
        color = CadenceStats.from_timestamps(
            [row["rgb_timestamp_hw_us"] for row in self._records]
        )
        depth = CadenceStats.from_timestamps(
            [row["depth_timestamp_hw_us"] for row in self._records]
        )
        report = CadenceReport(
            color=color,
            depth=depth,
            minimum_hz=self.adapter.profile.minimum_measured_fps,
        )
        if not report.passed:
            raise CadenceValidationError(
                f"persisted Source cadence failed: color={color.actual_hz:.3f}Hz "
                f"depth={depth.actual_hz:.3f}Hz; 30 FPS fallback is forbidden"
            )
        max_sync_error_ms = max(float(row["sync_error_ms"]) for row in self._records)
        max_allowed_sync_ms = float(
            self.quality_profile["acquisition"]["rgb_depth_sync"]["max_error_ms"]
        )
        if max_sync_error_ms >= max_allowed_sync_ms:
            raise RuntimeError(
                f"RGB-D sync error {max_sync_error_ms:.3f}ms exceeds <{max_allowed_sync_ms:g}ms"
            )
        depth_units = {float(row["depth_unit_mm"]) for row in self._records}
        if len(depth_units) != 1 or next(iter(depth_units)) <= 0:
            raise RuntimeError("depth scale changed or is invalid during capture")
        depth_unit_mm = next(iter(depth_units))

        calibration_path = self.paths.source / "calibration/intrinsics_extrinsics.json"
        _atomic_write_json(calibration_path, {
            "schema_version": "orbbec_rgbd_calibration_v1",
            "device": asdict(self.adapter.device_metadata),
            "profile": asdict(self.adapter.profile),
            "calibration": self.adapter.calibration.as_dict(),
        })
        record_source(
            self.paths,
            kind="native_rgbd",
            source=self._journal_path,
            config={
                "camera": self.adapter.device_metadata.name,
                "serial_number": self.adapter.device_metadata.serial_number,
                "firmware_version": self.adapter.device_metadata.firmware_version,
                "sdk_version": self.adapter.device_metadata.sdk_version,
                "backend": self.adapter.backend,
                "fps": self.adapter.profile.color.fps,
                "rgb": asdict(self.adapter.profile.color),
                "depth": asdict(self.adapter.profile.depth),
                "frame_count": len(self._records),
                "depth_scale_m_per_unit": depth_unit_mm / 1000.0,
                "alignment": "disabled_raw_depth_preserved",
                "interrupted_by_operator": interrupted,
                "startup_cadence": (
                    self.adapter.startup_cadence.as_dict()
                    if self.adapter.startup_cadence is not None
                    else None
                ),
                "persisted_cadence": report.as_dict(),
                "max_sync_error_ms": max_sync_error_ms,
                "calibration": "calibration/intrinsics_extrinsics.json",
                "frame_journal": self._journal_path.relative_to(self.paths.source).as_posix(),
            },
            hardware_timestamps_available=True,
            quality_profile=self.quality_profile,
        )
        write_native_rgbd_stream_index(
            self.paths,
            records=self._records,
            fps=self.adapter.profile.color.fps,
            camera=self.adapter.device_metadata.name,
            depth_scale_m_per_unit=depth_unit_mm / 1000.0,
            depth_storage_format="y16_le",
        )
        samples = []
        for row in self._records:
            for prefix, stream_id in (("color", "gemini336l_color"), ("depth", "gemini336l_depth")):
                global_timestamp = row.get(f"{prefix}_timestamp_global_us")
                samples.append({
                    "episode_index": 0,
                    "stream_id": stream_id,
                    "sample_index": row["source_frame_index"],
                    "device_timestamp_us": row[f"{prefix if prefix == 'depth' else 'rgb'}_timestamp_hw_us"],
                    "master_timestamp_us": global_timestamp or row[
                        f"{prefix if prefix == 'depth' else 'rgb'}_timestamp_hw_us"
                    ],
                    "timestamp_uncertainty_us": row["sync_error_ms"] * 1000.0,
                    "path": row["depth_raw_path"] if prefix == "depth" else row["rgb_path"],
                    "valid": True,
                })
        write_multisensor_source_index(
            self.paths,
            streams=[
                {
                    "stream_id": "gemini336l_color",
                    "sensor_id": "gemini336l_color",
                    "modality": "rgb",
                    "nominal_rate_hz": self.adapter.profile.color.fps,
                    "calibration_id": "intrinsics_extrinsics_v1",
                    "clock_id": "color_device_clock",
                    "source_path": "rgb_original/episode_000000",
                },
                {
                    "stream_id": "gemini336l_depth",
                    "sensor_id": "gemini336l_depth",
                    "modality": "raw_depth",
                    "nominal_rate_hz": self.adapter.profile.depth.fps,
                    "calibration_id": "intrinsics_extrinsics_v1",
                    "clock_id": "depth_device_clock",
                    "source_path": "depth/raw/episode_000000",
                },
            ],
            samples=samples,
            master_clock="orbbec_global_clock",
            clock_models={
                "orbbec_global_clock": {
                    "scale": 1.0,
                    "offset_us": 0.0,
                    "uncertainty_us": 0.0,
                    "method": "master_global_timestamp_identity",
                },
                "color_device_clock": _clock_model(self._records, "color"),
                "depth_device_clock": _clock_model(self._records, "depth"),
            },
        )
        write_source_checksums(self.paths)
        self.paths.mark_source_ready()
        return SourceCaptureResult(
            capture_root=str(self.paths.root),
            frame_count=len(self._records),
            interrupted=interrupted,
            color_actual_hz=color.actual_hz,
            depth_actual_hz=depth.actual_hz,
            max_sync_error_ms=max_sync_error_ms,
            journal_path=str(self._journal_path),
        )

    def capture(
        self,
        *,
        duration_s: float,
        max_frames: int = 0,
    ) -> SourceCaptureResult:
        if duration_s <= 0 or max_frames < 0:
            raise ValueError("duration_s must be positive and max_frames non-negative")
        interrupted = False
        self._prepare()
        self._start_writer()
        start = time.monotonic()
        submitted = 0
        try:
            try:
                while time.monotonic() - start < duration_s:
                    if max_frames and submitted >= max_frames:
                        break
                    self._enqueue(self.adapter.read(decode_color=False))
                    submitted += 1
            except KeyboardInterrupt:
                interrupted = True
            self._stop_writer()
            return self._finalize(interrupted=interrupted)
        except BaseException as error:
            try:
                self._stop_writer()
            except BaseException:
                pass
            self.paths.mark_source_failed(str(error))
            raise


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture native Gemini 336L RGB-D into Source")
    parser.add_argument("--capture-root", help="Existing empty Capture; default creates a new Capture")
    parser.add_argument("--captures-root", default=str(CAPTURES_ROOT))
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--queue-size", type=int, default=120)
    parser.add_argument("--serial")
    parser.add_argument("--backend", choices=("v4l2", "libuvc", "auto"), default="v4l2")
    parser.add_argument("--validate-seconds", type=float, default=12.0)
    parser.add_argument("--quality-profile", default="ego_fixed_rgbd_60hz_v1")
    args = parser.parse_args()

    capture = (
        open_capture(args.capture_root, Path(args.captures_root))
        if args.capture_root
        else create_capture(Path(args.captures_root))
    )
    adapter = Gemini336LAdapter(serial_number=args.serial, backend=args.backend)
    try:
        adapter.start(validate_cadence=False)
        adapter.validate_cadence(args.validate_seconds)
        result = Gemini336LCaptureWriter(
            capture,
            adapter,
            quality_profile=args.quality_profile,
            queue_size=args.queue_size,
        ).capture(duration_s=args.duration, max_frames=args.max_frames)
        print(json.dumps(asdict(result), ensure_ascii=False, indent=2))
        print("Source 已完成；Capture 保持 building，下一步用同一 --capture-root 构建 Ego。")
        return 0
    except BaseException as error:
        if not isinstance(error, KeyboardInterrupt):
            print(f"Source capture failed: {error}")
        raise
    finally:
        adapter.close()


if __name__ == "__main__":
    raise SystemExit(main())
