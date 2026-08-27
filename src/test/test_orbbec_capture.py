from __future__ import annotations

import json
import time
from datetime import datetime

import numpy as np
import pytest

from camera.capture_orbbec import Gemini336LCaptureWriter
from camera.orbbec_gemini336l import (
    CadenceReport,
    CadenceStats,
    CalibrationSnapshot,
    CameraExtrinsics,
    CameraIntrinsics,
    DeviceMetadata,
    RGBDFrame,
    load_gemini336l_profile,
)
from capture_bundle import CAPTURE_BUILDING, CAPTURE_FAILED, CAPTURE_READY, create_capture


class FakeAdapter:
    def __init__(self, *, interval_us: int = 16_667):
        self.profile = load_gemini336l_profile()
        self.backend = "v4l2"
        self.interval_us = interval_us
        self.index = 0
        intr_color = CameraIntrinsics(1280, 800, 600.0, 600.0, 640.0, 400.0, "NONE", (0.0,) * 8)
        intr_depth = CameraIntrinsics(848, 480, 410.0, 410.0, 424.0, 240.0, "NONE", (0.0,) * 8)
        self.calibration = CalibrationSnapshot(
            color=intr_color,
            depth=intr_depth,
            depth_to_color=CameraExtrinsics(
                "depth_optical_frame",
                "color_optical_frame",
                tuple(np.eye(3).reshape(-1)),
                (-0.024, 0.0, 0.0),
            ),
        )
        self.device_metadata = DeviceMetadata(
            "Orbbec Gemini 336L", "TEST_SERIAL", "1.4.60", "USB3.2", 0x2BC5, 0x0807, "2.9.3"
        )
        stats = CadenceStats.from_timestamps([1_000_000 + i * 16_667 for i in range(181)])
        self.startup_cadence = CadenceReport(stats, stats, 59.4)

    def read(self, *, decode_color: bool = True) -> RGBDFrame:
        assert decode_color is False
        self.index += 1
        timestamp = 10_000_000 + self.index * self.interval_us
        return RGBDFrame(
            sequence=self.index,
            color_bgr=None,
            color_mjpg=b"\xff\xd8native-mjpg\xff\xd9",
            depth_raw=np.asarray([[self.index, 2], [3, 4]], dtype=np.uint16),
            depth_unit_mm=1.0,
            color_frame_index=100 + self.index,
            depth_frame_index=200 + self.index,
            color_timestamp_hw_us=timestamp,
            depth_timestamp_hw_us=timestamp + 100,
            color_timestamp_global_us=timestamp + 1000,
            depth_timestamp_global_us=timestamp + 1100,
            color_timestamp_system_us=timestamp + 2000,
            depth_timestamp_system_us=timestamp + 2100,
            sync_error_ms=0.2,
            pairing_timestamp_source="global_timestamp_us",
            received_monotonic_ns=time.monotonic_ns(),
        )


def test_native_capture_writer_persists_source_contract(tmp_path):
    pq = pytest.importorskip("pyarrow.parquet")
    capture = create_capture(tmp_path, datetime(2026, 8, 27))
    result = Gemini336LCaptureWriter(
        capture,
        FakeAdapter(),
        queue_size=4,
    ).capture(duration_s=5.0, max_frames=5)

    assert result.frame_count == 5
    assert result.color_actual_hz >= 59.4
    assert result.depth_actual_hz >= 59.4
    assert result.max_sync_error_ms == pytest.approx(0.2)
    assert capture.status == CAPTURE_BUILDING
    manifest = json.loads(capture.bundle_manifest.read_text(encoding="utf-8"))
    assert manifest["stages"]["source"]["status"] == CAPTURE_READY
    assert manifest["stages"]["ego"]["status"] == CAPTURE_BUILDING

    rgb = capture.source / "rgb_original/episode_000000/frame_000000.jpg"
    depth = capture.source / "depth/raw/episode_000000/frame_000000.y16"
    assert rgb.read_bytes() == b"\xff\xd8native-mjpg\xff\xd9"
    assert np.frombuffer(depth.read_bytes(), dtype="<u2").tolist() == [1, 2, 3, 4]

    compatibility = pq.read_table(capture.source / "stream_index.parquet").to_pylist()
    assert len(compatibility) == 5
    assert compatibility[0]["rgb_path"].endswith("frame_000000.jpg")
    assert compatibility[0]["depth_raw_path"].endswith("frame_000000.y16")
    assert compatibility[0]["depth_aligned_path"] is None
    assert compatibility[0]["rgb_timestamp_hw_us"] == 10_016_667
    assert compatibility[0]["depth_timestamp_hw_us"] == 10_016_767

    streams = pq.read_table(capture.source / "streams.parquet").to_pylist()
    samples = pq.read_table(capture.source / "samples.parquet").to_pylist()
    assert {row["stream_id"] for row in streams} == {"gemini336l_color", "gemini336l_depth"}
    assert len(samples) == 10
    assert all(row["valid"] for row in samples)

    acquisition = json.loads((capture.source / "acquisition.json").read_text(encoding="utf-8"))
    assert acquisition["kind"] == "native_rgbd"
    assert acquisition["timebase"]["hardware_timestamps_available"] is True
    assert acquisition["config"]["alignment"] == "disabled_raw_depth_preserved"
    assert acquisition["config"]["persisted_cadence"]["passed"] is True
    assert (capture.source / "calibration/intrinsics_extrinsics.json").is_file()
    assert (capture.source / "checksums_original.json").is_file()


def test_native_capture_rejects_30hz_and_marks_source_failed(tmp_path):
    capture = create_capture(tmp_path, datetime(2026, 8, 27))
    writer = Gemini336LCaptureWriter(capture, FakeAdapter(interval_us=33_333), queue_size=4)
    with pytest.raises(Exception, match="30 FPS fallback is forbidden"):
        writer.capture(duration_s=5.0, max_frames=5)
    assert capture.status == CAPTURE_FAILED
    manifest = json.loads(capture.bundle_manifest.read_text(encoding="utf-8"))
    assert manifest["failure"]["stage"] == "source"
    assert manifest["stages"]["source"]["status"] == CAPTURE_FAILED


def test_native_capture_refuses_to_overwrite_existing_source(tmp_path):
    capture = create_capture(tmp_path, datetime(2026, 8, 27))
    journal = capture.source / "recordings/native_rgbd_frames.jsonl"
    journal.parent.mkdir(parents=True, exist_ok=True)
    journal.write_text("{}\n", encoding="utf-8")
    with pytest.raises(FileExistsError, match="already contains"):
        Gemini336LCaptureWriter(capture, FakeAdapter()).capture(duration_s=1.0, max_frames=2)
