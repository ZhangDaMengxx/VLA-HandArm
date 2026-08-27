from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from camera.orbbec_gemini336l import (
    CadenceStats,
    CadenceValidationError,
    CameraConfigurationError,
    Gemini336LAdapter,
    load_gemini336l_profile,
)


class EnumValue:
    def __init__(self, name: str):
        self.name = name

    def __repr__(self) -> str:
        return self.name


class FakeInfo:
    def __init__(self, *, pid: int = 0x0807):
        self.pid = pid

    def get_name(self):
        return "Orbbec Gemini 336L"

    def get_serial_number(self):
        return "CPC876300084"

    def get_firmware_version(self):
        return "1.4.60"

    def get_connection_type(self):
        return "USB3.2"

    def get_vid(self):
        return 0x2BC5

    def get_pid(self):
        return self.pid


class FakeDevice:
    def __init__(self, *, pid: int = 0x0807):
        self.info = FakeInfo(pid=pid)
        self.global_timestamps_enabled = False

    def get_device_info(self):
        return self.info

    def enable_global_timestamp(self, enabled):
        self.global_timestamps_enabled = bool(enabled)


class FakeDeviceList:
    def __init__(self, device):
        self.device = device

    def get_count(self):
        return 1

    def get_device_by_index(self, index):
        assert index == 0
        return self.device

    def get_device_by_serial_number(self, serial):
        if serial != self.device.info.get_serial_number():
            raise RuntimeError("serial not found")
        return self.device


class FakeDistortion:
    model = EnumValue("BROWN_CONRADY")
    k1 = 0.1
    k2 = -0.2
    k3 = 0.01
    k4 = 0.0
    k5 = 0.0
    k6 = 0.0
    p1 = 0.001
    p2 = -0.001


class FakeIntrinsic:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.fx = width * 0.8
        self.fy = height * 1.1
        self.cx = width / 2
        self.cy = height / 2


class FakeExtrinsic:
    rot = np.eye(3, dtype=np.float32).reshape(-1)
    transform = np.asarray([10.0, -2.0, 3.0], dtype=np.float32)


class FakeProfile:
    def __init__(self, width, height, fps, fmt):
        self.width = width
        self.height = height
        self.fps = fps
        self.fmt = fmt

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_fps(self):
        return self.fps

    def get_format(self):
        return self.fmt

    def get_intrinsic(self):
        return FakeIntrinsic(self.width, self.height)

    def get_distortion(self):
        return FakeDistortion()

    def get_extrinsic_to(self, target):
        assert target is not None
        return FakeExtrinsic()


class FakeProfileList:
    def __init__(self, sensor, sdk):
        self.sensor = sensor
        self.sdk = sdk

    def get_video_stream_profile(self, width, height, fmt, fps):
        if self.sensor == self.sdk.OBSensorType.COLOR_SENSOR:
            if self.sdk.wrong_profile:
                return FakeProfile(width, height, 30, fmt)
            return FakeProfile(width, height, fps, fmt)
        return FakeProfile(width, height, fps, fmt)


class FakeConfig:
    def __init__(self):
        self.streams = []
        self.align_mode = None
        self.aggregate_mode = None

    def enable_stream(self, profile):
        self.streams.append(profile)

    def set_align_mode(self, mode):
        self.align_mode = mode

    def set_frame_aggregate_output_mode(self, mode):
        self.aggregate_mode = mode


class FakeFrame:
    def __init__(self, spec, timestamp_us, index, payload, global_offset_us):
        self.spec = spec
        self.timestamp_us = timestamp_us
        self.index = index
        self.payload = payload
        self.global_offset_us = global_offset_us

    def get_width(self):
        return self.spec[0]

    def get_height(self):
        return self.spec[1]

    def get_format(self):
        return self.spec[2]

    def get_timestamp_us(self):
        return self.timestamp_us

    def get_global_timestamp_us(self):
        return self.timestamp_us + self.global_offset_us

    def get_system_timestamp_us(self):
        return self.timestamp_us + 5000

    def get_index(self):
        return self.index

    def get_data(self):
        return self.payload

    def get_depth_scale(self):
        return 1.0


class FakeFrames:
    def __init__(self, color, depth):
        self.color = color
        self.depth = depth

    def get_color_frame(self):
        return self.color

    def get_depth_frame(self):
        return self.depth


class FakePipeline:
    def __init__(self, device, sdk):
        self.device = device
        self.sdk = sdk
        self.index = 0
        self.started = False
        self.stopped = False
        self.frame_sync = False
        self.config = None

    def get_stream_profile_list(self, sensor):
        return FakeProfileList(sensor, self.sdk)

    def enable_frame_sync(self):
        self.frame_sync = True

    def start(self, config):
        self.started = True
        self.config = config

    def stop(self):
        self.stopped = True

    def wait_for_frames(self, timeout_ms):
        assert timeout_ms == 1000
        self.index += 1
        timestamp = 1_000_000 + self.index * self.sdk.interval_us
        color = FakeFrame(
            (1280, 800, self.sdk.OBFormat.MJPG), timestamp, self.index,
            self.sdk.color_payload, 100,
        )
        depth = FakeFrame(
            (848, 480, self.sdk.OBFormat.Y16), timestamp + 100, self.index,
            self.sdk.depth_payload, 200,
        )
        return FakeFrames(color, depth)


class FakeContext:
    def __init__(self, sdk):
        self.sdk = sdk
        self.backend = None

    def set_uvc_backend_type(self, backend):
        self.backend = backend

    def query_devices(self):
        return FakeDeviceList(self.sdk.device)


class FakeSDK:
    def __init__(self, *, interval_us=16_667, pid=0x0807, wrong_profile=False):
        self.interval_us = interval_us
        self.wrong_profile = wrong_profile
        self.device = FakeDevice(pid=pid)
        self.OBUvcBackendType = SimpleNamespace(
            V4L2=EnumValue("V4L2"), LIBUVC=EnumValue("LIBUVC"), AUTO=EnumValue("AUTO")
        )
        self.OBSensorType = SimpleNamespace(
            COLOR_SENSOR=EnumValue("COLOR_SENSOR"), DEPTH_SENSOR=EnumValue("DEPTH_SENSOR")
        )
        self.OBFormat = SimpleNamespace(MJPG=EnumValue("MJPG"), Y16=EnumValue("Y16"))
        self.OBAlignMode = SimpleNamespace(DISABLE=EnumValue("DISABLE"))
        self.OBFrameAggregateOutputMode = SimpleNamespace(
            FULL_FRAME_REQUIRE=EnumValue("FULL_FRAME_REQUIRE")
        )
        image = np.zeros((800, 1280, 3), dtype=np.uint8)
        ok, encoded = cv2.imencode(".jpg", image)
        assert ok
        self.color_payload = encoded
        self.depth_payload = np.zeros((480, 848), dtype=np.uint16).tobytes()
        self.pipeline = None
        self.context = None

    def Context(self):
        self.context = FakeContext(self)
        return self.context

    def Pipeline(self, device):
        self.pipeline = FakePipeline(device, self)
        return self.pipeline

    Config = FakeConfig

    @staticmethod
    def get_version():
        return "2.9.3"


def test_production_profile_is_exact_and_fail_closed():
    profile = load_gemini336l_profile()
    assert (profile.color.width, profile.color.height, profile.color.fps, profile.color.format) == (
        1280, 800, 60, "MJPG"
    )
    assert (profile.depth.width, profile.depth.height, profile.depth.fps, profile.depth.format) == (
        848, 480, 60, "Y16"
    )
    assert profile.minimum_measured_fps == 59.4
    assert profile.backend == "v4l2"


def test_profile_rejects_30_fps_fallback(tmp_path: Path):
    data = json.loads(Path("configs/camera/orbbec_gemini336l_60fps.json").read_text())
    next(item for item in data["streams"] if item["sensor"] == "depth")["fps"] = 30
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(CameraConfigurationError, match="30 FPS fallback is forbidden"):
        load_gemini336l_profile(path)


def test_cadence_stats_uses_device_timestamps_and_detects_regression():
    good = CadenceStats.from_timestamps([1_000_000 + i * 16_667 for i in range(181)])
    slow = CadenceStats.from_timestamps([1_000_000 + i * 33_333 for i in range(91)])
    backwards = CadenceStats.from_timestamps([1_000_000, 1_016_667, 1_010_000])
    assert good.actual_hz >= 59.4
    assert slow.actual_hz < 31
    assert backwards.nonmonotonic == 1


def test_adapter_selects_exact_profiles_and_returns_native_frame():
    sdk = FakeSDK()
    adapter = Gemini336LAdapter(sdk_module=sdk)
    report = adapter.start()
    try:
        assert report is not None and report.passed
        assert sdk.context.backend.name == "V4L2"
        assert sdk.device.global_timestamps_enabled
        assert sdk.pipeline.frame_sync
        assert sdk.pipeline.config.align_mode.name == "DISABLE"
        assert sdk.pipeline.config.aggregate_mode.name == "FULL_FRAME_REQUIRE"
        assert adapter.device_metadata.pid == 0x0807
        assert adapter.calibration.depth_to_color.translation_m == pytest.approx((0.01, -0.002, 0.003))

        frame = adapter.read()
        assert frame.color_bgr.shape == (800, 1280, 3)
        assert frame.depth_raw.shape == (480, 848)
        assert frame.depth_raw.dtype == np.uint16
        assert frame.color_mjpg.startswith(b"\xff\xd8")
        assert frame.depth_unit_mm == 1.0
        assert frame.sync_error_ms == pytest.approx(0.2)
        assert frame.pairing_timestamp_source == "global_timestamp_us"
    finally:
        adapter.close()
    assert sdk.pipeline.stopped


def test_adapter_rejects_runtime_profile_fallback():
    adapter = Gemini336LAdapter(sdk_module=FakeSDK(wrong_profile=True))
    with pytest.raises(CameraConfigurationError, match="SDK selected"):
        adapter.start()


def test_adapter_rejects_30hz_measured_cadence_and_closes():
    sdk = FakeSDK(interval_us=33_333)
    adapter = Gemini336LAdapter(sdk_module=sdk)
    with pytest.raises(CadenceValidationError, match="30 FPS fallback is forbidden"):
        adapter.start()
    assert not adapter.running
    assert sdk.pipeline.stopped


def test_adapter_rejects_wrong_usb_device():
    adapter = Gemini336LAdapter(sdk_module=FakeSDK(pid=0x1234))
    with pytest.raises(CameraConfigurationError, match="not found"):
        adapter.start()
