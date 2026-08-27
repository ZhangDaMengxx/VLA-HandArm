#!/usr/bin/env python3
"""Fail-closed Orbbec Gemini 336L RGB-D acquisition adapter.

The adapter owns one camera, selects the production profiles explicitly, and
returns copied native data with device timestamps and calibration. It does not
align depth, write Capture bundles, or issue robot commands.
"""
from __future__ import annotations

import argparse
import importlib
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from types import ModuleType
from typing import Any

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = REPO_ROOT / "configs/camera/orbbec_gemini336l_60fps.json"


class CameraConfigurationError(RuntimeError):
    """The connected device or selected stream does not match production."""


class CameraStreamError(RuntimeError):
    """A frame is missing, malformed, stale, or has an invalid timestamp."""


class CadenceValidationError(CameraStreamError):
    """Measured device timestamp cadence is below the production threshold."""


@dataclass(frozen=True)
class StreamSpec:
    sensor: str
    width: int
    height: int
    fps: int
    format: str


PRODUCTION_COLOR = StreamSpec("color", 1280, 800, 60, "MJPG")
PRODUCTION_DEPTH = StreamSpec("depth", 848, 480, 60, "Y16")


@dataclass(frozen=True)
class Gemini336LProfile:
    color: StreamSpec
    depth: StreamSpec
    backend: str
    minimum_measured_fps: float
    startup_validation_seconds: float
    frame_timeout_ms: int
    expected_name: str
    expected_vid: int
    expected_pid: int


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion_model: str
    distortion_coefficients: tuple[float, ...]


@dataclass(frozen=True)
class CameraExtrinsics:
    source_frame: str
    target_frame: str
    rotation: tuple[float, ...]
    translation_m: tuple[float, float, float]


@dataclass(frozen=True)
class CalibrationSnapshot:
    color: CameraIntrinsics
    depth: CameraIntrinsics
    depth_to_color: CameraExtrinsics

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class DeviceMetadata:
    name: str
    serial_number: str
    firmware_version: str
    connection_type: str
    vid: int
    pid: int
    sdk_version: str | None


@dataclass(frozen=True)
class CadenceStats:
    count: int
    first_timestamp_us: int
    last_timestamp_us: int
    actual_hz: float
    max_gap_ms: float
    nonmonotonic: int

    @classmethod
    def from_timestamps(cls, timestamps_us: list[int]) -> "CadenceStats":
        if len(timestamps_us) < 2:
            return cls(len(timestamps_us), 0, 0, 0.0, 0.0, 0)
        gaps = np.diff(np.asarray(timestamps_us, dtype=np.int64))
        positive = gaps[gaps > 0]
        elapsed_us = int(timestamps_us[-1]) - int(timestamps_us[0])
        actual_hz = (
            float(len(timestamps_us) - 1) * 1_000_000.0 / elapsed_us
            if elapsed_us > 0
            else 0.0
        )
        return cls(
            count=len(timestamps_us),
            first_timestamp_us=int(timestamps_us[0]),
            last_timestamp_us=int(timestamps_us[-1]),
            actual_hz=actual_hz,
            max_gap_ms=float(np.max(positive)) / 1000.0 if positive.size else 0.0,
            nonmonotonic=int(np.count_nonzero(gaps <= 0)),
        )


@dataclass(frozen=True)
class CadenceReport:
    color: CadenceStats
    depth: CadenceStats
    minimum_hz: float

    @property
    def passed(self) -> bool:
        return (
            self.color.actual_hz >= self.minimum_hz
            and self.depth.actual_hz >= self.minimum_hz
            and self.color.nonmonotonic == 0
            and self.depth.nonmonotonic == 0
        )

    def as_dict(self) -> dict[str, Any]:
        return asdict(self) | {"passed": self.passed}


@dataclass(frozen=True)
class RGBDFrame:
    sequence: int
    color_bgr: np.ndarray
    color_mjpg: bytes
    depth_raw: np.ndarray
    depth_unit_mm: float
    color_frame_index: int
    depth_frame_index: int
    color_timestamp_hw_us: int
    depth_timestamp_hw_us: int
    color_timestamp_global_us: int | None
    depth_timestamp_global_us: int | None
    color_timestamp_system_us: int | None
    depth_timestamp_system_us: int | None
    sync_error_ms: float
    pairing_timestamp_source: str
    received_monotonic_ns: int


def _parse_int(value: Any) -> int:
    if isinstance(value, str):
        return int(value, 0)
    return int(value)


def load_gemini336l_profile(path: str | Path = DEFAULT_CONFIG_PATH) -> Gemini336LProfile:
    config_path = Path(path)
    data = json.loads(config_path.read_text(encoding="utf-8"))
    streams: dict[str, StreamSpec] = {}
    for item in data.get("streams", []):
        spec = StreamSpec(
            sensor=str(item["sensor"]).lower(),
            width=int(item["width"]),
            height=int(item["height"]),
            fps=int(item["fps"]),
            format=str(item["format"]).upper(),
        )
        if spec.sensor in streams:
            raise CameraConfigurationError(f"duplicate stream: {spec.sensor}")
        streams[spec.sensor] = spec
    color = streams.get("color")
    depth = streams.get("depth")
    if color != PRODUCTION_COLOR or depth != PRODUCTION_DEPTH:
        raise CameraConfigurationError(
            "Gemini 336L production profile must be color 1280x800@60 MJPG "
            "and depth 848x480@60 Y16; 30 FPS fallback is forbidden"
        )
    device = data.get("device", {})
    backend = str(data.get("backend", "v4l2")).lower()
    if backend not in {"v4l2", "libuvc", "auto"}:
        raise CameraConfigurationError(f"unsupported UVC backend: {backend}")
    minimum_fps = float(data.get("minimum_measured_fps", 59.4))
    if minimum_fps < 59.4:
        raise CameraConfigurationError("minimum_measured_fps cannot be below 59.4")
    validation_seconds = float(data.get("startup_validation_seconds", 3.0))
    if validation_seconds < 2.0:
        raise CameraConfigurationError("startup_validation_seconds must be at least 2.0")
    frame_timeout_ms = int(data.get("frame_timeout_ms", 1000))
    if frame_timeout_ms < 100:
        raise CameraConfigurationError("frame_timeout_ms must be at least 100")
    return Gemini336LProfile(
        color=color,
        depth=depth,
        backend=backend,
        minimum_measured_fps=minimum_fps,
        startup_validation_seconds=validation_seconds,
        frame_timeout_ms=frame_timeout_ms,
        expected_name=str(device.get("name", "Gemini 336L")),
        expected_vid=_parse_int(device.get("vid", "0x2bc5")),
        expected_pid=_parse_int(device.get("pid", "0x0807")),
    )


def _enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if name:
        return str(name).upper()
    return str(value).rsplit(".", 1)[-1].upper()


def _optional_positive(callable_value: Any) -> int | None:
    try:
        value = int(callable_value())
    except (AttributeError, TypeError, ValueError, RuntimeError):
        return None
    return value if value > 0 else None


class Gemini336LAdapter:
    """Single-owner native RGB-D adapter for the fixed Gemini 336L profile."""

    def __init__(
        self,
        config_path: str | Path = DEFAULT_CONFIG_PATH,
        *,
        serial_number: str | None = None,
        backend: str | None = None,
        sdk_module: ModuleType | Any | None = None,
    ) -> None:
        self.profile = load_gemini336l_profile(config_path)
        selected_backend = (backend or self.profile.backend).lower()
        if selected_backend not in {"v4l2", "libuvc", "auto"}:
            raise CameraConfigurationError(f"unsupported UVC backend: {selected_backend}")
        self.backend = selected_backend
        self.serial_number = serial_number
        self._sdk = sdk_module
        self._context: Any | None = None
        self._device: Any | None = None
        self._pipeline: Any | None = None
        self._running = False
        self._sequence = 0
        self._last_color_timestamp_us = 0
        self._last_depth_timestamp_us = 0
        self.device_metadata: DeviceMetadata | None = None
        self.calibration: CalibrationSnapshot | None = None
        self.startup_cadence: CadenceReport | None = None

    @property
    def running(self) -> bool:
        return self._running

    def _load_sdk(self) -> Any:
        if self._sdk is not None:
            return self._sdk
        try:
            self._sdk = importlib.import_module("pyorbbecsdk")
        except ImportError as exc:
            raise RuntimeError(
                "pyorbbecsdk is not installed in this Python environment; build/install "
                "third_party/pyorbbecsdk-2-main before starting the Adapter"
            ) from exc
        return self._sdk

    def _select_device(self, devices: Any) -> Any:
        if int(devices.get_count()) == 0:
            raise CameraConfigurationError("no Orbbec camera found")
        if self.serial_number:
            return devices.get_device_by_serial_number(self.serial_number)
        matches = []
        for index in range(int(devices.get_count())):
            device = devices.get_device_by_index(index)
            info = device.get_device_info()
            if (
                int(info.get_vid()) == self.profile.expected_vid
                and int(info.get_pid()) == self.profile.expected_pid
            ):
                matches.append(device)
        if not matches:
            raise CameraConfigurationError(
                f"Gemini 336L {self.profile.expected_vid:04x}:{self.profile.expected_pid:04x} not found"
            )
        if len(matches) > 1:
            raise CameraConfigurationError("multiple Gemini 336L devices found; specify serial_number")
        return matches[0]

    def _device_metadata(self, device: Any, sdk: Any) -> DeviceMetadata:
        info = device.get_device_info()
        metadata = DeviceMetadata(
            name=str(info.get_name()),
            serial_number=str(info.get_serial_number()),
            firmware_version=str(info.get_firmware_version()),
            connection_type=str(info.get_connection_type()),
            vid=int(info.get_vid()),
            pid=int(info.get_pid()),
            sdk_version=str(sdk.get_version()) if hasattr(sdk, "get_version") else None,
        )
        if metadata.vid != self.profile.expected_vid or metadata.pid != self.profile.expected_pid:
            raise CameraConfigurationError(
                f"wrong camera USB ID {metadata.vid:04x}:{metadata.pid:04x}"
            )
        if self.profile.expected_name.lower() not in metadata.name.lower():
            raise CameraConfigurationError(
                f"wrong camera model {metadata.name!r}; expected {self.profile.expected_name!r}"
            )
        if self.serial_number and metadata.serial_number != self.serial_number:
            raise CameraConfigurationError("selected camera serial number does not match")
        return metadata

    @staticmethod
    def _assert_profile(profile: Any, spec: StreamSpec) -> None:
        actual = StreamSpec(
            sensor=spec.sensor,
            width=int(profile.get_width()),
            height=int(profile.get_height()),
            fps=int(profile.get_fps()),
            format=_enum_name(profile.get_format()),
        )
        if actual != spec:
            raise CameraConfigurationError(f"SDK selected {actual}, expected {spec}")

    @staticmethod
    def _intrinsics(profile: Any) -> CameraIntrinsics:
        intrinsic = profile.get_intrinsic()
        distortion = profile.get_distortion()
        return CameraIntrinsics(
            width=int(intrinsic.width),
            height=int(intrinsic.height),
            fx=float(intrinsic.fx),
            fy=float(intrinsic.fy),
            cx=float(intrinsic.cx),
            cy=float(intrinsic.cy),
            distortion_model=_enum_name(distortion.model),
            distortion_coefficients=tuple(
                float(getattr(distortion, key))
                for key in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2")
            ),
        )

    @classmethod
    def _calibration(cls, color_profile: Any, depth_profile: Any) -> CalibrationSnapshot:
        extrinsic = depth_profile.get_extrinsic_to(color_profile)
        rotation = tuple(float(value) for value in np.asarray(extrinsic.rot).reshape(9))
        translation = np.asarray(extrinsic.transform, dtype=np.float64).reshape(3) / 1000.0
        return CalibrationSnapshot(
            color=cls._intrinsics(color_profile),
            depth=cls._intrinsics(depth_profile),
            depth_to_color=CameraExtrinsics(
                source_frame="depth_optical_frame",
                target_frame="color_optical_frame",
                rotation=rotation,
                translation_m=tuple(float(value) for value in translation),
            ),
        )

    def start(self, *, validate_cadence: bool = True) -> CadenceReport | None:
        if self._running:
            raise RuntimeError("Gemini336LAdapter is already running")
        sdk = self._load_sdk()
        try:
            context = sdk.Context()
            backend_enum = getattr(sdk.OBUvcBackendType, self.backend.upper())
            context.set_uvc_backend_type(backend_enum)
            device = self._select_device(context.query_devices())
            metadata = self._device_metadata(device, sdk)
            if hasattr(device, "enable_global_timestamp"):
                device.enable_global_timestamp(True)
            pipeline = sdk.Pipeline(device)
            color_profile = pipeline.get_stream_profile_list(
                sdk.OBSensorType.COLOR_SENSOR
            ).get_video_stream_profile(
                self.profile.color.width,
                self.profile.color.height,
                sdk.OBFormat.MJPG,
                self.profile.color.fps,
            )
            depth_profile = pipeline.get_stream_profile_list(
                sdk.OBSensorType.DEPTH_SENSOR
            ).get_video_stream_profile(
                self.profile.depth.width,
                self.profile.depth.height,
                sdk.OBFormat.Y16,
                self.profile.depth.fps,
            )
            self._assert_profile(color_profile, self.profile.color)
            self._assert_profile(depth_profile, self.profile.depth)
            calibration = self._calibration(color_profile, depth_profile)

            config = sdk.Config()
            config.enable_stream(color_profile)
            config.enable_stream(depth_profile)
            config.set_align_mode(sdk.OBAlignMode.DISABLE)
            config.set_frame_aggregate_output_mode(
                sdk.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE
            )
            if hasattr(pipeline, "enable_frame_sync"):
                pipeline.enable_frame_sync()
            pipeline.start(config)
        except Exception:
            if "pipeline" in locals():
                try:
                    pipeline.stop()
                except Exception:
                    pass
            raise

        self._context = context
        self._device = device
        self._pipeline = pipeline
        self.device_metadata = metadata
        self.calibration = calibration
        self._running = True
        try:
            if validate_cadence:
                self.startup_cadence = self.validate_cadence(
                    self.profile.startup_validation_seconds
                )
            return self.startup_cadence
        except Exception:
            self.close()
            raise

    def _wait_pair(self) -> tuple[Any, Any]:
        if not self._running or self._pipeline is None:
            raise RuntimeError("Gemini336LAdapter is not running")
        frames = self._pipeline.wait_for_frames(self.profile.frame_timeout_ms)
        if frames is None:
            raise CameraStreamError("timed out waiting for RGB-D frames")
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if color is None or depth is None:
            raise CameraStreamError("incomplete RGB-D frame set; no fallback is allowed")
        self._assert_video_frame(color, self.profile.color)
        self._assert_video_frame(depth, self.profile.depth)
        return color, depth

    @staticmethod
    def _assert_video_frame(frame: Any, spec: StreamSpec) -> None:
        if (
            int(frame.get_width()) != spec.width
            or int(frame.get_height()) != spec.height
            or _enum_name(frame.get_format()) != spec.format
        ):
            raise CameraStreamError(f"runtime frame no longer matches {spec}")

    @staticmethod
    def _hardware_timestamp(frame: Any, stream: str) -> int:
        timestamp = int(frame.get_timestamp_us())
        if timestamp <= 0:
            raise CameraStreamError(f"{stream} frame has no hardware timestamp")
        return timestamp

    def validate_cadence(self, duration_s: float | None = None) -> CadenceReport:
        duration = float(duration_s or self.profile.startup_validation_seconds)
        if duration < 2.0:
            raise ValueError("cadence validation duration must be at least 2 seconds")
        color_timestamps: list[int] = []
        depth_timestamps: list[int] = []
        target_span_us = int(duration * 1_000_000)
        deadline = time.monotonic() + duration + max(2.0, self.profile.frame_timeout_ms / 500.0)
        while True:
            if time.monotonic() > deadline:
                raise CadenceValidationError("timed out before collecting the cadence window")
            color, depth = self._wait_pair()
            color_timestamps.append(self._hardware_timestamp(color, "color"))
            depth_timestamps.append(self._hardware_timestamp(depth, "depth"))
            color_span = color_timestamps[-1] - color_timestamps[0]
            depth_span = depth_timestamps[-1] - depth_timestamps[0]
            if color_span >= target_span_us and depth_span >= target_span_us:
                break
        report = CadenceReport(
            color=CadenceStats.from_timestamps(color_timestamps),
            depth=CadenceStats.from_timestamps(depth_timestamps),
            minimum_hz=self.profile.minimum_measured_fps,
        )
        if not report.passed:
            raise CadenceValidationError(
                "Gemini 336L 60 FPS validation failed: "
                f"color={report.color.actual_hz:.3f}Hz depth={report.depth.actual_hz:.3f}Hz "
                f"nonmonotonic={report.color.nonmonotonic}/{report.depth.nonmonotonic}; "
                "30 FPS fallback is forbidden"
            )
        self._last_color_timestamp_us = report.color.last_timestamp_us
        self._last_depth_timestamp_us = report.depth.last_timestamp_us
        self.startup_cadence = report
        return report

    def read(self) -> RGBDFrame:
        color, depth = self._wait_pair()
        color_timestamp = self._hardware_timestamp(color, "color")
        depth_timestamp = self._hardware_timestamp(depth, "depth")
        if color_timestamp <= self._last_color_timestamp_us:
            raise CameraStreamError("color hardware timestamp is not monotonic")
        if depth_timestamp <= self._last_depth_timestamp_us:
            raise CameraStreamError("depth hardware timestamp is not monotonic")
        self._last_color_timestamp_us = color_timestamp
        self._last_depth_timestamp_us = depth_timestamp

        color_bytes = np.asarray(color.get_data(), dtype=np.uint8).reshape(-1).tobytes()
        color_bgr = cv2.imdecode(np.frombuffer(color_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if color_bgr is None or color_bgr.shape != (
            self.profile.color.height,
            self.profile.color.width,
            3,
        ):
            raise CameraStreamError("failed to decode the native MJPG color frame")
        expected_depth_values = self.profile.depth.width * self.profile.depth.height
        depth_raw = np.frombuffer(depth.get_data(), dtype=np.uint16)
        if depth_raw.size != expected_depth_values:
            raise CameraStreamError(
                f"depth payload has {depth_raw.size} values, expected {expected_depth_values}"
            )
        depth_raw = depth_raw.reshape(
            self.profile.depth.height, self.profile.depth.width
        ).copy()

        color_global = _optional_positive(color.get_global_timestamp_us)
        depth_global = _optional_positive(depth.get_global_timestamp_us)
        if color_global is not None and depth_global is not None:
            sync_error_ms = abs(color_global - depth_global) / 1000.0
            pairing_source = "global_timestamp_us"
        else:
            sync_error_ms = abs(color_timestamp - depth_timestamp) / 1000.0
            pairing_source = "hardware_timestamp_us"
        self._sequence += 1
        return RGBDFrame(
            sequence=self._sequence,
            color_bgr=color_bgr,
            color_mjpg=color_bytes,
            depth_raw=depth_raw,
            depth_unit_mm=float(depth.get_depth_scale()),
            color_frame_index=int(color.get_index()),
            depth_frame_index=int(depth.get_index()),
            color_timestamp_hw_us=color_timestamp,
            depth_timestamp_hw_us=depth_timestamp,
            color_timestamp_global_us=color_global,
            depth_timestamp_global_us=depth_global,
            color_timestamp_system_us=_optional_positive(color.get_system_timestamp_us),
            depth_timestamp_system_us=_optional_positive(depth.get_system_timestamp_us),
            sync_error_ms=sync_error_ms,
            pairing_timestamp_source=pairing_source,
            received_monotonic_ns=time.monotonic_ns(),
        )

    def close(self) -> None:
        pipeline = self._pipeline
        self._running = False
        self._pipeline = None
        self._device = None
        self._context = None
        if pipeline is not None:
            pipeline.stop()

    def __enter__(self) -> "Gemini336LAdapter":
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Gemini 336L native RGB-D adapter smoke test")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--serial")
    parser.add_argument("--backend", choices=("v4l2", "libuvc", "auto"))
    parser.add_argument("--validate-seconds", type=float)
    parser.add_argument("--frames", type=int, default=3)
    args = parser.parse_args()
    adapter = Gemini336LAdapter(args.config, serial_number=args.serial, backend=args.backend)
    try:
        adapter.start(validate_cadence=False)
        report = adapter.validate_cadence(args.validate_seconds)
        samples = [adapter.read() for _ in range(max(args.frames, 0))]
        output = {
            "device": asdict(adapter.device_metadata) if adapter.device_metadata else None,
            "profile": asdict(adapter.profile),
            "cadence": report.as_dict(),
            "calibration": adapter.calibration.as_dict() if adapter.calibration else None,
            "samples": [
                {
                    "sequence": frame.sequence,
                    "color_frame_index": frame.color_frame_index,
                    "depth_frame_index": frame.depth_frame_index,
                    "color_timestamp_hw_us": frame.color_timestamp_hw_us,
                    "depth_timestamp_hw_us": frame.depth_timestamp_hw_us,
                    "sync_error_ms": frame.sync_error_ms,
                    "pairing_timestamp_source": frame.pairing_timestamp_source,
                    "depth_unit_mm": frame.depth_unit_mm,
                }
                for frame in samples
            ],
        }
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 0
    finally:
        adapter.close()


if __name__ == "__main__":
    raise SystemExit(main())
