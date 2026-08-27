"""Versioned acquisition and acceptance quality profiles.

Profiles are repository configuration. A full snapshot is copied into each
Capture so later acceptance runs do not silently inherit changed thresholds.
"""
from __future__ import annotations

import json
import os
import re
from pathlib import Path
from typing import Any


REPO = Path(__file__).resolve().parents[1]
PROFILE_ROOT = REPO / "configs/quality_profiles"
QUALITY_PROFILE_SCHEMA_VERSION = "1.1"
SUPPORTED_QUALITY_PROFILE_SCHEMA_VERSIONS = frozenset({"1.0", "1.1"})
PROFILE_ID_RE = re.compile(r"^[A-Za-z0-9_.-]+$")
OPERATORS = frozenset({"lt", "lte", "gt", "gte", "eq"})
SYNC_MODES = frozenset({
    "single_stream_video",
    "filename_paired_no_hardware_clock",
    "external_undeclared",
    "hardware_timestamp_paired",
})


def validate_quality_profile(profile: dict[str, Any]) -> dict[str, Any]:
    schema_version = profile.get("schema_version")
    if schema_version not in SUPPORTED_QUALITY_PROFILE_SCHEMA_VERSIONS:
        raise ValueError(
            f"unsupported quality profile schema {profile.get('schema_version')!r}; "
            f"supported: {sorted(SUPPORTED_QUALITY_PROFILE_SCHEMA_VERSIONS)}"
        )
    profile_id = profile.get("profile_id")
    if not isinstance(profile_id, str) or not PROFILE_ID_RE.fullmatch(profile_id):
        raise ValueError(f"invalid quality profile_id: {profile_id!r}")
    if not isinstance(profile.get("revision"), int) or profile["revision"] < 1:
        raise ValueError("quality profile revision must be a positive integer")
    source_kinds = profile.get("source_kinds")
    if not isinstance(source_kinds, list) or not source_kinds or not all(
        isinstance(value, str) and value for value in source_kinds
    ):
        raise ValueError("quality profile source_kinds must be a non-empty string list")
    acquisition = profile.get("acquisition")
    if not isinstance(acquisition, dict):
        raise ValueError("quality profile acquisition must be an object")
    if not isinstance(acquisition.get("device_class"), str) or not acquisition["device_class"]:
        raise ValueError("quality profile acquisition.device_class must be a non-empty string")
    if acquisition.get("sync_mode") not in SYNC_MODES:
        raise ValueError("quality profile acquisition.sync_mode is unsupported")
    for stream in ("rgb", "depth"):
        spec = acquisition.get(stream)
        if not isinstance(spec, dict) or not isinstance(spec.get("required"), bool):
            raise ValueError(f"quality profile acquisition.{stream}.required must be boolean")
        for key in ("min_fps", "min_measured_fps", "min_width", "min_height"):
            if key in spec and (
                isinstance(spec[key], bool)
                or not isinstance(spec[key], (int, float))
                or spec[key] <= 0
            ):
                raise ValueError(f"quality profile acquisition.{stream}.{key} must be positive")
        if (
            "min_measured_fps" in spec
            and "min_fps" not in spec
        ):
            raise ValueError(
                f"quality profile acquisition.{stream}.min_measured_fps requires min_fps"
            )
        if (
            "min_measured_fps" in spec
            and float(spec["min_measured_fps"]) > float(spec["min_fps"])
        ):
            raise ValueError(
                f"quality profile acquisition.{stream}.min_measured_fps cannot exceed min_fps"
            )
    timestamps = acquisition.get("hardware_timestamps")
    sync = acquisition.get("rgb_depth_sync")
    if not isinstance(timestamps, dict) or not isinstance(timestamps.get("required"), bool):
        raise ValueError("quality profile hardware_timestamps.required must be boolean")
    if not isinstance(sync, dict) or not isinstance(sync.get("required"), bool):
        raise ValueError("quality profile rgb_depth_sync.required must be boolean")
    if sync.get("required") and (
        isinstance(sync.get("max_error_ms"), bool)
        or not isinstance(sync.get("max_error_ms"), (int, float))
        or sync["max_error_ms"] <= 0
    ):
        raise ValueError("required RGB-D sync must define a positive max_error_ms")
    metrics = profile.get("metrics")
    if not isinstance(metrics, dict) or not metrics:
        raise ValueError("quality profile metrics must be a non-empty object")
    for key, spec in metrics.items():
        if not isinstance(spec, dict) or spec.get("operator") not in OPERATORS:
            raise ValueError(f"quality metric {key!r} has an invalid operator")
        if isinstance(spec.get("value"), bool) or not isinstance(spec.get("value"), (int, float)):
            raise ValueError(f"quality metric {key!r} must define a numeric value")
        if not isinstance(spec.get("unit"), str) or not isinstance(spec.get("label"), str):
            raise ValueError(f"quality metric {key!r} must define unit and label")
        if schema_version == "1.1":
            if not isinstance(spec.get("measurement_class"), str) or not spec["measurement_class"]:
                raise ValueError(
                    f"quality metric {key!r} must define measurement_class in schema 1.1"
                )
            if not isinstance(spec.get("ground_truth_required"), bool):
                raise ValueError(
                    f"quality metric {key!r} must define ground_truth_required in schema 1.1"
                )
    return profile


def load_quality_profile(value: str | Path) -> dict[str, Any]:
    raw = Path(value).expanduser()
    if raw.is_file():
        path = raw.resolve()
    else:
        identifier = str(value)
        if not PROFILE_ID_RE.fullmatch(identifier):
            raise ValueError(f"invalid quality profile name or missing path: {value}")
        path = PROFILE_ROOT / f"{identifier}.json"
    if not path.is_file():
        raise FileNotFoundError(f"quality profile does not exist: {path}")
    try:
        profile = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid quality profile JSON: {path}") from error
    if not isinstance(profile, dict):
        raise ValueError(f"quality profile root must be an object: {path}")
    return validate_quality_profile(profile)


def read_quality_profile_snapshot(source_root: str | Path) -> dict[str, Any] | None:
    path = Path(source_root).expanduser().resolve() / "quality_profile.json"
    if not path.is_file():
        return None
    try:
        profile = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid quality profile snapshot: {path}") from error
    if not isinstance(profile, dict):
        raise ValueError(f"quality profile snapshot root must be an object: {path}")
    return validate_quality_profile(profile)


def write_quality_profile_snapshot(
    source_root: str | Path,
    profile: dict[str, Any],
) -> Path:
    profile = validate_quality_profile(profile)
    destination = Path(source_root).expanduser().resolve() / "quality_profile.json"
    if destination.is_file():
        existing = read_quality_profile_snapshot(destination.parent)
        if existing != profile:
            raise ValueError(
                f"Capture quality profile snapshot is immutable and already differs: {destination}"
            )
        return destination
    destination.parent.mkdir(parents=True, exist_ok=True)
    tmp = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
    tmp.write_text(json.dumps(profile, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(destination)
    return destination


def metric_spec(profile: dict[str, Any], key: str) -> dict[str, Any]:
    try:
        return profile["metrics"][key]
    except KeyError as error:
        raise ValueError(
            f"quality profile {profile.get('profile_id')!r} has no metric {key!r}"
        ) from error


def threshold_text(spec: dict[str, Any]) -> str:
    symbols = {"lt": "<", "lte": "<=", "gt": ">", "gte": ">=", "eq": "="}
    return f"{symbols[spec['operator']]}{spec['value']:g}"


def evaluate_threshold(value: float | int, spec: dict[str, Any]) -> bool:
    expected = spec["value"]
    return {
        "lt": value < expected,
        "lte": value <= expected,
        "gt": value > expected,
        "gte": value >= expected,
        "eq": value == expected,
    }[spec["operator"]]
