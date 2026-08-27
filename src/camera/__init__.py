"""相机、标定和坐标变换工具。

本目录只负责传感器/标定层，不负责机器人运动控制或 EGO 重定向。
"""

from importlib import import_module

from .handeye import (
    HandEyeResult,
    make_transform,
    solve_eye_in_hand,
    transform_to_dict,
)

_ORBBEC_EXPORTS = {
    "CadenceReport",
    "CadenceValidationError",
    "CalibrationSnapshot",
    "CameraConfigurationError",
    "CameraStreamError",
    "Gemini336LAdapter",
    "RGBDFrame",
    "load_gemini336l_profile",
}


def __getattr__(name):
    if name in _ORBBEC_EXPORTS:
        return getattr(import_module(".orbbec_gemini336l", __name__), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = [
    "HandEyeResult",
    "make_transform",
    "solve_eye_in_hand",
    "transform_to_dict",
    *_ORBBEC_EXPORTS,
]
