"""Hand estimator adapter layer for canonical data capture.

Each backend must normalize its output into the canonical hand observation:
21 keypoints in MediaPipe/MANO order, a wrist pose, optional 2D keypoints,
visibility, and optional MANO parameters.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from estimate_wrist import estimate_wrist_pose
from single_hand_detector import SingleHandDetector


@dataclass
class HandObservation:
    keypoints_3d: np.ndarray
    wrist_pose: np.ndarray
    keypoints_2d: np.ndarray | None = None
    visibility: np.ndarray | None = None
    estimator: str = ""
    mano: dict[str, np.ndarray] = field(default_factory=dict)
    raw: Any = None


class HandEstimator:
    name = "base"

    def detect(self, rgb: np.ndarray) -> HandObservation | None:
        raise NotImplementedError


class MediaPipeHandEstimator(HandEstimator):
    name = "mediapipe"

    def __init__(
        self,
        hand_type: str = "Right",
        selfie: bool = False,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
    ):
        self.detector = SingleHandDetector(
            hand_type=hand_type,
            selfie=selfie,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

    def detect(self, rgb: np.ndarray) -> HandObservation | None:
        num, joint_pos, kp2d, wrist_rot = self.detector.detect(rgb)
        if num == 0:
            return None

        kp2d_px = SingleHandDetector.parse_keypoint_2d(kp2d, rgb.shape)
        T = estimate_wrist_pose(
            joint_pos,
            kp2d_px,
            wrist_rot,
            self.detector.operator2mano,
            rgb.shape,
        )
        visibility = np.ones(21, dtype=np.float32)
        return HandObservation(
            keypoints_3d=joint_pos.astype(np.float32),
            keypoints_2d=kp2d_px.astype(np.float32),
            visibility=visibility,
            wrist_pose=T.astype(np.float32),
            estimator=self.name,
            raw=kp2d,
        )


class WiLoRHandEstimator(HandEstimator):
    name = "wilor"

    def __init__(self, *args, **kwargs):
        raise NotImplementedError(
            "WiLoR adapter slot is reserved, but the WiLoR model/runtime is not "
            "installed in this repo yet. The adapter must remap WiLoR/MANO joints "
            "to canonical MediaPipe/MANO 21-keypoint order before writing data."
        )


ESTIMATORS = {
    MediaPipeHandEstimator.name: MediaPipeHandEstimator,
    WiLoRHandEstimator.name: WiLoRHandEstimator,
}


def make_hand_estimator(name: str, **kwargs) -> HandEstimator:
    key = name.lower()
    if key not in ESTIMATORS:
        raise SystemExit(f"unknown hand estimator '{name}', available: {sorted(ESTIMATORS)}")
    return ESTIMATORS[key](**kwargs)
