"""Real-time wrist observation, joint anchoring, and bounded robot mapping.

The browser supplies both MediaPipe image landmarks and world landmarks.  The
world landmarks provide palm orientation and hand shape, while image landmarks
provide the wrist position and apparent palm scale used by the monocular depth
estimate.  Only motion relative to an explicitly captured anchor is mapped to
the robot.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def _rotation_to_rotvec(matrix: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1e-8:
        return np.zeros(3)
    # At pi the usual skew(m) axis formula is numerically zero. Recover the
    # axis from the symmetric part so a palm flip cannot terminate the stream.
    if math.pi - angle < 1e-5:
        axis = np.sqrt(np.maximum((np.diag(matrix) + 1.0) * 0.5, 0.0))
        pivot = int(np.argmax(axis))
        if axis[pivot] > 1e-7:
            if pivot == 0:
                axis[1] = (matrix[0, 1] + matrix[1, 0]) / (4.0 * axis[0])
                axis[2] = (matrix[0, 2] + matrix[2, 0]) / (4.0 * axis[0])
            elif pivot == 1:
                axis[0] = (matrix[0, 1] + matrix[1, 0]) / (4.0 * axis[1])
                axis[2] = (matrix[1, 2] + matrix[2, 1]) / (4.0 * axis[1])
            else:
                axis[0] = (matrix[0, 2] + matrix[2, 0]) / (4.0 * axis[2])
                axis[1] = (matrix[1, 2] + matrix[2, 1]) / (4.0 * axis[2])
            axis = _unit(axis, "rotation axis")
            return axis * angle
    axis = np.array([
        matrix[2, 1] - matrix[1, 2],
        matrix[0, 2] - matrix[2, 0],
        matrix[1, 0] - matrix[0, 1],
    ])
    if float(np.linalg.norm(axis)) < 1e-7:
        # Covers matrices that are numerically between the identity and the
        # exact-pi branch. The principal eigenvector of (R + I) is the stable
        # rotation axis for both cases.
        _, vectors = np.linalg.eigh((matrix + np.eye(3)) * 0.5)
        axis = vectors[:, -1]
    axis = _unit(axis, "rotation axis")
    return axis * angle


def _rotvec_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    """Convert an axis-angle vector to a proper rotation matrix."""
    vector = np.asarray(rotvec, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError("rotvec must contain 3 finite values")
    angle = float(np.linalg.norm(vector))
    if angle < 1e-10:
        return np.eye(3)
    axis = vector / angle
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def _euler_xyz_to_matrix(euler: np.ndarray) -> np.ndarray:
    x, y, z = euler
    cx, cy, cz = np.cos(euler)
    sx, sy, sz = np.sin(euler)
    return np.array([
        [cy * cz, cz * sx * sy - cx * sz, sx * sz + cx * cz * sy],
        [cy * sz, cx * cz + sx * sy * sz, cx * sy * sz - cz * sx],
        [-sy, cy * sx, cx * cy],
    ])


def _matrix_to_euler_xyz(matrix: np.ndarray) -> np.ndarray:
    sy = float(np.clip(-matrix[2, 0], -1.0, 1.0))
    y = math.asin(sy)
    if abs(math.cos(y)) > 1e-7:
        x = math.atan2(matrix[2, 1], matrix[2, 2])
        z = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        x = math.atan2(-matrix[1, 2], matrix[1, 1])
        z = 0.0
    return np.array([x, y, z])


def _matrix_to_continuous_euler_xyz(
    matrix: np.ndarray, reference: np.ndarray
) -> np.ndarray:
    """Return the XYZ Euler representation closest to the previous frame.

    A rotation matrix has multiple equivalent Euler representations.  The
    principal representation switches branches when pitch crosses 90 degrees
    and wraps every axis at +/-pi.  Selecting the equivalent branch nearest to
    the previous frame preserves a continuous control signal before per-axis
    safety limits are applied.
    """
    previous = np.asarray(reference, dtype=np.float64)
    if previous.shape != (3,) or not np.all(np.isfinite(previous)):
        raise ValueError("reference must contain three finite Euler angles")
    principal = _matrix_to_euler_xyz(matrix)
    alternate = np.array([
        principal[0] + math.pi,
        math.pi - principal[1],
        principal[2] + math.pi,
    ])

    def nearest_wrap(candidate: np.ndarray) -> np.ndarray:
        return candidate + 2.0 * math.pi * np.round(
            (previous - candidate) / (2.0 * math.pi)
        )

    candidates = (nearest_wrap(principal), nearest_wrap(alternate))
    return min(candidates, key=lambda value: float(np.linalg.norm(value - previous)))


def _matrix_to_quaternion_xyzw(matrix: np.ndarray) -> np.ndarray:
    # Eigenvector formulation is compact and stable for all rotations.
    k = np.array([
        [matrix[0, 0] - matrix[1, 1] - matrix[2, 2], matrix[1, 0] + matrix[0, 1], matrix[2, 0] + matrix[0, 2], matrix[1, 2] - matrix[2, 1]],
        [matrix[1, 0] + matrix[0, 1], matrix[1, 1] - matrix[0, 0] - matrix[2, 2], matrix[2, 1] + matrix[1, 2], matrix[2, 0] - matrix[0, 2]],
        [matrix[2, 0] + matrix[0, 2], matrix[2, 1] + matrix[1, 2], matrix[2, 2] - matrix[0, 0] - matrix[1, 1], matrix[0, 1] - matrix[1, 0]],
        [matrix[1, 2] - matrix[2, 1], matrix[2, 0] - matrix[0, 2], matrix[0, 1] - matrix[1, 0], matrix[0, 0] + matrix[1, 1] + matrix[2, 2]],
    ]) / 3.0
    _, vectors = np.linalg.eigh(k)
    quaternion = vectors[:, -1]
    if quaternion[3] < 0:
        quaternion = -quaternion
    return quaternion


def _unit(vector: np.ndarray, label: str) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError(f"{label} is degenerate")
    return vector / norm


def _points(value: list[dict] | np.ndarray, label: str) -> np.ndarray:
    if isinstance(value, np.ndarray):
        points = np.asarray(value, dtype=np.float64)
    else:
        try:
            points = np.asarray(
                [[point["x"], point["y"], point["z"]] for point in value],
                dtype=np.float64,
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"{label} contains invalid points") from error
    if points.shape != (21, 3) or not np.all(np.isfinite(points)):
        raise ValueError(f"{label} must contain 21 finite xyz points")
    return points


_OPERATOR2MANO_RIGHT = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)
_OPERATOR2MANO_LEFT = np.array(
    [[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
    dtype=np.float64,
)


def _replay_physical_rotation(world: np.ndarray, handedness: str) -> np.ndarray:
    """Reproduce the offline replay estimator's MediaPipe/MANO wrist frame."""
    centered = world - world[0:1]
    palm = centered[[0, 5, 9], :]
    x_vector = palm[0] - palm[2]
    _, _, vh = np.linalg.svd(palm - np.mean(palm, axis=0, keepdims=True))
    normal = vh[2, :]
    x_axis = _unit(x_vector - np.dot(x_vector, normal) * normal, "replay wrist axis")
    z_axis = np.cross(x_axis, normal)
    z_axis = _unit(z_axis, "replay palm lateral axis")
    if float(np.dot(z_axis, palm[1] - palm[2])) < 0.0:
        normal *= -1.0
        z_axis *= -1.0
    mediapipe_rotation = np.stack([x_axis, normal, z_axis], axis=1)
    operator2mano = (
        _OPERATOR2MANO_LEFT if handedness == "left" else _OPERATOR2MANO_RIGHT
    )
    mano_rotation = mediapipe_rotation @ operator2mano
    physical_x = _unit(mano_rotation[:, 0], "replay palm normal")
    physical_z = mano_rotation[:, 2]
    physical_z = _unit(
        physical_z - physical_x * float(np.dot(physical_x, physical_z)),
        "replay wrist forward axis",
    )
    physical_y = _unit(np.cross(physical_z, physical_x), "replay wrist lateral axis")
    return np.stack([physical_x, physical_y, physical_z], axis=1)


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    matrix = np.sum(rotations, axis=0)
    u, _, vt = np.linalg.svd(matrix)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u @ vt)
    return u @ correction @ vt


def _rotation_distance_deg(left: np.ndarray, right: np.ndarray) -> float:
    return math.degrees(float(np.linalg.norm(_rotation_to_rotvec(left.T @ right))))


@dataclass(frozen=True)
class WristObservation:
    position: np.ndarray
    rotation: np.ndarray
    handedness: str
    handedness_score: float
    position_source: str = "monocular_scale"

    def protocol_pose(self) -> dict:
        quaternion = _matrix_to_quaternion_xyzw(self.rotation)
        return {
            "position": self.position.round(6).tolist(),
            "quaternion": quaternion.round(7).tolist(),
            "quaternion_order": "xyzw",
            "position_source": self.position_source,
            "frame": "camera",
        }


@dataclass(frozen=True)
class VectorFilterResult:
    value: np.ndarray
    reset: bool
    raw_delta: float
    filtered_delta: float


class OneEuroVectorFilter:
    """Adaptive low-pass filter for a finite-dimensional position vector."""

    def __init__(
        self,
        dimension: int = 3,
        *,
        min_cutoff_hz: float = 1.2,
        beta: float = 0.5,
        derivative_cutoff_hz: float = 1.0,
        reset_after_ms: float = 200.0,
    ) -> None:
        if dimension < 1:
            raise ValueError("dimension must be positive")
        if min_cutoff_hz <= 0 or derivative_cutoff_hz <= 0:
            raise ValueError("cutoff frequencies must be positive")
        if beta < 0 or reset_after_ms <= 0:
            raise ValueError("beta must be non-negative and reset interval positive")
        self.dimension = int(dimension)
        self._min_cutoff = float(min_cutoff_hz)
        self._beta = float(beta)
        self._derivative_cutoff = float(derivative_cutoff_hz)
        self._reset_after = float(reset_after_ms) / 1000.0
        self.reset()

    def reset(self) -> None:
        self._timestamp: float | None = None
        self._raw: np.ndarray | None = None
        self._filtered: np.ndarray | None = None
        self._derivative: np.ndarray | None = None

    def update(self, value: np.ndarray | list[float], timestamp: float) -> VectorFilterResult:
        values = np.asarray(value, dtype=np.float64)
        timestamp = float(timestamp)
        if values.shape != (self.dimension,) or not np.all(np.isfinite(values)):
            raise ValueError(f"value must contain {self.dimension} finite values")
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")

        if self._timestamp is None:
            self._timestamp = timestamp
            self._raw = values.copy()
            self._filtered = values.copy()
            self._derivative = np.zeros(self.dimension, dtype=np.float64)
            return VectorFilterResult(values.copy(), True, 0.0, 0.0)

        dt = timestamp - self._timestamp
        if dt <= 0 or dt > self._reset_after:
            self.reset()
            return self.update(values, timestamp)

        assert self._raw is not None
        assert self._filtered is not None
        assert self._derivative is not None
        raw_delta = float(np.linalg.norm(values - self._raw))
        derivative = (values - self._raw) / dt
        derivative_alpha = self._alpha(self._derivative_cutoff, dt)
        derivative_hat = self._lowpass(derivative, self._derivative, derivative_alpha)
        cutoff = self._min_cutoff + self._beta * np.abs(derivative_hat)
        filtered = self._lowpass(values, self._filtered, self._alpha(cutoff, dt))
        filtered_delta = float(np.linalg.norm(filtered - self._filtered))
        self._timestamp = timestamp
        self._raw = values.copy()
        self._filtered = filtered
        self._derivative = derivative_hat
        return VectorFilterResult(filtered.copy(), False, raw_delta, filtered_delta)

    @staticmethod
    def _alpha(cutoff_hz: float | np.ndarray, dt: float) -> float | np.ndarray:
        tau = 1.0 / (2.0 * math.pi * cutoff_hz)
        return 1.0 / (1.0 + tau / dt)

    @staticmethod
    def _lowpass(value: np.ndarray, previous: np.ndarray,
                 alpha: float | np.ndarray) -> np.ndarray:
        return alpha * value + (1.0 - alpha) * previous


@dataclass(frozen=True)
class RotationFilterResult:
    value: np.ndarray
    reset: bool
    raw_delta_rad: float
    filtered_delta_rad: float


class OneEuroRotationFilter:
    """One Euro filter operating on incremental rotations in SO(3)."""

    def __init__(
        self,
        *,
        min_cutoff_hz: float = 1.5,
        beta: float = 0.35,
        derivative_cutoff_hz: float = 1.0,
        reset_after_ms: float = 200.0,
    ) -> None:
        if min_cutoff_hz <= 0 or derivative_cutoff_hz <= 0:
            raise ValueError("cutoff frequencies must be positive")
        if beta < 0 or reset_after_ms <= 0:
            raise ValueError("beta must be non-negative and reset interval positive")
        self._min_cutoff = float(min_cutoff_hz)
        self._beta = float(beta)
        self._derivative_cutoff = float(derivative_cutoff_hz)
        self._reset_after = float(reset_after_ms) / 1000.0
        self.reset()

    def reset(self) -> None:
        self._timestamp: float | None = None
        self._raw: np.ndarray | None = None
        self._filtered: np.ndarray | None = None
        self._derivative: np.ndarray | None = None

    def update(self, rotation: np.ndarray, timestamp: float) -> RotationFilterResult:
        matrix = np.asarray(rotation, dtype=np.float64)
        timestamp = float(timestamp)
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError("rotation must be a finite 3x3 matrix")
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")

        if self._timestamp is None:
            self._timestamp = timestamp
            self._raw = matrix.copy()
            self._filtered = matrix.copy()
            self._derivative = np.zeros(3, dtype=np.float64)
            return RotationFilterResult(matrix.copy(), True, 0.0, 0.0)

        dt = timestamp - self._timestamp
        if dt <= 0 or dt > self._reset_after:
            self.reset()
            return self.update(matrix, timestamp)

        assert self._raw is not None
        assert self._filtered is not None
        assert self._derivative is not None
        raw_step = _rotation_to_rotvec(self._raw.T @ matrix)
        raw_delta = float(np.linalg.norm(raw_step))
        derivative = raw_step / dt
        derivative_alpha = OneEuroVectorFilter._alpha(self._derivative_cutoff, dt)
        derivative_hat = OneEuroVectorFilter._lowpass(
            derivative, self._derivative, derivative_alpha
        )
        cutoff = self._min_cutoff + self._beta * float(np.linalg.norm(derivative_hat))
        alpha = float(OneEuroVectorFilter._alpha(cutoff, dt))
        filter_error = _rotation_to_rotvec(self._filtered.T @ matrix)
        filtered_step = alpha * filter_error
        filtered = self._filtered @ _rotvec_to_matrix(filtered_step)

        self._timestamp = timestamp
        self._raw = matrix.copy()
        self._filtered = filtered
        self._derivative = derivative_hat
        return RotationFilterResult(
            filtered.copy(), False, raw_delta, float(np.linalg.norm(filtered_step))
        )


def estimate_wrist_observation(
    image_landmarks: list[dict] | np.ndarray,
    world_landmarks: list[dict] | np.ndarray,
    handedness: dict | None,
    *,
    focal_length_normalized: float = 1.2,
    palm_width_m: float = 0.085,
    palm_length_m: float = 0.09,
) -> WristObservation:
    """Estimate a camera-frame wrist pose from one MediaPipe hand result.

    The position is intentionally labelled ``monocular_scale``: it is suitable
    for bounded relative control after anchoring, but it is not metric ground
    truth.  A future aligned-depth source can replace this estimator without
    changing the mapper protocol.
    """
    image = _points(image_landmarks, "image_landmarks")
    world = _points(world_landmarks, "world_landmarks")
    label = str((handedness or {}).get("label") or "unknown").lower()
    score = float((handedness or {}).get("score") or 0.0)

    # Keep the same MediaPipe/MANO physical frame used by offline RGB replay.
    # The simpler live-only palm frame was removed after camera A/B testing.
    x_axis = _unit(world[5] - world[17], "palm lateral axis")
    y_axis = world[9] - world[0]
    y_axis = _unit(y_axis - np.dot(y_axis, x_axis) * x_axis, "palm forward axis")
    rotation = _replay_physical_rotation(world, label)

    palm_width = float(np.linalg.norm(image[5, :2] - image[17, :2]))
    palm_length = float(np.linalg.norm(image[0, :2] - image[9, :2]))
    if max(palm_width, palm_length) < 0.025:
        raise ValueError("palm is too small for monocular position estimation")
    # Correct each 2D span by the corresponding 3D palm-axis foreshortening.
    # Without this, turning palm-to-back collapses the lateral span and looks
    # exactly like a large move away from the camera. Near edge-on axes are
    # omitted because dividing by a tiny visibility amplifies landmark noise.
    world_width = float(np.linalg.norm(world[5] - world[17]))
    world_length = float(np.linalg.norm(world[9] - world[0]))
    # MediaPipe world landmarks preserve this hand's proportions and are a
    # better per-frame reference than a fixed average hand size.
    if world_width < 1e-6:
        world_width = float(palm_width_m)
    if world_length < 1e-6:
        world_length = float(palm_length_m)
    scale_candidates = []
    for span, metric, axis in (
        (palm_width, world_width, x_axis),
        (palm_length, world_length, y_axis),
    ):
        visibility = float(np.linalg.norm(axis[:2]))
        if visibility >= 0.15:
            scale_candidates.append(span / (max(float(metric), 1e-6) * visibility))
    if not scale_candidates:
        raise ValueError("palm axes are too foreshortened for position estimation")
    projected_scale = max(scale_candidates)
    depth = float(np.clip(focal_length_normalized / projected_scale, 0.20, 1.20))
    wrist_u, wrist_v = image[0, :2]
    position = np.array([
        (wrist_u - 0.5) * depth / focal_length_normalized,
        (wrist_v - 0.5) * depth / focal_length_normalized,
        depth,
    ])
    return WristObservation(position, rotation, label, score)


@dataclass(frozen=True)
class MappingResult:
    target_pose: np.ndarray
    position_limited: bool
    orientation_limited: bool
    orientation_delta_deg: tuple[float, float, float]
    orientation_limited_axes: tuple[bool, bool, bool]


class LiveWristMapper:
    """Server-authoritative anchor state and relative wrist mapping."""

    def __init__(
        self,
        *,
        anchor_frames: int = 12,
        ready_frames: int = 6,
        missing_limit: int = 3,
        position_limits_m: tuple[float, float, float] = (0.05, 0.05, 0.03),
        orientation_limits_deg: tuple[float, float, float] = (45.0, 25.0, 35.0),
        position_gain: tuple[float, float, float] = (1.0, 1.0, 1.0),
        position_basis: np.ndarray | None = None,
        rotation_basis: np.ndarray | None = None,
        track_orientation: bool = True,
    ) -> None:
        self.anchor_frames = max(3, int(anchor_frames))
        self.ready_frames = max(3, int(ready_frames))
        self.missing_limit = max(1, int(missing_limit))
        self.position_limits = np.asarray(position_limits_m, dtype=np.float64)
        self.set_orientation_limits_deg(
            tuple(-abs(float(value)) for value in orientation_limits_deg),
            tuple(abs(float(value)) for value in orientation_limits_deg),
        )
        self.position_gain = np.asarray(position_gain, dtype=np.float64)
        self.position_basis = np.asarray(
            position_basis if position_basis is not None else np.eye(3), dtype=np.float64
        )
        self.rotation_basis = np.asarray(
            rotation_basis if rotation_basis is not None else np.eye(3), dtype=np.float64
        )
        self.track_orientation = bool(track_orientation)
        self.state = "waiting"
        self.freeze_reason: str | None = None
        self.anchor_revision = 0
        self._recent: deque[WristObservation] = deque(maxlen=self.ready_frames)
        self._anchor_samples: deque[WristObservation] = deque(maxlen=self.anchor_frames)
        self._arm_anchor: np.ndarray | None = None
        self._hand_anchor: WristObservation | None = None
        self._orientation_euler = np.zeros(3, dtype=np.float64)
        self._missing = 0

    def set_orientation_limits_deg(
        self,
        lower: tuple[float, float, float],
        upper: tuple[float, float, float],
    ) -> None:
        lower_values = np.asarray(lower, dtype=np.float64)
        upper_values = np.asarray(upper, dtype=np.float64)
        if (
            lower_values.shape != (3,)
            or upper_values.shape != (3,)
            or not np.all(np.isfinite(lower_values))
            or not np.all(np.isfinite(upper_values))
            or np.any(lower_values >= upper_values)
        ):
            raise ValueError("orientation limits must be three finite lower/upper pairs")
        self.orientation_lower_limits = np.radians(lower_values)
        self.orientation_upper_limits = np.radians(upper_values)

    @property
    def ready_to_anchor(self) -> bool:
        # One valid observation is enough to expose the anchor action.  The
        # actual anchor is still built from a fixed multi-frame capture after
        # the user clicks, so monocular jitter cannot permanently disable UI.
        return bool(self._recent)

    @property
    def anchor_progress(self) -> int:
        return len(self._anchor_samples) if self.state == "anchoring" else 0

    def observe(self, observation: WristObservation) -> MappingResult | None:
        self._missing = 0
        self._recent.append(observation)
        if self.state == "anchoring":
            self._anchor_samples.append(observation)
            samples = list(self._anchor_samples)
            if len(samples) == self.anchor_frames:
                self._hand_anchor = self._aggregate(samples, trim_outliers=True)
                self.state = "following"
                self.freeze_reason = None
                self._orientation_euler[:] = 0.0
                self.anchor_revision += 1
            return None
        if self.state == "following" and self._hand_anchor is not None:
            return self._map(observation)
        return None

    def request_anchor(self, arm_anchor_pose: np.ndarray) -> bool:
        pose = np.asarray(arm_anchor_pose, dtype=np.float64)
        if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
            raise ValueError("arm_anchor_pose must be a finite 4x4 transform")
        self._arm_anchor = pose.copy()
        self._anchor_samples.clear()
        self._hand_anchor = None
        self._orientation_euler[:] = 0.0
        self.state = "anchoring"
        self.freeze_reason = None
        return True

    def mark_missing(self) -> None:
        self._missing += 1
        self._recent.clear()
        if self.state in ("anchoring", "following") and self._missing >= self.missing_limit:
            self.freeze("hand_lost")

    def freeze(self, reason: str = "user") -> None:
        self.state = "frozen"
        self.freeze_reason = reason
        self._anchor_samples.clear()
        self._orientation_euler[:] = 0.0

    def status(self) -> dict:
        samples = (
            list(self._anchor_samples)
            if self.state == "anchoring"
            else list(self._recent)
        )
        return {
            "tracking_state": self.state,
            "ready_to_anchor": self.ready_to_anchor,
            "anchor_progress": self.anchor_progress,
            "anchor_frames": self.anchor_frames,
            "anchor_revision": self.anchor_revision,
            "freeze_reason": self.freeze_reason,
            "orientation_tracking": self.track_orientation,
            "orientation_mode": "replay_world_left",
            "orientation_limits_deg": {
                "lower": np.degrees(self.orientation_lower_limits).round(2).tolist(),
                "upper": np.degrees(self.orientation_upper_limits).round(2).tolist(),
            },
            "stability": self._quality(samples),
        }

    def _map(self, observation: WristObservation) -> MappingResult:
        assert self._hand_anchor is not None and self._arm_anchor is not None
        if observation.handedness != self._hand_anchor.handedness:
            self.freeze("handedness_changed")
            raise ValueError("handedness changed after anchoring")

        raw_delta = observation.position - self._hand_anchor.position
        mapped_delta = self.position_basis @ (raw_delta * self.position_gain)
        limited_delta = np.clip(mapped_delta, -self.position_limits, self.position_limits)

        # Match derive_embodiment's RGB replay path:
        # dR = R_current @ R_anchor.T, then left-compose at the arm base.
        hand_delta = observation.rotation @ self._hand_anchor.rotation.T
        robot_delta = self.rotation_basis @ hand_delta @ self.rotation_basis.T
        euler = _matrix_to_continuous_euler_xyz(
            robot_delta, self._orientation_euler
        )
        self._orientation_euler = euler.copy()
        limited_euler = (
            np.clip(euler, self.orientation_lower_limits, self.orientation_upper_limits)
            if self.track_orientation else np.zeros(3)
        )
        limited_axes = ~np.isclose(euler, limited_euler, atol=1e-9)
        limited_rotation = _euler_xyz_to_matrix(limited_euler)

        target = self._arm_anchor.copy()
        target[:3, 3] += limited_delta
        target[:3, :3] = limited_rotation @ self._arm_anchor[:3, :3]
        return MappingResult(
            target_pose=target,
            position_limited=not np.allclose(mapped_delta, limited_delta, atol=1e-9),
            orientation_limited=bool(np.any(limited_axes)),
            orientation_delta_deg=tuple(float(value) for value in np.degrees(limited_euler)),
            orientation_limited_axes=tuple(bool(value) for value in limited_axes),
        )

    @staticmethod
    def _aggregate(
        samples: list[WristObservation], *, trim_outliers: bool = False
    ) -> WristObservation:
        selected = samples
        if trim_outliers and len(samples) >= 4:
            initial_position = np.median(
                np.stack([sample.position for sample in samples]), axis=0
            )
            initial_rotation = _mean_rotation([sample.rotation for sample in samples])
            scores = []
            for index, sample in enumerate(samples):
                position_error = float(np.linalg.norm(sample.position - initial_position))
                rotation_error = _rotation_distance_deg(initial_rotation, sample.rotation)
                scores.append((position_error / 0.02 + rotation_error / 10.0, index))
            keep_count = max(3, int(math.ceil(len(samples) * 0.75)))
            selected = [samples[index] for _, index in sorted(scores)[:keep_count]]

        labels = [sample.handedness for sample in selected]
        handedness = max(set(labels), key=labels.count)
        score = float(np.median([
            sample.handedness_score
            for sample in selected
            if sample.handedness == handedness
        ]))
        position = np.median(
            np.stack([sample.position for sample in selected]), axis=0
        )
        rotation = _mean_rotation([sample.rotation for sample in selected])
        return WristObservation(position, rotation, handedness, score)

    @staticmethod
    def _quality(samples: list[WristObservation]) -> dict:
        if not samples:
            return {
                "sample_count": 0,
                "position_error_m": None,
                "orientation_error_deg": None,
                "handedness_consistent": True,
                "stable": False,
            }
        center = LiveWristMapper._aggregate(samples)
        position_error = max(
            float(np.linalg.norm(sample.position - center.position)) for sample in samples
        )
        rotation_error = max(
            _rotation_distance_deg(center.rotation, sample.rotation) for sample in samples
        )
        handedness_consistent = len({sample.handedness for sample in samples}) == 1
        return {
            "sample_count": len(samples),
            "position_error_m": round(position_error, 5),
            "orientation_error_deg": round(rotation_error, 2),
            "handedness_consistent": handedness_consistent,
            "stable": bool(
                handedness_consistent
                and position_error <= 0.02
                and rotation_error <= 10.0
            ),
        }

    @staticmethod
    def _stable(samples: list[WristObservation]) -> bool:
        if len(samples) < 3 or len({sample.handedness for sample in samples}) != 1:
            return False
        center = LiveWristMapper._aggregate(samples)
        position_error = max(
            float(np.linalg.norm(sample.position - center.position)) for sample in samples
        )
        rotation_error = max(
            _rotation_distance_deg(center.rotation, sample.rotation) for sample in samples
        )
        return position_error <= 0.02 and rotation_error <= 10.0
