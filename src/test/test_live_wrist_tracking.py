from __future__ import annotations

import math

import numpy as np

from live_wrist_tracking import (
    LiveWristMapper,
    OneEuroRotationFilter,
    OneEuroVectorFilter,
    WristObservation,
    _euler_xyz_to_matrix,
    _matrix_to_continuous_euler_xyz,
    _matrix_to_euler_xyz,
    _rotation_to_rotvec,
    _rotvec_to_matrix,
    estimate_wrist_observation,
    _replay_physical_rotation,
)


def _landmarks(wrist_x: float = 0.5, palm_scale: float = 0.12):
    image = np.zeros((21, 3), dtype=float)
    image[:, :2] = [wrist_x, 0.5]
    image[0, :2] = [wrist_x, 0.55]
    image[5, :2] = [wrist_x - palm_scale / 2, 0.45]
    image[9, :2] = [wrist_x, 0.42]
    image[17, :2] = [wrist_x + palm_scale / 2, 0.45]

    world = np.zeros((21, 3), dtype=float)
    world[0] = [0.0, 0.0, 0.0]
    world[5] = [0.04, 0.06, 0.0]
    world[9] = [0.0, 0.08, 0.0]
    world[17] = [-0.04, 0.06, 0.0]
    return image, world


def _observation(position=(0.0, 0.0, 0.5), euler=(0.0, 0.0, 0.0)):
    return WristObservation(
        np.asarray(position, dtype=float),
        _euler_xyz_to_matrix(np.asarray(euler, dtype=float)),
        "right",
        0.99,
    )


def test_one_euro_position_filter_reduces_wrist_jitter():
    filt = OneEuroVectorFilter(min_cutoff_hz=1.2, beta=0.5)
    base = np.array([0.0, 0.0, 0.5])
    outputs = []
    raw_steps = []
    for index in range(30):
        sample = base + np.array([0.01 * (1 if index % 2 else -1), 0.0, 0.0])
        result = filt.update(sample, index / 30.0)
        outputs.append(result.value.copy())
        raw_steps.append(result.raw_delta)

    filtered_steps = [
        float(np.linalg.norm(right - left))
        for left, right in zip(outputs, outputs[1:])
    ]
    assert max(filtered_steps[5:]) < max(raw_steps[5:])
    assert np.max(np.abs(outputs[-1] - base)) < 0.01


def test_one_euro_position_filter_resets_after_tracking_gap():
    filt = OneEuroVectorFilter()
    filt.update([0.0, 0.0, 0.5], 0.0)
    result = filt.update([0.2, 0.0, 0.5], 0.3)
    assert result.reset is True
    assert np.allclose(result.value, [0.2, 0.0, 0.5])


def test_rotation_vector_round_trip():
    rotation = _euler_xyz_to_matrix(np.array([0.4, -0.3, 0.2]))
    assert np.allclose(_rotvec_to_matrix(_rotation_to_rotvec(rotation)), rotation, atol=1e-8)


def test_rotation_vector_handles_exact_half_turn():
    rotation = _euler_xyz_to_matrix(np.array([0.0, math.pi, 0.0]))
    vector = _rotation_to_rotvec(rotation)
    assert np.isclose(np.linalg.norm(vector), math.pi, atol=1e-6)
    assert np.allclose(_rotvec_to_matrix(vector), rotation, atol=1e-6)


def test_continuous_euler_keeps_pitch_branch_across_ninety_degrees():
    previous = np.radians([0.0, 80.0, 0.0])
    current = _matrix_to_continuous_euler_xyz(
        _euler_xyz_to_matrix(np.radians([0.0, 100.0, 0.0])), previous
    )

    assert np.allclose(np.degrees(current), [0.0, 100.0, 0.0], atol=1e-6)


def test_continuous_euler_unwraps_positive_half_turn():
    previous = np.radians([0.0, 0.0, 179.0])
    current = _matrix_to_continuous_euler_xyz(
        _euler_xyz_to_matrix(np.radians([0.0, 0.0, 181.0])), previous
    )

    assert np.allclose(np.degrees(current), [0.0, 0.0, 181.0], atol=1e-6)


def test_one_euro_rotation_filter_reduces_orientation_jitter():
    filt = OneEuroRotationFilter(min_cutoff_hz=1.5, beta=0.35)
    outputs = []
    raw_steps = []
    for index in range(30):
        angle = math.radians(3.0 if index % 2 else -3.0)
        result = filt.update(_euler_xyz_to_matrix(np.array([0.0, 0.0, angle])), index / 30.0)
        outputs.append(result.value)
        raw_steps.append(result.raw_delta_rad)

    filtered_steps = [
        np.linalg.norm(_rotation_to_rotvec(left.T @ right))
        for left, right in zip(outputs, outputs[1:])
    ]
    assert max(filtered_steps[5:]) < max(raw_steps[5:])


def test_one_euro_rotation_filter_resets_after_tracking_gap():
    filt = OneEuroRotationFilter()
    filt.update(np.eye(3), 0.0)
    rotation = _euler_xyz_to_matrix(np.array([0.2, 0.0, 0.0]))
    result = filt.update(rotation, 0.3)
    assert result.reset is True
    assert np.allclose(result.value, rotation)


def test_monocular_position_uses_image_location_and_scale():
    center_image, world = _landmarks(0.5, 0.12)
    right_image, _ = _landmarks(0.65, 0.12)
    small_image, _ = _landmarks(0.5, 0.06)
    small_image[0, 1] = 0.525
    small_image[9, 1] = 0.46

    center = estimate_wrist_observation(center_image, world, {"label": "Right", "score": 0.9})
    right = estimate_wrist_observation(right_image, world, {"label": "Right", "score": 0.9})
    farther = estimate_wrist_observation(small_image, world, {"label": "Right", "score": 0.9})

    assert center.position_source == "monocular_scale"
    assert right.position[0] > center.position[0]
    assert farther.position[2] > center.position[2]
    assert np.allclose(center.rotation.T @ center.rotation, np.eye(3), atol=1e-8)


def test_monocular_depth_does_not_jump_when_palm_width_foreshortens():
    front_image, front_world = _landmarks(0.5, 0.12)
    turned_image, turned_world = _landmarks(0.5, 0.12)
    angle = math.acos(0.035 / 0.12)
    for point in turned_image:
        point[0] = 0.5 + (point[0] - 0.5) * math.cos(angle)
    for point in turned_world:
        point[0], point[2] = (
            math.cos(angle) * point[0] + math.sin(angle) * point[2],
            -math.sin(angle) * point[0] + math.cos(angle) * point[2],
        )

    front = estimate_wrist_observation(front_image, front_world, {"label": "Right"})
    turned = estimate_wrist_observation(turned_image, turned_world, {"label": "Right"})

    assert abs(turned.position[2] - front.position[2]) < 0.01


def test_anchor_uses_multiple_frames_and_starts_without_jump():
    mapper = LiveWristMapper(anchor_frames=5, ready_frames=3)
    base = _observation()
    mapper.observe(base)
    assert mapper.ready_to_anchor
    assert mapper.request_anchor(np.eye(4))
    assert mapper.state == "anchoring"

    for _ in range(5):
        mapper.observe(base)
    assert mapper.state == "following"
    result = mapper.observe(base)
    assert result is not None
    assert np.allclose(result.target_pose, np.eye(4), atol=1e-8)
    assert mapper.anchor_revision == 1


def test_anchor_capture_completes_despite_normal_monocular_jitter():
    mapper = LiveWristMapper(anchor_frames=5, ready_frames=3)
    mapper.observe(_observation())
    mapper.request_anchor(np.eye(4))
    samples = [
        _observation(position=(0.0, 0.0, 0.50)),
        _observation(position=(0.01, -0.01, 0.52), euler=(0.02, 0.01, 0.0)),
        _observation(position=(-0.01, 0.01, 0.48), euler=(-0.02, 0.0, 0.01)),
        _observation(position=(0.08, 0.0, 0.60), euler=(0.30, 0.0, 0.0)),
        _observation(position=(0.0, 0.0, 0.51)),
    ]
    for sample in samples:
        mapper.observe(sample)

    assert mapper.state == "following"
    assert mapper.anchor_revision == 1
    assert mapper.status()["stability"]["sample_count"] >= 1


def test_one_valid_frame_enables_anchor_and_status_reports_quality():
    mapper = LiveWristMapper(anchor_frames=4, ready_frames=6)
    assert not mapper.ready_to_anchor
    mapper.observe(_observation())
    status = mapper.status()
    assert status["ready_to_anchor"]
    assert status["stability"]["sample_count"] == 1
    assert status["stability"]["position_error_m"] == 0.0


def test_relative_position_and_orientation_are_bounded():
    mapper = LiveWristMapper(anchor_frames=3, ready_frames=3)
    base = _observation()
    for _ in range(3):
        mapper.observe(base)
    mapper.request_anchor(np.eye(4))
    for _ in range(3):
        mapper.observe(base)

    moved = _observation(position=(0.2, -0.2, 0.7), euler=(1.2, -1.0, 1.1))
    result = mapper.observe(moved)
    assert result is not None
    assert result.position_limited and result.orientation_limited
    assert np.allclose(result.target_pose[:3, 3], [0.05, -0.05, 0.03], atol=1e-8)
    angles = _matrix_to_euler_xyz(result.target_pose[:3, :3])
    assert np.allclose(np.degrees(angles), [45.0, -25.0, 35.0], atol=1e-6)
    assert result.orientation_limited_axes == (True, True, True)


def test_asymmetric_orientation_limits_match_ready_pose_margin():
    base = _observation()
    mappers = []
    for _ in range(2):
        mapper = LiveWristMapper(anchor_frames=3, ready_frames=3)
        mapper.set_orientation_limits_deg(
            (-75.0, -60.0, -50.0), (75.0, 60.0, 100.0)
        )
        mapper.request_anchor(np.eye(4))
        for _ in range(3):
            mapper.observe(base)
        mappers.append(mapper)

    positive = mappers[0].observe(
        _observation(euler=(0.0, 0.0, math.radians(120.0)))
    )
    negative = mappers[1].observe(
        _observation(euler=(0.0, 0.0, math.radians(-120.0)))
    )

    assert positive is not None and negative is not None
    assert np.isclose(positive.orientation_delta_deg[2], 100.0)
    assert np.isclose(negative.orientation_delta_deg[2], -50.0)
    assert positive.orientation_limited_axes == (False, False, True)


def test_replay_mapping_stays_continuous_through_pitch_branch_change():
    mapper = LiveWristMapper(anchor_frames=3, ready_frames=3)
    mapper.set_orientation_limits_deg((-170.0, -170.0, -170.0),
                                      (170.0, 170.0, 170.0))
    base = _observation()
    mapper.request_anchor(np.eye(4))
    for _ in range(3):
        mapper.observe(base)

    before = mapper.observe(_observation(euler=(0.0, math.radians(80.0), 0.0)))
    after = mapper.observe(_observation(euler=(0.0, math.radians(100.0), 0.0)))

    assert before is not None and after is not None
    assert mapper.status()["orientation_mode"] == "replay_world_left"
    assert np.allclose(before.orientation_delta_deg, [0.0, 80.0, 0.0], atol=1e-6)
    assert np.allclose(after.orientation_delta_deg, [0.0, 100.0, 0.0], atol=1e-6)
    assert not after.orientation_limited


def test_replay_orientation_frame_is_orthonormal_and_right_handed():
    _, world = _landmarks()
    rotation = _replay_physical_rotation(world, "right")
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
    assert np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8)


def test_position_only_mapping_keeps_anchor_orientation():
    mapper = LiveWristMapper(anchor_frames=3, ready_frames=3, track_orientation=False)
    base = _observation(euler=(0.2, -0.1, 0.3))
    arm_anchor = np.eye(4)
    arm_anchor[:3, :3] = _euler_xyz_to_matrix(np.array([0.4, 0.1, -0.2]))
    mapper.observe(base)
    mapper.request_anchor(arm_anchor)
    for _ in range(3):
        mapper.observe(base)

    moved = _observation(position=(0.02, -0.01, 0.51), euler=(1.0, -0.7, 0.8))
    result = mapper.observe(moved)

    assert result is not None
    assert np.allclose(result.target_pose[:3, :3], arm_anchor[:3, :3], atol=1e-8)
    assert not np.allclose(result.target_pose[:3, 3], arm_anchor[:3, 3], atol=1e-8)
    assert mapper.status()["orientation_tracking"] is False


def test_missing_hand_freezes_following():
    mapper = LiveWristMapper(anchor_frames=3, ready_frames=3, missing_limit=2)
    base = _observation()
    for _ in range(3):
        mapper.observe(base)
    mapper.request_anchor(np.eye(4))
    for _ in range(3):
        mapper.observe(base)
    mapper.mark_missing()
    assert mapper.state == "following"
    mapper.mark_missing()
    assert mapper.state == "frozen"
    assert mapper.freeze_reason == "hand_lost"
