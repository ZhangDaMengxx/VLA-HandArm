#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import hand_feasibility as hf  # noqa: E402


SPEC_PATH = Path(__file__).resolve().parents[2] / "configs/hands/inspire_rh56dfx_right.json"


def _rig(tmp_path, **adapter_kwargs):
    spec = hf.HandModelSpec.load(SPEC_PATH)
    policy = replace(
        spec.policy,
        sample_interval_s=0.0,
        min_settle_s=0.0,
        settle_timeout_s=0.1,
        stable_samples=1,
        preflight_samples=2,
        single_step_u=0.25,
        interaction_step_u=0.1,
        boundary_resolution_u=0.01,
    )
    adapter = hf.MockHandAdapter(spec, **adapter_kwargs)
    adapter.connect(read_only=False)
    recorder = hf.ProfileRecorder(tmp_path / "profile.json", spec, adapter, policy)
    probe = hf.FeasibilityProbe(spec, adapter, recorder, policy=policy, sleep_fn=lambda _: None)
    return spec, policy, adapter, recorder, probe


def test_spec_uses_urdf_nominal_limits_and_raw_endpoints():
    spec = hf.HandModelSpec.load(SPEC_PATH)
    assert spec.asset_sha256 == hf._sha256(spec.asset_urdf)
    assert spec.angle_semantics == "asset_nominal_rad"
    assert spec.joint("right_thumb_2_joint").model_upper_rad == 0.48
    assert spec.joint("right_thumb_2_joint").nominal_source == "vendor_urdf"
    assert spec.joint("right_index_1_joint").model_upper_rad == 1.333
    assert spec.raw_from_targets({}) == [1000] * 6
    assert spec.raw_from_targets({name: 1.0 for name in spec.joint_names}) == [0] * 6


def test_spec_can_take_nominal_angle_range_from_datasheet(tmp_path):
    raw = json.loads(SPEC_PATH.read_text(encoding="utf-8"))
    raw["asset"]["urdf"] = str(
        Path(__file__).resolve().parents[2] / "assets/hand/urdf/inspire_hand_right.urdf")
    raw["joints"][1]["nominal_source"] = "vendor_datasheet_v1.09"
    raw["joints"][1]["nominal_range_rad"] = [0.0, 0.6]
    path = tmp_path / "datasheet_spec.json"
    path.write_text(json.dumps(raw), encoding="utf-8")
    spec = hf.HandModelSpec.load(path)
    thumb_pitch = spec.joint("right_thumb_2_joint")
    assert thumb_pitch.model_upper_rad == 0.6
    assert thumb_pitch.nominal_source == "vendor_datasheet_v1.09"


def test_preflight_records_status_but_does_not_use_it_as_contact_signal(tmp_path):
    _, _, adapter, recorder, probe = _rig(tmp_path)
    result = probe.preflight()
    assert result["status"] == "pass"
    assert result["status_register"] == [2] * 6
    assert "不用于" in result["note"]
    assert recorder.data["results"]["preflight"]["status"] == "pass"
    adapter.close()


def test_single_joint_scan_reaches_asset_endpoint_in_mock(tmp_path):
    spec, _, adapter, _, probe = _rig(tmp_path)
    probe.preflight()
    result = probe.run_single(["right_index_1_joint"])["right_index_1_joint"]
    assert result["evaluation_status"] == "complete"
    assert result["safe_max_u"] == 1.0
    assert result["safe_max_raw"] == 0
    assert result["safe_max_nominal_rad"] == spec.joint(
        "right_index_1_joint").model_upper_rad
    adapter.close()


def test_slow_motion_is_not_misclassified_as_tracking_failure(tmp_path):
    spec = hf.HandModelSpec.load(SPEC_PATH)
    policy = replace(
        spec.policy,
        sample_interval_s=0.1,
        min_settle_s=0.2,
        settle_timeout_s=0.3,
        full_stroke_timeout_s=20.0,
        stable_samples=2,
        stable_delta_raw=5,
        tracking_tolerance_raw=3,
    )

    class SlowAdapter(hf.MockHandAdapter):
        def command_raw(self, project_raw):
            self.target = list(project_raw)

        def observe(self):
            for i, target in enumerate(self.target):
                delta = target - self.raw[i]
                self.raw[i] += max(-5, min(5, delta))
            return super().observe()

    now = [0.0]
    adapter = SlowAdapter(spec)
    adapter.connect(read_only=False)
    recorder = hf.ProfileRecorder(tmp_path / "slow.json", spec, adapter, policy)
    probe = hf.FeasibilityProbe(
        spec, adapter, recorder, policy=policy,
        sleep_fn=lambda seconds: now.__setitem__(0, now[0] + seconds),
        monotonic_fn=lambda: now[0],
    )
    result = probe._command_targets(
        {"right_index_1_joint": 0.03},
        active_names=["right_index_1_joint"], label="slow")
    assert result["verdict"] == "feasible"
    assert result["tracking_error_raw"] <= policy.tracking_tolerance_raw
    assert result["elapsed_s"] > policy.min_settle_s
    adapter.close()


def test_tracking_timeout_without_contact_is_inconclusive(tmp_path):
    spec = hf.HandModelSpec.load(SPEC_PATH)
    policy = replace(
        spec.policy,
        sample_interval_s=0.1,
        min_settle_s=0.1,
        settle_timeout_s=0.2,
        full_stroke_timeout_s=0.0,
        stable_samples=1,
        tracking_tolerance_raw=3,
    )

    class StuckAdapter(hf.MockHandAdapter):
        def command_raw(self, project_raw):
            self.target = list(project_raw)

    now = [0.0]
    adapter = StuckAdapter(spec)
    adapter.connect(read_only=False)
    recorder = hf.ProfileRecorder(tmp_path / "stuck.json", spec, adapter, policy)
    probe = hf.FeasibilityProbe(
        spec, adapter, recorder, policy=policy,
        sleep_fn=lambda seconds: now.__setitem__(0, now[0] + seconds),
        monotonic_fn=lambda: now[0],
    )
    result = probe._command_targets(
        {"right_index_1_joint": 0.1},
        active_names=["right_index_1_joint"], label="stuck")
    assert result["verdict"] == "inconclusive"
    assert result["reasons"] == ["tracking_timeout_raw=100"]
    adapter.close()


def test_interaction_scan_finds_and_refines_mock_collision_boundary(tmp_path):
    _, _, adapter, recorder, probe = _rig(tmp_path)
    probe.preflight()
    summary = probe.run_interactions(["thumb_index_diagonal"])["thumb_index_diagonal"]
    by_level = {row["condition_level_u"]: row for row in summary["boundaries"]}
    assert by_level[0.4]["safe_max_u"] == 1.0
    assert 0.75 <= by_level[0.7]["safe_max_u"] <= 0.76
    assert 0.76 <= by_level[0.7]["first_infeasible_u"] <= 0.77
    points = recorder.data["results"]["interactions"]["thumb_index_diagonal"]["points"]
    assert any("steady_force_abs" in " ".join(p.get("reasons", [])) for p in points)
    assert any("stall_error" in " ".join(p.get("reasons", [])) for p in points)
    adapter.close()


def test_temperature_threshold_causes_hard_abort_and_persists_report(tmp_path):
    _, _, adapter, recorder, probe = _rig(tmp_path, temperature_c=55)
    with pytest.raises(hf.SafetyAbort, match="温度"):
        probe.preflight()
    recorder.abort("temperature")
    saved = json.loads((tmp_path / "profile.json").read_text(encoding="utf-8"))
    assert saved["status"] == "aborted"
    adapter.close()


def test_mock_soft_freeze_holds_actual_position_not_blocked_target(tmp_path):
    spec, _, adapter, recorder, _ = _rig(tmp_path)
    target = spec.raw_from_targets({
        "right_thumb_1_joint": 1.0,
        "right_thumb_2_joint": 1.0,
        "right_index_1_joint": 1.0,
    })
    adapter.command_raw(target)
    actual = adapter.observe().angle_raw
    assert actual is not None and actual != target
    assert hf._freeze_after_failure(adapter, recorder) is True
    assert adapter.target == actual
    assert recorder.data["events"][-1] == {
        "time": recorder.data["events"][-1]["time"],
        "kind": "soft_freeze",
        "ok": True,
    }
    adapter.close()


def test_short_telemetry_gap_is_tolerated_but_continuous_gap_aborts(tmp_path):
    _, _, adapter, _, probe = _rig(tmp_path / "short", missing_observations=1)
    assert probe.preflight()["status"] == "pass"
    adapter.close()

    _, policy, adapter, _, probe = _rig(
        tmp_path / "long", missing_observations=policy_missing_count())
    with pytest.raises(hf.SafetyAbort, match="没有取得"):
        probe.preflight()
    adapter.close()


def policy_missing_count() -> int:
    return 100


def test_resume_rejects_asset_or_condition_drift(tmp_path):
    spec, policy, adapter, recorder, _ = _rig(tmp_path)
    recorder.complete()
    resumed = hf.ProfileRecorder(tmp_path / "profile.json", spec, adapter, policy, resume=True)
    assert resumed.data["status"] == "building"
    with pytest.raises(hf.ProbeError, match="完整探测条件"):
        hf.ProfileRecorder(
            tmp_path / "profile.json", spec, adapter,
            replace(policy, speed=policy.speed + 1), resume=True)
    other = hf.MockHandAdapter(spec)
    other.identity = lambda: {"adapter": "other"}
    with pytest.raises(hf.ProbeError, match="设备/Adapter"):
        hf.ProfileRecorder(tmp_path / "profile.json", spec, other, policy, resume=True)
    adapter.close()


def test_cli_hardware_and_motion_gates_fail_before_connecting(tmp_path):
    out = tmp_path / "never.json"
    assert hf.main([
        "--adapter", "inspire", "--phase", "preflight", "--output", str(out),
    ]) == 2
    assert hf.main([
        "--adapter", "inspire", "--hardware", "--phase", "single",
        "--output", str(out),
    ]) == 2
    assert not out.exists()


def test_hardware_stage_order_is_rejected_before_serial_connect(tmp_path, monkeypatch):
    connects = []
    monkeypatch.setattr(hf.InspireRH56Adapter, "connect",
                        lambda self, **kwargs: connects.append(kwargs))
    token = hf.MOTION_CONFIRMATION
    assert hf.main([
        "--adapter", "inspire", "--hardware", "--phase", "all",
        "--allow-motion", token, "--output", str(tmp_path / "all.json"),
    ]) == 2
    assert hf.main([
        "--adapter", "inspire", "--hardware", "--phase", "interactions",
        "--allow-motion", token, "--output", str(tmp_path / "interaction.json"),
    ]) == 2
    assert connects == []


def test_inspire_connect_read_only_skips_runtime_writes(monkeypatch):
    import inspire_hand

    class FakeSerial:
        is_open = True

        def close(self):
            self.is_open = False

    monkeypatch.setitem(sys.modules, "serial", type("SerialModule", (), {
        "Serial": staticmethod(lambda *args, **kwargs: FakeSerial()),
    }))
    monkeypatch.setattr(inspire_hand.time, "sleep", lambda _: None)
    monkeypatch.setattr(inspire_hand.InspireHand, "read_regs",
                        lambda self, *args, **kwargs: (1,))
    writes = []
    monkeypatch.setattr(inspire_hand.InspireHand, "set_speed",
                        lambda self, value: writes.append(("speed", value)) or True)
    monkeypatch.setattr(inspire_hand.InspireHand, "set_force",
                        lambda self, value: writes.append(("force", value)) or True)

    hand = inspire_hand.InspireHand(inspire_hand.InspireHandConfig(
        mock=False, initialize_runtime=False))
    assert hand.connect() is True
    assert writes == []
    hand.disconnect()


def test_inspire_identity_records_optional_device_and_firmware():
    spec = hf.HandModelSpec.load(SPEC_PATH)
    adapter = hf.InspireRH56Adapter(
        spec, port="/dev/example", device_id="hand-lab-01", firmware="v1.09")
    identity = adapter.identity()
    assert identity["device_id"] == "hand-lab-01"
    assert identity["firmware"] == "v1.09"


def test_completed_profile_projects_vla_target_with_single_and_interaction_limits(tmp_path):
    spec, _, adapter, recorder, probe = _rig(tmp_path)
    probe.preflight()
    probe.run_single()
    probe.run_interactions()
    recorder.complete()

    with pytest.raises(hf.ProbeError, match="Mock Profile"):
        hf.FeasibilityEnvelope.load(spec, tmp_path / "profile.json")
    envelope = hf.FeasibilityEnvelope.load(
        spec, tmp_path / "profile.json", allow_mock=True)
    result = envelope.project({name: 1.0 for name in spec.joint_names}, margin_u=0.02)
    assert result["changed"] is True
    assert result["projected_u"]["right_thumb_1_joint"] == pytest.approx(0.98)
    assert result["projected_u"]["right_index_1_joint"] <= 0.74
    assert any(change["reason"].startswith("interaction:") for change in result["changes"])
    assert result["projected_raw"]["right_index_1_joint"] >= 260
    adapter.close()
