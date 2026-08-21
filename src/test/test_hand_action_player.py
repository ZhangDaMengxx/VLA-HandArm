#!/usr/bin/env python3
"""ActionPlayer 的时间轴覆盖调度与快速串口写回归。"""
from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import hand_console as hc                                      # noqa: E402
from action_sequences import ActionSequence, ActionStep        # noqa: E402
from gesture_pack import PLAYBACK_KEYFRAME, PLAYBACK_TIMELINE  # noqa: E402
from inspire_hand import InspireHand, InspireHandConfig        # noqa: E402


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now


class FakeHand:
    def __init__(self):
        self.cfg = SimpleNamespace(mock=False)
        self.normal = []
        self.fast = []

    def write_shorts(self, reg, vals):
        self.normal.append((reg, list(vals)))
        return True

    def write_shorts_fast(self, reg, vals):
        self.fast.append((reg, list(vals)))
        return True


def _sequence(mode):
    times = [0, 16_000_000, 33_000_000, 50_000_000]
    steps = [ActionStep(angles=[100 + i] * 6, speeds=[1000] * 6,
                        forces=[100] * 6, delay_ms=17 if i != 1 else 16,
                        t_ns=t) for i, t in enumerate(times)]
    # 锁住“最终目标必发”：末帧与前一帧相同也必须建立设备侧终点。
    steps[-1].angles = list(steps[-2].angles)
    return ActionSequence(index=-1, name="60fps", steps=steps, playback_mode=mode)


def test_timeline_latest_overwrites_expired_targets(monkeypatch):
    clock, hand = Clock(), FakeHand()
    monkeypatch.setattr(hc.time, "monotonic", clock.monotonic)
    player = hc.ActionPlayer(_sequence(PLAYBACK_TIMELINE), hand)
    player.start(start_at=0.0)

    first = player.tick()
    assert first["step"] == 1
    assert [x[0] for x in hand.normal] == ["SPEED_SET", "FORCE_SET"]

    clock.now = 0.040
    latest = player.tick()
    assert latest["step"] == 3, "40ms 时应直接发 33ms 目标"
    assert latest["skipped"] == 1 and player.skipped_steps == 1
    assert len(hand.normal) == 2, "速度和力未变化时不应重复写"

    clock.now = 0.055
    final = player.tick()
    assert final["step"] == 4 and final["angle_written"] is True
    assert len(hand.fast) == 3, "时间轴角度应快速写，且最终重复目标仍重发"

    clock.now = 0.068
    assert player.tick() is None and player.done
    assert player.summary()["sent"] == 3
    assert player.summary()["skipped"] == 1


def test_keyframe_strict_never_skips_overdue_step(monkeypatch):
    clock, hand = Clock(), FakeHand()
    monkeypatch.setattr(hc.time, "monotonic", clock.monotonic)
    player = hc.ActionPlayer(_sequence(PLAYBACK_KEYFRAME), hand)
    player.start(start_at=0.0)
    player.tick()
    clock.now = 0.040
    row = player.tick()
    assert row["step"] == 2 and row["skipped"] == 0
    assert not hand.fast, "严格关键帧继续使用带事务语义的普通角度写"
    assert [reg for reg, _ in hand.normal].count("ANGLE_SET") == 2


def test_fast_write_does_not_wait_for_reply():
    calls = []

    class Serial:
        def reset_input_buffer(self):
            calls.append("reset")

        def write(self, payload):
            calls.append(("write", len(payload)))

        def flush(self):
            calls.append("flush")

    hand = InspireHand(InspireHandConfig(mock=False))
    hand._sp = Serial()
    assert hand.write_shorts_fast("ANGLE_SET", [500] * 6)
    assert [x if isinstance(x, str) else x[0] for x in calls] == [
        "reset", "write", "flush"]


def test_screwdriver_451_frames_finishes_on_source_timeline(monkeypatch):
    """最终 JSON 在 200Hz 无阻塞调度下保持 7.507s 源时轴，不累计 16/17ms。"""
    from gesture_pack import GesturePack, to_action_sequence

    path = Path(__file__).resolve().parents[2] / "data/gestures/拿螺丝刀.json"
    pack = GesturePack.from_dict(json.loads(path.read_text(encoding="utf-8")))
    seq = to_action_sequence(pack)
    clock, hand = Clock(), FakeHand()
    monkeypatch.setattr(hc.time, "monotonic", clock.monotonic)
    player = hc.ActionPlayer(seq, hand)
    player.start(start_at=0.0)

    while not player.done and clock.now < 9.0:
        player.tick()
        if player.done:
            break
        clock.now += 0.005

    assert player.done
    assert player.skipped_steps == 0
    assert player.sent_steps == 452             # 回零 + 451 个源目标
    assert len(hand.fast) == 451
    assert [reg for reg, _ in hand.normal] == ["SPEED_SET", "FORCE_SET", "ANGLE_SET"]
    # 0.5s 回零 + 7.507s 末帧时刻 + 0.4s 末驻留，加一个 5ms tick 量化余量。
    assert 8.407 <= clock.now <= 8.412, clock.now
