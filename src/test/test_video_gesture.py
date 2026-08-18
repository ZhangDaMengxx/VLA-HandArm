#!/usr/bin/env python3
"""src/test/test_video_gesture.py — 挑帧 / 时序保真度的单元测试(不碰视频解码)。

只测纯数据变换那部分:pick_keyframes 和 frames_to_pack_frames。视频解码 +
MediaPipe 那段要真视频、跑得慢,放端到端测试里。

    python3 -m pytest src/test/test_video_gesture.py
"""
from __future__ import annotations

import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SRC))

import video_gesture as vg                                     # noqa: E402
from gesture_pack import PLAYER_TICK_MS                         # noqa: E402


def _seq(n, fps=30.0, stride=1, ramp=0.0):
    """造 n 帧:t 按 fps/stride 递增,rad 按 ramp 逐帧线性增长。"""
    return [{"frame": i * stride, "t": round(i * stride / fps, 6),
             "rad": [min(1.0, ramp * i)] * 6} for i in range(n)]


def test_hold_floor_is_tick_not_80():
    """驻留下限必须是**回放器 tick 周期**,不是凭空拍的 80ms。

    这条是"看起来很延迟"的根因回归测试:30fps 源 stride=1 帧间 33ms,原来被抬到
    80ms → 整段慢 2.4×。下限该等于 tick(比一个 tick 短的驻留落不到实处),
    不该再额外加慢放。
    """
    pf, tm = vg.frames_to_pack_frames(_seq(10, fps=30.0, stride=1))
    holds = [f["hold_ms"] for f in pf[:-1]]          # 末帧用 default,不看
    assert all(h == 33 for h in holds), f"30fps 应得 33ms,实得 {set(holds)}"
    assert tm["tick_ms"] == PLAYER_TICK_MS
    assert tm["floored"] == 0, "33ms 不该被抬(tick 就是 33)"
    assert abs(tm["stretch"] - 1.0) < 0.05, f"不该有拉伸,实得 {tm['stretch']}"


def test_no_stretch_at_common_strides():
    """常用 stride 下回放时长应贴近源时长。"""
    for stride in (1, 2, 3, 4, 6):
        pf, tm = vg.frames_to_pack_frames(_seq(20, fps=30.0, stride=stride))
        assert abs(tm["stretch"] - 1.0) < 0.06, \
            f"stride={stride} 拉伸 {tm['stretch']}"


def test_stretch_reported_when_source_faster_than_tick():
    """源比 tick 还快时必须**如实报告**拉伸倍数,不能默默慢放。

    阈值写成**相对 tick** 的,不写死倍数 —— PLAYER_TICK_MS 调整时(30Hz→100Hz
    就调过一次)写死的数字会假失败。取 tick 的 1/4 帧间隔,期望拉伸≈4×。
    """
    fps = 1000.0 / (PLAYER_TICK_MS / 4.0)          # 帧间 = tick/4
    pf, tm = vg.frames_to_pack_frames(_seq(10, fps=fps, stride=1))
    assert tm["floored"] > 0, f"帧间 {PLAYER_TICK_MS/4}ms 应被记成 floored"
    # 指标必须**如实**报出量级:早先把末帧的 default_hold_ms 也算进分子分母,
    # 4× 被稀释成 1.48×,看起来"还行",真实失真就被藏住了。
    assert tm["stretch"] > 3.0, f"帧间为 tick/4 时应报约 4× 拉伸,实得 {tm['stretch']}"
    assert all(f["hold_ms"] >= PLAYER_TICK_MS for f in pf)


def test_30fps_source_not_floored_at_current_tick():
    """30fps 素材(33ms)在当前 tick 下不该被抬 —— 那是最常见的素材帧率。"""
    pf, tm = vg.frames_to_pack_frames(_seq(20, fps=30.0, stride=1))
    assert tm["floored"] == 0, \
        f"33ms 间隔被抬了 {tm['floored']} 次,tick={PLAYER_TICK_MS}ms 太粗"
    assert abs(tm["stretch"] - 1.0) < 0.02, f"应无拉伸,实得 {tm['stretch']}"


def test_long_gap_capped():
    """漏检造成的长空档要截短,否则回放莫名停几秒。"""
    fr = [{"frame": 0, "t": 0.0, "rad": [0.0] * 6},
          {"frame": 300, "t": 10.0, "rad": [0.5] * 6},     # 10s 空档
          {"frame": 303, "t": 10.1, "rad": [0.6] * 6}]
    pf, tm = vg.frames_to_pack_frames(fr)
    assert pf[0]["hold_ms"] == vg.HOLD_MS_CEIL, f"应截到上限,实得 {pf[0]['hold_ms']}"
    assert tm["ceiled"] == 1


def test_pack_frames_shape():
    pf, _ = vg.frames_to_pack_frames(_seq(5), speed=321, force=123)
    assert len(pf) == 5
    for f in pf:
        assert len(f["rad"]) == 6
        assert f["speed"] == 321 and f["force"] == 123
        assert isinstance(f["hold_ms"], int) and f["hold_ms"] >= 0
        assert f["label"]


def test_pick_keyframes_first_and_last():
    fr = _seq(40, ramp=0.02)
    kf = vg.pick_keyframes(fr, eps=0.25, max_out=64)
    assert kf[0] is fr[0], "首帧必选"
    assert kf[-1]["t"] == fr[-1]["t"], "末帧要补上(手势收尾姿态通常就是要存的那个)"


def test_pick_keyframes_monotonic_in_eps():
    """eps 越大挑得越少 —— 阈值语义的基本保证。"""
    fr = _seq(120, ramp=0.01)
    counts = [len(vg.pick_keyframes(fr, eps=e, max_out=999))
              for e in (0.05, 0.1, 0.25, 0.5, 1.0)]
    assert all(counts[i] >= counts[i + 1] for i in range(len(counts) - 1)), counts


def test_pick_keyframes_respects_cap():
    fr = _seq(200, ramp=0.02)
    for cap in (1, 3, 12, 50):
        assert len(vg.pick_keyframes(fr, eps=0.05, max_out=cap)) <= cap


def test_pick_keyframes_uses_linf_not_euclidean():
    """一根手指大动要算新姿态;六根各微动不算。

    L∞ 和欧氏的区别就在这:六根各动 0.1 的欧氏范数是 0.245(接近 0.25 阈值),
    而一根动 0.2 的欧氏只有 0.2 —— 用欧氏会把"抖动"判成新姿态、把"单指动作"漏掉。
    """
    base = [0.0] * 6
    one_big = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]      # 单指大动 → 该选
    all_small = [0.1] * 6                          # 六根微动 → 不该选
    fr = [{"frame": 0, "t": 0.0, "rad": base},
          {"frame": 1, "t": 0.1, "rad": all_small},
          {"frame": 2, "t": 0.2, "rad": base},
          {"frame": 3, "t": 0.3, "rad": one_big}]
    kf = vg.pick_keyframes(fr, eps=0.25, max_out=99)
    picked = [f["frame"] for f in kf]
    assert 3 in picked, f"单指动 0.3 应被选中: {picked}"
    assert 1 not in picked, f"六根各动 0.1 不该被选(那是抖动): {picked}"


def test_pick_keyframes_empty():
    assert vg.pick_keyframes([]) == []


def test_pick_keyframes_single():
    fr = _seq(1)
    assert len(vg.pick_keyframes(fr)) == 1


def test_caps_are_consistent_with_pack():
    """抽取上限不能超过技能包能装的帧数,否则解完了存不进去。"""
    from gesture_pack import MAX_FRAMES
    assert vg.MAX_EXTRACT_FRAMES <= MAX_FRAMES, \
        f"抽取上限 {vg.MAX_EXTRACT_FRAMES} > 包上限 {MAX_FRAMES}"


def test_console_hz_matches_spawn_arg():
    """gesture_pack.CONSOLE_HZ 必须就是 app_web 起 console 时传的 --hz。

    两处不一致会让时序算错而**毫无报错**:算 hold_ms 时按 33ms 的分辨率,实际
    tick 却是 50ms,回放整段慢 1.5×,且没人知道为什么。
    """
    import re
    from gesture_pack import CONSOLE_HZ, PLAYER_HZ
    src = (SRC / "app_web.py").read_text(encoding="utf-8")
    m = re.search(r'"src/hand_console\.py".{0,200}?"--player-hz",\s*([^\]]+?)\]',
                  src, re.S)
    assert m, "没在 app_web.py 里找到 hand_console 的 --player-hz 实参"
    arg = m.group(1).strip()
    assert "PLAYER_HZ" in arg, \
        f"--player-hz 应该引用 PLAYER_HZ 而不是写死数字,实际是 {arg!r}"
    assert "CONSOLE_HZ" in src, "--hz 也该引用 CONSOLE_HZ"
    # tick 必须**远小于**最短驻留,不能同量级 —— 同量级时相位抖动会让每步
    # 时而 1 tick 时而 2 tick,整段明显拖慢(实测 33ms tick 播 33ms 帧慢 1.36×)。
    assert PLAYER_HZ >= 3 * CONSOLE_HZ, \
        f"PLAYER_HZ({PLAYER_HZ}) 应远大于遥测率({CONSOLE_HZ})"
    src_console = (SRC
                   / "hand_console.py").read_text(encoding="utf-8")
    assert "--player-hz" in src_console, "hand_console 得认 --player-hz"


DEG = 57.29577951308232


def _frames(vals, start=0, step=1, fps=30.0):
    """vals: 逐帧的关节角(rad),同一个值填满 6 维。"""
    return [{"frame": start + i * step, "t": round((start + i * step) / fps, 4),
             "rad": [v] * 6} for i, v in enumerate(vals)]


def test_segments_split_at_gap():
    f = _frames([0, 0, 0]) + _frames([0, 0], start=10)
    assert vg._segments(f) == [(0, 3), (3, 5)], vg._segments(f)
    assert vg._segments(_frames([0] * 5)) == [(0, 5)]
    assert vg._segments([]) == []


def test_despike_kills_single_spike():
    out, ch = vg.despike(_frames([0.1, 0.1, 0.9, 0.1, 0.1]))
    assert abs(out[2]["rad"][0] - 0.1) < 1e-9, out[2]["rad"][0]
    assert ch == 1, ch


def test_despike_keeps_real_step():
    """真实阶跃(连着两帧都在新位置)不该被中值吃掉 —— 那是手势的转换,不是噪声。"""
    out, _ = vg.despike(_frames([0.1, 0.1, 0.1, 0.9, 0.9, 0.9]))
    assert [round(r["rad"][0], 3) for r in out] == [0.1, 0.1, 0.1, 0.9, 0.9, 0.9]


def test_despike_does_not_cross_gap():
    """gap 两侧不能互相污染:那 333ms 里手真的动了,不是一帧的差。"""
    f = _frames([0.1, 0.1, 0.1]) + _frames([0.9, 0.9, 0.9], start=20)
    out, _ = vg.despike(f)
    assert out[2]["rad"][0] == 0.1 and out[3]["rad"][0] == 0.9


def test_despike_preserves_endpoints():
    out, _ = vg.despike(_frames([0.5, 0.1, 0.1, 0.1, 0.7]))
    assert out[0]["rad"][0] == 0.5 and out[-1]["rad"][0] == 0.7


def test_despike_change_count_ignores_subdegree():
    """改动计数只数 >SPIKE_VISIBLE_DEG 的。用 1e-9 当门槛会报出"改了 81% 的帧"——
    那测的是"碰过"不是"修好了"(中值几乎每帧都会挪一点,还要叠 round 的舍入)。"""
    tiny = 0.5 / DEG                                   # 0.5°,远小于 3° 门槛
    _out, ch = vg.despike(_frames([0.0, 0.0, tiny, 0.0, 0.0]))
    assert ch == 0, f"亚度级摆动不该算尖刺,实际 {ch}"


def test_quality_spike_is_not_bad_region():
    """单帧尖刺产生**两个**连续超速步(去+回)。判据若按"连续>=2步=坏区"会把每个
    尖刺都误报成坏区 —— 实测把 hand2 从 8 处灌水到 9 处。"""
    big = 25 / DEG                                     # 25°/帧 @30fps = 750°/s
    q = vg.quality(_frames([0.0, 0.0, big, 0.0, 0.0]))
    assert q["bad_regions"] == [], q["bad_regions"]
    assert q["isolated_spikes"] == 1, q


def test_quality_sustained_failure_is_bad_region():
    """中值修不掉的持续超速 = 真坏区(MediaPipe 丢了目标,整个邻域都是垃圾)。"""
    big = 25 / DEG
    q = vg.quality(_frames([0.0, 0.0, big, 2 * big, 3 * big, 4 * big]))
    assert len(q["bad_regions"]) == 1, q["bad_regions"]


def test_quality_reports_gap_without_smoothing_it():
    """gap 只报告,不填补 —— 两侧姿态差是真的,抹平等于编造中间过程。"""
    f = _frames([0.0, 0.0]) + _frames([0.5], start=12)
    q = vg.quality(f)
    assert len(q["gaps"]) == 1 and q["gaps"][0]["missing"] == 10, q["gaps"]
    assert q["max_gap_jump_deg"] > 28, q["max_gap_jump_deg"]
    assert q["bad_regions"] == [], "gap 不该被当成坏区(它不是超速,是没数据)"


def test_quality_clean_material_reports_nothing():
    q = vg.quality(_frames([i * 0.01 for i in range(20)]))
    assert q["bad_regions"] == [] and q["gaps"] == [] \
        and q["isolated_spikes"] == 0, q


def test_quality_consistent_with_despike():
    """quality 报的尖刺数必须等于 despike 实际改掉的数 —— 两者用同一判据,
    不该出现"报了尖刺但 despike 没动它"这种自相矛盾。"""
    big = 25 / DEG
    f = _frames([0.0, 0.0, big, 0.0, 0.0, 0.0, big, 0.0, 0.0])
    _clean, ch = vg.despike(f)
    assert vg.quality(f)["isolated_spikes"] == ch, (vg.quality(f), ch)


def test_no_savgol_smoothing():
    """锁死"不加 SavGol 平滑"这个决定。三条实测理由写在 video_gesture 的注释里:
    >8Hz 能量只占方差 0~2.7%(没白噪声可去)、关键点差分 lag-1 自相关 +0.10(相关
    噪声滤不掉)、30fps 下窗长和转换长度同量级必然抹平快速动作(快屈 RMSE
    0.44→3.59)。哪天要加,先重新测这三项。"""
    src = (SRC / "video_gesture.py").read_text("utf-8")
    assert "savgol_filter(" not in src, "video_gesture 不该调 savgol_filter"
    assert "from scipy" not in src and "import scipy" not in src, \
        "不该为了平滑把 scipy 拉进来(3 点中值纯 Python 就够)"
    assert vg.MEDIAN_K == 3, f"中值窗口该是 3(5 会啃掉短促动作),实际 {vg.MEDIAN_K}"


def main() -> int:
    tests = [(k, v) for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ok   {name}")
        except Exception as e:                                     # noqa: BLE001
            failed.append(name)
            print(f"  FAIL {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} 通过")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
