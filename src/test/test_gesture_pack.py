#!/usr/bin/env python3
"""src/test/test_gesture_pack.py — gesture_pack 的单元测试。

环境里**没有 pytest**(lerobot conda env 没装),所以用纯 assert + __main__ 跑:
    python3 -m pytest src/test/test_gesture_pack.py
装了 pytest 的话也能直接 pytest 收集,函数名是 test_* 的。
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# ⚠ 必须在 import gesture_pack **之前**设环境变量:gesture_root() 每次调用都读
# os.environ,所以其实晚设也行 —— 但 paths.DATA 是 import 期算的,而且真跑测试时
# 绝不能碰到真的 data/gestures/。用临时目录整个隔开。
_TMP = tempfile.mkdtemp(prefix="gesture_test_")
os.environ["HAND_GESTURE_DIR"] = _TMP

import gesture_pack as gp                                          # noqa: E402
from inspire_hand import HAND_JOINTS, HAND_LIMITS, RAW_MAP         # noqa: E402


def _frame(**kw):
    kw.setdefault("rad", [0.0] * 6)
    return gp.GestureFrame.build(**kw)


def _pack(n=2, **kw):
    kw.setdefault("name", "测试手势")
    return gp.GesturePack(frames=[_frame(hold_ms=100 * (i + 1)) for i in range(n)], **kw)


# ---------------------------------------------------------------- 换算
def test_rad_raw_roundtrip():
    """rad → raw → rad 必须稳定(第二次往返不再变化)。"""
    for rad in ([0.0] * 6, [HAND_LIMITS[n][1] for n in HAND_JOINTS],
                [0.5, 0.3, 0.7, 0.7, 0.7, 0.7]):
        raw = gp.rad_to_raw_proj(rad)
        back = gp.raw_proj_to_rad(raw)
        raw2 = gp.rad_to_raw_proj(back)
        assert raw == raw2, f"往返不稳定: {raw} != {raw2}"


def test_raw_endpoints():
    """全 raw 映射都是 invert=True:raw 1000 = 张开 = rad 下限。"""
    rad_open = gp.raw_proj_to_rad([1000] * 6)
    assert all(abs(r - HAND_LIMITS[n][0]) < 1e-6 for n, r in zip(HAND_JOINTS, rad_open)), \
        f"raw 1000 应该是张开位: {rad_open}"
    # raw 0 = 另一端,落在 min(RAW_MAP span, URDF 上限)。两个界哪个紧由关节而定:
    #   thumb_yaw   span 1.246165 == 限位 1.246165 → 两者相等
    #   当前正式资产的 thumb_pitch 和四指 span/limit 已统一为 URDF 标称值。
    # 写成 min() 保持换算对 span 与限位任一侧收紧时都安全。
    rad_closed = gp.raw_proj_to_rad([0] * 6)
    for n, r in zip(HAND_JOINTS, rad_closed):
        want = min(RAW_MAP[n][0], HAND_LIMITS[n][1])
        assert abs(r - want) < 1e-6, f"{n} raw 0 应为 {want},实为 {r}"


def test_vendor_permutation_is_inverse():
    """proj→vendor→proj 必须回到原值。两个函数分开实现,这里锁住它们互逆。"""
    src = [10, 20, 30, 40, 50, 60]
    assert gp.vendor_to_proj(gp.proj_to_vendor(src)) == src
    assert gp.proj_to_vendor(gp.vendor_to_proj(src)) == src


def test_vendor_maps_thumb_to_m5():
    """项目序第 0 位(thumb_yaw)必须落到厂商通道 5(拇指旋转)。
    这条锁住通道语义 —— 错了手指会整体反着动。"""
    v = gp.proj_to_vendor([0, 1, 2, 3, 4, 5])
    assert v[5] == 0 and v[4] == 1 and v[0] == 5, f"通道映射错: {v}"


# ---------------------------------------------------------------- 帧
def test_frame_from_raw_wins():
    """rad 和 raw 都给时以 raw 为准。"""
    f = gp.GestureFrame.build(rad=[0.0] * 6, raw_vendor=[500] * 6)
    assert f.raw_vendor == [500] * 6
    assert all(r > 0.01 for r in f.rad), f"rad 应该由 raw 反推,不是照抄传入值: {f.rad}"


def test_frame_rad_and_raw_always_agree():
    """两个字段必须表示**同一个**姿态 —— rad 转回去要等于存的 raw。

    不成立会怎样:3D 预览读 rad、回放写 raw,两边显示/动作不一致。而且
    ActionPlayer 在真机上直接 write_shorts ANGLE_SET,绕过 InspireHand 的 URDF
    夹取,所以越界的 raw 真的会把关节驱过限位。
    """
    cases = [
        [1000] * 6, [0] * 6, [500] * 6,
        [1000, 1000, 1000, 180, 120, 100],     # thumb_pitch raw 120 越 URDF 限位
        [7, 993, 141, 142, 0, 1000],
    ]
    for rv in cases:
        f = gp.GestureFrame.build(raw_vendor=rv)
        back = gp.proj_to_vendor(gp.rad_to_raw_proj(f.rad))
        assert back == f.raw_vendor, \
            f"raw={rv} → 存 {f.raw_vendor} 但 rad 折回是 {back}"


def test_frame_out_of_envelope_raw_is_snapped():
    """越界 raw 被吸附到限位对应的 raw,不是原样留着。
    thumb_pitch(厂商通道 4)raw 120 → 0.4224 rad,仍在当前 0.48 标称上限内。"""
    f = gp.GestureFrame.build(raw_vendor=[1000, 1000, 1000, 180, 120, 100])
    assert f.raw_vendor[4] == 120, f"当前标称范围内应保持 120,实为 {f.raw_vendor[4]}"
    assert abs(f.rad[1] - 0.4224) < 1e-4, f"thumb_pitch 映射错误: {f.rad[1]}"
    # 其余通道没越界,不该被动
    assert f.raw_vendor[3] == 180 and f.raw_vendor[5] == 100


def test_frame_clamps():
    f = gp.GestureFrame.build(raw_vendor=[-50, 9999, 0, 1000, 500, 500],
                              hold_ms=10 ** 9, speed=5000, force=-3)
    assert f.raw_vendor[0] == 0 and f.raw_vendor[1] == 1000
    assert f.hold_ms == gp.HOLD_MS_MAX and f.speed == 1000 and f.force == 0


def test_frame_rejects_bad_len():
    for bad in ([0.0] * 5, [0.0] * 7, []):
        try:
            gp.GestureFrame.build(rad=bad)
        except gp.GestureError:
            continue
        raise AssertionError(f"长度 {len(bad)} 应该被拒")


def test_frame_rejects_non_numeric():
    for bad in ([0.0, 0.0, 0.0, 0.0, 0.0, "x"], [0.0] * 5 + [None],
                [True] + [0.0] * 5):
        try:
            gp.GestureFrame.build(rad=bad)
        except gp.GestureError:
            continue
        raise AssertionError(f"{bad} 应该被拒")


def test_frame_needs_one_side():
    try:
        gp.GestureFrame.build()
    except gp.GestureError:
        return
    raise AssertionError("rad 和 raw 都不给应该被拒")


# ---------------------------------------------------------------- 包
def test_pack_roundtrip():
    p = _pack(3, note="备注", return_home_first=False,
              playback_mode=gp.PLAYBACK_TIMELINE)
    d = json.loads(json.dumps(p.to_dict(), ensure_ascii=False))
    q = gp.GesturePack.from_dict(d)
    assert q.name == p.name and len(q.frames) == 3
    assert q.note == "备注" and q.return_home_first is False
    assert q.playback_mode == gp.PLAYBACK_TIMELINE
    assert [f.raw_vendor for f in q.frames] == [f.raw_vendor for f in p.frames]
    assert q.duration_ms == 100 + 200 + 300


def test_pack_rejects_wrong_schema():
    d = _pack().to_dict()
    # ⚠ 原来这里拿 "hand_gesture_pack/2" 当"未知版本"举例,而 /2 在 2026-08-03
    # 成了正式版本(加了 t_ns),于是这条测试开始误报。举例用的版本号要挑
    # **明显不会实现**的,别用"下一个版本" —— 下一个版本总会到。
    for bad in ("hand_gesture_pack/99", "hand_gesture_pack", "", None,
                "action_sequence/1"):
        d["schema"] = bad
        try:
            gp.GesturePack.from_dict(d)
        except gp.GestureError:
            continue
        raise AssertionError(f"schema={bad!r} 应该被拒")


def test_pack_rejects_unknown_playback_mode():
    d = _pack().to_dict()
    d["playback_mode"] = "burst_catchup"
    try:
        gp.GesturePack.from_dict(d)
    except gp.GestureError as e:
        assert "playback_mode" in str(e)
        return
    raise AssertionError("未知 playback_mode 应该被拒")


def test_screwdriver_gesture_imports_as_timeline_latest():
    """用户确认的 451 帧视频动作包保持源时间轴语义。"""
    path = Path(__file__).resolve().parents[2] / "data/gestures/拿螺丝刀.json"
    pack = gp.GesturePack.from_dict(json.loads(path.read_text(encoding="utf-8")))
    assert len(pack.frames) == 451
    assert pack.playback_mode == gp.PLAYBACK_TIMELINE
    assert pack.frames[0].t_ns == 0 and pack.frames[-1].t_ns == 7_507_000_000
    assert pack.duration_ms == 7_907
    assert all(pack.frames[i].t_ns > pack.frames[i - 1].t_ns
               for i in range(1, len(pack.frames)))
    seq = gp.to_action_sequence(pack)
    assert seq.playback_mode == gp.PLAYBACK_TIMELINE
    assert seq.steps[1].t_ns == gp.HOME_HOLD_MS * 1_000_000


def test_pack_rejects_empty_frames():
    d = _pack().to_dict()
    d["frames"] = []
    try:
        gp.GesturePack.from_dict(d)
    except gp.GestureError:
        return
    raise AssertionError("空 frames 应该被拒")


def test_pack_rejects_bad_name():
    for bad in ("", "   ", None, "x" * 200, "带\n换行"):
        d = _pack().to_dict()
        d["name"] = bad
        try:
            gp.GesturePack.from_dict(d)
        except gp.GestureError:
            continue
        raise AssertionError(f"name={bad!r} 应该被拒")


def test_pack_rejects_mismatched_joint_order():
    """joint_order 对不上就拒,**不重排** —— 静默重排会把角度装到错的手指上。"""
    d = _pack().to_dict()
    d["joint_order"] = list(reversed(HAND_JOINTS))
    try:
        gp.GesturePack.from_dict(d)
    except gp.GestureError:
        return
    raise AssertionError("joint_order 不一致应该被拒")


def test_pack_frame_error_says_which_frame():
    d = _pack(3).to_dict()
    d["frames"][1]["raw_vendor"] = [0, 0, 0]
    try:
        gp.GesturePack.from_dict(d)
    except gp.GestureError as e:
        assert "第 2 帧" in str(e), f"错误信息该指出是第几帧: {e}"
        return
    raise AssertionError("坏帧应该被拒")


def test_pack_hand_written_without_raw():
    """手改的文件只写 rad,没有 raw_vendor —— 要能从 rad 推出来。"""
    d = _pack().to_dict()
    for f in d["frames"]:
        del f["raw_vendor"]
    q = gp.GesturePack.from_dict(d)
    assert q.frames[0].raw_vendor == [1000] * 6, q.frames[0].raw_vendor


# ---------------------------------------------------------------- 转 ActionSequence
def test_to_action_sequence_home_first():
    p = _pack(2, return_home_first=True)
    seq = gp.to_action_sequence(p, slot=7)
    assert len(seq.steps) == 3, "回零帧 + 2 帧"
    assert seq.slot == 7 and seq.name == p.name
    assert seq.steps[0].delay_ms == gp.HOME_HOLD_MS
    assert seq.steps[1].delay_ms == 100 and seq.steps[2].delay_ms == 200


def test_to_action_sequence_no_home():
    seq = gp.to_action_sequence(_pack(2, return_home_first=False))
    assert len(seq.steps) == 2, "不回零就没有额外帧"


def test_to_action_sequence_return_home_override():
    p = _pack(2, return_home_first=True)
    assert len(gp.to_action_sequence(p, return_home=False).steps) == 2
    q = _pack(2, return_home_first=False)
    assert len(gp.to_action_sequence(q, return_home=True).steps) == 3


def test_to_action_sequence_angles_are_project_order():
    """ActionStep.angles 必须是**项目序** —— ActionPlayer._send_angles() 会再做
    一次项目→厂商置换才写 ANGLE_SET。存的是厂商序,这里必须转回来。
    错了的话手指会整体反着动(拇指的值发给小指)。"""
    f = gp.GestureFrame.build(raw_vendor=[0, 100, 200, 300, 400, 500])
    seq = gp.to_action_sequence(
        gp.GesturePack(name="x", frames=[f], return_home_first=False))
    assert seq.steps[0].angles == gp.vendor_to_proj(f.raw_vendor)
    # 再走一遍 ActionPlayer 的置换,必须回到原始厂商序
    assert gp.proj_to_vendor(seq.steps[0].angles) == f.raw_vendor


def test_to_action_sequence_home_is_open_hand():
    seq = gp.to_action_sequence(_pack(1, return_home_first=True))
    home = gp.proj_to_vendor(seq.steps[0].angles)
    assert home == [1000] * 6, f"回零应该是全张开(raw 1000): {home}"


def test_to_action_sequence_expands_speed_force():
    f = gp.GestureFrame.build(rad=[0.0] * 6, speed=321, force=123)
    seq = gp.to_action_sequence(
        gp.GesturePack(name="x", frames=[f], return_home_first=False))
    assert seq.steps[0].speeds == [321] * 6
    assert seq.steps[0].forces == [123] * 6


# ---------------------------------------------------------------- 路径沙箱(安全)
ESCAPES = [
    "../evil.json", "../../etc/passwd.json", "a/../../evil.json",
    "/etc/passwd.json", "/tmp/evil.json", "//etc/evil.json",
    "..\\evil.json", "a\\..\\..\\evil.json",         # 反斜杠也当分隔符
    "./../evil.json", "sub/../../evil.json",
    ".hidden.json", "sub/.hidden.json",              # 点开头
    "evil.py", "evil.json.py", "evil", "evil.txt",   # 后缀
    "a/b/c/d/e/f/g/h/i/deep.json",                   # 层级
    "x" * 200 + ".json",                             # 段长
    "bad\x00name.json", "bad\nname.json",            # 控制字符
    "", "   ", "/", "//", "./",
]


def test_sandbox_rejects_escapes():
    for bad in ESCAPES:
        try:
            gp.resolve_pack_path(bad)
        except gp.GestureError:
            continue
        raise AssertionError(f"路径 {bad!r} 应该被拒,但通过了")


def test_sandbox_accepts_normal():
    root = gp.gesture_root()
    for good in ["a.json", "sub/a.json", "手势/OK手势.json", "a/b/c.json",
                 "带 空格.json", "A.JSON"]:
        p = gp.resolve_pack_path(good)
        assert p.is_relative_to(root), f"{good} → {p} 不在根内"


def test_sandbox_blocks_symlink_escape():
    """根目录里放一个指向 /tmp 的软链接,穿它写文件必须被拒。

    这条是 resolve() 而不是 normpath() 的理由:normpath 是纯字符串运算,不看文件
    系统,"gestures/link/evil.json" 在字符串层面完全合法,它挡不住。
    """
    root = gp.gesture_root()
    root.mkdir(parents=True, exist_ok=True)
    outside = Path(tempfile.mkdtemp(prefix="gesture_outside_"))
    link = root / "link"
    if link.is_symlink() or link.exists():
        link.unlink()
    try:
        link.symlink_to(outside, target_is_directory=True)
    except (OSError, NotImplementedError):
        return                                    # 不支持软链接的文件系统,跳过
    try:
        gp.resolve_pack_path("link/evil.json")
    except gp.GestureError:
        return
    finally:
        link.unlink(missing_ok=True)
    raise AssertionError("穿软链接逃出根目录应该被拒")


def test_sandbox_root_honours_env():
    assert gp.gesture_root() == Path(_TMP).resolve(), \
        f"HAND_GESTURE_DIR 没生效: {gp.gesture_root()}"


# ---------------------------------------------------------------- 文件读写
def test_save_load_delete():
    rel = "存取/往返.json"
    p = _pack(2, note="往返测试")
    path = gp.save_pack(rel, p)
    assert path.is_file() and path.parent.name == "存取"
    q = gp.load_pack(rel)
    assert q.name == p.name and len(q.frames) == 2 and q.note == "往返测试"
    assert q.created_at, "保存时应该自动打时间戳"
    gp.delete_pack(rel)
    assert not path.exists()


def test_save_is_atomic_no_temp_left():
    gp.save_pack("原子.json", _pack())
    leftover = [x.name for x in gp.gesture_root().rglob(".tmp_*")]
    assert not leftover, f"临时文件没清掉: {leftover}"


def test_save_overwrite_guard():
    gp.save_pack("守卫.json", _pack())
    try:
        gp.save_pack("守卫.json", _pack(), overwrite=False)
    except gp.GestureError:
        pass
    else:
        raise AssertionError("overwrite=False 遇到已存在应该拒")
    gp.save_pack("守卫.json", _pack(3))                    # 默认允许覆盖
    assert len(gp.load_pack("守卫.json").frames) == 3


def test_load_missing():
    try:
        gp.load_pack("不存在的包.json")
    except gp.GestureError:
        return
    raise AssertionError("不存在的文件应该拒")


def test_list_packs_reports_broken_file():
    """坏文件要出现在列表里带 error,不能让整个列表挂掉。"""
    root = gp.gesture_root()
    root.mkdir(parents=True, exist_ok=True)
    (root / "坏文件.json").write_text("{ 这不是 json", encoding="utf-8")
    gp.save_pack("好文件.json", _pack())
    items = gp.list_packs()
    by = {it["path"]: it for it in items}
    assert "坏文件.json" in by and by["坏文件.json"].get("error")
    assert "好文件.json" in by and not by["好文件.json"].get("error")
    (root / "坏文件.json").unlink()


def test_list_packs_skips_temp_files():
    root = gp.gesture_root()
    root.mkdir(parents=True, exist_ok=True)
    (root / ".tmp_half.json").write_text("{}", encoding="utf-8")
    assert not [it for it in gp.list_packs() if it["path"].startswith(".tmp_")]
    (root / ".tmp_half.json").unlink()


def test_find_by_name():
    gp.save_pack("查找/甲.json", gp.GesturePack(name="握拳", frames=[_frame()]))
    hits = gp.find_by_name("握拳")
    assert len(hits) == 1 and hits[0]["path"] == "查找/甲.json"
    assert gp.find_by_name("不存在的名字") == []
    assert gp.find_by_name("") == []


def test_find_by_name_case_insensitive_fallback():
    gp.save_pack("查找/ok.json", gp.GesturePack(name="OkSign", frames=[_frame()]))
    assert gp.find_by_name("OkSign")[0]["path"] == "查找/ok.json"
    assert gp.find_by_name("oksign")[0]["path"] == "查找/ok.json"


def test_find_by_name_duplicates_all_returned():
    """重名**不猜** —— 全部返回,由调用方决定(web 层转 409 + 候选列表)。"""
    gp.save_pack("重名/一.json", gp.GesturePack(name="同名", frames=[_frame()]))
    gp.save_pack("重名/二.json", gp.GesturePack(name="同名", frames=[_frame()]))
    assert len(gp.find_by_name("同名")) == 2


def test_find_by_name_exact_beats_case_insensitive():
    gp.save_pack("优先/A.json", gp.GesturePack(name="Fist", frames=[_frame()]))
    gp.save_pack("优先/b.json", gp.GesturePack(name="fist", frames=[_frame()]))
    hits = gp.find_by_name("Fist")
    assert len(hits) == 1 and hits[0]["name"] == "Fist", \
        "精确命中时不该把不分大小写的也算进来"


def _pack30(n: int = 600, with_tns: bool = True) -> dict:
    """n 帧 30fps 的包。with_tns=False 模拟 /1 旧文件(只有整数 hold_ms)。"""
    fr = []
    for i in range(n):
        f = {"rad": [0.1] * 6, "hold_ms": 33}
        if with_tns:
            f["t_ns"] = int(round(i * 1e9 / 30))
        fr.append(f)
    return {"schema": "hand_gesture_pack/2" if with_tns else "hand_gesture_pack/1",
            "name": "t30", "frames": fr}


def test_t_ns_is_authoritative_not_accumulated_hold_ms():
    """t_ns 必须零累积误差;整数 hold_ms 累加会漂 ~200ms/600帧。

    这是 2026-08-03 实测到的问题:hold_ms 是 int,30fps 真周期 33.3333…ms,
    存成 33 每帧少 0.333ms 且**单向不抵消**。臂侧按绝对时刻走、手侧按累加走,
    600 帧(20s)末尾错开 199.7ms —— 抓取动作里手已经合上而臂还没到位。
    """
    p = gp.GesturePack.from_dict(_pack30(600))
    ideal = 599 / 30
    err_us = abs(p.frames[-1].t_ns / 1e9 - ideal) * 1e6
    assert err_us < 1.0, f"t_ns 末帧误差 {err_us:.3f}us,应 <1us"
    worst = max(abs(f.t_ns - i / 30 * 1e9) for i, f in enumerate(p.frames)) / 1000
    assert worst < 1.0, f"逐帧最大误差 {worst:.3f}us"
    # 反证:累加 hold_ms 确实会漂,否则上面两条恒真
    drift = abs(p.frames[-1].t_ns / 1e6 - sum(f.hold_ms for f in p.frames[:-1]))
    assert drift > 100, f"累加 hold_ms 只漂 {drift:.1f}ms,反证不成立"


def test_old_schema1_pack_still_loads_and_gets_t_ns():
    """/1 旧文件要能读,并且在入口就补上 t_ns(下游只处理一种情况)。

    补出来的时刻**带原有漂移** —— 旧文件本来就按整数 hold_ms 录的,补不回精度。
    drift_ms() 对这种包恒为 0(因为就是按 hold_ms 累加补的),这本身就是标记。
    """
    p = gp.GesturePack.from_dict(_pack30(600, with_tns=False))
    assert all(f.t_ns is not None for f in p.frames), "/1 读进来应补齐 t_ns"
    assert p.frames[0].t_ns == 0
    assert abs(p.drift_ms()) < 1e-6, f"/1 补出来的 drift 应为 0,实得 {p.drift_ms()}"
    # 而它确实比理想时刻早 ~200ms —— 精度丢了,补不回来
    lost = (599 / 30) * 1000 - p.frames[-1].t_ns / 1e6
    assert 150 < lost < 250, f"旧包应比理想早约 200ms,实得 {lost:.1f}ms"


def test_t_ns_filled_records_how_many_were_reconstructed():
    """`t_ns_filled` 记着读文件时补了几帧 —— 下游要能分清"时刻来自视频源"和
    "时刻是累加出来的、带漂移"。

    ⚠ 为什么不能拿 drift_ms()==0 反推:补出来的包 drift 恒为 0,**但整数毫秒
    正好整除的包 drift 也是 0**(比如 10fps,每帧 100ms),而那种包时刻是准的。
    拿 drift==0 当"这是旧包"会误报在后者身上 —— 联合回放器第一版就是那么写的。
    """
    old = gp.GesturePack.from_dict(_pack30(600, with_tns=False))
    assert old.t_ns_filled == 600, f"/1 应补满 600 帧,实得 {old.t_ns_filled}"
    new = gp.GesturePack.from_dict(_pack30(600, with_tns=True))
    assert new.t_ns_filled == 0, f"/2 不该补,实得 {new.t_ns_filled}"
    # 反证:整数毫秒整除的包 —— drift 是 0 但它**不是**旧包
    d = _pack30(5, with_tns=True)
    for i, f in enumerate(d["frames"]):
        f["hold_ms"] = 100
        f["t_ns"] = i * 100_000_000
    exact = gp.GesturePack.from_dict(d)
    assert exact.t_ns_filled == 0, "整除的包不该被当成补出来的"
    assert abs(exact.drift_ms()) < 1e-6, "整除的包 drift 也是 0(所以 drift 不能当判据)"
    # t_ns_filled 不进文件 —— 它描述"这次怎么读进来的",不是包的内容
    assert "t_ns_filled" not in old.to_dict()


def test_to_action_sequence_offsets_t_ns_by_home_step():
    """return_home_first 时前面多一步,t_ns 必须整体后移,否则首帧截止已过期。"""
    p = gp.GesturePack.from_dict(_pack30(10))
    seq_h = gp.to_action_sequence(p)
    p2 = gp.GesturePack.from_dict({**_pack30(10), "return_home_first": False})
    seq_n = gp.to_action_sequence(p2)
    assert len(seq_h.steps) == len(seq_n.steps) + 1, "带 home 应多一步"
    off = seq_h.steps[1].t_ns - seq_n.steps[0].t_ns
    assert off == int(round(gp.HOME_HOLD_MS * 1e6)), \
        f"偏移应等于 HOME_HOLD_MS={gp.HOME_HOLD_MS}ms,实得 {off/1e6:.1f}ms"
    assert seq_n.steps[0].t_ns == 0, "不带 home 时首帧应在 0"


def main() -> int:
    tests = [(k, v) for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v) and v.__module__ == __name__]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ok   {name}")
        except Exception as e:                                     # noqa: BLE001
            failed.append((name, e))
            print(f"  FAIL {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} 通过   根目录={_TMP}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
