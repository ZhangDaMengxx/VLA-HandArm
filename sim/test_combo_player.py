#!/usr/bin/env python3
"""combo_player.py 的性质测试。**纯离线,不碰硬件**(臂和手都走 mock)。

重点验的都是实际踩过的:
  1. 跳帧策略**两侧各判** —— 第一版拿臂包的 mode 管两边,会跳掉手编的关键帧
  2. 时刻按 t_ns **定位**,不累加 —— 累加 600 帧漂 200ms
  3. mock 下手侧不能直接调 write_shorts —— 它 `_sp is None` 时恒 False,42 帧全假失败
  4. 入口校验:超限位 / t_ns 不递增 / schema 不认识,一律**拒绝**而不是夹一下继续
  5. resume() 要累计暂停时长,否则恢复瞬间两边一起跳一大段
"""
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

SIM = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM))

import combo_player as C                                    # noqa: E402
from nero_arm import ARM_JOINTS, NERO_ARM_LIMITS            # noqa: E402

_FAILS: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print(f"  ok   {name}")
    else:
        _FAILS.append(name)
        print(f"  FAIL {name}" + (f"  — {detail}" if detail else ""))


def mk_pack(n: int = 10, fps: float = 30.0, mode: str = "stream",
            **over) -> dict:
    """合法的最小臂包。rad 全 0(在所有关节限位内)。"""
    wps = []
    for i in range(n):
        t_ns = int(round(i / fps * 1e9))
        wps.append({"i": i, "t_ns": t_ns, "t": round(i / fps, 5),
                    "dt_ms": None if i == n - 1 else round(1000 / fps, 3),
                    "rad": [0.0] * 7})
    d = {"schema": C.ARM_SCHEMA, "name": "t", "joints": list(ARM_JOINTS),
         "mode": mode, "fps_src": fps, "n_waypoints": n,
         "duration_s": (n - 1) / fps, "waypoints": wps,
         "approach": {"rad": [0.0] * 7}}
    d.update(over)
    return d


def load(d: dict):
    """把 dict 写成临时文件再 load —— 走**真实的入口路径**,不绕过文件解析。"""
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False,
                                     encoding="utf-8") as f:
        json.dump(d, f)
        p = Path(f.name)
    try:
        return C.load_arm_pack(p)
    finally:
        p.unlink(missing_ok=True)


def expect_err(name: str, d: dict, frag: str) -> None:
    """这个包**必须**被拒,且理由里带 frag。"""
    try:
        load(d)
    except C.ComboError as e:
        check(name, frag in str(e), f"理由是 {e!r},没提到 {frag!r}")
        return
    check(name, False, "没有抛 ComboError —— 包被接受了")


def cue(t_ms: float, raw: int = 500) -> C.HandCue:
    return C.HandCue(t_ns=int(round(t_ms * 1e6)), raw_vendor=[raw] * 6,
                     speed=500, force=500)


def test_load_validation() -> None:
    print("\n=== 入口校验:不合法的包一律拒绝,不夹一下继续 ===")
    pk = load(mk_pack())
    check("合法包能读", len(pk.waypoints) == 10 and pk.mode == "stream")

    expect_err("schema 不认识 → 拒", mk_pack(schema="arm_traj_pack/99"),
               "schema")
    expect_err("joints 不一致 → 拒(不自动重排)",
               mk_pack(joints=["j1"] * 7), "不做自动重排")
    expect_err("waypoints 空 → 拒", mk_pack(waypoints=[]), "waypoints")
    expect_err("mode 不认识 → 拒", mk_pack(mode="stream2"), "mode")

    # 超限位:joint1 的下限是 -155°,给 -170°
    d = mk_pack()
    d["waypoints"][3]["rad"] = [NERO_ARM_LIMITS[0][0] - 0.3] + [0.0] * 6
    expect_err("超限位 → 拒(而不是夹了继续)", d, "超限位")

    # ⚠ 这一条是回归:prep_arm_traj 把 rad round 到 5 位,joint6 上限
    # radians(55)=0.95993… round 成 0.96000 就"超限"。容差必须放过它。
    d = mk_pack()
    d["waypoints"][2]["rad"] = [0.0] * 5 + [0.96, 0.0]
    try:
        load(d)
        check("取整残差(0.96 vs 上限 0.95993)放过", True)
    except C.ComboError as e:
        check("取整残差放过", False, str(e))

    # t_ns 必须严格递增 —— 不递增时"跳到最新帧"会跳错方向
    d = mk_pack()
    d["waypoints"][5]["t_ns"] = d["waypoints"][4]["t_ns"]
    expect_err("t_ns 不递增 → 拒", d, "递增")

    # /1 没有 t_ns:读得进来,但**必须警告**时间轴带漂移
    d = mk_pack(schema="arm_traj_pack/1")
    for w in d["waypoints"]:
        del w["t_ns"]
    pk = load(d)
    check("/1 能读", len(pk.waypoints) == 10)
    check("/1 警告时间轴带漂移",
          any("漂移" in w for w in pk.warnings), str(pk.warnings))
    # 累加 dt_ms(33.333)出来的时刻:末点应该 ≈ 9×33.333ms,不是 9/30s
    check("/1 的时刻从 dt_ms 累加", pk.waypoints[-1].t_ns > 0)


def test_skip_is_per_side() -> None:
    """跳帧策略两侧各判。**这是第一版的 bug**:拿臂包的 mode 管两边。"""
    print("\n=== 跳帧:两侧各判,不共用一个 flag ===")
    pk = load(mk_pack(mode="stream"))
    # 手侧是关键帧(间距 500ms),臂侧是流(33ms)
    keyframes = [cue(i * 500) for i in range(5)]
    pl = C.ComboPlayer(pk, None, keyframes, None)
    check("臂是 stream → 允许跳", pl.skip_arm is True)
    check("手是关键帧(500ms 间距)→ 不允许跳", pl.skip_hand is False)

    # 手侧是流(33ms)
    stream = [cue(i * 33.333) for i in range(20)]
    pl2 = C.ComboPlayer(pk, None, stream, None)
    check("手是流(33ms 间距)→ 允许跳", pl2.skip_hand is True)

    # 用中位数而不是均值:关键帧里插一个超长驻留,均值会被拉过阈值
    mixed = [cue(0), cue(33), cue(66), cue(99), cue(5099)]
    check("中位数判据不被单个长驻留带偏", C._hand_is_stream(mixed) is True,
          "均值 1275ms 会误判成关键帧")

    check("单帧包保守当关键帧(不跳)", C._hand_is_stream([cue(0)]) is False)


def test_advance_skip_semantics() -> None:
    print("\n=== _advance:跳到最新已到期帧 vs 一次一帧 ===")
    items = [cue(i * 100) for i in range(6)]      # 0,100,...,500ms
    now = int(350e6)                              # 350ms:0/100/200/300 都到期了
    j_skip = C.ComboPlayer._advance(items, 0, now, True)
    check("允许跳 → 直接到最后一个已到期帧(下标 3)", j_skip == 3, str(j_skip))
    j_no = C.ComboPlayer._advance(items, 0, now, False)
    check("不允许跳 → 只推进一帧(下标 0)", j_no == 0, str(j_no))
    # ⚠ 从下标 1 起算才测得到"没到期":下标 0 的 t_ns 是 0,任何 now ≥ 0 都已到期。
    # 第一版写的 _advance(items, 0, 50ms) 期望 -1,那是**测试写错了**不是代码错。
    check("没到期 → -1", C.ComboPlayer._advance(items, 1, int(50e6), True) == -1)
    check("下标 0 在 t=0 就到期", C.ComboPlayer._advance(items, 0, 0, True) == 0)
    check("走完 → -1", C.ComboPlayer._advance(items, 6, now, True) == -1)


def test_t_ns_not_accumulated() -> None:
    """时刻按 t_ns 定位。累加周期的话 30fps 600 帧漂 200ms。"""
    print("\n=== 时间轴:按 t_ns 定位,不累加 ===")
    n, fps = 600, 30.0
    pk = load(mk_pack(n=n, fps=fps))
    # 权威时刻 vs 累加取整毫秒
    want_ns = pk.waypoints[-1].t_ns
    acc_ns = sum(int(round(1000 / fps)) for _ in range(n - 1)) * 1_000_000
    drift_ms = (want_ns - acc_ns) / 1e6
    check(f"600 帧 t_ns 末帧 {want_ns/1e9:.6f}s ≈ 599/30",
          abs(want_ns / 1e9 - 599 / 30) < 1e-6)
    check(f"累加整数毫秒会漂 {drift_ms:.1f}ms(所以不能累加)",
          abs(drift_ms) > 190.0, f"只漂了 {drift_ms:.1f}ms")
    # 逐帧比:t_ns 定位的误差不累积
    worst = max(abs(w.t_ns / 1e9 - w.i_expect) for w in
                [type("W", (), {"t_ns": w.t_ns, "i_expect": i / fps})()
                 for i, w in enumerate(pk.waypoints)])
    check(f"逐帧最大误差 {worst*1e6:.3f}µs(不累积)", worst < 1e-6)


def test_mock_hand_send_does_not_false_fail() -> None:
    """mock 下手侧不能直接调 write_shorts —— 它 `_sp is None` 时恒 False。"""
    print("\n=== mock 手:不能走 write_shorts(否则每帧假失败)===")
    from inspire_hand import InspireHand, InspireHandConfig

    hand = InspireHand(InspireHandConfig(mock=True))
    hand.connect()
    check("mock 手的 write_shorts 确实恒 False(这是根因)",
          hand.write_shorts("ANGLE_SET", [500] * 6) is False)

    pk = load(mk_pack(n=3))
    cues = [cue(0), cue(33.333), cue(66.667)]
    pl = C.ComboPlayer(pk, None, cues, hand)
    ok = pl._send_hand(cues[0])
    check("_send_hand 在 mock 下返回 True", ok is True)
    # 而且 mock 状态要**真的跟着走**,不是空转返回 True
    before = list(hand._target_rad)
    pl._send_hand(C.HandCue(t_ns=0, raw_vendor=[900] * 6, speed=500, force=500))
    check("mock 状态跟着回放变(不是空转)",
          list(hand._target_rad) != before)
    hand.disconnect()


def test_pause_resume_accumulates() -> None:
    """resume() 要累计暂停时长,否则恢复瞬间两边一起跳一大段。"""
    print("\n=== 暂停/继续:累计暂停时长 ===")
    import time

    pk = load(mk_pack(n=100))
    pl = C.ComboPlayer(pk, None, None, None)
    pl.start()
    time.sleep(0.02)
    e1 = pl.elapsed_ns
    pl.pause()
    time.sleep(0.15)                       # 暂停 150ms
    pl.resume()
    e2 = pl.elapsed_ns
    # 暂停期间 elapsed 不该往前走。允许 tick 级的余量。
    check(f"暂停 150ms 后 elapsed 只涨了 {(e2-e1)/1e6:.1f}ms(不是 150)",
          (e2 - e1) / 1e6 < 20.0, f"涨了 {(e2-e1)/1e6:.1f}ms")
    check("_paused_total 记住了暂停时长",
          0.13 < pl._paused_total < 0.20, f"{pl._paused_total:.3f}s")


def test_full_mock_playback() -> None:
    """跑一整趟 mock:帧数对得上、没跳帧、末尾 done。"""
    print("\n=== 整趟 mock 回放 ===")
    import time

    from inspire_hand import InspireHand, InspireHandConfig
    from nero_arm import NeroArm

    # 短包:2s @ 30fps,跑得快
    pk = load(mk_pack(n=60))
    arm = NeroArm(mock=True)
    arm.connect()
    hand = InspireHand(InspireHandConfig(mock=True))
    hand.connect()
    cues = [cue(i * 33.3333) for i in range(60)]
    pl = C.ComboPlayer(pk, arm, cues, hand)
    check("preflight 在 mock 下通过", pl.preflight() == [])
    pl.start()
    t_end = time.monotonic() + 10.0
    while not pl.done and time.monotonic() < t_end:
        pl.tick()
        time.sleep(1.0 / C.TICK_HZ)
    check("跑到 done", pl.done is True)
    check(f"臂发满 {pl.sent_arm}+跳{pl.skipped_arm} = 60",
          pl.sent_arm + pl.skipped_arm == 60)
    check(f"手发满 {pl.sent_hand}+跳{pl.skipped_hand} = 60",
          pl.sent_hand + pl.skipped_hand == 60)
    check("臂没有失败帧", pl.fail_arm == 0, f"{pl.fail_arm} 帧失败")
    check("手没有失败帧", pl.fail_hand == 0, f"{pl.fail_hand} 帧失败")
    check("report 出得来", "臂:发" in C.report(pl))
    arm.disconnect()
    hand.disconnect()


def test_cpv_begin_end_pairing() -> None:
    """cpv_begin/end 必须成对 —— begin 关掉了 auto 切模式,不 end 会影响 move_j。"""
    print("\n=== CPV 进/出模式 ===")
    from nero_arm import NeroArm

    arm = NeroArm(mock=True)
    arm.connect()
    check("初始不在 CPV", arm.cpv_active is False)
    check("cpv_begin 成功", arm.cpv_begin() is True)
    check("begin 后 cpv_active", arm.cpv_active is True)
    arm.cpv_end()
    check("end 后清掉", arm.cpv_active is False)

    # 急停中拒发,和 move_j 同约定
    arm.estop()
    check("急停中 move_cpv_pos 拒发", arm.move_cpv_pos([0.0] * 7) is False)
    arm.reset()
    check("复位后能发", arm.move_cpv_pos([0.1] * 7) is True)

    # 限位夹取兜底(入口校验是"拒",这层是"夹")
    arm.move_cpv_pos([99.0] * 7)
    for i, v in enumerate(arm.target):
        lo, hi = NERO_ARM_LIMITS[i]
        if not (lo - 1e-9 <= v <= hi + 1e-9):
            check(f"move_cpv_pos 夹到限位内({ARM_JOINTS[i]})", False, f"{v}")
            break
    else:
        check("move_cpv_pos 夹到限位内", True)
    arm.disconnect()


def test_real_packs_load() -> None:
    """真包能读 —— 这一条抓过 prep_arm_traj 低通过冲顶出限位的 bug。"""
    print("\n=== 真包(out/arm_pack_*.json)===")
    packs = sorted((SIM / "out").glob("arm_pack_*.json"))
    if not packs:
        print("  跳过:out/ 里没有 arm_pack_*.json")
        return
    for p in packs:
        try:
            pk = C.load_arm_pack(p)
            check(f"{p.name} 能读({len(pk.waypoints)} 点,{pk.mode})", True)
        except C.ComboError as e:
            # ⚠ 真包读不了**就是失败**,不是"跳过"。2026-08-06 就是这一条抓到
            # prep_arm_traj 在低通**之后**没有再夹限位:filtfilt 在被夹出来的
            # 折角处过冲,joint5 顶出限位 2.42°。
            check(f"{p.name} 能读", False, str(e))


def test_combo_pack_adapter() -> None:
    """`load_combo_pack` 摊平成 (arm_pack, cues, meta),两侧同轴。"""
    print("\n=== combo_pack 适配 ===")
    import os
    import combo_pack as cbp
    # ⚠ 沙箱根指到临时目录 —— 不能往 data/combos/ 里写测试文件(那是用户的
    # 录制目录,测试留下的垃圾会出现在页面列表里)。combo_root() 每次调用都读
    # 环境变量,所以这里设完立刻生效,不用 reload。
    os.environ["COMBO_RECORD_DIR"] = tempfile.mkdtemp(prefix="combo_adapter_")

    frames = [{"arm_rad": [i * 0.05, 0.1, 0.0, 0.3, 0.0, 0.0, 0.0],
               "hand_rad": [0.5] * 6, "t_ns": i * 500_000_000, "hold_ms": 500,
               "speed": 500, "force": 500} for i in range(4)]
    d = {"schema": cbp.SCHEMA, "name": "adapter测试", "mode": "keyframe",
         "recorded_from": "mock", "frames": frames}
    cbp.save_pack("a.json", cbp.ComboPack.from_dict(d))
    p = cbp.combo_root() / "a.json"

    pack, cues, meta = C.load_combo_pack(str(p))
    check("臂点数 == 手帧数(同轴录的必然相等)",
          len(pack.waypoints) == len(cues) == 4,
          f"臂 {len(pack.waypoints)} 手 {len(cues)}")
    check("两侧 t_ns 逐帧相同(不需要对齐 t0)",
          all(w.t_ns == c.t_ns for w, c in zip(pack.waypoints, cues)))
    check("approach 就是第 0 帧(录制从当前位姿出发 → 幅度约为零)",
          pack.approach_rad == pack.waypoints[0].rad)
    check("keyframe → 臂包 mode=waypoints", pack.mode == "waypoints", pack.mode)
    check("meta 带出显式 mode 和 recorded_from",
          meta["mode"] == "keyframe" and meta["recorded_from"] == "mock", str(meta))

    # stream 模式映射
    d2 = dict(d, mode="stream", name="流")
    cbp.save_pack("b.json", cbp.ComboPack.from_dict(d2))
    pack2, _, meta2 = C.load_combo_pack(str(cbp.combo_root() / "b.json"))
    check("stream → 臂包 mode=stream", pack2.mode == "stream", pack2.mode)

    # ⚠ 显式 mode 必须**压过**帧间距启发式。这个包间距 500ms(中位 > 100ms),
    # 启发式会判成关键帧 → skip_hand=False;但 mode=stream 说了可以跳。
    # 不压过的话联合包的 mode 字段就白写了。
    heur = C._hand_is_stream(cues)
    check("启发式在这个包上会判成「关键帧」(500ms 间距)", heur is False, str(heur))
    check("而显式 mode=stream 与之相反 —— 所以必须显式优先",
          meta2["mode"] == "stream")

    # t_ns 不递增要拒
    bad = dict(d, frames=[dict(frames[0]), dict(frames[0])])   # 两帧同 t_ns
    cbp.save_pack("c.json", cbp.ComboPack.from_dict(bad))
    try:
        C.load_combo_pack(str(cbp.combo_root() / "c.json"))
        check("t_ns 不递增被拒", False, "没拒")
    except C.ComboError as e:
        check("t_ns 不递增被拒", "递增" in str(e), str(e))


def main() -> int:
    test_load_validation()
    test_skip_is_per_side()
    test_advance_skip_semantics()
    test_t_ns_not_accumulated()
    test_mock_hand_send_does_not_false_fail()
    test_pause_resume_accumulates()
    test_cpv_begin_end_pairing()
    test_real_packs_load()
    test_combo_pack_adapter()
    test_full_mock_playback()
    n = len(_FAILS)
    print(f"\n{'全部通过' if n == 0 else str(n) + ' 项失败: ' + ', '.join(_FAILS)}")
    return 1 if n else 0


if __name__ == "__main__":
    raise SystemExit(main())
