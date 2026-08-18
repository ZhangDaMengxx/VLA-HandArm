#!/usr/bin/env python3
"""验证「力闭环冻结」:检测到力就把当前位置写回 ANGLE_SET,能不能让力停在那儿。要连真手。

这是 hand_grasp 落地前最后一个未知量。前面已实测确定:
  · FORCE_SET 管不住刚性接触的力(阈值 40 时峰值仍 1014)
  · 参数调节范围只有 2-3 倍,最轻档食指稳态仍 535g,做不出"轻捏"
  · 所以只能力闭环主动停:读 FORCE_ACT → 达目标就冻结位置
但**冻结本身没验过**。要回答两件事:

  Q1 机制成立吗 —— 把 ANGLE_ACT 写回 ANGLE_SET,位置误差归零,电机是否真的松劲、
     力是否停止上升?(还是说固件另有逻辑,照样顶下去)
  Q2 过冲多少 —— 从"采样看到力过阈值"到"实际停住",有采样周期 + 读位置 + 写回
     三段延迟,期间手指还在走。多压出来的力必须量出来,否则阈值设不准。

⚠ **不放物体,用手指互顶**(拇指+食指)。可重复、不需要人在场,能回答 Q1。
   但手指互顶是**刚性接触**,实测过冲只有 5-6%,是容易的那一档;柔性物
   (海绵 13-21%)会更大。所以 Q2 在这里得到的是**乐观下界**,柔性物要另测。

对照组(不冻结)是必须的:不跟它比,就分不清"力停住了"是冻结起作用,还是这个
行程本来就到这儿为止。

用法:
  python3 src/test_force_freeze.py                 # 4 次冻结 + 4 次对照
  python3 src/test_force_freeze.py --trigger 150 --trials 3
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM_DIR))

from inspire_hand import (InspireHand, InspireHandConfig,  # noqa: E402
                          PROJECT_TO_VENDOR)

YAW, PITCH, INDEX = 0, 1, 2
NAMES = ["yaw", "pitch", "index", "middle", "ring", "pinky"]
OPEN = 1000
# 拇指先摆到的抓握位。按用户给的时序:拇指先到位,食指再去夹 ——
# 这样食指是唯一在运动的通道,力信号干净(见 hand-force-status-semantics 那条笔记)。
THUMB_YAW, THUMB_PITCH = 150, 141
TEMP_ABORT = 58          # 过温 Bit1 不可清除,只能等凉 —— 到这个温度就停,别硬跑


def to_vendor(proj):
    out = [-1] * 6
    for i, v in enumerate(proj):
        out[PROJECT_TO_VENDOR[i]] = int(v)
    return out


def one(hand, reg, nb=12, fmt="6h"):
    """单次读转项目序。闭环里**只能用单次读** —— 要求两次一致会等到运动停止,
    那就永远来不及在运动中触发冻结。读失败(偶发)返回 None,调用侧跳过这一拍。"""
    v = hand.read_regs(reg, nb, fmt)
    return None if v is None else [int(v[PROJECT_TO_VENDOR[i]]) for i in range(6)]


def stable(hand, reg, nb=12, fmt="6h", tries=16):
    prev = None
    for _ in range(tries):
        v = one(hand, reg, nb, fmt)
        if v is not None:
            if v == prev:
                return v
            prev = v
        time.sleep(0.07)
    return prev


def prep(hand, speed, force):
    """回张开位 + 拇指摆到抓握位。食指留在张开,等 trial 里再动。"""
    hand.set_force(200)
    hand.write_shorts("SPEED_SET", to_vendor([speed] * 6))
    hand.write_shorts("ANGLE_SET", to_vendor([OPEN] * 6))
    time.sleep(2.2)
    hand.write_shorts("ANGLE_SET",
                      to_vendor([THUMB_YAW, THUMB_PITCH, OPEN, OPEN, OPEN, OPEN]))
    time.sleep(2.0)
    hand.set_force(force)
    time.sleep(0.3)


def trial(hand, trigger, freeze, speed, force, tag):
    """食指合拢直到力过阈值。freeze=True 时把当前位置写回 ANGLE_SET。

    返回 dict:触发时的力/位置、冻结后的峰值和稳态、过冲量、各段延迟。
    """
    prep(hand, speed, force)
    hand.write_shorts("ANGLE_SET",
                      to_vendor([THUMB_YAW, THUMB_PITCH, 0, OPEN, OPEN, OPEN]))

    t0 = time.time()
    trig_f = trig_pos = None
    t_trig = t_frozen = None
    samples = 0
    # 尽快轮询,不 sleep —— 每次 read_regs 本身约 3ms,这就是采样上限
    while time.time() - t0 < 4.0:
        f = one(hand, "FORCE_ACT")
        samples += 1
        if f is None:
            continue
        if abs(f[INDEX]) >= trigger:
            t_trig = time.time()
            trig_f = list(f)
            if freeze:
                # 冻结:读当前实际位置,原样写回设定值。位置误差归零 → 电机不再
                # 往前推。这两步(读+写)各约 3ms,是过冲的主要来源之一。
                pos = one(hand, "ANGLE_ACT")
                if pos is not None:
                    trig_pos = list(pos)
                    hand.write_shorts("ANGLE_SET", to_vendor(pos))
                    t_frozen = time.time()
            break
    if t_trig is None:
        return {"tag": tag, "ok": False, "note": "4 秒内没到阈值"}

    # 冻结(或不冻结)之后继续采 1.8 秒,看力往哪走
    peak = abs(trig_f[INDEX])
    peak_thumb = abs(trig_f[PITCH])
    while time.time() - t_trig < 1.8:
        f = one(hand, "FORCE_ACT")
        if f is None:
            continue
        peak = max(peak, abs(f[INDEX]))
        peak_thumb = max(peak_thumb, abs(f[PITCH]))
    steady = stable(hand, "FORCE_ACT")
    end_pos = stable(hand, "ANGLE_ACT")

    hz = samples / max(1e-6, t_trig - t0)
    res = {
        "tag": tag, "ok": True, "freeze": freeze,
        "trig_force": abs(trig_f[INDEX]),
        "peak_index": peak, "peak_thumb": peak_thumb,
        "steady_index": abs(steady[INDEX]) if steady else None,
        "steady_thumb": abs(steady[PITCH]) if steady else None,
        "poll_hz": hz,
        "freeze_ms": (t_frozen - t_trig) * 1000 if t_frozen else None,
        "trig_pos_index": trig_pos[INDEX] if trig_pos else None,
        "end_pos_index": end_pos[INDEX] if end_pos else None,
    }
    if trig_pos and end_pos:
        # 位置过冲:冻结那一刻的位置 vs 最终停住的位置。raw 越小 = 越合,
        # 所以正值表示"冻结后又多合了这么多 counts"。
        res["pos_overshoot"] = trig_pos[INDEX] - end_pos[INDEX]
    return res


def show(r):
    if not r.get("ok"):
        print(f"  {r['tag']:14s} FAILED: {r.get('note')}")
        return
    ov = r["peak_index"] - r["trig_force"]
    pct = ov / max(1, r["trig_force"]) * 100
    print(f"  {r['tag']:14s} 触发{r['trig_force']:4d} → 峰值{r['peak_index']:4d} "
          f"(+{ov:4d}, {pct:5.0f}%) 稳态{r['steady_index'] or -1:4d}"
          f"  拇弯峰{r['peak_thumb']:4d}", end="")
    if r.get("freeze_ms") is not None:
        print(f"  冻结耗时{r['freeze_ms']:5.1f}ms", end="")
    if r.get("pos_overshoot") is not None:
        print(f"  位置过冲{r['pos_overshoot']:4d}counts", end="")
    print(f"  轮询{r['poll_hz']:.0f}Hz")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--trigger", type=int, default=150, help="触发冻结的力阈值")
    ap.add_argument("--speed", type=int, default=150)
    ap.add_argument("--force", type=int, default=300)
    ap.add_argument("--trials", type=int, default=4)
    ap.add_argument("--sweep-speed", action="store_true",
                    help="扫速度,找最小可达稳态力(回答'这只手能多轻')")
    args = ap.parse_args()

    hand = InspireHand(InspireHandConfig(port=args.port, mock=False))
    try:
        if not hand.connect():
            print("connect returned False")
            return 2
    except Exception as exc:                      # noqa: BLE001
        print(f"connect failed: {type(exc).__name__}: {exc}")
        return 2

    print("*** 不放物体,拇指+食指互顶 ***")
    print(f"trigger={args.trigger} speed={args.speed} force={args.force}\n")
    fz, ctl = [], []

    if args.sweep_speed:
        # 过冲的成因是"延迟期间走的距离 × 力-位移斜率"。斜率是物体性质,改不了;
        # 能改的只有距离,而距离 ∝ 速度。所以扫速度直接给出最小可达力。
        print("=== 扫速度:找最小可达稳态力")
        print(f"  {'speed':>6} {'trig':>5} {'steady':>7} {'pos_ov':>7} {'g/count':>8}")
        try:
            for sp in (300, 200, 150, 100, 50, 20):
                rows = [trial(hand, args.trigger, True, sp, args.force, f"s{sp}")
                        for _ in range(2)]
                good = [r for r in rows if r.get("ok")]
                if not good:
                    print(f"  {sp:6d}  (没到阈值)"); continue
                st = statistics.mean(r["steady_index"] for r in good
                                     if r.get("steady_index") is not None)
                tf = statistics.mean(r["trig_force"] for r in good)
                pv = [r["pos_overshoot"] for r in good
                      if r.get("pos_overshoot") is not None]
                po = statistics.mean(pv) if pv else 0
                slope = (st - tf) / po if po else float("nan")
                print(f"  {sp:6d} {tf:5.0f} {st:7.0f} {po:7.1f} {slope:8.0f}")
                t = hand.read_regs("TEMP", 6, "6B")
                if t and max(t) >= TEMP_ABORT:
                    print(f"  ! 温度 {max(t)}°C 到上限,停止"); break
        finally:
            hand.set_force(200)
            hand.write_shorts("ANGLE_SET", to_vendor([OPEN] * 6))
            time.sleep(1.5)
            hand.set_force(hand.cfg.init_force)
            hand.set_speed(hand.cfg.init_speed)
            hand.disconnect()
        print("\n→ 稳态随速度降 = 过冲由'延迟期间走的距离'主导,降速有效")
        print("  稳态不随速度降 = 已经触到别的下限(触发阈值本身/传感器噪声)")
        return 0

    try:
        t = hand.read_regs("TEMP", 6, "6B")
        print(f"start TEMP {list(t) if t else '?'}\n")

        print(f"=== A 冻结组({args.trials} 次)")
        for i in range(args.trials):
            r = trial(hand, args.trigger, True, args.speed, args.force, f"freeze#{i+1}")
            show(r)
            if r.get("ok"):
                fz.append(r)
            t = hand.read_regs("TEMP", 6, "6B")
            if t and max(t) >= TEMP_ABORT:
                print(f"  ! 温度 {max(t)}°C 到上限,停止"); break

        print(f"\n=== B 对照组:同样动作但**不冻结**({args.trials} 次)")
        print("    不跟它比就分不清'力停住了'是冻结起作用,还是行程本来到这儿为止")
        for i in range(args.trials):
            r = trial(hand, args.trigger, False, args.speed, args.force, f"ctl#{i+1}")
            show(r)
            if r.get("ok"):
                ctl.append(r)
            t = hand.read_regs("TEMP", 6, "6B")
            if t and max(t) >= TEMP_ABORT:
                print(f"  ! 温度 {max(t)}°C 到上限,停止"); break
    finally:
        hand.set_force(200)
        hand.write_shorts("ANGLE_SET", to_vendor([OPEN] * 6))
        time.sleep(1.5)
        hand.set_force(hand.cfg.init_force)
        hand.set_speed(hand.cfg.init_speed)
        t = hand.read_regs("TEMP", 6, "6B")
        print(f"\nend TEMP {list(t) if t else '?'}")
        hand.disconnect()

    print("\n" + "=" * 70)
    def avg(rows, k):
        vals = [r[k] for r in rows if r.get(k) is not None]
        return statistics.mean(vals) if vals else None
    for label, rows in (("冻结组", fz), ("对照组", ctl)):
        if not rows:
            print(f"{label}: 无有效数据"); continue
        pk, st = avg(rows, "peak_index"), avg(rows, "steady_index")
        print(f"{label}: 峰值均值 {pk:.0f} · 稳态均值 {st:.0f} · n={len(rows)}")
    if fz and ctl:
        pf, pc = avg(fz, "peak_index"), avg(ctl, "peak_index")
        sf, sc = avg(fz, "steady_index"), avg(ctl, "steady_index")
        print(f"\nQ1 机制成立吗: 冻结组峰值 {pf:.0f} vs 对照组 {pc:.0f}")
        if pf < pc * 0.85:
            print(f"   → 成立。冻结把峰值压低了 {(1-pf/pc)*100:.0f}%")
        else:
            print("   → **不成立**。冻结后力和不冻结差不多,写回位置没能让电机松劲。")
            print("      那么力闭环这条路不通,得改成限制位置行程。")
        print(f"   稳态: 冻结 {sf:.0f} vs 对照 {sc:.0f}")
        ov = avg(fz, "peak_index") - avg(fz, "trig_force")
        pos = avg(fz, "pos_overshoot")
        ms = avg(fz, "freeze_ms")
        tf = avg(fz, "trig_force")
        print(f"\nQ2 过冲: 触发 {tf:.0f}g → 稳态 {sf:.0f}g,即 +{ov:.0f}g"
              + (f" · 位置 {pos:.0f} counts" if pos is not None else "")
              + (f" · 冻结耗时 {ms:.1f}ms" if ms is not None else ""))
        if pos:
            print(f"   力-位移斜率 ≈ {ov / max(1, pos):.0f} g/count")
        # ⚠ 不要写成"阈值 = 目标力 - 过冲":目标 500g 时那是负数,无意义。
        # 真实含义是这套参数下的**最小可达稳态力**,调阈值降不下来 ——
        # 因为过冲的成因不是延迟(10ms 已经很快),是力-位移斜率太陡。
        print(f"   → **这套参数(speed={args.speed})下,刚性接触最小可达稳态 ≈ {sf:.0f}g**")
        print("      调触发阈值降不下来:延迟已经很短,瓶颈是斜率。要更轻只能降速。")
        print("   ⚠ 这是**刚性接触**(手指互顶),斜率最陡的一档。柔性物斜率平缓,")
        print("      同样的位置过冲产生的力增量小得多,要另外放物体测。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
