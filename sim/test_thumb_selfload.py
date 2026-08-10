#!/usr/bin/env python3
"""证伪/证实:拇指弯曲的力读数是不是**自身到位载荷**,与物体无关。要连真手,空手跑。

背景。拇指+食指捏物体的实测(2026-08-06):

    物体        参数            拇弯    食指
    笔(硬)   speed500/force500  1248    2270
    软       speed500/force500  1136     968
    笔(硬)   speed200/force305   375    1059
    软       speed200/force305   397     761

拇弯**随速度变、不随物体变**(高速都 1100-1250,低速都 375-397),食指两边差 2.3 倍。
用户给出的机制:操作时序是「拇指旋转到位 → 拇指弯曲到位 → 食指去夹」,所以拇指在
**接触发生之前**就走完停住了,它读到的是自己走到位那一下的载荷,不是物体反力。

这个脚本**空手**验证:只让拇指走到抓握位,食指不动。若拇弯仍读到 ~375(speed 200)
和 ~1200(speed 500),就证明那读数与物体完全无关。

顺带扫 pitch 的目标深度:若力只在接近行程末端时才起来,说明是撞自己的限位;
若各深度都有,说明是运动本身的惯性载荷。两者对包设计的含义不同 ——
前者可以靠"别命令到末端"避开,后者只能靠降速。

用法:
  python3 sim/test_thumb_selfload.py            # 手上不要有东西
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))

from inspire_hand import (InspireHand, InspireHandConfig,  # noqa: E402
                          PROJECT_TO_VENDOR)

YAW, PITCH, INDEX = 0, 1, 2
NAMES = ["yaw", "pitch", "index", "middle", "ring", "pinky"]
OPEN = 1000


def to_vendor(proj):
    out = [-1] * 6
    for i, v in enumerate(proj):
        out[PROJECT_TO_VENDOR[i]] = int(v)
    return out


def one(hand, reg, nb=12, fmt="6h"):
    """单次读,转项目序。**运动中采样用这个** —— 要求两次一致会一直等到停下,
    那就看不到过程了。读失败(约 40%)返回 None,调用侧跳过。"""
    v = hand.read_regs(reg, nb, fmt)
    return None if v is None else [int(v[PROJECT_TO_VENDOR[i]]) for i in range(6)]


def stable(hand, reg, nb=12, fmt="6h", tries=14):
    """连续两次一致才采信,用于**静止后**的终值。"""
    prev = None
    for _ in range(tries):
        v = one(hand, reg, nb, fmt)
        if v is not None:
            if v == prev:
                return v
            prev = v
        time.sleep(0.08)
    return prev


def open_hand(hand, settle=2.2):
    hand.set_force(200)
    hand.write_shorts("ANGLE_SET", to_vendor([OPEN] * 6))
    time.sleep(settle)


def move_thumb(hand, yaw_t, pitch_t, speed, force, label):
    """按用户的时序走:先 yaw 到位,再 pitch 到位。食指全程张开、不参与。
    返回 (拇旋峰值, 拇弯峰值, 食指峰值, 终位置)。"""
    open_hand(hand)
    hand.write_shorts("SPEED_SET", to_vendor([speed] * 6))
    hand.set_force(force)
    time.sleep(0.35)

    peak = [0] * 6

    def sweep(target, secs):
        hand.write_shorts("ANGLE_SET", to_vendor(target))
        t0 = time.time()
        while time.time() - t0 < secs:
            f = one(hand, "FORCE_ACT")
            if f:
                for i in range(6):
                    peak[i] = max(peak[i], abs(f[i]))
            time.sleep(0.045)

    # 第 1 步:只 yaw
    sweep([yaw_t, OPEN, OPEN, OPEN, OPEN, OPEN], 2.6)
    yaw_only = list(peak)
    # 第 2 步:再 pitch(yaw 保持)
    sweep([yaw_t, pitch_t, OPEN, OPEN, OPEN, OPEN], 2.6)

    act = stable(hand, "ANGLE_ACT")
    print(f"  {label}")
    print(f"    peak after yaw only : yaw={yaw_only[YAW]:5d} pitch={yaw_only[PITCH]:5d}")
    print(f"    peak after +pitch   : yaw={peak[YAW]:5d} pitch={peak[PITCH]:5d} "
          f"index={peak[INDEX]:4d}")
    if act:
        print(f"    final pos           : yaw={act[YAW]:4d} pitch={act[PITCH]:4d} "
              f"(cmd yaw={yaw_t} pitch={pitch_t})")
    return peak, act


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    args = ap.parse_args()

    hand = InspireHand(InspireHandConfig(port=args.port, mock=False))
    try:
        if not hand.connect():
            print("connect returned False")
            return 2
    except Exception as exc:                      # noqa: BLE001
        print(f"connect failed: {type(exc).__name__}: {exc}")
        return 2

    print("*** 手上不要有任何东西 ***\n")
    rows = []
    try:
        t = hand.read_regs("TEMP", 6, "6B")
        print(f"start TEMP {list(t) if t else '(read failed)'}\n")

        # A/B: 复现用户那两组参数,空手。拇指走到深屈位(pitch raw 141 = URDF 上限 0.6rad)
        print("=== A 空手,拇指走到深屈位(pitch=141,即 rad 路径能到的最深)")
        for speed, force in ((200, 305), (500, 500)):
            p, a = move_thumb(hand, 150, 141, speed, force,
                              f"speed={speed} force={force}")
            rows.append((f"deep s{speed}", p[PITCH]))
            print()

        # C: 扫 pitch 深度,固定低速。若力只在接近末端才起来 → 撞自己的限位;
        #    若各深度都有 → 是运动惯性载荷。含义不同。
        print("=== C 空手,扫 pitch 目标深度(speed=200 force=305)")
        print(f"  {'pitch_cmd':>10} {'pitch_act':>10} {'peak_pitch':>11}")
        for pt in (700, 500, 300, 141):
            p, a = move_thumb(hand, 150, pt, 200, 305, f"pitch_cmd={pt}")
            rows.append((f"pt{pt}", p[PITCH]))
            print()
    finally:
        open_hand(hand, settle=1.5)
        hand.set_force(hand.cfg.init_force)
        hand.set_speed(hand.cfg.init_speed)
        t = hand.read_regs("TEMP", 6, "6B")
        print(f"end TEMP {list(t) if t else '(read failed)'}")
        hand.disconnect()

    print("\n" + "=" * 62)
    print("空手拇弯峰值汇总:")
    for k, v in rows:
        print(f"  {k:12s} {v:5d}")
    print("\n对照真机捏物体实测: 低速(200/305) 375-397 · 高速(500/500) 1136-1248")
    mx = max(v for _, v in rows)
    # 判据写在这里,不预先写死结论 —— 第一版这里直接印"是自身载荷",而实测数据
    # 正好相反(空手 6-11,捏物体 375+)。把结论烤进脚本会让人照着读错。
    if mx < 100:
        print(f"空手最大只有 {mx} → **不是**自身到位载荷,捏物体时读到的是真实传导力。")
    else:
        print(f"空手就有 {mx} → 那读数含自身到位载荷,判据要扣掉这部分。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
