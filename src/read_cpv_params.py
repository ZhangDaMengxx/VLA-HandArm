#!/usr/bin/env python3
"""读 CPV 的伺服参数(加减速、轮廓速度、PID 增益)。**只读,但会发查询帧。**

## 为什么要读这个

`move_cpv_pos` 是逐关节位置伺服 + 梯形轮廓。轮廓长什么样由这几个参数决定:
  `ac` 加速度  `dc` 减速度  `vv` 轮廓速度  `pp` 位置环Kp  `kp`/`ki` 速度环Kp/Ki
30fps 逐帧重设目标时,臂跟得多紧、变形多少、滞后多少,全看这些值。
不知道它们就没法预测跟随表现,也没法判断要不要调。

## ⚠ 一个坑:读参数**可能会切模式**

SDK 的 `_get_cpv()` 里有 `self._maybe_set_motion_mode('cpv')` —— 它在
`_auto_set_motion_mode_enabled`(默认 **True**)打开时会真的发一次 `set_motion_mode`。
那不是只读:臂已使能时切进 CPV 模式,如果 CPV 的目标位置寄存器里存着**旧值**,
臂可能立刻朝那个旧目标动。

所以这个脚本:
  1. 先从 `0x2A1` 读 `mode_feedback`,记下**当前**模式
  2. `set_auto_set_motion_mode_enabled(False)` —— 关掉自动切
  3. 读参数
  4. 再读一次模式核对,**变了就报**

代价是关掉自动切之后,臂如果不在 CPV 模式,查询帧可能拿不到回应 ——
那种情况下会超时返回 None。拿不到也是结论:说明必须切模式才能读,
那就需要单独授权(`--allow-mode-switch`)。

发的是 `mode='r'` 查询帧,不写任何参数、不含位置指令。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

SIM = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM))

TYPES = [("ac", "加速度"), ("dc", "减速度"), ("vv", "轮廓速度"),
         ("pp", "位置环 Kp"), ("kp", "速度环 Kp"), ("ki", "速度环 Ki"),
         ("po", "当前位置目标"), ("sp", "当前速度目标")]


def read_mode(robot, wait: float = 1.0):
    """从 0x2A1 读当前控制模式和运动模式。纯读缓存,不发帧。"""
    deadline = time.monotonic() + wait
    while time.monotonic() < deadline:
        st = robot.get_arm_status()
        m = getattr(st, "msg", None)
        if m is not None:
            return {
                "ctrl_mode": str(getattr(m, "ctrl_mode", "?")),
                "mode_feedback": str(getattr(m, "mode_feedback", "?")),
                "motion_status": str(getattr(m, "motion_status", "?")),
            }
        time.sleep(0.05)
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--joints", default="1",
                    help="逗号分隔的关节号,默认只读关节1(先看一个够不够)")
    ap.add_argument("--timeout", type=float, default=1.0, help="每个参数等回应的秒数")
    ap.add_argument("--allow-mode-switch", action="store_true",
                    help="允许 SDK 自动切进 CPV 模式。⚠ 臂已使能时切模式可能立刻产生"
                         "运动(CPV 目标寄存器里可能存着旧值)。默认关")
    a = ap.parse_args()

    from nero_arm import NeroArm
    arm = NeroArm(mock=False, channel=a.channel, firmware="auto")
    arm.connect()
    r = arm.robot
    print(f"driver          : {arm.firmware_detected}")
    if arm.firmware_detected not in ("v112", "v120"):
        print(f"⚠ {arm.firmware_detected} 没有 CPV,读不了。需要固件 ≥1.12",
              file=sys.stderr)
        arm.disconnect()
        return 2

    m0 = read_mode(r)
    print(f"当前模式(读前)  : {m0}")
    print(f"使能状态        : {arm.enabled}")
    print(f"关节角(读前)    : {[round(v,4) for v in arm.read_angles()]}")

    if not a.allow_mode_switch:
        r.set_auto_set_motion_mode_enabled(False)
        print("\n已关闭 SDK 自动切模式 —— 读参数不会改臂的模式。")
        print("代价:臂不在 CPV 模式时查询帧可能没回应,那样会读到 None。")
    else:
        print("\n⚠ 允许自动切模式:SDK 会发 set_motion_mode('cpv')。")

    print(f"\n{'关节':<6}{'参数':<6}{'含义':<12}{'值':>14}")
    got = 0
    for j in [int(x) for x in a.joints.split(",")]:
        for t, label in TYPES:
            try:
                v = r._get_cpv(joint_index=j, type_=t, timeout=a.timeout,
                               min_interval=0.0)
            except Exception as e:                  # noqa: BLE001
                v = f"异常: {type(e).__name__}"
            if isinstance(v, (int, float)):
                got += 1
            print(f"joint{j:<1}{'':<1}{t:<6}{label:<12}{v if v is not None else '—— 无回应':>14}")

    m1 = read_mode(r)
    print(f"\n当前模式(读后)  : {m1}")
    if m0 and m1 and m0 != m1:
        print("⚠⚠ **模式变了** —— 读参数改了臂的状态。对比:", file=sys.stderr)
        for k in m0:
            if m0[k] != m1[k]:
                print(f"     {k}: {m0[k]} → {m1[k]}", file=sys.stderr)
    else:
        print("模式未变 ✓")
    ang = arm.read_angles()
    print(f"关节角(读后)    : {[round(v,4) for v in ang]}")
    if arm.connect_pose:
        d = max(abs(x - y) for x, y in zip(ang, arm.connect_pose))
        print(f"关节角偏移      : {d:.4f} rad = {d*57.2958:.3f}°  "
              f"{'(没动)' if d < 0.01 else '⚠ 动了!'}")
    if got == 0:
        print("\n一个参数都没读到。可能需要臂先进 CPV 模式 —— "
              "那要 --allow-mode-switch,并且先确认下方净空。")
    arm.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
