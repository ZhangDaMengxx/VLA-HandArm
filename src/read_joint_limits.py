#!/usr/bin/env python3
"""读**关节级**的角度/速度/加速度限制。只读,但会发查询帧。

## 为什么要这个(read_cpv_params.py 已经读了 CPV 那层)

限制是**两层**的:

    电机物理能力              ← 手册没写,读不到
      └─ 关节级限制           ← 本脚本:get_joint_angle_vel_limits / get_joint_acc_limits
           └─ CPV 轮廓参数    ← read_cpv_params.py:vv / ac / dc

松灵手册只给了**关节最大角速度**(J1-J3 180°/s、J4-J7 225°/s),`ac`/`dc` 的额定值
没有。但臂自己能报 —— 底层是发 `0x472`(`search_content` 0x01 查角度/最大速度、
0x02 查最大加速度),臂回 `0x47C`。

要回答的就一件事:**CPV 默认的 ac=143.2°/s² 上面还有多少余量。**
  · 关节级 > CPV 的 ac  → CPV 那个是保守的轮廓设置,可以往上调
  · 两个一样            → 143.2 就是天花板,只能靠 retime 拉慢

这个数决定了录制素材要放慢多少倍(retime 拉长 k 倍,加速度降 **k²** 倍)。

## ⚠ 读回来的是"当前设置",不是物理上限

`set_joint_acc_limits()` 存在,所以关节级限制本身也是可写的 —— `0x47C` 的名字就是
"反馈**当前**电机最大加速度限制"。真正的物理能力仍然读不到(需要转子惯量和减速比)。

顺带:`set_joint_acc_limits` 的编码是 `round(abs(v) * 1e4)`,而 `0x7FFF` 被当成
"不设置"的哨兵。如果字段是 16 位,可写上限就是 `0x7FFF/1e4 = 3.2767 rad/s²`
≈ **187.7°/s²**。这是从**编码宽度**推的,不是文档说的 —— 读出来的实际值可能推翻它。

## 安全

**只发查询帧,不发任何运动指令,不写任何参数。**
和 read_cpv_params.py 同一风险等级(都往总线发读请求)。

不需要关 auto_set_motion_mode —— 这两个查询走的是 `_request_and_get`,里面没有
`_maybe_set_motion_mode`(那是 `_get_cpv` 才有的)。所以读它们不会切模式。
本脚本仍然会前后各读一次模式核对,读到变化就报 —— 不靠"我以为不会"当保证。

    python3 src/read_joint_limits.py
    python3 src/read_joint_limits.py --channel can1
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

SIM = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM))

RAD2DEG = 57.29577951308232

# 手册给的额定最大角速度(deg/s),2026-08-03。并排打出来对照读回来的值。
RATED_SPD_DEG = [180.0, 180.0, 180.0, 225.0, 225.0, 225.0, 225.0]
# CPV 轮廓的出厂默认 ac(deg/s²),read_cpv_params.py 从真臂读的。
# 关节级读回来的值要和它比 —— 这就是本脚本要回答的那个问题。
CPV_AC_DEG = [114.6, 114.6, 143.2, 143.2, 143.2, 143.2, 143.2]


def _f(v, nd: int = 3) -> str:
    """None 打成 —— ,数字按 nd 位小数。读不到和读到 0 是两件事,不能都显示 0。"""
    return "——" if v is None else f"{v:.{nd}f}"


def read_mode(robot, wait: float = 1.0):
    """从 0x2A1 读当前控制模式和运动模式。纯读缓存,不发帧。

    复制自 read_cpv_params.py —— 前后各读一次,变了就报。
    """
    import time
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
    ap.add_argument("--timeout", type=float, default=1.5,
                    help="每个查询等回应的秒数(两次查询都用它)")
    a = ap.parse_args()

    from nero_arm import NeroArm
    arm = NeroArm(mock=False, channel=a.channel, firmware="auto")
    arm.connect()
    r = arm.robot
    print(f"driver          : {arm.firmware_detected}")
    if arm.firmware_detected not in ("v112", "v120"):
        print(f"⚠ {arm.firmware_detected} 不支持关节级限制查询(0x472/0x47C),"
              f"需要固件 ≥1.12", file=sys.stderr)
        arm.disconnect()
        return 2

    m0 = read_mode(r)
    print(f"当前模式(读前)  : {m0}")
    print(f"使能状态        : {arm.enabled}")
    ang0 = arm.read_angles()
    print(f"关节角(读前)    : {[round(v, 4) for v in ang0]}")
    print()

    # 1. 角度/速度限制(0x472 search_content=0x01 → 0x473)
    print("=== 关节角度 + 最大速度限制(search_content=0x01,CAN ID 0x473)===")
    # ⚠ 字段只有三个:max_angle_limit / min_angle_limit / max_joint_spd。
    # SDK 的 get_joint_angle_vel_limits docstring 里还列了个 `min_joint_spd`,
    # **消息类上没有那个属性** —— 照 docstring 写会 AttributeError(实测踩到)。
    # docstring 和消息类是两份,这里以消息类为准。
    print(f"{'joint':<7}{'min_rad':>9}{'max_rad':>9}{'范围°':>10}"
          f"{'max_spd':>10}{'°/s':>9}{'额定°/s':>9}")
    av_ok = 0
    for j in range(1, 8):
        try:
            res = r.get_joint_angle_vel_limits(joint_index=j, timeout=a.timeout)
        except Exception as e:                      # noqa: BLE001
            print(f"joint{j}  异常: {type(e).__name__} {e}")
            continue
        if res is None or res.msg is None:
            print(f"joint{j}  —— 无回应(超时 {a.timeout}s)")
            continue
        m = res.msg
        lo, hi, spd = m.min_angle_limit, m.max_angle_limit, m.max_joint_spd
        rng = (hi - lo) * RAD2DEG if (lo is not None and hi is not None) else None
        spd_deg = spd * RAD2DEG if spd is not None else None
        # 手册给的额定值,并排列出来对照 —— 读回来的和手册不一致本身就是结论
        rated = RATED_SPD_DEG[j - 1]
        print(f"joint{j}  {_f(lo, 4):>9}{_f(hi, 4):>9}{_f(rng, 1):>10}"
              f"{_f(spd, 4):>10}{_f(spd_deg, 1):>9}{rated:>9.0f}"
              + ("" if spd_deg is None or abs(spd_deg - rated) < 1.0
                 else f"   ⚠ 和手册差 {spd_deg - rated:+.1f}"))
        av_ok += 1

    # 2. 加速度限制(0x472 search_content=0x02 → 0x47C)
    print()
    print("=== 关节最大加速度限制(search_content=0x02,CAN ID 0x47C)===")
    print(f"{'joint':<7}{'rad/s²':>10}{'°/s²':>10}{'CPV的ac':>10}{'余量':>9}")
    ac_ok = 0
    accs: list[float | None] = []
    for j in range(1, 8):
        try:
            res = r.get_joint_acc_limits(joint_index=j, timeout=a.timeout)
        except Exception as e:                      # noqa: BLE001
            print(f"joint{j}  异常: {type(e).__name__} {e}")
            continue
        if res is None or res.msg is None:
            print(f"joint{j}  —— 无回应(超时 {a.timeout}s)")
            accs.append(None)
            continue
        acc = res.msg.max_joint_acc
        acc_deg = acc * RAD2DEG if acc is not None else None
        accs.append(acc_deg)
        cpv = CPV_AC_DEG[j - 1]
        # 余量 = 关节级 / CPV轮廓级。>1 说明 CPV 那个是保守设置,还能往上调。
        head = None if acc_deg is None else acc_deg / cpv
        print(f"joint{j}  {_f(acc, 4):>10}{_f(acc_deg, 1):>10}{cpv:>10.1f}"
              f"{('——' if head is None else f'{head:.2f}×'):>9}")
        ac_ok += 1

    # ---- 结论:CPV 的 ac 上面还有多少余量 ----
    got = [x for x in accs if x is not None]
    if got:
        ratios = [x / CPV_AC_DEG[i] for i, x in enumerate(accs) if x is not None]
        lo_r, hi_r = min(ratios), max(ratios)
        print()
        print("=== 结论 ===")
        if hi_r < 1.05:
            print(f"关节级加速度 ≈ CPV 的 ac(余量 {lo_r:.2f}~{hi_r:.2f}×)")
            print("  → **143.2°/s² 就是天花板**,提 CPV 的 ac 没用。")
            print("     素材超默认多少倍,就得 retime 拉长 √那么多倍(拉 k 倍降 k² 加速度)。")
        else:
            print(f"关节级加速度是 CPV 的 ac 的 {lo_r:.2f}~{hi_r:.2f}×")
            print(f"  → CPV 的 ac 是**保守的轮廓设置**,上面有余量。")
            print(f"     set_cpv_acc() 可以往上调到关节级那个值。")
            print(f"     ⚠ 但那是改臂的参数 —— 要先确认是否断电保存、"
                  f"会不会影响松灵客户端。")
        # p95 加速度需求是默认的 2.376 倍(prep 报告),换算成需要的 retime 倍数
        need_ratio = 2.376 / hi_r
        if need_ratio > 1.0:
            print(f"\n你的素材 p95 加速度是 CPV 默认的 2.376× —— "
                  f"用上全部余量后还差 {need_ratio:.2f}×,")
            print(f"  需要 retime ×{need_ratio ** 0.5:.2f} 才落进去。")
        else:
            print(f"\n你的素材 p95 加速度是 CPV 默认的 2.376× —— "
                  f"调到关节级上限后**装得下**,不需要 retime。")

    print()
    m1 = read_mode(r)
    print(f"当前模式(读后)  : {m1}")
    if m0 and m1 and m0 != m1:
        print("⚠⚠ **模式变了** —— 查询改了臂的状态。对比:", file=sys.stderr)
        for k in m0:
            if m0[k] != m1[k]:
                print(f"     {k}: {m0[k]} → {m1[k]}", file=sys.stderr)
    else:
        print("模式未变 ✓")

    ang1 = arm.read_angles()
    print(f"关节角(读后)    : {[round(v, 4) for v in ang1]}")
    if arm.connect_pose:
        d = max(abs(x - y) for x, y in zip(ang1, arm.connect_pose))
        print(f"关节角偏移      : {d:.4f} rad = {d * RAD2DEG:.3f}°  "
              f"{'(没动)' if d < 0.01 else '⚠ 动了!'}")

    if av_ok == 0 and ac_ok == 0:
        print("\n一个回应都没拿到 —— 总线可能不通,或者固件不支持这两个查询。")
    elif av_ok < 7 or ac_ok < 7:
        print(f"\n部分关节没回应(角度速度 {av_ok}/7、加速度 {ac_ok}/7)。"
              f"timeout={a.timeout}s 可能不够,试 --timeout 3.0")

    arm.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
