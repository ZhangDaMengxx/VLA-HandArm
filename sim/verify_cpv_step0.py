#!/usr/bin/env python3
"""第 0 步:验 `move_cpv_pos` 到底能不能驱动臂。**单关节、小幅、可中断。**

## 为什么需要这个脚本

`combo_player.py` 整套设计压在一个假设上:**CPV 位置指令真的能驱动臂**。
这个假设到今天为止**一帧都没在真臂上发过**。SDK 不会在 vcan0 上发东西
(`connect()` 读不到关节角就抛),所以只能上真臂,没有干跑这条路。

而**不能直接拿整条轨迹去验**。仿真已经算出来素材需求超 CPV 轮廓上限
(p95 加速度 2.376×),真跑会看到 25–47° 的跟踪误差 —— 那时候分不清是
「CPV 通路不通」还是「通路是通的但伺服饱和了」。**先把通路单独验干净。**

## 两段,分开验两件事

  A 段 · 单点阶跃 · 不流   → 验「CPV 能不能驱动臂」
  B 段 · 30fps 正弦 · 流   → 验「CPV 能不能吃下 30fps 的流」

A 段失败就不跑 B 段 —— B 段的任何结论都要以 A 段成立为前提。

A 段顺带把测的东西定死了:5° 阶跃在 ac=114.6°/s² 下是**加速度受限**的三角
轮廓(峰值速度 23.9°/s < vv 36.0,压根到不了 vv),所以 A 段量的是 `ac` 那条
路径,不是 `vv`。

## 只发 joint1(默认),但**七个关节都会收到指令**

`move_cpv_pos` 一次发 7 帧,这是回放时的真实路径,所以这里不加 mask ——
加了就不是在测被用到的那条路。另外六个关节收到的是「停在 t0 读到的位置」。
所以「只有 joint1 动」是一句**关于指令的话**;别的关节要是动了,那是结论
之一,不是噪声。

## 安全

  · 默认 `--mock`。真机要 `--no-mock` **加** `--yes`。
  · **不自动使能**。臂没使能就拒跑并说清楚 —— 替人使能是个会让人吃惊的状态变更。
  · 硬前提 `ctrl_mode == CAN_CTRL` 先查再跑(松灵客户端把臂留在 ETHERNET
    模式时,CPV 帧**不报错也不动**)。
  · joint1 绕竖直轴 → **重力不产生力矩**,失去伺服也不会掉。但整条臂会横扫:
    半径 r 处扫过 r·θ,0.6m 处 5° ≈ 52mm、10° ≈ 105mm。净空看**水平**方向。
  · Ctrl-C 立刻停发并 `cpv_end()`,**不补发回程** —— 中断时再发运动更糟。
  · 每段都以「回到 t0 位姿」收尾,正弦跑整数个周期所以自然回到起点。
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time
from pathlib import Path

SIM = Path(__file__).resolve().parent
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))
from nero_arm import (ARM_JOINTS, NERO_ARM_LIMITS, NERO_CPV_AC_DEG,   # noqa: E402
                      NERO_CPV_VV_DEG, NeroArm)

D = 180.0 / math.pi          # rad → deg
OUT = SIM / "out" / "cpv_step0"

# A 段判据:阶跃后残差。0.5° 和 combo_player.APPROACH_TOL_RAD 同一个数,
# 故意的 —— 回放前的到位判据就是这个,这里先证明它在真臂上是能达到的。
STEP_TOL_DEG = 0.5
# A 段「有没有动」的判据。摆脱不了读数噪声,所以不能用 0:
# 取 0.5° = 判据本身,意思是「至少动到能被判据分辨的程度」。
MOVED_MIN_DEG = 0.5

# B 段幅度比判据。伺服滞后不扣分(下面单独量 lag),但**幅度掉**说明跟不上。
AMP_RATIO_MIN = 0.80


def pct(xs: list[float], q: float) -> float:
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[min(len(s) - 1, max(0, int(round(q / 100 * (len(s) - 1)))))]


class Trace:
    """一段采样。存**七个关节全量**,不只 joint1 ——「别的关节没动」是要证明的
    结论之一,不存就证不了。

    ⚠ `read_angles()` 读的是 SDK parser 缓存,不是同步问一次臂。关节角帧实测
    222Hz(4.5ms 一帧),所以每个 meas 最多陈旧 4.5ms,再叠 usbip 的批处理延迟
    (实测批间隔 1.87ms、一批中位 10 帧)。**这是测量地板,不是伺服滞后。**
    下面报 lag 时会把它讲清楚,别把 4.5ms 当成臂的问题。
    """

    def __init__(self, tag: str) -> None:
        self.tag = tag
        self.t: list[float] = []                 # 相对本段起点的秒
        self.cmd: list[list[float]] = []         # 下发值(rad,7)
        self.meas: list[list[float]] = []        # 读回值(rad,7)
        self.sent_ok: list[bool] = []
        self.send_gap_ms: list[float] = []       # 相邻两次发送的实际间隔

    def add(self, t: float, cmd: list[float], meas: list[float], ok: bool) -> None:
        if self.t:
            self.send_gap_ms.append((t - self.t[-1]) * 1e3)
        self.t.append(t)
        self.cmd.append(list(cmd))
        self.meas.append(list(meas))
        self.sent_ok.append(ok)

    def col(self, j: int, which: str) -> list[float]:
        src = self.cmd if which == "cmd" else self.meas
        return [r[j] for r in src]

    @property
    def fails(self) -> int:
        return sum(1 for k in self.sent_ok if not k)


def run_segment(arm: NeroArm, tag: str, secs: float, hz: float,
                target_at) -> Trace:
    """按 hz 跑 secs 秒:每拍算目标 → 发 → 读。`target_at(t)` 回 7 个 rad。

    ⚠ 节拍用**绝对时刻**排(t0 + i/hz),不累加 sleep。累加会漂:600 帧 30fps
    实测漂 199.7ms —— 而这个脚本的整个意义就是量时序,自己先漂掉就没得测了。
    (和 combo_player 里「按 t_ns 定位,不累加」是同一条纪律。)
    """
    tr = Trace(tag)
    period = 1.0 / hz
    t0 = time.monotonic()
    i = 0
    while True:
        t = i * period
        if t > secs:
            break
        due = t0 + t
        now = time.monotonic()
        if due > now:
            time.sleep(due - now)
        cmd = target_at(t)
        ok = arm.move_cpv_pos(cmd)
        # 发完立刻读 —— 读到的是**上一拍**的响应,这是无法避免的(读回来的时候
        # 这一拍的指令还在路上)。分析 lag 时按整数拍位移搜,所以这个固定的
        # 一拍偏置会被吸收进 lag 里,不影响幅度和残差。
        tr.add(time.monotonic() - t0, cmd, arm.read_angles(), ok)
        i += 1
    return tr


def bad(msg: str, judge: bool) -> None:
    """报一条不合判据的观测。

    ⚠ `judge=False`(mock)时**不打 ✗**。mock 的读数在目标位附近摆 ±6.9°,
    必然不满足 0.5° 的稳态窗 —— 那不是失败,是 mock 没有被测的那个东西。
    第一版没分这两种,于是默认调用(mock)印出一堆 ✗ 并 exit 1,看着像
    「CPV 验证失败了」,而实际一帧都没上真臂。
    """
    print(("    ✗ " if judge else "    · (mock 不判) ") + msg, flush=True)


def stage_a(arm: NeroArm, base: list[float], j: int, amp_deg: float,
            hold: float, judge: bool) -> tuple[Trace, Trace, bool]:
    """A 段:阶跃到 base+amp,停住;再阶跃回 base,停住。

    两次阶跃是**两个独立证据**,不是一次运动的往返:去程能动、回程也能动,
    才排除掉「臂正好在往那个方向漂」这种解释。

    ⚠ 阶跃的意思是**指令**是阶跃 —— 臂不会阶跃地动,CPV 的梯形轮廓会把它
    走成 ac 限制下的三角/梯形。5° 在 114.6°/s² 下约 0.42s 走完(峰值速度
    23.9°/s,到不了 vv=36),所以 hold 给 1.5s 是宽裕的。
    """
    amp = amp_deg / D
    up = list(base)
    up[j] = base[j] + amp
    print(f"\n--- A 段 · 阶跃 {amp_deg:+.1f}° · 保持 {hold:.1f}s ---")
    ta = run_segment(arm, "A1_step_up", hold, 30.0, lambda _t: up)
    print(f"--- A 段 · 阶跃回 0 · 保持 {hold:.1f}s ---")
    tb = run_segment(arm, "A2_step_back", hold, 30.0, lambda _t: list(base))

    ok, evidence = True, 0
    for tr, want in ((ta, up[j]), (tb, base[j])):
        # 末段 20% 的样本当"稳态"。取末段而不是最后一帧:最后一帧撞上读数噪声
        # 就会把结论翻过来,而 20% 有几十个样本。
        tail = tr.col(j, "meas")[max(1, int(len(tr.t) * 0.8)):]
        res = [(v - want) * D for v in tail]
        span = abs(tr.col(j, "meas")[0] - want) * D   # 这段要走的距离
        rms = math.sqrt(statistics.fmean(v * v for v in res))
        print(f"  {tr.tag}: 需走 {span:6.2f}°  "
              f"稳态残差 rms {rms:5.2f}° / max {max(abs(v) for v in res):5.2f}°  "
              f"发送失败 {tr.fails}/{len(tr.t)}")
        if tr.fails:
            # 发送失败**在 mock 下也算失败** —— move_cpv_pos 的 mock 分支
            # 是直接 return True 的,它报 False 只可能是限位/急停,那是真问题。
            bad(f"有 {tr.fails} 帧没发出去", True)
            ok = False
        if rms > STEP_TOL_DEG:
            bad(f"没收敛到 {STEP_TOL_DEG}° 内", judge)
            if judge:
                ok = False
        elif span >= MOVED_MIN_DEG:
            # 需走的距离够大 **且** 收敛了 = 真的走过去了。
            evidence += 1
        else:
            print(f"    · 起点已在目标 {MOVED_MIN_DEG}° 内 —— 这段收敛不构成"
                  f"「动了」的证据")
    if judge and ok and not evidence:
        # 两段都"收敛"但都没需要走 → 臂压根没动,而残差判据全过。
        # 不拦这种情况的话 A 段会给出一个**假的通过**。
        bad("两段都没有实际位移 —— 残差小只是因为臂没动过", True)
        ok = False
    return ta, tb, ok


def stage_b(arm: NeroArm, base: list[float], j: int, amp_deg: float,
            freq: float, cycles: float, fps: float) -> Trace:
    """B 段:30fps 正弦流。

    为什么正弦不用斜坡:斜坡的折角处伺服会滞后,那个滞后混在跟踪误差里就分不出
    「流跟不上」和「折角本来就跟不上」。正弦光滑,而且峰值速度/加速度**有闭式解**,
    能事先算准落在轮廓上限的哪个位置。

    用 sin 不用 cos:sin(0)=0,起点就是 base,**不产生入场跳变**。跑整数个周期
    也自然回到 base,收尾不用补发回程。
    """
    amp = amp_deg / D
    w = 2.0 * math.pi * freq
    secs = cycles / freq
    v_max = amp * w * D                  # A·ω
    a_max = amp * w * w * D              # A·ω²
    print(f"\n--- B 段 · {fps:g}fps 正弦 · {amp_deg:.1f}° @ {freq:g}Hz × "
          f"{cycles:g} 周期 = {secs:.1f}s ---")
    print(f"  理论峰值: 速度 {v_max:6.2f}°/s ({v_max/NERO_CPV_VV_DEG[j]:.2f}× vv)"
          f"   加速度 {a_max:6.2f}°/s² ({a_max/NERO_CPV_AC_DEG[j]:.2f}× ac)")

    def target(t: float) -> list[float]:
        q = list(base)
        q[j] = base[j] + amp * math.sin(w * t)
        return q

    return run_segment(arm, "B_sine", secs, fps, target)


def analyze_b(tr: Trace, j: int, fps: float, freq: float, judge: bool) -> bool:
    """B 段分析:滞后、幅度、其它关节漂移、节拍。返回是否通过。"""
    n = len(tr.t)
    skip = min(n // 4, int(fps / max(freq, 1e-6) / 2))   # 跳掉入场半周期
    cmd, meas = tr.col(j, "cmd")[skip:], tr.col(j, "meas")[skip:]
    if len(cmd) < 8:
        bad(f"样本只有 {len(cmd)} 个,没法分析(--cycles/--fps 太小?)", True)
        return False

    # --- 滞后:按整数拍位移搜 rms 最小 ---
    # ⚠ 滞后**本身不是缺陷**。位置伺服必然滞后,而且这里还叠着测量地板
    # (parser 缓存 4.5ms + usbip 批处理)。要判的是**幅度有没有掉** ——
    # 那才是"跟不上"。所以 lag 只报不判。
    max_k = min(len(cmd) - 4, int(fps))          # 最多搜 1 秒
    best_k, best = 0, float("inf")
    for k in range(0, max(1, max_k)):
        m = len(cmd) - k
        r = math.sqrt(statistics.fmean(
            ((cmd[i] - meas[i + k]) * D) ** 2 for i in range(m)))
        if r < best:
            best_k, best = k, r
    raw = math.sqrt(statistics.fmean(
        ((cmd[i] - meas[i]) * D) ** 2 for i in range(len(cmd))))

    amp_cmd = (max(cmd) - min(cmd)) * D
    amp_meas = (max(meas) - min(meas)) * D
    ratio = amp_meas / amp_cmd if amp_cmd > 1e-9 else float("nan")
    print(f"  {ARM_JOINTS[j]}: 峰峰 指令 {amp_cmd:.2f}° / 实测 {amp_meas:.2f}°"
          f"  = {ratio:.3f}×")
    print(f"    跟踪误差 rms: 对齐前 {raw:5.2f}°   扣掉滞后后 {best:5.2f}°")
    print(f"    滞后 {best_k} 拍 = {best_k / fps * 1e3:.1f}ms"
          f"  (含测量地板 ~4.5ms,不计入判据)")

    ok = True
    if tr.fails:
        bad(f"发送失败 {tr.fails}/{n} 帧", True)     # 同 A 段:mock 下也算失败
        ok = False
    if not (ratio >= AMP_RATIO_MIN):
        # ⚠ `not (x >= k)` 不是 `x < k` —— ratio 可能是 nan(指令幅度为 0 时),
        # 而 nan < k 是 False,会**静默通过**。
        bad(f"幅度只剩 {ratio:.3f}× (判据 ≥{AMP_RATIO_MIN}) —— 跟不上", judge)
        if judge:
            ok = False
    # ⚠ 不写 `ok and _report_others(...)` —— 短路会在已经失败时**跳过**其它关节
    # 和节拍的诊断,而那正是失败时最该看的两项。先都跑完,再合并结论。
    others_ok = _report_others(tr, j, judge)
    tick_ok = _report_tick(tr, fps)
    return ok and others_ok and tick_ok


def _report_others(tr: Trace, j: int, judge: bool) -> bool:
    """另外六个关节动了多少。

    它们收到的指令是「停在 base」,所以峰峰值就是**指令没要求的运动**:
    可能是读数噪声,也可能是 joint1 转起来带的机械耦合/整机晃动。分不开,
    但先量出来 —— 量级本身能说明是哪一类。
    """
    moved = []
    for i in range(7):
        if i == j:
            continue
        c = tr.col(i, "meas")
        moved.append((i, (max(c) - min(c)) * D))
    worst_i, worst = max(moved, key=lambda p: p[1])
    print(f"    其它关节峰峰(指令都是「不动」): 最大 {ARM_JOINTS[worst_i]} "
          f"{worst:.2f}°  中位 {statistics.median([m for _, m in moved]):.2f}°")
    if worst > 2.0 and judge:
        # 只警告不判失败:这不是 CPV 通路的结论,而且真臂上的整机晃动是真实存在的。
        # `and judge`:mock 下这七个关节**全都**在摆 ±6.9°(峰峰 13.8°),
        # 必然触发 —— 印出来只是噪声,还会让人以为 mock 里发现了什么。
        print(f"      ⚠ {worst:.2f}° 不像纯噪声 —— 记下来,不计入判据")
    return True


def _report_tick(tr: Trace, fps: float) -> bool:
    """节拍:我们**自己**有没有按 fps 发出去。和臂无关,是本进程的调度问题。"""
    g = tr.send_gap_ms
    if not g:
        return True
    want = 1e3 / fps
    print(f"    发送节拍: 目标 {want:.1f}ms  中位 {statistics.median(g):.1f}  "
          f"p95 {pct(g, 95):.1f}  max {max(g):.1f}ms")
    if max(g) > want * 3:
        # ⚠ 判据看 max 不只看 p95:单次长停顿对回放是致命的(臂停在那儿),
        # 而 p95 会把它平均掉。measure_cpv_jitter 里踩过这个漏判。
        # 这条**mock 下也判**:节拍是本进程自己的调度,和臂无关,mock 里量的
        # 是同一个东西。
        bad(f"有一拍拖到 {max(g):.1f}ms(>3× 目标)—— 本进程调度跟不上", True)
        return False
    return True


def dump(traces: list[Trace], tag: str) -> Path:
    """原始采样落盘。**在真臂上跑一次的机会不便宜**,别只留终端输出 ——
    终端会滚掉,而重跑要重新占用臂和人。"""
    OUT.mkdir(parents=True, exist_ok=True)
    p = OUT / f"{tag}.csv"
    with p.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seg", "t_s", "sent_ok"]
                   + [f"cmd_{n}" for n in ARM_JOINTS]
                   + [f"meas_{n}" for n in ARM_JOINTS])
        for tr in traces:
            for i in range(len(tr.t)):
                w.writerow([tr.tag, f"{tr.t[i]:.6f}", int(tr.sent_ok[i])]
                           + [f"{v:.6f}" for v in tr.cmd[i]]
                           + [f"{v:.6f}" for v in tr.meas[i]])
    return p


def preflight(arm: NeroArm, base: list[float], j: int,
              amp_deg: float) -> str | None:
    """跑之前必须成立的条件。回错误字符串,None = 通过。"""
    if not arm.mock:
        mode = arm.read_ctrl_mode()
        if mode != "CAN_CTRL":
            # 这是 CPV 的**硬前提**:不在 CAN_CTRL 时 CPV 帧不报错也不动。
            # 没这一关的话,失败会长得像「CPV 不管用」,而真因是模式没切。
            return (f"ctrl_mode = {mode},不是 CAN_CTRL。CPV 帧在这个模式下"
                    f"**不报错也不动**。先在松灵客户端把臂切成 CAN 指令控制模式。")
        if not arm.read_enabled(wait=2.0):
            # 不替人使能:那是个会让人吃惊的状态变更,而且使能瞬间臂可能就动。
            return "臂未使能。本脚本**不自动使能** —— 请确认净空后手动使能再跑。"
    amp = amp_deg / D
    lo, hi = NERO_ARM_LIMITS[j]
    for name, v in (("base+amp", base[j] + amp), ("base-amp", base[j] - amp)):
        if v < lo or v > hi:
            return (f"{ARM_JOINTS[j]} {name} = {v * D:.1f}° 超限位 "
                    f"[{lo * D:.1f}, {hi * D:.1f}]。臂当前停在 {base[j] * D:.1f}°,"
                    f"离限位太近 —— 先手动挪开或减小 --amp。")
    return None


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", type=int, default=1, choices=range(1, 8),
                    help="动哪个关节(1-based)。默认 1 —— 竖直轴,重力不产生"
                         "力矩,失去伺服也不会掉")
    ap.add_argument("--amp", type=float, default=5.0, help="幅度(deg)")
    ap.add_argument("--freq", type=float, default=0.25,
                    help="B 段正弦频率(Hz)。**这是压力旋钮** —— 峰值加速度 "
                         "A·ω² 按 freq² 涨。默认 0.25Hz/5° 只用掉 0.11× ac,"
                         "故意留足余量(step 0a 验通路,不验饱和);想压到饱和"
                         "用 --freq 1.0(≈1.38× ac)")
    ap.add_argument("--cycles", type=float, default=2.0, help="B 段周期数")
    ap.add_argument("--fps", type=float, default=30.0, help="B 段下发率")
    ap.add_argument("--hold", type=float, default=1.5, help="A 段每次阶跃后保持(s)")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--firmware", default="auto")
    ap.add_argument("--no-mock", dest="mock", action="store_false",
                    help="打真臂。**臂会动**,需要配 --yes")
    ap.add_argument("--yes", action="store_true", help="确认净空、有人看着")
    ap.add_argument("--skip-a", action="store_true",
                    help="跳过 A 段直接跑 B 段。⚠ A 段没过时 B 段的结论无意义")
    a = ap.parse_args()
    j = a.joint - 1

    if not a.mock and not a.yes:
        sweep = a.amp / D * 0.6 * 1e3          # 0.6m 半径处扫过的弧长
        print(f"\n⚠ 将在**真臂** {ARM_JOINTS[j]} 上发 CPV 位置指令,幅度 "
              f"±{a.amp:.1f}°。臂会动。", file=sys.stderr)
        print(f"   {ARM_JOINTS[j]} 转 {a.amp:.1f}° 时,离基座 0.6m 处大约扫过 "
              f"{sweep:.0f}mm(实际取决于当前位姿)。", file=sys.stderr)
        if j == 0:
            print("   joint1 是竖直轴 —— 整条臂**横扫**,净空看水平方向。",
                  file=sys.stderr)
        else:
            print(f"   ⚠ {ARM_JOINTS[j]} 不是竖直轴 —— 重力在这个关节上有力矩,"
                  f"CPV 没接管伺服时臂会**掉**(无关节抱闸)。", file=sys.stderr)
        print("   确认下方/周围净空、有人看着、没有别的进程在控制臂,"
              "然后加 --yes 重跑。", file=sys.stderr)
        return 2
    return _run(a, j)


def _run(a, j: int) -> int:
    arm = NeroArm(mock=a.mock, channel=a.channel, firmware=a.firmware)
    print(f"接入 {'mock' if a.mock else a.channel} …", flush=True)
    try:
        arm.connect()
    except Exception as e:                                  # noqa: BLE001
        print(f"接入失败: {e}", file=sys.stderr)
        return 1

    traces: list[Trace] = []
    a_ok = b_ok = None
    try:
        base = arm.read_angles()
        print("t0 位姿: " + "  ".join(f"{n} {v*D:7.2f}°"
                                     for n, v in zip(ARM_JOINTS, base)))
        err = preflight(arm, base, j, a.amp)
        if err:
            # stdout + flush,同 A 段的理由:失败信息必须跟在 t0 位姿后面 ——
            # 这条报错里的「臂当前停在 X°」只有挨着位姿那行才读得懂。
            print(f"\n✗ 前置检查不通过: {err}", flush=True)
            return 2
        # mock 下不印使能:preflight 在 mock 分支跳过了使能检查,而 _enabled
        # 初值是 False —— 印出来就是「前置检查通过 / 使能=False」,自相矛盾。
        en = "(mock 未检查)" if arm.mock else str(arm.enabled)
        print(f"前置检查通过  ctrl_mode={arm.read_ctrl_mode()}  "
              f"使能={en}  driver={arm.firmware_detected}")

        if not arm.cpv_begin():
            print(f"✗ cpv_begin 失败: {arm.last_error}", file=sys.stderr)
            return 1
        judge = not arm.mock
        try:
            if not a.skip_a:
                ta, tb, a_ok = stage_a(arm, base, j, a.amp, a.hold, judge)
                traces += [ta, tb]
                if not a_ok:
                    # ⚠ 印在 stdout,不是 stderr。两个流各自缓冲,管道下这行会
                    # 跑到 t0 位姿**前面**去 —— 失败信息脱离上下文,真机上人看到的
                    # 就是一句没有前因的「没过」。实测踩到。
                    print("\n✗ A 段没过 —— **不跑 B 段**。B 段的任何结论都以"
                          "「CPV 能驱动臂」为前提,前提不成立时那些数没有意义。",
                          flush=True)
                    return 1
            tb2 = stage_b(arm, base, j, a.amp, a.freq, a.cycles, a.fps)
            traces.append(tb2)
            b_ok = analyze_b(tb2, j, a.fps, a.freq, judge)
        finally:
            # ⚠ 必须 end:cpv_begin 关掉了 auto_set_motion_mode,留着不恢复会让
            # 之后的 move_j 走在 cpv 模式下(行为未定义)。异常路径也要走到。
            arm.cpv_end()
    except KeyboardInterrupt:
        # 中断时**不补发回程** —— 再发运动指令比停在原地更糟。
        print("\n中断:已停发。臂停在最后一个目标位,没有补发回程。",
              file=sys.stderr)
        return 130
    finally:
        if traces:
            tag = f"{ARM_JOINTS[j]}_{'mock' if a.mock else 'real'}"
            print(f"\n原始采样 → {dump(traces, tag)}")
        arm.disconnect()

    return _verdict(a, a_ok, b_ok)


def _verdict(a, a_ok: bool | None, b_ok: bool | None) -> int:
    print("\n" + "=" * 60)
    if a.mock:
        # ⚠ mock 下**判据是不成立的**:mock 的 read_angles 在目标位附近摆
        # ±0.12rad(6.9°),永远进不了 0.5° 的稳态窗,幅度比也被那个摆动污染。
        # 所以 mock 只验代码路径能不能走通,不给通过/失败的结论。
        # (combo_player 的 approach() 里是同一个已知覆盖缺口。)
        print("mock 跑通了代码路径。**判据在 mock 下无意义** —— mock 的读数在"
              "目标位附近摆 ±6.9°,\n稳态窗(0.5°)和幅度比都测不出真东西。")
        print("真机: python3 sim/verify_cpv_step0.py --no-mock --yes")
        return 0
    if a_ok is False or b_ok is False:
        print("✗ 没过。CPV 通路**不能**按现在的假设用 —— combo_player 上真臂前"
              "必须先解决这个。")
        return 1
    if b_ok:
        print("✓ CPV 通路成立:能驱动臂,能吃下 "
              f"{a.fps:g}fps 的流,幅度没掉。")
        print("  注意这只证明了**通路**。素材需求超轮廓上限的问题还在"
              "(p95 加速度 2.376×),\n  整条轨迹要先修腕部翻转 + retime。")
        return 0
    print("? 结论不完整(可能用了 --skip-a 或中途退出)。看上面各段。")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
