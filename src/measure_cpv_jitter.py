#!/usr/bin/env python3
"""测**发送方向**的抖动:210 帧/秒能不能稳定发出去。

## 为什么必须测这个

收方向已经看到帧是**成批**到的(一批中位 10 帧、批间隔 1.87ms)—— 说明 usbip over
WSL 这一层在做批处理。发方向同理:`move_cpv_pos` 一个轨迹点要 7 帧,30fps = 210 帧/秒。
如果这 210 帧被 USB 批成一坨一坨地出去,臂收到的就不是"每 33ms 一个新目标",
而是"憋一下再来一串" —— 那 CPV 方案就不成立,得退回稀疏路点 + move_j。

总线本身不是瓶颈:实测底噪 3424 帧/秒 = 45.2% 占用,加 210 帧/秒到 48.0%。

## 默认模式:只读

不带 flag 时**只测收方向**,一帧都不发。这样能先把"总线现状"和"我们的读循环
跟不跟得上"量出来,不碰臂。

`--send` 才发帧,而且:
  · 默认用 `--dry-can`(vcan0)—— 完全不接触真臂
  · 打真 can0 要 `--real`,并且默认只发**查询帧**(CPV 读请求),不发位置指令
  · 要发位置指令必须再加 `--write-pos`,而且会先要求确认

⚠ 即使只发查询帧也是在**往总线上发东西**。按项目约定(硬件脚本默认只读)
这一步需要人明确授权,不能顺手跑。
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

SIM = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM))


def pct(xs: list[float], q: float) -> float:
    if not xs:
        return float("nan")
    s = sorted(xs)
    k = min(len(s) - 1, max(0, int(round(q / 100 * (len(s) - 1)))))
    return s[k]


def report(name: str, gaps_ms: list[float], target_ms: float,
           verdict: bool = True) -> None:
    """`verdict=False` 时不给结论。

    ⚠ 收方向**不能**套发方向的判据。收方向帧是成批到的(usbip),p99 天然超均值好几倍,
    那是正常现象、不是问题;而发方向 p99 超目标 2× 才意味着我们发不出节拍。
    第一版两边都打了结论,结果只读模式下也印出「CPV 方案要重新评估」——
    看着像测了发送并且失败了,实际一帧都没发。**判据要跟着被测对象走。**
    """
    if not gaps_ms:
        print(f"{name}: 没有样本")
        return
    over = sum(1 for g in gaps_ms if g > target_ms * 1.5)
    print(f"\n=== {name} ===  目标周期 {target_ms:.2f}ms  样本 {len(gaps_ms)}")
    print(f"  中位 {statistics.median(gaps_ms):7.3f}ms   均值 "
          f"{statistics.fmean(gaps_ms):7.3f}ms")
    for q in (50, 90, 95, 99):
        print(f"  p{q:<3} {pct(gaps_ms, q):7.3f}ms")
    print(f"  max  {max(gaps_ms):7.3f}ms   min {min(gaps_ms):7.3f}ms")
    print(f"  超目标 1.5× 的: {over} ({over/len(gaps_ms)*100:.1f}%)")
    if verdict:
        # CPV 要求"每 33ms 一个新目标"。p99 超 2× 目标就说明节拍交不出去。
        # ⚠ **必须同时看 max**:只看 p99 时,3 秒测试里的一个 3540ms 离群值也会被
        # 判成 OK(p99 才 11ms)。而单次 3.5 秒的停顿对回放是致命的 —— 臂会停在那儿。
        # 实测踩过这个漏判。
        p99, mx = pct(gaps_ms, 99), max(gaps_ms)
        if p99 >= target_ms * 2:
            print("  → ⚠ p99 超目标 2 倍 —— 疑似被 USB 批处理,CPV 方案要重新评估")
        elif mx >= target_ms * 5:
            print(f"  → ⚠ p99 正常但 **max {mx:.1f}ms 超目标 5 倍** —— 有离群停顿。"
                  f"回放时臂会在那儿卡一下,必须查清")
        else:
            print("  → OK —— 抖动在目标周期内(p99 和 max 都过)")


def measure_recv(channel: str, secs: float) -> None:
    """只读:量我们的读循环实际能拿到多密的新帧。一帧都不发。"""
    import can
    bus = can.interface.Bus(channel=channel, interface="socketcan",
                            receive_own_messages=False)
    try:
        arrivals: list[float] = []
        t_end = time.monotonic() + secs
        while time.monotonic() < t_end:
            m = bus.recv(timeout=0.5)
            if m is not None:
                arrivals.append(m.timestamp)
    finally:
        bus.shutdown()
    gaps = [(b - a) * 1000 for a, b in zip(arrivals, arrivals[1:])]
    print(f"收到 {len(arrivals)} 帧 / {secs:.1f}s = "
          f"{len(arrivals)/secs:.0f} 帧/秒(全 ID 合计)")
    # verdict=False:收方向成批到达是正常的,不适用发方向那个判据
    report("收方向 帧间隔(参考基线,成批到达属正常)", gaps, 1000 / 3424,
           verdict=False)


def _echo_listener(channel: str, joints: int, stop, out: list) -> None:
    """监听**我们自己发的**帧的本地回环。

    为什么这个比 `send()` 返回时刻更有意义:socketcan 的本地回环不是在 send 时
    立刻产生的 —— 内核在 TX **提交**时 `can_put_echo_skb()`,在驱动报告 TX
    **完成**时才 `can_get_echo_skb()` 把回环帧投递给其它 socket。对 gs_usb 来说
    "TX 完成"是设备通过 USB 回报的,所以回环时刻≈帧真正发出去的时刻(+USB 回程延迟)。

    `send()` 返回只说明"交给内核队列了",队列后面被 USB 批处理成什么样看不见。
    两个数一起看才能分清"我们发不出节拍"和"节拍发出去了但被 USB 攒住了"。

    ⚠ 这是**独立的 socket**,所以拿得到回环 —— 发送那个 socket 自己设了
    receive_own_messages=False,只屏蔽它自己,不影响别人。
    """
    import can
    bus = can.interface.Bus(
        channel=channel, interface="socketcan", receive_own_messages=True,
        can_filters=[{"can_id": 0x180 + j, "can_mask": 0x7FF}
                     for j in range(1, joints + 1)])
    try:
        while not stop.is_set():
            m = bus.recv(timeout=0.2)
            if m is not None:
                out.append((m.arbitration_id, m.timestamp))
    finally:
        bus.shutdown()


def measure_send(channel: str, fps: float, secs: float, joints: int,
                 write_pos: bool, echo: bool = False) -> None:
    """量发送方向:按 fps 发 `joints` 帧一组,看每组**实际**间隔多少。

    ⚠ 这里量的是 `bus.send()` **返回**的时刻,不是帧真正上线的时刻 ——
    socketcan 的 send 是把帧交给内核队列就返回。所以这个数是"我们能不能按节拍
    交出去"的下界:它抖了肯定不行,它不抖也**不保证**帧在线上不抖(USB 那层看不见)。
    要看线上的实际间隔得同时开 candump 对着 echo 帧数,那是下一步。
    """
    import can
    bus = can.interface.Bus(channel=channel, interface="socketcan",
                            receive_own_messages=False)
    # CPV 查询帧:mode='r' 只读参数,不改臂的任何状态。ID 0x180+关节号。
    # write_pos=True 时才发位置指令 —— 那会让臂动,必须显式要求。
    payload = bytearray(8)
    if not write_pos:
        # 'r' 模式 + type='po' 的读请求。字节布局照 SDK 的
        # _make_cpv_settings_and_queries_msg:byte0=mode/type 编码区,值区留 0。
        payload[0] = 0x00          # 保守:全 0 = 读请求,不写任何参数
    period = 1.0 / fps
    sent_at: list[float] = []
    echoes: list[tuple] = []
    stop = None
    if echo:
        import threading
        stop = threading.Event()
        threading.Thread(target=_echo_listener,
                         args=(channel, joints, stop, echoes),
                         daemon=True).start()
        time.sleep(0.2)              # 等 listener 的 socket 起来,免得漏头几组
        # ⚠ 清掉这 0.2s 里收到的东西 —— 连着跑多档时那里面有**上一轮的残留**回环。
        # 不清的后果实测过:3 秒的测试报出 3540ms 的间隙,而完整组数又是满的
        # (两件事不能同时成立,是假象的指纹)。
        # 用清 list 而不是按时间戳过滤:回环戳是 CLOCK_REALTIME、send 用 monotonic,
        # 跨时钟重建"发送起点"会累积漂移 —— 第一版就是那么写的,结果 30fps 档
        # 90 组只剩 1 组。清 list 不涉及任何时钟换算,是精确的。
        echoes.clear()
    t0 = time.monotonic()
    next_tick = t0
    t_end = t0 + secs
    try:
        while time.monotonic() < t_end:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.0005, next_tick - now))
                continue
            for j in range(1, joints + 1):
                bus.send(can.Message(arbitration_id=0x180 + j,
                                     data=bytes(payload), is_extended_id=False))
            sent_at.append(time.monotonic())
            next_tick += period
            if next_tick < time.monotonic():        # 落后就重新对齐
                next_tick = time.monotonic() + period
    finally:
        bus.shutdown()
        if stop is not None:
            time.sleep(0.3)          # 收尾:等最后几组的回环到齐
            stop.set()
    gaps = [(b - a) * 1000 for a, b in zip(sent_at, sent_at[1:])]
    print(f"\n发出 {len(sent_at)} 组 × {joints} 帧 = {len(sent_at)*joints} 帧 / "
          f"{secs:.1f}s = {len(sent_at)*joints/secs:.0f} 帧/秒")
    report(f"发方向 send() 返回间隔 ({fps:g}fps × {joints} 关节)",
           gaps, period * 1000)

    if not echo:
        return
    # 上一轮的残留已经在发送前 echoes.clear() 掉了,这里不需要再按时间过滤。
    # 回环:按关节 1 的帧切组,看**线上**的组间隔和组内散布
    j1 = [ts for cid, ts in echoes if cid == 0x181]
    egaps = [(b - a) * 1000 for a, b in zip(j1, j1[1:])]
    print(f"\n回环收到 {len(echoes)} 帧(应 ≈{len(sent_at)*joints}),"
          f"其中关节1 {len(j1)} 帧(应 ≈{len(sent_at)})")
    if len(j1) < len(sent_at) * 0.9:
        print("  ⚠ 回环帧明显少于发出的 —— listener 起晚了或被过滤掉了,"
              "下面的数不可信")
    report("发方向 **线上**组间隔(TX 完成回环)", egaps, period * 1000)
    # 组内散布:同一组 joints 帧在线上摊开多宽。决定"7 帧算不算同一时刻"。
    #
    # ⚠ 分组必须用**关节 1 的帧当边界**,不能按时间排序盲切 joints 帧一组。
    # 实测踩过:盲切时 50fps 那档报出组内散布 19.9ms(整整一个周期)、max 202ms,
    # 而 100fps / 200fps 又是正常的 0.47ms —— 这种"中间坏两头好"就是错位的指纹,
    # 不是物理现象。丢一帧或多收一帧,后面所有组就整体串位、跨周期取 min/max。
    byts = sorted(echoes, key=lambda x: x[1])
    groups: list[list[float]] = []
    for cid, ts in byts:
        if cid == 0x181 or not groups:      # 关节 1 开新组
            groups.append([ts])
        else:
            groups[-1].append(ts)
    full = [g for g in groups if len(g) == joints]
    spreads = [(max(g) - min(g)) * 1000 for g in full]
    if spreads:
        print(f"\n组内 {joints} 帧散布 (ms): 中位 {statistics.median(spreads):.3f}  "
              f"p95 {pct(spreads,95):.3f}  max {max(spreads):.3f}"
              f"   (完整组 {len(full)}/{len(groups)})")
        ratio = pct(spreads, 95) / (period * 1000) * 100
        print(f"  → p95 散布占周期的 {ratio:.1f}%"
              f"{' —— 可当同一时刻' if ratio < 10 else ' —— 偏宽,一个点的 7 个关节不同步'}")
        if len(full) < len(groups) * 0.9:
            print(f"  ⚠ 有 {len(groups)-len(full)} 组不足 {joints} 帧 —— 丢帧了,"
                  f"上面的散布只统计了完整组")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--fps", type=float, default=30.0, help="CPV 目标下发率")
    ap.add_argument("--joints", type=int, default=7)
    ap.add_argument("--send", action="store_true",
                    help="测发送方向。**会往总线发帧**,默认关")
    ap.add_argument("--real", action="store_true",
                    help="--send 时打真 channel。不加则要求 channel 是 vcan*")
    ap.add_argument("--write-pos", action="store_true",
                    help="发**位置指令**而非查询帧 —— 臂会动。需要二次确认")
    ap.add_argument("--yes", action="store_true", help="跳过确认(脚本化用)")
    ap.add_argument("--echo", action="store_true",
                    help="同时监听自己发的帧的本地回环,量**线上**间隔。"
                         "不额外发帧,纯多开一个只读 socket")
    a = ap.parse_args()

    print(f"通道 {a.channel}   时长 {a.secs}s")
    # 收方向永远测 —— 它是只读的,而且给发方向提供对比基线
    measure_recv(a.channel, a.secs)

    if not a.send:
        print("\n(只测了收方向,一帧没发。加 --send 测发送方向)")
        return 0

    if not a.real and not a.channel.startswith("vcan"):
        print(f"\n拒绝:--send 默认只允许 vcan*,当前是 {a.channel}。"
              f"确实要打真臂请显式加 --real", file=sys.stderr)
        return 2
    if a.write_pos and not a.yes:
        print("\n⚠ --write-pos 会发 CPV **位置指令**,臂会动。", file=sys.stderr)
        print("   臂已使能时这会立刻产生运动。确认下方净空、有人看着,"
              "然后加 --yes 重跑。", file=sys.stderr)
        return 2
    if a.real and not a.yes:
        print(f"\n⚠ 将往**真** {a.channel} 发 {a.fps:g}fps × {a.joints} "
              f"= {a.fps*a.joints:.0f} 帧/秒的查询帧(只读参数,不改臂状态)。",
              file=sys.stderr)
        print("   确认没有别的进程在控制臂,然后加 --yes 重跑。", file=sys.stderr)
        return 2

    measure_send(a.channel, a.fps, a.secs, a.joints, a.write_pos, a.echo)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
