#!/usr/bin/env python3
"""量出拇指侧摆(yaw)与食指弯曲的**干涉边界**。要连真手。

背景:同时给六个通道下 raw 0(全闭合),实测 ANGLE_ACT = [228, 230, 231, 0, 0, 0]
(项目序)。中指/无名/小指到底,拇指两关节和食指停在 228/230/231 —— 三个不同关节
停在 3 个计数内,是它们**互相顶住**了。中指往外,不和拇指相交,所以能到底。

这不是 bug,是运动学事实:yaw raw 0 = 对掌位(出平面 62°,拇指尖↔食指尖 8mm),
食指 raw 0 = 完全弯曲。两者同时到极限,必然撞在一起。

⚠ 全程用 **raw** 而不是 rad:URDF 与官方 xls 的 span 不一致(四指 1.470 vs 1.39626,
   拇指弯曲 0.600 vs 0.69813),在 rad 域里量会把几何问题和换算问题混在一起。

测试:
  T1 同时下发    复现卡死,拿到基线
  T2 分步下发    先四指再拇指,看能否绕开(决定包是单帧还是多帧)
  T3 yaw 扫描    pitch 固定 OPEN,逐个 yaw 测食指能闭到哪
  T4 速度比      同终点、不同到达时序 —— 证明撞的是半路不是终点
  T5 对角线      yaw=pitch 扫,**验能否重现 hand_pose.FEASIBLE**(先跑这个)
  T6 二维网格    覆盖手势实际工作区(T=100..300),t5 过了再跑

⚠ T3 产不出 `hand_pose.FEASIBLE`。那张表按 `T=max(yaw,pitch)` 索引、测点 300/450/600,
   而 T3 里 pitch 恒 1000 → T 恒 1000,且它扫的是 1000/800/…/0。
   `hand_pose.py:107` 标注「来自 T3」是**错的** —— 真实来源是一次对角线扫描,
   代码不在本文件里。T5 就是把它补回来。

⚠ 扫描实际使用 SCAN_SPEED=15,不是 init_speed=500:顶死力由速度主导
   (已知 50→272g · 150→717g · 300→941g；15 是更低的探索档)。每点后查 ERROR,
   过温位(Bit1,**不可清除**)
   一置上立刻中止。

用法:
  python3 src/test/hardware/test_thumb_index_collision.py                 # t1-t4
  python3 src/test/hardware/test_thumb_index_collision.py --only t5       # 先验方法(~5min)
  python3 src/test/hardware/test_thumb_index_collision.py --only t6       # 再扫二维(~20min)
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM_DIR))

from inspire_hand import (InspireHand, InspireHandConfig,  # noqa: E402
                          PROJECT_TO_VENDOR)

# 项目序索引,便于读代码
YAW, PITCH, INDEX, MIDDLE, RING, PINKY = range(6)
NAMES = ["yaw", "pitch", "index", "middle", "ring", "pinky"]
OPEN = 1000          # raw 1000 = 张开 / 拇指躺在掌面
CLOSED = 0           # raw 0    = 闭合 / 拇指对掌位
# 食指判定阈值:单独动时它能到 raw 0。停在这个值以上就说明被挡住了。
JAM_RAW = 60

# 扫描专用速度。**故意远低于 init_speed(500)**:顶死力由速度主导 ——
# 实测 speed 50→272g · 150→717g · 300→941g(2026-08-06)。扫描的本质是
# 反复顶死,点数又是 T3 的数倍,用默认 500 等于每次都拿 >941g 去怼。
SCAN_SPEED = 15  # 改低,测"刚接触"而非"顶死"(2026-08-10 第二次 t5)

# ERROR 寄存器位。Bit1 过温**不可清除**,只能断电重启手 —— 所以它是中止条件,
# 不是警告。Bit0 堵转是扫描的预期现象(那就是在测的东西),不中止。
ERR_STALL, ERR_OVERTEMP = 0x01, 0x02


def check_faults(hand: InspireHand, where: str) -> str | None:
    """读 ERROR/TEMP。返回非 None = 必须中止(过温)。

    ⚠ 原来整个脚本**从不读 ERROR** —— main() 只在首尾读 TEMP。而这个脚本干的事
    就是反复把两个关节怼在一起,过温位一旦置上不可清除。没这一关的话,损坏是
    静默累积的:你看到的只是"表扫完了",看不到第 8 个点之后手已经在过温里跑。
    """
    err = hand.read_regs("ERROR", 6, "6B")
    temp = hand.read_regs("TEMP", 6, "6B")
    if err is None:
        return None                      # 读不到不当故障:漏读率约 40%,见 HAND_DEBUG
    hot = [i for i, e in enumerate(err) if int(e) & ERR_OVERTEMP]
    if hot:
        return (f"{where}: 过温位置上(ERROR Bit1),厂商序通道 {hot},"
                f"TEMP={list(temp) if temp else '?'}。**不可清除,断电重启手**。")
    if temp and max(int(t) for t in temp) >= 55:
        return (f"{where}: TEMP={list(temp)} 已到 55℃,离过温很近 —— 主动停,"
                f"等凉了再继续(剩下的点没测完不如手坏了)。")
    return None


def to_vendor(proj: list[int]) -> list[int]:
    out = [-1] * 6
    for i, v in enumerate(proj):
        out[PROJECT_TO_VENDOR[i]] = int(v)
    return out


def read_proj(hand: InspireHand, reg: str, nb: int = 12, fmt: str = "6h",
              tries: int = 12) -> list[int] | None:
    """读 6 通道并转项目序。**要求连续两次读到相同值**才采信。

    为什么不能单次读:地址核对(parse_reply 的 want_addr)只挡得住"答的是别的
    寄存器",挡不住**同一寄存器的旧值** —— 缓冲里上一次的同址回复地址对、校验和对,
    会被当成本次结果收下。实测后果:命令拇指压过来,读回却是命令之前的位置,
    而 STATUS 同时说"位置到位",两件事互相矛盾 —— 那是两个不同时刻的帧拼在一起。

    连续两次一致不是万能(两个旧帧也可能一致),但配合调用侧的 settle 时间,
    足以把"运动中途的旧帧"挡掉。真正的解法是协议层加序号,那要改 inspire_hand。
    """
    prev = None
    for _ in range(tries):
        v = hand.read_regs(reg, nb, fmt)
        if v is not None:
            cur = [int(v[PROJECT_TO_VENDOR[i]]) for i in range(6)]
            if prev is not None and cur == prev:
                return cur
            prev = cur
        time.sleep(0.08)
    return prev          # 拿不到两次一致就退回最后一次,调用侧会看到它不稳


def send(hand: InspireHand, proj: list[int], verify: bool = True) -> bool:
    """下发角度,并**回读 ANGLE_SET 确认指令真的落进去了**。

    write_shorts 对写指令收不到回复也返回 True(手对写偶发不回但动作照做),
    所以它的返回值不能用来判断"指令到没到"。不确认就没法区分两件事:
    「命令没下去」和「命令下去了但手动不了」—— 前者是通信问题,后者是力学问题,
    诊断方向完全相反。
    """
    hand.write_shorts("ANGLE_SET", to_vendor(proj))
    if not verify:
        return True
    time.sleep(0.15)
    got = read_proj(hand, "ANGLE_SET")
    if got is None:
        print(f"    ! ANGLE_SET readback failed, cannot confirm command")
        return False
    if got != [int(v) for v in proj]:
        print(f"    ! command mismatch: sent {proj} but ANGLE_SET reads {got}")
        return False
    return True


def go_open(hand: InspireHand, settle: float = 2.0) -> None:
    """回全张开。**每个用例前后都要回**,否则会继承上一个用例的顶死状态。"""
    hand.set_force(200)
    send(hand, [OPEN] * 6)
    time.sleep(settle)


def fmt(vals: list[int] | None) -> str:
    if vals is None:
        return "(read failed)"
    return " ".join(f"{n}={v:4d}" for n, v in zip(NAMES, vals))


def t1_simultaneous(hand: InspireHand) -> dict:
    """T1 同时下发全闭合 —— 复现卡死,拿基线。"""
    print("\n" + "=" * 66)
    print("T1  simultaneous: all six -> raw 0 at once")
    go_open(hand)
    hand.set_force(250)
    time.sleep(0.3)
    send(hand, [CLOSED] * 6)
    time.sleep(3.0)
    act = read_proj(hand, "ANGLE_ACT")
    st = read_proj(hand, "STATUS", 6, "6B")
    print(f"  ACT    {fmt(act)}")
    print(f"  STATUS {fmt(st)}")
    jam = act is not None and act[INDEX] > JAM_RAW
    print(f"  -> index stopped at {act[INDEX] if act else '?'} "
          f"({'JAMMED' if jam else 'reached'})")
    go_open(hand)
    return {"act": act, "status": st, "jammed": jam}


def t2_sequenced(hand: InspireHand) -> dict:
    """T2 分步:先四指闭合到底,再让拇指过来。

    这个测试决定**包能不能是单帧**。如果分步能绕开,那么握拳这类动作天生是
    多帧序列(四指先合,拇指后压),不是一个姿态 —— 包格式必须支持时序,
    而不只是"一组目标角度"。
    """
    print("\n" + "=" * 66)
    print("T2  sequenced: four fingers close first, then thumb comes over")
    go_open(hand)
    hand.set_force(250)
    time.sleep(0.3)

    # 第一步:四指闭合,拇指留在掌面(yaw=OPEN)不参与
    send(hand, [OPEN, OPEN, CLOSED, CLOSED, CLOSED, CLOSED])
    time.sleep(2.5)
    mid = read_proj(hand, "ANGLE_ACT")
    print(f"  after fingers  {fmt(mid)}")

    # 第二步:四指保持,拇指压过来
    send(hand, [CLOSED, CLOSED, CLOSED, CLOSED, CLOSED, CLOSED])
    time.sleep(2.5)
    act = read_proj(hand, "ANGLE_ACT")
    st = read_proj(hand, "STATUS", 6, "6B")
    print(f"  after thumb    {fmt(act)}")
    print(f"  STATUS         {fmt(st)}")

    jam = act is not None and act[INDEX] > JAM_RAW
    better = (act is not None and mid is not None
              and act[INDEX] <= mid[INDEX] + 5)
    print(f"  -> index {act[INDEX] if act else '?'} "
          f"({'still jammed' if jam else 'reached'}); "
          f"held position: {better}")
    go_open(hand)
    return {"mid": mid, "act": act, "status": st, "jammed": jam}


def t3_yaw_sweep(hand: InspireHand) -> list[dict]:
    """T3 逐个 yaw 位置测食指能闭到哪 —— 产出可行域表。

    这张表就是包层需要的约束:给定拇指对掌程度,食指最多能弯到哪。
    拇指弯曲(pitch)固定在张开位,先把 yaw 单独隔离出来 —— 用户说冲突在 yaw。
    """
    print("\n" + "=" * 66)
    print("T3  yaw sweep: park thumb at each yaw, then close index fully")
    print("    (pitch held at OPEN to isolate yaw)")
    print(f"\n  {'yaw_cmd':>8} {'yaw_act':>8} {'index_act':>10} {'verdict':>10}")
    print("  " + "-" * 40)
    rows = []
    for yaw in (1000, 800, 600, 400, 200, 0):
        go_open(hand, settle=1.8)
        hand.set_force(250)
        time.sleep(0.2)
        # 先把拇指停到目标 yaw,食指仍张开
        send(hand, [yaw, OPEN, OPEN, OPEN, OPEN, OPEN])
        time.sleep(1.8)
        # 再单独关食指
        send(hand, [yaw, OPEN, CLOSED, OPEN, OPEN, OPEN])
        time.sleep(2.2)
        act = read_proj(hand, "ANGLE_ACT")
        if act is None:
            print(f"  {yaw:8d} {'?':>8} {'read fail':>10}")
            rows.append({"yaw_cmd": yaw, "yaw_act": None, "index_act": None})
            continue
        jam = act[INDEX] > JAM_RAW
        rows.append({"yaw_cmd": yaw, "yaw_act": act[YAW],
                     "index_act": act[INDEX], "jammed": jam})
        print(f"  {yaw:8d} {act[YAW]:8d} {act[INDEX]:10d} "
              f"{'JAM' if jam else 'ok':>10}")
    go_open(hand)
    return rows


def sweep(hand: InspireHand, pts: list[tuple[int, int]],
          title: str) -> list[dict]:
    """给定 (yaw, pitch) 点列,逐点测食指能闭到哪。**产出可行域表的通用版**。

    和 T3 的关系:T3 是这个函数的一个特例(pitch 恒 OPEN),但 T3 不能反过来产出
    `hand_pose.FEASIBLE` —— 那张表按 `T = max(yaw,pitch)` 索引、测点是 300/450/600,
    而 T3 里 pitch 恒 1000 使 T 恒等于 1000,且它扫的是 1000/800/…/0。
    **所以 `hand_pose.py:107` 写「来自 T3」是错的**,那张表来自一次沿 yaw=pitch
    对角线的扫描,而那次扫描的代码不在本文件里。t5 就是把它补回来。

    每点之前 go_open,所以顶死状态不继承;每点之后查过温,一旦置上立刻停。
    """
    print("\n" + "=" * 66)
    print(f"{title}  (speed={SCAN_SPEED}, 每点前回零、点后查过温)")
    print(f"\n  {'yaw_cmd':>8} {'pit_cmd':>8} {'T':>5} {'yaw_act':>8} "
          f"{'pit_act':>8} {'idx_act':>8} {'verdict':>8}")
    print("  " + "-" * 60)
    rows: list[dict] = []
    for yaw, pitch in pts:
        go_open(hand, settle=1.8)
        hand.set_speed(SCAN_SPEED)          # ← go_open 里 set_force 不改速度,这行必须在它之后
        hand.set_force(250)
        time.sleep(0.2)
        send(hand, [yaw, pitch, OPEN, OPEN, OPEN, OPEN])   # 拇指先到位
        time.sleep(2.2)                                     # 低速要更久
        send(hand, [yaw, pitch, CLOSED, OPEN, OPEN, OPEN])  # 再单独关食指
        time.sleep(3.0)
        act = read_proj(hand, "ANGLE_ACT")
        T = max(yaw, pitch)
        if act is None:
            print(f"  {yaw:8d} {pitch:8d} {T:5d} {'read fail':>26}")
            rows.append({"yaw_cmd": yaw, "pitch_cmd": pitch, "T": T,
                         "act": None, "jammed": None})
        else:
            jam = act[INDEX] > JAM_RAW
            rows.append({"yaw_cmd": yaw, "pitch_cmd": pitch, "T": T,
                         "yaw_act": act[YAW], "pitch_act": act[PITCH],
                         "index_act": act[INDEX], "jammed": jam})
            print(f"  {yaw:8d} {pitch:8d} {T:5d} {act[YAW]:8d} "
                  f"{act[PITCH]:8d} {act[INDEX]:8d} {'JAM' if jam else 'ok':>8}")
        fault = check_faults(hand, f"扫到 (yaw={yaw}, pitch={pitch}) 后")
        if fault:
            print(f"\n✗ 中止:{fault}")
            print(f"  已测 {len(rows)}/{len(pts)} 点,下面的表是不完整的。")
            break
    go_open(hand)
    hand.set_speed(hand.cfg.init_speed)
    return rows


def t5_diagonal(hand: InspireHand) -> list[dict]:
    """沿 yaw = pitch 对角线扫 —— **先验方法,再扩二维**。

    目的不是拿新数据,是**验这套扫法能不能重现 `hand_pose.FEASIBLE`** 的
    (300,225)/(450,52)/(600,0)。重现得了 → 扫法对、旧表可信、可以往二维扩;
    重现不了 → 旧表和新扫法至少有一个不对,那时候二维扫出来的数你也不知道该信哪个。
    先花 5 分钟验方法,比扫完 30 个点再发现对不上便宜。
    """
    return sweep(hand, [(t, t) for t in (100, 200, 300, 450, 600)],
                 "T5  diagonal (yaw = pitch) — 验证能否重现 FEASIBLE")


def t6_grid(hand: InspireHand) -> list[dict]:
    """二维网格 —— 覆盖**实际手势真正用的区间**。

    为什么要它:现有手势的 T 全部落在旧表最低测点 300 **以下**
    (折拇指 T=284、对掌 T=141),`index_min_raw` 在那儿走的是
    `if t <= 300: return 225` 的夹平常数 —— 一个没有实测支撑的值。
    物理上 T 越小拇指挡得越多、下界该越高,夹平在偏**宽松**的方向。

    网格特意在 100-300 密、300 以上疏:密的那段是手势真正待的地方。
    """
    pts = [(y, p) for y in (100, 200, 300) for p in (100, 200, 300)]
    pts += [(y, p) for y in (450, 600) for p in (100, 300, 600)]
    pts += [(0, 1000), (1000, 0)]     # 两个极端非对角点:验 max() 概括成不成立
    return sweep(hand, pts, "T6  2-D grid — 覆盖手势实际工作区(T=100..300 加密)")


def compare_feasible(rows: list[dict]) -> None:
    """把 t5 结果和 `hand_pose.FEASIBLE` 逐点对比。这是 t5 的**唯一目的**。"""
    try:
        sys.path.insert(0, str(SIM_DIR / "skills"))
        from hand_pose import FEASIBLE            # noqa: PLC0415
    except Exception as e:                        # noqa: BLE001
        print(f"\n(读不到 hand_pose.FEASIBLE: {type(e).__name__}: {e})")
        return
    print("\n  与 hand_pose.FEASIBLE 对比:")
    old = dict(FEASIBLE)
    for r in rows:
        if r.get("index_act") is None or r["T"] not in old:
            continue
        want, got = old[r["T"]], r["index_act"]
        d = got - want
        tag = "一致" if abs(d) <= 15 else "**对不上**"
        print(f"    T={r['T']:4d}  旧表 {want:4d}  实测 {got:4d}  差 {d:+5d}  {tag}")
    print("    (容差 ±15:单次读噪声 + 停位重复性。超出说明扫法或旧表有一个不对)")


def t4_speed_ratio(hand: InspireHand) -> list[dict]:
    """T4 **相对速度**决定路径撞不撞 —— 终点可达,撞的是半路。

    依据:T3 里食指在 yaw=0(完全对掌)也能闭到 raw 0,因为拇指**已经停在终点**了;
    T1 里六个通道同时出发,拇指还在往对掌位摆、食指已经在弯,两者在半路相遇。
    所以这不是几何不可达,是轨迹交叉 —— 同一个终点,不同的到达时序,结果不同。

    SPEED_SET 是 6 通道独立的,所以相对速度是唯一不改姿态就能避开的杠杆:
    让拇指先到位(拇指快/四指慢),食指再弯下来时拇指已经不在路上了。

    ⚠ 轨迹采样用**单次读**,不用 read_proj 的"两次一致"规则 —— 运动中数值本来
       就在变,要求两次一致会一直等到停下,那就看不到过程了。终点判定仍用 read_proj。
    """
    print("\n" + "=" * 66)
    print("T4  speed ratio: same target, different arrival order")
    configs = [
        ("equal fast   ", [1000] * 6),
        ("equal medium ", [500] * 6),
        ("equal slow   ", [100] * 6),
        ("thumb fast   ", [1000, 1000, 150, 150, 150, 150]),
        ("thumb slow   ", [150, 150, 1000, 1000, 1000, 1000]),
    ]
    rows = []
    for label, speeds in configs:
        go_open(hand, settle=2.0)
        hand.write_shorts("SPEED_SET", to_vendor(speeds))
        hand.set_force(250)
        time.sleep(0.35)
        send(hand, [CLOSED] * 6, verify=False)
        # 采样轨迹:看拇指和食指在哪个时刻相遇
        traj = []
        for _ in range(16):
            time.sleep(0.18)
            v = hand.read_regs("ANGLE_ACT", 12, "6h")
            if v is not None:
                traj.append([int(v[PROJECT_TO_VENDOR[i]]) for i in range(6)])
        time.sleep(1.2)
        act = read_proj(hand, "ANGLE_ACT")
        jam = act is not None and act[INDEX] > JAM_RAW
        rows.append({"label": label.strip(), "speeds": speeds,
                     "act": act, "jammed": jam})
        idx_path = " ".join(f"{s[INDEX]:4d}" for s in traj[:10])
        yaw_path = " ".join(f"{s[YAW]:4d}" for s in traj[:10])
        print(f"\n  {label} speeds={speeds}")
        print(f"    yaw   path {yaw_path}")
        print(f"    index path {idx_path}")
        print(f"    final {fmt(act)}")
        print(f"    -> {'JAMMED' if jam else 'reached'}")
    go_open(hand)
    hand.set_speed(hand.cfg.init_speed)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--only", choices=["t1", "t2", "t3", "t4", "t5", "t6"],
                    help="t5 = 对角线(先跑这个验方法);t6 = 二维网格(t5 过了再跑)")
    args = ap.parse_args()

    hand = InspireHand(InspireHandConfig(port=args.port, mock=False))
    try:
        if not hand.connect():
            print("connect returned False")
            return 2
    except Exception as exc:                      # noqa: BLE001
        print(f"connect failed: {type(exc).__name__}: {exc}")
        print("(USB forwarding may have dropped; re-attach with usbipd on Windows)")
        return 2

    t1 = t2 = None
    t3: list[dict] = []
    t4: list[dict] = []
    t5: list[dict] = []
    t6: list[dict] = []
    try:
        temp = hand.read_regs("TEMP", 6, "6B")
        print(f"start TEMP {list(temp) if temp else '(read failed)'}")
        # ⚠ 开跑前先查一次:上一次跑完可能就已经过温了,那种情况下这次的数全是废的
        # (过温降力矩),而且会在已经受损的关节上继续怼。
        fault = check_faults(hand, "开跑前")
        if fault:
            print(f"✗ {fault}")
            return 1
        # t5/t6 不进「跑全部」—— 它们各自要 5-20 分钟真机时间,得显式点名。
        if not args.only or args.only == "t1":
            t1 = t1_simultaneous(hand)
        if not args.only or args.only == "t2":
            t2 = t2_sequenced(hand)
        if not args.only or args.only == "t3":
            t3 = t3_yaw_sweep(hand)
        if not args.only or args.only == "t4":
            t4 = t4_speed_ratio(hand)
        if args.only == "t5":
            t5 = t5_diagonal(hand)
            compare_feasible(t5)
        if args.only == "t6":
            t6 = t6_grid(hand)
    finally:
        go_open(hand)
        hand.set_force(hand.cfg.init_force)
        temp = hand.read_regs("TEMP", 6, "6B")
        print(f"\nend TEMP {list(temp) if temp else '(read failed)'}")
        hand.disconnect()

    print("\n" + "=" * 66)
    print("conclusions")
    if t1:
        print(f"  T1 simultaneous: {'JAMMED' if t1['jammed'] else 'no jam'}")
    if t2:
        print(f"  T2 sequenced   : {'JAMMED' if t2['jammed'] else 'no jam'}"
              f"  <- if no jam, packs need time-ordered frames")
    ok = [r for r in t3 if r.get("jammed") is False]
    if t3:
        if ok:
            worst = min(r["yaw_cmd"] for r in ok)
            print(f"  T3 index closes fully while yaw >= {worst}")
            print(f"     so the pack constraint is: yaw_raw >= {worst} "
                  f"when index is fully flexed")
        else:
            print("  T3 index never closed fully at any yaw -- "
                  "thumb blocks it across the whole range")
    if t4:
        good = [r["label"] for r in t4 if r["jammed"] is False]
        bad = [r["label"] for r in t4 if r["jammed"] is True]
        print(f"  T4 reached : {good or '(none)'}")
        print(f"     jammed  : {bad or '(none)'}")
        if good and bad:
            print("     -> same target, outcome depends on speed ratio:")
            print("        the collision is a TRAJECTORY problem, not geometry.")
            print("        fix = per-channel speed in the pack, or split into frames.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
