#!/usr/bin/env python3
"""sim/skills/hand_pose.py — 手势规格层:五指语义 → 六关节弧度 + 可行域校验。

**为什么要这层**:清单里原来直接写 `hand: [1.112, 0.600, 1.07, 0.0, 0.0, 0.0]`。
这串数字有三个问题:看不出是什么手势、改一个数不知道会不会撞、每个新手势都要
先上真手量一遍。这层把它换成:

    pose: {thumb: opposed, index: 0.85, middle: open, ring: open, pinky: open}

归一量 n ∈ [0,1]:**0 = 张开/伸直,1 = 这台机器实际能到的最闭合**。六个通道统一。

⚠ n=1 不等于 raw 0。拇指弯曲的 URDF 上限(0.6)比 xls 实际行程(0.698)小 14%,
   所以拇指弯曲 n=1 → raw 141 而不是 0。这是已知的行程损失(见 HAND_DEBUG.md),
   不是这里的 bug。归一化按 min(span, URDF上限) 做,n=1 的语义才是"实际能到的头"。

**表为什么抄一份而不是 import inspire_hand**:那个模块在 sim/,skills/ 里 import 它
要把 sim/ 塞进 sys.path,而 sim/schema.py 和 skills/schema.py 同名 —— 之前踩过这个
遮蔽坑。抄一份的代价是会漂(2026-08-07 换 URDF 就漂过),所以 `--verify` 用
importlib **按文件路径**加载 inspire_hand 逐项核对,不走 sys.path。改了那边就跑一次。

自检:
    /usr/bin/python3 sim/skills/hand_pose.py            # 手势表 + 可行域
    /usr/bin/python3 sim/skills/hand_pose.py --verify    # 与 inspire_hand 核对
"""
from __future__ import annotations

from pathlib import Path

# 项目顺序,与 inspire_hand.HAND_JOINTS / backend.HAND_NAMES 一致(拇指在前)。
# ⚠ 不是 sim/schema.py 的 HAND_ACTUATED 顺序 —— 那个是四指在前、拇指在后。
HAND_JOINTS = ["right_thumb_1_joint", "right_thumb_2_joint",
               "right_index_1_joint", "right_middle_1_joint",
               "right_ring_1_joint", "right_little_1_joint"]

# (span_rad, invert) —— 抄自 inspire_hand.RAW_MAP,`--verify` 核对。
RAW_MAP = {
    "right_thumb_1_joint":   (1.246165, True),
    "right_thumb_2_joint": (0.69813, True),
    "right_index_1_joint":       (1.39626, True),
    "right_middle_1_joint":      (1.39626, True),
    "right_ring_1_joint":        (1.39626, True),
    "right_little_1_joint":       (1.39626, True),
}

# URDF 限位上限 —— 抄自 inspire_hand.HAND_LIMITS 的 hi,`--verify` 核对。
LIMIT_HI = {
    "right_thumb_1_joint": 1.246165,
    "right_thumb_2_joint": 0.6,      # < span 0.698,少 14% 行程
    "right_index_1_joint": 1.47,           # > span,所以有效上限是 span
    "right_middle_1_joint": 1.47,
    "right_ring_1_joint": 1.47,
    "right_little_1_joint": 1.47,
}

RAW_MIN, RAW_MAX = 0, 1000

# 每通道**实际可达**上限 = min(span, URDF上限)。归一量就按这个满量程算。
EFF_HI = {n: min(RAW_MAP[n][0], LIMIT_HI[n]) for n in HAND_JOINTS}


class PoseError(Exception):
    """手势规格写错了(未知状态名、n 越界、拇指给的不是两个值)。"""


# ---------------------------------------------------------------------------
# 状态名 → 归一量
# ---------------------------------------------------------------------------
# 四指(食指/中指/无名/小指):一个关节,直接给 n。
FINGER_STATES = {
    "open": 0.0,        # 伸直
    "relaxed": 0.25,    # 微屈,自然放松
    "half": 0.5,        # 半屈
    "curled": 0.75,     # 大幅弯曲但没到头
    "closed": 1.0,      # 满弯
}

# "limit" 不是常数,要等拇指位置定了才算 —— 在 resolve() 里解析,见 _limit_n。
FINGER_DERIVED = ("limit",)

# `limit` 相对干涉点留的余量(raw 计数)。
#
# ⚠ 为什么不能取 0:可行域表里的 225 是**卡住的位置**,不是"安全到位的位置" ——
#   T3 是把食指命令到 raw 0、它在 225 处停下来的。命令到正好 225 等于命令它贴着
#   拇指;命令到 225 以下就是持续顶(ERROR Bit0 堵转,而过温位不可清除)。
#   10 这个值不是猜的:hand_pinch 实测通过的 raw 234 相对干涉点 225 就是 +9。
LIMIT_MARGIN = 10

# 拇指两个关节(yaw 侧摆 / pitch 弯曲),必须成对给。
#
# 数值不是随手取的,是从**已实测通过的**清单项反算出来的(见 registry.yaml 里
# hand_pinch / hand_close 的注释):
#   opposed  ← hand_pinch  的 [1.112, 0.600] → n=[0.892, 1.0]
#   folded   ← hand_close  的 [1.0,   0.5  ] → n=[0.802, 0.833]
# 所以这两个状态是"真手上跑过、能到位"的,不是纸面推的。改它们要重新上手验。
#
# ⚠ yaw 的语义:n=1 是**对掌位**(拇指立起来、指向食指,能捏),n=0 是拇指躺在
#   掌面里(捏不上,指尖最近 47mm)。别按"1=收起来"理解 —— 这个通道 1 是"摆过去"。
THUMB_STATES = {
    "open": (0.0, 0.0),         # 完全打开:躺在掌面 + 伸直
    "up": (1.0, 0.0),           # 竖起来:对掌位但不弯 —— 点赞手势的拇指
    "opposed": (0.892, 1.0),    # 对掌 + 满弯:捏(实测自 hand_pinch)
    "folded": (0.802, 0.833),   # 收进掌心:握拳(实测自 hand_close)
    "side": (0.5, 0.4),         # 半摆半屈,过渡位,不与食指抢空间
}

# ---------------------------------------------------------------------------
# 拇指-食指可行域(2026-08-06 实测,test_thumb_index_collision.py T3)
# ---------------------------------------------------------------------------
# 拇指收进来时挡住食指,食指弯不到底。表按 raw 存(实测就是 raw,换算成 rad 会
# 引入 URDF/xls span 不一致的误差 —— 那个不一致正是当初误判"四指只能到 95%"的原因)。
#
#   T = max(thumb_yaw_raw, thumb_pitch_raw)   ← 有效变量
#   → 食指 raw 不能低于 index_min
#
# raw 越小越闭合,所以"食指 raw ≥ index_min"就是"食指别太闭合"。
# T 越大 = 拇指越张开 → 挡得越少 → index_min 越小。
FEASIBLE = [(300, 225), (450, 52), (600, 0)]

# ⚠ 这张表的外插限制,别当成全域真值:
#   实测是沿 **yaw = pitch = T** 的对角线扫的。用 max() 概括到非对角线,依据只有
#   一个点(yaw=0 + pitch=1000 时食指能闭到底 —— 说明任一关节张开就把拇指带离
#   食指路径,所以取 max 而不是 min/均值)。对角线之外目前只有这一个数据点。
#   要更准得补一次二维扫描(下一步清单里有)。
INDEX = HAND_JOINTS.index("right_index_1_joint")


def index_min_raw(thumb_yaw_raw: int, thumb_pitch_raw: int) -> int:
    """给定拇指位置,食指 raw 的下界(线性插值 FEASIBLE)。"""
    t = max(thumb_yaw_raw, thumb_pitch_raw)
    if t <= FEASIBLE[0][0]:
        return FEASIBLE[0][1]
    for (t0, v0), (t1, v1) in zip(FEASIBLE, FEASIBLE[1:]):
        if t <= t1:
            return int(round(v0 + (v1 - v0) * (t - t0) / (t1 - t0)))
    return FEASIBLE[-1][1]


# ---------------------------------------------------------------------------
# 换算
# ---------------------------------------------------------------------------
def n_to_rad(name: str, n: float) -> float:
    """归一量 → 弧度。n 越界直接报错,不夹取 —— 手势表是人写的,写错要看见。"""
    if not (0.0 - 1e-9 <= n <= 1.0 + 1e-9):
        raise PoseError(f"{name} 的归一量 {n} 不在 [0,1]")
    return min(max(n, 0.0), 1.0) * EFF_HI[name]


def rad_to_raw(name: str, rad: float) -> int:
    """弧度 → raw(0-1000)。与 inspire_hand.rad_to_raw(clamp_to_urdf=True) 同式。"""
    span, invert = RAW_MAP[name]
    rad = min(max(rad, 0.0), LIMIT_HI[name])
    frac = min(max(rad / span, 0.0), 1.0) if span > 0 else 0.0
    if invert:
        frac = 1.0 - frac
    return int(round(RAW_MIN + frac * (RAW_MAX - RAW_MIN)))


# ---------------------------------------------------------------------------
# 解析一条 pose 规格
# ---------------------------------------------------------------------------
FINGER_KEYS = ("index", "middle", "ring", "pinky")
POSE_KEYS = ("thumb",) + FINGER_KEYS


def raw_to_n(name: str, raw: int) -> float:
    """raw → 归一量。`limit` 要把可行域表(raw)换回 n,走这条。"""
    span, invert = RAW_MAP[name]
    frac = min(max((raw - RAW_MIN) / (RAW_MAX - RAW_MIN), 0.0), 1.0)
    if invert:
        frac = 1.0 - frac
    return min(max(frac * span, 0.0), EFF_HI[name]) / EFF_HI[name]


def _finger_n(key: str, val) -> float:
    """一根四指的取值:状态名或 0-1 的数。`limit` 不走这里(要拇指位置)。"""
    if isinstance(val, str):
        if val in FINGER_DERIVED:
            raise PoseError(f"{key}={val!r} 是派生状态,只能由 resolve() 解析 —— "
                            f"这是内部错误,不该发生")
        if val not in FINGER_STATES:
            raise PoseError(f"{key} 的状态名 {val!r} 未知(可选 "
                            f"{sorted(FINGER_STATES) + list(FINGER_DERIVED)},"
                            f"或直接给 0-1 的数)")
        return FINGER_STATES[val]
    if isinstance(val, bool) or not isinstance(val, (int, float)):
        raise PoseError(f"{key} 要状态名或 0-1 的数,给了 {val!r}")
    return float(val)


def _thumb_n(val) -> tuple[float, float]:
    """拇指取值:状态名,或 [yaw_n, pitch_n] 两个数。"""
    if isinstance(val, str):
        if val not in THUMB_STATES:
            raise PoseError(f"thumb 的状态名 {val!r} 未知(可选 "
                            f"{sorted(THUMB_STATES)},或给 [yaw_n, pitch_n])")
        return THUMB_STATES[val]
    if isinstance(val, (list, tuple)):
        if len(val) != 2:
            raise PoseError(f"thumb 给列表时要 2 个值 [yaw_n, pitch_n],给了 {len(val)}")
        return (_finger_n("thumb.yaw", val[0]), _finger_n("thumb.pitch", val[1]))
    raise PoseError(f"thumb 要状态名或 [yaw_n, pitch_n],给了 {val!r}")


def _limit_n(name: str, thumb_yaw_raw: int, thumb_pitch_raw: int) -> float:
    """`limit`(派生状态) → 归一量。这根手指在给定拇指位置下能到的最闭位置。

    食指 —— 查可行域表,加 LIMIT_MARGIN。
    中指/无名/小指 —— T1 实测(六通道同时下 0,它们都到 0)说明可以满弯,返回 1.0。
      ⚠ 但"六通道同时下 0"走的轨迹不是拇指先到位的轨迹,所以它们的 1.0 **只对**
         速度调好的同时闭合成立。理论上也需要二维扫描来建表,但现在没那张表。
    拇指 —— 不允许 limit(两个关节组合,派生不出单一值)。调用方已拦住。
    """
    if name == "right_index_1_joint":
        lim_raw = index_min_raw(thumb_yaw_raw, thumb_pitch_raw) + LIMIT_MARGIN
        return raw_to_n(name, lim_raw)
    # middle/ring/pinky 实测能到 raw 0,所以 limit = 1.0
    return 1.0


def resolve(pose: dict) -> list[float]:
    """pose 规格 → 6 个弧度(项目顺序)。缺的通道按 0(张开)补。

    不做可行域检查 —— 那是 check_feasible 的事,分开是为了让校验能报出
    "哪个通道差多少",而不是只说"不行"。
    """
    if not isinstance(pose, dict):
        raise PoseError(f"pose 必须是映射,给了 {type(pose).__name__}")
    unknown = set(pose) - set(POSE_KEYS)
    if unknown:
        raise PoseError(f"pose 未知键 {sorted(unknown)}(可选 {list(POSE_KEYS)})")
    # 先解析拇指,后面 `limit` 要它的 raw
    ty, tp = _thumb_n(pose["thumb"]) if "thumb" in pose else (0.0, 0.0)
    ty_raw = rad_to_raw(HAND_JOINTS[0], n_to_rad(HAND_JOINTS[0], ty))
    tp_raw = rad_to_raw(HAND_JOINTS[1], n_to_rad(HAND_JOINTS[1], tp))
    # 四指:先看是 `limit` 还是常规状态/数值
    out_n = [ty, tp]
    for key in FINGER_KEYS:
        val = pose.get(key)
        if val is None or val == "open":
            out_n.append(0.0)
        elif val == "limit":
            out_n.append(_limit_n(f"{key}_proximal_joint", ty_raw, tp_raw))
        else:
            out_n.append(_finger_n(key, val))
    return [n_to_rad(name, n) for name, n in zip(HAND_JOINTS, out_n)]


def check_feasible(rad6) -> str | None:
    """6 弧度是否落在拇指-食指可行域内。可行返回 None,不可行返回一句人话。

    只查这**一条**约束 —— 目前只量过它。中指/无名/小指往外张、不与拇指相交
    (实测同时下 raw 0 时这三根都到底),所以不需要查。别把"只查了一条"读成
    "已经全查过了"。
    """
    raws = [rad_to_raw(n, r) for n, r in zip(HAND_JOINTS, rad6)]
    lo = index_min_raw(raws[0], raws[1])
    if raws[INDEX] < lo:
        t = max(raws[0], raws[1])
        return (f"食指与拇指互顶:拇指在 T={t}(yaw {raws[0]}/pitch {raws[1]})时"
                f"食指 raw 不能低于 {lo},当前命令到 {raws[INDEX]}"
                f"(差 {lo - raws[INDEX]})。放松食指或让拇指张开一点。")
    return None


def compile_pose(pose: dict) -> tuple[list[float], str | None]:
    """resolve + check_feasible 一步走。返回 (6 弧度, 不可行原因或 None)。"""
    rad6 = resolve(pose)
    return rad6, check_feasible(rad6)


# ---------------------------------------------------------------------------
# 与 inspire_hand 核对(抄表必须配的那道检查)
# ---------------------------------------------------------------------------
def verify_against_driver() -> list[str]:
    """按**文件路径**加载 sim/inspire_hand.py 逐项核对。返回不一致列表。

    不用 `import inspire_hand` —— 那要把 sim/ 塞进 sys.path,而 sim/schema.py 会
    遮蔽 skills/schema.py。importlib.spec_from_file_location 不碰 sys.path。
    """
    import importlib.util
    import sys

    drv_path = Path(__file__).resolve().parent.parent / "inspire_hand.py"
    if not drv_path.exists():
        return [f"找不到 {drv_path}"]
    spec = importlib.util.spec_from_file_location("_ih_verify", drv_path)
    if spec is None or spec.loader is None:
        return [f"无法加载 {drv_path}"]
    mod = importlib.util.module_from_spec(spec)
    # 必须先进 sys.modules 再 exec —— @dataclass 会用 cls.__module__ 反查自己所在的
    # 模块字典(dataclasses._is_type),查不到就 AttributeError。名字带下划线前缀,
    # 避免和真的 `import inspire_hand` 撞。
    sys.modules[spec.name] = mod
    try:
        spec.loader.exec_module(mod)
    finally:
        sys.modules.pop(spec.name, None)

    bad = []
    if list(mod.HAND_JOINTS) != HAND_JOINTS:
        bad.append(f"HAND_JOINTS 顺序不一致:\n  驱动 {list(mod.HAND_JOINTS)}\n  本表 {HAND_JOINTS}")
    for name in HAND_JOINTS:
        d_span, d_inv = mod.RAW_MAP[name]
        span, inv = RAW_MAP[name]
        if abs(d_span - span) > 1e-9 or d_inv != inv:
            bad.append(f"RAW_MAP[{name}] 驱动=({d_span},{d_inv}) 本表=({span},{inv})")
        d_hi = mod.HAND_LIMITS[name][1]
        if abs(d_hi - LIMIT_HI[name]) > 1e-9:
            bad.append(f"LIMIT_HI[{name}] 驱动={d_hi} 本表={LIMIT_HI[name]}")
    return bad


# 自检用的样例手势 —— 前两条应与 registry.yaml 现值一致(反算的来源)。
DEMO = {
    # 捏用 limit:食指闭到"刚够碰上拇指" —— 有东西就停在东西上,没东西就停在
    # 干涉点前 10 counts。这个位置是**推出来的**,不用再上手量。
    "捏 (hand_pinch)":  {"thumb": "opposed", "index": "limit"},
    # 握拳不用 limit:它要留足余量(实测 raw 320,比干涉点宽 85),不是贴着拇指。
    "握拳 (hand_close)": {"thumb": "folded", "index": 0.680,
                          "middle": "closed", "ring": "closed", "pinky": "closed"},
    "张开":             {"thumb": "open"},
    "点赞":             {"thumb": "up", "index": "closed", "middle": "closed",
                          "ring": "closed", "pinky": "closed"},
    "数字1":            {"thumb": "folded", "index": "open", "middle": "closed",
                          "ring": "closed", "pinky": "closed"},
    "OK":               {"thumb": "opposed", "index": 0.767, "middle": "relaxed",
                          "ring": "relaxed", "pinky": "relaxed"},
    "★不可行(食指满弯+拇指对掌)": {"thumb": "opposed", "index": "closed"},
}


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--verify", action="store_true", help="与 inspire_hand 核对表")
    args = ap.parse_args(argv)

    if args.verify:
        bad = verify_against_driver()
        if bad:
            print("表与 inspire_hand 不一致 —— 改了驱动就要同步这里:")
            for b in bad:
                print("  ✗ " + b)
            return 1
        print("✓ HAND_JOINTS / RAW_MAP / LIMIT_HI 与 inspire_hand 一致")
        return 0

    print("有效满量程 EFF_HI = min(span, URDF上限):")
    for n in HAND_JOINTS:
        mark = "  ← 被 URDF 限位截掉" if EFF_HI[n] < RAW_MAP[n][0] - 1e-9 else ""
        print(f"  {n:30s} {EFF_HI[n]:.5f} rad  (span {RAW_MAP[n][0]:.5f}){mark}")

    print("\n可行域(T = max(thumb_yaw_raw, thumb_pitch_raw)):")
    print(f"  {'T':>5} {'食指 raw 下界':>14}")
    for t in (0, 150, 300, 375, 450, 525, 600, 800, 1000):
        print(f"  {t:5d} {index_min_raw(t, 0):>14d}")

    print("\n样例手势:")
    for label, pose in DEMO.items():
        rad6, why = compile_pose(pose)
        raws = [rad_to_raw(n, r) for n, r in zip(HAND_JOINTS, rad6)]
        print(f"\n  {label}")
        print(f"    rad  [{', '.join(f'{r:.3f}' for r in rad6)}]")
        print(f"    raw  {raws}")
        print(f"    可行 {'✓' if why is None else '✗ ' + why}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
