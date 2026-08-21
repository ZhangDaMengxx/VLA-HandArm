#!/usr/bin/env python3
"""臂轨迹离线预处理:去视觉抖动 → 限速 → 抽路点。**纯离线,不碰硬件。**

为什么必须有这一步(而不是直接把 npz 逐帧发下去):

`analyze_traj_demand.py` 测出来的分布是 p50≈4.7 / p95≈50 / **max 628** deg/s。
那个 628 是**伪影** —— 单帧 33ms 内跳 21°,人的手腕做不到,是视觉管线的抖动。
不管下游用 move_j 还是 move_js 还是 move_mit,把这种尖峰原样送下去都是拿阶跃
猛甩真臂。所以尖峰必须在**离线**这一层就消掉,不能指望臂的控制器替我们兜。

四步,每步都可单独关掉看效果:
  1. 夹限位   —— 超 NERO_ARM_LIMITS 的直接夹,不寄望下游
  2. 零相位低通 —— filtfilt(不是 lfilter)。离线才能用零相位:
                   普通低通会引入相位滞后,那等于凭空给轨迹加延迟
  3. 限速     —— 滤完仍超 --max-vel 的段落做迭代收缩
  4. 抽路点   —— 关节空间 RDP,容差直接就是形状误差上界(rad)

⚠ 输出的路点**保留时间戳**。抽稀是空间上的,不是时间上的 —— 路点之间该花多久
就是多久,不能因为点少了就走快。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt

SIM = Path(__file__).resolve().parent
OUT = SIM / "out"
RAD2DEG = 180.0 / math.pi

# 从 nero_arm 拿限位,不抄第二份 —— 抄了必然有一天两边不一致
import sys
sys.path.insert(0, str(SIM))
from nero_arm import (NERO_ARM_LIMITS, ARM_JOINTS,     # noqa: E402
                      NERO_CPV_VV_DEG, NERO_CPV_AC_DEG, NERO_RATED_SPD_DEG)
from capture_bundle import discover_trajectory_npz  # noqa: E402

VV = np.array(NERO_CPV_VV_DEG)      # CPV 轮廓速度上限 deg/s(出厂默认)
AC = np.array(NERO_CPV_AC_DEG)      # CPV 加/减速上限 deg/s²(出厂默认)


def cpv_fit(arm: np.ndarray, t: np.ndarray) -> dict:
    """轨迹相对 CPV **出厂默认**上限的占用比。>1 就是超了。

    为什么按**逐关节**算而不是把 7 个关节混在一起取分位数:轨迹能不能跑取决于
    **最差的那一个关节**。我一开始混着算得出"速度只超 1.38×",而按逐关节算
    joint1 实际超 **4.39×** —— joint1 是底座回转、行程最大,它一个就决定可行性。
    混合分位数把它埋掉了。
    """
    dt = np.diff(t)[:, None]
    vel = np.abs(np.diff(arm, axis=0)) / dt * RAD2DEG                  # deg/s
    acc = np.abs(np.diff(arm, 2, axis=0)) / dt[:-1] ** 2 * RAD2DEG     # deg/s²
    rv, ra = vel / VV, acc / AC
    worst_j = int(np.argmax(np.percentile(rv, 95, axis=0)))
    return {
        "vel_p95_ratio": round(float(np.percentile(rv, 95)), 3),
        "vel_max_ratio": round(float(rv.max()), 3),
        "acc_p95_ratio": round(float(np.percentile(ra, 95)), 3),
        "acc_max_ratio": round(float(ra.max()), 3),
        "worst_joint": ARM_JOINTS[worst_j],
        "worst_joint_vel_p95_deg": round(float(np.percentile(vel[:, worst_j], 95)), 1),
        "worst_joint_rated_pct": round(
            float(np.percentile(vel[:, worst_j], 95) / NERO_RATED_SPD_DEG[worst_j] * 100), 1),
        "fits_defaults": bool(np.percentile(rv, 95) <= 1.0
                              and np.percentile(ra, 95) <= 1.0),
    }


def vel_deg_s(arm: np.ndarray, fps: float) -> np.ndarray:
    """逐帧角速度绝对值 (N-1, 7),deg/s。"""
    return np.abs(np.diff(arm, axis=0)) * fps * RAD2DEG


def stat(v: np.ndarray, over: tuple[float, float] = (90.0, 180.0)) -> dict:
    """分布 + 两个越界计数。

    ⚠ 越界判定带 1e-6 容差。retime 之后速度**恰好**顶在上界上,浮点残差
    (实测 3.8e-12)会让裸 `v > 90` 数出 69 个"越界" —— 而 max 明明就是 90.0。
    那种自相矛盾的输出比没有输出更糟。
    """
    eps = 1e-6
    return {"p50": float(np.percentile(v, 50)), "p95": float(np.percentile(v, 95)),
            "max": float(v.max()),
            "over90": int((v > over[0] + eps).sum()),
            "over180": int((v > over[1] + eps).sum()), "n": int(v.size)}


def clamp_limits(arm: np.ndarray) -> tuple[np.ndarray, int]:
    """夹到 NERO_ARM_LIMITS。返回 (夹后, 被夹的元素数)。"""
    out = arm.copy()
    hit = 0
    for j, (lo, hi) in enumerate(NERO_ARM_LIMITS):
        before = out[:, j].copy()
        np.clip(out[:, j], lo, hi, out=out[:, j])
        hit += int((before != out[:, j]).sum())
    return out, hit


def lowpass(arm: np.ndarray, fps: float, cut_hz: float, order: int = 4) -> np.ndarray:
    """零相位低通。

    ⚠ 用 filtfilt 不用 lfilter:filtfilt 正反各滤一次,相位完全抵消。
    lfilter 会引入群延迟(4 阶巴特沃斯在 3Hz 处约 30-40ms),那等于凭空给轨迹
    加一段滞后 —— 我们本来就在担心臂跟不上,不能自己再加延迟。
    代价是 filtfilt **只能离线**(要看到未来的样本),正好这一步就是离线的。

    截止取 3Hz 的理由:源是 30fps → 奈奎斯特 15Hz。人手腕摆动主要能量在 3Hz 以下,
    而视觉抖动是**单帧**尖峰(能量在接近奈奎斯特的高频)。3Hz 把后者砍掉、前者留住。
    """
    nyq = fps / 2.0
    wn = min(max(cut_hz / nyq, 1e-3), 0.99)
    b, a = butter(order, wn, btype="low")
    # padlen 默认 3*max(len(a),len(b)),短轨迹会报错;按长度收一下
    pad = min(3 * max(len(a), len(b)), arm.shape[0] - 1)
    return filtfilt(b, a, arm, axis=0, padlen=max(pad, 0))


def retime(arm: np.ndarray, fps: float, max_deg_s: float) -> tuple[np.ndarray, dict]:
    """**拉时间**让速度落到上界内。返回 (每帧时刻 t[N], 报告)。轨迹本身不动。

    ⚠ 这里第一版写错过,记下来免得再犯:我当时把逐帧增量夹到 step_max 再累加回去
    ("砍位移")。速度上界确实精确了,但那是在**改轨迹形状** —— 实测末端偏到
    **76°**(joint6)。因为这些轨迹是真的需要 >90deg/s,不是个别尖峰;
    砍掉的位移永远补不回来,越砍越落后,终点直接跑飞。

    正确做法:形状一个字不改,**该慢就慢**。每段所需时间取
        dt_i = max_j |Δq_ij| / v_max
    再和原本的 1/fps 取大 —— 只拉长不压缩。得到非均匀时间轴:
      · 形状**完全**保真(偏差恒为 0)
      · 速度上界处处成立
      · 代价只有一个:总时长变长,而且变长多少是可报的

    这也正好和路点包的 `dt_ms` 对上:下游本来就是按每段时间走,不要求等间隔。

    ## ⚠ 两个已知缺陷(2026-08-06,读了真臂的关节级限制之后发现)

    **1. 它只约束速度,而真正卡住的是加速度。**
    `cpv_fit` 实测:速度 p95 只有上限的 **0.841×**(没超),加速度 p95 是 **2.376×**。
    也就是说按速度拉时间**拉不到点子上** —— 平常动作压根没顶到速度上限。
    拉长 k 倍时速度降 k 倍但加速度降 **k²** 倍,所以要按加速度算的话
    需要的倍数是 √2.376 ≈ **1.54×**,而按速度算会算出"不用拉"。
    要修就得把 `need` 改成同时满足两个约束(速度和加速度各算一个下界再取大)。

    **2. `max_deg_s` 是**一个标量**,而真实上限是逐关节的**
    (`NERO_CPV_VV_DEG`:J1–J4 36.0、J5–J7 44.9 deg/s)。
    现在 `d.max(axis=1)` 拿全关节最大位移除同一个上界 —— 等于让最紧的关节说话,
    偏保守但不精确。

    这两条**还没造成后果**:retime 是 opt-in(`--retime`)而且目前没在用
    (录制放慢之后时间轴保持均匀更划算)。但要用它之前必须先修。
    """
    d = np.abs(np.diff(arm, axis=0))                 # (N-1, 7) rad
    need = d.max(axis=1) / (max_deg_s / RAD2DEG)     # 每段所需秒数
    base = 1.0 / fps
    dt = np.maximum(need, base)
    t = np.concatenate([[0.0], np.cumsum(dt)])
    n_slow = int((need > base + 1e-12).sum())
    # ⚠ 源时长用 (N-1)/fps 不是 N/fps:t 累的是 N-1 段间隔,拿 N/fps 去比会
    # 差一段,算出 stretch_x = 0.999 这种"retime 把轨迹压短了"的假象(实测踩过)。
    # N 帧只界定 N-1 个间隔 —— 这是 fencepost,不是精度问题。
    dur_src = float((arm.shape[0] - 1) / fps)
    return t, {"max_deg_s": max_deg_s,
               "segments_slowed": n_slow,
               "segments_total": int(dt.size),
               "dur_src_s": round(dur_src, 3),
               "dur_out_s": round(float(t[-1]), 3),
               "stretch_x": round(float(t[-1] / dur_src), 3) if dur_src > 0 else 1.0,
               "slowest_dt_ms": round(float(dt.max() * 1000), 1)}


def rdp_indices(arm: np.ndarray, t: np.ndarray, tol_rad: float) -> list[int]:
    """按**时间参数化**的 RDP,返回保留的帧下标(含首尾)。

    容差 = 回放重建误差的上界(单关节,rad)。

    ⚠ 第一版用的是**垂距**(点到线段的最短距离),错了。实测:垂距容差 0.573°
    时实际重建误差 max **5.78°**,超 10 倍。原因是两个度量不是一回事 ——
    垂距允许落在线段上任意最近点,而回放是按**时刻**插值:
        q̂(t_k) = q_i + (t_k-t_i)/(t_j-t_i) · (q_j-q_i)
    该时刻对应的点不一定是最近点。所以要直接量这个式子的误差,
    量什么就保证什么,别用一个相关但不相等的量去代理。

    另外用 **max-norm**(逐关节取最大)而不是欧氏范数:限位、容差、
    "偏了多少度"这些都是逐关节的概念,7 维欧氏距离没有物理意义。
    """
    n = arm.shape[0]
    if n <= 2:
        return list(range(n))
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        dt = t[j] - t[i]
        s = ((t[i:j + 1] - t[i]) / dt if dt > 1e-12
             else np.zeros(j - i + 1))
        pred = arm[i] + s[:, None] * (arm[j] - arm[i])
        d = np.abs(arm[i:j + 1] - pred).max(axis=1)          # max-norm
        k = int(np.argmax(d))
        if d[k] > tol_rad:
            keep[i + k] = True
            stack.extend([(i, i + k), (i + k, j)])
    return np.flatnonzero(keep).tolist()


# /2:加 mode / uniform_dt_ms / cpv_fit,并且默认不再抽稀(全帧流)。
# 现在改 schema 是免费的 —— 播放器还没写,没有旧消费者要兼容。
SCHEMA = "arm_traj_pack/2"


def build_pack(name: str, arm: np.ndarray, t: np.ndarray, idx: list[int],
               fps: float, report: dict) -> dict:
    """路点包。**每个路点带自己的时刻** —— 抽稀是空间上的,不是时间上的。

    抽完点少了不等于该走快:两个路点之间原来花多久,现在还得花多久。
    时间轴来自 retime()(非均匀),所以 `t` 直接取那个数组,**不能**用 i/fps 算。

    `dt_ms` 是到下一个路点的时间。下游播放器按它决定何时发下一个点 ——
    但**到位驱动优先于时钟驱动**:move_j 是规划请求,没到位就发下一个会抢断。
    dt_ms 是"最少要等这么久",不是"到点就发"。
    """
    # ⚠ `t_ns`(整数纳秒)是**权威时刻**,`t` / `dt_ms` 是便利字段。
    # 播放器必须按 t_ns 定位,**不要累加 dt_ms**。
    # 为什么:30fps 真周期 33.3333…ms,是无限循环小数,**任何有限位十进制都表示不了**。
    # dt_ms 取整成 33 时 780 帧累加漂 260ms(1% 时长);取 3 位小数降到 0.26ms,
    # 但那只是把症状压小,累加放大的结构还在(素材换成 24 或 120fps 又会露出来)。
    # 整数纳秒是行业做法:ROS 的 builtin_interfaces/Duration 是 int32 sec +
    # uint32 nanosec,MP4/FFmpeg 用 timescale+整数 tick,PTP 用秒+纳秒 —— 没有一个
    # 用十进制小数存时间。残差 0.333ns/帧,2400 帧才 0.0008ms,
    # 比臂的控制周期(4.5ms)细 5600 倍。
    #
    # 顺带:均匀时间轴下最精确的其实是**有理数** —— 第 i 帧就是 i/fps 秒,
    # 一次算出、零误差。包里 `i` 和 `fps_src` 都在,播放器可以自己算。
    # t_ns 仍然存,是为了让播放器对"均匀"和"retime 过的非均匀"只有一条代码路径。
    pts = []
    for k, i in enumerate(idx):
        nxt = idx[k + 1] if k + 1 < len(idx) else None
        pts.append({
            "i": int(i),
            # t_ns 是**权威时刻**(整数纳秒,对齐 ROS builtin_interfaces/Duration)。
            # t 和 dt_ms 是便利字段,给人看的。播放器按 t_ns 定位。
            "t_ns": int(round(float(t[i]) * 1e9)),
            "t": round(float(t[i]), 5),
            "dt_ms": None if nxt is None else round((t[nxt] - t[i]) * 1000, 3),
            "rad": [round(float(v), 5) for v in arm[i]],
        })
    # ⚠ 起点**不在零位**。实测 robot_traj_nero_gripper_rgbd 首帧 joint1 = -111°。
    # 播放前必须先把臂从**当前位置**挪到这个起点,而那一步不能当成包里的第一个
    # 路点顺手发掉 —— 那等于让臂从任意姿态猛甩到 -111°,路径不可预测。
    # 所以显式导出 approach 段,让播放器把它当独立的、要单独确认的一步。
    q0 = arm[idx[0]]
    # 时间轴均匀吗 —— 均匀时给出 uniform_dt_ms,播放器可以直接按固定周期走,
    # 不必逐点看 dt_ms。非均匀(retime 过 / 抽稀过)时为 null,必须逐点读。
    dts = np.diff(t[idx])
    uniform = bool(len(dts) and (dts.max() - dts.min()) < 1e-9)
    return {"schema": SCHEMA, "name": name, "joints": list(ARM_JOINTS),
            # stream = 全帧、均匀周期,逐帧流式发(move_cpv_pos)
            # waypoints = 抽稀过,点之间要插值或等到位
            "mode": "stream" if (uniform and len(idx) == arm.shape[0]) else "waypoints",
            "uniform_dt_ms": round(float(dts[0] * 1000), 4) if uniform else None,
            "fps_src": fps, "n_src": int(arm.shape[0]),
            "n_waypoints": len(idx),
            "duration_s": round(float(t[-1]), 3),
            "approach": {
                "rad": [round(float(v), 5) for v in q0],
                "max_from_zero_deg": round(float(np.abs(q0).max() * RAD2DEG), 2),
                "note": "播放前需先到位。从当前姿态到这里的路径由臂自己规划,"
                        "必须低速 + 有人看着 + 单独确认",
            },
            "prep": report, "waypoints": pts}


def _row(tag: str, s: dict) -> str:
    return (f"{tag:<14}{s['p50']:>8.1f}{s['p95']:>8.1f}{s['max']:>9.1f}"
            f"{s['over90']:>9d}{s['over180']:>9d}")


def process(path: Path, a) -> dict:
    d = np.load(path, allow_pickle=True)
    if "arm" not in d.files:
        return {}
    arm0 = np.asarray(d["arm"], dtype=float)
    fps = a.fps
    print(f"\n=== {path.name}  {arm0.shape[0]} 帧 / {arm0.shape[0]/fps:.1f}s ===")
    print(f"{'阶段':<14}{'p50':>8}{'p95':>8}{'max':>9}{'>90':>9}{'>180':>9}   (deg/s)")
    print(_row("原始", stat(vel_deg_s(arm0, fps))))

    rep: dict = {"fps": fps}
    arm = arm0
    arm, n_clamp = clamp_limits(arm)
    rep["limit_clamped_elems"] = n_clamp
    if n_clamp:
        print(_row("夹限位", stat(vel_deg_s(arm, fps))) + f"   夹了 {n_clamp} 个元素")

    if not a.no_filter:
        arm = lowpass(arm, fps, a.cut_hz)
        rep["lowpass_hz"] = a.cut_hz
        print(_row(f"低通{a.cut_hz:g}Hz", stat(vel_deg_s(arm, fps))))
        # ⚠ 低通**之后必须再夹一次**。零相位 filtfilt 在被夹出来的那些
        # 折角处会过冲(Gibbs),把值顶回限位外面 —— 实测 robot_traj_nero_inspire
        # 低通后 joint5 超限 2.42°、19 个元素越界,而上面那次夹限位已经报"夹干净了"。
        # 这条是 2026-08-06 写联合回放器时发现的:combo_player 入口校验直接拒了包,
        # 而本文件的 docstring 写着「超 NERO_ARM_LIMITS 的直接夹,**不寄望下游**」——
        # 承诺和事实不符。第一次夹留着是为了报"原始素材本身越界多少",
        # 这一次是**最终闸门**。
        arm, n_post = clamp_limits(arm)
        rep["limit_clamped_after_lowpass"] = n_post
        if n_post:
            print(f"{'夹限位(低通后)':<14}低通过冲顶出限位 {n_post} 个元素,已夹回")

    # 形状代价:只有夹限位和低通会改形状。retime 不改(它只动时间轴),
    # 所以这一行必须在 retime **之前**算 —— 放后面会把"没改"说成"改了"。
    dev = np.abs(arm - arm0)
    rep["shape_dev_deg_p95"] = round(float(np.percentile(dev, 95) * RAD2DEG), 3)
    rep["shape_dev_deg_max"] = round(float(dev.max() * RAD2DEG), 3)
    print(f"{'形状代价':<14}相对原始:p95 {np.percentile(dev,95)*RAD2DEG:.2f}°  "
          f"max {dev.max()*RAD2DEG:.2f}° ({ARM_JOINTS[int(np.argmax(dev.max(0)))]})")

    t = np.arange(arm.shape[0]) / fps
    if a.retime:
        t, trep = retime(arm, fps, a.max_vel)
        rep["retime"] = trep
        # 非均匀时间轴上的速度:逐段 Δq/Δt
        vv = np.abs(np.diff(arm, axis=0)) / np.diff(t)[:, None] * RAD2DEG
        print(_row(f"拉时间{a.max_vel:g}", stat(vv))
              + f"   {trep['segments_slowed']}/{trep['segments_total']} 段放慢,"
                f"时长 {trep['dur_src_s']}s → {trep['dur_out_s']}s "
                f"(×{trep['stretch_x']}),形状偏差 0")

    # CPV 出厂默认上限装不装得下 —— 这是决定"要不要 retime / 要不要放慢重录"的依据
    fit = cpv_fit(arm, t)
    rep["cpv_fit"] = fit
    print(f"{'CPV默认上限':<14}速度 p95 {fit['vel_p95_ratio']:.2f}×  "
          f"加速度 p95 {fit['acc_p95_ratio']:.2f}×  "
          f"最差 {fit['worst_joint']}({fit['worst_joint_vel_p95_deg']:.0f}deg/s"
          f"={fit['worst_joint_rated_pct']:.0f}%额定)")
    if not fit["fits_defaults"]:
        need = max(fit["vel_p95_ratio"], math.sqrt(max(fit["acc_p95_ratio"], 1e-9)))
        print(f"{'':14}⚠ 超默认上限 —— 录制放慢 {need:.1f}× 即可落进去"
              f"(或 --retime 事后拉,或提高臂的 vv/ac)")

    far = float(np.abs(arm[0]).max() * RAD2DEG)
    print(f"{'起点':<14}距零位最大单关节 {far:.1f}° "
          f"({ARM_JOINTS[int(np.argmax(np.abs(arm[0])))]})"
          + ("  ⚠ 播放前需要独立的 approach 段" if far > 15 else ""))

    if a.rdp:
        idx = rdp_indices(arm, t, a.tol)
        rep["rdp_tol_rad"] = a.tol
        print(f"{'抽路点':<14}{len(idx)} 点 / {arm.shape[0]} 帧 "
              f"(压 {100*(1-len(idx)/arm.shape[0]):.1f}%),容差 {a.tol:g}rad "
              f"= {a.tol*RAD2DEG:.2f}°")
    else:
        # 不抽稀 = 全帧都是"路点"。流式发的时候本来就该逐帧发,
        # 这样也没有抽稀误差(原来 0.04rad 容差对应末端 4.7mm,现在是 0)。
        idx = list(range(arm.shape[0]))
        print(f"{'全帧流':<14}{len(idx)} 帧,均匀 {1000/fps:.2f}ms,无抽稀误差")
    return {"arm": arm, "t": t, "idx": idx, "rep": rep,
            "name": path.stem, "fps": fps}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fps", type=float, default=30.0,
                    help="源帧率(npz 无时间列,必须外部给)")
    ap.add_argument("--cut-hz", type=float, default=3.0,
                    help="低通截止。人手腕能量主要 <3Hz,视觉抖动在高频")
    # ⚠ 默认从 36.0 改过来(2026-08-06)。原来是 90.0,注释写"需求侧 p95≈50,
    # 留一倍余量" —— 那是个和臂无关的**猜测**。真臂的 CPV 轮廓速度上限读出来是
    # J1–J4 36.0 / J5–J7 44.9 deg/s,**比猜的 90 更低**,所以 90 那个"上界"
    # 压根不是上界。取七个关节里最小的 36.0 当标量默认(retime 只吃一个标量,
    # 见该函数 docstring 的缺陷 2)。
    ap.add_argument("--max-vel", type=float, default=min(NERO_CPV_VV_DEG),
                    help="单关节速度上界 deg/s。默认 = CPV 轮廓上限里最小的那个"
                         "(真臂读出来的,不是猜的)")
    ap.add_argument("--tol", type=float, default=0.01,
                    help="RDP 容差(rad)。0.01rad=0.57°,即形状误差上界")
    ap.add_argument("--no-filter", action="store_true", help="跳过低通(看对比用)")
    # ⚠ retime 和 RDP 现在**默认关闭**,理由:
    # 走 move_cpv_pos(逐关节位置伺服,重设目标只是重定向、不打断规划)可以直接
    # 30fps 流式发全帧。这两步是为 move_j 那条路准备的 —— move_j 每条都是规划请求,
    # 逐帧发会互相打断,所以当时必须抽成稀疏路点 + 等到位。
    # 而且**录制时放慢 2 倍**之后速度和加速度本来就落在 CPV 默认上限内
    # (实测 vel/vv p95=0.43、acc/ac p95=0.60),retime 压根没事可做。
    # 保留这两个开关是因为:①某条素材录得偏快时 retime 是现成兜底;
    # ②离线轨迹模式(一次上传 256 点、臂自己插值)必须抽稀。
    ap.add_argument("--retime", action="store_true",
                    help="拉时间让速度落到 --max-vel 内。默认关(录制放慢后不需要)")
    ap.add_argument("--rdp", action="store_true",
                    help="RDP 抽稀成稀疏路点。默认关(流式不需要,离线轨迹模式才要)")
    ap.add_argument("--emit", action="store_true",
                    help="写出 out/arm_pack_<name>.json。默认只报告不落盘")
    ap.add_argument("--capture-root", default=None,
                    help="Capture Bundle；不传则读取 datasets/captures/ 中最新一次")
    ap.add_argument("--legacy-out", action="store_true", help="显式扫描旧 src/out")
    ap.add_argument("files", nargs="*", help="显式轨迹路径；默认扫描当前 Capture")
    a = ap.parse_args()

    try:
        paths = [Path(f) for f in a.files] or discover_trajectory_npz(
            capture_root=a.capture_root,
            legacy_out=a.legacy_out,
        )
    except (FileNotFoundError, ValueError) as exc:
        print(f"没找到 npz：{exc}")
        return 1
    if not paths:
        print("没找到 npz。先跑 derive_embodiment.py --emit-traj")
        return 1

    done = 0
    for p in paths:
        r = process(p, a)
        if not r:
            print(f"跳过 {p.name}(无 arm 字段)")
            continue
        if a.emit:
            pack = build_pack(r["name"], r["arm"], r["t"], r["idx"],
                              r["fps"], r["rep"])
            dst = OUT / f"arm_pack_{r['name']}.json"
            dst.write_text(json.dumps(pack, ensure_ascii=False, indent=1),
                           encoding="utf-8")
            print(f"{'写出':<14}{dst.relative_to(SIM)}  "
                  f"{dst.stat().st_size/1024:.1f}KB")
        done += 1

    if not a.emit and done:
        print("\n(只报告没落盘。加 --emit 写 out/arm_pack_*.json)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
