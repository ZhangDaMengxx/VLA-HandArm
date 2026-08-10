#!/usr/bin/env python3
"""prep_arm_traj.py 的性质测试。**纯离线,不碰硬件。**

重点验三条**容易写错**的性质(每条都是实际踩过的):
  1. retime 只动时间轴,形状偏差**恒为 0** —— 第一版"砍位移"末端偏到 76°
  2. RDP 容差是**回放重建误差**的上界 —— 第一版用垂距,实测超 10 倍
  3. 越界计数带容差 —— retime 后速度恰好顶在上界,裸比较会假报 69 个越界
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

SIM = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM))

import prep_arm_traj as P                                  # noqa: E402
from nero_arm import NERO_ARM_LIMITS                       # noqa: E402

_FAILS: list[str] = []
FPS = 30.0
# 真轨迹样本。没有就跳过那一组(而不是假装通过)。
_cands = sorted((SIM / "out").glob("robot_traj_*.npz"))
OUT_NPZ = _cands[0] if _cands else None


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print(f"  ok   {name}")
    else:
        print(f"  FAIL {name}  {detail}")
        _FAILS.append(name)


def synth(n: int = 300, seed: int = 7) -> np.ndarray:
    """合成轨迹:慢摆动 + 几个单帧尖峰(冒充视觉抖动)。

    用合成而不是只用真 npz:尖峰位置已知,才能断言"低通确实把它削掉了"。
    """
    rng = np.random.default_rng(seed)
    t = np.arange(n) / FPS
    arm = np.stack([0.5 * np.sin(2 * np.pi * 0.4 * t + j) for j in range(7)], axis=1)
    for k in (50, 120, 200):                    # 单帧尖峰,~0.35rad = 20°/帧
        arm[k] += 0.35
    return arm + rng.normal(0, 0.001, arm.shape)


def main() -> int:
    arm0 = synth()

    print("\n=== 夹限位 ===")
    bad = arm0.copy()
    bad[10, 5] = 5.0                            # joint6 限 ±0.9599
    out, hit = P.clamp_limits(bad)
    lo, hi = NERO_ARM_LIMITS[5]
    check("超限被夹到边界", abs(out[10, 5] - hi) < 1e-12, repr(out[10, 5]))
    check("夹的元素数被报出", hit >= 1, str(hit))
    check("没超限的不动", np.allclose(out[11:], bad[11:]))

    print("\n=== 低通削尖峰,且零相位不引入滞后 ===")
    v0 = P.vel_deg_s(arm0, FPS)
    f = P.lowpass(arm0, FPS, 3.0)
    v1 = P.vel_deg_s(f, FPS)
    check("尖峰被削低", v1.max() < v0.max() * 0.6, f"{v0.max():.1f} → {v1.max():.1f}")
    # 零相位:滤前滤后的**相位**该对齐。用互相关峰值在 0 处来验。
    a = arm0[:, 0] - arm0[:, 0].mean()
    b = f[:, 0] - f[:, 0].mean()
    xc = np.correlate(b, a, mode="same")
    lag = int(np.argmax(xc)) - len(a) // 2
    check("零相位:互相关峰在 lag=0", lag == 0, f"lag={lag} 帧")
    # 反证:普通 lfilter 应当能看出滞后 —— 说明上面这条不是恒真
    from scipy.signal import butter, lfilter
    bb, aa = butter(4, 3.0 / (FPS / 2), btype="low")
    g = lfilter(bb, aa, arm0[:, 0])
    xc2 = np.correlate(g - g.mean(), a, mode="same")
    lag2 = int(np.argmax(xc2)) - len(a) // 2
    check("反证:lfilter 确有滞后(所以上一条有意义)", lag2 > 0, f"lag={lag2} 帧")

    print("\n=== retime:形状偏差恒为 0,速度上界成立 ===")
    # 这是第一版最大的错:"砍位移"让末端偏到 76°。retime 不碰 arm。
    before = f.copy()
    t, rep = P.retime(f, FPS, 90.0)
    check("轨迹数组一个字没改", np.array_equal(f, before))
    vv = np.abs(np.diff(f, axis=0)) / np.diff(t)[:, None] * P.RAD2DEG
    check("速度处处 ≤ 90(含浮点容差)", vv.max() <= 90.0 + 1e-6, f"{vv.max():.6f}")
    check("时间单调递增", np.all(np.diff(t) > 0))
    check("只拉长不压缩", np.all(np.diff(t) >= 1.0 / FPS - 1e-12))
    check("末端仍是原末端(没有末端漂移)",
          np.array_equal(f[-1], before[-1]))
    # ⚠ 必须 ≥1:retime 只拉长。第一版 dur_src_s 用 N/fps 而 t 只累 N-1 段,
    # 算出 0.999 —— "retime 把轨迹压短了"的假象。fencepost,不是精度问题。
    check("拉伸倍数 ≥1(只拉长)", rep["stretch_x"] >= 1.0, str(rep["stretch_x"]))
    check("源时长用 (N-1)/fps",
          abs(rep["dur_src_s"] - (f.shape[0] - 1) / FPS) < 1e-3, str(rep["dur_src_s"]))

    print("\n=== 越界计数带容差(否则和 max 自相矛盾) ===")
    s = P.stat(vv)
    check("retime 后 >90 计数为 0", s["over90"] == 0,
          f"over90={s['over90']} 而 max={s['max']:.9f}")

    print("\n=== RDP 容差 = 回放重建误差上界 ===")
    for tol in (0.02, 0.01, 0.004):
        idx = P.rdp_indices(f, t, tol)
        wt, wq = t[idx], f[idx]
        rec = np.stack([np.interp(t, wt, wq[:, j]) for j in range(7)], axis=1)
        err = float(np.abs(rec - f).max())
        check(f"tol={tol}: 重建误差 {err*P.RAD2DEG:.3f}° ≤ 界 {tol*P.RAD2DEG:.3f}°",
              err <= tol + 1e-9, f"{err:.6f} > {tol}")
    # 容差越小点越多 —— 单调性,写错方向的话这条会挂
    counts = [len(P.rdp_indices(f, t, x)) for x in (0.02, 0.01, 0.004)]
    check("容差越小路点越多", counts[0] < counts[1] < counts[2], str(counts))
    check("首尾一定保留",
          P.rdp_indices(f, t, 0.01)[0] == 0 and
          P.rdp_indices(f, t, 0.01)[-1] == f.shape[0] - 1)

    print("\n=== RDP 在**真轨迹**上也必须守界 ===")
    # ⚠ 这一组不能只用合成轨迹。反向验证时实测:把 rdp_indices 改回错的垂距版,
    # 合成轨迹上重建误差 0.368° —— **碰巧**也在 0.573° 界内,测试全绿。
    # 真 npz 上同一个错误版本是 5.781°,超界 10 倍。
    # 原因:合成轨迹平滑且 retime 几乎不改时间轴,垂距和按时间插值差别很小;
    # 真轨迹 retime 后时间轴严重非均匀,两个度量才分开。
    # 教训:性质测试的数据也要有**代表性**,平滑的合成数据会掩盖度量选错。
    real = OUT_NPZ
    if real is None:
        print("  skip 没有 out/robot_traj_*.npz,跳过真轨迹一组")
    else:
        ra, _ = P.clamp_limits(np.asarray(np.load(real, allow_pickle=True)["arm"], float))
        ra = P.lowpass(ra, FPS, 3.0)
        rt, _ = P.retime(ra, FPS, 90.0)
        for tol in (0.01, 0.004):
            ridx = P.rdp_indices(ra, rt, tol)
            rwt, rwq = rt[ridx], ra[ridx]
            rrec = np.stack([np.interp(rt, rwt, rwq[:, j]) for j in range(7)], axis=1)
            rerr = float(np.abs(rrec - ra).max())
            check(f"{real.name} tol={tol}: {rerr*P.RAD2DEG:.3f}° ≤ "
                  f"{tol*P.RAD2DEG:.3f}°", rerr <= tol + 1e-9, f"{rerr:.6f}")
        # 时间轴确实非均匀 —— 否则上面那组又退化成"碰巧通过"
        dts = np.diff(rt)
        check("真轨迹 retime 后时间轴非均匀(该组才有鉴别力)",
              dts.max() > dts.min() * 1.5, f"dt {dts.min()*1000:.1f}~{dts.max()*1000:.1f}ms")

    print("\n=== t_ns 是权威时刻,整数纳秒,零累积误差 ===")
    N = 2400                                  # gesture_pack.MAX_FRAMES 同量级
    tt2 = np.arange(N) / FPS
    a2 = np.resize(f, (N, 7))
    pk2 = P.build_pack("ns", a2, tt2, list(range(N)), FPS, {})
    w = pk2["waypoints"]
    check("每点都有 t_ns", all("t_ns" in x for x in w))
    check("t_ns 是整数", all(isinstance(x["t_ns"], int) for x in w),
          str(type(w[0]["t_ns"])))
    check("首点 t_ns == 0", w[0]["t_ns"] == 0, str(w[0]["t_ns"]))
    ideal_ns = (N - 1) / FPS * 1e9
    err_us = abs(w[-1]["t_ns"] - ideal_ns) / 1000
    check(f"{N} 帧末点 t_ns 误差 {err_us:.3f}us < 1us", err_us < 1.0, f"{err_us:.3f}us")
    # 逐点都要准,不能只末点准(那种可能是碰巧首末对上、中间歪)
    worst = max(abs(x["t_ns"] - i / FPS * 1e9) for i, x in enumerate(w)) / 1000
    check(f"逐点最大误差 {worst:.3f}us < 1us", worst < 1.0, f"{worst:.3f}us")
    # 反证:同样 2400 帧,整数毫秒累加会漂多少 —— 说明上面几条不是恒真
    drift = abs((N - 1) / FPS * 1000 - (N - 1) * round(1000 / FPS))
    check(f"反证:整数毫秒累加会漂 {drift:.0f}ms(所以 t_ns 有意义)", drift > 100,
          f"{drift:.1f}ms")

    print("\n=== dt_ms 累加不能漂 ===")
    # ⚠ 实测踩过:30fps 真周期 33.3333ms,dt_ms 取整成 33,780 帧累加漂 **260ms**
    # (1% 时长)。臂按累加值走、手按 t 走,末尾错开 260ms —— 正是要避免的不同步。
    tt = np.arange(600) / FPS
    idx_all = list(range(600))
    pk_u = P.build_pack("u", f[:600] if f.shape[0] >= 600 else np.resize(f, (600, 7)),
                        tt, idx_all, FPS, {})
    acc = sum(w["dt_ms"] for w in pk_u["waypoints"][:-1]) / 1000
    drift_ms = abs(pk_u["waypoints"][-1]["t"] - acc) * 1000
    check(f"600 帧累加 dt_ms 漂移 {drift_ms:.3f}ms < 1ms", drift_ms < 1.0,
          f"{drift_ms:.3f}ms")
    check("dt_ms 不是整数(取整就会漂)",
          any(abs(w["dt_ms"] - round(w["dt_ms"])) > 1e-9
              for w in pk_u["waypoints"][:-1]),
          "全是整数 → 会漂")
    check("均匀时间轴报 uniform_dt_ms", pk_u["uniform_dt_ms"] is not None)
    check("全帧 + 均匀 → mode=stream", pk_u["mode"] == "stream", pk_u["mode"])

    print("\n=== 抽稀过的包 mode=waypoints,uniform_dt_ms 为 null ===")
    sp = P.build_pack("s", f, t, P.rdp_indices(f, t, 0.01), FPS, {})
    check("mode=waypoints", sp["mode"] == "waypoints", sp["mode"])
    check("uniform_dt_ms 为 None", sp["uniform_dt_ms"] is None)

    print("\n=== cpv_fit 按逐关节算,不混合分位数 ===")
    # ⚠ 混着算会把最差关节埋掉:实测混合 p95 说"超 1.38×",而 joint1 实际超 4.39×
    fit = P.cpv_fit(f, t)
    for k in ("vel_p95_ratio", "acc_p95_ratio", "worst_joint",
              "worst_joint_rated_pct", "fits_defaults"):
        check(f"cpv_fit 有 {k}", k in fit, str(fit.keys()))
    check("worst_joint 是关节名", fit["worst_joint"] in P.ARM_JOINTS, fit["worst_joint"])
    src = (SIM / "prep_arm_traj.py").read_text(encoding="utf-8")
    check("retime 默认关(--retime 是 store_true)",
          'ap.add_argument("--retime", action="store_true"' in src)
    check("rdp 默认关(--rdp 是 store_true)",
          'ap.add_argument("--rdp", action="store_true"' in src)

    print("\n=== 包:approach 段 + dt_ms 来自非均匀时间轴 ===")
    idx = P.rdp_indices(f, t, 0.01)
    pk = P.build_pack("t", f, t, idx, FPS, rep)
    check("schema 正确", pk["schema"] == P.SCHEMA)
    check("有 approach 且报了距零位多远", "approach" in pk and
          pk["approach"]["max_from_zero_deg"] >= 0)
    check("末路点 dt_ms 为 None", pk["waypoints"][-1]["dt_ms"] is None)
    # dt_ms 必须来自 retime 的时间轴,不能是 (i差)/fps
    w0, w1 = pk["waypoints"][0], pk["waypoints"][1]
    # ⚠ 期望值不能取整 —— dt_ms 现在是 3 位小数的 float(取整会累积漂移,见上一组)
    want = round((t[w1["i"]] - t[w0["i"]]) * 1000, 3)
    check("dt_ms 用非均匀 t 算,不是 i/fps", abs(w0["dt_ms"] - want) < 1e-6,
          f"{w0['dt_ms']} vs {want}")
    check("时长 = t[-1] 而非 n/fps",
          abs(pk["duration_s"] - float(t[-1])) < 1e-3)

    n = len(_FAILS)
    print(f"\n{'全部通过' if n == 0 else str(n) + ' 项失败: ' + ', '.join(_FAILS)}")
    return 1 if n else 0


if __name__ == "__main__":
    raise SystemExit(main())
