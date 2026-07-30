"""一维滑块 MPC:亲眼看它"提前刹车"。
状态 [p, v]，动作 a(推力/加速度)。目标:停在 p=1.0，且 v=0，不过冲。
对比:贪心(只看1步) vs MPC(看未来N步一起解)。
力被限死 |a|<=A_MAX —— 这道栏就是"刹不住"的根,逼出提前刹车。
"""
import numpy as np
from scipy.optimize import minimize

DT   = 0.1        # 一帧时长
N    = 20         # MPC 往前看几步 (horizon)
A_MAX= 1.0        # 电机最大力 -> 这道栏是关键
P_TGT= 1.0        # 目标位置
W_P, W_V, W_A = 1.0, 1.0, 0.01   # 位置差/速度差/别猛推 的权重


def rollout(p0, v0, acc):
    """给定起点和一串加速度,用物理链条推出未来每帧的 p,v。
    这就是那两条'焊死的约束':v[k+1]=v[k]+a*dt, p[k+1]=p[k]+v*dt"""
    ps, vs = [], []
    p, v = p0, v0
    for a in acc:
        v = v + a * DT          # 新速度 = 旧速度 + 加的
        p = p + v * DT          # 新位置 = 旧位置 + 按速度走的
        ps.append(p); vs.append(v)
    return np.array(ps), np.array(vs)


def cost(acc, p0, v0):
    """目标函数:未来每帧 位置差² + 速度差² + 别猛推²，全加起来。"""
    ps, vs = rollout(p0, v0, acc)
    return (W_P*np.sum((ps - P_TGT)**2)
          + W_V*np.sum(vs**2)
          + W_A*np.sum(acc**2))


def mpc_solve(p0, v0, horizon):
    """解一个 N 步 QP:挑一整串 a[0..N-1]，力被 |a|<=A_MAX 约束。"""
    a0 = np.zeros(horizon)
    bounds = [(-A_MAX, A_MAX)] * horizon          # 力上限那道栏
    res = minimize(cost, a0, args=(p0, v0), bounds=bounds, method="SLSQP")
    return res.x                                  # 返回整串,但只用第0个


def greedy_a(p0, v0):
    """贪心:只看'这一步'怎么最快缩小位置差,完全不管速度/未来。
    还没到就全力冲(在力上限内),典型的'刹车太晚'。"""
    want = (P_TGT - p0) / DT / DT                 # 一步就想补上位置差需要的加速度
    return np.clip(want, -A_MAX, A_MAX)


def simulate(controller_name):
    """真·滚动时域:每真帧解一次,只走第0步,再重解。"""
    p, v = 0.0, 0.0
    print(f"\n=== {controller_name} ===")
    print(f"{'帧':>3} {'位置p':>8} {'速度v':>8} {'推力a':>8}  说明")
    for k in range(40):
        if controller_name == "MPC(看未来20步)":
            a = mpc_solve(p, v, N)[0]             # 解整串,只取第0个
        else:
            a = greedy_a(p, v)                     # 贪心只看当下
        note = ""
        if a < -0.05: note = "<< 反推/刹车"
        elif abs(a) < 0.05 and abs(v) < 0.02 and abs(p-P_TGT) < 0.02: note = "停稳✓"
        if k % 2 == 0 or note:
            print(f"{k:>3} {p:>8.3f} {v:>8.3f} {a:>8.3f}  {note}")
        v = v + a * DT
        p = p + v * DT
    err = abs(p - P_TGT)
    print(f"最终: 位置={p:.3f} (目标{P_TGT}), 速度={v:.3f}, "
          f"{'过冲!' if p > P_TGT + 0.02 else '停稳' if err<0.02 else ''} 位置误差={err*1000:.1f}mm")


if __name__ == "__main__":
    simulate("贪心(只看1步)")
    simulate("MPC(看未来20步)")
