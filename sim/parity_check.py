"""并行跑两条数据路径 + 比对,确认两层重构与旧单本体管线一致(切默认前的回归闸)。

旧路径:  detect_wrist.py  → full_traj.pkl → build_robot_traj.py → robot_traj.pkl
新两层:  build_canonical.py → canonical_ds → derive_embodiment.py --emit-traj → robot_traj_nero_inspire.pkl

两者用同一视频、同一稳定化参数,理论上应逐位近似(差异仅来自 canonical_ds 的 float32 存储)。
每次换新视频/改了任一路径的代码,跑这个确认没漂。PASS 阈值 = max|Δ| < 1e-4 rad。

用法: python sim/parity_check.py            (从头跑两条路再比)
       python sim/parity_check.py --compare-only   (只比现有产物,不重跑)
"""
import sys
import subprocess
import pickle
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
PY = sys.executable
THRESH = 1e-4   # rad

OLD = REPO / "sim/out/robot_traj.pkl"
NEW = REPO / "sim/out/robot_traj_nero_inspire.pkl"


def run(script, *args):
    cmd = [PY, str(REPO / "sim" / script), *args]
    print(f"\n>>> {' '.join([script, *args])}", flush=True)
    r = subprocess.run(cmd, cwd=str(REPO))
    if r.returncode != 0:
        raise SystemExit(f"步骤失败: {script} (rc={r.returncode})")


def compare():
    old = pickle.load(open(OLD, "rb"))
    new = pickle.load(open(NEW, "rb"))
    ok = True
    for k in ("arm", "hand"):
        a, b = np.asarray(old[k]), np.asarray(new[k])
        if a.shape != b.shape:
            print(f"  {k}: 形状不一致 {a.shape} vs {b.shape}  ✗")
            ok = False
            continue
        d = np.abs(a - b)
        flag = "✓" if d.max() < THRESH else "✗"
        print(f"  {k}: shape{a.shape}  max|Δ|={d.max():.2e}  mean|Δ|={d.mean():.2e} rad  {flag}")
        ok = ok and d.max() < THRESH
    names_ok = (old["arm_joint_names"] == new["arm_joint_names"]
                and old["hand_joint_names"] == new["hand_joint_names"])
    print(f"  关节名一致: {names_ok}  {'✓' if names_ok else '✗'}")
    ok = ok and names_ok
    print("\nPARITY:", "PASS ✓  两层路径与旧管线一致,可放心切默认" if ok
          else "FAIL ✗  两路径漂了,先查再切")
    return ok


def main():
    if "--compare-only" not in sys.argv:
        # 旧路径
        run("detect_wrist.py")
        run("build_robot_traj.py")
        # 新两层
        run("build_canonical.py")
        run("derive_embodiment.py", "--emit-traj")
    print("\n===== PARITY 比对(旧 robot_traj.pkl vs 新 robot_traj_nero_inspire.pkl)=====")
    sys.exit(0 if compare() else 1)


if __name__ == "__main__":
    main()
