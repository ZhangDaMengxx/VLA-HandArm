#!/usr/bin/env python3
"""sim/traj_player.py — 把离线轨迹(视频→IK 产出的 .npz)逐帧下发给机械臂+灵巧手。

跑在 **ROS2 系统 python3**(numpy 1.x,能读 derive_embodiment 存的 .npz;
注意读不了 numpy 2.x 的 .pkl,所以固定读同名 .npz)。它是"视频→轨迹→真机回放"
链路里缺的那一环:读 arm(N,7)+hand(N,12),按名字取 6 个驱动手关节,按 fps×speed
逐帧调 JointWriter.send() —— 复用 writer 的**限位夹取 + 发布**,不另造发布逻辑,
真机/仿真同一条路。订阅 /nero/estop:收到即停,当场退出。

进度打 stdout(JSON 行)供 app_web 解析:{"type":"progress","frame":i,"total":N,"pct":..}

用法(一般由 app_web 调):
  python3 sim/traj_player.py --npz sim/out/robot_traj_nero_inspire.npz --fps 30 --speed 1.0
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros_joint_writer import JointWriter, HAND_NAMES   # 复用发布+限位,单一真源


def _emit(ev: dict) -> None:
    print(json.dumps(ev, ensure_ascii=False), flush=True)


def _hand_cols(npz_hand_names: list[str]) -> list[int]:
    """把 npz 的 12 手关节按 writer 的 6 驱动名取列下标(按名字,顺序无关)。"""
    idx = []
    for n in HAND_NAMES:
        if n not in npz_hand_names:
            raise KeyError(f"npz 缺少驱动手关节 {n}(有:{npz_hand_names})")
        idx.append(npz_hand_names.index(n))
    return idx


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--npz", required=True, help="轨迹 .npz(derive_embodiment --emit-traj 产出)")
    ap.add_argument("--fps", type=float, default=30.0, help="回放帧率(npz 不含 fps,默认 30)")
    ap.add_argument("--speed", type=float, default=1.0, help="倍速(0.25~4),越大越快")
    args = ap.parse_args()

    data = np.load(args.npz, allow_pickle=True)
    arm = np.asarray(data["arm"], dtype=float)           # (N,7)
    hand = np.asarray(data["hand"], dtype=float)         # (N,12)
    hnames = [str(x) for x in data["hand_joint_names"]]
    cols = _hand_cols(hnames)
    N = min(len(arm), len(hand))
    dt = 1.0 / (max(1e-3, args.fps) * max(0.05, args.speed))
    _emit({"type": "start", "total": N, "fps": args.fps, "speed": args.speed})

    rclpy.init()
    node = JointWriter()
    stop = {"v": False}
    node.create_subscription(Bool, "/nero/estop",
                             lambda m: stop.__setitem__("v", stop["v"] or bool(m.data)), 10)
    rclpy.spin_once(node, timeout_sec=0.5)               # 让发布者/订阅先建立

    try:
        for i in range(N):
            rclpy.spin_once(node, timeout_sec=0.0)
            if stop["v"]:
                _emit({"type": "stopped", "frame": i, "reason": "estop"})
                break
            cmd = {"arm": [float(x) for x in arm[i]],
                   "hand": [float(hand[i, c]) for c in cols],
                   "duration": dt}
            node.send(cmd)
            if i % 5 == 0 or i == N - 1:
                _emit({"type": "progress", "frame": i + 1, "total": N,
                       "pct": round(100.0 * (i + 1) / N, 1)})
            time.sleep(dt)
        else:
            _emit({"type": "done", "total": N})
    except KeyboardInterrupt:
        _emit({"type": "stopped", "reason": "interrupt"})
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
