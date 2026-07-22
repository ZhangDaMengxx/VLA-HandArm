#!/usr/bin/env python3
"""sim/live_rerun.py — stdin 实时关节流 → pinocchio FK → Rerun serve(实时 3D + 关节曲线)。

跑在 **conda lerobot 环境**(有 pinocchio / rerun / trimesh),被 app_web.py 以子进程拉起。
它是"实时监控"的 3D 后端:从 stdin 逐行读 JSON 关节状态(由 ros_joint_reader.py 经
app_web.py 转发过来),每来一帧就算 FK、更新各连杆 Transform、追加关节角曲线,
Rerun serve 出去让浏览器 iframe 实时刷新。

和 replay_rerun.py 的区别:后者读离线 pkl 一次性记录 F 帧;本脚本是**无界实时流**,
帧序号随消息递增,时间轴用消息自带时间戳。复用 replay_rerun 的 RobotModel/load_meshes,
不重复造轮子。

输入(stdin 每行,ros_joint_reader.py 的格式):
  {"t":.., "names":[..13..], "pos":[..13..], "vel":[..]}
非 JSON 行 / {"type":..} 状态行忽略。

用法(一般由 app_web.py 调):
  fake_real_arm → ros_joint_reader.py | python sim/live_rerun.py --serve
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

sys.path.insert(0, str(Path(__file__).resolve().parent))
from replay_rerun import RobotModel, load_meshes, primary_ip, log   # 复用,不重造

REPO = Path(__file__).resolve().parents[1]

# 臂/手关节名分组(和 schema 一致):用于把 13 维拆成臂 7 + 手 6 喂给 RobotModel.make_q。
ARM_NAMES = [f"joint{i}" for i in range(1, 8)]
HAND_NAMES = ["thumb_proximal_yaw_joint", "thumb_proximal_pitch_joint",
              "index_proximal_joint", "middle_proximal_joint",
              "ring_proximal_joint", "pinky_proximal_joint"]


def _split(names, pos):
    """把一帧 (names,pos) 按名字映射成 (arm[7], hand[6]),缺失填 0 —— 兼容顺序变化/少发。"""
    d = {n: p for n, p in zip(names, pos)}
    arm = np.array([d.get(n, 0.0) for n in ARM_NAMES], dtype=float)
    hand = np.array([d.get(n, 0.0) for n in HAND_NAMES], dtype=float)
    return arm, hand


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=str(REPO / "sim/assets/nero_inspire_right.urdf"))
    ap.add_argument("--serve", action="store_true")
    ap.add_argument("--web-port", type=int, default=9090)
    ap.add_argument("--grpc-port", type=int, default=9876)
    ap.add_argument("--mem-limit", default="500MB",
                    help="Rerun 服务端内存上限,超出丢最旧(滑动窗口)。如 500MB / 25%%")
    ap.add_argument("--view-hz", type=float, default=30.0,
                    help="3D/曲线更新率上限(抽帧);数据源更快也只按此刷,防卡")
    args = ap.parse_args()

    model = RobotModel(Path(args.urdf))
    meshes = load_meshes(model)

    rr.init("nero_inspire_live")
    root = "world/live"
    bp = rrb.Blueprint(
        rrb.Vertical(
            rrb.Spatial3DView(origin="world", name="Robot · NERO+inspire(实时)"),
            rrb.TimeSeriesView(origin="joints", name="关节角(rad · 实时)"),
            row_shares=[3.0, 1.2],
        ),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="collapsed"),
    )

    serve_uri = None
    if args.serve:
        ip = primary_ip()
        # server_memory_limit:滑动窗口 —— 超上限自动丢最旧数据,内存封顶不再无界增长
        uri = rr.serve_grpc(grpc_port=args.grpc_port, server_memory_limit=args.mem_limit)
        serve_uri = uri.replace("127.0.0.1", ip).replace("0.0.0.0", ip)
        rr.serve_web_viewer(web_port=args.web_port, open_browser=False, connect_to=serve_uri)
    rr.send_blueprint(bp)

    # 静态:世界系 + 网格(顶点在 link 局部系,逐帧只更新 Transform)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    for m in meshes:
        if m is None:
            continue
        rr.log(f"{root}/{m['name']}",
               rr.Mesh3D(vertex_positions=m["V"], triangle_indices=m["F"],
                         vertex_normals=m["N"], albedo_factor=m["color"]), static=True)

    if args.serve:
        from urllib.parse import quote
        ip = primary_ip()
        full = f"http://{ip}:{args.web_port}/?url={quote(serve_uri, safe='')}"
        print("\n" + "=" * 72, flush=True)
        print("  Rerun 实时查看器已就绪。完整地址(带数据源):", flush=True)
        print(f"    {full}", flush=True)
        print("=" * 72 + "\n", flush=True)

    # ---- 无界实时流:逐行读 stdin,抽帧到 view-hz,FK,更新 ----
    import time
    log(f"等待 stdin 关节流…(3D 抽帧 {args.view_hz:.0f}Hz,内存上限 {args.mem_limit})")
    min_dt = 1.0 / args.view_hz if args.view_hz > 0 else 0.0
    fr = 0
    t0 = None
    last_render = 0.0
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if "pos" not in row or "names" not in row:
            continue                              # {"type":ready/closed} 等状态行
        # 抽帧:距上次渲染不足 min_dt 就丢弃(仍读走 stdin,不积压),防 100Hz 压垮
        now = time.monotonic()
        if now - last_render < min_dt:
            continue
        last_render = now
        arm, hand = _split(row["names"], row["pos"])
        t = row.get("t", fr)
        if t0 is None:
            t0 = t
        rr.set_time("frame", sequence=fr)
        rr.set_time("time", duration=max(0.0, t - t0))

        q = model.make_q(arm, ARM_NAMES, hand, HAND_NAMES)
        placements = model.placements(q)
        for i, m in enumerate(meshes):
            if m is None:
                continue
            M = placements[i]
            rr.log(f"{root}/{m['name']}",
                   rr.Transform3D(translation=M[:3, 3], mat3x3=M[:3, :3]))
        for k, n in enumerate(ARM_NAMES):
            rr.log(f"joints/live/arm/{n}", rr.Scalars(float(arm[k])))
        for k, n in enumerate(HAND_NAMES):
            rr.log(f"joints/live/hand/{n}", rr.Scalars(float(hand[k])))
        fr += 1

    log(f"stdin 结束,共 {fr} 帧。")


if __name__ == "__main__":
    main()
