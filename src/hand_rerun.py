#!/usr/bin/env python3
"""src/hand_rerun.py — 手部可视化:stdin 手关节流 → pinocchio FK(含 mimic)→ Rerun serve。

给 app_web.py 的「灵巧手调试」页用。和 live_rerun 的区别:
  · 只加载手 URDF(不带臂)
  · 手动补算 mimic 耦合关节(6 驱动 + 6 耦合 = 12 revolute 都动起来)
  · stdin 输入是 hand_console.py 的 {"type":"state","rad":[6],...},不是 ros_joint_reader

跑在 **conda lerobot-v3 主环境**(有 pinocchio / rerun)。

用法(一般由 app_web.py 调):
  python3 src/hand_console.py | python3 src/hand_rerun.py --serve
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

sys.path.insert(0, str(Path(__file__).resolve().parent))
from paths import REPO, HAND_ABSOLUTE_URDF              # noqa: E402
from replay_rerun import RobotModel, load_meshes, log, primary_ip   # noqa: E402

# 6 驱动关节:项目顺序(和 hand_console / inspire_hand 一致)
HAND_NAMES = ["right_thumb_1_joint", "right_thumb_2_joint",
              "right_index_1_joint", "right_middle_1_joint",
              "right_ring_1_joint", "right_little_1_joint"]

# 6 mimic 耦合(xacro 和 ros2_control 能自动处理,但 pinocchio 的 urdf parser 不认 mimic 标签)
# ⚠ 2026-08-10 更新:key 改为新 URDF 的实际关节名(right_thumb_3_joint 等)。
# ⚠ 和 src/web/hand3d.js 的 MIMIC 是同一份数据,改一处要改两处。
MIMIC = {
    "right_thumb_3_joint":  ("right_thumb_2_joint", 1.1425, 0.0),
    "right_thumb_4_joint":  ("right_thumb_2_joint", 0.857789, 0.0),  # 链式展平
    "right_index_2_joint":  ("right_index_1_joint",  1.1169, 0.0),
    "right_middle_2_joint": ("right_middle_1_joint", 1.1169, 0.0),
    "right_ring_2_joint":   ("right_ring_1_joint",   1.1169, 0.0),
    "right_little_2_joint": ("right_little_1_joint",  1.1169, 0.0),
}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=str(HAND_ABSOLUTE_URDF))
    ap.add_argument("--serve", action="store_true")
    ap.add_argument("--web-port", type=int, default=9095)
    ap.add_argument("--grpc-port", type=int, default=9881)
    ap.add_argument("--mem-limit", default="200MB")
    ap.add_argument("--view-hz", type=float, default=20.0)
    args = ap.parse_args()

    model = RobotModel(Path(args.urdf))
    meshes = load_meshes(model)

    rr.init("inspire_hand_debug")
    root = "world/hand"
    bp = rrb.Blueprint(
        rrb.Vertical(
            rrb.Spatial3DView(origin="world", name="灵巧手 · Inspire RH56DFX"),
            rrb.TimeSeriesView(origin="joints", name="关节角(rad · 驱动)"),
            row_shares=[3.5, 1.0],
        ),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="collapsed"),
    )

    if args.serve:
        ip = primary_ip()
        uri = rr.serve_grpc(grpc_port=args.grpc_port, server_memory_limit=args.mem_limit)
        serve_uri = uri.replace("127.0.0.1", ip).replace("0.0.0.0", ip)
        rr.serve_web_viewer(web_port=args.web_port, open_browser=False, connect_to=serve_uri)
    rr.send_blueprint(bp)

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    for m in meshes:
        if m is None:
            continue
        rr.log(f"{root}/{m['name']}",
               rr.Mesh3D(vertex_positions=m["V"], triangle_indices=m["F"],
                         vertex_normals=m["N"], albedo_factor=m["color"]), static=True)

    if args.serve:
        from urllib.parse import quote
        full = f"http://{ip}:{args.web_port}/?url={quote(serve_uri, safe='')}"
        print("\n" + "=" * 72, flush=True)
        print("  灵巧手调试 Rerun 已就绪:", flush=True)
        print(f"    {full}", flush=True)
        print("=" * 72 + "\n", flush=True)

    log(f"等待 hand_console.py 关节流…(抽帧 {args.view_hz:.0f}Hz)")
    min_dt = 1.0 / args.view_hz if args.view_hz > 0 else 0.0
    fr = 0
    last_render = 0.0
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get("type") != "state" or "rad" not in row:
            continue
        now = time.monotonic()
        if now - last_render < min_dt:
            continue
        last_render = now
        rad_driven = np.array(row["rad"], dtype=float)
        rr.set_time("frame", sequence=fr)
        rr.set_time("time", duration=row.get("t", fr * 0.05))

        # 6 驱动 → 补算 6 mimic → 合 12 维 q。和 live_rerun.py 不同,这里不需要 ARM。
        q = model.q0.copy()
        for i, n in enumerate(HAND_NAMES):
            qi = model.name_to_qidx.get(n)
            if qi is not None:
                q[qi] = rad_driven[i]
        for mim_name, (driver, mult, off) in MIMIC.items():
            qi_driver = model.name_to_qidx.get(driver)
            qi_mim = model.name_to_qidx.get(mim_name)
            if qi_driver is not None and qi_mim is not None:
                q[qi_mim] = q[qi_driver] * mult + off

        placements = model.placements(q)
        for i, m in enumerate(meshes):
            if m is None:
                continue
            M = placements[i]
            rr.log(f"{root}/{m['name']}",
                   rr.Transform3D(translation=M[:3, 3], mat3x3=M[:3, :3]))
        for i, n in enumerate(HAND_NAMES):
            rr.log(f"joints/hand/{n}", rr.Scalars(float(rad_driven[i])))
        fr += 1

    log(f"stdin 结束,共 {fr} 帧。")


if __name__ == "__main__":
    main()
