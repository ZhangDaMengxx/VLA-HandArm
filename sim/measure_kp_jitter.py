"""诊断:21 个手部关键点的逐帧 3D 跳变,定位"骨架/点云跳出去"的来源。

用原始深度反投(与 viz_orient_compare 一致,无剔除),统计每个点帧间位移。
分组:腕(0) / 掌骨MCP(5,9,13,17) / 指尖(4,8,12,16,20) / 其余指节。
大跳(>5cm)多集中在指尖=细手指/边缘穿越,对默认管线(手指走 MANO,不吃深度)无害;
若腕/掌骨也大跳=影响手腕位置与深度朝向,才是真问题。
"""
import sys, glob, json
from pathlib import Path
import numpy as np
import cv2

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from single_hand_detector import SingleHandDetector

ROOT = REPO / "kinect2_middle/kinect2_middle"
TIP = [4, 8, 12, 16, 20]
MCP = [5, 9, 13, 17]
WRIST = [0]


def depth_at(dm, u, v, r=2):
    h, w = dm.shape
    x, y = int(round(u)), int(round(v))
    if x < 0 or x >= w or y < 0 or y >= h:
        return 0.0, False
    p = dm[max(0,y-r):y+r+1, max(0,x-r):x+r+1]
    vv = p[np.isfinite(p) & (p > 0)]
    return (float(np.median(vv)), True) if vv.size else (0.0, False)


def main():
    intr = json.loads((ROOT/"calibration.json").read_text())["cameras"]["kinect2_middle"]["intrinsics"]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
    det = SingleHandDetector(hand_type="Right", selfie=False, max_num_hands=1)
    depth_files = {Path(p).name: p for p in glob.glob(str(ROOT/"depth/*.png"))}

    prev = None
    disp = {i: [] for i in range(21)}   # 每点的帧间位移(m)
    for cf in sorted(glob.glob(str(ROOT/"color/*.png"))):
        name = Path(cf).name
        if name not in depth_files:
            continue
        bgr = cv2.imread(cf, cv2.IMREAD_COLOR)
        depth = cv2.imread(depth_files[name], cv2.IMREAD_UNCHANGED)
        if bgr is None or depth is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        num, jp, lm, wrot = det.detect(rgb)
        if num == 0:
            prev = None; continue
        kp2d = SingleHandDetector.parse_keypoint_2d(lm, rgb.shape)
        dm = depth.astype(np.float32) * 0.001
        pts = np.full((21, 3), np.nan)
        for i, (u, v) in enumerate(kp2d):
            z, ok = depth_at(dm, u, v)
            if ok:
                pts[i] = [(u-cx)*z/fx, (v-cy)*z/fy, z]
        if prev is not None:
            for i in range(21):
                if np.isfinite(pts[i]).all() and np.isfinite(prev[i]).all():
                    disp[i].append(float(np.linalg.norm(pts[i]-prev[i])))
        prev = pts

    def stat(idxs, label):
        d = np.concatenate([disp[i] for i in idxs]) if idxs else np.array([])
        if d.size == 0:
            print(f"  {label:12s}: (无数据)"); return
        big = float((d > 0.05).mean() * 100)
        print(f"  {label:12s}: 帧间位移 中位={np.median(d)*1000:5.1f}mm  p90={np.percentile(d,90)*1000:6.1f}mm  "
              f"max={d.max()*1000:6.0f}mm  >5cm大跳={big:4.1f}%")

    print("原始深度反投的逐帧 3D 位移(无剔除,与 viz 一致):")
    stat(WRIST, "腕(0)")
    stat(MCP, "掌骨(5/9/13/17)")
    stat([i for i in range(21) if i not in WRIST+MCP+TIP], "其余指节")
    stat(TIP, "指尖(4/8..20)")
    print("\n判读: 大跳集中在指尖=对默认管线无害(手指走MANO); 腕/掌骨也大跳=影响朝向与位置。")


if __name__ == "__main__":
    main()
