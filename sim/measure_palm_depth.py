"""一次性测量:掌骨点(0/5/9/13/17)深度够不够干净来拟合手腕朝向。

指标:
  1. 每个掌骨点的深度有效率(非零/量程内的帧占比)
  2. 5点平面拟合残差(mm) —— 深度噪声代理,越小越平/越可信
  3. 深度法向 vs 单目手腕法向(palm normal)的角度差(度)
  4. 掌骨点深度的逐帧抖动(mm)
"""
import sys, glob, json
from pathlib import Path
import numpy as np
import cv2

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from single_hand_detector import SingleHandDetector
from estimate_wrist import physical_wrist_orientation

PALM = [0, 5, 9, 13, 17]         # 腕 + 4 掌指关节(MCP)
DEPTH_SCALE = 0.001              # uint16 mm -> m
RADIUS = 2
ROOT = REPO / "kinect2_middle/kinect2_middle"


def depth_at(depth_m, u, v, r=RADIUS):
    h, w = depth_m.shape
    x, y = int(round(u)), int(round(v))
    if x < 0 or x >= w or y < 0 or y >= h:
        return 0.0, False
    patch = depth_m[max(0, y-r):y+r+1, max(0, x-r):x+r+1]
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return 0.0, False
    return float(np.median(valid)), True


def fit_plane_normal(pts):
    """5x3 点 -> (法向, 残差mm)。SVD 最小奇异向量=法向,最小奇异值~残差。"""
    c = pts.mean(0)
    U, S, Vt = np.linalg.svd(pts - c)
    n = Vt[-1]
    resid = float(np.abs((pts - c) @ n).mean()) * 1000.0   # mm
    return n / (np.linalg.norm(n) + 1e-12), resid


def main():
    intr = json.loads((ROOT / "calibration.json").read_text())["cameras"]["kinect2_middle"]["intrinsics"]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
    det = SingleHandDetector(hand_type="Right", selfie=False, max_num_hands=1)

    color_files = sorted(glob.glob(str(ROOT / "color/*.png")))
    depth_files = {Path(p).name: p for p in glob.glob(str(ROOT / "depth/*.png"))}

    valid_count = np.zeros(5)
    resids, ang_diffs, n_det = [], [], 0
    prev_palm_z = None
    jitter = []

    for cf in color_files:
        name = Path(cf).name
        if name not in depth_files:
            continue
        bgr = cv2.imread(cf, cv2.IMREAD_COLOR)
        depth = cv2.imread(depth_files[name], cv2.IMREAD_UNCHANGED)
        if bgr is None or depth is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        num, joint_pos, kp2d_lm, wrist_rot = det.detect(rgb)
        if num == 0:
            continue
        n_det += 1
        kp2d = SingleHandDetector.parse_keypoint_2d(kp2d_lm, rgb.shape)
        depth_m = depth.astype(np.float32) * DEPTH_SCALE

        pts, ok = [], []
        for j, idx in enumerate(PALM):
            u, v = kp2d[idx]
            z, good = depth_at(depth_m, u, v)
            ok.append(good)
            valid_count[j] += int(good)
            pts.append([(u-cx)*z/fx, (v-cy)*z/fy, z] if good else [np.nan]*3)
        pts = np.array(pts)

        if all(ok):
            n_depth, resid = fit_plane_normal(pts)
            resids.append(resid)
            # 单目 palm normal = physical 朝向的 X 轴(见 estimate_wrist)
            R_mono = physical_wrist_orientation(wrist_rot, det.operator2mano)
            n_mono = R_mono[:, 0]
            cosang = abs(float(np.dot(n_depth, n_mono)))
            ang_diffs.append(np.degrees(np.arccos(np.clip(cosang, 0, 1))))
            palm_z = pts[:, 2].mean()
            if prev_palm_z is not None:
                jitter.append(abs(palm_z - prev_palm_z) * 1000.0)
            prev_palm_z = palm_z

    print(f"\n检测到手的帧: {n_det}/{len(color_files)}")
    print("掌骨点深度有效率:")
    for j, idx in enumerate(PALM):
        print(f"  kp{idx}: {valid_count[j]/max(n_det,1)*100:5.1f}%")
    if resids:
        r = np.array(resids)
        print(f"\n5点平面拟合残差(mm)  中位={np.median(r):.1f}  p90={np.percentile(r,90):.1f}  max={r.max():.1f}")
        print(f"  (残差=深度噪声代理;手掌近乎平面,残差应 <~5mm 才算干净)")
    if ang_diffs:
        a = np.array(ang_diffs)
        print(f"\n深度法向 vs 单目法向 角度差(度)  中位={np.median(a):.1f}  p90={np.percentile(a,90):.1f}  max={a.max():.1f}")
    if jitter:
        j = np.array(jitter)
        print(f"\n掌心深度逐帧抖动(mm)  中位={np.median(j):.1f}  p90={np.percentile(j,90):.1f}")
    print(f"\n可拟合平面(5点全有效)的帧: {len(resids)}/{n_det}")


if __name__ == "__main__":
    main()
