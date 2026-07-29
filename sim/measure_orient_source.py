"""根因定位:深度朝向逐帧抖动来自 X 轴符号翻转,还是 Z 轴(腕→中指)本身抖?

对连续帧分别量角度变化:
  法向(未定符号)  —— 平面 X 轴方向抖动(SVD 最小奇异向量,取绝对方向)
  法向符号翻转次数 —— 用单目定符号 vs 用时序连续定符号,各翻几次
  Z 轴(腕→中指)  —— 2 点向量方向抖动
  R_depth 整体     —— 现管线的朝向逐帧跳(对应 derive 的 clamp)
"""
import sys, glob, json
from pathlib import Path
import numpy as np
import cv2

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from single_hand_detector import SingleHandDetector
from estimate_wrist import physical_wrist_orientation, depth_wrist_orientation

PALM = [0, 5, 9, 13, 17]
ROOT = REPO / "kinect2_middle/kinect2_middle"


def depth_at(dm, u, v, r=2):
    h, w = dm.shape
    x, y = int(round(u)), int(round(v))
    if x < 0 or x >= w or y < 0 or y >= h:
        return 0.0, False
    p = dm[max(0,y-r):y+r+1, max(0,x-r):x+r+1]
    vv = p[np.isfinite(p) & (p > 0)]
    return (float(np.median(vv)), True) if vv.size else (0.0, False)


def ang(a, b):
    a = a/(np.linalg.norm(a)+1e-12); b = b/(np.linalg.norm(b)+1e-12)
    return np.degrees(np.arccos(np.clip(abs(float(a@b)), 0, 1)))


def R_ang(A, B):
    from scipy.spatial.transform import Rotation as Rot
    return np.degrees(np.linalg.norm((Rot.from_matrix(A).inv()*Rot.from_matrix(B)).as_rotvec()))


def main():
    intr = json.loads((ROOT/"calibration.json").read_text())["cameras"]["kinect2_middle"]["intrinsics"]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
    det = SingleHandDetector(hand_type="Right", selfie=False, max_num_hands=1)
    depth_files = {Path(p).name: p for p in glob.glob(str(ROOT/"depth/*.png"))}

    prev = {}
    d_normal, d_z, d_R = [], [], []
    flip_mono, flip_temporal = 0, 0
    n = 0
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
            prev = {}; continue
        kp2d = SingleHandDetector.parse_keypoint_2d(lm, rgb.shape)
        dm = depth.astype(np.float32) * 0.001
        pts, ok = [], True
        for idx in PALM:
            z, g = depth_at(dm, *kp2d[idx])
            if not g: ok = False; break
            pts.append([(kp2d[idx][0]-cx)*z/fx, (kp2d[idx][1]-cy)*z/fy, z])
        if not ok:
            prev = {}; continue
        pts = np.array(pts)
        R_mono = physical_wrist_orientation(wrot, det.operator2mano)
        R_depth, resid = depth_wrist_orientation(pts, R_mono)
        if R_depth is None or resid > 8.0:
            prev = {}; continue
        # 原始法向(未定符号)、Z 轴
        c = pts.mean(0); _, S, Vt = np.linalg.svd(pts - c)
        raw_n = Vt[-1] / (np.linalg.norm(Vt[-1])+1e-12)
        z_axis = pts[2] - pts[0]
        n += 1
        if prev:
            d_normal.append(ang(raw_n, prev["n"]))       # 无符号方向抖
            d_z.append(ang(z_axis, prev["z"]))
            d_R.append(R_ang(R_depth, prev["R"]))
            # 符号:单目定 vs 时序定,与上帧比是否翻了
            s_mono = np.sign(raw_n @ R_mono[:, 0])
            s_prev_mono = np.sign(prev["raw_n"] @ prev["Rmono"][:, 0])
            if s_mono != s_prev_mono: flip_mono += 1
            s_temp = np.sign(raw_n @ prev["signed_n"])   # 时序:与上帧已定符号法向同向
            if s_temp < 0: flip_temporal += 1
        signed_n = raw_n * (1 if (prev.get("signed_n") is None or raw_n @ prev["signed_n"] >= 0)
                            else -1) if prev else raw_n * np.sign(raw_n @ R_mono[:, 0])
        prev = {"n": raw_n, "z": z_axis, "R": R_depth, "raw_n": raw_n,
                "Rmono": R_mono, "signed_n": signed_n}

    def st(a, lbl):
        a = np.array(a)
        print(f"  {lbl:26s} 中位={np.median(a):5.1f}°  p90={np.percentile(a,90):5.1f}°  max={a.max():5.0f}°")
    print(f"深度朝向帧={n}, 相邻对={len(d_R)}")
    st(d_normal, "法向(无符号)方向抖")
    st(d_z, "Z轴(腕→中指)方向抖")
    st(d_R, "R_depth 整体逐帧跳")
    print(f"\n符号翻转: 单目定符号 {flip_mono} 次 / 时序定符号 {flip_temporal} 次 (共 {len(d_R)} 对)")
    print("判读: 若'单目定符号'翻转远多于'时序',则符号策略是抖动主因,改时序即可解决。")


if __name__ == "__main__":
    main()
