"""诊断:深度朝向 vs 单目朝向的相对旋转 R_rel = R_mono^T @ R_depth 是否为常量。

常量偏置 -> 深度是有效坐标系,只需把偏置吸收进 R_hand_ee 重新推导。
乱跳    -> 深度朝向本身不可靠(或我的 Z 轴定义与单目不同源)。
"""
import sys, glob, json
from pathlib import Path
import numpy as np
import cv2

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from single_hand_detector import SingleHandDetector
from estimate_wrist import physical_wrist_orientation, depth_wrist_orientation
from scipy.spatial.transform import Rotation as Rot

PALM = [0, 5, 9, 13, 17]
ROOT = REPO / "third_party/kinect2-middle/kinect2_middle"


def depth_at(dm, u, v, r=2):
    h, w = dm.shape
    x, y = int(round(u)), int(round(v))
    if x < 0 or x >= w or y < 0 or y >= h:
        return 0.0, False
    p = dm[max(0, y-r):y+r+1, max(0, x-r):x+r+1]
    v_ = p[np.isfinite(p) & (p > 0)]
    return (float(np.median(v_)), True) if v_.size else (0.0, False)


def main():
    intr = json.loads((ROOT/"calibration.json").read_text())["cameras"]["kinect2_middle"]["intrinsics"]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
    det = SingleHandDetector(hand_type="Right", selfie=False, max_num_hands=1)
    depth_files = {Path(p).name: p for p in glob.glob(str(ROOT/"depth/*.png"))}

    rotvecs = []
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
            continue
        kp2d = SingleHandDetector.parse_keypoint_2d(lm, rgb.shape)
        dm = depth.astype(np.float32) * 0.001
        pts, ok = [], []
        for idx in PALM:
            z, g = depth_at(dm, *kp2d[idx]); ok.append(g)
            pts.append([(kp2d[idx][0]-cx)*z/fx, (kp2d[idx][1]-cy)*z/fy, z] if g else [np.nan]*3)
        if not all(ok):
            continue
        R_mono = physical_wrist_orientation(wrot, det.operator2mano)
        R_depth, resid = depth_wrist_orientation(np.array(pts), R_mono)
        if R_depth is None or resid > 8.0:
            continue
        R_rel = R_mono.T @ R_depth
        rotvecs.append(Rot.from_matrix(R_rel).as_rotvec())

    V = np.array(rotvecs)
    angs = np.degrees(np.linalg.norm(V, axis=1))
    mean_rv = V.mean(0)
    mean_ang = np.degrees(np.linalg.norm(mean_rv))
    # 每帧 R_rel 相对"平均 R_rel"的残差角:小=常量偏置,大=乱跳
    R_mean = Rot.from_rotvec(mean_rv)
    resid_ang = np.degrees([np.linalg.norm((R_mean.inv()*Rot.from_rotvec(v)).as_rotvec()) for v in V])
    print(f"帧数(有效): {len(V)}")
    print(f"R_rel 旋转角:          中位={np.median(angs):.1f}°  p90={np.percentile(angs,90):.1f}°")
    print(f"平均 R_rel 的角度(偏置量): {mean_ang:.1f}°  轴={mean_rv/(np.linalg.norm(mean_rv)+1e-9)}")
    print(f"每帧偏离平均偏置的残差角:  中位={np.median(resid_ang):.1f}°  p90={np.percentile(resid_ang,90):.1f}°")
    print("\n判读: 残差角小(<~5°)=常量偏置可吸收进 R_hand_ee; 大=深度朝向本身抖/异源。")


if __name__ == "__main__":
    main()
