"""可视化对比:深度拟合手腕朝向 vs 单目朝向。用 Rerun serve-web 逐帧看。

两个视角:
  world/cloud   —— 深度反投的 21 点手部点云(真值几何)
  world/depth_frame, world/mono_frame —— 手腕处两个坐标系三元组(X 红=手掌法向, Z 蓝=腕→中指)
  human (2D)    —— RGB 手图上透视投影叠加两套轴(depth 实/mono 虚由颜色深浅区分)

判读:X 轴哪个真正垂直掌面、Z 轴哪个真正贴腕→中指方向,即为更准的朝向。
"""
import sys, glob, json
from pathlib import Path
import numpy as np
import cv2
import rerun as rr

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from single_hand_detector import SingleHandDetector
from estimate_wrist import physical_wrist_orientation, depth_wrist_orientation
from build_canonical_from_rgbd import TemporalDepthGate

PALM = [0, 5, 9, 13, 17]
ROOT = REPO / "third_party/kinect2-middle/kinect2_middle"
CONN = [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),(0,9),(9,10),(10,11),(11,12),
        (0,13),(13,14),(14,15),(15,16),(0,17),(17,18),(18,19),(19,20)]
AXIS_COLS = np.array([[255,64,64],[64,220,96],[64,142,255]], dtype=np.uint8)


def depth_at(dm, u, v, r=2):
    h, w = dm.shape
    x, y = int(round(u)), int(round(v))
    if x < 0 or x >= w or y < 0 or y >= h:
        return 0.0, False
    p = dm[max(0,y-r):y+r+1, max(0,x-r):x+r+1]
    vv = p[np.isfinite(p) & (p > 0)]
    return (float(np.median(vv)), True) if vv.size else (0.0, False)


def backproj_all(kp2d, dm, fx, fy, cx, cy):
    pts = np.full((21, 3), np.nan)
    val = np.zeros(21, bool)
    for i, (u, v) in enumerate(kp2d):
        z, ok = depth_at(dm, u, v)
        if ok:
            pts[i] = [(u-cx)*z/fx, (v-cy)*z/fy, z]; val[i] = True
    return pts, val


def project(P, fx, fy, cx, cy):
    """3D 相机系点 -> 2D 像素(透视)。"""
    if P[2] <= 1e-6:
        return None
    return np.array([P[0]*fx/P[2]+cx, P[1]*fy/P[2]+cy])


def draw_triad_2d(img, wrist3d, R, fx, fy, cx, cy, L=0.06, dashed=False):
    """把 3D 三元组透视投影到图上。X 红 Y 绿 Z 蓝;dashed=虚线(区分两套)。"""
    o = project(wrist3d, fx, fy, cx, cy)
    if o is None:
        return
    o_i = tuple(np.rint(o).astype(int))
    for a in range(3):
        end3d = wrist3d + R[:, a] * L
        e = project(end3d, fx, fy, cx, cy)
        if e is None:
            continue
        e_i = tuple(np.rint(e).astype(int))
        col = tuple(int(c) for c in AXIS_COLS[a][::-1])  # BGR
        if dashed:
            # 画虚线箭头:分段
            p = np.array(o_i, float); q = np.array(e_i, float); seg = 6
            d = q - p; ln = np.linalg.norm(d) + 1e-9; step = d / ln * seg
            for k in range(0, int(ln // seg), 2):
                a0 = tuple(np.rint(p + step*k).astype(int))
                a1 = tuple(np.rint(p + step*(k+1)).astype(int))
                cv2.line(img, a0, a1, col, 2, cv2.LINE_AA)
        else:
            cv2.arrowedLine(img, o_i, e_i, col, 3, cv2.LINE_AA, tipLength=0.2)


def main():
    intr = json.loads((ROOT/"calibration.json").read_text())["cameras"]["kinect2_middle"]["intrinsics"]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]
    det = SingleHandDetector(hand_type="Right", selfie=False, max_num_hands=1)
    depth_files = {Path(p).name: p for p in glob.glob(str(ROOT/"depth/*.png"))}
    color_files = sorted(glob.glob(str(ROOT/"color/*.png")))

    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", default=str(REPO / "orient_compare.rrd"),
                    help=".rrd 输出路径,用桌面版 Rerun 打开")
    ap.add_argument("--max-frames", type=int, default=0, help="0=全部")
    ap.add_argument("--depth-jump-thresh-m", type=float, default=0.0,
                    help="时序抗跳阈值(米);0=关闭(原始反投,能看到跳变)。设 0.05 看抗跳后")
    args = ap.parse_args()
    gate = TemporalDepthGate(args.depth_jump_thresh_m)

    rr.init("orient_compare")
    rr.save(args.save)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)  # 相机系:Y下 Z前
    print(f"写入 {args.save}")

    fr = 0
    for cf in color_files:
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
        pts, val = backproj_all(kp2d, dm, fx, fy, cx, cy)
        gate.filter(pts[:, 2].copy(), val)   # 时序抗跳:尖峰点置无效(与管线同逻辑)
        pts[~val] = np.nan                   # 被拒的点不画(不再跳出去)
        if not val[PALM].all():
            continue
        R_mono = physical_wrist_orientation(wrot, det.operator2mano)
        R_depth, resid = depth_wrist_orientation(pts[PALM], R_mono)
        if R_depth is None:
            continue
        wrist3d = pts[0]

        rr.set_time("frame", sequence=fr)
        # 3D 点云 + 骨架
        rr.log("world/cloud", rr.Points3D(pts[val], colors=[200,200,210], radii=0.004))
        segs = [[pts[a], pts[b]] for a, b in CONN if val[a] and val[b]]
        if segs:
            rr.log("world/skeleton", rr.LineStrips3D(segs, colors=[120,140,160], radii=0.0015))
        # 两个三元组(3D)
        for tag, R in (("depth_frame", R_depth), ("mono_frame", R_mono)):
            rr.log(f"world/{tag}", rr.Transform3D(translation=wrist3d, mat3x3=R))
            rr.log(f"world/{tag}/xyz", rr.Arrows3D(
                origins=np.zeros((3,3)), vectors=np.eye(3)*0.06,
                colors=AXIS_COLS, radii=0.003))
        # 2D 叠加:depth 实线,mono 虚线
        ov = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
        draw_triad_2d(ov, wrist3d, R_depth, fx, fy, cx, cy, dashed=False)
        draw_triad_2d(ov, wrist3d, R_mono, fx, fy, cx, cy, dashed=True)
        cv2.putText(ov, f"solid=DEPTH  dashed=MONO  resid={resid:.1f}mm", (12,26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(ov, f"solid=DEPTH  dashed=MONO  resid={resid:.1f}mm", (12,26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", ov)
        if ok:
            rr.log("human", rr.EncodedImage(contents=buf.tobytes(), media_type="image/jpeg"))
        fr += 1
        if args.max_frames and fr >= args.max_frames:
            break

    print(f"写完 {fr} 帧 -> {args.save}")
    print(f"Windows 桌面版打开: rerun {Path(args.save).name}  或直接拖入 Rerun 窗口")


if __name__ == "__main__":
    main()
