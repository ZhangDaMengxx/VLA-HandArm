"""验收指标测量:逐项测可测的路线图指标,不可测的明确标注(pass=None)。

返回结构化指标 + 可写 JSON,供 app_web 右侧"数据有效性·验收"卡按本体绑定显示。

用法:
    python3 sim/measure_acceptance.py --robot nero_inspire_rgbd --json sim/out/metrics_nero_inspire_rgbd.json
    python3 sim/measure_acceptance.py --robot nero_gripper_rgbd          # 仅打印
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))


def _clean(v):
    """把 numpy 标量转成干净的 Python 类型(避免 float32 的 10.6999… 尾巴)。"""
    if isinstance(v, np.bool_):
        return bool(v)
    if isinstance(v, np.integer):
        return int(v)
    if isinstance(v, np.floating):
        return round(float(v), 3)
    return v


def _metric(key, label, value, unit, threshold, passed, category, note=""):
    """一条结构化指标。passed: True/False/None(None=无法测)。category: canonical|embodiment。"""
    return {"key": key, "label": label, "value": _clean(value), "unit": unit,
            "threshold": threshold, "pass": _clean(passed), "category": category, "note": note}


def _hand_type(robot: str) -> str:
    """从本体名判手类型:含 gripper→夹爪;否则→inspire 灵巧手。"""
    return "gripper" if "gripper" in robot else "inspire"


def _canonical_df(canonical_dir: Path):
    import pandas as pd, glob
    f = sorted(glob.glob(str(canonical_dir / "data/**/*.parquet"), recursive=True))
    if not f:
        raise FileNotFoundError(f"无 canonical parquet: {canonical_dir}")
    return pd.read_parquet(f[0])


# MANO/MediaPipe 手骨连接(近似,用于骨长恒定性)
BONES = [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),(0,9),(9,10),(10,11),(11,12),
         (0,13),(13,14),(14,15),(15,16),(0,17),(17,18),(18,19),(19,20)]


def measure_gripper():
    """夹爪开合误差<1mm:驱动两指 prismatic 到指令行程,量指尖位移 vs 指令。返回指标列表。"""
    import pinocchio as pin
    m = pin.buildModelFromUrdf(str(REPO / "sim/assets/nero_gripper_right.urdf"))
    d = m.createData()
    fingers = [m.names[i] for i in range(len(m.names)) if "gripper_finger" in m.names[i]]
    tip_frames = [f.name for f in m.frames
                  if "finger" in f.name.lower() and f.type == pin.FrameType.BODY]
    rows = []
    for cmd in np.linspace(0.0, 0.05, 6):        # 指令行程 0~50mm
        q = pin.neutral(m)
        for n in fingers:
            q[m.joints[m.getJointId(n)].idx_q] = cmd
        pin.forwardKinematics(m, d, q)
        pin.updateFramePlacements(m, d)
        tips = [d.oMf[m.getFrameId(t)].translation.copy() for t in tip_frames[:2]]
        if len(tips) == 2:
            rows.append((cmd, np.linalg.norm(tips[0] - tips[1])))
    base = rows[0][1]
    max_e = 0.0
    for cmd, w in rows:
        single = 0.5 * (w - base)         # 单指行程 = 开口变化/2
        max_e = max(max_e, abs(single - cmd))
        print(f"[夹爪] cmd={cmd*1000:5.1f}mm 开口Δ={(w-base)*1000:6.2f}mm 单指行程={single*1000:5.1f}mm")
    print(f"[夹爪] 最大开合误差={max_e*1000:.3f}mm (阈值<1mm) 注:URDF运动学线性映射,非物理仿真")
    return [_metric("gripper_open", "夹爪开合误差", round(max_e*1000, 3), "mm", "<1", max_e*1000 < 1.0,
                    "embodiment", "URDF运动学:指令行程vs指尖位移。线性映射,非物理仿真")]


def measure_detect_scale(df):
    """检出率 + 三维尺度恒定性(骨长帧间标准差)。返回指标列表。"""
    n = len(df)
    vis = np.stack(df["observation.hand_visibility"].values)               # (N,21)
    detected = int((vis.sum(axis=1) > 0).sum())
    kp = np.stack(df["observation.hand_keypoints"].values).reshape(n, 21, 3)
    lengths = np.array([[np.linalg.norm(kp[t, a] - kp[t, b]) for a, b in BONES] for t in range(n)])
    std_l = lengths.std(axis=0)
    rate = detected / n * 100
    worst_mm = std_l.max() * 1000
    med_mm = np.median(std_l) * 1000
    print(f"[检出] {detected}/{n} = {rate:.1f}% (阈值≥90%)")
    print(f"[尺度] 骨长波动 中位std={med_mm:.1f}mm 最差骨={worst_mm:.1f}mm (阈值<10mm)")
    return [
        _metric("detect", "手部检出率", round(rate, 1), "%", "≥90", rate >= 90,
                "canonical", "vis>0 的帧占比"),
        _metric("scale", "尺度恒定性(最差骨)", round(worst_mm, 1), "mm", "<10", worst_mm < 10,
                "canonical", f"骨长帧间波动;中位{med_mm:.1f}mm。无真值,测稳定性代理"),
    ]


def measure_sync(df, robot):
    """LeRobot 加载 + 动作/视频同步:timestamp 均匀性 vs 1/fps。返回指标列表。"""
    info_p = REPO / f"sim/out/lerobot_ds_{robot}/meta/info.json"
    fps = json.loads(info_p.read_text())["fps"] if info_p.exists() else 30.0
    ts = df["timestamp"].values.astype(float)
    jitter_ms = np.abs(np.diff(ts) - 1.0 / fps) * 1000
    mx = float(jitter_ms.max())
    print(f"[同步] fps={fps} 帧间隔抖动 中位={np.median(jitter_ms):.3f}ms 最大={mx:.3f}ms (阈值<10ms)")
    return [_metric("sync", "动作/视频同步抖动", round(mx, 3), "ms", "<10", mx < 10,
                    "canonical", "帧间隔vs1/fps。timestamp同源合成,验内部一致非硬件同步")]


def measure_align(df):
    """RGB/Depth 对齐(间接):腕深度连续性 + 关键点有效性。返回指标列表。"""
    wp = np.stack(df["observation.wrist_pose"].values)                     # (N,7)
    kp = np.stack(df["observation.hand_keypoints"].values).reshape(-1, 21, 3)
    dz = np.abs(np.diff(wp[:, 2]))
    p99 = float(np.percentile(dz, 99) * 1000)
    finite = bool(np.isfinite(kp).all())
    no_collapse = bool(np.all(np.linalg.norm(kp[:, 12] - kp[:, 0], axis=1) > 0.05))
    ok = finite and no_collapse
    print(f"[对齐] 腕深度p99跳变={p99:.1f}mm 有限={finite} 无塌陷={no_collapse}")
    return [_metric("align", "RGB/Depth对齐", "OK" if ok else "异常", "", "无明显错位", ok,
                    "canonical", f"间接:腕深度连续(p99={p99:.1f}mm)+关键点有效。严格像素级测试缺原始未配准流")]


def measure_retarget(df, robot):
    """指尖重定向误差<1.5cm + 越限 + 跳变。robot FK 向量 vs 缩放人手向量。返回指标列表。"""
    from dex_retargeting.retargeting_config import RetargetingConfig
    from robot_specs import get_spec
    spec = get_spec(robot)
    kps = np.stack(df["observation.hand_keypoints"].values).reshape(-1, 21, 3)
    RetargetingConfig.set_default_urdf_dir(str(spec.urdf_dir))
    rt = RetargetingConfig.load_from_file(str(spec.retarget_cfg),
                                          override={"low_pass_alpha": 1.0}).build()
    opt = rt.optimizer; robot_m = opt.robot
    idx = np.asarray(opt.target_link_human_indices)
    origin_i, task_i = idx[0, :], idx[1, :]
    scaling = opt.scaling if hasattr(opt, "scaling") else 1.15
    o_links = [robot_m.get_link_index(n) for n in opt.origin_link_names]
    t_links = [robot_m.get_link_index(n) for n in opt.task_link_names]
    lower, upper = robot_m.joint_limits[:, 0], robot_m.joint_limits[:, 1]
    errs, qs = [], []
    for f in range(len(kps)):
        ref = kps[f][task_i, :] - kps[f][origin_i, :]
        q = rt.retarget(ref); qs.append(q)
        robot_m.compute_forward_kinematics(q)
        rob_vec = np.array([robot_m.get_link_pose(t)[:3, 3] - robot_m.get_link_pose(o)[:3, 3]
                            for o, t in zip(o_links, t_links)])
        errs.append(np.linalg.norm(rob_vec - ref * scaling, axis=1))
    errs = np.array(errs); qs = np.array(qs)
    med_cm = float(np.median(errs) * 100)
    over = np.maximum(lower - qs, qs - upper)
    real_viol = int((over > 1e-2).sum())
    jump_max = float(np.degrees(np.abs(np.diff(qs, axis=0)).max()))
    print(f"[重定向] 综合中位={med_cm:.2f}cm (阈值<1.5) 真越限={real_viol} 帧间跳变max={jump_max:.1f}°")
    return [
        _metric("retarget", "指尖重定向误差(中位)", round(med_cm, 2), "cm", "<1.5", med_cm < 1.5,
                "embodiment", "机器手FK向量vs缩放人手向量。拇指本体差异+单目噪声为主因"),
        _metric("joint_limit", "关节越限次数", real_viol, "", "=0", real_viol == 0,
                "embodiment", "真越限(>0.57°);贴限位的饱和不计"),
        _metric("joint_jump", "关节帧间跳变(最大)", round(jump_max, 1), "°", "平滑", jump_max < 30,
                "embodiment", "单步最大关节变化;大跳来自单目输入噪声"),
    ]


def run_all(robot: str, canonical_dir: Path):
    """按本体跑全部适用指标,返回 {robot, hand, metrics:[...]}。"""
    hand = _hand_type(robot)
    df = _canonical_df(canonical_dir)
    metrics = []
    metrics += measure_detect_scale(df)
    metrics += measure_sync(df, robot)
    metrics += measure_align(df)
    if hand == "gripper":
        metrics += measure_gripper()
    else:
        metrics += measure_retarget(df, robot)
    # 无法测项:内参重投影(缺标定角点数据)
    metrics.append(_metric("reproj", "内参重投影误差", None, "px", "<0.5", None,
                           "canonical", "缺标定角点/棋盘格原始数据,无法重算。需原始标定图像"))
    return {"robot": robot, "hand": hand, "metrics": metrics}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="nero_inspire_rgbd")
    ap.add_argument("--canonical", default=str(REPO / "sim/out/canonical_ds"))
    ap.add_argument("--json", default="")
    args = ap.parse_args()
    result = run_all(args.robot, Path(args.canonical))
    if args.json:
        def _np(o):
            if isinstance(o, np.bool_):
                return bool(o)
            if isinstance(o, np.integer):
                return int(o)
            if isinstance(o, np.floating):
                return float(o)
            raise TypeError(o)
        Path(args.json).write_text(json.dumps(result, ensure_ascii=False, indent=2, default=_np),
                                   encoding="utf-8")
        print(f"[验收] 写入 {args.json}")
