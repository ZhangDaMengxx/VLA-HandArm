"""src/replay_rerun.py — 人手视频 + NERO+inspire 机器人回放,在 Rerun 里同一时间轴硬同步可视化。

三块面板同一时间轴联动(拖时间轴 / 播放,三块一起走):
  - Human       : 源视频 + MediaPipe 21 点骨架
  - Robot (3D)  : NERO(7-DoF) + inspire(12) 装配网格,可鼠标轨道旋转/缩放
  - Joint angles: 臂 7 + 手 12 关节角曲线,竖直游标 = 当前帧

数据来自已算好的轨迹(默认 src/out/robot_traj_nero_inspire.pkl,两层路径产物;无则回退旧 robot_traj.pkl),不再实时 retarget。
支持多条轨迹做 A/B 对比(--traj 标签=路径,可重复),各自成一棵实体树,在左侧面板勾选显隐。

用法:
  # 1) 存 .rrd,拷到 Windows 用 Rerun 查看器(pip install rerun-sdk 后 `rerun 文件.rrd`,或桌面版)打开
  python src/replay_rerun.py
  # 2) 直接在浏览器看:WSL 起服务,Windows 浏览器打开打印出来的 URL
  python src/replay_rerun.py --serve
  # 3) A/B 对比两条 retarget 轨迹
  python src/replay_rerun.py --traj default=src/out/robot_traj.pkl --traj tippip=src/out/robot_traj_tippip.pkl
"""
from __future__ import annotations

import argparse
import pickle
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import pinocchio as pin
import rerun as rr
import rerun.blueprint as rrb
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as Rot

from estimate_wrist import physical_wrist_orientation
from robot_specs import get_spec, axis_tokens_to_R_hand_ee
from wrist_stabilize import attenuate_out_of_plane, gate_outliers

REPO = Path(__file__).resolve().parents[1]


def log(msg: str) -> None:
    print(f"[rerun-viz] {msg}", flush=True)


# 每指/每段配色(RGB 0-255),按 geom/link 名子串匹配 —— 让机器人一眼分得清结构。
PALETTE = {
    "thumb": (255, 82, 71),
    "index": (82, 199, 108),
    "middle": (64, 156, 255),
    "ring": (255, 196, 61),
    "pinky": (197, 108, 255),
    "base": (120, 124, 132),     # inspire 掌基座
    "link": (196, 202, 214),     # NERO 臂
}
DEFAULT_COLOR = (196, 202, 214)


def geom_color(name: str) -> Tuple[int, int, int]:
    low = name.lower()
    for key, col in PALETTE.items():
        if key in low:
            return col
    return DEFAULT_COLOR


# ---------------------------------------------------------------------------
# 机器人模型:pinocchio 建运动学 + 视觉几何,按轨迹里的关节名喂 full q。
# ---------------------------------------------------------------------------
class RobotModel:
    def __init__(self, urdf_path: Path):
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.visual = pin.buildGeomFromUrdf(self.model, str(urdf_path), pin.GeometryType.VISUAL)
        self.gdata = self.visual.createData()
        self.q0 = pin.neutral(self.model)
        self.name_to_qidx = {
            self.model.names[j]: self.model.joints[j].idx_q
            for j in range(1, self.model.njoints)
        }
        log(f"URDF={urdf_path.name} nq={self.model.nq} 视觉几何={self.visual.ngeoms}")

    def make_q(self, arm: np.ndarray, arm_names: List[str],
               hand: np.ndarray, hand_names: List[str]) -> np.ndarray:
        q = self.q0.copy()
        for k, n in enumerate(arm_names):
            qi = self.name_to_qidx.get(n)
            if qi is not None:
                q[qi] = arm[k]
        for k, n in enumerate(hand_names):
            # 平行夹爪:轨迹里是 1 个 "gripper_joint" 标量 = 开口宽度(m,与官方 SDK
            # move_gripper_m 同量纲)。URDF 是两个对称 prismatic 手指,各走开口的一半,
            # 所以这里乘 0.5(等价于官方 URDF 主关节 gripper 的 mimic multiplier=±0.5)。
            if n == "gripper_joint":
                for fj in ("gripper_finger_left_joint", "gripper_finger_right_joint"):
                    qi = self.name_to_qidx.get(fj)
                    if qi is not None:
                        q[qi] = hand[k] * 0.5
                continue
            qi = self.name_to_qidx.get(n)
            if qi is not None:
                q[qi] = hand[k]
        return q

    def placements(self, q: np.ndarray) -> List[np.ndarray]:
        """每个视觉几何体的世界位姿 oMg(4x4)。"""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(self.model, self.data, self.visual, self.gdata, q)
        return [self.gdata.oMg[i].homogeneous.copy() for i in range(self.visual.ngeoms)]

    def frame_placement(self, q: np.ndarray, frame_name: str) -> np.ndarray:
        """指定 frame 在机器人 base/world 里的位姿。"""
        fid = self.model.getFrameId(frame_name)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return np.array(self.data.oMf[fid].homogeneous)


def load_meshes(model: RobotModel) -> List[Optional[dict]]:
    """把每个视觉几何体加载成 rerun Mesh3D 需要的顶点/面/法线/颜色(局部坐标,含 meshScale)。"""
    import trimesh

    def _candidates(path: str) -> List[str]:
        """原路径 + .dae 的 .stl 等价物回退(NERO 视觉用 dae,但同目录/上级有等价 stl,免 pycollada)。"""
        p = Path(path)
        cands = [str(p)]
        if p.suffix.lower() == ".dae":
            # meshes/dae/link1.dae -> meshes/dae/link1.stl 和 meshes/link1.stl
            cands.append(str(p.with_suffix(".stl")))
            if p.parent.name == "dae":
                cands.append(str(p.parent.parent / (p.stem + ".stl")))
        return cands

    out: List[Optional[dict]] = []
    for i in range(model.visual.ngeoms):
        gobj = model.visual.geometryObjects[i]
        path = gobj.meshPath
        if not path:
            out.append(None)
            continue
        loaded, last_err = None, None
        for cand in _candidates(path):
            if not Path(cand).exists():
                continue
            try:
                loaded = trimesh.load(cand, process=False, force="mesh")
                break
            except Exception as e:
                last_err = e
        if loaded is None:
            log(f"网格加载失败({Path(path).name}): {last_err}")
            out.append(None)
            continue
        if loaded.vertices is None or len(loaded.vertices) == 0:
            out.append(None)
            continue
        scale = np.asarray(gobj.meshScale).reshape(3)
        V = np.asarray(loaded.vertices, np.float32) * scale
        F = np.asarray(loaded.faces, np.uint32)
        try:
            N = np.asarray(loaded.vertex_normals, np.float32)
        except Exception:
            N = None
        out.append(dict(name=gobj.name, V=V, F=F, N=N, color=geom_color(gobj.name)))
    n_ok = sum(x is not None for x in out)
    log(f"网格加载:{n_ok}/{model.visual.ngeoms} 成功")
    return out


# ---------------------------------------------------------------------------
# 轨迹加载(支持 A/B)
# ---------------------------------------------------------------------------
def parse_traj_args(items: List[str]) -> List[Tuple[str, Path]]:
    specs = []
    for it in items:
        label, sep, path = it.partition("=")
        if not sep:
            label, path = Path(it).stem, it
        p = Path(path)
        if not p.is_absolute():
            p = (REPO / p).resolve()
        specs.append((label, p))
    return specs


def load_traj(path: Path) -> dict:
    with open(path, "rb") as f:
        T = pickle.load(f)
    return dict(arm=np.asarray(T["arm"]), hand=np.asarray(T["hand"]),
                arm_names=list(T["arm_joint_names"]), hand_names=list(T["hand_joint_names"]))


def _pose_vec_to_mat(v: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = v[:3]
    T[:3, :3] = Rot.from_quat(v[3:7]).as_matrix()
    return T


def load_canonical_wrist_poses(root: Path) -> Optional[np.ndarray]:
    """读取 canonical_ds 里的 raw wrist pose。没有 canonical 时返回 None。"""
    try:
        import pandas as pd

        files = sorted((root / "data").glob("chunk-*/file-*.parquet"))
        if not files:
            return None
        df = pd.concat([pd.read_parquet(p) for p in files], ignore_index=True)
        if "frame_index" in df.columns:
            df = df.sort_values("frame_index")
        if "observation.wrist_pose" not in df.columns:
            return None
        vals = np.stack(df["observation.wrist_pose"].to_numpy()).astype(np.float64)
        return np.stack([_pose_vec_to_mat(v.reshape(-1)) for v in vals])
    except Exception as e:
        log(f"canonical wrist pose 不可用: {e}")
        return None


def _smooth_relative_positions(wrist_poses: np.ndarray, spec) -> np.ndarray:
    ps = np.asarray(wrist_poses[:, :3, 3], dtype=np.float64)
    if len(ps) >= spec.savgol_win:
        ps = savgol_filter(ps, spec.savgol_win, spec.savgol_poly, axis=0)
    position_basis = Rot.from_euler("xyz", spec.wrist_position_basis_rpy).as_matrix()
    dp = ((ps - ps[0]) @ position_basis.T) * float(spec.arm_position_gain)
    limit = float(spec.arm_position_limit_m)
    if limit > 0:
        norms = np.linalg.norm(dp, axis=1)
        mask = norms > limit
        dp[mask] *= (limit / norms[mask])[:, None]
    return dp


def compute_target_ee_poses(wrist_poses: np.ndarray, spec) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """按 derive_embodiment.py 的同一套映射,重算 IK 目标末端姿态。

    返回 (target_ee_poses, human_wrist_in_base):
      - target_ee_poses: IK 目标末端 4x4(N)。
      - human_wrist_in_base: anchored 模式下把 raw wrist 投到 base 系、并挪到 ee target 位置的
        4x4(N),用于在同一坐标系里直观对比 current/target;legacy 模式返回 None。
    """
    N = len(wrist_poses)
    quats = Rot.from_matrix(wrist_poses[:, :3, :3]).as_quat()
    for i in range(1, N):
        if np.dot(quats[i - 1], quats[i]) < 0:
            quats[i] = -quats[i]
    quats = gate_outliers(quats, spec.gate_deg)
    if N >= spec.savgol_win:
        quats_s = savgol_filter(quats, spec.savgol_win, spec.savgol_poly, axis=0)
    else:
        quats_s = quats
    quats_s /= np.linalg.norm(quats_s, axis=1, keepdims=True)
    Rs = Rot.from_quat(quats_s).as_matrix()
    Rs = attenuate_out_of_plane(Rs, spec.oop_alpha, ref=0)

    from nero_kin import NeroKin

    kin = NeroKin(spec.arm_urdf, ee_frame=spec.ee_frame)
    anchor = kin.fk(spec.q_home)
    aR, ap = anchor[:3, :3], anchor[:3, 3]

    out = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], N, axis=0)

    if spec.frame_mode == "metric":
        # 固定 R_base_world/anchor/scale 绝对映射(与 derive_embodiment._solve_arm_metric 同公式)。
        R_hand_ee = np.asarray(spec.R_hand_ee, dtype=np.float64).reshape(3, 3)
        R_base_world = np.asarray(spec.R_base_world, dtype=np.float64).reshape(3, 3)
        anchor_p = np.asarray(spec.p_base_anchor, dtype=np.float64).reshape(3)
        scale = float(spec.metric_scale)
        if spec.arm_position_mode == "fixed":
            dp = np.zeros((N, 3), dtype=np.float64)
        else:
            ps = np.asarray(wrist_poses[:, :3, 3], dtype=np.float64)
            if len(ps) >= spec.savgol_win:
                ps = savgol_filter(ps, spec.savgol_win, spec.savgol_poly, axis=0)
            dp = scale * ((ps - ps.mean(0)) @ R_base_world.T)
        human = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], N, axis=0)
        for f in range(N):
            out[f, :3, :3] = R_base_world @ Rs[f] @ R_hand_ee
            out[f, :3, 3] = anchor_p + dp[f]
            human[f, :3, :3] = R_base_world @ Rs[f]   # 人手投到 base,和 target 只差固定装配 R_hand_ee
            human[f, :3, 3] = anchor_p + dp[f]
        return out, human

    if spec.frame_mode == "anchored":
        R_hand_ee = np.asarray(spec.R_hand_ee, dtype=np.float64).reshape(3, 3)
        R_base_world = aR @ R_hand_ee.T @ Rs[0].T   # 与 derive_embodiment.anchored_base_world 一致
        if spec.arm_position_mode == "fixed":
            dp = np.zeros((N, 3), dtype=np.float64)
        else:
            ps = np.asarray(wrist_poses[:, :3, 3], dtype=np.float64)
            if len(ps) >= spec.savgol_win:
                ps = savgol_filter(ps, spec.savgol_win, spec.savgol_poly, axis=0)
            dp = ((ps - ps[0]) @ R_base_world.T) * float(spec.arm_position_gain)
            limit = float(spec.arm_position_limit_m)
            if limit > 0:
                norms = np.linalg.norm(dp, axis=1)
                mask = norms > limit
                dp[mask] *= (limit / norms[mask])[:, None]
        human = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], N, axis=0)
        for f in range(N):
            out[f, :3, :3] = R_base_world @ Rs[f] @ R_hand_ee
            out[f, :3, 3] = ap + dp[f]
            # human wrist 投到 base 系,和 target 同位置 —— 两者只差固定装配旋转 R_hand_ee
            human[f, :3, :3] = R_base_world @ Rs[f]
            human[f, :3, 3] = ap + dp[f]
        return out, human

    ee_fix = Rot.from_euler("xyz", spec.ee_frame_correction_rpy).as_matrix()
    B = np.asarray(spec.wrist_motion_basis_R, dtype=np.float64).reshape(3, 3)
    R0 = Rs[0]
    if spec.arm_position_mode == "fixed":
        dp = np.zeros((N, 3), dtype=np.float64)
    else:
        dp = _smooth_relative_positions(wrist_poses, spec)

    for f in range(N):
        if spec.wrist_rotation_compose == "left":
            dR_human = Rs[f] @ R0.T
            dR_robot = B @ dR_human @ B.T
            Rt = dR_robot @ aR @ ee_fix
        else:
            dR_human = R0.T @ Rs[f]
            dR_robot = B @ dR_human @ B.T
            Rt = aR @ dR_robot @ ee_fix
        out[f, :3, :3] = Rt
        out[f, :3, 3] = ap + dp[f]
    return out, None


AXIS_COLORS = np.array([
    [255, 64, 64],
    [64, 220, 96],
    [64, 142, 255],
], dtype=np.uint8)
AXIS_COLOR_NAMES = (("X", (255, 64, 64)), ("Y", (64, 220, 96)), ("Z", (64, 142, 255)))


def log_axes(entity: str, T: np.ndarray, length: float = 0.08, radius: float = 0.004) -> None:
    """在 entity 处画一个 XYZ 坐标轴。X=红,Y=绿,Z=蓝。"""
    rr.log(entity, rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3]))
    rr.log(
        f"{entity}/xyz",
        rr.Arrows3D(
            origins=np.zeros((3, 3), dtype=np.float32),
            vectors=np.eye(3, dtype=np.float32) * float(length),
            colors=AXIS_COLORS,
            radii=np.full(3, float(radius), dtype=np.float32),
        ),
    )


def draw_wrist_axes_on_image(
    image_rgb: np.ndarray,
    origin_px: np.ndarray,
    R: np.ndarray,
    length_px: float = 52.0,
) -> np.ndarray:
    """把 3D wrist frame 的 XYZ 方向投影到 Human 2D 面板。

    只画方向,不做相机透视投影。颜色与 3D Rerun 坐标轴一致:X 红、Y 绿、Z 蓝。
    """
    if origin_px is None or R is None:
        return image_rgb
    if not np.isfinite(origin_px).all() or not np.isfinite(R).all():
        return image_rgb
    out = image_rgb
    h, w = out.shape[:2]
    origin = np.asarray(origin_px, dtype=np.float64).reshape(2)
    if origin[0] < -w or origin[0] > 2 * w or origin[1] < -h or origin[1] > 2 * h:
        return out
    o = tuple(np.rint(origin).astype(int))
    cv2.circle(out, o, 5, (24, 28, 36), -1, cv2.LINE_AA)
    for axis_i, (label, color) in enumerate(AXIS_COLOR_NAMES):
        v = np.asarray(R[:2, axis_i], dtype=np.float64)
        n = float(np.linalg.norm(v))
        if n < 1e-6:
            continue
        end = origin + (v / n) * float(length_px)
        e = tuple(np.rint(end).astype(int))
        cv2.arrowedLine(out, o, e, color, 3, cv2.LINE_AA, tipLength=0.24)
        text_pos = tuple(np.rint(end + np.array([4.0, -4.0])).astype(int))
        cv2.putText(out, label, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    color, 2, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------------------
# 人手视频 + 骨架
# ---------------------------------------------------------------------------
def open_video(path: Path):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise FileNotFoundError(f"无法打开视频: {path}")
    return cap


class BlankVideo:
    def __init__(self, width: int = 640, height: int = 360):
        self.frame = np.full((height, width, 3), 245, dtype=np.uint8)
        cv2.putText(self.frame, "processed hand file", (32, height // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (82, 88, 102), 2, cv2.LINE_AA)

    def read(self):
        return True, self.frame.copy()

    def get(self, prop):
        if prop == cv2.CAP_PROP_FPS:
            return 30.0
        return 0.0

    def release(self):
        pass


HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17),
]


class SkeletonVideo:
    def __init__(self, points: np.ndarray, wrist_poses: Optional[np.ndarray] = None,
                 width: int = 640, height: int = 360):
        self.width = width
        self.height = height
        self.points = self._fit_to_canvas(points.astype(np.float32), width, height)
        self.wrist_poses = wrist_poses
        self.i = 0

    @classmethod
    def from_canonical(cls, root: Path):
        import pandas as pd

        files = sorted((root / "data").glob("chunk-*/file-*.parquet"))
        if not files:
            raise FileNotFoundError(f"canonical parquet not found under {root}")
        df = pd.concat([pd.read_parquet(p) for p in files], ignore_index=True)
        if "frame_index" in df.columns:
            df = df.sort_values("frame_index")
        wrist_poses = None
        if "observation.wrist_pose" in df.columns:
            vals = np.stack(df["observation.wrist_pose"].to_numpy()).astype(np.float64)
            wrist_poses = np.stack([_pose_vec_to_mat(v.reshape(-1)) for v in vals])
        kp2d = None
        if "observation.hand_keypoints_2d" in df.columns:
            kp2d = np.stack(df["observation.hand_keypoints_2d"].to_numpy()).reshape(-1, 21, 2)
            valid = np.isfinite(kp2d).all() and np.nanmax(np.ptp(kp2d, axis=1)) > 1e-4
            if valid:
                # External pipelines may export normalized image coordinates.
                if np.nanmax(np.abs(kp2d)) <= 2.0:
                    kp2d = kp2d.copy()
                    kp2d[:, :, 0] *= 640.0
                    kp2d[:, :, 1] *= 360.0
                return cls(kp2d, wrist_poses=wrist_poses)
        kps = np.stack(df["observation.hand_keypoints"].to_numpy()).reshape(-1, 21, 3)
        return cls(cls._project_3d(kps), wrist_poses=wrist_poses)

    @staticmethod
    def _fit_to_canvas(points: np.ndarray, width: int, height: int, margin: int = 42) -> np.ndarray:
        out = np.empty_like(points, dtype=np.float32)
        avail_w = max(float(width - 2 * margin), 1.0)
        avail_h = max(float(height - 2 * margin), 1.0)
        for i, pts in enumerate(points):
            valid = np.isfinite(pts).all(axis=1)
            if valid.sum() < 2:
                out[i] = np.array([width * 0.5, height * 0.55], dtype=np.float32)
                continue
            p = pts.copy()
            lo = np.nanmin(p[valid], axis=0)
            hi = np.nanmax(p[valid], axis=0)
            span = np.maximum(hi - lo, 1e-4)
            scale = min(avail_w / float(span[0]), avail_h / float(span[1]))
            center = (lo + hi) * 0.5
            target = np.array([width * 0.5, height * 0.55], dtype=np.float32)
            out[i] = (p - center) * scale + target
        return out

    @staticmethod
    def _project_3d(kps: np.ndarray, width: int = 640, height: int = 360) -> np.ndarray:
        flat = kps.reshape(-1, 3)
        flat = flat[np.isfinite(flat).all(axis=1)]
        if len(flat) < 3:
            return np.full((len(kps), 21, 2), [width * 0.5, height * 0.55], dtype=np.float32)
        center = np.nanmean(flat, axis=0, keepdims=True)
        centered = flat - center
        _, s, vh = np.linalg.svd(centered, full_matrices=False)
        if len(s) >= 2 and s[1] > max(s[0] * 1e-3, 1e-6):
            basis = vh[:2].T
            pts = (kps - center.reshape(1, 1, 3)) @ basis
        else:
            spreads = np.ptp(flat, axis=0)
            axes = np.argsort(spreads)[-2:]
            pts = kps[:, :, axes].copy()
        out = np.zeros_like(pts, dtype=np.float32)
        for i, p in enumerate(pts):
            p = p - np.nanmean(p, axis=0, keepdims=True)
            span = np.nanmax(np.ptp(p, axis=0))
            scale = 180.0 / max(float(span), 1e-4)
            out[i, :, 0] = p[:, 0] * scale + width * 0.5
            out[i, :, 1] = -p[:, 1] * scale + height * 0.55
        return out

    def read(self):
        idx = min(self.i, len(self.points) - 1)
        self.i += 1
        wrist_pose = None
        if self.wrist_poses is not None and len(self.wrist_poses):
            wrist_pose = self.wrist_poses[min(idx, len(self.wrist_poses) - 1)]
        return True, self._draw(self.points[idx], wrist_pose)

    def get(self, prop):
        if prop == cv2.CAP_PROP_FPS:
            return 30.0
        return 0.0

    def release(self):
        pass

    def _draw(self, pts: np.ndarray, wrist_pose: Optional[np.ndarray]) -> np.ndarray:
        img = np.full((self.height, self.width, 3), 248, dtype=np.uint8)
        cv2.putText(img, "processed hand skeleton", (24, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (82, 88, 102), 1, cv2.LINE_AA)
        pts_i = np.rint(pts).astype(int)
        colors = {
            "thumb": (71, 82, 255),
            "index": (108, 199, 82),
            "middle": (255, 156, 64),
            "ring": (61, 196, 255),
            "pinky": (255, 108, 197),
            "palm": (132, 124, 120),
        }
        for a, b in HAND_CONNECTIONS:
            col = colors["palm"]
            if max(a, b) <= 4:
                col = colors["thumb"]
            elif max(a, b) <= 8:
                col = colors["index"]
            elif max(a, b) <= 12:
                col = colors["middle"]
            elif max(a, b) <= 16:
                col = colors["ring"]
            elif max(a, b) <= 20:
                col = colors["pinky"]
            pa, pb = tuple(pts_i[a]), tuple(pts_i[b])
            cv2.line(img, pa, pb, col, 2, cv2.LINE_AA)
        for j, p in enumerate(pts_i):
            cv2.circle(img, tuple(p), 4 if j == 0 else 3, (32, 36, 44), -1, cv2.LINE_AA)
        if wrist_pose is not None:
            img = draw_wrist_axes_on_image(img, pts[0], wrist_pose[:3, :3])
        return img


def primary_ip() -> str:
    """WSL 的主 IP(非环回)——Windows 浏览器要用它连回 WSL。"""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--traj", action="append", default=[],
                    help="轨迹 pkl,可写 标签=路径;可重复做 A/B。默认 robot_traj_nero_inspire.pkl(回退 robot_traj.pkl)")
    ap.add_argument("--urdf", default="",
                    help="可视化 URDF;留空则用 RobotSpec.viz_urdf(inspire=灵巧手, gripper=平行夹爪)")
    ap.add_argument("--video", default=str(REPO / "data/hand_1.mp4"))
    ap.add_argument("--no-video", action="store_true",
                    help="不读取源视频,Human 面板使用占位帧;用于外部处理好的 hand file")
    ap.add_argument("--fps", type=float, default=25.0)
    ap.add_argument("--no-skeleton", dest="skeleton", action="store_false",
                    help="不在 human 面板上叠 MediaPipe 骨架(跳过重检测,更快)")
    ap.add_argument("--serve", action="store_true",
                    help="起 web 服务在浏览器看(默认改为存 .rrd)")
    ap.add_argument("--web-port", type=int, default=9090)
    ap.add_argument("--grpc-port", type=int, default=9876)
    ap.add_argument("--save", default=str(REPO / "src/out/replay.rrd"),
                    help="非 --serve 时写入的 .rrd 路径")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--debug-frames", action=argparse.BooleanOptionalAction, default=True,
                    help="在 3D 视图画 robot base/link7 和 canonical wrist raw 坐标轴")
    ap.add_argument("--ee-frame", default="link7",
                    help="调试坐标轴使用的机器人末端 frame 名")
    ap.add_argument("--robot", default="nero_inspire",
                    help="用于重算 debug IK target frame 的 RobotSpec 名")
    ap.add_argument("--arm-position-mode", choices=["fixed", "relative", "absolute"], default=None,
                    help="重算 debug IK target frame 时覆盖 RobotSpec.arm_position_mode;应与 derive_embodiment.py 生成轨迹时一致")
    ap.add_argument("--wrist-rotation-compose", choices=["left", "right"], default=None,
                    help="重算 debug IK target frame 时覆盖 RobotSpec.wrist_rotation_compose;应与 derive_embodiment.py 生成轨迹时一致")
    ap.add_argument("--r-hand-ee", default=None, metavar="HX,HY,HZ",
                    help="anchored 装配轴映射覆盖,逗号分隔(human X/Y/Z -> ee 轴,如 -Y,-Z,+X);须与 derive_embodiment 生成轨迹时一致")
    args = ap.parse_args()

    # 默认轨迹随 --robot 走(spec 名映射到 derive 产物);回退旧单本体路径。
    _default_traj = REPO / f"src/out/robot_traj_{args.robot}.pkl"
    if not _default_traj.exists():
        _alias = REPO / "src/out/robot_traj_nero_inspire.pkl"
        _default_traj = _alias if _alias.exists() else REPO / "src/out/robot_traj.pkl"
    traj_items = args.traj or [f"default={_default_traj}"]
    traj_specs = parse_traj_args(traj_items)

    spec = get_spec(args.robot)
    urdf_path = Path(args.urdf) if args.urdf else spec.viz_urdf
    model = RobotModel(urdf_path)
    meshes = load_meshes(model)
    if args.arm_position_mode is not None:
        spec.arm_position_mode = args.arm_position_mode
    if args.wrist_rotation_compose is not None:
        spec.wrist_rotation_compose = args.wrist_rotation_compose
    if args.r_hand_ee is not None:
        spec.R_hand_ee = axis_tokens_to_R_hand_ee(args.r_hand_ee.split(","))

    trajs = []
    for label, path in traj_specs:
        if not path.exists():
            log(f"跳过轨迹(找不到): {path}")
            continue
        T = load_traj(path)
        trajs.append((label, T))
        log(f"轨迹 '{label}': arm{T['arm'].shape} hand{T['hand'].shape}  <- {path.name}")
    if not trajs:
        raise SystemExit("没有可用的轨迹")

    F = min(len(T["arm"]) for _, T in trajs)
    if args.max_frames:
        F = min(F, args.max_frames)

    wrist_poses = load_canonical_wrist_poses(REPO / "src/out/canonical_ds") if args.debug_frames else None
    target_ee_poses = None
    human_wrist_in_base = None
    if wrist_poses is not None:
        F = min(F, len(wrist_poses))
        log(f"坐标系调试:读取 canonical wrist_pose {len(wrist_poses)} 帧")
        target_ee_poses, human_wrist_in_base = compute_target_ee_poses(wrist_poses[:F], spec)
        if human_wrist_in_base is not None:
            log("坐标系调试[anchored]:human_wrist_in_base 已用 R_base_world 投到机器人 base 系,与 target 同位置;两者只应差固定装配旋转 R_hand_ee,若逐帧同步转动=坐标系已自洽")
        else:
            log("坐标系调试[legacy]:human_wrist_raw 直接画在 world 下,未应用 T_base_camera;只能看轴向/相对运动,不能当作已和机器人 base 对齐")
        log("坐标系调试:robot_ee_target=derive 映射后的 IK 目标;robot_ee_current=IK 解出来后 FK(link7) 的实际末端")
    elif args.debug_frames:
        log("坐标系调试:未找到 canonical wrist_pose,只画 robot base/current ee")

    # 人手骨架检测器(可选)
    detector = None
    if args.skeleton:
        try:
            from single_hand_detector import SingleHandDetector  # src/ 内
        except ImportError:
            import sys
            sys.path.insert(0, str(REPO / "src"))
            from single_hand_detector import SingleHandDetector
        detector = SingleHandDetector(hand_type="Right", selfie=False)

    if args.no_video:
        try:
            cap = SkeletonVideo.from_canonical(REPO / "src/out/canonical_ds")
            log("Human 面板使用 canonical hand skeleton")
        except Exception as e:
            log(f"canonical skeleton unavailable, fallback blank: {e}")
            cap = BlankVideo()
    else:
        cap = open_video(Path(args.video))
    vid_fps = cap.get(cv2.CAP_PROP_FPS)
    fps = vid_fps if vid_fps and vid_fps > 1 else args.fps   # 真实播放帧率(轨迹与视频帧 1:1)

    # ---- rerun 初始化 + 布局 ----
    rr.init("nero_inspire_replay")
    robot_roots = [f"world/{label}" for label, _ in trajs]
    bp = rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                # 视图标题用英文:Rerun 画布 egui 字体不含 CJK 字形,中文会缺字/方块。
                rrb.Spatial2DView(origin="human", name="Human · video + skeleton"),
                rrb.Spatial3DView(origin="world", name="Robot · NERO + inspire"),
                column_shares=[1.0, 1.4],
            ),
            rrb.TimeSeriesView(origin="joints", name="Joint angles (rad)"),
            row_shares=[3.0, 1.2],
        ),
        rrb.SelectionPanel(state="collapsed"),
        # timeline="frame":让查看器默认停在帧序号轴(0..N-1),而非 Rerun 自动加的 log_time
        # 墙钟轴。这样 web 端 time_update 直接给帧号,右侧读数才能映射联动。
        rrb.TimePanel(state="collapsed", timeline="frame"),
    )

    serve_uri = None
    if args.serve:
        ip = primary_ip()
        # serve_grpc 返回 rerun+http://127.0.0.1:port/proxy;127.0.0.1 从 Windows 连不到 WSL,
        # 换成 WSL 实际 IP,查看器(浏览器里)才能拿到数据。
        uri = rr.serve_grpc(grpc_port=args.grpc_port)
        serve_uri = uri.replace("127.0.0.1", ip).replace("0.0.0.0", ip)
        rr.serve_web_viewer(web_port=args.web_port, open_browser=False, connect_to=serve_uri)
    else:
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        rr.save(args.save)
    rr.send_blueprint(bp)

    # 静态:世界坐标系约定(Z 向上)+ 每条轨迹的网格(顶点在 link 局部系,逐帧只更新 Transform)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    if args.debug_frames:
        log_axes("world/frames/robot_base", np.eye(4), length=0.16, radius=0.006)
        rr.log("world/frames/notes",
               rr.TextLog("Frames: robot_base=URDF/base. robot_ee_target=derive IK target. robot_ee_current=Pinocchio link7 after IK. human_wrist_raw=canonical wrist pose without T_base_camera."),
               static=True)
    for root in robot_roots:
        for m in meshes:
            if m is None:
                continue
            rr.log(f"{root}/{m['name']}",
                   rr.Mesh3D(vertex_positions=m["V"], triangle_indices=m["F"],
                             vertex_normals=m["N"], albedo_factor=m["color"]),
                   static=True)

    log(f"开始记录 {F} 帧,{len(trajs)} 条轨迹" + (",带骨架" if detector else ""))
    for fr in range(F):
        rr.set_time("frame", sequence=fr)
        rr.set_time("time", duration=fr / fps)   # 真实时间轴 → 查看器按真实帧率平滑播放

        # --- 机器人:每条轨迹算 q -> 更新各 geom 的 Transform ---
        for (label, T), root in zip(trajs, robot_roots):
            q = model.make_q(T["arm"][fr], T["arm_names"], T["hand"][fr], T["hand_names"])
            placements = model.placements(q)
            for i, m in enumerate(meshes):
                if m is None:
                    continue
                M = placements[i]
                rr.log(f"{root}/{m['name']}",
                       rr.Transform3D(translation=M[:3, 3], mat3x3=M[:3, :3]))
            if args.debug_frames:
                ee_T = model.frame_placement(q, args.ee_frame)
                log_axes(f"{root}/frames/robot_ee_current_{args.ee_frame}", ee_T)
                if target_ee_poses is not None:
                    log_axes(f"{root}/frames/robot_ee_target_{args.ee_frame}",
                             target_ee_poses[fr], length=0.11, radius=0.003)
            # --- 关节角曲线(按轨迹分组,A/B 时同图叠看每个关节的 raw vs stab) ---
            for k, n in enumerate(T["arm_names"]):
                rr.log(f"joints/{label}/arm/{n}", rr.Scalars(float(T["arm"][fr][k])))
            for k, n in enumerate(T["hand_names"]):
                rr.log(f"joints/{label}/hand/{n}", rr.Scalars(float(T["hand"][fr][k])))

        if args.debug_frames and human_wrist_in_base is not None:
            # anchored:投到 base 系、与 target 同位置。和 robot_ee_target 逐帧一起转 = 坐标系自洽。
            log_axes("world/frames/human_wrist_in_base", human_wrist_in_base[fr], length=0.13, radius=0.004)
        elif args.debug_frames and wrist_poses is not None:
            log_axes("world/frames/human_wrist_raw", wrist_poses[fr], length=0.11, radius=0.004)

        # --- 人手视频帧(+骨架) ---
        ok, frame = cap.read()
        if ok:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            if detector is not None:
                num, _, keypoint_2d, wrist_rot = detector.detect(rgb)
                if num and keypoint_2d is not None:
                    rgb = SingleHandDetector.draw_skeleton_on_image(rgb.copy(), keypoint_2d)
                    if wrist_rot is not None:
                        kp2d_px = SingleHandDetector.parse_keypoint_2d(keypoint_2d, rgb.shape)
                        R_wrist = physical_wrist_orientation(wrist_rot, detector.operator2mano)
                        rgb = draw_wrist_axes_on_image(rgb, kp2d_px[0], R_wrist)
            # JPEG 编码后再 log,体积比原始 RGB 小 1~2 个数量级
            ok_enc, buf = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                       [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok_enc:
                rr.log("human", rr.EncodedImage(contents=buf.tobytes(), media_type="image/jpeg"))

    cap.release()

    if args.serve:
        from urllib.parse import quote
        ip = primary_ip()
        # 完整 URL:web 查看器 + ?url= 指向数据源(带 WSL IP,Windows 直连)
        full = f"http://{ip}:{args.web_port}/?url={quote(serve_uri, safe='')}"
        print("\n" + "=" * 72, flush=True)
        print("  Rerun 查看器已就绪。在 Windows 浏览器打开这个完整地址(带数据源):", flush=True)
        print(f"    {full}", flush=True)
        print("", flush=True)
        print(f"  数据源: {serve_uri}", flush=True)
        print("  (若只开 http://<ip>:端口 看到空欢迎页,就是漏了 ?url= 那段)", flush=True)
        print("  Ctrl-C 退出服务。", flush=True)
        print("=" * 72 + "\n", flush=True)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    else:
        log(f"已写入 {args.save}。用 Rerun 查看器打开它(Windows: `rerun {Path(args.save).name}` 或桌面版拖入)。")


if __name__ == "__main__":
    main()
