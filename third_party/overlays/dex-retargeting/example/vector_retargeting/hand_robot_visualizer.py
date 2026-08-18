"""hand_robot_visualizer.py -- 人手(视频/摄像头) + 重定向灵巧手 并排同步可视化。单进程。

三块视图,同一主循环、同一帧号产出 => 硬同步(时间戳不可能错开):
  - Human    : 人手画面 + MediaPipe 21 点骨架
  - Robot·mesh : 重定向后的灵巧手实体网格 (CPU=pyrender/EGL, 或 GPU=SAPIEN)
  - Robot·skeleton : 灵巧手火柴骨架 (Pinocchio FK 投影 + cv2)

三块可分别以独立可拖拽窗口显示(需 WSLg/X 显示环境),并/或 hconcat 合成到一个 mp4。

必须在 example/vector_retargeting/ 目录下运行(为了 `from single_hand_detector import ...`)。

用法见文件末尾 __main__ 附近的注释,或:
    python hand_robot_visualizer.py --help
"""
from __future__ import annotations

# EGL 必须在 import pyrender / OpenGL 之前选好(纯 CPU/WSLg 上走 EGL 软/硬件离屏)。
import os
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

import math
import subprocess
import sys
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import pinocchio as pin
import tyro
from PIL import Image, ImageDraw, ImageFont

from dex_retargeting.constants import (
    HandType,
    RetargetingType,
    RobotName,
    get_default_config_path,
)
from dex_retargeting.retargeting_config import RetargetingConfig
from hand_perception import HandObservation, available_detectors, make_detector


def log(msg: str) -> None:
    print(f"[viz] {msg}", flush=True)


# ---------------------------------------------------------------------------
# UI 工具箱:Pillow 绘制的高级感 chrome —— 抗锯齿字体、圆角、半透明、分段控件。
# 深色主题参考 macOS/iOS(中性灰 + 单一蓝色强调),追求"简单高级"。
# ---------------------------------------------------------------------------
def _first_existing(paths: List[str]) -> Optional[str]:
    for p in paths:
        if os.path.exists(p):
            return p
    return None


_UB = "/usr/share/fonts/truetype/ubuntu"
_DV = "/usr/share/fonts/truetype/dejavu"
_FONT_FILES = {
    "regular": _first_existing([f"{_UB}/Ubuntu-R.ttf", f"{_DV}/DejaVuSans.ttf"]),
    "medium": _first_existing([f"{_UB}/Ubuntu-M.ttf", f"{_DV}/DejaVuSans.ttf"]),
    "light": _first_existing([f"{_UB}/Ubuntu-L.ttf", f"{_DV}/DejaVuSans.ttf"]),
    "bold": _first_existing([f"{_UB}/Ubuntu-B.ttf", f"{_DV}/DejaVuSans-Bold.ttf"]),
}
_FONT_CACHE: Dict = {}


def ui_font(size: int, weight: str = "regular"):
    key = (size, weight)
    if key not in _FONT_CACHE:
        path = _FONT_FILES.get(weight) or _FONT_FILES["regular"]
        _FONT_CACHE[key] = ImageFont.truetype(path, size) if path else ImageFont.load_default()
    return _FONT_CACHE[key]


# 主题色(RGBA)
UI = dict(
    toolbar=(30, 30, 32, 214),
    hairline=(255, 255, 255, 20),
    track=(120, 120, 128, 64),
    seg_sel=(94, 94, 99, 255),
    seg_shadow=(0, 0, 0, 85),
    divider=(255, 255, 255, 30),
    text=(255, 255, 255, 255),
    text2=(235, 235, 245, 145),
    text3=(235, 235, 245, 92),
    chip=(0, 0, 0, 125),
    accent=(10, 132, 255, 255),
    live=(255, 69, 58, 255),
    canvas=(18, 18, 20),
)


def _seg_control(d, x, y, h, items, active, font):
    """iOS/macOS 风分段控件。返回 (右边界x, [每段矩形])。"""
    pad = 15
    widths = [int(round(d.textlength(t, font=font))) + 2 * pad for t in items]
    total = sum(widths)
    d.rounded_rectangle([x, y, x + total, y + h], radius=9, fill=UI["track"])
    rects, cx = [], x
    for i, (t, w) in enumerate(zip(items, widths)):
        if i == active:
            d.rounded_rectangle([cx + 2, y + 3, cx + w - 2, y + h + 1], radius=7, fill=UI["seg_shadow"])
            d.rounded_rectangle([cx + 2, y + 2, cx + w - 2, y + h - 2], radius=7, fill=UI["seg_sel"])
        elif i > 0 and (i - 1) != active:
            d.line([cx, y + 9, cx, y + h - 9], fill=UI["divider"], width=1)
        d.text((cx + w / 2, y + h / 2), t, font=font,
               fill=UI["text"] if i == active else UI["text2"], anchor="mm")
        rects.append((cx, y, cx + w, y + h))
        cx += w
    return x + total, rects


def _chip(d, x, y, text, font, fg=None, bg=None, h=26, pad=11, radius=9):
    fg = fg or UI["text"]
    bg = bg or UI["chip"]
    w = int(round(d.textlength(text, font=font))) + 2 * pad
    d.rounded_rectangle([x, y, x + w, y + h], radius=radius, fill=bg)
    d.text((x + pad, y + h / 2), text, font=font, fill=fg, anchor="lm")
    return w


# 每指颜色(BGR),按 link/joint 名子串匹配 —— 火柴骨架用。
FINGER_COLORS_BGR = {
    "thumb": (48, 59, 255),    # 红
    "index": (89, 199, 52),    # 绿
    "middle": (255, 132, 10),  # 蓝
    "ring": (10, 214, 255),    # 黄
    "pinky": (146, 45, 255),   # 洋红
}
DEFAULT_BONE_BGR = (200, 200, 200)


def finger_color(name: str) -> Tuple[int, int, int]:
    for key, col in FINGER_COLORS_BGR.items():
        if key in name:
            return col
    return DEFAULT_BONE_BGR


# ---------------------------------------------------------------------------
# 1) 输入源
# ---------------------------------------------------------------------------
class FrameSource(ABC):
    fps: float = 30.0
    is_live: bool = False

    @abstractmethod
    def read(self) -> Optional[np.ndarray]:
        """返回一帧 BGR;结束返回 None。"""

    def release(self) -> None:  # noqa: B027 - 可选
        pass

    def reset(self) -> None:  # noqa: B027 - 可选(从头播放)
        pass


class VideoFrameSource(FrameSource):
    is_live = False

    def __init__(self, path: str, loop: bool = False):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise FileNotFoundError(f"无法打开视频: {path}")
        self.path = path
        self.loop = loop
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.fps = fps if fps and fps > 1 else 30.0

    def read(self):
        ok, frame = self.cap.read()
        if not ok and self.loop:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        return frame if ok else None

    def reset(self):
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    def release(self):
        self.cap.release()


class WebcamFrameSource(FrameSource):
    is_live = True

    def __init__(self, device: int = 0, width: Optional[int] = None, height: Optional[int] = None):
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头: {device}")
        if width:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.fps = 30.0

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self):
        self.cap.release()


# ---------------------------------------------------------------------------
# 2) 相机(mesh 与 skeleton 共用,保证两个机器人视图同视角)
# ---------------------------------------------------------------------------
def look_at(eye: np.ndarray, target: np.ndarray, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """OpenGL/pyrender 约定:相机沿本地 -z 看向 target。返回 4x4 world<-cam 位姿。"""
    eye = np.asarray(eye, float)
    target = np.asarray(target, float)
    up = np.asarray(up, float)
    fwd = target - eye
    n = np.linalg.norm(fwd)
    fwd = fwd / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
    z = -fwd
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-8:
        up = np.array([0.0, 1.0, 0.0])
        x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    pose = np.eye(4)
    pose[:3, 0] = x
    pose[:3, 1] = y
    pose[:3, 2] = z
    pose[:3, 3] = eye
    return pose


class Camera:
    """针孔相机:持有 world<-cam 位姿 + 竖直 fov。供 pyrender 与手动投影共用。"""

    def __init__(self, pose: np.ndarray, yfov: float, width: int, height: int):
        self.pose = pose
        self.yfov = yfov
        self.width = width
        self.height = height
        # 方形像素:fx = fy,由竖直 fov 定。
        self.f = (height / 2.0) / math.tan(yfov / 2.0)

    def project(self, pts_world: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """世界坐标 (N,3) -> 像素 u,v 与可见掩码(点在相机前方)。"""
        pts_world = np.asarray(pts_world, float).reshape(-1, 3)
        tcw = np.linalg.inv(self.pose)  # cam<-world
        pc = (tcw[:3, :3] @ pts_world.T + tcw[:3, 3:4]).T  # (N,3) 相机系
        zc = -pc[:, 2]  # 相机看向 -z,前方点 zc>0
        valid = zc > 1e-4
        zc_safe = np.where(valid, zc, 1.0)
        u = self.width / 2.0 + self.f * (pc[:, 0] / zc_safe)
        v = self.height / 2.0 - self.f * (pc[:, 1] / zc_safe)
        return u, v, valid


# ---------------------------------------------------------------------------
# 3) 机器人运动学模型(Pinocchio)—— mesh + skeleton 共用
# ---------------------------------------------------------------------------
class RobotModel:
    """从 URDF 建 Pinocchio 模型 + 视觉几何,按 retargeting 关节名喂 live qpos。

    完全复刻 render_robot_hand_meshcat.py 的建模方式(固定基座、按关节名映射、
    连续关节展开成 cos/sin),只是把结果用于离屏渲染/投影而非 MeshCat。
    """

    def __init__(self, config_path: str, robot_dir: Path):
        RetargetingConfig.set_default_urdf_dir(str(robot_dir))
        config = RetargetingConfig.load_from_file(config_path)

        urdf_path = Path(config.urdf_path)
        if not urdf_path.is_absolute():
            urdf_path = (robot_dir / urdf_path).resolve()
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF 不存在: {urdf_path}")

        self.urdf_path = urdf_path
        package_dirs = [
            str(urdf_path.parent),
            str(robot_dir),
            str(robot_dir.parent),
            str(robot_dir.parent.parent),
        ]
        # 运动学模型用原始 urdf(关节结构一致、稳)。
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()

        # 视觉几何:优先 _glb(纹理更好),pinocchio 若载不动就回退原始 urdf(MeshCat 走的就是它)。
        glb_path = Path(str(urdf_path).replace(".urdf", "_glb.urdf"))
        candidates = []
        if "glb" not in urdf_path.stem and glb_path.exists():
            candidates.append(glb_path)
        candidates.append(urdf_path)
        self.visual_model = None
        self.visual_urdf = urdf_path
        for cand in candidates:
            try:
                self.visual_model = pin.buildGeomFromUrdf(
                    self.model, str(cand), pin.GeometryType.VISUAL, package_dirs=package_dirs
                )
                self.visual_urdf = cand
                break
            except Exception as e:
                log(f"视觉几何构建失败({cand.name}): {e}")
        if self.visual_model is None:
            raise RuntimeError("无法构建任何视觉几何模型")
        self.geom_data = self.visual_model.createData()

        self.name_to_q = {
            self.model.names[j]: (self.model.joints[j].idx_q, self.model.joints[j].nq)
            for j in range(1, self.model.njoints)
        }
        self.q0 = pin.neutral(self.model)
        log(f"URDF(运动学)={self.urdf_path.name} 视觉={self.visual_urdf.name} "
            f"nq={self.model.nq} 视觉几何={self.visual_model.ngeoms}")

    def make_q(self, qpos: np.ndarray, joint_names: List[str]) -> np.ndarray:
        qpos = np.asarray(qpos, float)
        q = self.q0.copy()
        for i, name in enumerate(joint_names):
            slot = self.name_to_q.get(name)
            if slot is None:
                continue
            idx_q, nq = slot
            if nq == 1:
                q[idx_q] = qpos[i]
            elif nq == 2:  # 连续关节:存成 (cos, sin)
                q[idx_q] = math.cos(qpos[i])
                q[idx_q + 1] = math.sin(qpos[i])
        return q

    def forward(self, q: np.ndarray) -> None:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def geometry_placements(self, q: np.ndarray) -> List[np.ndarray]:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(self.model, self.data, self.visual_model, self.geom_data, q)
        return [self.geom_data.oMg[i].homogeneous for i in range(self.visual_model.ngeoms)]

    def bounding_sphere(self) -> Tuple[np.ndarray, float]:
        """neutral 位姿下所有 frame 的包围球(用于自动取景)。"""
        self.forward(self.q0)
        pts = np.array([self.data.oMf[f].translation for f in range(len(self.model.frames))])
        center = pts.mean(axis=0)
        radius = float(np.linalg.norm(pts - center, axis=1).max())
        return center, max(radius, 1e-3)

    def skeleton_segments(self, q: np.ndarray) -> Tuple[List[Tuple[np.ndarray, np.ndarray, Tuple[int, int, int]]],
                                                        List[Tuple[np.ndarray, Tuple[int, int, int]]]]:
        """按运动学树给出骨架线段与关节点(世界坐标 + 颜色)。"""
        self.forward(q)
        joint_pos = [self.data.oMi[j].translation for j in range(self.model.njoints)]
        segments = []
        for j in range(1, self.model.njoints):
            parent = self.model.parents[j]
            col = finger_color(self.model.names[j])
            segments.append((joint_pos[parent], joint_pos[j], col))
        # 指尖 frame(*_tip)接到其父关节
        nodes = [(joint_pos[j], finger_color(self.model.names[j])) for j in range(1, self.model.njoints)]
        for fid, fr in enumerate(self.model.frames):
            if fr.name.endswith("_tip"):
                tip = self.data.oMf[fid].translation
                col = finger_color(fr.name)
                segments.append((joint_pos[fr.parentJoint], tip, col))
                nodes.append((tip, col))
        return segments, nodes


# ---------------------------------------------------------------------------
# 4) 渲染器抽象 + 三种实现
# ---------------------------------------------------------------------------
class RobotRenderer(ABC):
    width: int = 512
    height: int = 512

    @abstractmethod
    def render(self, qpos: np.ndarray) -> np.ndarray:
        """返回 BGR uint8 图。"""

    def close(self) -> None:  # noqa: B027
        pass


class PyrenderMeshRenderer(RobotRenderer):
    """CPU 离屏网格渲染(EGL)。本机默认后端。"""

    def __init__(self, model: RobotModel, camera: Camera, supersample: int = 2):
        import pyrender  # 延迟导入,避免 sapien 路径也初始化 EGL
        import trimesh

        self.model = model
        self.camera = camera
        self.width = camera.width
        self.height = camera.height
        # 超采样:先渲 ss 倍分辨率再 INTER_AREA 缩小 => 抗锯齿 + 更锐(补 pyrender 无 MSAA)。
        self.ss = max(1, int(supersample))
        self.rw = self.width * self.ss
        self.rh = self.height * self.ss

        self.scene = pyrender.Scene(bg_color=[0.10, 0.10, 0.12, 1.0], ambient_light=[0.25, 0.25, 0.25])
        # 仅当网格自带材质缺失时才用兜底材质;有材质的保留 GLB 原材质。
        fallback = pyrender.MetallicRoughnessMaterial(
            baseColorFactor=[0.80, 0.82, 0.86, 1.0], metallicFactor=0.25, roughnessFactor=0.55)
        self.nodes: List[Tuple[int, object]] = []
        for i in range(model.visual_model.ngeoms):
            gobj = model.visual_model.geometryObjects[i]
            mesh_path = gobj.meshPath
            if not mesh_path or not os.path.exists(mesh_path):
                continue
            try:
                loaded = trimesh.load(mesh_path, process=False)
            except Exception as e:  # 单个网格失败不致命
                log(f"网格加载失败({os.path.basename(mesh_path)}): {e}")
                continue
            # 保留原材质:多几何体用 dump 把内部变换烘焙进去(别 force='mesh',那样会丢材质)。
            geoms = loaded.dump(concatenate=False) if isinstance(loaded, trimesh.Scene) else [loaded]
            scale = np.asarray(gobj.meshScale).reshape(3)
            for g in geoms:
                if not np.allclose(scale, 1.0):
                    g.apply_scale(scale)
                has_material = getattr(getattr(g, "visual", None), "kind", None) in ("texture", "vertex")
                prm = pyrender.Mesh.from_trimesh(g, smooth=True,
                                                 material=None if has_material else fallback)
                self.nodes.append((i, self.scene.add(prm, pose=np.eye(4))))

        if not self.nodes:
            raise RuntimeError("没有可渲染的视觉网格")

        # 相机 + 灯光节点句柄留着,每帧按 camera.pose 更新(支持鼠标 orbit)。
        self.cam_node = self.scene.add(
            pyrender.PerspectiveCamera(yfov=camera.yfov, aspectRatio=self.width / self.height),
            pose=camera.pose)
        self.key_light = self.scene.add(pyrender.DirectionalLight(color=[1, 1, 1], intensity=5.0),
                                        pose=camera.pose)
        self.fill_lights = []  # (node, 相对相机的偏移)
        for off in ([0.4, 0.4, 0.4], [-0.4, 0.3, 0.3]):
            node = self.scene.add(pyrender.DirectionalLight(color=[1, 1, 1], intensity=2.5), pose=camera.pose)
            self.fill_lights.append((node, np.asarray(off, float)))

        self.renderer = pyrender.OffscreenRenderer(self.rw, self.rh)
        log(f"PyrenderMeshRenderer 就绪:{len(self.nodes)} 网格 @ {self.width}x{self.height} "
            f"(渲染 {self.rw}x{self.rh}, {self.ss}x SSAA, EGL)")

    def _update_camera(self) -> None:
        pose = self.camera.pose
        self.scene.set_pose(self.cam_node, pose)
        self.scene.set_pose(self.key_light, pose)
        for node, off in self.fill_lights:
            p = pose.copy()
            p[:3, 3] = pose[:3, 3] + off
            self.scene.set_pose(node, p)

    def render(self, qpos: np.ndarray) -> np.ndarray:
        self._update_camera()  # 跟随 orbit
        q = self.model.make_q(qpos, self._joint_names)
        placements = self.model.geometry_placements(q)
        for i, node in self.nodes:
            self.scene.set_pose(node, placements[i])
        color, _ = self.renderer.render(self.scene)  # RGB uint8, rw x rh
        bgr = color[..., ::-1]
        if self.ss != 1:  # 高分辨率 -> 目标尺寸,INTER_AREA 即抗锯齿降采样
            bgr = cv2.resize(bgr, (self.width, self.height), interpolation=cv2.INTER_AREA)
        return np.ascontiguousarray(bgr)

    def bind_joint_names(self, joint_names: List[str]) -> None:
        self._joint_names = joint_names

    def close(self) -> None:
        try:
            self.renderer.delete()
        except Exception:
            pass


class SkeletonRenderer(RobotRenderer):
    """零 GL 火柴骨架:Pinocchio FK -> 与 mesh 同相机投影 -> cv2 画线。永远可用。"""

    def __init__(self, model: RobotModel, camera: Camera):
        self.model = model
        self.camera = camera
        self.width = camera.width
        self.height = camera.height

    def bind_joint_names(self, joint_names: List[str]) -> None:
        self._joint_names = joint_names

    def render(self, qpos: np.ndarray) -> np.ndarray:
        q = self.model.make_q(qpos, self._joint_names)
        segments, nodes = self.model.skeleton_segments(q)
        canvas = np.full((self.height, self.width, 3), (26, 22, 18), np.uint8)

        # 批量投影所有端点
        pts = []
        for a, b, _ in segments:
            pts.append(a)
            pts.append(b)
        for p, _ in nodes:
            pts.append(p)
        if not pts:
            return canvas
        u, v, valid = self.camera.project(np.array(pts))

        k = 0
        for a, b, col in segments:
            ua, va, oka = u[k], v[k], valid[k]
            ub, vb, okb = u[k + 1], v[k + 1], valid[k + 1]
            k += 2
            if oka and okb:
                cv2.line(canvas, (int(ua), int(va)), (int(ub), int(vb)), col, 2, cv2.LINE_AA)
        for _p, col in nodes:
            uu, vv, ok = u[k], v[k], valid[k]
            k += 1
            if ok:
                cv2.circle(canvas, (int(uu), int(vv)), 4, col, -1, cv2.LINE_AA)
        return canvas


# SAPIEN 探测子进程代码(auto 时用,隔离段错误)
_SAPIEN_PROBE = (
    "import sys\n"
    "try:\n"
    "    import sapien, numpy as np\n"
    "    sapien.render.set_camera_shader_dir('default')\n"
    "    sc = sapien.Scene()\n"
    "    cam = sc.add_camera('p', 64, 64, 1.0, 0.1, 10)\n"
    "    sc.update_render(); cam.take_picture(); cam.get_picture('Color')\n"
    "    print('SAPIEN_OK')\n"
    "except Exception as e:\n"
    "    print('SAPIEN_FAIL', repr(e)); sys.exit(1)\n"
)


def sapien_available() -> bool:
    """子进程里试渲染一帧;段错误只崩子进程,不影响本工具。"""
    try:
        r = subprocess.run([sys.executable, "-c", _SAPIEN_PROBE],
                           capture_output=True, text=True, timeout=90)
    except Exception as e:
        log(f"SAPIEN 探测异常: {e}")
        return False
    ok = r.returncode == 0 and "SAPIEN_OK" in r.stdout
    log(f"SAPIEN 探测: {'可用' if ok else '不可用'} (rc={r.returncode})")
    return ok


class SapienMeshRenderer(RobotRenderer):
    """GPU 离屏网格渲染(SAPIEN,光栅化)。为带可用 Vulkan 的机器准备。

    ⚠ 未在本机(CPU-only WSL, 无可用 Vulkan)验证。配方照搬 render_robot_hand.py,
    但用 default 光栅着色器 + 纯离屏相机、不开 viewer(避开 headless->光追 的坑)。
    """

    _SCALE = {"ability": 1.5, "dclaw": 1.25, "allegro": 1.4, "shadow": 0.9,
              "bhand": 1.5, "leap": 1.4, "svh": 1.5}
    _BASE_Z = {"ability": -0.15, "shadow": -0.2, "dclaw": -0.15, "allegro": -0.05,
               "bhand": -0.2, "leap": -0.15, "svh": -0.13, "inspire": -0.15}

    def __init__(self, config_path: str, robot_dir: Path, width: int = 512, height: int = 512):
        import sapien
        from sapien.asset import create_dome_envmap

        self.width = width
        self.height = height
        RetargetingConfig.set_default_urdf_dir(str(robot_dir))
        config = RetargetingConfig.load_from_file(config_path)

        sapien.render.set_viewer_shader_dir("default")
        sapien.render.set_camera_shader_dir("default")  # 光栅化,不用 "rt"
        self.scene = sapien.Scene()

        mat = sapien.render.RenderMaterial()
        mat.base_color = [0.06, 0.08, 0.12, 1]
        mat.metallic = 0.0
        mat.roughness = 0.9
        mat.specular = 0.8
        self.scene.add_ground(-0.2, render_material=mat, render_half_size=[1000, 1000])
        self.scene.add_directional_light(np.array([1, 1, -1]), np.array([3, 3, 3]))
        self.scene.add_point_light(np.array([2, 2, 2]), np.array([2, 2, 2]), shadow=False)
        self.scene.add_point_light(np.array([2, -2, 2]), np.array([2, 2, 2]), shadow=False)
        self.scene.set_environment_map(create_dome_envmap(sky_color=[0.2] * 3, ground_color=[0.2] * 3))

        self.cam = self.scene.add_camera("cam", width, height, 1.0, 0.1, 10)
        self.cam.set_local_pose(sapien.Pose([0.50, 0, 0.0], [0, 0, 0, -1]))

        loader = self.scene.create_urdf_loader()
        loader.load_multiple_collisions_from_file = True
        fp = Path(config.urdf_path)
        robot_name = fp.stem
        key = self._key(robot_name)
        loader.scale = self._SCALE.get(key, 1.0)
        glb = str(fp) if "glb" in robot_name else str(fp).replace(".urdf", "_glb.urdf")
        self.robot = loader.load(glb)
        if key in self._BASE_Z:
            self.robot.set_pose(sapien.Pose([0, 0, self._BASE_Z[key]]))
        self.scene.update_render()
        self._to_sapien: Optional[np.ndarray] = None
        log(f"SapienMeshRenderer 就绪(未在本机验证):{robot_name} scale={loader.scale}")

    @staticmethod
    def _key(name: str) -> str:
        for k in ("ability", "dclaw", "allegro", "shadow", "bhand", "leap", "svh", "inspire"):
            if k in name:
                return k
        return name

    def bind_joint_names(self, joint_names: List[str]) -> None:
        sj = [j.get_name() for j in self.robot.get_active_joints()]
        self._to_sapien = np.array([joint_names.index(n) for n in sj]).astype(int)

    def render(self, qpos: np.ndarray) -> np.ndarray:
        self.robot.set_qpos(np.asarray(qpos)[self._to_sapien])
        self.scene.update_render()
        self.cam.take_picture()
        rgb = self.cam.get_picture("Color")[..., :3]
        bgr = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)[..., ::-1]
        return np.ascontiguousarray(bgr)

    def close(self) -> None:
        self.scene = None


# ---------------------------------------------------------------------------
# 5) 辅助
# ---------------------------------------------------------------------------
def compute_ref_value(retargeting, joint_pos: np.ndarray) -> np.ndarray:
    """照抄 detect_from_video.py 的分支逻辑。"""
    rtype = retargeting.optimizer.retargeting_type
    indices = retargeting.optimizer.target_link_human_indices
    if rtype == "POSITION":
        return joint_pos[indices, :]
    origin_indices = indices[0, :]
    task_indices = indices[1, :]
    return joint_pos[task_indices, :] - joint_pos[origin_indices, :]


def put_label(img: np.ndarray, text: str, org=(10, 28), color=(255, 255, 255), scale=0.7) -> None:
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def fit_height(img: np.ndarray, h: int) -> np.ndarray:
    if img.shape[0] == h:
        return img
    w = int(round(img.shape[1] * h / img.shape[0]))
    interp = cv2.INTER_AREA if h < img.shape[0] else cv2.INTER_CUBIC  # 缩小用 AREA 更清晰
    return cv2.resize(img, (w, h), interpolation=interp)


def placeholder(width: int, height: int, text: str) -> np.ndarray:
    img = Image.new("RGB", (width, height), UI["canvas"])
    d = ImageDraw.Draw(img)
    d.text((width / 2, height / 2), text, font=ui_font(19, "light"),
           fill=(150, 150, 156), anchor="mm")
    return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class OrbitController:
    """转盘式相机:绕 center 的方位角/俯仰角/距离。鼠标拖拽改角度,滚轮改距离。"""

    def __init__(self, center, azim: float, elev: float, dist: float):
        self.center = np.asarray(center, float)
        self.azim = azim
        self.elev = elev
        self.dist = dist
        self._init = (azim, elev, dist)
        self.dmin = dist * 0.25
        self.dmax = dist * 4.0

    def eye(self) -> np.ndarray:
        ce = math.cos(self.elev)
        return self.center + self.dist * np.array(
            [ce * math.cos(self.azim), ce * math.sin(self.azim), math.sin(self.elev)])

    def pose(self) -> np.ndarray:
        return look_at(self.eye(), self.center)

    def drag(self, dx: float, dy: float) -> None:
        self.azim -= dx * 0.008
        self.elev = clamp(self.elev + dy * 0.008, math.radians(-85), math.radians(85))

    def zoom(self, steps: float) -> None:
        self.dist = clamp(self.dist * (0.88 ** steps), self.dmin, self.dmax)

    def reset(self) -> None:
        self.azim, self.elev, self.dist = self._init


def ask_open_file(initial_dir: str) -> Optional[str]:
    """弹系统文件对话框选视频;跑在独立子进程(zenity 优先, 退回 tkinter),避免与 cv2 的 Qt 冲突。"""
    import shutil

    if shutil.which("zenity"):
        try:
            r = subprocess.run(
                ["zenity", "--file-selection", "--title=select hand video",
                 f"--filename={initial_dir}/",
                 "--file-filter=Video | *.mp4 *.avi *.mov *.mkv *.webm *.MP4 *.MOV *.MKV",
                 "--file-filter=All files | *"],
                capture_output=True, text=True, timeout=600)
            return r.stdout.strip() or None
        except Exception as e:
            log(f"zenity 打开失败,改用 tkinter: {e}")

    code = (
        "import tkinter as tk\n"
        "from tkinter import filedialog as fd\n"
        "import sys\n"
        "r = tk.Tk(); r.withdraw(); r.update()\n"
        "p = fd.askopenfilename(title='select hand video',"
        " filetypes=[('Video','*.mp4 *.avi *.mov *.mkv *.webm'), ('All files','*.*')])\n"
        "sys.stdout.write(p or '')\n"
    )
    try:
        r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=600)
        return r.stdout.strip() or None
    except Exception as e:
        log(f"tkinter 打开失败: {e}")
        return None


# ---------------------------------------------------------------------------
# 6) 主程序
# ---------------------------------------------------------------------------
def _build_scene(robot_name, retargeting_type, hand_type, render_backend,
                 show_mesh, show_skeleton, panel_size, frame_margin, supersample=2,
                 retarget_config=""):
    """建 retargeting + model + camera + orbit + 渲染器。返回句柄字典。

    retarget_config: 逗号分隔的额外配置,用来在界面里和默认配置(A)对比切换。
      每项可写 `标签=路径` 或直接给路径;只给文件名(词干)时会在默认配置目录里找 <名>.yml。
      例:--retarget-config "tip+PIP=inspire_hand_right_tip_pip"
    """
    here = Path(__file__).absolute().parent
    robot_dir = here.parent.parent / "assets" / "robots" / "hands"
    RetargetingConfig.set_default_urdf_dir(str(robot_dir))

    default_cfg = str(get_default_config_path(robot_name, retargeting_type, hand_type))
    cfg_dir = Path(default_cfg).parent

    def _resolve(path: str) -> str:
        p = Path(path)
        if p.exists():
            return str(p)
        return str(cfg_dir / (p.name if p.suffix else p.name + ".yml"))  # 词干 -> 默认目录

    # 默认配置(A)排第一;--retarget-config 里的作为可切换的备选(B、C…)
    cfg_specs = [("default", default_cfg)]
    for item in (s.strip() for s in retarget_config.split(",")):
        if not item:
            continue
        label, sep, path = item.partition("=")
        if not sep:
            label, path = Path(item).stem, item
        cfg_specs.append((label, _resolve(path)))

    retargetings, cfg_path, joint_names = [], None, None  # cfg_path/joint_names 取首个有效配置
    for label, path in cfg_specs:
        if not Path(path).exists():
            log(f"跳过重定向配置(找不到文件): {path}")
            continue
        rt = RetargetingConfig.load_from_file(path).build()
        jn = list(rt.joint_names)
        if joint_names is None:
            joint_names, cfg_path = jn, path
        elif jn != joint_names:
            log(f"跳过配置 '{label}': 关节与默认不一致(不同机器人?),无法共用同一渲染器")
            continue
        retargetings.append((label, rt))
    if not retargetings:
        raise SystemExit("没有可用的重定向配置")
    retargeting = retargetings[0][1]
    if len(retargetings) > 1:
        log("可切换重定向配置(按 T): " + ", ".join(lbl for lbl, _ in retargetings))

    model = RobotModel(cfg_path, robot_dir)
    center, radius = model.bounding_sphere()
    yfov = 1.0
    dist = radius / math.tan(yfov / 2.0) * frame_margin
    # 初始视角 ≈ 之前验证过的方向([1,0.25,0.35])
    orbit = OrbitController(center, azim=math.radians(14), elev=math.radians(19), dist=dist)
    camera = Camera(orbit.pose(), yfov, panel_size, panel_size)

    mesh_renderer: Optional[RobotRenderer] = None
    if show_mesh:
        backend = render_backend
        if backend == "auto":
            backend = "sapien" if sapien_available() else "cpu"
        if backend == "sapien":
            try:
                mesh_renderer = SapienMeshRenderer(cfg_path, robot_dir, panel_size, panel_size)
            except Exception as e:
                log(f"SAPIEN 后端初始化失败,回退 pyrender: {e}")
        if mesh_renderer is None:
            try:
                mesh_renderer = PyrenderMeshRenderer(model, camera, supersample=supersample)
            except Exception as e:
                log(f"pyrender 网格渲染不可用,mesh 视图降级为骨架: {e}")
                mesh_renderer = None
        if mesh_renderer is not None:
            mesh_renderer.bind_joint_names(joint_names)

    skel_renderer: Optional[SkeletonRenderer] = None
    if show_skeleton or (show_mesh and mesh_renderer is None):
        skel_renderer = SkeletonRenderer(model, camera)
        skel_renderer.bind_joint_names(joint_names)

    return dict(cfg_path=cfg_path, retargeting=retargeting, retargetings=retargetings,
                joint_names=joint_names,
                model=model, camera=camera, orbit=orbit,
                mesh_renderer=mesh_renderer, skel_renderer=skel_renderer)


def _labels_for(view_order, mesh_renderer, show_mesh, source_type):
    mesh_degraded = show_mesh and mesh_renderer is None
    return {
        "human": f"Human ({source_type})",
        "mesh": ("Robot mesh [skeleton fallback]" if mesh_degraded
                 else f"Robot mesh ({'sapien' if isinstance(mesh_renderer, SapienMeshRenderer) else 'pyrender'})"),
        "skeleton": "Robot skeleton",
    }


class VizApp:
    """交互式单窗口:合成画面 + 鼠标 orbit(拖拽转/滚轮缩放) + 打开视频/切摄像头(按钮或 O、C 键)。"""

    WIN = "Hand Retargeting"
    TOOLBAR_H = 54

    def __init__(self, *, source, source_type, detector, retargetings, joint_names,
                 mesh_renderer, skel_renderer, camera, orbit, view_order, labels,
                 draw_human_skeleton, panel_size, view_height, initial_dir, camera_id=0):
        self.source = source
        self.source_type = source_type
        self.fps = source.fps
        self.detector = detector
        self.retargetings = retargetings          # [(label, SeqRetargeting)]; 共用同一机器人/渲染器
        self.active_cfg = 0
        self.retargeting = retargetings[0][1]
        self.cur_qpos_by_cfg = {}                  # 每帧对所有配置各算一次,切换时零延迟
        self.joint_names = joint_names
        self.mesh_renderer = mesh_renderer
        self.skel_renderer = skel_renderer
        self.camera = camera
        self.orbit = orbit
        self.view_order = view_order
        self.labels = labels
        self.draw_human_skeleton = draw_human_skeleton
        self.panel_size = panel_size
        self.view_height = view_height
        self.initial_dir = initial_dir
        self.camera_id = camera_id

        self.playing = True
        self.ended = False
        self.need_render = True
        self.cur_bgr = None
        self.cur_qpos = None
        self.cur_obs: Optional[HandObservation] = None
        self.num_box = 0
        self.idx = 0
        self.fps_disp = 0.0
        self._t_prev = time.time()
        self._last_advance = 0.0
        self._drag = None
        self._down = None
        self.layout = []                       # [(kind, x0, x1)] 在合成画布中的横向范围
        self._hit = []                         # [((x0,y0,x1,y1), action)] 顶栏可点区域
        self._ov_sig = None                    # chrome 缓存签名(状态没变就不重画)
        self.open_request = False
        self.cam_request = False

    # ---- 数据推进 ----
    def _advance(self) -> bool:
        frame = self.source.read()
        if frame is None:
            return False
        self.idx += 1
        self.cur_bgr = frame
        obs = self.detector.detect(frame)          # 统一接口:输入 BGR,输出 HandObservation
        self.cur_obs = obs
        self.num_box = obs.num_hands
        if obs.found:
            # 对每个配置各算一次 qpos(保持各自滤波器热身),渲染当前激活的那个
            self.cur_qpos_by_cfg = {
                label: rt.retarget(compute_ref_value(rt, obs.joint_pos))
                for label, rt in self.retargetings
            }
            self.cur_qpos = self.cur_qpos_by_cfg[self.retargetings[self.active_cfg][0]]
        return True

    def _robot(self, kind: str) -> np.ndarray:
        if kind == "mesh" and self.mesh_renderer is not None:
            return self.mesh_renderer.render(self.cur_qpos)
        return self.skel_renderer.render(self.cur_qpos)

    # ---- 合成一帧 ----
    def _compose(self) -> np.ndarray:
        self.camera.pose = self.orbit.pose()   # 跟随 orbit;mesh/skeleton 共用
        raw = {}
        if "human" in self.view_order:
            if self.cur_bgr is not None:
                h = self.cur_bgr.copy()
                if self.draw_human_skeleton and self.cur_obs is not None:
                    self.detector.draw(h, self.cur_obs)
            else:
                h = placeholder(self.panel_size, self.panel_size, "no video")
            raw["human"] = h
        for kind in ("mesh", "skeleton"):
            if kind not in self.view_order:
                continue
            raw[kind] = (placeholder(self.panel_size, self.panel_size, "waiting for hand...")
                         if self.cur_qpos is None else self._robot(kind))

        fitted, self.layout, x = [], [], 0
        for kind in self.view_order:
            im = fit_height(raw[kind], self.view_height)
            self.layout.append((kind, x, x + im.shape[1]))
            x += im.shape[1]
            fitted.append(im)
        return self._draw_chrome(np.hstack(fitted))

    # ---- 高级感 chrome:静态部分烘焙缓存,每帧只合成几条窄带 + 小 HUD(省 CPU) ----
    def _panel_title(self, kind: str) -> str:
        if kind == "human":
            return "Human · " + ("Camera" if self.source_type == "camera" else "Video")
        if kind == "mesh":
            return "Robot" if self.mesh_renderer is not None else "Robot · Skeleton"
        return "Skeleton"

    def _chrome_sig(self, W, Hc):
        return (W, Hc, self.source_type, self.active_cfg, self.playing,
                tuple(x1 for _, _, x1 in self.layout))

    def _build_chrome(self, W, Hc):
        """把不随帧变化的 chrome(顶栏/分段控件/标签/提示/暂停)烘焙成一张 RGBA 缓存。"""
        H = self.TOOLBAR_H
        ov = Image.new("RGBA", (W, Hc), (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)
        f_seg, f_lbl = ui_font(15, "medium"), ui_font(15, "medium")
        f_cap, f_hint = ui_font(12, "regular"), ui_font(13, "regular")

        for kind, x0, _ in self.layout:                       # 面板标签 chip
            _chip(d, x0 + 14, H + 12, self._panel_title(kind), f_lbl)

        d.rectangle([0, 0, W, H], fill=UI["toolbar"])          # 顶栏
        d.line([0, H, W, H], fill=UI["hairline"], width=1)
        seg_h, cy = 30, (H - 30) // 2
        self._hit = []

        def caption(x, text):                                  # 控件前的小标题 -> 一看就是可点控件
            d.text((x, H / 2), text, font=f_cap, fill=UI["text2"], anchor="lm")
            return x + int(round(d.textlength(text, font=f_cap))) + 9

        x = caption(16, "Source")
        endx, rs = _seg_control(d, x, cy, seg_h, ["Video", "Camera"],
                                0 if self.source_type == "video" else 1, f_seg)
        self._hit += [(rs[0], "open_video"), (rs[1], "camera")]
        x = endx + 22
        if len(self.retargetings) > 1:
            x = caption(x, "Retarget")
            labels = [lbl for lbl, _ in self.retargetings]
            endx, rc = _seg_control(d, x, cy, seg_h, labels, self.active_cfg, f_seg)
            self._hit += [(r, ("cfg", i)) for i, r in enumerate(rc)]
            x = endx + 22
        if not self.playing:                                   # 暂停指示(固定位,不与 HUD 争地方)
            pw = int(round(d.textlength("Paused", font=f_cap))) + 22
            d.rounded_rectangle([x, cy + 2, x + pw, cy + seg_h - 2], radius=8, fill=(120, 120, 128, 70))
            d.text((x + pw / 2, H / 2), "Paused", font=f_cap, fill=UI["text"], anchor="mm")

        hint = "drag to orbit      scroll to zoom      space to play / pause      Q to quit"
        hw = int(round(d.textlength(hint, font=f_hint))) + 24
        d.rounded_rectangle([14, Hc - 36, 14 + hw, Hc - 10], radius=9, fill=(0, 0, 0, 90))
        d.text((26, Hc - 23), hint, font=f_hint, fill=(235, 235, 245, 175), anchor="lm")

        rgba = np.asarray(ov)
        self._ov_rgb = rgba[:, :, :3].astype(np.float32)
        self._ov_a = rgba[:, :, 3:4].astype(np.float32) / 255.0
        self._ov_bands = [(0, H), (H + 12, min(H + 42, Hc)), (max(Hc - 42, 0), Hc)]
        self._ov_sig = self._chrome_sig(W, Hc)

    def _draw_hud(self, canvas, W):
        H = self.TOOLBAR_H
        hands = self.num_box
        hud = f"{self.fps_disp:.0f} FPS      {self.idx:05d}      {hands} hand" + ("" if hands == 1 else "s")
        strip = Image.new("RGBA", (380, H), (0, 0, 0, 0))
        ImageDraw.Draw(strip).text((380 - 16, H / 2), hud, font=ui_font(14, "regular"),
                                   fill=UI["text2"], anchor="rm")
        arr = np.asarray(strip)
        x0 = max(W - 380, 0)
        n = W - x0
        a = arr[:, :n, 3:4].astype(np.float32) / 255.0
        reg = canvas[0:H, x0:W].astype(np.float32)
        canvas[0:H, x0:W] = (reg * (1 - a) + arr[:, :n, :3].astype(np.float32) * a).astype(np.uint8)

    def _draw_chrome(self, canvas: np.ndarray) -> np.ndarray:
        Hc, W = canvas.shape[0], canvas.shape[1]
        if self._ov_sig != self._chrome_sig(W, Hc):
            self._build_chrome(W, Hc)
        for r0, r1 in self._ov_bands:                          # 只合成几条窄带(顶栏/标签/提示)
            if r1 > r0:
                a = self._ov_a[r0:r1]
                canvas[r0:r1] = (canvas[r0:r1].astype(np.float32) * (1.0 - a)
                                 + self._ov_rgb[r0:r1] * a).astype(np.uint8)
        for _, _, x1 in self.layout[:-1]:                      # 面板分隔发丝线(便宜的单列混合)
            if 0 < x1 < W:
                col = canvas[self.TOOLBAR_H:, x1:x1 + 1].astype(np.float32)
                canvas[self.TOOLBAR_H:, x1:x1 + 1] = (col * 0.85 + 34.0).astype(np.uint8)
        self._draw_hud(canvas, W)                              # 每帧变化的 HUD(小区域)
        return canvas

    def _over_robot(self, x: int) -> bool:
        for kind, a, b in self.layout:
            if a <= x < b:
                return kind in ("mesh", "skeleton")
        return False

    # ---- 鼠标 ----
    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._down = (x, y)
            self._drag = (x, y)
            self._orbiting = (y > self.TOOLBAR_H and self._over_robot(x))
        elif event == cv2.EVENT_MOUSEMOVE and (flags & cv2.EVENT_FLAG_LBUTTON) and self._drag is not None:
            dx, dy = x - self._drag[0], y - self._drag[1]
            self._drag = (x, y)
            if getattr(self, "_orbiting", False):
                self.orbit.drag(dx, dy)
                self.need_render = True
        elif event == cv2.EVENT_LBUTTONUP:
            if self._down is not None and abs(x - self._down[0]) < 4 and abs(y - self._down[1]) < 4:
                for (rx0, ry0, rx1, ry1), action in self._hit:   # 点到顶栏分段控件
                    if rx0 <= x <= rx1 and ry0 <= y <= ry1:
                        self._handle_action(action)
                        break
            self._down = self._drag = None
            self._orbiting = False
        elif event == cv2.EVENT_MOUSEWHEEL:
            self.orbit.zoom(cv2.getMouseWheelDelta(flags) / 120.0)
            self.need_render = True

    def _handle_action(self, action):
        if action == "open_video":
            self.open_request = True
        elif action == "camera":
            self.cam_request = True
        elif isinstance(action, tuple) and action[0] == "cfg":
            self._set_cfg(action[1])

    # ---- 打开 / 重启 ----
    def _open_dialog(self):
        path = ask_open_file(self.initial_dir)
        if not path or not os.path.exists(path):
            return
        try:
            newsrc = VideoFrameSource(path, loop=getattr(self.source, "loop", False))
        except Exception as e:
            log(f"打开视频失败: {e}")
            return
        try:
            self.source.release()
        except Exception:
            pass
        self.source = newsrc
        self.source_type = "video"
        self.fps = newsrc.fps
        self.initial_dir = os.path.dirname(path) or self.initial_dir
        self.labels["human"] = "Human (video)"
        self.idx = 0
        self.cur_qpos = self.cur_bgr = self.cur_obs = None
        self.ended = False
        self.playing = True
        log(f"已加载: {path}")

    def _open_camera(self):
        try:
            newsrc = WebcamFrameSource(self.camera_id)
        except Exception as e:
            log(f"打开摄像头失败: {e}")
            return
        try:
            self.source.release()
        except Exception:
            pass
        self.source = newsrc
        self.source_type = "camera"
        self.fps = newsrc.fps
        self.labels["human"] = "Human (camera)"
        self.idx = 0
        self.cur_qpos = self.cur_bgr = self.cur_obs = None
        self.ended = False
        self.playing = True
        log(f"已切到摄像头 {self.camera_id}")

    def _restart(self):
        try:
            self.source.reset()
        except Exception:
            pass
        self.idx = 0
        self.ended = False
        self.playing = True

    def _set_cfg(self, i: int):
        """切换到第 i 个重定向配置(A/B…)。各配置每帧都算好了,切换零延迟。"""
        if i == self.active_cfg or not (0 <= i < len(self.retargetings)):
            return
        self.active_cfg = i
        label = self.retargetings[i][0]
        self.retargeting = self.retargetings[i][1]
        if label in self.cur_qpos_by_cfg:
            self.cur_qpos = self.cur_qpos_by_cfg[label]
        self.need_render = True
        log(f"重定向配置 -> {label}")

    # ---- 主循环 ----
    def run(self):
        cv2.namedWindow(self.WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WIN, min(1600, self.view_height * 3 + 200), self.view_height + 60)
        cv2.setMouseCallback(self.WIN, self.on_mouse)
        try:
            while True:
                now = time.time()
                advanced = False
                if self.playing and (now - self._last_advance) >= 1.0 / max(self.fps, 1e-3):
                    if self._advance():
                        advanced = True
                        dt = now - self._t_prev
                        self._t_prev = now
                        if dt > 0:
                            self.fps_disp = (0.9 * self.fps_disp + 0.1 / dt) if self.fps_disp else 1.0 / dt
                    else:
                        self.playing, self.ended, self.need_render = False, True, True
                    self._last_advance = now

                if self.open_request:
                    self.open_request = False
                    self._open_dialog()
                    self.need_render = True
                if self.cam_request:
                    self.cam_request = False
                    self._open_camera()
                    self.need_render = True

                if advanced or self.need_render:
                    cv2.imshow(self.WIN, self._compose())
                    self.need_render = False

                key = cv2.waitKey(10) & 0xFF
                if key in (ord("q"), 27):
                    break
                elif key == ord(" "):
                    self._restart() if self.ended else setattr(self, "playing", not self.playing)
                    self.need_render = True
                elif key == ord("o"):
                    self.open_request = True
                elif key == ord("c"):
                    self.cam_request = True
                elif key == ord("r"):
                    self.orbit.reset()
                    self.need_render = True
                elif key in (ord("."), ord(",")):
                    self._advance()
                    self.need_render = True

                try:
                    if cv2.getWindowProperty(self.WIN, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except cv2.error:
                    break
        finally:
            self.source.release()
            cv2.destroyAllWindows()


def run_batch(source, detector, retargeting, joint_names, mesh_renderer, skel_renderer,
              camera, orbit, view_order, labels, draw_human_skeleton, panel_size,
              view_height, output_video_path, max_frames):
    """非交互:固定视角处理整段视频,hconcat 写一个 mp4。"""
    camera.pose = orbit.pose()   # 固定初始视角
    writer = None
    last_qpos = None
    idx = 0
    t_prev = time.time()
    fps_disp = 0.0
    try:
        while True:
            frame = source.read()
            if frame is None:
                break
            idx += 1
            if max_frames and idx > max_frames:
                idx -= 1
                break
            obs = detector.detect(frame)
            if obs.found:
                last_qpos = retargeting.retarget(compute_ref_value(retargeting, obs.joint_pos))
            num_box = obs.num_hands
            now = time.time()
            dt = now - t_prev
            t_prev = now
            if dt > 0:
                fps_disp = (0.9 * fps_disp + 0.1 / dt) if fps_disp else 1.0 / dt

            panels: Dict[str, np.ndarray] = {}
            if "human" in view_order:
                h = frame.copy()
                if draw_human_skeleton:
                    detector.draw(h, obs)
                panels["human"] = h
            for kind in ("mesh", "skeleton"):
                if kind not in view_order:
                    continue
                if last_qpos is None:
                    panels[kind] = placeholder(panel_size, panel_size, "waiting for hand...")
                elif kind == "mesh" and mesh_renderer is not None:
                    panels[kind] = mesh_renderer.render(last_qpos)
                else:
                    panels[kind] = skel_renderer.render(last_qpos)

            hud = f"frame {idx:05d} | {fps_disp:4.1f} FPS | hands {num_box}"
            for kind in view_order:
                put_label(panels[kind], labels[kind])
                put_label(panels[kind], hud, org=(10, panels[kind].shape[0] - 12), color=(0, 255, 0), scale=0.5)

            if output_video_path:
                row = np.hstack([fit_height(panels[kind], view_height) for kind in view_order])
                if writer is None:
                    Path(output_video_path).parent.mkdir(parents=True, exist_ok=True)
                    writer = cv2.VideoWriter(output_video_path, cv2.VideoWriter_fourcc(*"mp4v"),
                                             source.fps, (row.shape[1], row.shape[0]))
                writer.write(row)
    finally:
        source.release()
        if writer:
            writer.release()
    log(f"完成:处理 {idx} 帧" + (f" -> {output_video_path}" if output_video_path else ""))


def main(
    source_type: str = "video",                 # "video" | "camera"
    video_path: str = "data/human_hand_video.mp4",
    camera_id: int = 0,
    robot_name: RobotName = RobotName.inspire,
    retargeting_type: RetargetingType = RetargetingType.vector,
    hand_type: HandType = HandType.right,
    detector_name: str = "mediapipe",           # 感知模型:见 hand_perception.available_detectors()
    render_backend: str = "cpu",                # "cpu"(pyrender) | "sapien" | "auto"
    retarget_config: str = "",                  # 额外重定向配置(顶栏点击切换与默认对比);"标签=名/路径",逗号分隔
    output_video_path: Optional[str] = None,    # 批处理(--no-show-window)时写 mp4 的路径
    show_window: bool = True,                   # 交互式单窗口(WSLg/X 需可用显示)
    show_human: bool = True,
    show_mesh: bool = True,
    show_skeleton: bool = True,
    draw_human_skeleton: bool = True,
    panel_size: int = 640,                      # 机器人两屏输出分辨率(正方形),调大更清晰
    supersample: int = 1,                       # 超采样抗锯齿倍数:1=最快(原生分辨率,清晰);2=边缘更平滑但慢约 4 倍
    view_height: int = 640,                     # 合成/显示时统一高度
    frame_margin: float = 1.7,                  # 机器人初始取景余量,调小=手更大
    max_frames: int = 0,                        # >0 时只处理前 N 帧(批处理快速调试)
    loop_video: bool = False,
):
    """人手(视频/摄像头) + 重定向灵巧手 并排同步可视化。单进程,单窗口。

    交互(--show-window,默认):一个合成窗口。在机器人两屏上**拖拽旋转视角、滚轮缩放**;
    顶栏分段控件切换输入源(Video/Camera)和重定向配置;Space 播放/暂停,R 复位视角,
    逗号/句号单帧步进,Q 退出。给了 --retarget-config 时,顶栏会多出一个配置分段控件,点一下实时切换对比。
    批处理(--no-show-window + --output-video-path):固定视角处理整段视频,合成一个 mp4。

    A/B 对比示例(顶栏点 default / tip+PIP 分段控件切换):
      python hand_robot_visualizer.py --retarget-config "tip+PIP=inspire_hand_right_tip_pip"
    """
    log(f"source={source_type} robot={robot_name.name} type={retargeting_type.name} "
        f"backend={render_backend} detector={detector_name} show_window={show_window}")
    if detector_name not in available_detectors():
        raise SystemExit(f"未知 --detector-name '{detector_name}';可选: {available_detectors()}")
    detector = make_detector(detector_name, hand_type=hand_type.name.capitalize(), selfie=False)
    S = _build_scene(robot_name, retargeting_type, hand_type, render_backend,
                     show_mesh, show_skeleton, panel_size, frame_margin, supersample,
                     retarget_config=retarget_config)
    mesh_renderer, skel_renderer = S["mesh_renderer"], S["skel_renderer"]
    view_order = [k for k, on in (("human", show_human), ("mesh", show_mesh),
                                  ("skeleton", show_skeleton)) if on]
    labels = _labels_for(view_order, mesh_renderer, show_mesh, source_type)

    if source_type == "video":
        source: FrameSource = VideoFrameSource(video_path, loop_video)
    else:
        source = WebcamFrameSource(camera_id)

    try:
        if show_window:
            init_dir = os.path.dirname(os.path.abspath(video_path)) or str(Path.cwd())
            app = VizApp(source=source, source_type=source_type, detector=detector,
                         retargetings=S["retargetings"], joint_names=S["joint_names"],
                         mesh_renderer=mesh_renderer, skel_renderer=skel_renderer,
                         camera=S["camera"], orbit=S["orbit"], view_order=view_order,
                         labels=labels, draw_human_skeleton=draw_human_skeleton,
                         panel_size=panel_size, view_height=view_height, initial_dir=init_dir,
                         camera_id=camera_id)
            app.run()
        else:
            run_batch(source, detector, S["retargeting"], S["joint_names"], mesh_renderer,
                      skel_renderer, S["camera"], S["orbit"], view_order, labels,
                      draw_human_skeleton, panel_size, view_height, output_video_path, max_frames)
    finally:
        if mesh_renderer:
            mesh_renderer.close()


if __name__ == "__main__":
    tyro.cli(main)
