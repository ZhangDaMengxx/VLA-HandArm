# dex-retargeting 人手↔灵巧手 并排可视化模块 · 实现规格

> 交给 WSL 里的 AI 直接照此实现。目标是新增一个单进程可视化工具,把「人手画面(离线视频或实时摄像头)」和「重定向后的灵巧手 SAPIEN 渲染」并排合成到一个窗口/一个 mp4,并叠加人手骨架层与(尽力而为的)机器人骨架层。

---

## 0. 目标
新增单进程工具 `example/vector_retargeting/hand_robot_visualizer.py`:
- **左屏**:人手画面(离线 mp4 **或** 实时摄像头,二选一接口),叠加 MediaPipe 21 点骨架 + 关节点。
- **右屏**:重定向后的灵巧手,SAPIEN 离屏渲染成图,可叠加机器人关节骨架层。
- 两屏 `hconcat` 合成到**一个窗口**和/或**一个 mp4**。
- 两种输入源都走同一条 检测→重定向→渲染 管线。

---

## 1. 运行环境
- **WSL2 (Ubuntu) + Python 3.10**。sapien 要求 `<3.13`,别用 base 的 3.13。
- 独立环境:`conda create -n dexviz python=3.10 -y && conda activate dexviz`(不动 lerobot)。
- **Vulkan(关键)**:SAPIEN 渲染依赖 Vulkan,离屏渲染也要。
  - NVIDIA 显卡:装好 Windows 侧 NVIDIA 驱动,WSL 里 `nvidia-smi` 能出卡即可。
  - 验证:`vulkaninfo | head`。无 GPU 时装 `mesa-vulkan-drivers` 走 lavapipe 软渲染(慢但能出图)。
- **显示**:本机是 Win10,WSL2 默认无 WSLg → **默认 headless 出 mp4**;只有装了 WSLg/X server 才用 `--show-window` 开实时窗口。

---

## 2. 依赖安装
```bash
# 系统包
sudo apt update && sudo apt install -y \
  libvulkan1 vulkan-tools mesa-vulkan-drivers libgl1 libglib2.0-0 ffmpeg git

# python 依赖:一条命令拉齐核心 + example 全套
cd <repo>/dex-retargeting-main   # 含 pyproject.toml 的目录
pip install -e ".[example]"
# 等价于:numpy pytransform3d pin nlopt anytree pyyaml lxml
#        + tyro tqdm opencv-python mediapipe sapien==3.0.0b0 loguru
```
**坑**:`mediapipe` 常锁 `numpy<2`,而 dex-retargeting 声明 `numpy>=2`。若 pip 报冲突,以 mediapipe 能装为准装 `numpy<2`,再验证 `python -c "import dex_retargeting"` 正常(实测重定向不依赖 numpy2 特性)。

---

## 3. 补齐 URDF 资产(否则灵巧手加载不了)
`assets/` 是指向 `dexsuite/dex-urdf` 的子模块,当前为空:
```bash
cd <repo>/dex-retargeting-main
# 若是 git 仓库:
git submodule update --init assets
# 若是 zip 解压(无 .git):
rm -rf assets && git clone https://github.com/dexsuite/dex-urdf.git assets
```
验证这两个文件存在(渲染加载的是 `_glb` 版):
- `assets/robots/hands/inspire_hand/inspire_hand_right.urdf`
- `assets/robots/hands/inspire_hand/inspire_hand_right_glb.urdf`

---

## 4. 复用的现成积木(不要重写)
| 能力 | 位置 | 用法 |
|---|---|---|
| 人手检测 + 骨架绘制 | `single_hand_detector.py` `SingleHandDetector` | `detect(rgb)->(num_box, joint_pos[21,3], keypoint_2d, wrist_rot)`;`draw_skeleton_on_image(img, keypoint_2d, style="default")` |
| 场景/光照/相机/机器人加载样板 | `render_robot_hand.py` `render_by_sapien()` 43-138 行 | 逐机器人 scale/pose、`_glb.urdf` 替换、离屏取图 |
| 重定向 | `RetargetingConfig.load_from_file(cfg).build()` → `retarget(ref_value)->qpos` | |
| 枚举/默认配置路径 | `constants.py` | `RobotName/RetargetingType/HandType/get_default_config_path` |

参考:实时链路样例 `show_realtime_retargeting.py`(多进程,两个分开窗口);离线链路 `detect_from_video.py` + `render_robot_hand.py`。

---

## 5. 参考实现(整份新文件)
> 未在本机测试(本机无 sapien),SAPIEN 相机/link API 以 WSL 实际安装的 3.0.0b0 为准;标注 ⚠ 处需实测微调。放在 `example/vector_retargeting/`,**必须在该目录下运行**(为了 `from single_hand_detector import ...`)。

```python
"""hand_robot_visualizer.py — 人手(视频/摄像头) + 重定向灵巧手 并排可视化。单进程。"""
from __future__ import annotations
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import sapien
import tyro
import yaml
from loguru import logger
from sapien.asset import create_dome_envmap

from dex_retargeting.constants import (
    RobotName, RetargetingType, HandType, get_default_config_path,
)
from dex_retargeting.retargeting_config import RetargetingConfig
from single_hand_detector import SingleHandDetector


# ---------- 1) 两个输入源接口 ----------
class FrameSource(ABC):
    fps: float = 30.0
    is_live: bool = False
    @abstractmethod
    def read(self) -> Optional[np.ndarray]: ...   # 返回 BGR 帧;结束返回 None
    def release(self) -> None: ...

class VideoFrameSource(FrameSource):               # 离线视频
    is_live = False
    def __init__(self, path: str, loop: bool = False):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise FileNotFoundError(f"无法打开视频: {path}")
        self.loop = loop
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.fps = fps if fps and fps > 1 else 30.0
    def read(self):
        ok, frame = self.cap.read()
        if not ok and self.loop:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        return frame if ok else None
    def release(self):
        self.cap.release()

class WebcamFrameSource(FrameSource):              # 实时摄像头
    is_live = True
    def __init__(self, device=0, width=None, height=None):
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头: {device}")
        if width:  self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height: self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.fps = 30.0
    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None
    def release(self):
        self.cap.release()


# ---------- 2) 离屏渲染灵巧手 ----------
_SCALE = {"ability":1.5,"dclaw":1.25,"allegro":1.4,"shadow":0.9,"bhand":1.5,"leap":1.4,"svh":1.5}
_BASE_Z = {"ability":-0.15,"shadow":-0.2,"dclaw":-0.15,"allegro":-0.05,"bhand":-0.2,
           "leap":-0.15,"svh":-0.13,"inspire":-0.15}

class RobotRenderer:
    def __init__(self, config_path: str, robot_dir: str, width=600, height=600):
        RetargetingConfig.set_default_urdf_dir(robot_dir)
        self.config = RetargetingConfig.load_from_file(config_path)
        sapien.render.set_viewer_shader_dir("default")
        sapien.render.set_camera_shader_dir("default")     # 光栅化,快;不要用 "rt"
        self.scene = sapien.Scene()

        # 地面/光照/环境贴图:照抄 render_robot_hand.py 43-60 行
        mat = sapien.render.RenderMaterial()
        mat.base_color=[0.06,0.08,0.12,1]; mat.metallic=0.0; mat.roughness=0.9; mat.specular=0.8
        self.scene.add_ground(-0.2, render_material=mat, render_half_size=[1000,1000])
        self.scene.add_directional_light(np.array([1,1,-1]), np.array([3,3,3]))
        self.scene.add_point_light(np.array([2,2,2]), np.array([2,2,2]), shadow=False)
        self.scene.add_point_light(np.array([2,-2,2]), np.array([2,2,2]), shadow=False)
        self.scene.set_environment_map(create_dome_envmap(sky_color=[0.2]*3, ground_color=[0.2]*3))

        self.cam = self.scene.add_camera("cam", width, height, 1.0, 0.1, 10)  # fovy=1.0 rad
        self.cam.set_local_pose(sapien.Pose([0.50,0,0.0],[0,0,0,-1]))

        loader = self.scene.create_urdf_loader()
        loader.load_multiple_collisions_from_file = True
        fp = Path(self.config.urdf_path); self.robot_name = fp.stem
        key = self._key(self.robot_name)
        loader.scale = _SCALE.get(key, 1.0)
        glb = str(fp) if "glb" in self.robot_name else str(fp).replace(".urdf","_glb.urdf")
        self.robot = loader.load(glb)
        if key in _BASE_Z: self.robot.set_pose(sapien.Pose([0,0,_BASE_Z[key]]))
        self.scene.update_render()

        self._to_sapien = None
        self._links = {l.get_name(): l for l in self.robot.get_links()}   # ⚠ 若 API 不同见 §6.7

    @staticmethod
    def _key(name):
        for k in ("ability","dclaw","allegro","shadow","bhand","leap","svh","inspire"):
            if k in name: return k
        return name

    def set_joint_order(self, retargeting_joint_names: List[str]):
        sj = [j.get_name() for j in self.robot.get_active_joints()]
        self._to_sapien = np.array([retargeting_joint_names.index(n) for n in sj]).astype(int)

    def render(self, qpos) -> np.ndarray:                 # 返回 BGR
        self.robot.set_qpos(np.asarray(qpos)[self._to_sapien])
        self.scene.update_render(); self.cam.take_picture()
        rgb = self.cam.get_picture("Color")[..., :3]
        return (np.clip(rgb,0,1)*255).astype(np.uint8)[..., ::-1].copy()

    def project(self, names: List[str]) -> dict:          # link 名 -> (u,v) 像素;尽力而为
        try:
            ext = np.asarray(self.cam.get_extrinsic_matrix())   # 3x4 world->cam (OpenCV)  ⚠
            K   = np.asarray(self.cam.get_intrinsic_matrix())   # 3x3                        ⚠
        except Exception as e:
            logger.warning(f"取相机内外参失败,关闭机器人骨架层: {e}"); return {}
        out = {}
        for n in names:
            link = self._links.get(n)
            if link is None: continue
            pw = np.asarray(link.get_pose().p)                  # link 全局位置 xyz  ⚠
            pc = ext[:3,:3] @ pw + ext[:3,3]
            if pc[2] <= 1e-6: continue
            uv = K @ pc; out[n] = (int(uv[0]/uv[2]), int(uv[1]/uv[2]))
        return out

    def close(self): self.scene = None


# ---------- 3) 辅助 ----------
def compute_ref_value(retargeting, joint_pos):            # 照抄现有分支逻辑
    rtype = retargeting.optimizer.retargeting_type
    idx = retargeting.optimizer.target_link_human_indices
    if rtype == "POSITION":
        return joint_pos[idx, :]
    return joint_pos[idx[1, :], :] - joint_pos[idx[0, :], :]

def draw_robot_skeleton(bgr, pts: dict, bones: List[Tuple[str,str]]):
    for a,b in bones:
        if a in pts and b in pts:
            cv2.line(bgr, pts[a], pts[b], (0,255,0), 2)
    for name,(u,v) in pts.items():
        cv2.circle(bgr, (u,v), 4, (0,0,255), -1)
    return bgr

def stack(human_bgr, robot_bgr, h=600, labels=("Human","Robot"), hud=""):
    fit = lambda im: cv2.resize(im, (int(im.shape[1]*h/im.shape[0]), h))
    L,R = fit(human_bgr), fit(robot_bgr)
    canvas = np.hstack([L,R])
    cv2.putText(canvas, labels[0], (10,30), cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255),2)
    cv2.putText(canvas, labels[1], (L.shape[1]+10,30), cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255),2)
    if hud: cv2.putText(canvas, hud, (10,h-15), cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
    return canvas


# ---------- 4) 主循环 ----------
def main(
    source_type: str = "video",                 # "video" | "camera"  ← 两个输入源
    video_path: str = "data/human_hand_video.mp4",
    camera_id: int = 0,
    robot_name: RobotName = RobotName.inspire,
    retargeting_type: RetargetingType = RetargetingType.vector,
    hand_type: HandType = HandType.right,
    output_video_path: Optional[str] = "data/hand_robot_demo.mp4",
    show_window: bool = False,                   # 仅在有显示(WSLg/X)时开
    draw_human_skeleton: bool = True,
    draw_robot_skeleton: bool = True,
    loop_video: bool = False,
):
    cfg_path = str(get_default_config_path(robot_name, retargeting_type, hand_type))
    robot_dir = str(Path(__file__).absolute().parent.parent.parent / "assets" / "robots" / "hands")
    RetargetingConfig.set_default_urdf_dir(robot_dir)
    retargeting = RetargetingConfig.load_from_file(cfg_path).build()

    detector = SingleHandDetector(hand_type=hand_type.name.capitalize(), selfie=False)
    renderer = RobotRenderer(cfg_path, robot_dir)
    renderer.set_joint_order(retargeting.joint_names)

    # 机器人骨架:从 yaml 读 base->指尖 的连线
    ycfg = yaml.safe_load(open(cfg_path))["retargeting"]
    origin_names = ycfg.get("target_origin_link_names", [])
    task_names   = ycfg.get("target_task_link_names", [])
    bones = list(zip(origin_names, task_names))
    skel_links = sorted(set(origin_names) | set(task_names))

    source = (VideoFrameSource(video_path, loop_video) if source_type == "video"
              else WebcamFrameSource(camera_id))

    writer = None; last_qpos = None; t = time.time(); hud = ""
    logger.info(f"source={source_type} robot={robot_name.name} type={retargeting_type.name}")
    while True:
        frame = source.read()
        if frame is None: break
        num_box, joint_pos, keypoint_2d, _ = detector.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        human = frame.copy()
        if draw_human_skeleton and keypoint_2d is not None:
            human = detector.draw_skeleton_on_image(human, keypoint_2d, style="default")

        if joint_pos is not None:
            last_qpos = retargeting.retarget(compute_ref_value(retargeting, joint_pos))

        if last_qpos is None:
            robot_bgr = np.zeros((renderer.cam.get_height(), renderer.cam.get_width(), 3), np.uint8)
            cv2.putText(robot_bgr, "waiting for hand...", (30,60), cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,0,255),2)
        else:
            robot_bgr = renderer.render(last_qpos)
            if draw_robot_skeleton:
                pts = renderer.project(skel_links)
                if not pts: draw_robot_skeleton = False       # 投影不可用,自动降级
                else: robot_bgr = draw_robot_skeleton(robot_bgr, pts, bones)

        now = time.time(); dt = now - t; t = now
        if dt > 0: hud = f"{1.0/dt:5.1f} FPS  hands:{num_box}"
        canvas = stack(human, robot_bgr,
                       labels=(f"Human ({source_type})", f"{robot_name.name} (retargeted)"), hud=hud)

        if output_video_path:
            if writer is None:
                Path(output_video_path).parent.mkdir(parents=True, exist_ok=True)
                writer = cv2.VideoWriter(output_video_path, cv2.VideoWriter_fourcc(*"mp4v"),
                                         source.fps, (canvas.shape[1], canvas.shape[0]))
            writer.write(canvas)
        if show_window:
            cv2.imshow("dex: human vs robot", canvas)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27): break

    source.release()
    if writer: writer.release()
    cv2.destroyAllWindows(); renderer.close()
    logger.info(f"完成 → {output_video_path}")

if __name__ == "__main__":
    tyro.cli(main)
```

---

## 6. 关键点说明
1. **单进程**:原 `show_realtime_retargeting.py` 用多进程只因交互 Viewer 阻塞;我们用离屏相机取图,单进程即可合成,简单且天然 headless。
2. **ref_value 分支**、**关节顺序映射 `retargeting_to_sapien`**、**逐机器人 scale/pose**、**离屏取图** 全部照抄现有代码,行为一致。
3. **人手骨架层** = 现成 `draw_skeleton_on_image`(MediaPipe 21 点 + 连线),稳。
4. **机器人骨架层** = 把 yaml 里 `base→各指尖` link 的三维位置投影到渲染图上画点连线,**尽力而为**:取不到内外参或 link 位姿就自动降级(`draw_robot_skeleton=False`),不影响主体。
5. **⚠ 需实测的 3 处 SAPIEN API**(§5 标注):`robot.get_links()`/`link.get_pose().p`、`cam.get_extrinsic_matrix()`、`cam.get_intrinsic_matrix()`。3.0.0b0 应都有;若签名不同,按 §6.7 兜底。
6. **6.7 骨架层兜底方案**:若上述 link/相机投影 API 在装的 sapien 版本里对不上,改用 dex_retargeting 自带的 pinocchio FK 求 link 位姿(`retargeting.optimizer.robot.compute_forward_kinematics(qpos)` + `get_link_index/get_link_pose`),再乘 `loader.scale` 与 base 偏移对齐;实在不行就 `--no-draw-robot-skeleton` 只保人手骨架,主体照样并排演示。

---

## 7. 用法
```bash
cd example/vector_retargeting

# A) 离线视频 → 出 mp4(WSL 默认,最稳)
python hand_robot_visualizer.py --source-type video \
  --video-path data/human_hand_video.mp4 \
  --robot-name inspire --hand-type right --retargeting-type vector \
  --output-video-path data/hand_robot_demo.mp4

# B) 实时摄像头(需 WSLg/X 才能看窗口;否则录进 mp4)
python hand_robot_visualizer.py --source-type camera --camera-id 0 \
  --robot-name inspire --hand-type right --retargeting-type vector --show-window

# 关掉机器人骨架层
python hand_robot_visualizer.py --source-type video ... --no-draw-robot-skeleton
```
> tyro 会把 `bool` 参数暴露为 `--draw-robot-skeleton / --no-draw-robot-skeleton` 这种开关。

---

## 8. 冒烟测试顺序(先隔离环境问题,再验新代码)
```bash
python -c "import sapien, mediapipe, cv2, nlopt, pinocchio, tyro; print('deps ok')"
vulkaninfo | head                                   # Vulkan 在位
python detect_from_video.py --robot-name inspire --video-path data/human_hand_video.mp4 \
  --retargeting-type vector --hand-type right --output-path data/inspire.pkl   # 验检测+重定向+资产
python render_robot_hand.py --pickle-path data/inspire.pkl \
  --output-video-path data/inspire.mp4 --headless                              # 验离屏渲染+_glb
# 以上都过,再跑 §7 的新工具
```

---

## 9. 已知坑
- mediapipe vs numpy2 版本冲突(§2)。
- WSL 无 GPU Vulkan → 装 mesa 软渲染;`--show-window` 在无 WSLg 时会崩,用 mp4。
- `render_*` 加载的是 `_glb.urdf`,dex-urdf 里要有(§3 验证)。
- 相机 fovy=1.0 弧度、pose `[0.5,0,0]` 是照抄的取景,换机器人若跑偏可调 `RobotRenderer.__init__` 里的 `set_local_pose`。
- 样例视频 `data/human_hand_video.mp4` 是**右手**;`--hand-type` 要跟视频里的手一致,否则 MediaPipe 检不到目标手(`SingleHandDetector` 在 `selfie=False` 时内部会翻转左右)。
