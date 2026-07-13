# 人手 ↔ 灵巧手 并排同步可视化 · 架构与文件结构

`hand_robot_visualizer.py` 的结构说明:目录布局、文件内部分区、运行调用路径,以及**如何快速切换视觉感知模型**。
> 行号为当前版本的近似位置(改动后会漂移,以 `grep -n '^\(class \|def \)'` 为准)。

---

## 1. 目录结构(磁盘位置)

```
lerobotTest/
└─ dex-retargeting-main/
   └─ dex-retargeting-main/                     ← 仓库根(含 pyproject.toml)
      ├─ assets/robots/hands/inspire_hand/
      │     inspire_hand_right.urdf              ← 运动学(建 pinocchio 模型)
      │     inspire_hand_right_glb.urdf          ← 视觉网格(.glb,真实材质)
      │     meshes/visual/*.glb, meshes/collision/*.obj
      ├─ src/dex_retargeting/                    ← 算法包
      │     constants.py                         ← get_default_config_path / RobotName / …
      │     retargeting_config.py                ← RetargetingConfig(.build())
      │     configs/teleop/inspire_hand_right.yml← vector 配置(target_link_human_indices 等)
      └─ example/vector_retargeting/
            hand_robot_visualizer.py             ← ★ 主程序(UI + 渲染 + 主循环)
            hand_perception.py                   ← ★ 感知模型接口(换模型只动这里)
            single_hand_detector.py              ← 复用:MediaPipe 检测 + 画 21 点骨架
            VISUALIZER_ARCH.md                   ← 本文件
            detect_from_video.py / render_robot_hand.py / webgl_*.py  ← 参考/旧工具
            data/
              human_hand_video.mp4               ← 示例视频(默认输入)
              hand_robot_demo.mp4                ← 批处理产物
```

**必须在 `example/vector_retargeting/` 目录下运行**(为了 `from hand_perception import ...` 和其内部
`from single_hand_detector import ...`)。运行环境:conda 环境 `lerobot`(Python 3.10)。

---

## 2. 两个核心文件

| 文件 | 职责 |
|---|---|
| `hand_robot_visualizer.py` | 输入源、相机/orbit、机器人渲染器(pyrender/SAPIEN/骨架)、合成、交互窗口、主循环、CLI |
| `hand_perception.py` | **感知模型的统一接口** + MediaPipe 适配器 + 注册表。换模型只需在这里加一个适配器类 |

复用的仓库文件:`single_hand_detector.py`(被 MediaPipe 适配器封装)、`dex_retargeting` 包
(`RetargetingConfig`、`constants`)、`assets/` 里的 URDF/网格。

---

## 3. `hand_robot_visualizer.py` 内部分区(段落 + 行号)

```
§0 工具            log() 44 · finger_color() 59
§1 输入源          FrameSource 69 · VideoFrameSource 84 · WebcamFrameSource 110
                     read()/reset()/release() —— 统一的“取一帧 BGR”接口
§2 相机            look_at() 134 · Camera 157
                     Camera.project() —— 3D 世界点 → 像素(骨架层用,和网格同相机)
§3 运动学模型      RobotModel 184
                     __init__            建 pinocchio 模型 + _glb 视觉几何
                     make_q()            qpos → pin 配置向量(连续关节展开成 cos/sin)
                     geometry_placements() 每个视觉网格的世界位姿(mesh 渲染用)
                     bounding_sphere()   自动取景
                     skeleton_segments() 关节树 + 指尖连线(骨架渲染用)
§4 渲染器          RobotRenderer(抽象)297
                     PyrenderMeshRenderer 309   ← CPU 默认(EGL 离屏 + 超采样抗锯齿 + GLB 材质)
                     SkeletonRenderer      400   ← 纯 CPU 火柴骨架(cv2 投影画线)
                     sapien_available()    458   ← 子进程探测 SAPIEN(隔离段错误)
                     SapienMeshRenderer    471   ← GPU 后端(带可用 Vulkan 的机器)
§5 辅助            compute_ref_value 550 · put_label 561 · fit_height 566 · placeholder 574
                     clamp 580 · OrbitController 584 · ask_open_file 615(zenity/tkinter 文件框)
§6 主程序          _build_scene 651   建 retargeting/model/camera/orbit/renderers
                     _labels_for 698
                     VizApp 708         交互式单窗口(见下)
                     run_batch 928      非交互:固定视角出 mp4
                     main 992           CLI 入口(tyro)
```

`VizApp`(708)关键方法:
```
__init__ 713 · _advance 751(取帧→检测→重定向) · _robot 764 · _compose 770(合成一帧)
_draw_chrome(Pillow 顶栏:分段控件/标签 chip/HUD,并记录点击热区 self._hit) · on_mouse(拖拽→orbit,点分段控件→切源/切配置)
_open_dialog(选视频文件) · _open_camera(切到摄像头) · _set_cfg(切换重定向配置) · _restart · run(主循环)
```

---

## 4. `hand_perception.py` 结构

```
HandObservation (dataclass)   一帧的感知结果:found / num_hands / joint_pos(21×3) / keypoints_2d / raw
HandDetector (ABC)            接口:detect(frame_bgr)->HandObservation ; draw(img,obs)(默认通用画法)
register_detector(name)       类装饰器:注册一个模型
make_detector(name, **kw)     工厂:按名字造模型
available_detectors()         列出已注册的名字
MediaPipeHandDetector         默认实现(封装 single_hand_detector.SingleHandDetector),注册名 "mediapipe"
```

---

## 5. 运行调用路径

**启动**
```
python hand_robot_visualizer.py [参数]
└ __main__ → tyro.cli(main) → main() [992]
   ├ make_detector(detector_name, …)            # hand_perception:造感知模型(默认 mediapipe)
   ├ _build_scene() [651]
   │    ├ RetargetingConfig(...).build()         # 重定向器
   │    ├ RobotModel() [184]                      # pin 模型 + _glb 视觉几何
   │    ├ OrbitController [584] + Camera [157]    # 初始视角(mesh/skeleton 共用)
   │    ├ PyrenderMeshRenderer [309] (cpu) 或 SapienMeshRenderer [471]
   │    └ SkeletonRenderer [400]
   ├ VideoFrameSource / WebcamFrameSource
   └ show_window ? VizApp(...).run() [875] : run_batch() [928]
```

**交互主循环**(`VizApp.run` 875,每帧)
```
① 到点 → _advance() [751]
     source.read() → detector.detect(frame) → HandObservation
     obs.found ? 对每个配置各算 rt.retarget(compute_ref_value(obs.joint_pos)) → cur_qpos_by_cfg
                 激活配置的结果 → self.cur_qpos(点顶栏分段控件在配置间零延迟切换)
② open_request → _open_dialog() → ask_open_file()[615] → 换新 VideoFrameSource
   cam_request  → _open_camera()  → WebcamFrameSource(camera_id)
③ 需重绘 → _compose() [770]
     camera.pose = orbit.pose()                        # 跟随鼠标
     human    : detector.draw(h, obs)                   # 感知模型自己画骨架
     mesh     : PyrenderMeshRenderer.render() [378]     # _update_camera + geometry_placements + EGL 1280²→640
     skeleton : SkeletonRenderer.render() [412]         # skeleton_segments + camera.project
     fit_height + hconcat → _draw_chrome()[Pillow overlay] → 一张画布 → cv2.imshow
④ waitKey + 键盘;鼠标 on_mouse()[820] → orbit.drag/zoom → 下帧用新 camera.pose
```

**批处理**(`run_batch` 928):同样的检测→重定向→渲染,但相机固定、逐帧写进一个 mp4。

---

## 6. 换视觉感知模型(接口契约 + 步骤)

### 会不会崩?
接口化之前:**会**——旧代码写死了 `SingleHandDetector` 的返回元组和 MediaPipe 的数据类型。
接口化之后:主程序/重定向都不动,换模型 = 写一个适配器类。**但有一条硬契约必须满足**,否则重定向结果会错(不会崩):

### 契约(唯一要点)
`detect()` 返回的 `HandObservation.joint_pos` 必须是:
- 形状 **(21, 3)**,MediaPipe / MANO 的 21 点顺序(0=手腕,4/8/12/16/20 = 拇/食/中/无名/小指尖);
- 坐标系:**机器人 / MANO 系**(手腕平移到原点、已乘 operator2mano 旋转)。

这是 `dex_retargeting` 里 `target_link_human_indices`(如 `[[0…],[4,8,12,16,20]]`)索引的布局。
新模型若输出的点数/顺序/坐标系不同,就在**适配器内部转换成这个 21×3 布局**。

### 三步加一个模型(写在 `hand_perception.py`)
```python
@register_detector("mymodel")
class MyHandDetector(HandDetector):
    def __init__(self, hand_type="Right", selfie=False, **kw):
        ...                       # 载入你的模型
    def detect(self, frame_bgr):
        kp21x3 = ...              # 跑模型 → 转成 21×3 MANO 系(见契约)
        kp21x2 = ...              # 可选:像素坐标,用于画骨架
        return HandObservation(found=..., num_hands=..., joint_pos=kp21x3, keypoints_2d=kp21x2)
    # 可选:def draw(self, image_bgr, obs): 用模型自带画法覆盖默认画法
```
然后运行:
```bash
python hand_robot_visualizer.py --detector-name mymodel
```
不改主程序、不改重定向、不改渲染。

> **`"mymodel"` 该叫什么?** 它是你自己起的**占位名**,没有硬性规定——任意小写短名即可。
> 唯一要求:`@register_detector("X")` 的 `X` 和命令行 `--detector-name X` 用**同一个字符串**
> (类名 `MyHandDetector` 也随意)。约定用模型本身的名字,例如 `hamer`、`wilor`、`rtmpose`;
> `mediapipe` 已被默认实现占用。用 `available_detectors()` 可列出当前已注册的名字。

---

## 7. 一帧的数据流

```
帧(BGR) ──HandDetector.detect──▶ HandObservation.joint_pos (21×3, MANO)
                                        │
                       compute_ref_value(retargeting, joint_pos)      [550]
                                        │  (按 optimizer.retargeting_type 取 POSITION/VECTOR 分支)
                                        ▼
                          retargeting.retarget(ref_value) ─▶ qpos
                                        │
                    ┌───────────────────┼────────────────────┐
                    ▼                                          ▼
     RobotModel.make_q(qpos) → pin FK               (同一 qpos、同一 Camera)
        ├ geometry_placements → PyrenderMeshRenderer  (中屏:实体网格)
        └ skeleton_segments   → SkeletonRenderer      (右屏:火柴骨架)
```
人手左屏与两个机器人屏由**同一主循环、同一帧**产出 → 时间戳硬同步。

---

## 8. CLI 参数速查(`main`)

| 参数 | 默认 | 说明 |
|---|---|---|
| `--source-type` | `video` | `video` \| `camera` |
| `--video-path` | `data/human_hand_video.mp4` | 视频输入 |
| `--camera-id` | `0` | 摄像头编号 |
| `--robot-name` | `inspire` | 机器人手 |
| `--retargeting-type` | `vector` | 重定向类型 |
| `--hand-type` | `right` | 左/右手(要和视频里的手一致) |
| `--detector-name` | `mediapipe` | **感知模型**(见 `available_detectors()`) |
| `--render-backend` | `cpu` | `cpu`(pyrender) \| `sapien` \| `auto` |
| `--retarget-config` | `""` | 额外重定向配置,界面点顶栏分段控件与默认对比切换;写 `标签=名/路径`(名会在配置目录找 `<名>.yml`),逗号分隔多个 |
| `--show-window / --no-show-window` | `True` | 交互单窗口 / 批处理出 mp4 |
| `--show-human / --show-mesh / --show-skeleton` | `True` | 各屏开关 |
| `--panel-size` | `640` | 机器人屏输出分辨率(调大更清晰) |
| `--supersample` | `2` | 超采样倍数(抗锯齿;嫌慢设 `1`) |
| `--view-height` | `640` | 合成/显示统一高度 |
| `--frame-margin` | `1.7` | 初始取景余量(调小=手更大) |
| `--output-video-path` | `None` | 批处理写 mp4 的路径 |
| `--max-frames` | `0` | >0 只处理前 N 帧(调试) |

**交互**:拖拽=转视角 · 滚轮=缩放 · `Space`=播放/暂停 · `R`=复位视角 · `,`/`.`=单帧步进 · `Q`/`Esc`=退出。顶栏用 **Pillow 绘制的分段控件**(圆角、半透明、抗锯齿):**Source [Video | Camera]** 切输入源(Video 会弹文件对话框),**Retarget [default | …]** 切重定向配置;当前项高亮。点击即生效(不再有键盘 `T`)。`O`/`C` 仍可作为切视频/摄像头的快捷键。摄像头用 `--camera-id` 指定设备号(默认 0)。

**A/B 对比重定向目标**(tip-only vs shadow 式 tip+PIP):
```
python hand_robot_visualizer.py --retarget-config "tip+PIP=inspire_hand_right_tip_pip"
```
启动后顶栏出现 `default | tip+PIP` 分段控件,**点一下**即在两种重定向目标间切换(每帧对两个配置都算好了,切换无延迟;暂停时切换也能立即看到差异,便于逐帧对比)。可切多个:`--retarget-config "B=...,C=..."`。
