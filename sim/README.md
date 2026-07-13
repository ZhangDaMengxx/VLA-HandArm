# `sim/` 说明文档

NERO(7-DoF 臂)+ inspire(灵巧手)仿真与数据管线代码。对应项目 Phase A(见仓库根 `PROJECT_PLAN.md`)。

## 运行环境与方式

- Python 环境(WSL):`/home/zhang123/ros2_ws/enter/envs/lerobot/bin/python`,已装 mujoco 3.8.1 / pinocchio 4.0.0 / dex_retargeting / mediapipe / meshcat / lerobot。
- 从 Windows 调 WSL 运行(统一格式):
  ```bash
  MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-22.04 --cd '/home/zhang123/ros2_ws/lerobotTest' \
    -- /home/zhang123/ros2_ws/enter/envs/lerobot/bin/python sim/<脚本>.py
  ```
- 查看器类脚本会常驻,在 Windows 浏览器打开它打印的 `MESHCAT_URL`(如 `http://localhost:7009/static/`)。
- 注意:嵌套 `bash -lc '...'` 会挂;WSL 久空闲会休眠、首次调用可能超时(重试即可);MeshCat 端口每次重启会递增。

## 数据流 / 依赖关系

```
build_nero_inspire.py ─→ assets/nero_inspire_right.urdf ─┬─→ view_meshcat.py(静态查看)
                                                         ├─→ replay_assembly.py(回放)
                                                         ├─→ gesture_demo.py(手势演示)
                                                         └─→ analyze_*.py / find_home_pose.py(标定)
detect_and_retarget.py ─→ out/hand_traj.pkl ─────────────→ replay_assembly.py
schema.py + gestures.py ─────────────────────────────────→ gesture_demo.py(+ 未来 B-4 写数据集)
diag_gl.py / render_assembly.py ── GL 后端探测(结论:本机用 MeshCat,MuJoCo 离屏渲染不可用)
```

---

## 一、装配构建

### `build_nero_inspire.py`
把 NERO 臂 URDF 和 inspire 手 URDF 合并成一个装配 URDF,并在 MuJoCo 里验证加载。
- **产出**:`assets/nero_inspire_right.urdf`(nq=19:7 臂 + 12 手关节)。
- **关键变量**:
  - `NERO` / `INSP`:两个源 URDF 路径(NERO 来自 `pinocchio-kinematics-lite`,inspire 来自 `dex-retargeting`)。
  - `OUT`:输出装配 URDF 路径。
  - `MOUNT_XYZ = "0 0 0"`、`MOUNT_RPY = "0 0 0"`:**手相对法兰(link7)的安装变换**。当前=平贴法兰、手沿法兰伸出轴。改这两个值即可调安装朝向/位置(硬件到手后按真实装法标定)。
- **关键函数**:`abspath_meshes()` 把 mesh 路径转绝对(跨两个仓库树);`link_names()`/`child_links()` 找根 link;inspire 根 link = `base`,与 NERO 无命名冲突,故 inspire link 名原样保留(retargeting 依赖这些名字)。

---

## 二、标定 / 分析(一次性工具)

### `analyze_mount.py`
在 q=0 打印 link7 坐标系、手指方向 `finger_dir`、手掌法线 `palm_normal`(世界系 + link7 局部系),用来判断安装朝向对不对。当初据此定出"手指方向 = link7 z 轴"。

### `analyze_overlap.py`
打印每个碰撞网格在 q=0 的**世界坐标包围盒 center/size**,用来查手掌基座和手腕(link6)有没有重合。据此确认 `MOUNT_XYZ=0` 时掌基座贴住法兰、不悬空。

### `find_home_pose.py`
用 `NeroKinematics`(Pinocchio 逆解)求一个**初始关节姿态,使法兰伸出轴朝世界 +z**(手指竖直朝上)。多随机初值重启提高求解率。
- **关键变量**:`LIM`(7 关节限位)、`Rt`(目标朝向,approach z = 世界 +z)、`targets`(候选末端位置)。
- **产出(打印)**:`q_home = [1.2635, 0.9302, 2.6464, 1.7779, 1.0898, 0.6034, -0.6634]` —— 这串被硬编码进各查看器的 `Q_HOME_ARM`。

### `diag_gl.py`
探测 GL/渲染后端。结论:本机 `mujoco.Renderer` 不可用(缺 libOSMesa;EGL 撞 PyOpenGL 3.1.0 的 `EGLDeviceEXT`),但 `meshcat` 可用 → 检视台走 MeshCat。

### `render_assembly.py`
**已废弃**(留档)。曾尝试用 MuJoCo osmesa/egl 离屏渲染出 PNG,本机 GL 不支持而放弃。检视台改用 MeshCat(见下)。

---

## 三、检视台 / 回放(MeshCat,浏览器)

### `view_meshcat.py`
静态查看器:加载装配的**碰撞模型**,显示 home 姿态(手指朝上),起 MeshCat 服务并打印 URL。
- **关键变量**:`Q_HOME_ARM`(7 臂关节初始姿态,来自 `find_home_pose.py`);`urdf`(可命令行传入换别的装配)。

### `replay_assembly.py`
把 `out/hand_traj.pkl` 里的真实重定向轨迹**回放到装配手指上**(臂保持 home,循环动画)。
- **关键变量**:`Q_HOME_ARM`;`traj_path`(轨迹 pkl);`hand_qidx`(按关节名把 12 个手关节映射到模型 q 索引);`dt = 1/25`(帧间隔)。

### `gesture_demo.py`
**阶段 A 成品**:在装配上循环展示手势预设,平滑过渡。
- **关键变量**:`Q_HOME_ARM`;`ORDER = [open, point, victory, thumbs_up, ok, fist, open]`(展示顺序);`gesture_vec()`(手势 dict → 完整 q 向量)。

---

## 四、数据管线

### `detect_and_retarget.py`
真人手视频 → MediaPipe 检测 → dex-retargeting → **inspire 12 关节轨迹**,存盘。
- **关键变量**:
  - `CFG`:重定向配置 = `inspire_hand_right_local.yml`(开合幅度修复版)。
  - `URDF_DIR`:`assets/robots/hands`(retargeting 找 URDF 用)。
  - `OUT`:`out/hand_traj.pkl`。
  - `origin_i` / `task_i`:来自 `rt.optimizer.target_link_human_indices`,决定向量重定向的参考向量 `ref = joint_pos[task_i] - joint_pos[origin_i]`。
- **产出**:`out/hand_traj.pkl` = `dict(data=(F,12), wrist_rot=(F,3,3), joint_names=[12])`。`wrist_rot` 只是**朝向**(单目无度量位置,手腕位置要等深度)。

### `estimate_wrist.py` —— 手腕 6-DoF 估计(B-2)
从人手估计手腕位姿。朝向取自 MediaPipe(可靠);位置用单目手掌尺度启发式 `Z=f·L_米/L_像素`(近似)。
- **关键函数**:`estimate_wrist_pose(joint_pos, kp2d_px, wrist_rot, operator2mano, img_shape, focal_px=None, depth_lookup=None)` → 4×4(相机系)。
- **可插拔深度**:`depth_lookup(u,v)->Z` 给了就用真实深度(Femto ToF),否则单目近似——换 Femto 只需传这个,别处不动。

### `detect_wrist.py` —— B-2 跑通(视频→手指+手腕轨迹)
跑整段视频:检测 + 手指 retarget + 手腕估计,产出同步轨迹。
- **产出**:`out/full_traj.pkl` = `dict(hand=(F,12), wrist_pose=(F,4,4), joint_names)`。手腕位置为相机系单目近似;相机系→机器人基座对齐 + 逆解是 B-3。

### `schema.py` —— 两层数据 schema(项目基石)
- **常量**:`ARM_JOINTS`(7)、`HAND_ACTUATED`(6)、`STATE_DIM=13`、`ACTION_DIM=13`。
- **`CanonicalFrame`**(规范层,本体无关):`ego_rgb` / `ego_depth`(辅助) / `hand_keypoints`(21×3) / `wrist_pose`(4×4) / `task` / `timestamp`。
- **`EmbodimentFrame`**(本体层,一条 LeRobotDataset 记录):`observation_images_ego` / `observation_images_depth` / `observation_state`(13) / `action`(13 绝对关节目标) / `task` / `timestamp`。
- **`LEROBOT_FEATURES`**:LeRobotDataset 特征映射(B-4 写盘用)。
- **`canonical_to_embodiment(frame, retarget_hand, arm_ik, ...)`**:规范层一帧 → 本体层一帧(retarget 手 + IK 臂;action 取下一帧目标)。

### `gestures.py` —— 手势预设
- **`full12(index, middle, ring, pinky, thumb_pitch, thumb_yaw)`**:6 个驱动关节 → 完整 12 关节 dict,中间关节按 URDF mimic 比例算(`_finger_inter(p)=max(0, 1.06399p-0.04545)`;拇指 intermediate=1.334×pitch、distal=0.667×pitch)。
- **`CURL = 1.35`**:手指近端弯曲量(近上限 1.47)。
- **`GESTURES`**:`open / fist / point / victory / thumbs_up / ok`。**这些数值是手工填的第一版,可按视觉微调。**

---

## 五、生成物

- **`assets/nero_inspire_right.urdf`**:装配 URDF(由 `build_nero_inspire.py` 生成;mesh 用绝对路径,MuJoCo 加载 nq=19)。改了安装变换或源 URDF 后需重跑生成。
- **`out/hand_traj.pkl`**:手指重定向轨迹(由 `detect_and_retarget.py` 生成)。
- **`out/full_traj.pkl`**:同步 (手关节, 手腕位姿) 轨迹(由 `detect_wrist.py` 生成,B-2)。

---

## 六、B-3:人手 → 臂+手 完整回放(消抖)

`detect_wrist.py`(§四,已 `low_pass_alpha=1.0` 出满幅度)→ `full_traj.pkl` →

### `build_robot_traj.py`
把手腕位姿逆解成臂关节 + 消抖,产出本体层轨迹。
- 臂:手腕朝向(相对首帧)驱动、位置锚定 home 可达点(度量位置+相机系→基座对齐待 Femto/后续);热启动 NeroKinematics 逆解。
- **消抖 = Savitzky–Golay**(`WIN=11, POLY=3`;离线/零滞后/保幅度):手腕四元数、臂关节、手指各滤一遍。比朴素 EMA 好(EMA 会滞后+削峰)。实时驱动真机时改用 **1€ 滤波器**。
- **发现**:限制手指张开的不是低通(关掉几乎不变),是重定向映射;要更大张开调 `scaling_factor`(代价:握拳变弱)。
- **产出**:`out/robot_traj.pkl` = `dict(arm=(F,7), hand=(F,12), arm_joint_names, hand_joint_names)`(本体层 = obs.state/action 来源)。

### `replay_full.py`
加载 `robot_traj.pkl`,MeshCat 循环回放完整 [臂+手]——整条"人手视频 → 机械臂+灵巧手"同步动。

### `diag_jitter.py` / `diag_hand_amp.py`
诊断脚本:逐帧 |Δ| + 方向反转率(量抖动);手指开合幅度 + 平滑前后数据是否变化。

`out/robot_traj.pkl` 见上;`out/full_traj.pkl` 见 §五。

## 七、B-4 数据集 + C 训练验证准备

**训练方案 A(LoRA/QLoRA/全量/只训头 微调VLA)/ B(换VLA基座)/ C(ACT·Diffusion 从头训)吃同一个 `lerobot_ds`**。决定先走 C(最直接验证可训);但 ACT/DP 不在本 lerobot 0.4.4 构建里,真训要在 RTX 装标准 LeRobot。

### `build_dataset.py`
本体层轨迹(robot_traj)+ 视频 ego 帧(缩256)+ 语言标签 → **LeRobotDataset**(`out/lerobot_ds`)。
- state/action=[7臂+6驱动手]=13;obs.images.ego=视频;task=语言。
- **两坑**:`create(metadata_buffer_size=1)` 才刷 `meta/episodes` parquet;加载需 `HF_HUB_OFFLINE=1`(否则连 huggingface.co 报网络)。

### `verify_dataset.py`
独立进程回读验证(len / features / 样本形状)。

### `check_dataloader.py`
C 路线验证:`delta_timestamps` 取动作块(ACT 风格)→ torch DataLoader → batch action(8,16,13)。证明数据可被模仿学习训练管线消费。

### `probe_lerobot.py` / `probe_policies.py`
探 LeRobotDataset API / 可用 policy(本 0.4.4 只带 VLA:groot/pi0/pi05/pi0_fast/wall_x/xvla)。

**生成物** `out/lerobot_ds/`:LeRobotDataset(meta/ + data/ + videos/;710帧/1ep)。

## 关键常量速查

| 常量 | 值 / 含义 | 所在 |
|---|---|---|
| `Q_HOME_ARM` | `[1.2635, 0.9302, 2.6464, 1.7779, 1.0898, 0.6034, -0.6634]` 使法兰朝上 | view/replay/gesture_demo |
| `MOUNT_XYZ` / `MOUNT_RPY` | `"0 0 0"` / `"0 0 0"` 手平贴法兰 | build_nero_inspire |
| `STATE_DIM` / `ACTION_DIM` | 13 = 7 臂 + 6 手驱动 | schema |
| `CURL` | 1.35 手指弯曲量 | gestures |
| `CFG` | `inspire_hand_right_local.yml` 开合修复配置 | detect_and_retarget |

## 典型端到端流程

```bash
# 1. 生成装配(改过安装变换/源 URDF 后)
python sim/build_nero_inspire.py
# 2. 从视频提取重定向轨迹
python sim/detect_and_retarget.py
# 3. 回放到装配上看效果(浏览器打开打印的 URL)
python sim/replay_assembly.py
# 或:手势演示
python sim/gesture_demo.py
```
