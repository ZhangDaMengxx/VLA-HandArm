# src/ 说明

> **维护提示（2026-08-14）**：本文包含长期数据管线和大量阶段性调试结论。涉及资产
> 路径、灵巧手限位或装配参数时，以当前代码、根目录 `HARDWARE.md` 和
> `PROJECT_STATUS.md` 为准。现行 MCP 部署不在 `src/`，本地 Web combo 也不等于 MCP
> combo 能力。
>
> 特别注意：下文 `build_inspire_from_vendor.py` 段落描述的是旧生成链。该脚本仍输出
> `assets/inspire_hand/` 并保留 0.6/1.47 限位，而当前运行驱动和标准资产使用
> `assets/hand/` 与 0.48/1.333。修复脚本前不要运行它覆盖当前标准资产。

NERO(7-DoF 臂)+ inspire 灵巧手的仿真与数据管线代码。总体方案见仓库根 `PROJECT_PLAN.md`,快速上手见根 `README.md`。

实时合体跟随的机械臂 IK 由 `live_ik_scheduler.py` 以单 worker、latest-only 方式调度；
灵巧手目标先进入独立 mailbox，不等待 IK，过期或失效的 IK 结果不得下发。

## 数据流(两层架构)

核心设计:把「人做了什么」(**规范层**,本体无关)和「某台机器人转哪些关节」(**本体层**,每机器人一份)分开存。人手 21 点 → 机器人关节是**有损不可逆**投影;只存本体层等于「只留编译产物、丢了源码」,换本体就废。规范层是长期资产,换机器人只按新 URDF 重派生,采集不重来。

```text
data/*.mp4                           真人第一视角手势
   │  build_canonical.py             创建并绑定一次 Capture，不 retarget、不平滑
   ▼
datasets/captures/capture_<id>/
├── source/                           原始输入、标定、质量 profile 快照和校验表
├── ego/                 ★ 本体无关  独立 LeRobotDataset：RGB + 21 点 + wrist_pose + task
│   │  derive_embodiment.py --robot X
│   │    手：kp -> dex-retarget -> 12 关节 -> 取 6 驱动
│   │    臂：wrist_pose -> 稳定化 -> NeroKin IK
│   ▼
├── robot_datasets/X/target_revision_v001/retarget_v001/
│   ├── data|meta|videos/ ★ 训练数据  独立 LeRobotDataset：state/action/images.ego
│   └── exports/workbench/             robot_traj.pkl/.npz，仅可视化/工作台导出
├── lineage/                           Source -> Ego -> RobotDataset 血缘
└── reports/                           Capture 与 retargeting 验收报告
```

换本体 = 只在 `robot_specs.py` 加一个 RobotSpec,再 `derive_embodiment.py --robot 新名字`;规范层不动。

### 现有本体规格(`robot_specs.py`)

| `--robot` | 数据源 | `frame_mode` | 位置 | 可达 | 说明 |
|---|---|---|---|---|---|
| `nero_inspire_rgb`(别名 `nero_inspire`) | 普通 RGB 视频 | `legacy` | relative(5cm 限幅) | 663/780 | 单目,朝向相对首帧;日常默认 |
| `nero_inspire_rgbd` | kinect RGB-D | **`metric`** | absolute(米制) | **550/557** | 固定 base 绝对映射,几何正确,主路径 |
| `nero_inspire_rgbd_anchored` | kinect RGB-D | `anchored` | fixed(锁 home) | 555/557 | 首帧锚定 + 重摆 home,fallback/对照 |

三种 `frame_mode`:
- **`legacy`**:朝向走 `wrist_motion_basis_R` 的相对增量,位置走 `wrist_position_basis_rpy` 相对首帧。单目 RGB 用。
- **`anchored`**:从首帧手腕朝向反推 `R_base_world`,把相机↔世界固定旋转吸收进去。缺点:`q_home` 一身二职(既是 IK 种子又是映射锚),换数据集需重摆 home。
- **`metric`**:`R_base_world`/`p_base_anchor`/`metric_scale` 是**物理摆放定死的固定量**,不由数据反推;世界系米制手腕位姿经固定外参直接映到 base。`q_home` 退回纯 IK 种子(实际种子由 derive 按数据帧 bootstrap,只决定收敛到哪个 IK 分支,不改目标位姿)。RGB-D 度量数据的干净解。
  - 诚实标注①(**centroid 每段居中**):无真实 robot-camera 外参时的务实取舍——保运动形状 + 米制尺度,把每段手位置质心平移到 `p_base_anchor`,但不保跨数据集的绝对世界位置。真绝对位需机器人与相机共处一场景做外参标定。
  - 诚实标注②(**种子 bootstrap**):合法,因为只在关节量程上撒确定性网格选一个"扇面居中"的 IK 分支,不动目标位姿。naive 自然 home 种子只有 258/557,fan-centered 种子达 550/557。

## 端到端

```bash
python src/build_nero_inspire.py             # 装配 URDF(一次即可)
python src/build_canonical.py                # 视频 → 规范层(--video 指定视频,--hand-estimator mediapipe)
python src/build_canonical_from_processed.py --input hand_result.npz
python src/build_canonical_from_rgbd.py --input-root third_party/kinect2-middle/kinect2_middle --camera kinect2_middle
python src/derive_embodiment.py --robot nero_inspire_rgbd --emit-traj  # RGB-D metric 派生 + 轨迹
python src/replay_rerun.py --robot nero_inspire_rgbd --serve           # Rerun 三面板可视化(带同一 --robot)
```

Canonical 构建器默认新建 Capture，后续命令默认读取 `datasets/captures/` 中最新的 `ready` 批次。
构建中或失败的批次不会被隐式选择，但仍可用 `--capture-root` 显式诊断或重建。
重复运行或并行处理时应给整条命令传同一个 `--capture-root`，避免“最新一批”在中途改变。
每个新 Ego 的 `meta/coordinate_system.json` 使用 2.0 契约：普通 RGB 腕姿声明为
`episode0_camera`，标定 RGB-D 腕姿声明为 `scene_world`，3D/2D 关键点也各自声明 frame。
派生、验证、验收、分析和 Rerun 均读取该文件，不根据 `source_kind` 推断坐标语义。
质量判据由 `configs/quality_profiles/*.json` 版本化，并在构建时快照到
`source/quality_profile.json`。验收工具读取该快照；内部 timestamp cadence 不等同硬件同步。
profile schema 1.1 和验收 JSON 会区分绝对精度、稳定性代理与连续性；需要真值但缺失时
`value/pass` 均为 null，不用骨长或深度连续性代理替代。
需要旧 `src/out/` 时必须显式传 `--legacy-out`；旧目录不会自动移动或删除。Web/ROS 仍使用
Python 3.10 + `lerobot 0.4.4`；数据集生成与严格 v3 验证使用 `.envs/lerobot-v3` 中已验收的
Python 3.12.13 + `lerobot 0.6.1`。复现安装和离线校验命令见根目录 `HANDBOOK.md`。
各生成器在 `save_episode()` 后显式 `finalize()`；Ego/Robot 每个 episode 自动建立不覆盖人工
审核的 annotation，Robot 另有结构 QA。完整 Capture 使用
`verify_dataset.py --capture-bundle --capture-root <capture>` 校验。

拖拽上传视频的一键图形界面见根 `README.md` 的 `app_gradio.py`。

## 各组件

**Web 实时手部控制** `app_web.py` + `web/hand_tracker_tasks.js` +
`hand_target_filter.py` + `hand_target_mailbox.py`：浏览器使用本地
`@mediapipe/tasks-vision` Hand Landmarker，
按 Tasks GPU → CPU → Legacy 降级；21 点世界坐标经 `/ws/hand/mimic` 和
dex-retargeting 得到 6 个驱动关节。3D 预览立即返回原始 retarget 结果；真手目标先经
六关节 One Euro 自适应滤波和硬件分辨率级写入抑制，再由 30Hz latest-target mailbox
单独投递，
最多一个待发目标和一个真实 ACK 在途。超过 200ms 没有有效目标会重置滤波状态，
HTTP fallback 只维持 retarget/预览，不驱动真手。协议、真机数据和验收项见
`web/MEDIAPIPE_TASKS_MIGRATION.md` 与 `HAND_DEBUG.md`。

**通用灵巧手可行域自动化** `hand_feasibility.py` +
`configs/hands/inspire_rh56dfx_right.json`：datasheet/URDF 提供跨设备共享的资产标称 rad，
厂商 Adapter 在归一量/raw 域执行只读预检、单关节扫描和规范声明的低维 interaction，
以位置误差、稳态力、错误位、电流和温度组合判断并原子输出可恢复 Profile。完整 Profile
可把 VLA/遥操作目标保守投影到已验证区域；Mock Profile 默认禁止用于真机。使用方式和
换手约定见 `HAND_FEASIBILITY_AUTOMATION.md`。

**装配** `build_nero_inspire.py`:合成 NERO 臂 + RH56DF 适配法兰 + inspire 右手的装配 URDF,MuJoCo 验证加载(nq=19)。链路 `link7 → link8 → rh56df_adapter_flange → base → hand_base_link`。两段挂接变换(`FLANGE_MOUNT_*`、`MOUNT_*`)是从装配体 `nero_RH56DF.stl` 反解的,不是目视标定,三个件对装配体中位残差 0.27–0.63mm;推导脚本与结论见 `build_urdf/`。脚本还负责把臂的视觉网格由 `.dae` 换成同名 `.STL`(缺 pycollada 的查看器会整条臂不显示),并在源头补臂关节限位 effort=100/velocity=5(臂 URDF 原文是 0/0,ros2_control 推不动)。

**手 URDF 生成** `build_inspire_from_vendor.py`(2026-08-07):从厂家新 URDF(`assets/urdf_right/urdf_right_2025_4_18`)生成项目格式,输出覆盖 `assets/inspire_hand/inspire_hand_right.urdf`。做的事:①link/joint 名改回项目规范(`right_little_*` → `pinky_*` 等);②补 5 个 `*_tip` links(dex-retargeting 需要,用"最远 2% 顶点质心"推导);③STL→GLB(浏览器需要);④展平链式 mimic(dex-retargeting 不支持链式);⑤放宽 mimic 子关节 limit 到"驱动走满时的值"(厂家原文件 thumb 链不自洽,不改浏览器会在中途饱和);⑥驱动关节 limit 覆盖成与 `inspire_hand.py` HAND_LIMITS 一致(避免预览与硬件不一致)。新 URDF 修正了拇指旋转关节相对 base 的装配位置(老 dex-urdf 在 base→hand_base_link 插了人为的 -90°X,180°Z 中间层,新的没这层,所以 `base_joint` 现在是单位变换)。指尖位置相对老版移了 19-27mm(新 mesh 几何和坐标系都变了),retargeting 的 `scaling_factor` 可能要重调。旧版备份在同目录 `.bak_dexurdf`。厂家新限位值(`thumb_pitch` 0.48、四指 1.333)记录在脚本注释里但**未采用**(收紧会丢真手 17%/4.5% 行程),等实测后再决定;只同步了 `thumb_yaw` 1.246165(零行程损失)。⚠ `build_urdf/` 的标定脚本(`finger_check.py` 等)按老 hand_base_link 坐标系写的,现在那个系变了,要重跑标定得先调它们。

**运动学** `nero_kin.py`:NERO 正逆运动学,纯 pinocchio 从 URDF 读。`fk(q)`→4x4 位姿,`ik(T,q_init)`→关节角(阻尼最小二乘)。home 姿态(法兰朝上)存在 `robot_specs.py` 的 `q_home`。`test_nero_kin.py` 是它的单测。

**规范层** `build_canonical.py`:整段视频过 `hand_estimators.py` 统一接口。当前可用 `mediapipe` 后端,输出 21 点、2D 点、visibility 和 wrist pose;`wilor` 入口已预留,接入时必须 remap 到 canonical 21 点顺序。`estimate_wrist.py` 估手腕 6-DoF(位置用手掌尺度反推深度,单目近似,留了 `depth_lookup` 接口等 Femto 深度)。默认存 `<capture>/ego/`,不 retarget、不平滑。

**外部处理结果导入** `build_canonical_from_processed.py`:把其他电脑跑好的 MediaPipe/WiLoR 结果导成 canonical。支持 `.npz/.pkl/.json`;最低字段为 `hand_keypoints` `(N,21,3)`/`(N,63)` 和 `wrist_pose` `(N,7)`/`(N,4,4)`,可选 `hand_keypoints_2d`、`hand_visibility`、`fps`、`hand_estimator_id`、`wrist_pose_frame`。也可用 `--wrist-pose-frame episode0_camera|scene_world` 声明；输入字段与命令行冲突会拒绝导入。旧文件缺失声明时为兼容现有 Web 上传按 `episode0_camera` 导入，并在元数据标记为兼容默认，不能当作已标定世界系。Web 左侧“上传手部结果”按钮走这条路径;没有原视频时 Human 面板会从 canonical 画 2D/3D 投影手部骨架,用于对照机器人重定向是否贴合。

**RGB-D 融合导入** `build_canonical_from_rgbd.py`:读取 `color/frameXXX.png` + `depth/frameXXX.png` + `calibration.json`。先用 `--hand-estimator mediapipe` 得到 21 个 2D 点,再从 aligned depth 按内参反投影得到 wrist 的 metric 世界系位置,最后用 `extrinsics.direction=camera_to_world` 的 `wTc` 转世界系,写入 metric `observation.wrist_pose`。`observation.hand_keypoints` 默认仍写 MANO/手腕局部系,因为 dex-retargeting 要求 `joint_pos` 是 MANO 局部 21 点;如果强行写每个关键点的 depth-world 3D,手指容易查到物体/背景深度而误握拳。当前默认保持 MediaPipe 单手提取,默认 `--target-hand Right --max-num-hands 1`;在 `selfie=False` 下这个标签对应当前样例里画面上的目标手。需要调试逐点深度时可显式加 `--hand-keypoints-source depth_world`。

**本体层** `derive_embodiment.py`:读 canonical + `RobotSpec`(`robot_specs.py`)→ 手 retarget + 臂稳定化/IK + SavGol → `<capture>/robot_datasets/X/target_revision_v001/retarget_v001/`(加 `--emit-traj` 在其 `exports/workbench/` 出轨迹)。state/action = (13) = [7 臂 + 6 手驱动]。臂映射按 `spec.frame_mode` 分派(见上表):`metric`(RGB-D 默认,`_solve_arm_metric`,固定 base + 米制 + 种子 bootstrap)、`anchored`(`_solve_arm_anchored`,首帧锚定)、`legacy`(RGB 默认,相对首帧)。`--arm-position-mode` 可覆盖:`absolute`(metric 绝对米制)/`fixed`(锁 home/anchor,仅朝向)/`relative`(legacy 相对位移)。

**动态腕部方向映射**: `RobotSpec.wrist_motion_basis_R` 只作用在手腕相对旋转增量上,公式为 `dR_robot = B @ dR_human @ B.T`。矩阵 `B` 的列向量直接表示 human physical wrist X/Y/Z 分别映射到 robot ee 哪根轴;`ee_frame_correction_rpy` 仍只管初始安装/显示姿态。当前 NERO 试验矩阵为 `diag(1,-1,-1)`,即 human X→robot +X、human Y→robot -Y、human Z→robot -Z。CLI 仍可用 `--wrist-motion-basis-rpy` 临时把 RPY 转成矩阵覆盖默认值。

**深度位置解锁映射**: `RobotSpec.wrist_position_basis_rpy` 只作用在 wrist 相对平移上,当前试验值为 `(0, 0, -90°)`。5cm 限幅下 IK 成功率比原样 Kinect/world 位移更高;正式空间模仿仍需要 `T_base_camera`。

> **手腕朝向稳定化**(`wrist_stabilize.py`):臂晃动几乎全来自手腕朝向相对首帧漂到 43°,其中 91% 是**出平面**(手掌法向倾斜,单目深度估不准),面内滚转只有几度、基本是真手势。故 derive 默认开两道:`gate_deg`(残差门限剔离群跳变帧)+ `oop_alpha`(衰减出平面分量、保面内),参数在 RobotSpec 里。效果:臂运动幅度 184°→57°、IK 全收敛、真手势保留。是各向异性可观测性加权的轻量近似;完整 RTS/因子图待 Femto 深度。

**可视化** `replay_rerun.py`:三面板同一时间轴硬同步——Human(视频 + MediaPipe 骨架)、Robot 3D(装配网格,鼠标轨道旋转)、关节角曲线(游标跟随)。默认读最新 `ready` Capture 中 RobotDataset 的 `exports/workbench/robot_traj.pkl` 回放,不实时 retarget。

```bash
python src/replay_rerun.py                 # 存 <capture>/reports/replay.rrd,Rerun 查看器打开
python src/replay_rerun.py --serve         # 起 web,浏览器开打印的完整 URL
python src/replay_rerun.py --traj a=/path/to/raw.pkl --traj b=/path/to/filtered.pkl      # A/B 对比
```

默认会在 3D 视图额外画坐标系调试轴:

- `world/frames/robot_base`:URDF/Pinocchio 机器人 base,也就是当前 IK 的目标参考系。
- `world/<traj>/frames/robot_ee_current_link7`:当前轨迹下 NERO `link7` 末端坐标系。
- `world/frames/human_wrist_raw_<frame>`:canonical 里的原始 `observation.wrist_pose`，`<frame>` 来自坐标契约。它没有额外应用 `T_base_camera`，只是临时画到同一 Rerun 视图方便看轴向和相对运动，不能当作已经和机器人 base 对齐。

坐标轴颜色固定为 X=红、Y=绿、Z=蓝。想关掉调试轴用 `--no-debug-frames`;末端 frame 可用 `--ee-frame link7` 覆盖。

Human 2D 面板也会在手腕点叠加 wrist 坐标轴:

- 普通 mp4:使用实时 MediaPipe 检测得到的 wrist frame。
- 上传 `.npz/.pkl/.json` 后无原视频:使用 canonical `observation.wrist_pose` 画在生成的手骨骼点上。
- 这是 3D wrist 方向在 2D 图上的方向叠加,只用于确认 MediaPipe/WiLoR 认为的 wrist X/Y/Z 轴,不代表已完成 `T_base_camera` 对齐。

坑:`--serve` 要开脚本打印的**完整 URL**(含 `?url=rerun+http://<WSL-IP>:9876/proxy`),裸开 `IP:9090` 只有空欢迎页;数据源主机用 WSL IP(127.0.0.1 从 Windows 连不到)。视觉网格 `.dae` 自动回退同名 `.stl`(免装 pycollada);视频帧走 JPEG 编码,否则 .rrd 大一个数量级。

**数据结构** `schema.py`:锁定的两层 schema(canonical 帧 / embodiment 帧),含 `STATE_DIM`。
**手部估计器接口** `hand_estimators.py`:把 MediaPipe / WiLoR 等模型统一成 canonical `HandObservation`。公共输出是 `keypoints_3d`、`keypoints_2d`、`visibility`、`wrist_pose`;WiLoR/MANO 富层走 `mano` 字段。
**校验** `verify_dataset.py`:回读校验 LeRobotDataset(探正确的属性名)。
**学习脚本**(与管线无关,自用):`print_jacobian.py`(把某姿势的雅可比打屏看懂 J)、`solve_qp_step.py`(用雅可比把「末端想这么动」解成关节速度)。
**路径** `capture_bundle.py`:集中管理正式 Capture 数据路径、manifest、checksum 和血缘；`paths.py` 继续管理项目资产路径。

**质量口径** `quality_profiles.py` + `configs/quality_profiles/`:校验 profile、固化 Capture
快照并为 `measure_acceptance.py` 提供版本化阈值。旧 RGB/RGB-D/processed 默认使用兼容
profile；未来固定相机 60 Hz 目标需显式选择 `ego_fixed_rgbd_60hz_v1`。

## 语音控制(技能层 `skills/`)

一句话 → 技能调用:

```
文本框 或 🎤 说话(浏览器 Web Speech API)
   │  intent.py            模糊匹配 + 修饰词剥离 → skill_id + params
   ▼
POST /api/voice/parse      只解析,**不执行、不碰硬件**;每次解析都落盘
   │  页面弹确认卡,人点「执行」   ← 误识别最坏就是弹错一张框
   ▼
POST /api/voice/invoke     SSE;console_exec.py → arm_console / hand_console
```

**ASR 走浏览器端**(2026-08-06)。麦克风在**客户端**那台机器上,服务端(WSL)拿不到;
而且这台机器没 GPU,服务端跑 Whisper 是 CPU 解码、延迟到秒级。

两个硬约束:

- **需要安全上下文**。只有 `https://` 或 `localhost` 能拿麦克风权限。用 WSL IP
  (`172.25.x.x:7860`)打开会被浏览器直接拒,**这不是代码能绕的** —— 改用
  `localhost:7860`(WSL2 有 localhost 转发)或上 HTTPS。代码里检测到会说清怎么办。
- Chrome 的中文识别把音频发到 Google,**要外网**。

安全性质不变:ASR **只填文本框 + 触发解析,绝不执行**,和打字走完全同一条路。
所以"识别只解析、不执行"对语音自动成立 —— 听错最坏是弹错确认框。

**极性词纠错**:Web Speech API 没有热词偏置接口,只能事后纠(`ASR_FIX` 表)。
`夏使能`/`吓使能`/`下时能` → `下使能` 这类同音近音误写。**只纠能确定的**,
不做语义猜测 —— 纠错本身出错就是引入新的反向风险。原话和纠正后都落盘
(`text_raw` / `text`),能事后查纠错有没有纠反。

**为什么走 console 不走 ROS**:技能执行有两个后端 —— `runner.py`(ROS bridge)和
`console_exec.py`(两个 console)。臂走 can0、手走 RS485,console **独占**它们;ROS
bridge 会抢同一条通道,后果不是报错而是**互相覆盖**(见 `COMBO_DEBUG.md`)。真机验过
的是 console 那条,所以语音走它。确认闸(`runner.Gate`)与调用日志两条路**共用一份
实现**,不长出两套解释。

四道闸全在服务端强制,前端绕不过去:

1. **语音白名单** —— 清单里 `voice_enabled: false` 的技能语音命不中。
   `/api/voice/invoke` 把 source **硬写成** `voice`,前端说了不算。
2. **二次确认** —— 26 条技能 21 条 `need_confirm: true`,信封缺 `confirmed` 就拒。
   确认走**页面按钮**而不是再说一句「确认」:确认也过识别的话,误识别风险叠两层。
   免确认的 5 条,每条都有理由:

   | 技能 | 为什么免确认 |
   |---|---|
   | `estop` | 方向是 fail-safe,误停不误动 |
   | `hand_release` | **手没有急停通道**,急停只停臂、手保持当前位置。所以这是手唯一的"放开"入口。夹着东西等人点确认,等的就是持续夹着的时间;误触发=东西掉了(可恢复),延迟=夹坏(不可恢复) |
   | `hand_grip_soft/normal/firm` | **不产生运动**,只改下次抓握的力度档。要确认反而会让人放弃用语音调力度 |
3. **语音限速** —— `max_speed` 压过用户给的任何值(轨迹回放语音路径最快 1.0 倍)。
4. **通道/使能预检** —— console 没接入、臂未使能、急停生效中都当场拒。预检按清单的
   `requires` 判**而不是硬编码 id**,所以 `arm_reset` / `prepare_arm`(它们正是解除
   未使能与急停的手段)不会把自己拦下来。

**意图解析** `intent.py`(纯 Python,可脱机单测):三步 —— 整句精确命中别名表 → 剥掉
修饰词/数值后再精确 → 字 bigram 模糊匹配。不用编辑距离,因为中文命令只有 2-6 字,
一个字的增删就把距离比例拉到 0.2-0.3,阈值没法定。**重名不猜**:前两名分差在 margin
内就返回候选让人点(与 `gesture_pack.find_by_name` 同一原则),例如只说「手」会同时
命中张开/握拳,判 ambiguous 而不是随机选一个。修饰词只落到技能**自己声明过**的参数
上:「回零位慢一点」→ `duration` 7.5s;「握拳快一点」→ 没有可调参数,只提示不硬塞。
夹取与语音限速仍归 `schema.resolve_params`,本层不越界。

**落后检测** `console_exec.py`:`duration` 在 console 协议里**没有对应字段**(臂按
`speed_percent` 走,手近乎瞬时),所以它只是**本地节拍**,不是下给硬件的时长。于是每步
发完等够 hold 后量一次 `|遥测 − 目标|`,超 0.05 rad(≈2.9°)就报 `lag` 事件,并在
`done` 里带 `worst_lag_rad` / `lag_exceeded`。理由见 `COMBO_DEBUG.md`:共享时间轴只
保证命令一起发出,不保证硬件一起到位,静默吸收落后就是**假同步**。mock 联调实测
`go_home` 3 秒档收尾还差 0.11 rad,检测确实会报。

⚠ **手没有急停通道**(`hand_console.py` 里 estop 出现 0 次)。最接近的 `action_stop`
会把手**移动**到张开位 —— 那是运动不是停止,不能当急停用。所以 estop 只做两件事:停下
发循环 + 给臂发 estop,手保持当前位置,并在事件里明说,不假装手也停了。
`POST /api/voice/estop` 是**绕过执行队列直发臂**的:排在 SSE 后面等的话,长轨迹里要等
到下一步边界才生效 —— 那正是急停不能接受的延迟。

端点:`/api/voice/phrases`(能说什么)、`parse`(只解析)、`invoke`(执行)、
`stop`(停下发,**≠急停**)、`estop`(直发臂)。

### 两个语音面板 · 作用域

页面上有**两处**语音栏,共用同一份前端 JS(靠 `vc` / `hvc` 元素前缀区分)和同一套
后端闸,只有**作用域**不同:

| 位置 | `scope` | 能说什么 |
|---|---|---|
| 实时 Live · 语音 | `all` | 清单里全部技能(臂 + 手)**+ 手势包 + 臂手联合录制包** |
| 灵巧手调试 · 语音 | `hand` | 只有 `devices == {hand}` 的技能 + 手势包 |

手页刻意不列臂的技能:那页只有 RS485 一条通道,说了也只会被拒。也没有急停按钮 ——
手没有急停通道,放一个只会给人错觉。

⚠ **联合录制包(`combo_pack`)只进 `all`,不进 `hand`**。它会动臂,而 `hand` 作用域的
整个前提就是"手页说的话不该动臂"。开关是 `_list_pack_targets(include_combo=)`,
默认**不放** —— 默认值站在更安全的那一边,要放得显式要求。

> **2026-08-07 修**:combo 包原来**两个作用域都没进** —— `_list_pack_targets` 只扫
> `gesture_pack`,于是录好的「挥手」说了永远 `no_match`。同一天还修了它进池之后暴露的
> 一串 kind 判定漏改,见下面「三种 kind」。

**手势表 ≠ 技能包**,两者都能被语音命中但机制不同:`gestures.yaml` 在**加载期合成进
清单**(所以 `composite` 能引用它、`scope=hand` 自动包含它);技能包是磁盘上的独立文件,
作为**另一个目标池**参与同一次打分。想让一个手势能被组合引用,写手势表;想随时增删
不重启,用技能包。

**两种包,别混**。都作为磁盘文件参与同一次打分,但动的设备、沙箱根、播放器都不同:

| `kind` | 目录 | 动什么 | 播放器 | 进度看哪 |
|---|---|---|---|---|
| `gesture_pack` | `data/gestures/` | 只有手 | `hand_console.ActionPlayer` | `/ws/hand` 推,手页「技能包」栏 |
| `combo_pack` | `data/combos/` | **臂 + 手** | `arm_console` 里的 CPV 播放器 | 轮询 `/api/combo/play/status`,合体页回放栏 |

只放 `mode == "keyframe"` 的 combo 包进池。`stream` 包页面上放不了(要 CPV 逐关节
伺服),进了池就是"说得出来但一执行就报错" —— 那比 `no_match` 更让人困惑。

> **2026-08-06 修**:`all` 作用域的包列表原来写死成空 `[]`,于是合体页说包名
> **永远 no_match**。那不是安全考虑,是当初 Live 页只管臂时留下的;合体页现在有手的
> 通道,包是手势、它管得着。改在 `_scope_targets()`。

**技能包同池打分**。包是磁盘上随时增删的文件,不进 `registry.yaml`(清单是静态真源,
它的别名撞车校验建立在"内容固定"上)。所以包作为独立目标池参与**同一次打分** ——
同池是关键:包名和技能名撞车时才判得出 ambiguous。实测录一个叫「握拳」的包,说
「握拳」会返回两个候选(技能 `hand_close` + 那个包)让人点,而不是某一池悄悄赢。
两个同名包(不同目录)时,候选带的是**路径**而不是名字,否则按名字再查一次仍然
ambiguous,用户永远点不出结果。

**包的执行不走 SSE**,返回普通 JSON:回放在 console 进程内异步播,这一层拿不到逐帧
进度(进度看上面那张表各自的栏目)。硬编成 SSE 只会造出一个「立刻 start 紧接 done」的
假进度流。但它**仍然过确认闸、仍然落调用日志**
—— 不让前端直接打 `/api/hand/gesture/play` 或 `/api/combo/play` 就是为了这两件事:
`(原话, 包路径)` 的配对和技能那边一样是 VLA 标注原料,不能因为"这是包不是技能"就漏掉。
路径沙箱校验复用各自的 `load_pack`(内含 `resolve_pack_path`),不另写一份 —— 实测
`../../etc/passwd` 被拒。

⚠ **`kind` 由后端自己从池里反查,不信前端传的那个**。两个沙箱根可以各有一个同名文件,
信前端就会去错的根 load。前端传的 `kind` 只用来决定走不走包这条路,具体哪种由 `path`
在池里的归属定。

### 三种 kind:判「是不是包」只走 `PACK_KINDS`

`kind` 有**三个**值:`skill` / `gesture_pack` / `combo_pack`。判据集中在两处常量
(`skills/intent.py`),**别在业务代码里写 `== "gesture_pack"`**:

```python
PACK_KINDS   = ("gesture_pack", "combo_pack")
PACK_DEVICES = {"gesture_pack": ["hand"], "combo_pack": ["arm", "hand"]}
```

两份**刻意放一起**,还有 `assert` 咬着覆盖关系:加第四种包时只改 `PACK_KINDS` 能跑通
(判「是不是包」不报错),但设备表查不到 —— 而 `devices` 是确认框上「会动臂」那句提示的
**唯一来源**,报错方向的风险提示比不报更糟(人白清一次场,久了就不信这个提示了)。

前端拿不到 Python 常量,只能抄一份(`VC_PACK_KINDS` / `VC_PACK_DEVICES` +
`vcIsPack()` / `vcIsCombo()`)。`test_voice_combo_kind.py` 里有一条测试**对着两边比**,
抄漏了会红;另两条扫源码,在代码行里写裸字符串也会红。

**2026-08-07 实测踩过**:combo 包刚进池,7 处判据只认 `gesture_pack`,于是它全落到
"技能"那一边。症状五花八门,根因是同一个:

| 位置 | 表现 |
|---|---|
| `voice_parse` | **HTTP 500 + 纯文本** `Internal Server Error` |
| `voice_invoke` | 报「查不到这条技能」—— 理由和真实原因完全不搭 |
| `voice_phrases` | 「能说什么」把挥手标成**只动手** |
| `vcRun` | 拿 SSE reader 读 JSON 响应,进度栏一行不出 |
| `vcShowCands` | 歧义里点中 combo 包 → 送去手的那条路 |

那个 500 值得记一下**怎么误导人的**:combo 包的 `skill_id` 是 `None`(包不在清单里),
落到 else 就是 `console_targets(reg.get(None), reg)`,而 `targets()` 第一行就是
`spec.kind` → `AttributeError`。FastAPI 回的是纯文本,前端 `r.json()` 拿它去 parse,
报 **`SyntaxError: Unexpected token 'I'`** —— 症状指着前端的 JSON 解析器,defect 在服务端。
`out/voice_parses.jsonl` 是关键证据:`log_parse` 在崩之前就跑完了,所以**日志里记着一条
解析成功**,而客户端拿到 500。

### 力度修饰:正交维度,不是序列里的一步

「轻一点捏」「用力握拳」这类**修饰 + 动作**的说法,靠 `intent._extract` 剥修饰词、
`_apply_mods` 落到技能声明过的参数上 —— 和「慢一点」同一套机制。

**为什么不做成 `hand_close_soft` 这种组合技能**:力度是"怎么做"、不是"做什么",
它对任何手部动作都适用。做成修饰词则 3 档 × N 个动作只要 N 条清单条目;
做成组合技能就是 3N 条,组合爆炸。而且 `console_exec.translate()` 已保证力控排在
角度**之前**发,时序天然正确 —— 力控是状态,先发角度会让这次运动用上一条的阈值。

档位值是**实测常数**(见 `HAND_DEBUG.md` 力控语义那节),两处必须一致:
`intent.GRIP_LEVELS` 和清单里 `hand_grip_*` 三条。数值分叉就会出现「轻一点」和
「轻一点捏」力度不同这种说不清的行为。

⚠ 同一批词**既是修饰词又是别名**,这是有意的:

- 只说「轻一点」→ 第 1 步整句精确匹配 → `hand_grip_soft`(只调力度、不动作)
- 说「轻一点捏」→ 整句匹不上 → 剥掉修饰 →「捏」→ `hand_pinch` + soft 力度

第 1 步只匹配**整句**,所以两条路不打架。少了任一边都缺一种说法:不在别名表里
→ 单说「用力」变 no_match;不在 `SOFT_WORDS/FIRM_WORDS` → 「轻一点捏」只调力度不捏。

已知缺口:单字「轻」不在词表里(加进去会和「轻轻」重叠),所以「又轻又用力地捏」
只认出 firm 而不是判成冲突。同时说轻和重时**两个都丢**并提示 —— 猜错的后果是
握持力反向。

### 漏词落盘:唯一的真实语言样本来源

`/api/voice/parse` 每次解析都写一行到 `out/voice_parses.jsonl`,**成功和失败都记**。

**为什么成功也记**:只记失败的话能知道"漏了 50 条",但不知道是 50/60 还是
50/5000 —— 漏词率算不出来,而那正是判断"要不要上更强匹配"的唯一依据。

**为什么记在 endpoint 而不是 `intent.parse()` 里**:那个函数被测试和
`intent.py --all` 调用(一次跑几千条),记在里面会把日志灌满合成数据。

读的工具:

```bash
python3 src/analyze_voice_misses.py            # 漏词率 + 该补什么
python3 src/analyze_voice_misses.py --all-sources   # 含自检流量
```

它把漏词分**两类**,因为处理方式完全不同:

- **差一点就中**(最高分接近阈值)→ 补一条别名就解决,**不用动模型**
- **完全不认识**(最高分很低)→ 才需要更强的语义匹配

如果九成漏词是前者,正确答案是补清单。这个区分是整个工具的重点。

按 `source` 分开算漏词率(`text` 打字 / `asr` 语音):打字的漏是"清单没覆盖",
语音的漏可能是"听错了",两者修法完全不同。自检流量要显式传 `source=selftest`,
分析时默认排除 —— 它和真人打字的记录**长得一模一样**,混进去漏词率就是假的。

⚠ 里面是原始语音/文本内容,只留在本地 `src/out/`。

### 手势规格层(`skills/hand_pose.py`)

**为什么要它**:清单原来直接写 `hand: [1.112, 0.600, 1.07, 0.0, 0.0, 0.0]`。
这串数字看不出是什么手势、改一个不知道会不会撞、每个新手势要上真手量一遍。

现在写成五指语义:

```yaml
pose:
  thumb: opposed              # 对掌位(2 个关节一起设)
  index: limit                # 闭到"刚够碰上拇指"(自动推导,不用量)
  # middle/ring/pinky 省略 = 张开
```

- **归一量 `n ∈ [0,1]`**:0=张开,1=这台机器实际能到的最闭(拇指弯曲 URDF 限 0.6 < 实际 0.698,所以 n=1 → raw 141 不是 0)。
- **四指**:直接给 n,或状态名 `open/relaxed/half/curled/closed`。
- **拇指**:两个关节成对给,状态名 `open/up/opposed/folded/side`,或 `[yaw_n, pitch_n]`。
- **`limit` 是推导状态**:查可行域表算出"食指在当前拇指位置下能到的最闭位 + 10 counts 余量"。
  写 `index: limit` 的手势(捏/OK/比 1)不用上手量,改拇指姿态食指自动跟。

**可行域强校验**:不可行 = **加载失败**,不是警告。互顶姿态会堵转过温(Bit1 不可清),
宁可加载时炸。两条实测通过的清单已改用 pose(捏用 `limit`,握拳保留手写余量 `0.68`)。

加载期 schema 展开 pose → action.hand,下游(backend/exec/runner/前端)零改动。

### 手势表(`skills/gestures.yaml`)

纯造型手势不写在 `registry.yaml` 里。原因:手势之间**只有 pose 不一样**,
其余十几行样板(aliases/params/requires/safety/action)全同。写进清单等于把样板
抄 N 遍,加第 6 个手势要复制粘贴 15 行。

手势表里一个手势 = 两行:

```yaml
gestures:
  two:
    name: 比个2
    aliases: ["比个2", "比个二", "比2", "剪刀", "耶"]
    pose: {thumb: folded, index: open, middle: open, ring: closed, pinky: closed}
```

加载期合成成 `primitive` 技能(id = `hand_<key>`,如 `hand_two`),然后进同一份清单。
所以合成条目:

- 语音能直接命中(和手写技能同池打分)
- `composite` 的 `steps` 能引用
- 走**同一条**校验路径 —— pose 展开、可行域、别名撞车都查,不是特殊通道

样板由 `defaults` 统一给,单个手势写同名键即可覆盖。现有 9 个:
1/2/3/4/5、点赞、OK、石头、指。石头剪刀布复用其中三条,不重复定义 pose。

> 手势的 `hand_force` 默认 250,低于抓握的 300。手势不需要握力,阈值低意味着
> 碰到障碍更早停 —— 造型宁可停在半路,不要顶着东西继续弯。

### overlay:让臂和手同时动

`composite` 默认 `mode: sequence`(逐条按序发)。写 `mode: overlay` 则把子技能
**合成一条指令**同时发:

```yaml
- id: home_with_one
  kind: composite
  mode: overlay
  steps:
    - {skill: go_home}      # 臂回零 5.0s
    - {skill: hand_one}     # 手比 1  1.5s
```

**为什么是真并发**:臂和手是两个 console 进程,`translate()` 产出的两条分别写
各自的 stdin。臂的 `move_j` 是阻塞调用,但它阻塞的是**臂那个进程**,手照样动。
所以总时长取 `max(5.0, 1.5) = 5.0` 而不是相加。合成后下发顺序仍是
`arm angles → hand speed → hand force → hand angles`,力控排在角度前。

**约束在构造期报错**,不等真机:

| 约束 | 为什么 |
|---|---|
| 只收 `primitive` 子技能 | 轨迹几百帧、嵌套组合步数不定,"恰好一步"验不了 |
| 每个通道单来源 | 两条都给 `hand` 取谁都是猜 |
| 不许带 `estop`/`action`/`value` | 模式切换不是姿态。急停尤其要单独立刻发 |
| `duration` 取 max | 并行跑,要等慢的那个走完 |

> 反例记下来防止再试:`hand_grip_soft` + `hand_one` 做「轻轻比个1」**会被拦** ——
> 两条都声明了 `hand_speed`/`hand_force`。那个需求归修饰词机制(`intent.SOFT_WORDS`),
> 不归 overlay:力度是正交维度,不是序列里的一步。

**核对表**:`/usr/bin/python3 skills/hand_pose.py --verify` 检查本模块抄的
RAW_MAP / HAND_LIMITS 跟 inspire_hand 是否一致。换 URDF 改了那边就跑一次。

### 网页下发会不会"丢包"

**下发方向不会丢,反馈方向会。**

下发链路:浏览器 `fetch` → TCP → uvicorn → 专用线程 → `console.stdin.write` + `flush`
→ console 的 `readline()` 循环 → CAN / RS485。每一环都不静默丢:TCP 丢包自己重传,
断了就是 `fetch` 抛异常;第二条指令撞上单实例锁拿到的是 **409**(明确拒绝,不是排队
也不是丢弃);`stdin` 管道满了是**背压**(写阻塞),不是丢帧 —— console 逐行读,
`move_j` 是阻塞调用,所以指令严格串行;写失败被 `try/except` 接住变成 `warn`/`error`
事件推回页面。

反馈方向不同:SSE 是**一次性流,没有重放**。切页 / 刷新 / 断网 / 合盖之后,后面的
`progress`、`lag`、`done` 就永久看不到了。真正危险的不是"少看几条进度",而是
**丢了反馈不等于臂停下**:worker 线程还活着,还在往 stdin 写帧。所以断流时必须
`ex.stop()`,只清单实例锁是不够的 —— 清了锁下一次 invoke 立刻能进来,两个 executor
同时写同一个 stdin,console 收到的是两条轨迹交错拼出来的第三条(COMBO_DEBUG 说的
同通道双写)。实测:go_home(预计 5.0s)读 1.5s 掐断,调用日志记 `result=stopped`
而非 `done`,1.8s 后下一条 invoke 正常进入。

还有一类**不叫丢包但看着像**:指令发太快,臂还没走到就被新目标顶掉。这是覆盖不是
丢失,由落后检测(`|遥测 − 目标| > 0.05 rad`)报出来,不静默吸收。遥测广播
(`_broadcast`)队列满时**故意丢旧帧**,那是显示路径 —— 实时显示要最新值,不要积压。

### 从另一台电脑访问(局域网)

WSL2 是 NAT 网络:本机 `eth0` 是 `172.25.188.158/20`,网关 `172.25.176.1`。这个网段
局域网里别的机器路由不到。`uvicorn` 绑的是 `0.0.0.0:7860`,但那个 `0.0.0.0` 只在 WSL
内部有效 —— Windows 只做了 `localhost` 转发,**没有**把 7860 摊到局域网网卡上。
实测从 WSL 探网关侧:`2223` 开(SSH 有 portproxy,所以你能 ssh 进来),`7860` 不通。

推荐走 **SSH 隧道**,不用改 Windows 任何东西(而且本机 WSL interop 关着,改也得手动):

```bash
# 在另一台电脑上执行(2223 就是你现在 ssh 用的端口)
ssh -N -L 7860:localhost:7860 -p 2223 zhang123@<这台机器的局域网IP>
# 然后在那台电脑的浏览器打开 http://localhost:7860
```

这条路顺带解决 ASR 的卡点:客户端看到的是 `localhost`,**算安全上下文**,
`getUserMedia` / Web Speech API 不再被拒。而且臂的控制面板不会裸摊在局域网上 ——
这个 web 端**没有任何认证**,谁能连上谁就能使能机械臂。

```bash
python3 src/skills/intent.py --all                 # 178 项别名回归
python3 src/skills/intent.py "回零位慢一点"
python3 src/skills/intent.py --packs "OK"          # 把磁盘上的技能包也放进池子
python3 src/skills/console_exec.py --skill prepare_arm --confirmed   # 假 console 干跑
python3 src/skills/test_intent.py                  # 63 项(含技能包同池)
python3 src/skills/test_console_exec.py            # 61 项
python3 src/skills/test_hand_pose.py               # 75 项(姿态/可行域/手势表/overlay)
python3 src/skills/hand_pose.py --verify           # 核对抄来的驱动表有没有漂
python3 src/test/test_voice_combo_kind.py          # 9 项:三种 kind 的分流 + 前后端常量对齐
```

⚠ `test_voice_combo_kind.py` **不碰硬件**,真机接着的时候也能跑:parse/phrases 只读清单
和磁盘;invoke 那条**故意不传 `confirmed`**,在确认闸就返回,一帧都不下发。用「被闸拒」
证明「路由对了」,比真执行一次安全得多。它用 `TestClient` 起 app 但**不拉 console**
(`_console_ready()` 直接读 `_arm`/`_hand` 全局,都是 `None` 就报未接入),所以不抢
`can0` / `ttyUSB0`。

**ASR 还没接,且有个前置卡点**:WSL 里没有 ALSA / PulseAudio(`/proc/asound/cards`
不存在),服务端录不了音,只能浏览器采集。但 `app_web` 启动横幅让你用 WSL IP 打开,
那是**非安全上下文**,`getUserMedia` 与 Web Speech API 都会被浏览器拒。要接麦克风得先
从 Windows 开 `http://localhost:7860`(WSL2 有 localhost 转发,localhost 算安全上下文)
—— **这条尚未实测**。接上之后只需把识别文本填进同一个输入框,解析与执行两层都不用改。

## 生成物(不进仓库,可重建)

`assets/assembled/nero_inspire_right.urdf`(装配)、`datasets/captures/capture_*`(正式 Capture 数据)、
以及仅兼容保留的 `src/out/*` 旧产物。真实 Capture 和可重建输出不进入 Git。
