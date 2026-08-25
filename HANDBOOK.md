# 开发手册

本手册说明 `lerobotTest` 的当前开发边界。硬件参数看 [HARDWARE.md](HARDWARE.md)，
部署 MCP/Bridge 看 `/home/zhang123/ros2_ws/robot-mcp-server`。

## 仓库边界

```text
ros2_ws/
├── lerobotTest/              本仓库：驱动、Web、仿真、标定和 VLA 数据管线
├── robot-mcp-server/         现行 MCP Server + 可部署 robot-bridge
└── src/nero_inspire_ros2/    ROS2 包（独立仓库）
```

本仓库根目录 `bridge.py` 与 `mcp_server/` 是拆仓前快照。允许用于历史比较，但新功能、
部署修复和文档应先落到 `robot-mcp-server`，再按需同步共享驱动。

## 关键模块

| 文件 | 职责 |
|------|------|
| `src/paths.py` | 本仓资产路径常量 |
| `src/inspire_hand.py` | RH56DFX RS485 驱动和运行时限位 |
| `src/hand_feasibility.py` | 通用灵巧手 commissioning、Profile 和安全投影 |
| `configs/hands/*.json` | 手型资产标称角、Adapter 和探测策略 |
| `src/nero_arm.py` | NERO CAN/SDK 封装和运行时限位 |
| `src/nero_arm_bridge.py` | ROS2 硬件桥；真机默认只监控 |
| `src/hand_console.py` | 灵巧手调试和动作播放 |
| `src/hand_target_filter.py` | 摄像头真手目标的 One Euro 滤波和分辨率门限 |
| `src/hand_target_mailbox.py` | 摄像头 latest-target 调度、控制权和 ACK 背压 |
| `src/arm_console.py` | 机械臂调试，默认 mock 和低速 |
| `src/live_wrist_tracking.py` | 单目腕姿、联合锚定、相对映射和末端限幅 |
| `src/live_ik_worker.py` | 与轻量 Web 环境隔离的 NERO FK/IK 子进程 |
| `src/live_ik_scheduler.py` | 单 worker、latest-only 实时 IK 调度和过期结果失效 |
| `src/camera/` | 相机 Adapter、棋盘格检测和只读式 NERO 手眼标定 |
| `src/app_web.py` | Web 工作台和 WebSocket 后端 |
| `src/web/hand_tracker_tasks.js` | MediaPipe Tasks/Legacy 统一追踪适配层 |
| `src/web/combo_camera.js` | 合体页摄像头与锚定/冻结单按钮状态机 |
| `src/skills/` | 技能清单、执行后端和安全闸 |
| `src/build_nero_inspire.py` | 生成臂、法兰、手装配 URDF |
| `src/build_combo_viz.py` | 生成本地 Web 合体模型 |
| `src/lerobot_v3/` | Python 3.12 + LeRobot 0.6.1 主运行时入口 |
| `src/capture_bundle.py` | Capture 路径、版本目录、manifest、checksum 和血缘入口 |
| `src/quality_profiles.py` | 质量 profile 校验、阈值比较和 Capture 快照入口 |
| `src/compare_dataset_numeric.py` | 新旧 LeRobotDataset 数值列等价比较 |
| `src/test/` | 离线单元与结构测试 |
| `src/test/hardware/` | 需要真机、CAN 或串口的显式测试，不自动收集 |
| `third_party/` | 上游源码、厂商 SDK、外部数据和项目 overlay |

## 开发环境

命名 Conda 环境 `lerobot-v3` 是主运行时，覆盖 Web、RGB/RGB-D EGO、RobotDataset、
实时/离线 IK、Rerun、直接 CAN/串口控制和离线测试。已验收版本为
Python `3.12.13`、`lerobot 0.6.1`、CPU Torch `2.7.1`、`torchcodec 0.4.0`、
MediaPipe `0.10.14`、Pinocchio `4.1.0`、dex-retargeting `0.5.0` 和 Rerun `0.26.2`：

Web 实时遥测还固定使用 `websockets 16.1`。仅安装裸 `uvicorn` 不包含 WebSocket 协议
实现，会让 `/ws/hand`、`/ws/arm` 返回 404，表现为真机能动但 Three.js 不跟随。

```bash
conda create --override-channels -c conda-forge \
  -n lerobot-v3 python=3.12 pip -y
conda activate lerobot-v3

python -m pip install \
  torch==2.7.1+cpu torchvision==0.22.1+cpu \
  --index-url https://download.pytorch.org/whl/cpu

python -m pip install \
  -r environment/lerobot-v3-dataset.txt
python -m pip check

python src/lerobot_v3/app_web.py
```

ROS Humble 是唯一例外。Ubuntu 22.04 的 `rclpy` 是 CPython 3.10 扩展，因此使用独立的
`ros-humble` 薄环境，只包含 ROS reader/writer/runner 与硬件 SDK 所需的纯 Python 依赖：

```bash
conda create --override-channels -c conda-forge -n ros-humble python=3.10 pip -y
conda run -n ros-humble python -m pip install -r environment/ros-humble-bridge.txt
python src/ros_humble_env.py --check
```

`app_web.py` 自动在后台为 ROS 子进程加载 `/opt/ros/humble/setup.bash` 和工作区
`install/setup.bash`，用户不需要切换环境。手动启动桥时也从 V3 入口转交：

```bash
python src/ros_humble_env.py --run src/nero_arm_bridge.py --mock --enable-control
```

不要用是否能 import 来推断真机已经可用；CAN、串口、使能、急停和工作区仍需单独确认。

严格验证时给 Hugging Face 指定可写缓存，并显式离线运行：

```bash
HF_HOME=/tmp/lerobot-v3-hf \
HF_DATASETS_CACHE=/tmp/lerobot-v3-hf/datasets \
HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 \
python src/lerobot_v3/verify_dataset.py \
  --capture-root datasets/captures/capture_<id> --canonical --strict-v3

python src/lerobot_v3/verify_dataset.py \
  --capture-bundle --capture-root datasets/captures/capture_<id> \
  --json datasets/captures/capture_<id>/reports/capture_integrity_report.json
```

新生成 Capture 已通过官方 `LeRobotDataset` 回读和 `--strict-v3`。旧 `lerobot 0.4.4`
数据也能被 0.6.1 回读，但其 `tasks.parquet` 可能只有匿名索引列，不满足严格 v3 交付校验；
保留旧数据原样，不做就地修补。

四个 LeRobot 写盘入口都在 `save_episode()` 后显式调用 `finalize()`，所以构建命令返回时
Parquet footer、视频编码和元数据已经落盘，不依赖 Python 进程退出时的析构兜底。

## Capture 数据路径

三个 `build_canonical*` 入口默认各创建一个
`datasets/captures/capture_<YYYYMMDD>_<sequence>_<uuid>/`。后续命令不传路径时读取最新 `ready`
Capture；需要绑定指定批次时，整条链都传相同的 `--capture-root`：

```bash
python src/lerobot_v3/build_canonical.py --capture-root datasets/captures/capture_<id>
python src/lerobot_v3/derive_embodiment.py --capture-root datasets/captures/capture_<id> --robot nero_inspire --emit-traj
python src/lerobot_v3/measure_acceptance.py --capture-root datasets/captures/capture_<id> --robot nero_inspire
python src/lerobot_v3/verify_dataset.py --capture-root datasets/captures/capture_<id> --canonical
python src/lerobot_v3/replay_rerun.py --capture-root datasets/captures/capture_<id> --robot nero_inspire --serve
```

`<capture>/ego/` 与
`<capture>/robot_datasets/<target>/target_revision_v001/retarget_v001/` 分别是可加载数据集；
轨迹在 RobotDataset 的 `exports/workbench/`，验收汇总在 Capture 的 `reports/`。Web 完整
生成会创建并锁定一个 Capture，服务重启后默认选择最新 `ready` Capture。

`source/stream_index.parquet` 记录原始帧到 Ego 帧的可空映射及时间来源。当前旧视频/RGB-D
帧集没有硬件时间时使用 `fps_derived`，只表示处理时间轴，不作为相机同步精度证据。
该宽表服务当前单 RGB-D；眼镜、VIO/IMU、腕部设备和外部真值的异步多流将使用
`streams.parquet`/`samples.parquet` 长表并保留 `stream_index.parquet` 兼容视图。当前已提供
`write_multisensor_source_index()` 写入与基础校验，设备 Adapter 和 Source -> Ego 对齐尚未
接入；实施顺序与质量闸见 [EGO_DATA_STANDARD.md](EGO_DATA_STANDARD.md)。

质量口径位于 `configs/quality_profiles/*.json`，构建时完整复制到
`source/quality_profile.json`，同一 Capture 不允许静默替换。三个构建器的默认 profile 分别是
`legacy_rgb_video_30hz_v1`、`legacy_aligned_rgbd_30hz_v1` 和
`processed_observations_v1`。正式 60 Hz RGB-D 目标使用：

```bash
python src/lerobot_v3/build_canonical_from_rgbd.py --input-root <rgbd_root> \
  --quality-profile ego_fixed_rgbd_60hz_v1
```

该命令只选择验收口径，不会把 30 Hz、低分辨率或无硬件时间戳的输入变成达标数据。
`measure_acceptance.py` 默认读取 Capture 快照；对外部/旧根可显式传 `--quality-profile`。
当前 profile schema 1.1 将绝对精度、稳定性代理、连续性、时序和模型拟合分开。报告中的每项
都带 `measurement_class`、`measurement_basis`、`ground_truth_required` 和
`ground_truth_available`。手腕绝对位置误差只有在数据包含 `ground_truth.wrist_pose` 时判定；
静止抖动只有在存在 `annotation.wrist_stationary` 连续静止段时判定。缺失证据时 `pass=null`。

Ego 和 RobotDataset 会为每个 episode 创建 `annotations/episode_*.json`。生成器只补缺失文件，
不会覆盖人工审核。RobotDataset 同时生成 `qa/episode_*.json`：帧索引和数值有限性自动判定，
需要机器人模型或真值的限位、碰撞和指尖误差保持 `not_evaluated`。

`ego/meta/coordinate_system.json` 是坐标语义唯一入口。2.0 契约分别声明
`observation.wrist_pose`、3D 关键点和 2D 关键点所在 frame；固定相机 RGB 使用
`episode0_camera`，经 `camera_to_world` 外参变换的 RGB-D 使用 `scene_world`。外部处理结果可
内嵌 `wrist_pose_frame`，也可传 `--wrist-pose-frame`；旧文件缺省导入会明确标记
`compatibility_default_episode0_camera`。禁止根据 `source_kind` 或目录名推断坐标系。

旧 `src/out/` 只通过各命令的 `--legacy-out` 或 Web 的 `VLA_LEGACY_OUT=1` 读取。禁止把
`--legacy-out` 与 Capture/显式输出路径混用，也不要手工把旧目录伪装成新 Capture。正式数据
目录被 `.gitignore` 排除；只提交目录契约文档，不提交采集内容。

## 常见改动

### 修改灵巧手映射或限位

同步检查：

1. `src/inspire_hand.py` 的 `HAND_JOINTS`、`HAND_LIMITS`、`RAW_MAP`
2. `src/skills/hand_pose.py` 的复制表
3. `src/ros_joint_writer.py` 与 `src/build_inspire_from_vendor.py`
4. `assets/hand/urdf/inspire_hand_right.urdf`
5. 浏览器模型和 retargeting 配置
6. `HARDWARE.md`

当前资产标称基线为拇指弯曲 `0.48`、四指 `1.333`。修改后必须运行：

```bash
python3 src/skills/hand_pose.py --verify
python3 -m pytest src/test/test_hand_limit_consistency.py -q
```

两项都必须通过。资产标称 rad 与设备条件化 raw 包络是不同契约；禁止用未完成或
`aborted` 的真机 Profile 改写跨设备模型限位。

跨手型抽象统一采用 datasheet/URDF 的资产标称 rad，不要求逐台用外部量角器重建模型角。
设备 commissioning 只验证在指定速度、力、固件和路径下的 raw/归一量可行包络。型号规范
在 `configs/hands/`，执行和 Profile 契约见
[src/HAND_FEASIBILITY_AUTOMATION.md](src/HAND_FEASIBILITY_AUTOMATION.md)。先用 Mock 验证状态机：

```bash
python3 src/hand_feasibility.py --phase all \
  --output /tmp/inspire_mock_profile.json
```

真机先做不写参数、不运动的只读预检：

```bash
python3 src/hand_feasibility.py \
  --adapter inspire --hardware --phase preflight \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --output reports/hand_feasibility/inspire_preflight.json
```

真机运动还必须显式提供 `--allow-motion CONFIRM_HAND_MOTION`，并按 `single`、
`interactions --resume` 分阶段执行；真机禁止 `--phase all`。Profile 必须匹配型号、URDF
SHA-256、设备/固件（可读取时）和完整探测条件，Mock Profile 默认禁止用于真机。
当前自动化没有直接修复旧 `hand_pose` 表，也没有接入 Web/Bridge 运行时安全闸。

### 修改装配位置

当前参数定义在 `src/build_nero_inspire.py`：

```python
FLANGE_MOUNT_XYZ = "0 0 0.016489"
FLANGE_MOUNT_RPY = "0 0 1.570796"
MOUNT_XYZ = "0.000042 0. 0.002158"
MOUNT_RPY = "0 0 1.570796"
```

修改后执行：

```bash
python3 src/build_nero_inspire.py
python3 src/build_combo_viz.py
```

不要依赖旧迁移报告里的代码行号或 `MOUNT_XYZ/MOUNT_RPY`；那些文档保留的是当时状态。

### 修改 Web 摄像头链路

现行前端使用本地 vendored MediaPipe Tasks。合体页按客户端浏览器实际能力生成推理设备
清单：CPU 对应 WASM，GPU 对应 WebGL；macOS 的 Apple GPU 仍是 MediaPipe GPU delegate，
由浏览器把 WebGL 映射到 Metal，不是独立 MPS delegate。显式选择 CPU/GPU 时初始化失败会
直接报错，只有“自动选择”按 Tasks GPU → CPU → Legacy 降级。浏览器传输采用单帧在途和
latest-frame-wins；WebSocket 失败时 HTTP 只维持
retarget 与 3D 预览，不驱动真手。

后端硬件路径由 30Hz `LatestTargetMailbox` 调度：最多一个待发目标、一个
`hand_console` ACK 在途，新帧覆盖待发旧帧，WebSocket 断开清理控制权。不要重新增加
“WebSocket 返回关节角后再逐帧 POST `/api/hand/command`”的第二条硬件链。

retarget 后的六关节真手目标先经过 `OneEuroJointFilter`。现行参数为
`min_cutoff=1.5Hz`、`beta=2.5`、导数截止 `1.0Hz`，最小命令变化量为
`0.0005rad`。不要把该门限放大成普通位置死区：首轮 `0.015-0.02rad` 死区会积累
滤波尾差并在保持姿态时释放成可见台阶。3D 预览保持原始 retarget 结果。

修改后至少运行：

```bash
node src/test/web/hand_tracker_tasks.test.mjs
node src/test/web/hand_mimic_transport.test.mjs
python3 -m pytest src/test/test_hand_target_mailbox.py
python3 -m pytest src/test/test_hand_target_filter.py
python3 -m pytest src/test/test_hand_console_ack.py
python3 -m pytest src/test/test_stdin_lines.py
```

浏览器实测还应覆盖：启动、停止、重复启动、权限拒绝、WebSocket 断线恢复和页面切换。
切换顶层功能页时必须先等待旧页完成释放：手张开后断串口；臂在线、已使能且未冻结时回
七关节全零位，再断 CAN。回位失败或超时也必须释放通道。浏览器关闭/刷新走
`pagehide -> sendBeacon('/api/hardware/release')`，不要改成无法保证送达的普通异步请求。
`sendBeacon` 仍只是浏览器尽力通知：进程强杀、主机断电和断网可能使请求无法到达后端。
服务端使用按标签页 owner 的硬件租约补足这条路径：页面每 2 秒调用 heartbeat，租约默认
8 秒。硬件 start/stop/command 和 release 请求必须携带 `X-Hardware-Lease`；当前 owner 有效时
其他标签页返回 409。正常主动释放仍复位手并让臂回零；watchdog 超时则保持最后位置，只
停止会话并释放串口/CAN，避免网络抖动触发新运动。此时手可能继续夹持；机械臂退出路径
不调用 `disable()`，只关闭当前 SDK/CAN 会话并保持原使能状态。watchdog 不是急停替代品。

### 修改合体实时跟随

实时跟随入口只在“实时 Live · 合体”页。浏览器同时发送 world landmarks 和 image
landmarks：world landmarks 用于六关节手型与手掌姿态，image landmarks 用于标记为
`monocular_scale` 的单目腕部相对位置；不要把后者当作绝对米制测量。

单一状态按钮按当前状态执行“联合锚定并跟随”“冻结跟随”或“重新锚定并跟随”。首个有效
手帧即可点击；后端先用机械臂当前实际/最新关节做 FK，再固定采集 12 个有效帧，并剔除位置/
姿态综合偏差最大的 25% 样本后生成联合锚点。抖动值用于页面质量提示，不再作为无限等待的
硬门槛，因此锚定帧本身不应产生目标跳变。腕部位置和姿态均以联合锚点为基准做相对映射；
位置默认 Mock 限幅是 `±50/50/30mm`，真臂路径统一收紧到位置各轴 `±20mm`。真臂姿态按
`±45/±25/±35°` 三轴限幅；Mock 根据准备位固定末端位置的 5° 步进 IK 扫描结果使用
X `-90/+60°`、Y `-115/+50°`、Z `-175/+155°`，并在测得边界内保留 5-10° 余量。
协议顶层的 `orientation_delta_deg` 与 `orientation_limited_axes` 可用于判断具体哪一轴触顶。
实时腕部姿态只保留 `回放一致` 链路：复用 MediaPipe/MANO `operator2mano` frame、
相机/world 轴增量和左乘，与 `derive_embodiment` 的 RGB 回放路径一致。相对旋转矩阵转欧拉角时
会依据上一帧选择最近的等价分支，因此翻掌越过 90° 或 `±180°` 表示边界时不会镜像跳变；
连续展开后仍须经过逐轴限幅和 IK。
Mock 从远离关节限位的弯肘中间姿态启动，避免伸直位附近的小抖动放大为 IK 不稳定。
Mock 不再自行生成正弦摆动，关节反馈只在收到目标后变化。
腕部三轴位置在进入相对映射和 IK 前经过 One Euro 滤波，默认参数为
`min_cutoff=1.2Hz`、`beta=0.5`、导数截止 `1.0Hz`。滤波只处理腕部位置，不替代灵巧手六关节
滤波；联合锚定、冻结、丢手、跟随异常或采样间隔超过 `200ms` 时会清空状态，避免恢复时
把旧帧尾差带入新跟随段。协议状态中的 `wrist_position_filter` 提供原始/滤后步长和是否重置。
单目深度使用 MediaPipe world landmarks 的当前掌宽/掌长作为每帧尺度，并按其 3D 轴向可见度
补偿图像投影缩短，避免翻掌时横向投影缩短被误判为手腕大幅远离。腕部姿态转换为旋转向量并使用 `min_cutoff=1.8Hz`、`beta=0.8` 的
One Euro 滤波，
再做姿态限幅；状态通过 `wrist_orientation_filter` 返回。这样可保留挥手/手心翻转等低频动作，
又不会把每帧姿态噪声直接放大到机械臂。
统一跟随准备位为 `[0, -0.7, 0.002, 1.298, 0.002, -0.008, -0.591] rad`，伸直位为
`[0, 0, 0, 0, 0, 0, 0] rad`。每次启动摄像头都会先下发准备位，每次关闭或启动失败都会先
清空 latest-target、退出 CPV，再回到伸直位。Mock 同步更新 Three.js 和 Mock 反馈；真臂仅在
勾选“允许真臂跟随”后执行，并等待实际关节误差小于 `0.03 rad`。两段均使用关节空间插值，
路径不受控，启动时必须确认沿途无障碍。
若本次摄像头会话启用了灵巧手跟随，关闭或启动失败回滚时会先清空灵巧手 latest-target、
等待在途帧结束并下发全张开位，然后机械臂才回伸直位；未启用灵巧手跟随时不会主动移动手。

Mock 与真机使用同一个 `/ws/hand/mimic` 协议、NERO IK worker 和 7+6 目标结构。真臂必须
在线、已使能、未冻结，并由页面显式勾选实时跟随授权；不要取消这个安全门。丢手、左右手
变化、连续 3 次 IK 失败、急停/冻结、未使能或断线时应停止投递并冻结，恢复后必须重新锚定。

实时链路按“最新状态控制”解耦：浏览器仍保持一帧 WebSocket 请求在途和一个待发最新帧；
后端完成一次 retarget 后先把灵巧手送入 30Hz mailbox，再把腕部目标送入单 worker IK。
IK 最多一个求解在途、一个 pending，新帧覆盖未求解旧帧；响应返回 `ik_queued`、`ik_replaced`、
`ik_pending`、`ik_in_flight`，以及带 `source_frame_id` 的最近完成机械臂结果。IK 输入超过
`180ms` 或完成结果超过 `200ms` 直接丢弃。锚点、授权或会话变化后，即使旧求解随后完成也
不得进入 arm mailbox。锚定时的一次 FK 仍允许同步等待，因为没有有效机器人锚点就不能映射。

与行业常见实时控制方式的对照如下：

| 做法 | 常见语义 | 本项目结论 |
|------|----------|------------|
| ROS 2 QoS `KEEP_LAST(1)` / Servo 命令超时 | 消费最新状态，过期命令不补跑 | 采用：浏览器、IK 和硬件 mailbox 都有界且带时效 |
| 工业周期控制 | 感知/规划与设备周期分离，各通道独立限频 | 采用：手 30Hz、臂 30Hz 独立，设备内部各自串行等 ACK |
| 无界 FIFO IK 作业队列 | 每帧最终都执行，适合离线作业 | 拒绝：实时控制会积压并补跑历史动作 |
| 每帧并发 IK | 吞吐可能更高，但完成乱序且共享种子/客户端竞态 | 拒绝：固定单 worker，结果顺序和资源所有权可证明 |

合体页默认灵巧手速度为 `1000`，机械臂为 `50%`；机械臂独立调试页继续保持 `20%` 默认值。

这条链路不是人体上肢的完整重建：当前输入只有手部 landmarks，位置是单目表观尺度的相对量，
末端姿态只在锚点附近有限跟随，也没有肩、肘、相机到机器人基座的真实外参。因此它适合小范围腕部平移/旋转和
灵巧手手指映射，不能宣称完全复制人的肩肘腕轨迹。要扩大到整条手臂，需要加入肩/肘姿态跟踪、
RGB-D 或多相机深度、外参标定、全上肢 retargeting，以及工作空间、关节限位、碰撞和奇异位形约束。

修改后至少运行：

```bash
python3 -m pytest src/test/test_combo_page.py src/test/test_hand_target_mailbox.py src/test/test_live_ik_scheduler.py src/test/test_live_wrist_tracking.py -q
node src/test/web/hand_tracker_tasks.test.mjs
node src/test/web/hand_mimic_transport.test.mjs
node src/test/web/combo_camera.test.mjs
```

还要在桌面和移动端检查合体 Three.js 非空、整臂入镜、控件无重叠。当前只完成 Mock 与
浏览器验收，任何真实机械臂运动都必须另行安排低速真机测试。

### 修改 MCP 或部署 Bridge

在独立仓库工作：

```bash
cd /home/zhang123/ros2_ws/robot-mcp-server
```

现行 MCP 入口为 `/mcp`，能力为手和臂，不包含 combo、视觉 mimic 或通用 `/execute`。
共享驱动变更应分别核对独立仓库 `robot-bridge/` 与本仓 `src/`，不要假定它们自动同步。

## 验证层级

按风险逐层执行，文档审查期间不运行真实运动命令：

```bash
# 只读/静态
python3 src/skills/hand_pose.py --verify

# Web 前端
node src/test/web/hand_tracker_tasks.test.mjs
node src/test/web/hand_mimic_transport.test.mjs
python3 -m pytest src/test/test_hand_target_mailbox.py
python3 -m pytest src/test/test_hand_console_ack.py

# 全部默认离线测试；pytest.ini 排除 src/test/hardware/
python3 -m pytest

# 独立 MCP 仓库
cd /home/zhang123/ros2_ws/robot-mcp-server
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s mcp_server/tests -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s robot-bridge/tests -v
```

真机验证必须显式安排，记录硬件、固件、速度、初始姿态和急停条件。mock 通过只能说明
软件链路可运行，不能证明真机移动、安全限位或接线正确。

## 提交前检查

- `git diff` 中没有凭据、私钥、生成缓存或无关用户改动。
- 行为变化有对应测试；硬件变化有明确的真机验证状态。
- 更新 `CHANGELOG.md`、`PROJECT_STATUS.md` 和相关操作文档。
- 新 MCP 事实来自独立仓库代码，而不是本仓内嵌快照。
- 历史文档只加状态说明，不重写历史事实。

---

**最后核对**：2026-08-21
