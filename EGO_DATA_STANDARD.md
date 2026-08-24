# EGO 数据采集与质量验收规范

本文定义本项目人类第一人称（EGO）手部操作数据的采集、处理、质检和交付契约。
目标产物是机器人无关的 EGO Canonical 数据集；机器人关节状态、动作和重定向结果属于
下游 RobotDataset，不得写回 EGO 母带。

本文与以下现行文档共同构成完整契约：

- [src/CANONICAL_SPEC.md](src/CANONICAL_SPEC.md)：Canonical 字段和数值语义。
- [datasets/captures/README.md](datasets/captures/README.md)：Capture Bundle 目录、生命周期和校验。
- [TODO.md](TODO.md)：尚未完成的设备接入和验收任务。

发生冲突时，四元数、坐标、时间和字段语义以 `src/CANONICAL_SPEC.md` 为准；目录和
生命周期以 `datasets/captures/README.md` 为准。

## 1. 适用目标线

EGO 指以操作者第一人称视角为核心的人类演示数据，不是任意相机视频的统称。最终目标设备
是带 RGB 或 RGB-D、IMU/VIO 的眼镜；固定 RGB-D 相机是前期生产和算法验证来源，腕部设备与
外部相机是增强、同步或 Ground Truth 来源。

```text
眼镜 RGB/RGB-D + VIO/IMU + 可选腕部设备/外部真值
                         |
                         v
               EGO Canonical 母带
                         |
                 retarget / IK
                         v
                   RobotDataset
```

Source 保存传感器事实，EGO 保存人类操作语义，RobotDataset 保存特定本体结果。三层之间只
通过显式血缘和版本化 Adapter 连接。

## 2. 实施阶段

| 阶段 | 范围 | 交付目标 |
|---|---|---|
| 0 | 设备准入、内外参、时钟和质量工具 | 可复现标定与只读采集报告 |
| 1 | 固定 RGB-D 裸手采集 | Source 原始流 + 单手 EGO v3 |
| 2 | 固定 RGB-D 批量采集 | 多任务、多场景、人工审核和数据切分 |
| 3 | 灵巧手离线重定向 | 独立 RobotDataset + 限位/连续性/仿真 QA |
| 4 | 臂手组合派生与真机执行数据 | 13 维状态/动作、物理 QA 和执行回灌 |
| 5 | 移动或头戴原型 | 动态 c2w、VIO/IMU 和 episode0 世界系 |
| 6 | 眼镜 EGO + 腕部设备 + Ego-Exo 真值 | 多传感器正式 EGO 数据生产线 |

阶段号表示能力递进，不允许用后处理补造前一阶段没有采到的深度、硬件时间或真值。

## 3. 数值与坐标约定

### 3.1 基本单位

| 数据 | 契约 |
|---|---|
| 浮点长度 | 米（m） |
| 原始整数深度 | `uint16` + 显式 `depth_scale_m_per_unit` |
| 角度 | 弧度（rad） |
| LeRobot `timestamp` | float32 秒（s），相对 episode 起点 |
| Source 硬件时间 | int64 微秒（us），保留设备原始分辨率 |
| 同步残差和延迟 | 毫秒（ms） |
| 图像 | 左上原点，`+u` 向右、`+v` 向下 |

### 3.2 相机坐标系

使用 OpenCV 右手坐标系：原点为三维光学中心，`+X` 向右、`+Y` 向下、`+Z` 沿光轴
指向场景。光学中心不是主点；主点是光轴在二维图像平面的投影，由内参 `cx, cy` 表示。

### 3.3 位姿

统一存储：

```text
[tx, ty, tz, qx, qy, qz, qw]
```

四元数使用 Hamilton、`xyzw`、单位模。时间序列通过
`dot(q[t], q[t-1]) >= 0` 选择等价符号，保证相邻帧连续；不逐帧强制 `qw >= 0`。外部系统
需要 `qwxyz` 时只能由带 schema 和往返测试的导出 Adapter 转换。

所有变换采用 `T_A_B` 命名，语义为把 B 坐标中的齐次点变换到 A：

```text
p_A = T_A_B @ p_B
```

### 3.4 固定与移动世界系

- 阶段 1--4：使用首帧固定相机坐标 `episode0_camera`。不得把它静默解释为机器人基座或
  重力对齐的 `scene_world`。
- 阶段 5--6：设备 SDK 提供 `T_global_camera_t`（c2w）时，统一变换到 episode 首帧相机系：

```text
T_episode0_camera_hand_t =
    inverse(T_global_camera_0) @ T_global_camera_t @ T_camera_t_hand_t
```

`global` 只表示跟踪系统坐标，不默认等于机器人或物理场景世界系。需要重力对齐、标定板或
机器人基座时，必须再保存显式外参。

## 4. 采集能力标准

### 4.1 固定 RGB-D 目标 Profile

| 项 | 目标 |
|---|---|
| RGB | 1920x1080、60 fps、固定帧率 |
| 原生 Depth | 至少 640x480、60 fps |
| RGB-D | SDK 或硬件配对时间戳，残差 `< 10 ms` |
| 对齐 Depth | 重投影到 RGB 像素；不得把上采样分辨率声明为原生深度分辨率 |
| 曝光/白平衡/对焦 | 单 episode 内锁定；记录实际参数 |
| RGB 编码 | 使用固定环境可解码的高质量 codec，并记录 codec/pix_fmt |
| Depth 编码 | 原生容器、无损 `uint16` 或设备专用无损压缩 |

`ego_fixed_rgbd_60hz_v1` 只适用于达到该能力的固定设备。旧 30 fps 数据或未来眼镜不得靠
补帧套用该 Profile。

### 4.2 眼镜与腕部设备

眼镜按实际设备能力建立独立 Profile，不预设必须 60 fps。至少记录 RGB/RGB-D 模式、IMU
原生频率、VIO 输出频率、硬件时间域、温度/功耗降频和丢帧行为。

腕部 IMU用于高频方向和运动动态，不能通过双积分冒充长期绝对位置。腕部绝对位置来自
RGB-D、光学标记、外部相机或 Motion Capture。磁力计、UWB、数据手套、触觉和 sEMG 均为
可选能力，必须由独立 capability 和质量 Profile 声明。

## 5. EGO Canonical 字段

当前单手固定相机基线继续使用：

```text
observation.images.ego
observation.hand_keypoints
observation.hand_keypoints_2d
observation.hand_visibility
observation.wrist_pose
observation.hand_estimator_id
timestamp
```

阶段 5--6 的 schema revision 至少增加：

```text
observation.camera_pose
observation.camera_pose_valid
observation.camera_pose_confidence
observation.camera_tracking_state
observation.wrist_pose_valid
observation.wrist_pose_confidence
observation.wrist_pose_source
```

双手数据必须使用明确的 left/right 特征或版本化 hand-slot 表示，不能只给一个
`handedness` 后复用同一列。每个可缺失派生量必须配套 `valid`；不得以全零冒充不可用观测。

原始 depth 始终保存在 Source。是否把对齐 depth 作为 `observation.images.depth` 写入 EGO，
由版本化导出配置决定，并且必须能追溯到 Source 原始深度和标定。

## 6. Source 与时间同步

阶段 1--4 的单 RGB-D 可继续使用当前 `stream_index.parquet`。阶段 5--6 的异步多传感器
Source 采用长表：

```text
source/streams.parquet
  stream_id, sensor_id, modality, nominal_rate_hz, calibration_id

source/samples.parquet
  stream_id, sample_index, device_timestamp_us, master_timestamp_us
  path, valid, uncertainty_us

source/synchronization.json
  master_clock, clock models, measured offsets, drift and uncertainty
```

设备时钟映射使用显式模型 `t_master = a * t_device + b`。原始高频 IMU不先降采样保存；
EGO 构建器按目标时间轴选择、插值或聚合，并保存每项时间残差和来源样本。

## 7. 质量指标与真值规则

每项报告必须包含 metric 名称、统计量、单位、阈值、测量依据、样本数、真值可用性和
`pass=true/false/null`。缺少 Ground Truth 时，绝对精度项目必须为 `null`，不能由连续性或
稳定性代理替代。

### 7.1 Source 与标定

| 指标 | 目标 | 真值/测量要求 |
|---|---:|---|
| 内参重投影误差 | `< 0.5 px` | 标定板检测与重投影 |
| RGB-Depth 对齐误差 | `< 1 px` | 可验证的 RGB-D 标定目标 |
| 同源 RGB-D 同步 | `< 10 ms` | 成对硬件时间戳 |
| 多设备同步 | `< 10 ms` 目标 | 同步线、公共事件或时钟模型 |
| 图像-动态相机位姿 | `< 20 ms` | 设备时间戳及插值残差 |
| 原始流完整率 | `>= 99%` 目标 | 设备序号和时间戳连续性 |

### 7.2 EGO 手部质量

| 指标 | 目标 | 真值/测量要求 |
|---|---:|---|
| 手部检出率 | 固定/移动 `>= 90%`，头戴 `>= 80%` | 分母为人工标注的手在画面帧 |
| 静止腕部位置抖动 P95 | `< 1.5 cm` | 显式静止段；稳定性代理 |
| 腕部绝对位置误差 P95 | `< 1 cm` 目标 | 外部 RGB-D/Mocap/标定真值 |
| 三维尺度误差 P95 | `< 1 cm` 目标 | 明确骨长或关键点真值定义 |
| VIO ATE RMSE | `< 3 cm` 目标 | 公制真值、SE(3) 对齐规则和轨迹长度 |
| 腕部绝对朝向误差 | 设备试采后定阈值 | 四元数测地角 + 姿态真值 |

检出率、抖动、连续性和骨长稳定性不能证明绝对准确。阶段 6 正式阈值必须由一批
Ego-Exo 或 Motion Capture 金标准数据完成设备资格验证后冻结。

### 7.3 RobotDataset 重定向质量

重定向指标不参与 EGO ready 判定，只决定某个 RobotDataset revision 是否可交付：

| 指标 | 目标 | 规则 |
|---|---:|---|
| 指尖重定向误差 | `< 1 cm` 目标 | 明确同一坐标、FK 点和尺度映射 |
| 关节限位违反 | `0` | 使用对应资产 revision 与安全契约 |
| 帧间关节跳变 | 按 Robot Profile | 报告 max/P99，不凭目测 |
| 碰撞 | `0` 目标 | 有碰撞模型时判定，否则 `not_evaluated` |
| 接触时刻误差 | `< 10 ms` 目标 | 只有触觉/力真值时判定 |

夹爪阶段可以把拇指尖 `kp4` 与食指尖 `kp8` 距离映射为归一化开合，但必须记录参考区间、
裁剪和目标夹爪行程。

## 8. Capture 与 LeRobot 交付

正式根目录是：

```text
datasets/captures/capture_<YYYYMMDD>_<sequence:06d>_<uuid>/
```

其中只有以下目录是独立 LeRobotDataset v3 根：

```text
<capture>/ego/
<capture>/robot_datasets/<target>/<asset_revision>/<retarget_revision>/
```

Source、lineage、reports 和 Capture 根都不是 LeRobotDataset。Capture 使用
`building -> ready/failed` 生命周期，所有 Parquet footer、视频、元数据、sidecar 和 checksum
落盘后才能标为 ready。

每次正式交付必须同时通过：

```bash
conda run -n lerobot-v3 python src/lerobot_v3/verify_dataset.py \
  --capture-root <absolute-capture-path> --canonical --strict-v3

conda run -n lerobot-v3 python src/lerobot_v3/verify_dataset.py \
  --capture-bundle --capture-root <absolute-capture-path> \
  --json <capture>/reports/capture_integrity_report.json
```

还必须用 `lerobot==0.6.1` 的 `LeRobotDataset()` 实际回读所有声明的数据集。当前仓库中的旧
Capture 可作为兼容回读样例，但只有通过上述全部检查的新 Capture 才能作为交付金标准。

## 9. 数据治理

眼镜数据默认按可能包含旁观者、人脸、屏幕、语音和位置隐私处理。采集前记录许可范围；音频
默认关闭或独立授权；Source 与可发布派生物使用不同保留策略。匿名操作者 ID、设备序列号、
SDK/固件、标定 ID、代码 revision 和处理权重必须可追溯，但不得记录可复用凭据。

---

**规范状态**：现行设计基线；阶段 1--4 可直接据此实现，阶段 5--6 的具体设备阈值须在硬件
选型和小规模 Ground Truth 试采后冻结。
