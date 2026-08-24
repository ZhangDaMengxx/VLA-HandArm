# Capture Bundles

EGO 的眼镜主视角定义、固定相机阶段和质量验收见
[EGO_DATA_STANDARD.md](../../EGO_DATA_STANDARD.md)。本页只定义存储、生命周期和兼容边界。

正式采集数据按一次 Capture 一个目录保存：

```text
capture_<YYYYMMDD>_<sequence:06d>_<uuid>/
├── bundle.json
├── environment/
│   ├── runtime.json            # Python、平台及关键数据依赖版本
│   ├── requirements.txt        # 直接依赖约束
│   └── environment.lock        # 完整已安装包快照
├── source/
│   ├── acquisition.json
│   ├── quality_profile.json      # 本 Capture 不可变的版本化质量口径快照
│   ├── recordings/              # 原生容器或处理结果原文件
│   ├── rgb_original/            # 原视频或原分辨率 RGB 帧
│   ├── depth/
│   │   ├── raw/                 # 仅当采集设备实际提供时存在
│   │   └── aligned_to_rgb/      # 已对齐 RGB 的 uint16 原深度帧
│   ├── stream_index.parquet     # Source 文件、时间戳、同步和 Ego 帧映射
│   ├── calibration/
│   ├── checksums_original.json
│   └── retention.json
├── ego/                         # 独立 LeRobotDataset，含 annotations/episode_*.json
├── robot_datasets/
│   └── <target_id>/<target_revision>/<retarget_revision>/
│       ├── annotations/         # 人工审核，不覆盖已有文件
│       ├── qa/                  # 每 episode 自动结构 QA
│       └── ...                  # 独立 LeRobotDataset
├── lineage/
└── reports/
```

`ego/` 和最终的 `<retarget_revision>/` 可以分别传给 `LeRobotDataset()`。原始输入、可重建
轨迹导出和验收报告不得混入这两个数据集根目录。

构建器会按实际 episode 建立 annotation 占位，默认 `unreviewed`，再次生成只补缺失文件。
Robot QA 自动检查帧索引、state/action 非有限值；关节限位、碰撞和指尖误差缺少模型或真值时
保持 `not_evaluated`。完整 Capture 校验命令为：

```bash
conda run -n lerobot-v3 python src/lerobot_v3/verify_dataset.py \
  --capture-bundle --capture-root <capture> --json <capture>/reports/capture_integrity_report.json
```

Source 文件同盘时优先硬链接，跨盘时复制；两种方式都由 `checksums_original.json` 校验。
`stream_index.parquet` 的 `source_frame_index` 和可空 `ego_frame_index` 记录原始帧是否进入
Ego。硬件时间仅写入 `rgb_timestamp_hw_us`/`depth_timestamp_hw_us`；没有硬件时间的旧视频或
帧集保留空值，并把 `timestamp_source` 标记为 `fps_derived`，不能把推算时间当硬件时间。

该宽表是阶段 1--4 单 RGB-D 的现行格式。眼镜、VIO/IMU、左右腕部设备和外部真值构成多个
异步流时，使用可选的 `streams.parquet`、`samples.parquet` 和 `synchronization.json` 长表；
`capture_bundle.write_multisensor_source_index()` 已提供厂商无关的逐文件原子写入和基础校验，
但设备 Adapter 与 Source -> Ego 对齐消费者仍待实现。原 `stream_index.parquet` 保持兼容，
并将作为派生后的 Source -> Ego 对齐视图。全链完成前，不得把多个设备强行塞入
`rgb_*`/`depth_*` 固定列或丢弃原生高频采样。

质量 profile 的仓库源文件位于 `configs/quality_profiles/`。构建器把完整内容快照到 Source，
`acquisition.json` 同时记录 `profile_id` 和 revision；后续验收只读快照，不因仓库默认阈值更新
而改变旧 Capture 结论。当前 profile 分为：

- `legacy_rgb_video_30hz_v1`：既有固定相机 RGB 视频，不声明 Depth 或硬件同步
- `legacy_aligned_rgbd_30hz_v1`：既有 960×540 对齐帧集，按文件名配对，不声明硬件同步
- `processed_observations_v1`：外部处理结果，不反推相机能力
- `ego_fixed_rgbd_60hz_v1`：固定 RGB-D 生产 EGO Canonical 的目标，RGB 1920×1080@60、原生 Depth 至少 640×480@60、硬件时间戳且 RGB-D 残差 <10ms

未来眼镜和腕部设备使用独立 profile，不通过补帧或缺失流占位复用固定相机 profile。

每个 profile 还显式记录 `device_class` 和 `sync_mode`；当前同步模式区分单视频流、无硬件时钟
的文件名配对、外部未声明和成对硬件时间戳，不根据目录名推断。

profile schema 1.1 的每个指标还记录 `measurement_class` 和 `ground_truth_required`。验收报告
逐项输出测量依据及真值可用性。`ground_truth.wrist_pose` 缺失时手腕绝对位置误差为不可测；
骨长稳定性、腕部连续性和深度连续性仍可作为代理单独报告，但不能替代绝对精度或 RGB-D
像素对齐精度。旧 Capture 内的 schema 1.0 快照继续按原 revision 读取，不会被自动升级。

profile 是验收标准，不是转换器。给旧 30 Hz 数据选择 60 Hz profile 会产生失败项，不会补帧、
修改时间戳或改写样本。LeRobot `timestamp` 均匀性只验数据集内部 cadence；真实 RGB-D 同步
只从 Source 成对硬件时间戳计算，没有硬件时间时不得宣称通过。

旧 `src/out/` 不会自动移动或删除。需要读取旧产物时必须在命令中显式使用
`--legacy-out`；新采集默认写入本目录。

同一条处理链应始终传同一个 `--capture-root`。路径解析器会拒绝以下组合：

- `--capture-root` 与显式 canonical/RobotDataset 根混用
- canonical 和 RobotDataset 来自不同 Capture
- 把 Capture 内 `source/`、`reports/` 等非数据集目录当作 Ego 或 RobotDataset

不传路径时，Canonical 构建器新建 Capture，派生/验收/验证/回放和分析工具读取最新可用
Capture。`bundle.json` 的生命周期为 `building -> ready`，普通异常退出记为 `failed`；隐式读取
只选择 `ready`，所以新建后中断的半成品不会遮住上一份完整数据。显式 `--capture-root` 仍可
检查或重建 `building/failed` 批次。并行创建使用文件锁分配同日 sequence；UUID 继续保证目录
身份唯一。

当前路径迁移只改变存储位置，不改变现有 `xyzw` 四元数、MediaPipe 21 点顺序、坐标系、
矩阵乘法、IK、滤波或 retarget 数值语义。数据集专用 Python `3.12.13` + `lerobot 0.6.1`
环境已完成新 Capture 生成、官方回读和严格 v3 校验，复现依赖见
`environment/lerobot-v3-dataset.txt`。每个 Capture 的根级 `environment/` 保存实际生成环境。
旧 0.4.4 数据可由 0.6.1 回读，但其旧式 `tasks.parquet` 不满足 `--strict-v3`，不做就地改写。

`ego/meta/coordinate_system.json` 使用 `schema_version=2.0`，是坐标语义权威来源：

- 普通固定相机 RGB：`observation.wrist_pose.frame=episode0_camera`
- 有 `camera_to_world` 外参的 RGB-D：`observation.wrist_pose.frame=scene_world`
- 默认 MediaPipe/MANO 3D 手关键点：`wrist_local_mano`
- 显式 `--hand-keypoints-source depth_world`：3D 手关键点为 `scene_world`
- 2D 手关键点：`ego_rgb_pixels`

外部处理结果优先读取输入内的 `wrist_pose_frame`，也可用 `--wrist-pose-frame` 声明。旧文件
缺失时保留兼容默认，但 `declared_by` 会记录为 `compatibility_default_episode0_camera`。
消费者不得根据 Capture 目录、`source_kind` 或构建脚本名称猜测 frame。此次升级只写元数据并
校验声明，没有修改任何样本值、矩阵乘法、四元数顺序、IK、滤波或 retarget 运算。
