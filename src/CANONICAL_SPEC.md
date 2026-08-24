# 规范层数据规范(Canonical Data Spec)

规范层 = **母带(embodiment- & model-agnostic master)**。定档标准只有两条:
**① 目标 VLA 现在吃得下;② 以后换任何本体 / 任何模型重新派生都够用。**

这里的 **EGO** 特指以操作者眼镜/头戴第一视角为最终目标的人类演示数据。固定 RGB-D
相机是阶段 1--4 的生产与算法验证来源，腕部设备和外部相机是增强或 Ground Truth 来源；
它们不会改变 `<capture>/ego/` 作为机器人无关人类演示母带的含义。采集阶段、设备分档和
质量验收见 [../EGO_DATA_STANDARD.md](../EGO_DATA_STANDARD.md)。

两个独立的"无关"轴,用同一招(富的原始超集 + 薄适配器)解决:

- **主体无关(embodiment-agnostic)**:母带里**不烘进任何机器人关节**。只存人手 21 点 + 手腕 6-DoF + RGB-D + 语言(世界系)。变成某台机器人的 state/action 是 `derive_embodiment.py` 里 retarget 干的事 → 换机器人不重采。
- **模型无关(model-agnostic)**:母带里**不烘进任何模型的格式偏好**(分辨率、视角数、动作表示、容器格式、归一化)。这些全在各自的导出器里定 → 换模型不重采。

> **一句话原则:采集按"最贪婪的下游"取上确界,母带存超集;差异靠 derive 阶段的薄适配器解决,绝不回头重采。**
> 能把高清降低清、把 30fps 抽 5Hz;但**采时没拿到的,事后变不出来**。

---

## 0. 全局约定(所有字段都遵守)

| 约定 | 规则 |
|---|---|
| **长度** | 浮点字段统一**米(m)**;传感器原始整数深度保留 **uint16 毫米(mm)**,入库转 float32 米 |
| **角度** | 弧度(rad)。规范层不存关节角,只存位置/四元数,角度由运动链反推 |
| **四元数** | `[qx, qy, qz, qw]`(scipy `as_quat()` 的 **xyzw**),单位模,Hamilton 约定。语义 `p_参考 = R·p_物体 + t`,其中 `R = quat→matrix(q)`；序列按相邻点积非负做符号连续化 |
| **时间** | LeRobot `timestamp` 为 float32 秒(s)、首帧 t=0 单调递增；Source 原始硬件时间戳另存 int64 微秒(us) |
| **图像坐标** | 像素,原点左上,+u 右、+v 下 |
| **dtype** | 除深度/图像外,数值字段 float32(物理误差远大于 float32 的 ~0.03µm,无需 float64) |

### 单位说明:为什么是米,不是 cm/mm

浮点的精度是**相对精度**(float32 ~7 位有效数字)。换单位只是把数值和最小步长**同步缩放**,物理分辨率不变——`0.5 m` 与 `500 mm` 的 float32 步长都约 `0.03 µm`。**mm 不比 m 精确**。单位真正影响精度的只有**整数存储**(uint16 mm 步长 1mm),这正是深度原始保留 mm 的原因。浮点统一米,是为了和 URDF / Pinocchio / dex-retargeting / ROS(REP-103 强制米)全链路 SI 一致,避免每步 derive ×0.001 的换算 bug。

当前 schema 固定保留 `xyzw`，因为 SciPy、现有腕姿、回放、IK 和 retarget 全链均使用该顺序。
若外部交付必须使用 `qwxyz`，只能由显式导出适配器在边界转换，并同步提升 schema、记录来源
顺序和补充往返数值测试；禁止原地改写既有 Capture 或仅修改字段说明。时间序列使用
`dot(q[t], q[t-1]) >= 0` 选择等价符号，不逐帧强制 `qw >= 0`。

### 坐标系定义(写死)

- **episode0_camera**:episode 首帧相机系。固定相机阶段整段不变；移动/头戴阶段由动态
  c2w 轨迹统一回到该坐标系。
- **场景世界系 W**:右手系,原点 = 场景固定基准(标定板角点 / 机械臂基座),**+Z 竖直向上(逆重力)**,+X/+Y 水平。只有显式标定后才可声明为 `scene_world`。
- **相机系 C**:OpenCV 约定,原点光心,**+X 右、+Y 下、+Z 沿光轴指向场景**。→ Tier 0 现状系。
- **手腕 / MANO 系**:MANO canonical frame(`operator2mano` 旋进去的那个)。

若移动设备给出 `T_global_camera_t`（c2w），手腕位姿按以下公式归一到 episode 首帧：

```text
T_episode0_camera_hand_t = inverse(T_global_camera_0)
                           @ T_global_camera_t
                           @ T_camera_t_hand_t
```

所有变换采用 `T_A_B` 记号，表示把 B 坐标中的点变换到 A。SDK 跟踪 `global` 不会自动
成为机器人基座或重力对齐的 `scene_world`。

### Capture Bundle 存储契约（2026-08-20 现状）

正式数据默认按一次采集一个 Bundle 保存：

```text
datasets/captures/capture_<YYYYMMDD>_<sequence:06d>_<uuid>/
├── bundle.json
├── environment/                               # Python、依赖和完整环境快照
├── source/
├── ego/                                      # 独立 Ego LeRobotDataset 根
│   └── annotations/episode_*.json            # 人工审核 sidecar，默认 unreviewed
├── robot_datasets/<target>/target_revision_v001/retarget_v001/
│   ├── annotations/episode_*.json            # 机器人派生结果人工审核
│   └── qa/episode_*.json                     # 自动结构 QA + 未评估物理项
├── lineage/
└── reports/
```

路径权威实现是 `capture_bundle.py`。三个 `build_canonical*` 默认创建新 Capture；派生、验收、
验证、回放和 Web 必须绑定同一 Capture。`src/out/` 只保留 `--legacy-out` 显式读取，不自动
移动或删除。`bundle.json` 使用 `building/ready/failed` 生命周期，Ego 元数据及校验和写完后
才进入 `ready`；不指定路径的消费者只选择最新 `ready` Capture。

`ego_schema.json` 记录 MediaPipe 21 点、米/弧度和 `xyzw` 数值契约；
`coordinate_system.json` 2.0 逐字段记录坐标语义。普通固定相机 RGB 的 `wrist_pose` 是
`episode0_camera`，通过 `camera_to_world` 标定外参变换的 RGB-D 是 `scene_world`。默认 3D
手关键点保持 `wrist_local_mano` 以服务 dex-retargeting；只有显式 `depth_world` 构建才为
`scene_world`。2D 点始终属于 `ego_rgb_pixels`。消费者必须读取该元数据，不能凭目录名或
`source_kind` 推断。此次分层只增加声明与校验，没有转换任何关键点、四元数、矩阵或时间值。

Source 基础层现已保留原视频、处理结果原文件或参与构建的原分辨率 RGB/已对齐深度，并用
`stream_index.parquet` 记录 Source -> Ego 帧映射。硬件时间字段使用微秒，缺失时保持 null 并
标记 `fps_derived`；当前旧 Kinect 帧集没有原生容器、raw depth 或硬件时间，不能补造。

质量标准由 `configs/quality_profiles/*.json` 版本化，每个 Capture 在
`source/quality_profile.json` 保存不可变快照。`ego_fixed_rgbd_60hz_v1` 是正式固定相机目标；
旧 RGB、旧 960×540@30 RGB-D 和 processed 输入使用独立兼容 profile，不能宣称目标设备能力。
`measure_acceptance.py` 读取快照决定阈值，并将 Source 硬件同步与 LeRobot 内部 cadence 分开。
schema 1.1 进一步给每项阈值声明测量类别和是否需要真值。当前数据没有逐帧手腕真值时，
`wrist_position_absolute_error_p95_cm` 必须保持不可测；骨长帧间标准差、腕部帧间步长和深度
连续性只属于代理指标。可选 `ground_truth.wrist_pose` 用于逐帧绝对位置比较，可选
`annotation.wrist_stationary` 用于明确标注静止段；验收器不会从低运动量自动猜测静止段。

Web/ROS 运行环境仍是 Python `3.10.20` + `lerobot 0.4.4`。数据集专用环境已使用
Python `3.12.13` + `lerobot 0.6.1` 完成新 Capture 生成、官方回读和严格 LeRobot v3.0
结构校验；每个 Capture 根还保存 `environment/runtime.json`、`requirements.txt` 和
`environment.lock`。设备原生 RGB-D 采集和 episode 级 QA 仍未完成。旧 0.4.4 数据可由
0.6.1 回读，但旧 `tasks.parquet` 不满足当前严格交付校验，因此不应就地改写为新格式。
新数据集为每个 episode 生成 annotation；已有人工审核文件不会被重写。RobotDataset QA 自动
检查帧索引连续性和 state/action 有限性，而限位、碰撞与指尖绝对误差必须等真实证据接入后
才能判定。`verify_dataset.py --capture-bundle` 对整个 Capture 做严格 v3、血缘、sidecar 和
checksum 完整性检查。

---

## 1. 采集要求(capture requirements)

物理采集当下必须达标、事后补不回的项。下表是固定 RGB-D 目标；眼镜、IMU和腕部设备
必须使用设备专用 Profile，不能为了套用 60 fps 指标补帧。真正决定标签质量上限的是
**快门 + 曝光锁定 + 深度配准 + 标定 + 同步**。

| 项 | 要求 | 不达标的后果 |
|---|---|---|
| **RGB 分辨率** | ≥720p,建议 1080p,**存原生**(降采样放 derive) | 手指关节/小物体不可分辨 |
| **帧率** | 目标 RGB/Depth 60fps,**固定帧率 CFR** | VFR 会让时间戳与动作块对不齐 |
| **快门** | 优先**全局快门**;卷帘须配短曝光 | 快速手部运动被卷帘拍歪(几何畸变) |
| **曝光/白平衡/对焦** | **单条 episode 内锁定**,禁用 auto | 自动曝光致画面忽明忽暗、颜色漂移,VLA 误学 |
| **运动模糊** | 曝光时间尽量短 | 模糊的手,MediaPipe/WiLoR 都测不准 |
| **深度配准** | 深度**registered 到 RGB**,逐像素对应 | 不对齐无法把 2D 手点抬成 3D |
| **深度量程/模式** | 覆盖工作区(台面 ~0.3–2m;Femto NFOV 640×576@30 合适) | 量程外是空洞 |
| **深度盲区** | 反光/透明/深黑/边缘/强红外丢点 → 记为无效(0),**不插值** | ToF 物理限制,派生须能识别无效像素 |
| **标定** | 内参 + 畸变 + RGB-深度外参,**每次改动重标** | 无标定则深度和世界系全废 |
| **时间同步** | 每帧硬件时间戳;RGB/深度分流则互相对齐 | 多流不同频错位毁掉 3D 抬升 |
| **视野/安装** | 整条 episode 手和工作区都在画面内;头戴/固定的高度角度固定 | 手出画面的帧作废 |
| **光照** | 均匀漫射,避免强红外源(阳光/某些射灯)干扰 ToF | 红外"晃瞎"深度传感器 |
| **编码** | 母带无损或高码率,禁手机级强压缩 | 压缩块效应糊掉手部纹理 |

**固定相机正式目标**:1920×1080 RGB + 至少 640×480 原生深度 @60fps CFR + 全局快门/锁曝光 + 出厂或自标内外参 + 每帧硬件时间戳，RGB-D 同步残差 <10ms。对齐到 RGB 后的深度图尺寸不代表原生深度分辨率。30fps 旧数据只按对应 compatibility profile 留档和评估，不等于满足新数采目标。

---

## 2. 字段规范

标注:✅=现在就有 / 🔜=需 Femto / ⭐=需 WiLoR 才填(MediaPipe 留空)。

### 2.1 图像 / 深度

| 字段 | dtype | shape | 单位 | 坐标系 | 来源 | 状态 |
|---|---|---|---|---|---|---|
| `observation.images.ego` | uint8 | (H,W,3) | sRGB 0–255,RGB 序 | — | 相机 RGB | ✅(**存原生分辨率**) |
| `observation.images.depth` | float32 | (H,W) | 米,`0.0`=无效 | 对齐 RGB | Femto 深度(原始 uint16 mm→m) | 🔜 |
| `observation.images.depth_conf` | uint8 | (H,W) | 0–255 置信 | 对齐 RGB | Femto(若提供) | 🔜可选 |

### 2.2 相机标定

| 字段 | dtype | shape | 单位 | 说明 | 状态 |
|---|---|---|---|---|---|
| `camera.intrinsics` | float32 | (4,) | `fx,fy,cx,cy` 像素 | 另存对应图像尺寸 `(w,h)` 像素 | 🔜 |
| `camera.distortion` | float32 | (5,) | 无量纲 | OpenCV `k1,k2,p1,p2,k3` | 🔜 |
| `camera.extrinsics` | float32 | (7,) | `t`(m)+quat(xyzw) | `T_world_cam`:相机系在世界系位姿;Aria 用 SLAM 头姿 | 🔜 |

### 2.3 手部(估计器无关分层)

**关键点顺序归一化**:canonical 骨架 = **MediaPipe/MANO 21 点序**(见 §3)。MediaPipe 直接给此序;**WiLoR 输出须 remap 进此序**。否则不同估计器采的 episode 静默错序,混训即废。`KP_NAMES` 是唯一真相。

**必需层(公共,任何估计器都能给)**

| 字段 | dtype | shape | 单位 | 坐标系 | 来源 | 状态 |
|---|---|---|---|---|---|---|
| `observation.hand_keypoints` | float32 | (63,)=21×3 | 米 | 默认 `wrist_local_mano`;仅 `depth_world` 为 `scene_world` | 3D landmarks,序=`KP_NAMES` | ✅ |
| `observation.hand_keypoints_2d` | float32 | (42,)=21×2 | 像素 u,v | 图像 | 2D landmarks | ✅ |
| `observation.hand_visibility` | float32 | (21,) | 0–1 | — | presence/可见度 | ✅ |
| `observation.wrist_pose` | float32 | (7,) | `t`(m)+quat(xyzw) | RGB=`episode0_camera`;标定 RGB-D=`scene_world` | `pose_to_vec()`,rot=手腕系姿态 | ✅ `coordinate_system.json` 2.0 显式声明 |
| `observation.hand_estimator_id` | float32 | (1,) | — | — | `0=mediapipe,1=wilor` | ✅ |
| `handedness` | str/int | 标量 | — | — | `"right"/"left"` | 🔜(单手也显式存) |

**可选富层(仅 WiLoR 等参数化估计器)**

| 字段 | dtype | shape | 单位 | 说明 | 状态 |
|---|---|---|---|---|---|
| `mano.pose` | float32 | (45,) 或 (48,) | rad(轴角) | MANO 关节姿态 θ(是否含 global 见下) | ⭐ |
| `mano.global_orient` | float32 | (3,) | rad(轴角) | 手腕全局朝向 | ⭐ |
| `mano.betas` | float32 | (10,) | 无量纲 | MANO 形状 β | ⭐ |
| `mano.vertices` | float32 | (778,3) | 米 | mesh 顶点(可选,体积大) | ⭐可选 |

必需层保证任何估计器采的 episode 都可训、可派生;富层在 WiLoR 时填、MediaPipe 时留空,换估计器下游代码不改。

### 2.4 时间 / 同步

| 字段 | dtype | shape | 单位 | 说明 | 状态 |
|---|---|---|---|---|---|
| `timestamp` | float32 | 标量 | 秒 | 首帧=0 单调递增，由 LeRobot 0.6.1 生成 | ✅ |
| `timestamp_rgb` / `timestamp_depth` | float64 | 标量 | 秒 | 各流独立时间戳(Femto RGB/深度可能不同频) | 🔜 |

Source 中的原始设备时间继续使用 int64 微秒；上表只描述派生到 LeRobot 帧后的时间字段。
眼镜/移动阶段的 RGB、Depth、VIO、IMU和腕部设备是异步原始流，不要求先降采样成同帧率。

### 2.5 移动/头戴扩展字段

阶段 5--6 必须通过新的 schema revision 增加动态相机位姿、有效性、置信度和来源，不能把
VIO 轨迹只放在不可关联的日志中：

| 字段 | dtype/shape | 说明 |
|---|---|---|
| `observation.camera_pose` | float32 `(7,)` | `T_episode0_camera_camera_t`，`xyzw` |
| `observation.camera_pose_valid` | bool | 当前帧是否有可用动态位姿 |
| `observation.camera_pose_confidence` | float32 `(1,)` | 设备/融合器置信度 |
| `observation.camera_tracking_state` | int/string | tracking、relocalized、lost 等版本化枚举 |
| `observation.wrist_pose_valid` | bool | 腕姿是否可用于派生 |
| `observation.wrist_pose_confidence` | float32 `(1,)` | 腕姿融合置信度 |
| `observation.wrist_pose_source` | int/string | RGB-D、单目、视觉-IMU、外部真值等来源 |

双手数据必须由显式 left/right 特征或版本化 hand-slot schema 表达；不得只复用一个单手列并
依赖逐帧 `handedness` 猜测槽位。可缺失派生量必须有 validity，不能用全零冒充有效观测。

### 2.6 Per-episode 元数据(episode 级,非每帧)

| 字段 | 类型 | 单位/取值 | 说明 |
|---|---|---|---|
| `episode_id` | str | — | 唯一 id |
| `task` | str | UTF-8 | 指令原文,同任务建议多措辞 |
| `task_id` | int | — | 任务枚举,便于切分 |
| `success` | bool | 0/1 | 成功/失败标签(切 train/val、过滤必需) |
| `demonstrator` | str | — | 演示者 |
| `device` | str | — | `femto_bolt` / `aria` / … |
| `hand_estimator` | str | — | `mediapipe` / `wilor` / …(provenance) |
| `is_metric` | bool | 0/1 | 是否真米制(**取决于有无深度,不取决于估计器**;WiLoR 单目仍有尺度歧义) |
| `object_set` | list[str] | — | 场景物体身份 |
| `lighting` | str | — | 光照条件标签 |
| `date` | str | ISO8601 | 采集日期 |
| `calib_id` | str | — | 关联到哪套内外参标定 |

当前实现说明:

- `build_canonical.py` 已通过 `hand_estimators.py` 走估计器适配器接口。
- 当前可用后端为 `--hand-estimator mediapipe`。
- `--hand-estimator wilor` 已预留入口, 但 WiLoR 模型/runtime 未接入前会显式报错。
- `observation.hand_estimator_id` 是当前 LeRobot 写盘里的机器可读来源字段; episode 级字符串 `hand_estimator` 后续随元数据管理一起补。

---

## 3. Canonical 21 关键点顺序(KP_NAMES)

MediaPipe / dex-retargeting 手部 landmark 序。索引 0=手腕,每指 4 点(近→远)。WiLoR 输出必须 remap 到此序。

| idx | 名称 | idx | 名称 | idx | 名称 |
|---|---|---|---|---|---|
| 0 | wrist | 7 | index_dip | 14 | ring_pip |
| 1 | thumb_cmc | 8 | index_tip | 15 | ring_dip |
| 2 | thumb_mcp | 9 | middle_mcp | 16 | ring_tip |
| 3 | thumb_ip | 10 | middle_pip | 17 | pinky_mcp |
| 4 | thumb_tip | 11 | middle_dip | 18 | pinky_pip |
| 5 | index_mcp | 12 | middle_tip | 19 | pinky_dip |
| 6 | index_pip | 13 | ring_mcp | 20 | pinky_tip |

> 当前 `build_canonical.py` 用通配名 `kp{i}_{a}`(i∈0..20, a∈xyz)。语义索引即上表;`observation.hand_keypoints` 的 `(63,)` = 21 点 × (x,y,z) 展平。

---

## 4. 分档(Tier)

| 档 | 内容 | 能验证什么 |
|---|---|---|
| **Tier 0**(现状) | RGB256 + 手 kp + 手腕 + task,相机系单目,1 episode | 只证明 pipeline 通 |
| **Tier 1**(该定的档) | 原生分辨率 RGB + 度量深度 + 内外参 + 世界系手/手腕 + 每流时间戳 + per-episode 元数据(含 success) + **30–100 条带变化的 episode** + 多措辞语言 | 真正验证"数据可训"+"数据级本体无关" |
| **Tier 2**(Phase D+) | 物体 6-DoF/分割 + 多视角/多设备 + 子步骤标注 | 物体 grounding、多本体 |

**下一步施工顺序**:① ~~补深度+内外参+世界系,derive 去 home 锚定~~ —— kinect RGB-D 已落地:世界系米制 wrist_pose + `frame_mode=metric` 固定 base 绝对映射(550/557),home 锚定已去。**剩一件真数据缺口**:要机器人相对相机的真实 `T_base_camera`(绝对世界位),需机器人与相机共处一场景做外参标定;当前 metric 用 centroid 每段居中作务实替代。② episode 从 1 条堆到几十条带变化的。物体级 grounding 与第二本体留后。
