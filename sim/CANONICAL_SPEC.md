# 规范层数据规范(Canonical Data Spec)

规范层 = **母带(embodiment- & model-agnostic master)**。定档标准只有两条:
**① 目标 VLA 现在吃得下;② 以后换任何本体 / 任何模型重新派生都够用。**

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
| **四元数** | `[qx, qy, qz, qw]`(scipy `as_quat()` 的 **xyzw**),单位模,Hamilton 约定。语义 `p_参考 = R·p_物体 + t`,其中 `R = quat→matrix(q)` |
| **时间** | 秒(s),float64,首帧 t=0 单调递增;另存 Unix epoch(s) |
| **图像坐标** | 像素,原点左上,+u 右、+v 下 |
| **dtype** | 除深度/图像外,数值字段 float32(物理误差远大于 float32 的 ~0.03µm,无需 float64) |

### 单位说明:为什么是米,不是 cm/mm

浮点的精度是**相对精度**(float32 ~7 位有效数字)。换单位只是把数值和最小步长**同步缩放**,物理分辨率不变——`0.5 m` 与 `500 mm` 的 float32 步长都约 `0.03 µm`。**mm 不比 m 精确**。单位真正影响精度的只有**整数存储**(uint16 mm 步长 1mm),这正是深度原始保留 mm 的原因。浮点统一米,是为了和 URDF / Pinocchio / dex-retargeting / ROS(REP-103 强制米)全链路 SI 一致,避免每步 derive ×0.001 的换算 bug。

### 坐标系定义(写死)

- **世界系 W**:右手系,原点 = 场景固定基准(标定板角点 / 机械臂基座),**+Z 竖直向上(逆重力)**,+X/+Y 水平。→ Tier 1 目标系。
- **相机系 C**:OpenCV 约定,原点光心,**+X 右、+Y 下、+Z 沿光轴指向场景**。→ Tier 0 现状系。
- **手腕 / MANO 系**:MANO canonical frame(`operator2mano` 旋进去的那个)。

---

## 1. 采集要求(capture requirements)

物理采集当下必须达标、事后补不回的项。分辨率/帧率是下限,真正决定标签质量上限的是**快门 + 曝光锁定 + 深度配准 + 标定 + 同步**。

| 项 | 要求 | 不达标的后果 |
|---|---|---|
| **RGB 分辨率** | ≥720p,建议 1080p,**存原生**(降采样放 derive) | 手指关节/小物体不可分辨 |
| **帧率** | ≥30fps,**固定帧率 CFR** | VFR 会让时间戳与动作块对不齐 |
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

**当前建议下限**:1080p(至少 720p)RGB + 640×576 配准深度 @30fps CFR + 全局快门/锁曝光 + 出厂或自标内外参 + 每帧时间戳。这套采下来,MediaPipe 现在能用、WiLoR 以后能用、X-VLA 能训、换本体换模型都不重采。

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
| `observation.hand_keypoints` | float32 | (63,)=21×3 | 米 | T0=相机系 / **T1=世界系** | 3D landmarks,序=`KP_NAMES` | ✅(单目近似米制)/🔜真米制 |
| `observation.hand_keypoints_2d` | float32 | (42,)=21×2 | 像素 u,v | 图像 | 2D landmarks | ✅(近零成本,建议补) |
| `observation.hand_visibility` | float32 | (21,) | 0–1 | — | presence/可见度 | ✅ |
| `observation.wrist_pose` | float32 | (7,) | `t`(m)+quat(xyzw) | T0=相机系 / **T1=世界系** | `pose_to_vec()`,rot=手腕系姿态 | ✅/🔜去 home 锚定 |
| `handedness` | str/int | 标量 | — | — | `"right"/"left"` | ✅(单手也显式存) |

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
| `timestamp` | float64 | 标量 | 秒 | 首帧=0 单调递增(LeRobotDataset 自带 frame_index) | ✅ |
| `timestamp_rgb` / `timestamp_depth` | float64 | 标量 | 秒 | 各流独立时间戳(Femto RGB/深度可能不同频) | 🔜 |

### 2.5 Per-episode 元数据(episode 级,非每帧)

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

**下一步施工顺序**:① 等 Femto,把母带从"相机系单目 + 256 烘死"救出来(补深度+内外参+世界系,derive 去 home 锚定);② episode 从 1 条堆到几十条带变化的。这两步做完即可交付 Tier 1。物体级 grounding 与第二本体留后。
