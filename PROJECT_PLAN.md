# 机械臂 + 灵巧手 · 手势 / 抓取 / VLA 项目计划

> **长期方案文档。** 本文保留设计目标和阶段背景，但顶部的阶段状态停留在
> 2026-07-24。当前进度、风险和近期任务见 [PROJECT_STATUS.md](PROJECT_STATUS.md) 与
> [TODO.md](TODO.md)。

状态:Phase A 完成,Phase B 软件主链完成并进入 RGB-D/重定向验证(含 Rerun 可视化、手腕朝向稳定化、两层数据架构、RGB-D 离线融合、外部人手结果导入),Phase C 待在 RTX 上训练。更新 2026-07-24。任务见 Kiro #26–#34。逐次更改见 `更新日志.md`。

## 1. 目标

搭一条管线:真人第一视角(RGB-D)手部演示,retarget 成机械臂加灵巧手的关节动作,存成 LeRobotDataset,在 RTX 上微调 VLA,看这套数据能不能训出有效策略。交付物是这个验证本身,不是产品级机器人。

分三阶段:

- A 手势 / 摆姿势:纯运动学,不碰物体,先把臂加手的运动链打通。
- B 深度视觉 + 抓取:LLM 发高层指令,真人第一视角采集在这一步出数据。
- C VLA:ego 范式采集(RGB + 深度 + 视频)微调 VLA,验证数据可训。

## 2. 系统架构

```
   自然语言指令   "给我比个点赞" / "拿起红色方块"
        │
   ┌────▼─────────────────────────────┐
   │  LLM 规划层                        │  语言 → 技能调用序列
   └────┬─────────────────────────────┘
        │  联系②:LLM 和机器人 = 函数/工具调用
   ┌────▼─────────────────────────────┐
   │  技能包(point / grasp / move_to)  │
   └────┬─────────────────────────────┘
   ┌────▼──────────────┬──────────────┐
   │  手臂控制器(IK)    │  灵巧手控制器  │  联系①:手装在臂末端,一条运动链
   │  管去哪(6-DoF)     │ (retargeting) │  臂管到哪、手管怎么抓
   └────┬──────────────┴──────┬───────┘
   ┌────▼─────────────────────▼───────┐
   │        仿真器(MuJoCo)             │  联系③在这里执行和验证
   └──────────────────────────────────┘
```

三个联系:臂和手共享一条运动链;LLM 只出技能调用、不直接控关节;技能包在仿真里跑起来验证。

## 3. 选型

| 维度 | 值 |
|---|---|
| 机械臂 | 松灵 NERO,7 自由度,3kg。URDF 和网格已内置在 `assets/nero_description` |
| 灵巧手 | inspire,6 个驱动关节 |
| 数据源 | 真人第一视角:当前支持普通 RGB 视频、外部 MediaPipe/WiLoR 风格 npz/pkl/json、离线 RGB-D 帧目录;后期接 Orbbec Femto / RealSense / Azure Kinect 类采集 |
| 数据格式 | LeRobotDataset |
| 目标模型 | 先 ACT 从头训验证,再上 LeRobot 生态的 VLA(X-VLA)加 LoRA |
| 训练 | 本地 RTX |
| 开发 / 采集 / 仿真 | CPU 的 WSL + MuJoCo |

早期用 rm75_inspire(睿尔曼 RM75 加 inspire)当替身跑通过;现在 NERO 的 URDF 已经有了,主用 nero_inspire。RM75 和 NERO 都是 7 自由度加 inspire,以后正好拿来当第二个本体验主体无关性。

## 4. 两层数据结构

规范层(本体无关):ego RGB、人手关键点、2D 关键点、可见度、手腕 6-DoF、估计器来源、语言。RGB-D 路线会用对齐深度和相机内外参把 wrist position 融合成度量位置;当前深度图本身作为原始输入资产保留,尚未作为 canonical LeRobotDataset 的 `observation.images.depth` 持久字段。

本体层(每个机器人一份):对规范层按 URDF retarget 出的 LeRobotDataset:

| 字段 | 内容 |
|---|---|
| `observation.images.ego` | 第一视角 RGB(缩到 VLA 输入尺寸,存视频),喂 VLA |
| `observation.images.depth` | 计划字段。对齐深度图用于恢复手腕度量位姿、3D grounding,不喂基座 VLA;当前实现先把深度融合结果写入 `observation.wrist_pose` |
| `observation.state` | [7 臂 + 6 手] = 13 维当前关节 |
| `action` | [7 臂 + 6 手] = 13 维,绝对关节目标 |
| `task` | 自然语言指令 |

换新臂手只换 URDF,规范层不动。

## 5. 主体无关性

能验证的:① 原始人手数据和机器人无关、可复用(演示里没有机器人,采一次喂多个机器人,天然成立);② 管线和数据级的本体无关(换 URDF 即可,要实跑至少两个本体才算证明)。

验证不了的:③ 一个训好的模型直接跨本体(那要多本体共训,是下一个项目)。

"数据符合"是必要不充分。中间隔三个 gap:动作(目标本体的可达性封顶,比如 inspire 没有手指侧摆,人手扇形张开复现不了)、观测(训练画面是人手、部署是机器手)、动力学。retarget 的保真度就是标签质量的上限。

先在单本体(替身 rm75_inspire,目标 nero_inspire)跑通拿第一个结果,之后加第二个本体(比如 Franka + Allegro)坐实 ①②。

## 6. 分阶段进度

| 步 | 阶段 | 任务 | 做什么 | 产出 | 状态 |
|---|---|---|---|---|---|
| 1 | A | #26 | 组装 7 自由度臂 + inspire 运动链(rm75_inspire、nero_inspire) | 两装配加载 nq=19 | 完成 |
| 2 | A | #27 | 逆解 + retargeting 驱动 + MeshCat 检视台 | 视频→retarget→回放 | 完成 |
| 3 | A | #28 | 两层 schema + 手势预设 + 演示 | schema + 手势 demo | 完成 |
| 4 | B | #29 | RGB-D 采集 / 导入 | 离线 RGB-D 帧目录已可导入;真实相机采集仍待接 | 部分完成 |
| 5 | B | #30 | 人手 + 手腕 6-DoF 估计 | canonical_ds;普通 RGB 单目近似,RGB-D wrist position 度量融合 | 完成并验证中 |
| 6 | B | #31 | retarget→逆解→回放,SavGol 消抖 | robot_traj.pkl/npz + Rerun 回放 | 完成并调参中 |
| 7 | B | #32 | 写 LeRobotDataset | lerobot_ds_nero_inspire | 完成 |
| 8 | C | #33 | 验证数据可训:先 ACT 从头训(A/B/C 同数据),CPU 已验 dataloader 可消费,真训在 RTX | 验证结论 | 进行中(等 RTX) |
| 9 | D | #34 | 第二本体复跑,坐实主体无关性 | 主体无关证据 | 待办(依赖 C) |

## 7. 风险

1. retarget 保真度就是标签质量的上限(真人手到 inspire 有本体差异,见张开幅度和无侧摆的问题)。
2. 手腕 6-DoF 估计是 Phase B 的主要难点(比手指难,单目位置得靠深度)。当前 RGB-D 离线融合只能解决 wrist position 的度量问题,正式空间模仿仍需要 `T_base_camera`。
3. 换设备(Femto/RealSense/Azure Kinect 到 Aria)会破坏视觉一致性,视角和深度形态都变,得重采、做设备适配或域随机化。
4. 数据量:窄任务也得几十到上百条 episode 才验证得出东西。
5. NERO 7 自由度逆解有零空间,要用一致的策略(比如最小关节运动)出干净标签。
6. 深度不喂基座 VLA(X-VLA 只吃 RGB),深度是辅助;但 canonical 是否长期保存 depth 图还要定,否则后续复查深度质量会依赖原始目录。
7. WiLoR 目前只有适配器接口和外部结果导入规范,本仓库还没有安装/接入 WiLoR runtime。
8. Web 端当前只自动分普通视频和外部手部结果文件;RGB-D 目录/压缩包上传还未接入,需要单独加 source 类型。
9. 网格:NERO 视觉网格是 .dae、碰撞是 .stl,MuJoCo 只吃 .stl;检视台用 MeshCat/Rerun(浏览器)绕开了本机的 GL 渲染问题。

## 8. NERO 运动学参考

7 自由度(joint1..joint7),参数来自 pyAgxArm 的 `constants.py`,`assets/nero` 里有对应 URDF。

关节限位(度):

| 关节 | 下限 | 上限 |
|---|---|---|
| J1 | -155 | 155 |
| J2 | -100 | 100 |
| J3 | -158 | 158 |
| J4 | -58 | 123 |
| J5 | -158 | 158 |
| J6 | -42 | 55 |
| J7 | -90 | 90 |

MDH 参数 `(d, a, alpha, theta_offset)`,单位 m / rad:

```
(0.138,   0.0, 0.0, 0.0)
(0.0,     0.0, π/2, π)
(0.31,    0.0, π/2, π)
(0.0,     0.0, π/2, π)
(0.27001, 0.0, π/2, π/2)
(0.0,     0.0, π/2, π/2)
(0.0235,  0.0, π/2, 0.0)
```

## 9. 代码与资产

- 主管线代码在 `sim/`,说明见 `sim/README.md`;NERO 的 FK/IK 是 `sim/nero_kin.py`(纯 pinocchio)。
- `assets/`:NERO 和 inspire 的 URDF 加网格;`configs/`:重定向配置;`data/`:示例视频。都内置在仓库里。
- 装配 `sim/assets/nero_inspire_right.urdf` 由 `build_nero_inspire.py` 生成(NERO link7 挂 inspire base,加载 nq=19)。
- RGB-D 离线导入入口是 `sim/build_canonical_from_rgbd.py`,要求 `color/`、`depth/`、`calibration.json`;Web 端尚未自动调用。
- 外部 MediaPipe/WiLoR 风格结果导入入口是 `sim/build_canonical_from_processed.py`,支持 `.npz/.pkl/.json`。
- 第三方仅这两处用得到:overlays 里的早期可视化器(配 dex-retargeting),真机部署(松灵 pyAgxArm CAN SDK)。

## 10. 现状

- Phase A 完成:装配、逆解、retargeting 驱动、MeshCat 检视台、两层 schema、手势演示。
- Phase B 软件主链完成:手腕 6-DoF 估计、retarget→逆解→回放(SavGol 消抖)、写 LeRobotDataset;C 路线的 dataloader 可消费性也验过。
- 可视化升级(2026-07-13):`sim/replay_rerun.py` 用 Rerun 做同步多面板(人手视频+骨架 / 机器人 3D / 关节曲线),取代 MeshCat 单视图。
- 手腕朝向稳定化(2026-07-14):经它暴露的「臂大幅晃」查明是单目出平面朝向噪声(占漂移 91%),加 `sim/wrist_stabilize.py`(出平面降权 + 残差门限),臂运动 184°→57°、IK 仍 710/710、保面内真手势。完整因子图待 Femto。
- 两层数据架构落地(2026-07-14):`build_canonical.py`→`canonical_ds`(本体无关规范层)+ `derive_embodiment.py --robot X`(按 `robot_specs.py` 的 URDF 派生每本体 `lerobot_ds_X`)。换本体只加一个 RobotSpec、采集不重来。回归:派生轨迹与旧管线 max|Δ|~1e-7 rad。真第二本体属 Phase D。
- RGB-D 离线融合验证(2026-07-24):`build_canonical_from_rgbd.py` 能读取对齐 `color/depth/calibration.json`,用深度把 wrist position 写成度量位置;当前样例生成 557 帧,深度缺失帧用模型回退。为避免 depth-world 手指点污染 retarget,`observation.hand_keypoints` 默认仍保留 MANO/手腕局部 21 点。
- 相对模仿验证(2026-07-24):`derive_embodiment.py` 已支持 `--arm-position-mode relative/fixed`,当前本地验证版默认用 wrist 相对首帧平移解锁臂末端位置,5cm 限幅;动态腕部方向用 `wrist_motion_basis_R` 矩阵做坐标基变换。正式空间模仿仍要补 `T_base_camera`,普通 RGB 视频建议继续用 fixed。
- Web 端入口进展(2026-07-24):已支持上传普通视频和外部处理结果文件;RGB-D 目录/压缩包、纯相机录制包还没接入 Web,目前需要命令行手动生成 canonical。
- 仓库已重构成自足可移植(路径自动定位、assets/configs/data 内置、nero_kin 加 vendored 检测器),推到了 GitHub(ZhangDaMengxx/VLA-HandArm)。
- 剩下的关键资源/任务:Phase C 真训练在 RTX(见 `训练端部署.md`)、真实 RGB-D 相机采集接入、Web RGB-D 上传入口、`observation.images.depth` 是否入 canonical schema、正式 `T_base_camera` 标定、Phase D 第二本体。
- **逐次具体更改见 `更新日志.md`(带时间戳)。**
