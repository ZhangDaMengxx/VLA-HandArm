# 机械臂 + 灵巧手 · 手势 → 抓取 → VLA 项目计划

> 状态:规划已锁定,**Phase A-1 进行中** · 更新 2026-07-09
> 本文档是项目的单一事实来源(SSOT)。任务列表见 Kiro 任务 #26–#34。

---

## 1. 最终目标

**一句话**:搭一条"真人第一视角 RGB-D 演示 → retarget 成机器人动作 → 存 LeRobotDataset → 本地 RTX 上 LoRA 微调 X-VLA → 验证该数据能训出有效策略"的完整管线。

**交付物**:这个**验证实验本身** —— 证明"用 ego + 深度 + 视频这套方式采到的数据确实能训练出策略",而不是一个产品级机器人。

**三阶段愿景**:
- **A. 手势 / 摆姿势** —— 纯运动学,不碰物体(打通臂+手运动链)。
- **B. 深度视觉 + 抓取物体** —— LLM 发高层指令,真人第一视角采集在此产出数据集。
- **C. VLA 路线** —— EGO 范式采集(RGB+深度+视频)→ LoRA 微调 X-VLA,验证数据可训练。

---

## 2. 系统架构(分层控制栈)

```
   自然语言指令   "给我比个点赞" / "拿起红色方块"
        │
   ┌────▼─────────────────────────────┐
   │  LLM 规划层 (high-level planner)   │  语言 → 技能调用序列
   └────┬─────────────────────────────┘
        │  ← 联系②:LLM ↔ 机器人 = 函数/工具调用
   ┌────▼─────────────────────────────┐
   │  技能包 (skill library / 原语)      │  point() / grasp() / move_to(pose)
   └────┬─────────────────────────────┘
   ┌────▼──────────────┬──────────────┐
   │  手臂控制器 (IK)    │  灵巧手控制器  │  ← 联系①:臂↔手,手装在臂末端,一条运动链
   │  决定"去哪"(6-DoF) │ (retargeting) │     臂管到哪、手管怎么抓/比
   └────┬──────────────┴──────┬───────┘
   ┌────▼─────────────────────▼───────┐
   │        仿真器 (MuJoCo 物理+渲染)    │  ← 联系③在此执行与验证
   └──────────────────────────────────┘
```

---

## 3. 选型锁定

| 维度 | 锁定值 |
|---|---|
| 机械臂 | 松灵 NERO,7-DoF,3kg(URDF 待补;运动学参数已从 pyAgxArm 提取,见 §8) |
| 灵巧手 | inspire(6 驱动关节,已在做 dex-retargeting) |
| 数据源 | 真人第一视角:早期 Orbbec Femto(稠密 RGB-D);后期 Aria 类(轻量 RGB+轨迹) |
| 数据格式 | LeRobotDataset(NERO 原生 HDF5 → 桥接) |
| 目标模型 | LeRobot 生态 VLA(X-VLA)+ LoRA 微调 |
| 训练 | 本地 RTX 工作站 |
| 开发/采集/仿真 | CPU-WSL(Ubuntu 22.04)+ MuJoCo |
| **Phase A 仿真替身** | **rm75_inspire**(睿尔曼 RM75 7-DoF + inspire,装配 URDF 已在仓库,开箱即用) |

> 替身理由:RM75 与 NERO 都是 7-DoF 臂 + inspire 手;管线按 URDF 参数化设计,RM75→NERO 仅换配置,且两者天然构成 Phase D 主体无关性的两个本体。

---

## 4. 两层数据架构 + Schema

**规范层(本体无关,真正资产)**:ego RGB-D + 人手关键点 + 手腕 6-DoF(度量/统一坐标系)+ 物体/场景 + 语言。

**本体层(每机器人一份)**:对规范层按 **URDF 参数化 retarget** → LeRobotDataset。

| 字段 | 内容 |
|---|---|
| `observation.images.ego` | 第一视角 RGB(缩到 VLA 输入尺寸,存视频)→ 喂 VLA |
| `observation.images.depth` | 对齐深度图 → **辅助**(恢复手腕度量位姿/3D grounding),不喂基座 VLA |
| `observation.state` | [7 臂 + 6 手] = 13 维机器人本体状态 |
| `action` | [7 臂 + 6 手] = 13 维,**绝对关节目标**(retarget 直接输出,标签无歧义) |
| `task` | 自然语言指令(LLM 生成/规整) |

换新臂手:只换 URDF,规范层一字不动。

---

## 5. 主体无关性(能验证什么,别搞混)

| 含义 | 能否验证 | 说明 |
|---|---|---|
| ① 原始人手数据本体无关、可复用 | ✅ 天然成立 | 演示里没有机器人,采一次喂多个机器人 |
| ② 管线/数据级本体无关(换 URDF 即可) | ✅ 需实跑 ≥2 本体 | retarget 按 URDF 参数化,同批数据跑通两套臂手 |
| ③ 单模型跨本体 | ❌ 不在范围 | 需多本体共训,是下一个项目 |

**"数据符合"是必要不充分**,三个 gap:动作 gap(目标本体可达性封顶,如 inspire 无侧摆→人手扇形张开无法复现)、观测 gap(训练是人手/部署是机器手)、动力学 gap。retarget 保真度 = 标签质量天花板。

**方案一(已选)**:先在单本体(替身 rm75_inspire → 目标 NERO+inspire)跑通拿到第一个结果;之后加第二本体(如 Franka+Allegro)正式坐实 ①②。

---

## 6. 分阶段计划表

| 步 | 阶段 | 任务 | 做什么 | 机器/工具 | 产出 | 状态 |
|---|---|---|---|---|---|---|
| 1 | A | #26 | 组装 7-DoF 臂+inspire 运动链(rm75_inspire + nero_inspire) | WSL/MuJoCo | 两装配加载 nq=19 | 完成 ✓ |
| 2 | A | #27 | 逆解 + retargeting 驱动 + MeshCat 检视台 | WSL | 视频→retarget→回放 | 完成 ✓ |
| 3 | A | #28 | 两层 schema(sim/schema.py)+ 手势预设 + 演示 | WSL | schema + 手势 demo | 完成 ✓ |
| 4 | B | #29 | Orbbec Femto 同步 RGB-D 采集 | Femto+WSL | 原始 RGB-D | 待办 |
| 5 | B | #30 | 人手 + 手腕 6-DoF 估计(sim/estimate_wrist.py,朝向可靠+位置单目近似/可插拔深度) | WSL | full_traj.pkl(手+腕) | 完成 ✓ |
| 6 | B | #31 | retarget→逆解→回放(SavGol 消抖;臂朝向驱动,位置待 Femto) | WSL | robot_traj.pkl + 回放 | 完成 ✓ |
| 7 | B | #32 | 写 LeRobotDataset(state13/action13/ego视频/task;离线需 HF_HUB_OFFLINE=1) | WSL | lerobot_ds(710帧/1ep) | 完成 ✓ |
| 8 | C | #33 | 验证数据可训:先 ACT/DP 从头训(A/B/C 同数据);CPU 已验 dataloader 可消费,真训在 RTX(装标准 LeRobot) | RTX | 验证结论 | 进行中 |
| 9 | D | #34 | 第二本体复跑,坐实主体无关性 | RTX | 主体无关证据 | 待办(依赖 C) |

---

## 7. 风险与约束

1. **retarget 保真度 = 标签质量天花板**(真人手→inspire 有本体差异,见张开幅度/无侧摆问题)。
2. **手腕 6-DoF 估计是新硬骨头**(比手指难,Phase B 主要技术风险)。
3. **换设备(Femto→Aria)破坏视觉一致性**(视角/深度形态变,需重采或域随机化)。
4. **数据量**:LoRA 微调窄任务需几十~上百条 episode 才验证得出。
5. **NERO 7-DoF 逆解有零空间**,需一致策略(如最小关节运动)出干净标签。
6. **深度不喂基座 VLA**(X-VLA 只吃 RGB),深度是辅助标注。
7. **视觉网格**:NERO URDF+网格已在 `pinocchio-kinematics-lite`(视觉 .dae、碰撞 .stl);MuJoCo 不吃 .dae/.glb 视觉,用碰撞网格显示。检视台改用 MeshCat(浏览器)规避本机 GL 渲染问题(无 libOSMesa、EGL 撞 PyOpenGL3.1.0)。

---

## 8. NERO 运动学参考(来自 pyAgxArm SDK)

**自由度**:7(joint1..joint7)。**来源**:`pyAgxArm-master/.../pyAgxArm/api/constants.py`。

**关节限位(度)**:

| 关节 | 下限 | 上限 |
|---|---|---|
| J1 | -155 | 155 |
| J2 | -100 | 100 |
| J3 | -158 | 158 |
| J4 | -58 | 123 |
| J5 | -158 | 158 |
| J6 | -42 | 55 |
| J7 | -90 | 90 |

**MDH 参数** `(d, a, alpha, theta_offset)` 单位 m/rad:
```
(0.138,   0.0, 0.0,      0.0)
(0.0,     0.0, π/2,      π)
(0.31,    0.0, π/2,      π)
(0.0,     0.0, π/2,      π)
(0.27001, 0.0, π/2,      π/2)
(0.0,     0.0, π/2,      π/2)
(0.0235,  0.0, π/2,      0.0)
```
足够重建运动学精确的 FK/IK 链;缺外观网格。

---

## 9. 工作区现有资产(已扫描确认)

- **rm75_inspire 装配 URDF**(Phase A 替身):`dex-retargeting-main/.../assets/robots/assembly/rm75_inspire/rm75_inspire_right_hand.urdf`(带网格/惯量/限位)。
- **各 7-DoF 臂 URDF**:iiwa7/iiwa14、ur5e/ur10e、xarm7、rm75(在 `dex-urdf-main` 与 dex-retargeting assets)。
- **各灵巧手 URDF**:inspire、allegro、shadow、ability、leap、schunk、barrett。
- **inspire 手**:`.../assets/robots/hands/inspire_hand/inspire_hand_right.urdf`(+ 本项目已有的 `inspire_hand_right_local.yml` 张开幅度修复配置)。
- **dex-retargeting**:手部 retargeting 引擎(已在用)。
- **pyAgxArm SDK**:NERO/piper 真机 CAN 控制驱动 + MDH 运动学(部署真机时用)。
- **NERO URDF + 网格 + Pinocchio IK**(位于 `pinocchio-kinematics-lite-main`):`assets/nero/nero_description.urdf`(7-DoF 臂到 link7,STL 碰撞网格 + DAE 视觉)+ 现成 `NeroKinematics` FK/IK 库。
- **已生成装配**:`sim/assets/nero_inspire_right.urdf`(NERO link7 挂 inspire base),MuJoCo 加载 nq=19。

---

## 10. 当前状态与下一步

- [完成] 规划锁定并写入记忆;环境就绪(WSL 已装 mujoco 3.8.1 / pinocchio 4.0.0 / dex_retargeting / mediapipe,无需安装)。
- [完成] **Phase A-1**:rm75_inspire 与 nero_inspire 两个装配均在 MuJoCo 加载成功(各 nq=19,7 臂 + 12 手关节)。
- [下一步] **Phase A-2**:NeroKinematics 逆解 + dex-retargeting 驱动手 + 回放检视台;标定 link7→hand 安装变换。
