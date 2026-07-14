# sim/ 说明

NERO(7 自由度臂)加 inspire 灵巧手的仿真和数据管线代码。方案见仓库根 `PROJECT_PLAN.md`。

## 怎么跑

装好根目录 `requirements.txt` 的依赖后直接跑,比如:

```bash
python sim/build_nero_inspire.py
```

路径都由 `paths.py` 自动定位(仓库根、assets、data、configs、sim/out),换台机器也一样。查看器类脚本会常驻,把它打印的 MeshCat 地址(`http://localhost:PORT/static/`)在浏览器打开就行。

两个已知的坑:WSL 久空闲会休眠,首次调用可能超时,重试即可;MeshCat 端口每次重启会往上加。

## 数据流(端到端总线)

一段 mp4 到能喂 VLA 的训练数据,经两层、五个落点。★ = 持久化产物。

```
data/*.mp4                         唯一源头:真人第一视角手势,30fps
   │
   │  build_canonical.py           规范层「录母带」:MediaPipe 逐帧检测 + 估手腕位姿
   │                               不 retarget、不平滑,纯记录「人做了什么」
   ▼
out/canonical_ds/  ★ 本体无关      LeRobotDataset(robot_type=canonical),每帧:
   │                                 observation.images.ego     256×256 RGB(video)
   │                                 observation.hand_keypoints (63,)=21×3 MANO 米
   │                                 observation.wrist_pose     (7,)=[t3,quat4] 相机系
   │                                 task                       语言指令
   │                               ← 换机器人时这一步不重跑
   │
   │  derive_embodiment.py --robot X   本体层「编译」:canonical + RobotSpec(robot_specs.py)
   │    手: kp → dex-retarget → 12 关节 → 取 6 驱动
   │    臂: wrist_pose → 稳定化(gate+出平面衰减+SavGol)→ NeroKin IK(home 锚定)
   │    拼 state/action = (13) = [7 臂 + 6 手]
   ▼
out/lerobot_ds_X/  ★ 训练数据      LeRobotDataset:observation.state(13) / action(13) / images.ego
out/robot_traj_X.pkl  (--emit-traj) 仅可视化缓存,不是训练数据源
   │
   ▼  消费方(默认读派生产物,缺失自动回退旧产物)
replay_rerun.py --serve   浏览器三联屏(人手视频+骨架 | 机器人3D | 关节曲线)
verify_dataset.py / check_dataloader.py   回读校验 / dataloader 自检

换本体 = 只在 robot_specs.py 加一个 RobotSpec,再 derive_embodiment.py --robot 新名字;
①② 不动。parity_check.py 随时验两条路一致。

# 装配 URDF(与上面数据流并行,供可视化/仿真加载):
build_nero_inspire.py  → sim/assets/nero_inspire_right.urdf → view_meshcat / replay_* / gesture_demo

# 旧单本体管线(NERO 遗留,仍可用,采集时即 retarget→有损不可逆):
detect_wrist.py        → out/full_traj.pkl → build_robot_traj.py (wrist_stabilize) → out/robot_traj.pkl → replay_full.py / build_dataset.py
detect_and_retarget.py → out/hand_traj.pkl → replay_assembly.py
schema.py / robot_specs.py / gestures.py → 供上面各步用
```

## 装配

`build_nero_inspire.py`:把 NERO 臂和 inspire 手的 URDF 合成一个装配 URDF,在 MuJoCo 里验证加载(nq=19,7 臂 + 12 手)。URDF 从 `assets/nero` 和 `assets/inspire_hand` 读。`MOUNT_XYZ`、`MOUNT_RPY` 是手相对法兰(link7)的安装变换,现在是平贴法兰、手沿伸出轴,硬件到手后按真机装法改这两个值。inspire 的根 link 叫 `base`,和 NERO 的 link 名不冲突,所以保留原名(retargeting 靠这些名字)。

## 运动学

`nero_kin.py`:NERO 的正逆运动学,纯 pinocchio,从 assets 的 URDF 读。`fk(q)` 出 4x4 位姿,`ik(T, q_init)` 出关节角(阻尼最小二乘)。这个替代了 pinocchio-kinematics-lite,仓库不再依赖那个第三方库。

`find_home_pose.py`:求一个让法兰朝上(手指竖直)的初始姿态,多随机初值重启。求出的 `q_home = [1.2635, 0.9302, 2.6464, 1.7779, 1.0898, 0.6034, -0.6634]` 已经写进各查看器的 `Q_HOME_ARM`。

## 检视台(MeshCat,浏览器)

- `view_meshcat.py`:静态显示 home 姿态。
- `replay_assembly.py`:把 `out/hand_traj.pkl` 的手指轨迹回放到装配上,臂不动。
- `replay_full.py`:把 `out/robot_traj.pkl` 的完整 [臂+手] 轨迹回放,人手视频驱动机械臂加灵巧手。
- `gesture_demo.py`:循环展示手势预设(open / point / victory / thumbs_up / ok / fist),平滑过渡。

## 同步多面板可视化(Rerun,推荐)

`replay_rerun.py`:比 MeshCat 直观得多的同步可视化。三块面板同一时间轴硬同步 —— **Human**(源视频 + MediaPipe 骨架)、**Robot 3D**(NERO+inspire 装配网格,可鼠标轨道旋转/缩放)、**关节角曲线**(臂 7 + 手 12,游标跟随当前帧)。读 `out/robot_traj.pkl` 回放(不实时 retarget)。

```bash
python sim/replay_rerun.py                  # 默认存 out/replay.rrd,用 Rerun 查看器打开
python sim/replay_rerun.py --serve          # 起 web,在 Windows 浏览器开打印出来的完整 URL(带 ?url= 数据源)
# A/B 对比两条轨迹(各成实体树,左侧勾选显隐;关节曲线同图叠看)
python sim/replay_rerun.py --traj raw=out/robot_traj_raw.pkl --traj stab=out/robot_traj.pkl --save out/replay_ab.rrd
```

坑:`--serve` 后要开的是**完整 URL**(脚本会打印,含 `?url=rerun+http://<WSL-IP>:9876/proxy`),裸开 `http://IP:9090` 只有空欢迎页;数据源主机用 WSL IP(127.0.0.1 从 Windows 连不到)。NERO 视觉网格是 `.dae`,脚本自动回退到同目录 `.stl`(免装 pycollada)。视频帧走 JPEG 编码(否则 .rrd 会大一个数量级)。

## 两层数据架构(跨本体复用,推荐)

核心设计:把「人做了什么」(规范层,本体无关)和「某台机器人该转哪些关节」(本体层,每机器人一份)分开存。规范层是长期资产;换机器人只需拿它按新 URDF 重新派生,采集不重来。**为什么必须这样**:人手 21 点 → 机器人关节是**有损不可逆**投影(inspire 6 驱动关节丢掉了人手大量信息),只存本体层就等于「只留编译产物、丢了源码」,换本体就废。

- `build_canonical.py`:视频 → `out/canonical_ds`(LeRobotDataset)。每帧存 ego RGB(video)+ `observation.hand_keypoints`(21×3 MANO 米,扁平成 63)+ `observation.wrist_pose`(7=平移3+四元数4,相机系)+ task。**不 retarget、不平滑**,是纯母带。
- `robot_specs.py`:`RobotSpec`(URDF + 重定向配置 + 关节布局 + Q_HOME + 稳定化参数)。已定义 `nero_inspire`。**换机器人 = 加一个 RobotSpec**。
- `derive_embodiment.py --robot X`:读 `canonical_ds` + spec → 逐帧 retarget 手 + 稳定化/IK 臂 + SavGol → 写 `out/lerobot_ds_X`(+ `--emit-traj` 出 `robot_traj_X.pkl` 供 replay)。这是两层架构的「编译」步。
- 验证:derive 出的轨迹与旧 `build_robot_traj`(同参数)数值一致(max|Δ|~1e-7 rad),即两层重构无回归。
- 说明:LeRobotDataset 只是容器;规范层和本体层都装在里面,区别是**存什么列、绑不绑机器人**。keypoints/wrist 存扁平向量(多维 float 特征本版本未验证,已 probe 确认扁平安全)。`wrist_pose` 现处于相机系(单目位置近似),Femto 到手后换度量世界系、derive 去掉 home 锚定。

```bash
python sim/build_canonical.py                    # 视频 → 规范层数据集
python sim/derive_embodiment.py --emit-traj      # 规范层 → nero_inspire 本体数据集(+轨迹)
```

## 数据管线(旧单本体路径,NERO 遗留)

`detect_wrist.py`:整段视频过 MediaPipe 加 dex-retargeting 出手指轨迹,再估手腕 6-DoF,存 `out/full_traj.pkl`。配置从 `configs/inspire_hand_right_local.yml` 读,URDF 从 `assets` 读,视频从 `data` 读。手部检测器是 `sim/single_hand_detector.py`(从 dex-retargeting 拷来的)。

`estimate_wrist.py`:手腕位姿估计。朝向来自 MediaPipe,比较可靠;位置用手掌尺度反推深度,是单目近似。留了 `depth_lookup` 接口,Femto 到手把真实深度传进去就准了。

`detect_and_retarget.py`:只出手指轨迹(`out/hand_traj.pkl`),给 `replay_assembly` 用。`detect_wrist` 是它的超集,多了手腕估计。

`build_robot_traj.py`:把手腕位姿逆解成臂关节再消抖,出 `out/robot_traj.pkl`。臂用手腕朝向(相对首帧)驱动,位置锚定在 home 可达点(度量位置和相机系到基座的对齐等 Femto)。消抖用 Savitzky-Golay(离线、零滞后、保幅度),比朴素低通好;实时驱动真机时换 1€ 滤波器。手指张开幅度是重定向映射决定的,不是低通造成的,想更大就调配置里的 `scaling_factor`,代价是握拳变弱。

**手腕朝向稳定化**(见 `wrist_stabilize.py`):实验发现臂晃动几乎全来自手腕朝向相对首帧漂到 43°,其中 91% 是**出平面**(手掌法向倾斜,单目深度估不准),面内(图像内滚转)只有几度、基本是真手势。故 `build_robot_traj` 默认开两道稳定化:`--gate-deg`(默认 8,残差门限剔离群跳变帧)+ `--oop-alpha`(默认 0.4,衰减出平面朝向分量、保面内)。效果:臂运动总幅度 184°→57°、IK 仍 710/710 全收敛,真手势保留。关掉复现基线:`--oop-alpha 1.0 --gate-deg 0`。这是各向异性可观测性加权的轻量近似;完整 RTS/因子图待 Femto 深度。

`build_dataset.py`:本体层轨迹加视频帧加语言标签打成 LeRobotDataset(`out/lerobot_ds`)。state/action = [7 臂 + 6 手驱动] = 13。两个坑:`create` 要带 `metadata_buffer_size=1` 才把 episode 元数据落盘;加载要设 `HF_HUB_OFFLINE=1`,不然会去连 huggingface。

`schema.py`:两层数据结构。`CanonicalFrame` 是本体无关的人手数据(ego 图、手关键点、手腕位姿、语言);`EmbodimentFrame` 是一条 LeRobotDataset 记录(state 13、action 13、图、语言)。`canonical_to_embodiment` 把前者转后者(retarget 手加 IK 臂)。

`gestures.py`:手势预设。`full12()` 从 6 个驱动关节按 mimic 比例算出完整 12 关节。数值是第一版,可按视觉微调。

`verify_dataset.py` / `check_dataloader.py`:回读数据集验证;验证 dataloader 能出 ACT 风格的动作块 batch。

`parity_check.py`:并行跑旧单本体路径和新两层路径(同一视频),比对两者的 `robot_traj`,确认两层重构无回归(PASS 阈值 max|Δ|<1e-4 rad)。当前策略是两条路并行、稳了再把默认切到两层路径;每次换新视频或改了任一路径的代码,跑这个复验。`probe_canonical_feats.py`:探 LeRobotDataset 对扁平 float 特征的支持(规范层存储选型用)。

## 开发 / 诊断脚本

`analyze_mount.py`、`analyze_overlap.py` 当初用来定安装朝向、查网格有没有重合;`diag_jitter.py`、`diag_hand_amp.py` 量抖动和手指幅度;`diag_gl.py` 探过 GL 后端(结论是本机 MuJoCo 离屏渲染不行,改用 MeshCat);`render_assembly.py` 已废弃;`probe_lerobot.py`、`probe_policies.py` 探 lerobot 的 API 和可用 policy;`test_nero_kin.py` 测 nero_kin。

## 生成物(不进仓库,可重建)

`sim/assets/nero_inspire_right.urdf`(装配,含绝对 mesh 路径)、`out/*.pkl`(各阶段轨迹)、`out/lerobot_ds/`(数据集)。

## 端到端(两层路径 = 当前默认)

```bash
python sim/build_nero_inspire.py             # 装配
python sim/build_canonical.py                # 视频 → 规范层 canonical_ds(本体无关)
python sim/derive_embodiment.py --emit-traj  # 规范层 → nero_inspire 本体数据集 + 轨迹
python sim/replay_rerun.py --serve           # Rerun 可视化(默认读派生轨迹)
```

消费方(`replay_rerun` / `replay_full` / `verify_dataset` / `check_dataloader`)默认读两层派生产物(`robot_traj_nero_inspire.pkl` / `lerobot_ds_nero_inspire`),产物不存在时自动回退旧单本体产物。

旧单本体管线仍可用(应急/对比):`detect_wrist.py → build_robot_traj.py → build_dataset.py`。切换前用 `parity_check.py` 复验两条路一致。
