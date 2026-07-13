# sim/ 说明

NERO(7 自由度臂)加 inspire 灵巧手的仿真和数据管线代码。方案见仓库根 `PROJECT_PLAN.md`。

## 怎么跑

装好根目录 `requirements.txt` 的依赖后直接跑,比如:

```bash
python sim/build_nero_inspire.py
```

路径都由 `paths.py` 自动定位(仓库根、assets、data、configs、sim/out),换台机器也一样。查看器类脚本会常驻,把它打印的 MeshCat 地址(`http://localhost:PORT/static/`)在浏览器打开就行。

两个已知的坑:WSL 久空闲会休眠,首次调用可能超时,重试即可;MeshCat 端口每次重启会往上加。

## 数据流

```
build_nero_inspire.py  → sim/assets/nero_inspire_right.urdf → view_meshcat / replay_* / gesture_demo
detect_wrist.py        → out/full_traj.pkl → build_robot_traj.py → out/robot_traj.pkl → replay_full.py / build_dataset.py
detect_and_retarget.py → out/hand_traj.pkl → replay_assembly.py
schema.py / gestures.py → 供 gesture_demo 和 build_dataset 用
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

## 数据管线

`detect_wrist.py`:整段视频过 MediaPipe 加 dex-retargeting 出手指轨迹,再估手腕 6-DoF,存 `out/full_traj.pkl`。配置从 `configs/inspire_hand_right_local.yml` 读,URDF 从 `assets` 读,视频从 `data` 读。手部检测器是 `sim/single_hand_detector.py`(从 dex-retargeting 拷来的)。

`estimate_wrist.py`:手腕位姿估计。朝向来自 MediaPipe,比较可靠;位置用手掌尺度反推深度,是单目近似。留了 `depth_lookup` 接口,Femto 到手把真实深度传进去就准了。

`detect_and_retarget.py`:只出手指轨迹(`out/hand_traj.pkl`),给 `replay_assembly` 用。`detect_wrist` 是它的超集,多了手腕估计。

`build_robot_traj.py`:把手腕位姿逆解成臂关节再消抖,出 `out/robot_traj.pkl`。臂用手腕朝向(相对首帧)驱动,位置锚定在 home 可达点(度量位置和相机系到基座的对齐等 Femto)。消抖用 Savitzky-Golay(离线、零滞后、保幅度),比朴素低通好;实时驱动真机时换 1€ 滤波器。手指张开幅度是重定向映射决定的,不是低通造成的,想更大就调配置里的 `scaling_factor`,代价是握拳变弱。

`build_dataset.py`:本体层轨迹加视频帧加语言标签打成 LeRobotDataset(`out/lerobot_ds`)。state/action = [7 臂 + 6 手驱动] = 13。两个坑:`create` 要带 `metadata_buffer_size=1` 才把 episode 元数据落盘;加载要设 `HF_HUB_OFFLINE=1`,不然会去连 huggingface。

`schema.py`:两层数据结构。`CanonicalFrame` 是本体无关的人手数据(ego 图、手关键点、手腕位姿、语言);`EmbodimentFrame` 是一条 LeRobotDataset 记录(state 13、action 13、图、语言)。`canonical_to_embodiment` 把前者转后者(retarget 手加 IK 臂)。

`gestures.py`:手势预设。`full12()` 从 6 个驱动关节按 mimic 比例算出完整 12 关节。数值是第一版,可按视觉微调。

`verify_dataset.py` / `check_dataloader.py`:回读数据集验证;验证 dataloader 能出 ACT 风格的动作块 batch。

## 开发 / 诊断脚本

`analyze_mount.py`、`analyze_overlap.py` 当初用来定安装朝向、查网格有没有重合;`diag_jitter.py`、`diag_hand_amp.py` 量抖动和手指幅度;`diag_gl.py` 探过 GL 后端(结论是本机 MuJoCo 离屏渲染不行,改用 MeshCat);`render_assembly.py` 已废弃;`probe_lerobot.py`、`probe_policies.py` 探 lerobot 的 API 和可用 policy;`test_nero_kin.py` 测 nero_kin。

## 生成物(不进仓库,可重建)

`sim/assets/nero_inspire_right.urdf`(装配,含绝对 mesh 路径)、`out/*.pkl`(各阶段轨迹)、`out/lerobot_ds/`(数据集)。

## 端到端

```bash
python sim/build_nero_inspire.py    # 装配
python sim/detect_wrist.py          # 视频 → 手指+手腕轨迹
python sim/build_robot_traj.py      # 逆解+平滑 → 机器人轨迹
python sim/replay_full.py           # 浏览器回放
python sim/build_dataset.py         # 打包 LeRobotDataset
```
