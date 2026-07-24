# sim/ 说明

NERO(7-DoF 臂)+ inspire 灵巧手的仿真与数据管线代码。总体方案见仓库根 `PROJECT_PLAN.md`,快速上手见根 `README.md`。

## 数据流(两层架构)

核心设计:把「人做了什么」(**规范层**,本体无关)和「某台机器人转哪些关节」(**本体层**,每机器人一份)分开存。人手 21 点 → 机器人关节是**有损不可逆**投影;只存本体层等于「只留编译产物、丢了源码」,换本体就废。规范层是长期资产,换机器人只按新 URDF 重派生,采集不重来。

```
data/*.mp4                         真人第一视角手势(30fps)
   │  build_canonical.py           规范层「录母带」:HandEstimator 逐帧检测 + 估手腕位姿,不 retarget、不平滑
   ▼
out/canonical_ds/  ★ 本体无关      LeRobotDataset(canonical):ego RGB + hand_keypoints(21×3 MANO)
   │                                 + hand_keypoints_2d + hand_visibility + wrist_pose(7,相机系) + task
   │  derive_embodiment.py --robot X   本体层「编译」:canonical + RobotSpec
   │    手: kp → dex-retarget → 12 关节 → 取 6 驱动
   │    臂: wrist_pose → 稳定化 → NeroKin IK(home 锚定;可选位置相对首帧跟随)
   ▼
out/lerobot_ds_X/  ★ 训练数据      LeRobotDataset:state(13) / action(13) / images.ego
out/robot_traj_X.pkl (--emit-traj)  仅可视化缓存,非训练数据源
   │  replay_rerun.py --serve      浏览器三联屏(人手视频+骨架 | 机器人3D | 关节曲线)
```

换本体 = 只在 `robot_specs.py` 加一个 RobotSpec,再 `derive_embodiment.py --robot 新名字`;规范层不动。

## 端到端

```bash
python sim/build_nero_inspire.py             # 装配 URDF(一次即可)
python sim/build_canonical.py                # 视频 → 规范层(--video 指定视频,--hand-estimator mediapipe)
python sim/build_canonical_from_processed.py --input hand_result.npz
python sim/build_canonical_from_rgbd.py --input-root kinect2_middle/kinect2_middle --camera kinect2_middle
python sim/derive_embodiment.py --emit-traj  # 规范层 → 本体数据集 + 轨迹
python sim/replay_rerun.py --serve           # Rerun 三面板可视化
```

拖拽上传视频的一键图形界面见根 `README.md` 的 `app_gradio.py`。

## 各组件

**装配** `build_nero_inspire.py`:合成 NERO 臂 + inspire 手的装配 URDF,MuJoCo 验证加载(nq=19)。`MOUNT_XYZ/MOUNT_RPY` 是手相对法兰(link7)的安装变换,真机到手后按实装改。

**运动学** `nero_kin.py`:NERO 正逆运动学,纯 pinocchio 从 URDF 读。`fk(q)`→4x4 位姿,`ik(T,q_init)`→关节角(阻尼最小二乘)。home 姿态(法兰朝上)存在 `robot_specs.py` 的 `q_home`。`test_nero_kin.py` 是它的单测。

**规范层** `build_canonical.py`:整段视频过 `hand_estimators.py` 统一接口。当前可用 `mediapipe` 后端,输出 21 点、2D 点、visibility 和 wrist pose;`wilor` 入口已预留,接入时必须 remap 到 canonical 21 点顺序。`estimate_wrist.py` 估手腕 6-DoF(位置用手掌尺度反推深度,单目近似,留了 `depth_lookup` 接口等 Femto 深度)。存 `out/canonical_ds`,不 retarget、不平滑。

**外部处理结果导入** `build_canonical_from_processed.py`:把其他电脑跑好的 MediaPipe/WiLoR 结果导成 canonical。支持 `.npz/.pkl/.json`;最低字段为 `hand_keypoints` `(N,21,3)`/`(N,63)` 和 `wrist_pose` `(N,7)`/`(N,4,4)`,可选 `hand_keypoints_2d`、`hand_visibility`、`fps`、`hand_estimator_id`。Web 左侧“上传手部结果”按钮走这条路径;没有原视频时 Human 面板会从 canonical 画 2D/3D 投影手部骨架,用于对照机器人重定向是否贴合。

**RGB-D 融合导入** `build_canonical_from_rgbd.py`:读取 `color/frameXXX.png` + `depth/frameXXX.png` + `calibration.json`。先用 `--hand-estimator mediapipe` 得到 21 个 2D 点,再从 aligned depth 按内参反投影得到 wrist 的 metric 世界系位置,最后用 `extrinsics.direction=camera_to_world` 的 `wTc` 转世界系,写入 metric `observation.wrist_pose`。`observation.hand_keypoints` 默认仍写 MANO/手腕局部系,因为 dex-retargeting 要求 `joint_pos` 是 MANO 局部 21 点;如果强行写每个关键点的 depth-world 3D,手指容易查到物体/背景深度而误握拳。当前默认保持 MediaPipe 单手提取,默认 `--target-hand Right --max-num-hands 1`;在 `selfie=False` 下这个标签对应当前样例里画面上的目标手。需要调试逐点深度时可显式加 `--hand-keypoints-source depth_world`。

**本体层** `derive_embodiment.py`:读 canonical + `RobotSpec`(`robot_specs.py`)→ 手 retarget + 臂稳定化/IK + SavGol → `out/lerobot_ds_X`(加 `--emit-traj` 出轨迹)。state/action = (13) = [7 臂 + 6 手驱动]。臂末端位置默认仍用稳定的 home 锚点;要验证腕部位置解锁可加 `--arm-position-mode relative --arm-position-limit 0.05`,脚本会使用 `wrist_pose` 相对首帧的平移量叠加到 home 末端位置。

> **手腕朝向稳定化**(`wrist_stabilize.py`):臂晃动几乎全来自手腕朝向相对首帧漂到 43°,其中 91% 是**出平面**(手掌法向倾斜,单目深度估不准),面内滚转只有几度、基本是真手势。故 derive 默认开两道:`gate_deg`(残差门限剔离群跳变帧)+ `oop_alpha`(衰减出平面分量、保面内),参数在 RobotSpec 里。效果:臂运动幅度 184°→57°、IK 全收敛、真手势保留。是各向异性可观测性加权的轻量近似;完整 RTS/因子图待 Femto 深度。

**可视化** `replay_rerun.py`:三面板同一时间轴硬同步——Human(视频 + MediaPipe 骨架)、Robot 3D(装配网格,鼠标轨道旋转)、关节角曲线(游标跟随)。读 `robot_traj_*.pkl` 回放,不实时 retarget。

```bash
python sim/replay_rerun.py                 # 存 out/replay.rrd,Rerun 查看器打开
python sim/replay_rerun.py --serve         # 起 web,浏览器开打印的完整 URL
python sim/replay_rerun.py --traj a=out/robot_traj_raw.pkl --traj b=out/robot_traj.pkl   # A/B 对比
```

坑:`--serve` 要开脚本打印的**完整 URL**(含 `?url=rerun+http://<WSL-IP>:9876/proxy`),裸开 `IP:9090` 只有空欢迎页;数据源主机用 WSL IP(127.0.0.1 从 Windows 连不到)。视觉网格 `.dae` 自动回退同名 `.stl`(免装 pycollada);视频帧走 JPEG 编码,否则 .rrd 大一个数量级。

**数据结构** `schema.py`:锁定的两层 schema(canonical 帧 / embodiment 帧),含 `STATE_DIM`。
**手部估计器接口** `hand_estimators.py`:把 MediaPipe / WiLoR 等模型统一成 canonical `HandObservation`。公共输出是 `keypoints_3d`、`keypoints_2d`、`visibility`、`wrist_pose`;WiLoR/MANO 富层走 `mano` 字段。
**校验** `verify_dataset.py`:回读校验 LeRobotDataset(探正确的属性名)。
**学习脚本**(与管线无关,自用):`print_jacobian.py`(把某姿势的雅可比打屏看懂 J)、`solve_qp_step.py`(用雅可比把「末端想这么动」解成关节速度)。
**路径** `paths.py`:集中路径工具(各入口目前用 `__file__` 自动定位,此模块备用)。

## 生成物(不进仓库,可重建)

`sim/assets/nero_inspire_right.urdf`(装配)、`out/canonical_ds`、`out/lerobot_ds_*`、`out/robot_traj_*.pkl`、`out/*.rrd`。
