# 机械臂 + 灵巧手 手势 / VLA 数据管线

把一段人手视频转成机械臂(松灵 NERO,7 自由度)加灵巧手(inspire)的关节轨迹,再打包成 LeRobotDataset,用来验证这套数据能不能拿去训 VLA。

在 CPU 的 WSL 上跑,负责出数据和在浏览器里看效果;训练放到带 GPU 的机器上(见 `训练端部署.md`)。方案和进度见 `PROJECT_PLAN.md`,代码细节见 `sim/README.md`。

## 跑起来

Python 3.10 环境:

```bash
pip install -r requirements.txt
python sim/build_nero_inspire.py     # 生成 NERO+inspire 装配
python sim/gesture_demo.py           # 手势演示;浏览器打开它打印的 http://localhost:PORT/static/
```

完整管线:

```bash
python sim/detect_wrist.py       # 视频 → 手指 + 手腕轨迹
python sim/build_robot_traj.py   # 逆解 + 平滑 → 机器人轨迹
python sim/replay_full.py        # 浏览器回放整条 臂+手
python sim/build_dataset.py      # 打包成 LeRobotDataset
```

URDF、配置、示例视频都在仓库里(`assets/` `configs/` `data/`),脚本路径自动定位,换台机器 clone 下来装好依赖一样能跑。查看器走 MeshCat,在浏览器打开打印出来的地址(WSL 的 localhost 会转发到 Windows)。

## 目录

```
sim/         全部代码,说明见 sim/README.md
assets/      NERO 和 inspire 的 URDF 加网格
configs/     重定向配置
data/        示例视频,可换成自己的 mp4
overlays/    早期在 dex-retargeting 里加的可视化器和实验,可选
PROJECT_PLAN.md / 训练端部署.md / VISUALIZER_SPEC.md   文档
```

## 依赖

装 `requirements.txt` 里的包:mujoco、pin(pinocchio)、dex_retargeting、mediapipe、lerobot、scipy、opencv-python、meshcat、numpy、torch。手部检测器和 NERO 的正逆运动学已经放进 `sim/`(`nero_kin.py`、`single_hand_detector.py`),不用另装 pinocchio-kinematics-lite。

## 不进仓库

`sim/out/`(轨迹、数据集)和 `outputs/`(训练输出)都能重新生成,没有提交。
