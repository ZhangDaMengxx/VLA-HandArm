# 机械臂 + 灵巧手 · 手势 → 抓取 → VLA 数据管线

在 CPU-WSL 上把**真人手视频** → 重定向 + 手腕估计 + 逆解 + 平滑 → **机器人 [臂+手] 轨迹** → 打包成 **LeRobotDataset**,用于验证"采集的数据可训练 VLA"。

- 完整方案与进度:`PROJECT_PLAN.md`
- 主管线代码说明:`sim/README.md`
- 训练端(RTX)部署:`训练端部署.md`

---

## 快速开始(clone 到任何机器都能跑)

```bash
# 1. 装依赖(Python 3.10 环境)
pip install -r requirements.txt

# 2. 生成 NERO+inspire 装配(从内置 assets/ 读 URDF)
python sim/build_nero_inspire.py

# 3a. 手势演示(浏览器打开打印的 http://localhost:PORT/static/)
python sim/gesture_demo.py

# 3b. 或跑完整数据管线:视频 → 机器人轨迹 → 回放/数据集
python sim/detect_wrist.py       # 内置视频 → 手指+手腕轨迹 (sim/out/full_traj.pkl)
python sim/build_robot_traj.py   # 逆解+平滑 → 机器人轨迹 (sim/out/robot_traj.pkl)
python sim/replay_full.py        # 浏览器回放整条 [臂+手]
python sim/build_dataset.py      # → LeRobotDataset (sim/out/lerobot_ds)
```

**`assets/`(URDF+网格)、`configs/`(重定向配置)、`data/`(示例视频)都已内置**,路径全自动定位——不需要第三方仓库,换台机器 clone 下来装好依赖即可运行。查看器脚本会常驻并打印 MeshCat 地址,在浏览器打开即可(WSL 会把 localhost 转发到 Windows)。

## 仓库结构(自研)

| 路径 | 说明 |
|---|---|
| `sim/` | 主管线全部代码(装配、`nero_kin` 逆解、retarget 驱动、MeshCat 检视台、schema、手势、数据集)。详见 `sim/README.md` |
| `assets/` | 内置:`nero/`(NERO URDF+网格)、`inspire_hand/`(inspire URDF+网格) |
| `configs/` | `inspire_hand_right_local.yml`(开合修复的重定向配置) |
| `data/` | 示例手部视频(可替换成你自己的 mp4) |
| `overlays/dex-retargeting/` | 早期塞进 dex-retargeting 的可视化器/实验(**可选**,需配 dex-retargeting 仓库) |
| `PROJECT_PLAN.md` / `训练端部署.md` / `VISUALIZER_SPEC.md` | 文档 |

## 依赖

- **pip 装**(见 `requirements.txt`):mujoco、pin(pinocchio)、dex_retargeting、mediapipe、lerobot、scipy、opencv-python、meshcat、numpy、torch。
- `SingleHandDetector` 和 NERO 的 FK/IK(`sim/nero_kin.py`)已 **vendored/自研**进 `sim/`,不再依赖 pinocchio-kinematics-lite 仓库。
- **第三方仓库不入库**,只有两处用得到:`overlays/` 的可视化器(配 dex-retargeting)、真机部署(松灵 `pyAgxArm` CAN SDK)。

## 未入库(可重建 / 属产物)

- `sim/out/`(轨迹、LeRobotDataset)、`outputs/`(训练输出)——由管线重建。
- 项目记忆(Claude)在 `~/.claude/projects/.../memory/`,不在仓库,建议另存。
