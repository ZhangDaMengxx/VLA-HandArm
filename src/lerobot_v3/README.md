# LeRobot v3 主运行时

本目录是 Python 3.12 + LeRobot 0.6.1 主运行时的推荐入口。它负责 Web、视觉、实时/离线
IK、串口/CAN 控制、EGO/RobotDataset 和回放。只有 ROS Humble `rclpy` 子进程使用独立
Python 3.10 `ros-humble` 薄环境；Web 会自动在后台调用，不传输 RGB-D 或 IK 大数据。

## 地址

- 命名 Conda 环境：`/home/zhang123/miniconda3/envs/lerobot-v3`
- ROS Humble 环境：`/home/zhang123/miniconda3/envs/ros-humble`
- 可复现依赖：`environment/lerobot-v3-dataset.txt`
- 本目录入口：`src/lerobot_v3/`
- 共享实现：`src/*.py`，旧命令暂时保留兼容
- 输出数据：`datasets/captures/capture_<id>/`

仓库内 `.envs/lerobot-v3` 是克隆前的旧前缀环境，只作为定位兜底，不是当前推荐环境。
不要删除它，除非命名环境和全链数据已经再次验证且明确决定回收空间。

## 入口

| 入口 | 功能 |
|------|------|
| `build_canonical.py` | RGB 视频生成 EGO LeRobotDataset |
| `build_canonical_from_rgbd.py` | 对齐 RGB-D 生成米制 EGO |
| `build_canonical_from_processed.py` | 外部手姿结果生成 EGO |
| `derive_embodiment.py` | EGO 重定向为 RobotDataset |
| `measure_acceptance.py` | 数据与重定向质量验收 |
| `verify_dataset.py` | 官方回读、strict-v3 和 Capture 完整性校验 |
| `replay_rerun.py` | 离线轨迹和视频同步回放 |
| `compare_dataset_numeric.py` | 两份数据集数值列比较 |
| `app_web.py` | 完整 Web 工作台；自动分流 ROS Humble 子进程 |

## 使用

```bash
conda activate lerobot-v3

python src/lerobot_v3/app_web.py
python src/lerobot_v3/build_canonical.py --video <video>
python src/lerobot_v3/build_canonical_from_rgbd.py --input-root <rgbd_root> --camera <camera>
python src/lerobot_v3/derive_embodiment.py --capture-root <capture> --robot nero_inspire_rgbd --emit-traj
python src/lerobot_v3/measure_acceptance.py --capture-root <capture> --robot nero_inspire_rgbd
python src/lerobot_v3/verify_dataset.py --capture-root <capture> --canonical --strict-v3
python src/lerobot_v3/replay_rerun.py --capture-root <capture> --robot nero_inspire_rgbd --serve
```

入口会拒绝 Python 3.10 或非 LeRobot 0.6.1 环境，避免用旧运行时生成不满足交付契约的数据。
环境定位可独立检查：

```bash
python3 src/lerobot_v3/env.py --prefix
python3 src/lerobot_v3/env.py --python
python3 src/ros_humble_env.py --check
```

页面和关节协议没有因环境迁移而改变。`src/ros_humble_env.py --run ...` 可用于显式启动
ROS 硬件桥；桥的 `--mock`、`--no-mock`、`--enable-control` 仍必须由操作者选择，避免 Web
启动时在未知状态下自动接管 CAN/串口。
