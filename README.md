# 机械臂 + 灵巧手 · 手势 → 抓取 → VLA 数据管线(自研代码)

在 CPU-WSL 上,把**真人手视频** → 重定向 + 手腕估计 + 逆解 + 平滑 → **机器人 [臂+手] 轨迹** → 打包成 **LeRobotDataset**,用于验证"采集的数据可训练 VLA"。

- 完整方案与进度:`PROJECT_PLAN.md`(单一事实来源)
- 训练端(RTX)部署:`训练端部署.md`
- 主管线代码说明:`sim/README.md`

---

## 本仓库内容(我们自研的)

| 路径 | 说明 |
|---|---|
| `sim/` | **主管线**:NERO+inspire 装配、逆解、retarget 驱动、MeshCat 检视台、两层 schema、手势演示、数据集构建。详见 `sim/README.md` |
| `overlays/dex-retargeting/` | 我们塞进第三方 dex-retargeting 的文件(开合修复配置、可视化器、实验)。**用时拷回 dex-retargeting 对应路径**(见下) |
| `PROJECT_PLAN.md` | 项目方案 / 进度(SSOT) |
| `训练端部署.md` | RTX 上训 ACT/VLA 的 runbook |
| `VISUALIZER_SPEC.md` | 可视化器规格(早期) |
| `a1_check_env.py` / `a2_nero_probe.py` | 早期环境探针 |

## 第三方依赖(不入库,单独下载到本目录同级)

`.gitignore` 已排除。复现时需自行获取:

| 依赖 | 用途 |
|---|---|
| `dex-retargeting` | 手部重定向(`overlays/` 覆盖进它) |
| `dex-urdf` | 机器人 URDF / 网格 |
| `pinocchio-kinematics-lite` | NERO URDF + FK/IK |
| `pyAgxArm` | 松灵 NERO CAN 驱动 SDK(真机部署用) |
| `egozero` / `xr_teleoperate` / `unitree_sdk2` | 相关参考,暂未接入 |

## 复现步骤

1. 下载上述第三方仓库到本目录同级。
2. 把 `overlays/dex-retargeting/` 下的文件按相同相对路径拷进 `dex-retargeting-main/dex-retargeting-main/`。
3. WSL 里用装好 mujoco / pinocchio / dex_retargeting / mediapipe / lerobot 的 Python 环境,运行 `sim/` 下脚本(流程见 `sim/README.md`)。
4. 训练见 `训练端部署.md`。

## 未入库(可重建 / 属输入或产物)

- 生成物:`sim/out/`(轨迹、LeRobotDataset)、`outputs/`(训练输出)——由管线重建。
- 输入:测试视频 `dex-retargeting/.../vector_retargeting/data/hand_1.mp4`(**建议单独备份**,丢了没法重建数据集)。
- 项目记忆(Claude)在 `~/.claude/projects/.../memory/`,不在本仓库,建议另存备份。
