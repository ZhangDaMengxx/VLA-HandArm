# 机械臂 + 灵巧手 手势 / VLA 数据管线

把一段人手视频转成 NERO(7-DoF)+ inspire 手 的关节轨迹,打包成 LeRobotDataset,验证这套数据能否用来训 VLA。CPU 的 WSL 上出数据、浏览器里看效果;训练放到带 GPU 的机器(见 `训练端部署.md`)。总体方案见 `PROJECT_PLAN.md`。

## 跑起来(Python 3.10)

```bash
pip install -r requirements.txt
python sim/build_nero_inspire.py     # 生成 NERO+inspire 装配 URDF(一次即可)
```

两层数据管线:视频 → 规范层(本体无关)→ 本体轨迹 → 可视化。

```bash
python sim/build_canonical.py                 # 视频 → 规范层 canonical_ds
python sim/derive_embodiment.py --emit-traj   # 规范层 → 本体数据集 + robot_traj_*.pkl
python sim/replay_rerun.py --serve            # Rerun 三面板回放,浏览器开打印的地址
```

- 换视频:`build_canonical.py --video 路径`(不传则取 `data/` 第一个 mp4)。
- 换机器人:`derive_embodiment.py --robot 名字`(见 `sim/robot_specs.py`),规范层不用重采。

### 一键图形界面(可选)

拖拽上传视频 → 自动跑完上面三步 → 页面内嵌 Rerun 三面板。gradio 装在独立 venv,和本环境隔离(它依赖的新版 huggingface-hub 会和 lerobot 冲突):

```bash
/home/zhang123/gradio_venv/bin/python sim/app_gradio.py
# 浏览器打开打印的 http://<WSL_IP>:7860
```

## 目录

```
sim/         全部代码(核心:build_canonical / derive_embodiment / replay_rerun / app_gradio)
assets/      NERO 和 inspire 的 URDF + 网格
configs/     dex-retargeting 重定向配置
data/        输入视频(.mp4)
sim/out/     生成物(轨迹、数据集、.rrd),不进 git
```

## 依赖

`requirements.txt`:mujoco、pinocchio、dex_retargeting、mediapipe、lerobot、rerun-sdk、scipy、opencv-python、numpy、torch。手部检测器和 NERO 运动学已内置在 `sim/`。gradio 前端另装在独立 venv(见上)。
