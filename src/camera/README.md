# Camera / 标定工具

这里存放相机设备适配和坐标标定代码，不直接控制机械臂。

## 臂上相机手眼标定

`calibrate_handeye.py` 当前使用 OpenCV 相机适配器，求解固定棋盘格、相机安装在
机械臂末端的 eye-in-hand 问题。它只读取 NERO 关节角并用当前 URDF FK 计算
`T_base_ee`，绝不会发送运动指令。

```bash
python3 src/camera/calibrate_handeye.py \
  --intrinsics /path/to/orbbec_intrinsics.json \
  --camera-model orbbec_336 \
  --camera-index 0 \
  --board-cols 9 --board-rows 6 --square-size-m 0.025 \
  --arm-urdf assets/arm/urdf/nero_with_hand_flange_description.urdf \
  --ee-frame link8 \
  --no-mock \
  --output datasets/camera_calibration/orbbec336_handeye.json
```

棋盘格参数是“内角点”数量，不是黑白格子数量；`square-size-m` 是实际小格边长。
程序窗口持续显示检测状态，终端输入：

```text
next    采集当前姿态
undo    撤销最后一组样本
finish  样本足够后求解并保存
quit    退出且不保存
```

建议 15--25 个姿态，必须同时改变位置和朝向，不能只平移。默认只读机械臂；真实硬件
需要显式 `--no-mock`，并且仍由操作者使用现有安全控制方式移动机械臂。

`T_gripper_camera` 中的 `gripper` 就是 `--ee-frame` 指定的 URDF frame。默认基础臂模型
使用 `link7`；若相机安装参照实体末端法兰，使用示例中的带法兰 URDF 和 `link8`，输出
即为 `T_link8_camera`。不要把 `link7` 的结果改名冒充 `T_link8_camera`；二者之间还有
URDF 中已知的固定变换。

每次成功 `next` 都会保留原始棋盘格图像；最终 JSON 同时保存全部输入姿态、质量统计、
完整 `T_gripper_camera` 4x4 矩阵和本仓库约定的 `quaternion_xyzw`。所谓“最小误差”在
有限样本中没有可预知的全局终点，工具因此采用可配置的暂定阈值：默认至少 12 组、固定
标定板平移一致性 RMSE 不超过 3 mm、旋转一致性 RMSE 不超过 0.5 度。达到阈值后会提示
输入 `finish`；不会因为某一组偶然低误差而自动结束。

这两个 RMSE 衡量的是“固定棋盘格变换到 `robot_base` 后是否保持不动”的内部一致性，
并不是有外部 Ground Truth 的绝对外参误差。报告会逐样本保存残差并提示综合残差最大的
样本；它能发现坏样本和退化运动，但物理验收还需要保留姿态复投、独立测试点或外部真值。

## Orbbec Gemini 336L

### 当前真机基线

2026-08-27 已在 WSL2 + usbipd 下确认具体设备为 Gemini 336L：USB ID `2bc5:0807`、
序列号 `CPC876300084`、固件 `1.4.60`、USB 3.2，仓内 SDK 为 `2.9.3`。SDK 枚举到
Color、Depth、Accel、Gyro、Left IR 和 Right IR。

从仓库根目录执行：

```bash
cd third_party/OrbbecSDK
source ./setup.sh
./bin/ob_enumerate
```

`ob_enumerate` 是交互程序：输入设备编号 `0` 后才能继续查看传感器和 profile；不要用
超时退出本身判断设备失败。

SDK XML 的 Gemini 336L 默认配置是 30 FPS，因此 `enableStream(sensorType)`、默认 profile
或不带配置运行示例都不会自动提升到 60 FPS。官方 SDK 的正确方式是先枚举 profile，再
显式启用包含分辨率、格式和 `fps=60` 的 profile。仓库固定配置为：

```text
configs/camera/orbbec_gemini336l_60fps.json
RGB:   1280x800 @ 60 MJPG
Depth:  848x480 @ 60 Y16
```

可先用官方时间戳工具按该配置验收：

```bash
cd ~/ros2_ws/lerobotTest/third_party/OrbbecSDK
source ./setup.sh
./bin/ob_timestamp_tracker \
  -c ~/ros2_ws/lerobotTest/configs/camera/orbbec_gemini336l_60fps.json
```

### 2026-08-27 准入结果

官方无 GUI 录制器成功同时打开全部传感器并正常关闭、封装临时 bag：

```bash
./bin/ob_device_record_nogui
```

稳态观察窗口中 Color/Depth/左右 IR 约 `32.5--33 FPS`，Accel/Gyro 约
`214--218 FPS`。期间出现过一次视频流约 `9.9 FPS` 的短时窗口；当前结果证明设备原生
多流可采集，不等于 usbipd 长时间无掉帧验收。

硬件时间戳工具按默认 profile 自动选择：

- Color：`1280x720@30 MJPG`，1044 帧/约 34.86 秒，29.920 Hz；
- Depth：`848x480@30 Y16`，1055 帧/约 35.20 秒，29.946 Hz；
- 两路 Global 时间戳均单调；最近邻 RGB-Depth Global 时间残差平均绝对值约
  `1.123 ms`，最大值约 `32.025 ms`。

复测命令：

```bash
mkdir -p /tmp/orbbec_timestamp_check
cd /tmp/orbbec_timestamp_check
~/ros2_ws/lerobotTest/third_party/OrbbecSDK/bin/ob_timestamp_tracker -t 1
```

工具生成的 CSV 同时包含 `RecvTS(us)`、`SysTS(us)`、`GlobalTS(us)`、`DevTS(us)`；正式
Adapter 必须保存设备/Global 微秒时间戳，不能用 FPS 反推。当前样本存在约 2.98 秒共同
间隔，最大同步残差也超过 `<10 ms` 目标，因此长稳态和丢帧恢复仍未通过。

### 60 FPS 硬指标

Orbbec [Gemini 336L 官方产品页](https://www.orbbec.com/products/stereo-vision-camera/gemini-336l/)
声明 RGB 最高 `1280x800@60 FPS`、Depth 最高分辨率 `1280x800@30 FPS`。本机 SDK
profile 枚举进一步确认：RGB 支持
`1280x800/1280x720@60 MJPG`，Depth 支持 `848x480@60 Y16`，以及更低分辨率的 60 FPS。
SDK 的正式选择入口是 `StreamProfileList::getVideoStreamProfile(width, height, format, fps)`；
最后一个参数必须显式传 `60`，再把返回的 profile 交给 `Config::enableStream()`，不能使用
默认 profile 推断 60 FPS。对应 API 位于官方 SDK 的 `StreamProfile.hpp` 和仓内官方示例。

项目不采用默认 30 FPS。正式固定 RGB-D Capture 必须同时满足：

- RGB 显式配置 `1280x800@60 MJPG`；
- raw Depth 显式配置 `848x480@60 Y16`；
- 两路各自设备时间戳实测平均 `>=59.4 Hz`，即至少达到标称帧率的 99%；
- RGB-D 最大配对时间残差 `<10 ms`，原始流完整率 `>=99%`；
- 任一路缺少硬件时间戳、仅声明 60、实际退回 30 或靠补帧，验收均失败。

用户先在 Windows Orbbec Viewer 中验证 RGB `1280x800@60 MJPG` 与 Depth
`848x480@60 Y16` 可同时达到 60 FPS。随后项目用完全相同的双流 Profile 在 WSL + usbipd
复测：OrbbecSDK 2.9.3 的 V4L2 后端分别得到 `59.895/59.894 Hz`，LibUVC 后端分别得到
`59.816/59.894 Hz`；最大设备时间戳帧间隔约 `16.8--33.4 ms`，无时间戳倒退。两种后端
短时结果都达到 `>=59.4 Hz` 硬门槛，故 WSL + usbipd 可以提供该双流 60 FPS。

早期探针曾遇到 Depth 60 停在 STARTING、0 帧；重新通过 usbipd 附加设备后，相同 Depth
单流探针恢复为 `59.894 Hz`。该现象应视为 USB 附加或设备运行时状态异常，不能据此宣称
usbipd 或 LibUVC 存在 60 FPS 上限。生产链优先显式选择 V4L2，并在每次附加后运行仓库的
双流验收；失败时先停止所有相机 owner、重新附加设备并复测，禁止自动降级到 30 FPS。

从仓库根目录执行无 GUI 验收：

```bash
bash src/camera/check_orbbec_60fps.sh v4l2 12
```

脚本会编译轻量 C++ 探针，显式绑定设备和两路 60 FPS Profile，使用异步回调避免渲染阻塞，
并用两路设备时间戳分别计算实际 Hz。输出必须包含 `result=PASS threshold_hz=59.400`。
脚本还会确认 `/sys/class/video4linux/video*` 与 `/dev/video*` 数量一致；本次观察到 usbipd
重新附加后一度只创建 6/8 个 `/dev/video*` 节点，此时 V4L2 会缺少 Depth Profile，必须等
udev 补齐或重新附加后再验收。排查后端差异时也可显式传 `libuvc`；这不是生产自动降级机制。

SDK 确认设备存在 Hardware D2C profile，但当前 WSL + usbipd 下 30 Hz Hardware D2C
管线启动后只观察到 Color，没有得到配对的对齐 Depth。故当前判定为：

- 原生 Color/Depth/IR/IMU 多流：功能性通过；
- 原生 RGB/Depth 双路 60 FPS：短时通过；
- 设备/Global 微秒时间戳：可读取且单调；
- Hardware D2C 配对输出：未通过，需在 Source→Ego 对齐前复现和解决；
- 长时间 usbipd 稳定性、内参精度、RGB-D 像素对齐和手眼外参：尚未验收。

不要并行启动 `ob_multi_streams`、`ob_hw_d2c_align` 或项目 Adapter；相机是单 owner，
并行打开会得到 `uvc_open ... Return Code: -6`，这表示设备占用，不表示相机损坏。

### Adapter 要求

具体型号已经冻结为 Gemini 336L。基础原生 Adapter 已实现于
`src/camera/orbbec_gemini336l.py`，它默认使用 V4L2、独占一台相机并严格检查
`2bc5:0807`、型号名称和两路生产 Profile。启动时读取内参、畸变及 Depth→Color 外参，
禁用 D2C，并在不少于 2 秒的窗口内用设备时间戳验证两路均为 `>=59.4 Hz`；任一路缺帧、
时间戳倒退、格式改变或实测回退都会关闭管线并报错。

第一次在 `lerobot-v3` 环境使用时，从仓库根目录构建并安装仓内官方 Python wrapper：

```bash
bash src/camera/setup_pyorbbecsdk.sh \
  ~/miniconda3/envs/lerobot-v3/bin/python
```

脚本只安装 `pybind11` 构建依赖，并通过 `.pth` 将仓内编译好的官方扩展接入指定 Python
环境；不会安装上游用于 GUI 示例的 `pygame`/`pynput`，也不会用上游 `av==12.3.0` 覆盖
本项目已验收的 PyAV。SDK 源码、构建目录和运行日志仍不进入 Git。安装后执行 Adapter 真机冒烟：

```bash
PYTHONPATH=src ~/miniconda3/envs/lerobot-v3/bin/python \
  -m camera.orbbec_gemini336l --backend v4l2 --validate-seconds 12 --frames 3
```

输出包括设备/固件、实际 cadence、内参/畸变/外参和三个配对帧的设备时间戳。该命令不会
写 Capture，也不会启用 D2C 或控制机器人。相机必须是单 owner；先停止 Viewer 和其他探针。

2026-08-27 本机按上述命令通过：RGB/Depth 各 720 帧，设备时间戳为
`59.8945/59.8945 Hz`，最大帧间隔均 `16.697 ms` 且无倒退；紧接着读取的三个配对帧
Global 时间残差为 `0.153--0.158 ms`。这证明 Adapter 本身可保持已验收的双流速率，
但 12 秒样本仍不能替代后续长稳态验收。

`Gemini336LAdapter.read()` 当前返回：

- 解码后的 BGR 与原始 MJPG payload；
- 原生 `848x480 uint16` raw depth 及毫米单位 scale；
- Color/Depth frame index、设备/Global/系统微秒时间戳和配对残差；
- 启动时冻结的 Color/Depth 内参、畸变和 Depth→Color 外参。

### Capture Source 录制

原生 Source writer 已接入 `src/camera/capture_orbbec.py`。录制前先停止 Viewer、Web 相机和
其他 SDK 示例，确保 Gemini 336L 只有一个 owner，然后从仓库根目录运行：

```bash
PYTHONPATH=src ~/miniconda3/envs/lerobot-v3/bin/python \
  -m camera.capture_orbbec \
  --duration 60 \
  --validate-seconds 12 \
  --backend v4l2
```

默认在 `datasets/captures/` 新建 Capture。也可用 `--capture-root <capture>` 写入指定的空
Capture；writer 拒绝覆盖已有原生 RGB-D Source。采集期间原生 MJPG 直接保存为 `.jpg`，
未对齐 `848x480 uint16` 深度以 little-endian `.y16` 保存，不做 MJPG 解码、PNG 压缩或
D2C。有限异步队列一旦溢出、写盘失败、时间戳倒退、任一路实测低于 `59.4 Hz`、depth
scale 异常或最大同步残差达到 `10 ms`，本批 Source 会 fail-closed。

成功后会写入：

- `source/recordings/native_rgbd_frames.jsonl`：逐对原始 frame journal；
- `source/rgb_original/episode_000000/*.jpg` 与 `source/depth/raw/episode_000000/*.y16`；
- `source/calibration/intrinsics_extrinsics.json` 与 `acquisition.json`；
- `stream_index.parquet` 兼容宽表，以及 `streams.parquet`、`samples.parquet` 和
  `synchronization.json` 原生双流时间轴；
- `checksums_original.json`。

Source 成功只把 `bundle.json.stages.source` 标成 `ready`，整个 Capture 仍为 `building`；
这是有意的，因为 Ego 尚未构建。`depth_aligned_path` 保持空值，不会把 raw depth 冒充已对齐
深度。后续 Source→Ego 构建必须继续使用终端输出的同一个 `--capture-root`。

2026-08-27 本机使用 V4L2 完成 180 对真机写盘：RGB/Depth 均为 `59.8945 Hz`，最大配对
同步残差 `0.259 ms`。这证明短时原始写盘没有把双流降到 30 FPS；它不替代长时完整率、
USB 断连恢复、Hardware D2C 或物理手眼标定验收。

尚未实现的是对齐到 RGB 的 depth、对应逐帧 valid/对齐质量，以及 SDK 原生 bag 容器。当前
`recordings/` 中的 JSONL 是原始帧 journal，不应表述为 SDK bag。

手眼求解器只使用 RGB 棋盘格角点，深度流不能替代角点检测。Adapter 与 Source 全链通过
后，再用本页前述工具标定 `T_flange_wrist_camera`；在此之前不得让深度位置直接驱动机械臂。
