# 硬件说明文档 (HARDWARE)

本文说明松灵 NERO 七自由度机械臂与因时 RH56DFX-2R 右手在本项目中的硬件规格、通信接口、运行时约束和装配参数。

> 对齐基准：2026-08-27 本地项目。运行时参数以 `src/nero_arm.py` 和
> `src/inspire_hand.py` 为准；装配参数以 `src/build_nero_inspire.py` 为准。

---

## 1. 硬件清单

### 1.1 机械臂：松灵 NERO

**型号**：NERO 七自由度机械臂（AgileX Robotics / 松灵机器人）

**自由度**：7-DoF，`joint1` 至 `joint7` 均为旋转关节

**末端**：`link8` 法兰，可安装夹爪或 RH56DFX 灵巧手

#### 运行时关节限位

下表是项目控制代码实际采用的安全夹取范围，定义在
`src/nero_arm.py::NERO_ARM_LIMITS`。它比源 URDF 中的近似小数更精确。

| 关节 | 下限 (rad) | 上限 (rad) | 下限 (deg) | 上限 (deg) |
|------|-----------:|-----------:|-----------:|-----------:|
| joint1 | -2.7053 | 2.7053 | -155 | 155 |
| joint2 | -1.7453 | 1.7453 | -100 | 100 |
| joint3 | -2.7576 | 2.7576 | -158 | 158 |
| joint4 | -1.0123 | 2.1468 | -58 | 123 |
| joint5 | -2.7576 | 2.7576 | -158 | 158 |
| joint6 | -0.7330 | 0.9599 | -42 | 55 |
| joint7 | -1.5708 | 1.5708 | -90 | 90 |

源文件 `assets/arm/urdf/nero_description.urdf` 的关节 `effort` 和 `velocity`
为 `0`。装配体生成脚本会将其补成 `effort=100`、`velocity=5`，仅用于让
ROS/MuJoCo 控制链可运行，**不能把这两个值当作机械臂额定力矩或额定速度**。

机械臂手册额定最大速度为：J1-J3 180 deg/s，J4-J7 225 deg/s。2026-08-06
从当前真机读回的关节级速度上限约为：J1-J4 179.9 deg/s，J5-J7
224.6 deg/s。调试控制台默认只使用 20% 速度。

#### 通信

- SDK：`pyAgxArm`，本地源码位于 `third_party/pyAgxArm/pyAgxArm-master/`
- 总线：CAN，波特率 1 Mbit/s
- Linux：`socketcan`，默认通道 `can0`
- Windows：松灵 CAN 适配器，`agx_cando`，默认通道索引 `0`
- 底层封装：`src/nero_arm.py`
- ROS2 桥：`src/nero_arm_bridge.py`
- 单臂调试入口：`src/arm_console.py`

#### 直接调用

`NeroArm` 默认 `mock=True`，真机必须显式关闭 mock。机械臂运动具有较高风险，
首次测试应清空工作区、准备急停，并使用低速度。

```python
from src.nero_arm import NeroArm

arm = NeroArm(mock=False, channel="can0", firmware="auto")
arm.connect()
try:
    if not arm.enabled:
        arm.enable()
    arm.set_speed_percent(20)
    arm.move_j([j1, j2, j3, j4, j5, j6, j7])
finally:
    # disconnect 只释放 CAN，不自动回零或失能。
    arm.disconnect()
```

推荐使用 `firmware="auto"`：代码会先读取软件版本，再选择 `v111`、`v112`
或 `v120` 驱动，避免固件升级后仍使用写死的适配版本。

#### 实时腕姿跟随约束（2026-08-19）

合体页可将 MediaPipe 手腕相对位姿经 NERO IK 映射到 7 个机械臂关节。点击联合锚定时，
后端读取机械臂当前实际/最新关节并计算末端 FK，以此作为机器人锚点；首个有效手帧即可
发起锚定，随后固定采集 12 个有效帧并剔除偏差最大的 25% 样本，同时生成位置和姿态锚点，
不使用预设 home 位姿替代在线机械臂状态。采样抖动只作为质量提示，不阻塞锚定完成。

单目腕部位置来自图像关键点与手掌表观尺度，协议标记为 `monocular_scale`。它只能用于
锚定后的有限范围相对移动，不是深度相机提供的绝对米制位置。Mock 和真机路径统一使用
末端相对位置限幅 `±50/50/30mm`。机械臂末端姿态在锚点附近按
`X -90°/+60°、Y -115°/+50°、Z -175°/+155°` 限幅后跟随腕部相对旋转；掌心朝向会参与
姿态估计并驱动完整的腕部翻转范围。
后续可用对齐 RGB-D 替换位置估计，但必须保持
坐标系、时间戳和锚定契约显式可追溯。

腕部位置在相对位姿映射和 IK 之前使用三轴 One Euro 滤波，默认
`min_cutoff=1.2Hz`、`beta=0.5`、导数截止 `1.0Hz`，用于压制 MediaPipe 单目位置的高频抖动。
姿态使用旋转向量 One Euro 滤波，默认 `min_cutoff=1.8Hz`、`beta=0.8`，再进入三轴姿态限幅。
Mock 按准备位固定末端位置的 IK 扫描余量使用 X `-90/+60°`、Y `-115/+50°`、
Z `-175/+155°` 的非对称范围，真机沿用同一范围。翻掌时的单目深度使用 MediaPipe world landmarks
当前掌宽/掌长并补偿 3D 投影缩短，避免掌宽投影缩短被误认为远离相机。
实时姿态只保留与 RGB 回放一致的 MediaPipe/MANO frame、world 轴增量和左乘组合。
欧拉角按上一帧连续选择等价分支，避免翻掌越过 90°/`±180°` 时发生表示跳变；
真机授权门和关节/工作空间限位不变。
这些滤波不能补偿单目深度/姿态尺度误差，也不能让机械臂突破工作空间、关节限位、碰撞或奇异位形约束。

跟随准备位固定为 `[0, -0.7, 0.002, 1.298, 0.002, -0.008, -0.591] rad`，摄像头关闭位
固定为七关节全零伸直位。每次启动摄像头先移动到准备位；每次关闭或启动失败先清空
latest-target、退出 CPV，再回到伸直位。Mock 会同步更新 Three.js；真臂仅在显式授权下执行，
并等待实际关节误差小于 `0.03 rad`。两段移动均使用 `move_j` 关节空间插值，沿途必须保持
无障碍。

若本次摄像头会话启用了灵巧手跟随，关闭或启动失败时先清空灵巧手实时目标队列并等待在途
帧结束，再下发六关节全张开位，之后机械臂才回伸直位。未启用灵巧手跟随的会话不会误动手。

真臂实时跟随默认关闭，只有机械臂在线、已使能、未冻结且页面显式授权时才投递 CPV 目标。
丢手、handedness 变化、连续 3 次 IK 失败、急停/冻结、未使能、连接断开或页面停止都会
结束连续控制并要求重新锚定。2026-08-19 只完成 Mock WebSocket、IK 和 Three.js 验收，
尚未执行真实机械臂运动验证。

切换顶层功能页或关闭浏览器时，页面执行统一安全释放：灵巧手先回全张开位再断 RS485；
机械臂在线、已使能且未冻结时先回七关节全零伸直位，误差进入 `0.03 rad` 后断 CAN。
机械臂掉线、失能、冻结或 15 秒未到位时，不继续等待，仍必须关闭 CAN，避免后台占用通道。
自动回零仍是关节空间插值，工作区沿途必须无障碍。

顶层页面切换会等待上述流程结束。关闭或刷新浏览器时由 `pagehide/sendBeacon` 尽力调用
`POST /api/hardware/release`；浏览器被强制结束、主机断电或断网时无法保证请求送达。
因此当前机制不能替代物理急停，也不能作为无人值守运行时唯一的通道释放保障；严格保障
仍需服务端 heartbeat/lease watchdog。

---

### 1.2 灵巧手：因时 RH56DFX

**型号**：RH56DFX-2R（右手）

**自由度**：6 个驱动关节 + 6 个 URDF 耦合关节，共 12 个活动关节

#### 驱动关节和限位

项目接口的六维顺序固定如下，也是 `set_angles()`、`read_angles()` 和上层协议
使用的顺序：

| 序号 | 关节 | 含义 | 范围 (rad) |
|-----:|------|------|-------------|
| 0 | `right_thumb_1_joint` | 拇指侧摆 / 对掌 | 0 - 1.246165 |
| 1 | `right_thumb_2_joint` | 拇指弯曲 | 0 - 0.48 |
| 2 | `right_index_1_joint` | 食指 MCP | 0 - 1.333 |
| 3 | `right_middle_1_joint` | 中指 MCP | 0 - 1.333 |
| 4 | `right_ring_1_joint` | 无名指 MCP | 0 - 1.333 |
| 5 | `right_little_1_joint` | 小指 MCP | 0 - 1.333 |

耦合关系由 `assets/hand/urdf/inspire_hand_right.urdf` 的 `mimic` 标签描述：

- `right_thumb_3_joint` 跟随 `right_thumb_2_joint`
- `right_thumb_4_joint` 跟随 `right_thumb_3_joint`
- 四指的 `right_*_2_joint` 分别跟随对应的 `right_*_1_joint`

#### 通信

- 物理接口：RS485
- 串口参数：115200 baud，8N1
- 默认手 ID：1
- Linux 默认串口：`/dev/ttyUSB0`
- 可用环境变量 `INSPIRE_HAND_PORT` 覆盖串口
- Windows 可通过 `--hand-port COM5` 一类参数指定 COM 口
- 底层驱动：`src/inspire_hand.py`
- 单手调试入口：`src/hand_console.py`

#### 直接调用

```python
from src.inspire_hand import InspireHand, InspireHandConfig

cfg = InspireHandConfig(port="/dev/ttyUSB0", mock=False)
hand = InspireHand(cfg)
hand.connect()
try:
    angles = hand.read_angles()
    print(angles)
finally:
    hand.disconnect()
```

连接成功后驱动会重新写入运行期 `SPEED_SET` 和 `FORCE_SET`，默认均为 500，
避免沿用 flash 中未知的上电默认值。`FORCE_SET` 六个通道的有效范围均为
0-1000；“拇指可到 1500”只适用于另一个掉电保存的默认力寄存器，不适用于
运行期 `FORCE_SET`。

“实时 Live · 合体”页是显式例外：灵巧手实时动作速度默认显示为 1000，并在该页接入
灵巧手成功后立即下发 `SPEED_SET=1000`；若灵巧手已经在线，显式启动摄像头时还会再次
确认下发当前实时速度。若接入阶段下发失败，页面会终止本次接入并显示错误，不会出现
界面为 1000、硬件仍停留在 500 的状态。其他入口仍使用驱动默认 500。

#### 实时摄像头控制观测（2026-08-14）

MediaPipe Tasks → WebSocket → dex-retargeting → latest-target mailbox → RS485
真机链路中，retarget 通常 1-5ms、RS485 通常 4.6-8.0ms、目标到串口 ACK 约
7-39ms。软件链没有无界积压，压力样本最多覆盖一个待发旧目标。

同日分别在 `SPEED_SET=500/800/1000` 时观测到 418.5/336.8/110.6ms 的
`settled`。三次人手动作的目标幅度不同，这组数只证明提高速度能改善跟随趋势，
不能用于计算严格倍率。正式选定运行速度前，仍需用相同固定角度阶跃重复测试，并同时
记录力、电流、温度、机械冲击和噪声。每次重新接入仍会恢复代码默认值 500。

实时摄像头下发前的软件层会对六关节目标应用 One Euro 滤波，参数为
`min_cutoff=1.5Hz`、`beta=2.5`、导数截止 `1.0Hz`。最小命令变化门限为
`0.0005rad`，约等于或小于一个硬件 raw count。曾测试的 `0.015-0.02rad` 大死区会
积累滤波尾差并产生约 `0.02rad` 的末端台阶，已移除。该滤波不改变 RH56DFX 固件、
`SPEED_SET` 或 `FORCE_SET`，也不能提升本体内部位置环带宽。dex-retargeting 自带的
逐帧固定低通已设置为 `low_pass_alpha=1.0`（关闭），避免与 One Euro 叠加延迟。

---

### 1.3 RGB-D 相机：Orbbec Gemini 336L

2026-08-27 在 WSL2 + usbipd 下完成首次设备准入，当前设备基线如下：

| 项目 | 实测值 |
|------|--------|
| 型号 | Orbbec Gemini 336L |
| USB ID | `2bc5:0807` |
| 序列号 | `CPC876300084` |
| 固件 | `1.4.60` |
| SDK | OrbbecSDK `2.9.3` |
| 连接 | USB 3.2，经 usbipd 转发到 WSL2 |
| 传感器 | Color、Depth、Accel、Gyro、Left IR、Right IR |

官方无界面原生录制器成功同时打开全部六类传感器并正常封装临时 bag。稳定观察窗口中
Color/Depth/左右 IR 约 `32.5--33 FPS`，Accel/Gyro 约 `214--218 FPS`；曾出现一个
Color/Depth/IR 约 `9.9 FPS` 的短时窗口，因此这次只算功能性准入，不代表 WSL + usbipd
长时间稳定性已验收。这里使用的是 SDK 默认 30 FPS 配置，不是项目生产标准。

官方时间戳工具自动选择 Color `1280x720@30 MJPG` 和 Depth `848x480@30 Y16`，约 35 秒
内分别得到 1044/1055 帧，Global 时间戳均单调，平均速率为 29.920/29.946 Hz。按 Global
时间戳做最近邻配对时，平均绝对残差约 `1.123 ms`，最大值约 `32.025 ms`；最大值超过项目
`<10 ms` 的目标门限，且样本中存在约 2.98 秒的共同间隔，需在正式 Capture Source writer 中记录丢帧、
对齐 sample index 并做更长时间复测，不能只用平均值宣称同步通过。

固件/SDK 查询确认该型号提供 Hardware D2C profile；当前 WSL + usbipd 下 30 Hz 对齐管线
能够创建和启动，但探针只收到 Color、未收到对齐 Depth，所以 Hardware D2C 尚未通过验收。
正式采集前必须在单一相机 owner 下复测，确认能持续得到配对 RGB-D、保存未对齐 raw depth
和对齐 depth，并读取设备内参、畸变及 RGB-Depth 外参。当前相机数据还没有接入 Capture，
也没有参与机械臂控制。

Orbbec 官方 Gemini 336L 产品页给出的上限为：Depth `1280x800@30 FPS`、RGB
`1280x800@60 FPS`。这表示 Depth 的最高分辨率模式是 30 FPS，并不表示 Depth 最高只能
30 FPS；本机 SDK/固件实际暴露了 Depth `848x480`、`640x480`、`640x360`、`480x270`、
`424x240` 的 60 FPS Y16 profile，以及 RGB `1280x800/1280x720@60 MJPG` 等 profile。

项目固定 RGB-D 生产硬指标为 RGB `1280x800@60 MJPG`、raw Depth `848x480@60 Y16`：
两路标称都必须显式为 60 FPS，并分别用设备时间戳证明实测 `>=59.4 Hz`；禁止自动回退
30 FPS 或补帧。用户先在 Windows Orbbec Viewer 验证该双流组合均为 60 FPS；随后项目
使用 OrbbecSDK 2.9.3 在同一 WSL + usbipd 链路复现：V4L2 后端实测 RGB/Depth 为
`59.895/59.894 Hz`，LibUVC 后端复测为 `59.816/59.894 Hz`，均达到硬门槛。

早期探针曾遇到 Depth 60 停在 STARTING、0 帧；重新通过 usbipd 附加设备后，同一 Depth
单流探针恢复为 `59.894 Hz`。因此该现象应归类为 USB 附加或运行时瞬时故障，不能解释为
WSL、usbipd 或 LibUVC 的固有帧率上限。生产启动优先显式选择 V4L2，并在每次附加后运行
双流验收；V4L2 还要求 sysfs 枚举的 8 个 video 节点都已出现在 `/dev/video*`，缺节点会
导致 Depth Profile 缺失。失败时先停止所有相机 owner、重新附加 USB 并复测，禁止静默降级到 30 FPS。
固定配置见 `configs/camera/orbbec_gemini336l_60fps.json`。

相机的 SDK 命令、判定边界和下一步见 `src/camera/README.md`。

---

## 2. RH56DFX RS485 协议

该协议是因时自定义串口协议，**不是 Modbus-RTU**。

### 2.1 帧格式

多字节寄存器地址按小端排列，即低字节在前。校验和只有一个字节：从 `ID`
开始到数据末尾逐字节累加，取低 8 位。

```text
写请求：EB 90 [ID] [LEN] 12 [ADDR_L] [ADDR_H] [DATA...] [CHECKSUM]
读请求：EB 90 [ID] 04    11 [ADDR_L] [ADDR_H] [READ_LEN] [CHECKSUM]
回复：  90 EB [ID] [LEN] [CMD] [ADDR_L] [ADDR_H] [DATA...] [CHECKSUM]
```

例如读取地址 1000 (`0x03E8`) 的一个字节：

```text
EB 90 01 04 11 E8 03 01 02
```

驱动除了校验帧头和 checksum，还会核对回复中的寄存器地址，防止把上一事务的
迟到回复误当成本次结果。

### 2.2 关键寄存器

| 功能 | 地址 | 长度 | 范围 | R/W |
|------|-----:|------|------|-----|
| HAND_ID | 1000 | 1 byte | 设备 ID | R/W |
| CLEAR_ERROR | 1004 | 1 byte | 写 1 清错 | W |
| POS_SET | 1474 | 6 short | 0-2000 | W |
| ANGLE_SET | 1486 | 6 short | 0-1000 | W |
| FORCE_SET | 1498 | 6 short | 0-1000 | W |
| SPEED_SET | 1522 | 6 short | 0-1000 | W |
| POS_ACT | 1534 | 6 short | 位置反馈 | R |
| ANGLE_ACT | 1546 | 6 short | 0-1000 | R |
| FORCE_ACT | 1582 | 6 short | 力反馈 | R |
| ERROR | 1606 | 6 byte | 错误码 | R |
| STATUS | 1612 | 6 byte | 状态码 | R |
| TEMP | 1618 | 6 byte | 温度 | R |

### 2.3 通道顺序

厂商寄存器中的六通道顺序为：

```text
m=0  小指
m=1  无名指
m=2  中指
m=3  食指
m=4  拇指弯曲
m=5  拇指侧摆
```

项目顺序与厂商顺序完全相反：

```python
PROJECT_TO_VENDOR = [5, 4, 3, 2, 1, 0]
```

### 2.4 raw 与弧度转换

当前六个驱动通道均为 `invert=True`：`raw=1000` 对应 URDF 关节下限，
`raw=0` 对应 URDF 关节上限。四指和拇指弯曲表现为张开到闭合；拇指侧摆
则表现为完全打开到最大对掌位。

```python
frac = rad / span
raw = round((1.0 - frac) * 1000)

frac = 1.0 - raw / 1000.0
rad = frac * span
```

驱动默认会先按 `HAND_LIMITS` 夹取，再进行转换。`set_angles()` 的输入和
`read_angles()` 的输出始终是项目顺序和弧度单位。

---

## 3. 装配参数

### 3.1 RH56DF 适配法兰

- 材质：铝合金，建模密度 2700 kg/m3
- 质量：60.8 g
- 网格：`assets/arm/meshes/rh56df_adapter_flange.stl`
- STL 单位：毫米，URDF 中使用 `0.001 0.001 0.001` 缩放

### 3.2 固定关节变换

`link8` 到适配法兰：

```python
FLANGE_MOUNT_XYZ = "0 0 0.016489"
FLANGE_MOUNT_RPY = "0 0 1.570796"
```

适配法兰到手根 `R_base_link`：

```python
MOUNT_XYZ = "0.000042 0.0 0.002158"
MOUNT_RPY = "0 0 1.570796"
```

参数定义在 `src/build_nero_inspire.py`，生成结果为
`assets/assembled/nero_inspire_right.urdf`。安装量来自装配体
`nero_RH56DF.stl` 反解；手掌网格 ICP 残差约 0.36 mm。

### 3.3 坐标系约定

当机械臂 `q=0` 竖直时：

- 手心朝向 world -X
- 手指沿工具轴指向 +Z
- 小指位于 +Y 侧
- 拇指位于 -Y 侧

---

## 4. 供电要求

### 机械臂 NERO

- 电压：24 V DC
- 电流：至少 5 A，峰值需求可能更高

### 灵巧手 RH56DFX-2R

- 电压：24 V DC
- 电流：至少 5 A，依据 RH56BFX-DFX-2RL 参数表
- 不要套用旧 RH56DF3 手册中的 7-9 V 规格

供电数值来自厂商资料，不由项目代码校验。接线前仍应以当前硬件铭牌和对应型号
的厂商手册为最终依据。

---

## 5. 项目硬件入口

| 层级 | 文件 | 职责 |
|------|------|------|
| 灵巧手驱动 | `src/inspire_hand.py` | RS485 包帧、寄存器、角度/通道转换 |
| 机械臂驱动 | `src/nero_arm.py` | pyAgxArm、CAN、使能、运动和遥测 |
| HTTP Robot Bridge | `robot-mcp-server/robot-bridge/bridge.py` | 保持 MCP HTTP/Token 契约，通过 ROS2 Backend 调 Driver |
| ROS2 硬件 Driver | `src/nero_inspire_ros2/nero_inspire_hardware` | 独占 CAN/RS485、重连、状态诊断、ROS2 控制与 `/joint_states` |
| Web ROS2 worker | `src/ros_web_hardware.py` | 订阅 Driver 状态、调用控制 Service，不打开硬件设备 |
| 灵巧手控制台 | `src/hand_console.py` | 单独调试灵巧手 |
| 机械臂控制台 | `src/arm_console.py` | 单独调试机械臂，默认 mock 和 20% 速度 |
| 装配体生成 | `src/build_nero_inspire.py` | 生成并验证臂、法兰、手装配 URDF |

### 5.1 HTTP Robot Bridge 启动语义

```bash
cd ~/ros2_ws/robot-mcp-server/robot-bridge
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
.venv-ros/bin/python bridge.py --backend ros --host 127.0.0.1 --port 9000
```

- ROS2 Backend 下 mock/真机、CAN 和串口参数全部归 Hardware Driver，Bridge 不直接打开设备。
- MCP Server 的 `ROBOT_BRIDGE_URL`、`ROBOT_BRIDGE_TOKEN`、`X-Bridge-Token` 和 HTTP 路径不变。
- `--backend direct` 仅作迁移回退，不能和 Hardware Driver 同时占用设备。
- 根目录启动器允许 `Direct -> Web + Bridge` 同时启动两个服务进程，但这只是进程编排：
  Bridge 启动即连接硬件，Web 后续点击“接入”仍会尝试直连同一 CAN/串口。该模式没有跨进程
  owner 仲裁，不能据此认为双端同时下发是安全的。
- `/health` 的 `backend` 标识当前后端，arm/hand 状态来自 ROS Driver。

### 5.2 ROS2 hardware Driver 启动语义

正式入口 `nero_inspire_hardware` 默认臂和手都使用 mock 且只监控。真机必须显式设置
`arm_mock:=false hand_mock:=false`；只有 `enable_control:=true` 才创建控制 Topic 和
Service。该参数不会自动使能真实机械臂。

```bash
# 无硬件空跑，只监控
ros2 launch nero_inspire_hardware hardware.launch.py

# 无硬件空跑并验证控制链
ros2 launch nero_inspire_hardware hardware.launch.py enable_control:=true

# 真机并显式允许控制；首次上机先保持 enable_control:=false，只读确认后再切 true
ros2 launch nero_inspire_hardware hardware.launch.py \
  arm_mock:=false hand_mock:=false enable_control:=true \
  firmware:=auto hand_port:=/dev/inspire_hand

# 状态与诊断
ros2 topic echo /nero/driver_state
ros2 topic echo /diagnostics

# 明确使能；Driver 重连后必须重新人工执行
ros2 service call /nero/arm/set_enabled std_srvs/srv/SetBool '{data: true}'
```

Driver 分别维护 arm/hand 的 `DISCONNECTED -> CONNECTING -> READY/FAULT` 状态。
连续读失败会停止发布该设备的旧角度、关闭句柄并指数退避重连；另一个健康设备仍可继续
发布自己的部分 `JointState`。运动命令使用 volatile QoS，Driver 重启或重连不会重放旧目标。
重连只恢复通信，不自动使能电机。当前轨迹兼容 Topic 仍只执行最后一个路点，完整轨迹执行
后续迁移到 `FollowJointTrajectory` Action。

Web 真机会话通过 `ros_web_hardware.py` 访问上述 Topic/Service，不再启动直连 Console。
本地 mock 仍可使用 `hand_console.py`/`arm_console.py`。CPV 实时跟随通过正式 Driver 的
三段式 Service 执行；完整轨迹 Action、逐通道手力控和 clear-error 必须显式拒绝，禁止为
保留页面能力而回退直连。

---

## 6. 故障排除

### 6.1 灵巧手不响应

检查：

- 24 V 供电和极性是否正确
- RS485 A/B 是否接反
- 串口设备是否存在、是否被其他程序占用
- 当前用户是否有串口权限
- 波特率是否为 115200，手 ID 是否为 1 或实际配置值
- WSL 环境下 Windows 是否已通过 usbipd 转发 USB 串口

```bash
ls -l /dev/ttyUSB*
id -nG
python3 src/hand_console.py --no-mock --port /dev/ttyUSB0
```

`connect()` 会先读取 `HAND_ID`。串口能打开但无回复时会明确报错，不会自动进入 mock。
调试页、HTTP bridge 和手控制台不要同时占用同一个串口。

### 6.2 灵巧手角度方向或手指对应错误

依次核对：

- `HAND_JOINTS`：项目六维顺序
- `PROJECT_TO_VENDOR`：项目到厂商通道映射
- `RAW_MAP`：逐通道 span 和反向标记
- `HAND_LIMITS`：运行时夹取范围

`[0, 0, 0, 0, 0, 0]` 在 URDF 语义中是张开姿态，但真机下发会产生运动，
不要把它作为无人看守的通信探测命令。只验证通信时使用 `read_angles()`。

### 6.3 机械臂连接或发指令不运动

Linux 先检查 CAN：

```bash
ip -details link show can0
sudo ip link set can0 up type can bitrate 1000000
candump can0
python3 src/arm_console.py --no-mock --channel can0 --firmware auto --speed 20
```

重点检查：

- 机械臂是否上电、CAN 是否为 1 Mbit/s
- 是否能收到关节角推送帧
- 固件适配版本是否正确
- 机械臂是否已经使能
- CPV/轨迹回放前控制模式是否为 `CAN_CTRL`
- 是否处于急停状态；复位后必须重新使能

机械臂速度状态没有 SDK 读回接口。项目在自己未设置过速度时会报告未知，而不是
猜测为 100%。调试时应显式设置低速度。

### 6.4 MuJoCo 加载装配 URDF 失败

```bash
python3 src/build_nero_inspire.py
```

脚本会统一 mesh 路径、补齐仿真所需的关节 effort/velocity，并验证生成模型。
常见问题包括 mesh 路径失效、joint/link 重名或关节上下限错误。

---

## 7. 权限与设备检查

```bash
# 永久加入串口权限组，重新登录后生效
sudo usermod -a -G dialout $USER

# 查看 USB 串口和设备信息
ls -l /dev/ttyUSB*
udevadm info -a -n /dev/ttyUSB0 | grep -i serial
```

不建议用 `chmod 666 /dev/ttyUSB0` 作为长期方案；设备重新插拔后权限会恢复，且会将
串口开放给所有本机用户。

---

## 8. 安全注意事项

1. 以硬件铭牌和对应型号的厂商手册确认电压、极性和接地。
2. 机械臂真机必须显式关闭 mock，并在运动前确认使能状态和控制模式。
3. 首次运动使用低速度、小幅度单关节命令，确保急停可立即触达。
4. 不要依赖软件夹取代替机械限位和现场防护。
5. 灵巧手的力控值是接触阈值，不是恒定目标力；接触点变化会改变实际夹持效果。
6. 同一 CAN 或串口设备不要被多个控制程序同时占用。Direct 的 `Web + Bridge` 菜单不会
   改变这条约束；它只让两个服务同时在线。
7. 退出机械臂控制程序只会断开通信，不会自动回零、失能或解除现场风险。

### 8.1 资产标称参数一致性

本仓运行驱动、手势可行域换算、ROS writer、URDF 生成覆盖和正式 URDF 当前统一为：

- 拇指侧摆：span/limit `1.246165`
- 拇指弯曲：span/limit `0.48`
- 四指：span/limit `1.333`

```bash
python3 src/skills/hand_pose.py --verify
python3 -m pytest src/test/test_hand_limit_consistency.py -q
```

这两项离线校验当前通过。它们只证明资产标称映射一致，不证明具体设备在指定速度、
力和运动路径下的物理可行包络；后者必须使用完整、非 `aborted` 的真机 Profile。

### 8.2 通用可行域 commissioning

`src/hand_feasibility.py` 按型号规范自动探测灵巧手的 raw/归一量可行包络。统一动作空间的
rad 来自 datasheet/URDF 的资产标称角，不要求逐台外部量角；真机探测结果描述的是指定
设备、固件、速度、力和运动路径下的安全证据，不是独立角度真值。

RH56DFX 当前探测策略为 `speed=15`、`force=250`。只读预检连接时使用
`initialize_runtime=False`，不会写 `SPEED_SET`、`FORCE_SET` 或角度。`STATUS=2` 只记录，
不作为到位或接触判据；判定组合使用位置误差、稳态 `FORCE_ACT`、`ERROR`、`TEMP`，并在
Adapter 能提供时加入电流。连续缺样、过温或未知故障按 fail-closed 中止。

```bash
# 只读预检，不运动
python3 src/hand_feasibility.py \
  --adapter inspire --hardware --phase preflight \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --output reports/hand_feasibility/inspire_preflight.json

# 单关节阶段；现场清空、急停/断电可达后才授权
python3 src/hand_feasibility.py \
  --adapter inspire --hardware --phase single \
  --allow-motion CONFIRM_HAND_MOTION \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --output reports/hand_feasibility/inspire_profile.json

# 联合切片必须续写已有完整单关节证据
python3 src/hand_feasibility.py \
  --adapter inspire --hardware --phase interactions --resume \
  --allow-motion CONFIRM_HAND_MOTION \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --output reports/hand_feasibility/inspire_profile.json
```

真机禁止 `--phase all`。联合阶段会主动接近自碰撞边界，必须空载、现场有人并可立即断电。
异常或 `Ctrl+C` 时工具会尝试把当前 `ANGLE_ACT` 写回；这是 best-effort 软冻结，不是硬急停，
遥测丢失时也可能失败。完整流程和 Profile 接入规则见
[src/HAND_FEASIBILITY_AUTOMATION.md](src/HAND_FEASIBILITY_AUTOMATION.md)，参数漂移及 Bridge
影响见 [src/HAND_LIMIT_AUDIT_2026_08_21.md](src/HAND_LIMIT_AUDIT_2026_08_21.md)。

2026-08-24 已在设备 `inspire-rh56dfx-right-local` 上完成新的六关节空载单关节 Profile：
`speed=15`、`force=250`、60/60 点均为 `feasible`，六关节均达到 `safe_max_u=1.0/raw=0`；
全程 `ERROR=0`，峰值温度 52℃、峰值力绝对值 84，postflight 回到张开侧。报告位于本地
`reports/hand_feasibility/inspire_single_2026_08_24.json`，SHA-256 为
`f66618055e5848a735289ccd6bd9f234e3dfd26f3e60dc1a3a29eed3a55e1b2d`。`raw=0` 是通过判据的
命令端点；端点采样时拇指侧摆/弯曲反馈为 `0/0`，食/中/无名/小指反馈为 `48/59/55/60`，
均在既定 `tracking_tolerance_raw=60` 内，小指正好压线。该结果只证明指定条件下的单关节
自由行程，不证明联合姿态、自碰撞边界或独立物理角真值；interaction 尚未执行，条件化
安全投影也尚未接入 Web、Bridge 或 `hand_pose` 运行时。

---

## 9. 参考资料与模型来源

项目外部厂商资料：

- `仿人五指灵巧手--参数表(RH56BFX-DFX-2RL).docx`
- `因时机器人灵巧手--RH56用户手册V1.09cn.pdf`
- `关节与角度对应关系/关节角与0-1000 对应关系.xls`
- `基于Python灵巧手的上位机软件/inspire_hand.py`
- `灵巧手_SDK/C_Linux/hand_api.c`
- `inspire_hand/src/hand_control.cpp`

项目内模型和代码：

- 机械臂源 URDF：`assets/arm/urdf/nero_description.urdf`
- 灵巧手源 URDF：`assets/hand/urdf/inspire_hand_right.urdf`
- retargeting 六自由度 URDF：`assets/hand/urdf/inspire_hand_right_6dof.urdf`
- 臂手装配 URDF：`assets/assembled/nero_inspire_right.urdf`
- 灵巧手 URDF 来源：厂商 2025-04-18 SolidWorks 导出包
- 机械臂 SDK：`pyAgxArm-master/`

---

**最后核对**：2026-08-21

**维护者**：项目团队
