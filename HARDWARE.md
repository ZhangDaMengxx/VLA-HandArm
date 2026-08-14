# 硬件说明文档 (HARDWARE)

本文说明松灵 NERO 七自由度机械臂与因时 RH56DFX-2R 右手在本项目中的硬件规格、通信接口、运行时约束和装配参数。

> 对齐基准：2026-08-14 本地项目。运行时参数以 `sim/nero_arm.py` 和
> `sim/inspire_hand.py` 为准；装配参数以 `sim/build_nero_inspire.py` 为准。

---

## 1. 硬件清单

### 1.1 机械臂：松灵 NERO

**型号**：NERO 七自由度机械臂（AgileX Robotics / 松灵机器人）

**自由度**：7-DoF，`joint1` 至 `joint7` 均为旋转关节

**末端**：`link8` 法兰，可安装夹爪或 RH56DFX 灵巧手

#### 运行时关节限位

下表是项目控制代码实际采用的安全夹取范围，定义在
`sim/nero_arm.py::NERO_ARM_LIMITS`。它比源 URDF 中的近似小数更精确。

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

- SDK：`pyAgxArm`，本地源码位于 `pyAgxArm-master/pyAgxArm-master/`
- 总线：CAN，波特率 1 Mbit/s
- Linux：`socketcan`，默认通道 `can0`
- Windows：松灵 CAN 适配器，`agx_cando`，默认通道索引 `0`
- 底层封装：`sim/nero_arm.py`
- ROS2 桥：`sim/nero_arm_bridge.py`
- 单臂调试入口：`sim/arm_console.py`

#### 直接调用

`NeroArm` 默认 `mock=True`，真机必须显式关闭 mock。机械臂运动具有较高风险，
首次测试应清空工作区、准备急停，并使用低速度。

```python
from sim.nero_arm import NeroArm

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
- 底层驱动：`sim/inspire_hand.py`
- 单手调试入口：`sim/hand_console.py`

#### 直接调用

```python
from sim.inspire_hand import InspireHand, InspireHandConfig

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

参数定义在 `sim/build_nero_inspire.py`，生成结果为
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
| 灵巧手驱动 | `sim/inspire_hand.py` | RS485 包帧、寄存器、角度/通道转换 |
| 机械臂驱动 | `sim/nero_arm.py` | pyAgxArm、CAN、使能、运动和遥测 |
| HTTP 硬件代理 | `bridge.py` | MCP 服务使用的本机 HTTP 接口 |
| ROS2 桥 | `sim/nero_arm_bridge.py` | ROS2 轨迹话题与 `/joint_states` |
| 灵巧手控制台 | `sim/hand_console.py` | 单独调试灵巧手 |
| 机械臂控制台 | `sim/arm_console.py` | 单独调试机械臂，默认 mock 和 20% 速度 |
| 装配体生成 | `sim/build_nero_inspire.py` | 生成并验证臂、法兰、手装配 URDF |

### 5.1 HTTP bridge 启动语义

```bash
# mock：臂和手均不连接真实硬件
python bridge.py --mock --host 127.0.0.1 --port 9000

# Linux 真机
python bridge.py --hand-port /dev/ttyUSB0 --host 127.0.0.1 --port 9000

# Windows 真机示例
python bridge.py --hand-port COM5 --host 127.0.0.1 --port 9000
```

- 不加 `--mock` 时，灵巧手连接失败会终止 bridge 启动，不会静默回退 mock。
- 真机模式下机械臂连接失败只会告警，bridge 仍可继续提供灵巧手功能。
- `/health` 中 `hand` 和 `arm` 分别表示两套控制器是否存在，`mock` 表示当前是否为空跑。
- HTTP bridge 连接机械臂后不会自动使能；运动端点仍需遵守对应的使能和安全检查。

### 5.2 ROS2 bridge 启动语义

`sim/nero_arm_bridge.py` 默认臂和手都使用 mock。真机必须显式传
`--no-mock`，且真机默认只监控；只有再传 `--enable-control` 才订阅控制话题。

```bash
# 无硬件空跑，默认允许控制话题
python3 sim/nero_arm_bridge.py --mock

# 真机，只监控
python3 sim/nero_arm_bridge.py --no-mock

# 真机并显式允许控制
python3 sim/nero_arm_bridge.py --no-mock --enable-control --firmware v120
```

注意：ROS2 bridge 当前固件选项不含 `auto`；需要自动探测时优先使用
`sim/arm_console.py` 或直接调用 `NeroArm(firmware="auto")`。

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
python3 sim/hand_console.py --no-mock --port /dev/ttyUSB0
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
python3 sim/arm_console.py --no-mock --channel can0 --firmware auto --speed 20
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
python3 sim/build_nero_inspire.py
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
6. 同一 CAN 或串口设备不要被多个控制程序同时占用。
7. 退出机械臂控制程序只会断开通信，不会自动回零、失能或解除现场风险。

### 8.1 当前已知参数不一致

`sim/skills/hand_pose.py` 复制了一份用于可行域换算的 `RAW_MAP` 和 `LIMIT_HI`，但它
目前没有与 `sim/inspire_hand.py` 的运行时表同步：

- 拇指弯曲：驱动 span/limit 为 `0.48`，安全表仍为 `0.69813/0.6`
- 四指：驱动 span/limit 为 `1.333`，安全表仍为 `1.39626/1.47`

```bash
python3 sim/skills/hand_pose.py --verify
```

当前会报告 10 项不一致。独立 `robot-mcp-server/robot-bridge` 复制了相同的两份文件，
也存在同一问题。在完成参数决策、同步和真机验证前，可行域检查只能视为未对齐的保护
逻辑，不能作为硬件安全保证。

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

**最后核对**：2026-08-14

**维护者**：项目团队
