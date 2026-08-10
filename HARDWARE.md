# 硬件说明文档 (HARDWARE)

NERO-7机械臂 + 因时RH56DFX灵巧手的硬件规格和接口说明。

---

## 📋 硬件清单

### 1. 机械臂：NERO-7

**型号**：NERO-7 (越疆/AgileX机器人)
**自由度**：7-DoF
**关节配置**：
- Joint1~7：revolute（旋转关节）
- 末端法兰：可安装夹爪/灵巧手

**限位**（弧度）：
```python
# 臂关节限位（已在URDF中定义）
# 详见 assets/arm/urdf/nero_description.urdf
Effort: 100 N·m
Velocity: 5 rad/s
```

**通信**：
- SDK：pyAgxArm (`pyAgxArm-master/`)
- 协议：CAN（SDK封装）
- 接口：`sim/nero_arm_bridge.py`

**控制方法**：
```python
from nero_arm_bridge import NeroArm
arm = NeroArm()
arm.move_j([j1, j2, j3, j4, j5, j6, j7])  # 关节空间运动
```

---

### 2. 灵巧手：因时 RH56DFX

**型号**：RH56DFX-2R（右手）
**自由度**：6驱动 + 6耦合 = 12关节
**驱动关节**（新命名）：
1. `right_thumb_1_joint` - 拇指侧摆(yaw)
2. `right_thumb_2_joint` - 拇指弯曲(pitch)
3. `right_index_1_joint` - 食指MCP
4. `right_middle_1_joint` - 中指MCP
5. `right_ring_1_joint` - 无名指MCP
6. `right_little_1_joint` - 小指MCP

**耦合关节**（跟随）：
- thumb_3, thumb_4 (跟随thumb_2)
- index_2 (跟随index_1)
- middle_2 (跟随middle_1)
- ring_2 (跟随ring_1)
- little_2 (跟随little_1)

**限位**（新URDF值，弧度）：
```python
thumb_1 (yaw):   [0, 1.246165]  # 71.4°
thumb_2 (pitch): [0, 0.48]      # 27.5°
四指 (MCP):      [0, 1.333]     # 76.4°
```

**通信**：
- 协议：RS485
- 波特率：115200, 8N1
- 手ID：1（默认）
- 串口：`/dev/ttyUSB0`（默认）

**驱动代码**：`sim/inspire_hand.py`

---

## 🔌 通信协议

### RH56DFX RS485协议

**包格式**：
```
发送：EB 90 [ID] [LEN] [CMD] [ADDR_H] [ADDR_L] [DATA...] [CHK_H] [CHK_L]
回复：90 EB [ID] [LEN] [CMD] [ADDR_H] [ADDR_L] [DATA...] [CHK_H] [CHK_L]
```

**关键寄存器**：

| 功能 | 地址 | 长度 | 范围 | R/W |
|------|------|------|------|-----|
| HAND_ID | 1000 | 1 byte | - | R/W |
| ANGLE_SET | 1486 | 6 short | 0-1000 | W |
| ANGLE_ACT | 1546 | 6 short | 0-1000 | R |
| FORCE_SET | 1498 | 6 short | 0-1000 | W |
| SPEED_SET | 1522 | 6 short | 0-1000 | W |
| POS_SET | 1474 | 6 short | 0-2000 | W |
| CLEAR_ERROR | 1004 | 1 byte | 写1清错 | W |

**厂商通道顺序**（0-1000原始值）：
```
m=0: 小指   (little)
m=1: 无名指 (ring)
m=2: 中指   (middle)
m=3: 食指   (index)
m=4: 拇指弯曲 (thumb_pitch / thumb_2)
m=5: 拇指旋转 (thumb_yaw / thumb_1)
```

**项目通道映射**：
```python
PROJECT_TO_VENDOR = [5, 4, 3, 2, 1, 0]  # 完全逆序
```

**方向配置**：
```python
# 所有6个通道统一：
invert = True  # raw 1000 = 完全张开，raw 0 = 完全闭合
```

**raw值与弧度转换**：
```python
# 定义在 sim/inspire_hand.py 的 RAW_MAP
span = upper_limit  # 限位上限
raw = int((rad / span) * 1000)  # 弧度 → raw
rad = (raw / 1000.0) * span      # raw → 弧度
```

---

## 🔧 装配参数

### 适配法兰

**型号**：RH56DF适配法兰（定制件）
**材质**：铝合金（2700 kg/m³）
**质量**：60.8g
**STL**：`assets/arm/meshes/rh56df_adapter_flange.stl`

**装配点1：link8 → 法兰**
```python
XYZ = "0 0 0.016489"      # 米
RPY = "0 0 1.570796"      # 弧度 (0°, 0°, 90°)
```

**装配点2：法兰 → 手根(R_base_link)**
```python
XYZ = "0.000042 0.005962 0.002158"  # 米
RPY = "-1.570790 -0.000000 -1.570799"  # 弧度
```

**来源**：装配体 nero_RH56DF.stl 反解，ICP残差0.36mm

**修改**：编辑 `sim/build_nero_inspire.py` 第53-54行

---

## 📐 坐标系约定

### 右手坐标系（q=0臂竖直时）

**手部姿态**：
- 手心朝向：world -X
- 手指朝向：+Z（沿工具轴伸出）
- 小指侧：+Y
- 拇指侧：-Y

**link8坐标系**（法兰父坐标系）：
- X: 向前（机械臂工作空间内侧）
- Y: 向左
- Z: 向上（沿腕部旋转轴）

---

## ⚡ 供电要求

### 机械臂 NERO-7
- 电压：24V DC
- 电流：≥5A（峰值可能更高）

### 灵巧手 RH56DFX-2R
- 电压：24V DC
- 电流：≥5A（参数表 RH56BFX-DFX-2RL）
- **⚠️ 不要用旧RH56DF3手册的7-9V规格！**

---

## 🔍 故障排除

### 1. 灵巧手不响应

**检查清单**：
- [ ] 电源24V是否正常
- [ ] RS485接线正确（A-A, B-B）
- [ ] 串口设备 `/dev/ttyUSB0` 存在
- [ ] 波特率115200正确
- [ ] 手ID=1（或实际配置的ID）
- [ ] 没有其他程序占用串口

**测试命令**：
```bash
# 列出串口设备
ls /dev/ttyUSB*

# 测试通信
python3 sim/hand_console.py --no-mock
```

### 2. 角度不对

**可能原因**：
- 厂商通道映射错误 → 检查 `PROJECT_TO_VENDOR`
- 方向配置错误 → 检查 `invert=True`
- 限位值错误 → 检查 `HAND_LIMITS`

**验证**：
```python
from inspire_hand import InspireHand
hand = InspireHand(port='/dev/ttyUSB0', mock=False)
hand.set_angles([0, 0, 0, 0, 0, 0])  # 全部张开
hand.read_angles()  # 读取实际值
```

### 3. MuJoCo加载URDF失败

**常见原因**：
- mesh文件路径不是绝对路径
- joint/link名称冲突
- 限位值不合理（upper < lower）

**验证**：
```bash
python3 sim/build_nero_inspire.py
# 脚本自动验证MuJoCo加载
```

---

## 📚 参考资料

### 官方文档位置

**灵巧手资料**（项目外部）：
- 参数表：`仿人五指灵巧手--参数表(RH56BFX-DFX-2RL).docx`
- 用户手册：`因时机器人灵巧手--RH56用户手册V1.09cn.pdf`
- 关节角对应表：`关节与角度对应关系/关节角与0-1000 对应关系.xls`
- Python上位机：`基于Python灵巧手的上位机软件/inspire_hand.py`
- C SDK：`灵巧手_SDK/C_Linux/hand_api.c`
- ROS1驱动：`inspire_hand/src/hand_control.cpp`

**机械臂资料**：
- SDK：`pyAgxArm-master/` (已集成到项目)

**新URDF来源**：
- 厂商包：`urdf_right_2025_4_18/` (SolidWorks导出)
- 项目位置：`assets/hand/urdf/inspire_hand_right.urdf`

### 项目内已提取代码

| 功能 | 原始位置 | 项目位置 | 说明 |
|------|---------|----------|------|
| 灵巧手驱动 | Python上位机 | `sim/inspire_hand.py` | 核心驱动 |
| 机械臂桥接 | pyAgxArm SDK | `sim/nero_arm_bridge.py` | SDK封装 |
| 装配体生成 | - | `sim/build_nero_inspire.py` | 自研 |

---

## 🔧 硬件设置建议

### 串口权限

```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 或临时改权限
sudo chmod 666 /dev/ttyUSB0
```

### 查看串口信息

```bash
# 列出USB串口
ls -l /dev/ttyUSB*

# 查看串口详细信息
udevadm info -a -n /dev/ttyUSB0 | grep -i serial
```

---

## ⚠️ 安全注意事项

1. **电源**：确认24V电源极性正确
2. **通信**：RS485 A/B不要接反
3. **限位**：不要超出机械限位
4. **速度**：首次测试用低速
5. **急停**：确保有紧急停止按钮
6. **工作空间**：确认无障碍物
7. **力控**：设置合理的力控上限

---

**最后更新**：2026-08-10  
**维护者**：项目团队
