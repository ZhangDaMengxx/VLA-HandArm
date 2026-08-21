# 通用灵巧手可行域自动化

## 1. 目标与边界

`src/hand_feasibility.py` 用同一套状态机测试不同灵巧手。每个型号只需提供：

- 一个版本化 URDF/datasheet 资产规范；
- 一个把项目序 raw 命令和遥测接到厂家 SDK/总线的 Adapter；
- 由 URDF 碰撞邻接或厂家结构定义的少量重点 interaction。

资产中的 rad 是跨设备共享的**标称模型角**。探测器不会要求每台手人工量角，也不会
把 raw 反馈冒充独立角度真值。VLA 可在此基础上学习回差和任务精度，但不能绕过运行时
安全投影。

## 2. 与常见工程做法的对应

| 常见做法 | 本项目实现 |
|----------|------------|
| URDF/厂家表定义名义关节空间 | `HandModelSpec` 从 URDF 读取标称 rad，并保存资产 SHA-256 |
| 硬件接口隔离厂家协议 | `HandProbeAdapter`；当前提供 Mock 和 RH56DFX raw Adapter |
| commissioning 分阶段放权 | preflight、single、interactions 分开授权，默认仅 Mock |
| 从已知安全位保守扩张 | 每个候选点先回张开，先建立 fixture，再移动 probe joint |
| 多信号 fail-safe | 位置误差、稳态 `FORCE_ACT`、`ERROR`、电流和 `TEMP` 组合判断 |
| 边界自适应细分 | 粗扫描第一次失败后，二分到 `boundary_resolution_u` |
| 结果带条件和来源 | Profile 固化型号、URDF hash、固件可用信息、速度、力和全部探测点 |
| 中断可审计/可恢复 | 每点原子写 JSON；`--resume` 保留已完成项并重跑未完成项 |
| 未验证不等于通过 | 超时、缺遥测和 fixture 失败写为 `inconclusive` 或直接安全中止 |
| 异常先消除位置误差 | 过温、错误、异常或 `Ctrl+C` 时把当前 `ANGLE_ACT` 写回后断开 |

这套工具不是功能安全认证，也不能代替急停、机械防护或现场观察。自动化的价值是让
测试过程、判据和证据可重复，而不是把有接触风险的 commissioning 变成无人值守任务。
RH56DFX 没有独立软件 STOP 寄存器；“当前角写回”只是 best-effort 软冻结。若遥测已经
丢失，冻结会失败，必须依赖现场断电/急停，不能把关闭串口误认为停止运动。

## 3. 规范文件

当前型号规范：

```text
configs/hands/inspire_rh56dfx_right.json
```

关键字段：

- `asset.urdf`：标称模型和 rad 权威来源；
- `joints[].nominal_source`：当前关节采用 `vendor_urdf`，或配合
  `nominal_range_rad` 显式采用 datasheet/项目修订值；
- `joints[].raw_open/raw_closed`：Adapter 的控制端点；
- `interactions[]`：需要实测的联合可行域切片；
- `probe_policy`：速度、力、温度、缺样、跟踪误差和边界分辨率；
- `mock_constraints`：只用于验证状态机，不进入真机结论。

新手型不得复制 RH56DFX 的阈值。缺少电流或力传感时，相应阈值设为 `null`，但仍必须
提供位置反馈、错误状态和温度；能力不足的区域在报告中保留未验证状态。

## 4. 执行命令

### 4.1 完整 Mock

```bash
python3 src/hand_feasibility.py --phase all \
  --output /tmp/inspire_mock_profile.json
```

Mock 会模拟 thumb-index 自碰撞，用于验证扫描、细分、报告和恢复逻辑。

### 4.2 真机只读预检

```bash
python3 src/hand_feasibility.py \
  --adapter inspire --hardware --phase preflight \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --device-id hand-lab-01 --firmware v1.09 \
  --output reports/hand_feasibility/inspire_preflight.json
```

只读连接使用 `initialize_runtime=False`，不会在 `connect()` 时写速度、力或角度。

### 4.3 单关节运动

```bash
python3 src/hand_feasibility.py \
  --adapter inspire --hardware --phase single \
  --allow-motion CONFIRM_HAND_MOTION \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --output reports/hand_feasibility/inspire_single_index.json
```

不传 `--joint` 会按型号规范测试全部驱动关节，这是生成可供完整运行时 Projector 使用
的推荐方式。commissioning 调试时可重复传 `--joint` 只测指定关节，但不完整 Profile
不能用于全手动作投影。

### 4.4 联合可行域

```bash
python3 src/hand_feasibility.py \
  --adapter inspire --hardware --phase interactions \
  --interaction thumb_index_diagonal \
  --resume \
  --allow-motion CONFIRM_HAND_MOTION \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --output reports/hand_feasibility/inspire_single_index.json
```

联合阶段会主动接近自碰撞边界。真机禁止 `--phase all`；联合阶段必须 `--resume`
同一 Profile，并确认该 interaction 涉及的所有关节已有完整单关节证据。设备必须空载、
现场有人且可立即断电。

### 4.5 恢复中断报告

```bash
python3 src/hand_feasibility.py <原参数> \
  --resume --output reports/hand_feasibility/inspire_single_index.json
```

恢复时会验证 URDF SHA-256、设备/Adapter 身份和完整探测条件（包括速度、力、阈值、
采样与边界分辨率）。已完成项跳过；中断中的单项从张开位重新测试，不会从未知机械
姿态续跑。

## 5. Profile 语义

输出 schema 为 `hand_feasibility_profile/1`：

- `safe_max_u`：在本次速度、力和路径条件下最后一个通过的归一位置；
- `safe_max_raw`：对应厂家控制值；
- `safe_max_nominal_rad`：按资产 URDF 投影的标称角，仅供统一动作空间使用；
- `first_infeasible_u`：第一次失败边界，经二分细化；
- `points[]`：每个命令、实际 raw、跟踪误差、稳态/峰值力、电流、温度和错误位；
- `evaluation_status=inconclusive`：证据不足，运行时不得当作可行。

Profile 必须绑定型号、资产 revision、URDF hash、测试条件，以及可获得时的固件/设备
身份。换资产、固件、速度、力或探测阈值后只能作为历史证据，不能静默沿用为当前
运行时安全表。

## 6. 运行时接入原则

探测器只负责生产证据。后续运行时 Projector 应执行：

```text
VLA/遥操作目标
  -> 归一动作 u 或资产标称 rad
  -> 单关节 safe_max_u
  -> interaction 条件边界 + 安全余量
  -> Adapter raw/SDK 命令
```

Profile 尚未完整、资产 hash 不符或目标落在 `inconclusive` 区域时，Projector 应拒绝，
不能依靠 VLA “试一下”。路径可行性与终点可行性也要分别记录；同一个终点在不同
到达顺序、速度和力下可能有不同结果。

完成 Profile 后可直接离线验证投影：

```bash
python3 src/hand_feasibility.py \
  --project-profile /tmp/inspire_mock_profile.json \
  --allow-mock-profile \
  --target-u '{"right_thumb_1_joint":1,"right_thumb_2_joint":1,"right_index_1_joint":1}'
```

真实运行时不得添加 `--allow-mock-profile`。投影器要求所有单关节和规范声明的
interaction 都已完整通过；它对联合边界区间采用相邻测点中更保守的一端，不在未知
区间做乐观插值，并默认额外保留 `0.02u` 安全余量。
