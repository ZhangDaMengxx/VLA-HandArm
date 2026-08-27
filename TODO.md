# 待办事项

本页只保留尚未完成、能够执行的事项。完成情况和背景见
[PROJECT_STATUS.md](PROJECT_STATUS.md)。

## P0 安全与一致性

- [ ] 统一 `src/inspire_hand.py` 与 `src/skills/hand_pose.py` 的手指 span/limit
  - [x] 本仓资产标称映射已统一为拇指弯曲 `0.48`、四指 `1.333`；手势安全表、
    ROS writer、URDF 生成覆盖和正式 URDF 一致性测试通过（2026-08-24）
  - [x] 2026-08-21 完成第一阶段离线审计：参数来源、Bridge 执行路径、12 个语义
    姿态和 8 个动作包影响已记录到 `src/HAND_LIMIT_AUDIT_2026_08_21.md`
  - [x] 将资产标称 rad、raw 安全包络和可选残差标定拆成独立契约；新增通用型号
    规范、Mock/RH56 Adapter、自动探测 Profile 和安全投影（2026-08-21）
  - [x] 修正旧碰撞扫描脚本顶部 speed=50 与实际 `SCAN_SPEED=15` 的说明冲突
  - [x] 2026-08-21 完成真机只读 `preflight`：5/5 遥测有效、错误位和电流均为 0、
    最高温度 38℃；未写速度/力/角度，报告位于本地 `reports/hand_feasibility/`
  - [x] 首次单关节扫描在小指回张开误差 `73 raw` 时安全中止并软冻结；确认旧逻辑会把
    低速途中短时稳定误判为到位，已改为目标容差感知和按行程动态超时，离线回归 `39 passed`
  - [x] 2026-08-24 从新 Profile 完成全部六关节空载单关节扫描；60/60 点均为
    `feasible`，六关节均达到 `safe_max_u=1.0/raw=0`，旧 aborted Profile 只保留诊断
  - [ ] 用户另行明确批准并确认现场安全后执行 `interactions` 低速验证；旧 T5/T6 仅作
    历史对照，统一 rad 取资产标称值，真机只声明条件化 raw 包络
  - [ ] 完整真机 Profile 定案后接入 Web/Bridge 条件化安全投影，再做低速回归
  - [x] 同步独立仓库 `robot-mcp-server/robot-bridge/sim/`，部署 Bridge 的
    `hand_pose.py --verify` 和 15 项单测通过（2026-08-24）
- [ ] 轮换已经进入 Git 的 `ssl/key.pem`
  - 停止把现有私钥用于共享或生产环境
  - 将私钥移出跟踪并加入忽略规则
  - 单独评估是否需要清理 Git 历史及通知所有使用者
## P0 真机验证

- [ ] 核验灵巧手连接、只读反馈、运动和力控范围
  - [x] 只读 preflight/postflight 与六关节空载单关节低速全行程通过（2026-08-24）
  - [ ] 完成 interaction、自碰撞边界和力控范围专项验证
- [ ] 核验机械臂固件自动探测、CAN、使能、低速单关节运动、急停和复位
- [ ] 核验机械臂不可用时 Bridge 的 hand-only 降级
- [ ] 核验 MCP 心跳断线、恢复、健康状态和运动命令不自动重试
- [ ] 记录每项真机验证的日期、硬件版本、固件和初始条件
- [x] 验证摄像头 retarget → latest-target → ACK → RS485 真机控制链
  - 目标到串口 ACK 约 7-39ms，无无界积压，峰值覆盖 1 个待发旧目标

## P1 Web 与视觉

- [ ] 在“实时 Live · 合体”的视频跟随区增加连续录制按钮
  - 录制 WebSocket 已确认的 7 轴机械臂目标、6 轴灵巧手目标和源帧时间，不增加串口/CAN 请求
  - 复用 `combo_pack/1` 的 `mode=stream`、保存校验和 `timeline_latest` 回放链，不另造协议
  - 开始录制要求摄像头已启动且联合锚定进入 `following`；冻结、丢手、重锚、关闭摄像头或离页自动停止
  - 区分 `target` 与后续可选的 `actual` 遥测，记录 Mock/真机、推理后端、锚点和丢帧统计
  - 首版只录动作时间轴；RGB/landmarks 原始采集单独作为 Capture 数据功能，不塞入动作包
- [x] 单帧在途和 latest-frame-wins 背压
- [x] `stop()` 后禁止异步重连
- [x] MediaPipe Tasks 本地资源和兼容包装层单测
- [x] 后端 latest-target mailbox、真实 ACK 和 WebSocket 断线清理
- [x] 六关节 One Euro 滤波、200ms 状态重置和硬件分辨率级写入门限
- [x] 合体页联合位置/姿态锚定、NERO IK、7+6 目标协议与 Mock Three.js 联动（腕部姿态经过滤波和限幅后驱动末端有限旋转）
- [x] 解耦合体视频跟随的灵巧手实时链路与机械臂 IK（2026-08-20）
  - retarget 后灵巧手立即进入独立 30Hz mailbox；机械臂使用单 worker、单 pending 的 latest-only IK，不并发调用 `LiveIKClient`
  - WebSocket 保持一请求一响应，返回 IK 入队状态和带 `source_frame_id` 的最近完成结果
  - 目标携带 owner、session/frame、锚点和授权 revision；`180ms` 输入、`200ms` 结果时效以及完成后二次安全核验防止旧动作下发
  - 重锚、冻结、丢手、异常、授权变化和断线均使旧结果失效；worker 与 IK 子进程随 WebSocket 回收
  - 自动测试覆盖 100ms 慢 IK、单 pending、覆盖、过期丢弃及求解途中 release/close
- [ ] 实测异步合体链路性能与双真机效果
  - Mock 记录 MediaPipe FPS、推理 P95、WebSocket RTT、hand mailbox wait/replaced、IK age/replaced 和两条链路有效下发 Hz
  - 输入至少 30fps、RS485 ACK 正常时，开启机械臂跟随后灵巧手有效下发率不应明显低于 hand-only 基线
  - 双真机必须低速、净空且急停可用；确认机械臂只追最新 IK 目标，不补跑历史轨迹
- [ ] 真机复测滤波后的张手末端与静止手势
  - 静止阶段不得再出现 `raw_delta` 接近 0、`filtered_delta` 突跳约 0.02rad
  - 记录 `perf-hand/filter` 与 `perf-hand/tracking`，确认无可见台阶且跟随延迟可接受
- [ ] 浏览器摄像头实测 FPS、端到端延迟和断线恢复
- [ ] 覆盖真实摄像头权限拒绝、设备断开、Chrome/Edge 和页面切换恢复
- [x] 增加服务端硬件会话 heartbeat/lease watchdog 与多标签页所有权（2026-08-25）
  - 页面正常切换继续等待复位和断开；浏览器异常消失时由租约超时保持位置并释放串口/CAN
  - 覆盖进程强杀、断网、刷新、重复标签页和释放请求重复到达
  - hand/arm 独立 owner；新标签页点击接入会无提示替换对应 owner，保持位置断开旧通道
  - 服务端提供 acquire/heartbeat/status/release；默认 8 秒租约、2 秒页面续租
  - 直接控制、联合/语音回放和实时跟随校验 owner；主动释放保持原复位流程
- [ ] 验证真手 + Mock 臂的联合锚定、丢手冻结和重新锚定
- [ ] 验证真臂 + Mock 手的显式授权、使能、急停、限幅和连续 IK 失败冻结
- [ ] 在清空工作区和低速条件下验证双真机合体跟随，记录初始关节、固件和急停条件
- [ ] 用对齐 RGB-D 深度替换 `monocular_scale` 腕部位置，并保留同一锚定/协议契约
  - 上帝视角固定 RGB-D 负责手腕/物体全局米制位置和遮挡恢复
  - 臂上 RGB-D 负责末端附近的物体相对位姿与局部视觉伺服，不与全局位姿直接平均
  - 标定并版本化 `T_scene_overhead`、`T_scene_robot_base` 和 `T_flange_wrist_camera`
  - [x] 建立 `src/camera/` 和 eye-in-hand 棋盘格交互工具；只读关节角、URDF FK、
    `next/finish` 采样、退化检测、误差报告及 `xyzw`/4x4 输出已具备（2026-08-24）
  - [x] 确认设备为 Gemini 336L、固件 `1.4.60`、OrbbecSDK `2.9.3` 和 USB 3.2；原生
    Color/Depth/IR/IMU 多流及设备/Global 微秒时间戳功能性通过（2026-08-27）
  - [x] 实现 Gemini 336L 原生 RGB-D/硬件时间戳 Adapter：固定 V4L2 Profile、设备身份、
    原始 MJPG/raw depth、标定快照和 fail-closed cadence（2026-08-27）
  - [x] 将 Adapter 写入 Capture Source：原生 MJPG/Y16、标定、硬件时间戳、双流索引与
    checksum 已落盘；180 对真机写盘为 `59.8945/59.8945 Hz`、最大同步残差 `0.259 ms`
    （2026-08-27）
  - [ ] 将同一 Source 接入 Ego 构建，并用真机数据验证 `T_flange_wrist_camera`；短录
    Source 通过不代表 D2C、长稳态或物理手眼已闭环
  - [x] WSL + usbipd 原生双流达到硬门槛：V4L2 下 RGB `1280x800@60 MJPG` 与 raw Depth
    `848x480@60 Y16` 实测 `59.895/59.894 Hz`；LibUVC 复测也通过（2026-08-27）
  - [ ] 解决或规避 Hardware D2C 只有 Color、没有配对 Depth 的问题；正式采集链优先
    V4L2，附加异常时必须复测双路 60 FPS，不允许降到 30 FPS
  - [ ] 用设备时间戳复测 RGB/Depth 各自 `>=59.4 Hz`、原始流完整率 `>=99%`、RGB-D
    最大同步残差 `<10 ms` 和长时间掉帧；当前 30 FPS 冒烟样本最大残差 `32.025 ms`
  - 深度无效、时间戳过期或两路位姿冲突时冻结机械臂目标并进入重定位
- [ ] 用相同固定角度阶跃各重复 3 次，对照 `SPEED_SET=500/800/1000` 的 settled、力、电流和温度
- [ ] 验证局域网可信 HTTPS；不得继续使用已提交的旧私钥

## P1 ROS2 与数据

- [x] 为 ROS2 硬件 Driver 增加 arm/hand 独立状态、诊断、读 watchdog 与退避重连（Mock，2026-08-26）
- [ ] 在 WSL + usbipd 下执行串口/CAN 拔插测试，确认 `FAULT -> READY` 且不自动使能、不重放旧命令
- [x] 将硬件 Driver 与底层驱动正式迁入 `nero_inspire_ros2` package（Mock，2026-08-26）
- [x] 将 Web 真机的直接 CAN/串口 Console 路径改为 ROS 客户端（Mock，2026-08-26）
- [x] 抽象 Web `RobotBackend`，同时支持 ROS2、Direct 与 Mock 且保持业务协议不变（2026-08-27）
- [x] 增加根目录交互启动器，统一选择 ROS2/Direct/Mock 与 Web/Bridge/MCP 启动范围（2026-08-27）
- [x] Direct 启动菜单增加 Web + Bridge 同时启动选项，不改变现有硬件连接逻辑（2026-08-27）
- [ ] 将 Web hardware lease 与 MCP/Bridge 控制权统一为跨客户端单 owner
- [x] 保持 MCP Server/X-Token/HTTP 契约不变，为独立 Robot Bridge 增加 ROS2 Backend（Mock，2026-08-26）
- [x] 审查并清理独立 ROS2 仓库运行脚本中的旧灵巧手关节名（保留旧轨迹导入兼容映射）
- [ ] 验证 `ros2_control`、JointTrajectoryController 和 RViz2
- [ ] 完成灵巧手路径碰撞检查，并接入 retargeting 约束
- [ ] 按确认后的限位重新评估或录制手势包
- [ ] 将保留轨迹 NPZ 重导出为当前 6 驱动关节命名，并重新跑安全闸测试

## P2 后续能力

- [ ] 定版无损 VLA Canonical RobotDataset 和版本化模型 Adapter
  - 母带不得只保存当前模型使用的单维抓握手型或单一 `action` 向量；稳定保存视觉、语言、
    完整机器人状态、动作意图、实际下发命令和执行反馈，模型只通过版本化 Adapter 选取子集
  - 机械臂分别保存原始末端目标、经过 IK/限位/安全投影后实际下发的 7 轴 target，以及
    7 轴 actual；末端绝对位姿和增量动作必须声明参考坐标系、旋转表示、单位及运动学 revision
  - 灵巧手从首次采集起保存 6 轴 target、厂商 raw command、6 轴 actual、speed、force limit；
    真实力/触觉缺失时保持 invalid，不以 force 指令冒充反馈，`grasp_type` 只作语义标注
  - 为同一母带提供独立导出 revision，例如 `openvla_ee_gripper/1`、
    `dexterous_ee_hand6/1` 和 `joint13/1`；升级 action head 可以重新微调，但不得因旧导出降维
    丢失六关节或执行事实而重新采集
  - 输入母带按可扩展超集保存多视角图像、语言、关节状态、末端位姿及逐字段 valid/source；
    当前模型不使用的字段由 Adapter 屏蔽，不从母带删除
  - 在上述契约冻结后完善数据集验证、模型归一化统计、训练、离线评估和真机回灌流程

### Web VLA、远程推理与 ROS2 执行（2026-08-27 讨论基线）

> 本节记录拟实施架构，不代表功能已经接入。现阶段已决定跳过 ACT 基线，第一款接入模型
> 为 SmolVLA；同时冻结职责边界、主通信方向和 fail-closed 原则。具体 checkpoint、chunk
> 参数和真机阈值必须经过 Replay/Shadow 数据后再定版。参考实现包括 OpenPI Remote Inference、LeRobot
> Async Inference、NVIDIA GR00T Real-World Deployment、MoveIt Servo 和 ROS2
> JointTrajectoryController。

- [ ] 建立 Web VLA Orchestrator，Web 只负责配置、预览、状态和人工控制，不直接持有
  Orbbec USB、不直接连接模型 Token，也不直接向 Hardware Driver 发布动作
  - 后台统一编排 Camera Service、Observation Assembler、VLA Provider、Action Chunk
    Broker、Safety Projector、ROS2 VLA Executor 和 Capture recorder
  - 浏览器继续使用 FastAPI REST + WebSocket；远程模型主协议使用 gRPC；机器人主机内部
    执行使用 ROS2 Action/Service，三层协议不得混成一个隐式通道
  - 每台机器人同时最多一个 `Execute` session；`Shadow` 可不申请运动 owner，但仍需限制
    GPU/相机资源并记录 session identity

- [ ] 将 Gemini 336L 收敛为单 owner Camera Service，并把完整采集、模型取帧和 Web 预览解耦
  - Orbbec SDK callback 固定接收 RGB `1280x800@60 MJPG`、raw Depth `848x480@60 Y16`
    和硬件时间戳；Capture writer 继续保留 60 FPS 硬门槛，不因推理或页面卡顿降到 30 FPS
  - 原始 MJPG/Y16 通过独立有界写盘队列进入 Capture Source；队列溢出或写盘失败时 Source
    fail-closed，不能静默丢帧
  - 同一主机进程间优先使用共享内存 latest-frame ring buffer 或等价零/少复制 IPC；不使用
    ZMQ 在 WSL 内反复序列化每帧约 800 KiB 的 raw Y16
  - VLA Provider 按 `ModelManifest` 从最新帧生成模型专用输入，例如 `224x224 uint8 JPEG`；
    不从 Web 预览回读模型输入，也不改变 Source 原始帧
  - Web 只接收约 5--10 FPS 的压缩 RGB/Depth 可视化；预览帧率和延迟不得冒充 Capture 或
    VLA 实际观测频率
  - 推理队列、预览队列和写盘队列完全独立；远程服务器、浏览器或 ROS2 中任一消费者变慢
    都不能阻塞相机 callback

- [ ] 定义版本化 `VLAProvider` 与 `ModelManifest`，使 Orchestrator 不依赖 π0、GR00T 或
  LeRobot 的厂商字段
  - Provider 最小能力：`get_manifest()`、`health()`、`warmup()`、`start_session()`、
    `infer()`、`reset()`、`cancel()` 和幂等 `close()`
  - 首批实现：`MockVLAProvider`、`ReplayVLAProvider`、标准 `GrpcVLAProvider`、
    `OpenPIWebSocketProvider`、`LeRobotGrpcProvider` 和 `GR00TZmqProvider`
  - `ModelManifest` 固定 model/checkpoint ID、revision、artifact checksum、provider protocol、
    compatible embodiment、输入图像角色/数量/分辨率/色彩顺序/历史长度、语言要求、state
    features、action features、关节顺序、单位、坐标系、绝对/相对表示和归一化统计 checksum
  - Manifest 同时声明 action horizon、`action_dt`、建议执行步数、最大消息大小、预期推理
    时延和缺失相机策略；未显式声明时禁止用黑图、全零状态或猜测维度补齐
  - Provider 建连后先完成 manifest/schema/embodiment 握手和模型 warmup；兼容性失败时不得
    进入 `ARMED`，首帧冷启动延迟不得落到真机执行阶段

- [ ] 定义标准 gRPC VLA v1 双向流，作为本项目控制的远程推理服务器主协议
  - RPC 至少包括 `GetModelManifest`、`Health`、`StartSession`、双向 `Stream`、
    `StopSession`；控制消息与观测/动作消息均带 `schema_version`
  - `ObservationEnvelope` 至少包含 session/generation/observation ID、task text、相机角色、
    图像 payload、图像硬件时间、机器人 state、state 采样时间、robot/calibration revision、
    client monotonic capture/send time 和可选 deadline
  - `ActionChunkEnvelope` 至少包含 session/generation/source observation ID、model revision、
    action space、joint names、units、`action_dt`、horizon、actions、server inference timing 和
    client 可校验的新鲜度字段
  - Protobuf 中只允许有界 numeric/string/bytes 字段，禁止 `pickle`、Python object ndarray
    和任意对象反序列化；JPEG 已压缩，不重复启用 gRPC gzip
  - 初始单消息上限建议 `8 MiB`，最终值按相机数量实测；服务端观测处理队列固定深度 1，
    客户端动作缓冲有界，依靠 HTTP/2 flow control 和应用层 latest-only 共同处理背压
  - 局域网部署至少使用 TLS + API Token metadata；跨主机正式部署优先 mTLS，Token 不进入
    浏览器、Capture 或日志
  - gRPC keepalive、health、deadline、取消和 reconnect 必须测试；重连创建新 session 与
    generation，绝不恢复执行旧 chunk
  - OpenPI 原生 WebSocket 和 GR00T 原生 ZMQ 只封装在对应 Provider 内，不能让兼容协议
    扩散到 Camera Service、Web API 或 ROS2 执行契约

- [ ] 实现 Observation Assembler，以相机硬件时间为视觉事实、以本地 steady clock 为实时
  新鲜度依据
  - 对每个 inference observation 选择最新有效 RGB，并按图像时间插值/最近邻匹配机械臂
    7 轴、灵巧手 6 轴、使能/急停/故障状态；保存同步残差和选择依据
  - 保持本仓库米/rad、13 轴当前命名、四元数 `xyzw` 和显式 frame/revision 契约；模型需要
    `wxyz`、EEF delta 或归一化向量时只在版本化 Model Input Adapter 边界转换
  - 同时记录 robot monotonic age、UTC 审计时间和 camera hardware timestamp；实时 deadline
    只按机器人本地 monotonic clock 判断，不用两台未同步主机的 `time.time()` 相减
  - 跨主机使用 chrony/NTP 改善日志对齐，并记录 RTT 与 server-reported inference duration；
    one-way latency 仅在有可信时钟同步证据时展示
  - 深度仅在 Manifest 声明且 checkpoint 确实训练过对应 modality 时进入模型；否则 raw
    depth 保留给 Capture、空间定位和安全，不把 Y16 强塞给 RGB checkpoint

- [ ] 实现异步 Action Chunk Broker，解耦约 5--10 Hz 模型推理和约 30 Hz 本地动作执行
  - 观测待推理队列深度固定为 1，始终覆盖旧观测；每个请求带 observation ID，不允许多个
    旧观测排队造成动作越来越迟
  - 动作按 session、generation 和目标 timestep 排列；执行过、过期、来源 observation 太旧、
    action schema 不兼容或当前 session 已取消的 chunk 直接丢弃并计数
  - 当前 chunk 剩余比例低于可配置阈值时提前请求下一次推理；缓冲耗尽前没有合格新 chunk
    时保持当前位置并结束/暂停 session，禁止重复最后动作或无限开环
  - 新 chunk 只更新尚未执行的未来区间；先实现 latest replacement + 速度/加速度受限续接，
    Replay/Shadow 证明 chunk 边界稳定后再评估 weighted temporal ensemble 或 RTC
  - RTC 不作为第一版必需项；若后续启用，显式版本化 overlap/frozen/ramp 参数并保存上一
    chunk，验证 diffusion/flow policy 兼容性，不能把实验特性宣传为通用能力
  - 记录 action buffer ms、queue depth、stale/drop/deadline miss、chunk overlap discontinuity、
    intra/inter-chunk velocity/acceleration/jerk，以及推理 p50/p95/p99

- [ ] 定义 NERO + Inspire 模型动作适配层，模型输出不直接等同硬件命令
  - `nero_inspire_grasp_v1`：7 轴机械臂 + 1 维 versioned grasp scalar，通过已验收的参数化
    hand skill 映射到六个驱动关节，用于适配常见 7+1 夹爪 checkpoint；它不代表完整灵巧手
  - `nero_inspire_joint13_v1`：7 轴机械臂 + 6 轴灵巧手完整动作，需要本机器人数据、对应
    normalization stats 和专门 fine-tune，不能直接套 DROID/Franka/ALOHA checkpoint
  - 母带始终保留完整 7+6 target/actual/raw/feedback；7+1 只是模型导出和运行 Adapter，不能
    反向降低 Capture/RobotDataset 信息量
  - 模型可输出 relative joint/EEF action，但 Adapter 必须以响应对应 observation 的 state 为
    reference 转成显式绝对目标；ROS2 Executor 第一版只接受完整命名的绝对 joint target
  - 明确机械臂 rad、灵巧手项目 rad、速度/力限制和厂商 raw 的边界；禁止按向量长度猜语义

### 首个模型：SmolVLA（已决定跳过 ACT）

- [x] 决定不实现 ACT 训练/部署基线，直接从 SmolVLA 进入 VLA 主线（架构决策，2026-08-27）
  - 仍保留 MockVLA、Replay、离线 open-loop 和 Shadow 四类非运动验证；跳过 ACT 不等于
    跳过基础数据、action schema、归一化、chunk 连续性或 ROS2 安全验收
  - 若 SmolVLA 出现问题，必须通过 Replay/Shadow 区分数据、Adapter、远程通信、Action
    Broker、ROS2 Executor 和模型本身，不能因没有 ACT 对照而把所有失败归因于模型

- [ ] 以 Hugging Face `smolvla_base` 450M 为第一款训练和远程部署模型
  - 使用 LeRobot 官方 SmolVLA policy/processor，不复制或魔改模型主体；本项目只维护
    Dataset Export Adapter、ModelManifest、远程 Provider、动作适配和机器人安全执行边界
  - 模型运行在独立 GPU Server 环境；机器人 WSL 不安装完整训练依赖，只运行轻量 gRPC
    client、Camera Service、Observation Assembler、Action Broker 和 ROS2 Executor
  - 第一版通过本项目标准 gRPC VLA v1 服务暴露 SmolVLA；服务内部调用 LeRobot policy，
    不让 Web/ROS 依赖 LeRobot Python 对象或 checkpoint 目录结构
  - `SmolVLAGrpcProvider` 启动时读取 checkpoint config、processor config、feature schema 和
    normalization stats，生成不可变 `ModelManifest`；任何字段缺失或 checksum 不匹配都拒绝
    `StartSession`
  - 服务器必须实现模型加载、warmup、health、单 session 串行推理、显存/设备信息、推理
    timing、取消和优雅关闭；第一帧冷启动结果不得进入 Execute

- [ ] 第一阶段使用 `nero_inspire_grasp_v1`，不直接训练完整六指 Action Head
  - 模型 state/action 均采用 8 维有名特征：机械臂当前 7 轴项目关节顺序 + 1 维
    `grasp_scalar`，单位与范围由 embodiment manifest 明确声明
  - `grasp_scalar=0`/`1` 分别对应版本化、真机验收过的张开端与抓握端；中间值通过六指各自
    的单调曲线生成 6 轴目标，不允许对六个关节简单复制同一个 raw/rad 值
  - 从六轴 actual 反算 observation grasp state 时，投影到同一手型曲线并保存 residual；
    residual 超过阈值时 observation invalid，不能用平均值伪装为可靠的一维手状态
  - v1 只承诺单一张开到 power-grasp 手型族，不支持 pinch、OK、逐指操作或复杂在手操作；
    Web 必须展示当前 embodiment 能力，模型也不能通过 Prompt 越权请求未声明手型
  - 母带继续保存完整 hand6 target/actual/raw/force/speed；8 维模型输入输出只是可重建的
    export revision，后续 `nero_inspire_joint13_v1` 不需要重新采集已存在的六轴事实
  - 在真手静态端点、全行程投影误差和物体抓取稳定性完成前，`grasp_v1` 只允许 Replay/Shadow

- [ ] 冻结 SmolVLA v1 的输入契约，先用单固定场景 RGB + 机器人本体状态
  - 首个视觉字段固定为 `observation.images.scene`，来源为 Gemini 336L Color；Camera Service
    保留 `1280x800@60 MJPG`，Model Input Adapter 按 checkpoint 配置生成带 padding 的模型输入
  - 模型实际输入 resize/crop、JPEG decode 后 RGB 数组、色彩顺序、dtype 和归一化处理必须
    与训练 processor 完全一致，并把变换 revision 写入 ModelManifest
  - 第一版不把 raw Depth、对齐 Depth、浏览器 MediaPipe 关键点或单目腕姿输入 SmolVLA；
    Depth 继续用于 Capture、质量分析和未来安全/空间定位
  - 第一版不虚构 wrist camera 或黑图占位；只有 checkpoint/Manifest 显式声明可 mask 的缺失
    camera 时才允许 padding，之后新增腕部相机必须形成新 model revision
  - state 使用与图像时间匹配的 arm7 + grasp1 actual，并附带 Driver READY/enable/fault 作为
    Orchestrator gate；安全状态不作为普通可学习数值混入 action feature
  - Camera Source 60 FPS、Observation Assembler 取样频率、SmolVLA 推理频率和 ROS2 30 Hz
    执行频率分开配置；第一版推理目标可从 5--10 Hz 测起，不提前承诺硬实时

- [ ] 冻结 SmolVLA v1 的语言任务边界和首个真实任务
  - 首任务为“拿起桌面指定颜色的方块，放入固定托盘”，限制物体类型、桌面工作区、托盘
    区域、光照范围和机械臂初始姿态；先验证有限域，不宣称开放世界泛化
  - 建立版本化 task catalog，Web 可显示中文，但模型 v1 使用稳定 canonical prompt；中文输入
    先映射到 catalog task ID/canonical prompt，不直接把任意自然语言发送到真机模型
  - 每条 episode 保存用户原文、canonical prompt、task ID、Prompt Adapter revision 和任务
    成败；训练/验证不得只靠字符串相似度猜 task
  - 数据覆盖若干离散方块起点、颜色和托盘目标，每个 variation 重复采集；训练/验证/测试按
    episode、物体位置和场景分组，禁止相邻帧随机拆分造成数据泄漏
  - 首轮目标至少 100 条人工审核为 valid/success 的完整 episode；另外保留失败、中止和接管
    episode 作审计，不自动混入训练集。实际最低数据量以分组验证结果为准，不把 100 条当成功保证

- [ ] 建立 SmolVLA 专用、可重建的数据导出与训练流程
  - 从无损 Canonical RobotDataset 导出 `smolvla_nero_inspire_grasp_v1/<revision>`，选择 scene
    RGB、canonical task、arm7 + grasp1 state/action，并保留回到 Source/RobotDataset 的 lineage
  - 在导出前校验图像/state/action 时间对齐、valid、finite、joint order、grasp projection
    residual、episode outcome、帧率和动作连续性；缺失项不得用零补齐
  - normalization statistics 只从 train split 计算，并保存 statistics checksum；val/test 和
    部署必须复用同一份统计，禁止在真机运行时重新估计
  - 冻结 LeRobot/SmolVLA commit、Python/CUDA/Torch、processor、训练 config、seed、数据
    revision、checkpoint checksum 和评估脚本；训练产物进入模型仓库/对象存储，不混入 Capture
  - 训练阶段记录 loss、validation open-loop action error、chunk intra/inter continuity 和
    grasp/arm 分项误差；低 loss 本身不作为部署准入
  - 官方建议约 50 episode 起步只作参考；本项目因新机械臂、动作映射和真实灵巧手采用至少
    100 条 valid episode 的首轮门槛，后续按 failure bucket 定向补数

- [ ] 完成 SmolVLA 从离线到真机的逐级验收
  - Offline schema：用少量 Capture 完成 export、processor 回读、一次 infer 和 action
    unnormalize 往返，确认 8 维字段名/顺序/单位完全一致
  - Open-loop：在保留 test episode 上比较预测与实际 action chunk，分别报告 arm、grasp、
    FK 末端轨迹和 chunk 边界误差；不能只报告统一 MSE
  - Replay：远程 SmolVLA gRPC server 接收历史 ObservationEnvelope，验证 manifest、消息限制、
    observation ID、deadline、取消、重连和完整审计，不连接 ROS Driver
  - Shadow：真实 Camera + 真实 ROS state 连续运行，记录模型动作和 Safety Projector 结果但
    executor 无运动权限；覆盖所有 task variations、模型超时、网络抖动和物体丢失
  - Execute：先使用 `grasp_v1` 单任务、固定低速、限定工作区和独立人工使能；必须完成
    Mock 臂/真手、真臂/Mock 手后才能双真机
  - 成功率、最大 action age、推理 p95、tracking error、safety clamp、chunk jerk 和物体碰撞
    等最终准入阈值由 Replay/Shadow 基线生成并版本化，不能在看到结果后临时放宽

- [ ] SmolVLA v1 闭环后再进入后续模型，不并行扩散未稳定的动作契约
  - π0.5 为下一优先级：复用相同 `nero_inspire_grasp_v1`、gRPC envelope 和 ROS2 Executor，
    仅新增 OpenPI Input/Output Adapter 与 Provider，形成同数据同任务对照
  - GR00T N1.7 为完整 embodiment/relative EEF 研究项：需要独立 modality config、数据导出、
    normalization 和至少 16 GB 推理/约 40 GB 微调算力评估，不直接复用 DROID tag
  - `nero_inspire_joint13_v1` 只有在 hand6 真机反馈、interaction/碰撞安全和足够逐指示范闭环后
    才进入 SmolVLA/π0/GR00T 微调；维度上限足够不等于具备现成控制能力
  - OpenVLA-OFT、π0-FAST 和其他模型通过相同 Provider/Manifest 接口保留兼容位，但不进入
    第一阶段开发范围

- [ ] 在独立 ROS2 仓库实现 `vla_executor`，作为模型和 Hardware Driver 之间唯一实时执行边界
  - 第一阶段定义有反馈、可取消的 `ExecuteVLAChunk.action`，由 executor 在本地约 30 Hz
    插值并复用当前 CPV/有应答 Driver 接口；禁止远程推理服务器直接发布硬件 Topic
  - `Execute` session 必须加入 Web/MCP/Bridge 统一硬件 owner；VLA 不得与手动 Web、技能包、
    MCP 或 Direct Console 同时发送运动命令
  - 校验 session/generation、13 轴名称和顺序、finite、关节限位、step、速度、加速度、工作区、
    tracking error、Driver READY/enable/estop/fault 和命令年龄
  - 模型永远无权 enable、reset、clear-error 或解除急停；物理急停 > Driver 安全状态 > owner
    lease > Safety Projector > VLA action
  - Camera/robot state 超时、gRPC 断线、Web lease 失效、模型故障、tracking error 超限或用户
    Stop 时立即失效 generation、清空未来动作、平滑保持/停止并返回明确结果
  - `ros2_control`、JointTrajectoryController 和 MoveIt Servo 真机验收后，再选择正式轨迹
    Action 或 Servo；当前 Humble 不依赖 Rolling 才有的 positions-only chunk upsampling
  - task-space 模型未来可走 MoveIt Servo 的 Pose/Twist 输入和碰撞/奇异位形检查；joint-space
    第一版不得绕过现有限位和碰撞安全投影

- [ ] 新增 Web VLA 操作页和后端 API，页面是运维/实验工作台，不承担推理或控制循环
  - 页面包含 Provider、model/checkpoint、embodiment、相机角色、任务文本、Capture 选择、
    `Shadow/Execute` 模式，以及预热、开始、暂停、停止和本地急停命令
  - 状态机显式为 `DISCONNECTED -> IDLE -> PREFLIGHT -> WARMING -> SHADOW/ARMED -> RUNNING
    -> PAUSED/STOPPING -> IDLE`，任意阶段可进入 `FAULT`；`ARMED` 仍需操作者确认机械臂净空、
    低速、物理急停和人工使能
  - API 初步包括 `/api/vla/providers`、`/api/vla/models`、`/api/vla/preflight`、
    `/api/vla/session/start|pause|resume|stop|status`、`/ws/vla/telemetry` 和相机 preview WS
  - 实时显示 Camera RGB/Depth FPS、frame age、同步残差、网络 RTT、server inference、端到端
    action age、buffer ms、stale/drop、safety clamp、tracking error、ROS owner 和故障原因
  - 浏览器 Stop 必须先在机器人本地失效 generation，不等待远程服务器 ACK；页面刷新、断网、
    重复标签页和后端重启均覆盖 lease/watchdog 测试
  - Prompt、model、checkpoint、embodiment、相机角色或 action representation 变化时必须结束
    旧 session、清空动作队列并重新 warmup/preflight，禁止在线热换后续执行旧动作

- [ ] 将每次 VLA session 纳入 Capture/RobotDataset 血缘与审计，不只保存最终成功轨迹
  - Source 保留完整原始相机流；另行保存实际送入模型的 resize/crop/JPEG、observation ID、
    task prompt、机器人同步 state 和 ModelManifest snapshot
  - 保存模型原始 chunk、Model Action Adapter 后动作、Safety Projector 后动作、ROS 实际下发、
    关节反馈、执行误差、网络/推理时延、丢弃原因、人工暂停/接管/急停和最终 outcome
  - failed/interrupted/unsafe episode 默认不进入训练 split，但保留审计和失败分析；人工审核
    可以显式改变用途，构建器不得自动删除失败证据
  - 模型 revision、Provider 版本、协议 revision、robot/URDF/calibration/safety profile、代码
    commit 和 normalization checksum 全部进入不可变 session manifest

- [ ] 分阶段验收 Web VLA，任何阶段不通过都不得跳级真机执行
  - 阶段 A：Mock Camera + MockVLA + Mock ROS，覆盖 schema、状态机、取消、断线、迟到响应、
    queue overflow、generation 和 owner 冲突
  - 阶段 B：真实 Capture Replay + 远程 Provider，验证输入转换、模型 warmup、延迟分布、chunk
    数值和完整留存，不连接机器人
  - 阶段 C：真实 Camera + 真 ROS 状态的 Shadow，模型持续推理但动作绝不进入 Driver；离线回放
    检查限位、FK 轨迹、intra/inter-chunk 抖动和 safety projection
  - 阶段 D：Mock 臂/真手、真臂/Mock 手分别低速执行，再进行双真机限定工作区任务；每阶段
    固定 checkpoint、场景、物体、初始姿态、急停条件和 operator
  - 阶段 E：完成长时、网络抖动、服务器重启、USB 重连、Web 关闭、ROS fault、机械臂失能、
    物体遮挡和模型异常输出故障注入
  - 在 Shadow 数据前不冻结 action horizon、execute steps、replan threshold、最大 action age、
    interpolation/RTC 参数或模型成功率门槛；最终阈值进入版本化 deployment profile

- [ ] 增加仿真碰撞与场景测试
- [ ] 为 Web 7860 端口增加鉴权或可信反向代理边界
- [ ] 如果需要 MCP combo，先形成新的接口、安全和执行语义设计，再在独立仓库实现

## P1 Ego 数采结构迁移

> **2026-08-20 迁移记录：** 路径迁移已实施。三个 `build_canonical*` 构建器、
> `derive_embodiment.py`、`measure_acceptance.py`、`verify_dataset.py`、
> `replay_rerun.py` 和 `app_web.py` 已统一使用 Capture 契约。旧 `src/out/` 仅保留显式
> 兼容读取，不自动移动或删除。迁移未修改关键点顺序、`xyzw` 四元数、坐标系、矩阵乘法、
> IK、滤波或 retarget 算法；现有 Ego 780 帧、RobotDataset 557 帧回读成功，新旧路径
> 数值列最大绝对差为 `0`。数据集专用 Python 3.12.13 + LeRobot 0.6.1 环境也已建立；
> 新 3 帧完整 Capture 通过官方回读和严格 v3 校验，并保存根级 `environment/` 快照。

- [x] 新增 `datasets/captures/` 正式 Capture Bundle 根目录，正式采集数据不再直接写入 `src/out/`
- [x] 为每次采集建立 `capture_<date>_<seq>_<uuid>/`，包含 `source/`、`ego/`、`robot_datasets/`、`lineage/` 和 `reports/`
- [x] 建立 Source 基础层：原视频/处理结果/已对齐 RGB-D 留存、标定快照、Parquet 流索引、Source -> Ego 帧映射、校验和及留存策略
- [x] 将 Ego 输出迁移到 `<capture>/ego/`，确保可直接由当前 `LeRobotDataset()` 加载
- [x] 将机器人派生输出迁移到 `<capture>/robot_datasets/<target>/<asset_revision>/<retarget_revision>/`
- [x] 增加 Capture manifest、Source/Ego/Robot checksum、基础血缘及处理记录
- [x] 将 Web、Gradio、腕部分析和轨迹分析默认路径接入 Capture，并拒绝跨 Capture/错误子目录混用
- [x] 使用文件锁原子分配同日 Capture sequence，避免并发构建得到重复序号
- [x] 增加 Capture `building/ready/failed` 生命周期；隐式读取只选择最新 `ready` 批次，失败或中断批次仅可显式检查
- [x] 将当前四元数顺序显式版本化为 `xyzw`，路径迁移不做静默数值转换
- [x] 增加 episode 级 annotations、RobotDataset `qa/episode_*.json` 和 Capture 级完整 QA 校验器
  - Ego/Robot annotation 默认 `unreviewed` 且不覆盖已有人工审核；Robot QA 自动检查帧索引和 state/action 非有限值
  - 关节限位、碰撞和指尖绝对误差在没有证据时明确为 `not_evaluated`，不以结构代理冒充物理验收
  - `verify_dataset.py --capture-bundle` 校验 Source、环境、严格 v3、血缘、sidecar 覆盖和 SHA-256，可用 `--json` 输出报告
- [x] 决定当前规范继续使用 `xyzw`，与 SciPy/现有全链一致；未来需要 `qwxyz` 时仅通过新 schema 和显式导出适配器转换，不原地改写
- [x] 明确时间契约：LeRobot `timestamp` 使用秒；Source 硬件时间使用 `*_timestamp_hw_us`，相对时间与同步残差使用毫秒；无硬件时间时标记 `fps_derived`
- [x] 区分 `episode0_camera` 与 `scene_world`；`coordinate_system.json` 2.0 逐字段声明 frame，生产者显式写入、消费者读取校验且不按来源猜测
- [x] 将 RGB-D 帧率、分辨率和质量阈值配置为可版本化的 quality profile，按实际设备同步模式验收；Capture 保存不可变快照，旧 30Hz/无硬件时间数据不冒充 60Hz 目标能力
- [x] 区分手腕绝对精度（需要真值）与无真值条件下的抖动、连续性和骨长稳定性
  - quality profile schema 1.1 和验收 JSON 显式记录测量类别、依据及真值可用性
  - 缺少 `ground_truth.wrist_pose` 或静止段标注时保持 `pass=null`，不以代理指标冒充通过
- [x] 建立 Python 3.12.x + `lerobot[dataset]==0.6.1` 隔离环境，补齐并验收严格的 LeRobot v3.0 目录与元数据结构
  - 命名环境 `lerobot-v3` 已固定 Python 3.12.13、CPU Torch 2.7.1、LeRobot 0.6.1 和 TorchCodec 0.4.0
  - 已补齐 MediaPipe 0.10.14、Pinocchio 4.1.0、dex-retargeting 0.5.0、Rerun 0.26.2、Trimesh 和 pytest；四条离线主命令及模型/网格初始化通过，`pip check` 无冲突
  - [x] 将 `lerobot-v3` 提升为 Web/视觉/IK/数据/直接 CAN 与串口的主运行时；新增
    `ros-humble` Python 3.10 薄环境，Web 自动为 rclpy reader/writer/runner 加载 Humble
  - [x] 保持 Web API、WebSocket、13 轴关节顺序与硬件控制类不变；硬件桥模式继续显式选择
  - [x] Web 真机 hand/arm 会话改为 ROS2 worker，订阅 Driver 状态并通过 Service 下发基础控制，不再直接占用 CAN/串口
  - [x] 为 Driver 增加 CPV 三段式 Service 并恢复 Web 真机实时跟随（Mock，2026-08-26）
  - [x] 让 Web 联合包 keyframe 回放复用 CPV Service，并等待 prepare ACK、处理失败与完成清理（Mock，2026-08-26）
  - [ ] 为 Driver 增加完整轨迹 Action、逐通道手力控与 clear-error 接口
  - [ ] 将 Web hardware lease 与 MCP/Bridge 控制权统一为跨客户端单 owner
  - 新 3 帧 Capture 的 `meta/data/videos`、真实 task 字段和根级 `environment/` 快照通过官方加载及 `verify_dataset.py --strict-v3`
  - 旧 0.4.4 Ego/RobotDataset 可由 0.6.1 回读，但旧 `tasks.parquet` 仅有匿名索引列，不满足 strict-v3；保留原样，不就地改写
  - 四个 LeRobot 生成入口在 `save_episode()` 后显式 `finalize()`，命令返回前完成 Parquet footer、视频和元数据落盘

### 剩余任务（2026-08-20）

- [ ] 接入真实采集设备的完整 Source 数据
  - [x] Gemini 336L SDK 原生临时 bag 已成功封装，确认 Color/Depth/IR/IMU 均有实际帧；
    临时 `/tmp` 冒烟数据不是正式 Capture（2026-08-27）
  - [ ] 保存设备 SDK 原生 RGB-D bag 容器到 `source/recordings/`；当前已有逐帧 JSONL
    journal，但它不是 SDK bag
  - [x] 保存原生 MJPG、未对齐 raw Y16 depth 及对应标定快照（2026-08-27）
  - [ ] 生成对齐 RGB 的 depth，并保存逐帧 valid 与对齐质量；Hardware D2C 当前未通过
  - [x] 将 RGB/Depth 原始硬件微秒时间戳写入兼容宽表和多流长表，计算真实同步残差
    （2026-08-27）
  - 使用真实采集生成完整 Capture，并通过 quality profile、strict-v3、`--capture-bundle` 和官方 `LeRobotDataset` 验证
  - 现有旧 Kinect 帧集不具备上述原始信息，不得从文件名或 FPS 补造
- [ ] 在新流程完成真实采集和全链回归后淘汰旧 `src/out/`
  - 先确认构建、派生、验收、回放、Web 和分析工具均不再依赖隐式旧路径
  - 保留必要第三方/历史留档，只删除可重建的旧实验输出
  - 移除 `--legacy-out`/`VLA_LEGACY_OUT` 兼容入口前补迁移说明和回归测试

### EGO 眼镜与多传感器演进（2026-08-24）

- [x] 明确 EGO 是以操作者眼镜/头戴第一视角为最终目标的人类演示母带；固定 RGB-D 是
  阶段性生产源，腕部设备和外部相机是增强或 Ground Truth 来源
- [x] 定版基础数值契约：米/rad、LeRobot 秒、Source 硬件微秒、同步残差毫秒、四元数
  `xyzw`，并使用相邻四元数点积非负保证时间连续
- [x] 建立 [EGO_DATA_STANDARD.md](EGO_DATA_STANDARD.md)，定义阶段 0--6、固定/头戴设备
  分档、动态相机变换、真值规则、质量指标和 LeRobot v3 交付闸
- [ ] 升级 Source 多传感器索引，同时保持现行单 RGB-D `stream_index.parquet` 可读
  - [x] 新增厂商无关的长表逐文件原子写入与校验入口，拒绝重复 stream、未知时钟、时间倒退和
    不安全相对路径；现行宽表保持不变（2026-08-24）
  - [x] 新增 `streams.parquet`：stream、sensor、modality、原生频率和 calibration ID
  - [x] 新增 `samples.parquet`：设备/主时钟、路径、valid、uncertainty 和原始 sample index
  - [x] 新增 `synchronization.json`：主时钟及 `t_master = a*t_device+b` 的偏移/漂移模型
  - 保留原始高频 IMU；Source -> Ego 对齐视图记录选样、插值、残差和来源 sample
  - 覆盖旧宽表回读、多频率、丢样、重启、时钟回退和跨设备漂移测试
- [ ] 定义并实现版本化眼镜 Adapter 接口
  - 原生容器、RGB/Depth、内外参、VIO/c2w、IMU、设备/主时钟和跟踪状态统一进入 Source
  - 不在 Adapter 内做机器人重定向、补帧或把无真值数据声明为绝对准确
  - 设备型号/SDK确定后新增 `ego_glasses_rgb*_v1` quality profile，不复用固定相机 60Hz profile
- [ ] 将动态相机位姿纳入新的 EGO schema revision
  - 写入 `observation.camera_pose`、valid、confidence、tracking state 和时间残差
  - 使用 `inv(T_global_camera_0) @ T_global_camera_t @ T_camera_t_hand_t` 统一到
    `episode0_camera`，补矩阵方向和 `xyzw` 回归
  - 腕姿增加 valid/confidence/source；缺失值不得用全零冒充
- [ ] 形成双手 Canonical schema
  - 明确 left/right 或 hand-slot 稳定语义、21 点顺序、逐手 visibility 和 handedness
  - 单手旧 Capture 保持只读兼容，不原地改列或猜测槽位
- [ ] 接入腕部设备的可选能力层
  - 首选高频 6 轴 IMU + 视觉校正；禁止用 IMU 双积分冒充长期绝对位置
  - 光学标记、数据手套、触觉、sEMG 分别声明 capability、标定、valid 和独立频率
  - 未启用的能力不写全零训练特征；触觉/接触指标保持 `not_evaluated`
- [ ] 建立 Ego-Exo / Motion Capture 小规模 Ground Truth 资格数据
  - 测量 VIO ATE/RPE、腕部位置/朝向误差、三维尺度、RGB-D 对齐和时钟同步
  - 固定 demonstrator/scene/object/trajectory 与标定版本，保存真值来源和不确定性
  - 依据试采冻结头戴检出率、朝向误差和设备专用阈值，不以代理指标替代绝对精度
- [ ] 完成阶段化验收
  - 阶段 1：真实固定 RGB-D 原生 Source -> EGO -> strict-v3/Capture/官方回读
  - 阶段 2：至少 30--100 条多任务 episode，按人/场景/物体分组切 train/val/test
  - 阶段 3--4：重定向结果单独进入 RobotDataset，伪动作、遥操作命令和实际动作显式区分
  - 阶段 5：移动/头戴原型完成 c2w、重定位、丢跟踪和静止段验收
  - 阶段 6：眼镜 + 腕部设备 + 外部真值完成端到端 Capture 和质量报告
- [ ] 建立眼镜数据治理
  - 操作者/旁观者许可、音频默认关闭或独立授权、人脸/屏幕处理和发布边界
  - Source 原始数据与匿名派生数据分别定义 hot/cold/sampled/deleted 留存策略

---

**最后整理**：2026-08-27
