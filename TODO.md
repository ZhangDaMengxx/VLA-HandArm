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
  - [ ] 相机具体型号和 Orbbec SDK 确认后实现 336 原生 RGB-D/硬件时间戳 Adapter，并用
    真机数据验证 `T_flange_wrist_camera`；当前 OpenCV Adapter 不代表设备 Source 已闭环
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
  - 保存设备 SDK 原生 RGB-D 容器到 `source/recordings/`
  - 同时保存未对齐 raw depth、对齐 RGB 的 depth 及对应标定快照
  - 将 RGB/Depth 原始硬件微秒时间戳写入 `stream_index.parquet`，计算真实同步残差
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

**最后整理**：2026-08-24
