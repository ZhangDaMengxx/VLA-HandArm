# 更新日志 (CHANGELOG)

项目的所有重要变更都记录在这里。

格式基于 [Keep a Changelog](https://keepachangelog.com/)，版本号遵循日期格式。

---

## [2026-08-24] - 灵巧手资产映射与单关节验收

### Fixed (修复)

- 统一本仓手势安全表、ROS writer 和 URDF 生成覆盖到正式资产/驱动的
  `thumb_pitch=0.48`、四指 `1.333`。
- 修复 `limit` 派生状态使用旧关节名而退化为满弯 `raw 0`，捏合与 OK 手势
  恢复使用可行域下界加 `LIMIT_MARGIN`。
- 新增驱动、手势表、ROS writer、生成覆盖和正式 URDF 的一致性测试。

### Verification (验证)

- RH56DFX 真机在 `speed=15`、`force=250`、空载单关节路径下完成 60/60 个可行点；
  六关节命令端点均为 `safe_max_u=1.0/raw=0`，端点反馈为拇指 `0/0`、四指
  `48/59/55/60 raw`；峰值温度 52℃、峰值力绝对值 84，全程错误位为零。
- interaction 与自碰撞边界尚未执行；旧 `aborted` Profile 只保留诊断。

## [2026-08-21] - 开发基线收口

### Audit (审计)

- 新增 `src/HAND_LIMIT_AUDIT_2026_08_21.md`，完成灵巧手 span/limit 第一阶段离线审计
- 确认独立 Bridge 当前先用旧 `hand_pose` 映射检查可行域，再用新驱动映射写入 raw；
  12 个内置语义姿态中 10 个的实际 raw 与安全检查姿态不同
- 审计 8 个录制动作包：原样回放通常保持权威 `raw_vendor`，但旧包的 rad/3D 解释及
  重新保存存在兼容风险
- 给出 raw 物理标定、安全下发包络、URDF/Three.js 模型限位三层契约和分阶段真机
  准入方案；本阶段未连接串口、未下发动作、未修改限位
- 按型号抽象决策将统一角定义为 datasheet/URDF 资产标称 rad；新增 RH56 型号 JSON、
  Mock/真机 Adapter、分阶段 commissioning、边界二分、原子可恢复 Profile 和安全投影
- 硬件运动必须经过 `--hardware` 与 `--allow-motion CONFIRM_HAND_MOTION` 两级授权；
  Mock Profile 默认不能用于真实运行时，过温/未知故障/过流/连续缺样按 fail-closed 中止
- 完整 CLI Mock 与投影验收通过，相关新旧手部链定向回归 `37 passed`；该离线回归未连接真机

### Changed (变更)

- 同步项目入口、开发手册、硬件安全说明和 AI 文档导航，明确资产标称 rad、分阶段真机
  commissioning、软冻结边界，以及 Profile 尚未接入 Web/Bridge 的当前状态
- 将最终 451 帧视频动作包以 `data/gestures/拿螺丝刀.json` 纳入版本库，显式使用 `timeline_latest`；删除早期 645 帧链路样本 `episode_000000.json`
- 测试和现行文档统一引用实际动作包文件名，继续验证 7.507 秒源时间轴和末帧保持语义

### Fixed (修复)

- 真机首次单关节扫描暴露低速途中短时稳定被提前当作到位的问题：现在必须同时满足
  目标容差和稳定条件，等待上限按剩余 raw 行程动态扩展
- 纯跟踪超时且没有力/堵转证据时改记为 `inconclusive`，不再生成伪物理边界；新增慢速
  执行器和无接触卡住回归，旧 aborted Profile 禁止续用

### Verification (验证)

- RH56DFX 真机只读 preflight 通过：5/5 遥测有效、六通道错误位和电流为 0、最高温度
  38℃；`STATUS=2` 仅记录，未写速度、力或角度，尚未执行单关节运动
- 首次单关节扫描在小指回张开误差 `73 raw` 时安全中止并成功软冻结，未进入联合阶段；
  等待判据修复后的相关离线回归为 `39 passed`
- Python 编译、`git diff --check` 和三个 Web Node 测试通过
- 动作时间轴、播放器、视频导入和异步 IK 定向回归 `40 passed`
- 隔离已记录的手指限位漂移、旧轨迹夹具、测试变量和 TestClient 后台线程阻断项后，离线宽回归 `191 passed`、3 个既有 `PytestReturnNotNoneWarning`
- 使用现有 Ego 样例的真实 21 点完成 21 帧 Mock WebSocket 冒烟：联合锚定进入 `following`，6 轴手目标与 7 轴臂目标均成功入队，IK 示例耗时 `0.54ms`、结果年龄 `3.44ms`
- Mock 统一释放接口确认灵巧手释放、机械臂请求回零并释放；开发基线通过 `--ff-only` 同步到 `deploy` 分支
- 全量 `pytest` 仍会被 P0 手指 span/limit 不一致测试在收集阶段主动终止；未修改或掩盖该安全问题
- 未连接或驱动真机

## [2026-08-20] - Ego Capture Bundle 路径迁移

### Added (新增)

- 新增 `src/live_ik_scheduler.py`：合体实时跟随使用单 worker、深度 1 pending 的 latest-only IK 调度，记录 source frame、会话、锚点和授权版本
- 新增 `src/capture_bundle.py`，集中管理 Capture 创建、最新批次解析、Ego/RobotDataset/轨迹/报告路径
- 新增 `bundle.json`、Source/Ego/Robot 元数据、SHA-256 校验表和基础处理血缘
- 新增 Capture `building/ready/failed` 生命周期和异常退出失败记录
- 新增 Source 原始输入归档、`stream_index.parquet`、Source -> Ego 帧映射和显式时间来源
- 新增 `coordinate_system.json` 2.0 契约，逐字段声明 `episode0_camera`、`scene_world`、腕部局部系和 RGB 像素系
- 新增 `configs/quality_profiles/` 与 `src/quality_profiles.py`，提供 60 Hz 目标及三类旧数据兼容口径
- 每个 Capture 新增不可变 `source/quality_profile.json` 快照，acquisition 记录 profile ID 和 revision
- quality profile 升至 schema 1.1，阈值声明测量类别和是否需要真值；旧 schema 1.0 快照继续可读
- 新增 `src/compare_dataset_numeric.py` 及路径/数值回归测试
- 新增 `datasets/captures/README.md`；真实 `capture_*` 数据由 `.gitignore` 排除
- 新增 `.envs/lerobot-v3` 数据集专用环境约定和 `environment/lerobot-v3-dataset.txt` 可复现依赖
- 新增 `verify_dataset.py --strict-v3`，校验 v3 元数据、Parquet、视频、feature 及帧/episode/task 计数
- 每个 Capture 新增根级 `environment/runtime.json`、`requirements.txt` 和 `environment.lock` 生成环境快照
- Ego/RobotDataset 新增逐 episode annotation；默认 `unreviewed`，生成器不覆盖已有人工审核
- RobotDataset 新增逐 episode QA，自动检查帧索引与 state/action 有限性，物理项缺证据时标为 `not_evaluated`
- 新增 `verify_dataset.py --capture-bundle` 和 JSON 报告，覆盖 Source、环境、严格 v3、血缘、sidecar 与 SHA-256

### Changed (变更)

- 合体摄像头实时跟随的机械臂默认速度由 `20%` 调整为 `50%`；灵巧手实时速度继续保持 `1000`
- 合体跟随不再在灵巧手目标入队前同步等待机械臂 IK；灵巧手与 IK/机械臂分别走独立 latest-target 通道，WebSocket 立即返回 IK 入队状态和最近完成结果
- IK 输入超过 `180ms`、结果超过 `200ms`、会话/锚点/授权失效时直接丢弃；重锚、冻结、丢手、异常或断线会使在途结果失效并回收 worker
- 灵巧手调试的视频转动作改为逐源帧解析到实际 EOF，移除默认 `stride=3`、200 帧前端限制和 2000 帧后端限制；进度分开显示源帧与检出手帧
- 视频和密集 JSON 录制回放改为 `timeline_latest`：保留解码 PTS/`t_ns`，按墙钟只发送最新到期目标，不突发补发过期帧，最终目标保持必发
- 时间轴 `ANGLE_SET` 使用非等待回复的快速写；`SPEED_SET`/`FORCE_SET` 仅在变化时写，播放器提高到 200Hz 并报告覆盖帧数与最大调度延迟
- 录制器接受 `hand_gesture_pack/1`、`/2` 和旧无模式视频结构；`data/gestures/拿螺丝刀.json` 使用显式 `timeline_latest`，手工关键帧继续使用 `keyframe_strict`
- 三个 Canonical 构建器默认创建 Capture 并写入 `<capture>/ego/`
- `derive_embodiment.py` 默认写入 `<capture>/robot_datasets/<target>/target_revision_v001/retarget_v001/`
- 轨迹改存 RobotDataset `exports/workbench/`，验收汇总改存 Capture `reports/retargeting/`
- 验证、Rerun 回放和 Web 管线统一从一个活动 Capture 解析全部数据；隐式读取只选择最新 `ready` Capture，不让较新的半成品遮住完整批次
- 旧 `src/out/` 仅通过 `--legacy-out` 或 `VLA_LEGACY_OUT=1` 显式兼容，不自动移动或删除
- 当前 `xyzw` 四元数、关键点、矩阵、IK、滤波和 retarget 数值语义保持不变
- RGB 视频保留原始 MP4，处理结果保留原文件，RGB-D 保留参与构建的原分辨率 RGB 和对齐深度；同盘优先硬链接、跨盘复制兜底
- 普通 RGB、外部处理结果和标定 RGB-D 显式写入腕姿 frame；派生、验证、验收、腕部分析与 Rerun 读取并校验该声明，不按 `source_kind` 推断
- 三个 Canonical 构建器按来源选择默认质量 profile，支持 `--quality-profile` 显式选择；RGB-D 记录实际 RGB/Depth 尺寸
- `measure_acceptance.py` 按 Capture 快照读取阈值，分开报告 Source 设备能力、硬件同步和 LeRobot 内部 cadence；Web 验收卡显示 profile 与 Source 指标
- 验收报告分开输出手腕绝对位置误差、静止段抖动、位置/姿态连续性和骨长稳定性；缺真值或静止标注时返回 `pass=null`
- RGB-D 像素对齐缺参考对应点时不再用腕深度连续性判定通过；Web 验收卡标识“需真值”“稳定代理”和“连续性”
- LeRobot 数据集时间字段按 0.6.1 实际输出明确为 float32 秒；Source 硬件时间继续使用 int64 微秒
- 三个 Ego 构建器和 RobotDataset 派生器在 `save_episode()` 后显式 `finalize()`，命令返回前完成文件落盘
- 当前存储契约继续固定为 `xyzw`；未来 `qwxyz` 仅作为新 schema 的显式导出转换，不原地改写既有 Capture

### Verification (验证)

- 异步 IK、mailbox、腕部映射和合体页定向回归 `66 passed`；100ms 慢 IK、单 pending、旧目标覆盖、过期丢弃及求解途中 release/close 均有自动测试
- 隔离 6 个既有安全表、旧轨迹 fixture、测试变量和 TestClient 后台线程阻断文件后，离线宽回归 `190 passed`、3 个既有 `PytestReturnNotNoneWarning`；三个 Web Node 测试通过
- 系统 Python 下 Capture/Source、路径、坐标契约、旧 Capture 兼容、数值及 Web/腕部组合回归 `70 passed, 1 skipped`；跳过项为系统环境缺少 pyarrow
- quality profile、Capture、数值比较及 Web/腕部相关回归 `89 passed`；三个 Web Node 测试通过
- 绝对/代理指标拆分后，quality profile 定向测试 `12 passed`，隔离既有硬件、外部服务和旧轨迹 fixture 后离线回归 `169 passed`；三个 Web Node 测试通过
- schema 1.0 的既有 3 帧 Capture 可继续生成新报告：缺真值的绝对精度保持 `pass=null`，新旧 Ego 10 个数值列最大绝对差仍为 `0`
- 实际 lerobot 环境用同一 3 帧 processed 输入重建带 profile 快照的 Capture：状态为 `ready`、LeRobotDataset 回读 3 帧/1 episode，`device_class` 与 `sync_mode` 正确
- quality profile 接入前后同一输入的 10 个数值列最大绝对差为 `0`
- 在实际 lerobot 环境完成 3 帧 processed Source -> Ego 全链路：Capture 为 `ready`、硬件微秒时间戳和 Source -> Ego 映射正确、LeRobotDataset 回读成功
- 用同一 3 帧处理结果重建 2.0 坐标契约 Capture：回读显示 `episode0_camera`/`wrist_local_mano`，新旧 10 个数值列最大绝对差为 `0`
- 现有 Ego 780 帧与 RobotDataset 557 帧通过当前 `LeRobotDataset` 回读
- 同一旧数据通过新旧路径比较：Ego 10 个、Robot 7 个数值列最大绝对差均为 `0`
- Python 3.12.13 + LeRobot 0.6.1 数据集环境 `pip check` 通过；旧 Ego 780 帧和 RobotDataset 557 帧由官方类离线回读成功
- 新 3 帧完整 Capture 通过官方 LeRobot 0.6.1 加载和 `--strict-v3`，五个 processed 输入核心字段最大数值差为 `0`
- 新 Capture 根级 `environment/` 三项快照完整；旧 0.4.4 数据虽可回读，但其匿名列 `tasks.parquet` 被 strict-v3 正确拒绝
- Capture/strict-v3/quality/numeric 定向回归 `38 passed`；隔离既有硬件、外部服务和旧轨迹 fixture 后离线回归 `174 passed`，三个 Web Node 测试通过
- `verify_dataset.py` 将 RobotSpec 延迟到 CLI 主流程导入，strict-v3 库接口不再受技能测试顶层 `schema` 模块污染
- episode/Capture QA 定向测试 `26 passed`，相关组合回归 `50 passed`
- Python 3.12 环境重新生成的新 3 帧 Capture 在命令返回时通过 `--capture-bundle`、strict-v3 和官方 LeRobot 加载
- QA 改为逐 Parquet 聚合、checksum 改为分块读取后，定向回归 `41 passed`，最终离线宽回归 `177 passed`，三个 Web Node 测试通过
- Python 编译与 `git diff --check` 通过；未连接或驱动真机
- 全仓 pytest 仍被既有 Pinocchio 缺失、手势表参数漂移和旧轨迹技能 fixture 问题阻断；本次顶层回归另确认既有 `SIM` 测试变量与手势 raw 限位断言各 1 项失败

### Remaining (未完成)

- 采集设备原生 RGB-D 容器、未对齐 raw depth、真实相机硬件时间戳，以及限位/碰撞/指尖误差的物理 QA 证据尚未接入

---

## [2026-08-19] - 实时人手合体跟随

### Progress (进度)

- [x] 新增单目腕部位置估计、手掌姿态、连续多帧联合锚定、相对位姿映射和末端限幅核心
- [x] 新增纯离线单元测试，覆盖锚定无跳变、位置/姿态限幅及丢手冻结
- [x] 接入后端合体跟随协议、机械臂 IK、7 轴 latest-target CPV 和 Mock/真机安全门
- [x] 将视频跟随迁入合体页并完成合并锚定/冻结按钮状态机，移除灵巧手页重复入口
- [x] 接入 Three.js 联动并完成桌面/移动端浏览器验收

### Added (新增)

- 新增 `live_wrist_tracking.py`：world/image landmarks 腕姿估计、稳定检测、位置/姿态联合锚定和相对末端限幅
- 新增隔离 Pinocchio 依赖的 `live_ik_worker.py`，提供 NERO FK/IK JSON lines 接口
- 新增合体页摄像头与单按钮状态机；状态自动切换锚定、冻结和重新锚定
- `/ws/hand/mimic` 统一返回 7 轴机械臂、6 轴灵巧手、腕姿、锚定状态和安全/IK 结果
- 合体页状态行显示当前姿态三轴角度及是否触顶，便于观察回放一致链路和限位状态
- 合体页新增实时推理设备清单：按浏览器能力显示自动、CPU/WASM、GPU/WebGL；macOS 将 GPU 标识为 Apple GPU（WebGL/Metal）
- 新增页面关闭统一释放接口 `/api/hardware/release`，供 `pagehide/sendBeacon` 在浏览器销毁时执行回位和通道释放

### Changed (变更)

- 视频跟随只保留在“实时 Live · 合体”页，删除灵巧手页旧摄像头入口
- `arm_console` 增加 CPV 连续跟随命令，机械臂 latest-target mailbox 扩展为 7 轴
- Mock 和真机使用相同 WebSocket、IK 和 7+6 目标结构；真臂增加显式授权、在线、使能和冻结安全门
- “实时 Live · 合体”页的灵巧手速度默认改为 1000，接入成功及启动摄像头时显式下发当前实时速度；其他入口仍保留驱动默认 500
- Mock 机械臂初始姿态改为远离关节限位的弯肘中间姿态；腕部位置和姿态都以锚点为基准做相对映射，姿态按每轴限幅后进入 IK
- 合体腕部位置使用三轴 One Euro 滤波（`min_cutoff=1.2Hz`、`beta=0.5`），腕部姿态使用旋转向量 One Euro 滤波（`min_cutoff=1.8Hz`、`beta=0.8`）；200ms 丢帧、联合锚定、冻结、丢手或跟随异常时重置
- 删除 Mock 机械臂的自动正弦轻微摆动；Mock 关节反馈现在始终保持最后下发目标
- 统一跟随准备位为 `[0, -0.7, 0.002, 1.298, 0.002, -0.008, -0.591] rad`；真臂在获得显式授权后启动摄像头跟随前先移动到该位并等待到位
- 实测后移除 `实时局部轴` A/B 分支，实时腕姿固定使用与 RGB 回放一致的 MediaPipe/MANO frame、world 轴增量和左乘组合
- Mock 姿态限幅按准备位固定末端位置的 IK 扫描结果调整为 X `-90/+60°`、Y `-115/+50°`、Z `-175/+155°`；真机继续使用保守的 `±45/±25/±35°`
- 摄像头实时模式改为完整的机械臂姿态生命周期：每次启动都下发弯肘跟随准备位，每次关闭或启动失败都回到七关节全零伸直位；Mock 同步更新 Three.js，真机同步下发硬件并等待到位
- 摄像头关闭或启动失败回滚时，参与本次跟随的灵巧手先清空实时目标队列并恢复到全张开位，随后机械臂再回伸直位；Mock Three.js 同步显示张开
- 推理设备显式选择 CPU 或 GPU 时严格使用对应 MediaPipe Tasks delegate；仅“自动选择”允许 GPU → CPU → Legacy 降级，摄像头运行期间锁定清单
- 页面导航改为等待旧页面完成安全释放：灵巧手张开后断串口，机械臂回全零伸直位后断 CAN，再开放新页面接入

### Safety (安全)

- 联合锚点使用机械臂当前关节 FK，避免开始跟随时跳到预设位姿
- 单目位置只标记为 `monocular_scale` 相对量；真臂位置限幅收紧为各轴 `±20mm`，末端姿态只允许锚点附近的有限旋转
- 丢手、左右手变化、连续 3 次 IK 失败、急停/冻结、未使能和断线停止投递并冻结跟随
- 本次未执行真实机械臂运动；真机验证继续列在 `TODO.md`
- 摄像头姿态切换前先清空机械臂 latest-target、退出 CPV，再下发 `move_j`；真机仍要求在线、已使能、未急停和显式授权，并在每次启动时确认准备位与关闭回伸直位的无碰撞路径
- 灵巧手回张开命令会等待正在下发的实时帧结束，避免关闭摄像头时的末帧覆盖安全张开位
- 页面退出时机械臂仅在在线、已使能且未冻结时尝试回零；不满足条件、掉线或 15 秒超时仍会继续关闭 CAN，通道释放优先
- 明确浏览器关闭的 `pagehide/sendBeacon` 仅为尽力释放；进程强杀、断电或断网仍需后续服务端 heartbeat/lease watchdog 兜底

### Documentation (文档)

- 修正源码迁移后硬件调用示例仍引用 `sim.*` 的旧路径，统一为 `src.*`
- 在入口、开发、硬件、状态和待办文档中补齐页面释放机制的保证范围与后续租约任务

### Fixed (修复)

- 修复翻转手心/手背时仅按掌宽估深造成的伪平移：单目尺度改为 MediaPipe world 掌宽/掌长和 3D 投影补偿
- 提高腕部姿态 One Euro 对快速挥手的响应，并为 Mock 使用匹配准备位 IK/`joint7` 余量的非对称姿态限幅；真机继续保持保守范围
- WebSocket 增加 `orientation_delta_deg` 与 `orientation_limited_axes`，可直接判断姿态量和具体触顶轴
- 实时腕姿复用离线回放的 MediaPipe/MANO `operator2mano` 腕部 frame，不再只复用左乘组合
- 将姿态角和触顶轴同时放到 WebSocket 顶层，前端无需解析 `arm` 子对象即可显示
- 修复接近 180° 翻掌时旋转向量轴退化导致 WebSocket 中断；半转和近半转姿态统一从对称部分稳定求轴
- 修复翻掌越过 90° 时主值欧拉分解切换等价分支，导致另外两轴突跳约 180°、IK/Three.js 镜像跳变；现按上一帧选择最近的等价分支并跨 `±180°` 连续展开，再执行逐轴限幅
- 显式按 `configs/` 解析 dex-retargeting 的相对 URDF 路径，修复从仓库根启动新服务时找不到 `assets/hand` 的问题
- 移除锚定按钮前重复的 6 帧硬稳定门槛，首个有效手帧即可发起联合锚定
- 锚定改为固定采集 12 个有效帧并剔除偏差最大的 25% 样本，位置/姿态抖动保留为质量提示，不再无限阻塞
- 锚定边沿命令只在服务端确认应用后清除，避免丢手或无效帧吞掉用户点击
- 合体页显示锚定进度、位置抖动毫米数和姿态抖动角度
- 修复合体 Three.js 被摄像头目标与臂/手遥测交替写入导致的姿态闪烁；摄像头开启时独占 3D，关闭后恢复最近实际反馈

### Verification (验证)

- Mock 臂 WebSocket 连续采样 12 帧仅有 1 组关节角，确认未下发目标时完全静止
- 跟随准备位的 Mock/IK 局部可达性验证通过；真机准备位流程未执行真实运动
- 新 Mock 弯肘姿态下，WebSocket 位置跟随 IK 成功；探针产生 `17.7mm` 末端平移，FK 朝向变化仅 `0.000002deg`
- 相关 Python 测试 40 项通过，三个 Web Node 测试通过
- 浏览器写入所有权探针确认：摄像头开启期间遥测不覆盖 Three.js，摄像头目标正常更新，关闭后恢复实际反馈
- Mock WebSocket 状态完整经过 `waiting -> anchoring -> following`，返回 7+6 目标且 NERO IK 成功
- Mock 姿态探针在腕部位置不变时施加 20° 手掌旋转，IK 成功且七轴目标最大变化 `0.0159rad`，确认挥手姿态已进入机械臂链路
- One Euro 腕部位置单测确认交替抖动被压低、超过 200ms 的采样间隔直接重置；滤波状态通过 WebSocket `wrist_position_filter` 质量字段可观察
- 恢复姿态跟随，并改为 SO(3) 增量 One Euro 滤波，避免绝对旋转向量在 ±π 边界跳变；WebSocket 同时返回 `wrist_orientation_filter` 质量字段
- 几何探针确认翻掌 `0°→40°→80°` 时估计深度仅变化约 `2.5mm`
- 最终 Mock WebSocket 双向翻掌均保持 `following` 和 `ik_ok=true`；正向触到 Z 轴 `-50°` 时明确返回 `orientation_limited_axes=[false,false,true]`，反向达到约 `+74.4°` 未触顶，腕部位置变化约 `1.9mm`
- 本次相关回归为 Python `42 passed`、三个 Web 测试通过；全仓测试另有既有 Pinocchio 缺失和 hand/skill fixture 收集错误，未归因于本次改动
- Playwright 在 1440x900 和 390x844 验证无控制台错误、横向溢出或控件重叠
- Three.js 画布像素检查确认桌面和移动端装配体均非空并正常渲染
- 固定准备位与末端位置做 5° 步进 IK 扫描：X 轴约 `-95/+65°`、Y 轴约 `-120/+55°`、Z 轴约 `-180/+165°`，Mock 限幅在此基础上保留 5-10° 余量
- 新增 90° 欧拉分支和 `+180° -> +181°` 连续展开回归；本轮相关 Python 测试 `49 passed`，三个 Web Node 测试通过
- Mock WebSocket 纯翻掌 `0° -> -160°` 连续 61 帧保持 `following`、`ik_ok=true`、无触顶和符号反转，7 轴目标最大单帧变化 `0.0415rad`
- Playwright 桌面/移动端无控制台错误、溢出或控件重叠；Three.js 不同关节目标产生 `45,603` 个 PNG 字节变化，摄像头独占和停止后遥测恢复计数符合预期
- Mock 运行态依次执行 `home -> tracking_ready -> home`，服务端 `rad/target` 分别精确到达全零伸直位、`[0,-0.7,0.002,1.298,0.002,-0.008,-0.591]` 和全零伸直位；未连接 CAN
- Playwright 直接执行页面摄像头姿态生命周期，准备位/伸直位关节误差均为 `0`，Three.js 往返各产生 `45,253` 个 PNG 字节变化，浏览器控制台无错误
- 推理运行方式检测及严格 CPU/GPU 选择测试、页面释放结构测试通过；本轮相关 Python `52 passed`，三个 Web Node 测试通过

## [2026-08-18] - 源码、测试与第三方资产归整

### Changed (变更)

- 将 `sim/` 迁移为 `src/`，同步代码、部署脚本和现行文档中的运行路径
- 将测试集中到 `src/test/`，真机测试单独放入 `src/test/hardware/` 并从默认 pytest 收集中排除
- 将上游源码、厂商 SDK 和 RGB-D 资产集中到 `third_party/`，保留上游内部目录结构
- 将项目维护的 dex-retargeting 适配层迁到 `third_party/overlays/` 并继续进入 Git
- 修正 Web、回放、轨迹和 RGB-D 默认路径，以及测试迁移后的 Python/JavaScript 相对导入

### Removed (移除)

- 删除迁移完成后不再使用的 `migrate_*.py`、`verify_migration.py`、`final_summary.py` 和 `git_commit_migration.sh`

### Documentation (文档)

- 按 `README_DOCS.md` 的现行、专题和历史分层更新入口文档
- 新增 `third_party/README.md`，记录第三方目录边界和 Git 策略
- 合并 Web/Windows/WSL/HTTPS 文档为 `WEB_ACCESS.md`
- 合并 2026-08-10 资产、URDF、装配和 Combo 迁移报告为 `MIGRATION_2026_08_10.md`
- 将装配位置调整流程并入 `src/build_urdf/README.md`
- 新增 `AGENTS.md`，将 AI 明确引导到 `README_DOCS.md`

### Verification (验证)

- `src/` Python 语法编译和两个 Web Node 测试通过
- 合体页静态测试通过；系统 Python 的其余离线集合有 157 项通过
- 已知失败来自缺少 FastAPI/uvicorn/pinocchio 的解释器差异、既有 10 项手势限位不一致，以及旧轨迹 NPZ 的 12 关节命名契约

## [2026-08-14] - MediaPipe Tasks 与 latest-target 真机控制

### Added (新增)

- 本地化 `@mediapipe/tasks-vision@1.0.1`、Vision WASM 和 Hand Landmarker Full 模型，并记录固定校验值
- 新增 Tasks/Legacy 统一追踪适配层，支持 Tasks GPU → CPU → Legacy 自动降级
- 新增 30Hz `LatestTargetMailbox`：最多一个待发目标、一个真实 ACK 在途，250ms 旧目标丢弃，100ms ACK 超时
- 新增 `frame_id`/ACK token 关联及 frontend、mailbox、stdin、RS485、tracking 分段性能日志
- 新增六关节 One Euro 实时目标滤波和 `perf-hand/filter` 日志；滤波状态按 WebSocket 隔离

### Changed (变更)

- 摄像头循环改用 `requestVideoFrameCallback()`，不支持时回退 `requestAnimationFrame()`
- `/ws/hand/mimic` 在返回 3D 预览角度的同时投递硬件目标；HTTP fallback 只保留 retarget 与预览
- 连续视觉控制期间 `ANGLE_ACT` 保持 30Hz、`FORCE_ACT` 降至 10Hz，全量遥测在目标空闲 500ms 后补读
- `HandDebugSession` 增加 stdin 写锁、真实子进程 ACK 等待和单摄像头硬件控制权
- 真手目标在 retarget 后使用 `min_cutoff=1.5Hz`、`beta=2.5`、导数截止 `1.0Hz` 的 One Euro 滤波；超过 200ms 无目标时重置

### Fixed (修复)

- 移除会积累滤波尾差的 `0.015-0.02rad` 大位置死区，改为统一 `0.0005rad` 硬件分辨率门限，修复张手末端和保持姿态时约 `0.02rad` 的台阶式卡顿抖动

### Removed (移除)

- 移除 Tasks 主链对旧 `camera_utils.js` 帧循环的依赖
- 移除 WebSocket retarget 返回后逐帧追加 `/api/hand/command` HTTP 硬件请求的双重链路
- 移除无界旧目标排队；等待硬件 ACK 时只保留最新目标
- Legacy 引擎和旧资源仍保留用于灰度降级，尚未删除

### Verification (验证)

- Node 适配层与前端传输测试通过；mailbox 6 项单测、stdin 回归和 mock console ACK 探针通过
- One Euro 滤波 8 项单测通过，覆盖静止抑制、快速斜坡、200ms 重置和末端连续收敛
- mock WebSocket 全链路预热后：retarget 4.3ms、mailbox wait 4.8ms、目标到 ACK 5.18ms
- mock 滤波链路观测开销约 0.012-0.033ms；修正后的 0.0005rad 门限仍待真手复测
- RH56DFX 真机观测：retarget 通常 1-5ms、RS485 通常 4.6-8.0ms、目标到串口 ACK 约 7-39ms，队列最多覆盖一个旧目标
- `SPEED_SET` 500/800/1000 三次运行分别观测到 418.5/336.8/110.6ms settled；动作幅度不同，只能说明提速趋势，不能作为严格横向基准

### Commits (提交)

- `dc562a2` `feat(mimic): add MediaPipe Tasks hand tracker`
- `a48f9af` `refactor(mimic): add tracker engine fallback`
- `253e073` `perf(mimic): add hardware latency diagnostics`
- `806bcb4` `perf(mimic): add latest-target hardware control`

## [2026-08-14] - 文档时效性全面审查

### Changed (变更)

- 明确现行 MCP/Bridge 基准为独立 `robot-mcp-server` 仓库
- 重建文档导航，将现行操作文档、专题参考和历史报告分层
- 修正 MCP 标准入口、弃用 REST 路径、10 个工具、心跳与 hand-only 降级语义
- 更新 WebSocket latest-frame-wins、停止/重连和 MediaPipe Tasks 本地资源说明
- 更新当前项目状态、待办、硬件参数和装配参数

### Security (安全)

- 标记 `ssl/key.pem` 已进入 Git，不得继续作为共享或生产私钥使用
- 标记 `hand_pose.py` 与驱动存在 10 项参数不一致，禁止宣称可行域校验已对齐

### Deprecated (弃用)

- 本仓 `mcp_server/` 和根目录 `bridge.py` 作为拆仓前快照保留
- MCP combo、视觉 mimic 和 `/execute` 文档归档，不代表现行部署能力

### Verification (验证)

- MCP 心跳单测 3 项通过，机械臂 mock 单测 1 项通过
- Web MediaPipe/传输 Node 单测通过
- `hand_pose.py --verify` 与旧迁移脚本仍失败，已列入待办

## [2026-08-14] - 实时摄像头手部追踪 WebSocket 优化

### Added (新增)
- 新增 `GET /ws/hand/mimic` WebSocket 端点，接收 MediaPipe 21 点世界坐标并实时返回 6 个灵巧手关节角度和手势名称
- `sim/web/hand_mimic.js` 优先使用 WebSocket，并保留 `/api/hand/mimic` HTTP 降级路径
- 前端增加连接超时、断线重连和统一响应处理

### Verified (验证)
- 在 `lerobot` Conda 环境完成真实 WebSocket 握手和消息往返测试
- 21 点数据成功经过 dex-retargeting，返回 6 个 `right_*_joint` 关节角度
- 测试姿态成功识别为“张开手”；非法点数能返回协议错误
- Python、JavaScript 语法检查及相关代码文件的 `git diff --check` 通过

### Known Issues (已知问题)

> 以下是该阶段当时的已知问题，已由本页顶部的 latest-target 改造解决。

- WebSocket 发送尚无单帧在途/背压控制；后端处理低于摄像头帧率时可能积压旧帧
- `stop()` 主动关闭连接后，`onclose` 仍可能安排重连
- 浏览器端实际 FPS 和端到端延迟仍需在摄像头页面实测

---

## [2026-08-13] - 摄像头手势控制与回放优化

### Added (新增)
- 增加 MediaPipe 实时摄像头手势识别和完整手势控制流程
- 回放页面支持实时摄像头控制及 3D 懒加载
- 增加动态包扫描能力和 WSL 摄像头测试指南

### Changed (变更)
- 更新 retargeting 配置以适配厂商新 URDF 关节名
- 优化实时摄像头重定向和处理性能

### Fixed (修复)
- 修复骨骼点坐标计算错误
- 修复 3D 加载导致视频消失的问题
- 修复 `app_web` 中 `logger` 未定义导致的 HTTP 500 错误
- 隐藏回放页面暂不使用的关节时序图

---

## [2026-08-10] - 灵巧手URDF迁移 + Assets重构

### Added (新增)
- 采用厂商2025-04-18新URDF作为项目标准 (`assets/hand/urdf/inspire_hand_right.urdf`)
- 新的assets目录结构：`hand/`, `arm/`, `assembled/`, `viz/`
- 路径集中管理：当时由 `sim/paths.py` 管理，现为 `src/paths.py`
- 一次性迁移工具与报告后来合并到 `MIGRATION_2026_08_10.md`，工具已删除

### Changed (变更)
- **关节命名**（6个驱动关节）：
  - `thumb_proximal_yaw_joint` → `right_thumb_1_joint`
  - `thumb_proximal_pitch_joint` → `right_thumb_2_joint`
  - `index_proximal_joint` → `right_index_1_joint`
  - `middle_proximal_joint` → `right_middle_1_joint`
  - `ring_proximal_joint` → `right_ring_1_joint`
  - `pinky_proximal_joint` → `right_little_1_joint`

- **限位值**（SolidWorks导出值）：
  - thumb_1 (yaw): 1.308 → 1.246165 rad (-4.7%)
  - thumb_2 (pitch): 0.6 → 0.48 rad (-20%)
  - 四指 (MCP): 1.47 → 1.333 rad (-9.3%)

- **代码更新**：批量更新9个文件，50处关节名替换
  - `sim/inspire_hand.py` - 核心驱动配置
  - `sim/schema.py`, `sim/ros_joint_writer.py`
  - `sim/hand_rerun.py`, `sim/live_rerun.py`
  - `sim/build_inspire_from_vendor.py`
  - `sim/skills/backend.py`, `sim/skills/hand_pose.py`
  - 测试文件

- **目录重组**：
  - `assets/inspire_hand/` → 合并到 `assets/hand/`
  - `assets/nero_description/` → 移动到 `assets/arm/`
  - 新增 `assets/assembled/` - 装配体URDF
  - 新增 `assets/viz/` - 浏览器可视化产物

### Fixed (修复)
- 修复 `build_combo_viz.py` 支持新目录结构
- 更新web combo viz URDF使用新灵巧手关节名
- 法兰与灵巧手装配偏移修正（坐标系校准）
  - `MOUNT_RPY`: `"0 0 1.570796"` → `"-1.570790 -0.000000 -1.570799"`
- 修复console慢一个命令的bug（`sim/stdin_lines.py`）

### Removed (移除)
- `assets/urdf_right/` - 与新URDF重复（6MB）
- `sim/assets/inspire_hand_viz.urdf` - 重新生成

### Deprecated (弃用)
- 旧URDF备份到 `assets/hand_legacy/` 和 `assets/arm_legacy/`

---

## [2026-08-07] - 灵巧手URDF切换（dex-urdf → 官方包）

### Changed
- 手部URDF从 dex-urdf 版本切换到官方 `urdf_right_2025_4_18`
- 坐标系约定变化：base→hand_base_link joint origin 从 `rpy="-1.57079 0 3.14159"` 改为 `rpy="0 0 0"`

### Issues
- 导致法兰与手装配偏移（"圆心偏离"），2026-08-10已修复

---

## [2026-08-04] - 适配法兰装配验证

### Added
- 适配法兰link：`rh56df_adapter_flange`
- 装配体反解：从 nero_RH56DF.stl 反解出法兰→手的安装变换
- ICP验证：残差0.36mm

### Changed
- 臂URDF joint8原值 x=0.032 改为 0.031（官方xacro值）

---

## [2026-07-31] - 环境路径鲁棒化 + RH56 datasheet落地

### Added
- RH56官方手册 V1.09
- 关节角对应表：0-1000对应关系.xls
- 确认 `ANGLE_ACT = 1546` 对 RH56DFX 成立

### Changed
- 从12处硬编码路径改为一处真源（早期版本的路径管理）

---

## [2026-07-29 ~ 2026-07-30] - 握拳拆分 + 参数透传

### Added
- 握拳技能拆成原子阶段
- 技能26 → 28，测试255 → 263

### Fixed
- 修复握拳过程中拇指-食指互顶问题

---

## [2026-07-27] - 坐标系拆分与腕部运动基

### Added
- 腕部坐标系拆分
- wrist motion basis 改成矩阵表示
- world/body 轴向结论

---

## [2026-07-24] - 坐标系调试

### Added
- 坐标系调试入口和工具
- RGB-D 融合手姿势验证

---

## [2026-07-23] - Canonical 手部估计器适配

### Added
- 新增资料浏览结果
- Canonical 手部估计器适配层
- 视频手腕到 NERO 末端姿态映射

---

## 早期版本

更早期的变更记录见 `/home/zhang123/ros2_ws/更新日志.md`

---

## 维护说明

### 变更类型

- **Added**: 新增功能
- **Changed**: 功能变更
- **Deprecated**: 即将移除的功能
- **Removed**: 已移除的功能
- **Fixed**: Bug修复
- **Security**: 安全相关

### 提交规范

提交信息格式：`<type>(<scope>): <subject>`

类型：
- `feat`: 新功能
- `fix`: Bug修复
- `refactor`: 重构
- `docs`: 文档更新
- `style`: 代码格式
- `test`: 测试相关
- `chore`: 构建/工具链
