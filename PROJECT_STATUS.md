# 项目状态

**核对日期**：2026-08-25
**阶段**：Ego Capture、严格 LeRobot v3 与结构 QA 已完成，设备 Source/物理 QA 与真机安全验证待开展

## 当前结论

| 模块 | 状态 | 说明 |
|------|------|------|
| 硬件驱动 | 开发中 | 臂/手驱动已具备，仍需系统真机验收 |
| Web 手部追踪 | 真机主链已通 | 本机能力检测、GPU/CPU/Apple GPU 清单、Legacy 自动降级、21点 retarget、One Euro 滤波和真手驱动已接入 |
| Web 传输 | 真机验证通过 | 前端单帧在途、后端 latest-target、真实 ACK、停止清理已实现 |
| 合体视频跟随 | Mock 已验收 | 联合锚定、腕姿映射和 7+6 目标已闭环；手与 latest-only IK 异步解耦，真臂未验证 |
| 相机与标定 | 工具已建/真机待验 | eye-in-hand 交互工具已具备；Orbbec 336 原生 SDK Adapter、设备内参与物理手眼结果待相机到货后验证 |
| VLA 数据管线 | Capture/严格 v3/结构 QA 已验收 | 全链绑定同一 Capture，坐标和质量口径固化；Python 3.12 + LeRobot 0.6.1、episode sidecar 与 Capture 完整性校验通过，设备 Source 与物理 QA 证据待接入 |
| MCP/Bridge | 已拆仓 | 现行代码在 `/home/zhang123/ros2_ws/robot-mcp-server` |
| ROS2 | 待复核 | 独立仓库的关节命名和真机控制仍需验证 |
| 运行环境 | 主链已统一 | `lerobot-v3` 承接 Web/视觉/IK/数据/直接硬件；ROS Humble 自动分流到 Python 3.10 薄环境 |
| 目录与文档 | 已归整 | 源码迁到 `src/`，测试集中到 `src/test/`，第三方资产集中到 `third_party/` |

## MCP 现行基准

权威仓库：`/home/zhang123/ros2_ws/robot-mcp-server`，本次核对提交为
`f4e1c7e`（`feat: add bridge heartbeat monitoring`）。

现行能力：

- 标准 MCP：JSON-RPC `POST /mcp`，并支持相应 GET/DELETE 会话语义
- 兼容 REST：`/mcp_rest/tools/list`、`/mcp_rest/tools/call`，已弃用
- HTTP API：`/api/v1/hand/*`、`/api/v1/arm/*`
- 10 个 MCP 工具：4 个灵巧手工具、6 个机械臂工具
- Bridge 心跳、断线状态、恢复检测和降级健康检查
- 灵巧手真机连接失败时 Bridge 启动失败；机械臂失败时可 hand-only 运行

不属于现行 MCP 的能力：combo、视觉 mimic 和 Bridge `/execute`。这些只存在于本仓
内嵌实验快照或 Web 工作台，不应出现在部署能力清单中。

## 最近完成

### 2026-08-25

- Web 硬件控制新增每标签页 owner、2 秒 heartbeat 和服务端 8 秒租约 watchdog；第二标签页
  不能抢占有效 owner，超时复用手复位、臂回零并断开的统一释放路径
- 租约、重复释放和并发释放单测及页面静态回归通过；本轮未连接真机，进程强杀、断网和
  超时释放的双真机现场行为仍待低速净空验证

### 2026-08-24

- 将 `lerobot-v3` 提升为主运行时，补齐 FastAPI/Uvicorn、MuJoCo/MeshCat、CAN 和串口依赖；
  新增 `ros-humble` Python 3.10 薄环境与自动探测/加载 helper，Web API、WebSocket、13 轴
  关节顺序和硬件控制类保持不变
- 新增 `src/lerobot_v3/app_web.py` 统一入口；实时 IK/live Rerun 改走 V3，只有 rclpy
  reader/writer/runner 在后台加载 Humble。ROS 硬件桥的 Mock/只读/控制模式仍显式选择
- 新增 `src/camera/`：只读 NERO 关节角并以 URDF FK 配对棋盘格位姿，支持交互式
  `next/finish`、静止闸、退化检测、样本图像留存、误差报告和 `T_gripper_camera`
  4x4/`xyzw` 输出；合成真值及现有 NERO FK 定向回归通过，尚未连接真实相机或机械臂
- 将本仓手势安全表、ROS writer 和 URDF 生成覆盖统一到正式资产/驱动的
  `thumb_pitch=0.48`、四指 `1.333`，`hand_pose.py --verify` 通过
- 修复 `limit` 派生状态使用旧关节名而退化为满弯 `raw 0` 的问题；`hand_pinch` 和
  `hand_ok` 恢复为可行域下界加 `LIMIT_MARGIN`，当前食指目标为 `raw 235`
- 新增驱动、手势表、ROS writer、生成覆盖和正式 URDF 的一致性回归；相关离线测试
  `85 passed`，手势规格自检 `78/78` 通过；独立 Bridge 同步后 15 项单测通过
- 完成 RH56DFX 新真机单关节 Profile：`speed=15`、`force=250`，60/60 点均为
  `feasible`，六关节均达到 `safe_max_u=1.0/raw=0`；峰值温度 52℃、峰值力绝对值 84，
  全程错误位为零，postflight 回到张开侧。端点反馈为拇指 `0/0`、四指
  `48/59/55/60 raw`，其中小指正好位于 `60 raw` 跟踪容差边界
- 上述 Profile 只闭环空载单关节自由行程；thumb-index interaction、自碰撞边界和
  Web/Bridge 条件化安全投影仍待完成

### 2026-08-21

- 完成灵巧手限位第一阶段离线审计：当时确认运行驱动/标准资产使用 `0.48/1.333`，
  手势安全表仍使用旧 span/limit，Bridge 因而以旧映射检查、再以新映射下发；
  该漂移已于 2026-08-24 修复
- 量化 12 个语义姿态和 8 个录制动作包的映射影响；确认 10 个语义姿态实际 raw
  发生变化，录制包原样回放通常保持 raw，但 rad/3D 解释和重新保存存在兼容风险
- 形成 raw 物理标定、安全命令包络和模型限位三层契约建议及分阶段真机准入方案；
  本阶段未连接串口、未下发动作、未修改限位
- 按后续决策将统一角定义为 datasheet/URDF 资产标称 rad，不要求逐台人工量角；新增
  `hand_feasibility.py`、RH56/Mock Adapter、型号 JSON、原子可恢复 Profile 和安全投影
- 通用探测器覆盖只读预检、单关节保守扩张、声明式低维 interaction、边界二分以及
  温度/错误/电流/连续缺样硬中止；Mock Profile 默认禁止用于真实运行时
- 完整 CLI Mock 和投影验收通过，相关新旧手部链定向回归 `37 passed`；该离线回归未连接真机
- 完成 RH56DFX 真机只读 preflight：稳定路径为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，
  5/5 遥测有效，六通道 `ERROR=0`、`CURRENT=0`、最高温度 38℃；本阶段没有写速度、力或角度
- 首次真机单关节扫描完成前 5 个关节后，在小指回张开跟踪误差 `73 raw` 时 fail-closed
  中止并软冻结；未进入联合阶段。诊断确认低速执行器尚在运动时被旧稳定判据提前结算，
  已改为进入目标容差后才判到位、按命令行程动态延长超时，修复后离线回归 `39 passed`
- 审查并收口 Ego Capture、严格 LeRobot v3、异步合体跟随和视频动作时间轴的开发基线
- 将 451 帧“拿螺丝刀”视频动作包纳入版本库，移除早期 645 帧链路样本，并修正测试与现行文档中的旧文件名
- Python 编译、三个 Web Node 测试和 `40` 项动作/异步链路定向回归通过
- 隔离已记录的 P0 限位漂移、旧夹具和测试基础设施阻断后，离线宽回归 `191 passed`；未执行真机运动
- 21 帧真实关键点 Mock 冒烟完成联合锚定、6+7 轴目标、异步 IK 和统一释放；已用纯快进方式同步 `main` 到常驻 `deploy` 分支

### 2026-08-20

- 合体摄像头跟随默认灵巧手速度保持 `1000`，机械臂速度改为 `50%`；独立臂调试仍默认 `20%`
- 灵巧手目标与机械臂 IK 完成异步解耦：单 IK worker、一个 pending、时效丢弃和 source frame 关联，慢 IK 不再阻塞手目标入队
- 重锚、冻结、丢手、授权变化、异常或断线会废弃旧 IK；自动测试覆盖 100ms 慢 IK、覆盖、过期和求解途中关闭
- 新增集中式 `capture_bundle.py`，默认按日期、序号和 UUID 原子创建 Capture Bundle
- 三个 Canonical 构建器、机器人派生、验收、验证、Rerun 回放和 Web 管线全部改用同一 Capture 路径契约
- Ego 与 RobotDataset 成为两个独立数据集根；轨迹移入 RobotDataset `exports/workbench/`，报告移入 Capture `reports/`
- 增加 `bundle.json`、Source/Ego/Robot 元数据、SHA-256 校验表和 Source -> Ego -> RobotDataset 基础血缘
- Capture 新建时为 `building`，Ego 完整落盘后才标记 `ready`；默认读取跳过 `building/failed`，显式路径仍可诊断或重建
- Source 层保留原视频、处理结果或已对齐 RGB-D，使用 Parquet 记录原文件名、Source/Ego 帧映射、硬件/派生时间来源和同步残差
- 旧 `src/out/` 改为仅显式 `--legacy-out`/`VLA_LEGACY_OUT=1` 兼容，不自动移动或删除
- Gradio、腕部分析和轨迹分析工具改为默认跟随 Capture；混用不同 Capture 的显式根会被拒绝
- `coordinate_system.json` 升为 2.0：三类构建器逐字段声明坐标 frame，派生、验证、验收、分析和 Rerun 显式读取，不再按目录或来源类型推断
- 新增 4 个版本化 quality profile；每个 Capture 固化不可变快照，验收按实际帧率、分辨率、时间戳和同步能力判定
- 将 LeRobot 内部帧间隔一致性与 Source 硬件同步拆开；旧 Kinect 无硬件时间时不再可能误报同步通过
- quality profile 升至 schema 1.1；手腕绝对精度、静止抖动、位姿连续性和骨长稳定性分项报告，缺真值不再以代理结果代替
- 用现有数据验证 Ego 780 帧和 RobotDataset 557 帧均可回读；新旧路径数值列最大绝对差为 `0`
- 将命名环境 `lerobot-v3` 扩充为 Python 3.12.13 + LeRobot 0.6.1 完整离线链；MediaPipe、Pinocchio、dex-retargeting、Rerun 和网格加载实测通过，`pip check` 无冲突
- 当日先完成离线 V3 与实时 Python 3.10 分流；该过渡架构已在 2026-08-24 进一步收口为
  V3 主运行时 + 仅 rclpy 使用的 Python 3.10 薄环境
- 增加严格 v3 校验器和 Capture 根级生成环境快照；新 3 帧完整 Capture 通过官方加载及 strict-v3，五个 processed 核心字段最大差为 `0`
- Ego/RobotDataset 增加不覆盖人工审核的 episode annotation；Robot QA 自动检查索引和有限值，物理项缺证据时保持 `not_evaluated`
- 增加 `verify_dataset.py --capture-bundle`，覆盖 Source、环境、严格 v3、血缘、sidecar 和 SHA-256；四个生成入口显式 finalize 后再写元数据
- 当前 schema 决定继续使用 `xyzw`；外部若要求 `qwxyz`，只允许新 schema 的显式边界转换，不改写既有 Capture

### 2026-08-19

- 将实时视频跟随收敛到“实时 Live · 合体”页，移除灵巧手页重复摄像头入口
- 同时传输 world/image landmarks，分别用于手型/手掌姿态与单目腕部相对位置
- 增加位置和姿态联合锚定；机器人锚点取当前关节 FK，避免锚定时跳到预设位姿；腕部姿态经滤波和限幅后驱动末端有限旋转
- 单一按钮按状态完成锚定、冻结和重新锚定，Mock/真机共用 WebSocket、IK 与 7+6 目标协议
- 增加机械臂 CPV latest-target 路径及真臂显式授权、使能、冻结和在线安全门
- 在腕部相对位置和姿态进入 IK 前分别加入 One Euro 滤波；200ms 采样间隔和跟随生命周期边界会重置滤波状态
- 完成 Mock WebSocket 全链路及 Three.js 桌面/移动端验收，真实机械臂尚未运动验证

### 2026-08-18

- 将原 `sim/` 迁移为 `src/`，同步运行时、部署脚本和现行文档路径
- 将离线测试集中到 `src/test/`，真机测试隔离到 `src/test/hardware/`
- 将第三方源码、厂商 SDK、RGB-D 数据和 overlay 集中到 `third_party/`
- 保留上游内部目录结构，仅 `third_party/overlays/` 作为本项目代码进入 Git
- 修正合体页测试对旧 `src/assets` 生成目录的假设，改用顶层 `assets/`

### 2026-08-14

- 完成文档与本地硬件代码的首轮对齐
- 确认 MCP 独立仓库、分支、远端和最新提交
- 验证 MCP 心跳单测 3 项通过、机械臂 mock 单测 1 项通过
- Web 手势传输增加单帧在途、latest-frame-wins 和生命周期保护
- MediaPipe Tasks 改用本地资源并补充 Node 单测
- 摄像头真机控制改为 30Hz latest-target mailbox，移除逐帧二次 HTTP 硬件请求
- 增加 stdin 写锁、ACK token、单摄像头控制权和分段 `perf-hand` 日志
- RH56DFX 真机确认目标到串口 ACK 约 7-39ms，持续慢感主要来自手本体运动时间
- 真手目标增加六关节 One Euro 滤波；实测发现并移除导致约 0.02rad 台阶抖动的大死区，现使用 0.0005rad 分辨率门限

### 2026-08-10 至 2026-08-13

- 迁移到 2025-04-18 灵巧手 URDF 和新关节命名
- 重组 `assets/` 并生成臂手装配与 Web 模型
- 接通浏览器摄像头、手部关键点、retargeting 和 WebSocket 后端
- 完成 NERO 驱动、控制台、技能和联合动作的一批 mock/离线开发

阶段性成功报告已经归档；它们不再作为当前验收依据。

## 当前风险

### Critical

1. **真机条件化安全包络尚未闭环**

   本仓资产标称映射已对齐，2026-08-24 新 Profile 已完成六关节空载单关节全行程；
   但规范声明的 thumb-index interaction commissioning 尚未执行，Web/Bridge 运行时
   安全投影仍待完成。单关节通过不能替代联合姿态和自碰撞证据。详见
   `src/HAND_LIMIT_AUDIT_2026_08_21.md`。

2. **仓库中存在已跟踪 TLS 私钥**

   `ssl/key.pem` 与证书已进入 Git。该私钥必须轮换，不应继续用于共享或生产场景；
   移出跟踪和历史处理需要单独执行。

### Major

3. Web 摄像头已有真手延迟样本和 GPU/CPU 选择控件，但仍缺 Chrome/Edge/macOS、拒绝权限和断线恢复的完整验收矩阵。
4. 真机运动、急停、复位、限位和 MCP 断线行为尚未形成完整验收记录。
5. One Euro 滤波自动测试和 mock 链路已通过，0.0005rad 门限的张手末端与静止姿态仍需真机复测。
6. 两个保留轨迹 NPZ 使用旧 12 关节名，当前技能后端要求 6 个项目驱动关节名，轨迹安全闸测试不能通过。
7. 合体腕部位置当前是 `monocular_scale` 单目相对估计，姿态也受单目关键点噪声和限幅影响；真手+Mock 臂、
   真臂+Mock 手和双真机低速矩阵尚未验收。
8. 页面内切换可等待硬件复位和断开，但浏览器关闭依赖 `pagehide/sendBeacon` 尽力通知；进程强杀、
   主机断电或断网时没有服务端会话租约兜底，不能保证串口/CAN 一定及时释放。
9. Source 基础留存和索引已接入，但现有旧 Kinect 帧集没有设备原生 RGB-D 容器、未对齐 raw depth
   或硬件时间戳；这些数据只能由后续真实采集链提供，不能从文件名或 FPS 反推。
10. Robot episode QA 的关节限位、碰撞和指尖绝对误差尚无运行证据，当前明确标记为
    `not_evaluated`；结构通过不能替代物理验收。

## 验证记录

已通过：

```bash
node src/test/web/hand_tracker_tasks.test.mjs
node src/test/web/hand_mimic_transport.test.mjs
node src/test/web/combo_camera.test.mjs
/usr/bin/python3 -m pytest src/test/test_combo_page.py src/test/test_hand_target_mailbox.py src/test/test_live_wrist_tracking.py -q
/usr/bin/python3 -m pytest src/test/test_capture_bundle.py src/test/test_compare_dataset_numeric.py -q
conda run -n lerobot-v3 python src/lerobot_v3/verify_dataset.py --capture-root <capture> --canonical --strict-v3
conda run -n lerobot-v3 python src/lerobot_v3/verify_dataset.py --capture-bundle --capture-root <capture> --json <report>

cd /home/zhang123/ros2_ws/robot-mcp-server
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s mcp_server/tests -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s robot-bridge/tests -v
```

当前预期失败：

```bash
/usr/bin/python3 src/test/skills/test_runner_gates.py  # 旧轨迹 NPZ 关节名不兼容
```

2026-08-14 摄像头真机样本中，retarget 通常 1-5ms、RS485 通常 4.6-8.0ms、
目标到串口 ACK 约 7-39ms，`replaced` 为 0，峰值时覆盖 1 个待发旧目标。
500/800/1000 速度分别观测到 418.5/336.8/110.6ms settled，但动作幅度不同，
尚不能视为严格速度对照实验。

同日滤波首轮真机日志确认 `raw_delta=0.0002rad` 时曾释放
`filtered_delta=0.0200rad`，定位为大死区累积后的台阶。现行门限已降至
`0.0005rad`；自动测试通过，等待相同动作真机复测。

2026-08-20 系统 Python 下 Capture/Source、路径、数值及 Web/腕部组合回归为
`70 passed, 1 skipped`；跳过项仅为系统环境没有 pyarrow。实际 lerobot 环境的 3 帧
processed Source -> Ego 构建、时间索引、checksum 和 LeRobotDataset 回读通过。另一路手部/腕部
离线回归此前为 `35 passed`，三个 Web Node 测试通过。坐标契约升级后用同一 3 帧输入重建并
回读成功，新旧 10 个数值列最大绝对差为 `0`。全仓 pytest 仍在收集阶段被上述 Pinocchio、手势表
和旧轨迹 fixture 既有问题阻断；本轮未连接或驱动真机。

quality profile 增量完成后，Capture/数值/Web/腕部相关回归为 `89 passed`，三个 Web Node
测试继续通过；实际 lerobot 环境新建的 profile Capture 为 `ready` 并成功回读 3 帧/1 episode，
接入前后 10 个数值列最大绝对差仍为 `0`。顶层全量回归还存在既有 `SIM` 测试变量和手势 raw
限位断言失败，不属于本次数据质量改动。

绝对精度与代理指标拆分后，定向 quality profile 回归为 `12 passed`，隔离硬件、外部语音服务、
旧轨迹 fixture 和已记录失败后的离线回归为 `169 passed`，三个 Web Node 测试通过。旧 schema 1.0
Capture 仍可生成报告，缺少手腕真值时绝对精度为不可测；两份 3 帧 Ego 的 10 个数值列最大绝对差
仍为 `0`。本轮未连接或驱动真机。

Python 3.12.13 + LeRobot 0.6.1 数据集隔离环境完成后，`pip check` 通过；官方
`LeRobotDataset` 在离线模式下回读旧 Ego 780 帧和 RobotDataset 557 帧。新 3 帧完整 Capture
同时通过官方加载与 `--strict-v3`，五个 processed 输入核心字段最大数值差为 `0`，根级
`environment/` 快照完整。旧 0.4.4 数据仍可回读，但旧式匿名列 `tasks.parquet` 不满足严格
交付校验，数据保持原样。本轮未连接或驱动真机。

最终定向回归为 `38 passed`，隔离既有硬件、外部服务和旧轨迹 fixture 后离线宽回归为
`174 passed`，三个 Web Node 测试通过。宽回归发现并修复 strict-v3 校验器被技能测试同名
`schema` 模块污染的问题；RobotSpec 现仅在 CLI 主流程按需导入。本轮未连接或驱动真机。

episode sidecar 与 Capture QA 接入后，Capture/strict 定向测试为 `26 passed`，相关组合回归为
`50 passed`。Python 3.12 环境重新构建的新 3 帧 Capture 在命令返回时即可通过
`--capture-bundle`、strict-v3 和官方加载；这同时验证了显式 `finalize()` 已消除对进程析构
写 Parquet footer 的依赖。本轮未连接或驱动真机。

大数据内存加固后，Capture/strict-v3/quality/numeric 定向回归为 `41 passed`，隔离既有硬件、
外部服务和旧轨迹 fixture 后最终离线宽回归为 `177 passed`，三个 Web Node 测试通过。

## 下一步

1. 在新单关节 Profile 基础上另行批准并完成 thumb-index interaction commissioning，
   再将完整 Profile 接入 Web/Bridge 条件化安全投影。
2. 轮换和停止跟踪 TLS 私钥。
3. 补齐 Capture Source 原生 RGB-D、raw depth 和硬件时间戳，再接入限位、碰撞与指尖误差的物理 QA 证据。
4. 复测 One Euro 滤波后的张手末端和静止姿态，再完成浏览器兼容矩阵及相同固定角度阶跃速度测试。
5. 复核 ROS2 独立仓库的关节命名和控制链。
6. 按真手+Mock 臂、真臂+Mock 手、双真机低速顺序验收合体跟随，再接入 RGB-D 米制位置。

详细行动项见 [TODO.md](TODO.md)。
