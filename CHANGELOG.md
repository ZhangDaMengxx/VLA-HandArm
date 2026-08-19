# 更新日志 (CHANGELOG)

项目的所有重要变更都记录在这里。

格式基于 [Keep a Changelog](https://keepachangelog.com/)，版本号遵循日期格式。

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

### Safety (安全)

- 联合锚点使用机械臂当前关节 FK，避免开始跟随时跳到预设位姿
- 单目位置只标记为 `monocular_scale` 相对量；真臂位置限幅收紧为各轴 `±20mm`，末端姿态只允许锚点附近的有限旋转
- 丢手、左右手变化、连续 3 次 IK 失败、急停/冻结、未使能和断线停止投递并冻结跟随
- 本次未执行真实机械臂运动；真机验证继续列在 `TODO.md`
- 摄像头姿态切换前先清空机械臂 latest-target、退出 CPV，再下发 `move_j`；真机仍要求在线、已使能、未急停和显式授权，并在每次启动时确认准备位与关闭回伸直位的无碰撞路径
- 灵巧手回张开命令会等待正在下发的实时帧结束，避免关闭摄像头时的末帧覆盖安全张开位

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
