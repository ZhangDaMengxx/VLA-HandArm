# 待办事项

本页只保留尚未完成、能够执行的事项。完成情况和背景见
[PROJECT_STATUS.md](PROJECT_STATUS.md)。

## P0 安全与一致性

- [ ] 统一 `src/inspire_hand.py` 与 `src/skills/hand_pose.py` 的手指 span/limit
  - 当前 `python3 src/skills/hand_pose.py --verify` 报 10 项不一致
  - [x] 2026-08-21 完成第一阶段离线审计：参数来源、Bridge 执行路径、12 个语义
    姿态和 8 个动作包影响已记录到 `src/HAND_LIMIT_AUDIT_2026_08_21.md`
  - [x] 将资产标称 rad、raw 安全包络和可选残差标定拆成独立契约；新增通用型号
    规范、Mock/RH56 Adapter、自动探测 Profile 和安全投影（2026-08-21）
  - [x] 修正旧碰撞扫描脚本顶部 speed=50 与实际 `SCAN_SPEED=15` 的说明冲突
  - [x] 2026-08-21 完成真机只读 `preflight`：5/5 遥测有效、错误位和电流均为 0、
    最高温度 38℃；未写速度/力/角度，报告位于本地 `reports/hand_feasibility/`
  - [ ] 用户明确批准并确认现场安全后，按 `single -> interactions` 分阶段低速验证；
    旧 T5/T6 仅作历史对照，统一 rad 取资产标称值，真机只声明条件化 raw 包络
  - [ ] 参数定案后统一手势、Bridge、Web、ROS writer、动作包和碰撞检查，再做低速回归
  - 同步独立仓库 `robot-mcp-server/robot-bridge/sim/`
- [ ] 轮换已经进入 Git 的 `ssl/key.pem`
  - 停止把现有私钥用于共享或生产环境
  - 将私钥移出跟踪并加入忽略规则
  - 单独评估是否需要清理 Git 历史及通知所有使用者
## P0 真机验证

- [ ] 核验灵巧手连接、只读反馈、单关节低速运动和力控范围
- [ ] 核验机械臂固件自动探测、CAN、使能、低速单关节运动、急停和复位
- [ ] 核验机械臂不可用时 Bridge 的 hand-only 降级
- [ ] 核验 MCP 心跳断线、恢复、健康状态和运动命令不自动重试
- [ ] 记录每项真机验证的日期、硬件版本、固件和初始条件
- [x] 验证摄像头 retarget → latest-target → ACK → RS485 真机控制链
  - 目标到串口 ACK 约 7-39ms，无无界积压，峰值覆盖 1 个待发旧目标

## P1 Web 与视觉

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
- [ ] 增加服务端硬件会话 heartbeat/lease watchdog 与多标签页所有权
  - 页面正常切换继续等待复位和断开；浏览器异常消失时由租约超时强制释放串口/CAN
  - 覆盖进程强杀、断网、刷新、重复标签页和释放请求重复到达
- [ ] 验证真手 + Mock 臂的联合锚定、丢手冻结和重新锚定
- [ ] 验证真臂 + Mock 手的显式授权、使能、急停、限幅和连续 IK 失败冻结
- [ ] 在清空工作区和低速条件下验证双真机合体跟随，记录初始关节、固件和急停条件
- [ ] 用对齐 RGB-D 深度替换 `monocular_scale` 腕部位置，并保留同一锚定/协议契约
- [ ] 用相同固定角度阶跃各重复 3 次，对照 `SPEED_SET=500/800/1000` 的 settled、力、电流和温度
- [ ] 验证局域网可信 HTTPS；不得继续使用已提交的旧私钥

## P1 ROS2 与数据

- [ ] 审查独立 ROS2 仓库是否仍有旧灵巧手关节名
- [ ] 验证 `ros2_control`、JointTrajectoryController 和 RViz2
- [ ] 完成灵巧手路径碰撞检查，并接入 retargeting 约束
- [ ] 按确认后的限位重新评估或录制手势包
- [ ] 将保留轨迹 NPZ 重导出为当前 6 驱动关节命名，并重新跑安全闸测试

## P2 后续能力

- [ ] 完善 VLA 数据集验证和训练流程
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
  - `.envs/lerobot-v3` 已固定 Python 3.12.13、CPU Torch 2.7.1、LeRobot 0.6.1 和 TorchCodec 0.4.0，`pip check` 通过
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

---

**最后整理**：2026-08-21
