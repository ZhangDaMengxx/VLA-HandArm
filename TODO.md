# 待办事项

本页只保留尚未完成、能够执行的事项。完成情况和背景见
[PROJECT_STATUS.md](PROJECT_STATUS.md)。

## P0 安全与一致性

- [ ] 统一 `src/inspire_hand.py` 与 `src/skills/hand_pose.py` 的手指 span/limit
  - 当前 `python3 src/skills/hand_pose.py --verify` 报 10 项不一致
  - 先确认真机安全范围、URDF 表达和已录动作影响，再修改代码
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
- [ ] 真机复测滤波后的张手末端与静止手势
  - 静止阶段不得再出现 `raw_delta` 接近 0、`filtered_delta` 突跳约 0.02rad
  - 记录 `perf-hand/filter` 与 `perf-hand/tracking`，确认无可见台阶且跟随延迟可接受
- [ ] 浏览器摄像头实测 FPS、端到端延迟和断线恢复
- [ ] 覆盖真实摄像头权限拒绝、设备断开、Chrome/Edge 和页面切换恢复
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

- [ ] 新增 `datasets/captures/` 正式 Capture Bundle 根目录，正式采集数据不再直接写入 `src/out/`
- [ ] 为每次采集建立 `capture_<date>_<seq>_<uuid>/`，包含 `source/`、`ego/`、`robot_datasets/`、`lineage/` 和 `reports/`
- [ ] 建立 Source 原始层：保留原生 RGB-D 录制、原始/对齐深度、硬件微秒时间戳、标定快照、校验和及留存策略
- [ ] 将 Ego 输出迁移到 `<capture>/ego/`，确保可直接由 `LeRobotDataset()` 加载
- [ ] 将机器人派生输出迁移到 `<capture>/robot_datasets/<target>/<asset_revision>/<retarget_revision>/`
- [ ] 增加 Capture manifest、LeRobot v3 结构、checksum、血缘和 QA 校验器
- [ ] 统一并版本化四元数顺序：当前代码为 `xyzw`，新规范为 `qwxyz`，禁止静默切换
- [ ] 明确时间契约：LeRobot `timestamp` 使用秒；Source 保留 `timestamp_hw_us`，报告同步残差毫秒
- [ ] 区分 `episode0_camera` 与 `scene_world` 坐标系，并在 `coordinate_system.json` 中声明
- [ ] 将 RGB-D 帧率、分辨率和质量阈值配置为可版本化的 quality profile，按实际设备同步模式验收
- [ ] 区分手腕绝对精度（需要真值）与无真值条件下的抖动、连续性和骨长稳定性
- [ ] 建立 Python 3.12.x + `lerobot[dataset]==0.6.1` 隔离环境，完成 LeRobot v3 兼容性验证
- [ ] 新流程稳定后再淘汰 `src/out/` 中的旧实验输出

---

**最后整理**：2026-08-19
