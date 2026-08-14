# 待办事项

本页只保留尚未完成、能够执行的事项。完成情况和背景见
[PROJECT_STATUS.md](PROJECT_STATUS.md)。

## P0 安全与一致性

- [ ] 统一 `sim/inspire_hand.py` 与 `sim/skills/hand_pose.py` 的手指 span/limit
  - 当前 `python3 sim/skills/hand_pose.py --verify` 报 10 项不一致
  - 先确认真机安全范围、URDF 表达和已录动作影响，再修改代码
  - 同步独立仓库 `robot-mcp-server/robot-bridge/sim/`
- [ ] 轮换已经进入 Git 的 `ssl/key.pem`
  - 停止把现有私钥用于共享或生产环境
  - 将私钥移出跟踪并加入忽略规则
  - 单独评估是否需要清理 Git 历史及通知所有使用者
- [ ] 修复迁移验收脚本
  - `verify_migration.py` 仍检查 `assets/inspire_hand/`，当前路径是 `assets/hand/`
  - `final_summary.py` 仍输出旧路径且在错误状态下宣称完成
  - 验收工具应在失败时返回非零退出码

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
- [ ] 真机复测滤波后的张手末端与静止手势
  - 静止阶段不得再出现 `raw_delta` 接近 0、`filtered_delta` 突跳约 0.02rad
  - 记录 `perf-hand/filter` 与 `perf-hand/tracking`，确认无可见台阶且跟随延迟可接受
- [ ] 浏览器摄像头实测 FPS、端到端延迟和断线恢复
- [ ] 用相同固定角度阶跃各重复 3 次，对照 `SPEED_SET=500/800/1000` 的 settled、力、电流和温度
- [ ] 验证局域网可信 HTTPS；不得继续使用已提交的旧私钥

## P1 ROS2 与数据

- [ ] 审查独立 ROS2 仓库是否仍有旧灵巧手关节名
- [ ] 验证 `ros2_control`、JointTrajectoryController 和 RViz2
- [ ] 完成灵巧手路径碰撞检查，并接入 retargeting 约束
- [ ] 按确认后的限位重新评估或录制手势包

## P2 后续能力

- [ ] 完善 VLA 数据集验证和训练流程
- [ ] 增加仿真碰撞与场景测试
- [ ] 为 Web 7860 端口增加鉴权或可信反向代理边界
- [ ] 如果需要 MCP combo，先形成新的接口、安全和执行语义设计，再在独立仓库实现

---

**最后整理**：2026-08-14
