# 更新日志 (CHANGELOG)

项目的所有重要变更都记录在这里。

格式基于 [Keep a Changelog](https://keepachangelog.com/)，版本号遵循日期格式。

---

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
- 路径集中管理：`sim/paths.py` 作为唯一真源
- 迁移工具：`migrate_hand_joints.py`, `verify_migration.py`, `final_summary.py`
- 迁移文档：`MIGRATION_2026_08_10.md`, `MIGRATION_README.md`, `QUICKSTART.md`
- 装配成功报告：`ASSEMBLY_SUCCESS.md`
- Web更新报告：`WEB_COMBO_UPDATE.md`

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
