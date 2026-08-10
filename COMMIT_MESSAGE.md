# Git提交信息建议

## 联合提交：灵巧手URDF迁移 + Assets重构

```
feat: 灵巧手URDF迁移 + Assets目录标准化重构

真机到位后的重大更新，完成两项系统性改造：

## 1. 灵巧手URDF迁移（2025-04-18新版）

采用厂商SolidWorks导出的最新URDF作为项目标准。

### 关节命名变更（6个驱动关节）
- thumb_proximal_yaw_joint   → right_thumb_1_joint
- thumb_proximal_pitch_joint → right_thumb_2_joint  
- index_proximal_joint       → right_index_1_joint
- middle_proximal_joint      → right_middle_1_joint
- ring_proximal_joint        → right_ring_1_joint
- pinky_proximal_joint       → right_little_1_joint

### 限位值同步（SolidWorks导出值）
- thumb_1 (yaw):   [0, 1.246165] rad
- thumb_2 (pitch): [0, 0.48] rad  (-20%)
- 四指 (MCP):      [0, 1.333] rad (-9.3%)

### 代码更新
- sim/inspire_hand.py: 核心驱动配置（HAND_JOINTS/LIMITS/RAW_MAP）
- 批量更新9个文件，50处关节名替换：
  * schema.py, ros_joint_writer.py
  * hand_rerun.py, live_rerun.py  
  * build_inspire_from_vendor.py
  * skills/backend.py, skills/hand_pose.py
  * 测试文件

## 2. Assets目录标准化重构

### 新目录结构
```
assets/
├── hand/           - 手部URDF+mesh（STL，新关节名）
├── arm/            - 臂部URDF+mesh
├── assembled/      - 装配体URDF（pinocchio/MuJoCo用）
├── viz/            - 浏览器可视化产物（glb+相对路径URDF）
│   ├── arm/
│   ├── hand/
│   └── combo/
├── hand_legacy/    - 旧版备份
└── arm_legacy/     - 旧版备份
```

### 路径集中管理
- 新增 sim/paths.py: 所有路径的唯一真源
- 移除硬编码路径，clone到任何位置都能用

### 删除冗余
- assets/inspire_hand/   → 合并到 assets/hand/
- assets/urdf_right/     → 已删除（6MB重复）
- assets/nero_description/ → 移到 assets/arm/

### 可视化构建脚本更新
- build_hand_viz.py, build_arm_viz.py, build_combo_viz.py
- 支持新的标准化目录结构

## 兼容性影响

### 已录制手势包
限位收紧后，超限帧会被自动夹到新上限：
- 拇指pitch动作可能变弱（-20%）
- 四指抓握可能减弱（-9.3%）

**解决方案**: 重新录制或调高force参数补偿

### 关键技术点保持不变
- ✅ 厂商通道映射 PROJECT_TO_VENDOR = [5,4,3,2,1,0]
- ✅ 方向配置（所有通道 invert=True）
- ✅ RS485通信协议

## 文档
- MIGRATION_2026_08_10.md: 完整迁移记录
- ASSET_MIGRATION_PLAN.md: 目录重构方案
- MIGRATION_README.md: 快速参考
- QUICKSTART.md: 快速启动指南

## 工具
- migrate_hand_joints.py: 关节名自动迁移
- migrate_gesture_packs.py: 手势包路径迁移
- verify_migration.py: 自动验证
- final_summary.py: 迁移总结

---
变更统计: 232个删除，20+个新增
新URDF来源: 厂商urdf_right_2025_4_18 (SolidWorks导出)
迁移日期: 2026-08-10
```
