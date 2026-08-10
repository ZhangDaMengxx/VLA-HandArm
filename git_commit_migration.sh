#!/bin/bash
# git_commit_migration.sh — 提交灵巧手URDF迁移的Git建议脚本

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║      灵巧手URDF迁移 - Git提交建议                           ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

echo "📋 建议的Git提交流程："
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "1. 查看变更"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "git status"
echo "git diff sim/inspire_hand.py"
echo "git diff sim/"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "2. 添加新URDF文件（分步提交以便回滚）"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "# 提交1：备份旧URDF"
echo "git add assets/inspire_hand_legacy/"
echo "git commit -m 'backup: 备份旧版inspire_hand URDF到inspire_hand_legacy'"
echo ""
echo "# 提交2：添加新URDF"
echo "git add assets/inspire_hand/"
echo "git commit -m 'feat: 添加厂商2025-04-18新版inspire_hand URDF作为标准'"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "3. 提交代码变更"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "# 提交3：更新核心驱动"
echo "git add sim/inspire_hand.py"
echo "git commit -m 'refactor: 更新inspire_hand.py关节名和限位配置

- 采用新URDF关节命名（right_thumb_*_joint等）
- 同步新URDF限位值（SolidWorks导出）
- 更新RAW_MAP span值
- 添加详细注释说明限位变更原因
'"
echo ""

echo "# 提交4：批量更新其他代码"
echo "git add sim/schema.py sim/ros_joint_writer.py sim/hand_rerun.py sim/live_rerun.py"
echo "git add sim/build_inspire_from_vendor.py sim/skills/ sim/test_*.py"
echo "git commit -m 'refactor: 批量更新灵巧手关节名（9个文件，50处）

使用migrate_hand_joints.py自动迁移脚本完成。

变更：
- thumb_proximal_*_joint → right_thumb_*_joint
- index/middle/ring_proximal_joint → right_*/little_1_joint

受影响文件：
- sim/schema.py
- sim/ros_joint_writer.py
- sim/hand_rerun.py, sim/live_rerun.py
- sim/build_inspire_from_vendor.py
- sim/skills/backend.py, sim/skills/hand_pose.py
- sim/skills/test_hand_pose.py
- sim/test_combo_page.py
'"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "4. 提交文档和工具"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "# 提交5：更新文档"
echo "git add handarm_notes.md"
echo "git commit -m 'docs: 更新handarm_notes.md灵巧手URDF章节

- 更新标准URDF路径为新版本
- 更新关节顺序说明（新关节名）
- 添加迁移说明和关节名对照表
'"
echo ""

echo "# 提交6：添加迁移文档和工具"
echo "git add MIGRATION_2026_08_10.md MIGRATION_README.md"
echo "git add migrate_hand_joints.py verify_migration.py final_summary.py"
echo "git commit -m 'docs: 添加灵巧手URDF迁移文档和工具

- MIGRATION_2026_08_10.md：完整迁移记录
- MIGRATION_README.md：快速参考指南
- migrate_hand_joints.py：自动迁移工具
- verify_migration.py：自动验证脚本
- final_summary.py：迁移总结报告
'"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "5. 推送到远程"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "git push origin main"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "备选：单个大提交（不推荐，难以回滚）"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "git add ."
echo "git commit -m 'feat: 灵巧手URDF迁移到厂商2025-04-18新版本

真机已到位，采用厂商SolidWorks导出的最新URDF作为项目标准。

变更总结：
- 关节命名：旧命名（thumb_proximal_*）→ 新命名（right_thumb_*等）
- 限位收紧：thumb_pitch 20%，四指 9.3%
- 代码更新：9个文件，50处替换
- 旧版本备份：assets/inspire_hand_legacy/

详见：MIGRATION_2026_08_10.md
'"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "⚠️  注意事项"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "1. 建议采用分步提交（便于回滚）"
echo "2. 提交前运行 python3 verify_migration.py 确认无误"
echo "3. assets/inspire_hand/ 目录较大（meshes），首次push可能较慢"
echo "4. 考虑使用 Git LFS 管理 .STL 文件（如果仓库有配置）"
echo ""
