#!/usr/bin/env python3
"""migrate_hand_joints.py — 批量更新代码中的灵巧手关节名（旧URDF→新URDF）

背景：真机到位后，采用厂商2025-04-18新URDF作为项目标准。
此脚本将所有代码中的旧关节名替换为新关节名。

关节名映射（旧→新）：
  thumb_proximal_yaw_joint   → right_thumb_1_joint
  thumb_proximal_pitch_joint → right_thumb_2_joint
  index_proximal_joint       → right_index_1_joint
  middle_proximal_joint      → right_middle_1_joint
  ring_proximal_joint        → right_ring_1_joint
  pinky_proximal_joint       → right_little_1_joint

用法：
  python3 migrate_hand_joints.py --check      # 仅检查，不修改
  python3 migrate_hand_joints.py --apply      # 实际修改文件
"""
import re
import sys
from pathlib import Path

# 关节名映射
JOINT_MAPPING = {
    "thumb_proximal_yaw_joint": "right_thumb_1_joint",
    "thumb_proximal_pitch_joint": "right_thumb_2_joint",
    "index_proximal_joint": "right_index_1_joint",
    "middle_proximal_joint": "right_middle_1_joint",
    "ring_proximal_joint": "right_ring_1_joint",
    "pinky_proximal_joint": "right_little_1_joint",
}

# 需要更新的文件模式（排除inspire_hand.py - 已手动更新）
INCLUDE_PATTERNS = [
    "sim/**/*.py",
    "src/**/*.py",
]

EXCLUDE_PATTERNS = [
    "**/inspire_hand.py",      # 已手动更新
    "**/migrate_hand_joints.py",
    "**/__pycache__/**",
    "**/.*",
]

def should_process(path: Path) -> bool:
    """判断文件是否需要处理"""
    # 排除模式
    for pattern in EXCLUDE_PATTERNS:
        if path.match(pattern):
            return False
    return True

def migrate_file(path: Path, dry_run: bool = True) -> tuple[int, list[str]]:
    """迁移单个文件，返回(替换次数, 修改行列表)"""
    try:
        content = path.read_text(encoding='utf-8')
    except Exception as e:
        print(f"⚠️  读取失败: {path} - {e}")
        return 0, []

    original = content
    changes = []

    # 逐个替换关节名
    for old_name, new_name in JOINT_MAPPING.items():
        # 精确匹配整个单词（避免误替换）
        pattern = r'\b' + re.escape(old_name) + r'\b'
        matches = list(re.finditer(pattern, content))
        if matches:
            content = re.sub(pattern, new_name, content)
            changes.append(f"  {old_name} → {new_name} ({len(matches)}处)")

    if content != original:
        if not dry_run:
            path.write_text(content, encoding='utf-8')
        return len(changes), changes

    return 0, []

def main():
    repo_root = Path(__file__).parent

    # 解析命令行参数
    dry_run = "--apply" not in sys.argv
    mode = "检查模式（不修改文件）" if dry_run else "应用模式（实际修改）"

    print(f"=== 灵巧手关节名迁移工具 ===")
    print(f"模式: {mode}")
    print(f"根目录: {repo_root}\n")

    # 收集所有Python文件
    all_files = []
    for pattern in INCLUDE_PATTERNS:
        all_files.extend(repo_root.glob(pattern))

    files_to_process = [f for f in all_files if should_process(f)]
    print(f"待扫描文件: {len(files_to_process)}个\n")

    # 处理文件
    total_files_changed = 0
    total_replacements = 0

    for path in sorted(files_to_process):
        count, changes = migrate_file(path, dry_run=dry_run)
        if count > 0:
            total_files_changed += 1
            total_replacements += count
            print(f"{'🔄' if not dry_run else '📝'} {path.relative_to(repo_root)}")
            for change in changes:
                print(change)
            print()

    # 总结
    print("=" * 60)
    print(f"总计: {total_files_changed}个文件需要更新，{total_replacements}处替换")

    if dry_run:
        print("\n✅ 检查完成。运行以下命令实际应用修改：")
        print("   python3 migrate_hand_joints.py --apply")
    else:
        print("\n✅ 迁移完成！")
        print("\n后续步骤：")
        print("  1. 验证修改：git diff sim/")
        print("  2. 运行测试：python3 -m pytest sim/test_*.py")
        print("  3. 手动检查配置文件中的关节名引用")

if __name__ == "__main__":
    main()
