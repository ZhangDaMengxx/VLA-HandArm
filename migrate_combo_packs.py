#!/usr/bin/env python3
"""批量更新联合录制包的 joint_order_hand 字段:旧关节名 → 新关节名。

背景:同 gesture_pack 迁移,2026-08-10 换新 URDF 后关节名变了。

用法:
  python3 migrate_combo_packs.py        # 只检查
  python3 migrate_combo_packs.py --apply  # 实际修改
"""
import argparse
import json
from pathlib import Path

REPO = Path(__file__).resolve().parent
COMBOS_ROOT = REPO / "data/combos"

OLD_NAMES = [
    "thumb_proximal_yaw_joint",
    "thumb_proximal_pitch_joint",
    "index_proximal_joint",
    "middle_proximal_joint",
    "ring_proximal_joint",
    "pinky_proximal_joint",
]

NEW_NAMES = [
    "right_thumb_1_joint",
    "right_thumb_2_joint",
    "right_index_1_joint",
    "right_middle_1_joint",
    "right_ring_1_joint",
    "right_little_1_joint",
]


def migrate_pack(path: Path, dry_run: bool = True) -> tuple[bool, str]:
    """迁移一个联合录制包。返回 (是否需要改, 结果消息)。"""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:
        return False, f"读取失败: {e}"

    jo_hand = data.get("joint_order_hand")
    if jo_hand is None:
        return False, "无 joint_order_hand 字段"

    if jo_hand == NEW_NAMES:
        return False, "已经是新关节名"

    if jo_hand != OLD_NAMES:
        return False, f"joint_order_hand 不是预期的旧名,不敢自动改:{jo_hand}"

    # 需要迁移
    if dry_run:
        return True, f"需要更新:{OLD_NAMES[0]} → {NEW_NAMES[0]} ..."

    # 实际修改
    data["joint_order_hand"] = NEW_NAMES
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n",
                    encoding="utf-8")
    return True, "✓ 已更新"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--apply", action="store_true", help="实际修改文件(默认只检查)")
    args = ap.parse_args()

    if not COMBOS_ROOT.is_dir():
        print(f"联合录制包目录不存在:{COMBOS_ROOT}")
        return

    packs = sorted(COMBOS_ROOT.rglob("*.json"))
    if not packs:
        print(f"未找到联合录制包:{COMBOS_ROOT}")
        return

    print(f"{'[检查模式]' if not args.apply else '[应用模式]'} 扫描 {len(packs)} 个包...")
    print()

    need_update = []
    for p in packs:
        rel = p.relative_to(COMBOS_ROOT)
        changed, msg = migrate_pack(p, dry_run=not args.apply)
        status = "→" if changed else " "
        print(f"  {status} {rel}: {msg}")
        if changed:
            need_update.append(rel)

    print()
    if need_update:
        if args.apply:
            print(f"✓ 已更新 {len(need_update)} 个包")
        else:
            print(f"需要更新 {len(need_update)} 个包。运行 --apply 执行修改。")
    else:
        print("✓ 所有包都是新关节名,无需迁移")


if __name__ == "__main__":
    main()
