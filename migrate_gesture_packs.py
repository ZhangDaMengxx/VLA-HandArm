#!/usr/bin/env python3
"""批量更新技能包的 joint_order 字段:旧关节名 → 新关节名。

背景:2026-08-10 换用厂商新 URDF 后关节名变了,已录制的技能包里的 joint_order
字段还是旧名,加载时会被 gesture_pack.py 拒绝。这个脚本批量改写所有包。

用法:
  python3 migrate_gesture_packs.py --check    # 只检查,不修改
  python3 migrate_gesture_packs.py --apply    # 实际修改
"""
import argparse
import json
from pathlib import Path

REPO = Path(__file__).resolve().parent
GESTURES_ROOT = REPO / "data/gestures"

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
    """迁移一个技能包。返回 (是否需要改, 结果消息)。"""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:
        return False, f"读取失败: {e}"

    jo = data.get("joint_order")
    if jo is None:
        return False, "无 joint_order 字段(可能是旧版本,跳过)"

    if jo == NEW_NAMES:
        return False, "已经是新关节名"

    if jo != OLD_NAMES:
        return False, f"joint_order 不是预期的旧名,不敢自动改:{jo}"

    # 需要迁移
    if dry_run:
        return True, f"需要更新:{OLD_NAMES[0]} → {NEW_NAMES[0]} ..."

    # 实际修改
    data["joint_order"] = NEW_NAMES
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n",
                    encoding="utf-8")
    return True, "✓ 已更新"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--apply", action="store_true", help="实际修改文件(默认只检查)")
    args = ap.parse_args()

    if not GESTURES_ROOT.is_dir():
        print(f"技能包目录不存在:{GESTURES_ROOT}")
        return

    packs = sorted(GESTURES_ROOT.rglob("*.json"))
    if not packs:
        print(f"未找到技能包:{GESTURES_ROOT}")
        return

    print(f"{'[检查模式]' if not args.apply else '[应用模式]'} 扫描 {len(packs)} 个包...")
    print()

    need_update = []
    for p in packs:
        rel = p.relative_to(GESTURES_ROOT)
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
