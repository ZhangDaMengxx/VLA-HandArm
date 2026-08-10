#!/usr/bin/env python3
"""verify_migration.py — 验证灵巧手URDF迁移是否成功

检查项：
1. 新URDF文件存在且可解析
2. 旧URDF已备份
3. inspire_hand.py中的关节名已更新
4. 关节数量正确（6个驱动关节）
5. 限位值已同步新URDF
"""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

# 项目根目录
REPO = Path(__file__).parent

# 预期的新关节名
EXPECTED_JOINTS = [
    "right_thumb_1_joint",
    "right_thumb_2_joint",
    "right_index_1_joint",
    "right_middle_1_joint",
    "right_ring_1_joint",
    "right_little_1_joint",
]

# 预期的限位值（从新URDF）
EXPECTED_LIMITS = {
    "right_thumb_1_joint": (0.0, 1.246165),
    "right_thumb_2_joint": (0.0, 0.48),
    "right_index_1_joint": (0.0, 1.333),
    "right_middle_1_joint": (0.0, 1.333),
    "right_ring_1_joint": (0.0, 1.333),
    "right_little_1_joint": (0.0, 1.333),
}

def check_urdf_files():
    """检查URDF文件位置"""
    print("=" * 60)
    print("1. 检查URDF文件")
    print("=" * 60)

    new_urdf = REPO / "assets/inspire_hand/urdf/inspire_hand_right.urdf"
    legacy_urdf = REPO / "assets/inspire_hand_legacy/inspire_hand_right.urdf"

    if new_urdf.exists():
        print(f"✅ 新URDF存在: {new_urdf.relative_to(REPO)}")
    else:
        print(f"❌ 新URDF不存在: {new_urdf.relative_to(REPO)}")
        return False

    if legacy_urdf.exists():
        print(f"✅ 旧URDF已备份: {legacy_urdf.relative_to(REPO)}")
    else:
        print(f"⚠️  旧URDF备份不存在（可能之前就没有）")

    return True

def check_urdf_joints():
    """检查URDF中的关节定义"""
    print("\n" + "=" * 60)
    print("2. 检查URDF关节定义")
    print("=" * 60)

    new_urdf = REPO / "assets/inspire_hand/urdf/inspire_hand_right.urdf"

    try:
        tree = ET.parse(new_urdf)
        root = tree.getroot()

        # 提取驱动关节（revolute且无mimic）
        driving_joints = []
        for joint in root.findall('.//joint'):
            if joint.get('type') == 'revolute':
                mimic = joint.find('mimic')
                if mimic is None:
                    name = joint.get('name')
                    limit = joint.find('limit')
                    if limit is not None:
                        lower = float(limit.get('lower'))
                        upper = float(limit.get('upper'))
                        driving_joints.append((name, lower, upper))

        print(f"找到 {len(driving_joints)} 个驱动关节：")
        for name, lower, upper in sorted(driving_joints):
            print(f"  {name:30s} [{lower:.6f}, {upper:.6f}]")

        # 验证关节名
        urdf_joint_names = sorted([j[0] for j in driving_joints])
        expected_sorted = sorted(EXPECTED_JOINTS)

        if urdf_joint_names == expected_sorted:
            print("\n✅ URDF关节名符合预期")
            return True
        else:
            print("\n❌ URDF关节名不符合预期")
            print(f"预期: {expected_sorted}")
            print(f"实际: {urdf_joint_names}")
            return False

    except Exception as e:
        print(f"❌ 解析URDF失败: {e}")
        return False

def check_inspire_hand_py():
    """检查inspire_hand.py中的配置"""
    print("\n" + "=" * 60)
    print("3. 检查inspire_hand.py配置")
    print("=" * 60)

    inspire_hand = REPO / "sim/inspire_hand.py"

    try:
        # 导入模块
        sys.path.insert(0, str(REPO / "sim"))
        import inspire_hand as ih

        # 检查HAND_JOINTS
        print(f"HAND_JOINTS ({len(ih.HAND_JOINTS)}个):")
        for i, joint in enumerate(ih.HAND_JOINTS):
            expected = EXPECTED_JOINTS[i]
            status = "✅" if joint == expected else "❌"
            print(f"  {status} [{i}] {joint}")

        if ih.HAND_JOINTS == EXPECTED_JOINTS:
            print("✅ HAND_JOINTS配置正确")
        else:
            print("❌ HAND_JOINTS配置不正确")
            return False

        # 检查HAND_LIMITS
        print(f"\nHAND_LIMITS ({len(ih.HAND_LIMITS)}个):")
        all_correct = True
        for joint in EXPECTED_JOINTS:
            actual = ih.HAND_LIMITS.get(joint)
            expected = EXPECTED_LIMITS.get(joint)
            if actual == expected:
                print(f"  ✅ {joint:30s} {actual}")
            else:
                print(f"  ❌ {joint:30s} 预期{expected} 实际{actual}")
                all_correct = False

        if all_correct:
            print("✅ HAND_LIMITS配置正确")
            return True
        else:
            print("❌ HAND_LIMITS配置不正确")
            return False

    except Exception as e:
        print(f"❌ 检查inspire_hand.py失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def check_no_old_joints():
    """检查是否还有旧关节名残留"""
    print("\n" + "=" * 60)
    print("4. 检查旧关节名残留")
    print("=" * 60)

    old_joints = [
        "thumb_proximal_yaw_joint",
        "thumb_proximal_pitch_joint",
        "index_proximal_joint",
        "middle_proximal_joint",
        "ring_proximal_joint",
        "pinky_proximal_joint",
    ]

    # 搜索sim/目录下的.py文件（排除备份和迁移脚本）
    sim_dir = REPO / "sim"
    py_files = [f for f in sim_dir.rglob("*.py")
                if "inspire_hand_legacy" not in str(f)
                and "migrate_hand_joints" not in f.name
                and "verify_migration" not in f.name]

    found_old = []
    for py_file in py_files:
        try:
            content = py_file.read_text(encoding='utf-8')
            for old_joint in old_joints:
                if old_joint in content:
                    found_old.append((py_file.relative_to(REPO), old_joint))
        except Exception:
            pass

    if found_old:
        print("❌ 发现旧关节名残留：")
        for file, joint in found_old:
            print(f"  {file}: {joint}")
        return False
    else:
        print("✅ 未发现旧关节名残留")
        return True

def main():
    print("\n🔍 灵巧手URDF迁移验证")
    print("=" * 60)

    checks = [
        ("URDF文件", check_urdf_files),
        ("URDF关节", check_urdf_joints),
        ("inspire_hand.py", check_inspire_hand_py),
        ("旧关节名残留", check_no_old_joints),
    ]

    results = []
    for name, check_func in checks:
        try:
            result = check_func()
            results.append((name, result))
        except Exception as e:
            print(f"\n❌ {name}检查异常: {e}")
            results.append((name, False))

    # 总结
    print("\n" + "=" * 60)
    print("验证总结")
    print("=" * 60)

    for name, result in results:
        status = "✅" if result else "❌"
        print(f"{status} {name}")

    all_passed = all(r for _, r in results)

    if all_passed:
        print("\n✅✅✅ 所有检查通过！迁移成功！")
        print("\n下一步：")
        print("  1. 运行单元测试: python3 -m pytest sim/test_*.py")
        print("  2. 启动手部控制台: python3 sim/hand_console.py --no-mock")
        print("  3. 测试Web界面: ~/gradio_venv/bin/python sim/app_web.py")
        return 0
    else:
        print("\n❌ 部分检查未通过，请检查上述错误")
        return 1

if __name__ == "__main__":
    sys.exit(main())
