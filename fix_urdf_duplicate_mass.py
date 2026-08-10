#!/usr/bin/env python3
"""修复厂商 URDF 的重复 <mass> 标签。

right_thumb_3 link 的 <inertial> 里有两个 <mass>,MuJoCo 拒绝加载。
删除第一个(0.00378kg),保留第二个(0.00985kg)。
"""
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parent
URDF = REPO / "assets/hand/urdf/inspire_hand_right.urdf"

tree = ET.parse(URDF)
root = tree.getroot()

fixed = []
for link in root.findall("link"):
    inertial = link.find("inertial")
    if inertial is None:
        continue
    masses = inertial.findall("mass")
    if len(masses) > 1:
        # 删除除最后一个外的所有 mass
        for m in masses[:-1]:
            inertial.remove(m)
        fixed.append((link.get("name"), len(masses)))

if not fixed:
    print("✓ 无重复 <mass> 标签")
else:
    tree.write(URDF, encoding="utf-8", xml_declaration=True)
    for name, count in fixed:
        print(f"✓ {name}: 删除 {count-1} 个重复 <mass>")
    print(f"\n已修复并保存:{URDF.relative_to(REPO)}")
