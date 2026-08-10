#!/usr/bin/env python3
"""构建手部浏览器可视化 URDF:STL 相对路径 → glb 相对路径。

输入:assets/hand/urdf/inspire_hand_right.urdf(STL,相对路径 ../meshes/*.STL)
输出:assets/viz/hand/inspire_hand_right_viz.urdf(glb,相对路径 meshes/*.glb)

glb 已由 build_combo_viz.py 转好(13 个,从 combo/meshes 挑出来的那批)。
这里只做路径替换 + collision 删除。
"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SRC = REPO / "assets/hand/urdf/inspire_hand_right.urdf"
OUT = REPO / "assets/viz/hand/inspire_hand_right_viz.urdf"

tree = ET.parse(SRC)
root = tree.getroot()

# 删除所有 collision(浏览器不需要)
for link in root.findall("link"):
    for col in link.findall("collision"):
        link.remove(col)

# 替换 mesh 路径:../meshes/*.STL → meshes/*.glb
for mesh in root.iter("mesh"):
    fn = mesh.get("filename")
    if not fn:
        continue
    # ../meshes/R_base_link.STL → meshes/R_base_link.glb
    fn = re.sub(r'\.\./meshes/(.+)\.STL$', r'meshes/\1.glb', fn)
    mesh.set("filename", fn)

OUT.parent.mkdir(parents=True, exist_ok=True)
tree.write(OUT, encoding="utf-8", xml_declaration=True)
print(f"✓ {OUT.relative_to(REPO)}")
