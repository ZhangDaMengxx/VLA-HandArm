#!/usr/bin/env python3
"""构建手部 pinocchio 可视化 URDF:STL 相对路径 → STL 绝对路径。

hand_rerun.py 用 pinocchio 加载,它不认 mimic 标签,所以耦合关节要代码里补。
mesh 路径用绝对路径最稳(相对路径在不同工作目录下会解析错)。

输入:assets/hand/urdf/inspire_hand_right.urdf
输出:assets/assembled/inspire_hand_absolute.urdf
"""
import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SRC = REPO / "assets/hand/urdf/inspire_hand_right.urdf"
OUT = REPO / "assets/assembled/inspire_hand_absolute.urdf"

txt = SRC.read_text(encoding="utf-8")
# ../meshes/ → /home/.../assets/hand/meshes/
txt = re.sub(r'filename="\.\./meshes/', f'filename="{REPO}/assets/hand/meshes/', txt)

OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(txt, encoding="utf-8")
print(f"✓ {OUT.relative_to(REPO)}")
