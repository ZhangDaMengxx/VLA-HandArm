"""把 sim/assets 里的装配 URDF 转成"相对 mesh 路径"副本,供 VSCode URDF Visualizer 打开。

为什么要副本:
- pinocchio(replay_rerun 用)吃绝对路径最稳,原文件保持绝对路径不动。
- VSCode 插件把 mesh filename 当相对 URL 拼在 URDF 目录后面,绝对路径会被拼成
  `sim/assets//home/...` 而 404。副本改成 `../../assets/...` 即可。

用法: python3 sim/make_viz_urdf.py   然后在 VSCode 里打开 *_viz.urdf
"""
import xml.etree.ElementTree as ET
from pathlib import Path

from paths import REPO, ASSEMBLY_URDF, GRIPPER_URDF
SRC = [
    GRIPPER_URDF,
    ASSEMBLY_URDF,
]


def to_relative(src: Path) -> Path:
    tree = ET.parse(src)
    root = tree.getroot()
    missing = []
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        if not fn:
            continue
        p = Path(fn)
        if not p.is_absolute():
            p = (src.parent / fn).resolve()
        if not p.exists():
            missing.append(str(p))
        # 相对 URDF 所在目录,插件/URL 语义能正确折叠 ../
        import os
        mesh.set("filename", os.path.relpath(p, src.parent))
    out = src.with_name(src.stem + "_viz.urdf")
    tree.write(out, encoding="utf-8", xml_declaration=True)
    return out, missing


if __name__ == "__main__":
    for src in SRC:
        if not src.exists():
            print(f"skip (缺文件) {src}")
            continue
        out, missing = to_relative(src)
        print(f"wrote {out}")
        for m in missing:
            print(f"  !! mesh 不存在: {m}")
