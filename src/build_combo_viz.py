#!/usr/bin/env python3
"""src/build_combo_viz.py — 把**装配** URDF(臂+法兰+手)转成浏览器能吃的一份。

合体页(实时 Live)的 3D 要的是一整条链:base_link → link1..7 → link8 →
适配法兰 → 手 base → 五指。`assets/nero_inspire_right.urdf` 就是这条链,但它
浏览器**取不到**,两个原因:

  1. mesh 路径是**绝对文件系统路径**(`/home/.../assets/...`)—— 那是给 pinocchio
     用的。`nero_inspire_right_viz.urdf` 里是 `../../assets/...` 相对路径,同样
     出不了静态挂载的根。
  2. 臂那 9 个件的 visual 是 **STL**,已 vendor 的 GLTFLoader 读不了。

所以这里做和 `build_arm_viz.py` 同一件事,只是源换成装配 URDF:STL→glb、路径改成
挂载根内的相对路径、collision 整段删掉。产物 `assets/viz/combo/`。

## 为什么把 mesh **拷一份**而不是指到 /arm_assets 和 /hand_assets

那两个目录已经挂着,URDF 里写 `/arm_assets/meshes/Link1.glb` 也能加载,还省 9MB
磁盘和一次下载。没那么做是因为**产物就不再自洽**了:combo_viz 目录单独拿出去
(换个挂载点、给别的查看器、直接 file://)会全部 404,而坏在哪要顺着两层挂载配置
才看得出来。`build_arm_viz.py` 的产物是自洽的,这份保持一致 —— 9MB 换一个能独立
验证的目录,划得来。

## Link8 和适配法兰是这里新转的

`build_arm_viz.py` 的源是 `nero_description.urdf`,那份链到 link7 就结束了,所以
只转了 base_link + Link1..7。装配体多出 **Link8**(手腕末节)和
**rh56df_adapter_flange**(RH56DF 适配法兰,`build_nero_inspire.py` 从装配体
反解出来的挂接件)。手的 9 个 visual 本来就是 glb,直接拷。

输出:
  assets/viz/combo/nero_inspire_right_viz.urdf  +  meshes/*.glb
"""
from __future__ import annotations

import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# STL→glb 的实现直接复用,别再写第二份 —— glTF 规范那两个坑(POSITION 必须带
# min/max、逐面法线要丢掉让 three.js 自己算平滑法线)踩一次就够了。
from build_arm_viz import read_stl, write_glb

from paths import REPO, SIM, ASSEMBLY_URDF, VIZ, ARM_ROOT, HAND_ROOT

SRC_URDF = ASSEMBLY_URDF
OUT = VIZ / "combo"
OUT_URDF = OUT / "nero_inspire_right_viz.urdf"

# 已经转好的 glb 直接拿,别重复转:臂的 8 个在 build_arm_viz.py 的产物里。
# 手的 glb:先找 viz/hand(如果已转过),找不到就从 legacy 的 visual 目录拿,
# 都没有再去 hand/meshes 转 STL。键是**源文件名**(URDF 里 filename 的最后一段)。
_ARM_GLB = VIZ / "arm/meshes"
_HAND_GLB_VIZ = VIZ / "hand/meshes"
_HAND_GLB_LEGACY = HAND_ROOT.parent / "hand_legacy/inspire_hand_legacy/meshes/visual"
_HAND_STL = HAND_ROOT / "meshes"


def _resolve(name: str) -> tuple[Path, bool]:
    """按 mesh 文件名找源文件。返回 (路径, 是否需要 STL→glb 转换)。

    顺序有讲究:先找现成的 glb,找不到才回退去转 STL。
    查找顺序:
    1. VIZ/arm/meshes/*.glb (臂的glb)
    2. VIZ/hand/meshes/*.glb (手的glb,如果之前转过)
    3. hand_legacy/*/meshes/visual/*.glb (旧手的glb,兼容)
    4. ARM_ROOT/meshes/*.STL (臂的STL,需要转换)
    5. HAND_ROOT/meshes/*.STL (新手的STL,需要转换)
    """
    stem = Path(name).stem
    # 先找现成glb
    for d in (_ARM_GLB, _HAND_GLB_VIZ, _HAND_GLB_LEGACY):
        p = d / f"{stem}.glb"
        if p.is_file():
            return p, False
    # 没有现成 glb:去源目录找 STL 自己转
    for base_dir in (ARM_ROOT, HAND_ROOT):
        for cand in (f"{stem}.STL", f"{stem}.stl", name):
            p = base_dir / "meshes" / cand
            if p.is_file():
                return p, True
    raise FileNotFoundError(f"找不到 mesh 源:{name} (stem={stem})")


def main() -> int:
    if not SRC_URDF.is_file():
        print(f"找不到装配 URDF {SRC_URDF}\n先跑:python3 src/build_nero_inspire.py",
              file=sys.stderr)
        return 1
    (OUT / "meshes").mkdir(parents=True, exist_ok=True)
    tree = ET.parse(SRC_URDF)
    root = tree.getroot()

    done: dict[str, tuple[str, int, str]] = {}       # 源名 → (glb 名, KB, 来源)
    for link in root.findall("link"):
        # collision 整段删掉:浏览器不渲染它,留着只是让 URDF 大一倍、多 8 个
        # 取不到的 .obj 路径。pinocchio 要用的是 assets/ 下那两份原文,不是这个。
        for col in list(link.findall("collision")):
            link.remove(col)
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            src_name = mesh.get("filename", "").split("/")[-1]
            if src_name not in done:
                src, need_conv = _resolve(src_name)
                gname = Path(src_name).stem + ".glb"
                dst = OUT / "meshes" / gname
                if need_conv:
                    vb, _ = read_stl(src)
                    write_glb(vb, dst)
                else:
                    shutil.copyfile(src, dst)
                done[src_name] = (gname, dst.stat().st_size // 1024,
                                  "转换" if need_conv else "拷贝")
            mesh.set("filename", "meshes/" + done[src_name][0])

    tree.write(OUT_URDF, encoding="utf-8", xml_declaration=True)
    print(f"{'源 mesh':38s}{'glb':34s}{'KB':>6s}  来源")
    for src_name, (g, kb, how) in sorted(done.items()):
        print(f"{src_name:38s}{g:34s}{kb:6d}  {how}")
    total = sum(kb for _, kb, _ in done.values())
    print(f"\n{len(done)} 个 mesh 合计 {total/1024:.1f} MB → {OUT_URDF}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
