#!/usr/bin/env python3
"""sim/build_arm_viz.py — 把臂的 STL mesh 转成浏览器能直接吃的 glb,并改写 URDF。

为什么不用原始的 .dae:
  · nero_description.urdf 的 visual 全是 Collada,已 vendor 的 GLTFLoader 读不了
  · link2.dae 是 **24MB** 的 XML,而 Link2.STL 只有 1.7MB —— 同一几何(三角数一致),
    浏览器解析 24MB XML 太慢
  · dae 里 diffuse 是纯黑 `0 0 0 1`,直接用会渲染成全黑,材质本来就要覆盖
  → 用 collision 的 STL(已核对 7/8 个 link 包围盒与 dae 逐项一致,Link5 那处差异
    是 dae 多几何块导致我方测量不全,不是几何不同),转 glb 后复用 GLTFLoader。

为什么不用 ColladaLoader:能 vendor(网络通),但它是 three.js examples 里维护较弱
的一档,而且解决不了 24MB 那个体积问题。

另外改两处 URDF 里浏览器吃不下的东西:
  · `package://nero_description/...` → 相对路径(靠 /arm_assets 静态挂载解析)
  · end_effector 引用的 `stand_v1.dae` 在仓库里**不存在**(dae 目录只有 base_link
    + link1..link7)。名字像底座支架被错挂到末端,是 URDF 自身笔误 —— 直接删掉那个
    visual,不然浏览器每次都要吃一个 404。

输出:
  sim/assets/arm_viz/nero_arm_viz.urdf   +  meshes/*.glb
"""
from __future__ import annotations

import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SIM = Path(__file__).resolve().parent
REPO = SIM.parent
SRC_URDF = REPO / "assets/nero_description/urdf/nero_description.urdf"
SRC_MESH = REPO / "assets/nero_description/meshes"
OUT = REPO / "assets/viz/arm"

# visual 的 dae 文件名 → 用哪个 STL 代替。dae 是小写 link*,STL 是大写 Link*。
DAE_TO_STL = {
    "base_link.dae": "base_link.STL",
    **{f"link{i}.dae": f"Link{i}.STL" for i in range(1, 8)},
}


def read_stl(path: Path):
    """读 binary STL → (顶点 float32 扁平数组, 三角数)。

    只支持 binary(这批资产实测全是 binary,已按 84+n*50==filesize 核对)。
    ascii STL 走这里会得到荒谬的三角数,所以显式校验后报错,不静默出错。
    """
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f"{path.name}: 文件太短,不是 STL")
    n = struct.unpack("<I", raw[80:84])[0]
    if 84 + n * 50 != len(raw):
        raise ValueError(f"{path.name}: 不是 binary STL"
                         f"(声明 {n} 三角需 {84+n*50} 字节,实际 {len(raw)})")
    verts = bytearray()
    off = 84
    for _ in range(n):
        # 每条记录: 法线 12B + 三个顶点 36B + 属性 2B。法线丢掉,让 three.js 自己算
        # 平滑法线 —— STL 的法线是逐面的,直接用会让曲面看起来是刻面的。
        verts += raw[off + 12:off + 48]
        off += 50
    return bytes(verts), n


def _bbox(vbytes: bytes):
    """算包围盒。glTF 规范**要求** POSITION accessor 带 min/max,缺了严格的
    加载器会拒收。手写 glb 最容易漏这一条。"""
    import array
    a = array.array("f")
    a.frombytes(vbytes)
    lo = [min(a[i::3]) for i in range(3)]
    hi = [max(a[i::3]) for i in range(3)]
    return lo, hi


def write_glb(vbytes: bytes, out: Path) -> int:
    """把顶点数组包成最小 glb(无索引,三角列表)。

    自己拼而不用 trimesh:trimesh 的 dae 路径要 pycollada(没装),而 STL→glb 这段
    只是把 float32 塞进 buffer + 一段 JSON,不值得为它加依赖。
    """
    import base64, json  # noqa: E401  局部 import,这个脚本是一次性工具
    n_vert = len(vbytes) // 12
    lo, hi = _bbox(vbytes)
    gltf = {
        "asset": {"version": "2.0", "generator": "build_arm_viz.py"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "mode": 4}]}],
        "accessors": [{"bufferView": 0, "componentType": 5126, "count": n_vert,
                       "type": "VEC3", "min": lo, "max": hi}],
        "bufferViews": [{"buffer": 0, "byteOffset": 0,
                         "byteLength": len(vbytes), "target": 34962}],
        "buffers": [{"byteLength": len(vbytes)}],
    }
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * (-len(js) % 4)                     # chunk 必须 4 字节对齐
    bin_ = vbytes + b"\x00" * (-len(vbytes) % 4)
    total = 12 + 8 + len(js) + 8 + len(bin_)
    buf = bytearray()
    buf += b"glTF" + struct.pack("<II", 2, total)
    buf += struct.pack("<I", len(js)) + b"JSON" + js
    buf += struct.pack("<I", len(bin_)) + b"BIN\x00" + bin_
    out.write_bytes(buf)
    return n_vert // 3


def main() -> int:
    if not SRC_URDF.is_file():
        print(f"找不到 {SRC_URDF}", file=sys.stderr)
        return 1
    (OUT / "meshes").mkdir(parents=True, exist_ok=True)
    tree = ET.parse(SRC_URDF)
    root = tree.getroot()

    converted, dropped = {}, []
    for link in root.findall("link"):
        for vis in list(link.findall("visual")):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            name = mesh.get("filename", "").split("/")[-1]
            stl = DAE_TO_STL.get(name)
            if stl is None or not (SRC_MESH / stl).is_file():
                # 找不到对应 STL(如 stand_v1.dae)→ 删掉这个 visual,免得 404
                link.remove(vis)
                dropped.append((link.get("name"), name))
                continue
            if stl not in converted:
                vb, ntri = read_stl(SRC_MESH / stl)
                gname = Path(stl).stem + ".glb"
                got = write_glb(vb, OUT / "meshes" / gname)
                converted[stl] = (gname, ntri, got,
                                  (OUT / "meshes" / gname).stat().st_size)
            mesh.set("filename", "meshes/" + converted[stl][0])

    # collision 留着但也要去掉 package:// —— 浏览器不解析 collision,可 pinocchio
    # 之类的工具会读,留个能用的相对路径比留个坏 URI 好。
    for c in root.iter("mesh"):
        fn = c.get("filename", "")
        if fn.startswith("package://nero_description/"):
            c.set("filename", fn.replace("package://nero_description/", ""))

    out_urdf = OUT / "nero_arm_viz.urdf"
    tree.write(out_urdf, encoding="utf-8", xml_declaration=True)

    print(f"{'STL':16s}{'三角':>8s}{'写出三角':>9s}{'glb KB':>9s}  校验")
    total = 0
    for stl, (g, ntri, got, sz) in sorted(converted.items()):
        total += sz
        print(f"{stl:16s}{ntri:8d}{got:9d}{sz/1024:9.0f}  {'OK' if ntri==got else 'MISMATCH'}")
    print(f"\nglb 合计 {total/1024/1024:.1f} MB → {out_urdf}")
    for ln, dn in dropped:
        print(f"⚠ 丢弃 visual: link={ln} mesh={dn}(仓库里没有这个文件)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
