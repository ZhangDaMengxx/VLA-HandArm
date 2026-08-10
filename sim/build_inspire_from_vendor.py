#!/usr/bin/env python3
"""把厂家新 URDF 转成项目命名规范 + GLB mesh。

**为什么要这个脚本**
厂家 2025-04-18 版 URDF 修正了拇指旋转关节相对 base 的装配位置(老 dex-urdf 在
base→hand_base_link 之间插了人为的 -90°X,180°Z 旋转,不准)。但项目里 6 处代码引用
老命名,且 dex-retargeting 需要 5 个 *_tip / 浏览器需要 GLB / 厂家的拇指 limit 与
mimic 倍率互相矛盾(不修浏览器会在中途饱和)。所以写个脚本统一转换,不是手改 URDF。

**做的事**: ①改名 ②补 base+tip ③STL→GLB ④展平链式 mimic ⑤放宽 mimic 子关节 limit
⑥覆盖驱动关节 limit 对齐 HAND_LIMITS(避免预览与硬件不一致) ⑦修两个厂家笔误(重复 mass)

**输出**: 覆盖 assets/inspire_hand/inspire_hand_right.urdf + meshes/。旧版备份 .bak_dexurdf

**厂家新限位未采用**(thumb_pitch 0.48 / 四指 1.333),只同步了 thumb_yaw 1.246165,
理由见 inspire_hand.py 注释:收紧丢 17%/4.5% 行程。要改就 DRIVEN_LIMIT 和 HAND_LIMITS 同时改。

**下游影响**: 指尖移了 19-27mm,scaling_factor 可能要重调; build_urdf/ 标定脚本失效。
"""
from __future__ import annotations
import shutil, sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_arm_viz import read_stl, write_glb

REPO = Path(__file__).resolve().parents[1]
SRC = REPO / "assets/urdf_right/urdf_right_2025_4_18/urdf/urdf_right_2025_4_18.urdf"
SRC_MESH = REPO / "assets/urdf_right/urdf_right_2025_4_18/meshes"
OUT_URDF = REPO / "assets/inspire_hand/inspire_hand_right.urdf"
OUT_VIS = REPO / "assets/inspire_hand/meshes/visual"
OUT_COL = REPO / "assets/inspire_hand/meshes/collision"

LINK_MAP = {
    "R_base_link": "hand_base_link", "right_thumb_1": "thumb_proximal_base",
    "right_thumb_2": "thumb_proximal", "right_thumb_3": "thumb_intermediate",
    "right_thumb_4": "thumb_distal", "right_index_1": "index_proximal",
    "right_index_2": "index_intermediate", "right_middle_1": "middle_proximal",
    "right_middle_2": "middle_intermediate", "right_ring_1": "ring_proximal",
    "right_ring_2": "ring_intermediate", "right_little_1": "pinky_proximal",
    "right_little_2": "pinky_intermediate",
}

JOINT_MAP = {
    "right_thumb_1_joint": "right_thumb_1_joint",
    "right_thumb_2_joint": "right_thumb_2_joint",
    "right_thumb_3_joint": "thumb_intermediate_joint",
    "right_thumb_4_joint": "thumb_distal_joint",
    "right_index_1_joint": "right_index_1_joint",
    "right_index_2_joint": "index_intermediate_joint",
    "right_middle_1_joint": "right_middle_1_joint",
    "right_middle_2_joint": "middle_intermediate_joint",
    "right_ring_1_joint": "right_ring_1_joint",
    "right_ring_2_joint": "ring_intermediate_joint",
    "right_little_1_joint": "right_little_1_joint",
    "right_little_2_joint": "pinky_intermediate_joint",
}


def derive_tip_offset(mesh_stl: Path, frac=0.02):
    """tip 位置 = 距 link 原点最远 frac 比例顶点的质心。老 URDF 上实测误差 0.6~3.5mm。"""
    import numpy as np
    vb, _ = read_stl(mesh_stl)
    v = np.frombuffer(vb, dtype=np.float32).reshape(-1, 3)
    d = np.linalg.norm(v, axis=1)
    k = max(3, int(len(v) * frac))
    return v[np.argsort(d)[-k:]].mean(axis=0)


def _flatten_mimic(root: ET.Element):
    """把**链式** mimic 展平成直接引用驱动关节。

    厂家 URDF 里 thumb_distal mimic 的是 thumb_intermediate,而后者自己也是 mimic。
    URDF 规范没禁止,但 dex-retargeting 的 MimicJointKinematicAdaptor 要求 mimic 的
    source 必须在 target_joint_names(= 驱动关节)里,链式会直接抛
    `ValueError: 'thumb_intermediate_joint' is not in list`。
    展平后 URDF 与 hand3d.js / hand_rerun.py 的扁平 MIMIC 表语义一致,三处同一套数。
    复合公式:distal = (pitch*a + b)*c + d = pitch*(a*c) + (b*c + d)。
    """
    mim = {}                       # joint 名 → (mimic 元素, source, mult, off)
    for j in root.findall("joint"):
        m = j.find("mimic")
        if m is not None:
            mim[j.get("name")] = (m, m.get("joint"),
                                  float(m.get("multiplier", 1.0)),
                                  float(m.get("offset", 0.0)))
    for name, (el, src, mult, off) in mim.items():
        seen = {name}
        while src in mim:           # source 自己也是 mimic → 继续往上折
            if src in seen:
                raise ValueError(f"mimic 成环: {name} -> {src}")
            seen.add(src)
            _, up_src, up_mult, up_off = mim[src]
            mult, off, src = mult * up_mult, off + mult * up_off, up_src
        if el.get("joint") != src:
            print(f"  展平 mimic: {name} -> {src} x{mult:.6f} +{off:.6f}")
            el.set("joint", src)
            el.set("multiplier", f"{mult:.6f}")
            el.set("offset", f"{off:.6f}")


# 驱动关节 limit 覆盖:必须与 sim/inspire_hand.py 的 HAND_LIMITS 逐位一致。
#
# 为什么要覆盖厂家值:浏览器 urdf_view.js 按 URDF 的 lower/upper 夹取,硬件驱动按
# HAND_LIMITS 夹取。两边不一致 = **预览骗人** —— 拖到 0.6 时真手收到 0.6、3D 却停在
# 0.48,而"文件明明改了"和"画面明明没动"两边都成立,这类不一致最难查
# (gesture_pack.py:162 记过同一个坑的另一面)。
# 厂家新值留在注释里,等真手实测行程后再决定是否收紧;那时**两处同时改**。
#   thumb_yaw 1.246165(已采用) / thumb_pitch 0.48(未采用) / 四指 1.333(未采用)
DRIVEN_LIMIT = {
    "right_thumb_1_joint": (0.0, 1.246165),   # = 厂家新值,已同步
    "right_thumb_2_joint": (0.0, 0.6),      # 厂家新值 0.48,保留老值待实测
    "right_index_1_joint": (0.0, 1.47),           # 厂家新值 1.333,同上
    "right_middle_1_joint": (0.0, 1.47),
    "right_ring_1_joint": (0.0, 1.47),
    "right_little_1_joint": (0.0, 1.47),
}


def _override_driven_limits(root: ET.Element):
    """把 6 个驱动关节的 limit 改成 DRIVEN_LIMIT。**要在放宽 mimic 之前跑**,
    这样 mimic 子关节的行程是按覆盖后的驱动上限算的。"""
    for j in root.findall("joint"):
        tgt = DRIVEN_LIMIT.get(j.get("name"))
        l = j.find("limit")
        if tgt is None or l is None:
            continue
        old = (float(l.get("lower")), float(l.get("upper")))
        if abs(old[0] - tgt[0]) > 1e-9 or abs(old[1] - tgt[1]) > 1e-9:
            print(f"  覆盖 {j.get('name')}: 厂家[{old[0]:.4f},{old[1]:.4f}]"
                  f" -> [{tgt[0]:.4f},{tgt[1]:.4f}]")
            l.set("lower", f"{tgt[0]:.6f}")
            l.set("upper", f"{tgt[1]:.6f}")


def _widen_mimic_limits(root: ET.Element):
    """把 mimic 子关节的 limit 放宽到刚好容纳"驱动走满"的值。**必须在展平之后跑。**

    厂家 URDF 里拇指这两个是矛盾的:pitch 上限 0.48,按 ×1.1425 推 thumb_intermediate
    要到 0.5484,而它自己的 upper 只写 0.3578 —— 超了 10.9°。这不是我转换出来的,
    厂家原文件就有(thumb_4 同样超 7.7°);老 dex-urdf 那份的 upper 恰好等于
    "驱动上限×倍率"(0.6×1.334=0.8004 vs 写 0.8),是自洽的,新的没守这个约定。

    为什么必须修:浏览器 urdf_view.js 是**逐关节**按自己的 lower/upper 夹取的
    (web/urdf_view.js:275),不修的话拇指弯到 0.3578 就饱和 —— 表现就是"拇指第二节
    弯不动",而滑块还在往前走。pinocchio 侧同理。
    改 limit 而不是改倍率:倍率是连杆比例(厂家实测),limit 只是行程声明,后者才是笔误。
    """
    lim, mim = {}, {}
    for j in root.findall("joint"):
        l, m = j.find("limit"), j.find("mimic")
        if l is not None:
            lim[j.get("name")] = l
        if m is not None:
            mim[j.get("name")] = m
    for name, m in mim.items():
        src, mu, off = m.get("joint"), float(m.get("multiplier")), float(m.get("offset"))
        if src not in lim or name not in lim:
            continue
        d_lo, d_hi = float(lim[src].get("lower")), float(lim[src].get("upper"))
        ends = sorted((d_lo * mu + off, d_hi * mu + off))       # 倍率可能为负
        lo, hi = float(lim[name].get("lower")), float(lim[name].get("upper"))
        new_lo, new_hi = min(lo, ends[0]), max(hi, ends[1])
        if abs(new_lo - lo) > 1e-6 or abs(new_hi - hi) > 1e-6:
            print(f"  放宽 {name}: [{lo:.4f},{hi:.4f}] -> [{new_lo:.4f},{new_hi:.4f}]"
                  f"  (驱动 {src} 走满需 {ends[1]:.4f})")
            lim[name].set("lower", f"{new_lo:.6f}")
            lim[name].set("upper", f"{new_hi:.6f}")


def main() -> int:
    if not SRC.is_file():
        print(f"源不存在: {SRC}", file=sys.stderr)
        return 1
    root = ET.parse(SRC).getroot()
    new_root = ET.Element("robot", {"name": "inspire_hand_right"})
    ET.SubElement(new_root, "link", {"name": "base"})
    bj = ET.SubElement(new_root, "joint", {"name": "base_joint", "type": "fixed"})
    ET.SubElement(bj, "parent", {"link": "base"})
    ET.SubElement(bj, "child", {"link": "hand_base_link"})
    # ⚠ 单位变换,不是老 URDF 的 rpy="-1.57079 0 3.14159"。
    # 老 dex-urdf 有个人为中间层:base --(转 -90°X,180°Z)--> hand_base_link,手指挂后者。
    # 厂家新 URDF 没这层,R_base_link 的坐标系**就等于老的 base**(已验证:新 R_base_link
    # 质心经老 base_joint 旋转后 = 老 hand_base_link 质心,逐位吻合)。
    # 所以这里把 R_base_link 直接当 hand_base_link 用,base_joint 必须是单位变换 ——
    # 再套一次旋转就是双重旋转(第一版这么写,指尖跑到 base 系 -Y 方向 200mm 外)。
    # base 坐标系因此保持不变 → build_nero_inspire.py 的 MOUNT_XYZ/RPY 继续有效。
    ET.SubElement(bj, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    OUT_VIS.mkdir(parents=True, exist_ok=True)
    OUT_COL.mkdir(parents=True, exist_ok=True)
    mesh_done = {}
    for link in root.findall("link"):
        old = link.get("name")
        if old not in LINK_MAP:
            print(f"  跳过未映射 link: {old}")
            continue
        link.set("name", LINK_MAP[old])
        # 厂家 URDF 的 right_thumb_3 有**两个** <mass>(0.00378.. 和 0.00985)。
        # 哪个生效取决于解析器取第一个还是最后一个 —— pinocchio / MuJoCo / 浏览器
        # 可能不一致。保留后者(0.00985,与同级 inertia 量级相符),删掉多余的。
        for inert in link.findall("inertial"):
            masses = inert.findall("mass")
            for extra in masses[:-1]:
                inert.remove(extra)
                print(f"  修 {link.get('name')}: 删重复 mass={extra.get('value')}")
        for vis in link.findall("visual"):
            m = vis.find("geometry/mesh")
            if m is None or not m.get("filename"):
                continue
            sn = Path(m.get("filename")).name
            if sn not in mesh_done:
                vb, n = read_stl(SRC_MESH / sn)
                on = sn.replace(".STL", ".glb")
                write_glb(vb, OUT_VIS / on)
                mesh_done[sn] = on
                print(f"  {sn:22s} -> visual/{on:22s} ({n} tri)")
            m.set("filename", f"meshes/visual/{mesh_done[sn]}")
        for col in link.findall("collision"):
            m = col.find("geometry/mesh")
            if m is None or not m.get("filename"):
                continue
            sn = Path(m.get("filename")).name
            shutil.copy(SRC_MESH / sn, OUT_COL / sn)
            m.set("filename", f"meshes/collision/{sn}")
        new_root.append(link)
    for j in root.findall("joint"):
        if j.get("name") in JOINT_MAP:
            j.set("name", JOINT_MAP[j.get("name")])
        for el in (j.find("parent"), j.find("child")):
            if el is not None and el.get("link") in LINK_MAP:
                el.set("link", LINK_MAP[el.get("link")])
        # mimic 的 joint 属性也要跟着改名,漏了 pinocchio/浏览器都会找不到驱动关节
        mim = j.find("mimic")
        if mim is not None and mim.get("joint") in JOINT_MAP:
            mim.set("joint", JOINT_MAP[mim.get("joint")])
        new_root.append(j)
    _flatten_mimic(new_root)
    _override_driven_limits(new_root)
    _widen_mimic_limits(new_root)
    TIPS = [("thumb_distal", "thumb_tip", "right_thumb_4.STL"),
            ("index_intermediate", "index_tip", "right_index_2.STL"),
            ("middle_intermediate", "middle_tip", "right_middle_2.STL"),
            ("ring_intermediate", "ring_tip", "right_ring_2.STL"),
            ("pinky_intermediate", "pinky_tip", "right_little_2.STL")]
    for parent, tip, mesh in TIPS:
        ET.SubElement(new_root, "link", {"name": tip})
        tj = ET.SubElement(new_root, "joint", {"name": f"{tip}_joint", "type": "fixed"})
        ET.SubElement(tj, "parent", {"link": parent})
        ET.SubElement(tj, "child", {"link": tip})
        p = derive_tip_offset(SRC_MESH / mesh)
        ET.SubElement(tj, "origin",
                      {"xyz": f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}", "rpy": "0 0 0"})
        print(f"  tip {tip:12s} <- {parent:20s} xyz=({p[0]:+.4f},{p[1]:+.4f},{p[2]:+.4f})")
    OUT_URDF.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(new_root, space="  ")
    ET.ElementTree(new_root).write(OUT_URDF, encoding="utf-8", xml_declaration=True)
    print(f"\n写出 {OUT_URDF.relative_to(REPO)}"
          f"  ({len(mesh_done)} glb, {len(list(OUT_COL.glob('*.STL')))} stl)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
