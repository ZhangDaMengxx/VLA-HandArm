"""构建 NERO(7-DoF) + inspire 手 的装配 URDF,并在 MuJoCo 里验证加载。

设计约束:
- 保持 inspire 的 link/joint 名不变(dex-retargeting 配置依赖 base/index_tip/... 等名字)。
- NERO 与 inspire 无命名冲突(NERO: world/base_link/link1-7;inspire 根为 'base')。
- 所有 mesh 路径改为绝对路径;MuJoCo 用 .stl/.obj 碰撞网格,.dae/.glb 视觉会被跳过。
- link8(官方手/夹爪法兰) -> 适配法兰 -> inspire 'base' 的安装变换由装配体
  nero_RH56DF.stl 反解得到(非目视标定),推导与校核残差见下方常量块注释。
"""
import xml.etree.ElementTree as ET
from pathlib import Path

from paths import REPO, ASSETS, ARM_ROOT, HAND_ROOT, NERO_FLANGE_URDF, INSPIRE_URDF, ASSEMBLY_URDF

NERO = NERO_FLANGE_URDF
INSP = INSPIRE_URDF
OUT = ASSEMBLY_URDF
HAND_PARENT = "link8"
# 臂关节限位:臂 URDF 原文是 0/0(等于禁止运动),与 ROS 包里手工补的值保持一致。
ARM_EFFORT = "100"
ARM_VELOCITY = "5"

# 因时 RH56DF 适配法兰:插在 link8 与手之间。STL 为毫米,URDF 为米,需 0.001 缩放。
FLANGE_STL = ASSETS / "arm/meshes/rh56df_adapter_flange.stl"
FLANGE_LINK = "rh56df_adapter_flange"
FLANGE_SCALE = "0.001 0.001 0.001"
# 铝件(2700):体素法实测 22511mm^3 -> 60.8g;惯量绕质心,单位 kg·m^2。
FLANGE_MASS = "0.0608"
FLANGE_COM = "-0.000001 -0.000333 -0.005038"
FLANGE_INERTIA = {"ixx": "9.092e-06", "ixy": "0", "ixz": "0",
                  "iyy": "9.457e-06", "iyz": "-5.4e-08", "izz": "1.3210e-05"}

# 以下安装量不是目视标定,而是从装配体 nero_RH56DF.stl 反解出来的(该装配体含
# 臂 + link8 + 适配法兰 + 手)。两个独立测量支撑整条链:
#   (a) 关节7轴线:装配体腕部有个 Ø44.8 圆柱面,圆拟合残差 0.49mm,定出轴线位置;
#   (b) 手基座位姿:手掌网格 ICP,残差 0.36mm(旋转锁精确置换矩阵后仍 0.3555mm,
#       说明该变换严格轴对齐)。
# 法兰自身朝向另由 48 种轴对齐变换穷举定出:绕 Y 转 180° + 沿 Z 平移 -5mm,残差
# 0.12mm;若按单位变换摆放则为 1.43mm。注意 bbox 在两种摆法下都一样(z 范围 [-15,10]
# 关于 -2.5 反对称),不能用 bbox 判别 —— 要看切片轮廓:
#   * 局部 -Z 侧是方板(39.5x38.65,y=+17.5 处有切平面),朝臂,板内沉腔接 link8
#     的输出凸台,腔底为 Ø30 环形止口;
#   * 局部 +Z 侧是圆段(Ø37.8,带 Ø30 台阶),朝手,插进手腕孔,手安装面贴在局部
#     Z=+2.0 处。
# 注:臂 URDF 的 joint8 原为 x=0.032,2026-08-04 已改为 0.031(官方 xacro 值,且
#     Link8/stand_v2/revo2_flange 三个 mesh md5 相同=同一零件,不该有两个安装值;
#     装配体实测中位残差 0.311 vs 0.557mm)。故此处 Z 用 0.016489,不再含补偿。
FLANGE_MOUNT_XYZ = "0 0 0.016489"          # link8 -> 法兰
FLANGE_MOUNT_RPY = "0 0 1.570796"

# 法兰 -> 手根 'base'。q=0(臂竖直)时:手心朝 world -X、手指朝 +Z(沿工具轴伸出)、
# 小指在 +Y 侧、拇指在 -Y 侧 —— 即右手。
MOUNT_XYZ = "0.000042 0.005962 0.002158"
MOUNT_RPY = "0 0 1.570796"


def abspath_meshes(root, base):
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        if not fn:
            continue
        if fn.startswith("package://"):
            pkg_path = fn.removeprefix("package://")
            pkg, _, rel = pkg_path.partition("/")
            # package:// 映射到新布局
            if pkg == "nero_description":
                p = ARM_ROOT / rel
            elif pkg in ("inspire_hand", "urdf_right_2025_4_18"):
                p = HAND_ROOT / rel
            else:
                p = ASSETS / pkg / rel  # 通用回退
        else:
            p = Path(fn)
        if not p.is_absolute():
            p = (base / fn).resolve()
        mesh.set("filename", str(p))


def dae_visual_to_stl(root):
    """把 visual 里的 .dae 换成同名 .STL(臂的视觉网格)。

    为什么:臂的视觉网格是 .dae,而 trimesh / VSCode URDF Visualizer 等查看器要么
    缺 pycollada、要么不吃 collada,会**静默跳过**整条臂 —— 症状就是"只看得见手和
    法兰"。碰撞网格本来就是 .STL,这里让视觉也用 .STL,任何查看器都能开。
    STL 命名不统一(base_link.STL 但 Link1.STL),所以逐个候选试文件是否存在。
    """
    swapped, kept = [], []
    for vis in root.iter("visual"):
        for mesh in vis.iter("mesh"):
            fn = mesh.get("filename")
            if not fn or not fn.lower().endswith(".dae"):
                continue
            p = Path(fn)
            stem = p.stem
            cands = [stem, stem[0].upper() + stem[1:], stem.capitalize()]
            hit = next((c for name in cands for ext in (".STL", ".stl")
                        for c in [p.parent / f"{name}{ext}",
                                  p.parent.parent / f"{name}{ext}"]
                        if c.exists()), None)
            if hit is None:
                kept.append(p.name)
                continue
            mesh.set("filename", str(hit))
            swapped.append(f"{p.name} -> {hit.name}")
    print(f"visual .dae -> .STL: 换掉 {len(swapped)} 个", *swapped, sep="\n    ")
    if kept:
        print(f"  !! 找不到 STL 兄弟文件,仍为 .dae: {kept}")


def link_names(root):
    return {l.get("name") for l in root.findall("link")}


def child_links(root):
    return {j.find("child").get("link") for j in root.findall("joint")
            if j.find("child") is not None}


nero_root = ET.parse(NERO).getroot()
insp_root = ET.parse(INSP).getroot()
abspath_meshes(nero_root, NERO.parent)
abspath_meshes(insp_root, INSP.parent)
dae_visual_to_stl(nero_root)

# 臂 URDF 里 7 个关节写的是 effort="0" velocity="0",ros2_control 会因此完全推不动。
# 之前 ROS 包那份是手工补成 100/5 的,一重新生成就被打回 0 —— 所以在源头补,
# 让下游 import_from_assets.py 生成时自动带上。只补 effort/velocity,不动行程。
patched = []
for j in nero_root.findall("joint"):
    lim = j.find("limit")
    if lim is None or j.get("type") not in ("revolute", "prismatic"):
        continue
    if float(lim.get("effort", "0")) == 0.0:
        lim.set("effort", ARM_EFFORT)
        lim.set("velocity", ARM_VELOCITY)
        patched.append(j.get("name"))
print(f"补臂关节限位 effort={ARM_EFFORT} velocity={ARM_VELOCITY}: {patched}")

# .dae 自带材质,换成 .STL 后就没有颜色了,给没有 material 的臂 visual 补一个,
# 否则 RViz 里整条臂是默认灰。
for vis in nero_root.iter("visual"):
    if vis.find("material") is None:
        mat = ET.SubElement(vis, "material", {"name": "nero_grey"})
        ET.SubElement(mat, "color", {"rgba": "0.75 0.75 0.78 1"})

clash = link_names(nero_root) & link_names(insp_root)
print("link name clash:", clash or "none")

insp_roots = link_names(insp_root) - child_links(insp_root)
hand_root = "base" if "base" in insp_roots else sorted(insp_roots)[0]
print("inspire root link:", insp_roots, "-> using", hand_root)

robot = ET.Element("robot", {"name": "nero_inspire_right"})
for el in list(nero_root):
    if el.tag == "mujoco":
        continue
    robot.append(el)

# 适配法兰 link:视觉+碰撞同一 STL(毫米,靠 scale 缩放);惯量按铝件体素实测。
flange = ET.SubElement(robot, "link", {"name": FLANGE_LINK})
inert = ET.SubElement(flange, "inertial")
ET.SubElement(inert, "origin", {"xyz": FLANGE_COM, "rpy": "0 0 0"})
ET.SubElement(inert, "mass", {"value": FLANGE_MASS})
ET.SubElement(inert, "inertia", FLANGE_INERTIA)
for tag in ("visual", "collision"):
    node = ET.SubElement(flange, tag)
    ET.SubElement(node, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geom = ET.SubElement(node, "geometry")
    ET.SubElement(geom, "mesh", {"filename": str(FLANGE_STL), "scale": FLANGE_SCALE})

jf = ET.SubElement(robot, "joint", {"name": "link8_to_flange", "type": "fixed"})
ET.SubElement(jf, "parent", {"link": HAND_PARENT})
ET.SubElement(jf, "child", {"link": FLANGE_LINK})
ET.SubElement(jf, "origin", {"xyz": FLANGE_MOUNT_XYZ, "rpy": FLANGE_MOUNT_RPY})

# 手掌基座接在法兰上(MOUNT_* 由装配体反解,见上)
j = ET.SubElement(robot, "joint", {"name": "flange_to_hand", "type": "fixed"})
ET.SubElement(j, "parent", {"link": FLANGE_LINK})
ET.SubElement(j, "child", {"link": hand_root})
ET.SubElement(j, "origin", {"xyz": MOUNT_XYZ, "rpy": MOUNT_RPY})

for el in list(insp_root):
    if el.tag == "mujoco":
        continue
    robot.append(el)

OUT.parent.mkdir(parents=True, exist_ok=True)
ET.ElementTree(robot).write(OUT, encoding="utf-8", xml_declaration=True)
print("wrote", OUT)

try:
    import mujoco
except ModuleNotFoundError:
    print("skip MuJoCo validation: module 'mujoco' is not installed")
else:
    m = mujoco.MjModel.from_xml_path(str(OUT))
    jnames = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
    print(f"LOADED nq={m.nq} njnt={m.njnt} nbody={m.nbody} nmesh={m.nmesh}")
    print("joints:", jnames)
