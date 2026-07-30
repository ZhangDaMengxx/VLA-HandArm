"""构建 NERO(7-DoF) + inspire 手 的装配 URDF,并在 MuJoCo 里验证加载。

设计约束:
- 保持 inspire 的 link/joint 名不变(dex-retargeting 配置依赖 base/index_tip/... 等名字)。
- NERO 与 inspire 无命名冲突(NERO: world/base_link/link1-7;inspire 根为 'base')。
- 所有 mesh 路径改为绝对路径;MuJoCo 用 .stl/.obj 碰撞网格,.dae/.glb 视觉会被跳过。
- link8(官方手/夹爪法兰) -> inspire 'base' 的安装变换先用单位,加载成功后再按视觉标定。
"""
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
NERO = REPO / "assets/nero_description/urdf/nero_with_hand_flange_description.urdf"
INSP = REPO / "assets/inspire_hand/inspire_hand_right.urdf"
OUT = REPO / "sim/assets/nero_inspire_right.urdf"
HAND_PARENT = "link8"

# 因时 RH56DF 适配法兰:插在 link8 与手之间。CAD 已在装配坐标系导出(相对 link8
# 的位置/朝向烘焙进网格),故挂到 link8 用单位变换;STL 为毫米,URDF 为米,需 0.001 缩放。
FLANGE_STL = REPO / "assets/nero_description/meshes/NERO+因时RH56DF_适配法兰.stl"
FLANGE_LINK = "rh56df_adapter_flange"
FLANGE_SCALE = "0.001 0.001 0.001"

# 法兰局部几何(实测,米):厚 25mm,局部原点在两段交界处。
#   底面 Z=-0.015(圆形缺一块)贴 link8 输出面 —— 盘段 15mm;
#   顶面 Z=+0.010 贴灵巧手底座 —— 凸台段 10mm。
# link8 有两层顶面:主板 Z=+0.002、最外缘凸台 Z=+0.012(实测)。法兰底是实心 Ø40 盘,
# 坐在最外缘 Z=+0.012 上。法兰底面在其局部 Z=-0.015,故抬 0.012+0.015=0.027 使底面贴合。
FLANGE_MOUNT_XYZ = "0 0 0.027"   # link8 -> 法兰:底面(局部-15mm)落在 link8 外缘面 Z=+0.012
FLANGE_MOUNT_RPY = "0 0 0"

# 手基座:FK 实测其局部 base 系手指已沿 +Z 伸出、安装面法向 -Z。故 rpy 用单位即让手指
# 沿法兰输出轴 +Z、安装面朝下贴法兰顶面。放在法兰顶面(法兰局部 Z=+0.010)。clocking 待目视微调。
MOUNT_XYZ = "0 0 0.010"        # 法兰 -> 手基座:落在法兰顶面
MOUNT_RPY = "0 0 0"            # 单位:手指沿 +Z 输出轴,安装面(-Z)贴合法兰顶面


def abspath_meshes(root, base):
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        if not fn:
            continue
        if fn.startswith("package://"):
            pkg_path = fn.removeprefix("package://")
            pkg, _, rel = pkg_path.partition("/")
            p = REPO / "assets" / pkg / rel
        else:
            p = Path(fn)
        if not p.is_absolute():
            p = (base / fn).resolve()
        mesh.set("filename", str(p))


def link_names(root):
    return {l.get("name") for l in root.findall("link")}


def child_links(root):
    return {j.find("child").get("link") for j in root.findall("joint")
            if j.find("child") is not None}


nero_root = ET.parse(NERO).getroot()
insp_root = ET.parse(INSP).getroot()
abspath_meshes(nero_root, NERO.parent)
abspath_meshes(insp_root, INSP.parent)

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

# 适配法兰 link:CAD 顶点已在装配系,挂到 link8 用单位变换。视觉+碰撞同一 STL。
flange = ET.SubElement(robot, "link", {"name": FLANGE_LINK})
for tag in ("visual", "collision"):
    node = ET.SubElement(flange, tag)
    ET.SubElement(node, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geom = ET.SubElement(node, "geometry")
    ET.SubElement(geom, "mesh", {"filename": str(FLANGE_STL), "scale": FLANGE_SCALE})

jf = ET.SubElement(robot, "joint", {"name": "link8_to_flange", "type": "fixed"})
ET.SubElement(jf, "parent", {"link": HAND_PARENT})
ET.SubElement(jf, "child", {"link": FLANGE_LINK})
ET.SubElement(jf, "origin", {"xyz": FLANGE_MOUNT_XYZ, "rpy": FLANGE_MOUNT_RPY})

# 手掌基座接在法兰上(MOUNT_* 待目视标定)
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
