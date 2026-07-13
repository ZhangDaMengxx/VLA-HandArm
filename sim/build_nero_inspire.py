"""构建 NERO(7-DoF) + inspire 手 的装配 URDF,并在 MuJoCo 里验证加载。

设计约束:
- 保持 inspire 的 link/joint 名不变(dex-retargeting 配置依赖 base/index_tip/... 等名字)。
- NERO 与 inspire 无命名冲突(NERO: world/base_link/link1-7;inspire 根为 'base')。
- 所有 mesh 路径改为绝对路径;MuJoCo 用 .stl/.obj 碰撞网格,.dae/.glb 视觉会被跳过。
- link7 -> inspire 'base' 的安装变换先用单位,加载成功后再按视觉标定。
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

REPO = Path(__file__).resolve().parents[1]
NERO = REPO / "assets/nero/nero_description.urdf"
INSP = REPO / "assets/inspire_hand/inspire_hand_right.urdf"
OUT = REPO / "sim/assets/nero_inspire_right.urdf"

MOUNT_XYZ = "0 0 0"   # 平贴法兰:手掌基座直接坐在法兰面上(略包住手腕,符合真实装法)
MOUNT_RPY = "0 0 0"   # 平贴法兰:手沿法兰伸出轴(link7 z),不掰歪


def abspath_meshes(root, base):
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        if not fn:
            continue
        fn = fn.replace("package://", "")
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

j = ET.SubElement(robot, "joint", {"name": "nero_to_hand", "type": "fixed"})
ET.SubElement(j, "parent", {"link": "link7"})
ET.SubElement(j, "child", {"link": hand_root})
ET.SubElement(j, "origin", {"xyz": MOUNT_XYZ, "rpy": MOUNT_RPY})

for el in list(insp_root):
    if el.tag == "mujoco":
        continue
    robot.append(el)

OUT.parent.mkdir(parents=True, exist_ok=True)
ET.ElementTree(robot).write(OUT, encoding="utf-8", xml_declaration=True)
print("wrote", OUT)

m = mujoco.MjModel.from_xml_path(str(OUT))
jnames = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
print(f"LOADED nq={m.nq} njnt={m.njnt} nbody={m.nbody} nmesh={m.nmesh}")
print("joints:", jnames)
