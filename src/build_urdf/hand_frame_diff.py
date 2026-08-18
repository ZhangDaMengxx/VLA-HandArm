"""比较旧/新 inspire 手 URDF 的 base 系约定,解出两者之间的刚体变换。

做法:两份 URDF 的 link 名相同、手内部几何是同一只手,故对每个 link 分别在各自
base 系下做 q=0 的 FK,再逐 link 解 T_old_new = T_old_i @ inv(T_new_i)。
若所有 link 解出同一个变换 -> 差异纯粹是 base 系约定,可用它修挂接量。
"""
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path

D = Path(__file__).resolve().parents[2] / "assets/inspire_hand"
OLD = D / "inspire_hand_right.urdf.bak_dexurdf"
NEW = D / "inspire_hand_right.urdf"


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def T(xyz, rpy):
    M = np.eye(4)
    M[:3, :3] = rpy_to_R(*rpy)
    M[:3, 3] = xyz
    return M


def parse(path):
    root = ET.parse(path).getroot()
    links = [l.get("name") for l in root.findall("link")]
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0").split())] if o is not None else [0, 0, 0]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0").split())] if o is not None else [0, 0, 0]
        ax = j.find("axis")
        joints[j.get("name")] = dict(
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            xyz=xyz, rpy=rpy, type=j.get("type"),
            axis=[float(v) for v in ax.get("xyz").split()] if ax is not None else None)
    return root, links, joints


def fk(links, joints):
    """q=0 下每个 link 在 base 系的位姿。"""
    child_of = {v["child"]: (k, v) for k, v in joints.items()}
    roots = [l for l in links if l not in child_of]
    assert len(roots) == 1, roots
    out = {roots[0]: np.eye(4)}
    for _ in range(len(links)):
        for l in links:
            if l in out or l not in child_of:
                continue
            _, j = child_of[l]
            if j["parent"] in out:
                out[l] = out[j["parent"]] @ T(j["xyz"], j["rpy"])
    return out, roots[0]


ro, lo, jo = parse(OLD)
rn, ln, jn = parse(NEW)
print(f"old: {len(lo)} links {len(jo)} joints   new: {len(ln)} links {len(jn)} joints")
print("only in old:", sorted(set(lo) - set(ln)))
print("only in new:", sorted(set(ln) - set(lo)))

fo, root_o = fk(lo, jo)
fn_, root_n = fk(ln, jn)
print(f"root: old={root_o} new={root_n}")

common = [l for l in lo if l in fn_ and l in fo]
print(f"\n逐 link 解 T_old_new = T_old_i @ inv(T_new_i),共 {len(common)} 个 link:")
Ts = []
for l in common:
    M = fo[l] @ np.linalg.inv(fn_[l])
    Ts.append(M)
Ts = np.array(Ts)
med = np.median(Ts, axis=0)
spread = np.abs(Ts - med).max(axis=0)
np.set_printoptions(precision=6, suppress=True)
print("中位变换 T_old_new =\n", med)
print("各 link 解的最大离散(逐元素) =\n", spread)
print(f"\n平移离散 max = {spread[:3, 3].max() * 1000:.4f} mm")
print(f"旋转离散 max = {spread[:3, :3].max():.6f}")
