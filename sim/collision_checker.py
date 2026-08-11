"""拇指-食指自碰撞几何检查器(第 3 步)

用真实 URDF 几何 + FCL 网格碰撞,替代 hand_pose.FEASIBLE 的一维压缩表。

与旧表的区别:
  - 旧表: T = max(yaw_raw, pitch_raw) 一维索引,3 个测量点线性插值,域外取常数
  - 本模块: 从 URDF 读 joint origin/axis/mimic,正运动学后做 mesh-mesh 碰撞

用法:
    from collision_checker import ThumbIndexChecker
    ck = ThumbIndexChecker()
    r = ck.check([thumb_yaw, thumb_pitch, index, middle, ring, little])  # 弧度
    print(r.feasible, r.margin_mm)   # margin 仅 feasible 时有意义
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import NamedTuple

import numpy as np

URDF_PATH = Path(__file__).resolve().parent.parent / "assets/hand/urdf/inspire_hand_right.urdf"

THUMB_LINKS = ("right_thumb_1", "right_thumb_2", "right_thumb_3", "right_thumb_4")
INDEX_LINKS = ("right_index_1", "right_index_2")

# 顺序与 hand_pose.HAND_JOINTS 一致
DRIVEN_JOINTS = ("right_thumb_1_joint", "right_thumb_2_joint", "right_index_1_joint",
                 "right_middle_1_joint", "right_ring_1_joint", "right_little_1_joint")


class CollisionResult(NamedTuple):
    """margin_mm:最小间距(mm),仅 feasible=True 时有意义;碰撞时恒为 0.0。

    **不提供穿透深度** —— FCL 的 mesh-mesh `penetration_depth` 实测不可用:
    2026-08-11 用已知重叠的方块验证,重叠 0.5mm 和 10mm 都报 20mm(= 方块边长)。
    要衡量"碰多深",应在关节角上二分退到刚好脱离,而不是读这个字段。
    布尔判定(`fcl.collide` 的返回)是可信的。

    out_of_limit:超出 URDF 自身限位的关节名。**非空时本结果不可信** ——
    网格被转到 URDF 声称到不了的角度,算出来的是虚构构型(HAND_SAFETY_PLAN 陷阱二)。
    """
    feasible: bool
    margin_mm: float
    contact_point: tuple[float, float, float] | None
    pair: tuple[str, str] | None
    out_of_limit: tuple[str, ...] = ()


def _rpy_to_mat(rpy: np.ndarray) -> np.ndarray:
    """URDF 的 rpy 是固定轴 X-Y-Z 外旋,等价于 Rz @ Ry @ Rx。"""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _axis_angle_to_mat(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues 公式。axis 不要求已归一化。"""
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    x, y, z = axis / n
    c, s = np.cos(angle), np.sin(angle)
    C = 1.0 - c
    return np.array([
        [x * x * C + c,     x * y * C - z * s, x * z * C + y * s],
        [x * y * C + z * s, y * y * C + c,     y * z * C - x * s],
        [x * z * C - y * s, y * z * C + x * s, z * z * C + c],
    ])


def _xyz(node, attr: str, default=(0.0, 0.0, 0.0)) -> np.ndarray:
    if node is None or node.get(attr) is None:
        return np.array(default, dtype=float)
    return np.array([float(v) for v in node.get(attr).split()], dtype=float)


def _mat(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_to_mat(rpy)
    T[:3, 3] = xyz
    return T


class _Joint:
    __slots__ = ("name", "type", "parent", "child", "origin", "axis",
                 "lower", "upper", "mimic_src", "mimic_k", "mimic_off")

    def __init__(self, el: ET.Element):
        self.name = el.get("name")
        self.type = el.get("type")
        self.parent = el.find("parent").get("link")
        self.child = el.find("child").get("link")
        o = el.find("origin")
        self.origin = _mat(_xyz(o, "xyz"), _xyz(o, "rpy"))
        self.axis = _xyz(el.find("axis"), "xyz", (1.0, 0.0, 0.0))
        lim = el.find("limit")
        self.lower = float(lim.get("lower")) if lim is not None and lim.get("lower") else None
        self.upper = float(lim.get("upper")) if lim is not None and lim.get("upper") else None
        m = el.find("mimic")
        self.mimic_src = m.get("joint") if m is not None else None
        self.mimic_k = float(m.get("multiplier", 1.0)) if m is not None else 1.0
        self.mimic_off = float(m.get("offset", 0.0)) if m is not None else 0.0

    def transform(self, q: float) -> np.ndarray:
        if self.type in ("fixed",):
            return self.origin
        R = np.eye(4)
        R[:3, :3] = _axis_angle_to_mat(self.axis, q)
        return self.origin @ R


class URDFModel:
    """只解析本模块需要的部分:joint 链 + link 的 collision mesh。"""

    def __init__(self, path: Path = URDF_PATH):
        self.path = Path(path)
        root = ET.parse(self.path).getroot()
        self.joints = {j.get("name"): _Joint(j) for j in root.findall("joint")}
        self.parent_joint = {j.child: j for j in self.joints.values()}
        self.collisions: dict[str, list[tuple[np.ndarray, Path]]] = {}
        for link in root.findall("link"):
            items = []
            for col in link.findall("collision"):
                mesh = col.find("geometry/mesh")
                if mesh is None:
                    continue
                o = col.find("origin")
                items.append((_mat(_xyz(o, "xyz"), _xyz(o, "rpy")),
                              self._resolve(mesh.get("filename"))))
            if items:
                self.collisions[link.get("name")] = items

    def _resolve(self, filename: str) -> Path:
        for prefix in ("package://", "file://"):
            if filename.startswith(prefix):
                filename = filename[len(prefix):]
        return (self.path.parent / filename).resolve()

    def resolve_q(self, driven: dict[str, float]) -> dict[str, float]:
        """把主动关节角展开到全部关节(处理 URDF 的 mimic)。"""
        q = dict(driven)
        for _ in range(len(self.joints)):  # 迭代到收敛,mimic 可能链式依赖
            changed = False
            for name, j in self.joints.items():
                if j.mimic_src and name not in q and j.mimic_src in q:
                    q[name] = q[j.mimic_src] * j.mimic_k + j.mimic_off
                    changed = True
            if not changed:
                break
        return q

    def link_pose(self, link: str, q: dict[str, float]) -> np.ndarray:
        """从 link 沿 parent 链回溯到根,累乘得到 link 在根坐标系的位姿。"""
        T = np.eye(4)
        cur = link
        seen = set()
        while cur in self.parent_joint:
            if cur in seen:
                raise ValueError(f"URDF 链有环: {cur}")
            seen.add(cur)
            j = self.parent_joint[cur]
            T = j.transform(q.get(j.name, 0.0)) @ T
            cur = j.parent
        return T


class ThumbIndexChecker:
    """拇指 4 段 × 食指 2 段 = 8 对网格碰撞检查。"""

    def __init__(self, urdf: Path = URDF_PATH,
                 thumb_links=THUMB_LINKS, index_links=INDEX_LINKS):
        import fcl
        import trimesh
        self._fcl = fcl
        self.model = URDFModel(urdf)
        self.thumb_links = tuple(thumb_links)
        self.index_links = tuple(index_links)
        self._geom: dict[str, list[tuple[np.ndarray, object]]] = {}
        for link in self.thumb_links + self.index_links:
            entries = self.model.collisions.get(link)
            if not entries:
                raise ValueError(f"link {link} 没有 collision mesh")
            built = []
            for col_origin, mesh_path in entries:
                mesh = trimesh.load(mesh_path, force="mesh")
                built.append((col_origin, self._bvh(mesh)))
            self._geom[link] = built
        # 覆盖 URDF 的 mimic 系数为标定值(2026-08-11)
        self.model.joints["right_thumb_3_joint"].mimic_k = _MIMIC_K3_CALIBRATED

    def _bvh(self, mesh):
        """python-fcl 的 BVHModel 必须走 begin/add/end,带参构造会 segfault。"""
        fcl = self._fcl
        m = fcl.BVHModel()
        m.beginModel(len(mesh.vertices), len(mesh.faces))
        m.addSubModel(np.asarray(mesh.vertices, dtype=np.float64),
                      np.asarray(mesh.faces, dtype=np.int32))
        m.endModel()
        return m

    def _objects(self, link: str, q: dict[str, float]) -> list:
        fcl = self._fcl
        T_link = self.model.link_pose(link, q)
        out = []
        for col_origin, bvh in self._geom[link]:
            T = T_link @ col_origin
            tf = fcl.Transform(T[:3, :3], T[:3, 3])
            out.append(fcl.CollisionObject(bvh, tf))
        return out

    def check(self, q6, contact_detail: bool = True) -> CollisionResult:
        """q6: 弧度,顺序同 DRIVEN_JOINTS(= hand_pose.HAND_JOINTS)。"""
        fcl = self._fcl
        q6 = np.asarray(q6, dtype=float).ravel()
        if q6.size != len(DRIVEN_JOINTS):
            raise ValueError(f"需要 {len(DRIVEN_JOINTS)} 个角度,收到 {q6.size}")
        q = self.model.resolve_q(dict(zip(DRIVEN_JOINTS, q6.tolist())))
        oob = tuple(sorted(
            n for n, v in q.items()
            if (self.model.joints[n].lower is not None and v < self.model.joints[n].lower - 1e-6)
            or (self.model.joints[n].upper is not None and v > self.model.joints[n].upper + 1e-6)))

        thumb_objs = {l: self._objects(l, q) for l in self.thumb_links}
        index_objs = {l: self._objects(l, q) for l in self.index_links}

        gap = float("inf")
        gap_pair = None
        for tl, tobjs in thumb_objs.items():
            for il, iobjs in index_objs.items():
                for a in tobjs:
                    for b in iobjs:
                        req = fcl.CollisionRequest(enable_contact=contact_detail,
                                                   num_max_contacts=8)
                        res = fcl.CollisionResult()
                        if fcl.collide(a, b, req, res):
                            pos = None
                            if res.contacts:
                                pos = tuple(float(v) for v in res.contacts[0].pos)
                            return CollisionResult(False, 0.0, pos, (tl, il), oob)
                        dreq = fcl.DistanceRequest()
                        dres = fcl.DistanceResult()
                        d = fcl.distance(a, b, dreq, dres)
                        if d < gap:
                            gap, gap_pair = d, (tl, il)
        return CollisionResult(True, gap * 1000.0, None, gap_pair, oob)


# ---------------------------------------------------------------------------
# raw <-> rad:标定值(2026-08-11 从实测碰撞边界反推)
# ---------------------------------------------------------------------------
_SPAN = {"right_thumb_1_joint": 1.246165,
         "right_thumb_2_joint": 0.525,  # 从 (300,225)/(450,52)/(600,0) 拟合,见 calibrate_thumb_chain.py
         "right_index_1_joint": 1.39626, "right_middle_1_joint": 1.39626,
         "right_ring_1_joint": 1.39626, "right_little_1_joint": 1.39626}

# thumb_3 mimic 系数也要改:标定值 1.075(URDF 原值 1.1425 会让 T=450 偏紧)
# 在 ThumbIndexChecker.__init__ 里覆盖
_MIMIC_K3_CALIBRATED = 1.075


def raw_to_rad(name: str, raw: float) -> float:
    """raw 0=闭合 / 1000=张开,六路都是 invert。"""
    return (1.0 - min(max(raw / 1000.0, 0.0), 1.0)) * _SPAN[name]


def raw6_to_rad6(raw6) -> np.ndarray:
    return np.array([raw_to_rad(n, r) for n, r in zip(DRIVEN_JOINTS, raw6)])


def _scan_index(ck: "ThumbIndexChecker", T: int, step: int = 10, others: int = 1000):
    """固定 yaw=pitch=T,从张开(1000)扫到闭合(0),返回 [(raw, feasible), ...]。"""
    out = []
    for raw in range(1000, -1, -step):
        r = ck.check(raw6_to_rad6([T, T, raw, others, others, others]),
                     contact_detail=False)
        out.append((raw, r.feasible))
    return out


def _first_contact_raw(ck: "ThumbIndexChecker", T: int, step: int = 10):
    """几何首次接触的 index raw(从张开侧逼近);全程不碰返回 None。

    **不用二分** —— 2026-08-11 扫描发现可行性沿 index 非单调
    (T=500 时 index=100 判可行,而 200 和 0 判碰撞),二分会掉进错的一侧。
    """
    for raw, ok in _scan_index(ck, T, step):
        if not ok:
            return raw
    return None


def _selftest() -> int:
    ck = ThumbIndexChecker()
    print(f"URDF: {ck.model.path}")
    print(f"关节 {len(ck.model.joints)} 个,带 collision 的 link {len(ck.model.collisions)} 个")
    print(f"检查对: {len(ck.thumb_links)} × {len(ck.index_links)}\n")

    # 全张开(raw 1000)必须不碰
    open_rad = raw6_to_rad6([1000] * 6)
    r = ck.check(open_rad)
    print(f"[全张开]  feasible={r.feasible}  间距={r.margin_mm:.2f}mm")
    ok = r.feasible
    if not ok:
        print(f"  ✗ 全张开都判碰撞,几何或 FK 有错。pair={r.pair}")

    # 旧表的三个测量点。旧表值是**实测堵转位置**,已过首次接触,
    # 所以几何的接触点应当比它更靠"张开"一侧(判据一,HAND_SAFETY_PLAN:133)。
    print("\n[旧表测量点] 沿 index 从张开侧找几何首次接触")
    for T, idx_lo in ((300, 225), (450, 52), (600, 0)):
        edge = _first_contact_raw(ck, T)
        rel = "几何更保守 ✓" if edge is None or edge > idx_lo else "几何更松 ✗ 危险"
        shown = "全程不碰" if edge is None else f"{edge}"
        oob = ck.check(raw6_to_rad6([T, T, 1000, 1000, 1000, 1000])).out_of_limit
        warn = f"  ⚠ 越界:{','.join(s.replace('right_','').replace('_joint','') for s in oob)}" if oob else ""
        print(f"  T={T:4d}  实测堵转={idx_lo:4d}  几何接触={shown:>8s}  {rel}{warn}")

    print("\n注:带 ⚠ 的行,拇指角已超出 URDF 限位,几何结论不可信;")
    print("   thumb_2 的 raw↔rad 有三个互斥值(URDF 0.48 / hand_pose 0.6 / xlsx 0.698),")
    print("   raw<312 全落在争议区。要第 2 步真机标定才能定。")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_selftest())
