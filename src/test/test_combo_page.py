#!/usr/bin/env python3
"""src/test/test_combo_page.py — 合体页(实时 Live)的结构性测试。

这些是**静态**测试:读源文件和 URDF 产物,不起服务、不碰硬件。环境里没有浏览器,
所以浏览器端的东西只能这么锁 —— 但锁住的都是"错了会静默坏掉"的那类,而不是
"错了立刻报错"的那类。后者不用测,前者测不到就会在页面上以"看起来没加载"的形式出现。

    python3 -m pytest src/test/test_combo_page.py
"""
from __future__ import annotations

import json
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))
REPO = SIM.parent

WEB = SIM / "web"
COMBO = REPO / "assets/viz/combo"
COMBO_URDF = COMBO / "nero_inspire_right_viz.urdf"
SRC_URDF = REPO / "assets/assembled/nero_inspire_right.urdf"

_INDEX = (WEB / "index.html").read_text("utf-8")
_URDF_VIEW = (WEB / "urdf_view.js").read_text("utf-8")
_COMBO3D = (WEB / "combo3d.js").read_text("utf-8")
_HAND3D = (WEB / "hand3d.js").read_text("utf-8")
_APP = (SIM / "app_web.py").read_text("utf-8")

ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]
HAND_DRIVEN = ["right_thumb_1_joint", "right_thumb_2_joint",
               "right_index_1_joint", "right_middle_1_joint",
               "right_ring_1_joint", "right_little_1_joint"]


def _gltf_json(p: Path) -> dict:
    raw = p.read_bytes()
    magic, ver, total = struct.unpack("<4sII", raw[:12])
    assert magic == b"glTF" and ver == 2, f"{p.name}: 不是 glTF 2.0"
    assert total == len(raw), f"{p.name}: header 声明 {total} 实际 {len(raw)}"
    jl, jt = struct.unpack("<I4s", raw[12:20])
    assert jt == b"JSON", f"{p.name}: 第一个 chunk 不是 JSON"
    return json.loads(raw[20:20 + jl])


def _mesh_bbox(p: Path) -> list[float]:
    """glb 的 POSITION 包围盒尺寸。用来判单位(米 vs 毫米)。"""
    g = _gltf_json(p)
    lo = [1e18] * 3
    hi = [-1e18] * 3
    for m in g["meshes"]:
        for pr in m["primitives"]:
            a = g["accessors"][pr["attributes"]["POSITION"]]
            for i in range(3):
                lo[i] = min(lo[i], a["min"][i])
                hi[i] = max(hi[i], a["max"][i])
    return [hi[i] - lo[i] for i in range(3)]


# ------------------------------------------------------------ mesh scale
# ⚠ 这一组是本文件最重要的部分。它对应一个**实际发生过**的 bug:页面上"只看到法兰,
# 看不到臂和手"。根因不是 mesh 没加载 —— 19 个全加载成功了 —— 而是查看器漏读
# `<mesh scale>`,把毫米单位的法兰按 39.5 **米** 渲染,`_frameCamera()` 按整体
# 包围盒取景,相机被撑到 77m 外,0.75m 的臂只剩 1.2% 画面宽。
# 症状看着像"资源缺失",查起来会去翻加载器和静态挂载,而那两处都是好的。
def test_urdf_view_reads_mesh_scale():
    """查看器必须读 mesh scale。删掉它 = 装配页只剩一个放大千倍的法兰。"""
    assert "parseScale" in _URDF_VIEW, \
        "urdf_view.js 必须有 parseScale —— 漏读 scale 会让毫米单位的 mesh 放大 1000×"
    assert "o.scale.set(" in _URDF_VIEW, \
        "解析了 scale 但没 apply 到 mesh 上,等于没读"
    # 缺省必须是 1 1 1(URDF 规范),不能是 0 —— scale=0 会让 mesh 塌成一个点
    assert '"1 1 1"' in _URDF_VIEW, "scale 缺省值该是 URDF 规范的 1 1 1"
    assert "!== 0" in _URDF_VIEW, \
        "要挡 scale=0:那会把 mesh 塌成点,而且 three.js 不报错"


def test_scaled_mesh_still_needs_scale_in_product():
    """产物必须**保留** scale 属性,不能在构建时被丢掉。

    build_combo_viz 只改 filename,scale 原样带过来。哪天有人"顺手清理"掉它,
    查看器读了也没用 —— 两边任缺一个,法兰就回到 39.5m。
    """
    src_scaled = {m.get("filename").split("/")[-1]: m.get("scale")
                  for m in ET.parse(SRC_URDF).getroot().iter("mesh")
                  if m.get("scale")}
    assert src_scaled, "源装配 URDF 本来就该有带 scale 的 mesh(法兰是毫米单位)"
    out_scaled = {m.get("filename").split("/")[-1]: m.get("scale")
                  for m in ET.parse(COMBO_URDF).getroot().iter("mesh")
                  if m.get("scale")}
    for name, sc in src_scaled.items():
        stem = Path(name).stem
        hit = next((v for k, v in out_scaled.items() if Path(k).stem == stem), None)
        assert hit == sc, f"{stem} 的 scale 在产物里丢了或变了:源={sc} 产物={hit}"


def test_mesh_units_are_consistent_or_scaled():
    """每个 mesh 要么本身是米,要么带 scale 折回米。

    判据:折算后的最大边长必须 < 1m。装配体最大的件是 link2(0.26m),1m 有余量
    但足以挡住 1000× 的单位错。这条是**数据**测试 —— 换了新 mesh 也能挡住。
    """
    bad = []
    for link in ET.parse(COMBO_URDF).getroot().findall("link"):
        for vis in link.findall("visual"):
            m = vis.find("geometry/mesh")
            if m is None:
                continue
            sc = [float(x) for x in (m.get("scale") or "1 1 1").split()]
            dim = _mesh_bbox(COMBO / m.get("filename"))
            scaled = max(d * s for d, s in zip(dim, sc))
            if scaled >= 1.0:
                bad.append((link.get("name"), m.get("filename"), round(scaled, 2)))
    assert not bad, f"折算后仍 ≥1m,单位可疑(会带跑相机取景): {bad}"


# ------------------------------------------------------------ URDF 产物
def test_combo_urdf_has_all_13_driven_joints():
    """13 个驱动关节必须都在。缺一个 = 3D 里那部分不动,**且页面不报错**。"""
    names = {j.get("name") for j in ET.parse(COMBO_URDF).getroot().findall("joint")}
    missing = [n for n in ARM_JOINTS + HAND_DRIVEN if n not in names]
    assert not missing, f"装配 URDF 缺关节: {missing}"


def test_combo_urdf_mesh_paths_are_browser_resolvable():
    """mesh 路径必须能被浏览器顺着静态挂载取到。

    ⚠ 加载失败是**静默**的:_loadMesh 的 404 分支只 console.warn 然后 resolve(false)。
    所以路径错的症状是"模型空着但页面正常",查起来很费劲。四类都挡掉。
    """
    bad = []
    for m in ET.parse(COMBO_URDF).getroot().iter("mesh"):
        fn = m.get("filename", "")
        if fn.startswith("/") or (len(fn) > 1 and fn[1] == ":"):
            bad.append((fn, "绝对路径,浏览器取不到"))
        elif ".." in fn:
            bad.append((fn, "逃出静态挂载根"))
        elif "://" in fn:
            bad.append((fn, "package:// 之类的 URI,浏览器不解析"))
        elif not fn.endswith(".glb"):
            bad.append((fn, "已 vendor 的 GLTFLoader 只吃 glb"))
        elif not (COMBO / fn).is_file():
            bad.append((fn, "文件不存在"))
    assert not bad, f"mesh 路径有问题: {bad}"


def test_combo_urdf_origins_match_source():
    """产物的关节 origin 必须和源装配 URDF 逐位一致。

    构建脚本只该改 mesh 路径和删 collision。碰到 origin 的话几何就错了,而**看起来
    仍然像个机器人** —— 这种错不会报,只会让手的位置差几毫米到几厘米。
    实际踩过一次:源 URDF 13:44 重新生成(joint8 和 link8_to_flange 各差 1mm),
    而产物还是旧的。所以这条测试同时也是"产物过期"的检测。
    """
    def load(p):
        out = {}
        for j in ET.parse(p).getroot().findall("joint"):
            o = j.find("origin")
            out[j.get("name")] = (
                j.get("type"), j.find("parent").get("link"),
                j.find("child").get("link"),
                (o.get("xyz") if o is not None else None),
                (o.get("rpy") if o is not None else None))
        return out
    src, out = load(SRC_URDF), load(COMBO_URDF)
    assert set(src) == set(out), \
        f"关节集合不一致,产物过期了 —— 重跑 build_combo_viz.py。" \
        f"源多: {sorted(set(src)-set(out))} 产物多: {sorted(set(out)-set(src))}"
    diff = {k: (src[k], out[k]) for k in src if src[k] != out[k]}
    assert not diff, f"origin/父子关系不一致,产物过期了 —— 重跑 build_combo_viz.py: {diff}"


def test_combo_urdf_has_no_collision():
    """collision 该在构建时删掉:浏览器不渲染它,留着只是让 URDF 大一倍。"""
    n = len(ET.parse(COMBO_URDF).getroot().findall(".//collision"))
    assert n == 0, f"产物里还有 {n} 个 collision,构建时该删掉"


# ------------------------------------------------------------ mimic 单一真源
def test_combo3d_imports_mimic_instead_of_copying():
    """合体页的 mimic 必须从 hand3d.js import,不能抄。

    已经有 src/hand_rerun.py 和 web/hand3d.js 两份要同步了。抄第三份的话症状是
    "合体页手指第二节不弯,而单独的手页正常" —— 而且只在改过 mimic 之后才出现,
    很难联想到是拷贝没同步。所以直接查那几个系数有没有出现在 combo3d.js 里。
    """
    assert "from \"./hand3d.js\"" in _COMBO3D, "combo3d.js 该从 hand3d.js import"
    for lit in ("1.334", "0.667", "1.06399", "0.04545"):
        assert lit not in _COMBO3D, \
            f"combo3d.js 里出现了 mimic 系数 {lit} —— 那是抄的,该 import"
    for name in ("export const MIMIC", "export const DRIVEN",
                 "export function handJointMap"):
        assert name in _HAND3D, f"hand3d.js 该 {name}(combo3d 依赖它)"


def test_driven_joint_order_is_identical():
    """两处的驱动关节顺序必须一致,否则角度会装到别的手指上。

    顺序错了不会报错,只会让手做出别的姿势 —— 看起来像"重定向不准"。
    """
    import re
    m = re.search(r"export const DRIVEN = \[(.*?)\]", _HAND3D, re.S)
    assert m, "hand3d.js 里找不到 DRIVEN"
    got = re.findall(r'["\']([a-z0-9_]+)["\']', m.group(1))
    assert got == HAND_DRIVEN, f"DRIVEN 顺序变了: {got}"


# ------------------------------------------------------------ 页面行为
def test_combo_sliders_use_distinct_ids():
    """合体页滑块用独立 id 前缀,不复用调试页那两组。

    复用同 id 的后果:两页都初始化过之后 $() 命中第一个 —— 拖合体页的滑块改的是
    调试页那份值,而 3D 读的是自己这份。症状是"拖了没反应",极难查。
    """
    assert 'id="cbas_' in _INDEX and 'id="cbhs_' in _INDEX, \
        "合体页滑块该用 cbas_/cbhs_ 前缀"
    # 调试页那两组仍在,且没被合体页顶掉
    assert 'id="as_' in _INDEX and 'id="hs_' in _INDEX, \
        "调试页的 as_/hs_ 滑块不该被改动"


def test_switch_mode_tears_down_every_previous_mode():
    """切页必须按"从哪来"显式拆上一个模式。

    三个页面共用 app_web 里的 _hand / _arm **单例**会话。原来只在切回「回放」时才
    清理,于是臂页→手页会把 CAN 会话留着:手页显示"离线",臂其实还被我们占着,
    松灵客户端也接不回去。四条来路都要有对应的拆解。
    """
    i = _INDEX.index("function switchMode")
    body = _INDEX[i:i + 3000]
    for prev, fn in (('prev === "live"', "leaveCombo"),
                     ('prev === "hand"', "stopHandDebug"),
                     ('prev === "arm"', "stopArmDebug"),
                     ('prev === "replay"', "stopLive")):
        assert prev in body and fn in body, f"switchMode 里缺 {prev} → {fn}() 的拆解"


def test_combo_page_does_not_use_ros_bridge():
    """合体页只能走 console 端点,不能碰 ROS bridge 的 /api/command。

    bridge(ros_joint_writer)会抢 arm_console 已独占的 can0 和 hand_console 已独占的
    /dev/ttyUSB0 —— 同一条通道两个写者,后果不是报错而是**互相覆盖**。
    """
    i = _INDEX.index("async function cbDispatch")
    body = _INDEX[i:i + 2500]
    assert '"/api/arm/command"' in body and '"/api/hand/command"' in body, \
        "合体页下发该走两个 console 端点"
    assert '"/api/command"' not in body, \
        "合体页不能走 ROS bridge 的 /api/command(会和 console 抢通道)"
    # 老 Live 那条控制条在合体页永远不显示(它整条都走 bridge)
    j = _INDEX.index("function switchMode")
    assert '$("liveBar").style.display = "none"' in _INDEX[j:j + 3000], \
        "老 liveBar 走 ROS bridge,合体页必须永久隐藏它"


def test_linked_dispatch_sends_two_separate_commands():
    """联动 = 并发发**两条**,不合成一条。

    后端本来就是两个独立会话、两条通道(CAN / RS485),没有"同时到达"这回事。
    硬合成一条只会在 app_web 里多一层假原子性 —— 一边成功一边被 409 拒的时候
    更难说清到底动了什么。
    """
    i = _INDEX.index("async function cbDispatch")
    body = _INDEX[i:i + 2500]
    assert body.count("jobs.push") == 2, "该有两条独立的下发(臂一条、手一条)"
    assert "Promise.all" in body, "两条通道独立,该并发发而不是串行等"
    assert "bad.push" in body and "ok.push" in body, \
        "要分别记成功/失败,不能只报一个总的成败"


def test_partial_send_is_reported_not_silent():
    """联动时只有一边可用 → 必须写明"本次只发 X",不能静默发一半。

    静默发一半是最坏的:界面显示"已下发",实际只有手动了。而且"没接入"和
    "没使能"要分开说 —— 那是两个完全不同的动作要做。
    """
    i = _INDEX.index("function applyComboFlags")
    body = _INDEX[i:i + 2500]
    assert "联动只有一边可用" in body, "缺『只发一边』的显式提示"
    assert "本次只发" in body, "要说明本次实际发给了谁"
    assert "臂未使能" in body and "臂未接入" in body, \
        "『未使能』和『未接入』要分开提示"


def test_arm_defaults_to_mock_on_combo_page():
    """合体页的臂**默认 mock**,和机械臂调试页一致(手那边默认真机)。

    7 自由度工业臂的伤害量级不同,不做"连上就能动"。
    """
    i = _INDEX.index('id="cbArmMock"')
    assert "checked" in _INDEX[i:i + 120], "合体页臂的 mock 复选框该默认勾选"
    j = _INDEX.index('id="cbHandMock"')
    assert "checked" not in _INDEX[j:j + 120], "手默认真机,不该勾 mock"


def test_estop_requires_confirm_and_warns_about_falling():
    """急停必须先 confirm,且提示"会缓慢下落" —— 它不是定格。

    官方协议:急停后全部关节进阻尼模式,无关节抱闸,臂会缓慢下落。
    """
    i = _INDEX.index('$("cbEstop").onclick')
    body = _INDEX[i:i + 700]
    assert "confirm(" in body, "急停不能一点就发"
    assert "下落" in body, "确认框要说明臂会缓慢下落,不是定格"


def test_app_mounts_combo_assets():
    assert "/combo_assets" in _APP, "app_web 该挂 /combo_assets"
    assert "build_combo_viz.py" in _APP, "该注明产物怎么生成"


def main() -> int:
    tests = [(k, v) for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  ok   {name}")
        except Exception as e:                                     # noqa: BLE001
            failed.append((name, e))
            print(f"  FAIL {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} 通过")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
