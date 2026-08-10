#!/usr/bin/env python3
"""sim/test_voice_combo_kind.py — 语音路 kind 三态的回归测试。

起因:页面上说「挥手」报
    识别请求失败:SyntaxError: Unexpected token 'I', "Internal S"... is not valid JSON
看着像前端 JSON 解析 bug,其实是 /api/voice/parse **回了 500 纯文本**
"Internal Server Error" —— 前端 r.json() 拿它去 parse,第一个字符 'I' 就炸。

根因:kind 从两态(skill / gesture_pack)长到三态(+ combo_pack)时,**五处**
分支还拿 `== "gesture_pack"` 当「是不是包」的判据,combo 包全落到错的那边。
它的 skill_id 是 None(包不在清单里,靠 pack_path 定位),于是
    console_targets(reg.get(None), reg) → targets() 首行 spec.kind → AttributeError

这个文件盯的**不是**那一次崩,是「以后再加第四种 kind 会不会又漏一处」。所以
断言写成「combo 包必须和 gesture 包一样被当成包」,而不是去对某个具体字符串。

⚠ 不碰硬件:parse/phrases 只读清单和磁盘;invoke 那条**故意不传 confirmed**,
  在确认闸就返回,一帧都不下发 —— 真机接着的时候也能安全跑。
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

SIM = Path(__file__).resolve().parent
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))

_T: list = []
_SKIP: list[str] = []


def test(fn):
    _T.append(fn)
    return fn


class Skip(Exception):
    """没有素材可测(沙箱里没包)。不算失败,但要在汇总里显式说出来 ——
    静默跳过会让「0 失败」和「什么都没测」长得一样。"""


_CLIENT = None


def _client():
    """起 app 但**不拉 console**。

    _console_ready() 直接读 _arm/_hand 两个全局(不走 _get_arm()),都是 None
    时报「未接入」就返回 —— 所以这里不会去抢 can0 / ttyUSB0。缓存成模块级是
    因为 import app_web 有点慢,而这些用例都只读。
    """
    global _CLIENT
    if _CLIENT is None:
        from fastapi.testclient import TestClient

        import app_web
        _CLIENT = TestClient(app_web.app)
    return _CLIENT


def _phrases(scope="all") -> list[dict]:
    return _client().get("/api/voice/phrases",
                         params={"scope": scope}).json().get("phrases", [])


def _combo() -> dict:
    """挑一个 combo 包。测试不自己造包 —— 会污染真的沙箱目录。"""
    for p in _phrases("all"):
        if p.get("kind") == "combo_pack":
            return p
    raise Skip("data/combos/ 里没有包")


@test
def test_targets_crashes_on_none_spec():
    """复现根因本身:spec=None 时 targets() 一定崩。

    这条不是「测 bug」,是把「reg.get(未知 id) 回 None,而 targets() 不收 None」
    这个契约钉住 —— 调用方必须自己保证 spec 非 None。将来若给 targets() 加了
    None 保护,这条会失败,那时要连带检查还有没有别处依赖「崩」来暴露问题。
    """
    from skills.console_exec import targets
    from skills.schema import get_registry

    reg = get_registry()
    assert reg.get(None) is None, "未知 skill_id 应回 None"
    try:
        targets(reg.get(None), reg)
    except AttributeError:
        return
    raise AssertionError("targets(None) 没崩 —— 契约变了,见 docstring")


@test
def test_parse_combo_returns_json_not_500():
    """**主回归**:说 combo 包的名字必须回 200 JSON。

    修之前这里是 500 + 纯文本 body,前端 r.json() 报「Unexpected token 'I'」。
    status 和「body 能被 json 解析」两件都断:只断 status 的话,下次若回了
    200 但 body 不是 JSON,一样漏过去。
    """
    p = _combo()
    r = _client().post("/api/voice/parse",
                       json={"text": p["name"], "scope": "all", "source": "test"})
    assert r.status_code == 200, f"回了 {r.status_code}: {r.text[:80]!r}"
    d = r.json()                                    # 不是 JSON 会在这里抛
    assert d.get("ok") is True, f"应解析成功: {d}"
    assert d.get("kind") == "combo_pack", f"kind 错: {d.get('kind')}"


@test
def test_parse_combo_reports_arm_and_hand():
    """combo 包的 devices 必须含 arm —— 确认框靠它显示「⚠ 会动臂」。

    报成只有 hand 的后果是风险提示**方向性错误**:人以为只动手就点了执行。
    """
    p = _combo()
    d = _client().post("/api/voice/parse",
                       json={"text": p["name"], "scope": "all"}).json()
    assert set(d.get("devices") or []) == {"arm", "hand"}, \
        f"应报 arm+hand,实际 {d.get('devices')}"


@test
def test_phrases_does_not_hardcode_gesture_kind():
    """「能说什么」里 combo 包不能显示成纯手势包。

    修之前 phrases 对所有包写死 kind="gesture_pack" + devices=["hand"]。
    这一栏是给人照着念的 —— 念之前看不出哪句会动臂。
    """
    p = _combo()                                    # 找得到就说明 kind 没写死
    assert set(p.get("devices") or []) == {"arm", "hand"}, \
        f"phrases 里应报 arm+hand,实际 {p.get('devices')}"


@test
def test_gesture_pack_still_hand_only():
    """对照组:手势包不能被顺手改成 arm+hand。

    上面几条都在放宽「什么算包」,很容易改过头 —— 手势包只动手,报成会动臂
    是反向的误导(让人白清一次场,久了就不信这个提示了)。
    """
    gs = [p for p in _phrases("hand") if p.get("kind") == "gesture_pack"]
    if not gs:
        raise Skip("data/gestures/ 里没有包")
    for p in gs:
        assert set(p.get("devices") or []) == {"hand"}, \
            f"手势包 {p['name']} 应只报 hand,实际 {p.get('devices')}"


@test
def test_invoke_routes_combo_to_pack_path():
    """combo 包必须被 voice_invoke 分流到 _voice_play_pack。

    ⚠ **故意不传 confirmed**:那样 _voice_play_pack 在确认闸就 409 返回,
    一帧都不下发 —— 真机接着的时候跑这条也不会动。用「被闸拒」证明「路由对了」,
    比真执行一次安全得多。

    修之前 voice_invoke 只认 gesture_pack,combo 包会掉进技能执行器,而它
    skill_id 是 None,于是报「查不到这条技能」—— 理由和真实原因(该走包那条路)
    完全不搭,排查时会去清单里找一个根本不存在的技能。
    """
    p = _combo()
    r = _client().post("/api/voice/invoke",
                      json={"kind": "combo_pack", "pack_path": p["skill_id"],
                            "skill_id": None, "transcript": p["name"]})
    assert r.status_code == 409, \
        f"应被确认闸拒(409),实际 {r.status_code}: {r.text[:120]!r}"
    msg = r.json().get("msg") or ""
    assert "确认" in msg, f"应是确认闸的理由,实际 {msg!r}"


# ---- 前端源码扫描 ----------------------------------------------------------
# 前端没有测试框架,但这个 bug 有一半在 index.html 里(五处中三处)。扫源码不
# 优雅,可它能挡住「后端修了前端忘了」这种最常见的半修状态。

@test
def test_frontend_has_no_bare_gesture_pack_predicate():
    """前端不该再有「拿 == gesture_pack 当『是不是包』」的裸判据。

    三处都得连带认 combo_pack:vcShowConfirm(决定显示格式)、vcRun(决定按
    JSON 还是 SSE 读响应)、phrases 列表(决定标签)。漏 vcRun 那处的症状最阴:
    拿 res.body.getReader() 去读一个普通 JSON 响应,进度栏一行不出,看着像
    「没执行」,而其实臂已经在动了。
    """
    src = (SIM / "web" / "index.html").read_text(encoding="utf-8")
    bare = re.findall(r'kind === "gesture_pack"(?!\s*\|\|)', src)
    assert not bare, (f"还有 {len(bare)} 处只认 gesture_pack —— "
                      "每处都要 `|| …combo_pack`,或先算出 isCombo")


@test
def test_backend_uses_pack_kinds_not_bare_string():
    """后端判「是不是包」必须走 PACK_KINDS,不许在**代码**里写裸字符串。

    ⚠ 只扫代码行,注释里出现 `== "gesture_pack"` 是允许的 —— 讲这个坑本身就
    得把它写出来。第一版没排除注释,结果误报在自己的解释注释上。
    """
    src = (SIM / "app_web.py").read_text(encoding="utf-8")
    bad = []
    for i, line in enumerate(src.splitlines(), 1):
        code = line.split("#", 1)[0]                 # 粗暴但够用:本文件没有含 # 的串
        if re.search(r'==\s*"gesture_pack"', code):
            bad.append(i)
    assert not bad, (f"这些行在代码里写了裸 == \"gesture_pack\": {bad} —— "
                     "改用 `in PACK_KINDS` / PACK_DEVICES[kind]")


@test
def test_frontend_pack_tables_match_backend():
    """前端抄的那两份常量要和后端一致。

    前端拿不到 Python 常量,只能抄。抄漏的后果:加了第三种包,后端认了前端不认
    —— 那种「一半生效」的状态最难查,因为解析成功、确认框也弹,只在读响应时
    静默走错分支。
    """
    from skills.intent import PACK_DEVICES, PACK_KINDS

    src = (SIM / "web" / "index.html").read_text(encoding="utf-8")
    m = re.search(r'VC_PACK_KINDS\s*=\s*\[([^\]]*)\]', src)
    assert m, "前端找不到 VC_PACK_KINDS"
    fe_kinds = set(re.findall(r'"([^"]+)"', m.group(1)))
    assert fe_kinds == set(PACK_KINDS), \
        f"kind 表不一致:前端 {sorted(fe_kinds)} vs 后端 {sorted(PACK_KINDS)}"

    m = re.search(r'VC_PACK_DEVICES\s*=\s*\{(.*?)\}\s*;', src, re.S)
    assert m, "前端找不到 VC_PACK_DEVICES"
    for kind, devs in PACK_DEVICES.items():
        km = re.search(rf'{kind}\s*:\s*\[([^\]]*)\]', m.group(1))
        assert km, f"前端 VC_PACK_DEVICES 缺 {kind}"
        assert set(re.findall(r'"([^"]+)"', km.group(1))) == set(devs), \
            f"{kind} 的设备表不一致:后端 {devs}"


def main() -> int:
    bad = 0
    for fn in _T:
        try:
            fn()
            print(f"  ok   {fn.__name__}")
        except Skip as e:
            _SKIP.append(f"{fn.__name__}: {e}")
            print(f"  skip {fn.__name__}: {e}")
        except Exception as e:                              # noqa: BLE001
            bad += 1
            print(f"  FAIL {fn.__name__}: {type(e).__name__}: {e}")
    ran = len(_T) - len(_SKIP)
    print(f"\n{ran - bad}/{ran} 通过" + (f",{len(_SKIP)} 跳过" if _SKIP else ""))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
