"""测意图解析:一句话 → skill_id + params。只用标准库 + PyYAML。

    python3 sim/skills/test_intent.py

重点不是「能听懂多少话」,而是**三条安全性质**:
  1. 不猜:咬得太近必须判 ambiguous,绝不随机命中(误命中运动技能是真风险)。
  2. 不放行:voice_enabled=false 的技能即使命中也拒,且理由要能区分于「听不懂」。
  3. 不硬塞:修饰词只落到技能声明过的参数上,且最终仍过 resolve_params 夹取。
"""
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import intent as I  # noqa: E402
from schema import get_registry, load_registry  # noqa: E402

PASS, FAIL = [], []


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'✓' if cond else '✗'} {name}" + (f" · {detail}" if detail else ""))


def hits(text: str, sid: str, reg=None, **kw) -> None:
    it = I.parse(text, reg or REG, **kw)
    check(f"{text!r} → {sid}", it.skill_id == sid,
          f"实为 {it.skill_id}({it.reason}) conf={it.confidence:.2f}")


def rejects(text: str, reason: str, reg=None, **kw) -> None:
    it = I.parse(text, reg or REG, **kw)
    check(f"{text!r} 拒绝({reason})", it.skill_id is None and it.reason == reason,
          f"实为 {it.skill_id}({it.reason})")


def load_yaml(text: str):
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False,
                                     encoding="utf-8") as f:
        f.write(text)
        p = f.name
    try:
        return load_registry(p)
    finally:
        Path(p).unlink(missing_ok=True)


REG = get_registry()

# ---------------------------------------------------------------- 精确
print("\n[1] 整句精确命中(不该被修饰词逻辑干扰)")
for alias, sid in [("急停", "estop"), ("回零位", "go_home"), ("握拳", "hand_close"),
                   ("准备就绪", "prepare_arm"), ("上电", "arm_enable")]:
    hits(alias, sid)
# 「慢一点」是 set_speed_slow 的别名 —— 整句说它必须命中技能,而不是被当成修饰词
it = I.parse("慢一点", REG)
check("『慢一点』整句 → set_speed_slow(不当修饰词)",
      it.skill_id == "set_speed_slow" and it.confidence == 1.0,
      f"{it.skill_id} conf={it.confidence}")
check("精确命中 confidence=1.0", I.parse("急停", REG).confidence == 1.0)
check("大小写/空格不敏感", I.parse("  GO_HOME ", REG).skill_id == "go_home")

# ---------------------------------------------------------------- 修饰词
print("\n[2] 修饰词只落到声明过的参数上")
it = I.parse("回零位慢一点", REG)
check("『回零位慢一点』→ go_home + duration 变长",
      it.skill_id == "go_home" and it.params.get("duration") == 7.5,
      f"{it.skill_id} {it.params}")
it = I.parse("回放深度示教快一点", REG)
check("『回放…快一点』→ speed 加倍",
      it.skill_id == "replay_rgbd_demo" and it.params.get("speed") == 2.0,
      f"{it.skill_id} {it.params}")
it = I.parse("握拳快一点", REG)
check("hand_close 没有可调参数 → 不硬塞,只提示",
      it.skill_id == "hand_close" and not it.params
      and any("没有可调快慢" in n for n in it.notes), f"{it.params} {it.notes}")
# 注意用完整修饰词:词表里是「慢一点」这类多字词,不含裸「快」「慢」——
# 裸字会把「慢速」这类别名咬坏,所以刻意不收。
it = I.parse("回零位慢一点快一点", REG)
check("同句快+慢 → 两个修饰都丢并提示",
      it.skill_id == "go_home" and "duration" not in it.params
      and any("同时说了快和慢" in n for n in it.notes), f"{it.params} {it.notes}")

# ---------------------------------------------------------------- 显式数值
print("\n[3] 显式数值:阿拉伯与中文都吃")
for text, want in [("回零位用8秒", 8.0), ("回零位用八秒钟", 8.0),
                   ("回零位十二秒", 12.0), ("回零位用3.5秒", 3.5)]:
    it = I.parse(text, REG)
    check(f"{text!r} → duration={want:g}",
          it.skill_id == "go_home" and it.params.get("duration") == want,
          f"{it.skill_id} {it.params}")
it = I.parse("回放深度示教两倍", REG)
check("『两倍』→ speed=2(中文数字)",
      it.skill_id == "replay_rgbd_demo" and it.params.get("speed") == 2.0,
      f"{it.skill_id} {it.params}")
it = I.parse("握拳用8秒", REG)
check("hand_close 无 duration 参数 → 忽略秒数并提示",
      it.skill_id == "hand_close" and not it.params
      and any("没有 duration" in n for n in it.notes), f"{it.params} {it.notes}")

# ---------------------------------------------------------------- 不猜
print("\n[4] 重名不猜:咬得太近必须交回去")
it = I.parse("手", REG)
check("『手』→ ambiguous(张开/握拳同分)",
      it.skill_id is None and it.reason == "ambiguous"
      and {c.skill_id for c in it.candidates} >= {"hand_open", "hand_close"},
      f"{it.reason} {[c.skill_id for c in it.candidates]}")
check("ambiguous 时不产出 skill_id", I.parse("手", REG).skill_id is None)
check("ambiguous 也带候选给人选", len(I.parse("手", REG).candidates) >= 2)
rejects("今天天气不错", "no_match")
rejects("", "no_match")
rejects("慢一点点一点", "no_match")   # 剥完只剩噪声,不该硬凑一个技能
it = I.parse("慢慢", REG)
check("只说修饰词 → 不命中并说明", it.skill_id is None
      and any("没说动作" in n for n in it.notes), f"{it.reason} {it.notes}")

# ---------------------------------------------------------------- 语音白名单
print("\n[5] voice_enabled=false 必须拒,且理由区分于『听不懂』")
REG_NV = load_yaml("""
version: 1
skills:
  - id: secret_move
    name: 禁语音动作
    aliases: ["禁语音动作", "保密动作"]
    kind: primitive
    action: {arm: [0,0,0,0,0,0,0], duration: 3.0}
    safety: {voice_enabled: false, need_confirm: true}
""")
rejects("禁语音动作", "not_voice_enabled", reg=REG_NV)
it = I.parse("禁语音动作", REG_NV)
check("拒绝理由说明是清单不许,不是听不懂",
      any("不允许语音触发" in n for n in it.notes), str(it.notes))
check("非语音路径(voice_only=False)可命中",
      I.parse("禁语音动作", REG_NV, voice_only=False).skill_id == "secret_move")
# 真清单里**只有** `_` 前缀的内部技能不许语音。
# 那些是分阶段手势的中间态(如 _hand_thumb_folded),是实现细节:放进语音池会和
# 真手势抢匹配分,而且单独说"拇指折叠"没有意义。原来这条断言写的是"全都允许语音",
# 2026-08-10 拆 hand_close 时引入了第一批内部技能,所以改成断言这条**规则**
# 而不是断言一个会随清单变的数字。
_nv = [s.id for s in REG if not s.safety.voice_enabled]
check("只有 `_` 前缀的内部技能不许语音",
      _nv and all(i.startswith("_") for i in _nv), f"不许语音的: {_nv}")
check("其余全部允许语音",
      len(REG.voice_skills()) == len(REG) - len(_nv))

# ---------------------------------------------------------------- 信封
print("\n[6] 调用信封:原话与置信度要带上(VLA 标注原料)")
env = I.parse("回零位慢一点", REG).envelope(source="voice", request_id="r1")
check("信封含 skill_id/params", env["skill_id"] == "go_home"
      and env["params"]["duration"] == 7.5, str(env))
check("信封带原话 transcript", env["transcript"] == "回零位慢一点")
check("信封带 confidence", isinstance(env["confidence"], float))
check("信封默认 confirmed=False(确认要显式给)", env["confirmed"] is False)
check("信封 source=voice", env["source"] == "voice")
check("request_id 透传", env["request_id"] == "r1")
try:
    I.parse("今天天气不错", REG).envelope()
    check("未命中时 envelope() 抛错", False, "居然生成了信封")
except ValueError:
    check("未命中时 envelope() 抛错", True)

# ---------------------------------------------------------------- 与 schema 衔接
print("\n[7] 参数最终仍过 resolve_params(夹取与语音限速不在本模块)")
spec = REG.get("go_home")
it = I.parse("回零位用99秒", REG)          # 99 超出 range [2,15]
p, notes = spec.resolve_params(it.params, via_voice=True)
check("intent 不夹取,原样交给 schema", it.params.get("duration") == 99.0,
      str(it.params))
check("schema 把 99 秒夹到上限 15", p["duration"] == 15.0
      and any("夹取" in n for n in notes), f"{p} {notes}")
spec_r = REG.get("replay_rgbd_demo")
it = I.parse("回放深度示教快一点", REG)      # speed 2.0,语音上限 1.0
p, notes = spec_r.resolve_params(it.params, via_voice=True)
check("语音路径 speed 被压到 max_speed=1.0", p["speed"] == 1.0
      and any("限速" in n for n in notes), f"{p} {notes}")
p2, _ = spec_r.resolve_params(it.params, via_voice=False)
check("非语音路径不压速(限速只针对语音)", p2["speed"] == 2.0, str(p2))
check("need_confirm 透传给前端", I.parse("回零位", REG).need_confirm is True)
check("estop 是唯一免确认项", I.parse("急停", REG).need_confirm is False)

# ---------------------------------------------------------------- 口语化
print("\n[8] 口语化说法(能多吃就多吃,吃不下也不能乱猜)")
for text, sid in [("帮我回一下零位", "go_home"), ("把手张开", "hand_open"),
                  ("请准备一下", "prepare_arm"), ("给我握拳", "hand_close"),
                  ("现在急停", "estop"), ("麻烦复位一下", "arm_reset")]:
    hits(text, sid)

# ---------------------------------------------------------------- 手势技能包
print("\n[9] 手势技能包:与技能**同池**打分,撞名必须判 ambiguous")
# 用合成的 PackTarget,不碰磁盘 —— 测试不该依赖用户 data/gestures 里正好有什么包
P_OK = I.PackTarget(path="OK.json", name="OK", frames=2)
P_WAVE = I.PackTarget(path="挥手.json", name="挥手", frames=30)
PACKS = [P_OK, P_WAVE]

it = I.parse("挥手", REG, packs=PACKS)
check("包名精确命中 → kind=gesture_pack",
      it.ok and it.kind == "gesture_pack" and it.pack_path == "挥手.json",
      f"{it.kind} {it.pack_path} {it.reason}")
check("命中包时 skill_id 为空(别让调用方误当技能发)", it.skill_id is None)
check("包默认要确认(它会真的动手)", it.need_confirm is True)
check("包也吃口语化说法",
      I.parse("播放挥手", REG, packs=PACKS).pack_path == "挥手.json",
      str(I.parse("播放挥手", REG, packs=PACKS).to_public()))
it = I.parse("挥手快一点", REG, packs=PACKS)
check("包没有可调参数 → 修饰词忽略并提示",
      it.pack_path == "挥手.json" and not it.params
      and any("没有可调参数" in n for n in it.notes), f"{it.params} {it.notes}")

# 撞名:包名和技能名一样 —— 必须 ambiguous,不能某一池悄悄赢
P_FIST = I.PackTarget(path="my/握拳.json", name="握拳", frames=5)
it = I.parse("握拳", REG, packs=[P_FIST])
check("包名撞技能名 → ambiguous(不猜)",
      not it.ok and it.reason == "ambiguous", f"{it.reason} {it.skill_id}")
kinds = {c.kind for c in it.candidates}
check("候选同时含技能与包两种 kind", kinds == {"skill", "gesture_pack"}, str(kinds))
check("撞名提示说清是命中了几个",
      any("同时命中" in n for n in it.notes), str(it.notes))

# 两个同名包(不同目录):候选必须带路径,否则用户永远点不出结果
P_A = I.PackTarget(path="a/招手.json", name="招手")
P_B = I.PackTarget(path="b/招手.json", name="招手")
it = I.parse("招手", REG, packs=[P_A, P_B])
check("两个同名包 → ambiguous", not it.ok and it.reason == "ambiguous", it.reason)
paths = sorted(c.skill_id for c in it.candidates)
check("候选用**路径**区分,不是只给名字",
      paths == ["a/招手.json", "b/招手.json"], str(paths))

env = I.parse("挥手", REG, packs=PACKS).envelope(source="voice", confirmed=True)
check("信封带 kind=gesture_pack", env["kind"] == "gesture_pack", str(env))
check("信封带 pack_path", env["pack_path"] == "挥手.json", str(env))
check("信封仍带原话(包也要留标注原料)", env["transcript"] == "挥手")

check("不传 packs 时行为不变(纯技能)",
      I.parse("握拳", REG).skill_id == "hand_close"
      and I.parse("挥手", REG).ok is False)
check("坏包(缺 path/name)不进池",
      I.parse("挥手", REG, packs=[I.PackTarget(path="", name="挥手")]).ok is False)

# ---------------------------------------------------------------- 汇总
print("\n" + "=" * 60)
print(f"通过 {len(PASS)} · 失败 {len(FAIL)}")
if FAIL:
    for n in FAIL:
        print(f"  ✗ {n}")
    raise SystemExit(1)
print("全部通过")
