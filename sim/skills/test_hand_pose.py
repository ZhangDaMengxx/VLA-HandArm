#!/usr/bin/env python3
"""sim/skills/test_hand_pose.py — 手势规格层 + 手势表的单元测试。

纯 Python,不碰硬件。跑:
    /usr/bin/python3 sim/skills/test_hand_pose.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

import hand_pose as hp                                   # noqa: E402
from schema import get_registry                          # noqa: E402

_pass, _fail = 0, []


def check(label: str, cond: bool, extra: str = "") -> None:
    global _pass
    if cond:
        _pass += 1
        print(f"  ✓ {label}")
    else:
        _fail.append(label)
        print(f"  ✗ {label}" + (f"  ({extra})" if extra else ""))


def raises(label: str, fn, want: str = "") -> None:
    """fn 必须抛 PoseError,且报错文字含 want。"""
    try:
        fn()
    except hp.PoseError as e:
        check(label, want in str(e), f"报错是 {e!r},不含 {want!r}")
    except Exception as e:                                # noqa: BLE001
        check(label, False, f"抛了 {type(e).__name__} 而不是 PoseError: {e}")
    else:
        check(label, False, "没抛异常")


def raw_of(rad6) -> list[int]:
    return [hp.rad_to_raw(n, r) for n, r in zip(hp.HAND_JOINTS, rad6)]


# ---------------------------------------------------------------- 归一语义
print("\n[1] 归一量语义:0=张开, 1=实际能到的最闭")
check("n=0 → 全通道 raw 1000(张开)",
      raw_of(hp.resolve({"thumb": "open"})) == [1000] * 6)
r = hp.resolve({"thumb": [1.0, 1.0], "index": "closed", "middle": "closed",
                "ring": "closed", "pinky": "closed"})
check("四指 n=1 → raw 0", raw_of(r)[2:] == [0, 0, 0, 0])
check("拇指 yaw n=1 → raw 0(对掌位)", raw_of(r)[0] == 0)
# 拇指弯曲 URDF 上限 0.6 < span 0.698,所以 n=1 打不到 raw 0
check("拇指弯曲 n=1 → raw 141 而非 0(URDF 少 14% 行程)", raw_of(r)[1] == 141,
      f"实际 {raw_of(r)[1]}")
check("n=1 的弧度就是 EFF_HI",
      abs(hp.n_to_rad("right_thumb_2_joint", 1.0) - 0.6) < 1e-9)
check("EFF_HI = min(span, URDF上限)",
      hp.EFF_HI["right_thumb_2_joint"] == 0.6
      and hp.EFF_HI["right_index_1_joint"] == 1.39626)
check("缺的通道按张开补", raw_of(hp.resolve({"index": "closed"}))[3:] == [1000] * 3)
check("raw_to_n 是 n_to_rad+rad_to_raw 的逆(食指 n=0.5)",
      abs(hp.raw_to_n("right_index_1_joint",
                      hp.rad_to_raw("right_index_1_joint",
                                    hp.n_to_rad("right_index_1_joint", 0.5))) - 0.5) < 2e-3)

# ---------------------------------------------------------------- 错误输入
print("\n[2] 写错要报错,不静默夹取")
raises("未知四指状态名", lambda: hp.resolve({"index": "bent"}), "未知")
raises("未知拇指状态名", lambda: hp.resolve({"thumb": "curled"}), "未知")
raises("pose 未知键", lambda: hp.resolve({"palm": "flat"}), "未知键")
raises("拇指给 1 个值", lambda: hp.resolve({"thumb": [0.5]}), "2 个值")
raises("拇指给 3 个值", lambda: hp.resolve({"thumb": [0.5, 0.5, 0.5]}), "2 个值")
raises("n 超上界", lambda: hp.resolve({"index": 1.5}), "不在 [0,1]")
raises("n 负数", lambda: hp.resolve({"index": -0.1}), "不在 [0,1]")
raises("pose 不是映射", lambda: hp.resolve(["opposed"]), "必须是映射")
raises("四指给布尔", lambda: hp.resolve({"index": True}), "要状态名或")

# ---------------------------------------------------------------- 可行域
print("\n[3] 拇指-食指可行域")
check("T≤300 → 食指下界 225", hp.index_min_raw(300, 0) == 225
      and hp.index_min_raw(0, 0) == 225)
check("T≥600 → 无约束(下界 0)", hp.index_min_raw(600, 0) == 0
      and hp.index_min_raw(1000, 1000) == 0)
check("T=450 → 下界 52(实测点)", hp.index_min_raw(450, 0) == 52)
check("插值在 300..450 之间单调不增",
      all(hp.index_min_raw(t, 0) >= hp.index_min_raw(t + 25, 0)
          for t in range(300, 600, 25)))
check("有效变量是 max(yaw,pitch) 而非只看 yaw",
      hp.index_min_raw(0, 1000) == hp.index_min_raw(1000, 0) == 0)
check("两关节都收进来才堵", hp.index_min_raw(100, 100) == 225)
bad = hp.check_feasible(hp.resolve({"thumb": "opposed", "index": "closed"}))
check("对掌+食指满弯 → 判不可行", bad is not None and "互顶" in bad)
check("不可行原因给出差多少", bad is not None and "差 225" in bad, bad or "")
ok = hp.check_feasible(hp.resolve({"thumb": "opposed", "index": "limit"}))
check("对掌+食指 limit → 可行", ok is None, ok or "")
check("拇指躺平+食指满弯 → 可行(实测的非对角点)",
      hp.check_feasible(hp.resolve({"thumb": "open", "index": "closed"})) is None)

# ---------------------------------------------------------------- limit 推导
print("\n[4] limit 是推导状态,带防堵转余量")
lim = hp.resolve({"thumb": "opposed", "index": "limit"})
check("limit 落在干涉点 +LIMIT_MARGIN", raw_of(lim)[2] == 225 + hp.LIMIT_MARGIN,
      f"实际 raw {raw_of(lim)[2]}")
check("LIMIT_MARGIN > 0(表里 225 是卡住位,不是安全位)", hp.LIMIT_MARGIN > 0)
lim2 = hp.resolve({"thumb": "side", "index": "limit"})
check("换拇指姿态,limit 跟着变", raw_of(lim2)[2] != raw_of(lim)[2],
      f"{raw_of(lim2)[2]} vs {raw_of(lim)[2]}")
check("limit 在任何拇指姿态下都可行",
      all(hp.check_feasible(hp.resolve({"thumb": t, "index": "limit"})) is None
          for t in hp.THUMB_STATES))
check("中指 limit = 满弯(实测能到 raw 0)",
      raw_of(hp.resolve({"thumb": "open", "middle": "limit"}))[3] == 0)

# ---------------------------------------------------------------- 状态表来源
print("\n[5] 拇指状态是从实测清单项反算的,不是纸面推的")
check("opposed 反算自 hand_pinch [1.112, 0.600]",
      abs(hp.n_to_rad(hp.HAND_JOINTS[0], hp.THUMB_STATES["opposed"][0]) - 1.112) < 2e-3
      and abs(hp.n_to_rad(hp.HAND_JOINTS[1], hp.THUMB_STATES["opposed"][1]) - 0.600) < 2e-3)
check("folded 反算自 hand_close [1.0, 0.5]",
      abs(hp.n_to_rad(hp.HAND_JOINTS[0], hp.THUMB_STATES["folded"][0]) - 1.0) < 2e-3
      and abs(hp.n_to_rad(hp.HAND_JOINTS[1], hp.THUMB_STATES["folded"][1]) - 0.5) < 2e-3)
check("up = 对掌但不弯(点赞)", hp.THUMB_STATES["up"] == (1.0, 0.0))

# ---------------------------------------------------------------- 驱动表同步
print("\n[6] 抄来的表要和 inspire_hand 一致")
drift = hp.verify_against_driver()
check("RAW_MAP / HAND_LIMITS / HAND_JOINTS 无漂移", not drift,
      "; ".join(drift))

# ---------------------------------------------------------------- 清单集成
print("\n[7] 清单里的 pose 落到 action.hand")
reg = get_registry(reload=True)
pinch = reg.get("hand_pinch")
check("hand_pinch 有 pose 原文", isinstance(pinch.pose, dict)
      and pinch.pose.get("index") == "limit")
check("hand_pinch 展开出 6 个弧度", len(pinch.action.get("hand", [])) == 6)
check("展开值就是 compile_pose 的结果",
      raw_of(pinch.action["hand"]) == raw_of(hp.resolve(pinch.pose)))
check("力控/速度/时长仍在 action",
      pinch.action.get("hand_force") == 300 and pinch.action.get("duration") == 1.5)
check("to_public 暴露 pose 但不暴露 hand",
      pinch.to_public().get("pose") is not None
      and "hand" not in pinch.to_public())
# 握拳的**终态**现在由 _hand_fist_curl 持有 —— 2026-08-10 把 hand_close 拆成
# 「先折拇指、再收四指」两个阶段的 composite 来避开互顶,composite 自己没有 action。
# 这条断言验的是那个实测出来的余量(raw 320,比 limit 的 235 宽 85 counts),
# pose 一个字没改,只是搬了家。
curl = reg.get("_hand_fist_curl")
check("握拳终态食指留宽余量(raw 320,不是 limit 的 235)",
      raw_of(curl.action["hand"])[2] == 320, f"{raw_of(curl.action['hand'])[2]}")
# 第一阶段:拇指已到折叠位、四指还全开 —— 拇指整条路径都在空域里。
fold = reg.get("_hand_thumb_folded")
check("握拳第一阶段拇指到位、四指全开",
      raw_of(fold.action["hand"])[0] == raw_of(curl.action["hand"])[0]
      and raw_of(fold.action["hand"])[2] == 1000,
      f"拇指 {raw_of(fold.action['hand'])[0]} / 食指 {raw_of(fold.action['hand'])[2]}")
# 两个阶段的**终态拇指相同**才叫"拇指不再动" —— 不同的话第二阶段拇指会边收边动,
# 那就还是两边同时动,拆开就没意义了。
check("两阶段拇指目标一致(第二阶段拇指静止)",
      raw_of(fold.action["hand"])[:2] == raw_of(curl.action["hand"])[:2])

# ---------------------------------------------------------------- 手势表
print("\n[8] 手势表合成的技能")
gests = [s for s in reg if s.id.startswith("hand_") and s.pose is not None]
check("手势表合成了 ≥9 条", len(gests) >= 9, f"实际 {len(gests)}")
two = reg.get("hand_two")
check("hand_two 存在", two is not None)
check("hand_two 是 primitive", two is not None and two.kind == "primitive")
check("hand_two 语音可命中", two is not None and two.safety.voice_enabled)
check("hand_two 要确认(会产生运动)", two is not None and two.safety.need_confirm)
check("hand_two 展开出 6 弧度", two is not None and len(two.action["hand"]) == 6)
check("hand_two 食指+中指伸直、无名+小指收拢",
      two is not None and raw_of(two.action["hand"])[2:] == [1000, 1000, 0, 0],
      str(raw_of(two.action["hand"])[2:]) if two else "")
# force 的**具体数值**从 gestures.yaml 的 defaults 读,不在测试里抄第二份 ——
# 抄了的话每次调力度档都要改测试,而这条测试本来想验的是"defaults 真的套上了",
# 不是"那个数等于 250"。(2026-08-10 力度从 250/speed 150 调到 300/speed 500,
# 原来写死 250 的断言当场失败,就是这个原因。)
_g_defaults = yaml.safe_load(
    (Path(__file__).resolve().parent / "gestures.yaml").read_text(encoding="utf-8")
)["defaults"]
check("手势的 force 来自 gestures.yaml 的 defaults",
      two is not None and two.action.get("hand_force") == _g_defaults["hand_force"],
      f"技能里 {two.action.get('hand_force') if two else '?'} / "
      f"清单里 {_g_defaults['hand_force']}")
check("手势的 speed 也来自 defaults",
      two is not None and two.action.get("hand_speed") == _g_defaults["hand_speed"])
check("手势声明了力度参数(修饰词才落得进)",
      two is not None and "hand_force" in two.params and "hand_speed" in two.params)
check("hand_ok 的 limit 推导成 raw 235",
      raw_of(reg.get("hand_ok").action["hand"])[2] == 235)
check("每个手势都通过可行域",
      all(hp.check_feasible(s.action["hand"]) is None for s in gests))
check("手势 id 都是 hand_ 前缀", all(s.id.startswith("hand_") for s in gests))
check("剪刀和比个2 是同一条技能(不重复定义 pose)",
      "剪刀" in (reg.get("hand_two").aliases or []))

# ---------------------------------------------------------------- overlay
print("\n[9] overlay 算子")
from backend import (OVERLAY_EXCLUSIVE, OVERLAY_FORBIDDEN,  # noqa: E402
                     SkillError, _merge_overlay, make_backend)

hw = reg.get("home_with_one")
check("home_with_one 是 overlay", hw is not None and hw.mode == "overlay")
check("默认 mode 是 sequence", reg.get("prepare_arm").mode == "sequence")
be = make_backend(hw, reg)
hp_params, _ = hw.resolve_params({})
sts = list(be.steps(hp_params))
check("overlay 只产出一步", len(sts) == 1 and be.total(hp_params) == 1)
check("duration 取 max 而非 sum",
      sts[0].cmd["duration"] == 5.0 and be.duration_hint(hp_params) == 5.0)
check("臂和手都在同一条指令里",
      "arm" in sts[0].cmd and "hand" in sts[0].cmd)
check("力控/速度也带上",
      sts[0].cmd.get("hand_force") == _g_defaults["hand_force"],
      f"{sts[0].cmd.get('hand_force')}")

# 合并规则本身
check("不相交的键正常合",
      _merge_overlay([{"arm": [0] * 7}, {"hand": [0] * 6}], "t")
      == {"arm": [0] * 7, "hand": [0] * 6})
check("duration 取最大",
      _merge_overlay([{"arm": [0] * 7, "duration": 1.0},
                      {"hand": [0] * 6, "duration": 4.0}], "t")["duration"] == 4.0)
for k in OVERLAY_EXCLUSIVE:
    try:
        _merge_overlay([{k: 1}, {k: 2}], "t")
        check(f"{k} 双来源要报错", False, "没抛")
    except SkillError as e:
        check(f"{k} 双来源要报错", "冲突" in str(e))

# 构造期就该拦住,不能等到 steps()
def build(steps_, mode="overlay"):
    from schema import SkillSpec, SafetySpec
    sp = SkillSpec(id="_t", name="T", kind="composite", mode=mode,
                   steps=[{"skill": s} for s in steps_],
                   safety=SafetySpec(voice_enabled=False, need_confirm=True))
    return make_backend(sp, reg)


def rejects(label, steps_, want):
    try:
        build(steps_)
        check(label, False, "没抛异常")
    except SkillError as e:
        check(label, want in str(e), f"报错是 {str(e)[:70]!r}")


rejects("estop 不许叠", ["estop", "hand_one"], "模式切换")
rejects("action 类不许叠", ["arm_enable", "hand_one"], "模式切换")
rejects("trajectory 不许叠", ["replay_rgb_demo", "hand_one"], "只收 primitive")
rejects("嵌套 composite 不许叠", ["prepare_arm", "hand_one"], "只收 primitive")
rejects("两条都给 hand 要报错", ["hand_one", "hand_two"], "冲突")
rejects("力度档+手势会冲突(归修饰词,不归 overlay)",
        ["hand_grip_soft", "hand_one"], "冲突")
try:
    build(["go_home", "hand_two"])
    check("臂+手合法(通道不相交)", True)
except SkillError as e:
    check("臂+手合法(通道不相交)", False, str(e)[:70])
check("sequence 模式不受 overlay 约束(轨迹能进)",
      build(["replay_rgb_demo", "hand_one"], mode="sequence") is not None)

# ---------------------------------------------------------------- 收尾
print("\n" + "=" * 60)
print(f"通过 {_pass} · 失败 {len(_fail)}")
for f in _fail:
    print(f"  ✗ {f}")
raise SystemExit(1 if _fail else 0)
