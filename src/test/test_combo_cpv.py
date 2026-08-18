#!/usr/bin/env python3
"""src/test/test_combo_cpv.py — 联合录制包走 CPV 回放这条链路的测试。

覆盖这一轮踩到的四个真 bug(每个都在真机之外先被这里挡住):
  1. preflight() 回 list[str] 却被当 tuple 解包 —— 两个问题时**安全门反向失效**
  2. stop() 只置 stopped 不置 done —— 之后永久拒放,cpv_end 没人调
  3. elapsed_ns 暂停期间还在涨(实测 351→851ms)
  4. progress() 没夹下界,start_at 在未来时报负值

⚠ 全部 mock,不碰硬件。
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import combo_player as cp                                          # noqa: E402
from combo_player import ArmTrajPack, ArmWaypoint, ComboPlayer      # noqa: E402

_T: list = []


def case(fn):
    _T.append(fn)
    return fn


def _pack(n=4, step_ns=200_000_000) -> ArmTrajPack:
    wps = [ArmWaypoint(t_ns=i * step_ns, rad=[i * 0.02] + [0.0] * 6)
           for i in range(n)]
    return ArmTrajPack(name="t", mode="waypoints", waypoints=wps,
                       approach_rad=list(wps[0].rad))


def _player(**kw) -> ComboPlayer:
    return ComboPlayer(_pack(**kw), None, None, None)


def _combo_dict() -> dict:
    """一个最小的合法 combo 包 dict。两帧,手从张开到半合。"""
    import combo_pack as cbp
    return {
        "schema": cbp.SCHEMA, "name": "测试挥手", "mode": "keyframe",
        "recorded_from": "mock", "arm": "nero", "hand": "inspire_rh56dfx_right",
        "frames": [
            {"t_ns": 0, "hold_ms": 500, "speed": 500, "force": 200,
             "arm_rad": [0.0] * 7, "hand_rad": [0.2] * 6,
             "hand_raw": [1000] * 6},
            {"t_ns": 500_000_000, "hold_ms": 500, "speed": 500, "force": 200,
             "arm_rad": [0.1] + [0.0] * 6, "hand_rad": [0.8] * 6,
             "hand_raw": [400] * 6},
        ],
    }


def _hand_half(pack, gp) -> dict:
    """和 app_web._combo_start 里那段**保持一致**。

    ⚠ 抄一份到测试里是有意的:这样 _combo_start 改了字段名而没改这里的话,
    test_combo_hand_half_loads_as_gesture_pack 会失败 —— 而那正是我们想要的
    (那段转换错了不会报错,只会让手不动或动错)。
    """
    return {
        "schema": gp.SCHEMA, "name": pack.name, "hand": pack.hand,
        "return_home_first": False,
        "frames": [{"rad": list(f.hand_rad), "raw_vendor": list(f.hand_raw),
                    "hold_ms": f.hold_ms, "speed": f.speed, "force": f.force,
                    "label": f.label, "t_ns": f.t_ns} for f in pack.frames],
    }


# --------------------------------------------------------------- preflight
@case
def test_preflight_returns_list_not_tuple():
    """⚠ 这是**安全门反向失效**那个 bug 的回归测试。

    原来 arm_console 写的是 `ok, why = preflight()`。preflight 回的是 list[str]:
      []            -> ValueError,console 直接崩(handle 外面只有 KeyboardInterrupt)
      ['a']         -> ValueError
      ['a','b']     -> ok='a' 是**真值**,`if not ok` 不成立 -> **门直接放行**
    而「臂未使能」+「ctrl_mode 不是 CAN_CTRL」恰好就是两条 —— 最常见的组合。
    """
    out = _player().preflight()
    assert isinstance(out, list), f"preflight 必须回 list,得到 {type(out)}"
    assert all(isinstance(x, str) for x in out), "元素必须是 str"
    # 无臂时没有阻断项
    assert out == [], f"arm=None 时不该有阻断项,得到 {out}"
    # 证明当年那种解包在两个问题时会静默放行
    two = ["臂未使能", "ctrl_mode 不对"]
    ok, _why = two                      # 就是出事的那行
    assert ok, "复现:两个问题时解包出的 ok 是真值,于是 `if not ok` 放行"


# --------------------------------------------------------------- stop/done
@case
def test_stop_also_sets_done():
    """stop() 必须同时置 done,否则所有「等它结束」的循环永远等下去。

    实际后果:arm_console 主循环靠 done 清 _player,不清就是
      · cpv_end() 没人调 —— auto_set_motion_mode 一直关着
      · 下一个 combo_play 被「已经在回放」**永久**拒掉(实测踩到)
      · CLI 的 `while not pl.done` 死循环
    """
    pl = _player()
    pl.start()
    pl.stop()
    assert pl.stopped, "stopped 要置上"
    assert pl.done, "done 也要置上 —— 见 stop() 的注释"


@case
def test_stopped_distinguishes_interrupted_from_finished():
    """stopped 保留是为了区分「被打断」和「正常放完」。"""
    a = _player(n=2, step_ns=1_000_000)
    a.start()
    a.stop()
    assert a.stopped and a.done

    b = _player(n=2, step_ns=1_000_000)
    b.start()
    for _ in range(50):
        b.tick()
        if b.done:
            break
        time.sleep(0.002)
    assert b.done, "应该正常放完"
    assert not b.stopped, "正常放完 stopped 必须是 False"


# ------------------------------------------------------------- 暂停时的 elapsed
@case
def test_elapsed_frozen_while_paused():
    """⚠ 暂停期间 elapsed 必须**定住**。

    _paused_total 只在 resume() 里累加,elapsed_ns 光减它的话暂停期间这个数
    还在涨 —— 实测暂停 0.5s,报的 elapsed 从 351ms 涨到 851ms(而 tick 一帧
    都没发),前端进度条在暂停时继续走。
    """
    pl = _player()
    pl.start()
    time.sleep(0.05)
    pl.pause()
    e1 = pl.elapsed_ns
    time.sleep(0.15)
    e2 = pl.elapsed_ns
    drift_ms = (e2 - e1) / 1e6
    assert abs(drift_ms) < 5.0, f"暂停期间 elapsed 漂了 {drift_ms:.1f}ms(应≈0)"


@case
def test_elapsed_resumes_correctly():
    """恢复后要接着走,而且**不能**把暂停那段算进去。"""
    pl = _player()
    pl.start()
    time.sleep(0.08)
    before = pl.elapsed_ns
    pl.pause()
    time.sleep(0.2)                  # 暂停 200ms
    pl.resume()
    time.sleep(0.05)
    after = pl.elapsed_ns
    grew_ms = (after - before) / 1e6
    # 只应该长了 resume 之后那 50ms,不含暂停的 200ms
    assert 20 < grew_ms < 120, f"恢复后长了 {grew_ms:.0f}ms,期望 ~50ms(不含暂停的 200ms)"


@case
def test_tick_ignores_paused_elapsed():
    """tick() 在暂停时必须直接返回 —— 它看的是真实时钟推进,和「对外报告」
    不是一件事。改 elapsed_ns 不能影响定位逻辑。"""
    pl = _player()
    pl.start()
    pl.pause()
    assert pl.tick() is None, "暂停时 tick 必须回 None"
    i_before = pl.i_arm
    time.sleep(0.1)
    pl.tick()
    assert pl.i_arm == i_before, "暂停期间 tick 不能推进帧指针"


# --------------------------------------------------------------- start_at
@case
def test_start_at_sets_t0():
    """start_at 是跨进程对齐的机制 —— 两个 console 靠同一个 CLOCK_MONOTONIC
    时刻当 t0。⚠ 原来我在 arm_console 里写的是 `_player.start_at = ...`,
    ComboPlayer 没有 __slots__ 所以**静默接受**,而 start() 无参 —— 对齐
    根本没发生,而且不报错。
    """
    import inspect
    sig = inspect.signature(ComboPlayer.start)
    assert "start_at" in sig.parameters, \
        "start() 必须有 start_at 参数(不是往实例上贴属性)"
    future = time.monotonic() + 10.0
    pl = _player()
    pl.start(start_at=future)
    assert abs(pl._t0 - future) < 1e-6, "start_at 要成为 t0"
    # 起跑前 elapsed 是负的 —— 这是对的,tick 靠它判断「还没到第一帧」
    assert pl.elapsed_ns < 0, "起跑时刻在未来时 elapsed 应为负"


@case
def test_start_without_start_at_is_now():
    pl = _player()
    t = time.monotonic()
    pl.start()
    assert abs(pl._t0 - t) < 0.05, "不给 start_at 就是立刻开始"


@case
def test_progress_clamped_at_zero():
    """⚠ progress() 下界也要夹。start_at 在未来时 elapsed_ns 是负的
    (实测起跑前报 -40ms),不夹的话前端进度条显示负值。"""
    pl = _player()
    pl.start(start_at=time.monotonic() + 5.0)
    p = pl.progress()
    assert p == 0.0, f"起跑前 progress 应为 0.0,得到 {p}"
    assert 0.0 <= p <= 1.0


@case
def test_progress_clamped_at_one():
    pl = _player(n=2, step_ns=1_000_000)
    pl.start(start_at=time.monotonic() - 100.0)     # 早就该放完了
    assert pl.progress() == 1.0


@case
def test_tick_not_fired_before_start_at():
    """起跑时刻之前 tick 不能发帧 —— 负的 now_ns 只是「还没到第一帧」。"""
    pl = _player()
    pl.start(start_at=time.monotonic() + 5.0)
    for _ in range(5):
        pl.tick()
    assert pl.i_arm == 0, "起跑前不该推进"
    assert pl.sent_arm == 0, "起跑前不该发帧"


# ------------------------------------------------------ 手侧复用 gesture_play
@case
def test_combo_hand_half_loads_as_gesture_pack():
    """combo 包的手侧转成 gesture pack 的 dict 必须能被 GesturePack 吃下。

    ⚠ 这是「别自己拼 ActionStep」的回归测试。我一开始在 hand_console 里另写了
    个 "play" 命令自己拼 ActionStep,字段全错:angles 要 0-1000 原始值不是弧度,
    speeds/forces 是 6 个的列表不是标量,驻留字段叫 delay_ms 不是 hold_ms
    —— console 直接 NameError 崩(ActionStep 都没导入)。
    正确做法是转成 gesture pack 复用 to_action_sequence 那一整套。
    """
    import combo_pack as cbp
    import gesture_pack as gp
    pack = cbp.ComboPack.from_dict(_combo_dict())
    hp = _hand_half(pack, gp)
    g = gp.GesturePack.from_dict(hp)              # 不抛就算过
    assert g.name == pack.name
    assert len(g.frames) == len(pack.frames)
    seq = gp.to_action_sequence(g, slot=-1, return_home=False)
    assert len(seq.steps) == len(pack.frames), "return_home=False 时不该多一步"
    # angles 必须是 0-1000 原始值,而且和包里的 hand_raw 对得上(过项目序置换)
    assert seq.steps[0].angles == list(
        gp.vendor_to_proj(pack.frames[0].hand_raw))


@case
def test_combo_hand_half_shares_arm_timeline():
    """两侧必须是**同一条**时间轴 —— t_ns 原样带过去,不平移。"""
    import combo_pack as cbp
    import gesture_pack as gp
    pack = cbp.ComboPack.from_dict(_combo_dict())
    g = gp.GesturePack.from_dict(_hand_half(pack, gp))
    seq = gp.to_action_sequence(g, slot=-1, return_home=False)
    arm_t = [f.t_ns for f in pack.frames]
    hand_t = [s.t_ns for s in seq.steps]
    assert hand_t == arm_t, f"手侧时间轴 {hand_t} != 臂侧 {arm_t}"


@case
def test_return_home_forced_off():
    """⚠ return_home_first 必须强制 False。combo 包第 0 帧就是录制那一刻的姿态,
    回零会在正式动作前插一段**臂不知道的**手部运动(HOME_HOLD_MS),
    两侧时间轴当场错开。"""
    import combo_pack as cbp
    import gesture_pack as gp
    pack = cbp.ComboPack.from_dict(_combo_dict())
    hp = _hand_half(pack, gp)
    assert hp["return_home_first"] is False, "转出来的 dict 必须写死 False"
    g = gp.GesturePack.from_dict(hp)
    seq = gp.to_action_sequence(g, slot=-1, return_home=False)
    assert seq.steps[0].t_ns == 0, "第一步就该在 t=0,没有 home 偏移"


# ------------------------------------------------------------- web 层统一路径
@case
def test_combo_start_exists_and_is_the_only_path():
    """⚠ `_combo_start` 必须存在。它被 /api/combo/play 和 _voice_play_combo
    **两处**调 —— 原来只有 _voice_play_combo 引用它而函数根本没定义,
    语音调挥手会 NameError -> 500;而 ▶ 按钮走的是另一条(ComboPlaySession +
    move_j)。同一个包两条路放出来不一样,preflight 也是两份。
    """
    import app_web as w
    assert hasattr(w, "_combo_start"), "_combo_start 没定义"
    src = Path(SIM / "app_web.py").read_text(encoding="utf-8")
    n = src.count("_combo_start(pack")
    assert n >= 2, f"_combo_start 应被两处调用(▶ 和语音),只找到 {n}"
    assert "class ComboPlaySession" not in src, \
        "ComboPlaySession 是走 move_j 的旧路,已删 —— 留着会让人以为还有第二条路"


@case
def test_combo_start_returns_triple():
    import app_web as w
    import inspect
    sig = inspect.signature(w._combo_start)
    assert list(sig.parameters) == ["pack", "rel"], list(sig.parameters)
    # 两边都去空格再比 —— 只去一边的话 needle 里的空格永远匹配不上
    got = str(sig.return_annotation).replace(" ", "")
    assert got == "tuple[bool,str,int]", got


@case
def test_stream_pack_rejected_by_web():
    """stream 包页面上放不了。理由**不是**「web 层拿不到 NeroArm」(CPV 现在
    就跑在 arm_console 里),而是上千帧不该一次性下发。"""
    import combo_pack as cbp
    import app_web as w
    d = _combo_dict()
    d["mode"] = "stream"
    pack = cbp.ComboPack.from_dict(d)
    ok, msg, code = w._combo_start(pack, "x.json")
    assert not ok and code in (400, 409), (ok, msg, code)


def main() -> int:
    bad = 0
    for fn in _T:
        try:
            fn()
            print(f"  ok   {fn.__name__}")
        except Exception as e:                              # noqa: BLE001
            bad += 1
            print(f"  FAIL {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(_T) - bad}/{len(_T)} 通过")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
