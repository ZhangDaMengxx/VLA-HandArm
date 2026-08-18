#!/usr/bin/env python3
"""combo_pack 的单测。不碰硬件 —— 纯数据变换 + 文件读写。

⚠ 必须在 import combo_pack **之前**设 COMBO_RECORD_DIR:combo_root() 每次调用
都读环境变量,但 import 时如果 data/combos 不存在会走到真实路径去。
(和 test_gesture_pack.py 同一个坑,那边注释里记着。)
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
from pathlib import Path

SRC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SRC))

_TMP = tempfile.mkdtemp(prefix="combo_test_")
os.environ["COMBO_RECORD_DIR"] = _TMP

import combo_pack as cp                                          # noqa: E402

# 一个安全的臂位姿:七个关节都在限位中间附近,不贴边。
SAFE_ARM = [0.1, 0.2, 0.3, 0.4, 0.5, 0.1, 0.2]
SAFE_HAND = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7]


def frame_dict(t_ns: int = 0, arm=None, hand=None, **kw) -> dict:
    d = {"arm_rad": list(arm or SAFE_ARM), "hand_rad": list(hand or SAFE_HAND),
         "t_ns": t_ns, "hold_ms": 600}
    d.update(kw)
    return d


def pack_dict(nframes: int = 2, **kw) -> dict:
    d = {"schema": cp.SCHEMA, "name": "测试包", "mode": "keyframe",
         "recorded_from": "mock",
         "frames": [frame_dict(t_ns=i * 600_000_000) for i in range(nframes)]}
    d.update(kw)
    return d


def expect_err(fn, *a, what: str = "", **kw) -> str:
    """fn 必须抛 ComboError。返回消息文本给调用方断言内容。"""
    try:
        fn(*a, **kw)
    except cp.ComboError as e:
        return str(e)
    raise AssertionError(f"{what or fn.__name__} 应该抛 ComboError 但没抛")


# ---------------------------------------------------------------------------
# 入口校验
# ---------------------------------------------------------------------------
def test_schema_must_match():
    for bad in (None, "", "combo_pack/2", "hand_gesture_pack/2", "随便"):
        msg = expect_err(cp.ComboPack.from_dict, pack_dict(schema=bad),
                         what=f"schema={bad!r}")
        assert "schema" in msg, msg


def test_mode_must_be_keyframe_or_stream():
    """mode 是**显式**字段,不认识的值直接拒 —— 不能默认成 keyframe。

    默认掉的后果:一个本该流式播放的包被当关键帧,每帧按 hold_ms 驻留,
    30fps 的流会变成慢动作。那是能跑但错的,比拒掉难查。
    """
    for bad in ("stream_", "KEYFRAME", "auto", ""):
        msg = expect_err(cp.ComboPack.from_dict, pack_dict(mode=bad))
        assert "mode" in msg, msg
    for good in ("keyframe", "stream"):
        assert cp.ComboPack.from_dict(pack_dict(mode=good)).mode == good


def test_recorded_from_must_be_real_or_mock():
    msg = expect_err(cp.ComboPack.from_dict, pack_dict(recorded_from="sim"))
    assert "recorded_from" in msg, msg
    assert cp.ComboPack.from_dict(pack_dict(recorded_from="real")).recorded_from == "real"


def test_frames_empty_rejected():
    for bad in ([], None, "不是数组", {}):
        msg = expect_err(cp.ComboPack.from_dict, pack_dict(frames=bad))
        assert "frames" in msg, msg


def test_frames_over_limit_rejected():
    d = pack_dict(frames=[frame_dict(t_ns=i) for i in range(cp.MAX_FRAMES + 1)])
    msg = expect_err(cp.ComboPack.from_dict, d)
    assert str(cp.MAX_FRAMES) in msg, msg


def test_arm_over_limit_rejected_not_clamped():
    """臂角超限**拒绝**,不夹取。

    夹了就把"包里的数越界"这个事实抹掉了 —— 那个包会静默地播成另一个动作。
    和 combo_player 的入口校验同一条纪律。
    """
    lo, hi = cp.NERO_ARM_LIMITS[5]              # joint6,不对称的那个
    bad_arm = list(SAFE_ARM)
    bad_arm[5] = hi + 0.01                       # 超上限
    msg = expect_err(cp.ComboPack.from_dict,
                     pack_dict(frames=[frame_dict(arm=bad_arm)]))
    assert "joint6" in msg and "超限位" in msg, msg
    bad_arm[5] = lo - 0.01                       # 超下限
    msg = expect_err(cp.ComboPack.from_dict,
                     pack_dict(frames=[frame_dict(arm=bad_arm)]))
    assert "joint6" in msg, msg


def test_arm_limit_tolerance_boundary():
    """限位容差 1e-4:略微越界(取整残差量级)放过,真越界拒掉。

    ⚠ 这个容差不是"宽容一点",是**必需**的。实测过的那一次:
    `combo_player` 的入口校验拒了一个真包,joint6 = **0.9600** vs 上限
    **0.9599310885968813** —— 越界 6.89e-5 rad = 0.0039°,纯粹是存包时
    取整留下的残渣,没有物理意义。1e-4 rad = 0.0057°,刚好盖住这一档。

    ⚠ 这条测试原来写成"`round(radians(55), 5)` 会超上限",**那是错的**:
    0.9599310885968813 的第 6 位小数是 1,round 到 5 位是向下的 0.95993,
    落在限位**内**。取整往哪边跑取决于那个关节自己的第 6 位数字,不是普适规律。
    所以这里直接按**容差边界**断言,不依赖 round() 的行为。
    """
    _, hi = cp.NERO_ARM_LIMITS[5]                # joint6 上限
    inside = list(SAFE_ARM)
    inside[5] = hi + 5e-5                        # 越界但在容差内 → 收
    pack = cp.ComboPack.from_dict(pack_dict(frames=[frame_dict(arm=inside)]))
    assert len(pack.frames) == 1, "容差内的取整残差应该放过"

    outside = list(SAFE_ARM)
    outside[5] = hi + 2e-4                       # 越界超过容差 → 拒
    msg = expect_err(cp.ComboPack.from_dict,
                     pack_dict(frames=[frame_dict(arm=outside)]))
    assert "joint6" in msg and "超限位" in msg, msg


def test_dimension_errors_are_specific():
    """维度错要说清是哪个字段、期望几个 —— 不能只说"格式错"。"""
    msg = expect_err(cp.ComboFrame.build, SAFE_ARM[:6], SAFE_HAND, t_ns=0)
    assert "arm_rad" in msg and "7" in msg, msg
    msg = expect_err(cp.ComboFrame.build, SAFE_ARM, SAFE_HAND[:5], t_ns=0)
    assert "hand_rad" in msg and "6" in msg, msg


def test_bool_rejected_not_filtered():
    """bool 要**拒绝**,不能静默过滤。

    ⚠ 这是写这个文件时在自己代码里发现的 bug。原来是
    `[float(x) for x in vals if not isinstance(x, bool)]` —— True 被**过滤掉**,
    于是 6 个值变 5 个,报"需要 6 个收到 5"。那个报错指向的是错的原因:
    调用方会去数自己传了几个(明明是 6 个),而真因是其中一个是 bool。
    """
    bad = [0.1, True, 0.2, 0.3, 0.4, 0.5]
    msg = expect_err(cp.ComboFrame.build, SAFE_ARM, bad, t_ns=0)
    assert "非数值" in msg and "True" in msg, msg
    # 臂侧同理
    bad_arm = [0.1, 0.2, False, 0.4, 0.5, 0.1, 0.2]
    msg = expect_err(cp.ComboFrame.build, bad_arm, SAFE_HAND, t_ns=0)
    assert "非数值" in msg, msg


def test_string_raises_combo_error_not_bare_value_error():
    """非数值必须抛 ComboError,**不能**是裸 ValueError。

    ⚠ 也是自查发现的:原来 `float(x)` 对 "abc" 抛裸 ValueError,而 web 层
    只捕 ComboError 转 400 —— 裸的会冒成 **500**。校验失败报 500 是错的语义
    (客户端会以为是服务器坏了而重试)。
    """
    for bad in (["a"] * 6, [None] * 6, [{}] * 6):
        try:
            cp.ComboFrame.build(SAFE_ARM, bad, t_ns=0)
        except cp.ComboError:
            pass
        except ValueError as e:
            raise AssertionError(f"抛的是裸 ValueError 不是 ComboError: {e!r}")
        else:
            raise AssertionError(f"{bad!r} 应该被拒")


def test_t_ns_required():
    """combo_pack **要求**每帧有 t_ns,不从 hold_ms 累加补。

    ⚠ 和 gesture_pack 不同:那边要读 /1 版旧文件所以有 ensure_t_ns() 补齐。
    combo_pack 是新格式,没有旧文件要兼容 —— 缺 t_ns 一定是生成方的 bug,
    补出来只会把那个 bug 藏起来(而且补出来的时间轴带累加漂移)。
    """
    f = frame_dict()
    del f["t_ns"]
    msg = expect_err(cp.ComboPack.from_dict, pack_dict(frames=[f]))
    assert "t_ns" in msg, msg


def test_joint_order_validated_not_reordered():
    """joint_order 对不上就**拒**,不按它重排。

    重排听起来更宽容,但那要求关节名集合完全一致才安全;不一致时静默重排
    会把角度装到错的关节上 —— 那是会撞东西的错误。
    """
    msg = expect_err(cp.ComboPack.from_dict,
                     pack_dict(joint_order_arm=["a"] * 7))
    assert "joint_order_arm" in msg, msg
    msg = expect_err(cp.ComboPack.from_dict,
                     pack_dict(joint_order_hand=["b"] * 6))
    assert "joint_order_hand" in msg, msg
    # 正确的能过
    ok = cp.ComboPack.from_dict(pack_dict(
        joint_order_arm=list(cp.ARM_JOINTS), joint_order_hand=list(cp.HAND_JOINTS)))
    assert len(ok.frames) == 2


# ---------------------------------------------------------------------------
# 手侧 rad ↔ raw:两个字段必须表示同一个姿态
# ---------------------------------------------------------------------------
def test_hand_rad_and_raw_agree():
    """存进去的 rad 和 raw 必须**表示同一个姿态**。

    ⚠ 不折回的话手写/导入的越界 raw 会让两边打架:3D 预览按 rad 画,
    回放按 raw 写 ANGLE_SET —— 看到的和做的不一样。gesture_pack 里踩过。
    """
    f = cp.ComboFrame.build(SAFE_ARM, SAFE_HAND, t_ns=0)
    # 从存下来的 raw 反推 rad,应该和存下来的 rad 一致
    back = cp.gp.raw_proj_to_rad(cp.gp.vendor_to_proj(f.hand_raw))
    for i, (a, b) in enumerate(zip(f.hand_rad, back)):
        assert abs(a - b) < 1e-5, f"通道 {i}: rad {a} vs 从 raw 反推 {b}"


def test_hand_raw_takes_precedence():
    """两个都给时以 raw 为准 —— 它是真正上线的值。"""
    raw = [100, 200, 300, 400, 500, 600]
    f = cp.ComboFrame.build(SAFE_ARM, hand_rad=[0.0] * 6, hand_raw=raw, t_ns=0)
    # rad 应该是从 raw 推出来的,不是传进去的那个 [0.0]*6
    assert f.hand_rad != [0.0] * 6, "raw 没有优先"


def test_hand_raw_clamped_to_valid_range():
    f = cp.ComboFrame.build(SAFE_ARM, hand_raw=[-500, 9999, 0, 1000, 500, 500],
                            t_ns=0)
    for v in f.hand_raw:
        assert cp.gp.RAW_MIN <= v <= cp.gp.RAW_MAX, f"raw {v} 没夹到范围内"


# ---------------------------------------------------------------------------
# ee_pose:可选,读时重算,对不上警告
# ---------------------------------------------------------------------------
def test_ee_pose_computed_when_kin_available():
    if cp._kin is None:
        print("      (跳过:pinocchio 不可用)")
        return
    f = cp.ComboFrame.build(SAFE_ARM, SAFE_HAND, t_ns=0)
    assert f.ee_pose is not None and len(f.ee_pose) == 7, f"ee_pose = {f.ee_pose}"
    # 四元数应该是单位的
    import math
    qn = math.sqrt(sum(x * x for x in f.ee_pose[3:]))
    assert abs(qn - 1.0) < 1e-6, f"四元数不是单位: |q| = {qn}"


def test_ee_pose_mismatch_counted_not_fatal():
    """包里的 ee_pose 和重算的对不上 → 计数警告,**不拒包**。

    对不上说明包被手改过 arm_rad 而没更新 ee_pose。这时候 arm_rad 才是权威,
    继续用它 —— 拒掉的话手改一次包就废了,而 ee_pose 现在还没有消费方。
    """
    if cp._kin is None:
        print("      (跳过:pinocchio 不可用)")
        return
    f = frame_dict()
    f["ee_pose"] = [9.9, 9.9, 9.9, 1.0, 0.0, 0.0, 0.0]     # 明显错的
    pack = cp.ComboPack.from_dict(pack_dict(frames=[f]))
    assert pack.ee_mismatch == 1, f"应该记 1 个不一致,实际 {pack.ee_mismatch}"
    # 但 arm_rad 没被那个错的 ee_pose 影响
    assert pack.frames[0].arm_rad == [round(v, 6) for v in SAFE_ARM]


def test_ee_pose_absent_is_fine():
    """没有 ee_pose 不是错误 —— 它是可选字段(pinocchio 可能不可用)。"""
    pack = cp.ComboPack.from_dict(pack_dict(frames=[frame_dict()]))
    assert pack.ee_mismatch == 0


# ---------------------------------------------------------------------------
# 路径沙箱。⚠ 这是安全关键 —— 7860 没有认证,漏一个 .. 就是任意文件读写。
# ---------------------------------------------------------------------------
def test_sandbox_rejects_escapes():
    bad = ["../evil.json", "a/../../evil.json", "/etc/passwd.json", "",
           "..", "a/../..//evil.json", ".hidden.json", "a/.b/c.json"]
    for b in bad:
        expect_err(cp.resolve_pack_path, b, what=f"路径 {b!r}")


def test_sandbox_requires_json_suffix():
    """强制 .json —— 免得写出 .py/.sh 被别的东西捡去执行。"""
    for b in ("evil.py", "evil.sh", "noext", "a/b.yaml"):
        msg = expect_err(cp.resolve_pack_path, b)
        assert "json" in msg.lower(), msg


def test_sandbox_accepts_valid_paths():
    root = cp.combo_root()
    for good in ("a.json", "sub/b.json", "深/目录/c.json", "./d.json"):
        p = cp.resolve_pack_path(good)
        assert p.is_relative_to(root), f"{good} → {p} 逃出了 {root}"


def test_sandbox_blocks_symlink_escape():
    """软链接逃逸也要挡住。

    ⚠ 这一条是 `resolve()` 而不是 `normpath` 的**理由**:normpath 是纯字符串
    运算,不看文件系统 —— 根目录里放一个指向 /etc 的软链接,normpath 看不出来。
    """
    root = cp.combo_root()
    root.mkdir(parents=True, exist_ok=True)
    link = root / "link"
    if link.exists() or link.is_symlink():
        link.unlink()
    link.symlink_to("/etc")
    try:
        expect_err(cp.resolve_pack_path, "link/passwd.json")
    finally:
        link.unlink()


def test_env_override_works():
    assert cp.combo_root() == Path(_TMP).resolve(), \
        f"COMBO_RECORD_DIR 没生效: {cp.combo_root()}"


def test_sandbox_shared_with_gesture_pack():
    """沙箱实现是**共用**的(gesture_pack.resolve_in_root)。

    ⚠ 为什么要测这个:安全关键代码抄两份的话,以后修 bug 要记得修两处,
    而漏掉的那处**不会报错,只会静默地不安全**。这条测试锁住"确实是同一份"。
    """
    import gesture_pack as gpm
    assert hasattr(gpm, "resolve_in_root"), "共用入口不见了"
    # combo 的 resolve 必须真的走那一份(改掉它 combo 也应该跟着变)
    import inspect
    src = inspect.getsource(cp.resolve_pack_path)
    assert "resolve_in_root" in src, "combo 没有复用共用沙箱,自己抄了一份?"


# ---------------------------------------------------------------------------
# 文件读写
# ---------------------------------------------------------------------------
def test_save_load_roundtrip():
    pack = cp.ComboPack.from_dict(pack_dict(3))
    cp.save_pack("rt.json", pack)
    back = cp.load_pack("rt.json")
    assert back.name == pack.name and back.mode == pack.mode
    assert len(back.frames) == 3
    for a, b in zip(pack.frames, back.frames):
        assert a.arm_rad == b.arm_rad, f"臂角变了: {a.arm_rad} → {b.arm_rad}"
        assert a.hand_raw == b.hand_raw, f"手 raw 变了(会影响回放的位级一致性)"
        assert a.t_ns == b.t_ns


def test_save_is_atomic_no_temp_left():
    """原子写:成功后不留 .tmp_ 文件。

    直接 open(w) 写的话写一半崩了会留半个 JSON —— 那个文件之后每次列表都解析
    失败,看起来像"包坏了"而不是"上次没写完"。
    """
    cp.save_pack("atomic.json", cp.ComboPack.from_dict(pack_dict()))
    leftover = [x.name for x in cp.combo_root().rglob(".tmp_*")]
    assert not leftover, f"留下了临时文件: {leftover}"


def test_overwrite_guard():
    cp.save_pack("guard.json", cp.ComboPack.from_dict(pack_dict()))
    msg = expect_err(cp.save_pack, "guard.json",
                     cp.ComboPack.from_dict(pack_dict()), overwrite=False)
    assert "已存在" in msg, msg
    cp.save_pack("guard.json", cp.ComboPack.from_dict(pack_dict()), overwrite=True)


def test_load_missing_and_bad_json():
    expect_err(cp.load_pack, "nope.json")
    bad = cp.combo_root() / "bad.json"
    bad.parent.mkdir(parents=True, exist_ok=True)
    bad.write_text("{ 这不是 json", encoding="utf-8")
    msg = expect_err(cp.load_pack, "bad.json")
    assert "JSON" in msg, msg


def test_oversized_file_rejected():
    big = cp.combo_root() / "big.json"
    big.write_text("x" * (cp.MAX_FILE_BYTES + 1), encoding="utf-8")
    msg = expect_err(cp.load_pack, "big.json")
    assert "过大" in msg, msg


def test_delete_pack():
    cp.save_pack("del.json", cp.ComboPack.from_dict(pack_dict()))
    cp.delete_pack("del.json")
    assert not (cp.combo_root() / "del.json").exists()
    expect_err(cp.delete_pack, "del.json")          # 再删要报不存在


def test_list_packs_skips_broken():
    """坏包**跳过不报错** —— 一个坏文件不该让整个列表挂掉。"""
    cp.save_pack("list_ok.json", cp.ComboPack.from_dict(pack_dict(name="好包")))
    (cp.combo_root() / "list_bad.json").write_text("{", encoding="utf-8")
    names = [p["name"] for p in cp.list_packs()]
    assert "好包" in names, f"好包没列出来: {names}"


def test_to_dict_fills_ee_pose():
    """to_dict 时统一补 ee_pose(而不是每帧 build 都跑 FK)。"""
    if cp._kin is None:
        print("      (跳过:pinocchio 不可用)")
        return
    f = cp.ComboFrame(arm_rad=list(SAFE_ARM), hand_rad=list(SAFE_HAND),
                      hand_raw=[500] * 6, t_ns=0, ee_pose=None)
    pack = cp.ComboPack(name="x", frames=[f])
    d = pack.to_dict()
    assert d["frames"][0].get("ee_pose") is not None, "to_dict 没补 ee_pose"


def test_written_json_is_readable_and_pretty():
    """写出来的文件人要能读能改 —— 这个格式的用途之一就是手改。"""
    cp.save_pack("pretty.json", cp.ComboPack.from_dict(pack_dict()))
    text = (cp.combo_root() / "pretty.json").read_text(encoding="utf-8")
    assert "\n" in text and "  " in text, "没有缩进,手改起来很痛苦"
    d = json.loads(text)
    assert d["schema"] == cp.SCHEMA
    assert d["joint_order_arm"] == list(cp.ARM_JOINTS), "关节序没写进文件"


def test_duration_from_hold_ms():
    pack = cp.ComboPack.from_dict(pack_dict(3))
    assert pack.duration_ms == 1800, f"3 帧 × 600ms 应该是 1800,实际 {pack.duration_ms}"


# ---------------------------------------------------------------------------
def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    passed, failed = 0, []
    for name, fn in tests:
        try:
            fn()
            print(f"  ok   {name}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL {name}: {e}")
            failed.append(name)
        except Exception as e:                                # noqa: BLE001
            print(f"  ERR  {name}: {type(e).__name__}: {e}")
            failed.append(name)
    print(f"\n{passed}/{len(tests)} 通过   根目录={_TMP}")
    if failed:
        print("失败: " + ", ".join(failed))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
