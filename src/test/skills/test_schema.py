"""测技能清单的校验与参数归一。只用标准库 + PyYAML,两个 python 环境都能跑。

    python3 src/skills/test_schema.py

跑法与项目其他自检脚本一致:全过 RC=0,任一断言失败当场抛。
重点验证「坏清单必须被拒」——校验层要是放过错误,语音就会随机命中错技能。
"""
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "skills"))
from schema import RegistryError, load_registry  # noqa: E402

PASS, FAIL = [], []


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'✓' if cond else '✗'} {name}" + (f" · {detail}" if detail else ""))


def load_bad(yaml_text: str):
    """把一段 YAML 写临时文件后加载,返回 (registry|None, 错误信息)。"""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False,
                                     encoding="utf-8") as f:
        f.write(yaml_text)
        p = f.name
    try:
        return load_registry(p), ""
    except RegistryError as e:
        return None, str(e)
    finally:
        Path(p).unlink(missing_ok=True)


def expect_reject(name: str, yaml_text: str, needle: str) -> None:
    """坏清单必须被拒,且错误信息里要出现 needle(证明是因为这个原因被拒的)。"""
    reg, err = load_bad(yaml_text)
    check(name, reg is None and needle in err,
          "已拒绝" if reg is None else "误放过了!")


# ---------------------------------------------------------------- 真清单
print("\n[1] 真清单 registry.yaml")
reg = load_registry()
check("加载通过", len(reg) > 0, f"{len(reg)} 条技能")
check("estop 免确认", reg.get("estop").safety.need_confirm is False)
check("estop 允许语音", reg.get("estop").safety.voice_enabled is True)
check("别名精确匹配", reg.by_alias("回零位") is not None
      and reg.by_alias("回零位").id == "go_home")
check("别名归一化(空格/大小写)", reg.by_alias(" GO_HOME ") is not None
      and reg.by_alias(" GO_HOME ").id == "go_home")
check("未知别名返回 None", reg.by_alias("给我倒杯咖啡") is None)
check("composite 步骤全部可解析",
      all(reg.get(st["skill"]) is not None
          for s in reg if s.kind == "composite" for st in s.steps))
check("to_public 不泄漏 action 关节值",
      all("action" not in d for d in reg.to_public()))

# ---------------------------------------------------------------- 参数归一
print("\n[2] 参数归一与夹取")
rgbd = reg.get("replay_rgbd_demo")
p, notes = rgbd.resolve_params({})
check("缺省取 default", p["speed"] == 1.0, f"speed={p['speed']}")
p, notes = rgbd.resolve_params({"speed": 99})
check("超上限被夹取", p["speed"] == 4.0 and notes, f"speed={p['speed']}")
p, notes = rgbd.resolve_params({"speed": 0.01})
check("低于下限被夹取", p["speed"] == 0.25, f"speed={p['speed']}")
p, notes = rgbd.resolve_params({"speed": "abc"})
check("非法值回退 default", p["speed"] == 1.0 and bool(notes))
p, notes = rgbd.resolve_params({"speed": 2.0, "wat": 1})
check("未声明参数被丢弃", "wat" not in p and any("wat" in n for n in notes))
p, notes = rgbd.resolve_params({"speed": 4.0}, via_voice=True)
check("语音路径限速生效", p["speed"] == 1.0, f"speed={p['speed']} (max_speed=1.0)")
p, _ = rgbd.resolve_params({"speed": 4.0}, via_voice=False)
check("非语音路径不限速", p["speed"] == 4.0, f"speed={p['speed']}")
p, _ = reg.get("go_home").resolve_params({"duration": 999})
check("go_home duration 夹取", p["duration"] == 15.0, f"duration={p['duration']}")

# ---------------------------------------------------------------- 坏清单必须被拒
print("\n[3] 坏清单拒绝(校验层的真实价值)")

expect_reject("id 重复", """
version: 1
skills:
  - {id: a, name: A, kind: primitive, action: {estop: true}}
  - {id: a, name: B, kind: primitive, action: {estop: true}}
""", "id 重复")

expect_reject("别名撞车", """
version: 1
skills:
  - {id: a, name: 甲, kind: primitive, action: {estop: true}, aliases: ["停"]}
  - {id: b, name: 乙, kind: primitive, action: {estop: true}, aliases: ["停"]}
""", "别名/名称冲突")

expect_reject("别名撞上另一技能的名称", """
version: 1
skills:
  - {id: a, name: 回零, kind: primitive, action: {estop: true}}
  - {id: b, name: 乙, kind: primitive, action: {estop: true}, aliases: ["回零"]}
""", "别名/名称冲突")

expect_reject("composite 自引用", """
version: 1
skills:
  - {id: a, name: A, kind: composite, steps: [{skill: a}]}
""", "环路")

expect_reject("composite 互相引用成环", """
version: 1
skills:
  - {id: a, name: A, kind: composite, steps: [{skill: b}]}
  - {id: b, name: B, kind: composite, steps: [{skill: a}]}
""", "环路")

expect_reject("composite 引用不存在的技能", """
version: 1
skills:
  - {id: a, name: A, kind: composite, steps: [{skill: nope}]}
""", "引用不存在")

expect_reject("未知 kind", """
version: 1
skills:
  - {id: a, name: A, kind: magic, action: {estop: true}}
""", "kind=magic")

expect_reject("kind 缺专属字段", """
version: 1
skills:
  - {id: a, name: A, kind: trajectory}
""", "缺必需字段 source")

expect_reject("kind 字段串味", """
version: 1
skills:
  - {id: a, name: A, kind: primitive, action: {estop: true}, source: x.npz}
""", "不该带")

expect_reject("顶层字段拼错", """
version: 1
skills:
  - {id: a, name: A, kind: primitive, action: {estop: true}, saftey: {}}
""", "未知顶层字段")

expect_reject("requires 写了不认识的前置", """
version: 1
skills:
  - {id: a, name: A, kind: primitive, action: {estop: true}, requires: [moon_phase]}
""", "未知项 moon_phase")

expect_reject("params range 反了", """
version: 1
skills:
  - id: a
    name: A
    kind: primitive
    action: {estop: true}
    params: {speed: {type: float, range: [4.0, 0.25]}}
""", "下界大于上界")

expect_reject("缺 skills", "version: 1\n", "skills 必须是非空列表")

# ---------------------------------------------------------------- 好清单要放过
print("\n[4] 合法但极简的清单要能通过")
reg2, err = load_bad("""
version: 1
skills:
  - {id: only, name: 唯一, kind: primitive, action: {estop: true}}
""")
check("极简清单通过", reg2 is not None and len(reg2) == 1, err)
reg3, err = load_bad("""
version: 1
skills:
  - {id: leaf, name: 叶, kind: primitive, action: {estop: true}}
  - {id: mid,  name: 中, kind: composite, steps: [{skill: leaf}]}
  - {id: top,  name: 顶, kind: composite, steps: [{skill: mid}, {skill: leaf}]}
""")
check("composite 嵌套(无环)通过", reg3 is not None and len(reg3) == 3, err)
reg4, err = load_bad("""
version: 1
skills:
  - {id: t, name: T, kind: trajectory, source: src/out/does_not_exist.npz}
""")
check("轨迹文件缺失只警告不拒绝",
      reg4 is not None and any("不存在" in w for w in reg4.warnings),
      (reg4.warnings[0] if reg4 and reg4.warnings else err))
check("缺文件的轨迹 ready=False",
      reg4 is not None and reg4.to_public()[0]["ready"] is False)
check("未写 safety 时默认最严",
      reg2.get("only").safety.voice_enabled is False
      and reg2.get("only").safety.need_confirm is True)

# ---------------------------------------------------------------- 汇总
print("\n" + "=" * 60)
print(f"通过 {len(PASS)} · 失败 {len(FAIL)}")
if FAIL:
    for n in FAIL:
        print(f"  ✗ {n}")
    raise SystemExit(1)
print("全部通过")
