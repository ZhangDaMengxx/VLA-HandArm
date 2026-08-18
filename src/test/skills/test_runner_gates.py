"""测执行器的安全闸(干跑,不需要 ROS/bridge)。

    python3 src/skills/test_runner_gates.py

验证的是「该拒的必须拒」:确认闸、使能表态、语音白名单、语音限速、急停免确认。
这些判断错了,真机上就是没打招呼就动。
"""
import io
import json
import sys
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "skills"))
from runner import SkillRunner  # noqa: E402

PASS, FAIL = [], []
runner = SkillRunner(dry_run=True)


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'✓' if cond else '✗'} {name}" + (f" · {detail}" if detail else ""))


def run(env: dict) -> tuple[dict, list[dict]]:
    """跑一个信封,吞掉 stdout,返回 (结果, 全部事件)。"""
    buf = io.StringIO()
    with redirect_stdout(buf):
        res = runner.invoke(env)
    evs = []
    for ln in buf.getvalue().splitlines():
        try:
            evs.append(json.loads(ln))
        except json.JSONDecodeError:
            pass
    return res, evs


print("\n[1] 确认闸")
r, _ = run({"skill_id": "go_home", "assume_enabled": True})
check("未确认的 go_home 被拒", r["type"] == "error" and "confirmed" in r["msg"])
r, _ = run({"skill_id": "go_home", "confirmed": True})
check("确认但未表态使能 → 被拒",
      r["type"] == "error" and "arm_enabled" in r["msg"])
r, _ = run({"skill_id": "go_home", "confirmed": True, "assume_enabled": True})
check("确认 + 表态使能 → 放行", r["type"] == "done", f"sent={r.get('sent')}")

print("\n[2] 急停")
r, _ = run({"skill_id": "estop"})
check("estop 免确认直接放行", r["type"] == "done")
r, evs = run({"skill_id": "estop"})
cmds = [e for e in evs if e["type"] == "cmd"]
check("estop 只发急停,不带运动目标",
      len(cmds) == 1 and cmds[0]["cmd"] == {"estop": True},
      json.dumps(cmds[0]["cmd"], ensure_ascii=False) if cmds else "无")

print("\n[3] 语音白名单与限速")
r, evs = run({"skill_id": "replay_rgbd_demo", "source": "voice",
              "confirmed": True, "assume_enabled": True,
              "params": {"speed": 4.0}, "transcript": "回放深度示教"})
start = next((e for e in evs if e["type"] == "start"), {})
check("语音路径 speed 被压到 max_speed",
      r["type"] == "done" and any("限速" in n for n in start.get("notes", [])),
      "; ".join(start.get("notes", [])))
r2, evs2 = run({"skill_id": "replay_rgbd_demo", "source": "web",
                "confirmed": True, "assume_enabled": True,
                "params": {"speed": 4.0}})
s2 = next((e for e in evs2 if e["type"] == "start"), {})
check("非语音路径不限速",
      not any("限速" in n for n in s2.get("notes", [])),
      f"est={s2.get('est_seconds')}s vs 语音 {start.get('est_seconds')}s")

print("\n[4] 未知技能与参数")
r, _ = run({"skill_id": "fly_to_moon"})
check("未知技能被拒", r["type"] == "error" and "未知技能" in r["msg"])
r, _ = run({})
check("缺 skill_id 被拒", r["type"] == "error")
r, evs = run({"skill_id": "go_home", "confirmed": True, "assume_enabled": True,
              "params": {"duration": 999}})
cmd = next(e["cmd"] for e in evs if e["type"] == "cmd")
check("参数越界被夹取后才下发", cmd["duration"] == 15.0,
      f"duration={cmd['duration']}")

print("\n[5] 指令内容合法性(下发前的最后一道形状检查)")
r, evs = run({"skill_id": "replay_rgbd_demo", "confirmed": True,
              "assume_enabled": True, "source": "web"})
cmds = [e["cmd"] for e in evs if e["type"] == "cmd"]
check("轨迹步数 = 557 帧 + 1 接近段", len(cmds) == 558, f"{len(cmds)} 条")
check("每条 arm 都是 7 维", all(len(c["arm"]) == 7 for c in cmds))
check("每条 hand 都是 6 维", all(len(c["hand"]) == 6 for c in cmds))
check("首条是接近段(长 duration)", cmds[0]["duration"] == 3.0,
      f"duration={cmds[0]['duration']}")
check("接近段目标 = 轨迹首帧", cmds[0]["arm"] == cmds[1]["arm"])
check("回放段按 1/fps 节拍", abs(cmds[1]["duration"] - 1 / 30) < 1e-6)

print("\n[6] 组合技能按序展开")
r, evs = run({"skill_id": "prepare_arm", "confirmed": True,
              "assume_enabled": True})
cmds = [e["cmd"] for e in evs if e["type"] == "cmd"]
order = [c.get("action") or ("hand" if "hand" in c else "arm") for c in cmds]
check("prepare_arm 5 步顺序正确",
      order == ["reset", "enable", "set_speed", "hand", "arm"], str(order))
check("子步骤参数用清单写死值",
      cmds[-1]["duration"] == 6.0, f"go_home duration={cmds[-1]['duration']}")

print("\n[7] 调用日志(VLA 标注原料)")
from runner import LOG_PATH  # noqa: E402
check("日志文件已写", LOG_PATH.exists(), str(LOG_PATH))
if LOG_PATH.exists():
    lines = LOG_PATH.read_text(encoding="utf-8").strip().splitlines()
    recs = [json.loads(ln) for ln in lines]
    voiced = [r for r in recs if r.get("transcript")]
    check("语音原话被记录", bool(voiced),
          voiced[-1]["transcript"] if voiced else "无")
    check("被拒调用也留痕",
          any(r.get("result") == "gate_rejected" for r in recs))
    check("记录含 skill_id 配对",
          all("skill_id" in r for r in recs))

runner.shutdown()
print("\n" + "=" * 60)
print(f"通过 {len(PASS)} · 失败 {len(FAIL)}")
if FAIL:
    for n in FAIL:
        print(f"  ✗ {n}")
    raise SystemExit(1)
print("全部通过")
