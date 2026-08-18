#!/usr/bin/env python3
"""测 stdin 读命令不落后 —— 一次写进多行必须当轮全处理完。

    python3 -m pytest src/test/test_stdin_lines.py

这是回归测试,针对一个真机上看得见的 bug:手势技能发 3 行
(speed/force/angles),console 只处理掉第一行,后两行卡在
sys.stdin 的用户态缓冲里(select 看 fd,已经空了),要等**下一条**
命令到达才被读出来 —— 表现为「手慢一个命令」:发『比个1』再发
『握拳』,手先不动,然后做出『比个1』,然后又不动。
只发 1 行的『松手』不会卡,所以它看着是正常的。

用真子进程测,因为 bug 就在"跨进程管道 + select 与缓冲不同层"这件事上,
在同一进程里拿 StringIO 测不出来。
"""
import json
import subprocess
import sys
import textwrap
import time
from pathlib import Path

SIM = Path(__file__).resolve().parents[1]
PASS, FAIL = [], []


def check(name: str, cond: bool, detail: str = "") -> None:
    (PASS if cond else FAIL).append(name)
    print(f"  {'✓' if cond else '✗'} {name}" + (f" · {detail}" if detail else ""))


# 复刻 console 主循环的骨架:等到下一个 tick 截止,期间来的命令立刻处理。
CHILD = textwrap.dedent(f"""
    import json, sys, time
    sys.path.insert(0, {str(SIM)!r})
    from stdin_lines import StdinLines
    r = StdinLines()
    nxt = time.monotonic()
    while True:
        for line in r.poll(max(0.0, nxt - time.monotonic())):
            print(json.dumps({{"got": json.loads(line)["cmd"]}}), flush=True)
        if r.eof:
            break
        now = time.monotonic()
        if now >= nxt:
            nxt = now + 1.0 / 30.0
""")


def run_child(batches: list[list[dict]], gap: float = 1.0) -> list[list[str]]:
    """每个 batch 连着写(一次 flush 一条),batch 之间隔 gap 秒。
    返回每个 batch 之后新收到的命令名。"""
    p = subprocess.Popen([sys.executable, "-c", CHILD],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         text=True, bufsize=1)
    seen: list[str] = []
    out: list[list[str]] = []
    import threading

    def pump() -> None:
        for line in p.stdout:
            try:
                seen.append(json.loads(line)["got"])
            except (json.JSONDecodeError, KeyError):
                pass

    threading.Thread(target=pump, daemon=True).start()
    time.sleep(0.3)                       # 等子进程起来
    for batch in batches:
        mark = len(seen)
        for cmd in batch:
            p.stdin.write(json.dumps(cmd) + "\n")
            p.stdin.flush()
        time.sleep(gap)
        out.append(seen[mark:])
    p.stdin.close()
    p.wait(timeout=5)
    return out


def main() -> int:
    print("一次写进多行(手势 = speed/force/angles):")
    gesture = [{"cmd": "speed"}, {"cmd": "force"}, {"cmd": "angles"}]
    got = run_child([gesture])
    check("3 行连发,当轮就全处理完(不是只处理 1 行)",
          got[0] == ["speed", "force", "angles"], f"实收 {got[0]}")

    print("\n两个手势技能先后发(复现「慢一个命令」的场景):")
    got = run_child([gesture, [{"cmd": "speed"}, {"cmd": "force"},
                               {"cmd": "angles2"}]])
    check("第一个技能的 angles 不会拖到第二个技能才生效",
          got[0][-1:] == ["angles"], f"技能A 收到 {got[0]}")
    check("第二个技能自己那轮就收到自己的 angles",
          got[1][-1:] == ["angles2"], f"技能B 收到 {got[1]}")

    print("\n半行(TCP/管道切在中间)要等收全,不能当成完整命令:")
    p = subprocess.Popen([sys.executable, "-c", CHILD],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         text=True, bufsize=1)
    time.sleep(0.3)
    p.stdin.write('{"cmd": "hal')                 # 故意断在中间
    p.stdin.flush()
    time.sleep(0.4)
    p.stdin.write('f"}\n')                        # 补齐
    p.stdin.flush()
    time.sleep(0.4)
    p.stdin.close()
    out = p.stdout.read()
    p.wait(timeout=5)
    check("半行补齐后被当作一条完整命令处理",
          '"got": "half"' in out, f"输出 {out.strip()!r}")

    print(f"\n{len(PASS)} 通过 / {len(FAIL)} 失败")
    for f in FAIL:
        print(f"  ✗ {f}")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
