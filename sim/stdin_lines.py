#!/usr/bin/env python3
"""sim/stdin_lines.py — 从 stdin 按行读命令,**不被缓冲吞掉**。

为什么不能直接用 `select([sys.stdin]) + sys.stdin.readline()`:
那两个看的**不是同一层**。select 看的是文件描述符,readline 看的是
sys.stdin(TextIOWrapper)的用户态缓冲。readline 一次会把 fd 上**所有**
已到的字节抽进缓冲(read1 最多 8KB),只返回第一行 —— 剩下的行留在缓冲里,
fd 变空。于是下一轮 select 说"没数据",循环退出,那些行就**卡住了**,
直到下一条命令到达才被读出来。

后果是"慢一条命令":一个手势技能发 3 行(speed/force/angles),
console 只处理掉 speed,force 和 angles 卡在缓冲里 —— 手不动。
下一个手势到了,select 醒来,读出的却是**上一个**手势的 force 和 angles,
手做出上一个动作。只发 1 行的技能(松手/张开手)不会卡,因为它那一行
总是最后被读出来的,所以"松手不慢",而手势和握拳每次都慢一拍。

解法:select 之后用 os.read 直接读 fd,自己攒行。这样 select 看的和读的
是同一层 —— fd 空了就是真的没有完整命令了,缓冲里最多剩半行(还没收完的
那一行),它本来就该等下一块数据。

用法(替换原来的 while select(...) 内层循环):

    reader = StdinLines()
    ...
    for line in reader.poll(timeout):      # timeout 到下一个 tick 的截止
        handle(json.loads(line))
    if reader.eof:                         # 上层退出了
        break
"""
from __future__ import annotations

import os
import select
import sys

CHUNK = 65536


class StdinLines:
    """按行读 stdin,select 与读取同在 fd 层,不留卡住的行。

    poll() 会把**已经到达的所有完整行**一次全给出来 —— 排空语义和原来的
    内层 while 一致(命令一到立刻醒,已到的一起处理完再回去跑 tick)。
    """

    def __init__(self, stream=None) -> None:
        self._fd = (stream or sys.stdin).fileno()
        self._buf = b""
        self.eof = False

    def poll(self, timeout: float) -> list[str]:
        """等最多 timeout 秒,返回已到达的完整行(去掉行尾换行)。

        第一次等 timeout,之后只把已到的排空(timeout=0)—— 不会因为命令
        接连到达就一直不返回,tick 的截止时刻仍然守得住。
        """
        out: list[str] = []
        wait = max(0.0, timeout)
        while not self.eof:
            if not select.select([self._fd], [], [], wait)[0]:
                break
            wait = 0.0
            try:
                chunk = os.read(self._fd, CHUNK)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                self.eof = True
                break
            if not chunk:                       # fd 关闭 = 上层退出
                self.eof = True
                break
            self._buf += chunk
            # 只交出完整行;末尾那段没有换行的留着等下一块数据
            *lines, self._buf = self._buf.split(b"\n")
            for raw in lines:
                s = raw.decode("utf-8", "replace").strip()
                if s:
                    out.append(s)
        return out
