#!/usr/bin/env python3
"""sim/lerobot_env.py — 定位 lerobot conda 环境的**唯一真源**。

conda 环境装在哪儿是**机器的属性**,不是仓库的属性。历史上多个文件各自写死
`ros2_ws/enter/envs/lerobot/bin/python3`,环境搬到 `~/miniconda3` 后集体失效,
且要改 6 个地方。现在探测逻辑只有这一份,其它文件一律 import 它。

对外三个函数:
    lerobot_python() -> str          解释器绝对路径
    lerobot_site()   -> Path | None  同环境 site-packages(给 pyAgxArm 补纯 python 依赖)
    lerobot_prefix() -> Path | None  环境根目录

也可当 CLI 用(给 bash 脚本 / 文档命令行调):
    python3 sim/lerobot_env.py --python
    python3 sim/lerobot_env.py --site

本模块**只用标准库**,任何解释器都能跑 —— 它的职责就是找出该用哪个解释器,
所以自己不能依赖被找的那个环境。
"""
from __future__ import annotations

import os
import re
import sys
from pathlib import Path

# 必须和 ROS Humble 的 ABI 一致;跨版本的 site-packages 不能混用。
PREFERRED_PY = "3.10"
ENV_NAME = "lerobot"


def _prefix_candidates() -> list[Path]:
    """conda 环境根目录候选,按优先级。第一个存在的胜出。"""
    home = Path.home()
    cands: list[Path] = []
    if base := os.environ.get("CONDA_BASE"):
        cands.append(Path(base) / "envs" / ENV_NAME)
    cands += [
        home / "miniconda3/envs" / ENV_NAME,
        home / "anaconda3/envs" / ENV_NAME,
        home / "miniforge3/envs" / ENV_NAME,
        home / "mambaforge/envs" / ENV_NAME,
        Path("/opt/conda/envs") / ENV_NAME,
        # 旧布局:环境曾装在仓库内 ros2_ws/enter/。已迁走,留作兜底。
        Path(__file__).resolve().parents[2] / "enter/envs" / ENV_NAME,
    ]
    return cands


def lerobot_prefix() -> Path | None:
    """环境根目录。$LEROBOT_PREFIX > 从 $LEROBOT_PY 反推 > 候选探测。"""
    if env := os.environ.get("LEROBOT_PREFIX"):
        return Path(env)
    if py := os.environ.get("LEROBOT_PY"):
        # <prefix>/bin/python3 -> <prefix>
        p = Path(py).resolve().parent.parent
        if p.is_dir():
            return p
    return next((c for c in _prefix_candidates() if (c / "bin").is_dir()), None)


def lerobot_python() -> str:
    """解释器绝对路径。

    $LEROBOT_PY 显式指定时直接返回, **不回落到探测** —— 用户设错了就该看到设错的那个,
    静默换一个解释器会把错配藏起来。但这里只警告不抛异常:调用方拿它去 spawn 子进程,
    子进程自己会报 FileNotFoundError 并带上路径。

    一个候选都找不到时返回 "python3" 并警告,让调用方拿到一个能跑的字符串。
    """
    if env := os.environ.get("LEROBOT_PY"):
        if not Path(env).is_file():
            print(f"[lerobot_env] warn: $LEROBOT_PY 指向的文件不存在: {env} "
                  f"(显式设置不会被自动替换;unset 它可让本模块自行探测)",
                  file=sys.stderr, flush=True)
        return env
    prefix = lerobot_prefix()
    if prefix is not None:
        for name in (f"python{PREFERRED_PY}", "python3", "python"):
            exe = prefix / "bin" / name
            if exe.is_file():
                return str(exe)
    print(f"[lerobot_env] warn: 未找到 conda 环境 '{ENV_NAME}',回落到 PATH 里的 python3。"
          f"请设 LEROBOT_PY=<env>/bin/python3,或先 source robot_host_env.sh",
          file=sys.stderr, flush=True)
    return "python3"


def lerobot_site() -> Path | None:
    """同环境 site-packages。给 ROS 系统 python3 补 pyAgxArm 的纯 python 依赖用。

    ⚠ 不能用 glob('python3.*') 直接取第一个:环境 lib/ 下有 `python3.1`
    这类残留目录(实测本机就有),会撞上错的。这里只认 `python3.<minor>` 且目录里
    真有 site-packages,优先 PREFERRED_PY,其余按 minor 号从大到小。
    """
    if env := os.environ.get("LEROBOT_SITE"):
        return Path(env)
    prefix = lerobot_prefix()
    if prefix is None:
        return None
    preferred = prefix / f"lib/python{PREFERRED_PY}/site-packages"
    if preferred.is_dir():
        return preferred
    found: list[tuple[int, Path]] = []
    for d in (prefix / "lib").glob("python3.*"):
        m = re.fullmatch(r"python3\.(\d+)", d.name)
        if m and (d / "site-packages").is_dir():
            found.append((int(m.group(1)), d / "site-packages"))
    if not found:
        return None
    return max(found)[1]


def _main() -> int:
    args = sys.argv[1:] or ["--python"]
    what = args[0]
    if what == "--python":
        print(lerobot_python())
    elif what == "--site":
        p = lerobot_site()
        if p is None:
            return 1
        print(p)
    elif what == "--prefix":
        p = lerobot_prefix()
        if p is None:
            return 1
        print(p)
    else:
        print(f"用法: {sys.argv[0]} [--python|--site|--prefix]", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(_main())
