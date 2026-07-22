#!/usr/bin/env python3
"""Gradio 集成前端:拖拽上传视频 → 依次跑 build_canonical / derive_embodiment / replay_rerun,
把 Rerun 的三面板 web 查看器(人手视频+骨架 / 机器人3D / 关节角曲线)嵌进同一个网页。

架构要点(为什么这么分):
  本 app 只做【编排】,自己不 import lerobot / rerun / pinocchio。它用 lerobot 环境的
  python 以 subprocess 调 sim/ 下三个脚本。因此本 app 装在**独立 venv**(~/gradio_venv):
  gradio 依赖新版 huggingface-hub,和 lerobot 需要的旧版天然冲突,隔离开互不影响。

数据流(路径全自动串,见各脚本):
  上传视频 ──► build_canonical.py --video 视频 ──► sim/out/canonical_ds
           ──► derive_embodiment.py --emit-traj ──► sim/out/robot_traj_nero_inspire.pkl
           ──► replay_rerun.py --serve --video 视频 ──► Rerun web 查看器(嵌 iframe)

运行(在 gradio venv 里):
    /home/zhang123/gradio_venv/bin/python \
        /home/zhang123/ros2_ws/lerobotTest/sim/app_gradio.py
然后 Windows 浏览器打开启动时打印的 http://<WSL_IP>:7860
"""
from __future__ import annotations

import atexit
import os
import re
import shutil
import socket
import subprocess
import time
from pathlib import Path

import gradio as gr

# --- 路径 / 解释器(可用环境变量覆盖)---
REPO = Path(os.environ.get("LEROBOT_REPO", "/home/zhang123/ros2_ws/lerobotTest"))
LEROBOT_PY = os.environ.get("LEROBOT_PY",
                            "/home/zhang123/ros2_ws/enter/envs/lerobot/bin/python3")
GRADIO_PORT = int(os.environ.get("GRADIO_PORT", "7860"))

_URL_RE = re.compile(r"(https?://\S+\?url=\S+)")   # replay 打印的完整查看器地址
_replay_proc: subprocess.Popen | None = None


def _primary_ip() -> str:
    """WSL 的非环回 IP —— Windows 浏览器要用它连回来。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def _stop_replay() -> None:
    global _replay_proc
    if _replay_proc and _replay_proc.poll() is None:
        _replay_proc.terminate()
        try:
            _replay_proc.wait(timeout=5)
        except Exception:
            _replay_proc.kill()
    _replay_proc = None


atexit.register(_stop_replay)


def _tail(log: list[str]) -> str:
    return "\n".join(log[-40:])


def _bar(pct: float, caption: str, tone: str = "run") -> str:
    """极简进度条(苹果风:细、圆角、克制配色)。tone: run=蓝 / done=绿 / err=红。"""
    fill = {"run": "#0071e3", "done": "#34c759", "err": "#ff3b30"}.get(tone, "#0071e3")
    pct = max(0, min(100, pct))
    return (
        '<div style="padding:2px 2px 6px">'
        f'<div style="font-size:13px;color:#86868b;margin:0 0 10px;letter-spacing:.01em">{caption}</div>'
        '<div style="height:6px;background:#e9e9ee;border-radius:99px;overflow:hidden">'
        f'<div style="height:100%;width:{pct:.0f}%;background:{fill};'
        'border-radius:99px;transition:width .45s ease"></div></div></div>'
    )


def _creep(floor: float, ceil: float, lines: int, k: float = 60.0) -> float:
    """随读到的行数从 floor 渐近逼近 ceil(每阶段内让进度条有动感,不用预知总量)。"""
    return floor + (ceil - floor) * (1.0 - 1.0 / (1.0 + lines / k))


def _run_bar(cmd, log, caption, floor, ceil, viewer):
    """跑 subprocess,逐行读输出;按行数在 [floor,ceil] 内爬进度条,节流 yield。
    是生成器:中途 yield (进度条html, viewer, 日志);最后 yield ('__ok__', p.returncode==0)。"""
    log.append("$ " + " ".join(cmd))
    yield _bar(floor, caption), viewer, _tail(log)
    p = subprocess.Popen(cmd, cwd=str(REPO), stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, bufsize=1)
    last, lines = 0.0, 0
    assert p.stdout is not None
    for line in p.stdout:
        log.append(line.rstrip())
        lines += 1
        now = time.time()
        if now - last > 0.25:
            last = now
            yield _bar(_creep(floor, ceil, lines), caption), viewer, _tail(log)
    p.wait()
    yield "__ok__", p.returncode == 0


def _start_replay(video: str | None, log: list[str]) -> str | None:
    """后台起 replay_rerun --serve,读 stdout 直到解析出 web 地址,返回 URL(拿到即返回,进程留活)。"""
    global _replay_proc
    _stop_replay()
    cmd = [LEROBOT_PY, "sim/replay_rerun.py", "--serve"]
    if video:
        cmd += ["--video", str(video)]
    log.append("$ " + " ".join(cmd))
    _replay_proc = subprocess.Popen(cmd, cwd=str(REPO), stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, bufsize=1)
    deadline = time.time() + 180        # 它要建模型+加载网格+逐帧记录,给足时间
    assert _replay_proc.stdout is not None
    while time.time() < deadline:
        line = _replay_proc.stdout.readline()
        if not line:
            if _replay_proc.poll() is not None:
                break                    # 进程退了还没给 URL
            continue
        log.append(line.rstrip())
        m = _URL_RE.search(line)
        if m:
            return m.group(1)
    return None


def pipeline(video: str | None, skip_regen: bool):
    """Gradio 事件:生成器,边跑边驱动进度条 + 折叠日志,最后把 Rerun 嵌进查看器。
    输出三件套:(进度条 HTML, 查看器 HTML, 日志文本)。"""
    log: list[str] = []
    if not video:
        yield _bar(0, "请先上传视频(.mp4)", "err"), gr.update(), ""
        return

    if not skip_regen:
        # ① 规范层
        ok = None
        for out in _run_bar([LEROBOT_PY, "sim/build_canonical.py", "--video", str(video)],
                            log, "① 规范层 · 检测人手关键点", 6, 42, gr.update()):
            if out[0] == "__ok__":
                ok = out[1]
            else:
                yield out
        if not ok:
            yield _bar(42, "① 规范层生成失败 · 展开日志看详情", "err"), gr.update(), _tail(log)
            return
        # ② 本体层
        ok = None
        for out in _run_bar([LEROBOT_PY, "sim/derive_embodiment.py", "--emit-traj"],
                            log, "② 本体层 · 逐帧逆解 IK", 45, 80, gr.update()):
            if out[0] == "__ok__":
                ok = out[1]
            else:
                yield out
        if not ok:
            yield _bar(80, "② 本体层生成失败 · 展开日志看详情", "err"), gr.update(), _tail(log)
            return

    # ③ 回放
    yield _bar(84, "③ 启动 Rerun 服务", "run"), gr.update(), _tail(log)
    url = _start_replay(str(video), log)
    if not url:
        yield _bar(84, "③ 未获取到 Rerun 地址 · 展开日志看详情", "err"), gr.update(), _tail(log)
        return
    iframe = (f'<iframe src="{url}" width="100%" height="820" '
              f'style="border:0;border-radius:14px"></iframe>')
    yield _bar(100, "完成 · 已加载三面板", "done"), iframe, _tail(log)


_CSS = """
.gradio-container {max-width: 1180px !important; margin: 0 auto !important;
  font-family: -apple-system, BlinkMacSystemFont, "SF Pro Display", "Helvetica Neue", "PingFang SC", sans-serif;}
footer {display: none !important;}
.gradio-container .prose {color:#1d1d1f;}
"""

_PLACEHOLDER = ('<div style="height:820px;border-radius:14px;background:#f5f5f7;display:flex;'
                'align-items:center;justify-content:center;color:#a1a1a6;font-size:14px">'
                '三面板将在这里显示</div>')


def build_ui() -> gr.Blocks:
    with gr.Blocks(title="手势 → 机器人回放") as demo:   # theme/css 在 launch() 传(gradio 6)
        gr.HTML('<div style="padding:12px 2px 4px"><span style="font-size:21px;font-weight:600;'
                'letter-spacing:-.02em;color:#1d1d1f">手势 → 机器人回放</span></div>')
        with gr.Row(equal_height=False):
            with gr.Column(scale=1, min_width=280):
                video = gr.Video(sources=["upload"], label="视频", height=190)
                skip = gr.Checkbox(value=False, label="跳过重算(没换视频时)")
                run = gr.Button("生成并可视化", variant="primary", size="lg")
                progress = gr.HTML(_bar(0, "上传视频后点『生成并可视化』"))
                with gr.Accordion("运行日志", open=False):
                    logbox = gr.Textbox(show_label=False, lines=12, max_lines=12,
                                        container=False, autoscroll=True)
            with gr.Column(scale=3):
                viewer = gr.HTML(_PLACEHOLDER)
        run.click(pipeline, inputs=[video, skip], outputs=[progress, viewer, logbox])
    return demo


def main() -> None:
    ip = _primary_ip()
    print("=" * 72, flush=True)
    print(f"  Gradio 启动中。Windows 浏览器打开:  http://{ip}:{GRADIO_PORT}", flush=True)
    print("  (若 localhost 打不开就用上面这个 WSL IP 地址)", flush=True)
    print("=" * 72, flush=True)
    theme = gr.themes.Soft(
        primary_hue="blue", neutral_hue="slate", radius_size="lg",
        font=["-apple-system", "BlinkMacSystemFont", "SF Pro Display",
              "Helvetica Neue", "sans-serif"],
    )
    build_ui().launch(server_name="0.0.0.0", server_port=GRADIO_PORT,
                      theme=theme, css=_CSS)


if __name__ == "__main__":
    main()
