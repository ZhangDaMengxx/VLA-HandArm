#!/usr/bin/env python3
"""app_web.py — NERO·Inspire 回放工作台 Web 前端(FastAPI + SSE)。

布局仿 1.html 的全屏悬浮结构,配色按『可视化工作台-提示词.md』的白色极简科技风。
后端管线复用 app_gradio.py:subprocess 依次跑 build_canonical / derive_embodiment /
replay_rerun --serve,用 SSE 把进度/日志/Rerun 地址推给浏览器。Web、视觉、实时 IK 和直接
CAN/串口控制使用 lerobot-v3；只有 rclpy reader/writer/runner 在后台使用 ROS Humble
Python 3.10，均靠 subprocess 隔离。

运行:
    conda activate lerobot-v3
    python src/lerobot_v3/app_web.py
然后 Windows 浏览器打开启动时打印的 http://<WSL_IP>:7860
"""
from __future__ import annotations

import asyncio
import atexit
import concurrent.futures
import json
import os
import re
import shlex
import shutil
import socket
import subprocess
import tempfile
import time
from pathlib import Path

import threading

import numpy as np
import uvicorn
from fastapi import FastAPI, File, Form, Header, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

# 技能清单:纯 Python + PyYAML,本环境可直接 import(不牵连 rclpy/rerun)。
import sys as _sys
_sys.path.insert(0, str(Path(__file__).resolve().parent))
from skills.schema import RegistryError, get_registry   # noqa: E402
# 语音路径:意图解析 + console 执行适配。两者都是纯 Python(不 import rclpy),
# 所以能直接跑在 V3 主进程里,不必像 runner 那样起 ROS 子进程。
# ⚠ 坑:下面这几行会让 backend/runner 在模块级把 src/skills 和 sim 都插进 sys.path,
#   于是本进程里裸 `import schema` 解析到的是 **skills/schema.py**(backend 先把它
#   缓存进 sys.modules),不是数据集那份 src/schema.py。app_web 不用数据集 schema,
#   目前无碰撞;哪天要用,得写显式路径,别再靠导入顺序碰运气。
from skills.console_exec import ConsoleExecutor            # noqa: E402
from skills.console_exec import targets as console_targets  # noqa: E402
from skills.intent import PACK_DEVICES, PACK_KINDS         # noqa: E402
from skills.intent import PackTarget                       # noqa: E402
from skills.intent import parse as intent_parse            # noqa: E402
from skills.runner import _log_invocation, log_parse       # noqa: E402
from lerobot_v3.env import lerobot_v3_python            # noqa: E402
from ros_humble_env import ros_humble_python, ros_humble_setup, ros_log_dir  # noqa: E402
from hand_target_mailbox import HandTarget, LatestTargetMailbox  # noqa: E402
from hand_target_filter import OneEuroJointFilter  # noqa: E402
from hardware_lease import HardwareLease  # noqa: E402
from live_ik_scheduler import IKTarget, LatestIKScheduler  # noqa: E402
from nero_arm import NERO_HOME_POSE, NERO_TRACKING_READY_POSE  # noqa: E402
from live_wrist_tracking import (  # noqa: E402
    LiveWristMapper,
    OneEuroRotationFilter,
    OneEuroVectorFilter,
    WristObservation,
    estimate_wrist_observation,
)
from capture_bundle import (  # noqa: E402
    create_capture,
    latest_capture,
    resolve_data_paths,
)

# --- 路径 / 解释器(可用环境变量覆盖)---
REPO = Path(os.environ.get("LEROBOT_REPO", "/home/zhang123/ros2_ws/lerobotTest"))


LEROBOT_V3_PY = lerobot_v3_python()
# 主运行时默认就是 V3；保留 VLA_RUNTIME_PY 仅用于诊断或受控迁移。
VLA_RUNTIME_PY = os.environ.get("VLA_RUNTIME_PY", LEROBOT_V3_PY)
WEB_PORT = int(os.environ.get("WEB_PORT", "7860"))
WEB_DIR = Path(__file__).resolve().parent / "web"
SIM = Path(__file__).resolve().parent
# ROS2 子进程固定走 Humble 的 Python 3.10 ABI。解释器与 setup 均可显式覆盖，默认优先
# 使用命名环境 ros-humble，再回落仓库 venv 或 /usr/bin/python3。
ROS_SETUP = ros_humble_setup()
ROS_PYTHON = ros_humble_python()


def _ros_cmd(script_argv: list[str]) -> list[str]:
    """把 ROS2 python 脚本包进 bash -lc 'source ...; exec python3 ...',保证 rclpy 可见。

    每个参数都过 shlex.quote:参数里可能带空格/引号(如技能调用信封的 JSON,内含
    语音原话),不引用会被 bash 拆成多个参数,且构成注入面。
    """
    log_dir = ros_log_dir()
    argv = " ".join(shlex.quote(a) for a in script_argv)
    inner = (f"export ROS_LOG_DIR={shlex.quote(str(log_dir))} && {ROS_SETUP} "
             f"&& exec {shlex.quote(ROS_PYTHON)} {argv}")
    return ["bash", "-lc", inner]

_URL_RE = re.compile(r"(https?://\S+\?url=\S+)")   # replay 打印的完整查看器地址
GRPC_PORT = int(os.environ.get("RERUN_GRPC_PORT", "9876"))
WEB_VIEWER_PORT = int(os.environ.get("RERUN_WEB_PORT", "9090"))
_replay_proc: subprocess.Popen | None = None
_player_proc: subprocess.Popen | None = None      # 轨迹下发(traj_player)进程
_executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)

# 数据源 -> (机器人规格名, build 脚本参数)。RGB 走 legacy 相对(可达),RGB-D 走 metric 几何。
DATASETS = {
    "rgb":  {"robot": "nero_inspire_rgb",  "label": "普通 RGB"},
    "rgbd": {"robot": "nero_inspire_rgbd", "label": "kinect RGB-D"},
}
# RGB-D 输入是 kinect 目录(非上传视频),给默认值,可用环境变量覆盖或前端手动指定。
RGBD_INPUT_ROOT = os.environ.get(
    "RGBD_INPUT_ROOT", "third_party/kinect2-middle/kinect2_middle"
)
RGBD_CAMERA = os.environ.get("RGBD_CAMERA", "kinect2_middle")
CAPTURES_ROOT = REPO / "datasets/captures"
LEGACY_DATA_MODE = os.environ.get("VLA_LEGACY_OUT", "").strip().lower() in {
    "1", "true", "yes", "on",
}
_active_capture = None if LEGACY_DATA_MODE else latest_capture(CAPTURES_ROOT)


def _web_data_paths(robot: str):
    """Resolve every replay artifact from one active Capture (or explicit legacy mode)."""
    capture_root = _active_capture.root if _active_capture is not None else None
    return resolve_data_paths(
        robot,
        capture_root=capture_root,
        legacy_out=LEGACY_DATA_MODE,
        captures_root=CAPTURES_ROOT,
    )


def _traj_pkl(robot: str) -> Path:
    """derive_embodiment --emit-traj 产出的 pkl(按规格名);replay/play 都据此定位。"""
    try:
        return _web_data_paths(robot).trajectory_pkl
    except (ValueError, FileNotFoundError):
        return CAPTURES_ROOT / "_no_capture" / robot / "robot_traj.pkl"


def _metrics_json(robot: str) -> Path:
    """measure_acceptance --json 产出的验收指标缓存(按规格名);右侧验收卡据此显示。"""
    try:
        return _web_data_paths(robot).quality_report
    except (ValueError, FileNotFoundError):
        return CAPTURES_ROOT / "_no_capture" / robot / "metrics.json"


def _canonical_root(robot: str) -> Path:
    try:
        return _web_data_paths(robot).canonical_root
    except (ValueError, FileNotFoundError):
        return CAPTURES_ROOT / "_no_capture" / "ego"


def _original_video(robot: str) -> Path:
    try:
        paths = _web_data_paths(robot)
    except (ValueError, FileNotFoundError):
        return CAPTURES_ROOT / "_no_capture" / "recording_000000.mp4"
    if paths.original_video.exists():
        return paths.original_video
    candidates = sorted(paths.original_video.parent.glob("recording_000000.*"))
    return candidates[0] if candidates else paths.original_video


# /api/replay/play 用的 npz,随最近一次成功管线的机器人规格更新。
TRAJ_NPZ = _traj_pkl("nero_inspire_rgb").with_suffix(".npz")


def _primary_ip() -> str:
    """WSL 的非环回 IP —— Windows 浏览器要用它连回来。"""
    s: socket.socket | None = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        if s is not None:
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


def _stop_player() -> None:
    global _player_proc
    if _player_proc and _player_proc.poll() is None:
        _player_proc.terminate()
        try:
            _player_proc.wait(timeout=5)
        except Exception:
            _player_proc.kill()
    _player_proc = None


atexit.register(_stop_replay)
atexit.register(_stop_player)


HAND_GRPC_PORT = GRPC_PORT + 5          # 手部调试用独立端口,不和 replay/live 抢
HAND_WEB_PORT = WEB_VIEWER_PORT + 5


def _free_ports(log: list[str] | None = None, ports: tuple[int, ...] | None = None) -> None:
    """强制释放 Rerun 端口 —— 干掉任何还占着 grpc/web 端口的残留 replay 进程
    (包括 app_gradio.py 或上次崩溃留下的孤儿),否则新 serve 会因端口占用而启动失败。

    ports 不给就清 replay/live 那两个。手部会话用的是 HAND_* 那对,必须显式传 ——
    之前漏了这点,手部会话重启第二次会因端口还被自己的孤儿占着而起不来。"""
    for port in (ports or (GRPC_PORT, WEB_VIEWER_PORT)):
        try:
            subprocess.run(["fuser", "-k", f"{port}/tcp"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
        except Exception:                                    # noqa: BLE001
            pass
    if log is not None:
        log.append(f"[释放端口 {GRPC_PORT}/{WEB_VIEWER_PORT}]")
    time.sleep(0.8)                                          # 给内核回收端口留点时间


def _creep(floor: float, ceil: float, lines: int, k: float = 60.0) -> float:
    """随读到的行数从 floor 渐近逼近 ceil(每阶段内让进度条有动感,不用预知总量)。"""
    return floor + (ceil - floor) * (1.0 - 1.0 / (1.0 + lines / k))


def _run_step(cmd, log, caption, floor, ceil, emit) -> bool:
    """跑 subprocess,逐行读输出,按行数在 [floor,ceil] 内爬进度并 emit 事件。成功返回 True。"""
    log.append("$ " + " ".join(cmd))
    emit({"type": "progress", "pct": floor, "msg": caption})
    p = subprocess.Popen(cmd, cwd=str(REPO), stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, bufsize=1)
    last, lines = 0.0, 0
    assert p.stdout is not None
    for line in p.stdout:
        txt = line.rstrip()
        log.append(txt)
        lines += 1
        emit({"type": "log", "line": txt})
        now = time.time()
        if now - last > 0.25:
            last = now
            emit({"type": "progress", "pct": _creep(floor, ceil, lines), "msg": caption})
    p.wait()
    return p.returncode == 0


def _start_replay(video: str | None, log: list[str], robot: str = "nero_inspire_rgb",
                  no_video: bool = False) -> str | None:
    """后台起 replay_rerun --serve,读 stdout 直到解析出 web 地址,返回 URL(进程留活)。
    按 robot 规格加载对应的 robot_traj_<robot>.pkl,并把 --robot 传给 replay 保持映射一致。"""
    global _replay_proc
    _stop_replay()
    _free_ports(log)                    # 清掉任何孤儿 replay,避免端口占用导致启动失败
    cmd = [LEROBOT_V3_PY, "src/lerobot_v3/replay_rerun.py", "--serve",
           "--grpc-port", str(GRPC_PORT), "--web-port", str(WEB_VIEWER_PORT),
           "--robot", robot,
           "--traj", f"{robot}={_traj_pkl(robot)}"]
    if LEGACY_DATA_MODE:
        cmd.append("--legacy-out")
    elif _active_capture is not None:
        cmd += ["--capture-root", str(_active_capture.root)]
    if no_video:
        cmd += ["--no-video", "--no-skeleton"]
    elif video:
        cmd += ["--video", str(video)]
    log.append("$ " + " ".join(cmd))
    _replay_proc = subprocess.Popen(cmd, cwd=str(REPO), stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, bufsize=1)
    deadline = time.time() + 180        # 建模型+加载网格+逐帧记录,给足时间
    assert _replay_proc.stdout is not None
    while time.time() < deadline:
        line = _replay_proc.stdout.readline()
        if not line:
            if _replay_proc.poll() is not None:
                break
            continue
        log.append(line.rstrip())
        m = _URL_RE.search(line)
        if m:
            return m.group(1)
    return None


def run_pipeline(input_path: str | None, skip_regen: bool, dataset: str, emit,
                 source: str = "video", rgbd_dir: str = "", rgbd_camera: str = "",
                 hand: str = "inspire") -> None:
    """编排三步管线,全程 emit 事件:progress / log / rerun_url / done / error。
    dataset='rgb'(上传视频,legacy 相对)或 'rgbd'(手动指定 kinect 目录,metric 几何)。
    hand='inspire'(灵巧手 state=13)或 'gripper'(平行夹爪 state=8);本体 = nero_{hand}_{rgb|rgbd}。
    rgbd_dir/rgbd_camera 为空时回退环境变量默认。"""
    global TRAJ_NPZ, _active_capture
    log: list[str] = []
    ds = DATASETS.get(dataset, DATASETS["rgb"])
    is_rgbd = dataset == "rgbd"
    hand = hand if hand in ("inspire", "gripper") else "inspire"
    robot = f"nero_{hand}_{'rgbd' if is_rgbd else 'rgb'}"   # 本体×数据源组合出 RobotSpec 名
    # RGB-D 用指定 kinect 目录,不需要上传;RGB / 手部结果需要输入文件。
    if not is_rgbd and not input_path:
        emit({"type": "error", "msg": "请先上传视频或处理结果文件"})
        return
    rgbd_root = (rgbd_dir or RGBD_INPUT_ROOT).strip()
    rgbd_cam = (rgbd_camera or RGBD_CAMERA).strip()
    if is_rgbd and not (REPO / rgbd_root).exists() and not Path(rgbd_root).exists():
        emit({"type": "error", "msg": f"RGB-D 目录不存在: {rgbd_root}"})
        return
    if LEGACY_DATA_MODE:
        capture = None
        capture_args = ["--legacy-out"]
        data_paths = resolve_data_paths(robot, legacy_out=True)
    elif skip_regen:
        capture = _active_capture or latest_capture(CAPTURES_ROOT)
        if capture is None:
            emit({"type": "error", "msg": "没有可跳过生成的 Capture;请先运行一次完整管线"})
            return
        capture_args = ["--capture-root", str(capture.root)]
        data_paths = resolve_data_paths(robot, capture_root=capture.root)
    else:
        capture = create_capture(CAPTURES_ROOT)
        capture_args = ["--capture-root", str(capture.root)]
        data_paths = resolve_data_paths(robot, capture_root=capture.root)
        emit({"type": "log", "line": f"Capture: {capture.capture_id}"})
    if not skip_regen:
        if is_rgbd:
            cmd = [LEROBOT_V3_PY, "src/lerobot_v3/build_canonical_from_rgbd.py",
                   "--input-root", rgbd_root, "--camera", rgbd_cam, *capture_args]
            caption = f"① 规范层 · {ds['label']} 深度反投手部 · {rgbd_root}"
        elif source == "handfile":
            cmd = [LEROBOT_V3_PY, "src/lerobot_v3/build_canonical_from_processed.py", "--input", str(input_path),
                   *capture_args]
            caption = "① 规范层 · 导入外部手部结果"
        else:
            cmd = [LEROBOT_V3_PY, "src/lerobot_v3/build_canonical.py", "--video", str(input_path), *capture_args]
            caption = "① 规范层 · 检测人手关键点"
        if not _run_step(cmd, log, caption, 6, 42, emit):
            emit({"type": "error", "msg": "① 规范层生成失败 · 看日志"})
            return
        if not _run_step([LEROBOT_V3_PY, "src/lerobot_v3/derive_embodiment.py", "--robot", robot,
                          "--emit-traj", *capture_args],
                         log, f"② 本体层 · {ds['label']} 逐帧逆解 IK", 45, 78, emit):
            emit({"type": "error", "msg": "② 本体层生成失败 · 看日志"})
            return
        # ②' 验收指标:按本体测数据有效性,缓存 JSON 供右侧验收卡显示(失败不阻断管线)
        _run_step([LEROBOT_V3_PY, "src/lerobot_v3/measure_acceptance.py",
                   "--robot", robot, "--json", str(data_paths.quality_report), *capture_args],
                  log, f"②' 验收 · {ds['label']} 数据有效性指标", 78, 82, emit)
    TRAJ_NPZ = data_paths.trajectory_npz   # 供 /api/replay/play 定位
    # ③ 回放数据自检。**不再起 replay_rerun --serve** —— 回放页改成 three.js(replay3d.js)
    # 直接按帧号拉 /api/replay/keypoints + /api/traj/frames + 视频,全走 7860 同源。
    # 原来这步要加载 9MB mesh、白等最多 180s 的 URL、还占着 9090/9876 两个端口,
    # 远程访问时那两个端口还得再套一层转发 —— 现在整条都省了。
    emit({"type": "progress", "pct": 88, "msg": "③ 回放数据自检"})
    missing = [str(p.name) for p in (TRAJ_NPZ, data_paths.canonical_root / "meta/info.json")
               if not p.exists()]
    if missing:
        emit({"type": "error", "msg": f"③ 回放数据缺失: {', '.join(missing)} · 看日志"})
        return
    if capture is not None:
        _active_capture = capture
    emit({"type": "replay_ready", "robot": robot,
          "capture_id": capture.capture_id if capture is not None else "legacy"})
    emit({"type": "progress", "pct": 100, "msg": "完成 · 已加载三面板"})
    emit({"type": "done"})


# ---------------------------------------------------------------------------
# 实时监控:ros_joint_reader(ROS2)stdout → 转发到 WebSocket 客户端 + live_rerun stdin
# ---------------------------------------------------------------------------
class LiveSession:
    """一个实时会话:拉起 reader(ROS2 /joint_states)与 live_rerun(conda FK+serve),
    把 reader 的每帧 JSON 既广播给所有 WebSocket 客户端(数值),又喂给 live_rerun(3D)。
    线程读子进程 stdout,用 loop.call_soon_threadsafe 把帧塞进各客户端的 asyncio 队列。"""

    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop
        self.reader: subprocess.Popen | None = None
        self.live: subprocess.Popen | None = None
        self.writer: subprocess.Popen | None = None      # 控制指令持久写进程
        self.clients: set[asyncio.Queue] = set()
        self.latest: dict | None = None
        self.rerun_url: str | None = None
        self._threads: list[threading.Thread] = []
        self._running = False
        # 使能状态:bridge 不发布它,ROS 侧查不到。这里按「本会话发过什么指令」跟踪,
        # 作为技能 requires:arm_enabled 的依据。会话重启即回到 False。
        self.arm_enabled = False

    # ---- 生命周期 ----
    def start(self) -> None:
        if self._running:
            return
        _free_ports()                                     # 清 Rerun 端口
        self._running = True
        # live_rerun:conda python,serve 3D;stdin 收关节流
        # --view-hz:3D 抽帧上限(100Hz 数据也只按此刷);--mem-limit:Rerun 滑动窗口,防内存无界增长卡死
        self.live = subprocess.Popen(
            [VLA_RUNTIME_PY, "src/live_rerun.py", "--serve",
             "--grpc-port", str(GRPC_PORT), "--web-port", str(WEB_VIEWER_PORT),
             "--view-hz", "30", "--mem-limit", "500MB"],
            cwd=str(REPO), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1)
        # reader:ROS2 python,订阅 /joint_states → stdout
        self.reader = subprocess.Popen(
            _ros_cmd(["src/ros_joint_reader.py"]),
            cwd=str(REPO), stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1)
        self._ensure_writer()                             # 预热 writer,首次 jog 不丢/不卡
        self._threads = [
            threading.Thread(target=self._pump_reader, daemon=True),
            threading.Thread(target=self._pump_live_url, daemon=True),
        ]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self._running = False
        for p in (self.reader, self.live, self.writer):
            if p and p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=4)
                except Exception:
                    p.kill()
        self.reader = self.live = self.writer = None
        self.rerun_url = None
        self.arm_enabled = False          # 会话结束,使能状态的记录不再可信
        _free_ports()

    # ---- 后台线程 ----
    def _pump_reader(self) -> None:
        """读 reader stdout:每帧 JSON → 喂 live_rerun stdin + 广播给 WebSocket 客户端。"""
        assert self.reader is not None and self.reader.stdout is not None
        for line in self.reader.stdout:
            if not self._running:
                break
            line = line.strip()
            if not line:
                continue
            # 喂 live_rerun 的 3D(带换行)
            if self.live and self.live.stdin and self.live.poll() is None:
                try:
                    self.live.stdin.write(line + "\n")
                    self.live.stdin.flush()
                except Exception:
                    pass
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if "pos" not in row:
                continue                                  # 状态行
            self.latest = row
            self.loop.call_soon_threadsafe(self._broadcast, row)

    def _pump_live_url(self) -> None:
        """读 live_rerun stdout,抓出查看器 URL。"""
        assert self.live is not None and self.live.stdout is not None
        for line in self.live.stdout:
            if not self._running:
                break
            m = _URL_RE.search(line)
            if m:
                self.rerun_url = m.group(1)

    def _broadcast(self, row: dict) -> None:
        for q in list(self.clients):
            if q.full():
                try:
                    q.get_nowait()                        # 丢最旧,保最新(实时优先)
                except Exception:
                    pass
            q.put_nowait(row)

    # ---- 控制 ----
    def _ensure_writer(self) -> None:
        if self.writer and self.writer.poll() is None:
            return
        # stdout/stderr → DEVNULL:writer 的响应我们不用,且未排空的 PIPE 填满会阻塞 writer
        self.writer = subprocess.Popen(
            _ros_cmd(["src/ros_joint_writer.py"]),
            cwd=str(REPO), stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, text=True, bufsize=1)

    def command(self, cmd: dict) -> dict:
        """把一条控制指令写给 writer 子进程(流式)。返回是否已投递。"""
        self._ensure_writer()
        if not (self.writer and self.writer.stdin):
            return {"ok": False, "msg": "writer 未启动"}
        try:
            self.writer.stdin.write(json.dumps(cmd) + "\n")
            self.writer.stdin.flush()
            self.note_command(cmd)
            return {"ok": True, "sent": cmd}
        except Exception as e:                            # noqa: BLE001
            return {"ok": False, "msg": str(e)}

    def note_command(self, cmd: dict) -> None:
        """按下发的指令更新使能状态跟踪。技能执行器那边也会回调这个。"""
        act = cmd.get("action")
        if act == "enable":
            self.arm_enabled = True
        elif act in ("disable",) or cmd.get("estop"):
            self.arm_enabled = False      # 急停/下使能后必须重新使能才算就绪


_live: LiveSession | None = None


def _get_live() -> LiveSession:
    global _live
    if _live is None:
        _live = LiveSession(asyncio.get_event_loop())
    return _live


# ---------------------------------------------------------------------------
# 手部调试模式:hand_console(串口) + hand_rerun(3D)
# ---------------------------------------------------------------------------
class HandDebugSession:
    """灵巧手调试会话:console 独占串口,rerun 显示 3D,都走子进程。
    和 LiveSession 的区别:不走 ROS,不需要臂,只要 V3 主环境里的 pyserial。"""

    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop
        self.console: subprocess.Popen | None = None   # hand_console.py
        self.rerun: subprocess.Popen | None = None     # hand_rerun.py
        self.rerun_url: str | None = None
        self.clients: set[asyncio.Queue] = set()       # WebSocket 遥测订阅
        self._running = False
        self._threads: list[threading.Thread] = []
        self._actions: list[dict] = []                 # 缓存动作序列列表
        self.mock = False
        self.ready = False                             # console 报了 ready(串口已开)
        self.error: str | None = None                  # 串口打不开等致命错误
        self.port: str | None = None
        self.latest: dict | None = None                # 最近一帧遥测,给刚连上的客户端
        self._last_hw_tracking_log = 0.0
        self._stdin_lock = threading.Lock()
        self._ack_waiters: dict[str, asyncio.Future] = {}
        self._ack_seq = 0

    def start(self, mock: bool = False) -> None:
        """拉起会话:只起 hand_console(串口)。mock=False(默认)= 真开 /dev/ttyUSB0。

        **不再起 hand_rerun** —— 3D 改成浏览器端 three.js 直接渲染(web/hand3d.js),
        少一个 conda 子进程、少一个 WASM 查看器,也不占 Rerun 端口。
        hand_rerun.py 保留,命令行仍可用(`hand_console.py | hand_rerun.py --serve`)。

        串口开成没开成由 console 的第一条消息决定(ready / error fatal),
        这里只负责起进程 —— 结果通过 self.error / self.ready 反映,端点据此回报,
        不能默认"起了进程就是在线"。"""
        if self._running:
            return
        self._running = True
        self.mock = mock
        self.error = None
        self.ready = False
        # console:系统 python3(pyserial 在那儿),独占 RS485 串口
        flag = "--mock" if mock else "--no-mock"
        # --hz 决定 ActionPlayer.tick() 的调用周期 = 回放的时间分辨率。
        # 取 gesture_pack.CONSOLE_HZ(30)而不是写死 20:20Hz 的 tick 是 50ms,
        # 回放 30fps 源视频(帧间 33ms)时每帧都得等到 50ms,整段慢 1.5× —— 就是
        # 用户看到的"很延迟"。两处必须一致,所以从那边取常量,别各写一个数。
        from gesture_pack import CONSOLE_HZ, PLAYER_HZ
        self.console = subprocess.Popen(
            ["python3", "src/hand_console.py", flag,
             "--hz", str(CONSOLE_HZ), "--player-hz", str(PLAYER_HZ)],
            cwd=str(REPO), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self._threads = [threading.Thread(target=self._pump_console, daemon=True)]
        for t in self._threads:
            t.start()

    def wait_ready(self, timeout: float = 6.0) -> None:
        """阻塞等 console 的第一条消息(ready 或 error)。给端点判断串口开没开成。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.ready or self.error:
                return
            if self.console is not None and self.console.poll() is not None:
                if not self.error:
                    self.error = "hand_console 启动即退出"
                return
            time.sleep(0.1)
        if not self.ready and not self.error:
            self.error = f"等 hand_console 就绪超时({timeout:.0f}s)"

    def stop(self, *, home: bool = True) -> None:
        self._running = False
        try:
            self.loop.call_soon_threadsafe(self._cancel_ack_waiters)
        except RuntimeError:
            pass
        # 主动退出默认复位；watchdog 超时只释放串口，不发新的位置目标。
        if self.console and self.console.stdin and self.console.poll() is None:
            try:
                with self._stdin_lock:
                    self.console.stdin.write(json.dumps({"cmd": "quit", "home": home}) + "\n")
                    self.console.stdin.flush()
                self.console.wait(timeout=2)            # 等它按退出模式收尾并断开
            except Exception:                           # noqa: BLE001
                pass
        for p in (self.console, self.rerun):
            if p and p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=3)
                except Exception:                       # noqa: BLE001
                    p.kill()
        self.console = self.rerun = None
        self.rerun_url = None
        self.ready = False
        self.latest = None
        _free_ports(ports=(HAND_GRPC_PORT, HAND_WEB_PORT))

    def _pump_console(self) -> None:
        """读 console stdout:每帧 JSON → 喂 rerun stdin + 广播给 WebSocket 客户端。"""
        assert self.console is not None and self.console.stdout is not None
        for line in self.console.stdout:
            if not self._running:
                break
            line = line.strip()
            if not line:
                continue
            # 喂 rerun
            if self.rerun and self.rerun.stdin and self.rerun.poll() is None:
                try:
                    self.rerun.stdin.write(line + "\n")
                    self.rerun.stdin.flush()
                except Exception:                       # noqa: BLE001
                    pass
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            typ = row.get("type")
            if typ == "ready":
                self.ready = True                       # 串口已开(mock 下是"没开也算就绪")
                self.port = row.get("port")
                if "actions" in row:
                    self._actions = row["actions"]
            elif typ == "error" and row.get("fatal"):
                self.error = row.get("msg") or "串口打开失败"
            elif typ == "state":
                self.latest = row
                perf = row.get("tracking_perf") or {}
                now = time.monotonic()
                if perf and now - self._last_hw_tracking_log >= 0.5:
                    self._last_hw_tracking_log = now
                    settled = perf.get("settled_ms")
                    print(
                        "[perf-hand/tracking] "
                        f"id={perf.get('id')} target_age={perf.get('target_age_ms')}ms "
                        f"mean_err={perf.get('mean_err_rad')}rad "
                        f"max_err={perf.get('max_err_rad')}rad "
                        f"settled={settled if settled is not None else 'pending'}ms",
                        flush=True,
                    )
            elif typ == "ack" and row.get("cmd") == "angles":
                perf = row.get("perf") or {}
                if perf.get("ack_token"):
                    self.loop.call_soon_threadsafe(self._resolve_ack, row)
                if perf:
                    print(
                        "[perf-hand/serial] "
                        f"id={perf.get('id')} stdin_queue={perf.get('stdin_queue_ms')}ms "
                        f"RS485={perf.get('serial_ms')}ms "
                        f"enqueue_to_serial={perf.get('enqueue_to_serial_ms')}ms",
                        flush=True,
                    )
            # 广播给网页
            if typ in ("state", "action_step", "action_done", "error"):
                self.loop.call_soon_threadsafe(self._broadcast, row)

    def _pump_rerun_url(self) -> None:
        assert self.rerun is not None and self.rerun.stdout is not None
        for line in self.rerun.stdout:
            if not self._running:
                break
            m = _URL_RE.search(line)
            if m:
                self.rerun_url = m.group(1)

    def _broadcast(self, row: dict) -> None:
        for q in list(self.clients):
            if q.full():
                try:
                    q.get_nowait()
                except Exception:                       # noqa: BLE001
                    pass
            q.put_nowait(row)

    def command(self, cmd: dict) -> dict:
        """把一条控制指令写给 console。"""
        if not (self.console and self.console.stdin):
            return {"ok": False, "msg": "console 未启动"}
        try:
            with self._stdin_lock:
                self.console.stdin.write(json.dumps(cmd) + "\n")
                self.console.stdin.flush()
            return {"ok": True, "sent": cmd}
        except Exception as e:                          # noqa: BLE001
            return {"ok": False, "msg": str(e)}

    async def send_angles_wait_ack(
        self, angles: tuple[float, ...], frame_id: object
    ) -> dict:
        """Write one ANGLE_SET and complete only after hand_console ACKs it."""
        self._ack_seq += 1
        token = f"mimic-{self._ack_seq}"
        future = self.loop.create_future()
        self._ack_waiters[token] = future
        started_at = time.perf_counter()
        command = {
            "cmd": "angles",
            "rad": list(angles),
            "perf_id": frame_id,
            "_perf": {
                "id": frame_id,
                "ack_token": token,
                "source": "mimic",
                "enqueued_ns": time.perf_counter_ns(),
            },
        }
        result = self.command(command)
        print(
            f"[perf-hand/enqueue] id={frame_id} "
            f"FastAPI到stdin={(time.perf_counter()-started_at)*1000:.1f}ms",
            flush=True,
        )
        if not result.get("ok"):
            self._ack_waiters.pop(token, None)
            return result
        try:
            return await future
        finally:
            self._ack_waiters.pop(token, None)

    def _resolve_ack(self, row: dict) -> None:
        token = (row.get("perf") or {}).get("ack_token")
        future = self._ack_waiters.get(token)
        if future is not None and not future.done():
            future.set_result(row)

    def _cancel_ack_waiters(self) -> None:
        for future in self._ack_waiters.values():
            if not future.done():
                future.cancel()
        self._ack_waiters.clear()


class ArmDebugSession:
    """机械臂调试会话:arm_console.py 独占 can0。结构和 HandDebugSession 同构。

    刻意没和 HandDebugSession 合并成一个基类:手那套已经在真手上验过,重构它的收益
    不如风险大。两边的协议(start/stop/status/command + /ws)是同构的,合体页面
    同时开两个会话即可 —— 通道不冲突(RS485 vs CAN),各自独立。

    ⚠ 和手的实质差异:
      · 默认 mock=True(臂是 7 自由度工业臂,不做"连上就能动")
      · stop() **不回零** —— 臂在半途时回零路径未知,可能撞东西。只去使能。
      · 运动指令要先 enable,由 arm_console 侧强制
    """

    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop
        self.console: subprocess.Popen | None = None
        self.clients: set[asyncio.Queue] = set()
        self._running = False
        self._threads: list[threading.Thread] = []
        self.mock = True
        self.ready = False
        self.error: str | None = None
        self.channel: str | None = None
        self.limits: list[list[float]] | None = None    # console 报的 SDK 限位
        self.connect_pose: list[float] | None = None    # 接入那一刻的位姿
        self.latest: dict | None = None
        self._stdin_lock = threading.Lock()
        self._tracking_waiters: dict[str, asyncio.Future] = {}
        self._tracking_seq = 0
        self._tracking_active = False

    def start(self, mock: bool = True, speed: int = 20) -> None:
        """拉起 arm_console。mock=True 是默认 —— 和手页相反,理由见类注释。"""
        if self._running:
            return
        self._running = True
        self.mock = mock
        self.error = None
        self.ready = False
        flag = "--mock" if mock else "--no-mock"
        self.console = subprocess.Popen(
            # --firmware 不传 = 用 arm_console 的默认 auto(探到什么用什么)。
            # 之前这里什么都不传,而 arm_console 的默认是 "default",于是真机上
            # 一直用错的 driver 在跑 —— 这台臂是 1.11,该走 v111。
            ["python3", "src/arm_console.py", flag, "--hz", "20",
             "--speed", str(int(speed))],
            cwd=str(REPO), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self._threads = [threading.Thread(target=self._pump_console, daemon=True)]
        for t in self._threads:
            t.start()

    def wait_ready(self, timeout: float = 8.0) -> None:
        """阻塞等 console 第一条消息。CAN 握手可能比串口慢,超时给到 8s。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.ready or self.error:
                return
            if self.console is not None and self.console.poll() is not None:
                if not self.error:
                    self.error = "arm_console 启动即退出"
                return
            time.sleep(0.1)
        if not self.ready and not self.error:
            self.error = f"等 arm_console 就绪超时({timeout:.0f}s)"

    def stop(self) -> None:
        self._running = False
        try:
            self.loop.call_soon_threadsafe(self._cancel_tracking_waiters)
        except RuntimeError:
            pass
        # quit 让 console 走 finally:去使能 + 断 CAN。**不回零**(见类注释)。
        if self.console and self.console.stdin and self.console.poll() is None:
            try:
                with self._stdin_lock:
                    self.console.stdin.write('{"cmd":"quit"}\n')
                    self.console.stdin.flush()
                self.console.wait(timeout=3)
            except Exception:                           # noqa: BLE001
                pass
        if self.console and self.console.poll() is None:
            self.console.terminate()
            try:
                self.console.wait(timeout=3)
            except Exception:                           # noqa: BLE001
                self.console.kill()
        self.console = None
        self.ready = False
        self.latest = None
        self.connect_pose = None
        self._tracking_active = False

    def _pump_console(self) -> None:
        assert self.console is not None and self.console.stdout is not None
        for line in self.console.stdout:
            if not self._running:
                break
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            typ = row.get("type")
            if typ == "ready":
                self.ready = True
                self.channel = row.get("channel")
                self.limits = row.get("limits")
                self.connect_pose = row.get("connect_pose")
                # ready 里的 enabled 是从驱动器读的**真实**值 —— 臂可能本来就被
                # 松灵客户端使能着。塞进 latest,让 status/前端立刻拿到正确状态,
                # 不用等第一帧 state。
                self.latest = {"enabled": row.get("enabled", False),
                               "frozen": False,
                               "speed_percent": row.get("speed_percent")}
            elif typ == "error" and row.get("fatal"):
                self.error = row.get("msg") or "CAN 打开失败"
            elif typ == "state":
                self.latest = row
            elif typ == "ack" and row.get("cmd") == "tracking_angles":
                token = row.get("tracking_token")
                if token:
                    self.loop.call_soon_threadsafe(
                        self._resolve_tracking_ack, token, row
                    )
            if (typ in ("combo_ready", "combo_failed", "combo_done")
                    or (typ == "error" and row.get("cmd") in
                        ("combo_prepare", "combo_start"))
                    or (typ == "state" and row.get("combo"))):
                _handle_arm_combo_event(row)
            if typ in ("state", "ack", "error", "closed", "combo_ready",
                       "combo_failed", "combo_done"):
                self.loop.call_soon_threadsafe(self._broadcast, row)

    def _broadcast(self, row: dict) -> None:
        for q in list(self.clients):
            if q.full():
                try:
                    q.get_nowait()
                except Exception:                       # noqa: BLE001
                    pass
            q.put_nowait(row)

    def command(self, cmd: dict) -> dict:
        if not (self.console and self.console.stdin):
            return {"ok": False, "msg": "console 未启动"}
        try:
            with self._stdin_lock:
                self.console.stdin.write(json.dumps(cmd) + "\n")
                self.console.stdin.flush()
            return {"ok": True, "sent": cmd}
        except Exception as e:                          # noqa: BLE001
            return {"ok": False, "msg": str(e)}

    async def send_tracking_angles_wait_ack(
        self, angles: tuple[float, ...], frame_id: object
    ) -> dict:
        if not self._tracking_active:
            started = self.command({"cmd": "tracking_begin"})
            if not started.get("ok"):
                return started
            self._tracking_active = True
        self._tracking_seq += 1
        token = f"arm-track-{self._tracking_seq}"
        future = self.loop.create_future()
        self._tracking_waiters[token] = future
        result = self.command({
            "cmd": "tracking_angles",
            "rad": list(angles),
            "frame_id": frame_id,
            "tracking_token": token,
        })
        if not result.get("ok"):
            self._tracking_waiters.pop(token, None)
            return result
        try:
            return await future
        finally:
            self._tracking_waiters.pop(token, None)

    def end_tracking(self) -> None:
        if self._tracking_active:
            self.command({"cmd": "tracking_end"})
        self._tracking_active = False

    def _resolve_tracking_ack(self, token: str, row: dict) -> None:
        future = self._tracking_waiters.get(token)
        if future is not None and not future.done():
            future.set_result(row)

    def _cancel_tracking_waiters(self) -> None:
        for future in self._tracking_waiters.values():
            if not future.done():
                future.cancel()
        self._tracking_waiters.clear()


class LiveIKClient:
    """Synchronous JSON-line client for the Pinocchio worker process."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.process = subprocess.Popen(
            [VLA_RUNTIME_PY, "src/live_ik_worker.py"],
            cwd=str(REPO), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1,
        )
        ready = self._read()
        if not ready.get("ok"):
            self.close()
            raise RuntimeError(ready.get("error") or "IK worker failed to start")
        self.q_home = [float(value) for value in ready["q_home"]]

    def request(self, payload: dict) -> dict:
        with self._lock:
            if self.process.poll() is not None or not self.process.stdin:
                raise RuntimeError("IK worker is not running")
            self.process.stdin.write(json.dumps(payload) + "\n")
            self.process.stdin.flush()
            result = self._read()
            if not result.get("ok"):
                raise RuntimeError(result.get("error") or "IK worker request failed")
            return result

    def _read(self) -> dict:
        if not self.process.stdout:
            raise RuntimeError("IK worker stdout is unavailable")
        line = self.process.stdout.readline()
        if not line:
            detail = ""
            if self.process.stderr:
                detail = self.process.stderr.read().strip()
            raise RuntimeError(f"IK worker exited unexpectedly: {detail}")
        return json.loads(line)

    def close(self) -> None:
        if self.process.poll() is None:
            try:
                if self.process.stdin:
                    self.process.stdin.write('{"cmd":"close"}\n')
                    self.process.stdin.flush()
                self.process.wait(timeout=2)
            except Exception:  # noqa: BLE001
                self.process.terminate()
        if self.process.poll() is None:
            self.process.kill()


_hand: HandDebugSession | None = None
_arm: ArmDebugSession | None = None
_hand_target_mailbox: LatestTargetMailbox | None = None
_arm_target_mailbox: LatestTargetMailbox | None = None
_combo_pending_lock = threading.Lock()
_combo_pending: dict | None = None


def _get_arm() -> ArmDebugSession:
    global _arm
    if _arm is None:
        _arm = ArmDebugSession(asyncio.get_event_loop())
    return _arm


def _get_hand() -> HandDebugSession:
    global _hand
    if _hand is None:
        _hand = HandDebugSession(asyncio.get_event_loop())
    return _hand


async def _send_mimic_target(target: HandTarget) -> dict:
    hand = _get_hand()
    if not (hand.ready and hand.console and hand.console.poll() is None):
        return {"ok": False, "msg": "灵巧手未接入"}
    return await hand.send_angles_wait_ack(target.angles, target.frame_id)


def _report_mimic_target(perf: dict) -> None:
    print(
        "[perf-hand/mailbox] "
        f"id={perf.get('id')} replaced={perf.get('replaced')} "
        f"wait={perf.get('wait_ms')}ms age={perf.get('age_ms')}ms "
        f"ack={perf.get('status')}",
        flush=True,
    )


def _get_hand_target_mailbox() -> LatestTargetMailbox:
    global _hand_target_mailbox
    if _hand_target_mailbox is None:
        _hand_target_mailbox = LatestTargetMailbox(
            _send_mimic_target,
            rate_hz=30.0,
            max_age_ms=250.0,
            ack_timeout_ms=100.0,
            reporter=_report_mimic_target,
        )
    return _hand_target_mailbox


async def _send_live_arm_target(target: HandTarget) -> dict:
    arm = _get_arm()
    if not (arm.ready and arm.console and arm.console.poll() is None):
        return {"ok": False, "msg": "机械臂未接入"}
    latest = arm.latest or {}
    if not latest.get("enabled") or latest.get("frozen"):
        return {"ok": False, "msg": "机械臂未使能或已冻结"}
    return await arm.send_tracking_angles_wait_ack(target.angles, target.frame_id)


def _report_live_arm_target(perf: dict) -> None:
    print(
        "[perf-arm/mailbox] "
        f"id={perf.get('id')} replaced={perf.get('replaced')} "
        f"wait={perf.get('wait_ms')}ms age={perf.get('age_ms')}ms "
        f"ack={perf.get('status')}",
        flush=True,
    )


def _get_arm_target_mailbox() -> LatestTargetMailbox:
    global _arm_target_mailbox
    if _arm_target_mailbox is None:
        _arm_target_mailbox = LatestTargetMailbox(
            _send_live_arm_target,
            rate_hz=30.0,
            max_age_ms=200.0,
            ack_timeout_ms=120.0,
            angle_count=7,
            reporter=_report_live_arm_target,
        )
    return _arm_target_mailbox



# ---------------------------------------------------------------------------
# FastAPI
# ---------------------------------------------------------------------------
app = FastAPI(title="NERO·Inspire 回放工作台")
_hardware_lease = HardwareLease(ttl_s=8.0)
_hardware_release_lock: asyncio.Lock | None = None
_hardware_watchdog_task: asyncio.Task | None = None


def _lease_owner(owner: str | None) -> str:
    return str(owner or "").strip()


def _lease_error(owner: str | None, *, acquire: bool = False) -> JSONResponse | None:
    result = (_hardware_lease.acquire(owner) if acquire
              else _hardware_lease.heartbeat(owner))
    if result.ok:
        return None
    status = 409 if result.reason == "owner_busy" else 428
    return JSONResponse({"ok": False, "msg": "硬件会话已被另一个标签页占用"
                         if result.reason == "owner_busy"
                         else "缺少有效硬件会话租约",
                         "reason": result.reason,
                         "owner": result.owner,
                         "expires_at": result.expires_at}, status_code=status)


def _request_owner(header_owner: str | None, payload: dict | None = None,
                   query_owner: str | None = None) -> str:
    """Resolve the lease id from the preferred header, then body/query fallback."""
    body_owner = payload.get("owner") or payload.get("lease_id") if payload else None
    return _lease_owner(header_owner or body_owner or query_owner)


def _require_lease(header_owner: str | None, payload: dict | None = None,
                   *, query_owner: str | None = None,
                   acquire: bool = False) -> tuple[str, JSONResponse | None]:
    owner = _request_owner(header_owner, payload, query_owner)
    return owner, _lease_error(owner, acquire=acquire)


def _lease_response(result) -> JSONResponse:
    return JSONResponse({
        "ok": result.ok,
        "owner": result.owner,
        "expires_at": result.expires_at,
        "ttl_ms": int(_hardware_lease.ttl_s * 1000),
        **({"reason": result.reason} if not result.ok else {}),
    }, status_code=200 if result.ok else (409 if result.reason == "owner_busy" else 428))


def _release_lease_if_hardware_idle(owner: str) -> None:
    hand_alive = bool(_hand is not None and _hand.ready and _hand.console
                      and _hand.console.poll() is None)
    arm_alive = bool(_arm is not None and _arm.ready and _arm.console
                     and _arm.console.poll() is None)
    if not hand_alive and not arm_alive:
        _hardware_lease.release(owner)


@app.post("/api/hardware/lease/acquire")
async def hardware_lease_acquire(payload: dict | None = None,
                                 owner: str | None = Header(default=None,
                                                            alias="X-Hardware-Lease"),
                                 lease_id: str | None = None) -> JSONResponse:
    requested = _request_owner(owner, payload, lease_id)
    return _lease_response(_hardware_lease.acquire(requested))


@app.post("/api/hardware/lease/heartbeat")
async def hardware_lease_heartbeat(payload: dict | None = None,
                                   owner: str | None = Header(default=None,
                                                              alias="X-Hardware-Lease"),
                                   lease_id: str | None = None) -> JSONResponse:
    requested = _request_owner(owner, payload, lease_id)
    return _lease_response(_hardware_lease.heartbeat(requested))


@app.post("/api/hardware/lease/release")
async def hardware_lease_release(payload: dict | None = None,
                                 owner: str | None = Header(default=None,
                                                            alias="X-Hardware-Lease"),
                                 lease_id: str | None = None) -> JSONResponse:
    requested = _request_owner(owner, payload, lease_id)
    return _lease_response(_hardware_lease.release(requested))


@app.get("/api/hardware/lease/status")
async def hardware_lease_status(owner: str | None = Header(default=None,
                                                           alias="X-Hardware-Lease"),
                                lease_id: str | None = None) -> JSONResponse:
    current = _hardware_lease.owner
    expires_at = _hardware_lease.expires_at
    requested = _request_owner(owner, query_owner=lease_id)
    return JSONResponse({
        "ok": True,
        "owner": current,
        "expires_at": expires_at,
        "ttl_ms": int(_hardware_lease.ttl_s * 1000),
        "is_owner": bool(requested and requested == current),
    })


async def _hardware_release_impl(*, home: bool = True) -> dict:
    """Release hardware, optionally moving to the normal safe home poses first."""
    global _hardware_release_lock
    if _hardware_release_lock is None:
        _hardware_release_lock = asyncio.Lock()
    async with _hardware_release_lock:
        if _hand_target_mailbox is not None:
            _hand_target_mailbox.reset()
        hand_result = {"online": False, "released": True}
        try:
            if _hand is not None:
                alive = bool(_hand.ready and _hand.console and _hand.console.poll() is None)
                hand_result["online"] = alive
                await asyncio.get_event_loop().run_in_executor(
                    _executor, lambda: _hand.stop(home=home)
                )
        except Exception as error:  # noqa: BLE001
            hand_result.update(ok=False, error=str(error), released=False)
        arm_result = await _stop_arm_session(home=home)
        return {"ok": True, "hand": hand_result, "arm": arm_result}


async def _hardware_watchdog() -> None:
    while True:
        await asyncio.sleep(1.0)
        expired_owner = _hardware_lease.expired()
        if expired_owner is not None:
            print(f"[hardware-lease] owner={expired_owner} 超时,保持姿态并释放", flush=True)
            await _hardware_release_impl(home=False)


@app.on_event("startup")
async def _start_hardware_watchdog() -> None:
    global _hardware_watchdog_task
    _hardware_watchdog_task = asyncio.create_task(_hardware_watchdog())


@app.on_event("shutdown")
async def _stop_hardware_watchdog() -> None:
    global _hardware_watchdog_task
    if _hardware_watchdog_task is not None:
        _hardware_watchdog_task.cancel()
        try:
            await _hardware_watchdog_task
        except asyncio.CancelledError:
            pass
        _hardware_watchdog_task = None

# 静态资源:three.js 库(vendor)和手的 URDF + mesh。
# 浏览器端 3D 查看器要直接 fetch URDF 再按里面的相对路径取 mesh,所以挂的是
# assets/hand 原目录 —— URDF 里 mesh 路径是相对的(../meshes/*.STL),能被浏览器
# 解析。assets/assembled/inspire_hand_absolute.urdf 里是**绝对文件系统路径**,
# 那是给 pinocchio 用的,浏览器取不到。
@app.middleware("http")
async def _no_stale_static(request, call_next):
    """静态资源一律要求**重新验证**,不许拿缓存直接用。

    起因:改完 web/urdf_view.js 之后页面上看不到效果,查了半天磁盘和服务端都是新的 ——
    是浏览器拿了缓存里的旧模块。ES module 按 URL 缓存,而这些文件名是固定的
    (/static/urdf_view.js),所以改了内容 URL 不变,浏览器有权继续用旧的。
    这类问题最难查,因为"文件明明改了"和"页面明明没变"两边都是真的。

    no-cache 不等于不缓存:浏览器仍存着,只是每次带 ETag 问一句。没变就是 304
    (几十字节),变了才重下。代价可忽略,而"改了不生效"这个坑直接消失。
    ⚠ 别改成 max-age —— 那又会把 mesh 和 js 缓存住,踩同一个坑。
    """
    resp = await call_next(request)
    p = request.url.path
    if p.startswith(("/static/", "/hand_assets/", "/arm_assets/", "/combo_assets/")):
        resp.headers["Cache-Control"] = "no-cache, must-revalidate"
    return resp


app.mount("/static", StaticFiles(directory=str(WEB_DIR)), name="static")

# MediaPipe 本地文件
_VENDOR_DIR = WEB_DIR / "vendor"
if _VENDOR_DIR.is_dir():
    app.mount("/vendor", StaticFiles(directory=str(_VENDOR_DIR)), name="vendor")

_HAND_ASSETS = REPO / "assets/viz/hand"
if _HAND_ASSETS.is_dir():
    app.mount("/hand_assets", StaticFiles(directory=str(_HAND_ASSETS)), name="hand_assets")
# 臂:挂 build_arm_viz.py 的产物,**不是** assets/nero_description。
# 原 URDF 的 visual 是 Collada(GLTFLoader 读不了,link2.dae 还有 24MB),
# 且路径是 package:// —— 浏览器两样都吃不下。产物是 glb + 相对路径,共 7.0MB。
# 没生成过就先跑 `python3 src/build_arm_viz.py`。
_ARM_ASSETS = REPO / "assets/viz/arm"
if _ARM_ASSETS.is_dir():
    app.mount("/arm_assets", StaticFiles(directory=str(_ARM_ASSETS)), name="arm_assets")
# 合体页(实时 Live):整条装配链 base→link8→法兰→手。同样是产物,
# 源装配 URDF 的 mesh 是绝对路径 + STL,浏览器两样都取不到。
# 没生成过就先跑 `python3 src/build_combo_viz.py`。
_COMBO_ASSETS = REPO / "assets/viz/combo"
if _COMBO_ASSETS.is_dir():
    app.mount("/combo_assets", StaticFiles(directory=str(_COMBO_ASSETS)),
              name="combo_assets")


@app.get("/")
async def index() -> HTMLResponse:
    return HTMLResponse((WEB_DIR / "index.html").read_text(encoding="utf-8"))


@app.get("/api/status")
async def status() -> JSONResponse:
    alive = _replay_proc is not None and _replay_proc.poll() is None
    return JSONResponse({"serve": alive, "ip": _primary_ip()})


@app.post("/api/upload")
async def upload(file: UploadFile = File(...)) -> JSONResponse:
    """收视频或外部手部处理结果,存到临时目录,返回后续管线要用的绝对路径。"""
    safe = re.sub(r"[^\w.\-]", "_", file.filename or "upload")
    dst = Path(tempfile.gettempdir()) / f"nero_web_{safe}"
    with open(dst, "wb") as f:
        shutil.copyfileobj(file.file, f)
    suffix = Path(safe).suffix.lower()
    source = "handfile" if suffix in {".npz", ".pkl", ".pickle", ".json"} else "video"
    return JSONResponse({"path": str(dst), "name": file.filename, "source": source})


@app.get("/api/run")
async def run(video: str = "", skip: bool = False, dataset: str = "rgb",
              source: str = "video", rgbd_dir: str = "", rgbd_camera: str = "",
              hand: str = "inspire") -> StreamingResponse:
    """SSE:管线在线程里跑,事件推给浏览器(EventSource 消费)。
    dataset='rgb'|'rgbd' 决定 build 脚本 + 数据源;hand='inspire'|'gripper' 决定本体;
    本体规格 = nero_{hand}_{rgb|rgbd};rgbd_dir/rgbd_camera 手动指定 RGB-D 目录。"""
    async def stream():
        queue: asyncio.Queue = asyncio.Queue()
        loop = asyncio.get_event_loop()

        def emit(ev: dict) -> None:
            loop.call_soon_threadsafe(queue.put_nowait, ev)

        def worker() -> None:
            try:
                run_pipeline(video or None, skip, dataset, emit, source=source,
                             rgbd_dir=rgbd_dir, rgbd_camera=rgbd_camera, hand=hand)
            except Exception as e:                       # noqa: BLE001
                emit({"type": "error", "msg": f"管线异常: {e}"})
            finally:
                loop.call_soon_threadsafe(queue.put_nowait, None)

        loop.run_in_executor(_executor, worker)
        while True:
            ev = await queue.get()
            if ev is None:
                break
            yield f"data: {json.dumps(ev, ensure_ascii=False)}\n\n"

    return StreamingResponse(stream(), media_type="text/event-stream",
                             headers={"Cache-Control": "no-cache",
                                      "X-Accel-Buffering": "no"})


# ---- 实时监控端点 ----
@app.post("/api/live/start")
async def live_start() -> JSONResponse:
    live = _get_live()
    live.start()
    # 给 live_rerun 起 serve + 抓 URL 一点时间
    for _ in range(60):
        if live.rerun_url:
            break
        await asyncio.sleep(0.5)
    return JSONResponse({"ok": True, "rerun_url": live.rerun_url})


@app.post("/api/live/stop")
async def live_stop() -> JSONResponse:
    if _live is not None:
        _live.stop()
    return JSONResponse({"ok": True})


# ---- 轨迹下发端点(视频→轨迹→机械臂回放的最后一环)----
@app.get("/api/traj/frames")
async def traj_frames(robot: str = "") -> JSONResponse:
    """回放轨迹逐帧关节角(供右侧读数随 Rerun 游标联动)。
    读 robot_traj_<robot>.npz → {fps, arm_names, hand_names, arm[N][7], hand[N][12]}。"""
    import numpy as np
    npz = _traj_pkl(robot).with_suffix(".npz") if robot else TRAJ_NPZ
    if not npz.exists():
        return JSONResponse({"error": f"无轨迹: {npz.name}"}, status_code=404)
    d = np.load(npz, allow_pickle=True)
    fps = float(d["fps"]) if "fps" in d.files else 30.0
    return JSONResponse({
        "fps": fps,
        "arm_names": [str(x) for x in d["arm_joint_names"]],
        "hand_names": [str(x) for x in d["hand_joint_names"]],
        "arm": np.asarray(d["arm"], dtype=float).round(4).tolist(),
        "hand": np.asarray(d["hand"], dtype=float).round(4).tolist(),
    })


@app.get("/api/metrics")
async def metrics(robot: str = "") -> JSONResponse:
    """读缓存的验收指标 JSON(按本体);无缓存返回 measured=false。右侧验收卡据此渲染。"""
    if not robot:
        return JSONResponse({"measured": False, "msg": "未指定本体"})
    p = _metrics_json(robot)
    if not p.exists():
        return JSONResponse({"measured": False, "robot": robot, "msg": "未测 · 跑一次管线生成"})
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except Exception as e:                                # noqa: BLE001
        return JSONResponse({"measured": False, "robot": robot, "msg": f"缓存损坏: {e}"})
    data["measured"] = True
    return JSONResponse(data)


@app.get("/api/replay/keypoints")
async def replay_keypoints(robot: str = "") -> JSONResponse:
    """读 canonical parquet → {fps, frames: [{kp2d, vis, wrist_pose}]}。
    供 replay3d.js 画骨骼叠加和验证手腕姿态。"""
    import numpy as np
    import pyarrow.parquet as pq
    ds_root = _canonical_root(robot or "nero_inspire_rgb")
    parquet_files = sorted((ds_root / "data").rglob("*.parquet"))
    if not parquet_files:
        return JSONResponse({"error": "canonical parquet 不存在"}, status_code=404)
    import pandas as pd
    df = pd.concat([pd.read_parquet(p) for p in parquet_files], ignore_index=True)
    if "frame_index" in df.columns:
        df = df.sort_values("frame_index")
    kp2d = np.stack(df["observation.hand_keypoints_2d"].to_numpy()).reshape(-1, 21, 2)
    vis = np.stack(df["observation.hand_visibility"].to_numpy())
    wrist = np.stack(df["observation.wrist_pose"].to_numpy()) if "observation.wrist_pose" in df.columns else None
    fps = 30.0
    info_p = ds_root / "meta/info.json"
    if info_p.exists():
        fps = json.loads(info_p.read_text()).get("fps", 30.0)
    # kp2d 单位是**原视频像素**(parse_keypoint_2d: normalized × [w,h]),而 canonical
    # 视频被缩到 256×256。u 除以 src_w、v 除以 src_h —— 两个除数不同,前端得分轴缩放。
    # 优先量原始视频;没有就从 kp2d 范围反推(取 2 的幂次附近的常见分辨率兜底)。
    src_w = src_h = 0
    orig = _original_video(robot) if robot else None
    if orig and orig.exists():
        try:
            import cv2
            cap = cv2.VideoCapture(str(orig))
            src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            cap.release()
        except Exception:                                 # noqa: BLE001
            pass
    if not (src_w and src_h):                             # 兜底:按 kp2d 实际跨度估
        # kp2d 是像素坐标（正数），找最大值向上取整到20的倍数
        max_u = np.nanmax(kp2d[:, :, 0])  # 去掉 abs()
        max_v = np.nanmax(kp2d[:, :, 1])
        src_w = int(np.ceil(max_u / 20.0) * 20) if max_u > 0 else 540
        src_h = int(np.ceil(max_v / 20.0) * 20) if max_v > 0 else 960
        print(f"[replay_keypoints] 从kp2d推算源尺寸: max_u={max_u:.1f}, max_v={max_v:.1f} → {src_w}×{src_h}")
    else:
        print(f"[replay_keypoints] 从原始视频读取尺寸: {src_w}×{src_h}")
    frames_data = []
    for i in range(len(kp2d)):
        obj = {"kp2d": kp2d[i].round(2).tolist(), "vis": vis[i].round(3).tolist()}
        if wrist is not None:
            obj["wrist_pose"] = wrist[i].round(4).tolist()
        frames_data.append(obj)
    return JSONResponse({"fps": fps, "n_frames": len(frames_data),
                         "src_w": src_w, "src_h": src_h, "frames": frames_data})


@app.get("/api/replay/video/canonical")
async def replay_video_canonical():
    """规范层视频 256×256 @30fps，与 keypoints/traj 帧号严格 1:1 对齐。"""
    from starlette.responses import FileResponse
    p = _canonical_root("nero_inspire_rgb") / "videos/observation.images.ego/chunk-000/file-000.mp4"
    if not p.exists():
        return JSONResponse({"error": "canonical 视频不存在"}, status_code=404)
    return FileResponse(p, media_type="video/mp4")


@app.get("/api/replay/video/original")
async def replay_video_original(robot: str = ""):
    """原始上传视频(如果保存了)。⚠ 帧号可能和 canonical 不对齐(起始检测失败帧被跳过)。"""
    from starlette.responses import FileResponse
    p = _original_video(robot or "nero_inspire_rgb")
    if not p.exists():
        return JSONResponse({"error": "原始视频未保存"}, status_code=404)
    return FileResponse(p, media_type="video/mp4")


@app.get("/api/replay/play")
async def replay_play(speed: float = 1.0, fps: float = 30.0) -> StreamingResponse:
    """SSE:spawn traj_player 逐帧下发 npz 轨迹给 writer→bridge,进度推浏览器。
    需先开『实时 Live』(bridge 在跑、才有下发对象;Live 3D 显示真机回读)。"""
    async def stream():
        global _player_proc
        _stop_player()                                    # 单实例:先停旧的
        if TRAJ_NPZ is None or not TRAJ_NPZ.exists():
            yield f"data: {json.dumps({'type':'error','msg':'无轨迹,请先在回放模式跑一遍视频管线'}, ensure_ascii=False)}\n\n"
            return
        sp = max(0.25, min(4.0, speed))
        _player_proc = subprocess.Popen(
            _ros_cmd(["src/traj_player.py", "--npz", str(TRAJ_NPZ),
                      "--fps", str(fps), "--speed", str(sp)]),
            cwd=str(REPO), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        loop = asyncio.get_event_loop()
        q: asyncio.Queue = asyncio.Queue()

        def pump() -> None:
            assert _player_proc and _player_proc.stdout
            for ln in _player_proc.stdout:
                loop.call_soon_threadsafe(q.put_nowait, ln.strip())
            loop.call_soon_threadsafe(q.put_nowait, None)

        loop.run_in_executor(_executor, pump)
        while True:
            ln = await q.get()
            if ln is None:
                break
            if ln:
                yield f"data: {ln}\n\n"

    return StreamingResponse(stream(), media_type="text/event-stream",
                             headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.post("/api/replay/stop")
async def replay_stop() -> JSONResponse:
    _stop_player()
    return JSONResponse({"ok": True})


@app.get("/api/live/url")
async def live_url() -> JSONResponse:
    url = _live.rerun_url if _live else None
    return JSONResponse({"rerun_url": url})


@app.post("/api/command")
async def command(payload: dict) -> JSONResponse:
    """控制存根:{arm:[7], hand:[6], duration} 或 {estop:true}。经 writer 发 JointTrajectory。"""
    live = _get_live()
    return JSONResponse(live.command(payload))


# ---- 技能清单端点(只读;执行在后续 runner 接入)----
@app.get("/api/skills")
async def skills(reload: bool = False) -> JSONResponse:
    """技能清单。reload=true 时重读 registry.yaml —— 调字段设计时不必重启 Web。

    返回的是 SkillSpec.to_public():**不含** action 原始关节值,前端拿不到可直发的
    数值,只能按 id 走 /api/skills/invoke,保证限位夹取无法被绕过。
    """
    try:
        reg = get_registry(reload=reload)
    except RegistryError as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    live_on = _live is not None and _live.reader is not None
    return JSONResponse({
        "version": reg.version,
        "count": len(reg),
        "skills": reg.to_public(),
        "warnings": reg.warnings,
        "live_session": live_on,        # 前端据此灰掉 requires:[live_session] 的技能
        "arm_enabled": bool(_live.arm_enabled) if _live else False,
    })


_skill_proc: subprocess.Popen | None = None       # 技能执行:单实例


def _stop_skill() -> None:
    global _skill_proc
    if _skill_proc and _skill_proc.poll() is None:
        _skill_proc.terminate()
        try:
            _skill_proc.wait(timeout=4)
        except Exception:                                 # noqa: BLE001
            _skill_proc.kill()
    _skill_proc = None


atexit.register(_stop_skill)


@app.post("/api/skills/invoke")
async def skills_invoke(payload: dict) -> StreamingResponse:
    """SSE:执行一个技能。信封 {skill_id, params, source, confirmed, transcript}。

    执行落在 skills/runner.py 子进程(ROS2 侧),事件原样透传给浏览器。
    安全闸在 runner 里强制,本端点不放行任何东西 —— 只补两件它查不到的事:
      · assume_enabled:按本会话是否发过 arm_enable 填,不由前端随意声明
      · source:语音走 /api/voice/*,直接 POST 本端点的一律记为 web
    """
    async def stream():
        global _skill_proc
        _stop_skill()                                     # 单实例:先停旧的
        live = _get_live()
        env = {
            "skill_id": payload.get("skill_id"),
            "params": payload.get("params") or {},
            "source": payload.get("source") or "web",
            "request_id": payload.get("request_id"),
            "confirmed": bool(payload.get("confirmed")),
            "transcript": payload.get("transcript"),
            "confidence": payload.get("confidence"),
            # 使能表态只认服务端跟踪的状态,前端说了不算
            "assume_enabled": bool(live.arm_enabled),
        }
        _skill_proc = subprocess.Popen(
            _ros_cmd(["src/skills/runner.py", "--once", json.dumps(env)]),
            cwd=str(REPO), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        loop = asyncio.get_event_loop()
        q: asyncio.Queue = asyncio.Queue()

        def pump() -> None:
            assert _skill_proc and _skill_proc.stdout
            for ln in _skill_proc.stdout:
                loop.call_soon_threadsafe(q.put_nowait, ln.strip())
            loop.call_soon_threadsafe(q.put_nowait, None)

        loop.run_in_executor(_executor, pump)
        while True:
            ln = await q.get()
            if ln is None:
                break
            if not ln:
                continue
            # 技能执行成功后同步使能状态(enable/disable/estop 类技能会改它)
            try:
                ev = json.loads(ln)
                if ev.get("type") == "done":
                    _sync_enable_from_skill(live, ev.get("skill_id"))
            except json.JSONDecodeError:
                pass
            yield f"data: {ln}\n\n"

    return StreamingResponse(stream(), media_type="text/event-stream",
                             headers={"Cache-Control": "no-cache",
                                      "X-Accel-Buffering": "no"})


def _sync_enable_from_skill(live: LiveSession, skill_id: str | None) -> None:
    """技能执行完后更新使能跟踪。效果从清单的 action 推导,不硬编码 id ——
    清单里改了动作,这里自动跟上。"""
    if not skill_id:
        return
    try:
        reg = get_registry()
    except RegistryError:
        return
    spec = reg.get(skill_id)
    if spec is None:
        return
    eff = spec.enable_effect(reg)
    if eff is not None:
        live.arm_enabled = eff


@app.post("/api/skills/stop")
async def skills_stop() -> JSONResponse:
    """停掉正在执行的技能。注意:这只杀执行进程,不等于急停 —— 真要停机器人发 estop。"""
    _stop_skill()
    return JSONResponse({"ok": True})


# ============================ 语音路径 /api/voice/* ============================
# 与 /api/skills/invoke 的区别是**执行通道**:那条走 ROS(runner.py → bridge),
# 这条走 console(arm_console 独占 can0 / hand_console 独占 RS485)。两条不能同时
# 用 —— 同一条通道两个写者会互相覆盖(COMBO_DEBUG.md)。合体页用的是 console,
# 真机也是在 console 上验过的,所以语音走它。
#
# 三段式,刻意不合成一个端点:
#   parse    一句话 → 意图。**不执行、不碰硬件**
#   invoke   带 confirmed 的信封 → SSE 执行事件
#   stop     停止继续下发(≠ 急停)
# 分开正是「页面按钮确认」这条设计的落点:解析和执行之间必须有人点一下。
# ASR 以后接在 parse 之前(改的是谁产生 text),本层不动。

_voice_exec: ConsoleExecutor | None = None
_voice_lock = threading.Lock()


def _voice_play_pack(payload: dict) -> JSONResponse:
    """语音回放技能包。走**和按钮同一条**路,按 kind 分流:

      gesture_pack → gp.load_pack  → hand_console 的 gesture_play(ActionPlayer)
      combo_pack   → cbp.load_pack → arm_console 的 prepare/start(CPV)+ 手同轴

    为什么不让前端直接打 /api/hand/gesture/play 或 /api/combo/play:那样会绕过
    两件事 —— ① 二次确认闸;② 调用日志。日志里 (原话, 包路径) 的配对和技能那边
    一样是以后 VLA 的标注原料,不能因为「这是包不是技能」就漏掉。

    ⚠ kind **由后端自己从池里查**,不信前端传的那个。前端传 combo_pack 而实际是
    手势包(或反过来)会去错的沙箱根 load —— 两个根可以各有一个同名文件。
    前端的 kind 只用来决定走不走这个函数,具体是哪种由 path 在池里的归属定。

    ⚠ 返回 JSON 而不是 SSE:包的回放是异步播的,这里拿不到逐帧进度 ——
    进度在各自页面的栏目里由 /ws/hand 或 /api/combo/play/status 驱动。
    硬编成 SSE 只会造出一个「立刻 start 紧接 done」的假进度流。
    """
    rel = str(payload.get("pack_path") or "").strip()
    # 从池里反查 kind。查不到就当手势包(向后兼容:老前端不传 kind)。
    kind = "gesture_pack"
    for t in _list_pack_targets(include_combo=True):
        if t.path == rel:
            kind = t.kind
            break
    rec = {"skill_id": None, "kind": kind, "pack_path": rel,
           "source": "voice", "path": "console",
           "transcript": payload.get("transcript"),
           "confidence": payload.get("confidence"), "ts": time.time()}
    if not rel:
        _log_invocation({**rec, "result": "no_pack_path"})
        return JSONResponse({"ok": False, "msg": "缺 pack_path"}, status_code=400)
    # 确认闸:包会真的动硬件,和清单里的技能同一档,缺 confirmed 一律拒
    if not payload.get("confirmed"):
        _log_invocation({**rec, "result": "gate_rejected", "reason": "缺 confirmed"})
        return JSONResponse({"ok": False, "msg": "技能包需要确认:点『执行』再来"},
                            status_code=409)

    if kind == "combo_pack":
        return _voice_play_combo(rel, rec)

    import gesture_pack as gp
    try:
        pack = gp.load_pack(rel)                    # 内含 resolve_pack_path 沙箱校验
    except gp.GestureError as e:
        _log_invocation({**rec, "result": "bad_pack", "reason": str(e)})
        return JSONResponse({"ok": False, "msg": f"技能包不合法: {e}"},
                            status_code=400)
    except OSError as e:
        _log_invocation({**rec, "result": "bad_pack", "reason": str(e)})
        return JSONResponse({"ok": False, "msg": f"读文件失败: {e}"}, status_code=500)

    hand = _hand
    if hand is None or hand.console is None or hand.console.poll() is not None:
        _log_invocation({**rec, "result": "gate_rejected", "reason": "手未接入"})
        return JSONResponse({"ok": False, "msg": "未接入灵巧手,先点『接入』"},
                            status_code=409)
    res = hand.command({"cmd": "gesture_play", "pack": pack.to_dict()})
    rec["pack_name"] = pack.name
    _log_invocation({**rec, "result": "done" if res.get("ok") else "send_failed",
                     "frames": len(pack.frames)})
    return JSONResponse({**res, "kind": "gesture_pack", "name": pack.name,
                         "frames": len(pack.frames),
                         "duration_ms": pack.duration_ms,
                         "note": "已投递给 console;逐帧进度看手页『技能包』栏"})


def _handle_arm_combo_event(row: dict) -> None:
    """由 arm stdout 线程处理 prepare/ready/start 两阶段协调。"""
    global _combo_pending
    typ = row.get("type")
    token = row.get("token")
    with _combo_pending_lock:
        pending = _combo_pending
        if pending is not None and token and token != pending.get("token"):
            return
        if typ == "combo_failed" or (typ == "error" and row.get("cmd") == "combo_start"):
            hand = _get_hand()
            if hand.console and hand.console.poll() is None:
                hand.command({"cmd": "action_stop"})
            _combo_pending = None
            return
        if pending is None:
            return

        if typ == "state":
            if (row.get("combo") or {}).get("phase") == "playing":
                pending["phase"] = "playing"
            return
        if typ == "combo_done":
            _combo_pending = None
            return
        if typ == "error":
            _combo_pending = None
            hand = _get_hand()
            if hand.console and hand.console.poll() is None:
                hand.command({"cmd": "action_stop"})
            return
        if typ != "combo_ready" or pending.get("phase") != "preparing":
            return

        arm, hand = _get_arm(), _get_hand()
        if not (arm.console and arm.console.poll() is None
                and hand.console and hand.console.poll() is None):
            arm.command({"cmd": "combo_stop"})
            _combo_pending = None
            return

        # ready 之后重新取共同起点。200ms 覆盖两条 stdin 管道的调度抖动；两边使用
        # 同一个系统 CLOCK_MONOTONIC，所以到达先后不会变成固定时间轴偏差。
        start_at = time.monotonic() + 0.2
        arm_result = arm.command({"cmd": "combo_start", "token": token,
                                  "start_at": start_at})
        hand_result = hand.command({"cmd": "gesture_play", "pack": pending["hand_pack"],
                                    "return_home": False, "start_at": start_at})
        if not arm_result.get("ok") or not hand_result.get("ok"):
            arm.command({"cmd": "combo_stop"})
            hand.command({"cmd": "action_stop"})
            _combo_pending = None
            return
        pending["phase"] = "starting"


def _combo_start(pack, rel: str) -> tuple[bool, str, int]:
    """启动联合录制包回放,返回 (成功, 消息, HTTP 状态码)。

    CPV 路径分两阶段：先让 arm_console 非阻塞地 approach 首帧；收到 combo_ready
    后再给两个 console 下发共同 start_at。进度由 console 内部播放器管理。

    ⚠ 这是 ▶ 按钮和语音的统一路径 —— preflight 做一份,两边门槛一致。
    """
    global _combo_pending
    arm, hand = _get_arm(), _get_hand()
    if not (arm.console and arm.console.poll() is None):
        return False, "未接入臂", 409
    if not (hand.console and hand.console.poll() is None):
        return False, "未接入手", 409
    st = arm.latest or {}
    if not st.get("enabled"):
        return False, "臂未使能,先点『使能』", 409
    if st.get("frozen"):
        return False, "急停生效中,先点『复位』", 409
    if pack.mode == "stream":
        return False, "mode=stream 上千帧,要流式喂 —— 用 combo_player.py --combo", 400
    # 手侧:**复用 gesture_play**,不新造命令。
    # ⚠ combo 包的手侧字段和 GesturePack 逐个对得上(hand_rad→rad、hand_raw→
    # raw_vendor,其余同名),所以转成 gesture pack 的 dict 就能走 console 里
    # 已有的那条路 —— to_action_sequence + ActionPlayer 一整套时序/暂停/停止
    # 全部复用。我一开始在 hand_console 里另写了个 "play" 命令,自己拼
    # ActionStep,结果字段全错(angles 要 0-1000 原始值不是弧度,speeds/forces
    # 是 6 个的列表不是标量,驻留字段叫 delay_ms)—— console 直接 NameError 崩。
    # return_home_first 强制 False:combo 包的第 0 帧就是录制那一刻的姿态,
    # 回零会在正式动作前插一段臂不知道的手部运动,两侧时间轴当场错开。
    import gesture_pack as gp
    hand_pack = {
        "schema": gp.SCHEMA, "name": pack.name, "hand": pack.hand,
        "return_home_first": False,
        "frames": [{"rad": list(f.hand_rad), "raw_vendor": list(f.hand_raw),
                    "hold_ms": f.hold_ms, "speed": f.speed, "force": f.force,
                    "label": f.label, "t_ns": f.t_ns} for f in pack.frames],
    }
    token = f"combo-{time.monotonic_ns()}"
    wps = [{"t_ns": f.t_ns, "rad": list(f.arm_rad)} for f in pack.frames]
    with _combo_pending_lock:
        if _combo_pending is not None:
            return False, "已有联合回放正在准备或启动", 409
        _combo_pending = {"token": token, "name": pack.name, "path": rel,
                          "frames": len(pack.frames), "duration_ms": pack.duration_ms,
                          "phase": "preparing", "hand_pack": hand_pack}
    sent = arm.command({"cmd": "combo_prepare", "token": token,
                        "name": pack.name, "mode": pack.mode, "waypoints": wps})
    if not sent.get("ok"):
        with _combo_pending_lock:
            if _combo_pending and _combo_pending.get("token") == token:
                _combo_pending = None
        return False, sent.get("msg") or "下发 combo_prepare 失败", 500
    return True, "", 200


def _voice_play_combo(rel: str, rec: dict) -> JSONResponse:
    """语音回放联合录制包。**门槛比手势包高** —— 它会动臂。

    走和 ▶ 按钮同一条路(_combo_start),所以 preflight 只有一份:
    未接入 / 未使能 / 急停中 / stream 包一律拒。
    """
    import combo_pack as cbp
    try:
        pack = cbp.load_pack(rel)
    except cbp.ComboError as e:
        _log_invocation({**rec, "result": "bad_pack", "reason": str(e)})
        return JSONResponse({"ok": False, "msg": f"录制包不合法: {e}"},
                            status_code=400)
    except OSError as e:
        _log_invocation({**rec, "result": "bad_pack", "reason": str(e)})
        return JSONResponse({"ok": False, "msg": f"读文件失败: {e}"}, status_code=500)

    ok, msg, code = _combo_start(pack, rel)
    rec["pack_name"] = pack.name
    _log_invocation({**rec, "result": "done" if ok else "gate_rejected",
                     "reason": None if ok else msg, "frames": len(pack.frames)})
    if not ok:
        return JSONResponse({"ok": False, "msg": msg}, status_code=code)
    return JSONResponse({"ok": True, "kind": "combo_pack", "name": pack.name,
                         "frames": len(pack.frames),
                         "duration_ms": pack.duration_ms,
                         "note": "已开始回放;进度看合体页『录制包』栏"})


def _console_sess_view(get_sess):
    """(发送器, 状态探针)。会话没起就让状态探针返回 None —— 安全闸据此提示先『接入』,
    而不是在这里把会话凭空创建出来(创建了后面也没有硬件)。

    console 起了但还没来第一帧遥测时返回 **{}** 而不是 None:通道确实在,
    只是状态未知。于是安全闸会按「未使能」拒运动 —— 诚实,而不是谎报通道不存在。
    """
    def send(cmd: dict) -> dict:
        sess = get_sess()
        if sess is None or sess.console is None:
            return {"ok": False, "msg": "console 未启动"}
        return sess.command(cmd)

    def state():
        sess = get_sess()
        if sess is None or sess.console is None:
            return None
        return sess.latest if sess.latest is not None else {}
    return send, state


def _console_executor() -> ConsoleExecutor:
    a_send, a_state = _console_sess_view(lambda: _arm)
    h_send, h_state = _console_sess_view(lambda: _hand)
    return ConsoleExecutor(a_send, h_send, arm_state=a_state, hand_state=h_state)


# 语音作用域。手页只有 RS485 一条通道,把臂的技能列进去只会让人说了却被拒。
#   hand  灵巧手调试页:只用手的技能 + 已录制的手势技能包
#   all   合体页:清单里全部技能;**不含**技能包(包的录制/管理都在手页,
#         放到合体页会让同一句话在两页命中不同东西)。要放开就是下面一行的事。
VOICE_SCOPES = ("hand", "all")


def _scope_targets(scope: str, reg) -> tuple[list, list]:
    """返回 (该作用域允许的技能 id 列表, 技能包列表)。

    hand 作用域只放**纯手**技能:手部调试页里说「回零位」不该动臂。
    all 作用域放全部技能 **+ 手势包 + 联合录制包** —— 合体页同时控臂和手。
    (2026-08-06 修:原来 all 的包列表写死成空 [],于是合体页说包名永远 no_match。
     那不是安全考虑,是当初 Live 页只管臂时留下的;现在合体页有手的通道了。)

    ⚠ combo 包**只进 all**,不进 hand。它会动臂,而 hand 作用域的整个前提是
    "手页说的话不该动臂"。放进去就等于在手页给了一条动臂的路。
    (2026-08-07:combo 包原来两个作用域都没进 —— _list_pack_targets 只扫
     gesture_pack,所以录好的「挥手」说了永远 no_match。实测确认过。)
    """
    if scope == "hand":
        ids = [s.id for s in reg.voice_skills()
               if console_targets(s, reg) == {"hand"}]
        return ids, _list_pack_targets(include_combo=False)
    return [s.id for s in reg.voice_skills()], _list_pack_targets(include_combo=True)


def _list_pack_targets(include_combo: bool = False) -> list[PackTarget]:
    """磁盘上的包 → 意图池条目。坏包不进池(它播不了,列出来只会误导)。

    `include_combo`:是否把**臂+手联合录制包**(data/combos/)也放进来。
    默认不放 —— 见 _scope_targets:combo 包会动臂,`hand` 作用域里不该出现。

    ⚠ 两种包各带自己的 kind。不带的话执行层拿到 path 不知道用哪个沙箱根去 load,
    而两个根可以各有一个同名文件(见 PackTarget.kind 的注释)。

    ⚠ 一种包读失败**不能拖垮另一种**。所以两个 try 分开 —— 合成一个的话
    combos 目录权限出问题会连手势包一起吞掉,页面上表现为"所有包都说不出来了",
    而真因只是新加的那半坏了。
    """
    out: list[PackTarget] = []
    try:
        import gesture_pack as gp
        out += [PackTarget.from_list_item(it, "gesture_pack")
                for it in gp.list_packs() if not it.get("error")]
    except Exception:                                     # noqa: BLE001
        pass
    if include_combo:
        try:
            import combo_pack as cbp
            # ⚠ 只放 keyframe 的。stream 包页面上放不了(要 CPV 逐关节伺服),
            # 进池了就是"说得出来但一执行就报错" —— 那比 no_match 更让人困惑。
            out += [PackTarget.from_list_item(it, "combo_pack")
                    for it in cbp.list_packs()
                    if not it.get("error") and it.get("mode") == "keyframe"]
        except Exception:                                 # noqa: BLE001
            pass
    return out


def _scoped_registry(scope: str, reg):
    """把不在作用域里的技能挡在池外。

    做法是**临时构造一个只含允许项的 SkillRegistry**,而不是解析完再过滤 ——
    过滤在后面的话,手页说「回零位」会先命中 go_home 再被丢掉,用户看到的是
    「没听懂」;挡在池外则会落到别的候选或明确的 no_match,行为更可解释。
    """
    if scope != "hand":
        return reg
    from skills.schema import SkillRegistry
    ids, _ = _scope_targets(scope, reg)
    return SkillRegistry([reg.get(i) for i in ids], version=reg.version)


def _console_ready() -> dict:
    """两条通道的现状,给前端一并回去,省一次往返。"""
    a = (_arm.latest or {}) if (_arm and _arm.console) else None
    return {"arm_console": a is not None,
            "hand_console": bool(_hand and _hand.console),
            "arm_enabled": bool((a or {}).get("enabled")),
            "arm_frozen": bool((a or {}).get("frozen"))}


@app.get("/api/voice/phrases")
async def voice_phrases(scope: str = "all") -> JSONResponse:
    """能说什么。只列 voice_enabled 的技能及其别名,给前端做提示和补全。

    scope=hand 时只列手的技能,并把**已录制的手势技能包**一起列出来。
    """
    if scope not in VOICE_SCOPES:
        return JSONResponse({"error": f"scope 只能是 {VOICE_SCOPES}"},
                            status_code=400)
    try:
        reg = get_registry()
    except RegistryError as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    ids, packs = _scope_targets(scope, reg)
    out = []
    for sid in ids:
        s = reg.get(sid)
        out.append({"skill_id": s.id, "name": s.name, "aliases": list(s.aliases),
                    "need_confirm": s.safety.need_confirm, "desc": s.desc,
                    "kind": "skill",
                    "devices": sorted(console_targets(s, reg)),
                    "ready": s.source_exists() if s.kind == "trajectory" else True})
    for p in packs:
        # ⚠ 别写死 gesture_pack/["hand"] —— packs 里混着两种包(PackTarget.kind
        # 就是从池子里带过来的)。写死的后果:「能说什么」里 combo 包显示成
        # 「只动手的手势包」,而它会动臂。这是给人看的风险提示,报错方向的错
        # 比不报更糟。
        cb = p.kind == "combo_pack"
        out.append({"skill_id": p.path, "name": p.name, "aliases": [],
                    "need_confirm": p.need_confirm, "kind": p.kind,
                    "desc": (f"臂+手联合录制 · {p.frames} 帧 · {p.duration_ms} ms"
                             if cb else
                             f"已录制手势 · {p.frames} 帧 · {p.duration_ms} ms"),
                    # 设备表查 PACK_DEVICES,别在这就地判 —— 和 parse 那边
                    # 报的必须是同一份,不然确认框和「能说什么」会自相矛盾。
                    "devices": list(PACK_DEVICES[p.kind]), "ready": True})
    return JSONResponse({"count": len(out), "phrases": out, "scope": scope,
                         "packs": len(packs), "ready": _console_ready()})


@app.post("/api/voice/parse")
async def voice_parse(payload: dict) -> JSONResponse:
    """一句话 → 意图。**不执行、不碰硬件**,给页面弹确认框用。

    text 现在来自文本框;以后接 ASR 也是同一个入口 —— 变的是谁产生 text。
    返回里带 ready(两条通道现状),前端据此在确认框上直接说「先点接入」。
    """
    text = str((payload or {}).get("text") or "")
    scope = str((payload or {}).get("scope") or "all")
    if scope not in VOICE_SCOPES:
        return JSONResponse({"error": f"scope 只能是 {VOICE_SCOPES}"},
                            status_code=400)
    try:
        reg = get_registry()
    except RegistryError as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    _, packs = _scope_targets(scope, reg)
    it = intent_parse(text, _scoped_registry(scope, reg), voice_only=True,
                      packs=packs)
    # 落盘这次解析(成功和失败都记)。no_match/ambiguous 的原话是**真人真会说、
    # 而清单没覆盖**的说法 —— 模板扩写造不出这种东西。成功也记是为了有分母,
    # 否则漏词率算不出来。写失败不影响返回(log_parse 内部吞异常)。
    # source 记 text 而不是 voice:现在文本框和 ASR 走同一个入口,得能分开统计,
    # 否则将来接了 ASR 就分不清"人打错字"和"听错"这两类问题。
    # text_raw 只在语音路有:ASR 原话 vs 前端纠错后的话。两个都留下才能事后判断
    # 纠错有没有用、有没有纠反 —— 极性词纠反的后果是动作反向,必须可追溯。
    raw = (payload or {}).get("text_raw")
    log_parse(it, scope=scope,
              source=str((payload or {}).get("source") or "text"),
              extra=({"text_raw": str(raw)} if raw else None))
    out = it.to_public()
    out["ready"] = _console_ready()
    out["scope"] = scope
    if it.ok:
        # ⚠ 判「是不是包」用 PACK_KINDS,别写 `== "gesture_pack"`。
        # 漏了 combo_pack 的实测后果:它 skill_id 是 None(包不在清单里),
        # 落到 else 就是 console_targets(reg.get(None), reg),而 targets() 第一行
        # 就是 spec.kind → AttributeError → FastAPI 回 500 **纯文本**
        # "Internal Server Error"。前端 r.json() 拿它去 parse,报
        # 「Unexpected token 'I'」—— 长得像前端 bug,其实崩在这。
        # 包各自动什么设备是**固定**的,不查清单(它们本来就不在清单里)。
        out["devices"] = (PACK_DEVICES[it.kind] if it.kind in PACK_KINDS
                          else sorted(console_targets(reg.get(it.skill_id), reg)))
    return JSONResponse(out)


@app.post("/api/voice/invoke")
async def voice_invoke(payload: dict):
    """SSE:执行一个已确认的意图,走 console 路。

    source 一律**强制** voice,前端说了不算 —— 这样清单里的语音白名单
    (voice_enabled)和语音限速(max_speed)一定生效,绕不过去。
    confirmed 由前端点按钮给;need_confirm 的技能缺它会被安全闸拒。
    """
    global _voice_exec
    # ⚠ **所有包**都走 _voice_play_pack(它内部再按 kind 分流到手势/联合)。
    # 判据用 PACK_KINDS,不逐处写字符串。只认 gesture_pack 的话 combo 包会掉进
    # 下面的技能执行器,而它 skill_id 是 None —— 闸把它当「查不到的技能」拒掉,
    # 报的理由和真实原因(该走包那条路)完全不搭,排查时会往清单里找一个根本
    # 不存在的技能。
    if str((payload or {}).get("kind") or "skill") in PACK_KINDS:
        return _voice_play_pack(payload or {})
    env = {
        "skill_id": (payload or {}).get("skill_id"),
        "params": (payload or {}).get("params") or {},
        "source": "voice",
        "request_id": (payload or {}).get("request_id"),
        "confirmed": bool((payload or {}).get("confirmed")),
        "transcript": (payload or {}).get("transcript"),
        "confidence": (payload or {}).get("confidence"),
    }
    with _voice_lock:                       # 单实例:占位要原子,别两个技能同时下发
        if _voice_exec is not None:
            return JSONResponse({"ok": False, "msg": "已有技能在执行,先『停止』"},
                                status_code=409)
        ex = _console_executor()
        _voice_exec = ex

    async def stream():
        global _voice_exec
        loop = asyncio.get_event_loop()
        q: asyncio.Queue = asyncio.Queue()

        # 用专用线程,不占 _executor 那两个 worker —— 它们还要泵 console stdout,
        # 一条 558 帧的轨迹会把池子占住 20 秒,页面其它请求全卡。
        def worker() -> None:
            try:
                for ev in ex.invoke(env):
                    loop.call_soon_threadsafe(q.put_nowait, ev)
            except Exception as e:                        # noqa: BLE001
                loop.call_soon_threadsafe(
                    q.put_nowait, {"type": "error", "msg": f"执行器异常: {e}"})
            finally:
                loop.call_soon_threadsafe(q.put_nowait, None)

        threading.Thread(target=worker, daemon=True, name="voice-skill").start()
        try:
            while True:
                ev = await q.get()
                if ev is None:
                    break
                yield f"data: {json.dumps(ev, ensure_ascii=False)}\n\n"
        finally:
            # ⚠ 顺序很关键:**先叫停,再让位**。
            # 浏览器断开(切页/刷新/断网)会在这里进来,但 worker 线程还活着,
            # 还在往 console stdin 写帧 —— 臂会在没人看着的情况下继续走完。
            # 只清锁不叫停的后果不是"少动一下",是下一次 invoke 立刻能进来,
            # 于是两个 executor 同时写同一个 stdin:两条轨迹交错,console 收到
            # 的是拼接出来的第三条。这正是 COMBO_DEBUG 说的同通道双写。
            # 正常跑完时 worker 已退出,stop() 只是置个没人看的标志,无副作用。
            ex.stop()
            with _voice_lock:
                if _voice_exec is ex:
                    _voice_exec = None

    return StreamingResponse(stream(), media_type="text/event-stream",
                             headers={"Cache-Control": "no-cache",
                                      "X-Accel-Buffering": "no"})


@app.post("/api/voice/stop")
async def voice_stop() -> JSONResponse:
    """停止继续下发。

    ⚠ **不是急停**。已经发出去的那条指令臂还在走完。真要停机器人说「急停」——
    那是一条独立技能(免确认),会直接给臂发 estop,不排在本次执行后面等。
    """
    ex = _voice_exec
    if ex is None:
        return JSONResponse({"ok": True, "msg": "当前没有在执行的技能"})
    ex.stop()
    return JSONResponse({"ok": True, "msg": "已请求停止下发 —— 这不是急停"})


@app.post("/api/voice/estop")
async def voice_estop() -> JSONResponse:
    """急停:**绕过队列直接给臂发**,不等当前技能循环发现。

    这是语音「停」唯一正确的落点。若排在 invoke 的 SSE 后面等,长轨迹里
    要等到下一步边界才生效 —— 那正是急停不能接受的延迟。
    同时请求停止下发,避免停完又被后续帧顶起来。
    """
    ex = _voice_exec
    if ex is not None:
        ex.stop()
    send, state = _console_sess_view(lambda: _arm)
    if state() is None:
        return JSONResponse({"ok": False, "msg": "机械臂 console 没在跑,无处可发"},
                            status_code=409)
    res = send({"cmd": "estop"})
    return JSONResponse({**res, "warn": "急停只作用于臂:手没有急停通道,会保持"
                                        "当前位置。臂无抱闸会缓慢下落,注意净空。"})


@app.websocket("/ws/telemetry")
async def telemetry(ws: WebSocket) -> None:
    """实时关节数值流:客户端连上后,每帧 /joint_states 都推过来。"""
    await ws.accept()
    live = _get_live()
    q: asyncio.Queue = asyncio.Queue(maxsize=4)
    live.clients.add(q)
    try:
        if live.latest:
            await ws.send_json(live.latest)               # 立刻给一帧当前值
        while True:
            row = await q.get()
            await ws.send_json(row)
    except WebSocketDisconnect:
        pass
    except Exception:                                     # noqa: BLE001
        pass
    finally:
        live.clients.discard(q)


@app.post("/api/hand/start")
async def hand_start(mock: bool = False,
                     lease_id: str | None = Header(default=None,
                                                    alias="X-Hardware-Lease")) -> JSONResponse:
    """接入灵巧手:打开 /dev/ttyUSB0。3D 由浏览器端 three.js 负责,这里不起渲染进程。

    **默认接真手**(mock=False)。ok 反映的是**串口真的打开了**,不是"进程起来了" ——
    串口打不开时返回 ok=false + 原因,前端据此保持"离线",不能显示成在线。
    """
    owner, error = _require_lease(lease_id, acquire=True)
    if error:
        return error
    hand = _get_hand()
    hand.start(mock=mock)
    await asyncio.get_event_loop().run_in_executor(_executor, hand.wait_ready)
    if hand.error:
        hand.stop()                                   # 起不来就收干净,别留半开的会话
        _release_lease_if_hardware_idle(owner)
        return JSONResponse({"ok": False, "msg": hand.error, "mock": mock},
                            status_code=503)
    return JSONResponse({"ok": True, "mock": hand.mock, "port": hand.port})


@app.post("/api/hand/stop")
async def hand_stop(lease_id: str | None = Header(default=None,
                                                  alias="X-Hardware-Lease")) -> JSONResponse:
    """断开:console 先复位手到安全张开位,再关串口。"""
    _, error = _require_lease(lease_id)
    if error:
        return error
    if _hand_target_mailbox is not None:
        _hand_target_mailbox.reset()
    if _hand is not None:
        await asyncio.get_event_loop().run_in_executor(_executor, _hand.stop)
    return JSONResponse({"ok": True})


@app.get("/api/hand/status")
async def hand_status() -> JSONResponse:
    """手的在线状态 —— 右上角那个指示灯读这个。"""
    h = _hand
    alive = h is not None and h.console is not None and h.console.poll() is None
    return JSONResponse({
        "online": bool(alive and h.ready and not h.error),
        "mock": bool(h.mock) if h else False,
        "port": h.port if h else None,
        "error": h.error if h else None,
        "rerun_url": h.rerun_url if h else None,
    })


@app.get("/api/hand/url")
async def hand_url() -> JSONResponse:
    url = _hand.rerun_url if _hand else None
    return JSONResponse({"rerun_url": url})


@app.post("/api/hand/command")
async def hand_command(payload: dict,
                       lease_id: str | None = Header(default=None,
                                                      alias="X-Hardware-Lease")) -> JSONResponse:
    """手部控制指令:{cmd:"angles",rad:[6]} | {cmd:"speed",value:500} | {cmd:"home"} 等。
    协议见 hand_console.py 的 handle()。"""
    _, error = _require_lease(lease_id, payload)
    if error:
        return error
    import time
    t0 = time.perf_counter()

    hand = _get_hand()
    command = dict(payload)
    if command.get("cmd") == "home" and _hand_target_mailbox is not None:
        # 摄像头停止时可能仍有一帧 ANGLE_SET 正在等待串口 ACK。先丢弃 pending，
        # 再等 in-flight 结束，保证张开命令一定排在最后，不被旧跟随帧覆盖。
        _hand_target_mailbox.reset()
        deadline = time.monotonic() + 0.5
        while _hand_target_mailbox.in_flight_count and time.monotonic() < deadline:
            await asyncio.sleep(0.01)
    perf_id = command.get("perf_id")
    if command.get("cmd") == "angles":
        command["_perf"] = {
            "id": perf_id,
            "enqueued_ns": time.perf_counter_ns(),
        }
    result = hand.command(command)

    t1 = time.perf_counter()
    if payload.get("cmd") == "angles":
        print(
            f"[perf-hand/enqueue] id={perf_id} FastAPI到stdin={(t1-t0)*1000:.1f}ms",
            flush=True,
        )

    return JSONResponse(result)


@app.get("/api/hand/actions")
async def hand_actions() -> JSONResponse:
    """可用动作序列列表。直接解析 DefaultAction.txt —— 它是**文件内容**,不该依赖
    会话起没起,所以接入之前就能在页面上看到列表(点某一项才需要会话)。
    action_sequences 只用 stdlib,本环境可直接 import。"""
    try:
        from action_sequences import load_default_actions
        seqs = load_default_actions()
    except Exception as e:                                # noqa: BLE001
        return JSONResponse({"actions": [], "error": str(e)})
    return JSONResponse({"actions": [
        {"slot": s.slot, "index": s.index, "name": s.name, "steps": len(s.steps),
         "duration_ms": sum(st.delay_ms for st in s.steps)} for s in seqs
    ]})


@app.post("/api/hand/action/start")
async def hand_action_start(slot: int, lease_id: str | None = Header(default=None,
                                                                       alias="X-Hardware-Lease")) -> JSONResponse:
    """开始播放指定动作序列。slot = 列表位置(唯一);index 在文件里有重复,不能用来定位。"""
    _, error = _require_lease(lease_id)
    if error:
        return error
    hand = _get_hand()
    if hand.console is None or hand.console.poll() is not None:
        return JSONResponse({"ok": False, "msg": "未接入灵巧手,先点『接入灵巧手』"},
                            status_code=409)
    return JSONResponse(hand.command({"cmd": "action_start", "slot": slot}))


@app.post("/api/hand/action/pause")
async def hand_action_pause(lease_id: str | None = Header(default=None,
                                                          alias="X-Hardware-Lease")) -> JSONResponse:
    _, error = _require_lease(lease_id)
    if error:
        return error
    hand = _get_hand()
    return JSONResponse(hand.command({"cmd": "action_pause"}))


@app.post("/api/hand/action/resume")
async def hand_action_resume(lease_id: str | None = Header(default=None,
                                                           alias="X-Hardware-Lease")) -> JSONResponse:
    _, error = _require_lease(lease_id)
    if error:
        return error
    hand = _get_hand()
    return JSONResponse(hand.command({"cmd": "action_resume"}))


@app.post("/api/hand/action/stop")
async def hand_action_stop(lease_id: str | None = Header(default=None,
                                                         alias="X-Hardware-Lease")) -> JSONResponse:
    """停止当前动作,并复位手到初始张开位。技能包回放也走这个(同一个播放器)。"""
    _, error = _require_lease(lease_id)
    if error:
        return error
    hand = _get_hand()
    return JSONResponse(hand.command({"cmd": "action_stop"}))


# ---------------------------------------------------------------------------
# 手势技能包 —— 录制的关键帧序列,存 JSON,可按名回放
#
# ⚠ 路径沙箱**只在这一层**做:所有读写都过 gesture_pack.resolve_pack_path(),
#   console 只收内联的 pack 数据、不碰文件系统。7860 端口没有认证,校验散成
#   两份的话哪天只改一处就是远程任意文件读写。
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# 臂+手联合录制包(combo_pack)。和下面的手势包 API 刻意同构。
#
# ⚠ 路径沙箱**只在 combo_pack.resolve_pack_path() 那一层**做,和手势包共用一份
#   实现(gesture_pack.resolve_in_root)。这里不要自己拼路径。
# ⚠ 7860 端口**没有认证**。这四个端点里 save/delete 是写操作 —— 同一网段的任何人
#   都能调。录制包本身不驱动硬件(要显式点回放),但能写文件,所以路径沙箱是
#   唯一的防线,别绕过它。
# ---------------------------------------------------------------------------
@app.get("/api/combo/list")
async def combo_list() -> JSONResponse:
    """列出所有联合录制包。坏文件跳过,不整体报错。"""
    import combo_pack as cbp
    try:
        return JSONResponse({"ok": True, "root": str(cbp.combo_root()),
                             "packs": cbp.list_packs()})
    except OSError as e:
        return JSONResponse({"ok": False, "msg": f"读根目录失败: {e}",
                             "root": str(cbp.combo_root()), "packs": []},
                            status_code=500)


@app.get("/api/combo/get")
async def combo_get(path: str) -> JSONResponse:
    """读一个联合录制包的完整内容。"""
    import combo_pack as cbp
    try:
        pack = cbp.load_pack(path)
    except cbp.ComboError as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=400)
    except OSError as e:
        return JSONResponse({"ok": False, "msg": f"读文件失败: {e}"}, status_code=500)
    # ee_mismatch 报给前端:包被手改过 arm_rad 但没更新 ee_pose 时提示一下。
    # 不是错误(arm_rad 才是权威),但值得让人知道。
    return JSONResponse({"ok": True, "path": path, "pack": pack.to_dict(),
                         "ee_mismatch": pack.ee_mismatch})


@app.post("/api/combo/save")
async def combo_save(payload: dict) -> JSONResponse:
    """保存联合录制包。{"path":"名.json", "pack":{...}, "overwrite":true}

    校验先于落盘:pack 不合法就不建目录、不写文件(同手势包的理由 —— 否则
    手滑存一个坏包会在根目录留下空子目录)。
    """
    import combo_pack as cbp
    try:
        pack = cbp.ComboPack.from_dict(payload.get("pack") or {})
        rel = str(payload.get("path") or "")
        cbp.resolve_pack_path(rel)                    # 先验路径,后落盘
        p = cbp.save_pack(rel, pack, overwrite=bool(payload.get("overwrite", True)))
    except cbp.ComboError as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=400)
    except OSError as e:
        return JSONResponse({"ok": False, "msg": f"写文件失败: {e}"}, status_code=500)
    return JSONResponse({"ok": True, "path": cbp.rel_of(p), "abs": str(p),
                         "name": pack.name, "frames": len(pack.frames),
                         "mode": pack.mode, "recorded_from": pack.recorded_from,
                         "duration_ms": pack.duration_ms})


@app.get("/api/combo/check")
async def combo_check(path: str) -> JSONResponse:
    """回放前的体检:**只读,不动硬件**。给前端的确认框凑数据用。

    ⚠ 单独开一个端点而不是塞进 play:确认框要先给人看"会动多大"再让人点。
    play 里再查一遍(纵深防御)—— 这两次之间臂可能被别的东西动过。
    """
    import combo_pack as cbp
    try:
        pack = cbp.load_pack(path)
    except cbp.ComboError as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=400)
    if pack.mode != "keyframe":
        # ⚠ 理由**不再是**「web 层拿不到 NeroArm」—— CPV 现在就跑在 arm_console 里。
        # 真实原因是流式包 30fps × 几十秒 = 上千帧,一次 stdin JSON 递过去太大,
        # 而且 skip_arm 语义(落后就跳帧)下页面进度条没有意义。
        return JSONResponse({"ok": False, "msg":
            f"mode={pack.mode} 的包页面上放不了 —— 流式包上千帧,"
            f"要按流式喂而不是一次性下发。请用命令行:"
            f"combo_player.py --combo {path}"}, status_code=400)

    R2D = 180.0 / 3.141592653589793
    arm, hand = _get_arm(), _get_hand()
    arm_on = bool(arm.console and arm.console.poll() is None)
    hand_on = bool(hand.console and hand.console.poll() is None)
    st = arm.latest or {}
    # 帧间最大单关节跳变。⚠ 走 CPV 之后这个数的**含义变了**:原来(move_j)大跳变
    # 意味着规划被下一帧打断、臂走不到任何一个点;现在是位置环,重设目标不打断,
    # 后果是**速度**—— 跳变越大冲得越快。仍然要报,只是理由不同。
    worst_step, worst_at = 0.0, 0
    for k in range(1, len(pack.frames)):
        d = max(abs(a - b) for a, b in zip(pack.frames[k].arm_rad,
                                           pack.frames[k - 1].arm_rad))
        if d > worst_step:
            worst_step, worst_at = d, k + 1
    # 当前位姿到第 0 帧的距离 —— 这一段是**回放开始时立刻发生**的运动
    first_gap = None
    if st.get("rad"):
        first_gap = max(abs(a - b) for a, b in
                        zip(pack.frames[0].arm_rad, st["rad"]))
    return JSONResponse({
        "ok": True, "name": pack.name, "path": path,
        "frames": len(pack.frames), "duration_ms": pack.duration_ms,
        "mode": pack.mode, "recorded_from": pack.recorded_from,
        "arm_online": arm_on, "hand_online": hand_on,
        "arm_enabled": bool(st.get("enabled")), "arm_frozen": bool(st.get("frozen")),
        "speed_percent": st.get("speed_percent"),
        "worst_step_deg": round(worst_step * R2D, 1), "worst_step_at": worst_at,
        "first_gap_deg": None if first_gap is None else round(first_gap * R2D, 1),
    })


@app.post("/api/combo/play")
async def combo_play(payload: dict) -> JSONResponse:
    """回放录制包。{"path":"..."}

    ⚠ 这个端点**会同时驱动臂和手**。7860 没有认证 —— 同一网段的任何人都能调。
    能力上和已有的 /api/arm/command 同级(那个也让臂动),差别是这里发的是**一串**
    而不是一个点。把 preflight 做足是唯一的防线:未使能 / 急停中 / 未接入一律拒。
    """
    import combo_pack as cbp
    rel = str((payload or {}).get("path") or "").strip()
    if not rel:
        return JSONResponse({"ok": False, "msg": "要给 path"}, status_code=400)
    try:
        pack = cbp.load_pack(rel)
    except cbp.ComboError as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=400)

    ok, msg, code = _combo_start(pack, rel)
    if not ok:
        return JSONResponse({"ok": False, "msg": msg}, status_code=code)
    return JSONResponse({"ok": True, "name": pack.name, "path": rel,
                         "frames": len(pack.frames),
                         "duration_ms": pack.duration_ms})


@app.get("/api/combo/play/status")
async def combo_play_status() -> JSONResponse:
    """回放进度。正式播放取臂遥测；prepare/start 间隙取 Web 协调状态。

    ⚠ 不再用 ComboPlaySession。臂侧的 ComboPlayer 是唯一的进度来源(手侧跟着
    同一条 start_at 时间轴走,不单独报)。
    """
    st = (_get_arm().latest or {}).get("combo")
    if not st:
        with _combo_pending_lock:
            pending = dict(_combo_pending) if _combo_pending is not None else None
        if pending:
            return JSONResponse({"ok": True, "running": True,
                                 "name": pending["name"],
                                 "phase": pending["phase"], "progress": 0.0,
                                 "elapsed_ms": 0,
                                 "total_ms": pending["duration_ms"],
                                 "paused": False, "i": 0,
                                 "n": pending["frames"], "fail": 0})
        return JSONResponse({"ok": True, "running": False})
    return JSONResponse({"ok": True, "running": True, **st})


def _combo_ctl(arm_cmd: str, hand_cmd: str) -> JSONResponse:
    """把暂停/恢复/停止同时发给两个 console。

    ⚠ 两条指令**不可能真正同时到**(两个管道)。但暂停/恢复的语义对几毫秒的
    偏差不敏感 —— 各自记录自己的暂停时长,恢复后仍按 t_ns 定位,不累积漂移。
    """
    global _combo_pending
    arm, hand = _get_arm(), _get_hand()
    if arm_cmd == "combo_stop":
        with _combo_pending_lock:
            _combo_pending = None
    if arm.console and arm.console.poll() is None:
        arm.command({"cmd": arm_cmd})
    if hand.console and hand.console.poll() is None:
        hand.command({"cmd": hand_cmd})
    return JSONResponse({"ok": True})


@app.post("/api/combo/play/pause")
async def combo_play_pause() -> JSONResponse:
    return _combo_ctl("combo_pause", "action_pause")


@app.post("/api/combo/play/resume")
async def combo_play_resume() -> JSONResponse:
    return _combo_ctl("combo_resume", "action_resume")


@app.post("/api/combo/play/stop")
async def combo_play_stop() -> JSONResponse:
    """停止回放。臂停在最后收到的目标位,**不回零**(回零路径未知)。

    ⚠ 手侧用 action_stop,那个会 set_angles(HOME_RAD) 把手张开 —— 和臂「停在
    原地」不一致。这是有意的:手张开是安全姿态(不夹东西),臂回零反而危险
    (回零路径可能扫过障碍)。
    """
    return _combo_ctl("combo_stop", "action_stop")


@app.post("/api/combo/delete")
async def combo_delete(payload: dict) -> JSONResponse:
    import combo_pack as cbp
    try:
        cbp.delete_pack(str(payload.get("path") or ""))
    except cbp.ComboError as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=400)
    except OSError as e:
        return JSONResponse({"ok": False, "msg": f"删除失败: {e}"}, status_code=500)
    return JSONResponse({"ok": True})


@app.get("/api/hand/gestures")
async def gesture_list() -> JSONResponse:
    """列出根目录下所有技能包(递归)。坏文件带 error 字段列出来,不整体报错。"""
    import gesture_pack as gp
    try:
        return JSONResponse({"ok": True, "root": str(gp.gesture_root()),
                             "packs": gp.list_packs()})
    except OSError as e:
        return JSONResponse({"ok": False, "msg": f"读根目录失败: {e}",
                             "root": str(gp.gesture_root()), "packs": []},
                            status_code=500)


@app.get("/api/hand/gesture")
async def gesture_get(path: str) -> JSONResponse:
    """读一个技能包的完整内容(前端「编辑」用)。"""
    import gesture_pack as gp
    try:
        return JSONResponse({"ok": True, "path": path,
                             "pack": gp.load_pack(path).to_dict()})
    except gp.GestureError as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=400)
    except OSError as e:
        return JSONResponse({"ok": False, "msg": f"读文件失败: {e}"}, status_code=500)


@app.post("/api/hand/gesture/save")
async def gesture_save(payload: dict) -> JSONResponse:
    """保存技能包。{"path":"子目录/名.json", "pack":{...}, "overwrite":true}

    校验先于落盘:pack 不合法就不建目录、不写文件 —— 否则手滑存一个坏包会在
    根目录留下空子目录。
    """
    import gesture_pack as gp
    try:
        pack = gp.GesturePack.from_dict(payload.get("pack") or {})
        rel = str(payload.get("path") or "")
        gp.resolve_pack_path(rel)                     # 先验路径,后落盘
        p = gp.save_pack(rel, pack, overwrite=bool(payload.get("overwrite", True)))
    except gp.GestureError as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=400)
    except OSError as e:
        return JSONResponse({"ok": False, "msg": f"写文件失败: {e}"}, status_code=500)
    return JSONResponse({"ok": True, "path": gp.rel_of(p), "abs": str(p),
                         "name": pack.name, "frames": len(pack.frames),
                         "duration_ms": pack.duration_ms})


@app.post("/api/hand/gesture/delete")
async def gesture_delete(payload: dict) -> JSONResponse:
    import gesture_pack as gp
    try:
        gp.delete_pack(str(payload.get("path") or ""))
    except gp.GestureError as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=400)
    except OSError as e:
        return JSONResponse({"ok": False, "msg": f"删除失败: {e}"}, status_code=500)
    return JSONResponse({"ok": True})


# ---------------------------------------------------------------------------
# 视频 → MediaPipe → 重定向 → 关节角。给「视频」栏用。
#
# ⚠ 这条链**只读视频、不碰硬件**。解出来的角度要下发得先存成技能包,再显式点回放。
# ---------------------------------------------------------------------------
@app.get("/api/hand/videos")
async def hand_videos() -> JSONResponse:
    import video_gesture as vg
    return JSONResponse({"ok": True, "videos": vg.list_videos()})


@app.post("/api/hand/video/extract")
async def hand_video_extract(payload: dict) -> JSONResponse:
    """开一个抽取任务(后台线程),立刻返回。进度靠 /status 轮询。

    不做 SSE 是因为这里的进度是**单一数值**(done/total),轮询足够;SSE 那套
    (见 /api/run)是给多阶段管线的日志流用的。
    """
    import video_gesture as vg
    video = str(payload.get("video") or "").strip()
    if not video:
        return JSONResponse({"ok": False, "msg": "要给 video"}, status_code=400)
    # 去尖刺默认开。关掉是给"想看原始输出做对比"用的,不是给"觉得平滑不好"用的 ——
    # 这里做的是 3 点中值去单帧离群,不是平滑(video_gesture 里那段注释解释了为什么
    # **不**加 SavGol 平滑)。
    do_despike = payload.get("despike", True) is not False
    hand_type = "Left" if str(payload.get("hand_type", "Right")) == "Left" else "Right"
    if not Path(video).is_file():
        return JSONResponse({"ok": False, "msg": f"视频不存在: {video}"},
                            status_code=400)
    # ⚠ 占位必须在 run_in_executor **之前**,而且是原子的。读 job.running 再判断
    # 是不够的:executor 只把活排进队列就返回,工作线程还没把 running 置上,
    # 后一个请求就过检查了 —— 两个任务同时写同一个 _job,进度和结果全混。
    if not vg.claim(video, hand_type=hand_type):
        return JSONResponse({"ok": False, "msg": "已有抽取任务在跑,先取消"},
                            status_code=409)

    def worker() -> None:
        try:
            vg.extract(video, hand_type=hand_type, do_despike=do_despike)
        except Exception as e:                             # noqa: BLE001
            vg.release(str(e))            # 放掉任务位,否则永久卡 running

    asyncio.get_event_loop().run_in_executor(_executor, worker)
    return JSONResponse({"ok": True, "video": video, "stride": vg.DEFAULT_STRIDE,
                         "all_source_frames": True, "hand_type": hand_type})


@app.get("/api/hand/video/status")
async def hand_video_status() -> JSONResponse:
    import video_gesture as vg
    return JSONResponse({"ok": True, **vg.current_job().to_dict()})


@app.post("/api/hand/video/cancel")
async def hand_video_cancel() -> JSONResponse:
    import video_gesture as vg
    vg.cancel_job()
    return JSONResponse({"ok": True})


@app.get("/api/hand/video/keyframes")
async def hand_video_keyframes(eps: float = 0.25, max_out: int = 12,
                               speed: int = 500, force: int = 500,
                               mode: str = "key") -> JSONResponse:
    """挑帧 + 转成技能包帧格式,前端直接灌进录制器。

    mode="key"   自动挑关键帧(eps/max_out 生效)。适合"几个定格姿态"的手势。
    mode="all"   **全部检出帧**,不挑。适合要复现连续动作 —— 关键帧模式只留几个
                 定格姿态,中间的过程全丢了,回放就是一顿一顿的。

    eps 是姿态差阈值(rad,6 关节 L∞)。调小挑得多、调大挑得少。
    """
    import video_gesture as vg
    from gesture_pack import MAX_FRAMES
    job = vg.current_job()
    if not job.frames:
        return JSONResponse({"ok": False, "msg": "还没有抽取结果"}, status_code=409)
    dense = str(mode).lower() in ("all", "dense", "full")
    if dense:
        kf = job.frames[:MAX_FRAMES]
        eps = 0.0
        max_out = len(kf)
    else:
        eps = max(0.01, min(1.5, float(eps)))
        max_out = max(1, min(MAX_FRAMES, int(max_out)))
        kf = vg.pick_keyframes(job.frames, eps=eps, max_out=max_out)
    frames, timing = vg.frames_to_pack_frames(kf, speed=int(speed), force=int(force))
    return JSONResponse({"ok": True, "mode": "all" if dense else "key",
                         "eps": eps, "max_out": max_out,
                         "n_all": len(job.frames), "n_detected": len(job.frames),
                         "source_frames": job.done, "n_picked": len(kf),
                         "hit_cap": len(kf) < len(job.frames)
                                    if dense else len(kf) >= max_out,
                         "timing": timing, "quality": job.quality,
                         "despiked": job.despiked, "frames": frames})


@app.get("/api/hand/video/frames")
async def hand_video_frames(limit: int = 0) -> JSONResponse:
    """逐帧结果(给时间轴刷子用)。默认返回全部检出帧;limit>0 可显式截取。"""
    import video_gesture as vg
    job = vg.current_job()
    cap = max(1, int(limit)) if limit > 0 else len(job.frames)
    fr = job.frames[:cap]
    return JSONResponse({"ok": True, "source_frames": job.done,
                         "n_all": len(job.frames), "n_detected": len(job.frames),
                         "truncated": len(fr) < len(job.frames),
                         "quality": job.quality, "despiked": job.despiked,
                         "frames": fr})


@app.post("/api/hand/mimic")
async def hand_mimic(payload: dict) -> JSONResponse:
    """根据 MediaPipe 视觉数据实时计算关节角度（retargeting）

    payload: {
        "format": "mediapipe",
        "landmarks": [...],  # 21个3D点
    }

    返回: {
        "ok": true,
        "joint_angles": {...},  # 6个关节的弧度值
        "gesture": "..."        # 可选：识别的手势名称
    }
    """
    format_type = str(payload.get("format") or "mediapipe")
    landmarks = payload.get("landmarks") or []

    if format_type != "mediapipe":
        return JSONResponse(
            {"ok": False, "msg": f"仅支持 MediaPipe 格式，收到: {format_type}"},
            status_code=400
        )

    if len(landmarks) != 21:
        return JSONResponse(
            {"ok": False, "msg": f"需要 21 个关键点，收到: {len(landmarks)}"},
            status_code=400
        )

    try:
        # 调用 retargeting 计算关节角度
        joint_angles = await _mediapipe_to_joint_angles(landmarks)

        # 可选：识别手势名称（用于调试显示）
        gesture = _recognize_mediapipe_gesture(landmarks) or "未知"

        return JSONResponse({
            "ok": True,
            "frame_id": payload.get("frame_id"),
            "joint_angles": joint_angles,
            "gesture": gesture
        })

    except Exception as e:
        print(f"[hand_mimic] Retargeting 失败: {e}")
        return JSONResponse(
            {"ok": False, "msg": f"Retargeting 失败: {str(e)}"},
            status_code=500
        )


@app.websocket("/ws/hand/mimic")
async def ws_hand_mimic(websocket: WebSocket):
    """MediaPipe hand retargeting plus anchored arm/hand combo tracking."""
    await websocket.accept()
    owner = f"mimic-ws-{id(websocket)}"
    target_filter = OneEuroJointFilter()
    wrist_position_filter = OneEuroVectorFilter(
        min_cutoff_hz=1.2, beta=0.5, reset_after_ms=200.0
    )
    wrist_orientation_filter = OneEuroRotationFilter(
        min_cutoff_hz=1.8, beta=0.8, reset_after_ms=200.0
    )
    last_position_filter = {
        "reset": False, "raw_delta_m": 0.0, "filtered_delta_m": 0.0,
    }
    last_orientation_filter = {
        "reset": False, "raw_delta_rad": 0.0, "filtered_delta_rad": 0.0,
    }
    mapper = LiveWristMapper(
        position_basis=np.array([[0.0, 1.0, 0.0],
                                 [-1.0, 0.0, 0.0],
                                 [0.0, 0.0, 1.0]]),
        rotation_basis=np.array([[0.0, 0.0, -1.0],
                                 [1.0, 0.0, 0.0],
                                 [0.0, -1.0, 0.0]]),
        # 姿态相对锚定后再进入限幅和 IK，挥手等腕部旋转才能传到末端。
        track_orientation=True,
    )
    ik_client: LiveIKClient | None = None
    ik_scheduler: LatestIKScheduler | None = None
    session_generation = time.monotonic_ns()
    session_active = True
    authorization_revision = 0
    current_drive_arm = False
    current_allow_real_arm = False
    last_arm_targets: list[float] | None = None
    consecutive_ik_failures = 0
    joint_order = (
        "right_thumb_1_joint",
        "right_thumb_2_joint",
        "right_index_1_joint",
        "right_middle_1_joint",
        "right_ring_1_joint",
        "right_little_1_joint",
    )
    print("[ws] 合体跟随客户端已连接")

    def report_ik(perf: dict) -> None:
        print(
            "[perf-arm/ik] "
            f"id={perf.get('id')} replaced={perf.get('replaced')} "
            f"age={perf.get('age_ms')}ms solve={perf.get('ik_ms')}ms "
            f"status={perf.get('status')}",
            flush=True,
        )

    async def apply_ik_result(
        target: IKTarget, solved: dict, metrics: dict
    ) -> dict:
        nonlocal consecutive_ik_failures, last_arm_targets

        context = target.context
        result = {
            "queued": False,
            "ik_ok": bool(solved.get("ok", True) and solved.get("ik_ok")),
            "ik_ms": metrics["ik_ms"],
            "ik_age_ms": metrics["age_ms"],
            "ik_completed_replaced": metrics["replaced"],
            "source_frame_id": target.frame_id,
            "reason": None,
            "position_limited": bool(context.get("position_limited")),
            "orientation_limited": bool(context.get("orientation_limited")),
            "orientation_delta_deg": list(context.get("orientation_delta_deg") or ()),
            "orientation_limited_axes": list(
                context.get("orientation_limited_axes") or ()
            ),
        }
        valid_session = bool(
            session_active
            and target.session_generation == session_generation
            and target.anchor_revision == mapper.anchor_revision
            and mapper.state == "following"
            and context.get("authorization_revision") == authorization_revision
        )
        if not valid_session:
            result["reason"] = "stale_session"
            return result

        if not result["ik_ok"]:
            consecutive_ik_failures += 1
            result["reason"] = "ik_failed"
            if consecutive_ik_failures >= 3:
                mapper.freeze("ik_failed")
                if ik_scheduler is not None:
                    ik_scheduler.release(owner)
                if _arm_target_mailbox is not None:
                    _arm_target_mailbox.release(owner)
                if _arm is not None:
                    _arm.end_tracking()
            return result

        q = [float(value) for value in solved.get("q", ())]
        if len(q) != 7 or not all(np.isfinite(value) for value in q):
            result.update(ik_ok=False, reason="invalid_ik_result")
            consecutive_ik_failures += 1
            if consecutive_ik_failures >= 3:
                mapper.freeze("invalid_ik_result")
                if ik_scheduler is not None:
                    ik_scheduler.release(owner)
                if _arm_target_mailbox is not None:
                    _arm_target_mailbox.release(owner)
                if _arm is not None:
                    _arm.end_tracking()
            return result

        consecutive_ik_failures = 0
        last_arm_targets = q
        result["arm_joint_targets"] = q

        arm = _arm
        arm_online = bool(
            arm and arm.ready and arm.console and arm.console.poll() is None
        )
        latest = (arm.latest or {}) if arm else {}
        drive_arm = bool(current_drive_arm and context.get("drive_arm"))
        allow_real_arm = bool(
            current_allow_real_arm and context.get("allow_real_arm_tracking")
        )
        arm_safe = bool(
            arm_online and latest.get("enabled") and not latest.get("frozen")
            and (arm.mock or allow_real_arm)
        )
        if drive_arm and arm_safe:
            queued = _get_arm_target_mailbox().submit(
                owner, target.frame_id, q, created_at=target.created_at
            )
            result.update(
                queued=queued.accepted,
                replaced=queued.replaced,
                reason=queued.reason,
            )
        elif drive_arm:
            result["reason"] = (
                "offline" if not arm_online
                else "disabled" if not latest.get("enabled")
                else "frozen" if latest.get("frozen")
                else "real_arm_not_armed"
            )
        return result

    def ensure_ik_scheduler() -> LatestIKScheduler:
        nonlocal ik_scheduler
        if ik_client is None:
            raise RuntimeError("IK client is not ready")
        if ik_scheduler is None:
            def solve(target: IKTarget) -> dict:
                return ik_client.request({
                    "cmd": "solve",
                    "target_pose": list(target.target_pose),
                })

            ik_scheduler = LatestIKScheduler(
                solve,
                apply_ik_result,
                max_input_age_ms=180.0,
                max_result_age_ms=200.0,
                reporter=report_ik,
            )
        return ik_scheduler

    try:
        while True:
            data = await websocket.receive_json()
            received_at = time.monotonic()
            format_type = data.get("format")
            world_landmarks = data.get("landmarks", [])
            image_landmarks = data.get("image_landmarks", [])
            frame_id = data.get("frame_id")
            tracking_control = data.get("tracking_control")
            tracking_control_applied = False
            requested_drive_arm = bool(data.get("drive_arm"))
            requested_allow_real_arm = bool(data.get("allow_real_arm_tracking"))
            if (
                requested_drive_arm != current_drive_arm
                or requested_allow_real_arm != current_allow_real_arm
            ):
                was_driving_arm = current_drive_arm
                current_drive_arm = requested_drive_arm
                current_allow_real_arm = requested_allow_real_arm
                authorization_revision += 1
                if ik_scheduler is not None:
                    ik_scheduler.release(owner)
                if was_driving_arm and not current_drive_arm:
                    if _arm_target_mailbox is not None:
                        _arm_target_mailbox.release(owner)
                    if _arm is not None:
                        _arm.end_tracking()

            if format_type != "mediapipe":
                await websocket.send_json({
                    "ok": False,
                    "msg": f"仅支持 MediaPipe 格式，收到: {format_type}"
                })
                continue

            if tracking_control == "freeze":
                mapper.freeze("user")
                wrist_position_filter.reset()
                wrist_orientation_filter.reset()
                tracking_control_applied = True
                if ik_scheduler is not None:
                    ik_scheduler.release(owner)
                if _arm_target_mailbox is not None:
                    _arm_target_mailbox.release(owner)
                if _arm is not None:
                    _arm.end_tracking()

            if data.get("hand_present") is False:
                mapper.mark_missing()
                target_filter.reset()
                wrist_position_filter.reset()
                wrist_orientation_filter.reset()
                if ik_scheduler is not None:
                    ik_scheduler.release(owner)
                if _hand_target_mailbox is not None:
                    _hand_target_mailbox.release(owner)
                if _arm_target_mailbox is not None:
                    _arm_target_mailbox.release(owner)
                if _arm is not None:
                    _arm.end_tracking()
                await websocket.send_json({
                    "ok": True,
                    "frame_id": frame_id,
                    **mapper.status(),
                    "joint_angles": None,
                    "arm_joint_targets": last_arm_targets,
                    "hand_joint_targets": None,
                    "msg": "未检测到手部",
                    "hardware": {"queued": False, "reason": "hand_lost"},
                    "arm": {"queued": False, "ik_ok": False,
                            "reason": "hand_lost"},
                    "wrist_position_filter": last_position_filter,
                    "wrist_orientation_filter": last_orientation_filter,
                    "tracking_control_applied": tracking_control_applied,
                })
                continue

            if len(world_landmarks) != 21:
                await websocket.send_json({
                    "ok": False,
                    "msg": f"需要 21 个世界关键点，收到: {len(world_landmarks)}"
                })
                continue

            try:
                joint_angles = await _mediapipe_to_joint_angles(world_landmarks)
                gesture = _recognize_mediapipe_gesture(world_landmarks) or "未知"
                raw_hand_target = [joint_angles[name] for name in joint_order]

                wrist_observation = None
                mapping = None
                combo_mode = data.get("tracking_mode") == "combo"
                if combo_mode and len(image_landmarks) == 21:
                    wrist_observation = estimate_wrist_observation(
                        image_landmarks,
                        world_landmarks,
                        data.get("handedness"),
                    )
                    if tracking_control == "anchor":
                        wrist_position_filter.reset()
                        wrist_orientation_filter.reset()
                        if ik_scheduler is not None:
                            ik_scheduler.release(owner)
                        if _arm_target_mailbox is not None:
                            _arm_target_mailbox.release(owner)
                        if _arm is not None:
                            _arm.end_tracking()
                        if ik_client is None:
                            ik_client = await asyncio.get_event_loop().run_in_executor(
                                None, LiveIKClient
                            )
                        ensure_ik_scheduler()
                        arm = _arm
                        arm_online = bool(
                            arm and arm.ready and arm.console
                            and arm.console.poll() is None
                        )
                        latest = (arm.latest or {}) if arm else {}
                        current_q = (
                            latest.get("target") or latest.get("rad")
                            or (arm.connect_pose if arm else None)
                            or ik_client.q_home
                        )
                        fk_result = await asyncio.get_event_loop().run_in_executor(
                            None, ik_client.request,
                            {"cmd": "fk", "q": current_q},
                        )
                        arm_anchor = np.asarray(fk_result["pose"], dtype=float).reshape(4, 4)
                        if arm_online and not arm.mock:
                            mapper.position_limits[:] = 0.02
                            mapper.set_orientation_limits_deg(
                                (-45.0, -25.0, -35.0),
                                (45.0, 25.0, 35.0),
                            )
                        else:
                            mapper.position_limits[:] = [0.05, 0.05, 0.03]
                            # 固定准备位、固定末端位置的 5° 步进 IK 扫描可达约
                            # X -95/+65、Y -120/+55、Z -180/+165°。Mock 留
                            # 5-10° 余量；真机仍保留上面的保守限幅。
                            mapper.set_orientation_limits_deg(
                                (-90.0, -115.0, -175.0),
                                (60.0, 50.0, 155.0),
                            )
                        mapper.request_anchor(arm_anchor)
                        tracking_control_applied = True
                    filtered_position = wrist_position_filter.update(
                        wrist_observation.position, received_at
                    )
                    last_position_filter = {
                        "reset": filtered_position.reset,
                        "raw_delta_m": round(filtered_position.raw_delta, 6),
                        "filtered_delta_m": round(filtered_position.filtered_delta, 6),
                    }
                    filtered_orientation = wrist_orientation_filter.update(
                        wrist_observation.rotation, received_at
                    )
                    last_orientation_filter = {
                        "reset": filtered_orientation.reset,
                        "raw_delta_rad": round(filtered_orientation.raw_delta_rad, 6),
                        "filtered_delta_rad": round(
                            filtered_orientation.filtered_delta_rad, 6
                        ),
                    }
                    wrist_observation = WristObservation(
                        filtered_position.value,
                        filtered_orientation.value,
                        wrist_observation.handedness,
                        wrist_observation.handedness_score,
                        wrist_observation.position_source,
                    )
                    mapping = mapper.observe(wrist_observation)

                completed_arm = (
                    ik_scheduler.latest_result if ik_scheduler is not None else None
                )
                arm_result = {
                    "queued": False,
                    "ik_ok": mapper.state != "following",
                    "reason": "not_following",
                    "position_limited": False,
                    "orientation_limited": False,
                    "orientation_delta_deg": [0.0, 0.0, 0.0],
                    "orientation_limited_axes": [False, False, False],
                }
                if completed_arm is not None:
                    arm_result.update(completed_arm)
                if mapping is not None:
                    arm_result.update({
                        "reason": None,
                        "position_limited": mapping.position_limited,
                        "orientation_limited": mapping.orientation_limited,
                        "orientation_delta_deg": [
                            round(value, 2) for value in mapping.orientation_delta_deg
                        ],
                        "orientation_limited_axes": list(mapping.orientation_limited_axes),
                    })

                hardware = {"queued": False, "reason": "disabled"}
                drive_hand = bool(data.get("drive_hand", data.get("drive_hardware")))
                hand_gate = not combo_mode or mapper.state == "following"
                if drive_hand and hand_gate:
                    hand = _hand
                    online = bool(
                        hand and hand.ready and hand.console
                        and hand.console.poll() is None
                    )
                    if online:
                        filter_started = time.perf_counter()
                        filtered = target_filter.update(raw_hand_target, received_at)
                        filter_ms = (time.perf_counter() - filter_started) * 1000.0
                        print(
                            "[perf-hand/filter] "
                            f"id={frame_id} reset={int(filtered.reset)} "
                            f"raw_delta={filtered.raw_delta_rad:.4f}rad "
                            f"filtered_delta={filtered.filtered_delta_rad:.4f}rad "
                            f"suppressed={filtered.suppressed}/6 "
                            f"cost={filter_ms:.3f}ms",
                            flush=True,
                        )
                        if filtered.changed:
                            result = _get_hand_target_mailbox().submit(
                                owner,
                                frame_id,
                                filtered.angles,
                                created_at=received_at,
                            )
                            hardware = {
                                "queued": result.accepted,
                                "replaced": result.replaced,
                                "reason": result.reason,
                                "filtered": True,
                            }
                            if not result.accepted:
                                target_filter.reset()
                        else:
                            hardware = {
                                "queued": False,
                                "replaced": 0,
                                "reason": "filter_resolution",
                                "filtered": True,
                            }
                    else:
                        target_filter.reset()
                        if _hand_target_mailbox is not None:
                            _hand_target_mailbox.release(owner)
                        hardware = {"queued": False, "reason": "offline"}
                else:
                    target_filter.reset()
                    if _hand_target_mailbox is not None:
                        _hand_target_mailbox.release(owner)

                arm = _arm
                arm_online = bool(
                    arm and arm.ready and arm.console and arm.console.poll() is None
                )
                latest = (arm.latest or {}) if arm else {}
                arm_safe = bool(
                    arm_online and latest.get("enabled") and not latest.get("frozen")
                    and (arm.mock or current_allow_real_arm)
                )
                if mapping is not None:
                    scheduler = ensure_ik_scheduler()
                    queued_ik = scheduler.submit(
                        owner,
                        session_generation,
                        frame_id,
                        mapper.anchor_revision,
                        mapping.target_pose.reshape(-1).tolist(),
                        created_at=received_at,
                        context={
                            "authorization_revision": authorization_revision,
                            "drive_arm": current_drive_arm,
                            "allow_real_arm_tracking": current_allow_real_arm,
                            "position_limited": mapping.position_limited,
                            "orientation_limited": mapping.orientation_limited,
                            "orientation_delta_deg": [
                                round(value, 2)
                                for value in mapping.orientation_delta_deg
                            ],
                            "orientation_limited_axes": list(
                                mapping.orientation_limited_axes
                            ),
                        },
                    )
                    arm_result.update(
                        ik_queued=queued_ik.accepted,
                        ik_replaced=queued_ik.replaced,
                        ik_pending=scheduler.pending_count,
                        ik_in_flight=scheduler.in_flight_count,
                    )
                    if completed_arm is None:
                        arm_result.update(ik_ok=None, reason=queued_ik.reason)

                if current_drive_arm and not arm_safe:
                    arm_result["reason"] = (
                        "offline" if not arm_online
                        else "disabled" if not latest.get("enabled")
                        else "frozen" if latest.get("frozen")
                        else "real_arm_not_armed"
                        if arm and not arm.mock and not current_allow_real_arm
                        else arm_result.get("reason")
                    )
                elif not current_drive_arm and _arm_target_mailbox is not None:
                    _arm_target_mailbox.release(owner)
                if mapper.state != "following":
                    if ik_scheduler is not None:
                        ik_scheduler.release(owner)
                    if _arm_target_mailbox is not None:
                        _arm_target_mailbox.release(owner)
                    if _arm is not None:
                        _arm.end_tracking()

                await websocket.send_json({
                    "ok": True,
                    "frame_id": frame_id,
                    "joint_angles": joint_angles,
                    "hand_joint_targets": raw_hand_target,
                    "arm_joint_targets": last_arm_targets,
                    "gesture": gesture,
                    "hardware": hardware,
                    "arm": arm_result,
                    "orientation_delta_deg": arm_result.get("orientation_delta_deg"),
                    "orientation_limited_axes": arm_result.get("orientation_limited_axes"),
                    "wrist_pose": (
                        wrist_observation.protocol_pose() if wrist_observation else None
                    ),
                    "tracking_control_applied": tracking_control_applied,
                    "wrist_position_filter": last_position_filter,
                    "wrist_orientation_filter": last_orientation_filter,
                    **mapper.status(),
                })

            except Exception as e:
                wrist_position_filter.reset()
                wrist_orientation_filter.reset()
                if mapper.state in ("anchoring", "following"):
                    mapper.freeze(f"tracking_error:{type(e).__name__}")
                if ik_scheduler is not None:
                    ik_scheduler.release(owner)
                if _arm_target_mailbox is not None:
                    _arm_target_mailbox.release(owner)
                if _arm is not None:
                    _arm.end_tracking()
                print(f"[ws] 合体跟随失败: {e}")
                await websocket.send_json({
                    "ok": False,
                    "frame_id": frame_id,
                    "msg": f"跟随处理失败: {str(e)}",
                    "tracking_control_applied": False,
                    **mapper.status(),
                })

    except WebSocketDisconnect:
        print("[ws] 客户端已断开")
    except Exception as e:
        print(f"[ws] WebSocket 错误: {e}")
    finally:
        session_active = False
        if _hand_target_mailbox is not None:
            _hand_target_mailbox.release(owner)
        if ik_scheduler is not None:
            ik_scheduler.release(owner)
        if _arm_target_mailbox is not None:
            _arm_target_mailbox.release(owner)
        if _arm is not None:
            _arm.end_tracking()
        if ik_scheduler is not None:
            await ik_scheduler.close()
        if ik_client is not None:
            await asyncio.get_event_loop().run_in_executor(None, ik_client.close)



def _estimate_hand_frame(kp_array: "np.ndarray") -> "np.ndarray":
    """从 MediaPipe 关键点估计手腕局部坐标系（消除手腕旋转影响）

    参数:
        kp_array: (21, 3) 已归零到手腕的关键点

    返回:
        (3, 3) 旋转矩阵，将 MediaPipe 坐标对齐到手腕局部坐标系
    """
    import numpy as np

    # 使用手腕(0)、食指根(5)、中指根(9) 三点拟合坐标系
    points = kp_array[[0, 5, 9], :]

    # x 轴：从小指侧指向拇指侧（手腕 → 小指根的反向）
    x_vector = points[0] - points[2]

    # 用 SVD 拟合平面法向量
    centered = points - np.mean(points, axis=0, keepdims=True)
    u, s, v = np.linalg.svd(centered)
    normal = v[2, :]

    # Gram-Schmidt 正交化
    x = x_vector - np.sum(x_vector * normal) * normal
    x = x / np.linalg.norm(x)
    z = np.cross(x, normal)

    # 确保 z 轴从小指指向食指
    if np.sum(z * (points[1] - points[2])) < 0:
        normal *= -1
        z *= -1

    # 列向量为坐标轴
    frame = np.stack([x, normal, z], axis=1)
    return frame


async def _mediapipe_to_joint_angles(landmarks: list[dict]) -> dict:
    """MediaPipe 21点 → Inspire Hand 6关节角度（通过 dex_retargeting）

    返回: {
        "right_thumb_1_joint": 0.0,
        "right_thumb_2_joint": 0.0,
        "right_index_1_joint": 0.0,
        "right_middle_1_joint": 0.0,
        "right_ring_1_joint": 0.0,
        "right_little_1_joint": 0.0
    }
    """
    import time
    t0 = time.perf_counter()

    # 懒加载 retargeting（避免启动时加载大库）
    global _RETARGETING_ADAPTER
    if "_RETARGETING_ADAPTER" not in globals():
        try:
            from dex_retargeting.retargeting_config import RetargetingConfig
            config_path = REPO / "configs/inspire_hand_right_local.yml"
            # 配置里的 ../assets 相对于 configs/，不能依赖服务从哪个 cwd 启动。
            RetargetingConfig.set_default_urdf_dir(str(config_path.parent))
            config = RetargetingConfig.load_from_file(str(config_path))
            _RETARGETING_ADAPTER = config.build()  # 构建 SeqRetargeting 对象
            print(f"[retargeting] 已加载配置: {config_path}")
        except Exception as e:
            print(f"[retargeting] 加载失败: {e}")
            raise RuntimeError(f"Retargeting 初始化失败: {e}")

    # 转换 MediaPipe landmarks 为 numpy 数组 (21, 3)
    import numpy as np
    kp_array = np.array([[pt["x"], pt["y"], pt["z"]] for pt in landmarks], dtype=np.float32)

    # ========== 坐标预处理（与 SingleHandDetector 一致）==========
    # 1. 归零：所有点相对于手腕（第0点）
    kp_array = kp_array - kp_array[0:1, :]

    # 2. 估计手腕局部坐标系（消除手腕旋转影响）
    wrist_rot = _estimate_hand_frame(kp_array)

    # 3. 坐标系变换：MediaPipe → MANO
    OPERATOR2MANO_RIGHT = np.array([
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ], dtype=np.float32)
    joint_pos = kp_array @ wrist_rot @ OPERATOR2MANO_RIGHT
    # ===========================================================

    # 根据 retargeting 类型准备输入
    retargeting_type = _RETARGETING_ADAPTER.optimizer.retargeting_type
    indices = _RETARGETING_ADAPTER.optimizer.target_link_human_indices

    if retargeting_type == "POSITION":
        ref_value = joint_pos[indices, :]
    else:  # VECTOR
        origin_indices = indices[0, :]
        task_indices = indices[1, :]
        ref_value = joint_pos[task_indices, :] - joint_pos[origin_indices, :]

    t1 = time.perf_counter()

    # 调用 retargeting（返回 numpy 数组）
    qpos = _RETARGETING_ADAPTER.retarget(ref_value)

    t2 = time.perf_counter()

    # 获取关节名称并构建字典
    joint_names = _RETARGETING_ADAPTER.optimizer.robot.dof_joint_names
    result = {name: float(qpos[i]) for i, name in enumerate(joint_names)}

    t_total = (t2 - t0) * 1000
    t_preprocess = (t1 - t0) * 1000
    t_retarget = (t2 - t1) * 1000
    print(f"[perf] 总计 {t_total:.1f}ms (预处理 {t_preprocess:.1f}ms + retarget {t_retarget:.1f}ms)")

    return result


def _recognize_mediapipe_gesture(landmarks: list[dict]) -> str | None:
    """简化版手势识别：MediaPipe 21点 → 预定义手势名"""
    if len(landmarks) != 21:
        return None

    try:
        # 提取关键点
        wrist = landmarks[0]
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]
        middle_tip = landmarks[12]
        ring_tip = landmarks[16]
        pinky_tip = landmarks[20]

        # 计算展开度
        def dist(p1, p2):
            return ((p1.get('x', 0) - p2.get('x', 0))**2 +
                   (p1.get('y', 0) - p2.get('y', 0))**2 +
                   (p1.get('z', 0) - p2.get('z', 0))**2) ** 0.5

        middle_mcp = landmarks[9]
        hand_size = dist(middle_mcp, wrist)
        if hand_size < 0.01:
            return None

        thumb_norm = dist(thumb_tip, wrist) / hand_size
        index_norm = dist(index_tip, wrist) / hand_size
        middle_norm = dist(middle_tip, wrist) / hand_size
        ring_norm = dist(ring_tip, wrist) / hand_size
        pinky_norm = dist(pinky_tip, wrist) / hand_size

        # 手势识别逻辑（映射到已有的技能包名称）
        if all(d > 1.5 for d in [thumb_norm, index_norm, middle_norm, ring_norm, pinky_norm]):
            return "张开手"  # 需要对应技能包存在

        if thumb_norm > 1.3 and all(d < 1.2 for d in [index_norm, middle_norm, ring_norm, pinky_norm]):
            return "点赞"

        if index_norm > 1.5 and all(d < 1.2 for d in [thumb_norm, middle_norm, ring_norm, pinky_norm]):
            return "食指指向"

        if all(d < 1.2 for d in [thumb_norm, index_norm, middle_norm, ring_norm, pinky_norm]):
            return "握拳"

        return None

    except (KeyError, TypeError, ZeroDivisionError):
        return None


@app.post("/api/hand/gesture/play")
async def gesture_play(payload: dict,
                       lease_id: str | None = Header(default=None,
                                                      alias="X-Hardware-Lease")) -> JSONResponse:
    """回放技能包。{"path":"..."} 或 {"name":"OK手势"},可选 "return_home"。

    按名回放是留给语音/VLA 的入口。**重名不猜** —— 返回 409 + 候选路径列表,
    让调用方拿 path 再来一次。猜一个的话,同名两个包哪个被执行取决于文件系统
    遍历顺序,那是不可预期的机械动作。
    """
    _, lease_error = _require_lease(lease_id, payload)
    if lease_error:
        return lease_error
    import gesture_pack as gp
    rel = str(payload.get("path") or "").strip()
    if not rel:
        name = str(payload.get("name") or "").strip()
        if not name:
            return JSONResponse({"ok": False, "msg": "要给 path 或 name"},
                                status_code=400)
        hits = gp.find_by_name(name)
        if not hits:
            return JSONResponse({"ok": False, "msg": f"没有名为『{name}』的技能包"},
                                status_code=404)
        if len(hits) > 1:
            return JSONResponse(
                {"ok": False, "msg": f"『{name}』有 {len(hits)} 个同名包,请指定 path",
                 "candidates": [h["path"] for h in hits]}, status_code=409)
        rel = hits[0]["path"]
    try:
        pack = gp.load_pack(rel)
    except gp.GestureError as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=400)
    except OSError as e:
        return JSONResponse({"ok": False, "msg": f"读文件失败: {e}"}, status_code=500)

    # 会话检查放在**读包之后**:包本身不合法时先报格式错误更有用 —— 否则用户会
    # 以为"接入了就能播",接了之后才发现包是坏的。
    hand = _get_hand()
    if hand.console is None or hand.console.poll() is not None:
        return JSONResponse({"ok": False, "msg": "未接入灵巧手,先点『接入灵巧手』"},
                            status_code=409)
    # ⚠ HandDebugSession.command() 是**投递即返回**,不等 console 的 ack —— ack 是
    # 异步走 /ws/hand 回来的。所以这里不能指望从返回值里读到播放结果,只能确认
    # "写进 stdin 了"。名称/帧数一律用我们刚加载的 pack 现算,别去读 ack。
    home = payload.get("return_home")
    sent = hand.command({"cmd": "gesture_play", "pack": pack.to_dict(),
                         "return_home": home})
    if not sent.get("ok"):
        return JSONResponse({"ok": False, "msg": sent.get("msg", "投递失败")},
                            status_code=503)
    home_first = pack.return_home_first if home is None else bool(home)
    return JSONResponse({"ok": True, "path": rel, "name": pack.name,
                         "frames": len(pack.frames),
                         "steps": len(pack.frames) + (1 if home_first else 0),
                         "return_home": home_first,
                         "playback_mode": pack.playback_mode,
                         "duration_ms": pack.duration_ms})


@app.websocket("/ws/hand")
async def hand_telemetry(ws: WebSocket) -> None:
    """手部调试页的遥测流:实时角度/温度/力/电流等。"""
    await ws.accept()
    hand = _get_hand()
    q: asyncio.Queue = asyncio.Queue(maxsize=4)
    hand.clients.add(q)
    try:
        if hand.latest:
            await ws.send_json(hand.latest)               # 立刻给一帧,不用等下一个周期
        while True:
            row = await q.get()
            await ws.send_json(row)
    except WebSocketDisconnect:
        pass
    except Exception:                                     # noqa: BLE001
        pass
    finally:
        hand.clients.discard(q)


# ---------------------------------------------------------------------------
# 机械臂调试 —— 端点和手那批**同构**,合体页只需并行调两套
# ---------------------------------------------------------------------------
@app.post("/api/arm/start")
async def arm_start(mock: bool = True, speed: int = 20,
                    lease_id: str | None = Header(default=None,
                                                   alias="X-Hardware-Lease")) -> JSONResponse:
    """接入机械臂:打开 can0。3D 由浏览器端 three.js 负责。

    ⚠ **默认 mock=True**(和手页相反)。臂是 7 自由度工业臂,伤害量级不同,
    要接真机得显式传 mock=false。ok 反映的是 CAN 真的通了且读到关节角。
    """
    owner, error = _require_lease(lease_id, acquire=True)
    if error:
        return error
    arm = _get_arm()
    arm.start(mock=mock, speed=speed)
    await asyncio.get_event_loop().run_in_executor(_executor, arm.wait_ready)
    if arm.error:
        arm.stop()
        _release_lease_if_hardware_idle(owner)
        return JSONResponse({"ok": False, "msg": arm.error, "mock": mock},
                            status_code=503)
    return JSONResponse({"ok": True, "mock": arm.mock, "channel": arm.channel,
                         "limits": arm.limits, "connect_pose": arm.connect_pose,
                         "enabled": (arm.latest or {}).get("enabled", False)})


async def _stop_arm_session(*, home: bool) -> dict:
    """Optionally return to zero, then always release the CAN session."""
    arm = _arm
    if arm is None:
        return {
            "ok": True, "online": False, "home_requested": home,
            "home_ok": not home, "released": True,
        }
    alive = bool(arm.ready and arm.console and arm.console.poll() is None)
    result = {
        "ok": True,
        "online": alive,
        "home_requested": home,
        "home_ok": not home,
    }
    try:
        if home and alive:
            latest = arm.latest or {}
            if not latest.get("enabled"):
                result["home_reason"] = "disabled"
            elif latest.get("frozen"):
                result["home_reason"] = "frozen"
            else:
                if _arm_target_mailbox is not None:
                    _arm_target_mailbox.reset()
                    drain_deadline = time.monotonic() + 0.5
                    while (_arm_target_mailbox.in_flight_count
                           and time.monotonic() < drain_deadline):
                        await asyncio.sleep(0.01)
                arm.end_tracking()
                sent = arm.command({"cmd": "home"})
                if sent.get("ok"):
                    deadline = time.monotonic() + 15.0
                    while time.monotonic() < deadline:
                        current = (arm.latest or {}).get("rad")
                        if (isinstance(current, list) and len(current) == 7
                                and max(abs(float(value) - target) for value, target
                                        in zip(current, NERO_HOME_POSE)) <= 0.03):
                            result["home_ok"] = True
                            break
                        if not (arm.console and arm.console.poll() is None):
                            result["home_reason"] = "disconnected"
                            break
                        await asyncio.sleep(0.1)
                    else:
                        result["home_reason"] = "timeout"
                else:
                    result["home_reason"] = sent.get("msg") or "command_failed"
    finally:
        await asyncio.get_event_loop().run_in_executor(_executor, arm.stop)
        result["released"] = True
    return result


@app.post("/api/arm/stop")
async def arm_stop(home: bool = False,
                   lease_id: str | None = Header(default=None,
                                                  alias="X-Hardware-Lease")) -> JSONResponse:
    """断开 CAN；页面离开时可先回全零位，手动断开默认维持原语义。

    原样把臂交回给原来的控制方(常态是松灵客户端)。想回接入位姿走
    {cmd:"goto_connect_pose"},那是显式动作。理由见 ARM_DEBUG.md。
    """
    _, error = _require_lease(lease_id)
    if error:
        return error
    return JSONResponse(await _stop_arm_session(home=home))


@app.post("/api/hardware/release")
async def hardware_release(payload: dict | None = None,
                           lease_id: str | None = Header(default=None,
                                                          alias="X-Hardware-Lease")) -> JSONResponse:
    """页面关闭兜底：手张开、臂回零，随后释放串口和 CAN。

    回位条件不满足或超时也必须继续断开，避免浏览器退出后长期占用设备通道。
    """
    owner = _request_owner(lease_id, payload)
    release_result = _hardware_lease.release(owner)
    if not release_result.ok:
        return _lease_response(release_result)
    return JSONResponse(await _hardware_release_impl(home=True))


@app.get("/api/arm/status")
async def arm_status() -> JSONResponse:
    a = _arm
    alive = a is not None and a.console is not None and a.console.poll() is None
    latest = (a.latest or {}) if a else {}
    return JSONResponse({
        "online": bool(alive and a.ready and not a.error),
        "mock": bool(a.mock) if a else True,
        "channel": a.channel if a else None,
        "error": a.error if a else None,
        "limits": a.limits if a else None,
        "enabled": latest.get("enabled", False),
        "frozen": latest.get("frozen", False),
        "speed_percent": latest.get("speed_percent"),
        "rad": latest.get("rad"),
        "target": latest.get("target"),
        "home_pose": list(NERO_HOME_POSE),
        "tracking_ready_pose": list(NERO_TRACKING_READY_POSE),
        "connect_pose": a.connect_pose if a else None,
        "pose_drift": latest.get("pose_drift"),
    })


@app.post("/api/arm/command")
async def arm_command(payload: dict,
                      lease_id: str | None = Header(default=None,
                                                     alias="X-Hardware-Lease")) -> JSONResponse:
    """臂控制指令。协议见 arm_console.py 的 handle():
    {cmd:"angles",rad:[7]} | {cmd:"goto_tracking_ready"} | {cmd:"enable"}
    | {cmd:"disable"} | {cmd:"home"} | {cmd:"speed",value:20}
    | {cmd:"estop"} | {cmd:"reset"}

    运动类指令在**这一层**也先查一遍使能/急停状态。理由:command() 的 ok 只代表
    "写进了 console 的 stdin",真正的拒绝是异步经 /ws/arm 回来的 error 帧,HTTP
    响应反映不出来 —— 前端若信那个 ok=true 会显示"下发成功"而实际被拒。
    console 侧的检查保留(纵深防御),它才是最终把关的那道。
    """
    _, error = _require_lease(lease_id, payload)
    if error:
        return error
    arm = _get_arm()
    if (payload or {}).get("cmd") in (
        "angles", "home", "goto_connect_pose", "goto_tracking_ready"
    ):
        st = arm.latest or {}
        if not st.get("enabled"):
            return JSONResponse({"ok": False, "msg": "臂未使能,先点『使能』"},
                                status_code=409)
        if st.get("frozen"):
            return JSONResponse({"ok": False, "msg": "急停生效中,先点『复位』解除"},
                                status_code=409)
    return JSONResponse(arm.command(payload))


@app.post("/api/arm/camera_pose")
async def arm_camera_pose(payload: dict,
                          lease_id: str | None = Header(default=None,
                                                         alias="X-Hardware-Lease")) -> JSONResponse:
    """切换摄像头会话的机械臂准备位或伸直位。

    先清空 latest-target 并退出 CPV，确保旧跟随目标不会覆盖本次姿态切换。
    """
    _, error = _require_lease(lease_id, payload)
    if error:
        return error
    pose = str((payload or {}).get("pose") or "")
    commands = {
        "tracking_ready": ("goto_tracking_ready", NERO_TRACKING_READY_POSE),
        "home": ("home", NERO_HOME_POSE),
    }
    if pose not in commands:
        return JSONResponse(
            {"ok": False, "msg": "pose 只支持 tracking_ready 或 home"},
            status_code=400,
        )

    arm = _get_arm()
    alive = bool(arm.ready and arm.console and arm.console.poll() is None)
    if not alive:
        return JSONResponse({"ok": False, "msg": "机械臂未在线"}, status_code=409)
    latest = arm.latest or {}
    if not latest.get("enabled"):
        return JSONResponse({"ok": False, "msg": "臂未使能,先点『使能』"},
                            status_code=409)
    if latest.get("frozen"):
        return JSONResponse({"ok": False, "msg": "急停生效中,先点『复位』解除"},
                            status_code=409)

    if _arm_target_mailbox is not None:
        _arm_target_mailbox.reset()
    arm.end_tracking()
    command, target = commands[pose]
    result = arm.command({"cmd": command})
    if not result.get("ok"):
        return JSONResponse(result, status_code=503)
    return JSONResponse({
        "ok": True,
        "pose": pose,
        "target": list(target),
        "mock": bool(arm.mock),
    })


@app.websocket("/ws/arm")
async def arm_telemetry(ws: WebSocket) -> None:
    """臂调试页的遥测流:关节角/电流/力矩/使能状态。"""
    await ws.accept()
    arm = _get_arm()
    q: asyncio.Queue = asyncio.Queue(maxsize=4)
    arm.clients.add(q)
    try:
        if arm.latest:
            await ws.send_json(arm.latest)
        while True:
            row = await q.get()
            await ws.send_json(row)
    except WebSocketDisconnect:
        pass
    except Exception:                                     # noqa: BLE001
        pass
    finally:
        arm.clients.discard(q)


def _shutdown_live() -> None:
    if _live is not None:
        _live.stop()


def _shutdown_hand() -> None:
    if _hand is not None:
        _hand.stop()


def _shutdown_arm() -> None:
    if _arm is not None:
        _arm.stop()


atexit.register(_shutdown_live)
atexit.register(_shutdown_hand)
atexit.register(_shutdown_arm)


def main() -> None:
    ip = _primary_ip()

    # 检查是否有 SSL 证书（支持 HTTPS 以启用摄像头访问）
    ssl_keyfile = REPO / "ssl/key.pem"
    ssl_certfile = REPO / "ssl/cert.pem"
    use_ssl = ssl_keyfile.exists() and ssl_certfile.exists()

    protocol = "https" if use_ssl else "http"
    print("=" * 72, flush=True)
    print(f"  回放工作台启动中。浏览器打开:  {protocol}://{ip}:{WEB_PORT}", flush=True)
    if use_ssl:
        print(f"  ✅ HTTPS 已启用（支持摄像头访问）", flush=True)
        print(f"  ⚠️  首次访问需在浏览器中信任自签名证书", flush=True)
    else:
        print(f"  ⚠️  HTTP 模式（摄像头仅在 localhost 可用）", flush=True)
    print("  (若 localhost 打不开就用上面这个 IP 地址)", flush=True)
    print("=" * 72, flush=True)

    if use_ssl:
        uvicorn.run(
            app,
            host="0.0.0.0",
            port=WEB_PORT,
            log_level="warning",
            ssl_keyfile=str(ssl_keyfile),
            ssl_certfile=str(ssl_certfile)
        )
    else:
        uvicorn.run(app, host="0.0.0.0", port=WEB_PORT, log_level="warning")


if __name__ == "__main__":
    main()
