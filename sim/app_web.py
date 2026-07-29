#!/usr/bin/env python3
"""app_web.py — NERO·Inspire 回放工作台 Web 前端(FastAPI + SSE)。

布局仿 1.html 的全屏悬浮结构,配色按『可视化工作台-提示词.md』的白色极简科技风。
后端管线复用 app_gradio.py:subprocess 依次跑 build_canonical / derive_embodiment /
replay_rerun --serve,用 SSE 把进度/日志/Rerun 地址推给浏览器。本 app 只做编排,
自己不 import lerobot / rerun / pinocchio(它们装在 lerobot conda 环境,靠 subprocess 调)。

运行(在 gradio venv 里,该 venv 已装 fastapi/uvicorn):
    ~/gradio_venv/bin/python sim/app_web.py
然后 Windows 浏览器打开启动时打印的 http://<WSL_IP>:7860
"""
from __future__ import annotations

import asyncio
import atexit
import concurrent.futures
import json
import os
import re
import shutil
import socket
import subprocess
import tempfile
import time
from pathlib import Path

import threading

import uvicorn
from fastapi import FastAPI, File, Form, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse

# --- 路径 / 解释器(可用环境变量覆盖)---
REPO = Path(os.environ.get("LEROBOT_REPO", "/home/zhang123/ros2_ws/lerobotTest"))
LEROBOT_PY = os.environ.get("LEROBOT_PY",
                            "/home/zhang123/ros2_ws/enter/envs/lerobot/bin/python3")
WEB_PORT = int(os.environ.get("WEB_PORT", "7860"))
WEB_DIR = Path(__file__).resolve().parent / "web"
SIM = Path(__file__).resolve().parent
# ROS2 侧脚本要先 source humble + 工作区(system python3);conda 侧脚本用 LEROBOT_PY。
ROS_SETUP = os.environ.get(
    "ROS_SETUP",
    "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash")
ROS_PYTHON = os.environ.get(
    "ROS_PYTHON",
    "/home/zhang123/ros2_ws/enter/envs/lerobot/bin/python3",
)


def _ros_cmd(script_argv: list[str]) -> list[str]:
    """把 ROS2 python 脚本包进 bash -lc 'source ...; exec python3 ...',保证 rclpy 可见。"""
    ros_log_dir = Path(os.environ.get("ROS_LOG_DIR", "/home/zhang123/ros2_ws/.ros_log"))
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    inner = f"export ROS_LOG_DIR={ros_log_dir} && {ROS_SETUP} && exec {ROS_PYTHON} " + " ".join(script_argv)
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
RGBD_INPUT_ROOT = os.environ.get("RGBD_INPUT_ROOT", "kinect2_middle/kinect2_middle")
RGBD_CAMERA = os.environ.get("RGBD_CAMERA", "kinect2_middle")


def _traj_pkl(robot: str) -> Path:
    """derive_embodiment --emit-traj 产出的 pkl(按规格名);replay/play 都据此定位。"""
    return REPO / f"sim/out/robot_traj_{robot}.pkl"


# /api/replay/play 用的 npz,随最近一次成功管线的机器人规格更新。
TRAJ_NPZ = _traj_pkl("nero_inspire_rgb").with_suffix(".npz")


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


def _free_ports(log: list[str] | None = None) -> None:
    """强制释放 Rerun 端口 —— 干掉任何还占着 grpc/web 端口的残留 replay 进程
    (包括 app_gradio.py 或上次崩溃留下的孤儿),否则新 serve 会因端口占用而启动失败。"""
    for port in (GRPC_PORT, WEB_VIEWER_PORT):
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
    cmd = [LEROBOT_PY, "sim/replay_rerun.py", "--serve",
           "--grpc-port", str(GRPC_PORT), "--web-port", str(WEB_VIEWER_PORT),
           "--robot", robot,
           "--traj", f"{robot}={_traj_pkl(robot)}"]
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
    global TRAJ_NPZ
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
    if not skip_regen:
        if is_rgbd:
            cmd = [LEROBOT_PY, "sim/build_canonical_from_rgbd.py",
                   "--input-root", rgbd_root, "--camera", rgbd_cam]
            caption = f"① 规范层 · {ds['label']} 深度反投手部 · {rgbd_root}"
        elif source == "handfile":
            cmd = [LEROBOT_PY, "sim/build_canonical_from_processed.py", "--input", str(input_path)]
            caption = "① 规范层 · 导入外部手部结果"
        else:
            cmd = [LEROBOT_PY, "sim/build_canonical.py", "--video", str(input_path)]
            caption = "① 规范层 · 检测人手关键点"
        if not _run_step(cmd, log, caption, 6, 42, emit):
            emit({"type": "error", "msg": "① 规范层生成失败 · 看日志"})
            return
        if not _run_step([LEROBOT_PY, "sim/derive_embodiment.py", "--robot", robot, "--emit-traj"],
                         log, f"② 本体层 · {ds['label']} 逐帧逆解 IK", 45, 80, emit):
            emit({"type": "error", "msg": "② 本体层生成失败 · 看日志"})
            return
    TRAJ_NPZ = _traj_pkl(robot).with_suffix(".npz")   # 供 /api/replay/play 定位
    emit({"type": "progress", "pct": 84, "msg": "③ 启动 Rerun 服务"})
    url = _start_replay(str(input_path) if input_path else None, log,
                        robot=robot, no_video=(is_rgbd or source == "handfile"))
    if not url:
        emit({"type": "error", "msg": "③ 未获取到 Rerun 地址 · 看日志"})
        return
    emit({"type": "rerun_url", "url": url})
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

    # ---- 生命周期 ----
    def start(self) -> None:
        if self._running:
            return
        _free_ports()                                     # 清 Rerun 端口
        self._running = True
        # live_rerun:conda python,serve 3D;stdin 收关节流
        # --view-hz:3D 抽帧上限(100Hz 数据也只按此刷);--mem-limit:Rerun 滑动窗口,防内存无界增长卡死
        self.live = subprocess.Popen(
            [LEROBOT_PY, "sim/live_rerun.py", "--serve",
             "--grpc-port", str(GRPC_PORT), "--web-port", str(WEB_VIEWER_PORT),
             "--view-hz", "30", "--mem-limit", "500MB"],
            cwd=str(REPO), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1)
        # reader:ROS2 python,订阅 /joint_states → stdout
        self.reader = subprocess.Popen(
            _ros_cmd(["sim/ros_joint_reader.py"]),
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
            _ros_cmd(["sim/ros_joint_writer.py"]),
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
            return {"ok": True, "sent": cmd}
        except Exception as e:                            # noqa: BLE001
            return {"ok": False, "msg": str(e)}


_live: LiveSession | None = None


def _get_live() -> LiveSession:
    global _live
    if _live is None:
        _live = LiveSession(asyncio.get_event_loop())
    return _live


# ---------------------------------------------------------------------------
# FastAPI
# ---------------------------------------------------------------------------
app = FastAPI(title="NERO·Inspire 回放工作台")


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


@app.get("/api/replay/play")
async def replay_play(speed: float = 1.0, fps: float = 30.0) -> StreamingResponse:
    """SSE:spawn traj_player 逐帧下发 npz 轨迹给 writer→bridge,进度推浏览器。
    需先开『实时 Live』(bridge 在跑、才有下发对象;Live 3D 显示真机回读)。"""
    async def stream():
        global _player_proc
        _stop_player()                                    # 单实例:先停旧的
        if not TRAJ_NPZ.exists():
            yield f"data: {json.dumps({'type':'error','msg':'无轨迹,请先在回放模式跑一遍视频管线'}, ensure_ascii=False)}\n\n"
            return
        sp = max(0.25, min(4.0, speed))
        _player_proc = subprocess.Popen(
            _ros_cmd(["sim/traj_player.py", "--npz", str(TRAJ_NPZ),
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


def _shutdown_live() -> None:
    if _live is not None:
        _live.stop()


atexit.register(_shutdown_live)


def main() -> None:
    ip = _primary_ip()
    print("=" * 72, flush=True)
    print(f"  回放工作台启动中。Windows 浏览器打开:  http://{ip}:{WEB_PORT}", flush=True)
    print("  (若 localhost 打不开就用上面这个 WSL IP 地址)", flush=True)
    print("=" * 72, flush=True)
    uvicorn.run(app, host="0.0.0.0", port=WEB_PORT, log_level="warning")


if __name__ == "__main__":
    main()
