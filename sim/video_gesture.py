#!/usr/bin/env python3
"""sim/video_gesture.py — 视频 → MediaPipe 手部关键点 → 重定向 → 6 个驱动关节角。

给「灵巧手调试」页的「视频」栏用:把一段人手视频解成逐帧关节角,挑几帧存成手势
技能包(gesture_pack.py)。这样录手势不用手拖滑块,做一遍动作就行。

复用已有的三块,不重写:
  · single_hand_detector.SingleHandDetector   MediaPipe 21 关键点
  · hand_estimators.MediaPipeHandEstimator    归一化成 HandObservation
  · dex_retargeting + configs/inspire_hand_right_local.yml   关键点 → 关节角

⚠ 重定向输出的是 **12 个** dof(含耦合的 intermediate/distal),我们只要 6 个驱动关节
  (HAND_JOINTS)。按**名字**取,不按下标 —— dof 顺序由 URDF 决定,写死下标哪天
  URDF 改了会静默错位(角度装到别的手指上)。

⚠ 这个模块**只读视频、不碰硬件**。解出来的角度要下发得走 gesture_pack 那条路,
  由用户显式点回放。
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from inspire_hand import HAND_JOINTS, HAND_LIMITS
from paths import DATA, REPO

# 一次最多解多少帧。
# 原来是 600,配 stride=3 只够 20s 素材;但要**连贯**回放得用 stride=1,那 600 帧
# 只有 20s 且一动作一帧都不能少。放到 2000:30fps 下 stride=1 能覆盖 66s。
# 代价是解析时间(约 50ms/帧,2000 帧 100s),所以前端默认值仍取小的,由用户按需调大。
MAX_EXTRACT_FRAMES = 2000
DEFAULT_STRIDE = 3                  # 隔几帧取一帧;30fps 下 stride=3 约等于 10Hz
VIDEO_SUFFIXES = (".mp4", ".mov", ".avi", ".mkv", ".webm")


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# 视频清单
# ---------------------------------------------------------------------------
def list_videos() -> list[dict]:
    """data/ 下的视频 + 上次上传的临时文件。给前端下拉框用。"""
    out = []
    seen = set()
    for d in (DATA, Path("/tmp")):
        if not d.is_dir():
            continue
        pat = "nero_web_*" if d == Path("/tmp") else "*"
        try:
            for p in sorted(d.glob(pat)):
                if not p.is_file() or p.suffix.lower() not in VIDEO_SUFFIXES:
                    continue
                rp = str(p.resolve())
                if rp in seen:
                    continue
                seen.add(rp)
                out.append({"path": rp, "name": p.name,
                            "size_mb": round(p.stat().st_size / 1e6, 1),
                            "where": "data" if d == DATA else "upload"})
        except OSError:
            continue
    return out


# ---------------------------------------------------------------------------
# 重定向器(懒加载 + 复用)
# ---------------------------------------------------------------------------
_rt_lock = threading.Lock()
_rt_cache = None


def _get_retargeter():
    """构造重定向器。**低通关掉**(low_pass_alpha=1.0)。

    为什么关:低通是给实时遥操作用的(压手抖),但它引入延迟 —— 逐帧离线解析时,
    每帧的输出会拖上前几帧的残影,挑出来的关键帧就不是那一瞬间的真实姿态。
    平滑要做的话在**取完之后**对整条曲线做(零相位),不要在这里做单向滤波。
    """
    global _rt_cache
    with _rt_lock:
        if _rt_cache is not None:
            return _rt_cache
        from dex_retargeting.retargeting_config import RetargetingConfig
        RetargetingConfig.set_default_urdf_dir(str(REPO / "assets"))
        rt = RetargetingConfig.load_from_file(
            str(REPO / "configs/inspire_hand_right_local.yml"),
            override={"low_pass_alpha": 1.0}).build()
        names = list(rt.optimizer.robot.dof_joint_names)
        # 按**名字**取 6 个驱动关节的下标。不写死数字:dof 顺序由 URDF 决定,
        # URDF 一改就会静默错位 —— 角度装到别的手指上,看起来像"重定向不准"。
        missing = [n for n in HAND_JOINTS if n not in names]
        if missing:
            raise RuntimeError(f"重定向输出里缺关节 {missing};实际有 {names}")
        idx6 = [names.index(n) for n in HAND_JOINTS]
        _rt_cache = (rt, names, idx6)
        return _rt_cache


@dataclass
class ExtractJob:
    """一次抽取任务的状态。前端轮询 to_dict() 看进度。"""
    video: str = ""
    hand_type: str = "Right"
    stride: int = DEFAULT_STRIDE
    total: int = 0                 # 计划解多少帧
    done: int = 0                  # 已解多少帧
    detected: int = 0              # 其中检出手的帧数
    fps: float = 0.0
    running: bool = False
    error: str = ""
    frames: list = field(default_factory=list)
    started_at: float = 0.0
    finished_at: float = 0.0
    cancel: bool = False
    despiked: int = 0              # 被中值修掉的帧数
    quality: dict = field(default_factory=dict)

    def to_dict(self, *, with_frames: bool = False) -> dict:
        d = {"video": self.video, "name": Path(self.video).name if self.video else "",
             "hand_type": self.hand_type, "stride": self.stride,
             "total": self.total, "done": self.done, "detected": self.detected,
             "fps": round(self.fps, 2), "running": self.running,
             "error": self.error, "n_frames": len(self.frames),
             "despiked": self.despiked, "quality": self.quality,
             "elapsed": round((self.finished_at or time.monotonic())
                              - self.started_at, 1) if self.started_at else 0.0}
        if with_frames:
            d["frames"] = self.frames
        return d


_job = ExtractJob()
_job_lock = threading.Lock()


def current_job() -> ExtractJob:
    return _job


def cancel_job() -> None:
    with _job_lock:
        _job.cancel = True


def claim(video: str, *, stride: int, hand_type: str) -> bool:
    """原子地占下任务位。已有任务在跑就返回 False。

    ⚠ 必须**同步**占位,不能等 extract() 在工作线程里把 running 置 True ——
    run_in_executor 只是把活排进队列就返回,端点里那句 `if job.running` 在线程
    真正开跑之前是 False。两个请求前后脚进来时都能过检查,然后**互相覆盖对方的
    _job 状态**:进度条跳来跳去,frames 混着两段视频的结果。
    实测过:并发发两次,第二次返回 200 而不是 409,total 变成第二个视频的。
    """
    with _job_lock:
        if _job.running:
            return False
        _job.running = True          # 就地占位,后面 extract() 会把其余字段填全
        _job.video = video
        _job.stride = max(1, int(stride))
        _job.hand_type = hand_type
        _job.total = 0
        _job.done = 0
        _job.detected = 0
        _job.error = ""
        _job.frames = []
        _job.despiked = 0
        _job.quality = {}
        _job.started_at = time.monotonic()
        _job.finished_at = 0.0
        _job.cancel = False
        return True


def release(error: str = "") -> None:
    """占位失败/异常时把任务位放掉,否则会永久卡在 running。"""
    with _job_lock:
        _job.running = False
        _job.finished_at = time.monotonic()
        if error:
            _job.error = error


def retarget_one(obs_kp3d: np.ndarray) -> list[float]:
    """(21,3) MANO 关键点 → 6 个驱动关节角(rad,项目序),按 URDF 限位夹取。

    向量的构造方式必须和配置里的 target_link_human_indices 对齐:
    row0 是每根手指的**原点**关键点(拇指用手腕 0,四指用各自 MCP),row1 是指尖。
    照抄 derive_embodiment.retarget_hand 的写法 —— 那条路已经跑通过数据集。
    """
    rt, _names, idx6 = _get_retargeter()
    idx = np.asarray(rt.optimizer.target_link_human_indices)
    origin_i, task_i = idx[0, :], idx[1, :]
    ref = obs_kp3d[task_i, :] - obs_kp3d[origin_i, :]
    q = rt.retarget(ref)
    return [float(_clamp(q[i], *HAND_LIMITS[n]))
            for n, i in zip(HAND_JOINTS, idx6)]


def extract(video: str, *, stride: int = DEFAULT_STRIDE, hand_type: str = "Right",
            max_frames: int = MAX_EXTRACT_FRAMES,
            do_despike: bool = True,
            progress=None) -> ExtractJob:
    """解析视频,填 _job。阻塞式 —— 调用方负责扔到线程里。

    检不到手的帧**跳过而不是补零**:补零会在时间轴上插进一个"突然张开"的假姿态,
    挑帧时看不出那是漏检。跳过的话前端能看到帧号不连续,知道那段没检到。

    do_despike=False 保留原始输出,用来对比"清理前 vs 清理后"。默认开 —— 实测
    只改动 0.5%~4.1% 的帧、幅度均 0.26~0.64°,但去掉的正是视觉上"啪"一下的那几帧。
    """
    import cv2

    p = Path(video).expanduser()
    if not p.is_file():
        raise FileNotFoundError(f"视频不存在: {video}")

    # 任务位由调用方先 claim() 占好(见那里的注释:占位必须同步,不能等到这里)。
    # 直接调 extract() 而没走 claim 的场景(比如脚本里单跑)也要能用,所以这里补占。
    with _job_lock:
        if not _job.running:
            _job.running = True
            _job.started_at = _job.started_at or time.monotonic()
        _job.video = str(p.resolve())
        _job.hand_type = hand_type
        _job.stride = max(1, int(stride))

    cap = cv2.VideoCapture(str(p))
    if not cap.isOpened():
        with _job_lock:
            _job.running = False
            _job.error = f"打不开视频(编码不支持?): {p.name}"
        raise RuntimeError(_job.error)
    try:
        fps = float(cap.get(cv2.CAP_PROP_FPS)) or 30.0
        nframe = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        st = max(1, int(stride))
        plan = min(max_frames, (nframe + st - 1) // st) if nframe > 0 else max_frames
        with _job_lock:
            _job.fps = fps
            _job.total = plan

        # 估计器每次任务新建:MediaPipe 的 Hands 是**有状态**的(带跨帧跟踪),
        # 复用上一个视频的实例会把上一段的跟踪状态带进来。
        from hand_estimators import MediaPipeHandEstimator
        est = MediaPipeHandEstimator(hand_type=hand_type)

        i = 0
        while True:
            ok, bgr = cap.read()
            if not ok:
                break
            if i % st:
                i += 1
                continue
            with _job_lock:
                if _job.cancel:
                    break
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            rad = None
            try:
                obs = est.detect(rgb)
                if obs is not None:
                    rad = retarget_one(np.asarray(obs.keypoints_3d, dtype=np.float64))
            except Exception as e:                          # noqa: BLE001
                # 单帧失败不该中断整段抽取 —— 记下来继续。
                with _job_lock:
                    if not _job.error:
                        _job.error = f"第 {i} 帧解析异常(已跳过): {e}"
            with _job_lock:
                _job.done += 1
                if rad is not None and all(math.isfinite(x) for x in rad):
                    _job.detected += 1
                    _job.frames.append({"frame": i, "t": round(i / fps, 3),
                                        "rad": [round(x, 6) for x in rad]})
                if progress:
                    progress(_job.to_dict())
                if _job.done >= plan:
                    break
            i += 1
    finally:
        cap.release()
        with _job_lock:
            # 去尖刺 + 质量报告作为**后处理**:中值要看前后帧,逐帧边解边做拿不到
            # 后一帧。放在这里的另一个好处是下游(挑帧/预览/存包)全部自动拿到
            # 清理过的数据,不用每个消费者各记一遍要不要清理。
            if do_despike and _job.frames:
                _job.frames, _job.despiked = despike(_job.frames, stride=_job.stride)
            if _job.frames:
                # frames 已清理过,所以把真实尖刺数传进去,别让 quality 在干净数据上
                # 重数一遍(会得出个小得多的数,和 despiked 打架)。
                _job.quality = quality(_job.frames, stride=_job.stride,
                                       spikes=_job.despiked if do_despike else None)
            _job.running = False
            _job.finished_at = time.monotonic()
    return _job


# ---------------------------------------------------------------------------
# 逐帧清理 + 质量报告
# ---------------------------------------------------------------------------
# ⚠ 这里**故意不做** derive_embodiment 那样的 SavGol 平滑。三条实测理由:
#
#  1. 没有白噪声可去。welch 谱:>8Hz 的能量只占方差 0.0~2.7%,幅度 0.16~0.50°。
#     关键点差分的 lag-1 自相关是 **+0.10 / +0.05**(白噪声该是 -0.5)—— MediaPipe
#     内部带跨帧跟踪,它的误差本来就是**相关**的,和慢速真实运动分不开。时域滤波
#     去不掉相关噪声,只会抹信号。旁证:残差随窗长单调增长(w5 0.47° → w21 1.61°)
#     而不饱和 —— 真有宽带噪声的话窗长超过相关长度就该饱和了。
#  2. SavGol 对**尖刺无效**。它是最小二乘拟合,一个离群点会被摊成鼓包而不是删掉。
#     合成信号实测:尖刺残留 med0=33.3° vs med3=0.5°。去尖刺全靠中值。
#  3. 加了只赔保真。30fps 下真实手势转换只有几帧(400°/s 的快屈 = 6 帧),窗长一旦
#     同量级就必然抹平它。快屈段 RMSE:med3 单独 0.44 → +w5 1.86 → +w9 3.59。
#
# derive_embodiment 那边该用 SavGol:它吃的是 build_canonical 补齐过的**均匀网格**,
# 且服务于策略训练 —— 一致的平滑本身就是特征。我们要的是逐帧真实姿态,不一样。
#
# 中值窗口取 3 不取 5:5 帧窗会啃掉短促动作。合成的 8 帧点动,med5 只剩 87.4%。
MEDIAN_K = 3
# 人手指屈伸角速度上限的量级。超过它的**持续**段是跟踪失败,不是人做得出的动作。
MAX_JOINT_DEG_S = 600.0
# 「算改动过」的门槛。中值几乎每帧都会挪一点(实测改动幅度均 0.26~0.64°),用
# 1e-9 当门槛会报出"改了 81% 的帧"这种没意义的数 —— 那是"碰过",不是"修好了"。
# 3° 的依据:实测段内单帧偏离邻域中值 p50=0.32° p90=1.32°,而 2.1% 的帧偏离 >5°。
# 3° 正好落在这条尾巴的起点,把"真尖刺"和"正常亚度级摆动"分开。
SPIKE_VISIBLE_DEG = 3.0
_DEG = 57.29577951308232


def _segments(frames: list[dict], stride: int = 1) -> list[tuple[int, int]]:
    """按帧号连续性切段。漏检帧是**跳过**的,所以数组相邻 != 时间相邻。

    ⚠ 滤波绝对不能跨段做。实测 gap 处跳变中位 19.8°,而段内帧间 p90 只有 3.0° ——
    把 333ms 的跨度当成一帧去平滑,等于拿真实运动去污染两侧的姿态。
    """
    if not frames:
        return []
    cut = [0]
    for i in range(1, len(frames)):
        if frames[i]["frame"] - frames[i - 1]["frame"] > stride:
            cut.append(i)
    cut.append(len(frames))
    return list(zip(cut[:-1], cut[1:]))


def _median3(rows: list[list[float]]) -> list[list[float]]:
    """逐关节 3 点中值。纯 Python —— 只有 3 个数,不值得拉 scipy 进来。

    两端保留原值:补零会把边缘拉向 0,补邻值等于把边缘复制一遍。原值最诚实。
    """
    n = len(rows)
    if n < 3:
        return [list(r) for r in rows]
    out = [list(rows[0])]
    for i in range(1, n - 1):
        out.append([sorted(t)[1] for t in zip(rows[i - 1], rows[i], rows[i + 1])])
    out.append(list(rows[-1]))
    return out


def despike(frames: list[dict], *, k: int = MEDIAN_K,
            stride: int = 1) -> tuple[list[dict], int]:
    """3 点中值去单帧离群。按 gap 分段做。返回 (新帧, 被改动的帧数)。

    实测 hand2:段内单帧偏离邻域中值 p50 只有 0.32°(跟踪其实很好),但 2.1% 的帧
    偏离 >5°、最大 21° —— 那几个就是视觉上"啪"一下的来源。中值把它们换成邻域
    中值,对其余 97.9% 的帧几乎无改动(合成信号:平台 RMSE 0.30 vs 未滤波 2.85,
    而快屈段 0.44 = 未滤波的 0.44,一点没赔)。

    ⚠ 返回的 changed 只数**改动 >SPIKE_VISIBLE_DEG** 的帧。用 1e-9 当门槛的话
    中值几乎每帧都会挪一点(还要叠上 round 的舍入),会报出"改了 81% 的帧"——
    那测的是"碰过",不是"修好了",对判断素材质量毫无用处。
    """
    if k < 3 or len(frames) < 3:
        return [dict(f) for f in frames], 0
    out = [dict(f) for f in frames]
    changed = 0
    thr = SPIKE_VISIBLE_DEG / _DEG
    for a, b in _segments(frames, stride):
        if b - a < 3:
            continue
        sm = _median3([f["rad"] for f in frames[a:b]])
        for i, rad in enumerate(sm):
            old = frames[a + i]["rad"]
            if max(abs(x - y) for x, y in zip(rad, old)) > thr:
                changed += 1
            out[a + i]["rad"] = [round(x, 6) for x in rad]
    return out, changed


def quality(frames: list[dict], *, stride: int = 1,
            spikes: int | None = None) -> dict:
    """素材质量报告。**只报告,不修改** —— 这两类问题都不该默默平滑掉。

    · gap:漏检空档。两侧姿态差是**真的**(相机漏了那段,手确实动了),抹平它等于
      编造中间过程。回放时会是一次快速切换,这是素材的实情,该让人看见。
    · 坏区:去尖刺**之后**仍然超过人手生理速度上限的帧。这是 MediaPipe 丢了目标,
      整个邻域都是垃圾,中值和任何平滑都救不了。只能标出来让人重录或裁掉。
    · 尖刺:被 despike 修掉的帧(改动 >SPIKE_VISIBLE_DEG)。

    ⚠ 「是尖刺还是坏区」用**能不能被中值修掉**来分,不要拿区间两端的净位移去判。
    我先写的版本是后者(2 步且回到原位就算尖刺),在**动着的手**上必然失效:手在那
    几帧里本来也在动,净位移天然就大,于是每个尖刺都被误报成坏区 —— 实测把 hand2
    从 2 处灌水到 9 处。改成现在这样还有个附带好处:quality 和 despike 的判定
    天然一致,不会出现"报了尖刺但 despike 没动它"这种自相矛盾。
    """
    n = len(frames)
    rep = {"n_frames": n, "gaps": [], "bad_regions": [], "isolated_spikes": 0,
           "frame_step_p90_deg": 0.0, "max_gap_jump_deg": 0.0}
    if n < 2:
        return rep
    segs = _segments(frames, stride)
    rep["segments"] = len(segs)

    for i in range(1, n):
        prev, cur = frames[i - 1], frames[i]
        jump = max(abs(a - b) for a, b in zip(cur["rad"], prev["rad"])) * _DEG
        if cur["frame"] - prev["frame"] > stride:
            rep["gaps"].append({"after_frame": prev["frame"],
                                "missing": cur["frame"] - prev["frame"] - stride,
                                "t": round(prev["t"], 2),
                                "jump_deg": round(jump, 1)})
            rep["max_gap_jump_deg"] = max(rep["max_gap_jump_deg"], round(jump, 1))

    # 先去尖刺,再在**去完之后**的信号上找超速 —— 剩下的才是修不掉的。
    clean, n_spike = despike(frames, stride=stride)
    # ⚠ 传进来的 frames 可能**已经**被 despike 过(extract 就是这么用的)。那时上面
    # 这次 despike 只是为了拿干净信号找坏区,它的计数**不能**当尖刺数用 —— 干净数据
    # 上只会剩个别帧还能过门槛。实测:原始帧 16 个尖刺,清理后再数只剩 1 个。
    # 两个数各自都对但含义不同,同时显示("已修 16 个" + "1 处尖刺")就是自相矛盾。
    # 所以调用方清理过的话要把真实计数传进来。
    rep["isolated_spikes"] = n_spike if spikes is None else int(spikes)

    steps, over = [], []
    for si, (a, b) in enumerate(segs):
        for i in range(a + 1, b):
            dt = max(1e-6, clean[i]["t"] - clean[i - 1]["t"])
            dmax = max(abs(x - y) for x, y in
                       zip(clean[i]["rad"], clean[i - 1]["rad"])) * _DEG
            steps.append(dmax)
            if dmax / dt > MAX_JOINT_DEG_S:
                # 带上段号。合并坏区时**必须**同段 —— 见下面的注释。
                over.append((si, i))
    if steps:
        steps.sort()
        rep["frame_step_p90_deg"] = round(steps[min(len(steps) - 1,
                                                    int(len(steps) * 0.9))], 2)

    # 连续的超速步合成一个坏区。这里不再需要"回没回原位"那种启发式:
    # 能被中值修掉的已经在 clean 里被修掉了,还留在这儿的就是真失败。
    # 合并条件带段号 `si == last["_s"]`。当前**走不到**这条分支:上面的内层循环是
    # range(a+1, b),每段第一帧不产生 step,所以前段最大超速下标是 a1-1、后段最小是
    # a1+1,差 2,永不相邻(穷举 388800 种两段布局验证过,0 个反例)。
    # 留着是因为它把「不跨段合并」这个不变量写死在代码里:哪天有人把内层循环改成
    # 跨段算差分,这半个条件就能挡住 —— 否则 gap 两侧下标本来相邻(如 69 和 70),
    # 中间却隔着整段没数据,会被并成一个横跨空档的假坏区。
    for si, i in over:
        last = rep["bad_regions"][-1] if rep["bad_regions"] else None
        peak = round(max(abs(x - y) for x, y in
                         zip(clean[i]["rad"], clean[i - 1]["rad"])) * _DEG, 1)
        if last is not None and last["_s"] == si and last["_i"] == i - 1:
            last.update(_i=i, to_frame=clean[i]["frame"], n=last["n"] + 1,
                        peak_deg=max(last["peak_deg"], peak))
        else:
            rep["bad_regions"].append({
                "_s": si, "_i": i, "from_frame": clean[i - 1]["frame"],
                "to_frame": clean[i]["frame"], "t": round(clean[i - 1]["t"], 2),
                "n": 2, "peak_deg": peak})
    for b in rep["bad_regions"]:
        b.pop("_i", None)                        # 内部游标,不进 JSON
        b.pop("_s", None)
    return rep


# ---------------------------------------------------------------------------
# 自动挑关键帧
# ---------------------------------------------------------------------------
# 姿态差阈值(rad,6 关节的 L∞)。0.25 rad ≈ 14°,比 MediaPipe 自身的抖动大一档,
# 又足够分开"张开 / 半握 / 握紧"这类相邻姿态。
KEYFRAME_EPS = 0.25


def pick_keyframes(frames: list[dict], *, eps: float = KEYFRAME_EPS,
                   max_out: int = 12) -> list[dict]:
    """从逐帧序列里挑代表帧。贪心:和上一个选中帧差得够远就选。

    为什么用 L∞ 而不是欧氏距离:L∞ 是"**任何一根**手指动了超过阈值就算新姿态"。
    欧氏距离会把"一根手指大动"和"六根手指各微动"算成一样大,而前者是新手势、
    后者是抖动。手势的语义靠单指的位置,不靠总体位移量。

    首帧总是选。末帧也补上 —— 手势的收尾姿态通常就是要存的那个,让贪心决定要不要
    它会在最后一段变化平缓时把它丢掉。
    """
    if not frames:
        return []
    out = [frames[0]]
    for f in frames[1:]:
        # ⚠ 上限检查要在 append **之前**。写在后面是差一错:先 append 再判
        # len(out) >= max_out 才 break,于是最终长度是 max_out+1 —— max_out=12
        # 实际会出 13 帧。之前没暴露是因为测试数据都没顶到上限。
        if len(out) >= max_out:
            break
        prev = out[-1]["rad"]
        if max(abs(a - b) for a, b in zip(f["rad"], prev)) >= eps:
            out.append(f)
    # 末帧补上(贪心可能因为最后一段变化小而漏掉它)
    if frames[-1] is not out[-1] and len(out) < max_out:
        last = frames[-1]
        if max(abs(a - b) for a, b in zip(last["rad"], out[-1]["rad"])) > 1e-9:
            out.append(last)
    return out


# 驻留时间的上限。超过这个的间隔一律是**漏检造成的空档**(检不到手的帧被跳过,
# 时间轴上就留了个洞),照搬会让回放莫名停顿好几秒。
HOLD_MS_CEIL = 2000


def frames_to_pack_frames(frames: list[dict], *, speed: int = 500,
                          force: int = 500,
                          default_hold_ms: int = 400) -> tuple[list[dict], dict]:
    """帧序列 → 技能包帧(带 hold_ms)。返回 (帧, 时间保真度报告)。

    hold_ms 用**相邻帧的真实时间差**,这样回放节奏和视频里做动作的节奏一致。

    ⚠ 下限是 PLAYER_TICK_MS(回放器的时间分辨率),**不是**我原来拍的 80ms。
    原来那个 80 是错的:30fps 源在 stride=1 时帧间 33ms,被抬到 80ms 就是整段
    慢 2.4× —— 用户看到的"很延迟"就是这么来的。抬到 tick 周期是物理下限(比一个
    tick 短的驻留落不到实处),抬到 80 是凭空加的 2.4 倍慢放。

    上限 HOLD_MS_CEIL 仍然要:那是漏检空档,不是真实节奏。

    报告里带 stretch(实际总时长 / 源时长)。有拉伸就该让人看见,而不是默默慢放。
    """
    from gesture_pack import PLAYER_TICK_MS
    out = []
    gap_src = gap_play = floored = ceiled = 0
    for i, f in enumerate(frames):
        if i + 1 < len(frames):
            dt_ms = int(round((frames[i + 1]["t"] - f["t"]) * 1000))
            hold = int(_clamp(dt_ms, PLAYER_TICK_MS, HOLD_MS_CEIL))
            # ⚠ stretch 只统计**帧间间隔**,不把末帧的 default_hold_ms 算进去。
            # 算进去的话它在分子分母各出现一次,会把真实拉伸**稀释**掉:
            # 120fps 素材实际慢 4×,连着 400ms 末帧一起算只报 1.48× —— 那个数
            # 看起来"还行",于是真正的时序失真被藏住了。
            gap_src += dt_ms
            gap_play += hold
            if dt_ms < PLAYER_TICK_MS:
                floored += 1
            elif dt_ms > HOLD_MS_CEIL:
                ceiled += 1
        else:
            hold = default_hold_ms
        # ⚠ t_ns 直接从**源帧的时刻**换算,不从 hold_ms 累加。
        # 累加的话就把上面 int(round()) 的取整误差攒起来了:30fps 每帧少 0.333ms,
        # 600 帧末尾差 200ms(实测 199.7ms)。t_ns 每帧独立算,残差不累积。
        # 减去首帧时刻,让 t_ns 是"相对包起点"的 —— 素材可能不是从 0 秒开始截的。
        out.append({"label": f"t={f['t']:.2f}s", "rad": list(f["rad"]),
                    "speed": int(speed), "force": int(force), "hold_ms": hold,
                    "t_ns": int(round((f["t"] - frames[0]["t"]) * 1e9))})
    return out, {"src_ms": gap_src, "play_ms": gap_play,
                 "total_play_ms": sum(f["hold_ms"] for f in out),
                 "stretch": round(gap_play / gap_src, 3) if gap_src else 1.0,
                 "floored": floored, "ceiled": ceiled,
                 "tick_ms": PLAYER_TICK_MS}
