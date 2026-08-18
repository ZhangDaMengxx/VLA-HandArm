// replay3d.js — 回放页 three.js 重做:视频+骨骼叠加 + 3D 机器人 + 时序图,帧号对齐。
//
// 替代 Rerun 的 WebViewer(IP/端口依赖、WASM 卡死、逐帧拉伸失真),全走 7860 同源。
// 核心设计:帧号 f 是唯一对齐键,驱动三块:
//   1. 视频跳到 f/fps 秒,canvas 叠加 kp2d[f]
//   2. ComboViewer.setAll([arm[f], hand6[f]])
//   3. 时序图高亮 f 列
//
// ⚠ kp2d 单位是**原视频像素**(u∈[0,srcW], v∈[0,srcH],由 /api/replay/keypoints 报)。
// 视频帧和骨骼画进同一个 canvas、共用同一个变换(见 _render)—— 不能像 Rerun 那样
// 逐帧重新拉伸铺满画布,那样手的真实位置和大小全丢,验证就失效了。

// MediaPipe 手部 21 点连线定义(和 Rerun 一致)
const HAND_CONNECTIONS = [
  [0,1],[1,2],[2,3],[3,4],          // 拇指
  [0,5],[5,6],[6,7],[7,8],          // 食指
  [0,9],[9,10],[10,11],[11,12],     // 中指
  [0,13],[13,14],[14,15],[15,16],   // 无名指
  [0,17],[17,18],[18,19],[19,20],   // 小指
  [5,9],[9,13],[13,17]              // 掌心连线
];

// 6 个驱动关节,顺序必须和 combo3d.js 的 setAll / hand3d.js 的 DRIVEN 一致。
const HAND_DRIVEN_ORDER = ["thumb_proximal_yaw_joint", "thumb_proximal_pitch_joint",
                           "index_proximal_joint", "middle_proximal_joint",
                           "ring_proximal_joint", "pinky_proximal_joint"];

export class ReplayViewer {
  constructor(container) {
    this.container = container;
    this.currentFrame = 0;
    this.isPlaying = false;
    this.fps = 30;
    this.frames = [];            // [{kp2d, vis, wrist_pose}]
    this.trajArm = [];           // [N][7]
    this.trajHand = [];          // [N][12]
    this.comboViewer = null;     // 复用 ComboViewer (懒加载)
    this.videoSrc = "canonical"; // "canonical" | "original"

    this._initDOM();
  }

  _initDOM() {
    this.container.innerHTML = `
      <div class="replay-layout">
        <div class="replay-top">
          <div class="replay-video-panel">
            <video class="replay-video" muted playsinline></video>
            <canvas class="replay-canvas"></canvas>
            <div class="replay-video-switch">
              <label><input type="radio" name="videoSrc" value="canonical" checked> 规范层(256×256,对齐)</label>
              <label><input type="radio" name="videoSrc" value="original"> 原始视频</label>
              <span class="replay-video-msg"></span>
            </div>
          </div>
          <div class="replay-3d-panel" id="replay3dHost">
            <button id="load3DBtn" class="load-3d-btn">🎬 加载3D模型</button>
          </div>
        </div>
        <div class="replay-bottom">
          <canvas class="replay-chart"></canvas>
        </div>
        <div class="replay-controls">
          <button id="replayPlayPause">▶</button>
          <input type="range" id="replayScrubber" min="0" max="100" value="0">
          <span id="replayFrameLabel">0 / 0</span>
        </div>
      </div>
    `;

    this.video = this.container.querySelector(".replay-video");
    this.canvas = this.container.querySelector(".replay-canvas");
    this.ctx = this.canvas.getContext("2d");
    this.chartCanvas = this.container.querySelector(".replay-chart");
    this.chartCtx = this.chartCanvas.getContext("2d");
    this.scrubber = this.container.querySelector("#replayScrubber");
    this.frameLabel = this.container.querySelector("#replayFrameLabel");
    this.playPauseBtn = this.container.querySelector("#replayPlayPause");
    this.load3DBtn = this.container.querySelector("#load3DBtn");

    this._bindEvents();
  }

  _bindEvents() {
    // 刷子的值**就是帧号**(max 在 load() 里设为 N-1)。原来 max 是帧数而 value 按百分比
    // 算,两套单位混用 —— 拖到末尾会差几帧,正是"验证对不对"最不能忍的误差。
    this.scrubber.addEventListener("input", () => this.seekToFrame(+this.scrubber.value));
    this.playPauseBtn.addEventListener("click", () => this.togglePlay());

    // 点时序图直接跳帧 —— 时序图已隐藏，跳过
    // this.chartCanvas.addEventListener("click", (e) => {
    //   if (!this.frames.length) return;
    //   const r = this.chartCanvas.getBoundingClientRect();
    //   const ratio = (e.clientX - r.left) / Math.max(r.width, 1);
    //   this.seekToFrame(Math.round(ratio * (this.frames.length - 1)));
    // });

    this.container.querySelectorAll('input[name="videoSrc"]').forEach(radio => {
      radio.addEventListener("change", async (e) => {
        const prev = this.videoSrc;
        this.videoSrc = e.target.value;
        try { await this._loadVideo(); }
        catch (err) {
          this.videoSrc = prev;                    // 回退选中项,别让 UI 说谎
          const back = this.container.querySelector(`input[name="videoSrc"][value="${prev}"]`);
          if (back) back.checked = true;
          this._msg("原始视频未保存 · 重跑一次管线后可用");
          try { await this._loadVideo(); } catch (_) {}
        }
      });
    });

    // 实时摄像头切换
    // 懒加载3D模型
    this.load3DBtn.addEventListener("click", () => this._load3D());

    // ⚠ 设了 currentTime 之后帧是**异步**解码的,立刻画会画出上一帧 —— 帧号就和画面
    // 差一格,正是"验证对不对"最致命的错。所以等 seeked 落定再重画。
    this.video.addEventListener("seeked", () => this._render());
    this.video.addEventListener("loadeddata", () => this._render());

    // 面板尺寸变了要重算目标矩形(canvas 是按面板像素分配的,不跟视频尺寸绑定)。
    const panel = this.container.querySelector(".replay-video-panel");
    if (panel && window.ResizeObserver) {
      this._ro = new ResizeObserver(() => this._render());
      this._ro.observe(panel);
    }

    // 存引用:查看器每跑一次管线就重建一个,不解绑会叠加多份监听(按一次键跳好几帧)。
    this._onKey = (e) => {
      if (!this.frames.length) return;
      if (e.key === "ArrowLeft")  { e.preventDefault(); this.seekToFrame(this.currentFrame - 1); }
      if (e.key === "ArrowRight") { e.preventDefault(); this.seekToFrame(this.currentFrame + 1); }
      if (e.key === " ")          { e.preventDefault(); this.togglePlay(); }
    };
    document.addEventListener("keydown", this._onKey);
  }

  async load(robot) {
    try {
      // 拉骨骼点
      const kpRes = await fetch(`/api/replay/keypoints?robot=${robot}`);
      const kpData = await kpRes.json();
      if (kpData.error) throw new Error(kpData.error);
      this.fps = kpData.fps || 30;
      this.frames = kpData.frames;
      this.srcW = kpData.src_w || 540;      // kp2d 的像素基准,分轴缩放要用
      this.srcH = kpData.src_h || 960;
      this.robot = robot;
      // vis 全 0 = 这帧 MediaPipe 没检到手,build_canonical 用上一帧填充了 →
      // 该帧的 IK 结果不可信。预算成掩码,时序图画红条、帧号标注都用它。
      this._lost = this.frames.map(fr => fr.vis.every(v => v === 0));

      // 拉轨迹
      const trajRes = await fetch(`/api/traj/frames?robot=${robot}`);
      const trajData = await trajRes.json();
      if (trajData.error) throw new Error(trajData.error);
      this.trajArm = trajData.arm;
      this.trajHand = trajData.hand;
      // ⚠ npz 的 hand_joint_names 是**交错**顺序(index_prox, index_inter, middle_prox,
      // middle_inter, pinky_prox, pinky_inter, ring_prox, ring_inter, thumb_yaw,
      // thumb_pitch, thumb_inter, thumb_distal),和 DRIVEN 顺序完全不同。按**名字**
      // 取下标 —— slice(0,6) 会把 index_inter 当成 thumb_pitch 喂进去,手全错还看着像在动。
      this.handNames = trajData.hand_names || [];
      this._drivenIdx = HAND_DRIVEN_ORDER.map(n => this.handNames.indexOf(n));
      const miss = HAND_DRIVEN_ORDER.filter((_, i) => this._drivenIdx[i] < 0);
      if (miss.length) console.warn("[replay3d] 轨迹缺这些驱动关节,手不会动:", miss);
      // 臂:npz 的 arm_joint_names 是 joint1..joint7,和 combo3d 的 ARM_JOINTS 一致,可直传。
      // 每个关节各自的幅值范围,时序图按它归一化(固定 2π 量程会把小幅动作压成直线)。
      this._armRange = [];
      for (let j = 0; j < 7; j++) {
        let lo = Infinity, hi = -Infinity;
        for (const row of this.trajArm) {
          const v = row[j];
          if (v < lo) lo = v;
          if (v > hi) hi = v;
        }
        this._armRange.push([lo, hi]);
      }

      // 加载 3D (懒加载 - 不默认创建)
      // 用户点击"加载3D模型"按钮后才初始化

      // 加载视频
      await this._loadVideo();

      // 初始化 UI
      this.scrubber.max = this.frames.length - 1;
      this.seekToFrame(0);

    } catch (e) {
      console.error("replay 加载失败:", e);
      throw e;
    }
  }

  /** 切视频源。canonical=256×256(必然对齐);original=原分辨率(帧数相同才对齐)。
   *  ⚠ video.load() 不返回 Promise,要等 loadedmetadata 才知道真实尺寸。 */
  async _loadVideo() {
    const url = this.videoSrc === "canonical"
      ? "/api/replay/video/canonical"
      : `/api/replay/video/original?robot=${encodeURIComponent(this.robot || "")}`;
    const ready = new Promise((res, rej) => {
      const ok = () => { cleanup(); res(); };
      const bad = () => { cleanup(); rej(new Error("视频加载失败: " + url)); };
      const cleanup = () => {
        this.video.removeEventListener("loadedmetadata", ok);
        this.video.removeEventListener("error", bad);
      };
      this.video.addEventListener("loadedmetadata", ok, { once: true });
      this.video.addEventListener("error", bad, { once: true });
    });
    this.video.src = url;
    this.video.load();
    await ready;
    this.seekToFrame(this.currentFrame);   // 换源后按当前帧重画,别跳回 0
  }

  /** 显式跳帧(刷子/方向键):暂停 + 设视频时间 + 应用该帧。 */
  seekToFrame(f) {
    f = Math.max(0, Math.min(this.frames.length - 1, f));
    if (this.isPlaying) { this.isPlaying = false; this.playPauseBtn.textContent = "▶"; this._stopTick(); }
    this.video.pause();
    this.video.currentTime = f / this.fps;
    this._applyFrame(f);
  }

  /** 把第 f 帧应用到三块视图。播放时由 rAF 调,**不回写** video.currentTime ——
   *  边播边设 currentTime 会自己打断解码,画面卡成幻灯片。 */
  _applyFrame(f) {
    this.currentFrame = f;
    this._render();     // 画视频帧 + 骨骼
    if (this.comboViewer && this.comboViewer.ready) {
      const arm7 = this.trajArm[f], hand12 = this.trajHand[f];
      if (arm7 && hand12) {
        const hand6 = this._drivenIdx.map(i => (i >= 0 ? hand12[i] : 0));
        this.comboViewer.setAll([...arm7, ...hand6]);
      }
    }
    this.scrubber.value = f;
    const lost = this.frames[f].vis.every(v => v === 0);
    this.frameLabel.textContent = `${f} / ${this.frames.length - 1}${lost ? " · 检测丢失" : ""}`;
    // this._drawChart(f);  // 时序图已隐藏，跳过绘制
  }

  /** 把视频帧和骨骼画进**同一个** canvas。
   *  为什么不用 <video> + 叠加 canvas 两个元素:那需要两者的显示矩形严格一致,而
   *  canonical 是 256×256 方图(源其实是 540×960 竖拍被压扁的),object-fit 只会按
   *  图片自身比例摆放 —— 于是 canonical 显示成方的、original 显示成竖的,两个源
   *  看起来"不是同一个视频"。这里改成自己算目标矩形:一律按**源比例** srcW:srcH
   *  摆放,canonical 被拉回正确比例,两个源看起来一致;骨骼点用同一个变换,对齐
   *  就是构造性成立的,不再依赖 CSS 的行为。 */
  _render() {
    const c = this.canvas, ctx = this.ctx;
    const panel = c.parentElement;
    if (!panel) return;
    const cssW = panel.clientWidth, cssH = panel.clientHeight;
    if (!cssW || !cssH || !this.srcW || !this.srcH) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (c.width !== Math.round(cssW * dpr) || c.height !== Math.round(cssH * dpr)) {
      c.width = Math.round(cssW * dpr); c.height = Math.round(cssH * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);      // 之后一律用 CSS px 作图
    ctx.fillStyle = "#0a0b0d";                    // 盖住背后的 <video>,免得露出未缩放的原图
    ctx.fillRect(0, 0, cssW, cssH);

    // 目标矩形:按源比例 contain 进面板
    const ar = this.srcW / this.srcH;
    let w = cssW, h = cssW / ar;
    if (h > cssH) { h = cssH; w = cssH * ar; }
    const ox = (cssW - w) / 2, oy = (cssH - h) / 2;

    // 两种模式都画视频帧+骨骼叠加,验证对齐效果
    if (this.video && this.video.readyState >= 2) {
      try { ctx.drawImage(this.video, ox, oy, w, h); } catch (_) {}
    }
    this._drawSkeleton(ox, oy, w / this.srcW, h / this.srcH);
  }

  /** 画骨骼。(ox,oy)+缩放由 _render 给 —— 和视频帧用同一个变换,所以叠加必然对齐。 */
  _drawSkeleton(ox, oy, sx, sy) {
    const f = this.currentFrame;
    const fr = this.frames[f];
    if (!fr) return;
    const { kp2d, vis } = fr;   // kp2d 是 [[u,v] × 21],**不是**扁平数组
    const ctx = this.ctx;
    const X = u => ox + u * sx, Y = v => oy + v * sy;

    // 检测丢失帧用红框标(框住视频矩形,不是整个 canvas)
    const allInvis = vis.every(v => v === 0);
    if (allInvis) {
      ctx.strokeStyle = "rgba(255,0,0,0.6)"; ctx.lineWidth = 4;
      ctx.strokeRect(ox, oy, this.srcW * sx, this.srcH * sy);
      ctx.font = "14px sans-serif"; ctx.fillStyle = "red";
      ctx.fillText("检测丢失", ox + 10, oy + 20);
      return;
    }

    // 画连线
    ctx.strokeStyle = "rgba(0,255,128,0.8)"; ctx.lineWidth = 2;
    ctx.beginPath();
    for (const [a, b] of HAND_CONNECTIONS) {
      const pa = kp2d[a], pb = kp2d[b];
      if (!pa || !pb) continue;
      const x1 = X(pa[0]), y1 = Y(pa[1]);
      const x2 = X(pb[0]), y2 = Y(pb[1]);
      if (!isFinite(x1) || !isFinite(y1) || !isFinite(x2) || !isFinite(y2)) continue;
      ctx.moveTo(x1, y1); ctx.lineTo(x2, y2);
    }
    ctx.stroke();

    // 画关节点。手腕(0)画大一点,便于看朝向;置信度低的画空心。
    for (let i = 0; i < 21; i++) {
      const p = kp2d[i];
      if (!p) continue;
      const x = X(p[0]), y = Y(p[1]);
      if (!isFinite(x) || !isFinite(y)) continue;
      const r = i === 0 ? 5 : 3;
      ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2);
      if (vis[i] < 0.5) {                     // 低置信度:空心,提示这一点不可信
        ctx.strokeStyle = "rgba(255,200,0,0.9)"; ctx.lineWidth = 1.5; ctx.stroke();
      } else {
        ctx.fillStyle = i === 0 ? "rgba(0,200,255,0.95)" : "rgba(255,255,0,0.9)";
        ctx.fill();
      }
    }
  }

  _drawChart(highlightF) {
    const c = this.chartCanvas, ctx = this.chartCtx;
    const N = this.trajArm.length;
    if (!N) return;

    // 只在 CSS 尺寸真变了才重设 canvas 物理尺寸，避免每帧都清空重绘
    const cssW = c.clientWidth, cssH = c.clientHeight;
    if (!this._lastChartW || this._lastChartW !== cssW || this._lastChartH !== cssH) {
      c.width = cssW; c.height = cssH;
      this._lastChartW = cssW; this._lastChartH = cssH;
    }

    // 提亮背景，从近黑 #1a1c20 改成深灰 #2a2d33
    ctx.fillStyle = "#2a2d33";
    ctx.fillRect(0, 0, cssW, cssH);

    // 检测丢失帧画红条
    ctx.fillStyle = "rgba(255,60,60,0.18)";
    const bw = Math.max(1, cssW / N);
    for (let f = 0; f < N; f++) {
      if (this._lost && this._lost[f]) ctx.fillRect((f / (N - 1)) * cssW, 0, bw, cssH);
    }

    // 臂 7 关节各占一条带，按各自实际幅值归一化
    const colors = ["#f44", "#f80", "#fa0", "#8f4", "#4af", "#a4f", "#f4a"];
    const labels = ["关节1", "关节2", "关节3", "关节4", "关节5", "关节6", "关节7"];
    const h = cssH / 7;

    for (let j = 0; j < 7; j++) {
      const [lo, hi] = this._armRange[j];
      const span = Math.max(hi - lo, 1e-6);

      // 画曲线
      ctx.strokeStyle = colors[j]; ctx.lineWidth = 1.4;
      ctx.beginPath();
      for (let f = 0; f < N; f++) {
        const x = (f / (N - 1)) * cssW;
        const y = (j + 0.9) * h - ((this.trajArm[f][j] - lo) / span) * h * 0.8;
        f === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      }
      ctx.stroke();

      // 左侧标签：关节名称
      ctx.fillStyle = colors[j];
      ctx.font = "10px sans-serif";
      ctx.textAlign = "left";
      ctx.textBaseline = "top";
      ctx.fillText(labels[j], 5, j * h + 3);
    }

    // 当前帧竖线
    if (highlightF >= 0 && highlightF < N) {
      const x = (highlightF / (N - 1)) * cssW;
      ctx.strokeStyle = "rgba(255,255,255,0.75)"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, cssH); ctx.stroke();
    }
  }

  togglePlay() {
    if (!this.frames.length) return;
    this.isPlaying = !this.isPlaying;
    this.playPauseBtn.textContent = this.isPlaying ? "⏸" : "▶";
    if (!this.isPlaying) { this.video.pause(); this._stopTick(); return; }
    if (this.currentFrame >= this.frames.length - 1) {     // 播完了再按:从头
      this.video.currentTime = 0; this._applyFrame(0);
    }
    this.video.play().catch(() => {});     // muted + 用户点击触发,不会被自动播放策略拦
    this._tick();
  }

  /** 播放时让 <video> 自己走,用 rAF 读它的 currentTime 反推帧号。比 setTimeout 逐帧
   *  seek 稳:不打断解码,也不会和视频真实时间越播越漂。 */
  _tick() {
    if (!this.isPlaying) return;
    const N = this.frames.length;
    const f = Math.min(N - 1, Math.max(0, Math.round(this.video.currentTime * this.fps)));
    if (f !== this.currentFrame) this._applyFrame(f);
    if (this.video.ended || f >= N - 1) {
      this.isPlaying = false; this.playPauseBtn.textContent = "▶";
      this.video.pause(); this._stopTick(); return;
    }
    this._raf = requestAnimationFrame(() => this._tick());
  }

  _stopTick() { if (this._raf) { cancelAnimationFrame(this._raf); this._raf = null; } }

  _msg(text) {
    const el = this.container.querySelector(".replay-video-msg");
    if (!el) return;
    el.textContent = text;
    clearTimeout(this._msgTimer);
    this._msgTimer = setTimeout(() => { el.textContent = ""; }, 4000);
  }

  /** 每跑一次管线都会重建查看器 —— 不解绑就会叠加 keydown 监听和 rAF 循环。 */
  destroy() {
    this.isPlaying = false;
    this._stopTick();
    clearTimeout(this._msgTimer);
    if (this._onKey) document.removeEventListener("keydown", this._onKey);
    if (this._ro) { this._ro.disconnect(); this._ro = null; }
    try { this.video.pause(); this.video.removeAttribute("src"); this.video.load(); } catch (_) {}

    // 清理3D资源
    if (this.comboViewer && this.comboViewer.destroy) {
      this.comboViewer.destroy();
    }
    this.comboViewer = null;

    // 清理摄像头资源
    if (this.mimicController) {
      this.mimicController.stop();
      this.mimicController = null;
    }
  }

  /** 懒加载3D模型 */
  async _load3D() {
    if (this.comboViewer) return;  // 已加载

    try {
      this.load3DBtn.textContent = "加载中...";
      this.load3DBtn.disabled = true;

      if (!window.ComboViewer) {
        await new Promise(r => window.addEventListener("combo3d-ready", r, { once: true }));
      }

      const host = document.getElementById("replay3dHost");
      this.comboViewer = new window.ComboViewer(host);
      await this.comboViewer.load("/combo_assets/nero_inspire_right_viz.urdf");

      // 同步当前帧
      if (this.frames.length > 0) {
        this._applyFrame(this.currentFrame);
      }

      this.load3DBtn.remove();  // 加载完成后移除按钮
    } catch (err) {
      console.error("3D加载失败:", err);
      this.load3DBtn.textContent = "❌ 加载失败";
      this.load3DBtn.disabled = false;
    }
  }
}

// 就绪事件由 index.html 的桥接 script 在 import 后统一 dispatch —— 这里再发一次会
// 在 DOMContentLoaded 已过时永不触发,反而误导。
