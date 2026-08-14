// 实时摄像头手势控制：浏览器推理，后端只负责坐标转换和 dex-retargeting。
import { initializeHandTracker, requestedHandEngine } from "./hand_tracker_tasks.js";

const HAND_CONNECTIONS = [
  [0, 1], [1, 2], [2, 3], [3, 4],
  [0, 5], [5, 6], [6, 7], [7, 8],
  [0, 9], [9, 10], [10, 11], [11, 12],
  [0, 13], [13, 14], [14, 15], [15, 16],
  [0, 17], [17, 18], [18, 19], [19, 20],
  [5, 9], [9, 13], [13, 17]
];

export class HandMimicController {
  constructor(container, onJointAnglesCallback = null) {
    this.container = container;
    this.onJointAngles = onJointAnglesCallback;
    this.tracker = null;
    this.engine = null;
    this.fallbackReason = null;

    this.videoElement = null;
    this.canvasElement = null;
    this.canvasCtx = null;
    this.statusElement = null;

    this.active = false;
    this.generation = 0;
    this.frameHandle = null;
    this.frameHandleType = null;
    this.inferenceRunning = false;
    this.lastVideoTime = -1;

    this.frameCount = 0;
    this.lastFpsTime = 0;
    this.currentFps = 0;
    this.inferenceSamples = [];
    this.inferenceP95 = 0;
    this.frameSequence = 0;
    this.frameTimings = new Map();
    this.shouldDriveHardware = () => false;

    this.ws = null;
    this.wsConnectPromise = null;
    this.wsConnectFinish = null;
    this.wsReconnectTimer = null;
    this.transportInFlight = false;
    this.transportType = null;
    this.inFlightPayload = null;
    this.pendingPayload = null;
    this.httpAbortController = null;

    this._setupUI();
  }

  _setupUI() {
    this.container.innerHTML = `
      <div class="mimic-panel">
        <video class="mimic-video" autoplay playsinline muted></video>
        <canvas class="mimic-canvas"></canvas>
        <div class="mimic-status">初始化中...</div>
      </div>
    `;
    this.videoElement = this.container.querySelector(".mimic-video");
    this.canvasElement = this.container.querySelector(".mimic-canvas");
    this.canvasCtx = this.canvasElement.getContext("2d");
    this.statusElement = this.container.querySelector(".mimic-status");
  }

  async start() {
    if (this.active) return;
    this.active = true;
    const generation = ++this.generation;
    this._resetRuntimeState();

    try {
      this._setStatus("加载 MediaPipe...");
      const initialized = await initializeHandTracker({ engine: requestedHandEngine() });
      if (!this._isCurrent(generation)) {
        await initialized.tracker.close();
        return;
      }
      this.tracker = initialized.tracker;
      this.engine = initialized.engine;
      this.fallbackReason = initialized.fallbackReason;

      this._setStatus("连接服务器...");
      await this._connectWebSocket(generation);
      if (!this._isCurrent(generation)) return;

      this._setStatus("启动摄像头...");
      await this._startCamera(generation);
      if (!this._isCurrent(generation)) return;

      this.lastFpsTime = performance.now();
      this._setStatus(this._readyStatus());
      this._scheduleFrame(generation);
    } catch (error) {
      console.error("[HandMimic] 启动失败:", error);
      await this.stop();
      this._setStatus(`错误: ${error.message}`);
      throw error;
    }
  }

  _resetRuntimeState() {
    this.frameCount = 0;
    this.lastFpsTime = 0;
    this.currentFps = 0;
    this.inferenceSamples = [];
    this.inferenceP95 = 0;
    this.lastVideoTime = -1;
    this.pendingPayload = null;
    this.inFlightPayload = null;
    this.transportInFlight = false;
    this.transportType = null;
  }

  _isCurrent(generation) {
    return this.active && generation === this.generation;
  }

  setHardwareDriveCheck(checkFn) {
    this.shouldDriveHardware = typeof checkFn === "function" ? checkFn : () => false;
  }

  async _startCamera(generation) {
    if (!navigator.mediaDevices?.getUserMedia) {
      throw new Error("当前浏览器不支持摄像头访问");
    }
    const stream = await navigator.mediaDevices.getUserMedia({
      video: {
        facingMode: "user",
        width: { ideal: 480 },
        height: { ideal: 360 },
        frameRate: { ideal: 60, min: 30 }
      }
    });
    if (!this._isCurrent(generation)) {
      stream.getTracks().forEach((track) => track.stop());
      return;
    }

    this.videoElement.srcObject = stream;
    if (this.videoElement.readyState < HTMLMediaElement.HAVE_METADATA) {
      await new Promise((resolve, reject) => {
        this.videoElement.addEventListener("loadedmetadata", resolve, { once: true });
        this.videoElement.addEventListener("error", () => reject(new Error("摄像头视频加载失败")), { once: true });
      });
    }
    await this.videoElement.play();
    if (!this._isCurrent(generation)) return;

    this.canvasElement.width = this.videoElement.videoWidth;
    this.canvasElement.height = this.videoElement.videoHeight;
    // 画面和骨骼统一由 canvas 显示，避免 video/canvas 各自 object-fit 产生偏移。
    this.videoElement.style.visibility = "hidden";
    console.log(`[HandMimic] 摄像头分辨率: ${this.videoElement.videoWidth}x${this.videoElement.videoHeight}`);
  }

  _scheduleFrame(generation) {
    if (!this._isCurrent(generation) || this.frameHandle !== null) return;
    if (typeof this.videoElement.requestVideoFrameCallback === "function") {
      this.frameHandleType = "video";
      this.frameHandle = this.videoElement.requestVideoFrameCallback((timestamp, metadata) => {
        this.frameHandle = null;
        this._processFrame(timestamp, metadata, generation);
      });
    } else {
      this.frameHandleType = "raf";
      this.frameHandle = requestAnimationFrame((timestamp) => {
        this.frameHandle = null;
        this._processFrame(timestamp, null, generation);
      });
    }
  }

  async _processFrame(timestamp, metadata, generation) {
    if (!this._isCurrent(generation)) return;

    // rAF 可能比视频解码快；同一个 currentTime 只处理一次。
    if (!metadata && this.videoElement.currentTime === this.lastVideoTime) {
      this._scheduleFrame(generation);
      return;
    }
    this.lastVideoTime = metadata?.mediaTime ?? this.videoElement.currentTime;
    if (this.inferenceRunning) return;

    this.inferenceRunning = true;
    const startedAt = performance.now();
    try {
      this._drawVideoFrame();
      const result = await this.tracker.detect(this.videoElement, timestamp);
      if (this._isCurrent(generation)) this._onTrackerResult(result);
    } catch (error) {
      if (this._isCurrent(generation)) {
        console.warn("[HandMimic] 单帧推理失败，继续下一帧:", error);
        this._setStatus(`推理错误: ${error.message}`);
      }
    } finally {
      this.inferenceRunning = false;
      if (this._isCurrent(generation)) {
        this._recordInference(performance.now() - startedAt);
        this._scheduleFrame(generation);
      }
    }
  }

  _recordInference(durationMs) {
    this.frameCount++;
    this.inferenceSamples.push(durationMs);
    if (this.inferenceSamples.length > 300) this.inferenceSamples.shift();

    const now = performance.now();
    if (now - this.lastFpsTime >= 1000) {
      this.currentFps = Math.round(this.frameCount * 1000 / (now - this.lastFpsTime));
      const sorted = [...this.inferenceSamples].sort((a, b) => a - b);
      this.inferenceP95 = sorted[Math.max(0, Math.ceil(sorted.length * 0.95) - 1)] || 0;
      console.log(`[HandMimic] ${this.engine} FPS=${this.currentFps}, inference P95=${this.inferenceP95.toFixed(1)}ms`);
      this.frameCount = 0;
      this.lastFpsTime = now;
    }
  }

  _onTrackerResult(result) {
    const { landmarks, worldLandmarks } = result;

    if (landmarks?.length === 21) {
      this._drawConnectors(landmarks);
      this._drawLandmarks(landmarks);
    }
    this._drawFps();

    if (worldLandmarks) {
      const payload = this._makePayload(worldLandmarks);
      if (payload) this._queuePayload(payload);
    }
  }

  _drawVideoFrame() {
    const width = this.canvasElement.width;
    const height = this.canvasElement.height;
    this.canvasCtx.clearRect(0, 0, width, height);
    this.canvasCtx.drawImage(this.videoElement, 0, 0, width, height);
  }

  _makePayload(landmarks) {
    if (landmarks.length !== 21) {
      console.warn(`[HandMimic] 忽略非 21 点结果: ${landmarks.length}`);
      return null;
    }
    const points = landmarks.map(({ x, y, z }) => ({ x, y, z }));
    if (points.some((point) => !Number.isFinite(point.x) || !Number.isFinite(point.y) || !Number.isFinite(point.z))) {
      console.warn("[HandMimic] 忽略包含非有限坐标的结果");
      return null;
    }
    const frameId = ++this.frameSequence;
    this.frameTimings.set(frameId, performance.now());
    return {
      format: "mediapipe",
      landmarks: points,
      drive_hardware: Boolean(this.shouldDriveHardware()),
      frame_id: frameId
    };
  }

  _queuePayload(payload) {
    if (this.pendingPayload?.frame_id != null) {
      this.frameTimings.delete(this.pendingPayload.frame_id);
    }
    this.pendingPayload = payload;
    this._flushTransport();
  }

  _flushTransport() {
    if (!this.active || this.transportInFlight || !this.pendingPayload) return;
    const payload = this.pendingPayload;
    this.pendingPayload = null;

    if (this.ws?.readyState === WebSocket.OPEN) {
      try {
        this.ws.send(JSON.stringify(payload));
        this.transportInFlight = true;
        this.transportType = "websocket";
        this.inFlightPayload = payload;
        return;
      } catch (error) {
        console.warn("[HandMimic] WebSocket 发送失败，使用 HTTP:", error);
        this.pendingPayload = payload;
        this.ws.close();
      }
    } else {
      this.pendingPayload = payload;
    }

    this._sendPendingViaHttp();
  }

  async _sendPendingViaHttp() {
    if (!this.active || this.transportInFlight || !this.pendingPayload) return;
    const payload = this.pendingPayload;
    this.pendingPayload = null;
    this.transportInFlight = true;
    this.transportType = "http";
    this.inFlightPayload = payload;
    const controller = new AbortController();
    this.httpAbortController = controller;

    try {
      const response = await fetch("/api/hand/mimic", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
        signal: controller.signal
      });
      let result;
      try {
        result = await response.json();
      } catch (error) {
        console.warn("[HandMimic] HTTP 响应 JSON 无效，摄像头继续运行:", error);
        return;
      }
      if (this.active) this._handleServerResponse(result);
    } catch (error) {
      if (this.active && error.name !== "AbortError") {
        console.warn("[HandMimic] HTTP 请求失败，摄像头继续运行:", error);
        this._setStatus(`网络错误: ${error.message}`);
      }
    } finally {
      if (this.httpAbortController === controller) this.httpAbortController = null;
      if (this.transportType === "http") this._releaseTransport();
    }
  }

  _releaseTransport() {
    if (this.inFlightPayload?.frame_id != null) {
      this.frameTimings.delete(this.inFlightPayload.frame_id);
    }
    this.transportInFlight = false;
    this.transportType = null;
    this.inFlightPayload = null;
    this._flushTransport();
  }

  async _connectWebSocket(generation) {
    if (!this._isCurrent(generation) || this.ws?.readyState === WebSocket.OPEN) return;
    if (this.wsConnectPromise) return this.wsConnectPromise;

    const connectionPromise = new Promise((resolve) => {
      const protocol = location.protocol === "https:" ? "wss:" : "ws:";
      let socket;
      try {
        socket = new WebSocket(`${protocol}//${location.host}/ws/hand/mimic`);
      } catch (error) {
        console.warn("[HandMimic] WebSocket 不可用，暂用 HTTP:", error);
        resolve();
        return;
      }
      this.ws = socket;
      let settled = false;
      const finish = () => {
        if (settled) return;
        settled = true;
        clearTimeout(timeout);
        if (this.wsConnectFinish === finish) this.wsConnectFinish = null;
        resolve();
      };
      this.wsConnectFinish = finish;
      const timeout = setTimeout(() => {
        if (socket.readyState !== WebSocket.OPEN) socket.close();
        finish();
      }, 3000);

      socket.onopen = () => {
        if (!this._isCurrent(generation) || this.ws !== socket) {
          socket.close();
          finish();
          return;
        }
        console.log("[HandMimic] WebSocket 连接成功");
        finish();
        this._flushTransport();
      };
      socket.onmessage = (event) => this._handleWebSocketMessage(socket, event);
      socket.onerror = (error) => {
        console.warn("[HandMimic] WebSocket 错误，暂用 HTTP:", error);
        if (socket.readyState === WebSocket.CONNECTING) socket.close();
        finish();
      };
      socket.onclose = () => {
        finish();
        if (this.ws === socket) this.ws = null;
        if (this.transportInFlight && this.transportType === "websocket") {
          // 未收到响应的帧可能已丢失；没有更新帧时重试它，有更新帧时保留更新帧。
          this.pendingPayload ||= this.inFlightPayload;
          this.transportInFlight = false;
          this.transportType = null;
          this.inFlightPayload = null;
        }
        if (this._isCurrent(generation)) {
          this._flushTransport();
          this._scheduleWebSocketReconnect(generation);
        }
      };
    });
    this.wsConnectPromise = connectionPromise;
    try {
      await connectionPromise;
    } finally {
      if (this.wsConnectPromise === connectionPromise) this.wsConnectPromise = null;
    }
  }

  _handleWebSocketMessage(socket, event) {
    if (socket !== this.ws || !this.active) return;
    try {
      const result = JSON.parse(event.data);
      this._handleServerResponse(result);
    } catch (error) {
      console.warn("[HandMimic] WebSocket 响应 JSON 无效，摄像头继续运行:", error);
    } finally {
      if (this.transportInFlight && this.transportType === "websocket") {
        this._releaseTransport();
      }
    }
  }

  _scheduleWebSocketReconnect(generation) {
    if (!this._isCurrent(generation) || this.wsReconnectTimer) return;
    this.wsReconnectTimer = setTimeout(() => {
      this.wsReconnectTimer = null;
      if (this._isCurrent(generation)) this._connectWebSocket(generation);
    }, 5000);
  }

  _handleServerResponse(result) {
    if (result?.frame_id != null) {
      const startedAt = this.frameTimings.get(result.frame_id);
      this.frameTimings.delete(result.frame_id);
      if (startedAt != null) {
        const hardware = result.hardware || {};
        console.log(
          `[perf-hand/frontend] id=${result.frame_id} ` +
          `WS_RTT=${(performance.now() - startedAt).toFixed(1)}ms ` +
          `hardware=${hardware.queued ? "queued" : (hardware.reason || "off")}`
        );
      }
    }
    if (result?.ok) {
      this._setStatus(`${this._metricsStatus()} | ${result.gesture || "执行中"}`);
      if (this.onJointAngles && result.joint_angles) this.onJointAngles(result.joint_angles);
    } else {
      this._setStatus(`${this._metricsStatus()} | ${result?.msg || "未识别"}`);
    }
  }

  _readyStatus() {
    const fallback = this.fallbackReason ? " (Tasks 已降级)" : "";
    return `就绪 ${this.engine}/${this.tracker.delegate}${fallback}`;
  }

  _metricsStatus() {
    return `${this.engine}/${this.tracker?.delegate || "-"} ${this.currentFps} FPS P95 ${this.inferenceP95.toFixed(1)}ms`;
  }

  getMetrics() {
    return {
      requestedEngine: requestedHandEngine(),
      engine: this.engine,
      delegate: this.tracker?.delegate || null,
      fps: this.currentFps,
      inferenceP95Ms: this.inferenceP95,
      transportInFlight: this.transportInFlight ? 1 : 0,
      hasPendingFrame: Boolean(this.pendingPayload),
      driveHardware: Boolean(this.shouldDriveHardware())
    };
  }

  _drawConnectors(landmarks) {
    this.canvasCtx.strokeStyle = "#00ff00";
    this.canvasCtx.lineWidth = 2;
    for (const [start, end] of HAND_CONNECTIONS) {
      const pointA = landmarks[start];
      const pointB = landmarks[end];
      this.canvasCtx.beginPath();
      this.canvasCtx.moveTo(pointA.x * this.canvasElement.width, pointA.y * this.canvasElement.height);
      this.canvasCtx.lineTo(pointB.x * this.canvasElement.width, pointB.y * this.canvasElement.height);
      this.canvasCtx.stroke();
    }
  }

  _drawLandmarks(landmarks) {
    this.canvasCtx.fillStyle = "#00ff00";
    for (const point of landmarks) {
      this.canvasCtx.beginPath();
      this.canvasCtx.arc(
        point.x * this.canvasElement.width,
        point.y * this.canvasElement.height,
        4,
        0,
        2 * Math.PI
      );
      this.canvasCtx.fill();
    }
  }

  _drawFps() {
    const context = this.canvasCtx;
    context.font = "bold 24px Arial";
    context.textAlign = "right";
    context.textBaseline = "top";
    context.strokeStyle = "#000000";
    context.lineWidth = 4;
    context.strokeText(`${this.currentFps} FPS`, this.canvasElement.width - 10, 10);
    context.fillStyle = "#00ff00";
    context.fillText(`${this.currentFps} FPS`, this.canvasElement.width - 10, 10);
  }

  _cancelFrame() {
    if (this.frameHandle === null) return;
    if (this.frameHandleType === "video") {
      this.videoElement.cancelVideoFrameCallback?.(this.frameHandle);
    } else {
      cancelAnimationFrame(this.frameHandle);
    }
    this.frameHandle = null;
    this.frameHandleType = null;
  }

  async stop() {
    this.active = false;
    this.generation++;
    this._cancelFrame();

    if (this.wsReconnectTimer) clearTimeout(this.wsReconnectTimer);
    this.wsReconnectTimer = null;
    this.wsConnectFinish?.();
    this.wsConnectFinish = null;
    this.wsConnectPromise = null;
    this.httpAbortController?.abort();
    this.httpAbortController = null;

    const socket = this.ws;
    this.ws = null;
    if (socket) {
      socket.onopen = null;
      socket.onmessage = null;
      socket.onerror = null;
      socket.onclose = null;
      socket.close();
    }

    const stream = this.videoElement?.srcObject;
    if (stream) stream.getTracks().forEach((track) => track.stop());
    if (this.videoElement) {
      this.videoElement.pause();
      this.videoElement.srcObject = null;
      this.videoElement.style.visibility = "";
    }

    const tracker = this.tracker;
    this.tracker = null;
    if (tracker) await Promise.resolve(tracker.close()).catch((error) => {
      console.warn("[HandMimic] 关闭 tracker 失败:", error);
    });

    this.pendingPayload = null;
    this.inFlightPayload = null;
    this.frameTimings.clear();
    this.transportInFlight = false;
    this.transportType = null;
    this.inferenceRunning = false;
    this._setStatus("已停止");
  }

  _setStatus(message) {
    if (this.statusElement) this.statusElement.textContent = message;
  }
}
