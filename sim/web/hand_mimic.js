// sim/web/hand_mimic.js — 实时摄像头手势控制
// 使用 MediaPipe Hands (WASM) 进行前端推理，不占用服务器 GPU

// 使用本地托管的 MediaPipe 文件（避免 CDN 访问问题）
const MP_CDN = "/vendor/mediapipe";

export class HandMimicController {
  constructor(container, onJointAnglesCallback = null) {
    this.container = container;
    this.hands = null;
    this.camera = null;
    this.videoElement = null;
    this.canvasElement = null;
    this.canvasCtx = null;
    this.statusElement = null;
    this.onJointAngles = onJointAnglesCallback;  // 回调：将关节角度传给父组件

    this.lastSendTime = 0;
    this.sendInterval = 33; // 30 FPS 节流（更流畅）
    this._sending = false;  // 请求进行中标志

    // FPS 监控
    this.frameCount = 0;
    this.lastFpsTime = 0;
    this.currentFps = 0;

    this._setupUI();
  }

  _setupUI() {
    this.container.innerHTML = `
      <div class="mimic-panel">
        <video class="mimic-video" autoplay playsinline></video>
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
    try {
      this._setStatus("加载 MediaPipe...");
      await this._loadMediaPipe();

      this._setStatus("启动摄像头...");
      await this._startCamera();

      this._setStatus("✅ 就绪 - 对着镜头做手势");
    } catch (err) {
      this._setStatus(`❌ 错误: ${err.message}`);
      console.error("[HandMimic] 启动失败:", err);
    }
  }

  async _loadMediaPipe() {
    // 动态加载 MediaPipe Hands（本地文件）
    if (!window.Hands) {
      await this._loadScript(`${MP_CDN}/hands.js`);
    }
    if (!window.Camera) {
      await this._loadScript(`${MP_CDN}/camera_utils.js`);
    }

    this.hands = new window.Hands({
      locateFile: (file) => `${MP_CDN}/${file}`
    });

    this.hands.setOptions({
      maxNumHands: 1,              // 单手
      modelComplexity: 1,          // 1=full (我们只有这个模型文件)
      minDetectionConfidence: 0.5,
      minTrackingConfidence: 0.5
    });

    this.hands.onResults((results) => this._onResults(results));
  }

  async _startCamera() {
    // 请求摄像头权限（降低分辨率以提高帧率）
    const stream = await navigator.mediaDevices.getUserMedia({
      video: {
        facingMode: "user",     // 前置摄像头
        width: { ideal: 480 },   // 降低分辨率：640→480
        height: { ideal: 360 },  // 降低分辨率：480→360
        frameRate: { ideal: 60, min: 30 }  // 明确要求高帧率
      }
    });

    this.videoElement.srcObject = stream;

    // 等待视频元数据加载
    await new Promise((resolve) => {
      this.videoElement.onloadedmetadata = () => {
        this.videoElement.play();
        resolve();
      };
    });

    // 调整 canvas 尺寸匹配视频
    this.canvasElement.width = this.videoElement.videoWidth;
    this.canvasElement.height = this.videoElement.videoHeight;

    console.log(`[HandMimic] 摄像头分辨率: ${this.videoElement.videoWidth}x${this.videoElement.videoHeight}`);

    // MediaPipe Camera 工具自动处理帧循环
    this.camera = new window.Camera(this.videoElement, {
      onFrame: async () => {
        await this.hands.send({ image: this.videoElement });
      },
      width: this.videoElement.videoWidth,
      height: this.videoElement.videoHeight,
      fps: 60  // 设置目标帧率
    });

    await this.camera.start();
  }

  _onResults(results) {
    // FPS 监控
    this.frameCount++;
    const now = performance.now();
    if (now - this.lastFpsTime >= 1000) {
      this.currentFps = Math.round(this.frameCount * 1000 / (now - this.lastFpsTime));
      console.log(`[HandMimic] MediaPipe FPS: ${this.currentFps}`);
      this.frameCount = 0;
      this.lastFpsTime = now;
    }

    const w = this.canvasElement.width;
    const h = this.canvasElement.height;

    // 清空并绘制视频帧
    this.canvasCtx.save();
    this.canvasCtx.clearRect(0, 0, w, h);
    this.canvasCtx.drawImage(results.image, 0, 0, w, h);

    // 调试：检查是否检测到手
    if (!results.multiHandLandmarks || results.multiHandLandmarks.length === 0) {
      console.log("[HandMimic] 未检测到手");
      this.canvasCtx.restore();
      return;
    }

    if (results.multiHandLandmarks && results.multiHandLandmarks.length > 0) {
      const landmarks = results.multiHandLandmarks[0]; // 用于绘制（屏幕坐标）
      const worldLandmarks = results.multiHandWorldLandmarks?.[0]; // 用于重定向（米制3D坐标）

      // 绘制骨骼线（使用屏幕坐标）
      this._drawConnectors(landmarks);
      // 绘制关键点
      this._drawLandmarks(landmarks);

      // 立即发送到服务器，无节流（让 MediaPipe 和后端直接同步）
      if (worldLandmarks) {
        this._sendToServer(worldLandmarks);
      } else {
        console.warn("[HandMimic] multiHandWorldLandmarks 不可用");
      }
    }

    // 绘制 FPS 到视频右上角
    this._drawFps();

    this.canvasCtx.restore();
  }

  _drawConnectors(landmarks) {
    // MediaPipe Hands 21点连线定义
    const connections = [
      [0,1],[1,2],[2,3],[3,4],           // 拇指
      [0,5],[5,6],[6,7],[7,8],           // 食指
      [0,9],[9,10],[10,11],[11,12],      // 中指
      [0,13],[13,14],[14,15],[15,16],    // 无名指
      [0,17],[17,18],[18,19],[19,20],    // 小指
      [5,9],[9,13],[13,17]               // 掌部横线
    ];

    this.canvasCtx.strokeStyle = "#00ff00";
    this.canvasCtx.lineWidth = 2;

    for (const [i, j] of connections) {
      const pt1 = landmarks[i];
      const pt2 = landmarks[j];
      this.canvasCtx.beginPath();
      this.canvasCtx.moveTo(pt1.x * this.canvasElement.width, pt1.y * this.canvasElement.height);
      this.canvasCtx.lineTo(pt2.x * this.canvasElement.width, pt2.y * this.canvasElement.height);
      this.canvasCtx.stroke();
    }
  }

  _drawLandmarks(landmarks) {
    this.canvasCtx.fillStyle = "#00ff00";
    for (const pt of landmarks) {
      const x = pt.x * this.canvasElement.width;
      const y = pt.y * this.canvasElement.height;
      this.canvasCtx.beginPath();
      this.canvasCtx.arc(x, y, 4, 0, 2 * Math.PI);
      this.canvasCtx.fill();
    }
  }

  _drawFps() {
    const fps = this.currentFps || 0;
    const text = `${fps} FPS`;

    // 绘制位置：右上角，留10px边距
    const x = this.canvasElement.width - 10;
    const y = 30;

    // 设置字体
    this.canvasCtx.font = "bold 24px Arial";
    this.canvasCtx.textAlign = "right";
    this.canvasCtx.textBaseline = "top";

    // 绘制黑色描边（确保在任何背景下都清晰）
    this.canvasCtx.strokeStyle = "#000000";
    this.canvasCtx.lineWidth = 4;
    this.canvasCtx.strokeText(text, x, y);

    // 绘制白色文字
    this.canvasCtx.fillStyle = "#00ff00";  // 绿色，和骨骼点颜色一致
    this.canvasCtx.fillText(text, x, y);
  }

  async _sendToServer(landmarks) {
    // 非阻塞发送：不等待响应，避免阻塞下一帧
    // 如果上一次请求还在进行，跳过本次（避免请求堆积）
    if (this._sending) {
      return;
    }

    this._sending = true;

    try {
      // 转换为服务器期望的格式
      const payload = {
        format: "mediapipe",
        landmarks: landmarks.map(pt => ({
          x: pt.x,
          y: pt.y,
          z: pt.z
        }))
      };

      const response = await fetch("/api/hand/mimic", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload)
      });

      const result = await response.json();

      if (result.ok) {
        this._setStatus(`✅ ${this.currentFps} FPS | ${result.gesture || "执行中"}`);

        // 将关节角度传递给父组件（用于更新 3D 渲染）
        if (this.onJointAngles && result.joint_angles) {
          this.onJointAngles(result.joint_angles);
        }
      } else {
        this._setStatus(`⚠️ ${result.msg || "未识别"}`);
      }
    } catch (err) {
      console.error("[HandMimic] 发送失败:", err);
      this._setStatus(`❌ 网络错误: ${err.message}`);
    } finally {
      this._sending = false;
    }
  }

  stop() {
    if (this.camera) {
      this.camera.stop();
      this.camera = null;
    }

    if (this.videoElement && this.videoElement.srcObject) {
      const tracks = this.videoElement.srcObject.getTracks();
      tracks.forEach(track => track.stop());
      this.videoElement.srcObject = null;
    }

    if (this.hands) {
      this.hands.close();
      this.hands = null;
    }

    this._setStatus("已停止");
  }

  _loadScript(src) {
    return new Promise((resolve, reject) => {
      const script = document.createElement("script");
      script.src = src;
      script.onload = resolve;
      script.onerror = () => reject(new Error(`加载失败: ${src}`));
      document.head.appendChild(script);
    });
  }

  _setStatus(msg) {
    this.statusElement.textContent = msg;
  }
}
