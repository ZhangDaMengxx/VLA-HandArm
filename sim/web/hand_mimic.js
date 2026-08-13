// sim/web/hand_mimic.js — 实时摄像头手势控制
// 使用 MediaPipe Hands (WASM) 进行前端推理，不占用服务器 GPU

const MP_CDN = "https://cdn.jsdelivr.net/npm/@mediapipe";

export class HandMimicController {
  constructor(container) {
    this.container = container;
    this.hands = null;
    this.camera = null;
    this.videoElement = null;
    this.canvasElement = null;
    this.canvasCtx = null;
    this.statusElement = null;

    this.lastSendTime = 0;
    this.sendInterval = 100; // 10 FPS 节流

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
    // 动态加载 MediaPipe Hands
    if (!window.Hands) {
      await this._loadScript(`${MP_CDN}/hands@0.4.1675469240/hands.js`);
    }
    if (!window.Camera) {
      await this._loadScript(`${MP_CDN}/camera_utils@0.3.1640029074/camera_utils.js`);
    }

    this.hands = new window.Hands({
      locateFile: (file) => `${MP_CDN}/hands@0.4.1675469240/${file}`
    });

    this.hands.setOptions({
      maxNumHands: 1,              // 单手
      modelComplexity: 1,          // 0=lite, 1=full
      minDetectionConfidence: 0.5,
      minTrackingConfidence: 0.5
    });

    this.hands.onResults((results) => this._onResults(results));
  }

  async _startCamera() {
    // 请求摄像头权限（浏览器会提示用户）
    const stream = await navigator.mediaDevices.getUserMedia({
      video: {
        facingMode: "user",     // 前置摄像头
        width: { ideal: 640 },
        height: { ideal: 480 }
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

    // MediaPipe Camera 工具自动处理帧循环
    this.camera = new window.Camera(this.videoElement, {
      onFrame: async () => {
        await this.hands.send({ image: this.videoElement });
      },
      width: 640,
      height: 480
    });

    await this.camera.start();
  }

  _onResults(results) {
    const w = this.canvasElement.width;
    const h = this.canvasElement.height;

    // 清空并绘制视频帧
    this.canvasCtx.save();
    this.canvasCtx.clearRect(0, 0, w, h);
    this.canvasCtx.drawImage(results.image, 0, 0, w, h);

    if (results.multiHandLandmarks && results.multiHandLandmarks.length > 0) {
      const landmarks = results.multiHandLandmarks[0]; // 取第一只手

      // 绘制骨骼线
      this._drawConnectors(landmarks);
      // 绘制关键点
      this._drawLandmarks(landmarks);

      // 节流发送到服务器（10 FPS）
      const now = Date.now();
      if (now - this.lastSendTime >= this.sendInterval) {
        this.lastSendTime = now;
        this._sendToServer(landmarks);
      }
    }

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

  async _sendToServer(landmarks) {
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
        this._setStatus(`✅ ${result.gesture || "执行中"}`);
      } else {
        this._setStatus(`⚠️ ${result.msg || "未识别"}`);
      }
    } catch (err) {
      console.error("[HandMimic] 发送失败:", err);
      this._setStatus(`❌ 网络错误: ${err.message}`);
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
