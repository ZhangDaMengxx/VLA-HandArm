// src/web/hand_camera.js — 灵巧手页面的实时摄像头控制
// 复用 hand_mimic.js 的 MediaPipe 推理，但更新灵巧手页面的 3D 渲染器

import { HandMimicController } from "./hand_mimic.js";

export class HandCameraControl {
  constructor() {
    this.container = document.getElementById("handCameraContainer");
    this.toggleBtn = document.getElementById("handCameraToggle");
    this.mimicController = null;
    this.isActive = false;
    this.handViewer = null;  // 灵巧手 3D 查看器（由外部设置）
    this.isHardwareConnectedFn = null;  // 硬件连接状态检查函数

    this._bindEvents();
  }

  _bindEvents() {
    this.toggleBtn.addEventListener("click", () => this._toggle());
  }

  setHandViewer(viewer) {
    this.handViewer = viewer;
  }

  /**
   * 设置硬件连接状态检查函数
   * @param {Function} checkFn - 返回 true/false 的函数
   */
  setHardwareConnectedCheck(checkFn) {
    this.isHardwareConnectedFn = checkFn;
    this.mimicController?.setHardwareDriveCheck(checkFn);
  }

  async _toggle() {
    if (!this.isActive) {
      await this._start();
    } else {
      this._stop();
    }
  }

  async _start() {
    try {
      this.toggleBtn.disabled = true;
      this.toggleBtn.textContent = "启动中...";

      // 显示容器
      this.container.style.display = "block";

      // 创建摄像头控制器
      if (!this.mimicController) {
        this.mimicController = new HandMimicController(
          this.container,
          (jointAngles) => this._updateHand3D(jointAngles)
        );
        this.mimicController.setHardwareDriveCheck(this.isHardwareConnectedFn);
      }

      await this.mimicController.start();

      this.isActive = true;
      this.toggleBtn.textContent = "关闭摄像头";
      this.toggleBtn.classList.add("active");
      this.toggleBtn.disabled = false;
    } catch (err) {
      console.error("[HandCamera] 启动失败:", err);
      this.toggleBtn.textContent = "启动失败";
      this.toggleBtn.disabled = false;
      this.container.style.display = "none";
      alert(`摄像头启动失败: ${err.message}\n\n请确保：\n1. 使用 HTTPS 访问\n2. 已允许浏览器摄像头权限\n3. 摄像头未被其他应用占用`);
    }
  }

  _stop() {
    if (this.mimicController) {
      this.mimicController.stop().catch((err) => {
        console.warn("[HandCamera] 清理摄像头失败:", err);
      });
    }

    this.container.style.display = "none";
    this.isActive = false;
    this.toggleBtn.textContent = "启动摄像头";
    this.toggleBtn.classList.remove("active");
  }

  getMetrics() {
    return this.mimicController?.getMetrics() || null;
  }

  /**
   * 更新灵巧手 3D 渲染
   * @param {Object} jointAngles - 6 个关节的弧度值
   */
  _updateHand3D(jointAngles) {
    const t0 = performance.now();

    // 提取 6 个关节角度
    const jointNames = [
      "right_thumb_1_joint",
      "right_thumb_2_joint",
      "right_index_1_joint",
      "right_middle_1_joint",
      "right_ring_1_joint",
      "right_little_1_joint"
    ];

    const angles = jointNames.map(name => jointAngles[name] || 0);

    // 1. 更新 3D 预览（总是执行）
    if (this.handViewer && this.handViewer.ready) {
      this.handViewer.setJoints(angles);
    }

    console.log(`[HandCamera] 3D更新 ${(performance.now()-t0).toFixed(1)}ms`);
  }
}
