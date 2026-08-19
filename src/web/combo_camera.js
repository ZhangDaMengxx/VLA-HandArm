import { HandMimicController } from "./hand_mimic.js";

export class ComboCameraControl {
  constructor({ container, toggleBtn, followBtn, stateEl, getConfig,
                onBeforeStart, onResponse, onStop }) {
    this.container = container;
    this.toggleBtn = toggleBtn;
    this.followBtn = followBtn;
    this.stateEl = stateEl;
    this.getConfig = getConfig;
    this.onBeforeStart = onBeforeStart;
    this.onResponse = onResponse;
    this.onStop = onStop;
    this.mimic = null;
    this.active = false;
    this.trackingState = "waiting";
    this.readyToAnchor = false;
    this.anchorProgress = 0;
    this.anchorFrames = 12;
    this.stability = null;
    this.lastError = null;
    this._trackingControl = "none";
    this.orientationDelta = null;
    this.orientationLimitedAxes = null;
    this._bindEvents();
    this._renderState();
  }

  _bindEvents() {
    this.toggleBtn?.addEventListener("click", () => this.toggleCamera());
    this.followBtn?.addEventListener("click", () => this.toggleFollow());
  }

  async toggleCamera() {
    if (this.active) return this.stop();
    this.toggleBtn.disabled = true;
    this.toggleBtn.textContent = "启动中...";
    try {
      await this.onBeforeStart?.();
      this.container.style.display = "block";
      this.mimic ||= new HandMimicController(this.container, null, {
        getTrackingPayload: () => ({
          tracking_mode: "combo",
          tracking_control: this._trackingControl,
          drive_hand: Boolean(this.getConfig()?.driveHand),
          drive_arm: Boolean(this.getConfig()?.driveArm),
          allow_real_arm_tracking: Boolean(this.getConfig()?.allowRealArmTracking),
        }),
        onServerResponse: (result) => this._handleResponse(result),
      });
      await this.mimic.start();
      this.active = true;
      this._trackingControl = "none";
      this.toggleBtn.textContent = "关闭摄像头";
      this.toggleBtn.classList.add("active");
      this._renderState();
    } catch (error) {
      let rollbackError = null;
      try {
        await this.onStop?.({ startFailed: true });
      } catch (stopError) {
        rollbackError = stopError;
      }
      this.toggleBtn.textContent = "启动失败";
      this.container.style.display = "none";
      this._setState(
        `摄像头启动失败: ${error.message}`
        + (rollbackError ? ` · 设备回安全位失败: ${rollbackError.message}` : "")
      );
    } finally {
      this.toggleBtn.disabled = false;
    }
  }

  async stop() {
    this.toggleBtn.disabled = true;
    this.toggleBtn.textContent = "关闭中...";
    this._trackingControl = "freeze";
    if (this.mimic) await this.mimic.stop().catch(() => {});
    this.container.style.display = "none";
    this.active = false;
    this.trackingState = "waiting";
    this.readyToAnchor = false;
    this.anchorProgress = 0;
    this.stability = null;
    this.lastError = null;
    this.orientationDelta = null;
    this.orientationLimitedAxes = null;
    let stopError = null;
    try {
      await this.onStop?.({ startFailed: false });
    } catch (error) {
      stopError = error;
    } finally {
      this.toggleBtn.textContent = "启动摄像头";
      this.toggleBtn.classList.remove("active");
      this.toggleBtn.disabled = false;
      this._trackingControl = "none";
      this._renderState();
      if (stopError) this._setState(`摄像头已关闭 · ${stopError.message}`);
    }
  }

  toggleFollow() {
    if (!this.active) return;
    if (this.trackingState === "following" || this.trackingState === "anchoring") {
      this._trackingControl = "freeze";
      this.trackingState = "frozen";
    } else {
      this._trackingControl = "anchor";
      this.trackingState = "anchoring";
    }
    this._renderState();
  }

  _handleResponse(result) {
    if (result?.tracking_state) this.trackingState = result.tracking_state;
    if (Array.isArray(result?.orientation_delta_deg)) {
      this.orientationDelta = result.orientation_delta_deg;
    }
    if (Array.isArray(result?.orientation_limited_axes)) {
      this.orientationLimitedAxes = result.orientation_limited_axes;
    }
    this.readyToAnchor = Boolean(result?.ready_to_anchor);
    this.anchorProgress = Number(result?.anchor_progress) || 0;
    this.anchorFrames = Number(result?.anchor_frames) || 12;
    this.stability = result?.stability || null;
    this.lastError = result?.ok === false ? (result.msg || "跟随处理失败") : null;
    // Keep an anchor edge pending across missing/invalid frames. Clearing it on
    // any response could consume the click before a valid hand frame arrived.
    const anchorApplied = this._trackingControl === "anchor"
      && (result?.tracking_control_applied
        || this.trackingState === "anchoring"
        || this.trackingState === "following");
    const freezeApplied = this._trackingControl === "freeze"
      && (result?.tracking_control_applied || this.trackingState === "frozen");
    if (anchorApplied || freezeApplied) this._trackingControl = "none";
    this._renderState();
    this.onResponse?.(result);
  }

  _renderState() {
    if (this.followBtn) {
      const labels = {
        waiting: this.readyToAnchor ? "锚定并跟随" : "等待检测到手部",
        anchoring: "取消锚定",
        following: "冻结跟随",
        frozen: this.readyToAnchor ? "重新锚定并跟随" : "等待检测到手部",
        failed: "故障已冻结",
      };
      this.followBtn.textContent = this.active
        ? (labels[this.trackingState] || "锚定并跟随")
        : "启动摄像头后可锚定";
      this.followBtn.disabled = !this.active
        || ((this.trackingState === "waiting" || this.trackingState === "frozen")
          && !this.readyToAnchor);
    }
    if (!this.active) return this._setState("摄像头已关闭");
    if (this.lastError) return this._setState(this.lastError);
    if (this.trackingState === "anchoring") {
      const quality = this.stability;
      const detail = quality?.position_error_m != null
        ? ` · ${(quality.position_error_m * 1000).toFixed(0)}mm / ${quality.orientation_error_deg.toFixed(1)}°`
        : "";
      return this._setState(
        `正在联合锚定 ${this.anchorProgress}/${this.anchorFrames}${detail}`
      );
    }
    const labels = {
      waiting: this.readyToAnchor ? "已检测到手部，可锚定" : "等待检测到手部",
      following: "正在跟随",
      frozen: this.readyToAnchor ? "已冻结，可重新锚定" : "已冻结，等待检测到手部",
      failed: "故障已冻结",
    };
    const orientationLabel = this.orientationDelta
      ? ` · 姿态 ${this.orientationDelta.map((value) => `${Number(value).toFixed(1)}°`).join("/")}`
        + (this.orientationLimitedAxes?.some(Boolean) ? " · 已触顶" : "")
      : "";
    this._setState(`${labels[this.trackingState] || this.trackingState} · 回放一致${orientationLabel}`);
  }

  _setState(text) {
    if (this.stateEl) this.stateEl.textContent = text;
  }
}
