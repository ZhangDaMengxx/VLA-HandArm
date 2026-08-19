import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const sourceUrl = new URL("../../web/combo_camera.js", import.meta.url);
let source = await readFile(sourceUrl, "utf8");
source = source.replace(
  /import \{ HandMimicController \} from "\.\/hand_mimic\.js";/,
  "class HandMimicController {}"
);
const moduleUrl = `data:text/javascript;base64,${Buffer.from(source).toString("base64")}`;
const { ComboCameraControl } = await import(moduleUrl);

const button = () => ({
  disabled: false,
  textContent: "",
  classList: { add() {}, remove() {} },
  addEventListener() {},
});
const followBtn = button();
const stateEl = { textContent: "" };
const control = new ComboCameraControl({
  container: { style: {} },
  toggleBtn: button(),
  followBtn,
  stateEl,
  getConfig: () => ({}),
});

control.active = true;
control._renderState();
assert.match(stateEl.textContent, /回放一致/);

assert.equal(followBtn.disabled, true);
control.active = true;
control._handleResponse({ tracking_state: "waiting", ready_to_anchor: true });
assert.equal(followBtn.disabled, false);
assert.equal(followBtn.textContent, "锚定并跟随");

control.toggleFollow();
assert.equal(control._trackingControl, "anchor");
control._handleResponse({
  tracking_state: "anchoring",
  ready_to_anchor: true,
  tracking_control_applied: true,
  anchor_progress: 1,
  anchor_frames: 12,
  stability: { position_error_m: 0.004, orientation_error_deg: 2.5 },
});
assert.equal(control._trackingControl, "none", "anchor must be sent for one frame only");
assert.equal(followBtn.textContent, "取消锚定");
assert.match(stateEl.textContent, /1\/12.*4mm.*2\.5°/);

control._trackingControl = "anchor";
control._handleResponse({ tracking_state: "waiting", ready_to_anchor: false });
assert.equal(control._trackingControl, "anchor", "invalid frame must not consume anchor edge");

control._handleResponse({ tracking_state: "following", ready_to_anchor: true });
control.toggleFollow();
assert.equal(control._trackingControl, "freeze");
control._handleResponse({ tracking_state: "frozen", ready_to_anchor: true });
assert.equal(control._trackingControl, "none", "freeze must be sent for one frame only");
assert.equal(followBtn.textContent, "重新锚定并跟随");

const lifecycle = [];
const runtimeSelect = { disabled: false };
const lifecycleControl = new ComboCameraControl({
  container: { style: {} },
  toggleBtn: button(),
  followBtn: button(),
  stateEl: { textContent: "" },
  getConfig: () => ({}),
  runtimeSelect,
  onBeforeStart: async () => lifecycle.push("prepare-arm"),
  onStop: async () => lifecycle.push("home-arm"),
});
lifecycleControl.mimic = {
  active: false,
  async start() { this.active = true; lifecycle.push("camera-start"); },
  async stop() { this.active = false; lifecycle.push("camera-stop"); },
};
await lifecycleControl.toggleCamera();
assert.deepEqual(lifecycle, ["prepare-arm", "camera-start"]);
assert.equal(runtimeSelect.disabled, true);
await lifecycleControl.stop();
assert.deepEqual(lifecycle, ["prepare-arm", "camera-start", "camera-stop", "home-arm"]);
assert.equal(runtimeSelect.disabled, false);

console.log("combo_camera state tests passed");
