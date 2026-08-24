import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import vm from "node:vm";

const html = await readFile(new URL("../../web/index.html", import.meta.url), "utf8");
const plain = value => JSON.parse(JSON.stringify(value));

function section(start, end) {
  const a = html.indexOf(start);
  const b = html.indexOf(end, a);
  assert.ok(a >= 0 && b > a, `missing source section: ${start}`);
  return html.slice(a, b);
}

const updateHandState = section(
  "function updateHandState(row)",
  "// ===== 左侧功能 tab 栏 =====",
);
const handContext = { console };
vm.createContext(handContext);
vm.runInContext(`
  const calls = [];
  const elements = new Map();
  const $ = id => {
    if (!elements.has(id)) elements.set(id, { textContent: "", style: {} });
    return elements.get(id);
  };
  const PLAYBACK_TIMELINE = "timeline";
  const _hv = { setDriven: rad => calls.push(rad.slice()) };
  let _handPreviewOwns3d = true;
  let _handDirty = true;
  let _handNames = [];
  let _handActRad = null;
  let _packPlaying = null;
  let _playingAction = null;
  const _telFast = { hand: null };
  const _telLast = { hand: {} };
  const renderHandTelem = () => {};
  const setActionPaused = () => {};
  const document = { querySelectorAll: () => [] };
  ${updateHandState}

  updateHandState({ type: "state", rad: [0, 0, 0, 0, 0, 0] });
  updateHandState({ type: "action_step", angle_written: true, ok: true,
                    step: 1, total: 2, playback_mode: "keyframe" });
  updateHandState({ type: "state", rad: [1, 1, 1, 1, 1, 1] });

  _handPreviewOwns3d = true;
  updateHandState({ type: "action_step", angle_written: true, ok: false,
                    step: 2, total: 2, playback_mode: "keyframe" });
  updateHandState({ type: "state", rad: [2, 2, 2, 2, 2, 2] });
  globalThis.result = { calls, dirty: _handDirty, preview: _handPreviewOwns3d };
`, handContext);

assert.deepEqual(plain(handContext.result.calls), [[1, 1, 1, 1, 1, 1]]);
assert.equal(handContext.result.dirty, true, "playback must not claim sliders are synced");
assert.equal(handContext.result.preview, true, "a failed playback write must not take ownership");

const cbOnArmFrame = section("function cbOnArmFrame(row)", "// ---- 手的遥测帧 ----");
const cbOnHandFrame = section("function cbOnHandFrame(row)", "function cbErr(id, msg)");
const comboContext = { console };
vm.createContext(comboContext);
vm.runInContext(`
  const calls = [];
  const $ = () => null;
  const R2D = 180 / Math.PI;
  const ARM_DRIFT_WARN = 0.1;
  const ARM_SET_DEFS = Array.from({ length: 7 }, (_, i) => ({ name: "j" + i, lo: -3, hi: 3 }));
  const HAND_SET_DEFS = Array.from({ length: 6 }, () => ({ lo: 0, hi: 1.5 }));
  const _cv = {
    setArm: rad => calls.push(["arm", rad.slice()]),
    setHand: rad => calls.push(["hand", rad.slice()]),
  };
  let _cbPreviewOwns3d = true;
  let _cbDirty = true;
  let _cbArmLast = null;
  let _cbArmAct = null;
  let _cbHandAct = null;
  let _cbDrift = null;
  let _armLimits = null;
  const _telFast = { cbHand: null };
  const _telLast = { cbHand: {} };
  const comboCameraOwns3d = () => false;
  const applyComboFlags = () => {};
  const renderHandTelem = () => {};
  const cbErr = () => {};
  ${cbOnArmFrame}
  ${cbOnHandFrame}

  cbOnArmFrame({ type: "state", combo: { name: "test" }, rad: [1, 2, 3, 4, 5, 6, 7] });
  cbOnHandFrame({ type: "state", rad: [1, 2, 3, 4, 5, 6] });
  globalThis.result = { calls, dirty: _cbDirty, preview: _cbPreviewOwns3d };
`, comboContext);

assert.deepEqual(plain(comboContext.result.calls), [
  ["arm", [1, 2, 3, 4, 5, 6, 7]],
  ["hand", [1, 2, 3, 4, 5, 6]],
]);
assert.equal(comboContext.result.dirty, true, "combo playback must preserve unsaved edits");
assert.equal(comboContext.result.preview, false, "combo telemetry must own Three.js while playing");

console.log("skill playback Three.js ownership tests passed");
