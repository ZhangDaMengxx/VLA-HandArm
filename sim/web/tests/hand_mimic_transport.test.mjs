import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const sourceUrl = new URL("../hand_mimic.js", import.meta.url);
let source = await readFile(sourceUrl, "utf8");
source = source.replace(
  /import \{ initializeHandTracker, requestedHandEngine \} from "\.\/hand_tracker_tasks\.js";/,
  "const initializeHandTracker = async () => ({}); const requestedHandEngine = () => 'tasks';"
);
const moduleUrl = `data:text/javascript;base64,${Buffer.from(source).toString("base64")}`;
const { HandMimicController } = await import(moduleUrl);

globalThis.WebSocket = { OPEN: 1 };

const sent = [];
const socket = {
  readyState: WebSocket.OPEN,
  send(value) { sent.push(JSON.parse(value)); }
};
const controller = Object.create(HandMimicController.prototype);
Object.assign(controller, {
  active: true,
  ws: socket,
  transportInFlight: false,
  transportType: null,
  inFlightPayload: null,
  pendingPayload: null,
  _handleServerResponse() {}
});

const payload = (id) => ({ format: "mediapipe", id, landmarks: [] });
controller._queuePayload(payload(1));
controller._queuePayload(payload(2));
controller._queuePayload(payload(3));

assert.deepEqual(sent.map((item) => item.id), [1]);
assert.equal(controller.transportInFlight, true);
assert.equal(controller.pendingPayload.id, 3);

controller._handleWebSocketMessage(socket, { data: JSON.stringify({ ok: true }) });
assert.deepEqual(sent.map((item) => item.id), [1, 3]);
assert.equal(controller.pendingPayload, null);
assert.equal(controller.transportInFlight, true);

const originalWarn = console.warn;
console.warn = () => {};
controller._handleWebSocketMessage(socket, { data: "not-json" });
console.warn = originalWarn;
assert.equal(controller.transportInFlight, false);
assert.equal(controller.pendingPayload, null);

console.log("hand_mimic transport tests passed");
