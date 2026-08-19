import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const sourceUrl = new URL("../../web/hand_tracker_tasks.js", import.meta.url);
const source = await readFile(sourceUrl, "utf8");
const moduleUrl = `data:text/javascript;base64,${Buffer.from(source).toString("base64")}`;
const {
  TasksHandTracker,
  detectHandRuntimeCapabilities,
  initializeHandTracker,
  normalizeLegacyResult,
  normalizeTasksResult,
  requestedHandEngine
} = await import(moduleUrl);

const point = (value) => ({ x: value, y: value + 1, z: value + 2 });
const points = Array.from({ length: 21 }, (_, index) => point(index));

const tasksResult = normalizeTasksResult({
  landmarks: [points],
  worldLandmarks: [points],
  handedness: [[{ categoryName: "Right", score: 0.9 }]]
});
assert.equal(tasksResult.landmarks.length, 21);
assert.equal(tasksResult.worldLandmarks[20].z, 22);
assert.deepEqual(tasksResult.handedness, { label: "Right", score: 0.9 });

const legacyResult = normalizeLegacyResult({
  multiHandLandmarks: [points],
  multiHandWorldLandmarks: [points],
  multiHandedness: [{ label: "Left", score: 0.8 }]
});
assert.equal(legacyResult.landmarks.length, 21);
assert.deepEqual(legacyResult.handedness, { label: "Left", score: 0.8 });
assert.equal(requestedHandEngine("?hand_engine=legacy"), "legacy");
assert.equal(requestedHandEngine("?hand_engine=tasks"), "tasks");
assert.equal(requestedHandEngine(""), "tasks");

const createOptions = [];
const fakeModule = {
  FilesetResolver: { forVisionTasks: async () => ({}) },
  HandLandmarker: {
    createFromOptions: async (_vision, options) => {
      createOptions.push(options);
      if (options.baseOptions.delegate === "GPU") throw new Error("no webgl");
      return {
        detectForVideo: () => ({ landmarks: [points], worldLandmarks: [points] }),
        close() {}
      };
    }
  }
};
const warnings = [];
const tasksTracker = new TasksHandTracker({
  moduleLoader: async () => fakeModule,
  logger: { warn: (...args) => warnings.push(args) }
});
await tasksTracker.initialize();
assert.equal(tasksTracker.delegate, "CPU");
assert.match(tasksTracker.fallbackReason.message, /no webgl/);
assert.deepEqual(createOptions.map((options) => options.baseOptions.delegate), ["GPU", "CPU"]);
assert.equal(tasksTracker.detect({}, 10).worldLandmarks.length, 21);
assert.equal(warnings.length, 1);
tasksTracker.close();

const cpuCreateOptions = [];
const cpuTracker = new TasksHandTracker({
  delegate: "cpu",
  moduleLoader: async () => ({
    FilesetResolver: fakeModule.FilesetResolver,
    HandLandmarker: {
      createFromOptions: async (_vision, options) => {
        cpuCreateOptions.push(options);
        return { detectForVideo() {}, close() {} };
      }
    }
  })
});
await cpuTracker.initialize();
assert.equal(cpuTracker.delegate, "CPU");
assert.deepEqual(cpuCreateOptions.map(o => o.baseOptions.delegate), ["CPU"]);
cpuTracker.close();

const strictGpuOptions = [];
const strictGpu = new TasksHandTracker({
  delegate: "gpu",
  moduleLoader: async () => ({
    FilesetResolver: fakeModule.FilesetResolver,
    HandLandmarker: {
      createFromOptions: async (_vision, options) => {
        strictGpuOptions.push(options);
        throw new Error("gpu unavailable");
      }
    }
  })
});
await assert.rejects(() => strictGpu.initialize(), /gpu unavailable/);
assert.deepEqual(strictGpuOptions.map(o => o.baseOptions.delegate), ["GPU"]);

const fakeGl = {
  RENDERER: 0x1F01,
  getExtension: () => null,
  getParameter: () => "Apple M2",
};
const macCapabilities = detectHandRuntimeCapabilities({
  navigatorObject: { platform: "MacIntel", maxTouchPoints: 0 },
  documentObject: { createElement: () => ({ getContext: name => name === "webgl2" ? fakeGl : null }) },
  webAssembly: { instantiate() {} },
});
assert.equal(macCapabilities.isMac, true);
assert.equal(macCapabilities.renderer, "Apple M2");
assert.deepEqual(macCapabilities.modes.map(mode => mode.value), ["auto", "apple_gpu", "cpu"]);

const cpuOnlyCapabilities = detectHandRuntimeCapabilities({
  navigatorObject: { platform: "Linux x86_64" },
  documentObject: { createElement: () => ({ getContext: () => null }) },
  webAssembly: { instantiate() {} },
});
assert.deepEqual(cpuOnlyCapabilities.modes.map(mode => mode.value), ["auto", "cpu"]);

class FakeHands {
  setOptions() {}
  onResults(callback) { this.callback = callback; }
  async initialize() {}
  close() {}
}
globalThis.window = { Hands: FakeHands, location: { search: "" } };
const fallback = await initializeHandTracker({
  engine: "tasks",
  tasksOptions: { moduleLoader: async () => { throw new Error("missing tasks"); } },
  logger: { warn: (...args) => warnings.push(args) }
});
assert.equal(fallback.engine, "legacy");
assert.match(fallback.fallbackReason.message, /missing tasks/);
fallback.tracker.close();

await assert.rejects(() => initializeHandTracker({
  engine: "tasks",
  delegate: "gpu",
  tasksOptions: { moduleLoader: async () => { throw new Error("strict gpu missing"); } },
  logger: { warn: (...args) => warnings.push(args) }
}), /strict gpu missing/);

console.log("hand_tracker_tasks tests passed");
