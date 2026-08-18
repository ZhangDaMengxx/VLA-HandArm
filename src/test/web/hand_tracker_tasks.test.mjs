import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const sourceUrl = new URL("../../web/hand_tracker_tasks.js", import.meta.url);
const source = await readFile(sourceUrl, "utf8");
const moduleUrl = `data:text/javascript;base64,${Buffer.from(source).toString("base64")}`;
const {
  TasksHandTracker,
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
assert.deepEqual(createOptions.map((options) => options.baseOptions.delegate), ["GPU", "CPU"]);
assert.equal(tasksTracker.detect({}, 10).worldLandmarks.length, 21);
assert.equal(warnings.length, 1);
tasksTracker.close();

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

console.log("hand_tracker_tasks tests passed");
