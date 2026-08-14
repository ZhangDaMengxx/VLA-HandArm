const TASKS_ROOT = "/vendor/mediapipe-tasks";
const TASKS_MODULE_URL = `${TASKS_ROOT}/vision_bundle.mjs`;
const TASKS_WASM_PATH = `${TASKS_ROOT}/wasm`;
const TASKS_MODEL_PATH = `${TASKS_ROOT}/models/hand_landmarker_full.task`;
const LEGACY_ROOT = "/vendor/mediapipe";

function copyPoints(points) {
  if (!points) return null;
  return points.map(({ x, y, z, visibility, presence }) => ({
    x,
    y,
    z,
    ...(visibility === undefined ? {} : { visibility }),
    ...(presence === undefined ? {} : { presence })
  }));
}

function tasksHandedness(result) {
  const category = result.handedness?.[0]?.[0];
  if (!category) return null;
  return {
    label: category.categoryName || category.displayName || "",
    score: category.score ?? 0
  };
}

function legacyHandedness(result) {
  const classification = result.multiHandedness?.[0];
  if (!classification) return null;
  return {
    label: classification.label || "",
    score: classification.score ?? 0
  };
}

export function normalizeTasksResult(result) {
  return {
    landmarks: copyPoints(result.landmarks?.[0]),
    worldLandmarks: copyPoints(result.worldLandmarks?.[0]),
    handedness: tasksHandedness(result)
  };
}

export function normalizeLegacyResult(result) {
  return {
    landmarks: copyPoints(result.multiHandLandmarks?.[0]),
    worldLandmarks: copyPoints(result.multiHandWorldLandmarks?.[0]),
    handedness: legacyHandedness(result)
  };
}

export class TasksHandTracker {
  constructor({ moduleLoader = (url) => import(url), logger = console } = {}) {
    this.moduleLoader = moduleLoader;
    this.logger = logger;
    this.landmarker = null;
    this.delegate = null;
    this.lastTimestamp = -1;
  }

  async initialize() {
    const { FilesetResolver, HandLandmarker } = await this.moduleLoader(TASKS_MODULE_URL);
    const vision = await FilesetResolver.forVisionTasks(TASKS_WASM_PATH);
    const commonOptions = {
      runningMode: "VIDEO",
      numHands: 1,
      minHandDetectionConfidence: 0.5,
      minHandPresenceConfidence: 0.5,
      minTrackingConfidence: 0.5
    };

    try {
      this.landmarker = await HandLandmarker.createFromOptions(vision, {
        ...commonOptions,
        baseOptions: { modelAssetPath: TASKS_MODEL_PATH, delegate: "GPU" }
      });
      this.delegate = "GPU";
    } catch (gpuError) {
      this.logger.warn("[HandTracker] Tasks GPU 初始化失败，回退 CPU:", gpuError);
      this.landmarker = await HandLandmarker.createFromOptions(vision, {
        ...commonOptions,
        baseOptions: { modelAssetPath: TASKS_MODEL_PATH, delegate: "CPU" }
      });
      this.delegate = "CPU";
    }
  }

  detect(video, timestamp) {
    if (!this.landmarker) throw new Error("Tasks HandLandmarker 尚未初始化");
    const monotonicTimestamp = Math.max(Number(timestamp) || 0, this.lastTimestamp + 0.001);
    this.lastTimestamp = monotonicTimestamp;
    return normalizeTasksResult(this.landmarker.detectForVideo(video, monotonicTimestamp));
  }

  close() {
    this.landmarker?.close();
    this.landmarker = null;
  }
}

const scriptLoads = new Map();

function loadScript(src) {
  if (scriptLoads.has(src)) return scriptLoads.get(src);
  const promise = new Promise((resolve, reject) => {
    const script = document.createElement("script");
    script.src = src;
    script.onload = resolve;
    script.onerror = () => reject(new Error(`加载失败: ${src}`));
    document.head.appendChild(script);
  });
  scriptLoads.set(src, promise);
  promise.catch(() => scriptLoads.delete(src));
  return promise;
}

export class LegacyHandTracker {
  constructor({ logger = console } = {}) {
    this.logger = logger;
    this.hands = null;
    this.pending = null;
    this.delegate = "WASM";
  }

  async initialize() {
    if (!window.Hands) await loadScript(`${LEGACY_ROOT}/hands.js`);
    this.hands = new window.Hands({
      locateFile: (file) => `${LEGACY_ROOT}/${file}`
    });
    this.hands.setOptions({
      maxNumHands: 1,
      modelComplexity: 1,
      minDetectionConfidence: 0.5,
      minTrackingConfidence: 0.5
    });
    this.hands.onResults((result) => {
      const pending = this.pending;
      this.pending = null;
      pending?.resolve(normalizeLegacyResult(result));
    });
    await this.hands.initialize?.();
  }

  detect(video) {
    if (!this.hands) throw new Error("Legacy MediaPipe Hands 尚未初始化");
    if (this.pending) throw new Error("Legacy MediaPipe 同时收到多个推理请求");
    return new Promise((resolve, reject) => {
      this.pending = { resolve, reject };
      Promise.resolve(this.hands.send({ image: video })).catch((error) => {
        const pending = this.pending;
        this.pending = null;
        pending?.reject(error);
      });
    });
  }

  close() {
    this.pending?.resolve({ landmarks: null, worldLandmarks: null, handedness: null });
    this.pending = null;
    const closing = this.hands?.close();
    this.hands = null;
    return closing;
  }
}

export function requestedHandEngine(search = window.location.search) {
  return new URLSearchParams(search).get("hand_engine") === "legacy" ? "legacy" : "tasks";
}

export async function initializeHandTracker({
  engine = requestedHandEngine(),
  logger = console,
  tasksOptions = {},
  legacyOptions = {}
} = {}) {
  if (engine === "legacy") {
    const tracker = new LegacyHandTracker({ logger, ...legacyOptions });
    await tracker.initialize();
    return { tracker, engine: "legacy", fallbackReason: null };
  }

  const tasksTracker = new TasksHandTracker({ logger, ...tasksOptions });
  try {
    await tasksTracker.initialize();
    return { tracker: tasksTracker, engine: "tasks", fallbackReason: null };
  } catch (tasksError) {
    await Promise.resolve(tasksTracker.close()).catch(() => {});
    logger.warn("[HandTracker] Tasks 初始化失败，回退 Legacy:", tasksError);
    const legacyTracker = new LegacyHandTracker({ logger, ...legacyOptions });
    await legacyTracker.initialize();
    return { tracker: legacyTracker, engine: "legacy", fallbackReason: tasksError };
  }
}
