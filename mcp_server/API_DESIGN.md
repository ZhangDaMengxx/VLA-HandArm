# Web API 设计文档

## 架构概览

### 3层5端点体系

```
┌──────────────────────────────────────────┐
│  Level 1: Symbolic (符号输入)             │
├──────────────────────────────────────────┤
│  POST /api/v1/hand/gesture/{name}        │
│  POST /api/v1/combo/play                 │
└──────────────────────────────────────────┘

┌──────────────────────────────────────────┐
│  Level 2: Numeric (数值输入)              │
├──────────────────────────────────────────┤
│  POST /api/v1/hand/angles                │
│  POST /api/v1/arm/joints                 │
│  POST /api/v1/combo/keyframes            │
└──────────────────────────────────────────┘

┌──────────────────────────────────────────┐
│  Level 3: Perceptual (感知输入)           │
├──────────────────────────────────────────┤
│  POST /api/v1/hand/mimic                 │
└──────────────────────────────────────────┘
```

---

## API 端点详情

### Level 1: 符号输入（Symbolic）

#### 1.1 执行手势

```http
POST /api/v1/hand/gesture/{name}
```

**用途**：执行预定义的手势名称

**示例**：
```bash
curl -X POST http://localhost:8000/api/v1/hand/gesture/hand_thumbs_up
```

**响应**：
```json
{
  "ok": true,
  "gesture": "hand_thumbs_up",
  "angles": [...]
}
```

#### 1.2 执行联合动作

```http
POST /api/v1/combo/play
Content-Type: application/json

{
  "name": "伸手"
}
```

**可用动作**：
- `伸手` - 机械臂向前伸展
- `挥手` / `shake_hand` - 左右摆动招手
- `点赞` - 竖起大拇指
- `三指抓握` - 三指夹持姿态

**响应**：
```json
{
  "ok": true,
  "name": "伸手",
  "description": "机械臂向前伸展",
  "file": "伸手.json"
}
```

---

### Level 2: 数值输入（Numeric）

#### 2.1 设置手部角度

```http
POST /api/v1/hand/angles
Content-Type: application/json

{
  "angles": [0.0, 0.0, 1.5, 1.5, 1.5, 0.0]
}
```

**参数**：6个浮点数（弧度），对应6个手部关节

**响应**：
```json
{
  "ok": true,
  "angles": [0.0, 0.0, 1.5, 1.5, 1.5, 0.0]
}
```

#### 2.2 设置臂部角度

```http
POST /api/v1/arm/joints
Content-Type: application/json

{
  "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}
```

**参数**：7个浮点数（弧度），对应7个机械臂关节

#### 2.3 自定义多帧序列

```http
POST /api/v1/combo/keyframes
Content-Type: application/json

{
  "frames": [
    {
      "arm_rad": [7个角度],
      "hand_rad": [6个角度],
      "t_ns": 0,
      "hold_ms": 800,
      "speed": 500,
      "force": 500
    },
    {
      "arm_rad": [...],
      "hand_rad": [...],
      "t_ns": 1000000000,
      "hold_ms": 500,
      "speed": 500,
      "force": 500
    }
  ]
}
```

---

### Level 3: 感知输入（Perceptual）

#### 3.1 视觉驱动手部控制

```http
POST /api/v1/hand/mimic
Content-Type: application/json

{
  "format": "mediapipe",
  "landmarks": [
    {"x": 0.5, "y": 0.3, "z": 0.1},
    {"x": 0.6, "y": 0.2, "z": 0.05},
    ...  // 21个3D点
  ]
}
```

**支持格式**：
- `mediapipe` - MediaPipe Hands 21个关键点
- `wilor` - WILOR 全身姿态（待实现）

**工作原理**（阶段1）：
1. 识别离散手势（thumbs_up, open, fist, point）
2. 映射到预定义角度
3. 执行对应手势

**未来**（阶段2）：
- 连续角度映射
- 实时跟踪

**响应**：
```json
{
  "ok": true,
  "recognized_gesture": "hand_thumbs_up",
  "angles": [...]
}
```

---

## 查询端点

### 列出可用手势

```http
GET /api/v1/hand/gestures
```

**响应**：
```json
{
  "gestures": [
    {
      "name": "hand_thumbs_up",
      "description": "竖起大拇指"
    },
    ...
  ]
}
```

### 列出可用联合动作

```http
GET /api/v1/combo/list
```

**响应**：
```json
{
  "presets": [
    {
      "name": "伸手",
      "description": "机械臂向前伸展",
      "file": "伸手.json"
    },
    ...
  ]
}
```

### 查询状态

```http
GET /api/v1/hand/status
GET /api/v1/arm/status
GET /health
```

---

## 前端集成示例

### React + TypeScript

```typescript
// api/robot.ts
const API_BASE = "http://localhost:8000/api/v1";

export async function playGesture(name: string) {
  const res = await fetch(`${API_BASE}/hand/gesture/${name}`, {
    method: "POST"
  });
  return res.json();
}

export async function playCombo(name: string) {
  const res = await fetch(`${API_BASE}/combo/play`, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({name})
  });
  return res.json();
}

export async function setHandAngles(angles: number[]) {
  const res = await fetch(`${API_BASE}/hand/angles`, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({angles})
  });
  return res.json();
}

export async function mimicHand(landmarks: any[]) {
  const res = await fetch(`${API_BASE}/hand/mimic`, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({
      format: "mediapipe",
      landmarks
    })
  });
  return res.json();
}
```

### MediaPipe 集成

```typescript
import { Camera } from "@mediapipe/camera_utils";
import { Hands } from "@mediapipe/hands";

function MediaPipeControl() {
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    const hands = new Hands({
      locateFile: (file) => {
        return `https://cdn.jsdelivr.net/npm/@mediapipe/hands/${file}`;
      }
    });

    hands.setOptions({
      maxNumHands: 1,
      modelComplexity: 1,
      minDetectionConfidence: 0.5,
      minTrackingConfidence: 0.5
    });

    hands.onResults((results) => {
      if (results.multiHandLandmarks && results.multiHandLandmarks[0]) {
        const landmarks = results.multiHandLandmarks[0].map(lm => ({
          x: lm.x,
          y: lm.y,
          z: lm.z
        }));
        
        // 发送到机器人
        mimicHand(landmarks).catch(console.error);
      }
    });

    const camera = new Camera(videoRef.current!, {
      onFrame: async () => {
        await hands.send({image: videoRef.current!});
      },
      width: 640,
      height: 480
    });

    camera.start();

    return () => camera.stop();
  }, []);

  return <video ref={videoRef} />;
}
```

---

## 与 MCP 工具的对应关系

| Web API | MCP Tool | 功能 |
|---------|----------|------|
| `POST /api/v1/hand/gesture/{name}` | `set_hand_gesture(name)` | 符号手势 |
| `POST /api/v1/hand/angles` | `set_hand_angles(angles)` | 数值控制 |
| `POST /api/v1/combo/play` | `play_combo(name)` | 预设动作 |
| `POST /api/v1/combo/keyframes` | `play_keyframes(frames)` | 自定义序列 |
| `POST /api/v1/hand/mimic` | `mimic_hand(format, landmarks)` | 视觉驱动 |

**设计原则**：
- Web API 和 MCP Tool 共享相同的业务逻辑层（`RobotController`）
- REST 端点是工具的薄包装
- 保持概念一致，降低学习成本

---

## 安全与认证

### API Key 认证

```bash
curl -H "X-API-Key: your-api-key" \
  http://localhost:8000/api/v1/hand/gesture/hand_thumbs_up
```

### CORS 配置

在 `config.py` 中配置允许的来源：

```python
security:
  cors_origins:
    - "http://localhost:3000"
    - "https://your-frontend.com"
```

---

## 错误处理

### HTTP 状态码

- `200` - 成功
- `400` - 请求参数错误
- `404` - 手势/动作不存在
- `409` - 不可行的姿态
- `500` - 服务器内部错误
- `501` - 功能未实现（如 WILOR）
- `503` - 硬件代理不可用

### 错误响应格式

```json
{
  "detail": "未知手势: invalid_gesture（可用: hand_thumbs_up, hand_open, ...）"
}
```

---

## OpenAPI 文档

启动服务后访问：

```
http://localhost:8000/docs
```

自动生成的交互式 API 文档。
