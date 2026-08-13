"""机器人控制器 - 调用硬件代理"""
import httpx
from typing import Optional
import logging

logger = logging.getLogger(__name__)


class RobotController:
    def __init__(self, bridge_url: str, bridge_token: str = ""):
        self.bridge_url = bridge_url
        self.bridge_token = bridge_token
        self.client: Optional[httpx.AsyncClient] = None
        self._connected = False

    async def connect(self):
        """启动时连接（失败不致命）"""
        try:
            headers = {}
            if self.bridge_token:
                headers["X-Bridge-Token"] = self.bridge_token
            self.client = httpx.AsyncClient(
                base_url=self.bridge_url,
                timeout=10.0,
                headers=headers
            )

            # 测试连接
            resp = await self.client.get("/health")
            if resp.status_code == 200:
                logger.info(f"✓ 硬件代理已连接: {self.bridge_url}")
                self._connected = True
            else:
                logger.warning(f"硬件代理返回 {resp.status_code}，服务降级运行")
        except Exception as e:
            logger.warning(f"硬件代理连接失败: {e}")
            logger.warning("  服务将以降级模式运行")

    async def ensure_connected(self):
        """确保连接可用（懒加载 + 重连）"""
        if self._connected:
            return

        if self.client is None:
            await self.connect()
        else:
            # 尝试重连
            try:
                resp = await self.client.get("/health", timeout=2.0)
                if resp.status_code == 200:
                    self._connected = True
                    logger.info("✓ 重连成功")
            except Exception as e:
                raise RuntimeError(f"硬件代理不可用: {e}")

    async def disconnect(self):
        """关闭时断开"""
        if self.client:
            await self.client.aclose()
            logger.info("硬件代理已断开")

    # ========================================================================
    # 灵巧手
    # ========================================================================
    async def hand_status(self):
        """查询手状态"""
        await self.ensure_connected()
        resp = await self.client.get("/hand/status")
        resp.raise_for_status()
        return resp.json()

    async def hand_set_angles(self, angles: list[float]):
        """设置手关节角度。不可行姿态 bridge 回 409,原因原样透出。

        不转发 allow_infeasible —— 放行开关只给本机标定脚本,不给远端调用方。
        """
        await self.ensure_connected()
        resp = await self.client.post("/hand/angles", json={"angles": angles})
        if resp.status_code in (400, 409):
            raise ValueError(resp.json().get("detail", resp.text))
        resp.raise_for_status()
        return resp.json()

    async def hand_gesture(self, name: str):
        """执行手势。

        不可行的姿态 bridge 会回 409 —— 把它翻成明确的报错,别让
        raise_for_status 的通用消息把"为什么不可行"吃掉。
        """
        await self.ensure_connected()
        resp = await self.client.post(f"/hand/gesture/{name}")
        if resp.status_code in (400, 404, 409):
            detail = resp.json().get("detail", resp.text)
            raise ValueError(detail)
        resp.raise_for_status()
        return resp.json()

    async def hand_list_gestures(self):
        """列出可用手势"""
        await self.ensure_connected()
        resp = await self.client.get("/hand/gestures")
        resp.raise_for_status()
        return resp.json()

    # ========================================================================
    # 机械臂
    # ========================================================================
    async def arm_status(self):
        """查询臂状态"""
        await self.ensure_connected()
        resp = await self.client.get("/arm/status")
        resp.raise_for_status()
        return resp.json()

    async def arm_set_joints(self, joints: list[float]):
        """设置臂关节角度"""
        await self.ensure_connected()
        resp = await self.client.post("/arm/joints", json={"joints": joints})
        resp.raise_for_status()
        return resp.json()

    async def arm_enable(self):
        """使能机械臂"""
        await self.ensure_connected()
        resp = await self.client.post("/arm/enable")
        resp.raise_for_status()
        return resp.json()

    async def arm_disable(self):
        """下使能机械臂"""
        await self.ensure_connected()
        resp = await self.client.post("/arm/disable")
        resp.raise_for_status()
        return resp.json()

    async def arm_estop(self):
        """机械臂急停"""
        await self.ensure_connected()
        resp = await self.client.post("/arm/estop")
        resp.raise_for_status()
        return resp.json()

    async def arm_reset(self):
        """退出急停并重新使能"""
        await self.ensure_connected()
        resp = await self.client.post("/arm/reset")
        resp.raise_for_status()
        return resp.json()

    # ========================================================================
    # Combo（臂+手联合动作）
    # ========================================================================

    async def list_combos(self):
        """列出可用的联合动作（动态扫描磁盘）

        通过 Bridge 的 /combo/list 端点获取实际存在的文件，
        而不是依赖硬编码的映射表。
        """
        await self.ensure_connected()

        try:
            resp = await self.client.get("/combo/list")
            resp.raise_for_status()
            data = resp.json()

            return {
                "presets": [
                    {
                        "name": p["name"],
                        "path": p["path"],
                        "description": p.get("desc", ""),
                        "frames": p.get("frames", 0),
                        "duration_ms": p.get("duration_ms", 0),
                        "mode": p.get("mode", "keyframe")
                    }
                    for p in data.get("packs", []) if not p.get("error")
                ]
            }
        except httpx.HTTPStatusError as e:
            if e.response.status_code == 404:
                # Bridge 端点未实现，返回空列表
                logger.warning("Bridge 未实现 /combo/list 端点")
                return {"presets": []}
            raise

    async def play_combo(self, name: str):
        """执行联合动作（按名称查找）

        动态查找：
        1. 调用 Bridge 的 /combo/play 端点（按名称）
        2. Bridge 负责扫描目录、查找文件、处理重名
        3. 失败时提供可用列表
        """
        await self.ensure_connected()

        try:
            # Bridge 端点支持按名称播放（内部查找文件）
            resp = await self.client.post("/combo/play", json={"name": name})
            resp.raise_for_status()
            return resp.json()

        except httpx.HTTPStatusError as e:
            if e.response.status_code == 404:
                # 未找到，尝试获取可用列表提示用户
                try:
                    combos = await self.list_combos()
                    available = [c["name"] for c in combos["presets"]]
                    if available:
                        raise ValueError(f"未找到动作: {name}（可用: {', '.join(available)}）")
                    else:
                        raise ValueError(f"未找到动作: {name}（无可用动作）")
                except ValueError:
                    raise
                except Exception:
                    raise ValueError(f"未找到动作: {name}")

            elif e.response.status_code == 409:
                # 找到多个同名
                detail = e.response.json().get("detail", "")
                raise ValueError(f"找到多个同名动作: {detail}")

            else:
                raise

    async def play_keyframes(self, frames: list[dict]):
        """执行自定义多帧序列"""
        await self.ensure_connected()
        resp = await self.client.post("/combo/keyframes", json={"frames": frames})
        resp.raise_for_status()
        return resp.json()

    # ========================================================================
    # 视觉驱动（EGO 范式）
    # ========================================================================

    async def mimic_hand(self, format: str, landmarks: list[dict]):
        """根据视觉姿态估计数据控制灵巧手"""
        if format == "mediapipe":
            # 阶段1：识别离散手势
            gesture = self._recognize_mediapipe_gesture(landmarks)
            if gesture:
                logger.info(f"识别到手势: {gesture}")
                return await self.hand_gesture(gesture)

            # 阶段2：连续映射（未来实现）
            # angles = self._mediapipe_to_angles(landmarks)
            # return await self.hand_set_angles(angles)

            raise ValueError("未识别到已知手势，连续映射功能尚未实现")

        elif format == "wilor":
            raise NotImplementedError("WILOR 格式暂未支持")

        else:
            raise ValueError(f"未知格式: {format}（支持: mediapipe, wilor）")

    def _recognize_mediapipe_gesture(self, landmarks: list[dict]) -> str | None:
        """
        从 MediaPipe 关键点识别离散手势

        MediaPipe Hands 21个关键点：
        0: WRIST
        1-4: THUMB (CMC, MCP, IP, TIP)
        5-8: INDEX (MCP, PIP, DIP, TIP)
        9-12: MIDDLE
        13-16: RING
        17-20: PINKY
        """
        if len(landmarks) != 21:
            logger.warning(f"MediaPipe landmarks 应为21个点，实际: {len(landmarks)}")
            return None

        try:
            # 简化版：基于关键点相对位置识别
            wrist = landmarks[0]
            thumb_tip = landmarks[4]
            index_tip = landmarks[8]
            middle_tip = landmarks[12]
            ring_tip = landmarks[16]
            pinky_tip = landmarks[20]

            # 计算手指展开度（指尖到手腕的距离）
            def distance(p1, p2):
                return ((p1['x'] - p2['x'])**2 +
                       (p1['y'] - p2['y'])**2 +
                       (p1.get('z', 0) - p2.get('z', 0))**2) ** 0.5

            thumb_dist = distance(thumb_tip, wrist)
            index_dist = distance(index_tip, wrist)
            middle_dist = distance(middle_tip, wrist)
            ring_dist = distance(ring_tip, wrist)
            pinky_dist = distance(pinky_tip, wrist)

            # 归一化（相对于手腕到中指MCP的距离）
            middle_mcp = landmarks[9]
            hand_size = distance(middle_mcp, wrist)
            if hand_size < 0.01:
                return None

            thumb_norm = thumb_dist / hand_size
            index_norm = index_dist / hand_size
            middle_norm = middle_dist / hand_size
            ring_norm = ring_dist / hand_size
            pinky_norm = pinky_dist / hand_size

            # 手势识别逻辑
            # 所有手指展开 -> "open"
            if all(d > 1.5 for d in [thumb_norm, index_norm, middle_norm, ring_norm, pinky_norm]):
                return "hand_open"

            # 大拇指竖起，其他手指收起 -> "thumbs_up"
            if thumb_norm > 1.3 and all(d < 1.2 for d in [index_norm, middle_norm, ring_norm, pinky_norm]):
                return "hand_thumbs_up"

            # 食指竖起，其他收起 -> "point"
            if index_norm > 1.5 and all(d < 1.2 for d in [thumb_norm, middle_norm, ring_norm, pinky_norm]):
                return "hand_point"

            # 所有手指收起 -> "fist"
            if all(d < 1.2 for d in [thumb_norm, index_norm, middle_norm, ring_norm, pinky_norm]):
                return "hand_fist"

            # 未识别到明确手势
            return None

        except (KeyError, IndexError, TypeError) as e:
            logger.warning(f"MediaPipe 手势识别失败: {e}")
            return None
