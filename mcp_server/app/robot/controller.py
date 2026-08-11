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
