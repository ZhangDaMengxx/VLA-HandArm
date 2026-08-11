"""机器人控制器 - 调用硬件代理"""
import httpx
from typing import Optional


class RobotController:
    def __init__(self, bridge_url: str):
        self.bridge_url = bridge_url
        self.client: Optional[httpx.AsyncClient] = None

    async def connect(self):
        """启动时连接"""
        self.client = httpx.AsyncClient(base_url=self.bridge_url, timeout=10.0)

        # 测试连接
        resp = await self.client.get("/health")
        if resp.status_code != 200:
            raise RuntimeError(f"硬件代理连接失败: {resp.status_code}")

    async def disconnect(self):
        """关闭时断开"""
        if self.client:
            await self.client.aclose()

    # ========================================================================
    # 灵巧手
    # ========================================================================
    async def hand_status(self):
        """查询手状态"""
        resp = await self.client.get("/hand/status")
        resp.raise_for_status()
        return resp.json()

    async def hand_set_angles(self, angles: list[float]):
        """设置手关节角度"""
        resp = await self.client.post("/hand/angles", json={"angles": angles})
        resp.raise_for_status()
        return resp.json()

    async def hand_gesture(self, name: str):
        """执行手势"""
        resp = await self.client.post(f"/hand/gesture/{name}")
        resp.raise_for_status()
        return resp.json()

    # ========================================================================
    # 机械臂
    # ========================================================================
    async def arm_status(self):
        """查询臂状态"""
        resp = await self.client.get("/arm/status")
        resp.raise_for_status()
        return resp.json()

    async def arm_set_joints(self, joints: list[float]):
        """设置臂关节角度"""
        resp = await self.client.post("/arm/joints", json={"joints": joints})
        resp.raise_for_status()
        return resp.json()
