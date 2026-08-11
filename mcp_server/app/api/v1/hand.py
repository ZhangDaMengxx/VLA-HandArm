"""灵巧手 HTTP API"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

# 注入全局 robot controller（在 main.py 设置）
router = APIRouter()
robot = None  # 会在 main.py 注入


class HandAngles(BaseModel):
    angles: list[float]


@router.get("/status")
async def status():
    """查询手状态"""
    try:
        return await robot.hand_status()
    except Exception as e:
        raise HTTPException(500, str(e))


@router.post("/angles")
async def set_angles(req: HandAngles):
    """设置关节角度"""
    if len(req.angles) != 6:
        raise HTTPException(400, "Need 6 angles")

    try:
        return await robot.hand_set_angles(req.angles)
    except Exception as e:
        raise HTTPException(500, str(e))


@router.post("/gesture/{name}")
async def execute_gesture(name: str):
    """执行预设手势"""
    try:
        return await robot.hand_gesture(name)
    except Exception as e:
        raise HTTPException(500, str(e))
