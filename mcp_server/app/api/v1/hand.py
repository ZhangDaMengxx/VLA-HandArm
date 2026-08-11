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
    """设置关节角度。不可行姿态回 409。"""
    if len(req.angles) != 6:
        raise HTTPException(400, "Need 6 angles")

    try:
        return await robot.hand_set_angles(req.angles)
    except ValueError as e:
        raise HTTPException(409, str(e))
    except Exception as e:
        raise HTTPException(500, str(e))


@router.get("/gestures")
async def list_gestures():
    """列出可用手势"""
    try:
        return await robot.hand_list_gestures()
    except Exception as e:
        raise HTTPException(500, str(e))


@router.post("/gesture/{name}")
async def execute_gesture(name: str):
    """执行预设手势。不可行的姿态回 409,不是 500。"""
    try:
        return await robot.hand_gesture(name)
    except ValueError as e:
        # bridge 判定不可行/找不到,原样透出原因
        raise HTTPException(409, str(e))
    except Exception as e:
        raise HTTPException(500, str(e))
