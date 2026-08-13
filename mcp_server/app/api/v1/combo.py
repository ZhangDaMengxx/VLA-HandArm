"""臂+手联合动作 HTTP API"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter()
robot = None  # 会在 main.py 注入


class ComboPlayRequest(BaseModel):
    name: str


class ComboKeyframesRequest(BaseModel):
    frames: list[dict]


@router.get("/list")
async def list_combos():
    """列出可用的预设联合动作"""
    try:
        return await robot.list_combos()
    except Exception as e:
        raise HTTPException(500, str(e))


@router.post("/play")
async def play_combo(req: ComboPlayRequest):
    """
    执行预设联合动作

    可用动作：伸手、挥手、点赞、三指抓握等
    """
    try:
        return await robot.play_combo(req.name)
    except ValueError as e:
        raise HTTPException(404, f"未知动作: {e}")
    except Exception as e:
        raise HTTPException(500, str(e))


@router.post("/keyframes")
async def play_keyframes(req: ComboKeyframesRequest):
    """
    执行自定义多帧序列

    frames 格式：
    [
        {
            "arm_rad": [7个角度],
            "hand_rad": [6个角度],
            "t_ns": 时间戳,
            "hold_ms": 保持时间,
            "speed": 速度,
            "force": 力度
        },
        ...
    ]
    """
    try:
        return await robot.play_keyframes(req.frames)
    except Exception as e:
        raise HTTPException(500, str(e))
