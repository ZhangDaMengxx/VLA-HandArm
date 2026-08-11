"""机械臂 HTTP API"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter()
robot = None


class ArmJoints(BaseModel):
    joints: list[float]


@router.get("/status")
async def status():
    """查询臂状态"""
    try:
        return await robot.arm_status()
    except Exception as e:
        raise HTTPException(500, str(e))


@router.post("/joints")
async def set_joints(req: ArmJoints):
    """设置关节角度"""
    if len(req.joints) != 7:
        raise HTTPException(400, "Need 7 joints")

    try:
        return await robot.arm_set_joints(req.joints)
    except Exception as e:
        raise HTTPException(500, str(e))
