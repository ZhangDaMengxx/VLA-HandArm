"""MCP 协议实现（简化版）

参考: https://modelcontextprotocol.io/specification/2026-07-28
"""
import json

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Any

router = APIRouter()
robot = None  # 注入


class ToolCall(BaseModel):
    name: str
    arguments: dict[str, Any]


# ============================================================================
# 工具定义 —— 从 tools.py 读,不在这里再存一份
# ============================================================================
from .tools import TOOLS  # noqa: E402


# ============================================================================
# MCP 端点
# ============================================================================
@router.post("/tools/list")
async def list_tools():
    """列出可用工具"""
    return {"tools": TOOLS}


@router.post("/tools/call")
async def call_tool(call: ToolCall):
    """调用工具"""
    try:
        # 路由到对应的实现
        if call.name == "hand_set_angles":
            result = await robot.hand_set_angles(call.arguments["angles"])
        elif call.name == "hand_gesture":
            result = await robot.hand_gesture(call.arguments["name"])
        elif call.name == "hand_list_gestures":
            result = await robot.hand_list_gestures()
        elif call.name == "hand_status":
            result = await robot.hand_status()
        elif call.name == "arm_set_joints":
            result = await robot.arm_set_joints(call.arguments["joints"])
        elif call.name == "arm_status":
            result = await robot.arm_status()
        else:
            raise HTTPException(404, f"Tool '{call.name}' not found")

        return {"content": [{"type": "text", "text": json.dumps(result, ensure_ascii=False)}]}

    except Exception as e:
        return {
            "isError": True,
            "content": [{"type": "text", "text": f"Error: {e}"}]
        }
