"""MCP 协议实现（简化版）

参考: https://modelcontextprotocol.io/specification/2026-07-28
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Any

router = APIRouter()
robot = None  # 注入


class Tool(BaseModel):
    name: str
    description: str
    inputSchema: dict


class ToolCall(BaseModel):
    name: str
    arguments: dict[str, Any]


# ============================================================================
# 工具定义
# ============================================================================
TOOLS = [
    Tool(
        name="hand_set_angles",
        description="设置灵巧手 6 个关节角度（弧度）",
        inputSchema={
            "type": "object",
            "properties": {
                "angles": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 6,
                    "maxItems": 6,
                    "description": "6 个关节角度（rad）: [thumb_yaw, thumb_pitch, index, middle, ring, pinky]"
                }
            },
            "required": ["angles"]
        }
    ),
    Tool(
        name="hand_gesture",
        description="执行灵巧手预设手势",
        inputSchema={
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "enum": ["hand_ok", "hand_pinch", "fist", "open"],
                    "description": "手势名称"
                }
            },
            "required": ["name"]
        }
    ),
    Tool(
        name="hand_status",
        description="查询灵巧手当前状态（连接状态、关节角度）",
        inputSchema={
            "type": "object",
            "properties": {}
        }
    ),
    Tool(
        name="arm_set_joints",
        description="设置机械臂 7 个关节角度（弧度）",
        inputSchema={
            "type": "object",
            "properties": {
                "joints": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 7,
                    "maxItems": 7,
                    "description": "7 个关节角度（rad）"
                }
            },
            "required": ["joints"]
        }
    ),
    Tool(
        name="arm_status",
        description="查询机械臂当前状态",
        inputSchema={
            "type": "object",
            "properties": {}
        }
    ),
]


# ============================================================================
# MCP 端点
# ============================================================================
@router.post("/tools/list")
async def list_tools():
    """列出可用工具"""
    return {"tools": [t.model_dump() for t in TOOLS]}


@router.post("/tools/call")
async def call_tool(call: ToolCall):
    """调用工具"""
    try:
        # 路由到对应的实现
        if call.name == "hand_set_angles":
            result = await robot.hand_set_angles(call.arguments["angles"])
        elif call.name == "hand_gesture":
            result = await robot.hand_gesture(call.arguments["name"])
        elif call.name == "hand_status":
            result = await robot.hand_status()
        elif call.name == "arm_set_joints":
            result = await robot.arm_set_joints(call.arguments["joints"])
        elif call.name == "arm_status":
            result = await robot.arm_status()
        else:
            raise HTTPException(404, f"Tool '{call.name}' not found")

        return {"content": [{"type": "text", "text": str(result)}]}

    except Exception as e:
        return {
            "isError": True,
            "content": [{"type": "text", "text": f"Error: {e}"}]
        }
