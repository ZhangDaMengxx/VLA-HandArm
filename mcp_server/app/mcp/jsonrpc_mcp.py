"""真实 MCP 服务器 - JSON-RPC 2.0 手写实现

不依赖 mcp SDK 的复杂生命周期，直接实现 JSON-RPC 2.0 + MCP 方法。
可被 Claude Desktop / MCP Inspector / 任何 MCP 客户端连接。
"""
from fastapi import APIRouter, Request, HTTPException
from pydantic import BaseModel
from typing import Any, Optional

router = APIRouter()
robot = None  # 由 main.py 注入


class JSONRPCRequest(BaseModel):
    jsonrpc: str = "2.0"
    id: Optional[int | str] = None
    method: str
    params: Optional[dict[str, Any]] = None


class JSONRPCResponse(BaseModel):
    jsonrpc: str = "2.0"
    id: Optional[int | str]
    result: Optional[Any] = None
    error: Optional[dict[str, Any]] = None


@router.post("/mcp")
async def mcp_endpoint(request: Request):
    """MCP 协议端点 - JSON-RPC 2.0"""
    try:
        body = await request.json()
        req = JSONRPCRequest(**body)
    except Exception as e:
        return JSONRPCResponse(
            id=None,
            error={"code": -32700, "message": "Parse error", "data": str(e)}
        ).model_dump()

    try:
        if req.method == "initialize":
            result = await handle_initialize(req.params or {})
        elif req.method == "tools/list":
            result = await handle_tools_list(req.params or {})
        elif req.method == "tools/call":
            result = await handle_tools_call(req.params or {})
        else:
            return JSONRPCResponse(
                id=req.id,
                error={"code": -32601, "message": f"Method not found: {req.method}"}
            ).model_dump()

        return JSONRPCResponse(id=req.id, result=result).model_dump()

    except Exception as e:
        return JSONRPCResponse(
            id=req.id,
            error={"code": -32603, "message": "Internal error", "data": str(e)}
        ).model_dump()


async def handle_initialize(params: dict) -> dict:
    """initialize 握手"""
    return {
        "protocolVersion": "2024-11-05",
        "capabilities": {
            "tools": {}
        },
        "serverInfo": {
            "name": "MCP Robot Server",
            "version": "1.0.0"
        }
    }


async def handle_tools_list(params: dict) -> dict:
    """返回工具列表"""
    tools = [
        {
            "name": "hand_set_angles",
            "description": "设置灵巧手 6 个关节角度（弧度，0=张开）。会做拇指-食指可行域检查，"
                         "互顶姿态被拒并说明原因。优先用 hand_gesture 调预设手势。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "angles": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 6,
                        "maxItems": 6,
                        "description": "6 个关节角度（rad，0=张开）: "
                                     "[thumb_yaw, thumb_pitch, index, middle, ring, pinky]"
                    }
                },
                "required": ["angles"]
            }
        },
        {
            "name": "hand_list_gestures",
            "description": "列出可用的灵巧手手势及其含义。调 hand_gesture 前先用这个拿准确的 id。",
            "inputSchema": {"type": "object", "properties": {}}
        },
        {
            "name": "hand_gesture",
            "description": "执行灵巧手预设手势。id 必须来自 hand_list_gestures。"
                         "会先做可行域检查，互顶姿态被拒并说明原因。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "name": {
                        "type": "string",
                        "description": "手势 id，从 hand_list_gestures 获取"
                    }
                },
                "required": ["name"]
            }
        },
        {
            "name": "hand_status",
            "description": "查询灵巧手当前状态（连接状态、关节角度）",
            "inputSchema": {"type": "object", "properties": {}}
        },
        {
            "name": "arm_set_joints",
            "description": "设置机械臂 7 个关节角度（弧度）",
            "inputSchema": {
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
        },
        {
            "name": "arm_status",
            "description": "查询机械臂当前状态",
            "inputSchema": {"type": "object", "properties": {}}
        },
    ]
    return {"tools": tools}


async def handle_tools_call(params: dict) -> dict:
    """执行工具调用"""
    name = params.get("name")
    arguments = params.get("arguments", {})

    try:
        if name == "hand_set_angles":
            result = await robot.hand_set_angles(arguments["angles"])
        elif name == "hand_list_gestures":
            result = await robot.hand_list_gestures()
        elif name == "hand_gesture":
            result = await robot.hand_gesture(arguments["name"])
        elif name == "hand_status":
            result = await robot.hand_status()
        elif name == "arm_set_joints":
            result = await robot.arm_set_joints(arguments["joints"])
        elif name == "arm_status":
            result = await robot.arm_status()
        else:
            return {
                "content": [{"type": "text", "text": f"Unknown tool: {name}"}],
                "isError": True
            }

        return {
            "content": [{"type": "text", "text": str(result)}]
        }
    except ValueError as e:
        # 400/404/409 类错误
        return {
            "content": [{"type": "text", "text": f"Error: {e}"}],
            "isError": True
        }
    except Exception as e:
        return {
            "content": [{"type": "text", "text": f"Internal error: {e}"}],
            "isError": True
        }
