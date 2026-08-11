"""真实 MCP 服务器 —— JSON-RPC 2.0 over Streamable HTTP。

手写实现,不依赖 mcp SDK 的生命周期。可被 mcp-remote / MCP Inspector /
Claude Desktop(经 mcp-remote 代理)连接。

规范上容易踩的三个地方,都在下面处理了:

1. **通知不能回响应体**。客户端 initialize 之后立刻发 notifications/initialized,
   它没有 id。JSON-RPC 规定通知无响应;之前这里当成未知 method 回了一个
   id=null 的 error,严格客户端到这步就断连。现在回 202 空 body。

2. **成功响应不能带 error 字段**。规范要求 result / error 二者只出现一个。
   之前用 pydantic model 全字段序列化,成功时也带 "error": null。

3. **Mcp-Session-Id**。initialize 时生成并回在 header;后续请求带上就校验,
   不带也放行 —— 规范允许无状态服务端,而 mcp-remote 等客户端对会话的处理
   不完全一致,严格拒绝会挡掉能用的客户端。

工具定义在 tools.py(单一真源),不在这里。
"""
import json
import logging
import secrets

from fastapi import APIRouter, Request, Response
from fastapi.responses import JSONResponse, StreamingResponse

from .tools import TOOLS

logger = logging.getLogger(__name__)

router = APIRouter()
robot = None  # 由 main.py 注入

# 本服务端实现的协议版本。客户端报更高版本时按这个回 —— 规范要求服务端
# 回自己支持的版本,由客户端决定要不要继续。
PROTOCOL_VERSION = "2024-11-05"

# 活跃会话。进程内内存即可:会话只用来关联同一客户端的多次请求,
# 重启后客户端会重新 initialize。
_sessions: set[str] = set()


def _ok(req_id, result: dict) -> dict:
    """成功响应。**不带 error 字段** —— 见模块 docstring 第 2 条。"""
    return {"jsonrpc": "2.0", "id": req_id, "result": result}


def _err(req_id, code: int, message: str, data: str | None = None) -> dict:
    body: dict = {"code": code, "message": message}
    if data is not None:
        body["data"] = data
    return {"jsonrpc": "2.0", "id": req_id, "error": body}


def _text_result(payload, is_error: bool = False) -> dict:
    """把工具返回值包成 MCP 的 content 数组。

    用 json.dumps 而不是 str() —— str(dict) 出来的是 Python repr
    (单引号、True/False),不是合法 JSON,大模型解析容易出错。
    ensure_ascii=False 保留中文,不然手势名全变成 \\uXXXX,又长又难读。
    """
    if isinstance(payload, str):
        text = payload
    else:
        text = json.dumps(payload, ensure_ascii=False)
    out: dict = {"content": [{"type": "text", "text": text}]}
    if is_error:
        out["isError"] = True
    return out


# ============================================================================
# 方法实现
# ============================================================================
def handle_initialize(params: dict) -> dict:
    client = (params.get("clientInfo") or {}).get("name", "unknown")
    logger.info("MCP initialize ← %s (协议 %s)",
                client, params.get("protocolVersion", "未报"))
    return {
        "protocolVersion": PROTOCOL_VERSION,
        "capabilities": {"tools": {"listChanged": False}},
        "serverInfo": {"name": "robot-mcp-server", "version": "1.0.0"},
    }


async def handle_tools_call(params: dict) -> dict:
    name = params.get("name")
    args = params.get("arguments") or {}

    try:
        if name == "hand_list_gestures":
            result = await robot.hand_list_gestures()
        elif name == "hand_gesture":
            if "name" not in args:
                return _text_result("缺少参数 name（手势 id）", is_error=True)
            result = await robot.hand_gesture(args["name"])
        elif name == "hand_set_angles":
            if "angles" not in args:
                return _text_result("缺少参数 angles（6 个弧度值）", is_error=True)
            result = await robot.hand_set_angles(args["angles"])
        elif name == "hand_status":
            result = await robot.hand_status()
        elif name == "arm_status":
            result = await robot.arm_status()
        elif name == "arm_set_joints":
            if "joints" not in args:
                return _text_result("缺少参数 joints（7 个弧度值）", is_error=True)
            result = await robot.arm_set_joints(args["joints"])
        else:
            return _text_result(f"未知工具: {name}", is_error=True)
    except ValueError as e:
        # bridge 判定不可行 / 找不到手势 —— 原因原样透出,别吞掉
        return _text_result(str(e), is_error=True)
    except Exception as e:
        logger.warning("工具 %s 执行失败: %s", name, e)
        return _text_result(f"执行失败: {e}", is_error=True)

    return _text_result(result)


# ============================================================================
# 端点
# ============================================================================
@router.post("/mcp")
async def mcp_post(request: Request):
    """MCP 主端点。请求 → JSON-RPC 响应;通知 → 202 空 body。"""
    try:
        body = await request.json()
    except Exception as e:
        return JSONResponse(_err(None, -32700, "Parse error", str(e)),
                            status_code=400)

    if not isinstance(body, dict):
        # 批量请求(数组)在 2025-03-26 之后的规范里已移除,明确拒绝而不是静默出错
        return JSONResponse(
            _err(None, -32600, "Invalid Request",
                 "不支持批量请求,请单条发送"), status_code=400)

    method = body.get("method")
    req_id = body.get("id")
    params = body.get("params") or {}

    if not method:
        return JSONResponse(_err(req_id, -32600, "Invalid Request",
                                 "缺少 method"), status_code=400)

    # ---- 通知:无 id。规范要求不返回响应体 —— 见模块 docstring 第 1 条 ----
    if req_id is None:
        if method == "notifications/initialized":
            logger.info("MCP 握手完成")
        else:
            # 未知通知按规范静默忽略,不能回 error
            logger.debug("忽略未知通知: %s", method)
        return Response(status_code=202)

    headers: dict[str, str] = {}

    try:
        if method == "initialize":
            result = handle_initialize(params)
            sid = secrets.token_urlsafe(16)
            _sessions.add(sid)
            headers["Mcp-Session-Id"] = sid
        elif method == "ping":
            result = {}
        elif method == "tools/list":
            result = {"tools": TOOLS}
        elif method == "tools/call":
            result = await handle_tools_call(params)
        else:
            return JSONResponse(
                _err(req_id, -32601, f"Method not found: {method}"))
    except Exception as e:
        logger.exception("处理 %s 时出错", method)
        return JSONResponse(_err(req_id, -32603, "Internal error", str(e)))

    return _respond(request, _ok(req_id, result), headers)


def _respond(request: Request, payload: dict, headers: dict) -> Response:
    """按客户端 Accept 决定回 JSON 还是 SSE。

    Streamable HTTP 允许服务端二选一。只认 text/event-stream 的客户端
    收到 application/json 会报错,所以这里跟着 Accept 走。
    """
    accept = request.headers.get("accept", "")
    if "text/event-stream" in accept and "application/json" not in accept:
        body = f"event: message\ndata: {json.dumps(payload, ensure_ascii=False)}\n\n"
        return Response(content=body, media_type="text/event-stream",
                        headers={**headers, "Cache-Control": "no-cache"})
    return JSONResponse(payload, headers=headers)


@router.get("/mcp")
async def mcp_get(request: Request):
    """服务端 → 客户端的 SSE 流。

    本服务端不主动推消息(没有 sampling / roots / 进度通知),但客户端会开这条
    连接。开着不发东西即可,靠注释行保活防止中间代理超时断开。
    """
    if "text/event-stream" not in request.headers.get("accept", ""):
        return JSONResponse(
            {"detail": "此端点仅支持 text/event-stream"}, status_code=406)

    async def keepalive():
        import asyncio
        yield ": connected\n\n"
        while True:
            if await request.is_disconnected():
                break
            await asyncio.sleep(15)
            yield ": keepalive\n\n"

    return StreamingResponse(keepalive(), media_type="text/event-stream",
                             headers={"Cache-Control": "no-cache",
                                      "X-Accel-Buffering": "no"})


@router.delete("/mcp")
async def mcp_delete(request: Request):
    """客户端显式结束会话。"""
    sid = request.headers.get("mcp-session-id")
    if sid:
        _sessions.discard(sid)
    return Response(status_code=204)
