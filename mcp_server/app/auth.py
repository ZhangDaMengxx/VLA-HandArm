"""API Key 鉴权中间件。

做成中间件而不是 FastAPI 依赖:MCP 只有单个 POST /mcp 路由,而且将来若挂
ASGI 子应用,依赖注入盖不住,中间件能一并覆盖。

lan 模式不校验(局域网零配置);public 模式强制校验。
"""
import hmac
import logging

from fastapi import Request
from fastapi.responses import JSONResponse

logger = logging.getLogger(__name__)

HEADER = "X-API-Key"


def _key_ok(given: str, allowed: list[str]) -> bool:
    """定长比较,避免用字符串 == 泄露前缀信息。"""
    return any(hmac.compare_digest(given, k) for k in allowed)


def make_auth_middleware(mode: str, api_keys: list[str], public_paths: list[str]):
    """返回 http 中间件。mode != public 时直接放行。"""

    async def auth_middleware(request: Request, call_next):
        if mode != "public" or request.url.path in public_paths:
            return await call_next(request)

        given = request.headers.get(HEADER, "")
        if not given:
            return JSONResponse(
                status_code=401,
                content={"detail": f"缺少 {HEADER} 头。public 模式下所有"
                                   f"控制接口都要鉴权。"},
            )
        if not _key_ok(given, api_keys):
            # 只记来源和路径,不记 key 本身
            logger.warning("鉴权失败: %s %s from %s",
                           request.method, request.url.path,
                           request.client.host if request.client else "?")
            return JSONResponse(status_code=401,
                                content={"detail": f"{HEADER} 无效"})
        return await call_next(request)

    return auth_middleware
