"""MCP Server 主入口"""
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .config import config
from .auth import make_auth_middleware
from .robot.controller import RobotController
from .api.v1 import hand, arm
from .mcp import server as mcp_server_rest  # 旧的 REST 端点，保留向后兼容
from .mcp import jsonrpc_mcp  # 真实 MCP 协议 - JSON-RPC 2.0


# ============================================================================
# 全局控制器
# ============================================================================
robot = RobotController(config.robot.bridge_url, config.robot.bridge_token)


# ============================================================================
# 生命周期管理
# ============================================================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    """启动时连接硬件，关闭时断开"""
    import logging
    logging.basicConfig(level=logging.INFO)

    logger = logging.getLogger(__name__)

    if MODE == "public" and not _KEYS:
        raise RuntimeError(
            "security.mode=public 但没配 api_keys —— 这会把能驱动真实硬件的\n"
            "  接口裸暴露在公网。用 MCP_API_KEYS=<key> 传入,或改成 mode=lan。")

    logger.info("安全模式: %s (%s)", MODE,
                f"需 X-API-Key,已配 {len(_KEYS)} 把" if MODE == "public"
                else "不校验 API Key")
    if MODE == "lan":
        logger.warning("lan 模式:局域网内任何人都能驱动硬件。"
                       "放到 NAT 外面(纯 Ubuntu/云主机)前务必改成 public。")

    logger.info(f"🔗 连接硬件代理: {config.robot.bridge_url}")

    await robot.connect()

    yield

    logger.info("🔌 断开硬件代理...")
    await robot.disconnect()


# ============================================================================
# FastAPI 应用
# ============================================================================
app = FastAPI(
    lifespan=lifespan,
    title=config.server.title,
    version="1.0.0-mvp"
)

# ---- 安全分级 ----
# 中间件按**添加的逆序**执行,所以先加 auth、后加 CORS,实际是 CORS 先跑 ——
# 这样浏览器的预检 OPTIONS 不会被 401 挡掉。
MODE = config.resolve_mode()
_KEYS = config.security.api_keys

app.middleware("http")(make_auth_middleware(
    MODE, _KEYS, config.security.public_paths))

app.add_middleware(
    CORSMiddleware,
    # public 模式不能用 * —— 配了凭据的通配来源等于没设限
    allow_origins=config.security.cors_origins if MODE == "public" else ["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # 浏览器端 MCP 客户端要读会话头,不暴露的话拿不到
    expose_headers=["Mcp-Session-Id"],
)

# 注入 robot 到各个路由模块
hand.robot = robot
arm.robot = robot
mcp_server_rest.robot = robot  # 旧 REST 端点
jsonrpc_mcp.robot = robot  # 真 MCP 端点

# 挂载子路由
app.include_router(hand.router, prefix="/api/v1/hand", tags=["Hand"])
app.include_router(arm.router, prefix="/api/v1/arm", tags=["Arm"])
app.include_router(mcp_server_rest.router, prefix="/mcp_rest", tags=["MCP (REST, deprecated)"])
app.include_router(jsonrpc_mcp.router, tags=["MCP"])


# ============================================================================
# 健康检查
# ============================================================================
@app.get("/health")
async def health():
    try:
        hand_st = await robot.hand_status()
        arm_st = await robot.arm_status()
        return {
            "status": "ok",
            "hand": hand_st,
            "arm": arm_st
        }
    except Exception as e:
        return {
            "status": "degraded",
            "error": str(e)
        }


@app.get("/")
async def root():
    return {
        "name": config.server.title,
        "version": "1.0.0",
        "docs": "/docs",
        "mcp": "/mcp",  # 真实 MCP 协议端点
        "mcp_rest_deprecated": "/mcp_rest/tools/list"  # 旧 REST 端点（不兼容标准客户端）
    }
