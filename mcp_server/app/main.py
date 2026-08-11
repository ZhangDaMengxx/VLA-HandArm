"""MCP Server 主入口"""
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .config import config
from .robot.controller import RobotController
from .api.v1 import hand, arm
from .mcp import server as mcp_server


# ============================================================================
# 全局控制器
# ============================================================================
robot = RobotController(config.robot.bridge_url)


# ============================================================================
# 生命周期管理
# ============================================================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    """启动时连接硬件，关闭时断开"""
    import logging
    logging.basicConfig(level=logging.INFO)

    logger = logging.getLogger(__name__)
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

# CORS（局域网宽松，生产版本需要限制）
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 注入 robot 到各个路由模块
hand.robot = robot
arm.robot = robot
mcp_server.robot = robot

# 挂载子路由
app.include_router(hand.router, prefix="/api/v1/hand", tags=["Hand"])
app.include_router(arm.router, prefix="/api/v1/arm", tags=["Arm"])
app.include_router(mcp_server.router, prefix="/mcp", tags=["MCP"])


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
        "version": "1.0.0-mvp",
        "docs": "/docs",
        "mcp": "/mcp/tools/list"
    }
