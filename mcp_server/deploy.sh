#!/bin/bash
# 快速部署脚本 - 在云服务器上运行

set -e

echo "=== MCP Server 部署脚本 ==="
echo ""

# 检查必要的工具
command -v python3 >/dev/null 2>&1 || { echo "需要 Python 3"; exit 1; }
command -v pip >/dev/null 2>&1 || { echo "需要 pip"; exit 1; }

# 安装依赖
echo "📦 安装依赖..."
pip install -q fastapi uvicorn[standard] httpx pydantic pyyaml mcp

# 检查环境变量
if [ -z "$ROBOT_BRIDGE_URL" ]; then
    echo "⚠️  ROBOT_BRIDGE_URL 未设置，使用默认值"
    export ROBOT_BRIDGE_URL="http://localhost:9000"
fi

if [ -z "$MCP_SECURITY_MODE" ]; then
    echo "⚠️  MCP_SECURITY_MODE 未设置，设为 public（云端必须）"
    export MCP_SECURITY_MODE="public"
fi

if [ -z "$MCP_API_KEYS" ] && [ "$MCP_SECURITY_MODE" = "public" ]; then
    echo "❌ public 模式必须设置 MCP_API_KEYS"
    echo "   生成示例: python3 -c \"import secrets; print(secrets.token_urlsafe(32))\""
    exit 1
fi

# 显示配置
echo ""
echo "📋 当前配置:"
echo "  ROBOT_BRIDGE_URL: $ROBOT_BRIDGE_URL"
echo "  ROBOT_BRIDGE_TOKEN: ${ROBOT_BRIDGE_TOKEN:0:8}... (${#ROBOT_BRIDGE_TOKEN} 字符)"
echo "  MCP_SECURITY_MODE: $MCP_SECURITY_MODE"
echo "  MCP_API_KEYS: ${#MCP_API_KEYS} 个字符"
echo ""

# 启动服务
echo "🚀 启动 MCP Server..."
echo "   访问: http://0.0.0.0:8000"
echo "   健康检查: http://0.0.0.0:8000/health"
echo "   文档: http://0.0.0.0:8000/docs"
echo ""
echo "按 Ctrl+C 停止"
echo ""

cd "$(dirname "$0")"
uvicorn app.main:app --host 0.0.0.0 --port 8000
