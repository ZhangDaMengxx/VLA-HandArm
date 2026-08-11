#!/bin/bash
# 快速测试 MCP Server

set -e

BASE_URL="http://localhost:8000"

echo "=== 健康检查 ==="
curl -s $BASE_URL/health | python3 -m json.tool

echo -e "\n=== 灵巧手状态 ==="
curl -s $BASE_URL/api/v1/hand/status | python3 -m json.tool

echo -e "\n=== MCP 工具列表 ==="
curl -s -X POST $BASE_URL/mcp/tools/list | python3 -m json.tool

echo -e "\n=== MCP 调用: hand_status ==="
curl -s -X POST $BASE_URL/mcp/tools/call \
  -H "Content-Type: application/json" \
  -d '{"name": "hand_status", "arguments": {}}' | python3 -m json.tool

echo -e "\n=== MCP 调用: hand_set_angles ==="
curl -s -X POST $BASE_URL/mcp/tools/call \
  -H "Content-Type: application/json" \
  -d '{"name": "hand_set_angles", "arguments": {"angles": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}}' | python3 -m json.tool

echo -e "\n✓ 测试完成"
