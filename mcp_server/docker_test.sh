#!/bin/bash
# Docker 部署测试脚本
# 用法: bash docker_test.sh

set -e

echo "=== Step 1: 验证 Docker 权限 ==="
docker ps || { echo "✗ Docker 权限不足，请先重启 WSL"; exit 1; }
echo "✓ Docker 权限正常"

echo -e "\n=== Step 2: 启动硬件代理 (WSL host) ==="
cd /home/zhang123/ros2_ws/lerobotTest
if pgrep -f "bridge.py" > /dev/null; then
    echo "⚠ bridge.py 已运行"
else
    # 走 start_bridge.sh —— 它锁定带 pyserial 的解释器。裸 `python` 会
    # 因缺 serial 掉进 mock,测出来的"通过"是假的。
    bash start_bridge.sh > /tmp/bridge.log 2>&1 &
    sleep 4
    echo "✓ bridge 已启动"
fi

# 验证 bridge,并把真假模式打出来
H=$(curl -s http://localhost:9000/health) || { echo "✗ bridge 启动失败"; exit 1; }
echo "$H" | grep -q '"status"' || { echo "✗ bridge 无响应: $H"; exit 1; }
echo "✓ bridge 健康检查通过"
echo "  模式: $(echo "$H" | /usr/bin/python3 -c \
  'import json,sys; print(json.load(sys.stdin).get("mode","?"))')"
if echo "$H" | grep -q '"mock": *true'; then
    echo "  ⚠ 当前是 MOCK,后面的测试不会有真实运动"
fi

echo -e "\n=== Step 3: 构建 Docker 镜像 ==="
cd mcp_server
docker compose build
echo "✓ 镜像构建完成"

echo -e "\n=== Step 4: 启动容器 ==="
docker compose up -d
sleep 5
echo "✓ 容器已启动"

echo -e "\n=== Step 5: 验证容器 ==="
docker compose ps
echo ""
docker compose logs --tail=20

echo -e "\n=== Step 6: 测试 API ==="
echo "健康检查:"
curl -s http://localhost:8000/health | python3 -m json.tool || echo "✗ 健康检查失败"

echo -e "\n手状态:"
curl -s http://localhost:8000/api/v1/hand/status | python3 -m json.tool || echo "✗ 状态查询失败"

echo -e "\n=== 测试完成 ==="
echo "查看日志: docker compose logs -f"
echo "停止容器: docker compose down"
