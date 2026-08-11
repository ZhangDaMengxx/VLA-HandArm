#!/bin/bash
# 自动设置 Windows 端口转发（7860 端口）
# 用法: bash broadcast_port.sh

PORT=7860
WSL_IP=$(ip addr show eth0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')

if [ -z "$WSL_IP" ]; then
    echo "✗ 无法获取 WSL IP"
    exit 1
fi

echo "当前 WSL IP: $WSL_IP"
echo "端口: $PORT"
echo "局域网访问: http://192.168.1.189:$PORT"
echo ""

# 生成 PowerShell 命令
PS_CMD="netsh interface portproxy delete v4tov4 listenport=$PORT listenaddress=0.0.0.0; netsh interface portproxy add v4tov4 listenport=$PORT listenaddress=0.0.0.0 connectport=$PORT connectaddress=$WSL_IP; Write-Host '✓ 端口转发已更新'"

echo "=== 方法 1: 自动执行（推荐）==="
if command -v powershell.exe &> /dev/null; then
    echo "正在更新端口转发..."

    # 尝试直接执行（可能需要 UAC）
    powershell.exe -Command "Start-Process powershell -Verb RunAs -WindowStyle Hidden -ArgumentList '-Command', \"$PS_CMD\"" 2>&1

    if [ $? -eq 0 ]; then
        echo "✓ 自动设置完成（如弹出 UAC 请允许）"
        exit 0
    else
        echo "⚠ 自动执行失败，尝试方法 2"
    fi
else
    echo "✗ PowerShell 不可用（WSL interop 可能关闭）"
fi

echo ""
echo "=== 方法 2: 手动执行 ==="
echo "请在 Windows PowerShell（管理员）中执行："
echo ""
echo "netsh interface portproxy delete v4tov4 listenport=$PORT listenaddress=0.0.0.0"
echo "netsh interface portproxy add v4tov4 listenport=$PORT listenaddress=0.0.0.0 connectport=$PORT connectaddress=$WSL_IP"
echo ""
echo "验证: netsh interface portproxy show all"
