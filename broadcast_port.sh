#!/bin/bash
# 生成 Windows 端口转发命令，让局域网设备能访问 WSL 里的服务。
#
# 这是 **WSL2 的网络问题**，和上面跑什么协议无关 —— app_web(7860)、
# bridge(9000)、MCP(8000) 都一样需要。WSL2 走 NAT，外部访问不到它的 IP。
#
# 注意:
#   · 同一台 Windows 访问不需要代理,直接用 localhost:<port>(WSL2 自动中继)
#   · 只有**局域网其他设备**才需要下面这些规则
#   · WSL IP 每次 wsl --shutdown 重启后可能变,变了要重设
#   · 想彻底免掉代理: C:\Users\<你>\.wslconfig 写 [wsl2] networkingMode=mirrored
#
# 用法:
#   bash broadcast_port.sh              # 默认 7860 8000 9000
#   bash broadcast_port.sh 8000         # 只要某几个
set -u

PORTS=("$@")
[ ${#PORTS[@]} -eq 0 ] && PORTS=(7860 8000 9000)

WSL_IP=$(ip addr show eth0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
[ -z "$WSL_IP" ] && { echo "✗ 拿不到 WSL IP"; exit 1; }

echo "WSL IP: $WSL_IP"
echo "端口:   ${PORTS[*]}"
echo
echo "本机(WSL)监听情况:"
for p in "${PORTS[@]}"; do
    if ss -tlnp 2>/dev/null | grep -q ":$p "; then
        echo "  ✓ $p 有服务在听"
    else
        echo "  ✗ $p 没服务 —— 转发了也是空的"
    fi
done

echo
echo "════════ 复制到 Windows PowerShell(管理员)执行 ════════"
echo
for p in "${PORTS[@]}"; do
    echo "netsh interface portproxy delete v4tov4 listenport=$p listenaddress=0.0.0.0 2>\$null"
    echo "netsh interface portproxy add v4tov4 listenport=$p listenaddress=0.0.0.0 connectport=$p connectaddress=$WSL_IP"
    echo "New-NetFirewallRule -DisplayName 'WSL $p' -Direction Inbound -LocalPort $p -Protocol TCP -Action Allow -ErrorAction SilentlyContinue | Out-Null"
done
echo 'netsh interface portproxy show all'
echo 'ipconfig | findstr IPv4    # 局域网设备要用这里显示的 Windows IP'
echo
echo "════════════════════════════════════════════════════════"
echo
echo "WSL interop 已关(见 memory),所以脚本不能替你执行 —— 只能手动粘。"
