#!/bin/bash
# 在云服务器上运行，创建用户下载页面

set -e

echo "创建用户下载目录..."
mkdir -p ~/mcp-server/public

# 复制用户需要的文件
cp DEPLOY.md ~/mcp-server/public/USER_GUIDE.md

# 创建简单的下载页面
cat > ~/mcp-server/public/index.html << 'EOF'
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>机器人 MCP - 用户安装指南</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 800px;
            margin: 50px auto;
            padding: 20px;
            line-height: 1.6;
        }
        h1 { color: #333; }
        .download-box {
            background: #f4f4f4;
            padding: 20px;
            border-radius: 8px;
            margin: 20px 0;
        }
        .btn {
            display: inline-block;
            padding: 10px 20px;
            background: #007bff;
            color: white;
            text-decoration: none;
            border-radius: 5px;
            margin: 10px 10px 10px 0;
        }
        .btn:hover { background: #0056b3; }
        code {
            background: #f4f4f4;
            padding: 2px 6px;
            border-radius: 3px;
        }
    </style>
</head>
<body>
    <h1>🤖 机器人 MCP 服务 - 用户安装指南</h1>

    <div class="download-box">
        <h2>📥 下载安装包</h2>
        <a href="USER_GUIDE.md" class="btn" download>📘 下载用户手册</a>
        <p>下载后按照手册操作即可。</p>
    </div>

    <h2>快速开始</h2>
    <ol>
        <li>下载上面的用户手册</li>
        <li>按照手册准备硬件连接（串口/CAN）</li>
        <li>启动 bridge 程序</li>
        <li>开启隧道（cloudflared）</li>
        <li>将隧道 URL 和 token 发送给管理员</li>
        <li>配置 Claude Desktop</li>
    </ol>

    <h2>需要帮助？</h2>
    <p>遇到问题请联系管理员或查看完整文档。</p>
</body>
</html>
EOF

echo "✓ 下载页面已创建"
echo ""
echo "下一步："
echo "1. 用 nginx 或 Python 提供静态文件服务"
echo "2. 或者直接用 docker compose 添加一个静态文件服务"
echo ""
echo "示例（简单 HTTP 服务器）："
echo "  cd ~/mcp-server/public"
echo "  python3 -m http.server 8080"
echo ""
echo "用户访问: http://your-server-ip:8080"
