# Git 操作指南

## 仓库边界

| 目录 | 远端 | 用途 |
|------|------|------|
| `/home/zhang123/ros2_ws/lerobotTest` | `ZhangDaMengxx/VLA-HandArm.git` | 本体开发、Web、仿真和数据 |
| `/home/zhang123/ros2_ws/robot-mcp-server` | `ZhangDaMengxx/Moshu-robot-mcp-server.git` | 现行 MCP Server 和 Bridge |
| `/home/zhang123/ros2_ws/src/nero_inspire_ros2` | `ZhangDaMengxx/VLA-HandArm-Ros.git` | ROS2 包 |

三个目录是独立 Git 仓库。提交前先确认当前目录、远端和分支：

```bash
git rev-parse --show-toplevel
git remote -v
git branch --show-current
git status --short
```

不要在文档中记录个人私钥路径、密钥指纹、API Key 或 Token。

## 日常流程

```bash
git status --short
git diff
git add path/to/file
git diff --cached
git commit -m "docs: align documentation with current implementation"
git push origin HEAD
```

只暂存本次任务文件。工作区可能包含他人的未提交改动，不要用 `git add -A`、
`git reset --hard` 或 `git checkout --` 清理它们。

## 获取远端变化

```bash
git fetch origin
git log --oneline --decorate HEAD..origin/main
git pull --rebase origin main
```

执行 rebase 前确认工作区状态。发生冲突时逐文件理解双方修改；不要为了快速通过而覆盖
未知改动。

## 分支与提交

```bash
git switch -c docs/update-current-state
git commit -m "docs: update current deployment boundary"
git push -u origin docs/update-current-state
```

常用类型：`feat`、`fix`、`docs`、`test`、`refactor`、`chore`。提交应说明行为和验证，
不要在提交信息中写凭据或机器私有信息。

## 高风险操作

- 不要对共享 `main` 使用 `git push --force`。
- 需要改写自己的远端分支时优先 `--force-with-lease`，并先确认协作者状态。
- 删除分支、清理历史、撤销已发布提交前先确认影响范围。
- TLS 私钥一旦提交，应先轮换；仅删除当前文件不能消除 Git 历史副本。

## 跨仓同步

驱动文件在 `lerobotTest/sim/` 与 `robot-mcp-server/robot-bridge/sim/` 可能存在复制关系，
但不会自动同步。跨仓修改应：

1. 明确哪个仓库是该功能的权威来源。
2. 使用 `git diff --no-index` 对比共享文件。
3. 分别运行两个仓库的测试。
4. 在两个提交中互相引用原因或提交号。

MCP 路由、部署和心跳以 `robot-mcp-server` 为准；Web、VLA 和完整仿真以本仓为准。

---

**最后核对**：2026-08-14
