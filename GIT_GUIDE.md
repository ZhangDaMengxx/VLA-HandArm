# Git 操作指南 (GIT_GUIDE)

项目的Git配置、日常操作和最佳实践。

---

## 📦 仓库信息

### Python主仓库 (lerobotTest)

| 项目 | 信息 |
|------|------|
| **目录** | `/home/zhang123/ros2_ws/lerobotTest` |
| **远端** | `git@github.com:ZhangDaMengxx/VLA-HandArm.git` |
| **分支** | `main` |
| **访问** | SSH (读写权限正常) |
| **状态** | ✅ 正常（2026-08-10已恢复.git） |

### ROS2仓库 (nero_inspire_ros2)

| 项目 | 信息 |
|------|------|
| **目录** | `/home/zhang123/ros2_ws/src/nero_inspire_ros2` |
| **远端** | `git@github.com:ZhangDaMengxx/VLA-HandArm-Ros.git` (私有) |
| **分支** | `main` |
| **访问** | SSH (读写权限正常) |
| **状态** | ✅ 正常 |

---

## 🔑 SSH配置

### GitHub SSH密钥

```bash
私钥位置: ~/.ssh/id_ed25519_github
公钥指纹: SHA256:WkUy2179NSQfqgnrALUq7G1DAEG9pJ1Q+Ur/EmsWPYY
GitHub账号: ZhangDaMengxx
注释: zhang123@WIN-LVR560DOSUA-wsl
```

### SSH配置文件

`~/.ssh/config` 已配置：
```
Host github.com
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_github
    IdentitiesOnly yes
```

**注意**：`IdentitiesOnly yes` 防止SSH随机挑选密钥导致认证失败。

### 测试SSH连接

```bash
# 测试GitHub连接
ssh -T git@github.com
# 预期输出：Hi ZhangDaMengxx! You've successfully authenticated...

# 查看当前使用的密钥
ssh -v git@github.com 2>&1 | grep "identity file"
```

---

## 🔄 日常Git操作

### 1. 查看状态

```bash
# 查看工作区状态
git status

# 查看简洁状态
git status --short

# 查看分支情况
git branch -v

# 查看远端同步状态
git status -sb
```

### 2. 添加和提交

```bash
# 添加所有变更
git add -A

# 添加指定文件
git add 文件名

# 查看暂存区
git diff --cached

# 提交（附带详细信息）
git commit -m "feat: 功能描述

详细说明：
- 变更1
- 变更2
"

# 修改最近一次提交（未push前）
git commit --amend
```

### 3. 推送

```bash
# 推送到远端
git push origin main

# 首次推送新分支
git push -u origin 新分支名

# 强制推送（⚠️ 危险，确认后再用）
git push --force origin main
```

### 4. 拉取更新

```bash
# 拉取并合并
git pull origin main

# 拉取但不合并（查看变化）
git fetch origin
git log HEAD..origin/main

# 变基拉取（保持线性历史）
git pull --rebase origin main
```

### 5. 查看历史

```bash
# 查看提交历史
git log --oneline -10

# 查看图形历史
git log --graph --oneline --all

# 查看某个文件的历史
git log --follow 文件名

# 查看某次提交的详情
git show commit-hash

# 搜索提交信息
git log --grep="关键词"
```

---

## 🌿 分支管理

### 基本操作

```bash
# 创建新分支
git branch 分支名

# 切换分支
git checkout 分支名

# 创建并切换（推荐）
git checkout -b 分支名

# 查看所有分支
git branch -a

# 删除本地分支
git branch -d 分支名

# 删除远端分支
git push origin --delete 分支名
```

### 推荐工作流

```bash
# 1. 从main创建功能分支
git checkout -b feature/新功能

# 2. 在功能分支开发
git add .
git commit -m "feat: 实现新功能"

# 3. 推送功能分支
git push -u origin feature/新功能

# 4. 在GitHub创建PR
gh pr create --title "添加新功能" --body "详细说明"

# 5. PR合并后，更新main
git checkout main
git pull origin main

# 6. 删除功能分支
git branch -d feature/新功能
```

---

## 📝 提交规范

### 提交信息格式

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Type类型

| 类型 | 说明 | 示例 |
|------|------|------|
| `feat` | 新功能 | feat: 添加手势识别功能 |
| `fix` | Bug修复 | fix: 修复串口通信超时 |
| `refactor` | 重构 | refactor: 重构路径管理 |
| `docs` | 文档 | docs: 更新开发手册 |
| `style` | 代码格式 | style: 统一代码缩进 |
| `test` | 测试 | test: 添加单元测试 |
| `chore` | 构建/工具 | chore: 更新依赖版本 |
| `perf` | 性能优化 | perf: 优化URDF加载速度 |

### 示例

```bash
# 简单提交
git commit -m "fix: 修复Web combo页面显示旧模型"

# 详细提交
git commit -m "feat: 灵巧手URDF迁移到2025-04-18新版

- 更新关节命名（6个驱动关节）
- 同步限位值（SolidWorks导出）
- 批量更新代码（9个文件，50处）

关联: #123"
```

---

## 🔧 常见问题

### 1. 撤销操作

```bash
# 撤销工作区修改（未add）
git checkout -- 文件名

# 撤销暂存（已add未commit）
git reset HEAD 文件名

# 撤销最近一次提交（保留修改）
git reset --soft HEAD^

# 撤销最近一次提交（丢弃修改，⚠️ 危险）
git reset --hard HEAD^

# 撤销某个文件到指定版本
git checkout commit-hash -- 文件名
```

### 2. 处理冲突

```bash
# 拉取时出现冲突
git pull origin main
# 手动编辑冲突文件，保留需要的内容

# 标记为已解决
git add 冲突文件

# 完成合并
git commit

# 放弃合并
git merge --abort
```

### 3. 暂存工作

```bash
# 暂存当前工作
git stash

# 查看暂存列表
git stash list

# 恢复暂存
git stash pop

# 恢复指定暂存
git stash apply stash@{0}

# 删除暂存
git stash drop stash@{0}
```

### 4. 清理仓库

```bash
# 查看未跟踪文件
git clean -n

# 删除未跟踪文件
git clean -f

# 删除未跟踪文件和目录
git clean -fd

# 删除.gitignore中的文件
git clean -fx
```

---

## 🚫 .gitignore 配置

当前项目的 `.gitignore` 已配置忽略：

```gitignore
# 第三方依赖
/dex-retargeting-main/
/unitree_sdk2-main/
...

# 生成物
/outputs/
sim/out/*
*.rrd

# Python
__pycache__/
*.pyc

# 编辑器
.vscode/
.claude/

# 大文件
kinect2_middle/
assets/arm/meshes/nero_RH56DF.stl  # 117M
sim/build_urdf/_cache/              # 缓存
```

---

## 📋 最佳实践

### 1. 提交前检查

```bash
# 检查清单
git status              # 确认要提交的文件
git diff                # 查看具体改动
git diff --cached       # 查看暂存区改动
python3 verify_migration.py  # 运行验证（如果有）
```

### 2. 小步提交

- ✅ 每个提交只做一件事
- ✅ 提交信息清晰描述做了什么
- ✅ 保证每次提交代码可运行
- ❌ 不要把多个功能混在一个提交里

### 3. 分支策略

- **main分支**：稳定版本，只合并经过测试的代码
- **功能分支**：`feature/xxx` 开发新功能
- **修复分支**：`fix/xxx` 修复bug
- **临时分支**：用完即删

### 4. 推送前

```bash
# 1. 确保本地最新
git pull origin main

# 2. 运行测试
python3 -m pytest sim/test_*.py

# 3. 检查冲突
git status

# 4. 推送
git push origin main
```

---

## 🔐 安全注意

### 不要提交的内容

- ❌ SSH私钥
- ❌ 密码、Token
- ❌ `.env` 文件
- ❌ 大文件（>100MB）
- ❌ 临时文件、缓存

### 已经提交了敏感信息？

```bash
# ⚠️ 使用git filter-branch清除历史
# 这会重写历史，需要force push

# 或者使用BFG Repo-Cleaner
# https://reps-cleaner.github.io/
```

---

## 🛠️ 有用的Git命令

### 查看配置

```bash
# 查看所有配置
git config --list

# 查看用户配置
git config user.name
git config user.email

# 设置用户信息
git config user.name "ZhangDaMengxx"
git config user.email "ZhangDaMengxx@users.noreply.github.com"
```

### 别名配置

```bash
# 添加常用别名
git config --global alias.st status
git config --global alias.co checkout
git config --global alias.br branch
git config --global alias.ci commit
git config --global alias.lg "log --graph --oneline --all"

# 使用别名
git st      # = git status
git lg -10  # = git log --graph --oneline --all -10
```

### 查看差异

```bash
# 查看工作区 vs 暂存区
git diff

# 查看暂存区 vs HEAD
git diff --cached

# 查看工作区 vs HEAD
git diff HEAD

# 查看两个提交的差异
git diff commit1 commit2

# 查看文件在两个提交间的差异
git diff commit1 commit2 -- 文件名
```

---

## 📚 参考资源

### GitHub CLI (gh)

```bash
# 安装
# Ubuntu: apt install gh
# 或从 https://cli.github.com/

# 登录
gh auth login

# 创建PR
gh pr create

# 查看PR
gh pr list

# 查看issues
gh issue list
```

### 学习资源

- [Pro Git 书籍](https://git-scm.com/book/zh/v2)
- [GitHub文档](https://docs.github.com/cn)
- [Git官方文档](https://git-scm.com/docs)

---

## ✅ 快速参考

### 最常用命令

```bash
# 每天开始
git pull origin main

# 开发中
git status
git add .
git commit -m "feat: xxx"

# 提交前
git diff
git log -5

# 推送
git push origin main

# 查看历史
git log --oneline --graph -10
```

---

**最后更新**：2026-08-10  
**维护者**：项目团队

**另见**：
- [HANDBOOK.md](HANDBOOK.md) - 开发手册
- [CHANGELOG.md](CHANGELOG.md) - 提交规范示例
- [PROJECT_STATUS.md](PROJECT_STATUS.md) - 项目进度
