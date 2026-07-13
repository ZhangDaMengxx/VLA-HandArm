"""批量把 sim/*.py 里写死的 REPO 绝对路径改成自动定位(仓库根 = 脚本父目录的父目录)。"""
from pathlib import Path

HERE = Path(__file__).resolve().parent
OLD = 'REPO = Path("/home/zhang123/ros2_ws/lerobotTest")'
NEW = 'REPO = Path(__file__).resolve().parents[1]'

changed = []
for p in sorted((HERE / "sim").glob("*.py")):
    t = p.read_text(encoding="utf-8")
    if OLD in t:
        p.write_text(t.replace(OLD, NEW), encoding="utf-8")
        changed.append(p.name)
print("REPO 已改为自动定位的文件:", changed)
