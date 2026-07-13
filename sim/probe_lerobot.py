"""探 LeRobotDataset API(版本/导入路径/create·add_frame·save_episode 签名)。"""
import importlib
import inspect

import lerobot
print("lerobot", getattr(lerobot, "__version__", "?"))

LeRobotDataset = None
for path in ["lerobot.common.datasets.lerobot_dataset",
             "lerobot.datasets.lerobot_dataset",
             "lerobot.datasets"]:
    try:
        m = importlib.import_module(path)
        if hasattr(m, "LeRobotDataset"):
            LeRobotDataset = m.LeRobotDataset
            print("LeRobotDataset in:", path)
            break
    except Exception as e:
        print("no", path, type(e).__name__)

if LeRobotDataset is not None:
    print("create :", inspect.signature(LeRobotDataset.create))
    for me in ["add_frame", "save_episode", "consolidate", "__init__"]:
        if hasattr(LeRobotDataset, me):
            try:
                print(f"{me:14s}:", inspect.signature(getattr(LeRobotDataset, me)))
            except Exception:
                print(me, "exists (no sig)")
else:
    print("LeRobotDataset 未找到")
