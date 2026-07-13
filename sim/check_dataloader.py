"""C 路线验证:LeRobotDataset 能否被"动作分块(ACT 风格)"dataloader 消费。
用 delta_timestamps 让每个样本带一段未来 action(块),再过 torch DataLoader 冒烟。
不需 GPU、不需 ACT 模型本体——只证明数据可喂模仿学习训练管线。
"""
import os
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
from pathlib import Path

import torch
from torch.utils.data import DataLoader
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ROOT = Path("/home/zhang123/ros2_ws/lerobotTest/sim/out/lerobot_ds")
FPS = 30
CHUNK = 16  # 动作块长度(ACT 预测未来 16 步)
dt = {"action": [i / FPS for i in range(CHUNK)]}   # 未来 16 步的 action

ds = LeRobotDataset("local/nero_inspire_handdemo", root=str(ROOT), delta_timestamps=dt)
print("frames:", len(ds))
s = ds[0]
print("单样本:")
print("  observation.state :", tuple(s["observation.state"].shape))
print("  action(块)        :", tuple(s["action"].shape), " 期望 (%d,13)" % CHUNK)
print("  observation.images.ego:", tuple(s["observation.images.ego"].shape))
print("  task:", repr(s["task"]))

dl = DataLoader(ds, batch_size=8, shuffle=True, num_workers=0)
b = next(iter(dl))
print("一个 batch(bs=8):")
print("  action :", tuple(b["action"].shape))
print("  state  :", tuple(b["observation.state"].shape))
print("  image  :", tuple(b["observation.images.ego"].shape))
print("OK: 数据集可被 ACT/Diffusion 风格的模仿学习训练管线消费。")
