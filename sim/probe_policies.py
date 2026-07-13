"""列出 lerobot 0.4.4 实际提供的 policy 类型。"""
import importlib
import pkgutil

import lerobot
print("lerobot", getattr(lerobot, "__version__", "?"))

for base in ["lerobot.policies", "lerobot.common.policies"]:
    try:
        m = importlib.import_module(base)
        subs = sorted(x.name for x in pkgutil.iter_modules(m.__path__))
        print(f"{base}: {subs}")
        break
    except Exception as e:
        print("no", base, type(e).__name__)
