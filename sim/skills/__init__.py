"""技能层:清单(registry.yaml)+ 统一调用接口。

分层意图:
  schema.py   纯 Python,清单加载/校验/参数归一。两个 python 环境共用。
  backend.py  技能 → 动作序列的展开(待建)。
  runner.py   ROS2 侧执行,复用 sim/ros_joint_writer.py 的限位夹取(待建)。
  asr.py      语音转文本(待建)。
  intent.py   文本 → skill_id + 槽位(待建)。

上层(Web / 语音 / 未来 VLA)只发统一调用信封:
    {"skill_id": ..., "params": {...}, "source": "web|voice|vla",
     "request_id": ..., "confirmed": bool}
"""
from .schema import (  # noqa: F401
    ParamSpec,
    RegistryError,
    SafetySpec,
    SkillRegistry,
    SkillSpec,
    get_registry,
    load_registry,
)

__all__ = [
    "ParamSpec", "RegistryError", "SafetySpec", "SkillRegistry", "SkillSpec",
    "get_registry", "load_registry",
]
