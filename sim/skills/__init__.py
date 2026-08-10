"""技能层:清单(registry.yaml)+ 统一调用接口。

分层意图:
  schema.py       纯 Python,清单加载/校验/参数归一。两个 python 环境共用。
  backend.py      技能 → 动作序列的展开。
  intent.py       文本 → skill_id + 参数(纯 Python,模糊匹配 + 修饰词)。
  runner.py       ROS2 侧执行,复用 sim/ros_joint_writer.py 的限位夹取。
  console_exec.py console 侧执行(arm_console/hand_console),跑在 app_web 进程里。
  asr.py          语音转文本(待建;接上后填进 intent.parse 的入口即可)。

**两条执行路,不能同时用**:runner 走 ROS bridge,console_exec 走两个 console。
同一条 can0 / RS485 两个写者会互相覆盖(见 COMBO_DEBUG.md)。真机验过的是
console 那条,所以语音默认走它;确认闸与调用日志两边共用一份实现,不分叉。

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
