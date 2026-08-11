#!/usr/bin/env python3
"""硬件代理 - 在 WSL host 直接运行，暴露简单 HTTP 接口供容器调用

启动: python bridge.py --host 0.0.0.0 --port 9000

为什么需要: Docker 容器访问 WSL USB 设备很麻烦，用代理解耦。
"""
import argparse
import sys
from pathlib import Path

# sim: inspire_hand 在那里。sim/skills: schema / hand_pose 在那里。
# 注意 skills 必须排在 sim 前面 —— console_exec:350 提到过反过来会让
# sim/ 下的同名模块遮蔽 skills/schema.py。
sys.path.insert(0, str(Path(__file__).parent / "sim"))
sys.path.insert(0, str(Path(__file__).parent / "sim/skills"))

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn

app = FastAPI(title="Robot Hardware Bridge")

# ============================================================================
# 全局控制器（启动时初始化）
# ============================================================================
hand = None
arm = None


class HandAngles(BaseModel):
    angles: list[float]  # 6 个弧度值
    # 显式放行不可行姿态。只给标定/调试脚本用,MCP 工具不暴露这个字段 ——
    # 默认必须挡住,否则大模型能绕开手势那条路上的可行域闸。
    allow_infeasible: bool = False


class ArmJoints(BaseModel):
    joints: list[float]  # 7 个弧度值


# ============================================================================
# 灵巧手端点
# ============================================================================
@app.get("/hand/status")
async def hand_status():
    """查询手状态"""
    if hand is None:
        raise HTTPException(503, "Hand not connected")

    try:
        from inspire_hand import HAND_JOINTS
        angles = hand.read_angles()
        return {
            "connected": True,
            "angles": angles,
            "joints": dict(zip(HAND_JOINTS, angles))
        }
    except Exception as e:
        raise HTTPException(500, f"Read failed: {e}")


@app.post("/hand/angles")
async def hand_set_angles(req: HandAngles):
    """设置手关节角度（弧度）"""
    if hand is None:
        raise HTTPException(503, "Hand not connected")

    if len(req.angles) != 6:
        raise HTTPException(400, "Need 6 angles")

    # 可行域闸 —— 和手势那条路同一判据。放在 try 外面,不然下面的
    # except Exception 会把 409 吞成 500。
    import hand_pose as hp
    why = hp.check_feasible(req.angles)
    if why is not None and not req.allow_infeasible:
        raise HTTPException(409, f"姿态不可行,已拒绝下发: {why}")

    try:
        ok = hand.set_angles(req.angles)
    except Exception as e:
        raise HTTPException(500, f"Set angles failed: {e}")
    if not ok:
        raise HTTPException(500, "写 ANGLE_SET 失败")
    return {"ok": True, "feasible_warning": why}


_registry = None


def _get_registry():
    """拿加载好的技能表(registry.yaml + gestures.yaml 合成后)。

    走 schema.load_registry 而不是自己读 yaml —— 造型手势的真源是
    gestures.yaml,加载期才合成成 id=hand_<key> 的 primitive,而且
    `action.hand` 的弧度也是加载期展开好的。schema.py:148 明确要求
    执行侧只读 action.hand,不要在执行路径上二次展开 pose。
    """
    global _registry
    if _registry is None:
        import schema
        _registry = schema.load_registry()
    return _registry


@app.get("/hand/gestures")
async def hand_list_gestures():
    """列出可一步到位执行的手势。

    过滤掉 `_` 开头的 —— 那些是 composite 的内部积木,不给外部调。
    只列有 action.hand 的:trajectory 要走全帧预检、composite 要走
    runner 编排,都不在这个端点的职责里。
    """
    try:
        reg = _get_registry()
    except Exception as e:
        raise HTTPException(500, f"技能表加载失败: {e}")

    out = []
    for s in reg:
        if s.id.startswith("_"):
            continue
        if not (s.action and "hand" in s.action):
            continue
        out.append({
            "id": s.id,
            "name": s.name,
            "desc": s.desc,
            "aliases": list(s.aliases),
            "pose": s.pose,
            "need_confirm": s.safety.need_confirm,
        })
    return {"gestures": out, "count": len(out)}


@app.post("/hand/gesture/{name}")
async def hand_gesture(name: str):
    """执行预设手势。

    走 skills/hand_pose 的 resolve + check_feasible —— 和清单校验、console
    同一套判据。可行域不过 = 409,不下发(命令能"做出来"是靠堵转,不是姿态安全)。
    """
    if hand is None:
        raise HTTPException(503, "Hand not connected")

    try:
        reg = _get_registry()
    except Exception as e:
        raise HTTPException(500, f"技能表加载失败: {e}")

    # 先按 id 查,查不到再按别名(中文名也能命中)
    spec = reg.get(name) or reg.by_alias(name)
    if spec is None:
        raise HTTPException(404, f"手势 '{name}' 不在清单里")

    if not (spec.action and "hand" in spec.action):
        raise HTTPException(
            400,
            f"'{spec.id}' 没有 action.hand(kind={spec.kind})。"
            f"trajectory 要走全帧预检、composite 要走 runner 编排,"
            f"这个端点只做一步到位的姿态。")

    rad6 = list(spec.action["hand"])

    # 可行域复查。加载期已经查过一遍,这里再查是因为下发是不可逆动作 ——
    # 清单被改过、或加载期判据放宽过,都得在真正写寄存器前挡住。
    import hand_pose as hp
    why = hp.check_feasible(rad6)
    if why is not None:
        raise HTTPException(409, f"[{spec.id}] 姿态不可行,已拒绝下发: {why}")

    # 先设速度/力,再发角度。SPEED_SET 在这只手上速度与力矩耦合,
    # 不设会沿用 flash 里的旧值,可能出现"发了角度但几乎不动"。
    act = spec.action
    applied = {}
    if "hand_speed" in act:
        hand.set_speed(int(act["hand_speed"]))
        applied["hand_speed"] = int(act["hand_speed"])
    if "hand_force" in act:
        hand.set_force(int(act["hand_force"]))
        applied["hand_force"] = int(act["hand_force"])

    if not hand.set_angles(rad6):
        raise HTTPException(500, "写 ANGLE_SET 失败")

    return {"ok": True, "gesture": spec.id, "name": spec.name,
            "angles": rad6, "applied": applied}


# ============================================================================
# 机械臂端点（占位，等你提供接口）
# ============================================================================
@app.get("/arm/status")
async def arm_status():
    """查询臂状态"""
    if arm is None:
        return {"connected": False, "message": "Arm not implemented yet"}

    # TODO: 实现 arm.read_joints() 等
    return {"connected": True, "joints": [0.0] * 7}


@app.post("/arm/joints")
async def arm_set_joints(req: ArmJoints):
    """设置臂关节角度"""
    if arm is None:
        raise HTTPException(503, "Arm not connected")

    # TODO: arm.set_joints(req.joints)
    return {"ok": True}


# ============================================================================
# 健康检查
# ============================================================================
@app.get("/health")
async def health():
    return {
        "status": "ok",
        "hand": hand is not None,
        "arm": arm is not None
    }


# ============================================================================
# 启动时初始化硬件
# ============================================================================
@app.on_event("startup")
async def startup():
    global hand, arm

    from inspire_hand import InspireHand, InspireHandConfig

    # 灵巧手
    try:
        # 尝试连接真机，失败时用 mock
        try:
            cfg = InspireHandConfig(port="/dev/ttyUSB0", mock=False)
            hand = InspireHand(cfg)
            if hand.connect():
                print("✓ 灵巧手已连接: /dev/ttyUSB0")
            else:
                raise RuntimeError("connect() 返回 False")
        except Exception as e:
            print(f"⚠ 灵巧手连接失败: {e}")
            print("  使用 mock 模式")
            cfg = InspireHandConfig(mock=True)
            hand = InspireHand(cfg)

    except Exception as e:
        print(f"✗ 灵巧手初始化失败: {e}")
        hand = None

    # 机械臂（TODO）
    # try:
    #     from agx_arm import AgxArm
    #     arm = AgxArm()
    #     arm.connect("can0")
    # except Exception as e:
    #     print(f"⚠ 机械臂连接失败: {e}")
    #     arm = None


@app.on_event("shutdown")
async def shutdown():
    if hand is not None and hasattr(hand, 'disconnect'):
        hand.disconnect()
    if arm is not None:
        pass  # arm.disconnect()


# ============================================================================
# 命令行启动
# ============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9000)
    args = parser.parse_args()

    uvicorn.run(app, host=args.host, port=args.port)
