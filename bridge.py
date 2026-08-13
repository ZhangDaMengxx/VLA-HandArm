#!/usr/bin/env python3
"""硬件代理 - 在 WSL host 直接运行，暴露简单 HTTP 接口供容器调用

启动(必须用有 pyserial 的解释器):
    ~/miniconda3/envs/lerobot/bin/python bridge.py --host 0.0.0.0 --port 9000
mock(无硬件空跑)要**显式**加 --mock。

为什么需要: Docker 容器访问 WSL USB 设备很麻烦，用代理解耦。

⚠ 2026-08-11 教训: 早先版本连不上真手就静默退到 mock,而 /health 照样回
   "hand": true —— 结果对着 mock 测了一轮"成功",真手根本没动。现在:
   · 不加 --mock 时连不上 = 启动失败,不再偷偷降级
   · /health 和 /hand/status 都带 "mock" 字段,状态不会再含糊
"""
import argparse
import os
import sys
from pathlib import Path

# sim: inspire_hand 在那里。sim/skills: schema / hand_pose 在那里。
# 注意 skills 必须排在 sim 前面 —— console_exec:350 提到过反过来会让
# sim/ 下的同名模块遮蔽 skills/schema.py。
sys.path.insert(0, str(Path(__file__).parent / "sim"))
sys.path.insert(0, str(Path(__file__).parent / "sim/skills"))

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel
import uvicorn
import secrets

app = FastAPI(title="Robot Hardware Bridge")

# ============================================================================
# 安全认证
# ============================================================================
BRIDGE_TOKEN = os.environ.get("BRIDGE_TOKEN", "")  # 从环境变量读取

@app.middleware("http")
async def check_token(request: Request, call_next):
    """所有非健康检查的请求都需要 token"""
    if request.url.path == "/health":
        return await call_next(request)

    if not BRIDGE_TOKEN:
        # 没配 token 就警告但放行（开发时方便）
        return await call_next(request)

    token = request.headers.get("X-Bridge-Token", "")
    if not secrets.compare_digest(token, BRIDGE_TOKEN):
        return JSONResponse(
            {"detail": "Unauthorized - missing or invalid X-Bridge-Token"},
            status_code=401
        )

    return await call_next(request)

# ============================================================================
# 全局控制器（启动时初始化）
# ============================================================================
hand = None
arm = None
# 由 --mock 或环境变量 BRIDGE_MOCK=1 置位。不置位时连不上就启动失败。
WANT_MOCK = os.environ.get("BRIDGE_MOCK") == "1"


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
            # mock 下 read_angles 回的是**夹取后的目标值**,不是实测位置。
            # 注意夹取发生在 mock 判断之前(inspire_hand.py:377 vs 379),
            # 所以"读回值≠命令值"**不能**用来证明是真手。
            "mock": bool(hand.cfg.mock),
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
    """查询臂状态（包含使能/急停状态）"""
    if arm is None:
        return {"connected": False, "message": "Arm not connected"}

    try:
        joints = arm.read_angles()
        return {
            "connected": True,
            "enabled": arm.enabled,
            "frozen": arm.frozen,
            "joints": joints,
        }
    except Exception as e:
        raise HTTPException(500, f"Read arm status failed: {e}")


@app.post("/arm/enable")
async def arm_enable():
    """使能机械臂电机"""
    if arm is None:
        raise HTTPException(503, "Arm not connected")
    ok = arm.enable()
    if not ok:
        raise HTTPException(500, "enable() 失败，检查 CAN 通信")
    return {"ok": True, "enabled": arm.enabled}


@app.post("/arm/disable")
async def arm_disable():
    """下使能机械臂电机（进入安全状态）"""
    if arm is None:
        raise HTTPException(503, "Arm not connected")
    ok = arm.disable()
    if not ok:
        raise HTTPException(500, "disable() 失败")
    return {"ok": True, "enabled": arm.enabled}


@app.post("/arm/estop")
async def arm_estop():
    """急停：立即进入关节阻尼模式，电机失能。需要 /arm/reset 才能恢复。"""
    if arm is None:
        raise HTTPException(503, "Arm not connected")
    arm.estop()
    return {"ok": True, "frozen": arm.frozen, "enabled": arm.enabled}


@app.post("/arm/reset")
async def arm_reset():
    """退出急停阻尼模式并重新使能。急停后必须调这个才能恢复运动。"""
    if arm is None:
        raise HTTPException(503, "Arm not connected")
    arm.reset()
    return {"ok": True, "frozen": arm.frozen, "enabled": arm.enabled}


@app.post("/arm/joints")
async def arm_set_joints(req: ArmJoints):
    """设置臂关节角度"""
    if arm is None:
        raise HTTPException(503, "Arm not connected")

    if len(req.joints) != 7:
        raise HTTPException(400, "Need 7 joint angles")

    try:
        ok = arm.move_j(req.joints)
        if not ok:
            raise HTTPException(500, "move_j returned False (possibly e-stopped)")
        return {"ok": True, "joints": req.joints}
    except Exception as e:
        raise HTTPException(500, f"Move arm failed: {e}")


# ============================================================================
# 健康检查
# ============================================================================
@app.get("/health")
async def health():
    """mock 字段必须在最外层 —— 分不清真假是 2026-08-11 那次白测的直接原因。"""
    is_mock = bool(hand is not None and hand.cfg.mock)
    return {
        "status": "ok",
        "hand": hand is not None,
        "arm": arm is not None,
        "mock": is_mock,
        "mode": "MOCK(无真实运动)" if is_mock else "REAL",
        "python": sys.executable,
    }


# ============================================================================
# 启动时初始化硬件
# ============================================================================
@app.on_event("startup")
async def startup():
    global hand, arm

    from inspire_hand import InspireHand, InspireHandConfig

    if WANT_MOCK:
        hand = InspireHand(InspireHandConfig(mock=True))
        print("● 灵巧手: MOCK 模式(--mock 显式指定),不会有任何真实运动")
        return

    # 真机模式:连不上就抛,别静默退到 mock —— 那会让"测试通过"变成假象
    cfg = InspireHandConfig(mock=False)  # port 从环境变量 INSPIRE_HAND_PORT 读取
    hand = InspireHand(cfg)
    try:
        ok = hand.connect()
    except Exception as e:
        hand = None
        raise RuntimeError(
            f"灵巧手连接失败: {e}\n"
            f"  用的解释器: {sys.executable}\n"
            f"  需要 pyserial;已知可用: ~/miniconda3/envs/lerobot/bin/python\n"
            f"  确实要空跑就显式加 --mock\n"
            f"  串口由环境变量 INSPIRE_HAND_PORT 控制(默认 /dev/ttyUSB0)") from e
    if not ok:
        hand = None
        raise RuntimeError(
            "灵巧手 connect() 返回 False —— 打开了串口但读不到 HAND_ID。\n"
            "  查 24V 供电 / RS485 A-B 是否接反 / 手 ID=1 / usbipd 是否已转发")
    print(f"✓ 灵巧手已连接真机: {cfg.port} ({sys.executable})")

    # 机械臂
    try:
        from nero_arm import NeroArm
        import platform
        # Windows 用 agx_cando，Linux 用 socketcan
        arm_interface = "agx_cando" if platform.system() == "Windows" else "socketcan"
        # agx_cando 需要数字索引，socketcan 需要字符串 "can0"
        arm_channel = 0 if platform.system() == "Windows" else "can0"
        arm = NeroArm(mock=False, channel=arm_channel, interface=arm_interface)
        ok = arm.connect()
        if not ok:
            arm = None
            raise RuntimeError("机械臂 connect() 返回 False")
        print(f"✓ 机械臂已连接真机: {arm_interface} / channel {arm_channel} ({sys.executable})")
    except Exception as e:
        arm = None
        print(f"⚠ 机械臂连接失败: {e}\n"
              f"  Linux 需要: sudo ip link set can0 up type can bitrate 1000000\n"
              f"  Windows 需要: pip install python-can python-can-agx-cando pyAgxArm")


@app.on_event("shutdown")
async def shutdown():
    if hand is not None and hasattr(hand, 'disconnect'):
        hand.disconnect()
    if arm is not None and hasattr(arm, 'disconnect'):
        arm.disconnect()


# ============================================================================
# 命令行启动
# ============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--mock", action="store_true",
                        help="空跑,不连真机。不加这个时连不上会直接启动失败")
    parser.add_argument("--hand-port", default=None,
                        help="灵巧手 RS485 串口。Linux 形如 /dev/ttyUSB0,"
                             "Windows 形如 COM5。不传则用 INSPIRE_HAND_PORT,"
                             "再不然用平台默认值")
    args = parser.parse_args()

    if args.mock:
        globals()["WANT_MOCK"] = True

    # 命令行 > 环境变量。写回环境变量是因为 InspireHandConfig 从那里读,
    # 这样只有一处真源,不会出现"传了参数但连的是别的口"
    if args.hand_port:
        os.environ["INSPIRE_HAND_PORT"] = args.hand_port

    uvicorn.run(app, host=args.host, port=args.port)
