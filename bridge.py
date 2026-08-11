#!/usr/bin/env python3
"""硬件代理 - 在 WSL host 直接运行，暴露简单 HTTP 接口供容器调用

启动: python bridge.py --host 0.0.0.0 --port 9000

为什么需要: Docker 容器访问 WSL USB 设备很麻烦，用代理解耦。
"""
import argparse
import sys
from pathlib import Path

# 添加 sim 到路径
sys.path.insert(0, str(Path(__file__).parent / "sim"))

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
        angles = hand.read_angles()
        return {
            "connected": True,
            "angles": angles,
            "joints": dict(zip(hand.HAND_JOINTS, angles))
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

    try:
        hand.set_angles(req.angles)
        return {"ok": True}
    except Exception as e:
        raise HTTPException(500, f"Set angles failed: {e}")


@app.post("/hand/gesture/{name}")
async def hand_gesture(name: str):
    """执行预设手势（从 registry 读取）"""
    if hand is None:
        raise HTTPException(503, "Hand not connected")

    # 这里简化：直接从 skills/registry.yaml 读取
    # 生产版本应该用 backend.py 的逻辑
    import yaml
    registry_path = Path(__file__).parent / "sim/skills/registry.yaml"

    try:
        with open(registry_path) as f:
            registry = yaml.safe_load(f)

        if name not in registry:
            raise HTTPException(404, f"Gesture '{name}' not found")

        pose = registry[name]
        # 简化：假设 pose 有 angles 字段（需要实际解析逻辑）
        # 生产版本用 backend._resolve_pose
        if "angles" in pose:
            hand.set_angles(pose["angles"])
            return {"ok": True, "gesture": name}
        else:
            raise HTTPException(400, f"Gesture '{name}' has no angles")

    except FileNotFoundError:
        raise HTTPException(500, "Registry not found")
    except Exception as e:
        raise HTTPException(500, f"Execute failed: {e}")


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
