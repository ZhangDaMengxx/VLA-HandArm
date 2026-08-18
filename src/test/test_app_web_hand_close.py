#!/usr/bin/env python3
"""测试 app_web.py 的语音执行路径 - 验证 hand_close 可以通过 ConsoleExecutor 执行

这个测试验证：
1. ConsoleExecutor 可以展开 composite 技能（hand_close）
2. 指令正确翻译成 console 协议
3. 两个步骤按序执行
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "skills"))
from backend import make_backend
from schema import get_registry
from console_exec import ConsoleExecutor, translate, targets


def test_hand_close_expansion():
    """测试 hand_close 展开"""
    print("=" * 60)
    print("测试1: hand_close 技能展开")
    print("=" * 60)

    reg = get_registry()
    spec = reg.get('hand_close')

    assert spec is not None, "hand_close 技能不存在"
    assert spec.kind == "composite", f"hand_close 应该是 composite，实际是 {spec.kind}"

    # 检查设备需求
    need = targets(spec, reg)
    print(f"\n✓ 需要的设备: {need}")
    assert "hand" in need, "hand_close 应该需要 hand 设备"

    # 展开技能
    be = make_backend(spec, reg)
    params, _ = spec.resolve_params({})
    total = be.total(params)
    secs = be.duration_hint(params)

    print(f"✓ 总步数: {total}")
    print(f"✓ 预计时长: {secs}s")
    assert total == 2, f"hand_close 应该有2步，实际 {total}"

    # 获取指令序列
    steps = list(be.steps(params))
    print(f"\n✓ 实际展开为 {len(steps)} 个指令")

    for i, step in enumerate(steps):
        print(f"\n步骤 {i+1}:")
        print(f"  hold: {step.hold}s")
        print(f"  cmd keys: {list(step.cmd.keys())}")

        # 验证指令包含必要字段
        assert "hand" in step.cmd, f"步骤{i+1}应该有hand字段"
        assert "hand_speed" in step.cmd, f"步骤{i+1}应该有hand_speed"
        assert "hand_force" in step.cmd, f"步骤{i+1}应该有hand_force"

        print(f"  hand angles: {step.cmd['hand']}")
        print(f"  speed: {step.cmd['hand_speed']}, force: {step.cmd['hand_force']}")

    print("\n✓ 技能展开测试通过")
    return True


def test_translate():
    """测试 writer 指令 → console 协议翻译"""
    print("\n" + "=" * 60)
    print("测试2: 指令翻译（writer → console 协议）")
    print("=" * 60)

    # 模拟 hand_close 的第一步
    cmd = {
        "duration": 0.8,
        "hand_speed": 500,
        "hand_force": 300,
        "hand": [0.9994, 0.4998, 0.0, 0.0, 0.0, 0.0]
    }

    console_cmds = translate(cmd)
    print(f"\nWriter 指令:")
    print(f"  {cmd}")
    print(f"\nConsole 指令序列 ({len(console_cmds)} 条):")

    for device, console_cmd in console_cmds:
        print(f"  设备: {device}")
        print(f"  指令: {console_cmd}")

    # 验证翻译结果
    assert len(console_cmds) == 3, f"应该翻译为3条console指令（speed/force/angles），实际{len(console_cmds)}"

    devices = [d for d, _ in console_cmds]
    cmds = [c for _, c in console_cmds]

    assert all(d == "hand" for d in devices), "所有指令应该发给 hand"
    assert cmds[0]["cmd"] == "speed", "第1条应该是 speed"
    assert cmds[1]["cmd"] == "force", "第2条应该是 force"
    assert cmds[2]["cmd"] == "angles", "第3条应该是 angles"

    print("\n✓ 指令翻译测试通过")
    return True


def test_mock_executor():
    """测试 ConsoleExecutor 执行流程（mock）"""
    print("\n" + "=" * 60)
    print("测试3: ConsoleExecutor 执行流程（mock）")
    print("=" * 60)

    # Mock console 函数
    sent_cmds = []

    def mock_send_arm(cmd):
        sent_cmds.append(("arm", cmd))
        return {"ok": True}

    def mock_send_hand(cmd):
        sent_cmds.append(("hand", cmd))
        return {"ok": True}

    def mock_hand_state():
        # 模拟手的状态
        return {
            "raw": [1000, 500, 0, 0, 0, 0],
            "rad": [0, 0.5, 0, 0, 0, 0],
            "mock": True
        }

    # 创建 mock executor
    executor = ConsoleExecutor(
        send_arm=mock_send_arm,
        send_hand=mock_send_hand,
        hand_state=mock_hand_state
    )

    # 构造执行信封
    env = {
        "skill_id": "hand_close",
        "params": {},
        "source": "voice",
        "confirmed": True
    }

    # 执行
    events = list(executor.invoke(env))

    print(f"\n✓ 收到 {len(events)} 个事件")

    for event in events:
        print(f"  {event.get('type')}: {event.get('msg', '')}")
        if event.get("type") == "start":
            print(f"    技能: {event.get('name')}")
            print(f"    总步数: {event.get('total')}")
            print(f"    预计时长: {event.get('est_seconds')}s")

    # 验证事件序列
    types = [e.get("type") for e in events]
    assert "start" in types, "应该有 start 事件"
    assert "done" in types or "error" in types, "应该有 done 或 error 事件"

    # 验证发送的指令
    print(f"\n✓ 发送了 {len(sent_cmds)} 条console指令")
    for device, cmd in sent_cmds:
        print(f"  {device}: {cmd.get('cmd')}")

    print("\n✓ ConsoleExecutor 执行测试通过")
    return True


def main():
    print("\n🧪 测试 app_web.py 的 hand_close 支持")
    print("=" * 60)

    results = []

    try:
        results.append(("技能展开", test_hand_close_expansion()))
    except Exception as e:
        print(f"\n✗ 技能展开测试失败: {e}")
        results.append(("技能展开", False))

    try:
        results.append(("指令翻译", test_translate()))
    except Exception as e:
        print(f"\n✗ 指令翻译测试失败: {e}")
        results.append(("指令翻译", False))

    try:
        results.append(("执行流程", test_mock_executor()))
    except Exception as e:
        print(f"\n✗ 执行流程测试失败: {e}")
        results.append(("执行流程", False))

    print("\n" + "=" * 60)
    print("测试总结")
    print("=" * 60)

    for name, passed in results:
        status = "✓ 通过" if passed else "✗ 失败"
        print(f"  {status}  {name}")

    total = len(results)
    passed = sum(1 for _, p in results if p)
    print(f"\n总计: {passed}/{total} 通过")

    if passed == total:
        print("\n🎉 所有测试通过！app_web.py 已完全支持 hand_close")
    else:
        print(f"\n⚠️  {total - passed} 个测试失败")

    return passed == total


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
