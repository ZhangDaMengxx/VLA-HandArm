#!/usr/bin/env python3
"""测试新的 /execute 端点 - 验证 hand_close 可以执行"""
import requests
import json

BASE_URL = "http://127.0.0.1:9000"

def test_execute_hand_close():
    """测试执行 hand_close composite 技能"""
    print("=" * 60)
    print("测试 /execute 端点执行 hand_close")
    print("=" * 60)

    payload = {
        "skill": "hand_close",
        "params": {}
    }

    print(f"\nPOST {BASE_URL}/execute")
    print(f"Body: {json.dumps(payload, ensure_ascii=False, indent=2)}")

    try:
        resp = requests.post(f"{BASE_URL}/execute", json=payload, timeout=10)
        print(f"\nStatus: {resp.status_code}")
        print(f"Response: {json.dumps(resp.json(), ensure_ascii=False, indent=2)}")

        if resp.status_code == 200:
            print("\n✓ hand_close 执行成功！")
            return True
        else:
            print(f"\n✗ 执行失败: {resp.json()}")
            return False

    except requests.exceptions.ConnectionError:
        print("\n✗ 连接失败：bridge 服务未启动")
        print("   请先运行: python bridge.py --mock --host 127.0.0.1 --port 9000")
        return False
    except Exception as e:
        print(f"\n✗ 请求异常: {e}")
        return False


def test_execute_with_alias():
    """测试使用别名调用"""
    print("\n" + "=" * 60)
    print("测试用别名 '握拳' 调用")
    print("=" * 60)

    payload = {
        "skill": "握拳",  # 使用中文别名
        "params": {}
    }

    print(f"\nPOST {BASE_URL}/execute")
    print(f"Body: {json.dumps(payload, ensure_ascii=False, indent=2)}")

    try:
        resp = requests.post(f"{BASE_URL}/execute", json=payload, timeout=10)
        print(f"\nStatus: {resp.status_code}")
        print(f"Response: {json.dumps(resp.json(), ensure_ascii=False, indent=2)}")

        if resp.status_code == 200:
            print("\n✓ 别名调用成功！")
            return True
        else:
            print(f"\n✗ 执行失败: {resp.json()}")
            return False

    except Exception as e:
        print(f"\n✗ 请求异常: {e}")
        return False


def test_execute_with_params():
    """测试带参数调用（力度档）"""
    print("\n" + "=" * 60)
    print("测试 hand_close 带力度参数")
    print("=" * 60)

    payload = {
        "skill": "hand_close",
        "params": {
            "hand_force": 500,  # 用力档
            "hand_speed": 300
        }
    }

    print(f"\nPOST {BASE_URL}/execute")
    print(f"Body: {json.dumps(payload, ensure_ascii=False, indent=2)}")

    try:
        resp = requests.post(f"{BASE_URL}/execute", json=payload, timeout=10)
        print(f"\nStatus: {resp.status_code}")
        print(f"Response: {json.dumps(resp.json(), ensure_ascii=False, indent=2)}")

        if resp.status_code == 200:
            print("\n✓ 带参数调用成功！")
            return True
        else:
            print(f"\n✗ 执行失败: {resp.json()}")
            return False

    except Exception as e:
        print(f"\n✗ 请求异常: {e}")
        return False


def test_old_gesture_endpoint():
    """测试旧的 /hand/gesture 端点（应该拒绝 composite）"""
    print("\n" + "=" * 60)
    print("测试旧端点 /hand/gesture/hand_close（应该返回 400）")
    print("=" * 60)

    try:
        resp = requests.post(f"{BASE_URL}/hand/gesture/hand_close", timeout=5)
        print(f"\nStatus: {resp.status_code}")
        print(f"Response: {json.dumps(resp.json(), ensure_ascii=False, indent=2)}")

        if resp.status_code == 400:
            print("\n✓ 如预期，旧端点正确拒绝了 composite 技能")
            return True
        else:
            print("\n⚠ 未如预期，应该返回 400")
            return False

    except Exception as e:
        print(f"\n✗ 请求异常: {e}")
        return False


if __name__ == "__main__":
    print("\n🧪 测试 bridge.py 的 /execute 端点")
    print("=" * 60)
    print("确保 bridge 已启动:")
    print("  python bridge.py --mock --host 127.0.0.1 --port 9000")
    print("=" * 60)

    results = []
    results.append(("hand_close (ID)", test_execute_hand_close()))
    results.append(("握拳 (别名)", test_execute_with_alias()))
    results.append(("hand_close (带参数)", test_execute_with_params()))
    results.append(("旧端点拒绝 composite", test_old_gesture_endpoint()))

    print("\n" + "=" * 60)
    print("测试总结")
    print("=" * 60)
    for name, passed in results:
        status = "✓ 通过" if passed else "✗ 失败"
        print(f"  {status}  {name}")

    total = len(results)
    passed = sum(1 for _, p in results if p)
    print(f"\n总计: {passed}/{total} 通过")
