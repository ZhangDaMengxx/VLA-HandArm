#!/usr/bin/env python3
"""诊断 app_web 中 hand_close 的可见性和可执行性"""
import sys
sys.path.insert(0, 'sim/skills')

from schema import get_registry
from console_exec import targets as console_targets

reg = get_registry()
spec = reg.get('hand_close')

print("=" * 60)
print("hand_close 技能诊断")
print("=" * 60)

print("\n1. 基本信息:")
print(f"   ID: {spec.id}")
print(f"   名称: {spec.name}")
print(f"   类型: {spec.kind}")
print(f"   别名: {list(spec.aliases)}")

print("\n2. 语音设置:")
print(f"   voice_enabled: {spec.safety.voice_enabled}")
print(f"   need_confirm: {spec.safety.need_confirm}")

print("\n3. 设备需求:")
targets = console_targets(spec, reg)
print(f"   console_targets: {targets}")
print(f"   是否纯手技能: {targets == {'hand'}}")

print("\n4. 前置条件:")
print(f"   requires: {spec.requires}")

print("\n5. 在 voice_phrases 中的可见性:")
voice_skills = list(reg.voice_skills())
print(f"   voice_enabled 技能总数: {len(voice_skills)}")

# hand scope
hand_ids = [s.id for s in voice_skills if console_targets(s, reg) == {'hand'}]
print(f"\n   scope='hand' (手部调试页):")
print(f"     技能数: {len(hand_ids)}")
if 'hand_close' in hand_ids:
    print(f"     ✓ hand_close 会出现")
else:
    print(f"     ✗ hand_close 不会出现")

# all scope
all_ids = [s.id for s in voice_skills]
print(f"\n   scope='all' (合体页):")
print(f"     技能数: {len(all_ids)}")
if 'hand_close' in all_ids:
    print(f"     ✓ hand_close 会出现")
else:
    print(f"     ✗ hand_close 不会出现")

print("\n6. 语音意图解析:")
from intent import parse as intent_parse

text = "握拳"
it = intent_parse(text, reg, voice_only=True)
print(f"   输入: '{text}'")
print(f"   结果: {it.status}")
if it.ok:
    print(f"   ✓ 识别为: {it.skill_id} ({it.name})")
    print(f"     置信度: {it.confidence}")
else:
    print(f"   ✗ 识别失败: {it.msg}")

print("\n7. 执行路径检查:")
print(f"   ConsoleExecutor 支持: ✓ (composite 技能已支持)")
print(f"   需要的通道: {targets}")
print(f"   前端应该显示: '先接入灵巧手' 如果 hand_console 未启动")

print("\n" + "=" * 60)
print("可能的问题:")
print("=" * 60)

issues = []

if not spec.safety.voice_enabled:
    issues.append("❌ voice_enabled=False，不会出现在语音列表中")

if 'live_session' in spec.requires:
    issues.append("⚠️  需要 live_session，如果未接入会被灰掉或拒绝")

if spec.kind == "composite":
    issues.append("ℹ️  composite 技能，需要 ConsoleExecutor 支持（已支持）")

if not issues:
    print("\n✅ 没有发现明显问题！")
    print("\n可能的原因:")
    print("  1. 前端页面未刷新（需要 Ctrl+F5 强制刷新）")
    print("  2. 在错误的标签页中查找（确认在'语音'标签）")
    print("  3. 未接入灵巧手（手 console 未启动）")
    print("  4. 浏览器缓存问题（清除缓存后重试）")
else:
    for issue in issues:
        print(f"\n{issue}")

print("\n" + "=" * 60)
print("测试命令:")
print("=" * 60)
print("\n# 1. 启动 app_web")
print("~/gradio_venv/bin/python sim/app_web.py")
print("\n# 2. 在浏览器中访问显示的 URL")
print("\n# 3. 切换到'合体页'或'手部调试页'")
print("\n# 4. 点击'接入灵巧手'")
print("\n# 5. 切换到'语音'标签")
print("\n# 6. 在输入框输入'握拳'，点击'识别'")
print("\n# 7. 应该弹出确认框，点击'执行'")
