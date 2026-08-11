"""工具定义 —— 单一真源。

jsonrpc_mcp.py(真 MCP 协议)和 server.py(废弃的 REST 端点)都从这里读。
之前两边各存一份,描述已经开始漂 —— 大模型看到的指引取决于它打哪个端点,
这种分叉很难在测试里发现。

纯 dict 而不是 pydantic model:MCP 规范里 inputSchema 就是 JSON Schema,
FastAPI 直接序列化 dict 没问题,少一层转换。
"""

TOOLS = [
    {
        "name": "hand_list_gestures",
        "description": "列出可用的灵巧手手势及其含义。调 hand_gesture 前先用这个拿准确的 id，"
                       "不要凭猜测填 id。",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "hand_gesture",
        "description": "执行灵巧手预设手势。id 必须来自 hand_list_gestures。"
                       "会先做拇指-食指可行域检查，会导致手指互顶的姿态被拒绝并说明原因。"
                       "返回里的 mock 字段为 true 时表示空跑，硬件没有真实运动。",
        "inputSchema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "手势 id，从 hand_list_gestures 获取",
                }
            },
            "required": ["name"],
        },
    },
    {
        "name": "hand_set_angles",
        "description": "设置灵巧手 6 个关节角度（弧度，0=张开）。"
                       "会做拇指-食指可行域检查，会导致手指互顶的姿态被拒绝并说明原因；"
                       "优先用 hand_gesture 调预设手势，这个接口用于预设覆盖不到的姿态。",
        "inputSchema": {
            "type": "object",
            "properties": {
                "angles": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 6,
                    "maxItems": 6,
                    "description": "6 个关节角度（rad，0=张开）: "
                                   "[thumb_yaw, thumb_pitch, index, middle, ring, pinky]",
                }
            },
            "required": ["angles"],
        },
    },
    {
        "name": "hand_status",
        "description": "查询灵巧手当前状态（连接状态、关节角度）。"
                       "返回里的 mock 字段为 true 时表示当前是空跑模式，下发指令不会有真实运动。",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "arm_status",
        "description": "查询机械臂当前状态。注意：机械臂目前是未实现状态，"
                       "connected 会回 false。",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "arm_set_joints",
        "description": "设置机械臂 7 个关节角度（弧度）。"
                       "注意：机械臂目前是未实现状态，调用会失败。",
        "inputSchema": {
            "type": "object",
            "properties": {
                "joints": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 7,
                    "maxItems": 7,
                    "description": "7 个关节角度（rad）",
                }
            },
            "required": ["joints"],
        },
    },
]
