"""配置加载。

安全分级:
  · lan    —— 局域网,零配置开箱即用(不校验 API Key)
  · public —— 公网,强制 API Key + CORS 白名单
  · auto   —— 按本机 IP 猜:私网地址段判 lan,否则判 public

⚠ 这套服务能直接驱动真实硬件,而且 bridge 那一侧没有任何鉴权。
   把 mode 设成 lan 就意味着**局域网内任何人都能让机械臂动**。
   放到 WSL 之外(纯 Ubuntu / 云主机)时尤其注意:那边没有 NAT 挡着。
"""
import os
import socket
from pathlib import Path

import yaml
from pydantic import BaseModel


class RobotConfig(BaseModel):
    bridge_url: str
    bridge_token: str = ""  # 可选，用于 bridge 认证


class ServerConfig(BaseModel):
    host: str
    port: int
    title: str


class SecurityConfig(BaseModel):
    mode: str = "auto"                      # auto | lan | public
    api_keys: list[str] = []
    cors_origins: list[str] = ["*"]
    # 免鉴权路径:健康检查要留给探针/负载均衡,它不碰硬件
    public_paths: list[str] = ["/health", "/", "/docs", "/openapi.json"]


class Config(BaseModel):
    robot: RobotConfig
    server: ServerConfig
    security: SecurityConfig = SecurityConfig()

    def resolve_mode(self) -> str:
        """把 auto 落成 lan / public。"""
        if self.security.mode != "auto":
            return self.security.mode
        return "lan" if _in_private_net() else "public"


def _in_private_net() -> bool:
    """本机主 IP 是否在 RFC1918 私网段。

    用 UDP connect 拿"出口 IP" —— 不发包,只让内核选路由,所以不需要外网可达。
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("10.255.255.255", 1))
            ip = s.getsockname()[0]
        finally:
            s.close()
    except Exception:
        return True                        # 拿不到就按保守的 lan 处理
    return (ip.startswith("10.") or ip.startswith("192.168.")
            or any(ip.startswith(f"172.{n}.") for n in range(16, 32))
            or ip.startswith("127."))


def _env_list(name: str) -> list[str] | None:
    """逗号分隔的环境变量 → 列表。未设置返回 None(区别于设成空)。"""
    raw = os.environ.get(name)
    if raw is None:
        return None
    return [x.strip() for x in raw.split(",") if x.strip()]


def load_config() -> Config:
    config_path = Path(__file__).parent.parent / "config.yaml"
    with open(config_path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    data.setdefault("security", {})

    # 环境变量优先于 config.yaml —— 容器里不方便改文件,
    # 而且 API Key 不该写进会被提交的 yaml
    if v := os.environ.get("ROBOT_BRIDGE_URL"):
        data.setdefault("robot", {})["bridge_url"] = v
    if v := os.environ.get("ROBOT_BRIDGE_TOKEN"):
        data.setdefault("robot", {})["bridge_token"] = v
    if v := os.environ.get("MCP_SECURITY_MODE"):
        data["security"]["mode"] = v
    if (v := _env_list("MCP_API_KEYS")) is not None:
        data["security"]["api_keys"] = v
    if (v := _env_list("MCP_CORS_ORIGINS")) is not None:
        data["security"]["cors_origins"] = v

    return Config(**data)


config = load_config()
