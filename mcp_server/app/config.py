"""配置加载"""
from pathlib import Path
import yaml
from pydantic import BaseModel


class RobotConfig(BaseModel):
    bridge_url: str


class ServerConfig(BaseModel):
    host: str
    port: int
    title: str


class Config(BaseModel):
    robot: RobotConfig
    server: ServerConfig


def load_config() -> Config:
    config_path = Path(__file__).parent.parent / "config.yaml"
    with open(config_path) as f:
        data = yaml.safe_load(f)
    return Config(**data)


config = load_config()
