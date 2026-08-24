"""Keep every local hand command path on the same asset-nominal limits."""
from __future__ import annotations

import ast
import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
SRC = REPO / "src"
JOINTS = (
    "right_thumb_1_joint",
    "right_thumb_2_joint",
    "right_index_1_joint",
    "right_middle_1_joint",
    "right_ring_1_joint",
    "right_little_1_joint",
)


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    finally:
        sys.modules.pop(name, None)
    return module


def _literal_assignment(path: Path, name: str):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            if any(isinstance(target, ast.Name) and target.id == name for target in node.targets):
                if isinstance(node.value, ast.Dict):
                    return {
                        ast.literal_eval(key): ast.literal_eval(value)
                        for key, value in zip(node.value.keys, node.value.values)
                        if key is not None
                    }
                return ast.literal_eval(node.value)
    raise AssertionError(f"{path} does not define a literal {name}")


def _urdf_limits(path: Path) -> dict[str, tuple[float, float]]:
    result = {}
    for joint in ET.parse(path).getroot().findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name in JOINTS and limit is not None:
            result[name] = (float(limit.get("lower")), float(limit.get("upper")))
    return result


def test_local_hand_limits_match_driver_and_urdf():
    driver = _load_module("_hand_limit_driver", SRC / "inspire_hand.py")
    pose = _load_module("_hand_limit_pose", SRC / "skills/hand_pose.py")
    writer = _literal_assignment(SRC / "ros_joint_writer.py", "JOINT_LIMITS")
    generator = _literal_assignment(SRC / "build_inspire_from_vendor.py", "DRIVEN_LIMIT")
    urdf = _urdf_limits(REPO / "assets/hand/urdf/inspire_hand_right.urdf")

    assert tuple(driver.HAND_JOINTS) == JOINTS
    assert tuple(pose.HAND_JOINTS) == JOINTS
    assert set(urdf) == set(JOINTS)
    for name in JOINTS:
        expected = tuple(driver.HAND_LIMITS[name])
        assert tuple(writer[name]) == expected
        assert tuple(generator[name]) == expected
        assert urdf[name] == expected
        assert pose.LIMIT_HI[name] == expected[1]
        assert tuple(pose.RAW_MAP[name]) == tuple(driver.RAW_MAP[name])


def test_asset_span_matches_nominal_limit():
    driver = _load_module("_hand_span_driver", SRC / "inspire_hand.py")
    for name in JOINTS:
        span, invert = driver.RAW_MAP[name]
        lower, upper = driver.HAND_LIMITS[name]
        assert lower == 0.0
        assert span == upper
        assert invert is True
