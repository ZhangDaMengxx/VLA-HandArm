from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "assets/hand/urdf/inspire_hand_right_6dof.urdf"
CONFIG = ROOT / "configs/inspire_hand_right_local.yml"
APP = ROOT / "src/app_web.py"


def test_retarget_proxy_exposes_vendor_derived_fingertip_frames():
    root = ET.parse(URDF).getroot()
    expected = {
        "thumb_tip": ("right_thumb_4", "0.0137599 0.0204399 -0.0080762"),
        "index_tip": ("right_index_2", "0.0144754 0.0426952 -0.0056212"),
        "middle_tip": ("right_middle_2", "0.0161153 0.0456677 -0.0060162"),
        "ring_tip": ("right_ring_2", "0.0144739 0.0426943 -0.0056256"),
        "pinky_tip": ("right_little_2", "0.0117575 0.0353353 -0.0059712"),
    }
    links = {node.attrib["name"] for node in root.findall("link")}
    assert set(expected) <= links
    joints = {node.attrib["name"]: node for node in root.findall("joint")}
    for tip, (parent, xyz) in expected.items():
        joint = joints[f"{tip}_joint"]
        assert joint.attrib["type"] == "fixed"
        assert joint.find("parent").attrib["link"] == parent
        assert joint.find("origin").attrib["xyz"] == xyz


def test_retarget_config_targets_the_fingertip_frames():
    text = CONFIG.read_text(encoding="utf-8")
    match = re.search(r"target_task_link_names:\s*\[([^]]+)\]", text)
    assert match
    names = re.findall(r'"([^"]+)"', match.group(1))
    assert names == ["thumb_tip", "index_tip", "middle_tip", "ring_tip", "pinky_tip"]
    assert re.search(r"^\s*low_pass_alpha:\s*1\.0\s*$", text, re.MULTILINE)


def test_live_wrist_uses_the_mock_calibrated_envelope_for_real_and_mock():
    text = APP.read_text(encoding="utf-8")
    assert "mapper.position_limits[:] = [0.05, 0.05, 0.03]" in text
    assert "(-90.0, -115.0, -175.0)" in text
    assert "(60.0, 50.0, 155.0)" in text
    assert "mapper.position_limits[:] = 0.02" not in text
    assert "(-45.0, -25.0, -35.0)" not in text
