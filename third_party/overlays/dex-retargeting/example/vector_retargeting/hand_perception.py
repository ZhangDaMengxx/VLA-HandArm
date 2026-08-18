"""hand_perception.py — 手部感知模型的统一接口(让可视化器与具体模型解耦)。

换感知模型时,可视化器 / 重定向都**不用动**,只需按本接口写一个适配器类并注册。

# 契约(最重要)
`HandDetector.detect(frame_bgr)` 必须返回 `HandObservation`,其中 `joint_pos` 是重定向的唯一输入:
  - 形状 **(21, 3)**,MediaPipe / MANO 的 21 点顺序(0=手腕,4=拇指尖,8=食指尖,
    12=中指尖,16=无名指尖,20=小指尖);
  - 坐标系:**机器人 / MANO 系**(手腕平移到原点,并已乘 operator2mano 旋转)。
这是 `dex_retargeting` 里 `target_link_human_indices`(如 [[0..],[4,8,12,16,20]])索引的布局。
如果你的新模型输出的关键点数目 / 顺序 / 坐标系不同,**在适配器的 detect() 里把它转换成这个 21×3 布局**,
否则重定向结果会错(但不会崩)。

# 加一个新模型(三步)
    @register_detector("mymodel")
    class MyHandDetector(HandDetector):
        def __init__(self, hand_type="Right", selfie=False, **kw): ...
        def detect(self, frame_bgr):
            ...                       # 跑你的模型
            return HandObservation(found=..., num_hands=..., joint_pos=kp21x3,
                                   keypoints_2d=kp21x2_pixels)   # 2D 可选,用于画骨架
        # 可选:def draw(self, image_bgr, obs): 用你模型自带的画法覆盖默认画法
然后运行:`python hand_robot_visualizer.py --detector-name mymodel`
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np


@dataclass
class HandObservation:
    """一帧的手部感知结果。可视化器 + 重定向只依赖这个结构,不依赖任何具体模型。"""

    found: bool = False
    num_hands: int = 0
    # 重定向的唯一输入:21×3,MANO/机器人坐标系。见模块 docstring 的契约。None 表示这帧没检到手。
    joint_pos: Optional[np.ndarray] = None
    # 可选:21×2 像素坐标,用于在画面上画骨架(默认 draw() 用它)。
    keypoints_2d: Optional[np.ndarray] = None
    # 可选:模型原始输出(某些适配器在自定义 draw() 里用,如 MediaPipe 的 NormalizedLandmarkList)。
    raw: object = None


class HandDetector(ABC):
    """手部感知模型接口。换模型 = 实现一个子类 + @register_detector。"""

    # MediaPipe/MANO 21 点连线(供默认通用绘制)
    HAND_EDGES: List[Tuple[int, int]] = [
        (0, 1), (1, 2), (2, 3), (3, 4),            # thumb
        (0, 5), (5, 6), (6, 7), (7, 8),            # index
        (0, 9), (9, 10), (10, 11), (11, 12),       # middle
        (0, 13), (13, 14), (14, 15), (15, 16),     # ring
        (0, 17), (17, 18), (18, 19), (19, 20),     # pinky
        (5, 9), (9, 13), (13, 17),                 # palm arch
    ]

    @abstractmethod
    def detect(self, frame_bgr: np.ndarray) -> HandObservation:
        """输入 BGR 帧,返回 HandObservation。joint_pos 必须是 21×3 MANO 系(见模块契约)。"""
        raise NotImplementedError

    def draw(self, image_bgr: np.ndarray, obs: HandObservation) -> np.ndarray:
        """把手部骨架画到 image_bgr 上(就地修改并返回)。默认用 obs.keypoints_2d 通用画法;
        子类可覆盖成模型自带的更好看的画法。"""
        import cv2

        if obs.keypoints_2d is None:
            return image_bgr
        pts = np.asarray(obs.keypoints_2d)
        for a, b in self.HAND_EDGES:
            if a < len(pts) and b < len(pts):
                cv2.line(image_bgr, tuple(pts[a].astype(int)), tuple(pts[b].astype(int)),
                         (255, 255, 255), 2, cv2.LINE_AA)
        for p in pts:
            cv2.circle(image_bgr, tuple(np.asarray(p).astype(int)), 3, (0, 180, 255), -1, cv2.LINE_AA)
        return image_bgr


# ---------------------------------------------------------------------------
# 注册表 / 工厂
# ---------------------------------------------------------------------------
_REGISTRY: Dict[str, Callable[..., HandDetector]] = {}


def register_detector(name: str):
    """类装饰器:把一个 HandDetector 子类注册到 name 下。"""
    def deco(cls):
        _REGISTRY[name] = cls
        return cls
    return deco


def make_detector(name: str, **kwargs) -> HandDetector:
    """按名字造一个感知模型。kwargs 透传给该模型的 __init__(如 hand_type、selfie)。"""
    if name not in _REGISTRY:
        raise ValueError(f"未知感知模型 '{name}',可选: {sorted(_REGISTRY)}")
    return _REGISTRY[name](**kwargs)


def available_detectors() -> List[str]:
    return sorted(_REGISTRY)


# ---------------------------------------------------------------------------
# 默认实现:MediaPipe(封装现有 single_hand_detector.SingleHandDetector)
# ---------------------------------------------------------------------------
@register_detector("mediapipe")
class MediaPipeHandDetector(HandDetector):
    """默认后端:MediaPipe。它输出的就是标准 21×3 MANO 系,直接满足契约。"""

    def __init__(self, hand_type: str = "Right", selfie: bool = False, **kwargs):
        from single_hand_detector import SingleHandDetector  # 延迟导入(只有用到才依赖 mediapipe)
        self._d = SingleHandDetector(hand_type=hand_type, selfie=selfie, **kwargs)

    def detect(self, frame_bgr: np.ndarray) -> HandObservation:
        num_box, joint_pos, keypoint_2d, _wrist_rot = self._d.detect(frame_bgr[..., ::-1])  # BGR->RGB
        kp2d = None
        if keypoint_2d is not None:                    # 归一化 landmark -> 像素坐标(供通用画法/其他用途)
            h, w = frame_bgr.shape[:2]
            try:
                kp2d = np.array([[lm.x * w, lm.y * h] for lm in keypoint_2d.landmark], dtype=float)
            except Exception:
                kp2d = None
        return HandObservation(found=joint_pos is not None, num_hands=num_box,
                               joint_pos=joint_pos, keypoints_2d=kp2d, raw=keypoint_2d)

    def draw(self, image_bgr: np.ndarray, obs: HandObservation) -> np.ndarray:
        # 用 MediaPipe 自带的绘制(比通用画法好看);拿不到原始对象时退回通用画法。
        if obs.raw is not None:
            self._d.draw_skeleton_on_image(image_bgr, obs.raw, style="default")
            return image_bgr
        return super().draw(image_bgr, obs)


# ---------------------------------------------------------------------------
# 默认实现:MediaPipe(封装现有 single_hand_detector.SingleHandDetector)
# ---------------------------------------------------------------------------
@register_detector("WiLoR")
class MediaPipeHandDetector(HandDetector):
    """WiLoR。它输出的就是标准 21×3 MANO 系,直接满足契约。"""

    def __init__(self, hand_type: str = "Right", selfie: bool = False, **kwargs):
        from single_hand_detector import SingleHandDetector  # 延迟导入(只有用到才依赖 mediapipe)
        self._d = SingleHandDetector(hand_type=hand_type, selfie=selfie, **kwargs)

    def detect(self, frame_bgr: np.ndarray) -> HandObservation:
        num_box, joint_pos, keypoint_2d, _wrist_rot = self._d.detect(frame_bgr[..., ::-1])  # BGR->RGB
        kp2d = None
        if keypoint_2d is not None:                    # 归一化 landmark -> 像素坐标(供通用画法/其他用途)
            h, w = frame_bgr.shape[:2]
            try:
                kp2d = np.array([[lm.x * w, lm.y * h] for lm in keypoint_2d.landmark], dtype=float)
            except Exception:
                kp2d = None
        return HandObservation(found=joint_pos is not None, num_hands=num_box,
                               joint_pos=joint_pos, keypoints_2d=kp2d, raw=keypoint_2d)

    def draw(self, image_bgr: np.ndarray, obs: HandObservation) -> np.ndarray:
        # 用 MediaPipe 自带的绘制(比通用画法好看);拿不到原始对象时退回通用画法。
        if obs.raw is not None:
            self._d.draw_skeleton_on_image(image_bgr, obs.raw, style="default")
            return image_bgr
        return super().draw(image_bgr, obs)
