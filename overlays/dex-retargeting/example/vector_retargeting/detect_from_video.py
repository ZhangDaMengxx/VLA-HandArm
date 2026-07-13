import pickle
from pathlib import Path

import cv2
import numpy as np
import tqdm
import tyro

from dex_retargeting.constants import (
    RobotName,
    RetargetingType,
    HandType,
    get_default_config_path,
)
from dex_retargeting.retargeting_config import RetargetingConfig
from dex_retargeting.seq_retarget import SeqRetargeting
from single_hand_detector import SingleHandDetector


def retarget_video(
    retargeting: SeqRetargeting, video_path: str, output_path: str, config_path: str
):
    cap = cv2.VideoCapture(video_path)

    data = []
    wrist_data = []  # per-frame 3x3: robot base / MANO frame -> camera world frame

    if not cap.isOpened():
        print("Error: Could not open video file.")
    else:
        # Lower confidence (default 0.8) so partial occlusion (e.g. holding an
        # object) drops far fewer frames. 0.5 cut the worst gap from ~0.9s to ~0.1s
        # on the hand_book test; going below 0.5 gained nothing here.
        detector = SingleHandDetector(
            hand_type="Right",
            selfie=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        with tqdm.tqdm(total=length) as pbar:
            while cap.isOpened():
                ret, frame = cap.read()

                if not ret:
                    break

                rgb = frame[..., ::-1]
                num_box, joint_pos, keypoint_2d, mediapipe_wrist_rot = detector.detect(
                    rgb
                )
                if num_box == 0:
                    # Detection dropout (e.g. hand occluded by the held object).
                    # Hold the last valid pose instead of dropping the frame, so the
                    # timeline stays aligned and replay shows a brief freeze rather
                    # than a jump. Leading dropouts (no pose yet) are still skipped.
                    if len(data) > 0:
                        data.append(data[-1])
                        wrist_data.append(wrist_data[-1])
                    pbar.update(1)
                    continue

                retargeting_type = retargeting.optimizer.retargeting_type
                indices = retargeting.optimizer.target_link_human_indices
                if retargeting_type == "POSITION":
                    indices = indices
                    ref_value = joint_pos[indices, :]
                else:
                    origin_indices = indices[0, :]
                    task_indices = indices[1, :]
                    ref_value = (
                        joint_pos[task_indices, :] - joint_pos[origin_indices, :]
                    )
                qpos = retargeting.retarget(ref_value)
                data.append(qpos)
                # Global wrist orientation, which the finger retargeting discards.
                # R maps the robot base / MANO frame to the MediaPipe camera frame, so
                # it can drive a floating base in replay to show hand rotation/flip.
                wrist_data.append(mediapipe_wrist_rot @ detector.operator2mano)
                pbar.update(1)

        meta_data = dict(
            config_path=config_path,
            dof=len(retargeting.optimizer.robot.dof_joint_names),
            joint_names=retargeting.optimizer.robot.dof_joint_names,
        )

        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("wb") as f:
            pickle.dump(
                dict(data=data, wrist_rot=np.asarray(wrist_data), meta_data=meta_data),
                f,
            )

        retargeting.verbose()
        cap.release()
        cv2.destroyAllWindows()


def main(
    robot_name: RobotName,
    video_path: str,
    output_path: str,
    retargeting_type: RetargetingType,
    hand_type: HandType,
):
    """
    Detects the human hand pose from a video and translates the human pose trajectory into a robot pose trajectory.

    Args:
        robot_name: The identifier for the robot. This should match one of the default supported robots.
        video_path: The file path for the input video in .mp4 format.
        output_path: The file path for the output data in .pickle format.
        retargeting_type: The type of retargeting, each type corresponds to a different retargeting algorithm.
        hand_type: Specifies which hand is being tracked, either left or right.
            Please note that retargeting is specific to the same type of hand: a left robot hand can only be retargeted
            to another left robot hand, and the same applies for the right hand.
    """

    config_path = get_default_config_path(robot_name, retargeting_type, hand_type)
    robot_dir = (
        Path(__file__).absolute().parent.parent.parent / "assets" / "robots" / "hands"
    )
    RetargetingConfig.set_default_urdf_dir(str(robot_dir))
    retargeting = RetargetingConfig.load_from_file(config_path).build()
    retarget_video(retargeting, video_path, output_path, str(config_path))


if __name__ == "__main__":
    tyro.cli(main)
