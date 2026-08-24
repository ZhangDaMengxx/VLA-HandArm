#!/usr/bin/env python3
"""交互式 NERO 臂上相机(eye-in-hand)棋盘格手眼标定。

安全边界：本程序只读取关节角并计算 FK，**不会发送运动命令**。操作者用现有
控制台/示教器移动机械臂，待相机看清固定棋盘格后在终端输入 ``next``。

当前相机适配器使用 OpenCV ``VideoCapture``，用于先跑通流程。奥比中光 336
接入后只需把 SDK 帧转换成 BGR，并复用 ``detect_board``/``solve_eye_in_hand``；
手眼求解不依赖深度流。
"""
from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

try:  # direct: python src/camera/calibrate_handeye.py
    from handeye import solve_eye_in_hand, transform_to_dict
except ImportError:  # module: python -m camera.calibrate_handeye
    from .handeye import solve_eye_in_hand, transform_to_dict


class OpenCVCameraSource:
    def __init__(self, index: int, width: int = 0, height: int = 0, fps: int = 0):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开 OpenCV 相机 index={index}")
        if width:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps:
            self.cap.set(cv2.CAP_PROP_FPS, fps)
        self._latest: np.ndarray | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self) -> None:
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if ok:
                with self._lock:
                    self._latest = frame
            else:
                time.sleep(0.02)

    def latest(self) -> np.ndarray | None:
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.cap.release()


def load_intrinsics(path: Path, camera_name: str | None = None):
    data = json.loads(path.read_text(encoding="utf-8"))
    if camera_name:
        data = data["cameras"][camera_name]
    elif "cameras" in data:
        if len(data["cameras"]) != 1:
            raise ValueError("标定文件包含多台相机，请用 --camera-name 指定")
        data = next(iter(data["cameras"].values()))
    intr = data.get("intrinsics", data)
    try:
        fx, fy, cx, cy = (float(intr[k]) for k in ("fx", "fy", "cx", "cy"))
    except (KeyError, TypeError) as exc:
        raise ValueError("内参 JSON 需要 fx/fy/cx/cy") from exc
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist = np.asarray(data.get("dist_coeffs", data.get("distortion", [0, 0, 0, 0, 0])), dtype=np.float64).reshape(-1, 1)
    return K, dist, {"fx": fx, "fy": fy, "cx": cx, "cy": cy}


def board_object_points(cols: int, rows: int, square_size_m: float) -> np.ndarray:
    if cols < 3 or rows < 3 or square_size_m <= 0:
        raise ValueError("棋盘格内角点至少 3x3，格长必须为正数")
    obj = np.zeros((rows * cols, 3), dtype=np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square_size_m)
    return obj


def detect_board_with_pattern(frame, pattern, obj, K, dist):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
    if not found:
        classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, pattern, classic_flags)
        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4))
    if not found:
        return None, None
    ok, rvec, tvec = cv2.solvePnP(obj, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, None
    R, _ = cv2.Rodrigues(rvec)
    T_camera_target = np.eye(4, dtype=np.float64)
    T_camera_target[:3, :3] = R
    T_camera_target[:3, 3] = tvec.reshape(3)
    return corners, T_camera_target


class NeroArmPoseSource:
    """只读臂姿态：关节角 -> 当前 URDF ee frame 的 T_base_ee。"""
    def __init__(self, no_mock: bool, channel: str, firmware: str,
                 arm_urdf: Path, ee_frame: str):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from nero_arm import NeroArm
        from nero_kin import NeroKin
        self.arm = NeroArm(mock=not no_mock, channel=channel, firmware=firmware)
        self.kin = NeroKin(arm_urdf, ee_frame=ee_frame)
        self.arm_urdf = arm_urdf
        self.ee_frame = ee_frame

    def connect(self):
        self.arm.connect()

    def pose_when_still(self, samples: int = 6, interval_s: float = 0.03,
                        max_joint_range_rad: float = 0.002) -> np.ndarray:
        readings = []
        for _ in range(samples):
            q = np.asarray(self.arm.read_angles(), dtype=np.float64)
            if q.shape != (7,) or not np.all(np.isfinite(q)):
                raise RuntimeError("读不到 7 个有限的机械臂关节角")
            readings.append(q)
            time.sleep(interval_s)
        values = np.stack(readings)
        joint_range = np.ptp(values, axis=0)
        if float(np.max(joint_range)) > max_joint_range_rad:
            raise RuntimeError(
                f"机械臂仍在运动（最大关节变化 {np.max(joint_range):.5f} rad），本次拒绝采样"
            )
        return np.asarray(self.kin.fk(np.mean(values, axis=0)), dtype=np.float64)

    def close(self):
        self.arm.disconnect()


def _command_reader(out: queue.Queue[str]):
    while True:
        try:
            line = input().strip().lower()
        except EOFError:
            line = "quit"
        out.put(line)
        if line in {"quit", "q", "exit"}:
            return


def run(args) -> int:
    K, dist, intr_meta = load_intrinsics(Path(args.intrinsics), args.camera_name)
    obj = board_object_points(args.board_cols, args.board_rows, args.square_size_m)
    pattern = (args.board_cols, args.board_rows)
    arm_urdf = Path(args.arm_urdf).resolve()
    source = OpenCVCameraSource(args.camera_index, args.width, args.height, args.fps)
    try:
        arm = NeroArmPoseSource(args.no_mock, args.channel, args.firmware, arm_urdf, args.ee_frame)
        arm.connect()
    except Exception:
        source.close()
        raise
    output = Path(args.output)
    sample_dir = output.parent / f"{output.stem}_samples"
    sample_dir.mkdir(parents=True, exist_ok=True)
    commands: queue.Queue[str] = queue.Queue()
    threading.Thread(target=_command_reader, args=(commands,), daemon=True).start()
    base_poses, camera_poses = [], []
    print("相机预览已启动。移动机械臂时保持棋盘格固定且完整可见。")
    print("输入 next 采样；undo 撤销最后一组；finish 保存；quit 退出不保存。")
    try:
        while True:
            frame = source.latest()
            if frame is None:
                time.sleep(0.05)
                continue
            corners, T_ct = detect_board_with_pattern(frame, pattern, obj, K, dist)
            view = frame.copy()
            if corners is not None:
                cv2.drawChessboardCorners(view, pattern, corners, True)
                status = "棋盘格 OK"
            else:
                status = "未检测到棋盘格"
            cv2.putText(view, f"samples={len(base_poses)} {status} | next/finish/quit",
                        (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 220, 0) if corners is not None else (0, 0, 255), 2)
            if not args.headless:
                cv2.imshow("hand-eye calibration", view)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    return 0
            try:
                cmd = commands.get_nowait()
            except queue.Empty:
                continue
            if cmd in {"quit", "q", "exit"}:
                return 0
            if cmd in {"undo", "u"}:
                if not base_poses:
                    print("当前没有可撤销的样本")
                    continue
                base_poses.pop()
                camera_poses.pop()
                print(f"已撤销最后一组，当前剩余 {len(base_poses)} 组")
                continue
            if cmd in {"finish", "done"}:
                if len(base_poses) < args.min_samples:
                    print(f"还需要至少 {args.min_samples} 个样本，当前 {len(base_poses)}")
                    continue
                try:
                    result = solve_eye_in_hand(base_poses, camera_poses)
                except ValueError as exc:
                    print(f"当前样本还不能稳定求解：{exc}")
                    continue
                break
            if cmd not in {"next", "n", ""}:
                print("命令只能是 next、undo、finish 或 quit")
                continue
            # 使用 next 到达时刻的最新帧和关节角，尽量避免运动过程中的错配。
            frame = source.latest()
            corners, T_ct = detect_board_with_pattern(frame, pattern, obj, K, dist)
            if T_ct is None:
                print("本次未采样：当前帧没有可靠棋盘格角点")
                continue
            try:
                T_bg = arm.pose_when_still(max_joint_range_rad=args.max_joint_range_rad)
            except RuntimeError as exc:
                print(f"本次未采样：{exc}")
                continue
            base_poses.append(T_bg)
            camera_poses.append(T_ct)
            image_path = sample_dir / f"sample_{len(base_poses) - 1:03d}.png"
            cv2.imwrite(str(image_path), frame)
            print(f"已采样 {len(base_poses)} 组；T_base_{args.ee_frame} 与 T_camera_target 已保存到内存")
            if len(base_poses) >= 4:
                try:
                    result = solve_eye_in_hand(base_poses, camera_poses)
                except ValueError as exc:
                    print(f"当前姿态组合尚不可解：{exc}。请改变相机倾角后继续 next。")
                    continue
                print(f"一致性：平移 RMSE={result.translation_rmse_m*1000:.2f} mm，"
                      f"最大={result.translation_max_m*1000:.2f} mm；"
                      f"旋转 RMSE={result.rotation_rmse_deg:.3f} deg，最大={result.rotation_max_deg:.3f} deg")
                worst = int(np.argmax(
                    np.asarray(result.translation_errors_m) / max(args.max_translation_rmse_m, 1e-12)
                    + np.asarray(result.rotation_errors_deg) / max(args.max_rotation_rmse_deg, 1e-12)
                ))
                print(f"当前综合残差最大的样本：{worst}")
                if (len(base_poses) >= args.min_samples and
                        result.translation_rmse_m <= args.max_translation_rmse_m and
                        result.rotation_rmse_deg <= args.max_rotation_rmse_deg):
                    print("已达到误差阈值，输入 finish 保存结果；也可以继续 next 增加样本。")
    finally:
        source.close()
        arm.close()
        cv2.destroyAllWindows()
    result = solve_eye_in_hand(base_poses, camera_poses)
    out = {
        "schema": "handeye_eye_in_hand_v1",
        "camera_model": args.camera_model,
        "camera_source": "opencv_videocapture_adapter",
        "target": {"type": "chessboard", "inner_corners": [args.board_cols, args.board_rows],
                    "square_size_m": args.square_size_m},
        "camera_intrinsics": intr_meta,
        "coordinate_contract": {"vector": "column", "composition": "left_multiply",
                                 "translation_unit": "m", "quaternion_order": "xyzw"},
        "frames": {"base": "robot_base", "gripper": args.ee_frame, "camera": "camera",
                   "target": "calibration_board"},
        "T_gripper_camera": transform_to_dict(result.T_gripper_camera),
        "quality": {"samples": result.samples, "translation_rmse_m": result.translation_rmse_m,
                    "translation_max_m": result.translation_max_m,
                    "rotation_rmse_deg": result.rotation_rmse_deg,
                    "rotation_max_deg": result.rotation_max_deg,
                    "measurement_class": "internal_fixed_target_consistency",
                    "ground_truth_available": False,
                    "per_sample_translation_error_m": result.translation_errors_m,
                    "per_sample_rotation_error_deg": result.rotation_errors_deg},
        "source": {"intrinsics": str(Path(args.intrinsics).resolve()),
                    "arm_urdf": str(arm_urdf),
                    "sample_images": sample_dir.name,
                    "note": "本工具未发送任何机械臂运动命令；T_base_gripper 来自关节角+URDF FK。"},
        "samples": [
            {
                "index": i,
                "image": f"{sample_dir.name}/sample_{i:03d}.png",
                "T_base_gripper": T_bg.tolist(),
                "T_camera_target": T_ct.tolist(),
            }
            for i, (T_bg, T_ct) in enumerate(zip(base_poses, camera_poses))
        ],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(out, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"已写入 {output}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--intrinsics", required=True, help="包含 fx/fy/cx/cy 的 JSON")
    ap.add_argument("--camera-name", default=None)
    ap.add_argument("--camera-model", default="orbbec_336")
    ap.add_argument("--camera-index", type=int, default=0)
    ap.add_argument("--width", type=int, default=0)
    ap.add_argument("--height", type=int, default=0)
    ap.add_argument("--fps", type=int, default=0)
    ap.add_argument("--board-cols", type=int, required=True, help="棋盘格内角点列数")
    ap.add_argument("--board-rows", type=int, required=True, help="棋盘格内角点行数")
    ap.add_argument("--square-size-m", type=float, required=True, help="小格边长，米")
    ap.add_argument("--output", default="datasets/camera_calibration/orbbec336_handeye.json")
    ap.add_argument("--min-samples", type=int, default=12)
    ap.add_argument("--max-translation-rmse-m", type=float, default=0.003)
    ap.add_argument("--max-rotation-rmse-deg", type=float, default=0.5)
    ap.add_argument("--max-joint-range-rad", type=float, default=0.002,
                    help="next 采样窗口内任一关节的最大允许变化")
    ap.add_argument("--arm-urdf", default="assets/arm/urdf/nero_description.urdf",
                    help="用于 T_base_gripper FK 的 NERO URDF")
    ap.add_argument("--ee-frame", default="link7", help="URDF 中与相机刚性安装处对应的末端 frame")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--firmware", default="auto", choices=["auto", "default", "v111", "v112", "v120"])
    ap.add_argument("--no-mock", action="store_true", help="连接真实机械臂；默认 mock 仅用于离线流程检查")
    ap.add_argument("--headless", action="store_true", help="无 OpenCV 窗口，仅在终端输入命令")
    return run(ap.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
