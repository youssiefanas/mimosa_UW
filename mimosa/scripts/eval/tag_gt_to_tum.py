#!/usr/bin/env python3
"""Tag-based ground-truth trajectory extractor for a downward camera view.

Reads `sensor_msgs/CompressedImage` from a ROS 2 bag, detects a ChArUco
board or a single AprilTag in each frame, and writes the BODY pose in the
TAG world frame as a TUM trajectory.

The tag frame is treated as the world (map) frame; the body pose is
recovered as

    T_world_body = T_world_cam @ inv(T_body_cam)
                 = inv(T_cam_world) @ inv(T_body_cam)

where T_cam_world is what OpenCV pose estimation returns (rvec/tvec map
points in the tag frame into the camera frame).

`T_body_cam` is hard-coded at the top of this file (APRILTAG_T_BODY_CAM /
CHARUCO_T_BODY_CAM). Camera intrinsics and distortion default to the
dv_slam dataset YAML so this script and the VO pipeline share one source
of truth, and can be overridden with `--camera-yaml`.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# --- Global Configurations ---
APRILTAG_TOPIC = "/cam1/image_raw/compressed"
CHARUCO_TOPIC = "/cam1/image_raw/compressed"

APRILTAG_T_BODY_CAM = [0.21170000, -0.09723000, 0.00000000, -0.50000000, 0.50000184, -0.50000000, 0.49999816]
CHARUCO_T_BODY_CAM = [0.22, -0.15, 0.15, -0.50000000, 0.50000184, -0.50000000, 0.49999816] # front dwe cam

# CHARUCO_T_BODY_CAM = [0.00000000, 0.2500000, -0.0000000, -0.70710548, 0.70710808, 0.00000094, -0.00000094] # downlooking dwe cam

CHARUCO_SQUARES_X = 12
CHARUCO_SQUARES_Y = 9
CHARUCO_SQUARE_LENGTH = 0.060
CHARUCO_MARKER_LENGTH = 0.045
CHARUCO_DICT_NAME = "DICT_5X5_100"
# -----------------------------

def quat_from_R(R: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix to (qx, qy, qz, qw)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


def pose_from_extrinsic(v: list[float]) -> np.ndarray:
    """7-element [tx, ty, tz, qx, qy, qz, qw] -> 4x4."""
    if len(v) != 7:
        raise ValueError("extrinsic must be 7 elements [t(3), q(4)]")
    qx, qy, qz, qw = v[3], v[4], v[5], v[6]
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        raise ValueError("zero-norm quaternion in extrinsic")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    R = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = v[:3]
    return T


def load_intrinsics(camera_yaml: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load fx, fy, cx, cy and k1..k3, p1..p2 from a dv_slam dataset YAML."""
    with camera_yaml.open() as f:
        doc = yaml.safe_load(f)
    cam = doc["/**"]["ros__parameters"]["Camera"]
    fx, fy = float(cam["fx"]), float(cam["fy"])
    cx, cy = float(cam["cx"]), float(cam["cy"])
    k1 = float(cam.get("k1", 0.0))
    k2 = float(cam.get("k2", 0.0))
    p1 = float(cam.get("p1", 0.0))
    p2 = float(cam.get("p2", 0.0))
    k3 = float(cam.get("k3", 0.0))
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
    return K, D


def load_tag_spec(spec_yaml: Optional[Path], tag_type: str) -> dict:
    """Tag spec: ChArUco board params or AprilTag dimensions."""
    if spec_yaml is not None:
        with spec_yaml.open() as f:
            return yaml.safe_load(f)
    if tag_type == "apriltag":
        return {"family": "DICT_APRILTAG_36h11", "size": 0.15}
    raise ValueError("ChArUco requires --tag-spec (no default baked in).")


def make_charuco_board(spec: dict):
    dict_id = getattr(cv2.aruco, spec["family"])
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    board = cv2.aruco.CharucoBoard_create(
        int(spec["squaresX"]), int(spec["squaresY"]),
        float(spec["square_length"]), float(spec["marker_length"]),
        aruco_dict,
    )
    return aruco_dict, board


def make_detector_params() -> "cv2.aruco_DetectorParameters":
    """Tuned for the BlueROV2 underwater images: aggressive corner refinement,
    wide adaptive-threshold window range to cope with haze and uneven light."""
    p = cv2.aruco.DetectorParameters_create()
    if hasattr(cv2.aruco, "CORNER_REFINE_APRILTAG"):
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    else:
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.adaptiveThreshWinSizeMin = 5
    p.adaptiveThreshWinSizeMax = 45
    p.adaptiveThreshWinSizeStep = 10
    return p


def preprocess(img_color: np.ndarray, clahe) -> np.ndarray:
    """BGR -> grayscale -> CLAHE (matches the VO frontend preprocessing)."""
    gray = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)
    return clahe.apply(gray)


def detect_pose_apriltag(gray: np.ndarray, aruco_dict, params, tag_size: float,
                         K: np.ndarray, D: np.ndarray) -> Optional[np.ndarray]:
    """Return T_cam_tag (4x4) for a single tag, or None."""
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
    if ids is None or len(ids) == 0:
        return None
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, tag_size, K, D
    )
    # Single-tag GT: pick the largest marker by quad area to be robust to
    # spurious matches.
    areas = [cv2.contourArea(c.reshape(-1, 2)) for c in corners]
    idx = int(np.argmax(areas))
    rvec, tvec = rvecs[idx][0], tvecs[idx][0]
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec
    return T


def detect_pose_charuco(gray: np.ndarray, aruco_dict, params, board,
                        K: np.ndarray, D: np.ndarray) -> Optional[np.ndarray]:
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
    if ids is None or len(ids) == 0:
        return None
    n, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board
    )
    if n is None or n < 4:
        return None
    rvec = np.zeros((3, 1))
    tvec = np.zeros((3, 1))
    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        ch_corners, ch_ids, board, K, D, rvec, tvec
    )
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()
    return T


def main() -> int:
    repo_root = Path(__file__).resolve().parents[4]  # …/src
    default_cam = repo_root / "dv_slam/config/datasets/bluerov.yaml"

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", type=Path, help="Input ROS 2 bag directory.")
    ap.add_argument("out", type=Path, help="Output TUM file.")
    ap.add_argument("--tag-type", choices=["apriltag", "charuco"],
                    default="apriltag")
    ap.add_argument("--camera-yaml", type=Path, default=default_cam,
                    help=f"Camera intrinsics YAML (default {default_cam}).")
    ap.add_argument("--every-n", type=int, default=1,
                    help="Process every Nth frame (default 1).")
    args = ap.parse_args()

    K, D = load_intrinsics(args.camera_yaml)

    if args.tag_type == "apriltag":
        T_body_cam = pose_from_extrinsic(APRILTAG_T_BODY_CAM)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        tag_size = 0.15
        board = None
        image_topic = APRILTAG_TOPIC
    else:
        T_body_cam = pose_from_extrinsic(CHARUCO_T_BODY_CAM)
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, CHARUCO_DICT_NAME))
        board = cv2.aruco.CharucoBoard_create(
            CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y,
            CHARUCO_SQUARE_LENGTH, CHARUCO_MARKER_LENGTH, aruco_dict
        )
        tag_size = None
        image_topic = CHARUCO_TOPIC

    T_cam_body = np.linalg.inv(T_body_cam)

    ts = get_typestore(Stores.ROS2_JAZZY)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    detector_params = make_detector_params()
    clahe = cv2.createCLAHE(3.0, (8, 8))

    n_total = 0
    n_detected = 0
    with Reader(args.bag) as reader, args.out.open("w") as fout:
        fout.write("# timestamp tx ty tz qx qy qz qw\n")
        conns = [c for c in reader.connections if c.topic == image_topic]
        if not conns:
            print(f"[error] image topic {image_topic} not in bag",
                  file=sys.stderr)
            return 1
        if conns[0].msgtype != "sensor_msgs/msg/CompressedImage":
            print(f"[error] {image_topic} is {conns[0].msgtype}; "
                  f"this script handles CompressedImage only.", file=sys.stderr)
            return 1

        for conn, _rcv, raw in reader.messages(connections=conns):
            n_total += 1
            if (n_total - 1) % args.every_n != 0:
                continue
            msg = ts.deserialize_cdr(raw, conn.msgtype)
            img = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8),
                               cv2.IMREAD_COLOR)
            if img is None:
                continue
            gray = preprocess(img, clahe)

            if args.tag_type == "apriltag":
                T_cam_world = detect_pose_apriltag(
                    gray, aruco_dict, detector_params, tag_size, K, D)
            else:
                T_cam_world = detect_pose_charuco(
                    gray, aruco_dict, detector_params, board, K, D)
            if T_cam_world is None:
                continue

            T_world_cam = np.linalg.inv(T_cam_world)
            T_world_body = T_world_cam @ T_cam_body
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            tx, ty, tz = T_world_body[:3, 3]
            qx, qy, qz, qw = quat_from_R(T_world_body[:3, :3])
            fout.write(f"{t:.9f} {tx:.9f} {ty:.9f} {tz:.9f} "
                       f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")
            n_detected += 1

    print(f"[ok] frames={n_total} detected={n_detected} -> {args.out}")
    return 0 if n_detected > 0 else 2


if __name__ == "__main__":
    sys.exit(main())
