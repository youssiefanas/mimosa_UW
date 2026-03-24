#!/usr/bin/env python3
"""
TUM Path Publisher (ROS 2) - Publishes a Path message from a TUM format file to RViz.
TUM format: timestamp tx ty tz qx qy qz qw (space-delimited)

Usage:
  python3 tum_path_publisher_ros2.py path/to/trajectory.tum [--topic_name /tum_path] [--frame_id map] [--hz 0.5]

Then in RViz2, add a Path display subscribed to /tum_path.
"""

import argparse
import csv
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


def quaternion_matrix(q):
    """Return 4x4 rotation matrix from quaternion (x, y, z, w)."""
    q = np.array(q, dtype=np.float64)
    n = np.dot(q, q)
    if n < 1e-12:
        return np.identity(4)
    q *= np.sqrt(2.0 / n)
    o = np.outer(q, q)
    return np.array([
        [1.0 - o[1, 1] - o[2, 2], o[0, 1] - o[2, 3], o[0, 2] + o[1, 3], 0.0],
        [o[0, 1] + o[2, 3], 1.0 - o[0, 0] - o[2, 2], o[1, 2] - o[0, 3], 0.0],
        [o[0, 2] - o[1, 3], o[1, 2] + o[0, 3], 1.0 - o[0, 0] - o[1, 1], 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def quaternion_from_matrix(m):
    """Return quaternion (x, y, z, w) from 4x4 rotation matrix."""
    t = np.trace(m[:3, :3])
    if t > 0.0:
        s = 0.5 / np.sqrt(t + 1.0)
        return np.array([
            (m[2, 1] - m[1, 2]) * s,
            (m[0, 2] - m[2, 0]) * s,
            (m[1, 0] - m[0, 1]) * s,
            0.25 / s,
        ])
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        return np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s,
                         (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s])
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        return np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s,
                         (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s])
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        return np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s,
                         0.25 * s, (m[1, 0] - m[0, 1]) / s])


def make_transform_matrix(pos, quat):
    """Build a 4x4 homogeneous transform from position [x,y,z] and quaternion [x,y,z,w]."""
    mat = quaternion_matrix(quat)
    mat[:3, 3] = pos
    return mat


def apply_transform(ps, transform_matrix, side="left"):
    """Apply a 4x4 transform to a PoseStamped in-place."""
    p = ps.pose
    p_mat = make_transform_matrix(
        [p.position.x, p.position.y, p.position.z],
        [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
    )
    result = (transform_matrix @ p_mat) if side == "left" else (p_mat @ transform_matrix)
    new_quat = quaternion_from_matrix(result)
    p.position.x, p.position.y, p.position.z = (float(v) for v in result[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
        float(v) for v in new_quat
    )


def sec_to_stamp(t_sec):
    """Convert float seconds to builtin_interfaces/Time."""
    stamp = Time()
    stamp.sec = int(t_sec)
    stamp.nanosec = int((t_sec - stamp.sec) * 1e9)
    return stamp


class TumPathPublisher(Node):
    def __init__(self, args):
        super().__init__("tum_path_publisher")

        # Latching equivalent in ROS 2: transient local durability
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Path, args.topic_name, qos)

        # Build transform matrix if requested
        transform_matrix = None
        if args.transform is not None:
            transform_matrix = make_transform_matrix(args.transform[:3], args.transform[3:])
            self.get_logger().info(
                f"Applying {args.transform_side} transform: {args.transform}"
            )

        # Load path from TUM file
        self.path_msg = self._load_tum_file(
            args.file_name, args.frame_id, transform_matrix, args.transform_side
        )

        self.get_logger().info(
            f"Publishing {len(self.path_msg.poses)} poses on '{args.topic_name}' "
            f"in frame '{args.frame_id}' at {args.hz} Hz"
        )

        self.timer = self.create_timer(1.0 / args.hz, self._publish)

    def _load_tum_file(self, file_path, frame_id, transform_matrix, transform_side):
        path_msg = Path()
        path_msg.header.frame_id = frame_id

        try:
            with open(file_path) as f:
                reader = csv.reader(f, delimiter=" ")
                for line in reader:
                    if not line or line[0].startswith("#"):
                        continue
                    line = [item for item in line if item]
                    if len(line) < 8:
                        self.get_logger().warn(f"Skipping invalid line: {line}")
                        continue
                    try:
                        ps = PoseStamped()
                        ps.header.frame_id = frame_id
                        ps.header.stamp = sec_to_stamp(float(line[0]))
                        ps.pose.position.x = float(line[1])
                        ps.pose.position.y = float(line[2])
                        ps.pose.position.z = float(line[3])
                        ps.pose.orientation.x = float(line[4])
                        ps.pose.orientation.y = float(line[5])
                        ps.pose.orientation.z = float(line[6])
                        ps.pose.orientation.w = float(line[7])

                        if transform_matrix is not None:
                            apply_transform(ps, transform_matrix, transform_side)

                        path_msg.poses.append(ps)
                    except ValueError as e:
                        self.get_logger().warn(f"Error parsing line {line}: {e}")

            self.get_logger().info(f"Loaded {len(path_msg.poses)} poses from {file_path}")

        except FileNotFoundError:
            self.get_logger().error(f"File not found: {file_path}")
            sys.exit(1)

        return path_msg

    def _publish(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path_msg)


def main():
    parser = argparse.ArgumentParser(
        description="Publish a TUM trajectory as nav_msgs/Path to RViz (ROS 2)"
    )
    parser.add_argument("file_name", type=str, help="Path to TUM format file")
    parser.add_argument(
        "--frame_id", type=str, default="map", help="Frame ID (default: map)"
    )
    parser.add_argument(
        "--topic_name", type=str, default="tum_path",
        help="Topic name (default: tum_path)",
    )
    parser.add_argument(
        "--hz", type=float, default=0.5, help="Publish rate in Hz (default: 0.5)"
    )
    parser.add_argument(
        "--transform", type=float, nargs=7,
        metavar=("x", "y", "z", "qx", "qy", "qz", "qw"),
        help="Transform to apply as: x y z qx qy qz qw",
        default=None,
    )
    parser.add_argument(
        "--transform_side", type=str, choices=["left", "right"], default="left",
        help="Apply transform on left or right side (default: left)",
    )

    # Separate our args from ROS args (anything after --)
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = TumPathPublisher(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
