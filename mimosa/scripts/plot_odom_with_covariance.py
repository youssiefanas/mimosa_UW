#!/usr/bin/env python3

# Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree.

"""Plot mean and ±kσ covariance bands of nav_msgs/Odometry.

Live:    python3 plot_odom_with_covariance.py
Offline: python3 plot_odom_with_covariance.py --bag <bag_dir> [--save fig.png]
"""

import argparse
import os
import sys
import threading
from dataclasses import dataclass, field

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float):
    """ROS REP-103 quaternion → roll, pitch, yaw (rad)."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


@dataclass
class OdomBuffer:
    """Thread-safe parallel arrays of mean + sigma per DoF."""

    t: list = field(default_factory=list)
    # means
    x: list = field(default_factory=list)
    y: list = field(default_factory=list)
    z: list = field(default_factory=list)
    roll: list = field(default_factory=list)
    pitch: list = field(default_factory=list)
    yaw: list = field(default_factory=list)
    vx: list = field(default_factory=list)
    vy: list = field(default_factory=list)
    vz: list = field(default_factory=list)
    # sigmas
    sx: list = field(default_factory=list)
    sy: list = field(default_factory=list)
    sz: list = field(default_factory=list)
    sroll: list = field(default_factory=list)
    spitch: list = field(default_factory=list)
    syaw: list = field(default_factory=list)
    svx: list = field(default_factory=list)
    svy: list = field(default_factory=list)
    svz: list = field(default_factory=list)
    lock: threading.Lock = field(default_factory=threading.Lock)
    t0: float = None  # type: ignore

    def append(self, msg) -> None:
        ts = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        roll, pitch, yaw = quat_to_rpy(o.x, o.y, o.z, o.w)

        # ROS pose.covariance ordering: [x, y, z, rx, ry, rz] → diag at 0,7,14,21,28,35.
        # twist.covariance only has the top-left 3x3 (linear) populated.
        pcov = msg.pose.covariance
        tcov = msg.twist.covariance

        def s(d):
            return float(np.sqrt(max(d, 0.0)))

        with self.lock:
            if self.t0 is None:
                self.t0 = ts
            self.t.append(ts - self.t0)
            self.x.append(p.x); self.y.append(p.y); self.z.append(p.z)
            self.roll.append(np.degrees(roll))
            self.pitch.append(np.degrees(pitch))
            self.yaw.append(np.degrees(yaw))
            self.vx.append(v.x); self.vy.append(v.y); self.vz.append(v.z)
            self.sx.append(s(pcov[0])); self.sy.append(s(pcov[7])); self.sz.append(s(pcov[14]))
            # rotation sigma in radians → degrees for display parity with the mean
            self.sroll.append(np.degrees(s(pcov[21])))
            self.spitch.append(np.degrees(s(pcov[28])))
            self.syaw.append(np.degrees(s(pcov[35])))
            self.svx.append(s(tcov[0])); self.svy.append(s(tcov[7])); self.svz.append(s(tcov[14]))

    def snapshot(self):
        """Return a numpy-array snapshot under the lock for plotting."""
        with self.lock:
            return {k: np.asarray(getattr(self, k), dtype=float)
                    for k in ('t', 'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                              'vx', 'vy', 'vz',
                              'sx', 'sy', 'sz', 'sroll', 'spitch', 'syaw',
                              'svx', 'svy', 'svz')}


# 3x3 layout: (axis row, axis col, mean key, sigma key, title, unit)
PANELS = [
    (0, 0, 'x',     'sx',     'position x',    'm'),
    (0, 1, 'y',     'sy',     'position y',    'm'),
    (0, 2, 'z',     'sz',     'position z',    'm'),
    (1, 0, 'roll',  'sroll',  'roll',          'deg'),
    (1, 1, 'pitch', 'spitch', 'pitch',         'deg'),
    (1, 2, 'yaw',   'syaw',   'yaw',           'deg'),
    (2, 0, 'vx',    'svx',    'velocity vx (B)', 'm/s'),
    (2, 1, 'vy',    'svy',    'velocity vy (B)', 'm/s'),
    (2, 2, 'vz',    'svz',    'velocity vz (B)', 'm/s'),
]


def make_figure():
    fig, axes = plt.subplots(3, 3, sharex=True, figsize=(14, 9))
    for r, c, _, _, title, unit in PANELS:
        ax = axes[r, c]
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.grid(True, alpha=0.3)
        if r == 2:
            ax.set_xlabel('time [s]')
    fig.tight_layout()
    return fig, axes


# ANGLE_KEYS = {'roll', 'pitch', 'yaw'}


def redraw(axes, snap, k_sigma: float):
    if snap['t'].size == 0:
        return
    t = snap['t']
    for r, c, mkey, skey, title, unit in PANELS:
        ax = axes[r, c]
        m = snap[mkey]
        # # Unwrap RPY so visualization doesn't flip ±180° at the wrap boundary.
        # # Sigma stays in local tangent space — no wrap, no transformation needed.
        # if mkey in ANGLE_KEYS and m.size > 1:
        #     m = np.unwrap(m, period=360.0)
        s = snap[skey]
        ax.cla()
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.grid(True, alpha=0.3)
        if r == 2:
            ax.set_xlabel('time [s]')
        ax.plot(t, m, linewidth=1.2)
        ax.fill_between(t, m - k_sigma * s, m + k_sigma * s, alpha=0.25)
        if t.size > 1:
            ax.set_xlim(t[0], t[-1])


def run_live(topic: str, k_sigma: float) -> None:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = Node('odom_cov_plotter')
    buf = OdomBuffer()

    node.create_subscription(Odometry, topic, buf.append, 50)
    node.get_logger().info(f"subscribed to {topic}")

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, axes = make_figure()
    fig.suptitle(f"{topic}  ( ±{k_sigma:g}σ)", fontsize=11)
    fig.subplots_adjust(top=0.93)

    def _frame(_):
        redraw(axes, buf.snapshot(), k_sigma)

    ani = animation.FuncAnimation(fig, _frame, interval=200, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    _ = ani  # keep reference alive


def _detect_storage_id(bag_path: str) -> str:
    if os.path.isdir(bag_path):
        for name in os.listdir(bag_path):
            if name.endswith('.mcap'):
                return 'mcap'
            if name.endswith('.db3'):
                return 'sqlite3'
    if bag_path.endswith('.mcap'):
        return 'mcap'
    if bag_path.endswith('.db3'):
        return 'sqlite3'
    return 'sqlite3'


def run_offline(bag_path: str, topic: str, k_sigma: float, save: str | None) -> None:
    import rclpy  # noqa: F401  (needed for typesupport registration)
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py

    storage_id = _detect_storage_id(bag_path)
    storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    type_by_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_by_topic:
        sys.exit(f"topic {topic!r} not found in bag. Available: {sorted(type_by_topic)}")
    msg_type = get_message(type_by_topic[topic])

    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    buf = OdomBuffer()
    n = 0
    while reader.has_next():
        tname, raw, _t = reader.read_next()
        if tname != topic:
            continue
        msg = deserialize_message(raw, msg_type)
        buf.append(msg)
        n += 1
    print(f"loaded {n} messages from {bag_path}")

    fig, axes = make_figure()
    fig.suptitle(f"{topic}  ({os.path.basename(bag_path.rstrip('/'))}, ±{k_sigma:g}σ)", fontsize=11)
    fig.subplots_adjust(top=0.93)
    redraw(axes, buf.snapshot(), k_sigma)
    if save:
        fig.savefig(save, dpi=150, bbox_inches='tight')
        print(f"saved {save}")
    else:
        plt.show()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--topic', default='/mimosa_node/graph/odometry')
    p.add_argument('--bag', default=None,
                   help='rosbag2 directory (offline mode). If omitted, runs in live mode.')
    p.add_argument('--sigma', type=float, default=2.0,
                   help='band half-width in sigma units (default 2.0 for 95%% confidence)')
    p.add_argument('--save', default=None,
                   help='write figure to this path (offline mode only)')
    args = p.parse_args()

    if args.bag:
        run_offline(args.bag, args.topic, args.sigma, args.save)
    else:
        if args.save:
            print('--save is only supported in offline (--bag) mode; ignoring.', file=sys.stderr)
        run_live(args.topic, args.sigma)


if __name__ == '__main__':
    main()
