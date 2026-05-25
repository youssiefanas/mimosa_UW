#!/usr/bin/env python3
"""Extract a single nav_msgs/Odometry (or PoseStamped) topic to TUM.

Strict variant of `mavros_to_tum.py` — no topic fallbacks, fails loudly if
the requested topic is missing. Used to dump MIMOSA / VO / any other pose
topic from a bag.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


def extract_pose(msg, msgtype: str):
    if msgtype == "nav_msgs/msg/Odometry":
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        h = msg.header
    elif msgtype == "geometry_msgs/msg/PoseStamped":
        p = msg.pose.position
        o = msg.pose.orientation
        h = msg.header
    elif msgtype == "geometry_msgs/msg/PoseWithCovarianceStamped":
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        h = msg.header
    else:
        raise ValueError(f"unsupported msgtype: {msgtype}")
    t = h.stamp.sec + h.stamp.nanosec * 1e-9
    return t, (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", type=Path, help="Input ROS 2 bag directory.")
    ap.add_argument("out", type=Path, help="Output TUM file.")
    ap.add_argument("--topic", required=True, help="Pose-bearing topic.")
    args = ap.parse_args()

    ts = get_typestore(Stores.ROS2_JAZZY)
    with Reader(args.bag) as reader:
        conns = [c for c in reader.connections if c.topic == args.topic]
        if not conns:
            print(f"[error] topic {args.topic} not in {args.bag}",
                  file=sys.stderr)
            return 1
        if sum(c.msgcount for c in conns) == 0:
            print(f"[error] topic {args.topic} has 0 messages in {args.bag}",
                  file=sys.stderr)
            return 1

        msgtype = conns[0].msgtype
        print(f"[info] reading {args.topic} ({msgtype})", file=sys.stderr)

        args.out.parent.mkdir(parents=True, exist_ok=True)
        n = 0
        with args.out.open("w") as fout:
            fout.write("# timestamp tx ty tz qx qy qz qw\n")
            for conn, _ts, raw in reader.messages(connections=conns):
                msg = ts.deserialize_cdr(raw, conn.msgtype)
                t, (tx, ty, tz), (qx, qy, qz, qw) = extract_pose(msg, msgtype)
                fout.write(f"{t:.9f} {tx:.9f} {ty:.9f} {tz:.9f} "
                           f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")
                n += 1
        print(f"[ok] wrote {n} poses -> {args.out}")
    return 0 if n > 0 else 2


if __name__ == "__main__":
    sys.exit(main())
