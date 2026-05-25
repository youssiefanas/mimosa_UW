#!/usr/bin/env python3
"""Extract a MAVROS local-position trajectory from a ROS 2 bag to TUM format.

Reads a `nav_msgs/Odometry` or `geometry_msgs/PoseStamped` topic (default
`/bluerov2/local_position/odom`) and writes one TUM line per message:

    # timestamp tx ty tz qx qy qz qw

If the requested topic is missing the script falls back through the other
local_position topics in priority order (odom > pose > pose_cov).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


FALLBACK_TOPICS = (
    "/bluerov2/local_position/odom",
    "/bluerov2/local_position/pose",
    "/bluerov2/local_position/pose_cov",
)


def pick_topic(connections, requested: str) -> str | None:
    by_topic = {c.topic: c for c in connections if c.msgcount > 0}
    if requested in by_topic:
        return requested
    for fb in FALLBACK_TOPICS:
        if fb in by_topic:
            return fb
    return None


def extract_pose(msg, msgtype: str):
    """Return ((tx,ty,tz),(qx,qy,qz,qw)) and stamp seconds for a pose-bearing msg."""
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
    ap.add_argument("--topic", default="/bluerov2/local_position/odom",
                    help="Preferred topic. Falls back if missing.")
    args = ap.parse_args()

    ts = get_typestore(Stores.ROS2_JAZZY)
    with Reader(args.bag) as reader:
        topic = pick_topic(reader.connections, args.topic)
        if topic is None:
            print(f"[error] no MAVROS local-position topic found in {args.bag}",
                  file=sys.stderr)
            return 1
        if topic != args.topic:
            print(f"[warn] requested {args.topic} not present; using {topic}",
                  file=sys.stderr)

        conns = [c for c in reader.connections if c.topic == topic]
        msgtype = conns[0].msgtype
        print(f"[info] reading {topic} ({msgtype})", file=sys.stderr)

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
    return 0


if __name__ == "__main__":
    sys.exit(main())
