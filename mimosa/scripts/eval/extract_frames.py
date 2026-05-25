#!/usr/bin/env python3
"""Extract specific camera frames from a ROS 2 bag.
Extracts frames at:
- 15 seconds after bag start
- 40% of the bag duration
- 90% of the bag duration
for given compressed image topics.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", type=Path, help="Input ROS 2 bag directory.")
    ap.add_argument("outdir", type=Path, help="Output directory for images.")
    ap.add_argument("--topics", nargs="+", 
                    default=["/cam1/image_raw/compressed", "/cam2/image_raw/compressed"],
                    help="Camera topics to extract.")
    args = ap.parse_args()

    if not args.bag.exists():
        print(f"[error] {args.bag} does not exist.", file=sys.stderr)
        return 1

    args.outdir.mkdir(parents=True, exist_ok=True)

    ts = get_typestore(Stores.ROS2_JAZZY)

    with Reader(args.bag) as reader:
        # Check topics
        conns = [c for c in reader.connections if c.topic in args.topics]
        if not conns:
            print(f"[error] No requested topics found in {args.bag}", file=sys.stderr)
            return 1

        start_time = reader.start_time
        end_time = reader.end_time
        duration = end_time - start_time

        if duration <= 0:
            print(f"[error] Bag {args.bag} has zero or negative duration.", file=sys.stderr)
            return 1

        target_times = {
            "15s": start_time + 15 * 1e9,
            "40pct": start_time + 0.4 * duration,
            "90pct": start_time + 0.9 * duration,
        }

        # For each topic and each target time, store (best_diff, best_msg, best_ext)
        best_matches = {
            topic: {
                label: {"diff": float("inf"), "data": None, "format": None}
                for label in target_times
            }
            for topic in args.topics
        }

        print(f"[info] Reading {len(conns)} connection(s) from {args.bag.name}...")
        for conn, timestamp, raw in reader.messages(connections=conns):
            topic = conn.topic
            # Find the closest target time
            for label, t_target in target_times.items():
                diff = abs(timestamp - t_target)
                if diff < best_matches[topic][label]["diff"]:
                    # Decode only when we need to update to save time, actually decoding is fast enough
                    # or we can defer deserialization. Let's just keep the raw and deserialize later.
                    best_matches[topic][label]["diff"] = diff
                    best_matches[topic][label]["raw"] = raw
                    best_matches[topic][label]["msgtype"] = conn.msgtype

        # Write out the best matches
        n_written = 0
        timestamps_path = args.outdir / "timestamps.txt"
        with timestamps_path.open("w") as f_ts:
            f_ts.write("# filename timestamp(s)\n")
            for topic in args.topics:
                topic_clean = topic.strip("/").replace("/", "_")
                for label in target_times:
                    match = best_matches[topic][label]
                    if "raw" not in match:
                        continue
                    
                    msg = ts.deserialize_cdr(match["raw"], match["msgtype"])
                    
                    # CompressImage has format and data
                    fmt = msg.format.lower()
                    ext = ".jpg" if "jpeg" in fmt or "jpg" in fmt else ".png"
                    if "png" in fmt:
                        ext = ".png"

                    out_name = f"{args.bag.name}_{topic_clean}_{label}{ext}"
                    out_path = args.outdir / out_name
                    
                    with out_path.open("wb") as f:
                        f.write(msg.data)
                    
                    # Extract relative timestamp in seconds from message header
                    t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    t_rel = t_sec - (start_time * 1e-9)
                    f_ts.write(f"{out_name} {t_rel:.6f}\n")
                    
                    n_written += 1
                    print(f"[ok] Wrote {out_name} (diff: {match['diff']/1e9:.3f}s)")
        
        if n_written == 0:
            print(f"[warn] No frames written for {args.bag.name}")
            timestamps_path.unlink(missing_ok=True)
            return 1

    return 0

if __name__ == "__main__":
    sys.exit(main())
