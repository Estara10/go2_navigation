#!/usr/bin/env python3
"""Extract VINS odometry from a ROS 2 sqlite bag to the CSV used by go2_vins_replay.

Usage:
  source /opt/ros/humble/setup.bash
  python3 scripts/go2_nav_plan/tools/extract_vins_bag.py \
    --bag /path/to/bag_dir --topic /vins/odometry --out /tmp/odometry.csv

The bag is opened read-only. This script does not create a ROS node and does not
initialize DDS. It only reads SQLite records and deserializes Odometry messages.
"""

import argparse
import csv
import glob
import os
import sqlite3
import sys

import rclpy.serialization
from nav_msgs.msg import Odometry


def find_db(bag_path):
    if os.path.isfile(bag_path) and bag_path.endswith(".db3"):
        return bag_path
    if os.path.isdir(bag_path):
        dbs = sorted(glob.glob(os.path.join(bag_path, "*.db3")))
        if len(dbs) == 1:
            return dbs[0]
        if not dbs:
            raise SystemExit(f"no .db3 file in {bag_path}")
        raise SystemExit(f"multiple .db3 files in {bag_path}; pass one explicitly")
    raise SystemExit(f"bag path not found: {bag_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="bag directory or .db3 file")
    parser.add_argument("--topic", default="/vins/odometry")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    db = find_db(args.bag)
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    cur = con.cursor()
    topics = {name: tid for tid, name in cur.execute("select id, name from topics")}
    if args.topic not in topics:
        print(f"topic not found: {args.topic}", file=sys.stderr)
        print("available: " + ", ".join(sorted(topics)), file=sys.stderr)
        return 1

    rows = cur.execute(
        "select timestamp, data from messages where topic_id=? order by id",
        (topics[args.topic],),
    ).fetchall()

    with open(args.out, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            "bag_ns", "header_s", "x", "y", "z", "qx", "qy", "qz", "qw",
            "vx", "vy", "vz", "wx", "wy", "wz", "frame_id", "child_frame_id",
        ])
        for bag_ns, data in rows:
            msg = rclpy.serialization.deserialize_message(bytes(data), Odometry)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            v = msg.twist.twist.linear
            w = msg.twist.twist.angular
            writer.writerow([
                bag_ns, f"{stamp:.9f}", p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                v.x, v.y, v.z, w.x, w.y, w.z,
                msg.header.frame_id, msg.child_frame_id,
            ])

    print(f"{args.topic}: {len(rows)} rows -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
