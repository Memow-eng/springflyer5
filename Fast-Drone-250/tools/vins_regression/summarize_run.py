#!/usr/bin/env python3
import csv
import math
import os
import re
import sys


FIELDS = [
    "bag", "expect_init", "init_success", "pass",
    "odom_count", "imu_count",
    "first_stamp", "last_stamp", "duration",
    "end_norm", "max_rel", "max_step", "max_speed",
    "imu_first_cov0", "imu_last_cov0",
    "init_finish_count", "gate_accept_count", "sanity_reject_count", "failure_count",
    "run_dir",
]


def read_rows(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                rows.append({
                    k: float(row[k])
                    for k in ["stamp", "x", "y", "z", "vx", "vy", "vz", "pose_cov0"]
                })
            except (KeyError, TypeError, ValueError):
                continue
    return rows


def trajectory_metrics(rows):
    if not rows:
        return {
            "count": 0, "first_stamp": "", "last_stamp": "", "duration": "",
            "end_norm": "", "max_rel": "", "max_step": "", "max_speed": "",
            "first_cov0": "", "last_cov0": "",
        }

    p0 = rows[0]
    last = rows[-1]
    max_rel = 0.0
    max_step = 0.0
    max_speed = 0.0
    prev = None
    for row in rows:
        rel = math.sqrt((row["x"] - p0["x"]) ** 2 +
                        (row["y"] - p0["y"]) ** 2 +
                        (row["z"] - p0["z"]) ** 2)
        max_rel = max(max_rel, rel)
        speed = math.sqrt(row["vx"] ** 2 + row["vy"] ** 2 + row["vz"] ** 2)
        max_speed = max(max_speed, speed)
        if prev is not None:
            step = math.sqrt((row["x"] - prev["x"]) ** 2 +
                             (row["y"] - prev["y"]) ** 2 +
                             (row["z"] - prev["z"]) ** 2)
            max_step = max(max_step, step)
        prev = row

    end_norm = math.sqrt((last["x"] - p0["x"]) ** 2 +
                         (last["y"] - p0["y"]) ** 2 +
                         (last["z"] - p0["z"]) ** 2)
    return {
        "count": len(rows),
        "first_stamp": f"{p0['stamp']:.6f}",
        "last_stamp": f"{last['stamp']:.6f}",
        "duration": f"{last['stamp'] - p0['stamp']:.3f}",
        "end_norm": f"{end_norm:.4f}",
        "max_rel": f"{max_rel:.4f}",
        "max_step": f"{max_step:.4f}",
        "max_speed": f"{max_speed:.4f}",
        "first_cov0": f"{p0['pose_cov0']:.6g}",
        "last_cov0": f"{last['pose_cov0']:.6g}",
    }


def count_log(log_path, pattern):
    if not os.path.exists(log_path):
        return 0
    regex = re.compile(pattern)
    count = 0
    with open(log_path, errors="ignore") as f:
        for line in f:
            if regex.search(line):
                count += 1
    return count


def main():
    if len(sys.argv) != 4:
        print("usage: summarize_run.py RUN_DIR BAG EXPECT_INIT", file=sys.stderr)
        return 2

    run_dir, bag, expect_init = sys.argv[1:4]
    records = os.path.join(run_dir, "records")
    odom_rows = read_rows(os.path.join(records, "vins_fusion__odometry.csv"))
    imu_rows = read_rows(os.path.join(records, "vins_fusion__imu_propagate.csv"))
    odom = trajectory_metrics(odom_rows)
    imu = trajectory_metrics(imu_rows)

    log_path = os.path.join(run_dir, "vins_launch.log")
    init_finish_count = count_log(log_path, r"Initialization finish")
    gate_accept_count = count_log(log_path, r"stereo init (gate|prefilter) accept")
    sanity_reject_count = count_log(log_path, r"stereo init (sanity|reprojection) reject")
    failure_count = count_log(log_path, r"failure detection|VINS failure")
    init_success = int(bool(odom_rows) or init_finish_count > 0)

    if expect_init in ("0", "1"):
        passed = (init_success == int(expect_init)) and failure_count == 0
    else:
        passed = failure_count == 0

    row = {
        "bag": os.path.basename(bag),
        "expect_init": expect_init,
        "init_success": int(init_success),
        "pass": int(passed),
        "odom_count": odom["count"],
        "imu_count": imu["count"],
        "first_stamp": odom["first_stamp"],
        "last_stamp": odom["last_stamp"],
        "duration": odom["duration"],
        "end_norm": odom["end_norm"],
        "max_rel": odom["max_rel"],
        "max_step": odom["max_step"],
        "max_speed": odom["max_speed"],
        "imu_first_cov0": imu["first_cov0"],
        "imu_last_cov0": imu["last_cov0"],
        "init_finish_count": init_finish_count,
        "gate_accept_count": gate_accept_count,
        "sanity_reject_count": sanity_reject_count,
        "failure_count": failure_count,
        "run_dir": run_dir,
    }
    writer = csv.DictWriter(sys.stdout, fieldnames=FIELDS)
    writer.writerow(row)
    return 0


if __name__ == "__main__":
    sys.exit(main())
