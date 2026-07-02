#!/usr/bin/env python3
import csv
import os
import sys

import rospy
from nav_msgs.msg import Odometry


class Recorder:
    def __init__(self, topic, path):
        self.count = 0
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "recv_time", "stamp",
            "x", "y", "z", "qx", "qy", "qz", "qw",
            "vx", "vy", "vz", "pose_cov0",
        ])
        self.sub = rospy.Subscriber(topic, Odometry, self.cb, queue_size=1000, tcp_nodelay=True)

    def cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        self.writer.writerow([
            rospy.Time.now().to_sec(), msg.header.stamp.to_sec(),
            p.x, p.y, p.z, q.x, q.y, q.z, q.w,
            v.x, v.y, v.z, msg.pose.covariance[0],
        ])
        self.count += 1
        self.file.flush()

    def close(self):
        self.file.flush()
        self.file.close()


def main():
    if len(sys.argv) < 3:
        print("usage: record_odom_topics.py OUT_DIR TOPIC [TOPIC...]", file=sys.stderr)
        return 2

    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    rospy.init_node("vins_regression_odom_recorder", anonymous=True)

    recorders = []
    for topic in sys.argv[2:]:
        safe = topic.strip("/").replace("/", "__") or "root"
        recorders.append(Recorder(topic, os.path.join(out_dir, safe + ".csv")))

    rospy.logwarn("recording odometry topics: %s", ", ".join(sys.argv[2:]))
    try:
        rospy.spin()
    finally:
        for recorder in recorders:
            recorder.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
