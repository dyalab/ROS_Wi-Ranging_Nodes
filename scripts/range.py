#!/usr/bin/env python3

import argparse
import logging
import rospy
from std_msgs.msg import String

LOG = logging.getLogger()


def rssi_relative_dist(rss, r0, pr0, n=2):
    return r0 / (10 ** ((rss - pr0) / (10 * n)))


class RangeLog:
    def __init__(self):
        self.calibrated_r0 = rospy.get_param("rawr_calibrate_distance_m", 1)
        self.calibrated_pr0 = rospy.get_param("rawr_calibrate_rssi_db", -34)

        self.pub = rospy.Publisher("/rawr/range", String, queue_size=10)
        rospy.init_node("rawr_range", anonymous=False)

    def handle_rssi(self, msg):
        mac, rssi = msg.split()
        # TODO: should this do any filtering / aggregating?
        self.pub.publish(String(mac + "," + str(rssi_relative_dist(rssi, self.calibrated_r0, self.calibrated_pr0))))

    def run(self):
        rospy.Subscriber("/rawr/rssi", String, self.handle_rssi)
        rospy.spin()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("name", help="ignored, required for compat")
    ap.add_argument("log", help="ignored, required for compat")


    args = ap.parse_args()
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(asctime)s :: %(levelname)s :: %(name)s :: %(message)s")
    RangeLog().run()


if __name__ == "__main__":
    main()
