#!/usr/bin/env python

import argparse
import logging
import rospy
from ros_msgs.msg import KeyValue

LOG = logging.getLogger()


def rssi_relative_dist(rss, r0, pr0, n=2):
    return r0 / (10 ** ((rss - pr0) / (10 * n)))


class RangeLog:
    def __init__(self):
        self.calibrated_r0 = rospy.get_param("rawr_calibrate_distance_m")
        self.calibrated_pr0 = rospy.get_param("rawr_calibrate_rssi_db")

        self.pub = rospy.Publisher("/rawr/range", KeyValue, queue_size=10)
        rospy.init_node("rawr_range", anonymous=False)

    def handle_rssi(self, msg):
        mac = msg.key
        rssi = msg.value
        # TODO: should this do any filtering / aggregating?
        self.pub.publish(KeyValue(mac, str(rssi_relative_dist(rssi, self.calibrated_r0, self.calibrated_pr0))))

    def run(self):
        rospy.Subscriber("/rawr/rssi", KeyValue, self.handle_rssi)
        rospy.spin()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", "-v", action="store_true")

    args = ap.parse_args()
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(asctime)s :: %(levelname)s :: %(name)s :: %(message)s")
    RangeLog().run()


if __name__ == "__main__":
    main()
