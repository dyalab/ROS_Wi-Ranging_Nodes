#!/usr/bin/env python

import argparse
import logging
import rospy
from std_msgs.msg import String

LOG = logging.getLogger()


class Calibrate:
    def __init__(self):
        rospy.init_node("rawr_cal", anonymous=False)
        self.N = 10  # average over 10
        self.captured = list()

    def handle_rssi(self, msg):

        mac, rssi = msg.split()
        self.captured.append(rssi)
        if len(self.captured >= self.N):
            rssi_sum = sum([float(v) for v in self.captured])
            rssi_avg = rssi_sum / len(self.captured)
            print(f"Average RSSI: {rssi_avg}")
            print(f"To apply this value for future ranging tests, run the following:")
            print(f"rosparam set rawr_calibrate_rssi_db {rssi_avg}")
            rospy.signal_shutdown("Complete.")

    def run(self):
        rospy.Subscriber("/rawr/rssi", String, self.handle_rssi)
        rospy.spin()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", "-v", action="store_true")

    args = ap.parse_args()
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(asctime)s :: %(levelname)s :: %(name)s :: %(message)s")
    Calibrate().run()


if __name__ == "__main__":
    main()
