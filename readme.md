# Ros Ad-hoc Wifi Range

Publish inter-node range in an ad-hoc network as ros topic(s)

# Associated Paper
TODO

# Building

These ROS nodes are written solely in python using `rospy`, so builing is simple
```bash
cd /path/to/ros-adhoc-wifi-range
catkin_make install
```

# Configuration

The robots must be in an ad-hoc network for this system to function. This is due to how the `iw` exposes the RSSI readings. `wifi-fi.py` uses `iw` to get each station's RSSI, which in ad-hoc mode, means each other robot. In a traditional network, the station refers instead to the access point. If the robots are not in an ad-hoc network, a more sophisticated methods for measuring inter-robot RSSI should be developed. This repo is modular, however, so if there is desire to create such as system, refactoring or replacing `wifi-if.py` to publish RSSI to the `/rawr/rssi` topic would work with the current `range.py`.

The following ROS parameters need to be set:
* `adhoc-ifname`: the Wi-Fi interface used in the ad-hoc network, e.g. `wlan1`.
* `macs_to_monitor`: a list of MAC address to computer range for. In this case, likely the MACs of the other robots, e.g. `"[12:34:56:78:ab:cd, 09:87:65:43:32:10]"`
* `mac_inactive_threshold_ms`: threshold in ms to use stale data. If data from the `iw` command is older than this value, treat the data as stale, and do not process it.
* `rawr_calibrate_distance_m`: the distance in meters used during the calibration step, more in [calibration](#calibration)
* `rawr_calibrate_rssi_db`: the rssi value recorded during calibration

# Running

Launch the ROS nodes in two terminals:
```bash
roslaunch rawr_pkg wifi-if
```

and 

```bash
roslaunch rawr_pkg range
```


# Calibration

To get accurate measurements from the RSSI-based ranging, it is required to calibrate the link between the robots.
TODO
