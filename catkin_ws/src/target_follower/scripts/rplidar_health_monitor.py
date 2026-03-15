#!/usr/bin/env python3
import os
import pwd
import grp
import sys

import rospy
from sensor_msgs.msg import LaserScan


class RplidarHealthMonitor:
    def __init__(self):
        self.device = rospy.get_param("~device", "/dev/ttyUSB0")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.startup_timeout = float(rospy.get_param("~startup_timeout", 20.0))
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 3.0))
        self.check_interval = float(rospy.get_param("~check_interval", 0.2))

        self.last_msg_time = None
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)

    def _scan_cb(self, _msg):
        self.last_msg_time = rospy.Time.now()

    def _fail(self, message):
        rospy.logfatal(message)
        sys.exit(1)

    def _check_device(self):
        if not os.path.exists(self.device):
            self._fail("[rplidar_health_monitor] device not found: %s" % self.device)

        can_read = os.access(self.device, os.R_OK)
        can_write = os.access(self.device, os.W_OK)
        if can_read and can_write:
            return

        st = os.stat(self.device)
        owner = pwd.getpwuid(st.st_uid).pw_name
        group = grp.getgrgid(st.st_gid).gr_name
        mode = oct(st.st_mode & 0o777)
        user_groups = [g.gr_name for g in grp.getgrall() if os.getenv("USER") in g.gr_mem]

        self._fail(
            "[rplidar_health_monitor] no rw permission for %s (owner=%s group=%s mode=%s user=%s extra_groups=%s)"
            % (self.device, owner, group, mode, os.getenv("USER"), user_groups)
        )

    def _wait_startup_scan(self):
        start = rospy.Time.now()
        timeout = rospy.Duration.from_sec(self.startup_timeout)
        rate = rospy.Rate(max(1.0, 1.0 / self.check_interval))

        while not rospy.is_shutdown():
            if self.last_msg_time is not None:
                rospy.loginfo("[rplidar_health_monitor] first scan received on %s", self.scan_topic)
                return
            if rospy.Time.now() - start > timeout:
                self._fail(
                    "[rplidar_health_monitor] no scan on %s within %.1fs"
                    % (self.scan_topic, self.startup_timeout)
                )
            rate.sleep()

    def spin(self):
        self._check_device()
        rospy.loginfo("[rplidar_health_monitor] device and permission check passed: %s", self.device)
        self._wait_startup_scan()

        rate = rospy.Rate(max(1.0, 1.0 / self.check_interval))
        stale = rospy.Duration.from_sec(self.stale_timeout)
        while not rospy.is_shutdown():
            if self.last_msg_time is None or (rospy.Time.now() - self.last_msg_time) > stale:
                self._fail(
                    "[rplidar_health_monitor] scan stream stale on %s (timeout %.1fs)"
                    % (self.scan_topic, self.stale_timeout)
                )
            rate.sleep()


def main():
    rospy.init_node("rplidar_health_monitor")
    monitor = RplidarHealthMonitor()
    monitor.spin()


if __name__ == "__main__":
    main()
