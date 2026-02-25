#!/usr/bin/env python3
import json
import socket
import time

import rospy
from std_msgs.msg import Bool


def _as_int_flag(value):
    return 1 if bool(value) else 0


class NavigationSuccessUdpBridge:
    def __init__(self):
        self.in_topic = rospy.get_param("~in_topic", "/target_follower/result")
        self.out_host = rospy.get_param("~out_host", "127.0.0.1")
        self.out_port = int(rospy.get_param("~out_port", 16041))
        self.rising_edge_only = bool(rospy.get_param("~rising_edge_only", True))
        self.trigger_on_true_only = bool(rospy.get_param("~trigger_on_true_only", True))
        self.source = rospy.get_param("~source", "navigation_success_udp_bridge")

        self._last_value = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rospy.Subscriber(self.in_topic, Bool, self._cb, queue_size=10)
        rospy.loginfo(
            "navigation_success_udp_bridge: %s -> UDP %s:%s (rising_edge_only=%s, true_only=%s)",
            self.in_topic,
            self.out_host,
            self.out_port,
            self.rising_edge_only,
            self.trigger_on_true_only,
        )

    def _cb(self, msg):
        value = bool(msg.data)
        should_send = True

        if self.trigger_on_true_only and not value:
            should_send = False

        if self.rising_edge_only:
            if self._last_value is None:
                should_send = should_send and value
            else:
                should_send = should_send and (not self._last_value and value)

        self._last_value = value
        if not should_send:
            return

        payload = {
            "stamp": time.time(),
            "navigation_success": _as_int_flag(value),
            "source": self.source,
        }
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        try:
            self.sock.sendto(data, (self.out_host, self.out_port))
            rospy.loginfo_throttle(2.0, "Sent navigation_success=1 to %s:%s", self.out_host, self.out_port)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "UDP send failed: %s", str(e))


if __name__ == "__main__":
    rospy.init_node("navigation_success_udp_bridge")
    NavigationSuccessUdpBridge()
    rospy.spin()
