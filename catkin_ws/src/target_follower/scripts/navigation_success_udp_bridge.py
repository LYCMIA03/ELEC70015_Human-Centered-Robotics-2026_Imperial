#!/usr/bin/env python3
import json
import socket
import time

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String


class NavigationSuccessUdpBridge:
    def __init__(self):
        self.in_topic = rospy.get_param("~in_topic", "/target_follower/result")
        self.status_topic = rospy.get_param("~status_topic", "/target_follower/status")
        self.action_topic = rospy.get_param("~action_topic", "/trash_action")
        self.out_host = rospy.get_param("~out_host", "127.0.0.1")
        self.out_port = int(rospy.get_param("~out_port", 16041))
        self.rising_edge_only = bool(rospy.get_param("~rising_edge_only", True))
        self.trigger_on_true_only = bool(rospy.get_param("~trigger_on_true_only", True))
        self.enable_waiting_action_retrigger = bool(
            rospy.get_param("~enable_waiting_action_retrigger", True)
        )
        self.resend_interval_s = float(rospy.get_param("~resend_interval_s", 2.0))
        self.source = rospy.get_param("~source", "navigation_success_udp_bridge")

        self._last_value = None
        self._in_waiting_action = False
        self._last_send_ts = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rospy.Subscriber(self.in_topic, Bool, self._cb, queue_size=10)
        rospy.Subscriber(self.status_topic, String, self._status_cb, queue_size=10)
        rospy.Subscriber(self.action_topic, Bool, self._action_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.2), self._tick)
        rospy.loginfo(
            "navigation_success_udp_bridge: %s -> UDP %s:%s "
            "(rising_edge_only=%s, true_only=%s, retrigger=%s/%ss)",
            self.in_topic,
            self.out_host,
            self.out_port,
            self.rising_edge_only,
            self.trigger_on_true_only,
            self.enable_waiting_action_retrigger,
            self.resend_interval_s,
        )

    def _send_trigger(self, reason):
        payload = {
            "stamp": time.time(),
            "navigation_success": 1,
            "reason": reason,
            "source": self.source,
        }
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        try:
            self.sock.sendto(data, (self.out_host, self.out_port))
            self._last_send_ts = time.time()
            rospy.loginfo_throttle(
                1.0,
                "Sent navigation_success=1 to %s:%s (%s)",
                self.out_host,
                self.out_port,
                reason,
            )
        except Exception as e:
            rospy.logwarn_throttle(2.0, "UDP send failed: %s", str(e))

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
        if should_send:
            self._send_trigger("result_cb")

    def _status_cb(self, msg):
        self._in_waiting_action = (msg.data.strip() == "WAITING_ACTION")

    def _action_cb(self, _msg):
        # Dialogue result reached ROS; stop any retrigger spam for this cycle.
        self._in_waiting_action = False

    def _tick(self, _event):
        if not self.enable_waiting_action_retrigger:
            return
        if not self._in_waiting_action:
            return
        now = time.time()
        if (now - self._last_send_ts) < max(0.2, self.resend_interval_s):
            return
        self._send_trigger("waiting_action_retry")


if __name__ == "__main__":
    rospy.init_node("navigation_success_udp_bridge")
    NavigationSuccessUdpBridge()
    rospy.spin()
