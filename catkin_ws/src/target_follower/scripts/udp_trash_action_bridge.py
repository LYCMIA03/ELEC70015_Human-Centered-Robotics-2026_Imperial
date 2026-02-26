#!/usr/bin/env python3
import json
import socket

import rospy
from std_msgs.msg import Bool


def _parse_boolish(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(int(value))
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("1", "true", "yes", "y", "proceed"):
            return True
        if text in ("0", "false", "no", "n", "decline", "terminate"):
            return False
    return None


def _extract_action(payload):
    if isinstance(payload, dict):
        for key in ("trash_action", "action", "value", "result"):
            if key in payload:
                parsed = _parse_boolish(payload[key])
                if parsed is not None:
                    return parsed
        for key in ("decision", "outcome"):
            if key in payload:
                parsed = _parse_boolish(payload[key])
                if parsed is not None:
                    return parsed
        return None
    return _parse_boolish(payload)


class UdpTrashActionBridge:
    def __init__(self):
        self.bind_host = rospy.get_param("~bind_host", "0.0.0.0")
        self.bind_port = int(rospy.get_param("~bind_port", 16032))
        self.out_topic = rospy.get_param("~out_topic", "/trash_action")

        self.pub = rospy.Publisher(self.out_topic, Bool, queue_size=10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((self.bind_host, self.bind_port))
        except OSError as e:
            if getattr(e, "errno", None) == 98:
                rospy.logerr(
                    "UDP bind failed on %s:%s (address already in use). "
                    "Another udp_trash_action_bridge may still be running.",
                    self.bind_host,
                    self.bind_port,
                )
            raise
        self.sock.settimeout(0.2)
        rospy.loginfo(
            "udp_trash_action_bridge listening on %s:%s -> %s",
            self.bind_host,
            self.bind_port,
            self.out_topic,
        )

    def spin(self):
        while not rospy.is_shutdown():
            try:
                data, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except Exception as e:
                rospy.logwarn_throttle(2.0, "UDP recv error: %s", str(e))
                continue

            try:
                text = data.decode("utf-8").strip()
                payload = json.loads(text) if text.startswith("{") else text
                action = _extract_action(payload)
                if action is None:
                    raise ValueError("No parseable action field in payload")
            except Exception as e:
                rospy.logwarn_throttle(2.0, "Bad payload from %s: %s", addr, str(e))
                continue

            self.pub.publish(Bool(data=bool(action)))
            rospy.loginfo_throttle(1.0, "Published %s=%s", self.out_topic, bool(action))


if __name__ == "__main__":
    rospy.init_node("udp_trash_action_bridge")
    UdpTrashActionBridge().spin()
