#!/usr/bin/env python3
import json
import socket

import rospy
from geometry_msgs.msg import PointStamped


class UdpTargetBridge:
    def __init__(self):
        self.bind_host = rospy.get_param("~bind_host", "0.0.0.0")
        self.bind_port = int(rospy.get_param("~bind_port", 15001))
        self.out_topic = rospy.get_param("~out_topic", "/trash_detection/target_point")
        self.default_frame = rospy.get_param("~default_frame", "camera_link")
        self.stamp_now = bool(rospy.get_param("~stamp_now", True))

        self.pub = rospy.Publisher(self.out_topic, PointStamped, queue_size=10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.bind_host, self.bind_port))
        self.sock.settimeout(0.2)
        rospy.loginfo("udp_target_bridge listening on %s:%s -> %s", self.bind_host, self.bind_port, self.out_topic)

    def _to_msg(self, payload):
        msg = PointStamped()
        msg.header.frame_id = str(payload.get("frame_id") or self.default_frame)

        if self.stamp_now:
            msg.header.stamp = rospy.Time.now()
        else:
            t = payload.get("stamp", 0.0)
            sec = int(t)
            nsec = int((float(t) - sec) * 1e9)
            msg.header.stamp = rospy.Time(sec, max(0, nsec))

        msg.point.x = float(payload["x"])
        msg.point.y = float(payload["y"])
        msg.point.z = float(payload["z"])
        return msg

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
                payload = json.loads(data.decode("utf-8"))
                msg = self._to_msg(payload)
            except Exception as e:
                rospy.logwarn_throttle(2.0, "Bad payload from %s: %s", addr, str(e))
                continue

            self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("udp_target_bridge")
    UdpTargetBridge().spin()
