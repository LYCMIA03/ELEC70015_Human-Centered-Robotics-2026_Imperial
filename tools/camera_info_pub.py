#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import CameraInfo, Image

class CameraInfoFromImageStamp:
    def __init__(self):
        self.width  = rospy.get_param("~width", 640)
        self.height = rospy.get_param("~height", 480)
        self.hfov   = rospy.get_param("~horizontal_fov", 1.047)  # rad
        self.frame  = rospy.get_param("~frame_id", "depth_camera_link")
        self.out_topic = rospy.get_param("~out_topic", "/sim_p3at/depth_camera/depth/camera_info")
        self.in_image_topic = rospy.get_param("~in_image_topic", "/sim_p3at/depth_camera/depth/image_raw")

        fx = self.width / (2.0 * math.tan(self.hfov / 2.0))
        fy = fx
        cx = self.width / 2.0
        cy = self.height / 2.0

        self.msg = CameraInfo()
        self.msg.width = self.width
        self.msg.height = self.height
        self.msg.distortion_model = "plumb_bob"
        self.msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.msg.K = [fx, 0.0, cx,
                      0.0, fy, cy,
                      0.0, 0.0, 1.0]
        self.msg.R = [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]
        self.msg.P = [fx, 0.0, cx, 0.0,
                      0.0, fy, cy, 0.0,
                      0.0, 0.0, 1.0, 0.0]

        self.pub = rospy.Publisher(self.out_topic, CameraInfo, queue_size=10)
        self.sub = rospy.Subscriber(self.in_image_topic, Image, self.cb, queue_size=10)

    def cb(self, img: Image):
        # 核心：让 camera_info 的时间戳与 image 完全一致
        self.msg.header.stamp = img.header.stamp
        self.msg.header.frame_id = self.frame
        self.pub.publish(self.msg)

def main():
    rospy.init_node("camera_info_from_image_stamp", anonymous=True)
    CameraInfoFromImageStamp()
    rospy.spin()

if __name__ == "__main__":
    main()
