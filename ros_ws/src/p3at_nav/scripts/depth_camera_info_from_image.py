#!/usr/bin/env python3
"""
Generate camera_info from depth image for perfect synchronization.
"""
import rospy
from sensor_msgs.msg import Image, CameraInfo

class DepthCameraInfoGenerator:
    def __init__(self):
        rospy.init_node('depth_camera_info_generator', anonymous=False)
        
        # Subscribe to TRUE depth image
        self.image_sub = rospy.Subscriber(
            '/sim_p3at/camera/depth/depth/image_raw',
            Image,
            self.callback,
            queue_size=1
        )
        
        # Publish to MATCHING camera_info topic (depth/depth/camera_info)
        self.info_pub = rospy.Publisher(
            '/sim_p3at/camera/depth/depth/camera_info',
            CameraInfo,
            queue_size=1
        )
        
        rospy.loginfo("Depth camera_info: depth/depth/image_raw -> depth/depth/camera_info")
    
    def callback(self, img_msg):
        info = CameraInfo()
        
        # 使用图像的时间戳 - 完美同步!
        info.header = img_msg.header
        info.header.frame_id = "camera_depth_optical_frame"
        
        # 图像尺寸
        info.height = img_msg.height
        info.width = img_msg.width
        
        # D435i 典型参数
        info.distortion_model = "plumb_bob"
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 内参矩阵 K
        fx = fy = 554.254691191187
        cx = info.width / 2.0
        cy = info.height / 2.0
        info.K = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        
        # 旋转矩阵 R
        info.R = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        
        # 投影矩阵 P
        info.P = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        
        self.info_pub.publish(info)

if __name__ == '__main__':
    try:
        generator = DepthCameraInfoGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
