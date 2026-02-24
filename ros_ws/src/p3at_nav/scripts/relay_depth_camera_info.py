#!/usr/bin/env python3
"""
Relay color camera_info to depth camera_info for RealSense simulation.
Preserves timestamp for synchronization with depth images.
"""
import rospy
from sensor_msgs.msg import CameraInfo

class CameraInfoRelay:
    def __init__(self):
        rospy.init_node('camera_info_relay', anonymous=False)
        
        # Subscribe to color camera_info
        self.color_sub = rospy.Subscriber(
            '/sim_p3at/camera/color/camera_info',
            CameraInfo,
            self.callback,
            queue_size=10  # 增加队列防止丢帧
        )
        
        # Publish to depth camera_info
        self.depth_pub = rospy.Publisher(
            '/sim_p3at/camera/depth/camera_info',
            CameraInfo,
            queue_size=10
        )
        
        rospy.loginfo("Camera info relay: color -> depth (preserving timestamps)")
    
    def callback(self, msg):
        # 创建新消息,保留时间戳但修改 frame_id
        depth_info = CameraInfo()
        depth_info.header = msg.header  # 保留完整的 header (包括时间戳)
        depth_info.header.frame_id = "camera_depth_optical_frame"  # 只改 frame_id
        
        # 复制所有相机参数
        depth_info.height = msg.height
        depth_info.width = msg.width
        depth_info.distortion_model = msg.distortion_model
        depth_info.D = msg.D
        depth_info.K = msg.K
        depth_info.R = msg.R
        depth_info.P = msg.P
        depth_info.binning_x = msg.binning_x
        depth_info.binning_y = msg.binning_y
        depth_info.roi = msg.roi
        
        self.depth_pub.publish(depth_info)

if __name__ == '__main__':
    try:
        relay = CameraInfoRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
