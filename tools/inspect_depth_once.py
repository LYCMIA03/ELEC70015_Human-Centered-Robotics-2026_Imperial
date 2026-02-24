#!/usr/bin/env python3
# Inspect one depth frame and print NaN ratio / min / max.
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node("inspect_depth_once", anonymous=True)
    topic = rospy.get_param("~image_topic", "/sim_p3at/depth_camera/depth/image_raw")

    msg = rospy.wait_for_message(topic, Image, timeout=5.0)
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")  # float32 depth

    arr = np.array(img, dtype=np.float32)
    nan_ratio = np.isnan(arr).mean()
    finite = arr[np.isfinite(arr)]

    print("encoding:", msg.encoding)
    print("shape:", arr.shape)
    print("nan_ratio:", float(nan_ratio))

    if finite.size == 0:
        print("finite_count: 0 (all NaN/Inf)")
    else:
        print("finite_count:", int(finite.size))
        print("min_depth:", float(finite.min()))
        print("max_depth:", float(finite.max()))

if __name__ == "__main__":
    main()
