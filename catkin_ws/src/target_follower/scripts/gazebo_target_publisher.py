#!/usr/bin/env python3

"""Publish a Gazebo model pose as PoseStamped.

Key point
Gazebo's /gazebo/get_model_state returns a pose expressed in the Gazebo
reference frame you request. The outgoing PoseStamped header.frame_id must
match that reference frame in your ROS TF tree. If you publish a "world" pose
and label it "odom", downstream consumers will chase the wrong goal.

Default behavior in this script
- Query the target pose relative to the robot base link in Gazebo
  (reference_frame="<robot_model_name>::base_link").
- Publish it as a PoseStamped with frame_id="base_link".

That makes it easy for a follower node to transform the target pose into
"map" or "odom" using normal TF.
"""

import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState


def main() -> None:
    rospy.init_node("gazebo_target_publisher")

    model_name = rospy.get_param("~model_name", "target")
    robot_model_name = rospy.get_param("~robot_model_name", "p3at")

    out_frame = rospy.get_param("~out_frame", "base_link")
    default_reference_frame = f"{robot_model_name}::base_link"
    reference_frame = rospy.get_param("~reference_frame", default_reference_frame)

    topic = rospy.get_param("~topic", "/target_pose")
    rate_hz = float(rospy.get_param("~rate_hz", 10.0))

    # Stamp messages so consumers can do timeout logic.
    stamp_now = bool(rospy.get_param("~stamp_now", True))

    pub = rospy.Publisher(topic, PoseStamped, queue_size=10)

    rospy.wait_for_service("/gazebo/get_model_state")
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    rospy.loginfo(
        "GazeboTargetPublisher ready, model=%s, reference_frame=%s, out_frame=%s",
        model_name,
        reference_frame,
        out_frame,
    )

    r = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        try:
            resp = get_state(model_name, reference_frame)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(2.0, "get_model_state service call failed: %s", str(e))
            r.sleep()
            continue

        if not resp.success:
            rospy.logwarn_throttle(
                2.0,
                "get_model_state failed: model=[%s] reference_frame=[%s] msg=[%s]",
                model_name,
                reference_frame,
                resp.status_message,
            )
            r.sleep()
            continue

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now() if stamp_now else rospy.Time(0)
        msg.header.frame_id = out_frame
        msg.pose = resp.pose
        pub.publish(msg)

        r.sleep()


if __name__ == "__main__":
    main()
