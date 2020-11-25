#! /usr/bin/env python
import rospy
import math
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped

import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

def callback_transform_pose(ar_marker):

    if math.isnan(ar_marker.position.x) == False:
    # **Assuming /tf2 topic is being broadcasted

        tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        transform = tf_buffer.lookup_transform("base_link",
                                            "camera_color_optical_frame", #source frame
                                            rospy.Time(0), #get the tf at first available time
                                            rospy.Duration(0.1)) #wait for 1 second

        ar_pose = PoseStamped()
        ar_pose.pose = ar_marker

        transformed_pose = tf2_geometry_msgs.do_transform_pose(ar_pose, transform)

        print transformed_pose

        ar_transformed_pose_stamped = PoseStamped()
        ar_transformed_pose_stamped.header.frame_id = "base_link"
        ar_transformed_pose_stamped.header.stamp = rospy.Time.now()
        ar_transformed_pose_stamped.pose = transformed_pose.pose

        # this pubished pose is perfect for vislauization in rviz
        pub_ar_marker_rviz.publish(ar_transformed_pose_stamped)
        print ar_transformed_pose_stamped

        pub_ar_marker.publish(transformed_pose.pose)


if __name__=='__main__': 
# Test Case
    rospy.init_node("ar_marker_transform")

    sub_ar_marker = rospy.Subscriber("marker_info", Pose, callback_transform_pose, queue_size=1)
    pub_ar_marker_rviz = rospy.Publisher("ar_transformed_pose_stamped", PoseStamped, queue_size=1)
    pub_ar_marker = rospy.Publisher("ar_transformed_pose", Pose, queue_size=1)

    rospy.spin()
