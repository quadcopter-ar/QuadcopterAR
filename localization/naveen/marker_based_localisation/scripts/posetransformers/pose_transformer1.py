#!/usr/bin/env python  
import rospy

import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

def handle_camera_pose(msg):
    br1 = tf.TransformBroadcaster()
    br2 = tf.TransformBroadcaster()
    rate = rospy.Rate(30)

    trans = msg.pose.pose.position
    rot = msg.pose.pose.orientation
    br1.sendTransform((trans.x, trans.y, trans.z),
                     (rot.x, rot.y, rot.z, rot.w),
                     rospy.Time.now(),
                     "base_link",
                     "odom")
    br2.sendTransform((0.02, 0.1, 0.1),
                     (1, 0, 0, 0),
                     rospy.Time.now(),
                     "camera",
                     "base_link")

if __name__ == '__main__':
    rospy.init_node('pose_transformer')
    print "Started node"
    
    rospy.Subscriber('/mavros/vision_pose/pose_cov',
                     PoseWithCovarianceStamped,
                     handle_camera_pose)
    print "Subscribed to Pose"
    rospy.spin()