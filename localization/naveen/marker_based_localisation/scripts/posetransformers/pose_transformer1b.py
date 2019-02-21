#!/usr/bin/env python  
import rospy

import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

def handle_camera_pose(msg):
    
    br2 = tf.TransformBroadcaster()
    

    
    br2.sendTransform((-0.02, 0.1, 0.1),
                     (0, 0, 0, 1),
                     rospy.Time.now(),
                     "base_link",
                     "camera")

if __name__ == '__main__':
    rospy.init_node('pose_transformer1b')
    print "Started node"
    
    rospy.Subscriber('/mavros/vision_pose/pose_cov',
                     PoseWithCovarianceStamped,
                     handle_camera_pose)
    print "Subscribed to Pose"
    rospy.spin()