#!/usr/bin/env python  
#import roslib
#roslib.load_manifest('learning_tf')
import rospy

import tf
from geometry_msgs.msg import Pose

def handle_pose(msg):
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(30)
    

    br.sendTransform((-0.108, -0.1397, 1.0),
                     (1, 0, 0, 0),
                     rospy.Time.now(),
                     "fid100",
                     "map")
    rate.sleep()
if __name__ == '__main__':
    rospy.init_node('pose_transformer')
    print "Started node"
    
    rospy.Subscriber('/fiducial_pose',
                     PoseWithCovarianceStamped,
                     handle_pose)
    print "Subscribed to Pose"
    rospy.spin()