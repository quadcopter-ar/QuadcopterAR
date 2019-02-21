#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
from nav_msgs.msg import Odometry

def handle_solo_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "solo1",
                     "world")

if __name__ == '__main__':
    rospy.init_node('solo_tf_broadcaster')
    print "Started node"
    
    rospy.Subscriber('/mavros/global_position/local',
                     Odometry,
                     handle_solo_pose)
    print "Subscribed to Pose"
    rospy.spin()