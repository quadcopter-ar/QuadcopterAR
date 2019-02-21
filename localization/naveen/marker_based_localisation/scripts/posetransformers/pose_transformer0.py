#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
from fiducial_msgs.msg import FiducialTransformArray

def handle_camera_pose(msg):
    br=[]
    for i in range(0,11):
        br[i] = tf.TransformBroadcaster()
    

    trans = msg.transforms[0].transform.translation
    rot = msg.transforms[0].transform.rotation
    br1.sendTransform((trans.x, trans.y, trans.z),
                     (rot.x, rot.y, rot.z, rot.w),
                     rospy.Time.now(),
                     "solo1",
                     "world")

if __name__ == '__main__':
    rospy.init_node('pose_transformer')
    print "Started node"
    
    rospy.Subscriber('/fiducial_transforms',
                     FiducialTransformArray,
                     handle_camera_pose)
    print "Subscribed to Pose"
    rospy.spin()