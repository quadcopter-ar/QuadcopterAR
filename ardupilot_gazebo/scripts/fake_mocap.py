#!/usr/bin/env python

##
#
# Simulate our optitrack motion capture system based on gazebo. 
#
# Takes the pose of our model from /gazebo/model_states and 
# republishes it to /Robot_1/pose, which is what our mocap system provides
#
##

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import Mavlink

# Global variables for keeping track of time
secs = 0
nsecs = 0

class ObjectTracker():
    """
    Keeps track of the gazebo object with the given name, 
    and can return its position/orientation on demand. 
    """
    def __init__(self):
        self.all_data = ModelStates()
        
        rospy.init_node('object_pose_grabber')
        pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=10)  # make sure /gazebo/model_states is published

    def callback(self, data):
        """Handle new messages describing all object locations"""
        self.all_data = data

    def get_pose(self, object_name):
        """Return the position and orientation of a given object"""
        
        idx = None
        # find the object out of the list
        for i in range(len(self.all_data.name)):
            if (self.all_data.name[i] == object_name):
                idx = i

        if idx is not None:
            return self.all_data.pose[idx]
        else:
            print("Error: object %s does not seem to exist.\n Available objects: %s" % (object_name, self.all_data.name))

def time_callback(data):
    """
    Update time information so we can publish PoseStamped
    to the optitrack topic
    """
    global secs
    global nsecs

    secs = data.header.stamp.secs
    nsecs = data.header.stamp.nsecs


def fake_pose():
    pose = Pose()

    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0

    return pose



if __name__=="__main__":

    #drone = "iris_demo"
    #t = ObjectTracker()
    
    
    pose_pub = rospy.Publisher('/Robot_1/pose', PoseStamped, queue_size=10)
    time_sub = rospy.Subscriber('/mavlink/from', Mavlink, time_callback)

    rate = rospy.Rate(30)  # Hz

    seq = 0
    while not rospy.is_shutdown():
        #pose = t.get_pose(drone)
        pose = fake_pose()
        pose_stamped = PoseStamped()

        pose_stamped.header.seq = seq
        pose_stamped.header.stamp.secs = secs
        pose_stamped.header.stamp.nsecs = nsecs
        pose_stamped.header.frame_id = "map" 
        pose_stamped.pose = pose

        pose_pub.publish(pose_stamped)

        seq += 1
        rate.sleep()