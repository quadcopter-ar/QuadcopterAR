#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from mavros_msgs.msg import Mavlink
from mavros.mavlink import convert_to_bytes
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
import sys

# Global variables to keep track of time from the FCU
secs = None
nsecs = None

class fifo(object):
    """ A simple buffer """
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

def time_callback(data):
    """
    Synchronize time of messages we send with the FCU. Extracts the FCU 
    time from a TIMESYNC mavlink message, and uses this to set global variables
    """
    global secs
    global nsecs

    secs = data.header.stamp.secs
    nsecs = data.header.stamp.nsecs

def pose_callback(data):
    """
    Republish the given Pose information on a new topic,
    with timestamps that match the FCU
    """
    delay_ms = 0  # approximate measurement delay in ms 
    delay_ns = delay_ms*1e6

    if (secs is not None) and (nsecs is not None):
        data.header.stamp.secs = secs
        data.header.stamp.nsecs = nsecs + delay_ns  # include some measurement delay

        pose_pub.publish(data)
    



if __name__=="__main__":
    pose_x = 0.0
    pose_y = 0.0
    pose_z = 0.0

    out_topic = "/vrpn/pose"

    rospy.init_node("vrpn_publisher")
    # mavlink_pub = rospy.Publisher("/mavlink/to", Mavlink, queue_size=20)
    time_sub = rospy.Subscriber("/mavlink/from", Mavlink, time_callback)
    pose_pub = rospy.Publisher(out_topic, PoseStamped, queue_size=10)
    f = fifo()
    mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)

    # while mavlink_pub.get_num_connections() <= 0:
    #     # wait for everything to initialize   
    #     pass
    print("[ VRPN] Waiting for MAVLink connection.")

    while secs is None:
        pass

    print("[ VRPN] MAVLink connection established.")

    print("[ VRPN] ")
    
    rate = rospy.Rate(30)
    seq = 0
    
    while secs is not None:
        pose_stamped = PoseStamped()

        pose_stamped.header.seq = 0
        pose_stamped.header.stamp.secs = secs
        pose_stamped.header.stamp.nsecs = nsecs
        pose_stamped.header.frame_id = "map" 
    
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        pose_stamped.pose.position.x = pose_x
        pose_stamped.pose.position.y = pose_y
        pose_stamped.pose.position.z = pose_z
        pose_pub.publish(pose_stamped)

        #print("Pose published.")

        seq += 1
        pose_x += 0.05
        rate.sleep()
    
