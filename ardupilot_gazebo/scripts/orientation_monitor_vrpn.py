#!/usr/bin/env python
# Remember to CHMOD

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Mavlink
from mavros.mavlink import convert_to_bytes
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sys


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


def pose_callback(pose):
    """
    Republish the given Pose information on a new topic,
    with timestamps that match the FCU
    """

    (r, p, y) = euler_from_quaternion([pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w])

    # roll = -roll

    r = r * 180 / 3.14
    p = p * 180 / 3.14
    y = y * 180 / 3.14

    print((r,p,y))

    # # # print(euler)
    # (x, y, z, w) = quaternion_from_euler(0,.1*pose_count)

if __name__=="__main__":
    #in_topic = "/mavros/local_position/pose"
    in_topic = "/vrpn_client_node/solo/pose"
    rospy.init_node("orientation_monitor")


    time_sub = rospy.Subscriber("/mavlink/from", Mavlink, time_callback)

    f = fifo()
    mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)

    while secs is None:
        pass

    pose_sub = rospy.Subscriber(in_topic, PoseStamped, pose_callback)

    rospy.spin()