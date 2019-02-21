#!/usr/bin/env python

##
#
# Subscribte to pose messages from one topic and relay them to another topic,
# changing the timestamps to match those coming from the FCU. 
#
# Also, send a SET_GPS_GLOBAL_ORIGIN message at the beginning (before republishing anything)
# so that we can have a notion of global location without having a real GPS (ie indoors)
#
##

import rospy
from geometry_msgs.msg import PoseStamped
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

    b = convert_to_bytes(data)
    m = mav.decode(b)

    if (m.get_msgId() == 111):  # timesync message
        s = m.tc1 / 1.0e9
        t = rospy.Time.from_sec(s)
        secs = t.secs
        nsecs = t.nsecs

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
    try:
        if len(sys.argv) < 3:
            print("Usage: pose_republisher.py [input_pose_topic] [output_pose_topic]")
            sys.exit(1)

        in_topic = sys.argv[1]
        out_topic = sys.argv[2]

        rospy.init_node("pose_republisher")
        mavlink_pub = rospy.Publisher("/mavlink/to", Mavlink, queue_size=20)
        pose_pub = rospy.Publisher(out_topic, PoseStamped, queue_size=10)
        time_sub = rospy.Subscriber("/mavlink/from", Mavlink, time_callback)

        # Setup mavlink instance
        f = fifo()
        mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)

        while mavlink_pub.get_num_connections() <= 0:
            # wait for everything to initialize
            pass
       
        # Then start republishing from in_topic to out_topic
        pose_sub = rospy.Subscriber(in_topic, PoseStamped, pose_callback)  # automatically republishes to out_topic

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
