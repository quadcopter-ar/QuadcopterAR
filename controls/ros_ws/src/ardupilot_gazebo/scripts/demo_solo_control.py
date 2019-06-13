#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2014 Vladimir Ermakov.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md

from __future__ import print_function

import sys
import argparse
import rospy
import mavros
import time
import math

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import tf
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, PointStamped
from mavros_msgs.msg import OverrideRCIn
from mavros import setpoint as SP
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL

sp_timer = None
sp_pose = None
mode_service = None
arm_service = None
takeoff_service = None

def sp_timer_callback(event):
    global sp_pose

    pos_pub = SP.get_pub_position_local(queue_size=10)

    sp_pose.header.stamp = rospy.get_rostime()
    
    q = quaternion_from_euler(0, 0, 0)
    sp_pose.pose.orientation = Quaternion(*q)

    pos_pub.publish(sp_pose)

def takeoff():
    
    # Mode Guided
    print(mode_service(custom_mode="4"))
    
    # Arm
    print(arm_service(True))

    # Takeoff 10
    print(takeoff_service(altitude=1.1))

def main():
    rospy.init_node("demo_solo_control")
    global sp_timer, sp_pose
    global mode_service, arm_service, takeoff_service

    mavros.set_namespace(mavros.DEFAULT_NAMESPACE)

    print('[ INFO] Please check GPS lock.')

    print('[ INFO] UAV1 is taking off.')

    mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    takeoff()

    time.sleep(5)
    
    print('[ INFO] UAV1 goes to (0, 0, 1.1).')



    if sp_timer is None:
        sp_pose = PoseStamped(header=Header(stamp=rospy.get_rostime(),frame_id="map"))
        sp_pose.pose.position = Point(0, 0, 1.1)
        sp_timer = rospy.Timer(rospy.Duration(.5), sp_timer_callback)

    time.sleep(5)

    sp_pose.pose.position = Point(.5, 0, 1.1)

    time.sleep(5)

    sp_pose.pose.position = Point(0, .5, 1.1)

    time.sleep(5)

    sp_pose.pose.position = Point(-.5, 0, 1.1)

    time.sleep(5)

    sp_pose.pose.position = Point(0, -.5, 1.1)

    time.sleep(5)

    sp_pose.pose.position = Point(0, 0, 1.1)

    time.sleep(8)

    print('[ INFO] UAV landing')
    mode_service(custom_mode="LAND")

    rospy.spin()

if __name__ == '__main__':
    main()
