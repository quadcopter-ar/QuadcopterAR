#!/usr/bin/env python

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
from mavros_msgs.srv import StreamRate, StreamRateRequest

sp_timer = None
sp_pose = None
mode_service = None
arm_service = None
takeoff_service = None
uav1_pose = None
uav2_pose = None
curr_joy = None
joy_timer = None

axes_map = {
    'roll': 1,
    'pitch': 0,
    'yaw': 3,
    'throttle': 4
}

axes_scale = {
    'roll': 1.5,
    'pitch': 1.5,
    'yaw': 0.1,
    'throttle': 0.2
}

button_map = {
    'arm' : 0,
    'disarm' : 1,
    'takeoff': 2,
    'land': 3,
    'enable': 4
}

def sp_timer_callback(event):
    global sp_pose
    # print(sp_pose.pose)
    mavros.set_namespace("uav1/mavros") # uav2/mavros/setpoint_position/local
    pos_pub = SP.get_pub_position_local(queue_size=10)

    sp_pose.header.stamp = rospy.get_rostime()
    sp_pose.pose.position = Point(10, 0, 10)
    eu = euler_from_quaternion((0, 0, 0, 1))
    q = quaternion_from_euler(0, 0, -eu[2])
    sp_pose.pose.orientation = Quaternion(*q)

    pos_pub.publish(sp_pose)

    mavros.set_namespace("uav2/mavros") # uav2/mavros/setpoint_position/local
    pos_pub = SP.get_pub_position_local(queue_size=10)

    sp_pose.header.stamp = rospy.get_rostime()
    sp_pose.pose.position = Point(-10, 0, 10)
    eu = euler_from_quaternion((0, 0, 0, 1))
    q = quaternion_from_euler(0, 0, -eu[2])
    sp_pose.pose.orientation = Quaternion(*q)

    pos_pub.publish(sp_pose)

def takeoff():
    
    # Mode Guided
    print(mode_service(custom_mode="4"))
    
    # Arm
    print(arm_service(True))

    # Takeoff 10
    print(takeoff_service(altitude=10))

def pose1_callback(pose):
    global uav1_pose
    if uav1_pose is None:
        print('[ INFO] uav1/mavros/local_position/pose is subscribed.')
    uav1_pose = pose

def pose2_callback(pose):
    global uav2_pose
    if uav2_pose is None:
        print('[ INFO] uav1/mavros/local_position/pose is subscribed.')
    uav2_pose = pose

def get_axis(j, n):
    return j.axes[axes_map[n]] * axes_scale[n]

def get_buttons(j, n):
    return j.buttons[ button_map[n]]

def joy_timer_callback(event):
    global uav1_pose, uav2_pose, curr_joy, sp_pose

    roll = get_axis(curr_joy, 'roll')
    pitch = -get_axis(curr_joy, 'pitch')
    yaw = get_axis(curr_joy, 'yaw')
    throttle = get_axis(curr_joy, 'throttle')

    

    mavros.set_namespace("uav1/mavros")

    set_rate = rospy.ServiceProxy(mavros.get_topic('set_stream_rate'), StreamRate)
    rate_arg = 20.0
    try:
        set_rate(stream_id=0, message_rate=rate_arg, on_off=(rate_arg != 0))
    except rospy.ServiceException as ex:
        fault(ex)

    pos_pub = SP.get_pub_position_local(queue_size=10)

    if sp_pose is None:
        print('[ INFO] Going to default location.')
        print('[ INFO] You can use the joystick to control the drone now.')
        sp_pose = PoseStamped(header=Header(stamp=rospy.get_rostime(),frame_id="map"))
        sp_pose.header.stamp = rospy.get_rostime()
        sp_pose.pose.position = Point(10, 0, 10)
        eu = euler_from_quaternion((0, 0, 0, 1))
        q = quaternion_from_euler(0, 0, -eu[2])
        sp_pose.pose.orientation = Quaternion(*q)


    # When idle stop changing setpoint to prevent idle shift
    idle_threshold = [.7, .7, .05, 0]

    sp_pose.header.stamp = rospy.get_rostime()

    eu = euler_from_quaternion((sp_pose.pose.orientation.x, sp_pose.pose.orientation.y, sp_pose.pose.orientation.z, sp_pose.pose.orientation.w))
    
    theta = eu[2]
    x = roll
    y = pitch

    if abs(roll) > idle_threshold[0]:
        sp_pose.pose.position.x = uav1_pose.pose.position.x + x * math.cos(theta) - y * math.sin(theta)
    if abs(pitch) > idle_threshold[1]:
        sp_pose.pose.position.y = uav1_pose.pose.position.y + x * math.sin(theta) + y * math.cos(theta)
        # sp_pose.pose.position.z = uav1_pose.pose.position.z + throttle
    if abs(yaw) > idle_threshold[2]:
        q = quaternion_from_euler(eu[0], eu[1], eu[2] + yaw)
        sp_pose.pose.orientation = Quaternion(*q)
    # eu = euler_from_quaternion((0, 0, 0, 1))
    # q = quaternion_from_euler(0, 0, -eu[2])
    # sp_pose.pose.orientation = Quaternion(uav1_pose.pose.orientation.x, uav1_pose.pose.orientation.y, uav1_pose.pose.orientation.z, uav1_pose.pose.orientation.w)

    pos_pub.publish(sp_pose)

def joy_callback(joy):
    global curr_joy, joy_timer

    curr_joy = joy
    if joy_timer is None and uav1_pose is not None and uav2_pose is not None:
        print('[ INFO] Joystick connected.')
        # 0.5 is the lowerest delay to avoid unexpected behavior of the drone
        joy_timer = rospy.Timer(rospy.Duration(0.5), joy_timer_callback)

def main():
    rospy.init_node("local_controller")

    pose1_sub = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, pose1_callback)
    pose2_sub = rospy.Subscriber("/uav2/mavros/local_position/pose", PoseStamped, pose2_callback)

    joy_sub = rospy.Subscriber("joy", Joy, joy_callback)



    rospy.spin()

if __name__ == '__main__':
    main()