#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
PKG = 'marker_based_localisation'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import math

# ROS messages.
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Imu
from marker_based_localisation.msg import Eulers

class QuatToEuler():
    def __init__(self):
        self.got_new_msg = False
        self.euler_msg = Eulers()

        # Create subscribers and publishers.
        posesub = rospy.Subscriber("pose", PoseStamped, self.pose_callback)
        posecovsub = rospy.Subscriber("posecov", PoseWithCovarianceStamped, self.posecov_callback)
        odomsub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        sub_imu   = rospy.Subscriber("imu", Imu, self.imu_callback)
 
        pub_euler = rospy.Publisher("euler", Eulers, queue_size=1)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                pub_euler.publish(self.euler_msg)
                self.got_new_msg = False

    # IMU callback function.
    def imu_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.fill_euler_msg(msg, r, p, y)
    
    # Odometry callback function.
    def odom_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.fill_euler_msg(msg, r, p, y)

    # Pose callback function.
    def pose_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.fill_euler_msg(msg, r, p, y)

    # posecov callback function.
    def posecov_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.fill_euler_msg(msg, r, p, y)

    # Fill in Euler angle message.
    def fill_euler_msg(self, msg, r, p, y):
        self.got_new_msg = True
        self.euler_msg.header.stamp = msg.header.stamp
        self.euler_msg.roll  = math.degrees(r)
        self.euler_msg.pitch = math.degrees(p)
        self.euler_msg.yaw   = math.degrees(y)

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('quat_to_euler')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException: pass