#!/usr/bin/env python

##
#
# Control a MAV via mavros
#
##

import rospy
import threading
from geometry_msgs.msg import Pose, PoseStamped, Twist
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL

class MavController:
    """
    A simple object to help interface with mavros 
    """
    def __init__(self, namespace=""):

        #rospy.init_node("mav_control_node_")
        rospy.Subscriber(namespace + "/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber(namespace + "/mavros/rc/in", RCIn, self.rc_callback)

        self.cmd_pos_pub = rospy.Publisher(namespace + "/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher(namespace + "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher(namespace + "/mavros/rc/override", OverrideRCIn, queue_size=1)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy(namespace + '/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy(namespace + '/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy(namespace + '/mavros/cmd/takeoff', CommandTOL)

        self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()

    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose

    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work. 
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz(self, x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        self.goto(pose)

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default. 
        """
        cmd_vel = Twist()
        
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)
    
    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        # Set to stabilize mode for arming
        mode_resp = self.mode_service(custom_mode="0")
        self.arm()

        # Set to guided mode 
        mode_resp = self.mode_service(custom_mode="4")

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        if takeoff_resp.success is not True:
            print(takeoff_resp)

        return takeoff_resp

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly, 
        land, and disarm. 
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()

def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    c = MavController()
    rospy.sleep(1)

    print("Takeoff")
    c.takeoff()
    rospy.sleep(10)

    print("Waypoint 1: position control")
    c.goto_xyz(1,1,1)
    rospy.sleep(5)
    print("Waypoint 2: position control")
    c.goto_xyz(0,0,1)
    rospy.sleep(5)

    print("Velocity Setpoint 1")
    c.set_vel(0,1,0)
    rospy.sleep(5)
    print("Velocity Setpoint 2")
    c.set_vel(0,-1,0)
    rospy.sleep(5)
    print("Velocity Setpoint 3")
    c.set_vel(0,0,0)
    rospy.sleep(5)
   
    print("Landing")
    c.land()

def multiagent_demo():
    """
    A simple demonstration of using mavros commands to control two UAVs
    """

    # Specify different controller objects for each UAV
    c1 = MavController(namespace="/uav1")
    c2 = MavController(namespace="/uav2")
    rospy.sleep(1)

    print("Takeoff")
    c1.takeoff(2)
    c2.takeoff(2)
    rospy.sleep(10)

    print("Waypoint 1: position control")
    c1.goto_xyz(1,1,2)
    rospy.sleep(5)
    print("Waypoint 2: position control")
    c2.goto_xyz(-1,-1,2)
    rospy.sleep(5)

    print("Landing")
    c1.land()
    c2.land()

if __name__=="__main__":
    # need to initialize a ros node before creating any MavController instances
    rospy.init_node("mav_control_node")

    simple_demo()
    #multiagent_demo()
