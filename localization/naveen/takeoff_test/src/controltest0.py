#!/usr/bin/env python  

import roslib
import rospy
import tf
import std_msgs
import time
import mavros
import mavros_msgs.srv

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

from mavros import setpoint as SP
from mavros import command
from threading import Thread



print "Starting Test..."

IS_APM = False

class Solo(object):

    def __init__(self,solo_id):
        self.solo_id = solo_id
        mavros_string = "/mavros"
        mavros.set_namespace(mavros_string)
        self.mavros_string = mavros_string
        self.pub_vel = SP.get_pub_velocity_cmd_vel(queue_size=10)
        self.current_joy_throttle = 0.1

    def start_subs(self):
        pass

    def setmode(self,base_mode=0,custom_mode="OFFBOARD",delay=0.1):
        set_mode = rospy.ServiceProxy(self.mavros_string+'/set_mode', mavros_msgs.srv.SetMode)  
        if IS_APM:
            
            if custom_mode == "OFFBOARD":
                custom_mode = "GUIDED"
            if custom_mode == "AUTO.LAND":
                custom_mode = "LAND"
            if custom_mode == "MANUAL":
                custom_mode = "STABILIZE"
            if custom_mode == "POSCTL":
                custom_mode = "LOITER"
        ret = set_mode(base_mode=base_mode, custom_mode=custom_mode)
        print "Changing modes: ", ret
        time.sleep(delay)

    
    def arm(self):
        arm = rospy.ServiceProxy(self.mavros_string+'/cmd/arming', mavros_msgs.srv.CommandBool)  
        print "Arm: ", arm(True)
        
    def disarm(self):
        arm = rospy.ServiceProxy(self.mavros_string+'/cmd/arming', mavros_msgs.srv.CommandBool)  
        print "Disarm: ", arm(False)

    def takeoff(self):
        arm = rospy.ServiceProxy(self.mavros_string+'/cmd/arming', mavros_msgs.srv.CommandBool)  
        print "Disarm: ", arm(False)

    def subscribe_joy_thread(self):
        s = Thread(target=self.subscribe_joy, args=())
        s.daemon = True
        s.start()

    def subscribe_joy(self):
        rospy.Subscriber("joy",Joy,self.handle_joy)
         
        rospy.spin()
    
    def handle_joy(self,joy_data):
        magnitude = 2.0 # in meters/sec
        self.current_joy_throttle = 0.0 + magnitude*joy_data.axes[1]

    def joy_control_thread(self):
        t = Thread(target = self.joy_control, args = ())
        t.daemon = True
        t.start()
    
    

    def joy_control(self):
        rate = rospy.Rate(30)   # 30hz
        
        msg = SP.TwistStamped(
            header=SP.Header(
                frame_id="Joystick",  # doesn't matter
                stamp=rospy.Time.now()),    # stamp should update
            )
        i=0
        
        while not rospy.is_shutdown():
        
                       
            msg.twist.linear.z = self.current_joy_throttle
            self.pub_vel.publish(msg)
            #rate.sleep()
            print i
            i+=1
        
             
        
        



if __name__ == '__main__':
    
    rospy.init_node("Joystick2Solo")
    
    s1 = Solo(1) #solo object
    s1.start_subs()


    s1.subscribe_joy_thread()
    time.sleep(0.1)
    print "Joystick data received"
    s1.joy_control_thread()
    time.sleep(0.1)
    print "Joystick control running"

    s1.setmode(custom_mode="OFFBOARD")
    print "Mode Set"
    time.sleep(1)
    s1.arm()
    #s1.setmode(custom_mode="OFFBOARD")
    print "Armed"
    print "ready to control"
    


    
    
    

    

