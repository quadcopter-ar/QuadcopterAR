#!/usr/bin/env python  
import roslib
import rospy
import tf
import std_msgs
from math import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import geometry_msgs.msg
import mavros
import mavros_msgs.srv
from mavros import setpoint as SP
from mavros import command
from threading import Thread
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import time
import threading
import thread
from pid_controller.pid import PID
from random import randint

print "broadcasting"

import utm

IS_APM = True

class rcOverride:
    def __init__(self, copter_id = "1", mavros_string="/mavros/copter1"):
        rospy.init_node('rc_override'+copter_id)
        mavros.set_namespace(mavros_string)  # initialize mavros module with default namespace

        self.rc = OverrideRCIn()
        self.override_pub = rospy.Publisher(mavros_string+"/rc/override", OverrideRCIn, queue_size=10)

    def get_RC(self):
        return self.rc
        
    def clean_RC(self):
        RC = []
        for x in len(range(self.rc)):
            RC = 1500
        self.override_pub.publish(RC)
        
    def ruin_RC(self):
        RC = []
        for x in self.rc:
            RC.append(randint(1000,2000))
        self.override_pub.publish(RC)
        
    def publish_RC(self, RC):
        self.override_pub.publish(RC)
        

class posVel:
    def __init__(self, copter_id = "1"):
        self.copter_id = copter_id
        mavros_string = "/mavros"
        #rospy.init_node('velocity_goto_'+copter_id)
        mavros.set_namespace(mavros_string)  # initialize mavros module with default namespace

        self.pid_alt = PID(p=0.1, i=0.004, d=3.0)

        self.mavros_string = mavros_string

        self.final_alt = 0.0
        self.final_pos_x = 0.0
        self.final_pos_y = 0.0        
        self.final_vel = 0.0
        
        self.cur_rad = 0.0
        self.cur_alt = 0.0
        self.cur_pos_x = 0.0
        self.cur_pos_y = 0.0
        self.cur_vel = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.pose_open = []

        self.alt_control = True
        self.override_nav = False
        self.reached = True
        self.done = False

        self.last_sign_dist = 0.0

        # for local button handling
        self.click = " "
        self.button_sub = rospy.Subscriber("abpause_buttons", std_msgs.msg.String, self.handle_buttons)

        # publisher for mavros/copter*/setpoint_position/local
        self.pub_vel = SP.get_pub_velocity_cmd_vel(queue_size=10)
        # subscriber for mavros/copter*/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'local'), SP.PoseStamped, self.temp)

    def handle_buttons(self, msg):
        self.click = str(msg)[6:]

    def temp(self, topic):
        pass

    def start_subs(self):
        pass

    def update(self, com_x, com_y, com_z):
        self.alt_control = True
        self.reached = False
        self.override_nav = False
        self.final_pos_x = com_x
        self.final_pos_y = com_y
        self.final_alt = com_z

        self.pid_alt.target = self.final_alt

    def set_velocity(self, vel_x, vel_y, vel_z):
        self.override_nav = True
        self.vx = vel_x
        self.vy = vel_y
        self.vz = vel_z

    def subscribe_pose(self):
        rospy.Subscriber(self.mavros_string+'/global_position/local',
                         Odometry,
                         self.handle_pose)
         
        rospy.spin()

    def subscribe_pose_thread(self):
        s = Thread(target=self.subscribe_pose, args=())
        s.daemon = True
        s.start()

    def arm(self):
        arm = rospy.ServiceProxy(self.mavros_string+'/cmd/arming', mavros_msgs.srv.CommandBool)  
        print "Arm: ", arm(True)
        
    def disarm(self):
        arm = rospy.ServiceProxy(self.mavros_string+'/cmd/arming', mavros_msgs.srv.CommandBool)  
        print "Disarm: ", arm(False)

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

    def takeoff_velocity(self, alt=7):
        self.alt_control = False
        while self.cur_alt < alt - 1:
            print "CUR ALT: ", self.cur_alt, "GOAL: ", alt
            #self.set_velocity(0, 0, 1.5)
            self.update(self.cur_pos_x, self.cur_pos_y, alt)
 
        time.sleep(0.1)
        #self.set_velocity(0, 0, 0)

        self.final_alt = alt
        
        rospy.loginfo("Reached target Alt!")

    def handle_pose(self, msg):
        pos = msg.pose.pose.position
        qq = msg.pose.pose.orientation

        self.pose_open = qq

        q = (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)

        euler = euler_from_quaternion(q)

        self.cur_rad = euler[2]

        self.cur_pos_x = pos.x 
        self.cur_pos_y = pos.y
        self.cur_alt = pos.z

        
    def navigate(self):
        rate = rospy.Rate(30)   # 30hz
        magnitude = 1.0  # in meters/sec

        msg = SP.TwistStamped(
            header=SP.Header(
                frame_id="base_footprint",  # doesn't matter
                stamp=rospy.Time.now()),    # stamp should update
        )
        i =0

        self.home_lat = self.cur_pos_x
        self.home_lon = self.cur_pos_y
        self.home_alt = self.cur_alt
        
        while not rospy.is_shutdown():
            if self.click == "ABORT":
                self.disarm()
                break

            if not self.override_nav:  # heavy stuff right about here
                vector_base = self.final_pos_x - self.cur_pos_x
                vector_height = self.final_pos_y - self.cur_pos_y
                try:
                    slope = vector_base/(vector_height+0.000001)
                    p_slope = -vector_height/(vector_base+0.000001)
                except:
                    print "This should never happen..."

                copter_rad = self.cur_rad
                vector_rad = atan(slope)
                if self.final_pos_y < self.cur_pos_y:
                    vector_rad = vector_rad - pi

                glob_vx = sin(vector_rad)
                glob_vy = cos(vector_rad)

                beta = ((vector_rad-copter_rad) * (180.0/pi) + 360.0*100.0) % (360.0)
                beta = (beta + 90.0) / (180.0/pi)

                #print beta

                if True: # not self.reached:   #this is issues
                    cx = self.cur_pos_x
                    cy = self.cur_pos_y
                    fx = self.final_pos_x
                    fy = self.final_pos_y

                    b_c = cy - cx * p_slope 
                    b_f = fy - fx * p_slope
                    sign_dist = b_f - b_c 

                    if self.last_sign_dist < 0.0 and sign_dist > 0.0:
                        self.reached = True
                    if self.last_sign_dist > 0.0 and sign_dist < 0.0:
                        self.reached = True

                    #print "THE", self.last_sign_dist, sign_dist, self.reached

                    self.last_sign_dist = sign_dist 

                if self.reached:
                    self.last_sign_dist = 0.0

                if True: #else:    #this switch is gonna cause isses
                    master_scalar = 1.0

                    master_hype = sqrt((cx - fx)**2.0 + (cy - fy)**2.0)

                    if master_hype > 1.0:
                        master_scalar = 1.0
                    else:
                        master_scalar = master_hype

                    self.vx = sin(beta) * master_scalar
                    self.vy = cos(beta) * master_scalar

                    #print "THE VIX: ", self.vx, " THE VIY: ", self.vy

            if True:
                if self.alt_control:
                    #pid_offset = self.pid_alt.update(self.cur_alt)
                    #if pid_offset > 1.0:
                    #    pid_offset = 1.0
                    #if pid_offset < -1.0:
                    #    pid_offset = -1.0
                    if self.vy > 0.5:
                        self.vy = 0.5
                    if self.vy < -0.5:
                        self.vy = -0.5
                    if self.vx > 0.5:
                        self.vx = 0.5
                    if self.vx < -0.5:
                        self.vx = -0.5

                    #ned.
                    if self.final_alt > self.cur_alt:
                        self.vz = 0.5
                    if self.final_alt < self.cur_alt:
                        self.vz = -0.5
                    if abs(self.final_alt-self.cur_alt) < 0.9:
                        self.vz = 0.0

                    msg.twist.linear = geometry_msgs.msg.Vector3(self.vx*magnitude, self.vy*magnitude, self.vz*magnitude)
                else:
                    msg.twist.linear = geometry_msgs.msg.Vector3(self.vx*magnitude, self.vy*magnitude, self.vz*magnitude)

            if True:
                #print self.vx, self.vy

                self.pub_vel.publish(msg)
            
            rate.sleep()
            i +=1


    def land_velocity(self):
        while self.cur_alt > 7.0:
            self.update(self.home_lat, self.home_lon, 7.0)
            print "landing: ", self.cur_alt
        self.update(self.home_lat, self.home_lon, 5.0)
        alts = 5.0
        while self.cur_alt < 0.45:
            alts = alts - 0.1
            self.update(self.home_lat, self.home_lon, alts)
            print "slowly landing: ", self.cur_alt
            time.sleep(0.5)
        print "Landed, disarming"
        self.update(self.home_lat, self.home_lon, 0)
            
    def get_copter_id(self):
        return self.copter_id
            
    def get_lat_lon_alt(self):
        return (self.cur_pos_x, self.cur_pos_y, self.cur_alt)

    def get_home_lat_lon_alt(self):
        return (self.home_lat, self.home_lon, self.home_alt)

    def start_navigating(self):
        t = Thread(target = self.navigate, args = ())
        t.daemon = True
        t.start()

class SmartRTL:
    def __init__(self, copters):
        self.initial_alt_drop = 5
        self.copters = copters
        self.sorted_copters = []
        copters_by_alt = {}
        for cop in copters:
            copters_by_alt[cop] = cop.get_lat_lon_alt()[-1]

        self.sorted_copters = sorted(copters_by_alt)
                
        print "SORTED COPTERS", [c.get_copter_id() for c in self.sorted_copters]

        for w in self.sorted_copters[::-1][:-1]:
            self.raise_cops(w)
            
        for x in self.sorted_copters:
            self.land_cop(x)

    def raise_cops(self,cop):
        cur_pos_x, cur_pos_y, cur_alt = cop.get_lat_lon_alt()
        self.raise_height = cur_alt + 5
        cop.update(cur_pos_x, cur_pos_y, self.raise_height)
        while cur_alt < self.raise_height-1:
            cur_pos_x, cur_pos_y, cur_alt = cop.get_lat_lon_alt()
            print "Copter", cop.copter_id, " altitude: ",cur_alt
        #cop.set_velocity(0.0,0.0,0.0)
    
    def land_cop(self,cop):
        cur_pos_x, cur_pos_y, cur_alt = cop.get_lat_lon_alt()
        home_lat, home_lon, home_alt = cop.get_home_lat_lon_alt()
        
        print "RTLing Copter", cop.copter_id
        self.drop_height = cur_alt-self.initial_alt_drop
        
        print "Copter", cop.copter_id, " dropping..."
        time.sleep(1)
        
        cop.update(cur_pos_x, cur_pos_y, self.drop_height)
        while cur_alt > self.drop_height+2.0:
            cur_pos_x, cur_pos_y, cur_alt = cop.get_lat_lon_alt()
            print "cur_alt: ", cur_alt, "check", self.drop_height+1.0
            #print "Copter", cop.copter_id, " altitude: ",cur_alt
        #cop.update(0.0,0.0,0.0)
        
        print "Copter", cop.copter_id, "going to home location..."
        time.sleep(1)
        
        cop.update(home_lat, home_lon, self.drop_height)
        while not cop.reached:
            cur_pos_x, cur_pos_y, cur_alt = cop.get_lat_lon_alt()
            time.sleep(0.025)
            #cop.update(home_lat, home_lon, self.drop_height)
            print "Copter", cop.copter_id, " drop height", self.drop_height," altitude: ", cur_alt
        #cop.update(0.0,0.0,0.0)
        cop.setmode(custom_mode="AUTO.LAND")
        #cop.land_velocity()

class SafeTakeoff:
    def __init__(self, copters, offsets_x, offsets_y, alt = 20.0):
        self.cops = copters

        self.offs_x = offsets_x
        self.offs_y = offsets_y
        
        self.ids = []
        for i in range(len(self.cops)):
            self.ids.append(i)

        self.offs_hype = []
        for o in range(len(self.cops)):
            h = sqrt(self.offs_y[o] **2.0 + self.offs_x[o] **2.0)
            self.offs_hype.append(h)

        self.alt = alt

        #running_x = 0.0
        #running_y = 0.0
        #for cop in copters:
        #    running_x = running_x + cop.cur_pos_x
        #    running_y = running_y + cop.cur_pos_y

        #self.center_x = running_x / float(len(copters))
        #self.center_y = running_y / float(len(copters))

        self.sorted_ids = [x for (y,x) in sorted(zip(self.offs_hype, self.ids))]

        self.center_x = self.cops[0].cur_pos_x
        self.center_y = self.cops[0].cur_pos_y

        for i in self.sorted_ids[::-1]:
            self.takeoff_cop(i)

    def takeoff_cop(self, id):
        self.cops[id].setmode(custom_mode = "OFFBOARD")
        self.cops[id].arm()

        time.sleep(0.25)

        self.cops[id].takeoff_velocity(alt = self.alt)
        #self.cops[id].update(self.center_x + self.offs_x[id], self.center_y + self.offs_y[id], self.alt)

        while not self.cops[id].reached:
            self.cops[id].update(self.center_x + self.offs_x[id], self.center_y + self.offs_y[id], self.alt)
            print "not reached, x: ", self.cops[id].vx, " y: ", self.cops[id].vy, " alt: ",self.alt
            time.sleep(0.1)

        
if __name__ == '__main__':
    rospy.init_node("velocity_goto")

    pv = posVel()
    pv.start_subs()
    #pv2.start_subs()
    pv.subscribe_pose_thread()
    #pv2.subscribe_pose_thread()

    time.sleep(0.1)

    pv.start_navigating()
    #pv2.start_navigating()

    time.sleep(0.1)

    print "set mode"
    pv.setmode(custom_mode="OFFBOARD")
    #pv2.setmode(custom_mode="OFFBOARD")
    pv.arm()

    time.sleep(0.1)
    pv.takeoff_velocity()
    
    #pv2.arm()

    time.sleep(0.1)
    #pv2.takeoff_velocity()
    print "out of takeoff"

    #utm_coords = utm.from_latlon(37.8733893, -122.3026196)
    utm_coords = utm.from_latlon(37.873178, -122.302849)
    print "going to gps", utm_coords, "current: ", pv.get_lat_lon_alt()
    #pv.update(utm_coords[0], utm_coords[1], 40.0)
    pv.update(465717.78528424399, 5249399.629721744, 40.0)
    #pv2.update(465717.78528424399, 5249399.629721744, 20.0)
    #pv.update(pv.get_lat_lon_alt()[0], pv.get_lat_lon_alt()[1], 40.0) 
    while not pv.reached:
        time.sleep(0.025)

    print "at gps, waiting"
    time.sleep(2.0)

    print "done"
    
    copters = [pv]
    SmartRTL(copters)
    #pv.land_velocity()

    print "Landed!"