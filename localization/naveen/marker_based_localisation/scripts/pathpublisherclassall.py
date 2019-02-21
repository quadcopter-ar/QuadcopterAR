#!/usr/bin/env python
from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy

import sys
import time

global xAnt1,xAnt2
global yAnt1,yAnt2
global zAnt1,zAnt2
global cont1,cont2

class PathGen:

    def posecallback(self,data):
            
        #To avoid repeating the values, it is found that the received values are differents
            if (self.xAnt1 != data.pose.position.x and self.yAnt1 != data.pose.position.y and self.zAnt1 != data.pose.position.z):
                #Set a atributes of the msg
                data.header.seq = self.path.header.seq + 1
                self.path.header.frame_id="map_fid"
                self.path.header.stamp=rospy.Time.now()
                data.header.stamp = self.path.header.stamp
                self.path.poses.append(data)
                #Published the msg

            self.cont1=self.cont1+1

            rospy.loginfo("Count: %i" % self.cont1)
            if self.cont1>self.max_append_path1:
                self.path.poses.pop(0)

            # pub.publish(path)

        #Save the last position
            self.xAnt1=data.pose.orientation.x
            self.yAnt1=data.pose.position.y
            self.zAnt1=data.pose.position.z
            # return path

    def posecovcallback_1(self,data):
        

    #Is created the pose msg, its necessary do it each time because Python manages objects by reference, 
        #and does not make deep copies unless explicitly asked to do so.
        self.pose = PoseStamped()    

    #Set a atributes of the msg
        self.pose.header.frame_id = "map_fid"
        self.pose.pose.position.x = float(data.pose.pose.position.x)
        self.pose.pose.position.y = float(data.pose.pose.position.y)
        self.pose.pose.position.z = float(data.pose.pose.position.z)
        # self.pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        # self.pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        # self.pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        # self.pose.pose.orientation.w = float(data.pose.pose.orientation.w)

    #To avoid repeating the values, it is found that the received values are differents
        if (self.xAnt2 != self.pose.pose.position.x and self.yAnt2 != self.pose.pose.position.y and self.zAnt2 != self.pose.pose.position.z):
            #Set a atributes of the msg
            self.pose.header.seq = self.path2.header.seq + 1
            self.path2.header.frame_id="map_fid"
            self.path2.header.stamp=rospy.Time.now()
            self.pose.header.stamp = self.path2.header.stamp
            self.path2.poses.append(self.pose)
            #Published the msg

        self.cont2=self.cont2+1

        rospy.loginfo("VIS Count: %i" % self.cont2)
        if self.cont2>self.max_append_path2:
            self.path2.poses.pop(0)

        # pub2.publish(path2)

    #Save the last position
        self.xAnt2=self.pose.pose.orientation.x
        self.yAnt2=self.pose.pose.position.y
        self.zAnt2=self.pose.pose.position.z
        # return path2

    def posecovcallback_2(self,data):
        

    #Is created the pose msg, its necessary do it each time because Python manages objects by reference, 
        #and does not make deep copies unless explicitly asked to do so.
        self.pose_odom = PoseStamped()    

    #Set a atributes of the msg
        self.pose_odom.header.frame_id = "map_fid"
        self.pose_odom.pose.position.x = float(data.pose.pose.position.x)
        self.pose_odom.pose.position.y = float(data.pose.pose.position.y)
        self.pose_odom.pose.position.z = float(data.pose.pose.position.z)
        # self.pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        # self.pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        # self.pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        # self.pose.pose.orientation.w = float(data.pose.pose.orientation.w)

    #To avoid repeating the values, it is found that the received values are differents
        if (self.xAnt3 != self.pose.pose.position.x and self.yAnt3 != self.pose.pose.position.y and self.zAnt3 != self.pose.pose.position.z):
            #Set a atributes of the msg
            self.pose_odom.header.seq = self.path3.header.seq + 1
            self.path3.header.frame_id="map_fid"
            self.path3.header.stamp=rospy.Time.now()
            self.pose_odom.header.stamp = self.path3.header.stamp
            self.path3.poses.append(self.pose_odom)
            #Published the msg

        self.cont3=self.cont3+1

        rospy.loginfo("ODOMCount: %i" % self.cont3)
        if self.cont3>self.max_append_path3:
            self.path3.poses.pop(0)

        # pub2.publish(path2)

    #Save the last position
        self.xAnt3=self.pose_odom.pose.orientation.x
        self.yAnt3=self.pose_odom.pose.position.y
        self.zAnt3=self.pose_odom.pose.position.z
        # return path2

    def __init__(self):
        
        self.xAnt1=0.0
        self.yAnt1=0.0
        self.zAnt1=0.0
        self.cont1=0
        self.xAnt2=0.0
        self.yAnt2=0.0
        self.zAnt2=0.0
        self.cont2=0
        self.xAnt3=0.0
        self.yAnt3=0.0
        self.zAnt3=0.0
        self.cont3=0
        self.pose = PoseStamped()
        self.pose_odom = PoseStamped()
        self.path = Path()
        self.path2 = Path()
        self.path3 = Path()
        
        
    
    # max size of array pose msg from the path
        if not rospy.has_param("~max_list_append_path1"):
                rospy.logwarn('The parameter max_list_append_path1 dont exists')
        self.max_append_path1 = rospy.set_param("~max_list_append_path1",1000) 
        self.max_append_path1 = 1000
        if not (self.max_append_path1 > 0):
                rospy.logwarn('The parameter max_list_append_path1 not is correct')
                sys.exit()
        
        if not rospy.has_param("~max_list_append_path2"):
                rospy.logwarn('The parameter max_list_append_path2 dont exists')
        self.max_append_path2 = rospy.set_param("~max_list_append_path2",1000) 
        self.max_append_path2 = 1000
        if not (self.max_append_path2 > 0):
                rospy.logwarn('The parameter max_list_append_path2 not is correct')
                sys.exit()
        
        if not rospy.has_param("~max_list_append_path3"):
                rospy.logwarn('The parameter max_list_append_path3 dont exists')
        self.max_append_path3 = rospy.set_param("~max_list_append_path3",1000) 
        self.max_append_path3 = 1000
        if not (self.max_append_path3 > 0):
                rospy.logwarn('The parameter max_list_append_path3 not is correct')
                sys.exit()

        rospy.Subscriber('/ground_truth/pose', PoseStamped, self.posecallback)
        rospy.Subscriber('/fiducial_pose', PoseWithCovarianceStamped, self.posecovcallback_1)
        rospy.Subscriber('/odometry/filtered', Odometry, self.posecovcallback_2) 

        pub = rospy.Publisher('/path', Path, queue_size=1)
        pub2 = rospy.Publisher('/visionpath', Path, queue_size=1)
        pub3 = rospy.Publisher('/estimatepath', Path, queue_size=1)
        # rate = rospy.Rate(20)
        # rate = rospy.Rate(30) # 30hz

        while not rospy.is_shutdown():
            pub.publish(self.path)
            pub2.publish(self.path2)
            pub3.publish(self.path3)
            # rate.sleep()

            # if self.path_pub_:
                
            #     self.path_pub_= False


if __name__ == '__main__':
    #Variable initialization
    # global xAnt1
    # global yAnt1
    # global zAnt1
    # global cont1
    



    #Node and msg initialization
    rospy.init_node('path_plotter')
    try:
             
        #rospy.spin()
        pathpub=PathGen()
    except rospy.ROSInterruptException:
        pass