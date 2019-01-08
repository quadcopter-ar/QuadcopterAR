#!/usr/bin/env python
from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy

import sys
import json
from math import sqrt
from collections import deque

import time

global xAnt1
global yAnt1
global zAnt1
global cont1

def posecallback(data):
        

    #Is created the pose msg, its necessary do it each time because Python manages objects by reference, 
        #and does not make deep copies unless explicitly asked to do so.
    #     pose = PoseStamped()    

    # #Set a atributes of the msg
    #     pose.header.frame_id = "map_fid"
    #     pose.pose.position.x = float(data.pose.position.x)
    #     pose.pose.position.y = float(data.pose.position.y)
    #     pose.pose.position.z = float(data.pose.position.z)
    #     pose.pose.orientation.x = float(data.pose.orientation.x)
    #     pose.pose.orientation.y = float(data.pose.orientation.y)
    #     pose.pose.orientation.z = float(data.pose.orientation.z)
    #     pose.pose.orientation.w = float(data.pose.orientation.w)

    #To avoid repeating the values, it is found that the received values are differents
        if (xAnt1 != data.pose.position.x and yAnt1 != data.pose.position.y and zAnt1 != data.pose.position.z):
            #Set a atributes of the msg
            data.header.seq = path1.header.seq + 1
            path1.header.frame_id="map_fid"
            path1.header.stamp=rospy.Time.now()
            data.header.stamp = path1.header.stamp
            path1.poses.append(data)
            #Published the msg

        cont1=cont1+1

        rospy.loginfo("TRUTHCount: %i" % cont1)
        if cont1>max_append:
            path1.poses.pop(0)

        pub1.publish(path1)

    #Save the last position
        xAnt1=data.pose.orientation.x
        yAnt1=data.pose.position.y
        zAnt1=data.pose.position.z
        return path1

def posecovcallback(data):
        global xAnt2
        global yAnt2
        global zAnt2
        global cont2

    #Is created the pose msg, its necessary do it each time because Python manages objects by reference, 
        #and does not make deep copies unless explicitly asked to do so.
        pose = PoseStamped()    

    #Set a atributes of the msg
        pose.header.frame_id = "map_fid"
        pose.pose.position.x = float(data.pose.pose.position.x)
        pose.pose.position.y = float(data.pose.pose.position.y)
        pose.pose.position.z = float(data.pose.pose.position.z)
        pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        pose.pose.orientation.w = float(data.pose.pose.orientation.w)

    #To avoid repeating the values, it is found that the received values are differents
        if (xAnt2 != pose.pose.position.x and yAnt2 != pose.pose.position.y and zAnt2 != pose.pose.position.z):
            #Set a atributes of the msg
            pose.header.seq = path2.header.seq + 1
            path2.header.frame_id="map_fid"
            path2.header.stamp=rospy.Time.now()
            pose.header.stamp = path2.header.stamp
            path2.poses.append(pose)
            #Published the msg

        cont2=cont2+1

        rospy.loginfo("VIS/ODOMCount: %i" % cont2)
        if cont2>max_append:
            path2.poses.pop(0)

        pub2.publish(path2)

    #Save the last position
        xAnt2=pose.pose.orientation.x
        yAnt2=pose.pose.position.y
        zAnt2=pose.pose.position.z
        return path2



if __name__ == '__main__':
    #Variable initialization
    global xAnt1,xAnt2
    global yAnt1,yAnt2
    global zAnt1,zAnt2
    global cont1, cont2
    xAnt1=0.0
    yAnt1=0.0
    zAnt1=0.0
    xAnt2=0.0
    yAnt2=0.0
    zAnt2=0.0
    cont1=0
    cont2=0


    #Node and msg initialization
    rospy.init_node('path_plotter')


    #Rosparams that are set in the launch
    #max size of array pose msg from the path
    if not rospy.has_param("~max_list_append"):
            rospy.logwarn('The parameter max_list_append dont exists')
    max_append = rospy.set_param("~max_list_append",3000) 
    max_append = 3000
    if not (max_append > 0):
            rospy.logwarn('The parameter max_list_append not is correct')
            sys.exit()
    pub1 = rospy.Publisher('/groundtruthpath', Path, queue_size=1)
    pub2 = rospy.Publisher('/visionpath', Path, queue_size=1)
    pub3 = rospy.Publisher('/estimatepath', Path, queue_size=1)


    path1 = Path()
    path2 = Path()
    path3 = Path()
    

    #Subscription to the topic
    rospy.Subscriber('/ground_truth/pose', PoseStamped, posecallback) 
    rospy.Subscriber('/fiducial_pose', PoseWithCovarianceStamped, posecovcallback) 
    rospy.Subscriber('/odometry/filtered', Odometry, posecovcallback) 
    rate = rospy.Rate(30) # 30hz

    try:
        while not rospy.is_shutdown():
            #rospy.spin()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass