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

global xAnt
global yAnt
global zAnt
global cont

class PathGen:

    def posecallback(self,data):
            
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
            if (self.xAnt != data.pose.position.x and self.yAnt != data.pose.position.y and self.zAnt != data.pose.position.z):
                #Set a atributes of the msg
                data.header.seq = self.path.header.seq + 1
                self.path.header.frame_id="map_fid"
                self.path.header.stamp=rospy.Time.now()
                data.header.stamp = self.path.header.stamp
                self.path.poses.append(data)
                #Published the msg

            self.cont=self.cont+1

            rospy.loginfo("Count: %i" % self.cont)
            if self.cont>self.max_append:
                self.path.poses.pop(0)

            # pub.publish(path)

        #Save the last position
            self.xAnt=data.pose.orientation.x
            self.yAnt=data.pose.position.y
            self.zAnt=data.pose.position.z
            # return path

    def __init__(self):
        
        self.xAnt=0.0
        self.yAnt=0.0
        self.zAnt=0.0
        self.cont=0
        self.msg = PoseStamped()
        self.path = Path()
        self.path_pub_ = False
        
    
    # max size of array pose msg from the path
        if not rospy.has_param("~max_list_append"):
                rospy.logwarn('The parameter max_list_append dont exists')
        self.max_append = rospy.set_param("~max_list_append",1000) 
        self.max_append = 1000
        if not (self.max_append > 0):
                rospy.logwarn('The parameter max_list_append not is correct')
                sys.exit()

        rospy.Subscriber('/ground_truth/pose', PoseStamped, self.posecallback)
        pub = rospy.Publisher('/path', Path, queue_size=1)
         

        # rate = rospy.Rate(30) # 30hz

        while not rospy.is_shutdown():
            pub.publish(self.path)
            # if self.path_pub_:
                
            #     self.path_pub_= False


if __name__ == '__main__':
    #Variable initialization
    # global xAnt
    # global yAnt
    # global zAnt
    # global cont
    



    #Node and msg initialization
    rospy.init_node('path_plotter')
    try:
             
        #rospy.spin()
        pathpub=PathGen()
    except rospy.ROSInterruptException:
        pass