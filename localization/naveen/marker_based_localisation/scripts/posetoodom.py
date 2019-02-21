#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Header

class Rewriter:
    
    def __init__(self):        
        
        rospy.Subscriber('/fiducial_pose',PoseWithCovarianceStamped,self.posetoodom)
        self.pub = rospy.Publisher('/fiducial_pose/odom',Odometry,queue_size = 10)
        rospy.spin()

    def posetoodom(self,data):

        
        #construct a new message from Imu data Class and add header
        msg = Odometry(header=Header(stamp=rospy.get_rostime(),frame_id='map_fid'))
        msg.child_frame_id = 'base_link_fid'
                
        # rewrite new topic
        msg.pose = data.pose        
        self.pub.publish(msg)
        

if __name__ == '__main__':
    try:
        rospy.init_node('posetoodom')
        rospy.wait_for_message('/fiducial_pose',PoseWithCovarianceStamped,timeout=20)
        rospy.loginfo('pose Received')
        new_topic = Rewriter()
        rospy.loginfo('odom Publishing')
    except rospy.ROSInterruptException:
        pass