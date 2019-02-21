#!/usr/bin/python

"""
Reads /mavros/imu/data_raw, subtracts first value(while stationary) from subsequent values 
and publishes corrected topic /mavros/imu/adjusted
"""

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

class Bias_corrector:
    
    def __init__(self,first_bias):
        
        self.first_msg = first_bias
        self.sub_bad_imu = rospy.Subscriber('/mavros/imu/data',Imu,self.correct_bias)
        self.pub_good_imu = rospy.Publisher('/mavros/imu/adjusted',Imu,queue_size = 10)
        rospy.spin()

    def correct_bias(self,bad_data):

        
        #construct a new message from Imu data Class and add header
        msg = Imu(header=Header(stamp=rospy.get_rostime(),frame_id='base_link_fid'))
        
        # adjust bad data by subracting bias 
        good_x = -(bad_data.linear_acceleration.x-self.first_msg.linear_acceleration.x)
        good_y = -(bad_data.linear_acceleration.y-self.first_msg.linear_acceleration.y)
        good_z = bad_data.linear_acceleration.z-self.first_msg.linear_acceleration.z

        good_rx = bad_data.orientation.x-self.first_msg.orientation.x
        good_ry = bad_data.orientation.y-self.first_msg.orientation.y
        good_rz = bad_data.orientation.z-self.first_msg.orientation.z
        good_rw = bad_data.orientation.w-self.first_msg.orientation.w+1.0
        
        # rewrite new topic
        
        msg.angular_velocity = bad_data.angular_velocity
        msg.angular_velocity_covariance = bad_data.angular_velocity_covariance
        msg.orientation_covariance = bad_data.orientation_covariance
        msg.linear_acceleration_covariance = bad_data.linear_acceleration_covariance

        msg.linear_acceleration.x = good_x
        msg.linear_acceleration.y = good_y
        msg.linear_acceleration.z = good_z

        msg.orientation.x = good_rx
        msg.orientation.y = good_ry
        msg.orientation.z = good_rz
        msg.orientation.w = good_rw

        self.pub_good_imu.publish(msg)
        rospy.loginfo('Bias corrected IMU')

if __name__ == '__main__':
    try:
        rospy.init_node('bias_adjuster')
        first_msg = rospy.wait_for_message('/mavros/imu/data',Imu,timeout=20)
        Bias_corrector(first_msg)
    except rospy.ROSInterruptException:
        pass
