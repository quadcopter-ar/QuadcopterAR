#!/usr/bin/env python

import socketserver
import threading
import time
import rospy
import mavros
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point, PointStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float64
from mavros_msgs.msg import Mavlink
from mavros.mavlink import convert_to_bytes
from mavros.mavlink import convert_to_rosmsg
from mavros.utils import *
from mavros import setpoint as SP
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, StreamRate, StreamRateRequest
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sys

# Unity
position = []
orientation = []
uav1_position = [5, 0, 1.5]
uav1_orientation = [0, 0, 0, 0]
uav2_position = [-5, 0, 1.5]
uav2_orientation = [0, 0, 0, 0]
unity_timer = None
uav1_sp_timer = None
uav2_sp_timer = None
clients = []

# Global variables to keep track of time from the FCU
uav1_secs = None
uav2_secs = None

pose_count = 0
solo_pose = None
sp_pose = None
uav1_sp_pose = None
uav2_sp_pose = None

origin_set = False

class fifo(object):
    """ A simple buffer """
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

def uav1_time_callback(data):
    """
    Synchronize time of messages we send with the FCU. Extracts the FCU 
    time from a TIMESYNC mavlink message, and uses this to set global variables
    """
    global uav1_secs
    global uav1_sp_timer
    global uav1_sp_pose

    mavros.set_namespace("uav1/mavros")

    if data.header.stamp.secs > uav1_secs:
        set_rate = rospy.ServiceProxy(mavros.get_topic('set_stream_rate'), StreamRate)
        rate_arg = 15.0
        try:
            set_rate(stream_id=0, message_rate=rate_arg, on_off=(rate_arg != 0))
        except rospy.ServiceException as ex:
            fault(ex)

    uav1_secs = data.header.stamp.secs
    
    if uav1_sp_timer is None and len(clients) > 0:
        uav1_sp_pose = PoseStamped(header=Header(stamp=rospy.get_rostime(),frame_id="map"))
        uav1_sp_timer = rospy.Timer(rospy.Duration(0.5), uav1_sp_timer_callback)

def uav2_time_callback(data):
    global uav2_secs
    global uav2_sp_timer
    global uav2_sp_pose

    mavros.set_namespace("uav2/mavros")

    if data.header.stamp.secs > uav2_secs:
        set_rate = rospy.ServiceProxy(mavros.get_topic('set_stream_rate'), StreamRate)
        rate_arg = 15.0
        try:
            set_rate(stream_id=0, message_rate=rate_arg, on_off=(rate_arg != 0))
        except rospy.ServiceException as ex:
            fault(ex)

    uav2_secs = data.header.stamp.secs
    
    if uav2_sp_timer is None and len(clients) > 1:
        uav2_sp_pose = PoseStamped(header=Header(stamp=rospy.get_rostime(),frame_id="map"))
        uav2_sp_timer = rospy.Timer(rospy.Duration(0.5), uav2_sp_timer_callback)

def uav1_pose_callback(solo_pose):
    global uav1_position, uav1_orientation
    uav1_position[0] = solo_pose.pose.position.x
    uav1_position[1] = solo_pose.pose.position.y
    uav1_position[2] = solo_pose.pose.position.z
    uav1_orientation[0] = solo_pose.pose.orientation.w
    uav1_orientation[1] = solo_pose.pose.orientation.x
    uav1_orientation[2] = solo_pose.pose.orientation.y
    uav1_orientation[3] = solo_pose.pose.orientation.z

def uav2_pose_callback(solo_pose):
    global uav2_position, uav2_orientation
    uav2_position[0] = solo_pose.pose.position.x
    uav2_position[1] = solo_pose.pose.position.y
    uav2_position[2] = solo_pose.pose.position.z
    uav2_orientation[0] = solo_pose.pose.orientation.w
    uav2_orientation[1] = solo_pose.pose.orientation.x
    uav2_orientation[2] = solo_pose.pose.orientation.y
    uav2_orientation[3] = solo_pose.pose.orientation.z



class TCPHandler(socketserver.BaseRequestHandler):
    def setup(self):
        print('[INFO] New connection.')
        clients.append(self.request)
        self.client_index = len(clients)
        #self.request.sendall('')
        #self.feedback_data = json.dumps(pose).encode('UTF-8')
        
        # self.feedback_data = (get_pose_str()).encode('UTF-8')
        # self.request.sendall(self.feedback_data)

    def handle(self):
        global uav1_sp_pose, uav2_sp_pose

        while True:
            try:
                self.data = self.request.recv(1024).decode('UTF-8', 'ignore').strip()
            except:
                continue
            if not self.data:
                break
            raw_str = self.data
            # print(self.request == clients[0])
            # print('[INFO] Receive message: ' + raw_str)

            elems = raw_str.split(',')

            if self.request == clients[0]:
                if uav1_sp_pose != None:
                    uav1_sp_pose.pose.position = Point(x=float(elems[0]), y=float(elems[1]), z=float(elems[2]))
                    uav1_sp_pose.pose.orientation.x = float(elems[3])
                    uav1_sp_pose.pose.orientation.y = float(elems[4])
                    uav1_sp_pose.pose.orientation.z = float(elems[5])
                    uav1_sp_pose.pose.orientation.w = float(elems[6])
                    print(uav1_sp_pose)
            if len(clients) > 1 and self.request == clients[1]:
                if uav2_sp_pose != None:
                    uav2_sp_pose.pose.position = Point(x=float(elems[0]), y=float(elems[1]), z=float(elems[2]))
                    uav2_sp_pose.pose.orientation.x = float(elems[3])
                    uav2_sp_pose.pose.orientation.y = float(elems[4])
                    uav2_sp_pose.pose.orientation.z = float(elems[5])
                    uav2_sp_pose.pose.orientation.w = float(elems[6])
                    print(uav2_sp_pose)
            

            #self.feedback_data = ('Recieved').encode('UTF-8')
            #self.feedback_data = json.dumps(pose).encode('UTF-8')

    def finish(self):
        print('[INFO] Client connection lost.')
        clients.remove(self.request)        

    def remove(self):
        print('[INFO] Client removed.')
        clients.remove(self.request)

def send_pose():
    self.feedback_data = (get_pose_str()).encode('UTF-8')
    self.request.sendall(self.feedback_data)
    unity_timer = threading.Timer(2.0, hello, ["Hawk"])
    unity_timer.start()

def get_pose_str(index):
    global uav1_position, uav2_position, uav1_orientation, uav2_orientation

    pose_str_arr = []
    if index == 0:
        pose_str_arr.append(str(uav1_position[0]))
        pose_str_arr.append(str(uav1_position[1]))
        pose_str_arr.append(str(uav1_position[2]))
        pose_str_arr.append(str(uav1_orientation[0]))
        pose_str_arr.append(str(uav1_orientation[1]))
        pose_str_arr.append(str(uav1_orientation[2]))
        pose_str_arr.append(str(uav1_orientation[3]))
    elif index == 1:
        pose_str_arr.append(str(uav2_position[0]))
        pose_str_arr.append(str(uav2_position[1]))
        pose_str_arr.append(str(uav2_position[2]))
        pose_str_arr.append(str(uav2_orientation[0]))
        pose_str_arr.append(str(uav2_orientation[1]))
        pose_str_arr.append(str(uav2_orientation[2]))
        pose_str_arr.append(str(uav2_orientation[3]))

    pose_str = ','.join(str(x) for x in pose_str_arr)

    return pose_str

def timer_callback(event):
    global clients
    if len(clients) > 0:
        try:
            clients[0].send((get_pose_str(0)).encode('UTF-8'))
            if len(clients) > 1:
                clients[1].send((get_pose_str(1)).encode('UTF-8'))
        except:
            print("[ERROR] Connection glitch. (handled)")

def uav1_sp_timer_callback(event):
    global uav1_sp_pose
    print('sp_timer_callback')

    mavros.set_namespace("uav1/mavros") # uav2/mavros/setpoint_position/local
    pos_pub = SP.get_pub_position_local(queue_size=10)
    if uav1_sp_pose != None:
        uav1_sp_pose.header.stamp = rospy.get_rostime()
        # sp_pose.pose.position = Point(x=px, y=py, z=pz)
        eu = euler_from_quaternion((uav1_sp_pose.pose.orientation.x, uav1_sp_pose.pose.orientation.y, uav1_sp_pose.pose.orientation.z, uav1_sp_pose.pose.orientation.w))
        q = quaternion_from_euler(0, 0, -eu[2])
        uav1_sp_pose.pose.orientation = Quaternion(*q)
        
        print(uav1_sp_pose.pose)

        pos_pub.publish(uav1_sp_pose)

def uav2_sp_timer_callback(event):
    global uav2_sp_pose
    mavros.set_namespace("uav2/mavros") # uav2/mavros/setpoint_position/local
    pos_pub = SP.get_pub_position_local(queue_size=10)
    if uav2_sp_pose != None:
        uav2_sp_pose.header.stamp = rospy.get_rostime()
        # sp_pose.pose.position = Point(x=px, y=py, z=pz)
        eu = euler_from_quaternion((uav2_sp_pose.pose.orientation.x, uav2_sp_pose.pose.orientation.y, uav2_sp_pose.pose.orientation.z, uav2_sp_pose.pose.orientation.w))
        q = quaternion_from_euler(0, 0, -eu[2])
        uav2_sp_pose.pose.orientation = Quaternion(*q)
        
        print(uav2_sp_pose.pose)

        pos_pub.publish(uav2_sp_pose)



if __name__=="__main__":
    # in_topic = ""
    #in_topic = "/vrpn_client_node/solo/pose"
    #out_topic = "/mavros/mocap/pose"
    
    # Server config
    host = ''
    port = 13579

    position = [0,0,0]
    orientation = [0,0,0,0]
    
    rospy.init_node("unity_comm")
    mavros.set_namespace(mavros.DEFAULT_NAMESPACE)

    isConn = False
    while not isConn:
        try:
            server = socketserver.ThreadingTCPServer((host, port), TCPHandler)
            isConn = True
        except:
            continue

    clients = []
    print('[INFO] TCP Server Initialized. (Port=' + str(port) + ')' )
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()


    # !!!!!!!!PLEASE CHANGE STREAM RATE FIRST!!!!!!!!
    if unity_timer is None:
        unity_timer = rospy.Timer(rospy.Duration(1.0 / 15), timer_callback)



    uav1_time_sub = rospy.Subscriber("/uav1/mavlink/from", Mavlink, uav1_time_callback)
    uav2_time_sub = rospy.Subscriber("/uav2/mavlink/from", Mavlink, uav2_time_callback)
    uav1_pose_sub = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, uav1_pose_callback)
    uav2_pose_sub = rospy.Subscriber("/uav2/mavros/local_position/pose", PoseStamped, uav2_pose_callback)

    print("[INFO] Waiting for MAVLink connection.")
    print("      (If no response, please run: roslaunch mavros apm.launch)")

    while uav1_secs is None:
        pass

    print("[INFO] UAV 1 MAVLink Connection established.")

    while uav2_secs is None:
        pass

    print("[INFO] UAV 2 MAVLink Connection established.")

    print("[INFO] Drone pose subscribed.")

    rospy.spin()