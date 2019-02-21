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
unity_timer = None
sp_timer = None
clients = []

# Global variables to keep track of time from the FCU
secs = None
nsecs = None

pose_count = 0
solo_pose = None
sp_pose = None

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

def time_callback(data):
    """
    Synchronize time of messages we send with the FCU. Extracts the FCU 
    time from a TIMESYNC mavlink message, and uses this to set global variables
    """
    global secs
    global nsecs
    global sp_timer
    global sp_pose

    if data.header.stamp.secs > secs:
        set_rate = rospy.ServiceProxy(mavros.get_topic('set_stream_rate'), StreamRate)
        rate_arg = 15.0
        try:
            set_rate(stream_id=0, message_rate=rate_arg, on_off=(rate_arg != 0))
        except rospy.ServiceException as ex:
            fault(ex)

    secs = data.header.stamp.secs
    nsecs = data.header.stamp.nsecs
    
    if sp_timer is None and len(clients) > 0:
        sp_pose = PoseStamped(header=Header(stamp=rospy.get_rostime(),frame_id="map"))
        sp_timer = rospy.Timer(rospy.Duration(0.05), sp_timer_callback)
    

def solo_pose_callback(solo_pose):
    global position, orientation
    position[0] = solo_pose.pose.position.x
    position[1] = solo_pose.pose.position.y
    position[2] = solo_pose.pose.position.z
    orientation[0] = solo_pose.pose.orientation.w
    orientation[1] = solo_pose.pose.orientation.x
    orientation[2] = solo_pose.pose.orientation.y
    orientation[3] = solo_pose.pose.orientation.z

class TCPHandler(socketserver.BaseRequestHandler):
    def setup(self):
        print('[INFO] New connection.')
        clients.append(self.request)
        #self.request.sendall('')
        #self.feedback_data = json.dumps(pose).encode('UTF-8')
        
        self.feedback_data = (get_pose_str()).encode('UTF-8')
        self.request.sendall(self.feedback_data)

    def handle(self):
        while True:
            try:
                self.data = self.request.recv(1024).decode('UTF-8', 'ignore').strip()
            except:
                continue
            if not self.data:
                break
            raw_str = self.data
            # print('[INFO] Receive message: ' + raw_str)

            elems = raw_str.split(',')

            if sp_pose != None:
                sp_pose.pose.position = Point(x=float(elems[0]), y=float(elems[1]), z=float(elems[2]))
                sp_pose.pose.orientation.x = float(elems[3])
                sp_pose.pose.orientation.y = float(elems[4])
                sp_pose.pose.orientation.z = float(elems[5])
                sp_pose.pose.orientation.w = float(elems[6])
                # print(sp_pose.pose)

            

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

def get_pose_str():
    global position, orientation
    pose_str = str(position[0]) + ',' + str(position[1]) + ',' + str(position[2]) + ',' + str(orientation[0]) + ',' + str(orientation[1]) + ',' + str(orientation[2]) + ',' + str(orientation[3])
    return pose_str

def timer_callback(event):
    global clients
    if len(clients) > 0:
        feedback_data = (get_pose_str()).encode('UTF-8')
        try:
            clients[0].sendall(feedback_data)
        except:
            print("[ERROR] Connection glitch. (handled)")

def sp_timer_callback(event):
    global sp_pose
    # print(sp_pose.pose)
    pos_pub = SP.get_pub_position_local(queue_size=10)

    sp_pose.header.stamp = rospy.get_rostime()
    # sp_pose.pose.position = Point(x=px, y=py, z=pz)
    eu = euler_from_quaternion((sp_pose.pose.orientation.x, sp_pose.pose.orientation.y, sp_pose.pose.orientation.z, sp_pose.pose.orientation.w))
    q = quaternion_from_euler(0, 0, -eu[2])
    sp_pose.pose.orientation = Quaternion(*q)
    
    print(sp_pose.pose)

    pos_pub.publish(sp_pose)



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



    time_sub = rospy.Subscriber("/mavlink/from", Mavlink, time_callback)
    solo_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, solo_pose_callback)

    print("[INFO] Waiting for MAVLink connection.")
    print("      (If no response, please run: roslaunch mavros apm.launch)")

    while secs is None:
        pass

    print("[INFO] MAVLink Connection established.")

    print("[INFO] Drone pose subscribed.")

    rospy.spin()