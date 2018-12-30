#!/usr/bin/env python

import socketserver
import threading
import time
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from mavros_msgs.msg import Mavlink
from mavros.mavlink import convert_to_bytes
from mavros.mavlink import convert_to_rosmsg
from mavros.utils import *
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, StreamRate, StreamRateRequest
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sys

# Unity
pose = []
unity_timer = None
clients = []

# Global variables to keep track of time from the FCU
secs = None
nsecs = None

pose_count = 0
solo_pose = None

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

    secs = data.header.stamp.secs
    nsecs = data.header.stamp.nsecs
    # set_rate = rospy.ServiceProxy(mavros.get_topic('set_stream_rate'), StreamRate)
    # try:
    #     set_rate(stream_id=id, message_rate=rate_arg, on_off=(rate_arg != 0))
    # except rospy.ServiceException as ex:
    #     fault(ex)

def solo_pose_callback(solo_pose):
    global pose
    pose[0] = solo_pose.pose.position.x
    pose[1] = solo_pose.pose.position.y
    pose[2] = solo_pose.pose.position.z

def pose_callback(pose):
    """
    Republish the given Pose information on a new topic,
    with timestamps that match the FCU
    """
    global pose_count

    pose_count += 1

    # Reduce republish frequency
    if not pose_count % 6 == 0:
        return

    # pose.pose.orientation.x = 0.0
    # pose.pose.orientation.y = 0.0
    # pose.pose.orientation.z = 0.0
    # pose.pose.orientation.w = 1.0

    if solo_pose is not None:
        pose.pose.orientation = solo_pose.pose.orientation
    else:
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

    delay_ms = 100  # approximate measurement delay in ms 
    delay_ns = delay_ms*1e6

    pose.header.frame_id = 'map'

    if (secs is not None) and (nsecs is not None):
        pose.header.stamp.secs = secs
        pose.header.stamp.nsecs = nsecs + delay_ns
        pose_pub.publish(pose)

def send_message(msg, mav, pub):
    """
    Send a mavlink message
    """
    msg.pack(mav)
    rosmsg = convert_to_rosmsg(msg)
    pub.publish(rosmsg)

    print("sent message %s" % msg)

def set_global_origin(mav, pub):
    """
    Send a mavlink SET_GPS_GLOBAL_ORIGIN message, which allows us
    to use local position information without a GPS.
    """
    #target_system = mav.srcSystem
    target_system = 0   # 0 --> broadcast to everyone
    lattitude = lat
    longitude = lon
    altitude = alt

    msg = MAV_APM.MAVLink_set_gps_global_origin_message(
            target_system,
            lattitude, 
            longitude,
            altitude)

    send_message(msg, mav, pub)

def set_home_position(mav, pub):
    """
    Send a mavlink SET_HOME_POSITION message, which should allow
    us to use local position information without a GPS
    """
    target_system = 0  # broadcast to everyone

    lattitude = lat
    longitude = lon
    altitude = alt
    
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    msg = MAV_APM.MAVLink_set_home_position_message(
            target_system,
            lattitude,
            longitude,
            altitude,
            x,
            y,
            z,
            q,
            approach_x,
            approach_y,
            approach_z)

    send_message(msg, mav, pub)

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
            print('[INFO] Receive message: ' + self.data)
            #self.feedback_data = ('Recieved').encode('UTF-8')
            #self.feedback_data = json.dumps(pose).encode('UTF-8')
            self.feedback_data = (get_pose_str()).encode('UTF-8')
            self.request.sendall(self.feedback_data)
            pose_update()

    def finish(self):
        print('[INFO] Client left.')
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
    global pose
    pose_str = str(pose[0]) + ',' + str(pose[1]) + ',' + str(pose[2])
    return pose_str

def pose_update():
    global pose

def timer_callback(event):
    global pose, clients
    if len(clients) > 0:
        feedback_data = (get_pose_str()).encode('UTF-8')
        clients[0].sendall(feedback_data)

if __name__=="__main__":
    # in_topic = ""
    #in_topic = "/vrpn_client_node/solo/pose"
    #out_topic = "/mavros/mocap/pose"
    
    # Server config
    host = ''
    port = 13579

    pose = [0,0,0]
    

    rospy.init_node("unity_comm")


    isConn = False
    while not isConn:
        try:
            server = socketserver.ThreadingTCPServer((host, port), TCPHandler)
            isConn = True
        except:
            continue
    clients = []
    print('[INFO] TCP Server Initialized.')
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()

    if unity_timer is None:
        unity_timer = rospy.Timer(rospy.Duration(1.0 / 15), timer_callback)


    # mavlink_pub = rospy.Publisher("/mavlink/to", Mavlink, queue_size=20)
    time_sub = rospy.Subscriber("/mavlink/from", Mavlink, time_callback)
    solo_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, solo_pose_callback)
    # pose_pub = rospy.Publisher(out_topic, PoseStamped, queue_size=10)
    f = fifo()
    mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)

    # while mavlink_pub.get_num_connections() <= 0:
    #     # wait for everything to initialize   
    #     pass
    # print("[ INFO] Waiting for MAVLink connection.")

    while secs is None:
        pass

    # print("[ INFO] Connection established.")

    print("[INFO] Solo pose subscribed.")

    # pose_sub = rospy.Subscriber(in_topic, PoseStamped, pose_callback)

    rospy.spin()