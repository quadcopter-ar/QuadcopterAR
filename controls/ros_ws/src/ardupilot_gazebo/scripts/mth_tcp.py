#!/usr/bin/env python

# import socketserver
# import threading
# import time

# import rospy
# import os
# from geometry_msgs.msg import PoseStamped
# from mavros_msgs.msg import Mavlink
# from mavros.mavlink import convert_to_bytes
# from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
# from tf.transformations import quaternion_from_euler, euler_from_quaternion
# import sys

pose = []
unity_timer = None

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
			self.data = self.request.recv(1024).decode('UTF-8', 'ignore').strip()
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
	pose[0] += .01


def solo_pose_callback(solo_pose):
	global pose
	pose[0] = solo_pose.pose.position.x
	pose[1] = solo_pose.pose.position.y
	pose[2] = solo_pose.pose.position.z

if __name__ == '__main__':
	# Server config
	host = ''
	port = 13579

	pose = [0,0,0]

	server = socketserver.ThreadingTCPServer((host, port), TCPHandler)
	clients = []
	print('[INFO] TCP Server Initialized.')
	server_thread = threading.Thread(target=server.serve_forever)
	server_thread.daemon = True
	server_thread.start()

	# ROS
	in_solo_topic = "/mavros/local_position/pose"
	rospy.init_node("tcp_server")
	# global display_timer

 #    if display_timer is None:
 #        display_timer = rospy.Timer(rospy.Duration(0.5), display_timer_callback)

 #    time_sub = rospy.Subscriber("/mavlink/from", Mavlink, time_callback)

    f = fifo()
    mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)

    while secs is None:
        pass

    solo_pose_sub = rospy.Subscriber(in_solo_topic, PoseStamped, solo_pose_callback)

    rospy.spin()

	

	# Timer


	# Cmd input
	# while True:
	# 	cmd = input()

	# 	if cmd == 'ls':
	# 		print(str(len(clients)) + ' clients online.')
	# 	elif cmd == 'exit' or cmd == 'quit':
	# 		for i in range(0, len(clients)):
	# 			clients[i].close()
	# 			#clients.remove(clients[0])
	# 		exit(0)