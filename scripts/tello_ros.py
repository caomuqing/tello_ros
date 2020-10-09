#!/usr/bin/env python

import rospy
import socket
import threading
import time
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

speed = 100
tello1_address = ('192.168.1.101', 8889)
tello2_address = ('192.168.1.102', 8889)
tello3_address = ('192.168.1.103', 8889)

local1_address = ('', 9010)
local2_address = ('', 9011)
local3_address = ('', 9012)

sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock1.bind(local1_address)
sock2.bind(local2_address)
sock3.bind(local3_address)

def land_cb(data):
	if data.data==True:
		sendAll("land", 0)
		time.sleep(5)
		#enquireBattery()

def stop_cb(data):
	if data.data==True:
		sendAll("stop", 0)

def takeoff_cb(data):
	if data.data==True:
		sendAll("takeoff", 0)

def command1_cb(data):
	send101("go "+str(int(data.x*100))+" "+str(int(data.y*100))+" "+str(int(data.z*100))+" "+str(speed), 0)

def command2_cb(data):
	send102("go "+str(int(data.x*100))+" "+str(int(data.y*100))+" "+str(int(data.z*100))+" "+str(speed), 0)

def command3_cb(data):
	send103("go "+str(int(data.x*100))+" "+str(int(data.y*100))+" "+str(int(data.z*100))+" "+str(speed), 0)

def receive():
	# Continuously loop and listen for incoming messages
	while True:
		# Try to receive the message otherwise print the exception
		try:
			response1, ip_address = sock1.recvfrom(128)
			response2, ip_address = sock2.recvfrom(128)
			response3, ip_address = sock3.recvfrom(128)
			print("Received message: from Tello EDU #1: " + response1.decode(encoding='utf-8'))
			print("Received message: from Tello EDU #2: " + response2.decode(encoding='utf-8'))
			print("Received message: from Tello EDU #3: " + response2.decode(encoding='utf-8'))
		except Exception as e:
			# If there's an error close the socket and break out of the loop
			sock1.close()
			sock2.close()
			sock3.close()
			print("Error receiving: " + str(e))
			break

def receive101():
	# Continuously loop and listen for incoming messages
	while True:
		# Try to receive the message otherwise print the exception
		try:
			response1, ip_address = sock1.recvfrom(128)
			print("Received message: from Tello EDU #1: " + response1.decode(encoding='utf-8'))
		except Exception as e:
			# If there's an error close the socket and break out of the loop
			sock1.close()
			print("Error receiving: " + str(e))
			break

def receive102():
	# Continuously loop and listen for incoming messages
	while True:
		# Try to receive the message otherwise print the exception
		try:
			response2, ip_address = sock2.recvfrom(128)
			print("Received message: from Tello EDU #2: " + response2.decode(encoding='utf-8'))
		except Exception as e:
			# If there's an error close the socket and break out of the loop
			sock2.close()
			print("Error receiving: " + str(e))
			break

def receive103():
	# Continuously loop and listen for incoming messages
	while True:
		# Try to receive the message otherwise print the exception
		try:
			response3, ip_address = sock3.recvfrom(128)
			print("Received message: from Tello EDU #3: " + response3.decode(encoding='utf-8'))
		except Exception as e:
			# If there's an error close the socket and break out of the loop
			sock3.close()
			print("Error receiving: " + str(e))
			break

def enquireBattery(self, event=None):
	sendAll("battery?", 0);
	# for i in range(1,500):
	# 	try:
	# 		response1, ip_address = sock1.recvfrom(128)
	# 		response2, ip_address = sock2.recvfrom(128)
	# 		response3, ip_address = sock3.recvfrom(128)
	# 		rospy.loginfo("Tello 101 has battery percentage " + response1.decode(encoding='utf-8'))
	# 		rospy.loginfo("Tello 102 has battery percentage " + response2.decode(encoding='utf-8'))
	# 		rospy.loginfo("Tello 103 has battery percentage " + response3.decode(encoding='utf-8'))
	# 	except Exception as e:
	# 		# If there's an error close the socket and break out of the loop
	# 		sock1.close()
	# 		sock2.close()
	# 		sock3.close()
	# 		print("Error receiving: " + str(e))
	# 		break


def send101(message, delay):
	try:
		sock1.sendto(message.encode(), tello1_address)
		print("Sending message: " + message)
	except Exception as e:
		print("Error sending: " + str(e))
	time.sleep(delay)

def send102(message, delay):
	try:
		sock2.sendto(message.encode(), tello2_address)
		print("Sending message: " + message)
	except Exception as e:
		print("Error sending: " + str(e))
	time.sleep(delay)

def send103(message, delay):
	try:
		sock3.sendto(message.encode(), tello3_address)
		print("Sending message: " + message)
	except Exception as e:
		print("Error sending: " + str(e))

	time.sleep(delay)

def sendAll(message, delay):
	try:
		sock1.sendto(message.encode(), tello1_address)
		sock2.sendto(message.encode(), tello2_address)
		sock3.sendto(message.encode(), tello3_address)
		print("Sending message: " + message)
	except Exception as e:
		print("Error sending: " + str(e))
	time.sleep(delay)


def main():
	rospy.init_node('tello_swarm', anonymous=True)
	#rate = rospy.Rate(10)  # 10hz



	# Put Tello into command mode
	sendAll("command", 3)
	#enquireBattery()
	rospy.Timer(rospy.Duration(2.0), enquireBattery)

	receive101Thread = threading.Thread(target=receive101)
	receive101Thread.daemon = True
	receive101Thread.start()

	receive102Thread = threading.Thread(target=receive102)
	receive102Thread.daemon = True
	receive102Thread.start()

	receive103Thread = threading.Thread(target=receive103)
	receive103Thread.daemon = True
	receive103Thread.start()

	rospy.Subscriber('/uav1/command/positionchange', Vector3, command1_cb)
	rospy.Subscriber('/uav2/command/positionchange', Vector3, command2_cb)
	rospy.Subscriber('/uav3/command/positionchange', Vector3, command3_cb)
	rospy.Subscriber('/uav/command/land', Bool, land_cb)
	rospy.Subscriber('/uav/command/stop', Bool, stop_cb)
	rospy.Subscriber('/uav/command/takeoff',Bool,takeoff_cb)

	#pub = rospy.Publisher('/servo/pos', Int16, queue_size=10)
	#pub.publish(servo_position)
	rospy.spin()


if __name__ == '__main__':
		try:
				main()
		except rospy.ROSInterruptException:
				pass

