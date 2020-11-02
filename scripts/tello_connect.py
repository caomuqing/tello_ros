#!/usr/bin/env python

#import rospy
import socket
import threading
import time
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

tello1_address = ('192.168.10.1', 8889)
local1_address = ('', 9010)
wifi_name="tello_swarm"
password="12345678"


sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock1.bind(local1_address)


def receive():
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

receiveThread = threading.Thread(target=receive)
receiveThread.daemon = True
receiveThread.start()


def send(message, delay):
	try:
		sock1.sendto(message.encode(), tello1_address)
		print("Sending message: " + message)
	except Exception as e:
		print("Error sending: " + str(e))
	time.sleep(delay)


def main():
	#rate = rospy.Rate(10)  # 10hz
	# Put Tello into command mode


	send("command", 3)
	send("battery?", 3)
	send("ap "+wifi_name+" "+password,1)


	#pub = rospy.Publisher('/servo/pos', Int16, queue_size=10)
	#pub.publish(servo_position)


if __name__ == '__main__':
		try:
				main()
		except Exception:
				pass
