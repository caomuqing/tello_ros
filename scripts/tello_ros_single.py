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
speed_mid=50
speed_slow=25
# tello1_address = ('192.168.1.101', 8889)
# local1_address = ('', 9010)
# sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# uav_id=1;
prev_cmdy=0
got_response=True
pos_residue=0






def main():
	rospy.init_node('tello_swarm', anonymous=True)
	#rate = rospy.Rate(10)  # 10hz
	uav_id=rospy.get_param('~uav_id')
	tello1_address = ('192.168.1.10'+str(uav_id), 8889)
	local1_address = ('', 9009+uav_id)
	print(tello1_address)
	sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock1.bind(local1_address)
	def send(message, delay):
		global got_response
		if not got_response:
			rospy.logwarn("Telle #"+str(uav_id)+": previous command did not get response!")
		got_response=False
		time.sleep(0.1*(uav_id-1))
		try:
			sock1.sendto(message.encode(), tello1_address)
			print("Sending message to Tello #"+str(uav_id)+": " + message)
		except Exception as e:
			print("Error sending: " + str(e))
		time.sleep(delay)

	def land_cb(data):
		if data.data==True:
			send("land", 0)
			time.sleep(5)
			#enquireBattery()

	def stop_cb(data):
		if data.data==True:
			send("stop", 0)

	def takeoff_cb(data):
		if data.data==True:
			send("takeoff", 0)


	def command1_cb(data):
		global prev_cmdy
		global pos_residue
		commandy=int(round(data.y*100))

		if data.y<0.011 and data.y>-0.011:
			send("go "+str(int(data.x*100))+" "+str(commandy)+" "+str(int(data.z*100))+" "+str(speed_mid), 0)
		elif commandy-pos_residue<20 and commandy-pos_residue>-20:
			if commandy-pos_residue>0:
				send("go "+str(int(data.x*100))+" "+str(20)+" "+str(int(data.z*100))+" "+str(speed_slow), 0.0)
				pos_residue=20-commandy-pos_residue
			elif commandy-pos_residue<=0:
				send("go "+str(int(data.x*100))+" "+str(-20)+" "+str(int(data.z*100))+" "+str(speed_slow), 0.0)
				pos_residue=-20-commandy-pos_residue
		else:
			speedd=40+(abs(commandy)-20)*2
			if speedd>100:
				speedd=100
			send("go "+str(int(data.x*100))+" "+str(commandy-pos_residue)+" "+str(int(data.z*100))+" "+str(speedd), 0)
			pos_residue=0

	def receive():
		# Continuously loop and listen for incoming messages
		global got_response
		while True:
			# Try to receive the message otherwise print the exception
			try:
				response1, ip_address = sock1.recvfrom(128)
				print("Received message: from Tello EDU #"+str(uav_id)+": " + response1.decode(encoding='utf-8'))
				got_response=True
			except Exception as e:
				# If there's an error close the socket and break out of the loop
				sock1.close()
				print("Error receiving: " + str(e))
				break


	def enquireBattery(self, event=None):
		send("battery?", 0);

	# Put Tello into command mode
	send("command", 3)
	send("battery?", 0)
	#rospy.Timer(rospy.Duration(10.0), enquireBattery)

	receiveThread = threading.Thread(target=receive)
	receiveThread.daemon = True
	receiveThread.start()

	rospy.Subscriber('/uav'+str(uav_id)+'/command/positionchange', Vector3, command1_cb)
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

