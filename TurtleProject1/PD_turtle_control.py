#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def control():
	rospy.init_node('PD_turtle_control', anonymous = True)
	control_publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
	control_msg = Twist()

	control_msg.linear.x = 0
	control_msg.linear.y = 0
	control_msg.linear.z = 0
	control_msg.angular.x = 0
	control_msg.angular.y = 0
	control_msg.angular.z = 0

	control_subscriber = rospy.Subscriber()

	while not rospy.is_shutdown():
		poscx = 
		poscy = 


		#print("Your current position is: ")
		#print(poscx)
		#print(poscy)
		posx = input("Give your desired 'x' coordinate: ")
		posy = input("Give your desired 'y' coordinate: ")




		