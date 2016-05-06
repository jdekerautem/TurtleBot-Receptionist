#!/usr/bin/python2

# This node sends the commands to the motors on the Arduino via the serial port. 
# It receives the data from both the nodes listener_arduino and android/virtual_joystick.

import rospy
import math
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, Int16, Bool


# This function is executed when some commands from the phone arrive. 
# It calculates the direction & speed of each motor on the buggy
# Direction: -1=>STOP  0=>BACKWARD  1=>FORWARD
def callback_cmd_Android(cmd):
	# If the android phone is not giving any turn command, the steer_turtlebot does, and turtlebot will turn
	# towards the most prominent face
	if cmd.angular.z == 0:
		cmd.angular.z = steer_cmd.angular.z

	pub.publish(cmd)


def callback_cmd_steer(cmd):
	steer_cmd = cmd


# Recovers the commands from the android phone and distance from 'listener_arduino' and sends the commands to arduino
def command_motors():
	# Initialization of steer_cmd, which is a copy of cmd from steer_turtlebot
	global steer_cmd
	steer_cmd = Twist()

	# Publishes on the topic /cmd_motors
	global pub 
	pub = rospy.Publisher('/base_controller/command', Twist, queue_size=3)

	rospy.init_node('remocon', anonymous=False)

	rospy.Subscriber('/steer_turtlebot/cmd_vel', Twist, callback_cmd_steer)

	# Subscription to topic from Android Phone	
	rospy.Subscriber('cmd_vel', Twist, callback_cmd_Android)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()



if __name__ == '__main__':
	print("debut remocon")
	try:
		command_motors()
	except rospy.ROSInterruptException:
		pass

