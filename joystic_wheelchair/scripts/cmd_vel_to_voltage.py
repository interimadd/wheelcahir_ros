#!/usr/bin/env python
# coding: UTF-8

import rospy
from joystic_wheelchair.msg import VoltageInput
from geometry_msgs.msg import Twist

UP    = 273
DOWN  = 274
RIGHT = 275
LEFT  = 276
SPACE = 32

MID_VOL = 2.8
OUT_VOL = [0.7,1.0,1.5,2.0]

class CalculateVoltage:

	vol = VoltageInput()
	vol_index = 2

	def __init__(self):
		rospy.init_node('keyboard_to_voltage')
		self.pub = rospy.Publisher('input_voltage', VoltageInput, queue_size=10)
		self.vol.x = MID_VOL
		self.vol.y = MID_VOL
	
	def subscribe_key_data(self):
		rospy.Subscriber("cmd_vel", Twist, self.cmd_velCallback)

	def cmd_velCallback(self,msg):
		self.vol.x = MID_VOL + msg.linear.x * 2.5
		self.vol.y = MID_VOL - msg.angular.z * 1.8

	def calculate_and_publish_voltage(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.pub.publish(self.vol)
			rate.sleep()

if __name__ == '__main__':
	calclater = CalculateVoltage()
	calclater.subscribe_key_data()
	calclater.calculate_and_publish_voltage()
