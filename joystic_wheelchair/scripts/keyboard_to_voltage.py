#!/usr/bin/env python
# coding: UTF-8

import rospy
from joystic_wheelchair.msg import VoltageInput
from keyboard.msg import Key

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
		rospy.Subscriber("keyboard/keydown", Key, self.keyboarddownCallback)
		rospy.Subscriber("keyboard/keyup", Key, self.keyboardupCallback)

	def keyboarddownCallback(self,key):
		if key.code == UP:
			self.vol.x = MID_VOL + OUT_VOL[self.vol_index]
		elif key.code == DOWN:
			self.vol.x = MID_VOL - OUT_VOL[self.vol_index] * 1.2
		elif key.code == RIGHT:
			self.vol.y = MID_VOL + OUT_VOL[self.vol_index]
		elif key.code == LEFT:
			self.vol.y = MID_VOL - OUT_VOL[self.vol_index] * 1.2
		elif key.code == SPACE:
			self.vol_index += 1
			if self.vol_index > 3:
				self.vol_index = 0
			rospy.loginfo("vel_change_to %d",self.vol_index)

	def keyboardupCallback(self,key):
		if key.code == UP:
			self.vol.x = MID_VOL
		elif key.code == DOWN:
			self.vol.x = MID_VOL
		elif key.code == RIGHT:
			self.vol.y = MID_VOL
		elif key.code == LEFT:
			self.vol.y = MID_VOL

	def calculate_and_publish_voltage(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.pub.publish(self.vol)
			rate.sleep()

if __name__ == '__main__':
	calclater = CalculateVoltage()
	calclater.subscribe_key_data()
	calclater.calculate_and_publish_voltage()
