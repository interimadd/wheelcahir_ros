#!/usr/bin/env python
# coding: UTF-8

import rospy
from  std_msgs.msg import Int8


class Calculater:

	num1 = 0
	num2 = 0

	def __init__(self):
		rospy.init_node('calculte_test')
		self.pub = rospy.Publisher('result', Int8, queue_size=10)
		self.pub_num = Int8()
	
	def start_subscribe(self):
		rospy.Subscriber("topic1", Int8, self.callback1)
		rospy.Subscriber("topic2", Int8, self.callback2)

	def callback1(self,num):
		self.num1 = num.data

	def callback2(self,num):
		self.num2 = num.data

	def calculate_and_publish(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.pub_num.data = self.num1 * self.num2
			self.pub.publish(self.pub_num)
			rate.sleep()


if __name__ == '__main__':

	calclater = Calculater()
	calclater.start_subscribe()
	calclater.calculate_and_publish()
