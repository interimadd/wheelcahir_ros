#!/usr/bin/env python
# coding: UTF-8

import rospy
import time
import serial
from joystic_wheelchair.msg import VoltageInput

x_input  = 2.5
y_input = 2.5

def callback(vol):
	global x_input
	global y_input
	x_input = vol.x
	y_input = vol.y


def connect_with_arduino():
	port = rospy.get_param('joystic_port',"/dev/ttyACM0")
	try:
		ser = serial.Serial(port,9600)
		ser.setDTR(False)
		time.sleep(1)
		ser.setDTR(True)
		print "start connection with joystic"
	except:
		print "cannot start connection with joystic"

	time.sleep(2)
	rate = rospy.Rate(5.0)

	while  not rospy.is_shutdown():
		send_voltage_to_arduino(ser)
		rate.sleep()

	ser.close()


def send_voltage_to_arduino(ser):

	vol_x = int(x_input*10)
	if 0 <= vol_x and vol_x < 10:
		ser.write("VX0"+str(vol_x)+"\n")
	elif 10 <= vol_x and vol_x < 51:
		ser.write("VX"+str(vol_x)+"\n")

	vol_y = int(y_input*10)
	if 0 <= vol_y and vol_y < 10:
		ser.write("VY0"+str(vol_y)+"\n")
	elif 10<= vol_y and vol_y < 51:
		ser.write("VY"+str(vol_y)+"\n")
	

if __name__ == '__main__':
	rospy.init_node('voltage_to_joystic_input')
	rospy.Subscriber("input_voltage", VoltageInput, callback)
	connect_with_arduino()
