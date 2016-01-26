#!/usr/bin/env python
# coding: UTF-8

import rospy
import time
import serial
from numpy import *
from multiprocessing import Process, Value


def conecct_with_canusb_process(tmp_v):

	# シリアル通信開始
	port = rospy.get_param('canusb_port',"/dev/ttyUSB0")
	try:
		ser = serial.Serial(port,9600)
		ser.setDTR(False)
		time.sleep(1)
		ser.setDTR(True)
		print "start connection with canusb"
	except:
		print "cannot start connection with canusb"

	# CANUSBのCANポートを開くコマンド
	# 改行コマンド×複数回→通信速度設定→ポートオープン
	time.sleep(1)
	ser.write("\r")
	ser.write("\r")
	time.sleep(1)
	ser.write("C\r")
	time.sleep(1)
	ser.write("S6\r")
	time.sleep(1)
	ser.write("O\r")

	print "can open"

	line = ""
	i = 0

	while  not rospy.is_shutdown():
		serial_bit = ser.read()
		if serial_bit == "\r":
			if i >100:
				print line
				i = 0
				print line[:4]
				print line[4]
				print line[5:]
			line = ""
		else:
			line = line + serial_bit
			i = i + 1


	ser.close()


def other_process(tmp_v):

	rospy.init_node('contact_to_canusb', anonymous=True)

	while  not rospy.is_shutdown():
		rate = rospy.Rate(1)
		rate.sleep()


if __name__ == '__main__':
	tmp_v = Value('d',0)
	conecct_with_canusb_process(tmp_v)
	"""
	connect_process = Process( target = conecct_with_canusb_process,args=(tmp_v))
	other_process = Process( target = other_process,args=(tmp_v))
	try:
		connect_process.start()
		other_process.start()
	except rospy.ROSInterruptException:
		pass
	"""